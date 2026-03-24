# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import math
import time
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from functools import cached_property
from typing import Any, Dict, Optional
import numpy as np
from rclpy.executors import MultiThreadedExecutor

# Import Interbotix API
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# Import LeRobot components  
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.constants import OBS_STATE
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_w250_interbotix import W250InterbotixConfig
from .constants import W250_REST_POSITION

logger = logging.getLogger(__name__)

# This is basically: LeRobot (framework IA)  <-->  W250Interbotix  <-->  ROS2 / Interbotix API  <-->  Robot físico

class W250Interbotix(Robot):
    """
    WidowX-250 robot using official Interbotix Python API with ROS2
    """

    config_class = W250InterbotixConfig
    name = "w250_interbotix"

    def __init__(self, config: W250InterbotixConfig):
        super().__init__(config)
        self.config = config
        
        # Initialize Interbotix robot (will be set in connect())
        self.bot: Optional[InterbotixManipulatorXS] = None
        
        # State tracking
        self._is_connected = False
        self._current_positions: Dict[str, float] = {}
        self._ros_executor: Optional[MultiThreadedExecutor] = None
        self._ros_spin_thread: Optional[threading.Thread] = None
        
        # Position conversion mappings (normalized LeRobot <-> Interbotix radians)
        # WidowX 250 6DOF has 6 joints (waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate)
        self._joint_names = [
            "waist",
            "shoulder",
            "elbow",
            "forearm_roll",  # Joint 6 - Forearm rotation
            "wrist_angle",
            "wrist_rotate"
        ]

        # Position limits for normalization (based on 6DOF specs, will be updated from robot info)
        # Joint limits from spec: Waist ±180°, Shoulder -108 to 114°, Elbow -123 to 92°,
        # Forearm Roll ±180°, Wrist Angle -100 to 123°, Wrist Rotate ±180°
        self._joint_limits = {
            "waist": (-np.pi, np.pi),  # +-180°
            "shoulder": (-1.88, 1.99),  # -108° to 114° (approx -1.88 to 1.99 rad)
            "elbow": (-2.15, 1.60),  # -123° to 92° (approx -2.15 to 1.60 rad)
            "forearm_roll": (-np.pi, np.pi),  # +-180° (Joint 6)
            "wrist_angle": (-1.745, 2.15),  # -100° to 123° (approx -1.745 to 2.15 rad)
            "wrist_rotate": (-np.pi, np.pi)  # +-180°
        } # THIS IS A FALLBACK - we will update these limits from the robot info during connection
        # TODO: potential test, are we using this fallabck? or are we successfully updating from robot info?
        
        # Initialize cameras
        self.cameras = make_cameras_from_configs(config.cameras)
        
        # Thread safety
        self._state_lock = threading.Lock()
        
        logger.info(f"W250Interbotix initialized with model: {config.robot_model}")

    @property
    def _motors_ft(self) -> Dict[str, type]:
        """Motor feature types for LeRobot compatibility"""
        return {f"{joint}.pos": float for joint in self._joint_names + ["gripper"]}

    @property
    def _cameras_ft(self) -> Dict[str, tuple]:
        """Camera feature types for LeRobot compatibility.
        For cameras with use_depth=True, adds a companion '{cam}_depth' key (H, W, 3) uint8
        encoded as normalized grayscale-as-RGB (max 3 m), compatible with video encoding.
        """
        features: Dict[str, tuple] = {}
        for cam in self.cameras:
            cam_cfg = self.config.cameras[cam]
            features[cam] = (cam_cfg.height, cam_cfg.width, 3)
            if getattr(cam_cfg, "use_depth", False):
                features[f"{cam}_depth"] = (cam_cfg.height, cam_cfg.width, 3)
        return features

    @cached_property
    def observation_features(self) -> Dict[str, type | tuple]:
        """Combined observation features"""
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> Dict[str, type]:
        """Action features (motor positions only)"""
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        """Check if robot and cameras are connected"""
        cameras_connected = all(cam.is_connected for cam in self.cameras.values())
        return self._is_connected and cameras_connected

    def _verify_ros2_setup(self) -> None:
        """
        Verify ROS2 environment is properly configured
        Checks for common ROS2 setup issues before attempting to connect.
        """
        import os
        import subprocess

        # Check if ROS2 is sourced
        if 'ROS_DISTRO' not in os.environ:
            logger.warning(
                "ROS_DISTRO not found in environment. "
                "Make sure to source your ROS2 installation:\n"
                "  $ source /opt/ros/<distro>/setup.bash"
            )

        # Check if interbotix workspace is sourced (if needed)
        if 'INTERBOTIX_WS' in os.environ:
            logger.debug(f"Interbotix workspace: {os.environ['INTERBOTIX_WS']}")

        # Verify control node is running
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                nodes = result.stdout.strip().split('\n')
                expected_node = f"/{self.config.robot_name}"
                if not any(expected_node in node for node in nodes):
                    logger.warning(
                        f"Control node '{expected_node}' not found in running nodes. "
                        f"You may need to run:\n"
                        f"  $ ros2 launch interbotix_xsarm_control xsarm_control.launch.py "
                        f"robot_model:={self.config.robot_model}"
                    )
            else:
                logger.debug("Could not check ROS2 nodes (ros2 command may not be available)")
        except (subprocess.TimeoutExpired, FileNotFoundError, Exception) as e:
            logger.debug(f"Could not verify ROS2 node status: {e}")

    def connect(self, calibrate: bool = True) -> None:
        """
        Connect to the WidowX-250 robot using Interbotix API

        Args:
            calibrate: Whether to calibrate the robot (handled by Interbotix internally)

        Note:
            For ROS2, ensure the control launch file is running first:
            $ ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting to {self} using Interbotix API...")

        # Verify ROS2 environment
        self._verify_ros2_setup()

        try:
            # Initialize Interbotix robot
            # Note: init_node parameter removed - not supported in this ROS2 version
            self.bot = InterbotixManipulatorXS(
                robot_model=self.config.robot_model,
                group_name=self.config.group_name,
                gripper_name=self.config.gripper_name,
                robot_name=self.config.robot_name,
                moving_time=self.config.moving_time,
                accel_time=self.config.accel_time,
                gripper_pressure=self.config.gripper_pressure,
                gripper_pressure_lower_limit=self.config.gripper_pressure_lower_limit,
                gripper_pressure_upper_limit=self.config.gripper_pressure_upper_limit
            )
            
            # Update joint limits from robot info
            self._update_joint_limits()

            # Set gripper to PWM mode so xs_sdk_sim interprets ±effort as incremental rotation.
            # In xs_sdk_sim, 'position' mode (the default) treats the cmd value as a target angle
            # in radians — so our ±250 PWM effort would be misread as a 250-rad position target,
            # giving garbage left_finger values and making G/H behave identically.
            # Setting 'pwm' mode first ensures: angle += effort/2000 per sim tick (correct).
            self._set_gripper_pwm_mode()

            # Keep tmr_gripper_state ENABLED — it is the hardware safety mechanism that
            # auto-stops the gripper when the finger reaches its physical limits.
            # It only acts when gripper_moving=True, which we control explicitly in send_action.

            # Start a background executor so joint-state callbacks fire continuously.
            # InterbotixManipulatorXS does NOT spin its node after __init__, so without
            # this the joint_states subscriber never processes new messages and
            # get_joint_positions() always returns the initial REST position.
            self._ros_executor = MultiThreadedExecutor()
            self._ros_executor.add_node(self.bot.core.robot_node)
            self._ros_spin_thread = threading.Thread(
                target=self._ros_executor.spin, daemon=True
            )
            self._ros_spin_thread.start()
            logger.info("ROS2 background executor started for joint-state updates")

            # Read initial positions
            self._update_positions()

            self._is_connected = True
            
            # Connect cameras
            for cam in self.cameras.values():
                cam.connect()
                
            # Set up robot (move to home if needed)
            if calibrate:
                self.calibrate()
            
            logger.info(f"{self} connected successfully")
            logger.info(
                f"IMPORTANT: run lerobot_record with --dataset.fps={self.config.fps} "
                f"to match robot fps (moving_time={self.config.moving_time:.4f}s)"
            )
            
        except Exception as e:
            logger.error(f"Failed to connect to {self}: {e}")
            self._is_connected = False
            if self.bot:
                self.bot = None
            raise

    def _update_joint_limits(self):
        """Update joint limits from Interbotix robot info"""
        try:
            group_info = self.bot.arm.group_info
            for i, joint_name in enumerate(self._joint_names):
                if i < len(group_info.joint_lower_limits) and i < len(group_info.joint_upper_limits):
                    lower = group_info.joint_lower_limits[i]
                    upper = group_info.joint_upper_limits[i]
                    self._joint_limits[joint_name] = (lower, upper)
                    logger.debug(f"Joint {joint_name} limits: [{lower:.3f}, {upper:.3f}]")
        except Exception as e:
            logger.warning(f"Could not update joint limits: {e}")

    def _set_gripper_pwm_mode(self) -> None:
        """
        Force the gripper into PWM operating mode.

        xs_sdk_sim defaults the gripper to 'position' mode (from modes.yaml).
        In position mode, the JointSingleCommand.cmd value is treated as a target
        angle in radians — so sending ±250 (PWM effort) is misread as ±250 rad,
        producing garbage left_finger positions and making open/close identical.

        In 'pwm' mode xs_sdk_sim does: angle += cmd/2000.0 each sim tick, which
        is the correct incremental behaviour for PWM control.

        This is safe to call on real hardware too: 'pwm' mode is how the
        Interbotix gripper driver normally expects to be used.
        """
        if not self.bot:
            return
        try:
            core = self.bot.gripper.core
            if hasattr(core, 'robot_set_operating_modes'):
                core.robot_set_operating_modes("single", "gripper", "pwm")
                logger.info("Gripper operating mode set to PWM")
            else:
                # Fallback: call service directly via srv_set_operating_modes
                from interbotix_xs_msgs.srv import OperatingModes
                req = OperatingModes.Request()
                req.cmd_type = "single"
                req.name = "gripper"
                req.mode = "pwm"
                req.profile_type = "velocity"
                req.profile_velocity = 0
                req.profile_acceleration = 0
                future = core.srv_set_operating_modes.call_async(req)
                import rclpy
                rclpy.spin_until_future_complete(core.node, future, timeout_sec=2.0)
                logger.info("Gripper operating mode set to PWM (via async service)")
        except Exception as e:
            logger.warning(f"Could not set gripper to PWM mode: {e}")

    def _normalize_position(self, joint_name: str, position: float) -> float:
        """Convert joint position from radians to normalized [-1, 1] range"""
        lower, upper = self._joint_limits.get(joint_name, (-1.0, 1.0))
        if upper == lower:
            return 0.0
        return 2.0 * (position - lower) / (upper - lower) - 1.0

    def _denormalize_position(self, joint_name: str, normalized_pos: float) -> float:
        """Convert normalized position [-1, 1] back to joint radians"""
        # Clamp to valid range
        normalized_pos = max(-1.0, min(1.0, normalized_pos))
        
        lower, upper = self._joint_limits.get(joint_name, (-1.0, 1.0))
        return lower + (normalized_pos + 1.0) * (upper - lower) / 2.0

    def _update_positions(self):
        """Update current joint positions from robot"""
        if not self.bot:
            return
            
        try:
            # Get actual joint positions from joint state feedback (not commanded positions)
            joint_positions = self.bot.arm.get_joint_positions()
            
            with self._state_lock:
                # Update arm joints
                for i, joint_name in enumerate(self._joint_names):
                    if i < len(joint_positions):
                        normalized_pos = self._normalize_position(joint_name, joint_positions[i])
                        self._current_positions[f"{joint_name}.pos"] = normalized_pos
                
                # Get gripper position (0=closed, 1=open for LeRobot compatibility)
                # Use get_finger_position() — returns left_finger joint in METERS (linear)
                # Limits come directly from gripper_info to avoid hardcoded values
                try:
                    if hasattr(self.bot, 'gripper') and hasattr(self.bot.gripper, 'get_finger_position'):
                        finger_pos = self.bot.gripper.get_finger_position()  # meters
                        gripper_min = self.bot.gripper.left_finger_lower_limit
                        gripper_max = self.bot.gripper.left_finger_upper_limit
                        gripper_pos = (finger_pos - gripper_min) / (gripper_max - gripper_min)
                        gripper_pos = max(0.0, min(1.0, gripper_pos))  # Clamp to [0, 1]
                    else:
                        gripper_pos = 0.0
                    self._current_positions["gripper.pos"] = gripper_pos
                except Exception as e:
                    # Fallback if gripper state unavailable
                    logger.debug(f"Could not read gripper position: {e}")
                    self._current_positions["gripper.pos"] = 0.0
                    
        except Exception as e:
            logger.error(f"Failed to update positions: {e}")

    @property
    def is_calibrated(self) -> bool:
        """Check if robot is calibrated (always True for Interbotix API)"""
        return self._is_connected

    def calibrate(self) -> None:
        """
        Calibration for Interbotix robot

        The Interbotix API handles calibration internally. This method performs
        a two-step calibration process:
        1. Move to HOME position briefly (all joints at 0)
        2. Move to REST position (safe, ready-to-record position)

        IMPORTANT: This method handles the initialization, no real calibration
        """
        if not self.bot:
            logger.error("Robot not connected, cannot calibrate")
            return

        logger.info("Calibrating robot...")

        # Calibration moves are large (full range from HOME → REST), so we need
        # enough moving_time to stay under the π rad/s velocity limit.
        # Worst case: wrist_angle REST ≈ 1.62 rad from HOME → needs ≥ 0.52 s.
        # We use 2.0 s to be safe, then restore the configured moving_time after.
        CALIB_MOVING_TIME = 2.0
        CALIB_ACCEL_TIME  = 0.5

        try:
            self.bot.arm.set_trajectory_time(CALIB_MOVING_TIME, CALIB_ACCEL_TIME)

            # Step 1: Move to HOME position (all joints at 0) using built-in API method
            logger.info("Step 1/2: Moving to HOME position (all joints at 0, wrist aligned)")
            self.bot.arm.go_to_home_pose(blocking=True)
            time.sleep(0.5)  # Brief pause at home position

            # Step 2: Move to REST position (safe, ready-to-record)
            # Convert normalized positions to radians for each joint
            # Joint order: [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
            rest_positions = [
                self._denormalize_position("waist", W250_REST_POSITION["waist.pos"]),
                self._denormalize_position("shoulder", W250_REST_POSITION["shoulder.pos"]),
                self._denormalize_position("elbow", W250_REST_POSITION["elbow.pos"]),
                self._denormalize_position("forearm_roll", W250_REST_POSITION["forearm_roll.pos"]),
                self._denormalize_position("wrist_angle", W250_REST_POSITION["wrist_angle.pos"]),
                self._denormalize_position("wrist_rotate", W250_REST_POSITION["wrist_rotate.pos"]),
            ]

            logger.info("Step 2/2: Moving to REST position (safe, ready-to-record)")
            logger.info(f"  REST normalized: waist={W250_REST_POSITION['waist.pos']:.2f}, "
                       f"shoulder={W250_REST_POSITION['shoulder.pos']:.2f}, "
                       f"elbow={W250_REST_POSITION['elbow.pos']:.2f}, "
                       f"forearm_roll={W250_REST_POSITION['forearm_roll.pos']:.2f}, "
                       f"wrist_angle={W250_REST_POSITION['wrist_angle.pos']:.2f}, "
                       f"wrist_rotate={W250_REST_POSITION['wrist_rotate.pos']:.2f}")
            logger.info(f"  REST radians: waist={rest_positions[0]:.3f}, "
                       f"shoulder={rest_positions[1]:.3f}, "
                       f"elbow={rest_positions[2]:.3f}, "
                       f"forearm_roll={rest_positions[3]:.3f}, "
                       f"wrist_angle={rest_positions[4]:.3f}, "
                       f"wrist_rotate={rest_positions[5]:.3f}")

            if not self.bot.arm.set_joint_positions(rest_positions, blocking=True):
                logger.warning("REST position rejected by Interbotix — positions may be outside joint limits")

            # Restore normal trajectory timing for teleop
            self.bot.arm.set_trajectory_time(self.config.moving_time, self.config.accel_time)
            logger.debug(f"Restored trajectory time: moving={self.config.moving_time}s accel={self.config.accel_time}s")

            # Handle gripper - move to open/closed position based on REST position state
            # Use release()/grasp() from the Interbotix API, NOT raw effort+sleep.
            # release()/grasp() call gripper_controller() which:
            #   1. Checks left_finger position against URDF limits before sending
            #   2. Sets gripper_moving=True so the 50Hz tmr_gripper_state timer auto-stops
            #      the motor the moment the finger reaches its limit (prevents angle runaway in sim)
            if hasattr(self.bot, 'gripper'):
                try:
                    if W250_REST_POSITION["gripper.pos"] > 0.5:
                        logger.info("  Opening gripper (release)...")
                        self.bot.gripper.release(delay=2.0)  # timer auto-stops at upper_limit
                        logger.info("  Gripper opened")
                    else:
                        logger.info("  Closing gripper (grasp)...")
                        self.bot.gripper.grasp(delay=2.0)  # timer auto-stops at lower_limit
                        logger.info("  Gripper closed")
                except Exception as e:
                    logger.warning(f"Could not command gripper during calibration: {e}")

            # Update positions after calibration
            self._update_positions()

            logger.info("Calibration completed - Robot at REST position (ready to record)")

        except Exception as e:
            # Always restore trajectory time even if calibration fails partway through
            try:
                self.bot.arm.set_trajectory_time(self.config.moving_time, self.config.accel_time)
            except Exception:
                pass
            logger.error(f"Calibration failed: {e}")
            raise

    def configure(self) -> None:
        """
        Configure robot parameters using Interbotix API.
        This method sets up operational parameters like velocity and acceleration limits.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        logger.info("Configuring robot parameters...")
        
        try:
            # Set conservative movement parameters for safe operation
            # These parameters help prevent sudden movements
            
            # Gripper pressure is already configured during initialization
            # The gripper_pressure parameter is passed to InterbotixManipulatorXS constructor
            logger.debug(f"Gripper pressure configured at initialization: {self.config.gripper_pressure}")
            
            # Set operating mode to position control for all joints
            try:
                if hasattr(self.bot.arm.core, 'robot_set_operating_modes'):
                    self.bot.arm.core.robot_set_operating_modes("group", "arm", "position")
                    logger.debug("Set arm operating mode to position control")
            except Exception as e:
                logger.debug(f"Could not set operating mode: {e}")
            
            # Configure movement timing
            moving_time = getattr(self.config, 'moving_time', 2.0)
            accel_time = getattr(self.config, 'accel_time', 0.3)
            if hasattr(self.bot.arm, 'set_trajectory_time'):
                self.bot.arm.set_trajectory_time(moving_time, accel_time)
                logger.debug(f"Set trajectory time to {moving_time}s (accel={accel_time}s)")
            
            logger.info("Robot configuration completed successfully")
            
        except Exception as e:
            logger.warning(f"Some configuration parameters could not be set: {e}")
            # Don't raise exception as basic operation should still work

    def get_observation(self) -> Dict[str, Any]:
        """
        Get current robot observation (joint positions + camera images)
        
        Returns:
            Dictionary with joint positions and camera images
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict = {}

        # Read all cameras in PARALLEL so their async_read() waits overlap.
        # With N cameras at fps Hz, sequential reads cost N × (1/fps) worst-case;
        # parallel reads cost 1 × (1/fps) — essential for staying within budget.
        _MAX_DEPTH_MM = 700.0  # clip depth at 700 mm (70 cm workspace); normalize to uint8 [0,255]

        def _read_one_camera(cam_key: str, cam: Any) -> Dict[str, Any]:
            t0 = time.perf_counter()
            color = cam.async_read()
            result: Dict[str, Any] = {cam_key: color}
            if getattr(cam, "use_depth", False):
                with cam.frame_lock:
                    depth_raw = cam.latest_depth_frame
                    if depth_raw is not None:
                        depth_raw = depth_raw.copy()
                if depth_raw is not None:
                    depth_f = np.clip(depth_raw.astype(np.float32) / _MAX_DEPTH_MM, 0.0, 1.0)
                    depth_u8 = (depth_f * 255).astype(np.uint8)
                    result[f"{cam_key}_depth"] = np.repeat(depth_u8[:, :, np.newaxis], 3, axis=2)
            logger.debug(f"{self} read {cam_key}: {(time.perf_counter()-t0)*1e3:.1f}ms")
            return result

        if self.cameras:
            with ThreadPoolExecutor(max_workers=len(self.cameras)) as pool:
                futures = {pool.submit(_read_one_camera, k, v): k for k, v in self.cameras.items()}
                for fut in as_completed(futures):
                    obs_dict.update(fut.result())

        # Update and get joint positions right after camera frame
        start = time.perf_counter()
        self._update_positions()

        with self._state_lock:
            obs_dict.update(self._current_positions.copy())

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read joint states: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: Dict[str, float]) -> Dict[str, float]:
        """
        Send action to robot (move to target joint positions)
        
        Args:
            action: Dictionary of joint positions (normalized -1 to 1)
            
        Returns:
            Dictionary of actual commands sent (potentially clipped)
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if not self.bot:
            raise DeviceNotConnectedError("Interbotix robot not initialized")

        # Extract joint positions from action (convert Tensors/arrays to plain float)
        goal_pos = {key.removesuffix(".pos"): float(val) for key, val in action.items() if key.endswith(".pos")}
        
        # Apply safety limits if configured
        if self.config.max_relative_target is not None:
            with self._state_lock:
                present_pos = self._current_positions.copy()
            
            goal_present_pos = {key: (g_pos, present_pos.get(f"{key}.pos", 0.0)) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        try:
            # Send arm joint commands
            # Fetch current positions once before the loop (avoid repeated ROS2 service calls)
            current_joints = self.bot.arm.get_joint_positions()
            arm_positions = []
            for idx, joint_name in enumerate(self._joint_names):
                if joint_name in goal_pos:
                    # Convert normalized position to radians
                    joint_rad = self._denormalize_position(joint_name, goal_pos[joint_name])
                    arm_positions.append(joint_rad)
                else:
                    # Keep current position if not specified
                    arm_positions.append(current_joints[idx] if idx < len(current_joints) else 0.0)
            
            # Send joint position command (non-blocking for real-time control)
            # set_joint_positions returns False if any position is outside URDF limits OR
            # if the required speed (|goal-commanded| / moving_time) exceeds joint velocity limits.
            #
            # Velocity rejection happens when the keyboard jumps far (R key, 0 key, or key repeat
            # burst). In that case, clamp each joint to the maximum achievable delta this cycle
            # (velocity_limit × moving_time) so the robot always makes progress toward the target.
            if not self.bot.arm.set_joint_positions(arm_positions, blocking=False):
                try:
                    moving_time = self.bot.arm.moving_time
                    vel_limits = self.bot.arm.group_info.joint_velocity_limits
                    last_cmd = list(self.bot.arm.joint_commands)
                    clamped = []
                    for goal, curr, vl in zip(arm_positions, last_cmd, vel_limits):
                        max_step = vl * moving_time * 0.95  # 5% margin under limit
                        delta = goal - curr
                        if abs(delta) > max_step:
                            clamped.append(curr + math.copysign(max_step, delta))
                        else:
                            clamped.append(goal)
                    if not self.bot.arm.set_joint_positions(clamped, blocking=False):
                        logger.debug(f"Clamped positions also rejected (position limit): {clamped}")
                    else:
                        logger.debug(f"Velocity-clamped arm positions toward target")
                except Exception as e:
                    logger.warning(f"set_joint_positions rejected and clamping failed: {e}")
            
            # Gripper effort control — direction-based, fire-and-forget per cycle
            #
            # The Interbotix gripper is PWM/current-controlled:
            #   positive effort → motor opens finger
            #   negative effort → motor closes finger
            #   zero effort     → motor stops
            #
            # delta = gripper_cmd - current_physical_norm
            #   delta > 0  →  commanded target is MORE open than current  → open motor
            #   delta < 0  →  commanded target is LESS open than current  → close motor
            #   delta ≈ 0  →  at target                                   → stop motor
            #
            # Using physical position (not a running counter) means:
            #   1. Teleop: counter clamped at [0,1]; delta vs physical stays nonzero
            #      as long as gripper hasn't reached the target → no saturation.
            #   2. Policy inference: policy predicts physical [0,1] target; delta vs
            #      current physical gives the correct drive direction every step.
            #
            # gripper_moving=True lets the safety timer (tmr_gripper_state, 50 Hz)
            # auto-stop the motor when the finger reaches its hardware limits.
            if "gripper" in goal_pos and hasattr(self.bot, 'gripper'):
                gripper_cmd = goal_pos["gripper"]

                try:
                    max_effort = abs(self.bot.gripper.gripper_value)
                    finger_pos = self.bot.gripper.get_finger_position()
                    lower = self.bot.gripper.left_finger_lower_limit
                    upper = self.bot.gripper.left_finger_upper_limit
                    finger_norm = (finger_pos - lower) / (upper - lower) if upper > lower else 0.5

                    delta = gripper_cmd - finger_norm

                    if delta > 0.001 and finger_pos < upper:
                        # Target is more open than current and not at limit → open motor
                        self.bot.gripper.gripper_command.cmd = max_effort
                        self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                        self.bot.gripper.gripper_moving = True
                        logger.debug(f"Gripper OPEN  effort=+{max_effort:.0f}  finger={finger_norm:.3f}  target={gripper_cmd:.3f}  delta={delta:+.3f}")
                    elif delta < -0.001 and finger_pos > lower:
                        # Target is more closed than current and not at limit → close motor
                        self.bot.gripper.gripper_command.cmd = -max_effort
                        self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                        self.bot.gripper.gripper_moving = True
                        logger.debug(f"Gripper CLOSE effort={-max_effort:.0f}  finger={finger_norm:.3f}  target={gripper_cmd:.3f}  delta={delta:+.3f}")
                    elif delta > 0.001 and finger_pos >= upper:
                        # Already at upper limit
                        self.bot.gripper.gripper_command.cmd = 0.0
                        self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                        self.bot.gripper.gripper_moving = False
                        logger.debug(f"Gripper OPEN  blocked — already at upper_limit ({finger_pos:.4f} >= {upper:.4f})")
                    elif delta < -0.001 and finger_pos <= lower:
                        # Already at lower limit
                        self.bot.gripper.gripper_command.cmd = 0.0
                        self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                        self.bot.gripper.gripper_moving = False
                        logger.debug(f"Gripper CLOSE blocked — already at lower_limit ({finger_pos:.4f} <= {lower:.4f})")
                    else:
                        # At target → stop motor
                        self.bot.gripper.gripper_command.cmd = 0.0
                        self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                        self.bot.gripper.gripper_moving = False
                except Exception as e:
                    logger.warning(f"Failed to command gripper: {e}")
            
            logger.debug(f"Sent action to robot: {goal_pos}")
            
        except Exception as e:
            logger.error(f"Failed to send action: {e}")
            raise

        # Return the actual commands sent
        return {f"{joint}.pos": val for joint, val in goal_pos.items()}

    def _shutdown_ros2(self) -> None:
        """Stop the Interbotix executor thread, destroy the node, and shut down rclpy."""
        import rclpy

        # 0. Shut down the background executor we started in connect()
        if self._ros_executor is not None:
            try:
                self._ros_executor.shutdown(timeout_sec=2.0)
                logger.debug("Background ROS2 executor shutdown")
            except Exception as e:
                logger.debug(f"background executor.shutdown: {e}")
            self._ros_executor = None
        if self._ros_spin_thread is not None and self._ros_spin_thread.is_alive():
            try:
                self._ros_spin_thread.join(timeout=2.0)
                logger.debug("Background ROS2 spin thread joined")
            except Exception as e:
                logger.debug(f"background spin_thread.join: {e}")
            self._ros_spin_thread = None

        core = getattr(self.bot, "core", None)
        if core is None:
            # core may live on arm or gripper
            core = getattr(getattr(self.bot, "arm", None), "core", None)

        if core is not None:
            # 1. Stop the MultiThreadedExecutor spin loop
            executor = getattr(core, "executor", None)
            if executor is not None:
                try:
                    executor.shutdown(timeout_sec=2.0)
                    logger.debug("ROS2 executor shutdown")
                except Exception as e:
                    logger.debug(f"executor.shutdown: {e}")

            # 2. Join the spin thread so it exits cleanly
            spin_thread = getattr(core, "thread", None)
            if spin_thread is not None and spin_thread.is_alive():
                try:
                    spin_thread.join(timeout=2.0)
                    logger.debug("ROS2 spin thread joined")
                except Exception as e:
                    logger.debug(f"spin_thread.join: {e}")

            # 3. Destroy the node
            try:
                core.destroy_node()
                logger.debug("ROS2 node destroyed")
            except Exception as e:
                logger.debug(f"destroy_node: {e}")

        # 4. Shutdown the rclpy context (safe to call even if already done)
        try:
            if rclpy.ok():
                rclpy.try_shutdown()
                logger.info("rclpy shutdown complete")
        except Exception as e:
            logger.debug(f"rclpy.try_shutdown: {e}")

        logger.info("Interbotix robot shutdown complete")

    def disconnect(self) -> None:
        """Disconnect from robot and cameras"""
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        logger.info(f"Disconnecting {self}...")

        # Stop gripper before disconnect
        if self.bot and hasattr(self.bot, 'gripper'):
            try:
                # Stop any ongoing gripper movement
                self.bot.gripper.gripper_moving = False

                # Send zero effort command to stop gripper motor
                self.bot.gripper.gripper_command.cmd = 0.0
                self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                logger.info("Gripper stopped (effort=0)")

                # Disable gripper torque to stop all gripper commands immediately
                if hasattr(self.bot.gripper.core, 'robot_torque_enable'):
                    self.bot.gripper.core.robot_torque_enable('single', 'gripper', False)
                    logger.info("Gripper torque disabled")
            except Exception as e:
                logger.warning(f"Could not stop gripper: {e}")

        # Disconnect cameras
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting camera: {e}")

        # Clean up robot connection and shutdown ROS2 node
        self._is_connected = False
        if self.bot:
            try:
                # Disable torque for all motors to stop any ongoing commands
                if hasattr(self.bot.arm.core, 'robot_torque_enable'):
                    self.bot.arm.core.robot_torque_enable('group', 'arm', False)
                    logger.info("Arm torque disabled")

                # Shutdown the ROS2 executor and node
                self._shutdown_ros2()
            except Exception as e:
                logger.warning(f"Error during robot shutdown: {e}")
            finally:
                self.bot = None

        logger.info(f"{self} disconnected")

    def __str__(self) -> str:
        return f"W250Interbotix({self.config.robot_model})"

    def __repr__(self) -> str:
        return self.__str__()