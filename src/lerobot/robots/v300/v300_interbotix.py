import logging
import math
import time
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from functools import cached_property
from typing import Any, Dict, List, Optional
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.constants import OBS_STATE
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_v300_interbotix import V300InterbotixConfig
from .constants import V300_REST_POSITION

logger = logging.getLogger(__name__)


class _JointStateListener(Node):
    """Dedicated ROS2 node that subscribes to joint_states on its own executor."""

    def __init__(self, topic: str, joint_names: List[str]):
        super().__init__("lerobot_joint_state_listener")
        self._joint_names = joint_names
        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, float]] = None  # raw radians keyed by joint name
        self.create_subscription(JointState, topic, self._cb, 10)
        logger.info(f"JointStateListener subscribed to {topic}")

    def _cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        with self._lock:
            self._latest = {n: name_to_pos[n] for n in self._joint_names if n in name_to_pos}

    @property
    def latest_positions(self) -> Optional[Dict[str, float]]:
        with self._lock:
            return dict(self._latest) if self._latest is not None else None


class V300Interbotix(Robot):
    """
    ViperX-300 S robot using official Interbotix Python API with ROS2
    """

    config_class = V300InterbotixConfig
    name = "v300_interbotix"

    def __init__(self, config: V300InterbotixConfig):
        super().__init__(config)
        self.config = config

        self.bot: Optional[InterbotixManipulatorXS] = None

        self._is_connected = False
        self._current_positions: Dict[str, float] = {}
        self._ros_executor: Optional[MultiThreadedExecutor] = None
        self._ros_spin_thread: Optional[threading.Thread] = None
        # Separate node+executor so we don't conflict with the Interbotix internal executor
        self._js_listener: Optional[_JointStateListener] = None

        self._joint_names = [
            "waist",
            "shoulder",
            "elbow",
            "forearm_roll",
            "wrist_angle",
            "wrist_rotate"
        ]

        # Fallback limits; overwritten from robot info in _update_joint_limits
        self._joint_limits = {
            "waist": (-np.pi, np.pi),
            "shoulder": (-1.88, 1.99),
            "elbow": (-2.15, 1.60),
            "forearm_roll": (-np.pi, np.pi),
            "wrist_angle": (-1.745, 2.15),
            "wrist_rotate": (-np.pi, np.pi)
        }

        self.cameras = make_cameras_from_configs(config.cameras)
        self._state_lock = threading.Lock()

        logger.info(f"V300Interbotix initialized with model: {config.robot_model}")

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

        if 'ROS_DISTRO' not in os.environ:
            logger.warning(
                "ROS_DISTRO not found in environment. "
                "Make sure to source your ROS2 installation:\n"
                "  $ source /opt/ros/<distro>/setup.bash"
            )

        if 'INTERBOTIX_WS' in os.environ:
            logger.debug(f"Interbotix workspace: {os.environ['INTERBOTIX_WS']}")

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
        Connect to the ViperX-300 S robot using Interbotix API

        Args:
            calibrate: Whether to calibrate the robot (handled by Interbotix internally)

        Note:
            For ROS2, ensure the control launch file is running first:
            $ ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting to {self} using Interbotix API...")

        self._verify_ros2_setup()

        try:
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

            self._update_joint_limits()

            # xs_sdk_sim defaults gripper to 'position' mode, which misreads ±250 PWM effort
            # as ±250 rad target angle. PWM mode interprets it as angle += cmd/2000 per tick.
            self._set_gripper_pwm_mode()

            # Separate listener node so we don't add robot_node to a second executor,
            # which would conflict with Interbotix's internal executor and break joint updates.
            js_topic = f"/{self.config.robot_name}/joint_states"
            # Include left_finger so the gripper encoder is read from the topic too
            arm_joints = self._joint_names + ["left_finger"]
            self._js_listener = _JointStateListener(js_topic, arm_joints)
            self._ros_executor = MultiThreadedExecutor()
            self._ros_executor.add_node(self._js_listener)
            self._ros_spin_thread = threading.Thread(
                target=self._ros_executor.spin, daemon=True
            )
            self._ros_spin_thread.start()
            logger.info(f"JointStateListener executor started on {js_topic}")

            self._update_positions()

            self._is_connected = True

            for cam in self.cameras.values():
                cam.connect()

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
        normalized_pos = max(-1.0, min(1.0, normalized_pos))
        lower, upper = self._joint_limits.get(joint_name, (-1.0, 1.0))
        return lower + (normalized_pos + 1.0) * (upper - lower) / 2.0

    def _update_positions(self):
        """Update current joint positions from robot"""
        if not self.bot:
            return

        try:
            # Prefer the dedicated listener (fresh encoder values from topic) over
            # get_joint_positions(), which returns stale commanded positions.
            listener_data = self._js_listener.latest_positions if self._js_listener else None

            with self._state_lock:
                for i, joint_name in enumerate(self._joint_names):
                    if listener_data is not None and joint_name in listener_data:
                        raw_rad = listener_data[joint_name]
                        normalized_pos = self._normalize_position(joint_name, raw_rad)
                    else:
                        joint_positions = self.bot.arm.get_joint_positions()
                        normalized_pos = self._normalize_position(joint_name, joint_positions[i]) if i < len(joint_positions) else 0.0
                    self._current_positions[f"{joint_name}.pos"] = normalized_pos

                # Get gripper position (0=closed, 1=open for LeRobot compatibility)
                try:
                    gripper_min = self.bot.gripper.left_finger_lower_limit
                    gripper_max = self.bot.gripper.left_finger_upper_limit
                    if listener_data is not None and "left_finger" in listener_data:
                        finger_pos = listener_data["left_finger"]  # radians from topic
                    elif hasattr(self.bot, 'gripper') and hasattr(self.bot.gripper, 'get_finger_position'):
                        finger_pos = self.bot.gripper.get_finger_position()  # meters (stale fallback)
                    else:
                        finger_pos = gripper_min
                    gripper_pos = (finger_pos - gripper_min) / (gripper_max - gripper_min)
                    gripper_pos = max(0.0, min(1.0, gripper_pos))
                    self._current_positions["gripper.pos"] = gripper_pos
                except Exception as e:
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

        # Large HOME→REST moves need enough time to stay under the π rad/s velocity limit.
        # wrist_angle REST ≈ 1.62 rad from HOME → needs ≥ 0.52 s; using 2.0 s for safety.
        CALIB_MOVING_TIME = 2.0
        CALIB_ACCEL_TIME = 0.5

        try:
            self.bot.arm.set_trajectory_time(CALIB_MOVING_TIME, CALIB_ACCEL_TIME)

            logger.info("Step 1/2: Moving to HOME position (all joints at 0, wrist aligned)")
            self.bot.arm.go_to_home_pose(blocking=True)
            time.sleep(0.5)

            rest_positions = [
                self._denormalize_position("waist", V300_REST_POSITION["waist.pos"]),
                self._denormalize_position("shoulder", V300_REST_POSITION["shoulder.pos"]),
                self._denormalize_position("elbow", V300_REST_POSITION["elbow.pos"]),
                self._denormalize_position("forearm_roll", V300_REST_POSITION["forearm_roll.pos"]),
                self._denormalize_position("wrist_angle", V300_REST_POSITION["wrist_angle.pos"]),
                self._denormalize_position("wrist_rotate", V300_REST_POSITION["wrist_rotate.pos"]),
            ]

            logger.info("Step 2/2: Moving to REST position (safe, ready-to-record)")
            logger.info(f"  REST normalized: waist={V300_REST_POSITION['waist.pos']:.2f}, "
                       f"shoulder={V300_REST_POSITION['shoulder.pos']:.2f}, "
                       f"elbow={V300_REST_POSITION['elbow.pos']:.2f}, "
                       f"forearm_roll={V300_REST_POSITION['forearm_roll.pos']:.2f}, "
                       f"wrist_angle={V300_REST_POSITION['wrist_angle.pos']:.2f}, "
                       f"wrist_rotate={V300_REST_POSITION['wrist_rotate.pos']:.2f}")
            logger.info(f"  REST radians: waist={rest_positions[0]:.3f}, "
                       f"shoulder={rest_positions[1]:.3f}, "
                       f"elbow={rest_positions[2]:.3f}, "
                       f"forearm_roll={rest_positions[3]:.3f}, "
                       f"wrist_angle={rest_positions[4]:.3f}, "
                       f"wrist_rotate={rest_positions[5]:.3f}")

            if not self.bot.arm.set_joint_positions(rest_positions, blocking=True):
                logger.warning("REST position rejected by Interbotix — positions may be outside joint limits")

            self.bot.arm.set_trajectory_time(self.config.moving_time, self.config.accel_time)
            logger.debug(f"Restored trajectory time: moving={self.config.moving_time}s accel={self.config.accel_time}s")

            # Use release()/grasp() rather than raw effort+sleep: they check URDF limits before
            # sending and set gripper_moving=True so the 50 Hz safety timer auto-stops the motor.
            if hasattr(self.bot, 'gripper'):
                try:
                    if V300_REST_POSITION["gripper.pos"] > 0.5:
                        logger.info("  Opening gripper (release)...")
                        self.bot.gripper.release(delay=2.0)
                        logger.info("  Gripper opened")
                    else:
                        logger.info("  Closing gripper (grasp)...")
                        self.bot.gripper.grasp(delay=2.0)
                        logger.info("  Gripper closed")
                except Exception as e:
                    logger.warning(f"Could not command gripper during calibration: {e}")

            self._update_positions()

            logger.info("Calibration completed - Robot at REST position (ready to record)")

        except Exception as e:
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
            logger.debug(f"Gripper pressure configured at initialization: {self.config.gripper_pressure}")

            try:
                if hasattr(self.bot.arm.core, 'robot_set_operating_modes'):
                    self.bot.arm.core.robot_set_operating_modes("group", "arm", "position")
                    logger.debug("Set arm operating mode to position control")
            except Exception as e:
                logger.debug(f"Could not set operating mode: {e}")

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

        # Read all cameras in parallel so their async_read() waits overlap.
        _MAX_DEPTH_MM = 700.0  # clip depth at 700 mm; normalize to uint8 [0,255]

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

        goal_pos = {key.removesuffix(".pos"): float(val) for key, val in action.items() if key.endswith(".pos")}

        if self.config.max_relative_target is not None:
            with self._state_lock:
                present_pos = self._current_positions.copy()

            goal_present_pos = {key: (g_pos, present_pos.get(f"{key}.pos", 0.0)) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        try:
            # Fetch current positions once before the loop (avoid repeated ROS2 service calls)
            current_joints = self.bot.arm.get_joint_positions()
            arm_positions = []
            for idx, joint_name in enumerate(self._joint_names):
                if joint_name in goal_pos:
                    joint_rad = self._denormalize_position(joint_name, goal_pos[joint_name])
                    arm_positions.append(joint_rad)
                else:
                    arm_positions.append(current_joints[idx] if idx < len(current_joints) else 0.0)

            # set_joint_positions rejects if speed (|goal-commanded| / moving_time) exceeds velocity limits.
            # On rejection, clamp each joint to the max achievable delta this cycle so the robot makes progress.
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

            # Gripper is PWM/current-controlled: positive effort opens, negative closes, zero stops.
            # delta = gripper_cmd - current_physical_norm drives direction each cycle.
            # gripper_moving=True lets the 50 Hz safety timer auto-stop at hardware limits.
            if "gripper" in goal_pos and hasattr(self.bot, 'gripper'):
                gripper_cmd = goal_pos["gripper"]

                try:
                    max_effort = abs(self.bot.gripper.gripper_value)

                    # Use the ROS-topic-based gripper position so replay and delta calc are consistent.
                    with self._state_lock:
                        finger_norm = self._current_positions.get("gripper.pos", 0.5)

                    delta = gripper_cmd - finger_norm

                    if delta > 0.05:
                        self.bot.gripper.gripper_command.cmd = max_effort
                        self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                        self.bot.gripper.gripper_moving = True
                        logger.debug(f"Gripper OPEN  effort=+{max_effort:.0f}  finger={finger_norm:.3f}  target={gripper_cmd:.3f}  delta={delta:+.3f}")
                    elif delta < -0.05:
                        self.bot.gripper.gripper_command.cmd = -max_effort
                        self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                        self.bot.gripper.gripper_moving = True
                        logger.debug(f"Gripper CLOSE effort={-max_effort:.0f}  finger={finger_norm:.3f}  target={gripper_cmd:.3f}  delta={delta:+.3f}")
                    else:
                        self.bot.gripper.gripper_command.cmd = 0.0
                        self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                        self.bot.gripper.gripper_moving = False
                        logger.debug(f"Gripper HOLD  finger={finger_norm:.3f}  target={gripper_cmd:.3f}  delta={delta:+.3f}")
                except Exception as e:
                    logger.warning(f"Failed to command gripper: {e}")

            logger.debug(f"Sent action to robot: {goal_pos}")

        except Exception as e:
            logger.error(f"Failed to send action: {e}")
            raise

        return {f"{joint}.pos": val for joint, val in goal_pos.items()}

    def _shutdown_ros2(self) -> None:
        """Stop the Interbotix executor thread, destroy the node, and shut down rclpy."""
        import rclpy

        if self._ros_executor is not None:
            try:
                self._ros_executor.shutdown(timeout_sec=2.0)
                logger.debug("JointStateListener executor shutdown")
            except Exception as e:
                logger.debug(f"JointStateListener executor.shutdown: {e}")
            self._ros_executor = None
        if self._ros_spin_thread is not None and self._ros_spin_thread.is_alive():
            try:
                self._ros_spin_thread.join(timeout=2.0)
                logger.debug("JointStateListener spin thread joined")
            except Exception as e:
                logger.debug(f"JointStateListener spin_thread.join: {e}")
            self._ros_spin_thread = None
        if self._js_listener is not None:
            try:
                self._js_listener.destroy_node()
                logger.debug("JointStateListener node destroyed")
            except Exception as e:
                logger.debug(f"JointStateListener destroy_node: {e}")
            self._js_listener = None

        core = getattr(self.bot, "core", None)
        if core is None:
            core = getattr(getattr(self.bot, "arm", None), "core", None)

        if core is not None:
            executor = getattr(core, "executor", None)
            if executor is not None:
                try:
                    executor.shutdown(timeout_sec=2.0)
                    logger.debug("ROS2 executor shutdown")
                except Exception as e:
                    logger.debug(f"executor.shutdown: {e}")

            spin_thread = getattr(core, "thread", None)
            if spin_thread is not None and spin_thread.is_alive():
                try:
                    spin_thread.join(timeout=2.0)
                    logger.debug("ROS2 spin thread joined")
                except Exception as e:
                    logger.debug(f"spin_thread.join: {e}")

            try:
                core.destroy_node()
                logger.debug("ROS2 node destroyed")
            except Exception as e:
                logger.debug(f"destroy_node: {e}")

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

        if self.bot and hasattr(self.bot, 'gripper'):
            try:
                self.bot.gripper.gripper_moving = False
                self.bot.gripper.gripper_command.cmd = 0.0
                self.bot.gripper.core.pub_single.publish(self.bot.gripper.gripper_command)
                logger.info("Gripper stopped (effort=0)")

                if hasattr(self.bot.gripper.core, 'robot_torque_enable'):
                    self.bot.gripper.core.robot_torque_enable('single', 'gripper', False)
                    logger.info("Gripper torque disabled")
            except Exception as e:
                logger.warning(f"Could not stop gripper: {e}")

        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting camera: {e}")

        self._is_connected = False
        if self.bot:
            try:
                if hasattr(self.bot.arm.core, 'robot_torque_enable'):
                    self.bot.arm.core.robot_torque_enable('group', 'arm', False)
                    logger.info("Arm torque disabled")

                self._shutdown_ros2()
            except Exception as e:
                logger.warning(f"Error during robot shutdown: {e}")
            finally:
                self.bot = None

        logger.info(f"{self} disconnected")

    def __str__(self) -> str:
        return f"V300Interbotix({self.config.robot_model})"

    def __repr__(self) -> str:
        return self.__str__()
