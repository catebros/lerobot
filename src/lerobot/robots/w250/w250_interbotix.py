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
import time
import threading
from functools import cached_property
from typing import Any, Dict, Optional
import numpy as np

# Import Interbotix API
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# Import LeRobot components  
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.constants import OBS_STATE
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_w250_interbotix import W250InterbotixConfig

logger = logging.getLogger(__name__)


class W250Interbotix(Robot):
    """
    WidowX-250 robot using official Interbotix Python API with ROS2
    
    This implementation provides the same LeRobot interface while using
    the official Interbotix API underneath for maximum compatibility and reliability.
    
    Maintains the same method signatures as the original W250 class for
    seamless integration with existing LeRobot workflows.
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
        self._last_gripper_command: Optional[str] = None  # Track last gripper state to avoid loops
        
        # Position conversion mappings (normalized LeRobot <-> Interbotix radians)
        self._joint_names = [
            "waist",
            "shoulder", 
            "elbow",
            "wrist_angle",
            "wrist_rotate"
        ]
        
        # Position limits for normalization (will be updated from robot info)
        self._joint_limits = {
            "waist": (-np.pi, np.pi),
            "shoulder": (-1.88, 1.99),  
            "elbow": (-2.15, 1.60),
            "wrist_angle": (-1.745, 1.745),
            "wrist_rotate": (-3.14, 3.14)  # Can rotate continuously
        }
        
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
        """Camera feature types for LeRobot compatibility"""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) 
            for cam in self.cameras
        }

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

        # Verify control node is running (optional check)
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
            # Get joint positions
            joint_positions = self.bot.arm.get_joint_commands()
            
            with self._state_lock:
                # Update arm joints
                for i, joint_name in enumerate(self._joint_names):
                    if i < len(joint_positions):
                        normalized_pos = self._normalize_position(joint_name, joint_positions[i])
                        self._current_positions[f"{joint_name}.pos"] = normalized_pos
                
                # Get gripper position (0=closed, 1=open for LeRobot compatibility)
                # Use get_gripper_position() method from ROS2 API
                try:
                    if hasattr(self.bot, 'gripper') and hasattr(self.bot.gripper, 'get_gripper_position'):
                        # Get current gripper position in radians
                        raw_gripper_pos = self.bot.gripper.get_gripper_position()
                        # Normalize to [0, 1] range (0=closed, 1=open)
                        # Interbotix gripper typically ranges from ~0.015 (closed) to ~0.037 (open) radians
                        gripper_min, gripper_max = 0.015, 0.037
                        gripper_pos = (raw_gripper_pos - gripper_min) / (gripper_max - gripper_min)
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
        
        The Interbotix API handles calibration internally, so we just
        move the robot to a safe home position.
        """
        if not self.bot:
            logger.error("Robot not connected, cannot calibrate")
            return
            
        logger.info("Calibrating robot (moving to home position)...")
        
        try:
            # Move to home position
            self.bot.arm.go_to_home_pose()

            # Open gripper using release() method (no delay for faster calibration)
            if hasattr(self.bot, 'gripper'):
                self.bot.gripper.release(delay=0)
            
            # Update positions after calibration
            self._update_positions()
            
            logger.info("Robot calibration completed")
            
        except Exception as e:
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
            if hasattr(self.bot.arm, 'set_trajectory_time'):
                self.bot.arm.set_trajectory_time(moving_time)
                logger.debug(f"Set trajectory time to {moving_time}s")
            
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

        # Update and get joint positions
        start = time.perf_counter()
        self._update_positions()
        
        with self._state_lock:
            obs_dict.update(self._current_positions.copy())
            
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read joint states: {dt_ms:.1f}ms")

        # Capture camera images
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

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

        # Extract joint positions from action
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        
        # Apply safety limits if configured
        if self.config.max_relative_target is not None:
            with self._state_lock:
                present_pos = self._current_positions.copy()
            
            goal_present_pos = {key: (g_pos, present_pos.get(f"{key}.pos", 0.0)) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        try:
            # Send arm joint commands
            arm_positions = []
            for joint_name in self._joint_names:
                if joint_name in goal_pos:
                    # Convert normalized position to radians
                    joint_rad = self._denormalize_position(joint_name, goal_pos[joint_name])
                    arm_positions.append(joint_rad)
                else:
                    # Keep current position if not specified
                    current_joints = self.bot.arm.get_joint_commands()
                    idx = self._joint_names.index(joint_name)
                    arm_positions.append(current_joints[idx] if idx < len(current_joints) else 0.0)
            
            # Send joint position command (non-blocking for real-time control)
            self.bot.arm.set_joint_positions(arm_positions, blocking=False)
            
            # Handle gripper command using release()/grasp() methods
            # Only send command if state has changed to avoid infinite loop
            if "gripper" in goal_pos and hasattr(self.bot, 'gripper'):
                gripper_cmd = goal_pos["gripper"]
                desired_state = "open" if gripper_cmd > 0.5 else "closed"

                # Only send command if gripper state changed
                if self._last_gripper_command != desired_state:
                    if desired_state == "open":
                        self.bot.gripper.release(delay=0)
                        logger.debug("Gripper: releasing (opening)")
                    else:
                        self.bot.gripper.grasp(delay=0)
                        logger.debug("Gripper: grasping (closing)")
                    self._last_gripper_command = desired_state
            
            logger.debug(f"Sent action to robot: {goal_pos}")
            
        except Exception as e:
            logger.error(f"Failed to send action: {e}")
            raise

        # Return the actual commands sent
        return {f"{joint}.pos": val for joint, val in goal_pos.items()}

    def disconnect(self) -> None:
        """Disconnect from robot and cameras"""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        logger.info(f"Disconnecting {self}...")

        # Stop robot movement and disable torque if configured
        if self.bot and self.config.disable_torque_on_disconnect:
            try:
                # Move to sleep position
                self.bot.arm.go_to_sleep_pose()
                logger.info("Robot moved to sleep position")
            except Exception as e:
                logger.warning(f"Could not move to sleep position: {e}")

        # Disconnect cameras
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting camera: {e}")

        # Clean up robot connection
        self._is_connected = False
        if self.bot:
            # Interbotix API cleanup happens automatically
            self.bot = None

        logger.info(f"{self} disconnected")

    def __str__(self) -> str:
        return f"W250Interbotix({self.config.robot_model})"

    def __repr__(self) -> str:
        return self.__str__()