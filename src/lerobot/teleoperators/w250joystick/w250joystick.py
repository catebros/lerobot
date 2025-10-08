#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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
import threading
from enum import IntEnum
from typing import Any

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
except ImportError:
    raise ImportError(
        "ROS2 (rclpy) is required for W250JoystickTeleop. "
        "Please install ROS2 and source your workspace: "
        "https://docs.ros.org/en/rolling/Installation.html"
    )

from ..teleoperator import Teleoperator
from ..utils import TeleopEvents
from .config_w250joystick import W250JoystickConfig


class GripperAction(IntEnum):
    CLOSE = 0
    STAY = 1
    OPEN = 2


gripper_action_map = {
    "close": GripperAction.CLOSE.value,
    "open": GripperAction.OPEN.value,
    "stay": GripperAction.STAY.value,
}


class JoyConController(Node):
    """ROS2 node to receive Joy messages from Nintendo Switch Left JoyCon."""

    def __init__(self, config: W250JoystickConfig):
        super().__init__(config.ros2_node_name)
        self.config = config

        # Current state
        self.axes = []
        self.buttons = []
        self.lock = threading.Lock()

        # Episode control flags
        self.episode_end_status = None
        self.intervention_flag = False

        # Gripper commands
        self.open_gripper_command = False
        self.close_gripper_command = False

        # Subscribe to joy topic
        self.subscription = self.create_subscription(Joy, config.joy_topic, self.joy_callback, 10)

        self.get_logger().info(
            f"W250 Left JoyCon Controller initialized, listening to {config.joy_topic}"
        )
        self.get_logger().info("Left JoyCon controls:")
        self.get_logger().info("  Analog stick: Move in X-Y plane")
        self.get_logger().info("  D-pad Up/Down: Move in Z axis (up/down)")
        self.get_logger().info("  Minus button: Open gripper")
        self.get_logger().info("  ZL button: Close gripper")
        self.get_logger().info("  L button: Enable intervention")
        self.get_logger().info("  Stick button (L3): End episode with SUCCESS")
        self.get_logger().info("  Capture button: Rerecord episode")

    def joy_callback(self, msg: Joy):
        """Callback for Joy messages from ROS2."""
        with self.lock:
            self.axes = list(msg.axes)
            self.buttons = list(msg.buttons)

    def get_axes_values(self):
        """Get current joystick axes values."""
        with self.lock:
            return self.axes.copy() if self.axes else []

    def get_button_state(self, button_idx: int) -> bool:
        """Get state of a specific button."""
        with self.lock:
            if self.buttons and 0 <= button_idx < len(self.buttons):
                return bool(self.buttons[button_idx])
            return False

    def update_state(self):
        """Update episode control state based on button presses."""
        # Check intervention button (L button)
        self.intervention_flag = self.get_button_state(self.config.button_l)

        # Check gripper buttons
        self.open_gripper_command = self.get_button_state(self.config.button_minus)
        self.close_gripper_command = self.get_button_state(self.config.button_zl)

        # Check episode end buttons
        if self.get_button_state(self.config.button_stick):
            self.episode_end_status = TeleopEvents.SUCCESS
        elif self.get_button_state(self.config.button_capture):
            self.episode_end_status = TeleopEvents.RERECORD_EPISODE
        else:
            self.episode_end_status = None


class W250JoystickTeleop(Teleoperator):
    """
    Teleop class to use Nintendo Switch Left JoyCon inputs via ROS2 for control.
    Requires ROS2 joy node to be running with paired Left JoyCon.

    To use this teleoperator:
    1. Pair your Left JoyCon via Bluetooth
    2. Install and run the ROS2 joy node:
       ros2 run joy joy_node
    3. Run your robot control with this teleoperator
    """

    config_class = W250JoystickConfig
    name = "w250joystick"

    def __init__(self, config: W250JoystickConfig):
        super().__init__(config)
        self.config = config
        self.joycon_node = None
        self.ros_thread = None
        self.executor = None

    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (4,),
                "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2, "gripper": 3},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (3,),
                "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2},
            }

    @property
    def feedback_features(self) -> dict:
        return {}

    def connect(self) -> None:
        """Initialize ROS2 and start the JoyCon controller node."""
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Create the JoyCon controller node
        self.joycon_node = JoyConController(self.config)

        # Start spinning in a separate thread
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.joycon_node)

        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()

        logging.info("W250 JoyCon teleoperator connected and listening for joy messages")

    def _spin_ros(self):
        """Spin ROS2 executor in a separate thread."""
        try:
            self.executor.spin()
        except Exception as e:
            logging.error(f"Error in ROS2 spin: {e}")

    def get_action(self) -> dict[str, Any]:
        """Get action from Left JoyCon inputs."""
        if self.joycon_node is None:
            raise RuntimeError("JoyCon controller not connected. Call connect() first.")

        # Update the controller state
        self.joycon_node.update_state()

        # Get axes values
        axes = self.joycon_node.get_axes_values()

        # Default to zero movement
        delta_x = 0.0
        delta_y = 0.0
        delta_z = 0.0

        if axes:
            # Get analog stick values and apply deadzone
            stick_x = axes[self.config.stick_x_axis] if len(axes) > self.config.stick_x_axis else 0.0
            stick_y = axes[self.config.stick_y_axis] if len(axes) > self.config.stick_y_axis else 0.0

            # Apply deadzone
            stick_x = 0.0 if abs(stick_x) < self.config.deadzone else stick_x
            stick_y = 0.0 if abs(stick_y) < self.config.deadzone else stick_y

            # Map to deltas (invert Y axis as joystick up is typically negative)
            delta_x = -stick_y * self.config.x_step_size  # Forward/Backward
            delta_y = stick_x * self.config.y_step_size  # Left/Right

        # Handle Z-axis movement with D-pad buttons
        if self.joycon_node.get_button_state(self.config.button_up):
            delta_z = self.config.z_step_size  # Move up
        elif self.joycon_node.get_button_state(self.config.button_down):
            delta_z = -self.config.z_step_size  # Move down

        # Create action dictionary
        action_dict = {
            "delta_x": float(delta_x),
            "delta_y": float(delta_y),
            "delta_z": float(delta_z),
        }

        # Handle gripper
        gripper_action = GripperAction.STAY.value
        if self.config.use_gripper:
            if self.joycon_node.open_gripper_command and not self.joycon_node.close_gripper_command:
                gripper_action = GripperAction.OPEN.value
            elif self.joycon_node.close_gripper_command and not self.joycon_node.open_gripper_command:
                gripper_action = GripperAction.CLOSE.value
            action_dict["gripper"] = gripper_action

        return action_dict

    def get_teleop_events(self) -> dict[str, Any]:
        """
        Get extra control events from the JoyCons such as intervention status,
        episode termination, success indicators, etc.

        Returns:
            Dictionary containing:
                - is_intervention: bool - Whether human is currently intervening
                - terminate_episode: bool - Whether to terminate the current episode
                - success: bool - Whether the episode was successful
                - rerecord_episode: bool - Whether to rerecord the episode
        """
        if self.joycon_node is None:
            return {
                TeleopEvents.IS_INTERVENTION: False,
                TeleopEvents.TERMINATE_EPISODE: False,
                TeleopEvents.SUCCESS: False,
                TeleopEvents.RERECORD_EPISODE: False,
            }

        # Update state
        self.joycon_node.update_state()

        # Check if intervention is active
        is_intervention = self.joycon_node.intervention_flag

        # Get episode end status
        episode_end_status = self.joycon_node.episode_end_status
        terminate_episode = episode_end_status in [
            TeleopEvents.RERECORD_EPISODE,
            TeleopEvents.FAILURE,
        ]
        success = episode_end_status == TeleopEvents.SUCCESS
        rerecord_episode = episode_end_status == TeleopEvents.RERECORD_EPISODE

        return {
            TeleopEvents.IS_INTERVENTION: is_intervention,
            TeleopEvents.TERMINATE_EPISODE: terminate_episode,
            TeleopEvents.SUCCESS: success,
            TeleopEvents.RERECORD_EPISODE: rerecord_episode,
        }

    def disconnect(self) -> None:
        """Disconnect from the JoyCon controller and cleanup ROS2."""
        if self.executor is not None:
            self.executor.shutdown()

        if self.joycon_node is not None:
            self.joycon_node.destroy_node()
            self.joycon_node = None

        if self.ros_thread is not None and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
            self.ros_thread = None

        logging.info("W250 JoyCon teleoperator disconnected")

    def is_connected(self) -> bool:
        """Check if JoyCon controller is connected."""
        return self.joycon_node is not None

    def calibrate(self) -> None:
        """Calibrate the JoyCon controller."""
        # No calibration needed for JoyCons with ROS2
        pass

    def is_calibrated(self) -> bool:
        """Check if JoyCon controller is calibrated."""
        # JoyCons don't require calibration with ROS2
        return True

    def configure(self) -> None:
        """Configure the JoyCon controller."""
        # No additional configuration needed
        pass

    def send_feedback(self, feedback: dict) -> None:
        """Send feedback to the JoyCon controller."""
        # JoyCons don't support feedback in this implementation
        pass
