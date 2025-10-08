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
import time
from enum import IntEnum
from typing import Any, Dict

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


class GamepadController(Node):
    """ROS2 node to receive Joy messages from Logitech F710 gamepad."""

    def __init__(self, config: W250JoystickConfig):
        super().__init__(config.ros2_node_name)
        self.config = config

        # Current state from ROS2
        self.axes = []
        self.buttons = []
        self.lock = threading.Lock()

        # Episode control flags
        self.episode_end_status = None
        self.intervention_flag = True  # Start with intervention enabled

        # Current joint positions (incremental control)
        self.current_positions = {
            "waist.pos": 0.0,
            "shoulder.pos": 0.0,
            "elbow.pos": 0.0,
            "wrist_angle.pos": 0.0,
            "wrist_rotate.pos": 0.0,
            "gripper.pos": 0.5,  # Start half-open
        }

        # Subscribe to joy topic
        self.subscription = self.create_subscription(Joy, config.joy_topic, self.joy_callback, 10)

        self.get_logger().info(
            f"W250 Logitech F710 Gamepad Controller initialized, listening to {config.joy_topic}"
        )
        self.get_logger().info("=== Logitech F710 Gamepad Controls ===")
        self.get_logger().info("  Left Stick X: Waist rotation (left/right)")
        self.get_logger().info("  Left Stick Y: Shoulder (up/down)")
        self.get_logger().info("  Right Stick X: Wrist angle")
        self.get_logger().info("  Right Stick Y: Elbow (up/down)")
        self.get_logger().info("  LT/RT (Triggers): Wrist rotate")
        self.get_logger().info("  LB: Open gripper")
        self.get_logger().info("  RB: Close gripper")
        self.get_logger().info("  Back: Toggle intervention (emergency stop)")
        self.get_logger().info("  Y: Reset to home position")
        self.get_logger().info("  Start: Rerecord episode")

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

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to analog input."""
        deadzone = self.config.deadzone
        return 0.0 if abs(value) < deadzone else value

    def _clamp_position(self, joint_name: str, position: float) -> float:
        """Clamp joint position to valid range."""
        if joint_name == "gripper":
            return max(0.0, min(1.0, position))
        else:
            return max(-1.0, min(1.0, position))

    def get_joint_increments(self) -> Dict[str, float]:
        """Calculate joint increments from F710 gamepad input."""
        increments = {}

        axes = self.get_axes_values()
        if not axes or len(axes) < 6:
            return {joint: 0.0 for joint in self.current_positions.keys()}

        # Left stick controls - Waist and Shoulder
        # Left Stick X -> Waist (left/right rotation)
        left_stick_x = self._apply_deadzone(axes[self.config.left_stick_x_axis] if len(axes) > self.config.left_stick_x_axis else 0.0)
        increments["waist.pos"] = left_stick_x * self.config.y_step_size

        # Left Stick Y -> Shoulder (up/down)
        left_stick_y = self._apply_deadzone(axes[self.config.left_stick_y_axis] if len(axes) > self.config.left_stick_y_axis else 0.0)
        increments["shoulder.pos"] = -left_stick_y * self.config.x_step_size  # Inverted for intuitive control

        # Right stick controls - Elbow and Wrist angle
        # Right Stick Y -> Elbow (up/down)
        right_stick_y = self._apply_deadzone(axes[self.config.right_stick_y_axis] if len(axes) > self.config.right_stick_y_axis else 0.0)
        increments["elbow.pos"] = -right_stick_y * self.config.z_step_size  # Inverted for intuitive control

        # Right Stick X -> Wrist angle
        right_stick_x = self._apply_deadzone(axes[self.config.right_stick_x_axis] if len(axes) > self.config.right_stick_x_axis else 0.0)
        increments["wrist_angle.pos"] = right_stick_x * self.config.wrist_step_size

        # Triggers control - Wrist rotate
        # LT and RT are typically -1 to 1, with -1 being unpressed
        left_trigger = axes[self.config.left_trigger_axis] if len(axes) > self.config.left_trigger_axis else -1.0
        right_trigger = axes[self.config.right_trigger_axis] if len(axes) > self.config.right_trigger_axis else -1.0

        # Normalize triggers from -1..1 to 0..1 (0 is unpressed, 1 is fully pressed)
        left_trigger_pressed = (left_trigger + 1.0) / 2.0
        right_trigger_pressed = (right_trigger + 1.0) / 2.0

        # Calculate wrist rotate increment
        wrist_rotate_increment = 0.0
        if left_trigger_pressed > 0.1:  # Small deadzone
            wrist_rotate_increment -= left_trigger_pressed * self.config.wrist_step_size
        if right_trigger_pressed > 0.1:
            wrist_rotate_increment += right_trigger_pressed * self.config.wrist_step_size
        increments["wrist_rotate.pos"] = wrist_rotate_increment

        # Gripper increments - kept unchanged as requested
        if self.get_button_state(self.config.button_lb):
            increments["gripper.pos"] = -0.02  # Open (negative)
        elif self.get_button_state(self.config.button_rb):
            increments["gripper.pos"] = 0.02  # Close (positive)
        else:
            increments["gripper.pos"] = 0.0

        return increments

    def apply_increments(self, increments: Dict[str, float]) -> Dict[str, float]:
        """Apply increments to current positions and return new positions."""
        new_positions = {}

        for joint, increment in increments.items():
            current_pos = self.current_positions.get(joint, 0.0)
            new_pos = current_pos + increment
            new_pos = self._clamp_position(joint.split('.')[0], new_pos)

            self.current_positions[joint] = new_pos
            new_positions[joint] = new_pos

        return new_positions

    def reset_to_home(self):
        """Reset all joints to home position."""
        for joint in self.current_positions:
            if joint == "gripper.pos":
                self.current_positions[joint] = 0.5  # Half-open
            else:
                self.current_positions[joint] = 0.0  # Center

    def update_state(self):
        """Update episode control state based on button presses."""
        # Check intervention button (Back button) - toggle on press
        if self.get_button_state(self.config.button_back):
            self.intervention_flag = not self.intervention_flag
            time.sleep(0.2)  # Debounce

        # Check home button (Y button)
        if self.get_button_state(self.config.button_y):
            self.reset_to_home()
            time.sleep(0.2)  # Debounce

        # Check episode end buttons
        if self.get_button_state(self.config.button_start):
            self.episode_end_status = TeleopEvents.RERECORD_EPISODE
        else:
            self.episode_end_status = None


class W250JoystickTeleop(Teleoperator):
    """
    Teleop class to use Logitech F710 gamepad inputs via ROS2 for control.
    Requires ROS2 joy node to be running with connected F710 gamepad.

    To use this teleoperator:
    1. Connect your Logitech F710 gamepad via USB or wireless dongle
    2. Ensure the mode switch on the back is set to 'X' (XInput mode)
    3. Install and run the ROS2 joy node:
       ros2 run joy joy_node
    4. Run your robot control with this teleoperator
    """

    config_class = W250JoystickConfig
    name = "w250joystick"

    def __init__(self, config: W250JoystickConfig):
        super().__init__(config)
        self.config = config
        self.gamepad_node = None
        self.ros_thread = None
        self.executor = None

    @property
    def action_features(self) -> dict:
        """Define the structure of actions produced by this teleoperator."""
        return {
            "waist.pos": float,
            "shoulder.pos": float,
            "elbow.pos": float,
            "wrist_angle.pos": float,
            "wrist_rotate.pos": float,
            "gripper.pos": float,
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    def connect(self) -> None:
        """Initialize ROS2 and start the gamepad controller node."""
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Create the gamepad controller node
        self.gamepad_node = GamepadController(self.config)

        # Start spinning in a separate thread
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.gamepad_node)

        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()

        logging.info("W250 F710 gamepad teleoperator connected and listening for joy messages")

    def _spin_ros(self):
        """Spin ROS2 executor in a separate thread."""
        try:
            self.executor.spin()
        except Exception as e:
            logging.error(f"Error in ROS2 spin: {e}")

    def get_action(self) -> dict[str, Any]:
        """Get action from F710 gamepad inputs - returns absolute joint positions."""
        if self.gamepad_node is None:
            raise RuntimeError("Gamepad controller not connected. Call connect() first.")

        # Update the controller state
        self.gamepad_node.update_state()

        # Get increments from gamepad input
        increments = self.gamepad_node.get_joint_increments()

        # Apply increments to get new absolute positions
        positions = self.gamepad_node.apply_increments(increments)

        return positions

    def get_teleop_events(self) -> dict[str, Any]:
        """
        Get extra control events from the gamepad such as intervention status,
        episode termination, success indicators, etc.

        Returns:
            Dictionary containing:
                - is_intervention: bool - Whether human is currently intervening
                - terminate_episode: bool - Whether to terminate the current episode
                - success: bool - Whether the episode was successful
                - rerecord_episode: bool - Whether to rerecord the episode
        """
        if self.gamepad_node is None:
            return {
                TeleopEvents.IS_INTERVENTION: False,
                TeleopEvents.TERMINATE_EPISODE: False,
                TeleopEvents.SUCCESS: False,
                TeleopEvents.RERECORD_EPISODE: False,
            }

        # Update state
        self.gamepad_node.update_state()

        # Check if intervention is active
        is_intervention = self.gamepad_node.intervention_flag

        # Get episode end status
        episode_end_status = self.gamepad_node.episode_end_status
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
        """Disconnect from the gamepad controller and cleanup ROS2."""
        if self.executor is not None:
            self.executor.shutdown()

        if self.gamepad_node is not None:
            self.gamepad_node.destroy_node()
            self.gamepad_node = None

        if self.ros_thread is not None and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
            self.ros_thread = None

        logging.info("W250 F710 gamepad teleoperator disconnected")

    def is_connected(self) -> bool:
        """Check if gamepad controller is connected."""
        return self.gamepad_node is not None

    def calibrate(self) -> None:
        """Calibrate the gamepad controller."""
        # No calibration needed for gamepad with ROS2
        pass

    def is_calibrated(self) -> bool:
        """Check if gamepad controller is calibrated."""
        # Gamepad doesn't require calibration with ROS2
        return True

    def configure(self) -> None:
        """Configure the gamepad controller."""
        # No additional configuration needed
        pass

    def send_feedback(self, feedback: dict) -> None:
        """Send feedback to the gamepad controller."""
        # F710 doesn't support feedback in this implementation
        pass
