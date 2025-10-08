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


class JoyConController(Node):
    """ROS2 node to receive Joy messages from Nintendo Switch Left JoyCon."""

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
            f"W250 Left JoyCon Controller initialized, listening to {config.joy_topic}"
        )
        self.get_logger().info("=== Left JoyCon Intuitive Controls ===")
        self.get_logger().info("  Analog Stick X: Waist rotation (left/right)")
        self.get_logger().info("  Analog Stick Y: Shoulder (up/down)")
        self.get_logger().info("  D-pad Up: Elbow extend")
        self.get_logger().info("  D-pad Down: Elbow contract")
        self.get_logger().info("  D-pad Left: Wrist angle down")
        self.get_logger().info("  D-pad Right: Wrist angle up")
        self.get_logger().info("  Minus (-): Open gripper")
        self.get_logger().info("  ZL: Close gripper")
        self.get_logger().info("  L: Toggle intervention (emergency stop)")
        self.get_logger().info("  Stick button (L3): Reset to home position")

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
        """Calculate joint increments from joystick input (Interbotix-style mapping)."""
        increments = {}

        axes = self.get_axes_values()
        if not axes or len(axes) < 2:
            return {joint: 0.0 for joint in self.current_positions.keys()}

        # Analog stick controls (continuous)
        # Stick X -> Waist (left/right rotation)
        stick_x = self._apply_deadzone(axes[self.config.stick_x_axis] if len(axes) > self.config.stick_x_axis else 0.0)
        increments["waist.pos"] = stick_x * self.config.y_step_size  # Using y_step_size for waist

        # Stick Y -> Shoulder (up/down)
        stick_y = self._apply_deadzone(axes[self.config.stick_y_axis] if len(axes) > self.config.stick_y_axis else 0.0)
        increments["shoulder.pos"] = -stick_y * self.config.x_step_size  # Inverted for intuitive control

        # D-pad controls (discrete buttons)
        # Up/Down -> Elbow
        if self.get_button_state(self.config.button_up):
            increments["elbow.pos"] = self.config.z_step_size
        elif self.get_button_state(self.config.button_down):
            increments["elbow.pos"] = -self.config.z_step_size
        else:
            increments["elbow.pos"] = 0.0

        # Left/Right -> Wrist angle
        if self.get_button_state(self.config.button_left):
            increments["wrist_angle.pos"] = -self.config.z_step_size * 0.5  # Slower
        elif self.get_button_state(self.config.button_right):
            increments["wrist_angle.pos"] = self.config.z_step_size * 0.5
        else:
            increments["wrist_angle.pos"] = 0.0

        # Wrist rotate - not mapped yet (could use L2/R2 if available)
        increments["wrist_rotate.pos"] = 0.0

        # Gripper increments
        if self.get_button_state(self.config.button_minus):
            increments["gripper.pos"] = -0.02  # Open (negative)
        elif self.get_button_state(self.config.button_zl):
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
        # Check intervention button (L button) - toggle on press
        if self.get_button_state(self.config.button_l):
            self.intervention_flag = not self.intervention_flag
            time.sleep(0.2)  # Debounce

        # Check home button (stick button)
        if self.get_button_state(self.config.button_stick):
            self.reset_to_home()
            time.sleep(0.2)  # Debounce

        # Check episode end buttons
        if self.get_button_state(self.config.button_capture):
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
        """Get action from Left JoyCon inputs - returns absolute joint positions."""
        if self.joycon_node is None:
            raise RuntimeError("JoyCon controller not connected. Call connect() first.")

        # Update the controller state
        self.joycon_node.update_state()

        # Get increments from joystick input
        increments = self.joycon_node.get_joint_increments()

        # Apply increments to get new absolute positions
        positions = self.joycon_node.apply_increments(increments)

        return positions

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
