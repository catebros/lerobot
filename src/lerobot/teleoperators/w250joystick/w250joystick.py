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

from lerobot.robots.w250.constants import W250_REST_POSITION

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

        self.axes = []
        self.buttons = []
        self.lock = threading.Lock()

        self.episode_end_status = None
        self.intervention_flag = True

        self._save_pose_requested = False
        self._replay_pose_requested = False

        # Initialized to REST so the first action after robot.connect() (which calibrates
        # to REST) produces zero delta — no lurch on start. Call sync_with_robot() to update.
        self.current_positions = dict(W250_REST_POSITION)

        # Previous button states for edge detection (debounce without sleep)
        self._prev_buttons: list = []

        self.subscription = self.create_subscription(Joy, config.joy_topic, self.joy_callback, 10)

        self.get_logger().info(
            f"W250 Logitech F710 Gamepad Controller initialized, listening to {config.joy_topic}"
        )
        self.get_logger().info("=== Logitech F710 Gamepad Controls (6 DOF) ===")
        self.get_logger().info("  Left Stick X:  Waist rotation (left/right)")
        self.get_logger().info("  Left Stick Y:  Shoulder (up/down)")
        self.get_logger().info("  Right Stick Y: Elbow (up/down)")
        self.get_logger().info("  Right Stick X: Wrist angle (left/right)")
        self.get_logger().info("  D-Pad Left/Right: Forearm roll (Joint 6)")
        self.get_logger().info("  LT/RT (Triggers): Wrist rotate")
        self.get_logger().info("  LB: Open gripper")
        self.get_logger().info("  RB: Close gripper")
        self.get_logger().info("  Back: Toggle intervention (emergency stop)")
        self.get_logger().info("  Y: Reset to rest position (tucked/safe)")
        self.get_logger().info("  Start: Rerecord episode")
        self.get_logger().info("  A: Save current pose to /tmp/saved_pose.json")
        self.get_logger().info("  B: Replay saved pose + print round-trip error")

    def joy_callback(self, msg: Joy):
        with self.lock:
            self.axes = list(msg.axes)
            self.buttons = list(msg.buttons)

    def get_axes_values(self):
        with self.lock:
            return self.axes.copy() if self.axes else []

    def get_button_state(self, button_idx: int) -> bool:
        with self.lock:
            if self.buttons and 0 <= button_idx < len(self.buttons):
                return bool(self.buttons[button_idx])
            return False

    def _apply_deadzone(self, value: float) -> float:
        deadzone = self.config.deadzone
        return 0.0 if abs(value) < deadzone else value

    def _clamp_position(self, joint_name: str, position: float) -> float:
        if joint_name == "gripper":
            return max(0.0, min(1.0, position))
        else:
            return max(-1.0, min(1.0, position))

    def get_joint_increments(self) -> Dict[str, float]:
        """Calculate joint increments from Logitech F710 gamepad input (6DOF)."""
        increments = {}

        axes = self.get_axes_values()
        if not axes or len(axes) < 4:
            return {joint: 0.0 for joint in self.current_positions.keys()}

        left_stick_x = self._apply_deadzone(axes[self.config.left_stick_x_axis] if len(axes) > self.config.left_stick_x_axis else 0.0)
        increments["waist.pos"] = left_stick_x * self.config.y_step_size

        left_stick_y = self._apply_deadzone(axes[self.config.left_stick_y_axis] if len(axes) > self.config.left_stick_y_axis else 0.0)
        increments["shoulder.pos"] = -left_stick_y * self.config.x_step_size  # inverted for intuitive control

        right_stick_y = self._apply_deadzone(axes[self.config.right_stick_y_axis] if len(axes) > self.config.right_stick_y_axis else 0.0)
        increments["elbow.pos"] = -right_stick_y * self.config.z_step_size  # inverted for intuitive control

        right_stick_x = self._apply_deadzone(axes[self.config.right_stick_x_axis] if len(axes) > self.config.right_stick_x_axis else 0.0)
        increments["wrist_angle.pos"] = right_stick_x * self.config.wrist_step_size

        forearm_roll_increment = 0.0
        if len(axes) > self.config.dpad_x_axis:
            dpad_x = axes[self.config.dpad_x_axis]
            if abs(dpad_x) > 0.5:
                forearm_roll_increment = dpad_x * self.config.forearm_step_size
        increments["forearm_roll.pos"] = forearm_roll_increment

        left_trigger = self._apply_deadzone(axes[self.config.left_trigger_axis] if len(axes) > self.config.left_trigger_axis else 0.0)
        right_trigger = self._apply_deadzone(axes[self.config.right_trigger_axis] if len(axes) > self.config.right_trigger_axis else 0.0)
        # Triggers return negative values when pressed on Logitech F710:
        # LT negative = rotate counter-clockwise, RT negative = rotate clockwise
        increments["wrist_rotate.pos"] = (right_trigger - left_trigger) * self.config.wrist_step_size

        if self.get_button_state(self.config.button_l1):
            increments["gripper.pos"] = self.config.gripper_step_size
        elif self.get_button_state(self.config.button_r1):
            increments["gripper.pos"] = -self.config.gripper_step_size
        else:
            increments["gripper.pos"] = 0.0

        return increments

    def apply_increments(self, increments: Dict[str, float]) -> Dict[str, float]:
        new_positions = {}
        for joint, increment in increments.items():
            current_pos = self.current_positions.get(joint, 0.0)
            new_pos = self._clamp_position(joint.split('.')[0], current_pos + increment)
            self.current_positions[joint] = new_pos
            new_positions[joint] = new_pos
        return new_positions

    def reset_to_home(self):
        self.current_positions.update(W250_REST_POSITION)

    def sync_positions(self, robot_positions: Dict[str, float]):
        """Sync internal position tracking with robot's actual positions to prevent unwanted movements."""
        for joint_key in self.current_positions:
            if joint_key in robot_positions:
                self.current_positions[joint_key] = robot_positions[joint_key]
                self.get_logger().info(f"Synced {joint_key}: {robot_positions[joint_key]:.3f}")

    def _button_rising_edge(self, idx: int) -> bool:
        """Return True only on the rising edge (just pressed) for button idx."""
        with self.lock:
            buttons = self.buttons.copy() if self.buttons else []
        curr = bool(buttons[idx]) if idx < len(buttons) else False
        prev = bool(self._prev_buttons[idx]) if idx < len(self._prev_buttons) else False
        return curr and not prev

    def update_state(self):
        """Update episode control state based on button presses."""
        with self.lock:
            buttons = self.buttons.copy() if self.buttons else []

        if self._button_rising_edge(self.config.button_select):
            self.intervention_flag = not self.intervention_flag

        if self._button_rising_edge(self.config.button_triangle):
            self.reset_to_home()

        if self.get_button_state(self.config.button_start):
            self.episode_end_status = TeleopEvents.RERECORD_EPISODE
        else:
            self.episode_end_status = None

        if self._button_rising_edge(self.config.button_a):
            self._save_pose_requested = True
        if self._button_rising_edge(self.config.button_b):
            self._replay_pose_requested = True

        self._prev_buttons = buttons

    def consume_save_pose(self) -> bool:
        """Return True (once) if A was pressed since the last call."""
        if self._save_pose_requested:
            self._save_pose_requested = False
            return True
        return False

    def consume_replay_pose(self) -> bool:
        """Return True (once) if B was pressed since the last call."""
        if self._replay_pose_requested:
            self._replay_pose_requested = False
            return True
        return False


class W250JoystickTeleop(Teleoperator):
    """
    Teleop class to use Logitech F710 gamepad inputs via ROS2 for control.
    Requires ROS2 joy node to be running with connected Logitech F710 gamepad.

    Controls all 6 DOF of the W250 robot arm (6DOF model):
    - Waist rotation (left stick X)
    - Shoulder (left stick Y)
    - Elbow (right stick Y)
    - Forearm roll (D-Pad left/right) - Joint 6
    - Wrist angle (right stick X)
    - Wrist rotate (LT/RT triggers)
    Plus gripper control (LB/RB buttons)

    To use this teleoperator:
    1. Connect your Logitech F710 gamepad via USB
    2. Set the mode switch on back to 'X' (XInput mode)
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
        return {
            "waist.pos": float,
            "shoulder.pos": float,
            "elbow.pos": float,
            "forearm_roll.pos": float,
            "wrist_angle.pos": float,
            "wrist_rotate.pos": float,
            "gripper.pos": float,
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self.gamepad_node = GamepadController(self.config)

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.gamepad_node)

        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()

        logging.info("W250 Logitech F710 gamepad teleoperator connected and listening for joy messages")

    def _spin_ros(self):
        try:
            self.executor.spin()
        except Exception as e:
            logging.error(f"Error in ROS2 spin: {e}")

    def consume_save_pose(self) -> bool:
        """Return True (once) if A was pressed since the last call."""
        if self.gamepad_node is None:
            return False
        return self.gamepad_node.consume_save_pose()

    def consume_replay_pose(self) -> bool:
        """Return True (once) if B was pressed since the last call."""
        if self.gamepad_node is None:
            return False
        return self.gamepad_node.consume_replay_pose()

    def sync_with_robot(self, robot_observation: dict[str, Any]) -> None:
        """Sync position tracking with robot's actual state to prevent unwanted movements on start."""
        if self.gamepad_node is None:
            raise RuntimeError("Gamepad controller not connected. Call connect() first.")

        self.gamepad_node.sync_positions(robot_observation)
        logging.info("Joystick positions synced with robot state")

    def get_action(self) -> dict[str, Any]:
        if self.gamepad_node is None:
            raise RuntimeError("Gamepad controller not connected. Call connect() first.")

        self.gamepad_node.update_state()
        increments = self.gamepad_node.get_joint_increments()
        return self.gamepad_node.apply_increments(increments)

    def get_teleop_events(self) -> dict[str, Any]:
        if self.gamepad_node is None:
            return {
                TeleopEvents.IS_INTERVENTION: False,
                TeleopEvents.TERMINATE_EPISODE: False,
                TeleopEvents.SUCCESS: False,
                TeleopEvents.RERECORD_EPISODE: False,
            }

        self.gamepad_node.update_state()

        is_intervention = self.gamepad_node.intervention_flag
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
        if self.executor is not None:
            self.executor.shutdown()

        if self.gamepad_node is not None:
            self.gamepad_node.destroy_node()
            self.gamepad_node = None

        if self.ros_thread is not None and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
            self.ros_thread = None

        logging.info("W250 Logitech F710 gamepad teleoperator disconnected")

    @property
    def is_connected(self) -> bool:
        return self.gamepad_node is not None

    def calibrate(self) -> None:
        pass

    @property
    def is_calibrated(self) -> bool:
        return True

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass
