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

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("w250joystick")
@dataclass
class W250JoystickConfig(TeleoperatorConfig):
    """Configuration for Logitech F710 gamepad teleoperator using ROS2 (6DOF)."""

    ros2_node_name: str = "w250_gamepad_teleop"
    joy_topic: str = "/joy"

    use_gripper: bool = True
    deadzone: float = 0.1

    control_hz: float = 15.0  # must match robot fps and dataset fps

    x_step_size: float = 0.015
    y_step_size: float = 0.015
    z_step_size: float = 0.015
    forearm_step_size: float = 0.012
    wrist_step_size: float = 0.012
    gripper_step_size: float = 0.03

    # Logitech F710 axis mappings (XInput mode)
    left_stick_x_axis: int = 0   # Waist rotation (left/right)
    left_stick_y_axis: int = 1   # Shoulder (up/down)
    right_stick_x_axis: int = 3  # Wrist angle
    right_stick_y_axis: int = 4  # Elbow (up/down)
    dpad_x_axis: int = 6         # Forearm roll (left/right)
    dpad_y_axis: int = 7         # unused
    left_trigger_axis: int = 2   # LT — wrist rotate counter-clockwise
    right_trigger_axis: int = 5  # RT — wrist rotate clockwise

    # Logitech F710 button mappings (XInput mode)
    button_a: int = 0
    button_b: int = 1
    button_x: int = 2
    button_y: int = 3 # Reset to home position
    button_lb: int = 4  # Open gripper
    button_rb: int = 5  # Close gripper
    button_back: int = 6 # Toggle intervention
    button_start: int = 7 # Rerecord episode
    button_l3: int = 9
    button_r3: int = 10

    # Legacy PS3 compatibility names
    @property
    def button_l1(self) -> int:
        return self.button_lb

    @property
    def button_r1(self) -> int:
        return self.button_rb

    @property
    def button_triangle(self) -> int:
        return self.button_y

    @property
    def button_select(self) -> int:
        return self.button_back
