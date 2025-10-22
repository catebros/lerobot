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

    # ROS2 settings
    ros2_node_name: str = "w250_gamepad_teleop"
    joy_topic: str = "/joy"

    # Control settings
    use_gripper: bool = True
    deadzone: float = 0.1

    # Step sizes balanced for smooth control (moving_time=0.5s)
    x_step_size: float = 0.015  # Shoulder increment
    y_step_size: float = 0.015  # Waist increment
    z_step_size: float = 0.015  # Elbow increment
    forearm_step_size: float = 0.012  # Forearm roll increment (Joint 6)
    wrist_step_size: float = 0.012  # Wrist angle/rotate increment
    gripper_step_size: float = 0.03  # Gripper increment

    # Logitech F710 controller axis mappings (XInput mode)
    # Left analog stick - Waist and Shoulder control
    left_stick_x_axis: int = 0  # Waist rotation (left/right)
    left_stick_y_axis: int = 1  # Shoulder (up/down)

    # Right analog stick - Elbow and Wrist angle control
    right_stick_x_axis: int = 3  # Wrist angle
    right_stick_y_axis: int = 4  # Elbow (up/down)

    # D-Pad - Forearm roll control (Joint 6)
    dpad_x_axis: int = 6  # D-Pad horizontal (left/right) - Forearm roll
    dpad_y_axis: int = 7  # D-Pad vertical (up/down) - unused

    # Triggers - Wrist rotate control (analog axes on Logitech F710)
    left_trigger_axis: int = 2   # LT (axis) - Wrist rotate counter-clockwise
    right_trigger_axis: int = 5  # RT (axis) - Wrist rotate clockwise

    # Logitech F710 button mappings (XInput mode)
    button_a: int = 0       # A button - unused
    button_b: int = 1       # B button - unused
    button_x: int = 2       # X button - unused
    button_y: int = 3       # Y button - Reset to home position

    button_lb: int = 4      # LB button - Open gripper
    button_rb: int = 5      # RB button - Close gripper

    button_back: int = 6    # Back button - Toggle intervention (emergency stop)
    button_start: int = 7   # Start button - Rerecord episode

    button_l3: int = 9      # Left stick button - unused
    button_r3: int = 10     # Right stick button - unused

    # Legacy PS3 compatibility names (kept for backward compatibility)
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
