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
    """Configuration for Logitech F710 gamepad teleoperator using ROS2."""

    # ROS2 settings
    ros2_node_name: str = "w250_gamepad_teleop"
    joy_topic: str = "/joy"

    # Control settings
    use_gripper: bool = True
    deadzone: float = 0.1

    # Step sizes for movement
    x_step_size: float = 0.01  # Shoulder increment
    y_step_size: float = 0.01  # Waist increment
    z_step_size: float = 0.01  # Elbow increment
    wrist_step_size: float = 0.008  # Wrist angle/rotate increment

    # Logitech F710 axis mappings (XInput mode - switch set to X)
    # Left analog stick - Waist and Shoulder control
    left_stick_x_axis: int = 0  # Waist rotation (left/right)
    left_stick_y_axis: int = 1  # Shoulder (up/down)

    # Right analog stick - Elbow and Wrist angle control
    right_stick_x_axis: int = 3  # Wrist angle
    right_stick_y_axis: int = 4  # Elbow (up/down)

    # Triggers - Wrist rotate control
    left_trigger_axis: int = 2   # Wrist rotate counter-clockwise
    right_trigger_axis: int = 5  # Wrist rotate clockwise

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
