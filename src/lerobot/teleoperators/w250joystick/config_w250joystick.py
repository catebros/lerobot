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
    """Configuration for PS3 gamepad teleoperator using ROS2."""

    # ROS2 settings
    ros2_node_name: str = "w250_gamepad_teleop"
    joy_topic: str = "/joy"

    # Control settings
    use_gripper: bool = True
    deadzone: float = 0.1

    # Step sizes for movement (larger values = faster/bigger movements)
    x_step_size: float = 0.03  # Shoulder increment
    y_step_size: float = 0.03  # Waist increment
    z_step_size: float = 0.03  # Elbow increment
    wrist_step_size: float = 0.025  # Wrist angle/rotate increment
    gripper_step_size: float = 0.05  # Gripper increment

    # PS3 controller axis mappings
    # Left analog stick - Waist and Shoulder control
    left_stick_x_axis: int = 0  # Waist rotation (left/right)
    left_stick_y_axis: int = 1  # Shoulder (up/down)

    # Right analog stick - Elbow and Wrist angle control
    right_stick_x_axis: int = 2  # Wrist angle
    right_stick_y_axis: int = 3  # Elbow (up/down)

    # Triggers - Wrist rotate control (L2/R2)
    left_trigger_axis: int = 12   # L2 - Wrist rotate counter-clockwise
    right_trigger_axis: int = 13  # R2 - Wrist rotate clockwise

    # PS3 controller button mappings
    button_x: int = 14      # X button (Cross) - unused
    button_circle: int = 13  # Circle button - unused
    button_square: int = 15  # Square button - unused
    button_triangle: int = 12  # Triangle button - Reset to home position

    button_l1: int = 10     # L1 button - Open gripper
    button_r1: int = 11     # R1 button - Close gripper

    button_select: int = 0  # Select button - Toggle intervention (emergency stop)
    button_start: int = 3   # Start button - Rerecord episode

    button_l3: int = 1      # Left stick button - unused
    button_r3: int = 2      # Right stick button - unused
