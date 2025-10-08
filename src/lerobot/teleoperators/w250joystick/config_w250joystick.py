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
    """Configuration for Nintendo Switch JoyCon teleoperator using ROS2."""

    # ROS2 settings
    ros2_node_name: str = "w250_joycon_teleop"
    joy_topic: str = "/joy"

    # Control settings
    use_gripper: bool = True
    deadzone: float = 0.1

    # Step sizes for movement (in meters)
    x_step_size: float = 0.01
    y_step_size: float = 0.01
    z_step_size: float = 0.01

    # JoyCon axis mappings (using ROS2 joy node standard)
    # Left JoyCon - movement control
    left_stick_x_axis: int = 0  # Left/Right movement
    left_stick_y_axis: int = 1  # Forward/Backward movement

    # Right JoyCon - Z-axis and rotation
    right_stick_y_axis: int = 3  # Up/Down movement

    # Button mappings for JoyCons
    # Left JoyCon buttons
    button_l: int = 4  # L button
    button_zl: int = 6  # ZL button

    # Right JoyCon buttons
    button_r: int = 5  # R button
    button_zr: int = 7  # ZR button
    button_a: int = 0  # A button
    button_b: int = 1  # B button
    button_x: int = 2  # X button
    button_y: int = 3  # Y button

    # Special buttons
    button_plus: int = 9  # Plus button (success)
    button_minus: int = 8  # Minus button (rerecord)
