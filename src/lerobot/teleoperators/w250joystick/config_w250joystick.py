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
    """Configuration for Nintendo Switch Left JoyCon teleoperator using ROS2."""

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

    # Left JoyCon axis mappings (using ROS2 joy node standard)
    # Analog stick for X-Y plane movement
    stick_x_axis: int = 0  # Left/Right movement (Y axis)
    stick_y_axis: int = 1  # Forward/Backward movement (X axis)

    # Left JoyCon button mappings
    button_up: int = 0  # D-pad Up - Move up (Z+)
    button_down: int = 1  # D-pad Down - Move down (Z-)
    button_left: int = 2  # D-pad Left
    button_right: int = 3  # D-pad Right

    button_l: int = 4  # L button - Intervention
    button_zl: int = 6  # ZL button - Close gripper

    button_minus: int = 8  # Minus button - Open gripper
    button_capture: int = 9  # Capture button - Rerecord episode
    button_stick: int = 10  # Stick button (L3) - Success
