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


@TeleoperatorConfig.register_subclass("w250keyboard")
@dataclass
class W250KeyboardConfig(TeleoperatorConfig):
    """
    Configuration for keyboard teleoperator for W250 robot.

    Works in terminal (no DISPLAY required) — uses termios raw mode.

    Controls:
        A / D       → Waist       (rotate left / right)
        W / S       → Shoulder    (up / down)
        I / K       → Elbow       (up / down)
        J / L       → Forearm roll (left / right)
        U / O       → Wrist angle  (up / down)
        T / Y       → Wrist rotate (left / right)
        G           → Gripper open
        H           → Gripper close
        R           → Reset to REST position
        0           → Reset to HOME (all joints at 0)
        +  / -      → Increase / decrease step size
        Q / ESC     → Quit
    """

    # Step size per keypress (normalized units, arm joints in [-1,1])
    step_size: float = 0.02

    # Min/max step size when using +/- keys
    step_size_min: float = 0.005
    step_size_max: float = 0.1
    step_size_increment: float = 0.005

    # Gripper step size (range [0,1])
    gripper_step_size: float = 0.1

    # Control loop frequency (Hz) — MUST match robot fps and dataset fps.
    # Default matches W250InterbotixConfig.fps default (10 Hz).
    control_hz: float = 12.0
