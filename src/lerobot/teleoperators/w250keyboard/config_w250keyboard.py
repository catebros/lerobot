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
    """Configuration for W250 keyboard teleoperator (termios raw mode, no DISPLAY needed)."""

    step_size: float = 0.02
    step_size_min: float = 0.005
    step_size_max: float = 0.1
    step_size_increment: float = 0.005
    gripper_step_size: float = 0.1
    control_hz: float = 15.0
