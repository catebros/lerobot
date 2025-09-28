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

# Direct LeRobot implementation
from .config_w250 import W250Config
from .w250 import W250

# Interbotix ROS2 API implementation (optional)
try:
    from .config_w250_interbotix import W250InterbotixConfig
    from .w250_interbotix import W250Interbotix
    __all__ = ["W250Config", "W250", "W250InterbotixConfig", "W250Interbotix"]
except ImportError as e:
    # Interbotix API not available, only direct implementation
    print(f"Note: Interbotix ROS2 API not available ({e}). Using direct implementation only.")
    __all__ = ["W250Config", "W250"]
