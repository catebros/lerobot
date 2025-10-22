# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""
Constants for W250 robot, including standard positions.

These positions are used across different components (robot, teleoperator, tests)
to ensure consistency.
"""

# Home position: All joints at neutral (0.0 normalized)
# This is the standard calibration position
# WidowX 250 6DOF configuration (7 joints + gripper)
W250_HOME_POSITION = {
    "waist.pos": 0.0,
    "shoulder.pos": 0.0,
    "elbow.pos": 0.0,
    "forearm_roll.pos": 0.0,  # Forearm rotation (6DOF model)
    "wrist_angle.pos": 0.0,
    "wrist_rotate.pos": 0.0,
    "gripper.pos": 0.0,  # Open
}

# Rest position: Safe, compact, tucked configuration
# Use this position for:
# - Resetting between recording episodes
# - Safe storage/transport position
# - Initial position before teleop
#
# IMPORTANT NOTES (6DOF model):
# - forearm_roll: ROTATION of the forearm (joint 6)
# - wrist_angle: ROTATION/TWIST of the wrist
# - wrist_rotate: PITCH/TILT of the wrist (up/down angle)
W250_REST_POSITION = {
    "waist.pos": 0.0,
    "shoulder.pos": -0.6, #
    "elbow.pos": 0.4, #
    "forearm_roll.pos": 0.0,
    "wrist_angle.pos": 0.73, #
    "wrist_rotate.pos": 0.0,
    "gripper.pos": 1.0, #
}
