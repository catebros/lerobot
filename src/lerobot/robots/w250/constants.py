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
W250_HOME_POSITION = {
    "waist.pos": 0.0,
    "shoulder.pos": 0.0,
    "elbow.pos": 0.0,
    "wrist_angle.pos": -0.6,
    "wrist_rotate.pos": 0.0,
    "gripper.pos": 0.0,  # Open
}

# Rest position: Safe, compact, tucked configuration
# Use this position for:
# - Resetting between recording episodes
# - Safe storage/transport position
# - Initial position before teleop
#
# IMPORTANT NOTES:
# - wrist_angle controls the ROTATION/TWIST of the wrist (like turning a screwdriver)
# - wrist_rotate controls the PITCH/TILT of the wrist (up/down angle)
# - wrist_angle = 0.0 keeps the wrist STRAIGHT/ALIGNED (not rotated)
W250_REST_POSITION = {
    "waist.pos": 0.0,          # Centered
    "shoulder.pos": -0.5,      # Lifted up
    "elbow.pos": 0.6,          # Bent inward
    "wrist_angle.pos": 0.3,    # STRAIGHT/ALIGNED (no rotation) - THIS IS THE TWIST
    "wrist_rotate.pos": -0.3,  # Angled down (prevents camera/gripper collision) - THIS IS THE TILT
    "gripper.pos": 1.0,        # Closed
}
