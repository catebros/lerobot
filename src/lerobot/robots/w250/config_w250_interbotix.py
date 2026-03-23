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
Configuration class for WidowX-250 robot using Interbotix ROS2 API
"""

import logging
from dataclasses import dataclass, field
from typing import Optional
from pathlib import Path

# Import the base Robot configuration
from ..robot import RobotConfig
from lerobot.cameras import CameraConfig

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("w250_interbotix")
@dataclass
class W250InterbotixConfig(RobotConfig):
    """
    Configuration for WidowX-250 robot using Interbotix Python API with ROS2

    This config maintains compatibility with the LeRobot interface while using
    the official Interbotix ROS2 implementation underneath.

    Frequency budget — everything must agree:
    ┌─────────────────────────────────────────────────────────────────┐
    │  fps          = control loop rate = dataset recording rate      │
    │  moving_time  = 1 / fps   (auto-computed if left as None)       │
    │  accel_time   = moving_time / 4  (auto-computed if left None)   │
    │  camera fps   = fps   (set in each CameraConfig)                │
    │  lerobot_record --dataset.fps = fps                             │
    └─────────────────────────────────────────────────────────────────┘
    Change only `fps` and the rest follow automatically.
    """

    # Robot model name for Interbotix API
    robot_model: str = "wx250s"

    # ROS2 namespace for the robot
    robot_name: str = "wx250s"

    # Joint group name (standard Interbotix configuration)
    group_name: str = "arm"

    # Gripper joint name (standard Interbotix configuration)
    gripper_name: str = "gripper"

    # ── Frequency ─────────────────────────────────────────────────────────────
    # Single source of truth for the whole system.
    # All timing parameters are derived from this value in __post_init__.
    # Must match: dataset fps, camera fps, and control loop rate.
    fps: int = 15

    # moving_time controls how fast the robot moves between positions.
    # None → auto-derived as 1/fps so the robot completes each move in exactly
    # one control period. This guarantees teleop and policy inference use the
    # same dynamics. Set explicitly (e.g. 0.15) only if you need slower teleop.
    moving_time: Optional[float] = None  # seconds  (auto: 1/fps)
    accel_time: Optional[float] = None   # seconds  (auto: moving_time / 4)

    # Gripper control parameters
    gripper_pressure: float = 0.5          # Gripper pressure (0.0-1.0)
    gripper_pressure_lower_limit: int = 150  # Minimum effort (PWM)
    gripper_pressure_upper_limit: int = 350  # Maximum effort (PWM)

    # ROS2 specific settings
    use_moveit: bool = False

    # Camera configurations (inherited from base RobotConfig)
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Motor calibration settings
    calibration_dir: Path = Path("~/.cache/huggingface/lerobot/calibration")

    # Safety limits
    max_relative_target: Optional[float] = None
    disable_torque_on_disconnect: bool = True

    # Motion profile settings (Interbotix specific)
    profile_type: str = "time_based"

    def __post_init__(self):
        """Derive timing from fps and validate configuration."""
        super().__post_init__()

        # Auto-compute timing from fps
        if self.moving_time is None:
            self.moving_time = 1.0 / self.fps
        if self.accel_time is None:
            self.accel_time = self.moving_time / 4.0

        # Validate robot model
        valid_models = ["wx200", "wx250", "wx250s", "vx250", "vx300", "vx300s"]
        if self.robot_model not in valid_models:
            logger.warning(f"Robot model '{self.robot_model}' not in standard Interbotix models: {valid_models}")

        # Validate pressure settings
        if not 0.0 <= self.gripper_pressure <= 1.0:
            raise ValueError(f"gripper_pressure must be between 0.0 and 1.0, got {self.gripper_pressure}")

        if self.gripper_pressure_lower_limit >= self.gripper_pressure_upper_limit:
            raise ValueError("gripper_pressure_lower_limit must be less than gripper_pressure_upper_limit")

        # Validate timing
        if self.accel_time > self.moving_time / 2:
            raise ValueError(
                f"accel_time ({self.accel_time:.4f}s) must be <= moving_time/2 ({self.moving_time/2:.4f}s)"
            )

        # Force all cameras to run at the same fps as the control loop.
        # If a camera was configured with fps=None (unset) or a different value,
        # override it here so the whole system is always in sync.
        for cam_name, cam_cfg in self.cameras.items():
            if cam_cfg.fps is None:
                cam_cfg.fps = self.fps
                logger.debug(f"Camera '{cam_name}': fps auto-set to {self.fps}")
            elif cam_cfg.fps != self.fps:
                logger.warning(
                    f"Camera '{cam_name}' fps={cam_cfg.fps} overridden to match robot fps={self.fps}."
                )
                cam_cfg.fps = self.fps

        # Ensure calibration directory exists
        self.calibration_dir = Path(self.calibration_dir).expanduser()
        self.calibration_dir.mkdir(parents=True, exist_ok=True)

        logger.info(
            f"W250InterbotixConfig: model={self.robot_model}  "
            f"fps={self.fps}  moving_time={self.moving_time:.4f}s  accel_time={self.accel_time:.4f}s"
        )
