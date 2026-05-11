import logging
from dataclasses import dataclass, field
from typing import Optional
from pathlib import Path

from ..robot import RobotConfig
from lerobot.cameras import CameraConfig

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("v300_interbotix")
@dataclass
class V300InterbotixConfig(RobotConfig):
    """
    Configuration for ViperX-300 S robot using Interbotix Python API with ROS2

    Change only `fps` — moving_time and accel_time are derived automatically.
    All cameras and the dataset must use the same fps.
    """

    robot_model: str = "vx300s"
    robot_name: str = "vx300s"
    group_name: str = "arm"
    gripper_name: str = "gripper"

    fps: int = 15
    moving_time: Optional[float] = None  # auto: 1/fps
    accel_time: Optional[float] = None   # auto: moving_time / 4

    gripper_pressure: float = 0.5
    gripper_pressure_lower_limit: int = 150
    gripper_pressure_upper_limit: int = 350

    use_moveit: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    calibration_dir: Path = Path("~/.cache/huggingface/lerobot/calibration")
    max_relative_target: Optional[float] = None
    disable_torque_on_disconnect: bool = True
    profile_type: str = "time_based"

    def __post_init__(self):
        """Derive timing from fps and validate configuration."""
        super().__post_init__()

        if self.moving_time is None:
            self.moving_time = 1.0 / self.fps
        if self.accel_time is None:
            self.accel_time = self.moving_time / 4.0

        valid_models = ["vx250", "vx300", "vx300s"]
        if self.robot_model not in valid_models:
            logger.warning(f"Robot model '{self.robot_model}' not in standard ViperX models: {valid_models}")

        if not 0.0 <= self.gripper_pressure <= 1.0:
            raise ValueError(f"gripper_pressure must be between 0.0 and 1.0, got {self.gripper_pressure}")

        if self.gripper_pressure_lower_limit >= self.gripper_pressure_upper_limit:
            raise ValueError("gripper_pressure_lower_limit must be less than gripper_pressure_upper_limit")

        if self.accel_time > self.moving_time / 2:
            raise ValueError(
                f"accel_time ({self.accel_time:.4f}s) must be <= moving_time/2 ({self.moving_time/2:.4f}s)"
            )

        for cam_name, cam_cfg in self.cameras.items():
            if cam_cfg.fps is None:
                cam_cfg.fps = self.fps
                logger.debug(f"Camera '{cam_name}': fps auto-set to {self.fps}")
            elif cam_cfg.fps != self.fps:
                logger.warning(
                    f"Camera '{cam_name}' fps={cam_cfg.fps} overridden to match robot fps={self.fps}."
                )
                cam_cfg.fps = self.fps

        self.calibration_dir = Path(self.calibration_dir).expanduser()
        self.calibration_dir.mkdir(parents=True, exist_ok=True)

        logger.info(
            f"V300InterbotixConfig: model={self.robot_model}  "
            f"fps={self.fps}  moving_time={self.moving_time:.4f}s  accel_time={self.accel_time:.4f}s"
        )
