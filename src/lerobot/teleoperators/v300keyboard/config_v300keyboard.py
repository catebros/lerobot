from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("v300keyboard")
@dataclass
class V300KeyboardConfig(TeleoperatorConfig):
    """Configuration for V300 keyboard teleoperator (termios raw mode, no DISPLAY needed)."""

    step_size: float = 0.02
    step_size_min: float = 0.005
    step_size_max: float = 0.1
    step_size_increment: float = 0.005
    gripper_step_size: float = 0.1
    control_hz: float = 15.0
