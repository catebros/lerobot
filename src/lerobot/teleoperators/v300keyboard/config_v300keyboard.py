from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("v300keyboard")
@dataclass
class V300KeyboardConfig(TeleoperatorConfig):
    """
    Configuration for keyboard teleoperator for V300 robot.

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

    step_size: float = 0.02
    step_size_min: float = 0.005
    step_size_max: float = 0.1
    step_size_increment: float = 0.005
    gripper_step_size: float = 0.1
    control_hz: float = 15.0
