"""
Keyboard teleoperator for ViperX-300 S robot (6DOF).

Reads keyboard input via termios raw mode — works in terminal without DISPLAY or pynput.
Produces incremental joint position commands compatible with V300Interbotix.send_action().
"""

import logging
import sys
import termios
import threading
import tty
from typing import Any

from lerobot.robots.v300.constants import V300_REST_POSITION

from ..teleoperator import Teleoperator
from ..utils import TeleopEvents
from .config_v300keyboard import V300KeyboardConfig

logger = logging.getLogger(__name__)


_KEY_MAP: dict[str, tuple[str, float]] = {
    # Waist
    "a": ("waist.pos", +1.0),
    "d": ("waist.pos", -1.0),
    # Shoulder
    "w": ("shoulder.pos", +1.0),
    "s": ("shoulder.pos", -1.0),
    # Elbow
    "i": ("elbow.pos", +1.0),
    "k": ("elbow.pos", -1.0),
    # Forearm roll
    "j": ("forearm_roll.pos", +1.0),
    "l": ("forearm_roll.pos", -1.0),
    # Wrist angle
    "u": ("wrist_angle.pos", +1.0),
    "o": ("wrist_angle.pos", -1.0),
    # Wrist rotate
    "t": ("wrist_rotate.pos", +1.0),
    "y": ("wrist_rotate.pos", -1.0),
}

_GRIPPER_OPEN_KEY = "g"
_GRIPPER_CLOSE_KEY = "h"
_RESET_REST_KEY = "r"
_RESET_HOME_KEY = "0"
_STEP_INCREASE_KEY = "+"
_STEP_DECREASE_KEY = "-"
_QUIT_KEYS = {"q"}


class V300KeyboardTeleop(Teleoperator):
    """
    Keyboard teleoperator for ViperX-300 S 6DOF robot.

    Maintains internal absolute joint positions and applies incremental
    changes based on key presses. Compatible with V300Interbotix.send_action().

    Uses termios raw mode — no GUI or DISPLAY required. Works in WSL terminal.
    """

    config_class = V300KeyboardConfig
    name = "v300keyboard"

    def __init__(self, config: V300KeyboardConfig):
        super().__init__(config)
        self.config = config

        self._step_size = config.step_size
        self._is_connected = False
        self._quit_requested = False

        self._positions: dict[str, float] = dict(V300_REST_POSITION)

        self._intervention_flag: bool = False
        self._save_pose_requested = False
        self._replay_pose_requested = False

        self._reader_thread: threading.Thread | None = None
        self._old_termios: list | None = None

    @property
    def action_features(self) -> dict[str, type]:
        return {
            "waist.pos":        float,
            "shoulder.pos":     float,
            "elbow.pos":        float,
            "forearm_roll.pos": float,
            "wrist_angle.pos":  float,
            "wrist_rotate.pos": float,
            "gripper.pos":      float,
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected and (
            self._reader_thread is not None and self._reader_thread.is_alive()
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self) -> None:
        if self._is_connected:
            raise RuntimeError("V300KeyboardTeleop already connected")

        if not sys.stdin.isatty():
            raise RuntimeError(
                "stdin is not a TTY — keyboard teleop requires an interactive terminal"
            )

        self._old_termios = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self._is_connected = True
        self._quit_requested = False

        self._reader_thread = threading.Thread(
            target=self._read_keys, daemon=True, name="v300kb-reader"
        )
        self._reader_thread.start()

        logger.info("V300KeyboardTeleop connected")
        self._print_controls()

    def disconnect(self) -> None:
        if not self._is_connected:
            return

        self._is_connected = False
        self._quit_requested = True

        if self._old_termios is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_termios)
            except Exception:
                pass
            self._old_termios = None

        logger.info("V300KeyboardTeleop disconnected")

    def get_action(self) -> dict[str, float]:
        return dict(self._positions)

    def get_teleop_events(self) -> dict[str, Any]:
        is_intervention = self._intervention_flag
        self._intervention_flag = False
        return {
            TeleopEvents.IS_INTERVENTION: is_intervention,
            TeleopEvents.TERMINATE_EPISODE: self._quit_requested,
            TeleopEvents.SUCCESS: False,
            TeleopEvents.RERECORD_EPISODE: False,
        }

    def send_feedback(self, feedback: dict) -> None:
        pass

    def consume_save_pose(self) -> bool:
        if self._save_pose_requested:
            self._save_pose_requested = False
            return True
        return False

    def consume_replay_pose(self) -> bool:
        if self._replay_pose_requested:
            self._replay_pose_requested = False
            return True
        return False

    def sync_with_robot(self, robot_observation: dict[str, float]) -> None:
        for key in self._positions:
            if key in robot_observation:
                self._positions[key] = float(robot_observation[key])
        logger.info("Keyboard positions synced with robot state")

    def _read_keys(self) -> None:
        while self._is_connected:
            try:
                ch = sys.stdin.read(1)
            except Exception as e:
                break

            if not ch:
                continue

            if ch in _QUIT_KEYS:
                self._quit_requested = True
                self._is_connected = False
                break

            if ch == _STEP_INCREASE_KEY:
                self._step_size = min(self.config.step_size_max, self._step_size + self.config.step_size_increment)
                sys.stdout.write(f"\r[keyboard] step_size={self._step_size:.3f}  \r")
                sys.stdout.flush()
                continue

            if ch == _STEP_DECREASE_KEY:
                self._step_size = max(self.config.step_size_min, self._step_size - self.config.step_size_increment)
                sys.stdout.write(f"\r[keyboard] step_size={self._step_size:.3f}  \r")
                sys.stdout.flush()
                continue

            if ch == _RESET_REST_KEY:
                self._positions.update(V300_REST_POSITION)
                sys.stdout.write("\r[keyboard] Reset to REST position  \r")
                sys.stdout.flush()
                continue

            if ch == _RESET_HOME_KEY:
                for k in self._positions:
                    self._positions[k] = 0.0
                sys.stdout.write("\r[keyboard] Reset to HOME position  \r")
                sys.stdout.flush()
                continue

            if ch == "p":
                self._save_pose_requested = True
                sys.stdout.write("\r[keyboard] Saving pose...             \r")
                sys.stdout.flush()
                continue

            if ch == "f":
                self._replay_pose_requested = True
                sys.stdout.write("\r[keyboard] Replaying pose...          \r")
                sys.stdout.flush()
                continue

            if ch == _GRIPPER_OPEN_KEY:
                self._positions["gripper.pos"] = min(1.0, self._positions["gripper.pos"] + self.config.gripper_step_size)
                self._intervention_flag = True
                continue

            if ch == _GRIPPER_CLOSE_KEY:
                self._positions["gripper.pos"] = max(0.0, self._positions["gripper.pos"] - self.config.gripper_step_size)
                self._intervention_flag = True
                continue

            if ch in _KEY_MAP:
                joint, direction = _KEY_MAP[ch]
                delta = direction * self._step_size
                self._positions[joint] = max(-1.0, min(1.0, self._positions[joint] + delta))
                self._intervention_flag = True

    def _print_controls(self) -> None:
        controls = (
            "\n"
            "╔══════════════════════════════════════════╗\n"
            "║      V300 Keyboard Teleoperator          ║\n"
            "╠══════════════════════════════════════════╣\n"
            "║  A / D   → Waist       (left / right)   ║\n"
            "║  W / S   → Shoulder    (up / down)      ║\n"
            "║  I / K   → Elbow       (up / down)      ║\n"
            "║  J / L   → Forearm roll                 ║\n"
            "║  U / O   → Wrist angle                  ║\n"
            "║  T / Y   → Wrist rotate                 ║\n"
            "║  G       → Gripper open                 ║\n"
            "║  H       → Gripper close                ║\n"
            "║  R       → Reset to REST position       ║\n"
            "║  0       → Reset to HOME (all zeros)    ║\n"
            "║  + / -   → Step size up / down          ║\n"
            "║  P       → Save current pose            ║\n"
            "║  F       → Replay saved pose + error    ║\n"
            "║  Q       → Quit                         ║\n"
            "╚══════════════════════════════════════════╝\n"
        )
        sys.stdout.write(controls)
        sys.stdout.flush()
