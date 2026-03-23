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

"""
Keyboard teleoperator for WidowX-250 robot (6DOF).

Reads keyboard input via termios raw mode — works in terminal without DISPLAY or pynput.
Produces incremental joint position commands compatible with W250Interbotix.send_action().
"""

import logging
import sys
import termios
import threading
import tty
from typing import Any

from lerobot.robots.w250.constants import W250_REST_POSITION

from ..teleoperator import Teleoperator
from ..utils import TeleopEvents
from .config_w250keyboard import W250KeyboardConfig

logger = logging.getLogger(__name__)


# Maps key character → (joint_key, direction)
# direction: +1 or -1 applied to step_size
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
_QUIT_KEYS = {"q"}  # ESC intentionally excluded: arrow keys produce \x1b[C/D/A/B which would trigger it


class W250KeyboardTeleop(Teleoperator):
    """
    Keyboard teleoperator for WidowX-250 6DOF robot.

    Maintains internal absolute joint positions and applies incremental
    changes based on key presses. Compatible with W250Interbotix.send_action().

    Uses termios raw mode — no GUI or DISPLAY required. Works in WSL terminal.
    """

    config_class = W250KeyboardConfig
    name = "w250keyboard"

    def __init__(self, config: W250KeyboardConfig):
        super().__init__(config)
        self.config = config

        self._step_size = config.step_size
        self._is_connected = False
        self._quit_requested = False

        # Current absolute positions (normalized).
        # Initialized to REST position so the first action after robot.connect()
        # (which calibrates to REST) produces zero delta — no lurch on start.
        # Call sync_with_robot() to update if the robot is somewhere else.
        self._positions: dict[str, float] = dict(W250_REST_POSITION)

        # Set to True whenever a movement key is pressed; consumed by get_teleop_events()
        self._intervention_flag: bool = False

        # One-shot flags set by P / F keys, consumed by the main loop
        self._save_pose_requested = False
        self._replay_pose_requested = False

        # Background thread that reads keyboard
        self._reader_thread: threading.Thread | None = None

        # Saved terminal settings for restore on disconnect
        self._old_termios: list | None = None

    # ── LeRobot interface ──────────────────────────────────────────────────

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
            raise RuntimeError("W250KeyboardTeleop already connected")

        if not sys.stdin.isatty():
            raise RuntimeError(
                "stdin is not a TTY — keyboard teleop requires an interactive terminal"
            )

        # Save current terminal settings and switch to cbreak mode.
        # cbreak (vs raw): no line buffering, still char-by-char, but signal
        # characters (Ctrl+C → SIGINT) still work — required for lerobot-record.
        self._old_termios = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self._is_connected = True
        self._quit_requested = False

        # Start background reader thread
        self._reader_thread = threading.Thread(
            target=self._read_keys, daemon=True, name="w250kb-reader"
        )
        self._reader_thread.start()

        logger.info("W250KeyboardTeleop connected")
        self._print_controls()

    def disconnect(self) -> None:
        if not self._is_connected:
            return

        self._is_connected = False
        self._quit_requested = True

        # Restore terminal
        if self._old_termios is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_termios)
            except Exception:
                pass
            self._old_termios = None

        logger.info("W250KeyboardTeleop disconnected")

    def get_action(self) -> dict[str, float]:
        """Return current absolute joint positions."""
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

    # ── Pose save / replay signals ─────────────────────────────────────────

    def consume_save_pose(self) -> bool:
        """Return True (once) if P was pressed since the last call."""
        if self._save_pose_requested:
            self._save_pose_requested = False
            return True
        return False

    def consume_replay_pose(self) -> bool:
        """Return True (once) if F was pressed since the last call."""
        if self._replay_pose_requested:
            self._replay_pose_requested = False
            return True
        return False

    # ── Sync with robot ────────────────────────────────────────────────────

    def sync_with_robot(self, robot_observation: dict[str, float]) -> None:
        """
        Sync internal positions with robot's actual state.
        Call after connect() and before the control loop to prevent jumps.
        """
        for key in self._positions:
            if key in robot_observation:
                self._positions[key] = float(robot_observation[key])
        logger.info("Keyboard positions synced with robot state")

    # ── Internal ───────────────────────────────────────────────────────────

    def _read_keys(self) -> None:
        """Background thread: reads chars from stdin and updates _positions.

        Arrow keys produce a 3-byte escape sequence (\x1b [ C/D/A/B).  None
        of those bytes appear in _KEY_MAP or _QUIT_KEYS, so they are silently
        ignored — pressing the right-arrow key to advance to the next episode
        (handled by pynput in init_keyboard_listener) no longer kills this
        thread.  Only 'q' quits the teleop reader.
        """
        _exit_reason = "disconnected"
        while self._is_connected:
            try:
                ch = sys.stdin.read(1)
            except Exception as e:
                _exit_reason = f"exception: {e!r}"
                break

            if not ch:
                # stdin returned empty — non-blocking flicker or transient EOF;
                # keep spinning rather than killing the thread.
                continue

            # Quit (q only — ESC is NOT a quit key here because arrow keys
            # produce \x1b[A/B/C/D which would trigger it by accident)
            if ch in _QUIT_KEYS:
                _exit_reason = "q key"
                self._quit_requested = True
                self._is_connected = False
                break

            # Step size adjustment (instant, not held)
            if ch == _STEP_INCREASE_KEY:
                self._step_size = min(
                    self.config.step_size_max,
                    self._step_size + self.config.step_size_increment
                )
                # Print without disrupting terminal (best effort)
                sys.stdout.write(f"\r[keyboard] step_size={self._step_size:.3f}  \r")
                sys.stdout.flush()
                continue

            if ch == _STEP_DECREASE_KEY:
                self._step_size = max(
                    self.config.step_size_min,
                    self._step_size - self.config.step_size_increment
                )
                sys.stdout.write(f"\r[keyboard] step_size={self._step_size:.3f}  \r")
                sys.stdout.flush()
                continue

            # Reset to REST position
            if ch == _RESET_REST_KEY:
                self._positions.update(W250_REST_POSITION)
                sys.stdout.write("\r[keyboard] Reset to REST position  \r")
                sys.stdout.flush()
                continue

            # Reset to HOME (all zeros)
            if ch == _RESET_HOME_KEY:
                for k in self._positions:
                    self._positions[k] = 0.0
                sys.stdout.write("\r[keyboard] Reset to HOME position  \r")
                sys.stdout.flush()
                continue

            # Pose save (P) / replay (F)
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

            # Gripper — clamped to [0, 1] so the dataset records a physical target position.
            # Saturation (motor stopping after a few presses) is no longer a problem because
            # send_action() computes delta = gripper_cmd - current_physical_position, not
            # delta = gripper_cmd - last_counter. Even when the counter is clamped at 1.0,
            # the delta vs physical position stays positive as long as the gripper isn't
            # fully open yet → motor keeps driving.
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
            "║      W250 Keyboard Teleoperator          ║\n"
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
