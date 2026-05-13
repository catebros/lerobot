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

class W250KeyboardTeleop(Teleoperator):

    config_class = W250KeyboardConfig
    name = "w250keyboard"

    def __init__(self, config: W250KeyboardConfig):
        super().__init__(config)
        self.config = config

        self._step_size = config.step_size
        self._is_connected = False
        self._quit_requested = False

        # Initialized to REST so the first action after robot.connect() (which calibrates
        # to REST) produces zero delta  no lurch on start. Call sync_with_robot() to update.
        self._positions: dict[str, float] = dict(W250_REST_POSITION)

        self._intervention_flag: bool = False
        self._save_pose_requested = False
        self._replay_pose_requested = False

        self._reader_thread: threading.Thread | None = None
        self._old_termios: list | None = None

    @property
    def action_features(self) -> dict[str, type]:
        return {
            "waist.pos": float,
            "shoulder.pos": float,
            "elbow.pos": float,
            "forearm_roll.pos": float,
            "wrist_angle.pos": float,
            "wrist_rotate.pos": float,
            "gripper.pos": float,
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected 

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
        self._old_termios = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self._is_connected = True
        self._quit_requested = False

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

    def sync_with_robot(self, robot_observation: dict[str, float]) -> None:
        """
        Sync internal positions with robot's actual state.
        Call after connect() and before the control loop to prevent jumps.
        """
        for key in self._positions:
            if key in robot_observation:
                self._positions[key] = float(robot_observation[key])
        logger.info("Keyboard positions synced with robot state")

