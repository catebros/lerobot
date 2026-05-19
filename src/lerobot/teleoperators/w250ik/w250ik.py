import json
import logging
import math
from pathlib import Path
from typing import Any

import modern_robotics as mr
import numpy as np
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd

from lerobot.robots.w250.constants import (
    W250_JOINT_LIMITS,
    W250_JOINT_NAMES,
    W250_REST_POSITION,
)

from ..teleoperator import Teleoperator
from ..utils import TeleopEvents
from .config_w250ik import W250IKConfig

logger = logging.getLogger(__name__)

_TWO_PI = 2.0 * math.pi


# ── Joint-space helpers ───────────────────────────────────────────────────────

def _wrap_theta_list(theta_list, lower_limits, upper_limits):
    theta_list = (theta_list + math.pi) % _TWO_PI - math.pi
    for i, (lo, hi) in enumerate(zip(lower_limits, upper_limits)):
        if round(float(theta_list[i]), 3) < round(lo, 3):
            theta_list[i] += _TWO_PI
        elif round(float(theta_list[i]), 3) > round(hi, 3):
            theta_list[i] -= _TWO_PI
    return theta_list


def _check_joint_limits(theta_list, lower_limits, upper_limits):
    for theta, lo, hi in zip(theta_list, lower_limits, upper_limits):
        if float(theta) < lo - 1e-3 or float(theta) > hi + 1e-3:
            return False
    return True


def _normalize(joint_name, rad):
    lo, hi = W250_JOINT_LIMITS[joint_name]
    return 2.0 * (rad - lo) / (hi - lo) - 1.0


def _theta_list_to_normalized(theta_list):
    return {
        f"{name}.pos": _normalize(name, float(rad))
        for name, rad in zip(W250_JOINT_NAMES, theta_list)
    }


# ── Type aliases ──────────────────────────────────────────────────────────────

_Waypoints = list[tuple[dict[str, float], int]]
_Frames    = list[dict[str, float]]
_Episode   = tuple[_Frames, _Frames]


def _expand_to_frames(waypoints: _Waypoints) -> _Frames:
    """Linear interpolation between waypoints → per-frame list."""
    if not waypoints:
        return []
    frames: _Frames = []
    pos0, n0 = waypoints[0]
    frames.extend([dict(pos0)] * n0)
    for i in range(1, len(waypoints)):
        pos_prev = waypoints[i - 1][0]
        pos_curr, n = waypoints[i]
        for j in range(n):
            t = (j + 1) / n
            frames.append({k: pos_prev[k] + (pos_curr[k] - pos_prev[k]) * t for k in pos_curr})
    return frames


# ── Teleoperator ──────────────────────────────────────────────────────────────

class W250IKTeleop(Teleoperator):
    """Autonomous IK teleoperator for the WidowX-250."""

    config_class = W250IKConfig
    name = "w250ik"

    def __init__(self, config: W250IKConfig):
        super().__init__(config)
        self.config = config
        self._is_connected: bool = False
        self._episodes: list[_Episode] = []
        self._raw_episodes: list[dict] = []
        self._ep_idx:    int  = 0
        self._in_reset:  bool = False
        self._frame:     int  = 0
        self._ep_done:   bool = False
        self._rst_held:  bool = False  # True while holding last reset frame

    # ── Teleoperator interface ────────────────────────────────────────────────

    @property
    def action_features(self):
        return {f"{j}.pos": float for j in W250_JOINT_NAMES + ["gripper"]}

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self._is_connected

    @property
    def is_calibrated(self):
        return True

    def calibrate(self): pass
    def configure(self): pass

    def _frames(self, duration_s: float) -> int:
        return max(1, round(duration_s * self.config.fps))

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise RuntimeError("W250IKTeleop already connected")
        logger.info(f"W250IKTeleop: fps={self.config.fps}")
        logger.info(f"W250IKTeleop: loading positions from {self.config.positions_file}")
        self._episodes = self._build_all_episodes()
        n_rec   = sum(len(rec) for rec, _   in self._episodes)
        n_reset = sum(len(rst) for _,   rst in self._episodes)
        logger.info(
            f"W250IKTeleop: {len(self._episodes)} episodes ready "
            f"({n_rec} record frames + {n_reset} reset frames)"
        )
        self._ep_idx = self.config.start_episode_idx
        self._in_reset = False
        self._frame  = 0; self._ep_done  = False
        self._is_connected = True

        if self._ep_idx > 0:
            logger.info(f"W250IKTeleop: resuming from episode {self._ep_idx + 1}/{len(self._episodes)}")

        # Print first-episode placement instructions — no pointing phase runs before ep 1.
        first_ep = self._raw_episodes[self._ep_idx]
        label = first_ep.get("label", f"ep{self._ep_idx + 1}")
        print("\n" + "="*60)
        print(f"  BEFORE STARTING: place the bowl for EPISODE {self._ep_idx + 1}")
        print(f"  Label : {label}")
        print(f"  x={first_ep['x']:.4f}  y={first_ep['y']:.4f}  z={first_ep['z']:.4f}")
        print(f"  roll  : {math.degrees(first_ep['roll']):.1f}°")
        print("  Press Ctrl+C to abort, or just start recording once ready.")
        print("="*60 + "\n")

    def disconnect(self) -> None:
        self._is_connected = False
        logger.info("W250IKTeleop disconnected")

    def get_action(self) -> dict[str, float]:
        if not self._is_connected:
            raise RuntimeError("W250IKTeleop is not connected")
        rec_frames, rst_frames = self._episodes[self._ep_idx]
        frames = rst_frames if self._in_reset else rec_frames
        idx = min(self._frame, len(frames) - 1)
        pos = frames[idx]
        self._frame += 1
        if self._frame >= len(frames):
            if not self._in_reset:
                self._frame = len(frames)
                self._ep_done = True
            else:
                # Hold at last reset frame; _advance_episode() is called by
                # lerobot_record after the reset loop via on_episode_end().
                self._frame = len(frames) - 1
                self._rst_held = True
        return dict(pos)

    def get_teleop_events(self) -> dict[str, Any]:
        done = self._ep_done
        if done:
            self._ep_done  = False
            self._in_reset = True
            self._frame    = 0
            logger.info(f"W250IKTeleop: ep {self._ep_idx + 1} done — entering reset")
        return {
            TeleopEvents.TERMINATE_EPISODE: done,
            TeleopEvents.SUCCESS:           done,
            TeleopEvents.IS_INTERVENTION:   False,
            TeleopEvents.RERECORD_EPISODE:  False,
        }

    def send_feedback(self, feedback: dict) -> None:
        pass

    def on_episode_end(self) -> None:
        """Called by lerobot_record after the reset loop ends to advance to the next episode."""
        if self._in_reset:
            self._rst_held = False
            self._advance_episode()

    def _advance_episode(self) -> None:
        next_idx = self._ep_idx + 1
        if next_idx >= len(self._episodes):
            logger.info("W250IKTeleop: all episodes completed.")
            return
        self._ep_idx   = next_idx
        self._in_reset = False
        self._frame    = 0
        logger.info(f"W250IKTeleop: ready for episode {self._ep_idx + 1}/{len(self._episodes)}")

    @property
    def current_episode_index(self): return self._ep_idx

    @property
    def total_episodes(self): return len(self._episodes)

    # ── IK ────────────────────────────────────────────────────────────────────

    def _get_robot_description(self):
        model_key = self.config.robot_model.replace("-", "_")
        robot_des = getattr(mrd, model_key, None)
        if robot_des is None:
            raise ValueError(f"Robot model '{self.config.robot_model}' not found in mr_descriptions.")
        return robot_des

    def _ik_to_waypoint(
        self,
        robot_des,
        lower_limits,
        upper_limits,
        label: str,
        x: float, y: float, z: float,
        roll: float,
        gripper: float,
        n_frames: int,
    ) -> tuple[dict[str, float], int]:
        """
        Solve IK for an EE target and return (joint_positions, n_frames).

        Target orientation uses the interbotix EE convention:
          col 0  EE-X = (0, 0, -1)                — approach: gripper points down
          col 1  EE-Y = (-cos(roll), sin(roll), 0) — jaw axis, fixed in world frame
          col 2  EE-Z = EE-X × EE-Y               — right-hand complement

        The rotation matrix is constant for a given roll offset — the IK
        finds the correct waist and wrist joints for each (x, y, z) automatically.
        """
        c, s = math.cos(roll), math.sin(roll)
        R = np.array([
            [0.0,  -c,   s],
            [0.0,   s,   c],
            [-1.0,  0.0, 0.0],
        ])
        T_sd = np.eye(4)
        T_sd[:3, :3] = R
        T_sd[:3, 3]  = [x, y, z]

        yaw = math.atan2(y, x)
        n   = robot_des.Slist.shape[1]
        seeds = [
            np.array([yaw] + [0.0] * (n - 1)),
            np.zeros(n),
            np.array([-math.pi / 30 * 40] + [0.0] * (n - 1)),
            np.array([math.pi / 30 * 40]  + [0.0] * (n - 1)),
        ]

        theta = np.zeros(n)
        ok    = False
        for seed in seeds:
            theta, success = mr.IKinSpace(
                Slist=robot_des.Slist, M=robot_des.M, T=T_sd,
                thetalist0=seed, eomg=0.001, ev=0.001,
            )
            if success:
                theta = _wrap_theta_list(theta, lower_limits, upper_limits)
                if _check_joint_limits(theta, lower_limits, upper_limits):
                    ok = True
                    break

        if not ok:
            raise ValueError(
                f"IK failed for '{label}' at "
                f"(x={x:.3f}, y={y:.3f}, z={z:.3f}, roll={math.degrees(roll):.1f}°). "
                "Check that the position is within reach and adjust bowl_positions.json."
            )

        if label in ("hover", "centre"):
            T_ee = mr.FKinSpace(robot_des.M, robot_des.Slist, theta)
            jaw  = math.degrees(math.atan2(float(T_ee[1, 1]), float(T_ee[0, 1])))
            logger.info(
                f"  {label:8s}  ({x:.3f},{y:.3f},{z:.3f})"
                f"  waist={math.degrees(float(theta[0])):+.1f}°"
                f"  wrist_r={math.degrees(float(theta[5])):+.1f}°"
                f"  jaw_world={jaw:+.1f}°"
            )

        pos_dict = _theta_list_to_normalized(theta)
        pos_dict["gripper.pos"] = float(gripper)
        return pos_dict, n_frames

    def _build_pick_place_episode(
        self,
        robot_des,
        lower_limits,
        upper_limits,
        x_bowl: float, y_bowl: float, z_bowl: float, roll: float,
        center_x: float, center_y: float, center_z: float,
        next_bowl: tuple[float, float, float, float] | None = None,
    ) -> tuple[_Waypoints, _Waypoints]:
        cfg = self.config
        OPEN, CLOSED = 1.0, 0.0

        def wp(label, x, y, z, r, grip, n_frames):
            return self._ik_to_waypoint(
                robot_des, lower_limits, upper_limits,
                label, x, y, z, r, grip, n_frames,
            )

        def bowl_wp(label, x, y, z, grip, n_frames):
            return wp(label, x, y, z, roll, grip, n_frames)

        def centre_wp(grip, n_frames):
            return wp("centre", center_x, center_y, center_z, 0.0, grip, n_frames)

        rest_dict = {**W250_REST_POSITION, "gripper.pos": OPEN}
        z_hover = z_bowl + cfg.approach_height
        z_lift  = z_bowl + cfg.lift_height
        f = self._frames

        record_wps = [
            (rest_dict,                                                         f(cfg.duration_rest_start)),
            bowl_wp("hover",   x_bowl, y_bowl, z_hover,    OPEN,   f(cfg.duration_approach)),
            bowl_wp("descend", x_bowl, y_bowl, z_bowl,     OPEN,   f(cfg.duration_descend)),
            bowl_wp("grip",    x_bowl, y_bowl, z_bowl,     CLOSED, f(cfg.duration_close_gripper)),
            bowl_wp("lift",    x_bowl, y_bowl, z_lift,     CLOSED, f(cfg.duration_lift)),
            centre_wp(CLOSED, f(cfg.duration_transport)),
            centre_wp(CLOSED, f(cfg.duration_centre_hold)),
        ]

        reset_wps: _Waypoints = [
            centre_wp(OPEN, f(cfg.duration_release)),
        ]

        # If there is a next bowl position and pointing is enabled, hover over it
        # so the user can place the bowl directly under the gripper.
        if next_bowl is not None and cfg.duration_pointing > 0:
            nx, ny, nz, nroll = next_bowl
            nz_hover = nz + cfg.approach_height
            reset_wps.append(
                wp("point_next", nx, ny, nz_hover, nroll, OPEN, f(cfg.duration_pointing))
            )

        reset_wps.append((rest_dict, f(cfg.duration_rest_end)))

        return record_wps, reset_wps

    def _build_all_episodes(self) -> list[_Episode]:
        cfg = self.config
        positions_path = Path(cfg.positions_file)
        if not positions_path.exists():
            raise FileNotFoundError(
                f"Positions file not found: {positions_path}\n"
                "Run record_bowl_positions.py to create it."
            )

        with open(positions_path) as f:
            data = json.load(f)

        if "center" in data:
            center = data["center"]
            cx, cy, cz = float(center["x"]), float(center["y"]), float(center["z"])
            logger.info(f"Center: ({cx:.3f}, {cy:.3f}, {cz:.3f})  [from JSON]")
        else:
            cx, cy, cz = cfg.center_x, cfg.center_y, cfg.center_z
            logger.info(f"Center: ({cx:.3f}, {cy:.3f}, {cz:.3f})  [from config defaults]")

        robot_des    = self._get_robot_description()
        lower_limits = [W250_JOINT_LIMITS[n][0] for n in W250_JOINT_NAMES]
        upper_limits  = [W250_JOINT_LIMITS[n][1] for n in W250_JOINT_NAMES]

        raw = data["episodes"]
        self._raw_episodes = raw
        episodes = []
        for i, ep in enumerate(raw):
            label = ep.get("label", f"ep{i+1}")
            x, y, z = float(ep["x"]), float(ep["y"]), float(ep["z"])
            roll    = float(ep.get("roll", 0.0))

            # Next bowl for the pointing step (None on the last episode)
            if i + 1 < len(raw):
                nep  = raw[i + 1]
                next_bowl = (float(nep["x"]), float(nep["y"]), float(nep["z"]), float(nep.get("roll", 0.0)))
                next_label = nep.get("label", f"ep{i+2}")
                logger.info(
                    f"Episode {i+1:2d} '{label}': ({x:.3f},{y:.3f},{z:.3f}) roll={math.degrees(roll):.1f}°"
                    f"  →  next: '{next_label}'"
                )
            else:
                next_bowl  = None
                next_label = None
                logger.info(f"Episode {i+1:2d} '{label}': ({x:.3f},{y:.3f},{z:.3f}) roll={math.degrees(roll):.1f}°  [last]")

            rec_wps, rst_wps = self._build_pick_place_episode(
                robot_des, lower_limits, upper_limits,
                x, y, z, roll, cx, cy, cz,
                next_bowl=next_bowl,
            )
            episodes.append((_expand_to_frames(rec_wps), _expand_to_frames(rst_wps)))

        rec_total = sum(len(rec) for rec, _ in episodes)
        rst_total = sum(len(rst) for _, rst in episodes)
        logger.info(
            f"Built {len(episodes)} episodes — "
            f"record: {rec_total} frames ({rec_total/cfg.fps:.1f}s), "
            f"reset: {rst_total} frames ({rst_total/cfg.fps:.1f}s)"
        )
        return episodes
