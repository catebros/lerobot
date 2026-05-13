import logging
import math
from typing import Any

import modern_robotics as mr
import numpy as np
from interbotix_common_modules.angle_manipulation import euler_angles_to_rotation_matrix
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


def _wrap_theta_list(
    theta_list: np.ndarray,
    lower_limits: list[float],
    upper_limits: list[float],
) -> np.ndarray:
    """Wrap joint angles to [-π, π) then shift into [lower, upper] if needed."""
    theta_list = (theta_list + math.pi) % _TWO_PI - math.pi
    for i, (lo, hi) in enumerate(zip(lower_limits, upper_limits)):
        if round(float(theta_list[i]), 3) < round(lo, 3):
            theta_list[i] += _TWO_PI
        elif round(float(theta_list[i]), 3) > round(hi, 3):
            theta_list[i] -= _TWO_PI
    return theta_list


def _check_joint_limits(
    theta_list: np.ndarray,
    lower_limits: list[float],
    upper_limits: list[float],
) -> bool:
    for theta, lo, hi in zip(theta_list, lower_limits, upper_limits):
        if float(theta) < lo - 1e-3 or float(theta) > hi + 1e-3:
            return False
    return True


def _normalize(joint_name: str, rad: float) -> float:
    lo, hi = W250_JOINT_LIMITS[joint_name]
    return 2.0 * (rad - lo) / (hi - lo) - 1.0


def _theta_list_to_normalized(theta_list: np.ndarray) -> dict[str, float]:
    return {
        f"{name}.pos": _normalize(name, float(rad))
        for name, rad in zip(W250_JOINT_NAMES, theta_list)
    }


def _solve_ik(
    robot_des: Any,
    lower_limits: list[float],
    upper_limits: list[float],
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float | None = None,
) -> tuple[np.ndarray, bool]:
    """
    Solve IK using the same approach as interbotix's set_ee_pose_components.

    Returns (theta_list, success).  theta_list is always populated (last
    attempted solution) even when success=False, so callers can log it.
    """
    n_joints = robot_des.Slist.shape[1]

    # Auto-compute yaw so the gripper faces the bowl (same logic as interbotix)
    if yaw is None:
        yaw = math.atan2(y, x)

    # Build 4×4 target transform — identical to interbotix set_ee_pose_matrix
    T_sd = np.eye(4)
    T_sd[:3, :3] = euler_angles_to_rotation_matrix([roll, pitch, yaw])
    T_sd[:3, 3] = [x, y, z]

    # Three initial guesses used by interbotix (see arm.py ~line 223)
    initial_guesses = [
        np.zeros(n_joints),
        np.array([-math.pi / 30 * 40] + [0.0] * (n_joints - 1)),  # -2.094 rad
        np.array([math.pi / 30 * 40] + [0.0] * (n_joints - 1)),   # +2.094 rad
    ]

    last_theta = np.zeros(n_joints)
    for guess in initial_guesses:
        theta_list, success = mr.IKinSpace(
            Slist=robot_des.Slist,
            M=robot_des.M,
            T=T_sd,
            thetalist0=guess,
            eomg=0.001,
            ev=0.001,
        )
        last_theta = theta_list
        if success:
            theta_list = _wrap_theta_list(theta_list, lower_limits, upper_limits)
            if _check_joint_limits(theta_list, lower_limits, upper_limits):
                return theta_list, True

    return last_theta, False


# Teleoperator
_Waypoints = list[tuple[dict[str, float], int]]   # (joint_positions, n_frames)
_Frames    = list[dict[str, float]]               # one dict per system tick
_Episode   = tuple[_Frames, _Frames]              # (record_frames, reset_frames)


def _expand_to_frames(waypoints: _Waypoints) -> _Frames:
    """
    Convert a list of (joint_positions, n_frames) waypoints into a flat
    per-frame sequence via joint-space linear interpolation.

    The first waypoint is held for its full n_frames (robot is already there).
    Every subsequent waypoint is reached by interpolating from the previous
    position over n_frames steps — one new joint target per system tick.

    Gripper is also interpolated, giving a smooth open/close motion over the
    configured frame count instead of an abrupt switch.
    """
    if not waypoints:
        return []

    frames: _Frames = []

    # First waypoint: hold (robot is already at REST / starting position)
    pos0, n0 = waypoints[0]
    frames.extend([dict(pos0)] * n0)

    # Remaining waypoints: linear interpolation from previous to current
    for i in range(1, len(waypoints)):
        pos_prev = waypoints[i - 1][0]
        pos_curr, n = waypoints[i]
        for j in range(n):
            t = (j + 1) / n          # t goes from 1/n … 1.0
            frame = {
                k: pos_prev[k] + (pos_curr[k] - pos_prev[k]) * t
                for k in pos_curr
            }
            frames.append(frame)

    return frames


class W250IKTeleop(Teleoperator):
    """
    Autonomous IK teleoperator for the WidowX-250. """

    config_class = W250IKConfig
    name = "w250ik"

    def __init__(self, config: W250IKConfig):
        super().__init__(config)
        self.config = config

        self._is_connected: bool = False

        # Pre-computed episodes — each is (record_frames, reset_frames).
        # Every entry is a flat per-tick list: one joint-position dict per frame.
        self._episodes: list[_Episode] = []

        # Playback state
        self._ep_idx:   int  = 0      # which episode
        self._in_reset: bool = False  # True while serving reset frames
        self._frame:    int  = 0      # index into the active frame list

        # Set True when the last record frame is reached; consumed by get_teleop_events()
        self._ep_done: bool = False

    # Teleoperator interface

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in W250_JOINT_NAMES + ["gripper"]}

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

    def _frames(self, duration_s: float) -> int:
        """Convert a duration in seconds to a frame count at the configured fps."""
        return max(1, round(duration_s * self.config.fps))

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise RuntimeError("W250IKTeleop already connected")

        logger.info(
            f"W250IKTeleop: fps={self.config.fps}  "
            f"(must match robot.fps and --dataset.fps — all cameras are forced to robot.fps)"
        )
        logger.info("W250IKTeleop: solving IK for all episodes …")
        self._episodes = self._build_all_episodes()
        n_rec   = sum(len(rec) for rec, _   in self._episodes)
        n_reset = sum(len(rst) for _,   rst in self._episodes)
        logger.info(
            f"W250IKTeleop: {len(self._episodes)} episodes ready "
            f"({n_rec} record waypoints + {n_reset} reset waypoints)"
        )

        self._ep_idx   = 0
        self._in_reset = False
        self._frame    = 0
        self._ep_done  = False
        self._is_connected = True

    def disconnect(self) -> None:
        self._is_connected = False
        logger.info("W250IKTeleop disconnected")

    def get_action(self) -> dict[str, float]:
        if not self._is_connected:
            raise RuntimeError("W250IKTeleop is not connected")

        rec_frames, rst_frames = self._episodes[self._ep_idx]
        frames = rst_frames if self._in_reset else rec_frames

        # Clamp so we never index out-of-range while waiting for the framework
        idx = min(self._frame, len(frames) - 1)
        pos = frames[idx]

        # Advance
        self._frame += 1
        if self._frame >= len(frames):
            if not self._in_reset:
                # Record sequence finished — signal episode done once.
                # Keep serving the last frame until get_teleop_events() fires.
                self._frame = len(frames)   # stay at end
                self._ep_done = True
            else:
                # Reset sequence finished — move to next episode.
                self._advance_episode()

        return dict(pos)

    def get_teleop_events(self) -> dict[str, Any]:
        """
        Called once per frame by lerobot-record.
        When recording waypoints are exhausted:
          - returns TERMINATE_EPISODE=True (record loop saves episode)
          - switches to reset waypoints so the reset loop gets release+REST
        """
        done = self._ep_done
        if done:
            self._ep_done  = False
            self._in_reset = True
            self._frame    = 0
            logger.info(
                f"W250IKTeleop: ep {self._ep_idx + 1} done — "
                "entering reset (release bowl, return to REST)"
            )
        return {
            TeleopEvents.TERMINATE_EPISODE: done,
            TeleopEvents.SUCCESS:           done,
            TeleopEvents.IS_INTERVENTION:   False,
            TeleopEvents.RERECORD_EPISODE:  False,
        }

    def send_feedback(self, feedback: dict) -> None:
        pass

    # ── Episode management ────────────────────────────────────────────────────

    def _advance_episode(self) -> None:
        """Called when reset frames end — move to the next episode's record phase."""
        next_idx = self._ep_idx + 1
        if next_idx >= len(self._episodes):
            logger.info("W250IKTeleop: all episodes completed.")
            # Stay at last REST frame; the framework stops via num_episodes.
            return
        self._ep_idx   = next_idx
        self._in_reset = False
        self._frame    = 0
        logger.info(
            f"W250IKTeleop: ready for episode {self._ep_idx + 1}/{len(self._episodes)}"
        )

    @property
    def current_episode_index(self) -> int:
        return self._ep_idx

    @property
    def total_episodes(self) -> int:
        return len(self._episodes)

    # ── IK pre-computation ────────────────────────────────────────────────────

    def _get_robot_description(self):
        """Retrieve the interbotix mr_descriptions class for the configured model."""
        model_key = self.config.robot_model.replace("-", "_")
        robot_des = getattr(mrd, model_key, None)
        if robot_des is None:
            raise ValueError(
                f"Robot model '{self.config.robot_model}' not found in "
                "interbotix_xs_modules.xs_robot.mr_descriptions. "
                f"Available models: {[k for k in dir(mrd) if not k.startswith('_')]}"
            )
        return robot_des

    def _ik_to_waypoint(
        self,
        robot_des,
        lower_limits: list[float],
        upper_limits: list[float],
        label: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        gripper: float,
        n_frames: int,
    ) -> tuple[dict[str, float], int]:
        """Solve IK and return a (positions_dict, n_frames) waypoint."""
        theta, ok = _solve_ik(robot_des, lower_limits, upper_limits, x, y, z, roll, pitch)
        if not ok:
            raise ValueError(
                f"IK failed for '{label}' at "
                f"(x={x:.3f}, y={y:.3f}, z={z:.3f}, roll={roll:.3f}, pitch={pitch:.3f}). "
                "Adjust the bowl grid or motion parameters in W250IKConfig."
            )
        pos_dict = _theta_list_to_normalized(theta)
        pos_dict["gripper.pos"] = float(gripper)
        return pos_dict, n_frames

    def _build_pick_place_episode(
        self,
        robot_des,
        lower_limits: list[float],
        upper_limits: list[float],
        x_bowl: float,
        y_bowl: float,
        roll: float,
        center_x: float,
        center_y: float,
    ) -> _Episode:
        """
        Build (record_wps, reset_wps) for one pick-and-place episode.

        Record waypoints (saved to dataset):
          1) REST + open gripper  — human places bowl
          2) Hover above bowl
          3) Descend to bowl
          4) Close gripper
          5) Lift bowl
          6) Arrive at centre holding bowl  ← TERMINATE_EPISODE fires here

        Reset waypoints (not recorded):
          7) Open gripper at centre         — bowl left at centre
          8) Return to REST                 — ready for next episode
        """
        cfg = self.config
        OPEN   = 1.0
        CLOSED = 0.0

        def wp(label, x, y, z, grip, n_frames):
            return self._ik_to_waypoint(
                robot_des, lower_limits, upper_limits,
                label, x, y, z, roll, cfg.grasp_pitch, grip, n_frames,
            )

        def centre_wp(grip, n_frames):
            return self._ik_to_waypoint(
                robot_des, lower_limits, upper_limits,
                "centre", center_x, center_y, cfg.center_z,
                0.0, cfg.grasp_pitch, grip, n_frames,
            )

        rest_dict = dict(W250_REST_POSITION)
        rest_dict["gripper.pos"] = OPEN

        z_hover = cfg.bowl_z + cfg.approach_height
        z_lift  = cfg.bowl_z + cfg.lift_height

        f = self._frames  # seconds → frame count helper

        record_wps = [
            # 1. REST — human places bowl during this pause
            (rest_dict,                                                  f(cfg.duration_rest_start)),
            # 2. Hover above bowl (gripper open)
            wp("hover",   x_bowl, y_bowl, z_hover,    OPEN,   f(cfg.duration_approach)),
            # 3. Descend to grasp height
            wp("descend", x_bowl, y_bowl, cfg.bowl_z,  OPEN,   f(cfg.duration_descend)),
            # 4. Close gripper — grip the bowl
            wp("grip",    x_bowl, y_bowl, cfg.bowl_z,  CLOSED, f(cfg.duration_close_gripper)),
            # 5. Lift bowl
            wp("lift",    x_bowl, y_bowl, z_lift,      CLOSED, f(cfg.duration_lift)),
            # 6. Carry bowl to centre at 10 cm above surface — episode ends here
            centre_wp(CLOSED, f(cfg.duration_transport)),
        ]

        reset_wps = [
            # 7. Open gripper — release bowl; it stays at centre
            centre_wp(OPEN, f(cfg.duration_release)),
            # 8. Return to REST for next episode
            (rest_dict, f(cfg.duration_rest_end)),
        ]

        return record_wps, reset_wps

    def _build_all_episodes(self) -> list[list[tuple[dict[str, float], int]]]:
        cfg = self.config
        robot_des = self._get_robot_description()
        lower_limits = [W250_JOINT_LIMITS[n][0] for n in W250_JOINT_NAMES]
        upper_limits  = [W250_JOINT_LIMITS[n][1] for n in W250_JOINT_NAMES]

        # ── Derive centre (x, y) from the REST waist angle ───────────────────
        # REST waist.pos is normalised; convert back to radians so the centre
        # lies exactly along the same direction the arm faces at REST.
        waist_lo, waist_hi = W250_JOINT_LIMITS["waist"]
        waist_rest_rad = waist_lo + (W250_REST_POSITION["waist.pos"] + 1.0) * (waist_hi - waist_lo) / 2.0
        center_x = cfg.center_reach * math.cos(waist_rest_rad)
        center_y = cfg.center_reach * math.sin(waist_rest_rad)
        logger.info(
            f"Centre position: x={center_x:.4f}  y={center_y:.4f}  z={cfg.center_z:.3f}  "
            f"(waist_rest={math.degrees(waist_rest_rad):.1f}°, reach={cfg.center_reach:.3f}m)"
        )

        # Bowl grid — row-major (depth rows outer, width columns inner)
        x_rows = [cfg.bowl_x_near, cfg.bowl_x_mid, cfg.bowl_x_far]
        y_cols = [cfg.bowl_y_left2, cfg.bowl_y_left1, cfg.bowl_y_right1, cfg.bowl_y_right2]

        episodes = []

        def make_episode(x, y, roll) -> _Episode:
            rec_wps, rst_wps = self._build_pick_place_episode(
                robot_des, lower_limits, upper_limits, x, y, roll, center_x, center_y
            )
            return _expand_to_frames(rec_wps), _expand_to_frames(rst_wps)

        # ── 12 base episodes (roll = grasp_roll, default 0) ──────────────────
        for x in x_rows:
            for y in y_cols:
                episodes.append(make_episode(x, y, cfg.grasp_roll))
                logger.debug(f"  base episode {len(episodes)}: bowl=({x:.2f}, {y:.2f})")

        # ── Rotation-variation episodes ───────────────────────────────────────
        if cfg.add_rotation_episodes:
            for base_idx in [
                cfg.rotation_ep_0, cfg.rotation_ep_1, cfg.rotation_ep_2,
                cfg.rotation_ep_3, cfg.rotation_ep_4,
            ]:
                if not (0 <= base_idx < 12):
                    logger.warning(f"rotation_ep index {base_idx} out of range — skipping")
                    continue
                x = x_rows[base_idx // 4]
                y = y_cols[base_idx % 4]
                for roll_val, label in [(cfg.rotation_roll_a, "+45°"), (cfg.rotation_roll_b, "−45°")]:
                    episodes.append(make_episode(x, y, roll_val))
                    logger.debug(f"  rotation ep {len(episodes)}: bowl=({x:.2f},{y:.2f}) roll={label}")

        rec_total = sum(len(rec) for rec, _   in episodes)
        rst_total = sum(len(rst) for _,   rst in episodes)
        logger.info(
            f"Built {len(episodes)} episodes — "
            f"record: {rec_total} frames ({rec_total / cfg.fps:.1f}s), "
            f"reset:  {rst_total} frames ({rst_total / cfg.fps:.1f}s)"
        )
        return episodes
