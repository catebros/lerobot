"""
Configuration for the W250 IK-based autonomous teleoperator.

Episode positions are loaded from a JSON file produced by record_bowl_positions.py.
The JSON contains:
  center   — drop-off position {x, y, z, roll}
  episodes — list of grasp positions [{label, x, y, z, roll}, ...]

fps MUST match robot.fps and --dataset.fps.
Phase durations are in seconds; frame counts are derived at connect() time.
"""

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("w250ik")
@dataclass
class W250IKConfig(TeleoperatorConfig):
    # ── System frequency ─────────────────────────────────────────────────────
    fps: float = 15.0

    # ── IK model ─────────────────────────────────────────────────────────────
    robot_model: str = "wx250s"

    # ── Positions file ───────────────────────────────────────────────────────
    # JSON produced by record_bowl_positions.py.
    # Use /mnt/c/... paths when running in WSL.
    positions_file: str = "/home/cbarbero/bowl_positions.json"

    # ── Centre (drop-off) position ────────────────────────────────────────────
    # Where the robot releases the object. Hardcoded here so you don't need to
    # re-record it every session. The JSON file may optionally override these
    # with a "center" key; if absent, these values are used.
    #
    # The rest waist.pos=0.5 (normalized) = 90° → the rest direction is the +Y axis.
    # At reach=0.20 m:  x = reach·cos(90°) = 0,  y = reach·sin(90°) = 0.20
    center_x: float = 0.0
    center_y: float = 0.20
    center_z: float = 0.10   # 10 cm above the surface

    # ── Motion parameters ─────────────────────────────────────────────────────
    approach_height: float = 0.08   # m above grasp z for hover
    lift_height:     float = 0.10   # m above grasp z after gripping

    # ── Phase durations (seconds) ─────────────────────────────────────────────
    duration_rest_start:    float = 1.0
    duration_approach:      float = 1.7
    duration_descend:       float = 1.3
    duration_close_gripper: float = 7.0
    duration_lift:          float = 1.3
    duration_transport:     float = 2.0
    duration_centre_hold:   float = 1.0   # hold at centre before episode ends

    # ── Reset-phase durations (not recorded) ──────────────────────────────────
    duration_release:  float = 8.0
    # Robot hovers over the NEXT bowl position after releasing so you can place
    # the bowl directly under the gripper. Set to 0.0 to disable.
    # Last episode always skips this (no next position).
    duration_pointing: float = 8.0
    duration_rest_end: float = 12.0
