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
    positions_file: str = "/mnt/c/Users/cater/Code/roblab/bowl_positions.json"

    # ── Motion parameters ─────────────────────────────────────────────────────
    approach_height: float = 0.08   # m above grasp z for hover
    lift_height:     float = 0.10   # m above grasp z after gripping

    # ── Phase durations (seconds) ─────────────────────────────────────────────
    duration_rest_start:    float = 3.0
    duration_approach:      float = 1.7
    duration_descend:       float = 1.3
    duration_close_gripper: float = 1.7
    duration_lift:          float = 1.3
    duration_transport:     float = 2.0

    # ── Reset-phase durations (not recorded) ──────────────────────────────────
    duration_release:  float = 1.3
    duration_rest_end: float = 2.0
