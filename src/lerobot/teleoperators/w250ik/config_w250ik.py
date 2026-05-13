"""
Configuration for the W250 IK-based autonomous teleoperator.

The robot picks a bowl from each cell of a 4-column × 3-row Cartesian grid
and carries it to a fixed centre position 10 cm above the surface.

Grid coordinates are in the robot base frame (metres):
  X axis  — depth  (away from robot, 3 rows: near / mid / far)
  Y axis  — width  (left/right,       4 cols: left2 / left1 / right1 / right2)
  Z axis  — height (up)

All positions and orientations must be tuned to the physical workspace before
the first recording session.

fps MUST match robot.fps (W250InterbotixConfig) and --dataset.fps passed to
lerobot-record.  Phase durations are in seconds; frame counts are derived as
round(duration_s * fps) at connect() time so timing stays correct regardless
of the chosen frequency.
"""

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("w250ik")
@dataclass
class W250IKConfig(TeleoperatorConfig):
    # ── System frequency ─────────────────────────────────────────────────────
    # Must match robot.fps and --dataset.fps.
    fps: float = 15.0

    # ── IK model ─────────────────────────────────────────────────────────────
    robot_model: str = "wx250s"

    # ── Bowl grid (robot base frame, metres) ─────────────────────────────────
    # 3 depth rows (X axis)
    bowl_x_near: float = 0.15
    bowl_x_mid:  float = 0.22
    bowl_x_far:  float = 0.30
    # 4 width columns (Y axis)
    bowl_y_left2:  float = -0.15
    bowl_y_left1:  float = -0.05
    bowl_y_right1: float =  0.05
    bowl_y_right2: float =  0.15
    # Z: grasp contact height above robot base plane
    bowl_z: float = 0.02

    # ── Centre position ───────────────────────────────────────────────────────
    # Episode ends here: arm holds the bowl 10 cm above the surface.
    # The centre lies along the REST waist direction (waist.pos=0.5 → 90°),
    # so (x, y) = (reach·cos(90°), reach·sin(90°)) = (0, reach).
    # center_reach is the horizontal distance from the robot base; center_z is
    # the height above the base plane.  The (x, y) are derived automatically
    # at connect() time so this stays correct if the REST position ever changes.
    center_reach: float = 0.20   # horizontal distance from base along REST waist direction
    center_z:     float = 0.10   # 10 cm above surface

    # ── Motion parameters ─────────────────────────────────────────────────────
    approach_height: float = 0.08   # m above bowl_z for the hover position
    lift_height:     float = 0.10   # m above bowl_z after grasping

    # Gripper orientation when picking (pointing straight down)
    grasp_pitch: float = 1.5708     # π/2 rad
    grasp_roll:  float = 0.0

    # ── Rotation-variation episodes ───────────────────────────────────────────
    add_rotation_episodes: bool = True
    rotation_roll_a: float =  0.785   # +45° (π/4)
    rotation_roll_b: float = -0.785   # −45°
    # Indices (0-based) into the 12 base episodes that get rotation variants
    rotation_ep_0: int = 0
    rotation_ep_1: int = 3
    rotation_ep_2: int = 6
    rotation_ep_3: int = 9
    rotation_ep_4: int = 11

    # ── Phase durations (seconds) ─────────────────────────────────────────────
    # Frame counts = round(duration_s * fps), computed at connect() time.
    # This keeps timing correct at any system frequency.

    # REST at start: human places bowl during this pause (recorded)
    duration_rest_start:    float = 3.0
    # Arm moves from REST to hover above bowl (recorded)
    duration_approach:      float = 1.7
    # Arm descends to grasp height (recorded)
    duration_descend:       float = 1.3
    # Gripper closes and grips bowl (recorded)
    duration_close_gripper: float = 1.7
    # Arm lifts bowl (recorded)
    duration_lift:          float = 1.3
    # Arm carries bowl to centre — episode ends here (recorded)
    duration_transport:     float = 2.0

    # ── Reset-phase durations (not recorded) ──────────────────────────────────
    # Gripper opens at centre; bowl is left there
    duration_release:  float = 1.3
    # Arm returns to REST (ready for next episode)
    duration_rest_end: float = 2.0
