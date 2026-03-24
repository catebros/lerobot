"""
Test script: verify that positions read from the ROS topic match the actions sent.

Workflow:
  1. RECORD  — execute a hardcoded trajectory, store (timestamp, action, observed_position)
               using the same robot.send_action / robot.get_observation calls that lerobot-record uses.
  2. REPLAY ACTIONS  — move the robot by sending the recorded *actions* (open-loop command replay).
  3. REPLAY POSITIONS — move the robot by sending the recorded *observed positions* as commands.

If the robot follows both replays identically the position reading is correct.
If REPLAY POSITIONS drifts or behaves differently there is a bug in how positions are captured.

Usage:
    python3 src/lerobot/scripts/test_positions_vs_actions.py
"""

import json
import logging
import time
from pathlib import Path

import numpy as np

from lerobot.robots.w250.config_w250_interbotix import W250InterbotixConfig
from lerobot.robots.w250.w250_interbotix import W250Interbotix

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Config — edit these to match your setup
# ---------------------------------------------------------------------------
ROBOT_MODEL = "wx250s"
ROBOT_NAME = "wx250s"
FPS = 15
STEP_DURATION_S = 1.0 / FPS   # how long to wait between steps during record

# Output file — saved next to this script
SAVE_PATH = Path(__file__).parent / "test_positions_vs_actions.json"

# ---------------------------------------------------------------------------
# Hardcoded trajectory (normalized positions, same format as lerobot actions)
# Each waypoint is held for WAYPOINT_HOLD_S seconds during recording.
# ---------------------------------------------------------------------------
WAYPOINT_HOLD_S = 2.0   # seconds per waypoint

# Joints: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper
# Values in normalized [-1, 1] range (gripper: 0=closed, 1=open)
TRAJECTORY = [
    # name, waypoint dict
    ("rest",     {"waist.pos":  0.5, "shoulder.pos": -0.6, "elbow.pos":  0.4,
                  "forearm_roll.pos": 0.0, "wrist_angle.pos":  0.73,
                  "wrist_rotate.pos": 0.0, "gripper.pos": 1.0}),

    ("reach",    {"waist.pos":  0.0, "shoulder.pos":  0.2, "elbow.pos":  0.1,
                  "forearm_roll.pos": 0.0, "wrist_angle.pos": -0.2,
                  "wrist_rotate.pos": 0.0, "gripper.pos": 1.0}),

    ("close_gripper", {"waist.pos":  0.0, "shoulder.pos":  0.2, "elbow.pos":  0.1,
                       "forearm_roll.pos": 0.0, "wrist_angle.pos": -0.2,
                       "wrist_rotate.pos": 0.0, "gripper.pos": 0.0}),

    ("lift",     {"waist.pos":  0.0, "shoulder.pos": -0.1, "elbow.pos":  0.3,
                  "forearm_roll.pos": 0.0, "wrist_angle.pos":  0.2,
                  "wrist_rotate.pos": 0.0, "gripper.pos": 0.0}),

    ("back_rest", {"waist.pos":  0.5, "shoulder.pos": -0.6, "elbow.pos":  0.4,
                   "forearm_roll.pos": 0.0, "wrist_angle.pos":  0.73,
                   "wrist_rotate.pos": 0.0, "gripper.pos": 1.0}),
]

JOINT_KEYS = [
    "waist.pos", "shoulder.pos", "elbow.pos",
    "forearm_roll.pos", "wrist_angle.pos", "wrist_rotate.pos", "gripper.pos",
]


def make_robot() -> W250Interbotix:
    cfg = W250InterbotixConfig(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        fps=FPS,
        cameras={},   # no cameras needed for this test
    )
    return W250Interbotix(cfg)


# ---------------------------------------------------------------------------
# Phase 1: Record
# ---------------------------------------------------------------------------
def phase_record(robot: W250Interbotix) -> list[dict]:
    """Execute the hardcoded trajectory and store (action, observed_position) per step."""
    logger.info("=== PHASE 1: RECORD ===")
    records = []

    steps_per_waypoint = max(1, int(WAYPOINT_HOLD_S * FPS))

    for wp_name, waypoint in TRAJECTORY:
        logger.info(f"  Waypoint: {wp_name}")
        for step in range(steps_per_waypoint):
            t_start = time.perf_counter()

            # Same call as lerobot-record: send action first, then read observation
            robot.send_action(waypoint)
            obs = robot.get_observation()

            # Extract only joint positions from observation (drop camera images)
            obs_positions = {k: float(obs[k]) for k in JOINT_KEYS if k in obs}
            action_dict   = {k: float(waypoint[k]) for k in JOINT_KEYS}

            records.append({
                "waypoint": wp_name,
                "step": step,
                "action": action_dict,
                "observed_position": obs_positions,
            })

            # Rate limiting
            elapsed = time.perf_counter() - t_start
            sleep_t = STEP_DURATION_S - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    logger.info(f"  Recorded {len(records)} steps total")
    return records


# ---------------------------------------------------------------------------
# Phase 2 & 3: Replay
# ---------------------------------------------------------------------------
def phase_replay(robot: W250Interbotix, records: list[dict], use_positions: bool) -> list[dict]:
    mode = "POSITIONS" if use_positions else "ACTIONS"
    logger.info(f"=== PHASE REPLAY {mode} ===")
    replay_log = []

    for rec in records:
        t_start = time.perf_counter()

        cmd = rec["observed_position"] if use_positions else rec["action"]
        robot.send_action(cmd)
        obs = robot.get_observation()
        obs_positions = {k: float(obs[k]) for k in JOINT_KEYS if k in obs}

        # Error between what we commanded and what the robot reports
        errors = {k: abs(cmd[k] - obs_positions.get(k, float("nan"))) for k in JOINT_KEYS}

        replay_log.append({
            "waypoint": rec["waypoint"],
            "step": rec["step"],
            "commanded": cmd,
            "observed": obs_positions,
            "abs_error": errors,
        })

        elapsed = time.perf_counter() - t_start
        sleep_t = STEP_DURATION_S - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    return replay_log


# ---------------------------------------------------------------------------
# Analysis helpers
# ---------------------------------------------------------------------------
def print_summary(label: str, replay_log: list[dict]):
    all_errors = {k: [] for k in JOINT_KEYS}
    for entry in replay_log:
        for k, v in entry["abs_error"].items():
            if not np.isnan(v):
                all_errors[k].append(v)

    logger.info(f"\n--- {label} error summary (normalized units) ---")
    for k in JOINT_KEYS:
        errs = all_errors[k]
        if errs:
            logger.info(f"  {k:25s}  mean={np.mean(errs):.4f}  max={np.max(errs):.4f}")


def compare_replays(action_log: list[dict], position_log: list[dict]):
    """Compare the final observed positions between the two replay modes."""
    logger.info("\n--- Comparison: action_replay vs position_replay observed positions ---")
    diffs = {k: [] for k in JOINT_KEYS}
    for a, p in zip(action_log, position_log):
        for k in JOINT_KEYS:
            diff = abs(a["observed"].get(k, 0.0) - p["observed"].get(k, 0.0))
            diffs[k].append(diff)
    for k in JOINT_KEYS:
        logger.info(f"  {k:25s}  mean_diff={np.mean(diffs[k]):.4f}  max_diff={np.max(diffs[k]):.4f}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    robot = make_robot()
    robot.connect()

    try:
        # ---- Phase 1: record ----
        records = phase_record(robot)

        # Save raw data
        SAVE_PATH.write_text(json.dumps(records, indent=2))
        logger.info(f"Saved recording to {SAVE_PATH}")

        # Quick sanity print: action vs observed for first & last step
        logger.info("\n--- Action vs Observed (first step) ---")
        first = records[0]
        for k in JOINT_KEYS:
            a = first["action"][k]
            o = first["observed_position"].get(k, float("nan"))
            logger.info(f"  {k:25s}  action={a:+.4f}  observed={o:+.4f}  diff={abs(a-o):.4f}")

        logger.info("\n--- Action vs Observed (last step) ---")
        last = records[-1]
        for k in JOINT_KEYS:
            a = last["action"][k]
            o = last["observed_position"].get(k, float("nan"))
            logger.info(f"  {k:25s}  action={a:+.4f}  observed={o:+.4f}  diff={abs(a-o):.4f}")

        input("\nPress ENTER to start REPLAY ACTIONS (open-loop command replay)...")

        # ---- Phase 2: replay actions ----
        action_replay_log = phase_replay(robot, records, use_positions=False)
        print_summary("REPLAY ACTIONS", action_replay_log)

        input("\nPress ENTER to start REPLAY POSITIONS (using recorded observed positions as commands)...")

        # ---- Phase 3: replay positions ----
        position_replay_log = phase_replay(robot, records, use_positions=True)
        print_summary("REPLAY POSITIONS", position_replay_log)

        # ---- Compare both replays ----
        compare_replays(action_replay_log, position_replay_log)

        logger.info("\nDone. If REPLAY POSITIONS has similar errors to REPLAY ACTIONS, positions are captured correctly.")

    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
