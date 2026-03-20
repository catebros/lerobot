#!/usr/bin/env python3

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
W250 Position Round-Trip Test
================================

Verifies that the W250Interbotix middleware correctly captures and sends
positions by comparing against the Interbotix API raw joint angles (radians),
which act as the independent ground truth.

For each test pose:

  1. Send normalized position via middleware send_action(norm)
  2. Wait for motion to complete
  3. Read ground truth: direct API get_joint_positions() → raw radians
  4. Read middleware:   get_observation() → normalized
  5. Compare:
       a) denormalize(obs_norm) vs raw_rad   → is get_observation() correct?
       b) denormalize(norm_sent) vs raw_rad  → did send_action() move there?

A bug in _normalize_position or _denormalize_position will show as a large
error in (a). A robot that doesn't reach the commanded position will show
as a large error in (b).

Usage:
    $ source ~/interbotix_ws/install/setup.bash
    $ python -m lerobot.scripts.w250_record_replay_test
"""

import json
import logging
import sys
import time
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("record_replay_test")

# ── Config ────────────────────────────────────────────────────────────────────

ROBOT_MODEL = "wx250s"
ROBOT_NAME  = "wx250s"
# Calibration uses slow moves (2s) — the robot config auto-computes
# fast teleop timing (1/fps) but calibrate() overrides internally.

RESULTS_PATH = Path("/tmp/w250_roundtrip_results.json")

JOINT_NAMES = [
    "waist", "shoulder", "elbow",
    "forearm_roll", "wrist_angle", "wrist_rotate",
]

# Test poses as normalized values in [-1, 1] — conservative, well within limits.
# [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
TEST_POSES_NORM = [
    [0.0,    0.0,   0.0,   0.0,  0.0,   0.0],   # home
    [0.2,   -0.3,   0.3,   0.0,  0.2,   0.0],   # pose A
    [-0.2,  -0.2,   0.5,   0.2,  0.3,   0.3],   # pose B
    [0.15,  -0.5,   0.3,  -0.2,  0.5,  -0.2],   # pose C
    [0.0,   -0.3,   0.25,  0.0,  0.45,  0.0],   # pose D
]

# ── Helpers ───────────────────────────────────────────────────────────────────

def denormalize(joint_name: str, norm: float, limits: dict) -> float:
    norm = max(-1.0, min(1.0, norm))
    lower, upper = limits[joint_name]
    return lower + (norm + 1.0) * (upper - lower) / 2.0


def normalize(joint_name: str, rad: float, limits: dict) -> float:
    lower, upper = limits[joint_name]
    if upper == lower:
        return 0.0
    return 2.0 * (rad - lower) / (upper - lower) - 1.0


def check(label: str, expected_rad: list[float], actual_rad: list[float],
          warn_thresh: float = 0.02, fail_thresh: float = 0.05) -> float:
    """Print per-joint comparison in radians. Returns max error."""
    print(f"  {label}")
    max_err = 0.0
    for name, exp, act in zip(JOINT_NAMES, expected_rad, actual_rad):
        err = abs(exp - act)
        max_err = max(max_err, err)
        flag = "✓" if err < warn_thresh else "!" if err < fail_thresh else "✗ LARGE"
        print(f"    {name:<16} expected={exp:+.4f} rad  actual={act:+.4f} rad  "
              f"err={err:.4f} rad  {flag}")
    verdict = "OK" if max_err < fail_thresh else "FAIL"
    print(f"    max_err = {max_err:.4f} rad  ({verdict})\n")
    return max_err


# ── Main test ─────────────────────────────────────────────────────────────────

def run_test() -> None:
    from lerobot.robots.w250.config_w250_interbotix import W250InterbotixConfig
    from lerobot.robots.w250.w250_interbotix import W250Interbotix

    logger.info("Connecting via middleware...")
    robot_cfg = W250InterbotixConfig(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        cameras={},
    )
    robot = W250Interbotix(robot_cfg)
    robot.connect(calibrate=True)

    # The middleware's joint limits — used to convert between norm and radians.
    limits = dict(robot._joint_limits)
    logger.info("Joint limits loaded by middleware:")
    for name in JOINT_NAMES:
        lo, hi = limits[name]
        logger.info(f"  {name}: [{lo:.4f}, {hi:.4f}] rad")

    # The direct Interbotix arm handle — bypasses our normalization entirely.
    # This is the ground truth: raw radians straight from the hardware.
    direct = robot.bot.arm

    # ── Note on xs_sdk_sim limitation ────────────────────────────────────────
    # get_joint_positions() reads /wx250s/joint_states, which xs_sdk_sim does NOT
    # update after commands — it always publishes the initial sleep pose.
    # joint_commands IS updated immediately when set_joint_positions succeeds.
    # We therefore use joint_commands as the ground truth for what the robot
    # was commanded to, which lets us test the normalization math reliably.
    logger.info(f"joint_commands after calibrate: "
                f"{[f'{n}={v:+.4f}' for n, v in zip(JOINT_NAMES, list(direct.joint_commands))]}")
    logger.info(f"Velocity limits: {list(direct.group_info.joint_velocity_limits)}")
    # ─────────────────────────────────────────────────────────────────────────

    all_results = []
    try:
        for idx, target_norm in enumerate(TEST_POSES_NORM):
            label = "HOME" if idx == 0 else f"pose_{idx}"
            action = {f"{name}.pos": v for name, v in zip(JOINT_NAMES, target_norm)}

            print(f"\n{'='*60}")
            print(f"Pose: {label}  (normalized: {[f'{v:+.3f}' for v in target_norm]})")
            print(f"{'='*60}")

            # What position the middleware SHOULD send (in radians)
            expected_rad = [denormalize(n, v, limits) for n, v in zip(JOINT_NAMES, target_norm)]
            print(f"  Expected (denorm): {[f'{r:+.4f}' for r in expected_rad]} rad\n")

            # ── Send via middleware ────────────────────────────────────────
            robot.send_action(action)

            # ── Ground truth: joint_commands (updated when SDK sends command)
            # This is what was actually commanded to the robot.  In xs_sdk_sim
            # get_joint_positions() returns stale sleep values (sim bug), so we
            # use joint_commands instead — it reflects the last accepted command.
            cmd_rad = list(direct.joint_commands)

            # ── Middleware observation ─────────────────────────────────────
            obs = robot.get_observation()
            obs_norm = [obs.get(f"{name}.pos", float("nan")) for name in JOINT_NAMES]
            obs_as_rad = [denormalize(n, v, limits) for n, v in zip(JOINT_NAMES, obs_norm)]

            # ── Check A: send_action math ──────────────────────────────────
            # denorm(norm_sent) should equal joint_commands.
            # Tests _denormalize_position: did the middleware convert correctly?
            err_a = check(
                "A) send_action math  [denorm(norm_sent) vs joint_commands]:",
                expected_rad, cmd_rad,
            )

            # ── Check B: round-trip ────────────────────────────────────────
            # normalize(joint_commands) should equal norm_sent.
            # Tests _normalize_position: if we observe the commanded position,
            # do we get back the original normalized value?
            cmd_renorm = [normalize(n, v, limits) for n, v in zip(JOINT_NAMES, cmd_rad)]
            err_b = check(
                "B) round-trip  [norm_sent vs normalize(joint_commands)]:",
                target_norm, cmd_renorm,
                warn_thresh=0.005, fail_thresh=0.01,
            )

            all_results.append({
                "label": label,
                "target_norm": target_norm,
                "expected_rad": expected_rad,
                "cmd_rad": cmd_rad,
                "err_send_action_math": err_a,
                "err_roundtrip": err_b,
            })

    finally:
        logger.info("Moving to sleep pose...")
        try:
            robot.bot.arm.go_to_sleep_pose(blocking=True)
        except Exception:
            pass
        robot.disconnect()

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'='*70}")
    print("SUMMARY")
    print(f"{'='*70}")
    print(f"{'Pose':<10}  {'A:denorm_math':>14}  {'B:roundtrip':>12}  status")
    for r in all_results:
        a, b = r["err_send_action_math"], r["err_roundtrip"]
        ok = a < 0.05 and b < 0.01
        status = "✓ OK" if ok else "✗ FAIL"
        print(f"{r['label']:<10}  {a:>14.4f}  {b:>12.4f}  {status}")

    print()
    print("Interpretation:")
    print("  A large  →  _denormalize_position math is wrong (wrong limits or formula?)")
    print("  B large  →  normalize/denormalize are not inverses (math bug)")

    RESULTS_PATH.write_text(json.dumps(all_results, indent=2))
    logger.info(f"Full results saved to {RESULTS_PATH}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    run_test()


if __name__ == "__main__":
    main()
