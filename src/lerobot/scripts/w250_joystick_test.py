#!/usr/bin/env python3

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
W250 Joystick Teleoperation Test Script
=========================================

Tests the W250Interbotix robot middleware by teleoprating it with a
Logitech F710 gamepad via ROS2 joy node.

Requirements:
  - Joystick attached to WSL (see run_w250_joystick_test.sh for usbipd steps)
  - ros2 run joy joy_node  (launched automatically by the .sh script)

Usage (run via shell script — handles usbipd + joy_node automatically):
    $ bash ~/run_w250_joystick_test.sh

Controls:
    Left Stick X    → Waist       (left / right)
    Left Stick Y    → Shoulder    (up / down)
    Right Stick Y   → Elbow       (up / down)
    Right Stick X   → Wrist angle
    D-Pad L/R       → Forearm roll
    LT / RT         → Wrist rotate
    LB              → Gripper open
    RB              → Gripper close
    Y               → Reset to REST position
    Back            → Toggle intervention flag
    A               → Save current pose to /tmp/saved_pose.json
    B               → Replay saved pose + print round-trip error
    Start           → Rerecord episode signal
    Ctrl+C          → Quit
"""

import csv
import json
import logging
import signal
import sys
import time
from pathlib import Path

# ── Setup logging ─────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("w250_joystick_test")

# ── Imports ───────────────────────────────────────────────────────────────────
from lerobot.robots.w250.config_w250_interbotix import W250InterbotixConfig
from lerobot.robots.w250.w250_interbotix import W250Interbotix
from lerobot.teleoperators.w250joystick import W250JoystickConfig, W250JoystickTeleop


# ── Configuration ─────────────────────────────────────────────────────────────

ROBOT_MODEL = "wx250s"
ROBOT_NAME  = "wx250s"

# Frequency — single value, all timing derived from it.
# Must match: dataset fps, camera fps, W250InterbotixConfig.fps.
FPS        = 15
DISPLAY_HZ = 4.0

CONTROL_HZ  = float(FPS)
MOVING_TIME = 0.15            # safe for teleop; use 1/FPS for policy replay
ACCEL_TIME  = MOVING_TIME / 4

CALIBRATE_ON_CONNECT = True

GRIPPER_LOG_PATH = Path("/tmp/gripper_log_joystick.csv")
POSE_LOG_PATH    = Path("/tmp/saved_pose.json")


# ── Helpers ───────────────────────────────────────────────────────────────────

def format_state(obs: dict) -> str:
    keys = [
        "waist.pos", "shoulder.pos", "elbow.pos",
        "forearm_roll.pos", "wrist_angle.pos", "wrist_rotate.pos", "gripper.pos",
    ]
    parts = []
    for k in keys:
        if k in obs:
            short = k.replace(".pos", "")[:6]
            parts.append(f"{short}:{obs[k]:+.3f}")
    return "  ".join(parts)


def format_action_diff(action: dict, obs: dict) -> str:
    diffs = []
    for k in action:
        if k in obs and isinstance(obs[k], float):
            diff = action[k] - obs[k]
            if abs(diff) > 0.01:
                diffs.append(f"{k.replace('.pos','')}:{diff:+.3f}")
    return ", ".join(diffs) if diffs else "—"


def save_pose(obs: dict) -> None:
    data = {k: v for k, v in obs.items() if isinstance(v, float)}
    POSE_LOG_PATH.write_text(json.dumps(data, indent=2))
    print(f"\n[POSE] Saved → {POSE_LOG_PATH}: {format_state(data)}")
    sys.stdout.flush()


def replay_pose(robot, path: Path) -> None:
    if not path.exists():
        print("\n[POSE] No saved pose found — press A first.")
        sys.stdout.flush()
        return

    saved = json.loads(path.read_text())
    arm_keys = [k for k in saved if k != "gripper.pos"]

    print(f"\n[POSE] Replaying: {format_state(saved)}")
    sys.stdout.flush()

    action = {k: saved[k] for k in arm_keys}
    robot.send_action(action)

    time.sleep(robot.config.moving_time + 0.2)

    obs_after = robot.get_observation()
    print("[POSE] Round-trip error (saved → action → observed):")
    max_err = 0.0
    for k in arm_keys:
        if k in obs_after:
            err = abs(saved[k] - obs_after[k])
            max_err = max(max_err, err)
            flag = "  ✓" if err < 0.02 else "  !" if err < 0.05 else "  ✗ LARGE"
            print(f"  {k:<22} saved={saved[k]:+.4f}  obs={obs_after[k]:+.4f}  err={err:.4f}{flag}")
    print(f"  max_err={max_err:.4f}  ({'OK' if max_err < 0.05 else 'CHECK LIMITS'})")
    sys.stdout.flush()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    robot_cfg = W250InterbotixConfig(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        fps=FPS,
        cameras={},
    )

    teleop_cfg = W250JoystickConfig(
        control_hz=float(FPS),
        step_size=0.03,
        gripper_step_size=0.15,
    )

    robot  = W250Interbotix(robot_cfg)
    teleop = W250JoystickTeleop(teleop_cfg)

    def _sigint_handler(sig, frame):
        logger.info("SIGINT — shutting down...")
        if teleop.is_connected:
            teleop.disconnect()
        if robot.is_connected:
            robot.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint_handler)

    # ── Connect ───────────────────────────────────────────────────────────────
    logger.info(f"Connecting to robot: {ROBOT_MODEL}")
    try:
        robot.connect(calibrate=CALIBRATE_ON_CONNECT)
    except Exception as e:
        logger.error(f"Failed to connect robot: {e}")
        sys.exit(1)

    logger.info("Robot connected. Reading initial state...")
    try:
        initial_obs = robot.get_observation()
    except Exception as e:
        logger.error(f"Failed to read initial observation: {e}")
        robot.disconnect()
        sys.exit(1)

    logger.info(f"Initial state: {format_state(initial_obs)}")

    teleop.connect()
    teleop.sync_with_robot(initial_obs)
    logger.info("Joystick connected and synced with robot state.")
    logger.info("Waiting for first joy message on /joy topic...")

    # ── Gripper CSV log ────────────────────────────────────────────────────────
    gripper_log_f = open(GRIPPER_LOG_PATH, "w", newline="")
    gripper_csv   = csv.writer(gripper_log_f)
    gripper_csv.writerow([
        "t_s", "step", "event",
        "kb_gripper", "kb_delta",
        "effort_sent",
        "finger_m", "finger_norm", "lower_m", "upper_m",
        "gripper_moving", "note",
    ])
    logger.info(f"Gripper log → {GRIPPER_LOG_PATH}")

    def _read_gripper_hw():
        try:
            g = robot.bot.gripper
            effort    = float(g.gripper_command.cmd)
            finger_m  = float(g.get_finger_position())
            lower     = float(g.left_finger_lower_limit)
            upper     = float(g.left_finger_upper_limit)
            moving    = bool(g.gripper_moving)
            return effort, finger_m, lower, upper, moving
        except Exception:
            return 0.0, 0.0, 0.015, 0.037, False

    # ── Control loop ──────────────────────────────────────────────────────────
    loop_dt      = 1.0 / CONTROL_HZ
    display_dt   = 1.0 / DISPLAY_HZ
    last_disp_t  = 0.0

    step                  = 0
    total_action_time_ms  = 0.0
    total_obs_time_ms     = 0.0
    t_start               = time.perf_counter()
    prev_kb_gripper       = initial_obs.get("gripper.pos", 1.0)

    logger.info(f"Control loop at {CONTROL_HZ:.0f} Hz. Press Ctrl+C to quit.\n")

    while teleop.is_connected:
        loop_start = time.perf_counter()

        # ── Teleop events ──────────────────────────────────────────────────
        events = teleop.get_teleop_events()
        if events.get("terminate_episode", False):
            logger.info("Terminate signal from gamepad.")
            break

        # ── Pose save / replay (A / B) ─────────────────────────────────────
        if teleop.consume_save_pose():
            try:
                current_obs = robot.get_observation()
                save_pose(current_obs)
                teleop.sync_with_robot(current_obs)
            except Exception as e:
                print(f"\n[POSE] Save failed: {e}")

        if teleop.consume_replay_pose():
            try:
                replay_pose(robot, POSE_LOG_PATH)
                current_obs = robot.get_observation()
                teleop.sync_with_robot(current_obs)
            except Exception as e:
                print(f"\n[POSE] Replay failed: {e}")

        # ── Get action ─────────────────────────────────────────────────────
        action          = teleop.get_action()
        kb_gripper      = action.get("gripper.pos", 1.0)
        kb_delta        = kb_gripper - prev_kb_gripper
        prev_kb_gripper = kb_gripper

        # ── Send action ────────────────────────────────────────────────────
        t0 = time.perf_counter()
        try:
            sent_action = robot.send_action(action)
        except Exception as e:
            logger.error(f"send_action failed: {e}")
            break
        action_ms = (time.perf_counter() - t0) * 1e3
        total_action_time_ms += action_ms

        # ── Gripper hardware state ─────────────────────────────────────────
        effort, finger_m, lower_m, upper_m, gripper_moving = _read_gripper_hw()
        finger_norm = (finger_m - lower_m) / (upper_m - lower_m) if upper_m > lower_m else 0.0
        t_s         = time.perf_counter() - t_start

        if kb_delta > 0.001:
            event = "LB_PRESSED"
        elif kb_delta < -0.001:
            event = "RB_PRESSED"
        else:
            event = "STEP"

        if effort > 1.0:
            note = "open"
        elif effort < -1.0:
            note = "close"
        elif finger_m >= upper_m - 0.0001:
            note = "at_upper"
        elif finger_m <= lower_m + 0.0001:
            note = "at_lower"
        else:
            note = "stop"

        if event != "STEP" or step % 5 == 0:
            gripper_csv.writerow([
                f"{t_s:.3f}", step, event,
                f"{kb_gripper:.4f}", f"{kb_delta:+.4f}",
                f"{effort:.1f}",
                f"{finger_m:.5f}", f"{finger_norm:.4f}",
                f"{lower_m:.5f}", f"{upper_m:.5f}",
                gripper_moving, note,
            ])
            gripper_log_f.flush()

        # ── Get observation ────────────────────────────────────────────────
        t0 = time.perf_counter()
        try:
            obs = robot.get_observation()
        except Exception as e:
            logger.error(f"get_observation failed: {e}")
            break
        obs_ms = (time.perf_counter() - t0) * 1e3
        total_obs_time_ms += obs_ms

        step += 1

        # ── Display ────────────────────────────────────────────────────────
        now = time.perf_counter()
        if now - last_disp_t >= display_dt:
            last_disp_t = now
            avg_act = total_action_time_ms / step
            avg_obs = total_obs_time_ms / step
            diff_str = format_action_diff(sent_action, obs)
            print(
                f"\r[step {step:5d}] {format_state(obs)}"
                f"  |  diff=[{diff_str}]"
                f"  |  act:{avg_act:.1f}ms obs:{avg_obs:.1f}ms"
                "    ",
                end="",
                flush=True,
            )

        # ── Loop rate ──────────────────────────────────────────────────────
        elapsed = time.perf_counter() - loop_start
        sleep_t = loop_dt - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    print()
    gripper_log_f.close()
    logger.info(f"Gripper log saved to {GRIPPER_LOG_PATH}")

    if teleop.is_connected:
        teleop.disconnect()

    logger.info(f"Control loop ended after {step} steps.")
    if step > 0:
        logger.info(
            f"Avg latencies — send_action: {total_action_time_ms/step:.1f}ms  "
            f"get_observation: {total_obs_time_ms/step:.1f}ms"
        )

    logger.info("Moving robot to sleep pose...")
    try:
        robot.bot.arm.go_to_sleep_pose(blocking=True)
    except Exception as e:
        logger.warning(f"Could not move to sleep pose: {e}")

    robot.disconnect()
    logger.info("Done.")


if __name__ == "__main__":
    main()
