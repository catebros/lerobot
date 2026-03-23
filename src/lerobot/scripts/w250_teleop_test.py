#!/usr/bin/env python3

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
W250 Keyboard Teleoperation Test Script
========================================

Tests the W250Interbotix robot middleware by teleoprating it with the keyboard.
Inspired by lerobot_record.py but simplified for simulation/testing purposes:
  - No dataset recording
  - Prints real-time robot state (joint positions + timing)
  - Validates that send_action() and get_observation() work correctly
  - Tests gripper open/close behavior

Usage (in WSL with ROS2 sourced and interbotix control node running):

    # Terminal 1 — launch interbotix control
    $ source ~/interbotix_ws/install/setup.bash
    $ ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250s

    # Terminal 2 — run this script
    $ source ~/interbotix_ws/install/setup.bash
    $ cd ~/your_lerobot_ws
    $ python -m lerobot.scripts.w250_teleop_test

Controls:
    A / D       Waist       (left / right)
    W / S       Shoulder    (up / down)
    I / K       Elbow       (up / down)
    J / L       Forearm roll
    U / O       Wrist angle
    T / Y       Wrist rotate
    G           Gripper open
    H           Gripper close
    R           Reset to REST position
    0           Reset to HOME
    + / -       Step size up / down
    P           Save current pose to /tmp/saved_pose.json
    F           Load saved pose and send to robot, then print error
    Q / ESC     Quit
"""

import csv
import json
import logging
import signal
import sys
import time
from pathlib import Path

# ── Setup logging before any other imports ────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("w250_teleop_test")

# ── Imports ───────────────────────────────────────────────────────────────────
from lerobot.robots.w250.config_w250_interbotix import W250InterbotixConfig
from lerobot.robots.w250.w250_interbotix import W250Interbotix
from lerobot.teleoperators.w250keyboard import W250KeyboardConfig, W250KeyboardTeleop


# ── Configuration — adjust to match your setup ───────────────────────────────

ROBOT_MODEL = "wx250s"       # Interbotix model name (wx250, wx250s, etc.)
ROBOT_NAME  = "wx250s"       # ROS2 namespace

# Frequency — single value, all timing derived from it.
# Must match: dataset fps, camera fps, W250InterbotixConfig.fps.
FPS         = 15             # Hz — change only this line
DISPLAY_HZ  = 5.0            # how often to print state to terminal (Hz)

# Derived — do not change directly
CONTROL_HZ  = float(FPS)
MOVING_TIME = 0.15            # safe for teleop; use 1/FPS for policy replay
ACCEL_TIME  = MOVING_TIME / 4

# Set True if you want the robot to start by moving to HOME then REST
CALIBRATE_ON_CONNECT = True

# Gripper log file — written to /tmp for easy access from WSL
GRIPPER_LOG_PATH = Path("/tmp/gripper_log.csv")

# Saved pose file (P = save, F = replay)
POSE_LOG_PATH = Path("/tmp/saved_pose.json")


# ── Helpers ───────────────────────────────────────────────────────────────────

def format_state(obs: dict[str, float]) -> str:
    """Format joint state as a compact one-line string."""
    joint_keys = [
        "waist.pos", "shoulder.pos", "elbow.pos",
        "forearm_roll.pos", "wrist_angle.pos", "wrist_rotate.pos", "gripper.pos",
    ]
    parts = []
    for k in joint_keys:
        if k in obs:
            short = k.replace(".pos", "")[:6]
            parts.append(f"{short}:{obs[k]:+.3f}")
    return "  ".join(parts)


def format_action_diff(action: dict[str, float], obs: dict[str, float]) -> str:
    """Show difference between commanded and observed positions."""
    diffs = []
    for k in action:
        if k in obs and isinstance(obs[k], float):
            diff = action[k] - obs[k]
            if abs(diff) > 0.01:
                diffs.append(f"{k.replace('.pos','')}:{diff:+.3f}")
    return ", ".join(diffs) if diffs else "—"


def save_pose(obs: dict[str, float]) -> None:
    """Save current observation to POSE_LOG_PATH."""
    data = {k: v for k, v in obs.items() if isinstance(v, float)}
    POSE_LOG_PATH.write_text(json.dumps(data, indent=2))
    sys.stdout.write(f"\n[POSE] Saved to {POSE_LOG_PATH}: {format_state(data)}\n")
    sys.stdout.flush()


def replay_pose(robot, path: Path) -> None:
    """
    Load saved pose and send it as an action. Then read back the observation
    and print per-joint error so we can verify round-trip accuracy.

    Note: gripper is excluded from replay — it is effort-controlled and a
    direct position replay doesn't make sense for it.
    """
    if not path.exists():
        sys.stdout.write("\n[POSE] No saved pose found — press P first.\n")
        sys.stdout.flush()
        return

    saved = json.loads(path.read_text())
    arm_keys = [k for k in saved if k != "gripper.pos"]

    sys.stdout.write(f"\n[POSE] Replaying: {format_state(saved)}\n")
    sys.stdout.flush()

    # Send action (arm joints only — gripper excluded)
    action = {k: saved[k] for k in arm_keys}
    robot.send_action(action)

    # Wait for the robot to (approximately) finish moving
    time.sleep(robot.config.moving_time + 0.2)

    # Read back observation and compare
    obs_after = robot.get_observation()
    sys.stdout.write("[POSE] Round-trip error (saved → action → observed):\n")
    max_err = 0.0
    for k in arm_keys:
        if k in obs_after:
            err = abs(saved[k] - obs_after[k])
            max_err = max(max_err, err)
            flag = "  ✓" if err < 0.02 else "  !" if err < 0.05 else "  ✗ LARGE"
            sys.stdout.write(f"  {k:<22} saved={saved[k]:+.4f}  obs={obs_after[k]:+.4f}  err={err:.4f}{flag}\n")
    sys.stdout.write(f"  max_err={max_err:.4f}  ({'OK' if max_err < 0.05 else 'CHECK LIMITS'})\n")
    sys.stdout.flush()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    # Build robot config
    robot_cfg = W250InterbotixConfig(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        fps=FPS,          # moving_time and accel_time auto-derived
        cameras={},       # no cameras for this test
    )

    # Build keyboard teleop config
    teleop_cfg = W250KeyboardConfig(
        step_size=0.01,
        gripper_step_size=0.10,
        control_hz=float(FPS),
    )

    robot = W250Interbotix(robot_cfg)
    teleop = W250KeyboardTeleop(teleop_cfg)

    # Handle Ctrl+C gracefully — restore terminal before exiting
    def _sigint_handler(sig, frame):
        logger.info("SIGINT received — shutting down...")
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

    # Connect keyboard and sync positions with robot state
    teleop.connect()
    teleop.sync_with_robot(initial_obs)

    # ── Gripper CSV log ────────────────────────────────────────────────────────
    # Columns:
    #   t_s          — seconds since start
    #   step         — control loop step number
    #   event        — G_PRESSED | H_PRESSED | STEP
    #   kb_gripper   — keyboard gripper.pos  [0,1]
    #   kb_delta     — change in keyboard gripper.pos this cycle
    #   effort_sent  — PWM effort published (+250 / −250 / 0)
    #   finger_m     — left_finger actual position [meters]
    #   finger_norm  — finger_m normalized to [0,1]
    #   lower_m      — left_finger URDF lower limit [meters]
    #   upper_m      — left_finger URDF upper limit [meters]
    #   gripper_moving — gripper_moving flag (True/False)
    #   note         — at_upper | at_lower | open | close | stop | n/a
    gripper_log_f = open(GRIPPER_LOG_PATH, "w", newline="")
    gripper_csv = csv.writer(gripper_log_f)
    gripper_csv.writerow([
        "t_s", "step", "event",
        "kb_gripper", "kb_delta",
        "effort_sent",
        "finger_m", "finger_norm", "lower_m", "upper_m",
        "gripper_moving", "note",
    ])
    logger.info(f"Gripper log → {GRIPPER_LOG_PATH}")

    def _read_gripper_hw():
        """Read gripper hardware state. Returns (effort, finger_m, lower, upper, moving)."""
        try:
            g = robot.bot.gripper
            effort = float(g.gripper_command.cmd)
            finger_m = float(g.get_finger_position())
            lower = float(g.left_finger_lower_limit)
            upper = float(g.left_finger_upper_limit)
            moving = bool(g.gripper_moving)
            return effort, finger_m, lower, upper, moving
        except Exception:
            return 0.0, 0.0, 0.015, 0.037, False

    # ── Control loop ──────────────────────────────────────────────────────────
    loop_dt = 1.0 / CONTROL_HZ
    display_dt = 1.0 / DISPLAY_HZ
    last_display_t = 0.0

    step = 0
    total_action_time_ms = 0.0
    total_obs_time_ms = 0.0
    t_start = time.perf_counter()
    prev_kb_gripper = initial_obs.get("gripper.pos", 0.0)

    logger.info(f"Starting control loop at {CONTROL_HZ:.0f} Hz. Press Q or ESC to quit.\n")

    while teleop.is_connected:
        loop_start = time.perf_counter()

        # ── Get teleop events ──────────────────────────────────────────────
        events = teleop.get_teleop_events()
        if events.get("terminate_episode", False):
            logger.info("Quit requested from keyboard.")
            break

        # ── Pose save / replay (P / F) ─────────────────────────────────────
        if teleop.consume_save_pose():
            # Read latest observation and save it
            try:
                current_obs = robot.get_observation()
                save_pose(current_obs)
                teleop.sync_with_robot(current_obs)  # re-sync keyboard to actual pos
            except Exception as e:
                sys.stdout.write(f"\n[POSE] Save failed: {e}\n")
                sys.stdout.flush()

        if teleop.consume_replay_pose():
            try:
                replay_pose(robot, POSE_LOG_PATH)
                # Re-sync keyboard so next action doesn't jump
                current_obs = robot.get_observation()
                teleop.sync_with_robot(current_obs)
            except Exception as e:
                sys.stdout.write(f"\n[POSE] Replay failed: {e}\n")
                sys.stdout.flush()

        # ── Get action from keyboard ───────────────────────────────────────
        action = teleop.get_action()
        kb_gripper = action.get("gripper.pos", 0.0)
        kb_delta = kb_gripper - prev_kb_gripper
        prev_kb_gripper = kb_gripper

        # ── Send action to robot ───────────────────────────────────────────
        t0 = time.perf_counter()
        try:
            sent_action = robot.send_action(action)
        except Exception as e:
            logger.error(f"send_action failed: {e}")
            break
        action_ms = (time.perf_counter() - t0) * 1e3
        total_action_time_ms += action_ms

        # ── Read gripper hardware state (after send_action) ───────────────
        effort, finger_m, lower_m, upper_m, gripper_moving = _read_gripper_hw()
        finger_norm = (finger_m - lower_m) / (upper_m - lower_m) if (upper_m > lower_m) else 0.0
        t_s = time.perf_counter() - t_start

        # Classify event
        if kb_delta > 0.001:
            event = "G_PRESSED"
        elif kb_delta < -0.001:
            event = "H_PRESSED"
        else:
            event = "STEP"

        # Note what the motor is doing
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

        # Write CSV row — always for G/H events, every 5 steps otherwise
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

        # ── Read observation from robot ────────────────────────────────────
        t0 = time.perf_counter()
        try:
            obs = robot.get_observation()
        except Exception as e:
            logger.error(f"get_observation failed: {e}")
            break
        obs_ms = (time.perf_counter() - t0) * 1e3
        total_obs_time_ms += obs_ms

        step += 1

        # ── Display state periodically ────────────────────────────────────
        now = time.perf_counter()
        if now - last_display_t >= display_dt:
            last_display_t = now
            avg_action = total_action_time_ms / step
            avg_obs = total_obs_time_ms / step
            diff_str = format_action_diff(sent_action, obs)

            sys.stdout.write(
                f"\r[step {step:5d}] {format_state(obs)}"
                f"  |  cmd_diff=[{diff_str}]"
                f"  |  act:{avg_action:.1f}ms obs:{avg_obs:.1f}ms"
                f"  |  step_size={teleop._step_size:.3f}"
                "    "
            )
            sys.stdout.flush()

        # ── Maintain loop rate ─────────────────────────────────────────────
        elapsed = time.perf_counter() - loop_start
        sleep_t = loop_dt - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    sys.stdout.write("\n")
    sys.stdout.flush()

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

    logger.info("Moving robot to sleep pose before disconnect...")
    try:
        robot.bot.arm.go_to_sleep_pose(blocking=True)
    except Exception as e:
        logger.warning(f"Could not move to sleep pose: {e}")

    robot.disconnect()
    logger.info("Done.")


if __name__ == "__main__":
    main()
