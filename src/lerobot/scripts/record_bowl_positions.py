#!/usr/bin/env python3
"""
record_bowl_positions.py — Interactively record end-effector positions for the W250 IK teleoperator.

Workflow:
  1. Connects to the wx250s via interbotix API and disables arm torques (gravity-comp mode).
  2. You physically move the arm to the CENTER (drop-off) position → press Enter.
  3. You move the arm to each BOWL (grasp) position → press Enter for each.
  4. Computes XYZ from FK and saves everything to a JSON file.

Usage:
    source ~/interbotix_ws/install/setup.bash
    python -m lerobot.scripts.record_bowl_positions --output /path/to/bowl_positions.json --num-bowls 2

The output JSON is loaded by the W250IKTeleop (w250ik) teleoperator during lerobot-record.
"""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import modern_robotics as mr
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

ROBOT_MODEL = "wx250s"
ROBOT_NAME  = "wx250s"

JOINT_NAMES = [
    "waist", "shoulder", "elbow",
    "forearm_roll", "wrist_angle", "wrist_rotate",
]

JOINT_TOPIC = f"/{ROBOT_NAME}/joint_states"


# ── ROS2 joint-state reader ──────────────────────────────────────────────────

class _JointReader(Node):
    """Thin ROS2 node that holds the latest joint positions from the hardware."""

    def __init__(self):
        super().__init__("bowl_position_recorder")
        self._lock = threading.Lock()
        self._positions: list[float] | None = None
        self.create_subscription(JointState, JOINT_TOPIC, self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        with self._lock:
            if all(n in name_to_pos for n in JOINT_NAMES):
                self._positions = [name_to_pos[n] for n in JOINT_NAMES]

    def get_positions(self) -> list[float] | None:
        with self._lock:
            return list(self._positions) if self._positions is not None else None


# ── Helpers ───────────────────────────────────────────────────────────────────

def _wait_for_positions(reader: _JointReader, executor, timeout: float = 5.0) -> list[float]:
    """Spin until we have a fresh joint-state reading."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        pos = reader.get_positions()
        if pos is not None:
            return pos
    raise TimeoutError(
        f"No joint states received on {JOINT_TOPIC} within {timeout}s. "
        "Is the robot running and ROS2 sourced?"
    )


def _read_ee_pose(reader: _JointReader, executor, robot_des) -> tuple[float, float, float, float]:
    """Return current EE position (x, y, z) in metres and roll in radians using FK.

    The roll is the jaw-axis orientation in the world XY plane, matching the
    convention used by W250IKTeleop._ik_to_waypoint:
      EE-Y column of R = [-cos(roll), sin(roll), 0]
    so roll = atan2(R[1,1], -R[0,1]).
    """
    thetas = _wait_for_positions(reader, executor)
    T = mr.FKinSpace(robot_des.M, robot_des.Slist, np.array(thetas))
    x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    R = T[:3, :3]
    roll = math.atan2(float(R[1, 1]), float(-R[0, 1]))
    return x, y, z, roll


def _record_one(reader: _JointReader, executor, robot_des, label: str) -> dict[str, float]:
    """Pause until the user presses Enter, then capture the current EE pose."""
    input(f"\n  → Physically move the arm to the {label.upper()} position and press ENTER ...")
    # Spin briefly to get the very latest state after the user releases the arm.
    for _ in range(10):
        executor.spin_once(timeout_sec=0.02)
    x, y, z, roll = _read_ee_pose(reader, executor, robot_des)
    print(f"    Captured: x={x:+.4f}  y={y:+.4f}  z={z:+.4f}  roll={math.degrees(roll):+.1f}°  ({roll:+.4f} rad)")
    return {"x": round(x, 5), "y": round(y, 5), "z": round(z, 5), "roll": round(roll, 5)}


# ── Torque helpers ────────────────────────────────────────────────────────────

def _set_torques(bot: InterbotixManipulatorXS, enable: bool) -> None:
    state = "ENABLED" if enable else "DISABLED (gravity-comp mode)"
    try:
        bot.core.robot_torque_enable(cmd_type="group", name="arm", enable=enable)
        print(f"  Arm torques: {state}")
    except Exception as exc:
        print(f"  WARNING — could not change torque state: {exc}")
        if not enable:
            print("  You can still move the robot using keyboard teleop or another method.")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record bowl/center positions for the W250 IK teleoperator.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--output", default="bowl_positions.json",
        help="Path to write the JSON file. Should match positions_file in config_w250ik.",
    )
    parser.add_argument(
        "--num-bowls", type=int, default=1,
        help="Number of bowl (grasp) positions to record.",
    )
    # --roll is intentionally removed: roll is now read from the actual FK of the robot pose.
    parser.add_argument(
        "--settle-time", type=float, default=3.0,
        help="Seconds to keep torques off after each capture so you can safely move the arm away. "
             "Applies between positions and after the last one. Press Ctrl+C during the wait to skip it.",
    )
    parser.add_argument(
        "--record-center", action="store_true", default=False,
        help="Also record the center (drop-off) position. "
             "By default the center is taken from the config defaults (center_x/y/z) "
             "and is NOT written to the JSON, so lerobot-record always uses the config value.",
    )
    args = parser.parse_args()

    output_path = Path(args.output)

    print(f"\n=== W250 Position Recorder ===")
    print(f"  Output         : {output_path}")
    print(f"  Bowls          : {args.num_bowls}")
    print(f"  Roll           : read from robot FK at capture time")
    print(f"  Record center  : {'yes' if args.record_center else 'no (using config default)'}")

    # ── ROS2 init ──────────────────────────────────────────────────────────────
    rclpy.init()
    reader   = _JointReader()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(reader)

    robot_des = getattr(mrd, ROBOT_MODEL.replace("-", "_"), None)
    if robot_des is None:
        print(f"ERROR: robot model '{ROBOT_MODEL}' not found in mr_descriptions.")
        rclpy.shutdown()
        sys.exit(1)

    # ── Connect robot & disable torques ───────────────────────────────────────
    print("\nConnecting to robot…")
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        moving_time=2.0,
        accel_time=0.3,
    )
    print("Connected.")

    print("\nDisabling arm torques so you can move the arm by hand.")
    print("Re-enable power at any time by pressing Ctrl+C (torques are restored on exit).")
    _set_torques(bot, enable=False)

    try:
        # ── Optionally record center ───────────────────────────────────────────
        center: dict | None = None
        total_steps = args.num_bowls + (1 if args.record_center else 0)
        step = 1

        if args.record_center:
            print(f"\n[Step {step}/{total_steps}]  Record the CENTER (drop-off) position.")
            print("  This is where the robot releases the object.")
            center = _record_one(reader, executor, robot_des, "center")
            step += 1
        else:
            print("\n  Center will use config defaults (center_x/y/z in config_w250ik.py).")
            print("  Pass --record-center to override.")

        # ── Record bowl positions ──────────────────────────────────────────────
        episodes: list[dict] = []
        for i in range(args.num_bowls):
            label = "bowl" if args.num_bowls == 1 else f"bowl_{i + 1}"
            print(f"\n[Step {step}/{total_steps}]  Record BOWL position {i + 1}/{args.num_bowls}.")
            print("  This is where the robot grasps the object.")
            pos = _record_one(reader, executor, robot_des, label)
            pos["label"] = label
            episodes.append(pos)
            step += 1

            is_last = (i == args.num_bowls - 1)
            if is_last:
                print(f"\n  All positions captured. Move the arm to a safe position.")
                print(f"  Torques re-enable in {args.settle_time:.0f} s  (Ctrl+C to skip wait)...")
            else:
                print(f"\n  Position captured. Move the arm away before the next step.")
                print(f"  Next step starts in {args.settle_time:.0f} s  (Ctrl+C to skip wait)...")

            try:
                for remaining in range(int(args.settle_time), 0, -1):
                    print(f"    {remaining}...", end="\r", flush=True)
                    time.sleep(1)
                print()
            except KeyboardInterrupt:
                print("\n  Wait skipped.")

    except KeyboardInterrupt:
        print("\nInterrupted — no file saved.")
        _set_torques(bot, enable=True)
        rclpy.shutdown()
        sys.exit(0)

    # ── Save ───────────────────────────────────────────────────────────────────
    _set_torques(bot, enable=True)
    rclpy.shutdown()

    # Only include "center" in JSON if it was explicitly recorded;
    # otherwise lerobot-record will use the config defaults.
    data: dict = {"episodes": episodes}
    if center is not None:
        data["center"] = center
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(data, indent=2))

    print(f"\n{'='*50}")
    print(f"Saved to {output_path}")
    print(json.dumps(data, indent=2))
    print(f"\nNext step — run the IK choreography recording:")
    print(f"  python -m lerobot.scripts.lerobot_record \\")
    print(f"      --robot.type=w250_interbotix \\")
    print(f"      --teleop.type=w250ik \\")
    print(f"      --teleop.positions_file={output_path.resolve()} \\")
    print(f"      --dataset.repo_id=<your-repo-id> \\")
    print(f"      --dataset.num_episodes={args.num_bowls}")


if __name__ == "__main__":
    main()
