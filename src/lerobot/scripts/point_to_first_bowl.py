#!/usr/bin/env python3
"""
Moves the robot to hover above the first bowl position in bowl_positions.json.
Run this before lerobot-record so you know where to place the bowl.

Usage:
    source ~/interbotix_ws/install/setup.bash
    python3 -m lerobot.scripts.point_to_first_bowl
"""
import json
import math
import sys
import time

import modern_robotics as mr
import numpy as np
import rclpy
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from lerobot.robots.w250.constants import W250_JOINT_LIMITS, W250_JOINT_NAMES
from lerobot.teleoperators.w250ik.config_w250ik import W250IKConfig

_TWO_PI = 2.0 * math.pi


def _wrap(theta, lo, hi):
    theta = (theta + math.pi) % _TWO_PI - math.pi
    for i, (l, h) in enumerate(zip(lo, hi)):
        if round(float(theta[i]), 3) < round(l, 3):
            theta[i] += _TWO_PI
        elif round(float(theta[i]), 3) > round(h, 3):
            theta[i] -= _TWO_PI
    return theta


def solve_ik(robot_des, lo, hi, x, y, z, roll):
    c, s = math.cos(roll), math.sin(roll)
    R = np.array([
        [0.0, -c,  s],
        [0.0,  s,  c],
        [-1.0, 0.0, 0.0],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    n = robot_des.Slist.shape[1]
    yaw = math.atan2(y, x)
    for seed in [
        np.array([yaw] + [0.0] * (n - 1)),
        np.zeros(n),
        np.array([-math.pi / 30 * 40] + [0.0] * (n - 1)),
        np.array([math.pi / 30 * 40] + [0.0] * (n - 1)),
    ]:
        theta, ok = mr.IKinSpace(robot_des.Slist, robot_des.M, T, seed, 0.001, 0.001)
        if ok:
            theta = _wrap(theta, lo, hi)
            if all(float(theta[i]) >= lo[i] - 1e-3 and float(theta[i]) <= hi[i] + 1e-3
                   for i in range(n)):
                return theta
    return None


def main():
    ik_cfg = W250IKConfig()

    with open(ik_cfg.positions_file) as f:
        data = json.load(f)

    ep = data["episodes"][0]
    label = ep.get("label", "ep1")
    x, y, z = float(ep["x"]), float(ep["y"]), float(ep["z"])
    roll = float(ep.get("roll", 0.0))
    z_hover = z + ik_cfg.approach_height

    print(f"\nFirst episode : {label}")
    print(f"  Bowl pos    : x={x:.4f}  y={y:.4f}  z={z:.4f}")
    print(f"  Roll        : {math.degrees(roll):.1f}°")
    print(f"  Hover height: z={z_hover:.4f}")

    robot_model = ik_cfg.robot_model
    robot_des = getattr(mrd, robot_model.replace("-", "_"))
    lo = [W250_JOINT_LIMITS[n][0] for n in W250_JOINT_NAMES]
    hi = [W250_JOINT_LIMITS[n][1] for n in W250_JOINT_NAMES]

    theta = solve_ik(robot_des, lo, hi, x, y, z_hover, roll)
    if theta is None:
        print("ERROR: IK failed for hover position.")
        sys.exit(1)

    print(f"  waist={math.degrees(float(theta[0])):+.1f}°  "
          f"shoulder={math.degrees(float(theta[1])):+.1f}°  "
          f"elbow={math.degrees(float(theta[2])):+.1f}°  "
          f"wrist_r={math.degrees(float(theta[5])):+.1f}°")

    print("\nConnecting to robot...")
    rclpy.init()
    bot = InterbotixManipulatorXS(
        robot_model=robot_model,
        robot_name=robot_model,
        moving_time=3.0,
        accel_time=0.5,
    )

    print(f"Moving to hover above '{label}' (takes ~3 s)...")
    bot.arm.set_joint_positions(list(theta))
    bot.gripper.release()

    print(f"\nRobot is hovering above '{label}'.")
    print("Place the bowl directly below the gripper, then run lerobot-record.")
    input("\nPress ENTER to return to rest position and disconnect...")

    bot.arm.go_to_sleep_pose()
    time.sleep(1.5)
    rclpy.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
