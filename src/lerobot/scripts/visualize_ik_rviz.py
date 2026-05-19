#!/usr/bin/env python3
"""
Visualize IK trajectories for all bowl episodes in RViz.
Uses the SAME W250IKTeleop that runs during lerobot-record — no robot hardware needed.

Workflow:
  # Terminal 1 — launch URDF + RViz (no hardware):
  ros2 launch interbotix_xsarm_descriptions xsarm_description.launch.py \
      robot_model:=wx250s use_rviz:=true

  # Terminal 2 — run this script:
  source ~/interbotix_ws/install/setup.bash
  python3 -m lerobot.scripts.visualize_ik_rviz
  python3 -m lerobot.scripts.visualize_ik_rviz --episode 0
  python3 -m lerobot.scripts.visualize_ik_rviz --speed 3.0 --show-reset
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lerobot.robots.w250.constants import W250_JOINT_LIMITS, W250_JOINT_NAMES
from lerobot.teleoperators.w250ik.config_w250ik import W250IKConfig
from lerobot.teleoperators.w250ik.w250ik import W250IKTeleop

ROBOT_NAME = "wx250s"

# Interbotix wx250s gripper servo range (radians), for visual only
_GRIPPER_OPEN_RAD   =  0.037
_GRIPPER_CLOSED_RAD = -0.017


def _denorm(joint_name: str, value: float) -> float:
    lo, hi = W250_JOINT_LIMITS[joint_name]
    return lo + (value + 1.0) * (hi - lo) / 2.0


def _gripper_rad(normalized: float) -> float:
    return _GRIPPER_CLOSED_RAD + normalized * (_GRIPPER_OPEN_RAD - _GRIPPER_CLOSED_RAD)


class IKVisualizerNode(Node):
    def __init__(self):
        super().__init__("ik_visualizer")
        self.pub = self.create_publisher(JointState, f"/{ROBOT_NAME}/joint_states", 10)

    def publish(self, frame: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in W250_JOINT_NAMES:
            key = f"{name}.pos"
            if key in frame:
                msg.name.append(name)
                msg.position.append(_denorm(name, frame[key]))
        if "gripper.pos" in frame:
            msg.name.append("gripper")
            msg.position.append(_gripper_rad(frame["gripper.pos"]))
        self.pub.publish(msg)


def play_frames(node: IKVisualizerNode, frames: list[dict], dt: float) -> None:
    for frame in frames:
        node.publish(frame)
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(dt)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=int, default=None,
                        help="Single episode to play (default: all 49)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier (default 1.0, try 3.0 for fast preview)")
    parser.add_argument("--show-reset", action="store_true",
                        help="Also animate the reset phase (not recorded by lerobot)")
    args = parser.parse_args()

    cfg    = W250IKConfig()
    teleop = W250IKTeleop(cfg)

    print("Building IK trajectories for all episodes (same as lerobot-record)...")
    teleop.connect(calibrate=False)

    n_ep = teleop.total_episodes
    fps  = cfg.fps
    dt   = 1.0 / (fps * args.speed)

    episodes = list(range(n_ep)) if args.episode is None else [args.episode]
    total_record_frames = sum(len(teleop._episodes[i][0]) for i in episodes)
    total_reset_frames  = sum(len(teleop._episodes[i][1]) for i in episodes)
    print(f"\nEpisodes to play   : {len(episodes)}/{n_ep}")
    print(f"Record frames total: {total_record_frames} ({total_record_frames/fps:.1f}s at 1×)")
    if args.show_reset:
        print(f"Reset frames total : {total_reset_frames} ({total_reset_frames/fps:.1f}s at 1×)")
    print(f"Playback speed     : {args.speed}×  →  eta {(total_record_frames/fps)/args.speed:.0f}s")
    print("\nPublishing to RViz — make sure Terminal 1 launched xsarm_description.launch.py\n")

    rclpy.init()
    node = IKVisualizerNode()

    try:
        for ep_idx in episodes:
            rec_frames, rst_frames = teleop._episodes[ep_idx]
            raw   = teleop._raw_episodes[ep_idx]
            label = raw.get("label", f"ep{ep_idx + 1}")
            print(f"  [{ep_idx + 1:2d}/{n_ep}] {label:20s}  "
                  f"rec={len(rec_frames):4d} fr ({len(rec_frames)/fps:.1f}s)  "
                  f"rst={len(rst_frames):4d} fr ({len(rst_frames)/fps:.1f}s)")

            play_frames(node, rec_frames, dt)

            if args.show_reset:
                play_frames(node, rst_frames, dt)
            else:
                # Hold last rest frame briefly so RViz doesn't jump between episodes
                node.publish(rst_frames[-1])
                rclpy.spin_once(node, timeout_sec=0)
                time.sleep(0.4)

        print("\nDone — all episodes played.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        teleop.disconnect()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
