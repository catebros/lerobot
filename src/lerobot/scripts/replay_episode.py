"""
Replay a recorded episode from a LeRobotDataset using:
  - Mode 'actions'   : sends the recorded actions  (what was commanded)
  - Mode 'positions' : sends the recorded observed positions (what was physically there)

Usage:
    python3 src/lerobot/scripts/replay_episode.py --mode actions
    python3 src/lerobot/scripts/replay_episode.py --mode positions
    python3 src/lerobot/scripts/replay_episode.py --mode actions   --episode 0
    python3 src/lerobot/scripts/replay_episode.py --mode positions --episode 0
"""

import argparse
import logging
import time

import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.w250.config_w250_interbotix import W250InterbotixConfig
from lerobot.robots.w250.w250_interbotix import W250Interbotix

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

DATASET_REPO_ID = "catebros/w250-pick-place-test"
DATASET_ROOT    = "datasets/catebros/w250-pick-place-test"
ROBOT_MODEL     = "wx250s"
ROBOT_NAME      = "wx250s"
FPS             = 15

JOINT_KEYS = [
    "waist.pos", "shoulder.pos", "elbow.pos",
    "forearm_roll.pos", "wrist_angle.pos", "wrist_rotate.pos", "gripper.pos",
]


def load_episode(episode_idx: int) -> tuple[list[dict], list[dict]]:
    ds = LeRobotDataset(DATASET_REPO_ID, root=DATASET_ROOT)
    ep = ds.hf_dataset.filter(lambda x: x["episode_index"] == episode_idx)

    actions   = np.array(ep["action"])    # (T, 7)
    states    = np.array(ep["observation.state"])  # (T, 7)

    action_list   = [{k: float(actions[t, i])   for i, k in enumerate(JOINT_KEYS)} for t in range(len(actions))]
    position_list = [{k: float(states[t, i])    for i, k in enumerate(JOINT_KEYS)} for t in range(len(states))]

    logger.info(f"Loaded episode {episode_idx}: {len(action_list)} frames")
    return action_list, position_list


def make_robot() -> W250Interbotix:
    cfg = W250InterbotixConfig(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        fps=FPS,
        cameras={},
    )
    return W250Interbotix(cfg)


def replay(robot: W250Interbotix, commands: list[dict], label: str):
    logger.info(f"=== REPLAY: {label} ({len(commands)} frames at {FPS} fps) ===")
    step_s = 1.0 / FPS

    for t, cmd in enumerate(commands):
        t_start = time.perf_counter()

        robot.send_action(cmd)

        elapsed = time.perf_counter() - t_start
        sleep_t = step_s - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

        if t % 50 == 0:
            logger.info(f"  step {t:4d}/{len(commands)}  gripper={cmd['gripper.pos']:.3f}")

    logger.info(f"  Done: {label}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["actions", "positions"], required=True,
                        help="Replay using recorded actions or recorded observed positions")
    parser.add_argument("--episode", type=int, default=0)
    args = parser.parse_args()

    action_list, position_list = load_episode(args.episode)

    robot = make_robot()
    robot.connect()

    try:
        if args.mode == "actions":
            replay(robot, action_list, label="ACTIONS")
        else:
            replay(robot, position_list, label="POSITIONS")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
