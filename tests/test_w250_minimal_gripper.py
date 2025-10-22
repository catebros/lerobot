#!/usr/bin/env python3
"""
Minimal gripper test - just one open, one close, then immediate shutdown
"""

import logging
import sys
import time
from pathlib import Path

# Add lerobot to path if not installed
repo_root = Path(__file__).parent.parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_minimal_gripper():
    """Minimal test to isolate the loop issue"""

    logger.info("="*60)
    logger.info("MINIMAL GRIPPER TEST")
    logger.info("="*60)

    try:
        from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
    except ImportError as e:
        logger.error(f"✗ Failed to import: {e}")
        return False

    # Create robot
    config = W250InterbotixConfig(
        robot_model="wx250s",
        robot_name="wx250s",
        moving_time=2.0,
        accel_time=0.3,
    )
    robot = W250Interbotix(config)

    try:
        logger.info("\n1. CONNECTING...")
        robot.connect(calibrate=True)
        logger.info("✓ Connected")
        time.sleep(2)

        logger.info("\n2. OPENING GRIPPER...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 1.0
        robot.send_action(action)
        time.sleep(2)
        logger.info("✓ Open command sent")

        logger.info("\n3. CLOSING GRIPPER...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 0.0
        robot.send_action(action)
        time.sleep(2)
        logger.info("✓ Close command sent")

        logger.info("\n4. DISCONNECTING (watch for loop after this)...")
        robot.disconnect()
        logger.info("✓ Disconnect complete")

        logger.info("\n5. SCRIPT ENDING - If gripper loops now, it's ROS2 issue")
        logger.info("   Press Ctrl+C if you see the loop continuing...")

        # Keep script alive to monitor
        for i in range(10):
            logger.info(f"   Waiting... {i+1}/10s")
            time.sleep(1)

        logger.info("\n✓ TEST COMPLETE - No loop detected!")
        return True

    except Exception as e:
        logger.error(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_minimal_gripper()
    sys.exit(0 if success else 1)
