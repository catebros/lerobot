#!/usr/bin/env python3
"""
Single gripper command test - send EXACTLY one open and one close
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


def test_single_gripper_command():
    """Test sending exactly one open and one close command"""

    logger.info("="*60)
    logger.info("SINGLE GRIPPER COMMAND TEST")
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
        logger.info("\n1. CONNECTING (no gripper commands should happen)...")
        robot.connect(calibrate=True)
        logger.info("✓ Connected")
        time.sleep(2)

        logger.info("\n2. SENDING EXACTLY ONE OPEN COMMAND...")
        logger.info("   Watch: gripper should open ONCE only")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 1.0

        logger.info("   >>> Calling send_action() ONCE with gripper.pos=1.0")
        robot.send_action(action)

        logger.info("   >>> Waiting 3 seconds (gripper should NOT move again)...")
        time.sleep(3)
        logger.info("   ✓ If gripper only opened once, this is correct!")

        logger.info("\n3. SENDING EXACTLY ONE CLOSE COMMAND...")
        logger.info("   Watch: gripper should close ONCE only")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 0.0

        logger.info("   >>> Calling send_action() ONCE with gripper.pos=0.0")
        robot.send_action(action)

        logger.info("   >>> Waiting 3 seconds (gripper should NOT move again)...")
        time.sleep(3)
        logger.info("   ✓ If gripper only closed once, this is correct!")

        logger.info("\n4. SENDING SAME COMMAND AGAIN (should be ignored by threshold)...")
        logger.info("   Watch: gripper should NOT move")
        action["gripper.pos"] = 0.0  # Same as before

        logger.info("   >>> Calling send_action() with gripper.pos=0.0 (same as last)")
        robot.send_action(action)

        logger.info("   >>> Waiting 2 seconds...")
        time.sleep(2)
        logger.info("   ✓ Gripper should not have moved (command filtered by threshold)")

        logger.info("\n5. DISCONNECTING...")
        robot.disconnect()
        logger.info("✓ Disconnect complete")

        logger.info("\n6. Waiting 5 seconds to check for loop...")
        for i in range(5):
            logger.info(f"   {i+1}/5s - Gripper should be STOPPED")
            time.sleep(1)

        logger.info("\n✓ TEST COMPLETE!")
        logger.info("\nEXPECTED BEHAVIOR:")
        logger.info("  - Phase 2: Gripper opens ONCE")
        logger.info("  - Phase 3: Gripper closes ONCE")
        logger.info("  - Phase 4: Gripper does NOT move (filtered)")
        logger.info("  - Phase 5: No loop after disconnect")

        return True

    except Exception as e:
        logger.error(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_single_gripper_command()
    sys.exit(0 if success else 1)
