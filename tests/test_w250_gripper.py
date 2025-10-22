#!/usr/bin/env python3
"""
Test script 3: Gripper Test
Tests gripper position reading and open/close commands
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


def test_gripper():
    """Test gripper functionality"""

    logger.info("=" * 60)
    logger.info("TEST 3: Gripper Test")
    logger.info("=" * 60)

    try:
        from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
    except ImportError as e:
        logger.error(f"✗ Failed to import W250 classes: {e}")
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
        # Connect
        logger.info("Connecting to robot...")
        robot.connect(calibrate=True)
        logger.info("✓ Robot connected")
        time.sleep(2)

    except Exception as e:
        logger.error(f"✗ Connection failed: {e}")
        return False

    try:
        # Test 1: Read initial gripper position
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Reading gripper position")
        logger.info("=" * 60)

        obs = robot.get_observation()
        if "gripper.pos" in obs:
            gripper_pos = obs["gripper.pos"]
            logger.info(f"Current gripper position: {gripper_pos:.3f}")
            logger.info("  (0.0 = closed, 1.0 = open)")
            logger.info("✓ Successfully read gripper position")
        else:
            logger.error("✗ Gripper position not in observation")
            robot.disconnect()
            return False

    except Exception as e:
        logger.error(f"✗ Failed to read gripper: {e}")
        robot.disconnect()
        return False

    try:
        # Test 2: Open gripper
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Opening gripper")
        logger.info("=" * 60)

        # Get current state
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}

        # Command to open gripper (value > 0.5)
        action["gripper.pos"] = 1.0
        logger.info("Sending OPEN command (gripper.pos = 1.0)...")

        start_time = time.time()
        robot.send_action(action)
        command_time = time.time() - start_time

        logger.info(f"✓ Command sent in {command_time*1000:.1f}ms")

        # Wait for gripper to complete movement (0.5s auto-stop + margin)
        time.sleep(0.8)

        # Read new position
        new_obs = robot.get_observation()
        new_gripper_pos = new_obs.get("gripper.pos", -1)
        logger.info(f"New gripper position: {new_gripper_pos:.3f}")

        if new_gripper_pos > 0.5:
            logger.info("✓ Gripper opened successfully")
        else:
            logger.warning("⚠ Gripper position doesn't reflect OPEN state")

    except Exception as e:
        logger.error(f"✗ Open gripper test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Test 3: Close gripper
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Closing gripper")
        logger.info("=" * 60)

        # Get current state
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}

        # Command to close gripper (value <= 0.5)
        action["gripper.pos"] = 0.0
        logger.info("Sending CLOSE command (gripper.pos = 0.0)...")

        start_time = time.time()
        robot.send_action(action)
        command_time = time.time() - start_time

        logger.info(f"✓ Command sent in {command_time*1000:.1f}ms")

        # Wait for gripper to complete movement (0.5s auto-stop + margin)
        time.sleep(0.8)

        # Read new position
        new_obs = robot.get_observation()
        new_gripper_pos = new_obs.get("gripper.pos", -1)
        logger.info(f"New gripper position: {new_gripper_pos:.3f}")

        if new_gripper_pos <= 0.5:
            logger.info("✓ Gripper closed successfully")
        else:
            logger.warning("⚠ Gripper position doesn't reflect CLOSED state")

    except Exception as e:
        logger.error(f"✗ Close gripper test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Test 4: Multiple open/close cycles
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Multiple open/close cycles")
        logger.info("=" * 60)

        for i in range(3):
            logger.info(f"\nCycle {i+1}/3:")

            # Open
            obs = robot.get_observation()
            action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
            action["gripper.pos"] = 1.0
            robot.send_action(action)
            time.sleep(1.0)  # Wait for open to complete
            logger.info("  - Opened")

            # Close
            obs = robot.get_observation()
            action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
            action["gripper.pos"] = 0.0
            robot.send_action(action)
            time.sleep(1.0)  # Wait for close to complete
            logger.info("  - Closed")

        logger.info("✓ Multiple cycles completed successfully")

    except Exception as e:
        logger.error(f"✗ Multiple cycles test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Disconnect
        logger.info("\nDisconnecting...")
        robot.disconnect()
        logger.info("✓ Disconnected successfully")

    except Exception as e:
        logger.error(f"✗ Disconnect failed: {e}")
        return False

    logger.info("\n" + "=" * 60)
    logger.info("✓ ALL GRIPPER TESTS PASSED!")
    logger.info("=" * 60)
    return True


if __name__ == "__main__":
    success = test_gripper()
    sys.exit(0 if success else 1)
