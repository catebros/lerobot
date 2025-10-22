#!/usr/bin/env python3
"""
Test script 2: Joint Movement Test
Tests reading joint positions and sending simple movements
"""

import logging
import sys
import time
import numpy as np
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


def test_joints():
    """Test joint reading and movement"""

    logger.info("=" * 60)
    logger.info("TEST 2: Joint Movement Test")
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
        # Connect with calibration
        logger.info("Connecting and calibrating robot...")
        robot.connect(calibrate=True)
        logger.info("✓ Robot connected and calibrated")

        # Wait for calibration to complete
        time.sleep(2)

    except Exception as e:
        logger.error(f"✗ Connection failed: {e}")
        return False

    try:
        # Test 1: Read current observation
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Reading current observation")
        logger.info("=" * 60)

        obs = robot.get_observation()

        logger.info("Current joint positions (normalized to [-1, 1]):")
        for joint in ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate", "gripper"]:
            key = f"{joint}.pos"
            if key in obs:
                logger.info(f"  - {joint:15s}: {obs[key]:+.3f}")

        logger.info("✓ Successfully read observation")

    except Exception as e:
        logger.error(f"✗ Failed to read observation: {e}")
        robot.disconnect()
        return False

    try:
        # Test 2: Small movement test (blocking=False verification)
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Small joint movement (testing blocking=False)")
        logger.info("=" * 60)

        # Get current positions
        initial_obs = robot.get_observation()

        # Create small movement command (move waist slightly)
        action = {}
        for key in robot.action_features.keys():
            if key in initial_obs:
                action[key] = initial_obs[key]

        # Small movement on waist (0.1 in normalized space)
        if "waist.pos" in action:
            original_waist = action["waist.pos"]
            action["waist.pos"] = max(-1.0, min(1.0, original_waist + 0.1))
            logger.info(f"Moving waist from {original_waist:.3f} to {action['waist.pos']:.3f}")

        # Send action
        start_time = time.time()
        robot.send_action(action)
        command_time = time.time() - start_time

        logger.info(f"✓ Command sent in {command_time*1000:.1f}ms")
        logger.info("  (Should be <10ms with blocking=False)")

        if command_time < 0.1:  # Should be very fast with blocking=False
            logger.info("✓ Command is non-blocking (correct!)")
        else:
            logger.warning("⚠ Command took too long, might be blocking")

        # Wait for movement to complete
        time.sleep(2.5)

        # Read new position
        new_obs = robot.get_observation()
        if "waist.pos" in new_obs:
            logger.info(f"New waist position: {new_obs['waist.pos']:.3f}")
            logger.info("✓ Movement completed")

    except Exception as e:
        logger.error(f"✗ Movement test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Test 3: Return to home
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Return to home position")
        logger.info("=" * 60)

        robot.calibrate()
        time.sleep(2.5)
        logger.info("✓ Returned to home position")

    except Exception as e:
        logger.error(f"✗ Failed to return home: {e}")
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
    logger.info("✓ ALL JOINT TESTS PASSED!")
    logger.info("=" * 60)
    return True


if __name__ == "__main__":
    success = test_joints()
    sys.exit(0 if success else 1)
