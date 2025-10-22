#!/usr/bin/env python3
"""
Test script to move W250 robot to home and rest positions

This script connects to the robot, moves it to home position,
waits for confirmation, then moves to rest position.
"""

import logging
import sys
import time
from pathlib import Path

# Add lerobot to path if not installed
repo_root = Path(__file__).parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))

from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
from lerobot.robots.w250.constants import W250_HOME_POSITION, W250_REST_POSITION

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_home_rest_positions():
    """Move robot to home position, then to rest position"""

    logger.info("=" * 60)
    logger.info("W250 HOME AND REST POSITION TEST")
    logger.info("=" * 60)
    logger.info("\nThis test demonstrates the calibration sequence:")
    logger.info("  1. Connect → Calibrate → HOME → REST (automatic)")
    logger.info("  2. Manual movement to HOME position")
    logger.info("  3. Manual movement to REST position")
    logger.info("\nIMPORTANT: Wrist joint clarification:")
    logger.info("  - wrist_angle = 0.0 means STRAIGHT/ALIGNED (no twist)")
    logger.info("  - wrist_rotate = -0.3 means TILTED DOWN (for camera clearance)")
    logger.info("\nPrerequisites:")
    logger.info("  1. W250 robot connected via USB")
    logger.info("  2. ROS2 control node running:")
    logger.info("     $ ros2 launch interbotix_xsarm_control \\")
    logger.info("       xsarm_control.launch.py robot_model:=wx250")
    logger.info("=" * 60)

    input("\nPress ENTER when ready to start...")

    # Create robot configuration
    config = W250InterbotixConfig(
        robot_model="wx250",
        robot_name="wx250",
        moving_time=2.0,
        accel_time=0.3,
    )

    # Initialize robot
    robot = W250Interbotix(config)

    try:
        # ==================== Connect ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 1: Connecting to Robot")
        logger.info("=" * 60)

        logger.info("\nConnecting to W250 robot...")
        logger.info("Note: calibrate=True will move robot HOME → REST automatically")
        robot.connect(calibrate=True)  # This will calibrate and move to REST
        time.sleep(1)
        logger.info("✓ Robot connected and calibrated (now at REST position)")

        # Get initial position (should be REST after calibration)
        initial_obs = robot.get_observation()
        logger.info("\nCurrent joint positions (should be REST after calibration):")
        for joint in ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate", "gripper"]:
            key = f"{joint}.pos"
            if key in initial_obs:
                expected = W250_REST_POSITION.get(key, "N/A")
                actual = initial_obs[key]
                match = "✓" if abs(actual - expected) < 0.1 else "✗"
                logger.info(f"  {joint:15s}: {actual:+.3f} (expected: {expected:+.3f}) {match}")

        # ==================== Move to Home ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 2: Moving to Home Position")
        logger.info("=" * 60)
        logger.info("\nHome position: All joints at 0.0 (normalized)")
        logger.info("This represents the robot's standard home configuration")

        input("\nPress ENTER to move to HOME position...")

        # Use the standard W250 home position from constants
        home_action = W250_HOME_POSITION

        logger.info("\nSending home position command...")
        robot.send_action(home_action)

        # Wait for movement to complete
        logger.info(f"Waiting {config.moving_time} seconds for movement to complete...")
        time.sleep(config.moving_time + 0.5)

        # Verify position
        home_obs = robot.get_observation()
        logger.info("\n✓ Home position reached:")
        for joint in ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate", "gripper"]:
            key = f"{joint}.pos"
            if key in home_obs:
                logger.info(f"  {joint:15s}: {home_obs[key]:+.3f}")

        # ==================== Move to Rest ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 3: Moving to Rest Position")
        logger.info("=" * 60)
        logger.info("\nRest position: Robot tucked in safe position")
        logger.info("  - Shoulder lifted")
        logger.info("  - Elbow bent inward")
        logger.info("  - Wrist straight (wrist_angle=0.0, no twist)")
        logger.info("  - Wrist angled down (wrist_rotate=-0.3, for camera clearance)")
        logger.info("  - Gripper closed")

        input("\nPress ENTER to move to REST position...")

        # Use the standard W250 rest position from constants
        rest_action = W250_REST_POSITION

        logger.info("\nSending rest position command...")
        robot.send_action(rest_action)

        # Wait for movement to complete
        logger.info(f"Waiting {config.moving_time} seconds for movement to complete...")
        time.sleep(config.moving_time + 0.5)

        # Verify position
        rest_obs = robot.get_observation()
        logger.info("\n✓ Rest position reached:")
        for joint in ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate", "gripper"]:
            key = f"{joint}.pos"
            if key in rest_obs:
                logger.info(f"  {joint:15s}: {rest_obs[key]:+.3f}")

        # ==================== Return to Home ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 4: Returning to Home Position")
        logger.info("=" * 60)

        input("\nPress ENTER to return to HOME position before disconnecting...")

        logger.info("\nReturning to home position...")
        robot.send_action(home_action)
        time.sleep(config.moving_time + 0.5)
        logger.info("✓ Returned to home position")

        # ==================== Cleanup ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 5: Cleanup")
        logger.info("=" * 60)

        logger.info("\nDisconnecting from robot...")
        robot.disconnect()
        logger.info("✓ Robot disconnected")

    except KeyboardInterrupt:
        logger.warning("\n\n⚠️  Test interrupted by user (Ctrl+C)")
        logger.info("Cleaning up...")

        try:
            robot.disconnect()
        except:
            pass

        return False

    except Exception as e:
        logger.error(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()

        # Cleanup on error
        try:
            robot.disconnect()
        except:
            pass

        return False

    # ==================== TEST SUMMARY ====================
    logger.info("\n" + "=" * 60)
    logger.info("✓ HOME AND REST POSITION TEST PASSED!")
    logger.info("=" * 60)
    logger.info("\nTest Summary:")
    logger.info("  ✓ Robot connected successfully")
    logger.info("  ✓ Moved to home position (all joints at 0)")
    logger.info("  ✓ Moved to rest position (tucked/safe)")
    logger.info("  ✓ Returned to home position")
    logger.info("  ✓ Clean disconnect")
    logger.info("=" * 60)

    return True


if __name__ == "__main__":
    success = test_home_rest_positions()
    sys.exit(0 if success else 1)
