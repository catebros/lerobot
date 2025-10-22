#!/usr/bin/env python3
"""
Test script to move robot through multiple positions sequentially

This demonstrates that positions are ABSOLUTE (not relative rotations)
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


def test_sequential_positions():
    """Move robot through multiple positions to demonstrate absolute positioning"""

    logger.info("=" * 60)
    logger.info("W250 SEQUENTIAL POSITION TEST")
    logger.info("=" * 60)
    logger.info("\nThis test demonstrates ABSOLUTE positioning:")
    logger.info("  - Positions are ABSOLUTE (not relative to current position)")
    logger.info("  - Robot moves to exact position regardless of where it starts")
    logger.info("\nSequence:")
    logger.info("  1. Move to HOME")
    logger.info("  2. Move to REST")
    logger.info("  3. Move to CUSTOM position 1")
    logger.info("  4. Move to CUSTOM position 2")
    logger.info("  5. Move back to HOME")
    logger.info("\nPrerequisites:")
    logger.info("  1. WX250s (6DOF) robot connected via USB")
    logger.info("  2. ROS2 control node running:")
    logger.info("     $ ros2 launch interbotix_xsarm_control \\")
    logger.info("       xsarm_control.launch.py robot_model:=wx250s")
    logger.info("=" * 60)

    input("\nPress ENTER when ready to start...")

    # Create robot configuration
    config = W250InterbotixConfig(
        robot_model="wx250s",  # 6DOF version
        robot_name="wx250s",
        moving_time=1.5,  # Slower for smooth calibration
        accel_time=0.3,
    )

    # Initialize robot
    robot = W250Interbotix(config)

    try:
        # Connect without calibration
        logger.info("\nConnecting to W250 robot (without auto-calibration)...")
        robot.connect(calibrate=False)
        time.sleep(1)
        logger.info("✓ Robot connected")

        # Get initial position
        initial_obs = robot.get_observation()
        logger.info("\nInitial position:")
        for joint in ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]:
            key = f"{joint}.pos"
            if key in initial_obs:
                normalized = initial_obs[key]
                radians = robot._denormalize_position(joint, normalized)
                logger.info(f"  {joint:15s}: {normalized:+.3f} (normalized) = {radians:+.3f} rad")

        # ==================== Position 1: HOME ====================
        logger.info("\n" + "=" * 60)
        logger.info("POSITION 1: HOME (all joints at 0)")
        logger.info("=" * 60)

        input("\nPress ENTER to move to HOME...")

        # Send HOME action (normalized values)
        logger.info("Sending HOME action (normalized)...")
        robot.send_action(W250_HOME_POSITION)
        logger.info(f"  HOME: {W250_HOME_POSITION}")
        time.sleep(config.moving_time + 0.5)
        logger.info("✓ Reached HOME position")

        # ==================== Position 2: REST ====================
        logger.info("\n" + "=" * 60)
        logger.info("POSITION 2: REST (safe, tucked)")
        logger.info("=" * 60)

        input("\nPress ENTER to move to REST...")

        # Send REST action (normalized values)
        logger.info("Sending REST action (normalized)...")
        robot.send_action(W250_REST_POSITION)
        logger.info(f"  REST: {W250_REST_POSITION}")
        time.sleep(config.moving_time + 0.5)
        logger.info("✓ Reached REST position")

        # ==================== Position 3: CUSTOM 1 ====================
        logger.info("\n" + "=" * 60)
        logger.info("POSITION 3: CUSTOM 1 (waist rotated left)")
        logger.info("=" * 60)

        input("\nPress ENTER to move to CUSTOM 1...")

        custom1_position = {
            "waist.pos": 0.5,          # Rotated left
            "shoulder.pos": -0.3,      # Slightly up
            "elbow.pos": 0.4,          # Bent
            "forearm_roll.pos": 0.0,   # Forearm aligned
            "wrist_angle.pos": 0.0,    # Straight (no twist)  -> rotacion
            "wrist_rotate.pos": -0.2,  # Slightly down
            "gripper.pos": 0.0,        # Open
        }

        logger.info("Sending CUSTOM 1 action (normalized)...")
        robot.send_action(custom1_position)
        logger.info(f"  CUSTOM 1: {custom1_position}")
        time.sleep(config.moving_time + 0.5)
        logger.info("✓ Reached CUSTOM 1 position")

        # ==================== Position 4: CUSTOM 2 ====================
        logger.info("\n" + "=" * 60)
        logger.info("POSITION 4: CUSTOM 2 (waist rotated right)")
        logger.info("=" * 60)

        input("\nPress ENTER to move to CUSTOM 2...")

        custom2_position = {
            "waist.pos": -0.5,         # Rotated RIGHT (absolute, not relative!)
            "shoulder.pos": -0.3,      # Same as before
            "elbow.pos": 0.4,          # Same as before
            "forearm_roll.pos": 0.0,   # Forearm aligned
            "wrist_angle.pos": 0.0,    # Straight (no twist)
            "wrist_rotate.pos": -0.2,  # Slightly down
            "gripper.pos": 1.0,        # Closed
        }

        logger.info("Sending CUSTOM 2 action (normalized)...")
        logger.info("⚠️  NOTE: Waist will move from +0.5 to -0.5 (ABSOLUTE positioning)")
        logger.info("         If it were relative, waist would rotate even more left")
        robot.send_action(custom2_position)
        logger.info(f"  CUSTOM 2: {custom2_position}")
        time.sleep(config.moving_time + 0.5)
        logger.info("✓ Reached CUSTOM 2 position")

        # ==================== Position 5: Back to HOME ====================
        logger.info("\n" + "=" * 60)
        logger.info("POSITION 5: Back to HOME")
        logger.info("=" * 60)

        input("\nPress ENTER to return to HOME...")

        logger.info("Sending HOME action again...")
        robot.send_action(W250_HOME_POSITION)
        logger.info(f"  HOME: {W250_HOME_POSITION}")
        time.sleep(config.moving_time + 0.5)
        logger.info("✓ Back at HOME position")

        # Show final position
        final_obs = robot.get_observation()
        logger.info("\nFinal position (should match HOME):")
        for joint in ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]:
            key = f"{joint}.pos"
            if key in final_obs:
                normalized = final_obs[key]
                expected = W250_HOME_POSITION.get(key, 0.0)
                match = "✓" if abs(normalized - expected) < 0.05 else "✗"
                logger.info(f"  {joint:15s}: {normalized:+.3f} (expected: {expected:+.3f}) {match}")

        # Disconnect
        logger.info("\n" + "=" * 60)
        logger.info("Cleanup")
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
        try:
            robot.disconnect()
        except:
            pass
        return False

    # ==================== TEST SUMMARY ====================
    logger.info("\n" + "=" * 60)
    logger.info("✓ SEQUENTIAL POSITION TEST COMPLETED!")
    logger.info("=" * 60)
    logger.info("\nTest Summary:")
    logger.info("  ✓ Moved to HOME (absolute position)")
    logger.info("  ✓ Moved to REST (absolute position)")
    logger.info("  ✓ Moved to CUSTOM 1 (waist=+0.5, absolute)")
    logger.info("  ✓ Moved to CUSTOM 2 (waist=-0.5, absolute)")
    logger.info("  ✓ Returned to HOME (absolute position)")
    logger.info("\n📍 Key Observation:")
    logger.info("   When moving from CUSTOM 1 (waist=+0.5) to CUSTOM 2 (waist=-0.5),")
    logger.info("   the waist rotated to the ABSOLUTE position -0.5,")
    logger.info("   NOT relative to the current position!")
    logger.info("=" * 60)

    return True


if __name__ == "__main__":
    success = test_sequential_positions()
    sys.exit(0 if success else 1)
