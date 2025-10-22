#!/usr/bin/env python3
"""
Test script to find the correct wrist_rotate value for straight/aligned position

This script allows you to manually adjust the wrist_rotate angle to find
the value that makes the wrist straight/aligned/horizontal.
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

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_wrist_alignment():
    """Find the correct wrist_rotate value for straight/aligned position"""

    logger.info("=" * 60)
    logger.info("W250 WRIST ALIGNMENT FINDER")
    logger.info("=" * 60)
    logger.info("\nThis script helps you find the correct wrist_rotate value")
    logger.info("for a straight/aligned/horizontal wrist position.")
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
        moving_time=1.5,  # Faster for testing
        accel_time=0.3,
    )

    # Initialize robot
    robot = W250Interbotix(config)

    try:
        # Connect (without calibration to avoid automatic movement)
        logger.info("\nConnecting to W250 robot...")
        robot.connect(calibrate=False)
        time.sleep(1)
        logger.info("✓ Robot connected")

        # Get current position
        obs = robot.get_observation()
        current_wrist_rotate_normalized = obs.get("wrist_rotate.pos", 0.0)
        current_wrist_rotate_rad = robot._denormalize_position("wrist_rotate", current_wrist_rotate_normalized)

        logger.info(f"\nCurrent wrist_rotate:")
        logger.info(f"  Normalized: {current_wrist_rotate_normalized:.3f}")
        logger.info(f"  Radians:    {current_wrist_rotate_rad:.3f}")

        # Move to HOME position first
        logger.info("\n" + "=" * 60)
        logger.info("Moving to HOME position")
        logger.info("=" * 60)
        logger.info("\nJoint positions (radians): [waist, shoulder, elbow, wrist_angle, wrist_rotate]")

        home_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # All at 0 including wrist_rotate
        logger.info(f"Setting: {home_positions}")
        logger.info("  waist=0, shoulder=0, elbow=0, wrist_angle=0, wrist_rotate=0")
        robot.bot.arm.set_joint_positions(home_positions, blocking=True)
        time.sleep(1.5)

        logger.info("\n✓ At HOME position (all joints at 0)")
        logger.info("\n⚠️  IMPORTANT: Look at the wrist now!")
        logger.info("    Is it STRAIGHT/ALIGNED/HORIZONTAL?")
        response = input("    If YES, we're done! If NO, press ENTER to find correct value: ").strip().lower()

        if response == 'yes' or response == 'y':
            logger.info("\n✓ Great! wrist_rotate = 0.0 is already correct!")
            logger.info("No changes needed to the code.")
            robot.disconnect()
            return True

        # Test different wrist_rotate values
        logger.info("\n" + "=" * 60)
        logger.info("FINDING STRAIGHT WRIST_ROTATE VALUE")
        logger.info("=" * 60)

        # Start from -π to +π in steps
        test_values_rad = [
            -3.14, -2.5, -2.0, -1.5, -1.0, -0.5,
            0.0,
            0.5, 1.0, 1.5, 2.0, 2.5, 3.14
        ]

        logger.info("\nI will move the wrist_rotate through different angles.")
        logger.info("Watch the robot and note which value makes it STRAIGHT/ALIGNED/HORIZONTAL")
        input("\nPress ENTER to start the test sequence...")

        for test_rad in test_values_rad:
            logger.info(f"\n{'='*60}")
            logger.info(f"Testing joint value = {test_rad:.3f} radians ({test_rad * 180 / 3.14159:.1f} degrees)")
            logger.info(f"{'='*60}")

            # IMPORTANT: Testing if it's actually wrist_angle (index 3) that rotates
            # Move only index 3, keep everything else at 0
            test_positions = [0.0, 0.0, 0.0, test_rad, 0.0]
            logger.info(f"Positions: [waist=0, shoulder=0, elbow=0, JOINT_3={test_rad:.3f}, joint_4=0]")
            logger.info("⚠️  ONLY joint index 3 (wrist_angle?) is changing, watch it ROTATE/TWIST")
            robot.bot.arm.set_joint_positions(test_positions, blocking=True)
            time.sleep(1.5)

            # Ask user
            response = input("Is the wrist STRAIGHT/ALIGNED now? (y/n/quit): ").strip().lower()

            if response == 'y':
                logger.info("\n" + "=" * 60)
                logger.info("✓ FOUND THE CORRECT VALUE!")
                logger.info("=" * 60)
                logger.info(f"\nWrist is straight at:")
                logger.info(f"  Joint index 3 = {test_rad:.3f} radians")
                logger.info(f"  Joint index 3 = {test_rad * 180 / 3.14159:.1f} degrees")

                # Calculate normalized value for wrist_angle
                lower, upper = robot._joint_limits.get("wrist_angle", (-1.745, 1.745))
                normalized = 2.0 * (test_rad - lower) / (upper - lower) - 1.0

                logger.info(f"  normalized   = {normalized:.3f} (for LeRobot)")

                logger.info("\n" + "=" * 60)
                logger.info("⚠️  IMPORTANT DISCOVERY:")
                logger.info("=" * 60)
                logger.info("\nIt appears that joint index 3 (labeled 'wrist_angle') is actually")
                logger.info("the one that ROTATES/TWISTS the wrist!")
                logger.info("\nThis means the joint names might be swapped in the Interbotix API.")
                logger.info("\nUPDATE INSTRUCTIONS:")
                logger.info("=" * 60)
                logger.info("\n1. Update src/lerobot/robots/w250/constants.py:")
                logger.info(f"   Change W250_REST_POSITION['wrist_angle.pos'] = {normalized:.3f}")
                logger.info(f"   Change W250_HOME_POSITION['wrist_angle.pos'] = {normalized:.3f}")
                logger.info("\n2. Update src/lerobot/robots/w250/w250_interbotix.py:")
                logger.info("   In calibrate() method, change:")
                logger.info(f"     home_position = [0.0, 0.0, 0.0, {test_rad:.3f}, 0.0]")
                logger.info(f"     rest joint_3 = {test_rad:.3f}")
                logger.info("\nOR we might need to swap the joint name mappings in the code!")

                break
            elif response == 'quit':
                logger.info("\nTest cancelled by user")
                break
            else:
                logger.info("Moving to next test value...")

        # Return to safe position
        logger.info("\n" + "=" * 60)
        logger.info("Cleanup: Returning to HOME")
        logger.info("=" * 60)

        robot.bot.arm.set_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0], blocking=True)
        time.sleep(1)

        # Disconnect
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

    logger.info("\n" + "=" * 60)
    logger.info("✓ WRIST ALIGNMENT TEST COMPLETED")
    logger.info("=" * 60)

    return True


if __name__ == "__main__":
    success = test_wrist_alignment()
    sys.exit(0 if success else 1)
