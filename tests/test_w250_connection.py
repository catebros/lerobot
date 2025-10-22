#!/usr/bin/env python3
"""
Test script 1: Basic W250 Connection Test
Tests if the robot can connect and disconnect properly with ROS2
"""

import logging
import sys
from pathlib import Path

# Add lerobot to path if not installed
repo_root = Path(__file__).parent.parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))
    logger_setup = logging.getLogger(__name__)
    logger_setup.info(f"Added {repo_root / 'src'} to Python path")

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_connection():
    """Test basic connection to W250 robot"""

    logger.info("=" * 60)
    logger.info("TEST 1: Basic Connection Test")
    logger.info("=" * 60)

    try:
        # Import the robot class
        from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
        logger.info("✓ Successfully imported W250Interbotix classes")

    except ImportError as e:
        logger.error(f"✗ Failed to import W250 classes: {e}")
        logger.error("Make sure interbotix_xs_modules is installed:")
        logger.error("  $ pip install interbotix_xs_modules")
        return False

    try:
        # Create configuration
        config = W250InterbotixConfig(
            robot_model="wx250s",
            robot_name="wx250s",
            moving_time=2.0,
            accel_time=0.3,
        )
        logger.info("✓ Created robot configuration")
        logger.info(f"  - Robot model: {config.robot_model}")
        logger.info(f"  - Init node: {config.init_node}")
        logger.info(f"  - Moving time: {config.moving_time}s")

    except Exception as e:
        logger.error(f"✗ Failed to create configuration: {e}")
        return False

    try:
        # Create robot instance
        robot = W250Interbotix(config)
        logger.info("✓ Created robot instance")
        logger.info(f"  - Robot: {robot}")
        logger.info(f"  - Connected: {robot.is_connected}")

    except Exception as e:
        logger.error(f"✗ Failed to create robot instance: {e}")
        return False

    try:
        # Check observation and action features
        logger.info("\n" + "=" * 60)
        logger.info("Observation Features:")
        for key, value in robot.observation_features.items():
            logger.info(f"  - {key}: {value}")

        logger.info("\nAction Features:")
        for key, value in robot.action_features.items():
            logger.info(f"  - {key}: {value}")

    except Exception as e:
        logger.error(f"✗ Failed to get features: {e}")
        return False

    try:
        # Attempt connection (without calibration first)
        logger.info("\n" + "=" * 60)
        logger.info("Attempting to connect to robot...")
        logger.info("NOTE: Make sure the control node is running:")
        logger.info("  $ ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250s")
        logger.info("")

        robot.connect(calibrate=False)
        logger.info("✓ Successfully connected to robot!")
        logger.info(f"  - Is connected: {robot.is_connected}")
        logger.info(f"  - Is calibrated: {robot.is_calibrated}")

    except Exception as e:
        logger.error(f"✗ Failed to connect to robot: {e}")
        logger.error("\nPossible issues:")
        logger.error("  1. Control node not running")
        logger.error("  2. ROS2 not sourced")
        logger.error("  3. Robot not powered on or connected")
        return False

    try:
        # Test disconnect
        logger.info("\nAttempting to disconnect...")
        robot.disconnect()
        logger.info("✓ Successfully disconnected from robot")
        logger.info(f"  - Is connected: {robot.is_connected}")

    except Exception as e:
        logger.error(f"✗ Failed to disconnect: {e}")
        return False

    logger.info("\n" + "=" * 60)
    logger.info("✓ ALL CONNECTION TESTS PASSED!")
    logger.info("=" * 60)
    return True


if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)
