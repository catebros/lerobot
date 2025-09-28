#!/usr/bin/env python3

"""
Example script to test the WidowX 250 (W250) robot arm.
This script demonstrates basic functionality including connection, calibration, and movement.
"""

import time
import logging
from pathlib import Path

from lerobot.robots.w250 import W250, W250Config

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_w250_basic():
    """Test basic W250 functionality"""
    
    # Load configuration
    config = W250Config(
        port="/dev/ttyUSB0",  # Adjust this to your actual port
        max_relative_target=5.0,
        disable_torque_on_disconnect=True
    )
    
    # Create robot instance
    robot = W250(config)
    
    try:
        logger.info("Connecting to W250...")
        robot.connect(calibrate=False)  # Set to True for first use
        
        logger.info("Robot connected successfully!")
        
        # Test observation
        logger.info("Reading current position...")
        obs = robot.get_observation()
        logger.info(f"Current position: {obs}")
        
        # Test small movement (if calibrated)
        if robot.is_calibrated:
            logger.info("Testing small movement...")
            
            # Get current position
            current_pos = {key: value for key, value in obs.items() if key.endswith('.pos')}
            
            # Create small movement
            target_pos = current_pos.copy()
            target_pos["waist.pos"] += 0.05  # Small movement in waist
            
            logger.info(f"Moving to: {target_pos}")
            robot.send_action(target_pos)
            
            # Wait a bit
            time.sleep(2)
            
            # Move back
            logger.info("Moving back to original position...")
            robot.send_action(current_pos)
            time.sleep(2)
        else:
            logger.warning("Robot not calibrated. Skipping movement test.")
            
    except Exception as e:
        logger.error(f"Error during testing: {e}")
        
    finally:
        if robot.is_connected:
            logger.info("Disconnecting robot...")
            robot.disconnect()
            logger.info("Robot disconnected.")


def calibrate_w250():
    """Calibrate the W250 robot with detailed guidance"""
    
    config = W250Config(
        port="/dev/ttyUSB0",  # Adjust this to your actual port
        max_relative_target=5.0,
        disable_torque_on_disconnect=True
    )
    
    robot = W250(config)
    
    try:
        logger.info("=== WidowX-250 Calibration Process ===")
        logger.info("This process will guide you through calibrating your robot.")
        logger.info("Please ensure the robot is powered and properly connected.")
        
        logger.info("Connecting to W250...")
        robot.connect(calibrate=False)
        
        logger.info("Starting comprehensive calibration process...")
        logger.info("Follow the on-screen instructions carefully.")
        
        robot.calibrate()
        
        logger.info("✅ Calibration process completed successfully!")
        logger.info("Your robot is now ready for normal operation.")
        
    except KeyboardInterrupt:
        logger.warning("Calibration interrupted by user")
    except Exception as e:
        logger.error(f"Error during calibration: {e}")
        logger.error("Please check connections and try again")
        
    finally:
        if robot.is_connected:
            robot.disconnect()
            logger.info("Robot disconnected safely.")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="W250 Robot Test Script")
    parser.add_argument("--calibrate", action="store_true", 
                       help="Run calibration instead of basic test")
    
    args = parser.parse_args()
    
    if args.calibrate:
        calibrate_w250()
    else:
        test_w250_basic()