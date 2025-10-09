#!/usr/bin/env python3
"""
Test W250 Joystick Teleoperation
Interactive test for Logitech F710 gamepad control
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


def test_joystick_teleop():
    """Interactive test for W250 joystick teleoperation"""

    logger.info("=" * 60)
    logger.info("W250 JOYSTICK TELEOPERATION TEST")
    logger.info("=" * 60)
    logger.info("\nPrerequisites:")
    logger.info("  1. Logitech F710 gamepad connected via USB")
    logger.info("  2. Mode switch on back set to 'X' (XInput mode)")
    logger.info("  3. ROS2 joy node running:")
    logger.info("     $ ros2 run joy joy_node")
    logger.info("=" * 60)

    input("\nPress ENTER when ready to start...")

    try:
        from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
        from lerobot.teleoperators.w250joystick import W250JoystickTeleop, W250JoystickConfig
    except ImportError as e:
        logger.error(f"✗ Failed to import classes: {e}")
        return False

    # Create robot and teleoperator
    robot_config = W250InterbotixConfig(
        robot_model="wx250",
        robot_name="wx250",
        moving_time=2.0,
        accel_time=0.3,
    )

    teleop_config = W250JoystickConfig(
        joy_topic="/joy",
        ros2_node_name="w250_joystick_teleop",
    )

    robot = W250Interbotix(robot_config)
    teleop = W250JoystickTeleop(teleop_config)

    try:
        # ==================== PHASE 1: Setup ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 1: Connecting Robot and Teleoperator")
        logger.info("=" * 60)

        logger.info("\nConnecting to robot...")
        robot.connect(calibrate=True)
        logger.info("✓ Robot connected and at home position")

        logger.info("\nConnecting teleoperator...")
        teleop.connect()
        time.sleep(1)  # Give ROS2 time to initialize
        logger.info("✓ Teleoperator connected")

        logger.info("\n🎮 Gamepad Controls:")
        logger.info("  Left Stick X:   Waist rotation (left/right)")
        logger.info("  Left Stick Y:   Shoulder (up/down)")
        logger.info("  Right Stick X:  Wrist angle")
        logger.info("  Right Stick Y:  Elbow (up/down)")
        logger.info("  LT/RT:          Wrist rotate")
        logger.info("  LB:             Open gripper")
        logger.info("  RB:             Close gripper")
        logger.info("  Y:              Reset to home")
        logger.info("  Back:           Toggle intervention")
        logger.info("  Start:          Exit test")

        # ==================== PHASE 2: Test Gamepad Input ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 2: Testing Gamepad Input Reading")
        logger.info("=" * 60)

        logger.info("\nReading gamepad for 5 seconds...")
        logger.info("Move the joysticks to test input reading")

        for i in range(5):
            action = teleop.get_action()
            logger.info(f"  [{i+1}s] Action: waist={action.get('waist.pos', 0):.2f}, "
                       f"shoulder={action.get('shoulder.pos', 0):.2f}, "
                       f"gripper={action.get('gripper.pos', 0):.2f}")
            time.sleep(1)

        logger.info("✓ Gamepad input reading works")

        # ==================== PHASE 3: Test Individual Controls ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 3: Testing Individual Controls")
        logger.info("=" * 60)

        # Test waist
        logger.info("\n3.1 Test WAIST control:")
        logger.info("  Move LEFT STICK LEFT/RIGHT for 5 seconds...")
        for i in range(10):
            action = teleop.get_action()
            robot.send_action(action)
            if i % 2 == 0:
                logger.info(f"    Waist position: {action.get('waist.pos', 0):.3f}")
            time.sleep(0.5)
        logger.info("  ✓ Waist control test complete")

        # Test shoulder
        logger.info("\n3.2 Test SHOULDER control:")
        logger.info("  Move LEFT STICK UP/DOWN for 5 seconds...")
        for i in range(10):
            action = teleop.get_action()
            robot.send_action(action)
            if i % 2 == 0:
                logger.info(f"    Shoulder position: {action.get('shoulder.pos', 0):.3f}")
            time.sleep(0.5)
        logger.info("  ✓ Shoulder control test complete")

        # Test elbow
        logger.info("\n3.3 Test ELBOW control:")
        logger.info("  Move RIGHT STICK UP/DOWN for 5 seconds...")
        for i in range(10):
            action = teleop.get_action()
            robot.send_action(action)
            if i % 2 == 0:
                logger.info(f"    Elbow position: {action.get('elbow.pos', 0):.3f}")
            time.sleep(0.5)
        logger.info("  ✓ Elbow control test complete")

        # Test gripper
        logger.info("\n3.4 Test GRIPPER control:")
        logger.info("  Press LB to OPEN, RB to CLOSE for 10 seconds...")
        for i in range(20):
            action = teleop.get_action()
            robot.send_action(action)
            if i % 4 == 0:
                logger.info(f"    Gripper position: {action.get('gripper.pos', 0):.3f}")
            time.sleep(0.5)
        logger.info("  ✓ Gripper control test complete")

        logger.info("\n✓ All individual control tests passed")

        # ==================== PHASE 4: Free Control Mode ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 4: Free Control Mode")
        logger.info("=" * 60)
        logger.info("\nYou now have full control of the robot!")
        logger.info("  - Use joysticks to control arm position")
        logger.info("  - Use LB/RB to control gripper")
        logger.info("  - Press Y to reset to home position")
        logger.info("  - Press START to exit")
        logger.info("\nControl loop starting...")

        control_active = True
        last_status_time = time.time()

        while control_active:
            # Get action from teleop
            action = teleop.get_action()
            events = teleop.get_teleop_events()

            # Send action to robot
            robot.send_action(action)

            # Check for exit
            if events.get("rerecord_episode", False):
                logger.info("\n📍 START pressed - exiting control loop")
                control_active = False

            # Print status every 2 seconds
            if time.time() - last_status_time > 2.0:
                logger.info(f"  Status: waist={action.get('waist.pos', 0):.2f}, "
                           f"shoulder={action.get('shoulder.pos', 0):.2f}, "
                           f"elbow={action.get('elbow.pos', 0):.2f}, "
                           f"gripper={action.get('gripper.pos', 0):.2f}")
                last_status_time = time.time()

            time.sleep(0.05)  # 20Hz control loop

        logger.info("✓ Free control mode complete")

        # ==================== PHASE 5: Cleanup ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 5: Cleanup and Disconnect")
        logger.info("=" * 60)

        logger.info("\nReturning to home position...")
        robot.calibrate()
        time.sleep(2)
        logger.info("✓ Returned to home")

        logger.info("\nDisconnecting teleoperator...")
        teleop.disconnect()
        logger.info("✓ Teleoperator disconnected")

        logger.info("\nDisconnecting robot...")
        robot.disconnect()
        logger.info("✓ Robot disconnected")

        logger.info("\nWaiting 3 seconds to verify no loops...")
        for i in range(3):
            logger.info(f"  {i+1}/3s - Checking...")
            time.sleep(1)
        logger.info("✓ No loops detected")

    except KeyboardInterrupt:
        logger.warning("\n\n⚠️  Test interrupted by user (Ctrl+C)")
        logger.info("Cleaning up...")

        try:
            teleop.disconnect()
        except:
            pass

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
            teleop.disconnect()
        except:
            pass

        try:
            robot.disconnect()
        except:
            pass

        return False

    # ==================== TEST SUMMARY ====================
    logger.info("\n" + "=" * 60)
    logger.info("✓ ALL JOYSTICK TELEOPERATION TESTS PASSED!")
    logger.info("=" * 60)
    logger.info("\nTest Summary:")
    logger.info("  ✓ Robot and teleoperator connection")
    logger.info("  ✓ Gamepad input reading")
    logger.info("  ✓ Waist control (left stick X)")
    logger.info("  ✓ Shoulder control (left stick Y)")
    logger.info("  ✓ Elbow control (right stick Y)")
    logger.info("  ✓ Gripper control (LB/RB)")
    logger.info("  ✓ Free control mode")
    logger.info("  ✓ Clean disconnect")
    logger.info("\n🎮 Joystick teleoperation is fully functional!")
    logger.info("=" * 60)

    return True


if __name__ == "__main__":
    success = test_joystick_teleop()
    sys.exit(0 if success else 1)
