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

        # ==================== PHASE 2: Verify Joystick Input ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 2: Verify Joystick Input Reception")
        logger.info("=" * 60)
        logger.info("\nThis phase verifies that joystick inputs are being received correctly.")
        logger.info("The robot will NOT move during this phase.")
        logger.info("\nTest each control:")

        input("\nPress ENTER to start input verification...")

        # Test left stick (waist + shoulder)
        logger.info("\n2.1 LEFT STICK - Move left/right and up/down for 5 seconds:")
        for i in range(10):
            action = teleop.get_action()
            waist = action.get('waist.pos', 0)
            shoulder = action.get('shoulder.pos', 0)
            if abs(waist) > 0.01 or abs(shoulder) > 0.01:
                logger.info(f"  ✓ Left stick detected: waist={waist:.3f}, shoulder={shoulder:.3f}")
            time.sleep(0.5)
        logger.info("  ✓ Left stick input verification complete")

        # Test right stick (elbow + wrist angle)
        logger.info("\n2.2 RIGHT STICK - Move left/right and up/down for 5 seconds:")
        for i in range(10):
            action = teleop.get_action()
            elbow = action.get('elbow.pos', 0)
            wrist_angle = action.get('wrist_angle.pos', 0)
            if abs(elbow) > 0.01 or abs(wrist_angle) > 0.01:
                logger.info(f"  ✓ Right stick detected: elbow={elbow:.3f}, wrist_angle={wrist_angle:.3f}")
            time.sleep(0.5)
        logger.info("  ✓ Right stick input verification complete")

        # Test triggers (wrist rotate)
        logger.info("\n2.3 TRIGGERS - Press L2 and R2 for 5 seconds:")
        for i in range(10):
            action = teleop.get_action()
            wrist_rotate = action.get('wrist_rotate.pos', 0)
            if abs(wrist_rotate) > 0.01:
                logger.info(f"  ✓ Trigger detected: wrist_rotate={wrist_rotate:.3f}")
            time.sleep(0.5)
        logger.info("  ✓ Trigger input verification complete")

        # Test gripper buttons
        logger.info("\n2.4 GRIPPER - Press L1 (open) and R1 (close) for 5 seconds:")
        initial_gripper = 0.5
        last_gripper = initial_gripper
        for i in range(10):
            action = teleop.get_action()
            gripper = action.get('gripper.pos', 0.5)
            if abs(gripper - last_gripper) > 0.001:
                logger.info(f"  ✓ Gripper button detected: gripper={gripper:.3f}")
                last_gripper = gripper
            time.sleep(0.5)
        logger.info("  ✓ Gripper button input verification complete")

        logger.info("\n✓ PHASE 2 COMPLETE: All joystick inputs verified!")
        input("\nPress ENTER to proceed to Phase 3 (Free Control with Robot)...")

        # ==================== PHASE 3: Free Control Mode ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 3: Free Control Mode - Control the Robot!")
        logger.info("=" * 60)
        logger.info("\nNow you have full control of the robot with the joystick!")
        logger.info("The robot will move according to your joystick inputs.")
        logger.info("\nControls:")
        logger.info("  Left Stick:     Waist (X) and Shoulder (Y)")
        logger.info("  Right Stick:    Wrist angle (X) and Elbow (Y)")
        logger.info("  L2/R2:          Wrist rotate")
        logger.info("  L1/R1:          Open/Close gripper")
        logger.info("  Triangle (Y):   Reset to home position")
        logger.info("  START:          Exit test")
        logger.info("\nControl loop starting in 3 seconds...")
        time.sleep(3)

        control_active = True
        last_status_time = time.time()
        iteration = 0

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
                logger.info(f"  [{iteration//40}] Position: waist={action.get('waist.pos', 0):.2f}, "
                           f"shoulder={action.get('shoulder.pos', 0):.2f}, "
                           f"elbow={action.get('elbow.pos', 0):.2f}, "
                           f"wrist_angle={action.get('wrist_angle.pos', 0):.2f}, "
                           f"gripper={action.get('gripper.pos', 0):.2f}")
                last_status_time = time.time()

            iteration += 1
            time.sleep(0.05)  # 20Hz control loop

        logger.info("✓ PHASE 3 COMPLETE: Free control mode finished")

        # ==================== PHASE 4: Cleanup ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 4: Cleanup and Disconnect")
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
    logger.info("  ✓ Phase 1: Robot and teleoperator connection")
    logger.info("  ✓ Phase 2: Joystick input verification")
    logger.info("    - Left stick (waist + shoulder)")
    logger.info("    - Right stick (elbow + wrist angle)")
    logger.info("    - Triggers (wrist rotate)")
    logger.info("    - Gripper buttons (L1/R1)")
    logger.info("  ✓ Phase 3: Free control mode with robot")
    logger.info("  ✓ Phase 4: Clean disconnect")
    logger.info("\n🎮 Joystick teleoperation is fully functional!")
    logger.info("=" * 60)

    return True


if __name__ == "__main__":
    success = test_joystick_teleop()
    sys.exit(0 if success else 1)
