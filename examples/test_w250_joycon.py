#!/usr/bin/env python3
"""
Test script for controlling W250 robot with Logitech F710 gamepad.

Uses intuitive Interbotix-style control with incremental positions.
"""

import logging
import time
from lerobot.teleoperators.w250joystick import W250JoystickConfig, W250JoystickTeleop
from lerobot.robots.w250 import W250InterbotixConfig, W250Interbotix

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    print("=== W250 + Logitech F710 Gamepad Intuitive Control ===\n")
    print("Control Strategy:")
    print("  - Incremental joint position control (like Interbotix)")
    print("  - Full 5-DOF control with dual analog sticks")
    print("  - Emergency stop and home position\n")

    # Configure teleoperator
    teleop_config = W250JoystickConfig(
        x_step_size=0.015,      # Shoulder increment
        y_step_size=0.015,      # Waist increment
        z_step_size=0.015,      # Elbow increment
        wrist_step_size=0.01,   # Wrist angle/rotate increment
        deadzone=0.15
    )
    teleop = W250JoystickTeleop(teleop_config)

    # Configure robot
    robot_config = W250InterbotixConfig(
        robot_model="wx250s",
        disable_torque_on_disconnect=True,
        moving_time=0.5  # Faster for teleoperation
    )
    robot = W250Interbotix(robot_config)

    print("Connecting to F710 gamepad...")
    try:
        teleop.connect()
        print("✓ F710 gamepad connected\n")
    except Exception as e:
        print(f"✗ Failed to connect to F710 gamepad: {e}")
        print("Make sure:")
        print("  1. F710 is connected via USB or wireless dongle")
        print("  2. Mode switch on back is set to 'X' (XInput mode)")
        print("  3. ROS2 joy node is running: ros2 run joy joy_node")
        return

    print("Connecting to W250 robot...")
    try:
        # Connect without calibrating to avoid automatic movements
        robot.connect(calibrate=False)
        print("✓ Robot connected (without auto-calibration)\n")
    except Exception as e:
        print(f"✗ Failed to connect to robot: {e}")
        teleop.disconnect()
        return

    print("=== Logitech F710 Gamepad Controls ===")
    print("Left Stick X (←→): Waist rotation")
    print("Left Stick Y (↑↓): Shoulder (arm up/down)")
    print("Right Stick X (←→): Wrist angle")
    print("Right Stick Y (↑↓): Elbow (extend/contract)")
    print("LT/RT (Triggers): Wrist rotate")
    print("LB: Open gripper (hold)")
    print("RB: Close gripper (hold)")
    print("Back: Toggle emergency stop")
    print("Y: Reset to home position")
    print("Press Ctrl+C to stop\n")

    print("Starting control loop at 20Hz...\n")

    try:
        loop_count = 0
        last_send_time = time.time()
        control_rate = 20  # Hz

        # Statistics
        stats = {
            'commands_sent': 0,
            'homes': 0,
            'emergency_stops': 0
        }

        last_intervention = True

        while True:
            current_time = time.time()

            # Rate limiting
            dt = current_time - last_send_time
            if dt < 1.0 / control_rate:
                time.sleep(0.001)
                continue

            last_send_time = current_time

            # Get teleoperator action (absolute joint positions)
            try:
                action = teleop.get_action()
                events = teleop.get_teleop_events()
            except Exception as e:
                logger.error(f"Error getting action: {e}")
                break

            # Check intervention status
            intervention = events.get('is_intervention', True)
            if intervention != last_intervention:
                if intervention:
                    print("✓ Intervention ENABLED - robot will move")
                else:
                    print("⚠ Intervention DISABLED - emergency stop active")
                    stats['emergency_stops'] += 1
                last_intervention = intervention

            # Only send commands if intervention is active
            if intervention:
                try:
                    # Send action to robot
                    robot.send_action(action)
                    stats['commands_sent'] += 1

                    # Display status every 20 iterations
                    if loop_count % 20 == 0:
                        print(f"[{loop_count:4d}] "
                              f"Waist:{action.get('waist.pos', 0):+.2f} "
                              f"Shoulder:{action.get('shoulder.pos', 0):+.2f} "
                              f"Elbow:{action.get('elbow.pos', 0):+.2f} "
                              f"Wrist:{action.get('wrist_angle.pos', 0):+.2f} "
                              f"Grip:{action.get('gripper.pos', 0):.2f} "
                              f"| Cmds:{stats['commands_sent']}")

                except Exception as e:
                    logger.error(f"Error sending action: {e}")
                    break

            loop_count += 1

    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nDisconnecting...")
        print(f"\nSession Statistics:")
        print(f"  Commands sent: {stats['commands_sent']}")
        print(f"  Emergency stops: {stats['emergency_stops']}")

        try:
            teleop.disconnect()
            print("✓ F710 gamepad disconnected")
        except Exception as e:
            print(f"✗ Error disconnecting F710 gamepad: {e}")

        try:
            robot.disconnect()
            print("✓ Robot disconnected")
        except Exception as e:
            print(f"✗ Error disconnecting robot: {e}")

        print("\nDone!")


if __name__ == "__main__":
    main()
