#!/usr/bin/env python3
"""
Debug script - ONLY reads JoyCon, does NOT send commands to robot.
Use this to verify JoyCon is working correctly.
"""

import time
from lerobot.teleoperators.w250joystick import W250JoystickConfig, W250JoystickTeleop

def main():
    print("=== JoyCon Debug (NO ROBOT CONTROL) ===\n")

    # Configure teleoperator
    teleop_config = W250JoystickConfig(
        x_step_size=0.01,
        y_step_size=0.01,
        z_step_size=0.01,
        deadzone=0.15
    )
    teleop = W250JoystickTeleop(teleop_config)

    print("Connecting to JoyCon...")
    try:
        teleop.connect()
        print("✓ JoyCon connected\n")
    except Exception as e:
        print(f"✗ Failed to connect to JoyCon: {e}")
        return

    print("=== Monitoring JoyCon Input ===")
    print("Move the stick and press buttons to see output")
    print("Press Ctrl+C to stop\n")

    try:
        last_gripper = 1
        count = 0

        while True:
            # Get teleoperator action
            action = teleop.get_action()
            events = teleop.get_teleop_events()

            gripper = action.get("gripper", 1)

            # Print when values change or every 20 iterations
            if (count % 20 == 0 or
                gripper != last_gripper or
                abs(action['delta_x']) > 0.001 or
                abs(action['delta_y']) > 0.001 or
                abs(action['delta_z']) > 0.001):

                gripper_state = ["CLOSE", "STAY", "OPEN"][gripper]
                print(f"[{count:4d}] dx={action['delta_x']:+.3f} "
                      f"dy={action['delta_y']:+.3f} "
                      f"dz={action['delta_z']:+.3f} "
                      f"gripper={gripper_state} ({gripper})")

                if gripper != last_gripper:
                    print(f"       ^^^ GRIPPER CHANGED: {last_gripper} -> {gripper}")

            last_gripper = gripper
            count += 1
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        teleop.disconnect()
        print("Disconnected")


if __name__ == "__main__":
    main()
