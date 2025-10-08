#!/usr/bin/env python3
"""
Test script for controlling W250 robot with Nintendo Switch Left JoyCon.

This script demonstrates teleoperation using differential control (deltas)
from the JoyCon mapped to end-effector movements.
"""

import time
import numpy as np
from lerobot.teleoperators.w250joystick import W250JoystickConfig, W250JoystickTeleop
from lerobot.robots.w250 import W250InterbotixConfig, W250Interbotix


class DifferentialController:
    """Converts differential actions (deltas) to absolute joint positions."""

    def __init__(self, robot):
        self.robot = robot
        self.current_gripper_state = 1.0  # Start open (normalized)

    def apply_delta_action(self, delta_action, gripper_action):
        """
        Apply delta movements to current robot state.

        Args:
            delta_action: Dict with delta_x, delta_y, delta_z (in meters)
            gripper_action: 0=close, 1=stay, 2=open

        Returns:
            Dict with absolute joint positions to send to robot
        """
        # Get current observation
        obs = self.robot.get_observation()

        # For now, we'll keep joint positions constant and only control gripper
        # In a full implementation, you'd use inverse kinematics to convert
        # delta_x, delta_y, delta_z to joint angle changes

        action = {}

        # Keep all joint positions at current values
        for joint_name in ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]:
            key = f"{joint_name}.pos"
            if key in obs:
                action[key] = obs[key]

        # Handle gripper state changes
        if gripper_action == 0:  # Close
            self.current_gripper_state = 0.0
        elif gripper_action == 2:  # Open
            self.current_gripper_state = 1.0
        # If gripper_action == 1 (stay), keep current state

        action["gripper.pos"] = self.current_gripper_state

        return action


def main():
    print("=== W250 + Left JoyCon Control Test ===\n")

    # Configure teleoperator
    teleop_config = W250JoystickConfig(
        x_step_size=0.01,
        y_step_size=0.01,
        z_step_size=0.01,
        deadzone=0.15
    )
    teleop = W250JoystickTeleop(teleop_config)

    # Configure robot
    robot_config = W250InterbotixConfig(
        robot_model="wx250s",
        disable_torque_on_disconnect=True
    )
    robot = W250Interbotix(robot_config)

    # Create differential controller
    controller = DifferentialController(robot)

    print("Connecting to JoyCon...")
    try:
        teleop.connect()
        print("✓ JoyCon connected\n")
    except Exception as e:
        print(f"✗ Failed to connect to JoyCon: {e}")
        print("Make sure:")
        print("  1. JoyCon is paired via Bluetooth")
        print("  2. ROS2 joy node is running: ros2 run joy joy_node")
        return

    print("Connecting to W250 robot...")
    try:
        robot.connect()
        print("✓ Robot connected\n")
    except Exception as e:
        print(f"✗ Failed to connect to robot: {e}")
        teleop.disconnect()
        return

    print("=== Controls ===")
    print("Analog stick: Move in X-Y plane (NOT IMPLEMENTED YET - only gripper works)")
    print("D-pad Up/Down: Move in Z axis (NOT IMPLEMENTED YET)")
    print("Minus (-): Open gripper")
    print("ZL: Close gripper")
    print("L: Enable intervention")
    print("Press Ctrl+C to stop\n")

    print("Starting control loop...\n")

    try:
        loop_count = 0
        last_gripper_action = 1  # Start with stay

        while True:
            # Get teleoperator action
            teleop_action = teleop.get_action()
            events = teleop.get_teleop_events()

            # Get gripper action
            gripper_action = teleop_action.get("gripper", 1)

            # Only print when something changes
            if loop_count % 10 == 0 or gripper_action != last_gripper_action:
                gripper_state = ["CLOSE", "STAY", "OPEN"][gripper_action]
                print(f"dx={teleop_action['delta_x']:+.3f} "
                      f"dy={teleop_action['delta_y']:+.3f} "
                      f"dz={teleop_action['delta_z']:+.3f} "
                      f"gripper={gripper_state}")

            # Convert to absolute positions and send to robot
            # Only send action if gripper state changed
            if gripper_action != last_gripper_action:
                robot_action = controller.apply_delta_action(teleop_action, gripper_action)
                robot.send_action(robot_action)
                print(f"  → Sent gripper command: {robot_action['gripper.pos']:.1f}")

            last_gripper_action = gripper_action
            loop_count += 1

            # Control at ~20Hz
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nDisconnecting...")
        try:
            teleop.disconnect()
            print("✓ JoyCon disconnected")
        except Exception as e:
            print(f"✗ Error disconnecting JoyCon: {e}")

        try:
            robot.disconnect()
            print("✓ Robot disconnected")
        except Exception as e:
            print(f"✗ Error disconnecting robot: {e}")

        print("\nDone!")


if __name__ == "__main__":
    main()
