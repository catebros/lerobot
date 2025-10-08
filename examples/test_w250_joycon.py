#!/usr/bin/env python3
"""
Test script for controlling W250 robot with Nintendo Switch Left JoyCon.

This script uses hybrid control:
- Cartesian control (X, Z) via Interbotix IK
- Joint control (Waist) for lateral movement
- Gripper control via pressure commands
"""

import logging
import time
import numpy as np
from lerobot.teleoperators.w250joystick import W250JoystickConfig, W250JoystickTeleop
from lerobot.robots.w250 import W250InterbotixConfig, W250Interbotix

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class HybridCartesianController:
    """
    Hybrid controller combining:
    - Cartesian control for X and Z movements (using Interbotix IK)
    - Direct joint control for Waist (Y lateral approximation)
    - Gripper pressure control
    """

    def __init__(self, robot, config=None):
        self.robot = robot
        self.current_gripper_state = 1.0  # Start open (normalized)

        # Scaling factors for smoother control
        if config is None:
            config = {}
        self.x_scale = config.get('x_scale', 1.0)  # meters per delta
        self.y_scale = config.get('y_scale', 0.15)  # radians per delta for waist
        self.z_scale = config.get('z_scale', 1.0)  # meters per delta

        # Cartesian trajectory parameters
        self.moving_time = config.get('moving_time', 0.15)
        self.wp_moving_time = config.get('wp_moving_time', 0.08)
        self.wp_accel_time = config.get('wp_accel_time', 0.04)
        self.wp_period = config.get('wp_period', 0.02)

        # Minimum delta to consider (dead zone)
        self.min_delta = 0.001

        logger.info("HybridCartesianController initialized")
        logger.info(f"  X scale: {self.x_scale}, Y scale (waist): {self.y_scale}, Z scale: {self.z_scale}")

    def apply_delta_action(self, delta_action, gripper_action):
        """
        Apply delta movements using hybrid control strategy.

        Args:
            delta_action: Dict with delta_x, delta_y, delta_z (in meters or normalized)
            gripper_action: 0=close, 1=stay, 2=open

        Returns:
            Dict with status information
        """
        result = {
            'cartesian_success': None,
            'waist_moved': False,
            'gripper_changed': False
        }

        # 1. Cartesian Control for X and Z movements
        delta_x = delta_action['delta_x'] * self.x_scale
        delta_z = delta_action['delta_z'] * self.z_scale

        if abs(delta_x) > self.min_delta or abs(delta_z) > self.min_delta:
            try:
                # Use Interbotix's set_ee_cartesian_trajectory for IK
                # Note: y must be 0 for 5 DOF arms like WX250
                success = self.robot.bot.arm.set_ee_cartesian_trajectory(
                    x=float(delta_x),
                    y=0.0,  # Must be 0 for 5 DOF
                    z=float(delta_z),
                    roll=0.0,
                    pitch=0.0,
                    yaw=0.0,
                    moving_time=self.moving_time,
                    wp_moving_time=self.wp_moving_time,
                    wp_accel_time=self.wp_accel_time,
                    wp_period=self.wp_period
                )
                result['cartesian_success'] = success
                if not success:
                    logger.warning("Cartesian trajectory failed (IK may be infeasible)")
            except Exception as e:
                logger.error(f"Cartesian control error: {e}")
                result['cartesian_success'] = False

        # 2. Joint Control for Waist (lateral movement approximation)
        delta_y = delta_action['delta_y']
        if abs(delta_y) > self.min_delta:
            try:
                # Get current waist position (normalized)
                obs = self.robot.get_observation()
                current_waist_norm = obs.get('waist.pos', 0.0)

                # Calculate delta in joint space (radians)
                # delta_y is from joystick, convert to waist rotation
                delta_waist = delta_y * self.y_scale
                new_waist_norm = current_waist_norm + delta_waist

                # Apply limits (normalized -1 to 1, maps to -pi to pi)
                new_waist_norm = np.clip(new_waist_norm, -1.0, 1.0)

                # Convert to radians for Interbotix
                new_waist_rad = self.robot._denormalize_position('waist', new_waist_norm)

                # Get all current joint positions
                current_positions = self.robot.bot.arm.get_joint_commands()

                # Update only waist (index 0)
                new_positions = list(current_positions)
                new_positions[0] = float(new_waist_rad)

                # Send command
                self.robot.bot.arm.set_joint_positions(
                    new_positions,
                    blocking=False
                )
                result['waist_moved'] = True

            except Exception as e:
                logger.error(f"Waist control error: {e}")

        # 3. Gripper Control
        old_gripper_state = self.current_gripper_state

        if gripper_action == 0:  # Close
            self.current_gripper_state = 0.0
        elif gripper_action == 2:  # Open
            self.current_gripper_state = 1.0
        # If gripper_action == 1 (stay), keep current state

        # Only send gripper command if state changed
        if self.current_gripper_state != old_gripper_state:
            try:
                gripper_action_dict = {"gripper.pos": self.current_gripper_state}
                self.robot.send_action(gripper_action_dict)
                result['gripper_changed'] = True
            except Exception as e:
                logger.error(f"Gripper control error: {e}")

        return result


def main():
    print("=== W250 + Left JoyCon Hybrid Control ===\n")
    print("Control Strategy:")
    print("  - Cartesian IK for X/Z movements")
    print("  - Direct waist control for lateral movement")
    print("  - Pressure-based gripper control\n")

    # Configure teleoperator
    teleop_config = W250JoystickConfig(
        x_step_size=0.01,  # 1cm per full stick deflection
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

    # Configure hybrid controller
    controller_config = {
        'x_scale': 1.0,      # Full scale for forward/backward
        'y_scale': 0.15,     # Scaled down for waist rotation
        'z_scale': 1.0,      # Full scale for up/down
        'moving_time': 0.15,
        'wp_moving_time': 0.08,
        'wp_accel_time': 0.04,
        'wp_period': 0.02
    }
    controller = HybridCartesianController(robot, controller_config)

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
        # Connect without calibrating to avoid automatic movements
        robot.connect(calibrate=False)
        print("✓ Robot connected (without auto-calibration)\n")
    except Exception as e:
        print(f"✗ Failed to connect to robot: {e}")
        teleop.disconnect()
        return

    print("=== Controls ===")
    print("Analog stick Y (↑↓): Move forward/backward (X axis)")
    print("Analog stick X (←→): Rotate base for lateral movement (Waist)")
    print("D-pad Up/Down: Move up/down (Z axis)")
    print("Minus (-): Open gripper")
    print("ZL: Close gripper")
    print("L: Enable intervention")
    print("Press Ctrl+C to stop\n")

    print("Starting control loop at 20Hz...\n")

    try:
        loop_count = 0
        last_gripper_action = 1  # Start with stay
        last_send_time = time.time()
        control_rate = 20  # Hz

        # Statistics
        stats = {
            'cartesian_commands': 0,
            'waist_commands': 0,
            'gripper_commands': 0,
            'ik_failures': 0
        }

        while True:
            current_time = time.time()

            # Rate limiting
            dt = current_time - last_send_time
            if dt < 1.0 / control_rate:
                time.sleep(0.001)
                continue

            last_send_time = current_time

            # Get teleoperator action
            teleop_action = teleop.get_action()
            events = teleop.get_teleop_events()

            # Get gripper action
            gripper_action = teleop_action.get("gripper", 1)

            # Check if there's any movement command
            has_movement = (
                abs(teleop_action['delta_x']) > 0.001 or
                abs(teleop_action['delta_y']) > 0.001 or
                abs(teleop_action['delta_z']) > 0.001
            )

            # Apply control (always, to handle continuous movements)
            if has_movement or gripper_action != last_gripper_action:
                result = controller.apply_delta_action(teleop_action, gripper_action)

                # Update statistics
                if result['cartesian_success'] is True:
                    stats['cartesian_commands'] += 1
                elif result['cartesian_success'] is False:
                    stats['ik_failures'] += 1

                if result['waist_moved']:
                    stats['waist_commands'] += 1

                if result['gripper_changed']:
                    stats['gripper_commands'] += 1

            # Display status
            if loop_count % 20 == 0 or gripper_action != last_gripper_action or has_movement:
                gripper_state = ["CLOSE", "STAY", "OPEN"][gripper_action]
                print(f"[{loop_count:4d}] dx={teleop_action['delta_x']:+.3f} "
                      f"dy={teleop_action['delta_y']:+.3f} "
                      f"dz={teleop_action['delta_z']:+.3f} "
                      f"grip={gripper_state} "
                      f"| Cart:{stats['cartesian_commands']} Waist:{stats['waist_commands']} "
                      f"Grip:{stats['gripper_commands']} IK_fail:{stats['ik_failures']}")

            last_gripper_action = gripper_action
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
        print(f"  Cartesian commands sent: {stats['cartesian_commands']}")
        print(f"  Waist commands sent: {stats['waist_commands']}")
        print(f"  Gripper commands sent: {stats['gripper_commands']}")
        print(f"  IK failures: {stats['ik_failures']}")

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
