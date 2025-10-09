#!/usr/bin/env python3
"""
Show Joystick Inputs in Real-Time
Displays which buttons and axes are being activated on the Logitech F710 gamepad
"""

import logging
import sys
import time
from pathlib import Path

# Add lerobot to path if not installed
repo_root = Path(__file__).parent.parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
except ImportError:
    print("ERROR: ROS2 (rclpy) is required!")
    print("Please install ROS2 and source your workspace:")
    print("https://docs.ros.org/en/rolling/Installation.html")
    sys.exit(1)

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(message)s'
)
logger = logging.getLogger(__name__)


class JoystickMonitor(Node):
    """Monitor and display joystick inputs in real-time."""

    def __init__(self):
        super().__init__('joystick_monitor')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.last_axes = []
        self.last_buttons = []
        self.first_message = True

        logger.info("=" * 70)
        logger.info("JOYSTICK INPUT MONITOR")
        logger.info("=" * 70)
        logger.info("Listening for joystick inputs on /joy topic...")
        logger.info("Make sure to run: ros2 run joy joy_node")
        logger.info("Press Ctrl+C to exit")
        logger.info("=" * 70)
        logger.info("")

    def joy_callback(self, msg: Joy):
        """Callback to display joystick state changes."""

        if self.first_message:
            logger.info(f"✓ Joystick connected!")
            logger.info(f"  Axes count: {len(msg.axes)}")
            logger.info(f"  Buttons count: {len(msg.buttons)}")
            logger.info("")
            self.first_message = False

        # Check for axis changes (with deadzone)
        for i, value in enumerate(msg.axes):
            if abs(value) > 0.1:  # Deadzone
                if i >= len(self.last_axes) or abs(value - self.last_axes[i]) > 0.05:
                    axis_name = self._get_axis_name(i)
                    logger.info(f"🕹️  AXIS {i:2d} ({axis_name:20s}): {value:+.3f}")

        # Check for button presses
        for i, value in enumerate(msg.buttons):
            if value == 1:  # Button pressed
                if i >= len(self.last_buttons) or self.last_buttons[i] != value:
                    button_name = self._get_button_name(i)
                    logger.info(f"🔘 BUTTON {i:2d} ({button_name:20s}): PRESSED")

        # Update last state
        self.last_axes = list(msg.axes)
        self.last_buttons = list(msg.buttons)

    def _get_axis_name(self, axis_id: int) -> str:
        """Get human-readable name for axis (Logitech F710 XInput mode)."""
        axis_names = {
            0: "Left Stick X",
            1: "Left Stick Y",
            2: "LT (Left Trigger)",
            3: "Right Stick X",
            4: "Right Stick Y",
            5: "RT (Right Trigger)",
            6: "D-Pad X",
            7: "D-Pad Y",
        }
        return axis_names.get(axis_id, "Unknown")

    def _get_button_name(self, button_id: int) -> str:
        """Get human-readable name for button (Logitech F710 XInput mode)."""
        button_names = {
            0: "A",
            1: "B",
            2: "X",
            3: "Y",
            4: "LB",
            5: "RB",
            6: "Back",
            7: "Start",
            8: "Guide/Logitech",
            9: "L3 (Left Stick)",
            10: "R3 (Right Stick)",
        }
        return button_names.get(button_id, "Unknown")


def main():
    """Main function to run the joystick monitor."""

    # Initialize ROS2
    rclpy.init()

    # Create monitor node
    monitor = JoystickMonitor()

    try:
        # Spin the node
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        logger.info("\n")
        logger.info("=" * 70)
        logger.info("Monitor stopped by user (Ctrl+C)")
        logger.info("=" * 70)
    finally:
        # Cleanup
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
