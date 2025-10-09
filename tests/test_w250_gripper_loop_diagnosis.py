#!/usr/bin/env python3
"""
Test script: Gripper Loop Diagnosis
Diagnoses the infinite gripper open/close loop issue by monitoring gripper commands
at different stages of operation.
"""

import logging
import sys
import time
from pathlib import Path
import threading

# Add lerobot to path if not installed
repo_root = Path(__file__).parent.parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))

# Setup logging with detailed output
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s.%(msecs)03d - %(name)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


class GripperMonitor:
    """Monitor gripper commands to detect loops"""

    def __init__(self):
        self.commands = []
        self.monitoring = False
        self._lock = threading.Lock()

    def start(self):
        """Start monitoring"""
        with self._lock:
            self.commands = []
            self.monitoring = True
        logger.info("🔍 Gripper monitoring STARTED")

    def stop(self):
        """Stop monitoring"""
        with self._lock:
            self.monitoring = False
        logger.info("🛑 Gripper monitoring STOPPED")

    def log_command(self, cmd_type: str, value: float = None):
        """Log a gripper command"""
        with self._lock:
            if self.monitoring:
                timestamp = time.time()
                entry = {
                    'timestamp': timestamp,
                    'cmd_type': cmd_type,
                    'value': value
                }
                self.commands.append(entry)
                logger.debug(f"📊 Gripper command logged: {cmd_type} value={value}")

    def analyze(self):
        """Analyze commands for loop patterns"""
        with self._lock:
            if len(self.commands) == 0:
                logger.info("No gripper commands recorded")
                return

            logger.info(f"\n{'='*60}")
            logger.info(f"GRIPPER COMMAND ANALYSIS")
            logger.info(f"{'='*60}")
            logger.info(f"Total commands: {len(self.commands)}")

            # Show all commands with timestamps
            logger.info("\nCommand timeline:")
            start_time = self.commands[0]['timestamp']
            for i, cmd in enumerate(self.commands):
                elapsed = cmd['timestamp'] - start_time
                logger.info(f"  [{i+1:3d}] t={elapsed:6.3f}s  {cmd['cmd_type']:20s}  value={cmd['value']}")

            # Detect patterns
            logger.info("\nPattern detection:")

            # Count command types
            cmd_counts = {}
            for cmd in self.commands:
                cmd_type = cmd['cmd_type']
                cmd_counts[cmd_type] = cmd_counts.get(cmd_type, 0) + 1

            for cmd_type, count in cmd_counts.items():
                logger.info(f"  - {cmd_type}: {count} times")

            # Detect rapid alternations (potential loop)
            if len(self.commands) >= 2:
                alternations = 0
                for i in range(1, len(self.commands)):
                    if self.commands[i]['cmd_type'] != self.commands[i-1]['cmd_type']:
                        alternations += 1

                logger.info(f"\nAlternations between commands: {alternations}")
                if alternations > len(self.commands) * 0.5:
                    logger.error("⚠️  HIGH ALTERNATION RATE - POSSIBLE LOOP DETECTED!")

            # Check for commands after test completion
            logger.info(f"\n{'='*60}\n")


# Monkey patch to intercept gripper commands
monitor = GripperMonitor()

def patch_robot_for_monitoring(robot):
    """Patch robot methods to monitor gripper commands"""

    # Patch gripper methods if they exist
    if hasattr(robot, 'bot') and robot.bot and hasattr(robot.bot, 'gripper'):
        gripper = robot.bot.gripper

        # Patch set_pressure
        if hasattr(gripper, 'set_pressure'):
            original_set_pressure = gripper.set_pressure
            def monitored_set_pressure(pressure, *args, **kwargs):
                monitor.log_command('set_pressure', pressure)
                return original_set_pressure(pressure, *args, **kwargs)
            gripper.set_pressure = monitored_set_pressure
            logger.info("✓ Patched gripper.set_pressure()")

        # Patch grasp
        if hasattr(gripper, 'grasp'):
            original_grasp = gripper.grasp
            def monitored_grasp(*args, **kwargs):
                monitor.log_command('grasp')
                return original_grasp(*args, **kwargs)
            gripper.grasp = monitored_grasp
            logger.info("✓ Patched gripper.grasp()")

        # Patch release
        if hasattr(gripper, 'release'):
            original_release = gripper.release
            def monitored_release(*args, **kwargs):
                monitor.log_command('release')
                return original_release(*args, **kwargs)
            gripper.release = monitored_release
            logger.info("✓ Patched gripper.release()")

    # Patch send_action to monitor action commands
    original_send_action = robot.send_action
    def monitored_send_action(action):
        if 'gripper.pos' in action:
            monitor.log_command('send_action[gripper.pos]', action['gripper.pos'])
        return original_send_action(action)
    robot.send_action = monitored_send_action
    logger.info("✓ Patched robot.send_action()")


def test_gripper_loop_diagnosis():
    """Diagnose the gripper loop issue"""

    logger.info("=" * 60)
    logger.info("TEST: Gripper Loop Diagnosis")
    logger.info("=" * 60)

    try:
        from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
    except ImportError as e:
        logger.error(f"✗ Failed to import W250 classes: {e}")
        return False

    # Create robot
    config = W250InterbotixConfig(
        robot_model="wx250",
        robot_name="wx250",
        moving_time=2.0,
        accel_time=0.3,
    )
    robot = W250Interbotix(config)

    try:
        # Connect
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 1: Connection")
        logger.info("=" * 60)

        monitor.start()
        robot.connect(calibrate=True)
        logger.info("✓ Robot connected")
        time.sleep(2)

        logger.info("\nGripper commands during connection:")
        monitor.analyze()

    except Exception as e:
        logger.error(f"✗ Connection failed: {e}")
        monitor.stop()
        return False

    # Patch robot to monitor commands
    patch_robot_for_monitoring(robot)

    try:
        # Test Phase 2: Single open command
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 2: Single OPEN command")
        logger.info("=" * 60)

        monitor.start()

        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 1.0

        logger.info("Sending OPEN command...")
        robot.send_action(action)

        logger.info("Waiting 2 seconds...")
        time.sleep(2)

        logger.info("\nGripper commands during OPEN:")
        monitor.analyze()

    except Exception as e:
        logger.error(f"✗ Open test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Test Phase 3: Single close command
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 3: Single CLOSE command")
        logger.info("=" * 60)

        monitor.start()

        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 0.0

        logger.info("Sending CLOSE command...")
        robot.send_action(action)

        logger.info("Waiting 2 seconds...")
        time.sleep(2)

        logger.info("\nGripper commands during CLOSE:")
        monitor.analyze()

    except Exception as e:
        logger.error(f"✗ Close test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Test Phase 4: After script completion
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 4: Disconnection")
        logger.info("=" * 60)

        monitor.start()

        logger.info("Disconnecting...")
        robot.disconnect()
        logger.info("✓ Disconnect called")

        logger.info("Waiting 5 seconds to check for post-disconnect commands...")
        time.sleep(5)

        logger.info("\nGripper commands during/after disconnect:")
        monitor.analyze()

    except Exception as e:
        logger.error(f"✗ Disconnect failed: {e}")
        return False

    logger.info("\n" + "=" * 60)
    logger.info("DIAGNOSIS COMPLETE")
    logger.info("=" * 60)
    logger.info("\nCheck the command analysis above to identify:")
    logger.info("  1. Which phase generates the most commands")
    logger.info("  2. Whether commands continue after script ends")
    logger.info("  3. If there's a pattern of alternating open/close")
    logger.info("  4. The specific function causing the loop")
    logger.info("=" * 60)

    return True


if __name__ == "__main__":
    try:
        success = test_gripper_loop_diagnosis()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        logger.warning("\n\n⚠️  Test interrupted by user")
        logger.info("Analyzing commands received before interruption:")
        monitor.stop()
        monitor.analyze()
        sys.exit(1)
