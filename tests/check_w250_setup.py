#!/usr/bin/env python3
"""
Pre-flight check for W250 testing
Verifies all requirements before running tests
"""

import os
import sys
import subprocess
import logging
from pathlib import Path

# Add lerobot to path if not installed
repo_root = Path(__file__).parent.parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


def check_item(description: str, check_func) -> bool:
    """Run a check and display result"""
    try:
        result, message = check_func()
        status = "✓" if result else "✗"
        logger.info(f"{status} {description}")
        if message:
            for line in message.split('\n'):
                logger.info(f"    {line}")
        return result
    except Exception as e:
        logger.info(f"✗ {description}")
        logger.info(f"    Error: {e}")
        return False


def check_ros2():
    """Check if ROS2 is installed and sourced"""
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        return True, f"ROS2 {ros_distro} detected"
    else:
        return False, "ROS_DISTRO not found\n    Run: source /opt/ros/<distro>/setup.bash"


def check_interbotix_module():
    """Check if interbotix_xs_modules is installed"""
    try:
        import interbotix_xs_modules
        return True, f"Installed at: {interbotix_xs_modules.__file__}"
    except ImportError:
        return False, "Not installed\n    Run: pip install interbotix_xs_modules"


def check_lerobot():
    """Check if lerobot is available"""
    try:
        import lerobot
        return True, f"Installed at: {lerobot.__file__}"
    except ImportError:
        return False, "Not installed\n    Check your lerobot installation"


def check_w250_module():
    """Check if W250 module can be imported"""
    try:
        from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
        return True, "W250 module loads successfully"
    except ImportError as e:
        return False, f"Import failed: {e}"


def check_control_node():
    """Check if control node is running"""
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=3
        )
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            wx250_nodes = [n for n in nodes if 'wx250' in n]
            if wx250_nodes:
                return True, f"Found node(s): {', '.join(wx250_nodes)}"
            else:
                return False, "Node not found\n    Run: ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250"
        else:
            return False, "Could not list ROS2 nodes"
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False, "ros2 command not available"


def check_usb_device():
    """Check if USB device exists"""
    usb_devices = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyDXL']
    found = [dev for dev in usb_devices if os.path.exists(dev)]

    if found:
        return True, f"Found: {', '.join(found)}"
    else:
        return False, f"No USB devices found in: {', '.join(usb_devices)}\n    Check if robot is connected and powered"


def check_permissions():
    """Check if user has permissions for USB devices"""
    try:
        result = subprocess.run(
            ['groups'],
            capture_output=True,
            text=True,
            timeout=1
        )
        groups = result.stdout.strip()
        if 'dialout' in groups:
            return True, "User in dialout group"
        else:
            return False, "User NOT in dialout group\n    Run: sudo usermod -aG dialout $USER\n    Then logout/login"
    except:
        return None, "Could not check groups"


def main():
    """Run all checks"""

    logger.info("=" * 70)
    logger.info("W250 ROBOT SETUP VERIFICATION")
    logger.info("=" * 70)
    logger.info("")

    checks = [
        ("ROS2 Environment", check_ros2),
        ("Python Package: interbotix_xs_modules", check_interbotix_module),
        ("Python Package: lerobot", check_lerobot),
        ("W250 Robot Module", check_w250_module),
        ("USB Device Connected", check_usb_device),
        ("User Permissions (dialout group)", check_permissions),
        ("ROS2 Control Node Running", check_control_node),
    ]

    results = []
    for description, check_func in checks:
        result = check_item(description, check_func)
        results.append(result)
        logger.info("")

    # Summary
    passed = sum(results)
    total = len(results)

    logger.info("=" * 70)
    logger.info(f"SUMMARY: {passed}/{total} checks passed")
    logger.info("=" * 70)
    logger.info("")

    if passed == total:
        logger.info("✓ All checks passed! Ready to run tests.")
        logger.info("")
        logger.info("Run tests with:")
        logger.info("  $ python test_w250_connection.py")
        logger.info("  $ python test_w250_joints.py")
        logger.info("  $ python test_w250_gripper.py")
        logger.info("  $ python test_w250_observations.py")
        logger.info("")
        logger.info("Or run all tests:")
        logger.info("  $ python run_all_w250_tests.py")
        return True
    else:
        logger.info("⚠️  Some checks failed. Please fix the issues above before running tests.")
        logger.info("")
        logger.info("For detailed troubleshooting, see:")
        logger.info("  README_W250_TESTS.md")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
