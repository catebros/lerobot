#!/usr/bin/env python3
"""
Master test runner for W250 robot
Runs all tests in sequence with proper error handling
"""

import logging
import sys
import subprocess
import time
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def run_test(test_name: str, test_file: str) -> bool:
    """Run a single test and return success status"""

    logger.info("\n" + "=" * 70)
    logger.info(f"RUNNING: {test_name}")
    logger.info("=" * 70)

    try:
        result = subprocess.run(
            [sys.executable, test_file],
            capture_output=False,
            text=True,
            timeout=120  # 2 minute timeout per test
        )

        if result.returncode == 0:
            logger.info(f"✓ {test_name} PASSED")
            return True
        else:
            logger.error(f"✗ {test_name} FAILED (exit code: {result.returncode})")
            return False

    except subprocess.TimeoutExpired:
        logger.error(f"✗ {test_name} TIMED OUT (>120s)")
        return False
    except Exception as e:
        logger.error(f"✗ {test_name} ERROR: {e}")
        return False


def main():
    """Run all W250 tests in sequence"""

    logger.info("=" * 70)
    logger.info("W250 ROBOT TEST SUITE")
    logger.info("=" * 70)

    # Get test directory
    test_dir = Path(__file__).parent

    # Check prerequisites
    logger.info("\nChecking prerequisites...")

    # Check if running on Windows
    if sys.platform == "win32":
        logger.error("✗ These tests are designed for Linux/ROS2 environment")
        logger.error("  Please run on a system with ROS2 installed")
        return False

    # Check ROS2 environment
    import os
    if 'ROS_DISTRO' not in os.environ:
        logger.warning("⚠ ROS_DISTRO not found in environment")
        logger.warning("  Make sure to source ROS2:")
        logger.warning("    $ source /opt/ros/<distro>/setup.bash")
        response = input("\nContinue anyway? (y/n): ")
        if response.lower() != 'y':
            return False

    logger.info("✓ Prerequisites check completed\n")

    # Prompt user to start control node
    logger.info("=" * 70)
    logger.info("IMPORTANT: Before running tests, make sure the control node is running:")
    logger.info("  $ ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250")
    logger.info("=" * 70)
    response = input("\nIs the control node running? (y/n): ")

    if response.lower() != 'y':
        logger.info("Please start the control node first, then re-run this script")
        return False

    # Define tests in execution order
    tests = [
        ("Test 1: Connection", "test_w250_connection.py"),
        ("Test 2: Joint Movements", "test_w250_joints.py"),
        ("Test 3: Gripper", "test_w250_gripper.py"),
        ("Test 4: Observations", "test_w250_observations.py"),
    ]

    results = {}
    start_time = time.time()

    # Run each test
    for test_name, test_file in tests:
        test_path = test_dir / test_file

        if not test_path.exists():
            logger.error(f"✗ Test file not found: {test_file}")
            results[test_name] = False
            continue

        # Pause between tests
        if results:  # Not the first test
            logger.info("\nWaiting 3 seconds before next test...")
            time.sleep(3)

        # Run test
        success = run_test(test_name, str(test_path))
        results[test_name] = success

        # If test failed, ask user if they want to continue
        if not success:
            response = input("\nTest failed. Continue with remaining tests? (y/n): ")
            if response.lower() != 'y':
                logger.info("Test suite aborted by user")
                break

    # Print summary
    total_time = time.time() - start_time

    logger.info("\n" + "=" * 70)
    logger.info("TEST SUITE SUMMARY")
    logger.info("=" * 70)

    passed = sum(1 for v in results.values() if v)
    total = len(results)

    for test_name, success in results.items():
        status = "✓ PASSED" if success else "✗ FAILED"
        logger.info(f"{status:10s} - {test_name}")

    logger.info("\n" + "-" * 70)
    logger.info(f"Results: {passed}/{total} tests passed")
    logger.info(f"Time: {total_time:.1f}s")
    logger.info("=" * 70)

    if passed == total:
        logger.info("\n🎉 ALL TESTS PASSED! 🎉")
        logger.info("\nYour W250 ROS2 implementation is working correctly!")
        logger.info("Next steps:")
        logger.info("  1. Implement teleoperator")
        logger.info("  2. Record datasets")
        logger.info("  3. Train policies")
        return True
    else:
        logger.info(f"\n⚠️  {total - passed} test(s) failed")
        logger.info("\nPlease check:")
        logger.info("  - Control node is running")
        logger.info("  - Robot is connected and powered")
        logger.info("  - ROS2 environment is properly sourced")
        logger.info("  - Refer to README_W250_TESTS.md for troubleshooting")
        return False


if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        logger.info("\n\nTest suite interrupted by user")
        sys.exit(1)
    except Exception as e:
        logger.error(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
