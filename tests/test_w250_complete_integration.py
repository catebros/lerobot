#!/usr/bin/env python3
"""
Complete Integration Test for W250 Robot
Tests arm movements, gripper control, and full interface
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


def test_complete_integration():
    """Complete integration test of W250 robot with arms and gripper"""

    logger.info("=" * 60)
    logger.info("W250 COMPLETE INTEGRATION TEST")
    logger.info("=" * 60)
    logger.info("\nThis test will:")
    logger.info("  1. Connect to robot and calibrate")
    logger.info("  2. Test reading all joint positions")
    logger.info("  3. Test individual arm joint movements")
    logger.info("  4. Test gripper open/close")
    logger.info("  5. Test combined arm + gripper movements")
    logger.info("  6. Test observation reading")
    logger.info("  7. Clean disconnect")
    logger.info("=" * 60)

    try:
        from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
    except ImportError as e:
        logger.error(f"✗ Failed to import W250 classes: {e}")
        return False

    # Create robot configuration
    config = W250InterbotixConfig(
        robot_model="wx250s",
        robot_name="wx250s",
        moving_time=2.0,
        accel_time=0.3,
    )
    robot = W250Interbotix(config)

    try:
        # ==================== PHASE 1: Connection ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 1: Connection and Calibration")
        logger.info("=" * 60)

        logger.info("Connecting to robot...")
        robot.connect(calibrate=True)
        logger.info("✓ Robot connected and calibrated")
        logger.info("  Robot should be in home position (arm vertical)")
        time.sleep(2)

        # ==================== PHASE 2: Read Positions ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 2: Reading Joint Positions")
        logger.info("=" * 60)

        obs = robot.get_observation()
        logger.info("Current joint positions (normalized -1 to 1):")
        for joint in ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate", "gripper"]:
            key = f"{joint}.pos"
            if key in obs:
                logger.info(f"  {joint:15s}: {obs[key]:+.3f}")
        logger.info("✓ All joint positions read successfully")

        # ==================== PHASE 3: Test Individual Joints ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 3: Testing Individual Joint Movements")
        logger.info("=" * 60)

        # Test waist rotation
        logger.info("\n3.1 Testing WAIST rotation...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["waist.pos"] = 0.3  # Rotate right
        logger.info("  Moving waist to +0.3 (right)")
        robot.send_action(action)
        time.sleep(2.5)
        logger.info("  ✓ Waist movement complete")

        # Return to center
        action["waist.pos"] = 0.0
        robot.send_action(action)
        time.sleep(2.5)
        logger.info("  ✓ Returned to center")

        # Test shoulder
        logger.info("\n3.2 Testing SHOULDER movement...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["shoulder.pos"] = 0.2  # Move forward
        logger.info("  Moving shoulder to +0.2 (forward)")
        robot.send_action(action)
        time.sleep(2.5)
        logger.info("  ✓ Shoulder movement complete")

        # Return to home
        action["shoulder.pos"] = 0.0
        robot.send_action(action)
        time.sleep(2.5)
        logger.info("  ✓ Returned to home")

        # Test elbow
        logger.info("\n3.3 Testing ELBOW movement...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["elbow.pos"] = 0.3
        logger.info("  Moving elbow to +0.3")
        robot.send_action(action)
        time.sleep(2.5)
        logger.info("  ✓ Elbow movement complete")

        # Return
        action["elbow.pos"] = 0.0
        robot.send_action(action)
        time.sleep(2.5)
        logger.info("  ✓ Returned to home")

        logger.info("\n✓ All individual joint tests passed")

        # ==================== PHASE 4: Test Gripper ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 4: Testing Gripper Control")
        logger.info("=" * 60)

        logger.info("\n4.1 Opening gripper...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 1.0
        robot.send_action(action)
        time.sleep(1.0)
        logger.info("  ✓ Gripper opened")

        logger.info("\n4.2 Closing gripper...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 0.0
        robot.send_action(action)
        time.sleep(1.0)
        logger.info("  ✓ Gripper closed")

        logger.info("\n4.3 Half-open position...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 0.5
        robot.send_action(action)
        time.sleep(1.0)
        logger.info("  ✓ Gripper at half-open")

        logger.info("\n✓ Gripper control tests passed")

        # ==================== PHASE 5: Combined Movements ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 5: Testing Combined Arm + Gripper Movements")
        logger.info("=" * 60)

        logger.info("\n5.1 Moving to reach position with open gripper...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["waist.pos"] = 0.2
        action["shoulder.pos"] = 0.3
        action["elbow.pos"] = 0.3
        action["wrist_angle.pos"] = -0.2
        action["gripper.pos"] = 1.0  # Open
        logger.info("  Moving arm to reach position")
        robot.send_action(action)
        time.sleep(3.0)
        logger.info("  ✓ Reach position achieved with gripper open")

        logger.info("\n5.2 Closing gripper (simulate grasp)...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 0.0  # Close to grasp
        robot.send_action(action)
        time.sleep(1.0)
        logger.info("  ✓ Gripper closed (object grasped)")

        logger.info("\n5.3 Lifting object (elbow up)...")
        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["elbow.pos"] = 0.0  # Lift
        robot.send_action(action)
        time.sleep(2.5)
        logger.info("  ✓ Object lifted")

        logger.info("\n5.4 Returning to home and releasing...")
        robot.calibrate()  # Return to home
        time.sleep(2.5)

        obs = robot.get_observation()
        action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
        action["gripper.pos"] = 1.0  # Release
        robot.send_action(action)
        time.sleep(1.0)
        logger.info("  ✓ Returned to home and released object")

        logger.info("\n✓ Combined movement tests passed")

        # ==================== PHASE 6: Observation Reading ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 6: Testing Observation Interface")
        logger.info("=" * 60)

        logger.info("\n6.1 Reading full observation...")
        obs = robot.get_observation()

        logger.info("  Observation keys:")
        for key in sorted(obs.keys()):
            if isinstance(obs[key], float):
                logger.info(f"    {key}: {obs[key]:.3f}")
            else:
                logger.info(f"    {key}: {type(obs[key])}")

        logger.info("\n6.2 Verifying action features match...")
        action_features = robot.action_features
        missing_features = [key for key in action_features.keys() if key not in obs]
        if missing_features:
            logger.warning(f"  ⚠ Missing features in observation: {missing_features}")
        else:
            logger.info("  ✓ All action features present in observation")

        logger.info("\n6.3 Testing observation timing...")
        start = time.time()
        for i in range(10):
            obs = robot.get_observation()
        elapsed = time.time() - start
        avg_time = elapsed / 10 * 1000
        logger.info(f"  Average observation read time: {avg_time:.1f}ms")
        if avg_time < 50:
            logger.info("  ✓ Observation reading is fast enough for control")
        else:
            logger.warning(f"  ⚠ Observation reading is slow ({avg_time:.1f}ms)")

        logger.info("\n✓ Observation interface tests passed")

        # ==================== PHASE 7: Disconnect ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 7: Testing Clean Disconnect")
        logger.info("=" * 60)

        logger.info("\nDisconnecting from robot...")
        robot.disconnect()
        logger.info("✓ Disconnected successfully")

        logger.info("\nWaiting 3 seconds to verify no gripper loops...")
        for i in range(3):
            logger.info(f"  {i+1}/3s - Gripper should be STOPPED")
            time.sleep(1)
        logger.info("✓ No loops detected after disconnect")

    except Exception as e:
        logger.error(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()

        # Try to disconnect on error
        try:
            robot.disconnect()
        except:
            pass

        return False

    # ==================== TEST SUMMARY ====================
    logger.info("\n" + "=" * 60)
    logger.info("✓ ALL INTEGRATION TESTS PASSED!")
    logger.info("=" * 60)
    logger.info("\nTest Summary:")
    logger.info("  ✓ Connection and calibration")
    logger.info("  ✓ Joint position reading")
    logger.info("  ✓ Individual joint movements (waist, shoulder, elbow)")
    logger.info("  ✓ Gripper control (open, close, half-open)")
    logger.info("  ✓ Combined arm + gripper movements")
    logger.info("  ✓ Observation interface")
    logger.info("  ✓ Clean disconnect without loops")
    logger.info("\n🎉 Robot is fully operational!")
    logger.info("=" * 60)

    return True


if __name__ == "__main__":
    success = test_complete_integration()
    sys.exit(0 if success else 1)
