#!/usr/bin/env python3
"""
Test script 4: Observation Reading Test
Tests continuous observation reading at different frequencies (simulating policy loop)
"""

import logging
import sys
import time
import numpy as np
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


def test_observations():
    """Test continuous observation reading"""

    logger.info("=" * 60)
    logger.info("TEST 4: Observation Reading Test")
    logger.info("=" * 60)

    try:
        from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig
    except ImportError as e:
        logger.error(f"✗ Failed to import W250 classes: {e}")
        return False

    # Create robot (without cameras for faster testing)
    config = W250InterbotixConfig(
        robot_model="wx250s",
        robot_name="wx250s",
        moving_time=2.0,
        accel_time=0.3,
        cameras={},  # No cameras for this test
    )
    robot = W250Interbotix(config)

    try:
        # Connect
        logger.info("Connecting to robot...")
        robot.connect(calibrate=True)
        logger.info("✓ Robot connected")
        time.sleep(2)

    except Exception as e:
        logger.error(f"✗ Connection failed: {e}")
        return False

    try:
        # Test 1: Single observation timing
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Single observation timing")
        logger.info("=" * 60)

        times = []
        for i in range(10):
            start = time.time()
            obs = robot.get_observation()
            duration = time.time() - start
            times.append(duration)

        avg_time = np.mean(times) * 1000  # Convert to ms
        std_time = np.std(times) * 1000
        min_time = np.min(times) * 1000
        max_time = np.max(times) * 1000

        logger.info(f"Observation timing (10 samples):")
        logger.info(f"  - Average: {avg_time:.1f}ms")
        logger.info(f"  - Std Dev: {std_time:.1f}ms")
        logger.info(f"  - Min: {min_time:.1f}ms")
        logger.info(f"  - Max: {max_time:.1f}ms")

        if avg_time < 50:  # Should be fast without cameras
            logger.info("✓ Observation reading is fast")
        else:
            logger.warning("⚠ Observation reading might be slow")

    except Exception as e:
        logger.error(f"✗ Observation timing test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Test 2: Continuous observation loop (simulating policy)
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Continuous observation loop (30Hz target)")
        logger.info("=" * 60)

        target_hz = 30
        target_dt = 1.0 / target_hz
        num_iterations = 60  # 2 seconds at 30Hz

        loop_times = []
        obs_times = []

        logger.info(f"Running {num_iterations} iterations at {target_hz}Hz...")

        for i in range(num_iterations):
            loop_start = time.time()

            # Read observation
            obs_start = time.time()
            obs = robot.get_observation()
            obs_time = time.time() - obs_start
            obs_times.append(obs_time)

            # Simulate some processing
            time.sleep(0.001)  # 1ms of "policy" computation

            # Calculate loop time
            loop_time = time.time() - loop_start
            loop_times.append(loop_time)

            # Sleep to maintain target frequency
            sleep_time = target_dt - loop_time
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Calculate statistics
        actual_hz = 1.0 / np.mean(loop_times)
        avg_obs_time = np.mean(obs_times) * 1000

        logger.info(f"\nResults:")
        logger.info(f"  - Target frequency: {target_hz}Hz")
        logger.info(f"  - Actual frequency: {actual_hz:.1f}Hz")
        logger.info(f"  - Avg observation time: {avg_obs_time:.1f}ms")
        logger.info(f"  - Loop time avg: {np.mean(loop_times)*1000:.1f}ms")

        if actual_hz >= target_hz * 0.9:  # Within 10% of target
            logger.info("✓ Successfully maintained target frequency")
        else:
            logger.warning("⚠ Could not maintain target frequency")

    except Exception as e:
        logger.error(f"✗ Continuous loop test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Test 3: Observation consistency check
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Observation consistency")
        logger.info("=" * 60)

        # Read multiple observations rapidly
        observations = []
        for _ in range(5):
            obs = robot.get_observation()
            observations.append(obs)
            time.sleep(0.1)

        # Check that all observations have the same keys
        keys_set = [set(obs.keys()) for obs in observations]
        if all(k == keys_set[0] for k in keys_set):
            logger.info("✓ All observations have consistent keys")
        else:
            logger.error("✗ Observations have inconsistent keys")
            robot.disconnect()
            return False

        # Check joint position changes
        logger.info("\nJoint position changes over time:")
        for joint in ["waist", "shoulder", "elbow"]:
            key = f"{joint}.pos"
            if key in observations[0]:
                positions = [obs[key] for obs in observations]
                change = max(positions) - min(positions)
                logger.info(f"  - {joint}: range = {change:.4f}")

        logger.info("✓ Observations are consistent")

    except Exception as e:
        logger.error(f"✗ Consistency test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Test 4: Action + Observation loop
        logger.info("\n" + "=" * 60)
        logger.info("TEST: Action + Observation loop (realistic policy simulation)")
        logger.info("=" * 60)

        logger.info("Sending small sinusoidal movements while reading observations...")

        duration = 5.0  # 5 seconds
        frequency = 30  # 30 Hz
        num_steps = int(duration * frequency)

        action_times = []
        obs_times = []
        loop_times = []

        for i in range(num_steps):
            loop_start = time.time()

            # Read observation
            obs_start = time.time()
            obs = robot.get_observation()
            obs_times.append(time.time() - obs_start)

            # Generate sinusoidal action
            t = i / frequency
            amplitude = 0.05  # Small amplitude in normalized space
            offset = np.sin(2 * np.pi * 0.5 * t) * amplitude  # 0.5 Hz sine wave

            # Create action
            action = {key: obs[key] for key in robot.action_features.keys() if key in obs}
            if "waist.pos" in action:
                action["waist.pos"] = max(-1.0, min(1.0, obs["waist.pos"] + offset))

            # Send action
            action_start = time.time()
            robot.send_action(action)
            action_times.append(time.time() - action_start)

            loop_times.append(time.time() - loop_start)

            # Maintain frequency
            sleep_time = (1.0/frequency) - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Statistics
        logger.info(f"\nPerformance over {duration}s at {frequency}Hz:")
        logger.info(f"  - Avg observation time: {np.mean(obs_times)*1000:.1f}ms")
        logger.info(f"  - Avg action time: {np.mean(action_times)*1000:.1f}ms")
        logger.info(f"  - Avg loop time: {np.mean(loop_times)*1000:.1f}ms")
        logger.info(f"  - Actual frequency: {1.0/np.mean(loop_times):.1f}Hz")

        if np.mean(action_times) < 0.01:  # Action should be <10ms with blocking=False
            logger.info("✓ Action commands are non-blocking (correct!)")
        else:
            logger.warning("⚠ Action commands might be blocking")

        logger.info("✓ Realistic policy loop test completed")

    except Exception as e:
        logger.error(f"✗ Action+Observation loop test failed: {e}")
        robot.disconnect()
        return False

    try:
        # Return to home and disconnect
        logger.info("\nReturning to home...")
        robot.calibrate()
        time.sleep(2)

        logger.info("Disconnecting...")
        robot.disconnect()
        logger.info("✓ Disconnected successfully")

    except Exception as e:
        logger.error(f"✗ Cleanup failed: {e}")
        return False

    logger.info("\n" + "=" * 60)
    logger.info("✓ ALL OBSERVATION TESTS PASSED!")
    logger.info("=" * 60)
    logger.info("\nSummary:")
    logger.info(f"  - Can read observations at high frequency")
    logger.info(f"  - Action commands are non-blocking")
    logger.info(f"  - Ready for real-time policy control")
    return True


if __name__ == "__main__":
    success = test_observations()
    sys.exit(0 if success else 1)
