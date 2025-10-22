#!/usr/bin/env python3
"""
Test both RealSense cameras can stream simultaneously
"""

import sys
import time
from pathlib import Path

# Add lerobot to path
repo_root = Path(__file__).parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))

from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

print("=" * 60)
print("RealSense Dual Camera Streaming Test")
print("=" * 60)

# Camera configurations from w250_config.yaml
camera1_config = RealSenseCameraConfig(
    serial_number_or_name="220322061194",
    fps=30,
    width=640,
    height=480,
    color_mode="rgb",
    use_depth=False,
)

camera2_config = RealSenseCameraConfig(
    serial_number_or_name="220222065523",
    fps=30,
    width=640,
    height=480,
    color_mode="rgb",
    use_depth=False,
)

print("\n1. Initializing Camera 1 (220322061194)...")
camera1 = RealSenseCamera(camera1_config)
try:
    camera1.connect()
    print("✅ Camera 1 connected successfully")
except Exception as e:
    print(f"❌ Camera 1 failed to connect: {e}")
    sys.exit(1)

print("\n2. Initializing Camera 2 (220222065523)...")
camera2 = RealSenseCamera(camera2_config)
try:
    camera2.connect()
    print("✅ Camera 2 connected successfully")
except Exception as e:
    print(f"❌ Camera 2 failed to connect: {e}")
    camera1.disconnect()
    sys.exit(1)

print("\n3. Testing simultaneous streaming (10 frames)...")
try:
    for i in range(10):
        start_time = time.time()

        # Read from both cameras
        frame1 = camera1.read()
        frame2 = camera2.read()

        read_time = (time.time() - start_time) * 1000  # ms

        print(f"  Frame {i+1}/10: "
              f"Camera1={frame1.shape}, "
              f"Camera2={frame2.shape}, "
              f"Time={read_time:.1f}ms")

        time.sleep(0.03)  # ~30 FPS

    print("\n✅ SUCCESS: Both cameras streaming simultaneously!")
    print(f"   Camera 1: {frame1.shape} RGB")
    print(f"   Camera 2: {frame2.shape} RGB")

except Exception as e:
    print(f"\n❌ ERROR during streaming: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("\n4. Disconnecting cameras...")
    try:
        camera1.disconnect()
        print("✅ Camera 1 disconnected")
    except:
        pass

    try:
        camera2.disconnect()
        print("✅ Camera 2 disconnected")
    except:
        pass

print("\n" + "=" * 60)
print("Test complete!")
print("=" * 60)
