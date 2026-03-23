#!/usr/bin/env python3
"""
Move W250 to REST position and capture a photo from both RealSense cameras.
Images saved to /tmp/cam_wrist.png, /tmp/cam_top.png (+ _depth variants).

Usage:
    python3 -m lerobot.scripts.w250_camera_test
"""

import time
from pathlib import Path

import cv2
import numpy as np

from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
from lerobot.robots.w250.config_w250_interbotix import W250InterbotixConfig
from lerobot.robots.w250.w250_interbotix import W250Interbotix

CAMERAS = {
    "cam_wrist": "217222063884",
    "cam_top":   "220322061194",
}
FPS    = 15
WIDTH  = 640
HEIGHT = 480
OUT    = Path("/tmp")


def main():
    # ── Robot ─────────────────────────────────────────────────────────────────
    robot_cfg = W250InterbotixConfig(
        robot_model="wx250s",
        robot_name="wx250s",
        fps=FPS,
        cameras={},           # no cameras via robot — we handle them manually
    )
    robot = W250Interbotix(robot_cfg)
    robot.connect(calibrate=True)   # goes HOME → REST
    print("Robot at REST position.")

    # ── Cameras ───────────────────────────────────────────────────────────────
    cams = {}
    for name, sn in CAMERAS.items():
        cfg = RealSenseCameraConfig(
            serial_number_or_name=sn,
            fps=FPS, width=WIDTH, height=HEIGHT,
            use_depth=True,
        )
        cam = RealSenseCamera(cfg)
        cam.connect()
        cams[name] = cam
        print(f"Camera {name} ({sn}) connected: {cam.width}x{cam.height}@{cam.fps}fps")

    # ── Capture ───────────────────────────────────────────────────────────────
    time.sleep(0.5)   # let auto-exposure settle

    for name, cam in cams.items():
        color = cam.async_read()                          # (H, W, 3) RGB uint8
        with cam.frame_lock:
            depth_raw = cam.latest_depth_frame.copy()    # (H, W) uint16 mm

        # color: RGB → BGR for cv2
        color_bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        color_path = OUT / f"{name}.png"
        cv2.imwrite(str(color_path), color_bgr)
        print(f"  {name} color  → {color_path}  shape={color.shape}")

        # depth: normalize to uint8 for visualisation (max 700 mm = 70 cm workspace)
        depth_vis = np.clip(depth_raw.astype(np.float32) / 700.0, 0, 1)
        depth_vis = (depth_vis * 255).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        depth_path = OUT / f"{name}_depth.png"
        cv2.imwrite(str(depth_path), depth_vis)
        print(f"  {name} depth  → {depth_path}  max={depth_raw.max()}mm  min_nonzero={depth_raw[depth_raw>0].min() if (depth_raw>0).any() else 0}mm")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    for cam in cams.values():
        cam.disconnect()

    robot.bot.arm.go_to_sleep_pose(blocking=True)
    robot.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
