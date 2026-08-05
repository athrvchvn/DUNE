#!/usr/bin/env python3
"""
debug_view.py — live cv2 preview of the Kinect color feed with a metric
floor-grid overlay, for bring-up/sanity-checking the mount and intrinsics.
Ports the ppm = fx / mounting_height_m math from liveGridOverlay.m.

Standalone bring-up tool — not part of the WS-integrated GUI (the live
product path is on-demand single-frame capture, see gui.html's VISION step).

Usage:
    python debug_view.py            # press 'q' to quit
"""
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.vision_config import VisionConfig, DEFAULT_CONFIG_PATH
from vision.kinect_capture import KinectCapture
from vision.backproject import mirror_correct


def draw_grid(frame, cfg: VisionConfig, spacing_m: float = 0.5, extent_m: float = 4.0):
    """Overlay a metric floor grid, valid near image-centre nadir (see README caveats)."""
    ppm = cfg.fx / cfg.mounting_height_m   # pixels-per-metre at image centre
    h, w = frame.shape[:2]
    cx, cy = int(cfg.cx), int(cfg.cy)
    n = int(extent_m / spacing_m)
    for i in range(-n, n + 1):
        off = int(i * spacing_m * ppm)
        cv2.line(frame, (0, cy + off), (w, cy + off), (0, 120, 0), 1)
        cv2.line(frame, (cx + off, 0), (cx + off, h), (0, 120, 0), 1)
    cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    return frame


def main():
    cfg = VisionConfig.load(DEFAULT_CONFIG_PATH)
    cap = KinectCapture()
    cap.start()
    print("Press 'q' to quit.")
    got_first_frame = False
    try:
        while True:
            frame = cap.get_latest_frame()
            if frame is None:
                time.sleep(0.02)   # yield instead of hammering the lock/GIL
                continue
            if not got_first_frame:
                got_first_frame = True
                print(f"first frame received: {frame.shape} — opening window...")
            frame = mirror_correct(frame, cfg)
            draw_grid(frame, cfg)
            cv2.imshow('Kinect debug view', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
