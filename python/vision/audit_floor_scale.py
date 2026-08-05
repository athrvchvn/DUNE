#!/usr/bin/env python3
"""
audit_floor_scale.py — verify placement accuracy: click tape-measured floor
mark pairs, compare measured (FLAT-PLANE vs REGISTERED) distance to the true
value, bucketed by radius zone. Scoped Python port of vision/auditFloorScale.m
(the MATLAB NADIR-vs-CALIB comparison is replaced here with FLAT-PLANE-vs-
REGISTERED — the two modes actually relevant to this project's
compute_world() dispatcher, since the MATLAB CALIB column depended on a
floor-homography calibration stage this project never builds).

This is the actual "same accuracy as the colleague" confirmation: after
running the ⑦REG world registration, run this with >=6 pairs (centre, left/
right edge, x and y directions) and expect the REGISTERED column's ratios to
cluster near 1.000 uniformly across zones, unlike FLAT-PLANE's expected
edge-skew (the tilt-related "squeeze" this whole feature exists to fix).

Usage:
    python audit_floor_scale.py
    python audit_floor_scale.py --z 0.30    # marks are 0.30m off the floor
"""
import sys
import time
import argparse
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.vision_config import VisionConfig, DEFAULT_CONFIG_PATH
from vision.kinect_capture import KinectCapture
from vision.backproject import mirror_correct, pixel_to_world
from vision.world_registration import load_registration, pixel_to_world_registered

# Zone thresholds (px from the principal point), ported from auditFloorScale.m
CENTRE_PX = 350.0
MID_PX    = 650.0

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
REG_PATH  = REPO_ROOT / 'matlab' / 'config' / 'kinect_world_registration.json'


def zone_of(u: float, v: float, cx: float, cy: float) -> str:
    r = float(np.hypot(u - cx, v - cy))
    if r < CENTRE_PX:
        return 'centre'
    if r < MID_PX:
        return 'mid'
    return 'edge'


class ClickCollector:
    """Collects mouse clicks on an OpenCV window, two at a time."""

    def __init__(self, window_name: str):
        self.window_name = window_name
        self.points: list = []
        cv2.setMouseCallback(window_name, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((float(x), float(y)))

    def get_two(self, frame):
        """Blocks until 2 points are clicked, or 'q' is pressed (returns None)."""
        self.points = []
        while len(self.points) < 2:
            preview = frame.copy()
            for p in self.points:
                cv2.drawMarker(preview, (int(p[0]), int(p[1])), (0, 255, 0),
                                cv2.MARKER_CROSS, 20, 2)
            cv2.imshow(self.window_name, preview)
            key = cv2.waitKey(20) & 0xFF
            if key == ord('q'):
                return None
        return self.points[0], self.points[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--z', type=float, default=0.0,
                     help='height of the floor marks above the floor, metres (default 0.0)')
    ap.add_argument('--config', default=str(DEFAULT_CONFIG_PATH))
    args = ap.parse_args()

    cfg = VisionConfig.load(args.config)
    reg = load_registration(REG_PATH)
    if reg is None:
        print("No world registration found — REGISTERED column will just repeat FLAT-PLANE.")
        print("(run the ⑦REG GUI step first for a meaningful comparison)\n")

    cap = KinectCapture()
    cap.start()
    print("Waiting for a frame...")
    frame = None
    for _ in range(50):
        frame = cap.get_latest_frame()
        if frame is not None:
            break
        time.sleep(0.2)
    cap.stop()   # single-snapshot tool — no need to hold the device open while clicking
    if frame is None:
        print("No frame received — check the Kinect connection")
        sys.exit(1)
    frame = mirror_correct(frame, cfg)

    window = "audit_floor_scale — click 2 points per pair, 'q' to finish"
    cv2.namedWindow(window)
    collector = ClickCollector(window)

    results = []
    print("For each pair: click TWO tape-measured floor marks, then enter the true distance.")
    print("Aim for >=6 pairs spanning centre, left/right edge, x and y directions.")
    print("Press 'q' in the image window when done adding pairs.\n")
    try:
        while True:
            pair = collector.get_two(frame)
            if pair is None:
                break
            (u1, v1), (u2, v2) = pair
            true_d_str = input(f"  points ({u1:.0f},{v1:.0f})-({u2:.0f},{v2:.0f}): "
                                f"true distance (m), blank to discard: ").strip()
            if not true_d_str:
                continue
            try:
                true_d = float(true_d_str)
            except ValueError:
                print("  bad number, discarded")
                continue
            if true_d <= 0:
                print("  distance must be positive, discarded")
                continue

            fx1, fy1, _ = pixel_to_world(u1, v1, cfg, z_world=args.z)
            fx2, fy2, _ = pixel_to_world(u2, v2, cfg, z_world=args.z)
            flat_d = float(np.hypot(fx2 - fx1, fy2 - fy1))

            if reg is not None:
                rx1, ry1, _ = pixel_to_world_registered(u1, v1, reg, cfg, z_world=args.z)
                rx2, ry2, _ = pixel_to_world_registered(u2, v2, reg, cfg, z_world=args.z)
                reg_d = float(np.hypot(rx2 - rx1, ry2 - ry1))
            else:
                reg_d = flat_d

            zone = zone_of((u1 + u2) / 2, (v1 + v2) / 2, cfg.cx, cfg.cy)
            results.append({'true': true_d, 'flat': flat_d, 'reg': reg_d, 'zone': zone})
            print(f"  true={true_d:.3f} m  FLAT-PLANE={flat_d:.3f} m ({flat_d / true_d:.3f}x)  "
                  f"REGISTERED={reg_d:.3f} m ({reg_d / true_d:.3f}x)  zone={zone}\n")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

    if not results:
        print("No pairs captured.")
        return

    print("\n" + "=" * 74)
    print(f"{'zone':<8} {'n':<4} {'FLAT-PLANE mean ratio':<26} {'REGISTERED mean ratio':<26}")
    print("=" * 74)
    for zone in ('centre', 'mid', 'edge'):
        rows = [r for r in results if r['zone'] == zone]
        if not rows:
            continue
        flat_ratios = [r['flat'] / r['true'] for r in rows]
        reg_ratios  = [r['reg'] / r['true'] for r in rows]
        print(f"{zone:<8} {len(rows):<4} "
              f"{np.mean(flat_ratios):.4f} (std {np.std(flat_ratios):.4f}){'':<6} "
              f"{np.mean(reg_ratios):.4f} (std {np.std(reg_ratios):.4f})")

    all_flat = [r['flat'] / r['true'] for r in results]
    all_reg  = [r['reg'] / r['true'] for r in results]
    print("-" * 74)
    print(f"{'ALL':<8} {len(results):<4} "
          f"{np.mean(all_flat):.4f} (std {np.std(all_flat):.4f}){'':<6} "
          f"{np.mean(all_reg):.4f} (std {np.std(all_reg):.4f})")
    print("\nExpect REGISTERED to cluster near 1.000 uniformly across zones; FLAT-PLANE "
          "typically drifts from 1.000 toward the edge zone if the camera is tilted "
          "(the original 'squeeze' this feature exists to fix).")


if __name__ == '__main__':
    main()
