#!/usr/bin/env python3
"""
height_calibration.py — validate/correct the Kinect's mounting height using
one printed AprilTag of known size and known physical height, porting
calibrate_apriltag_z.m's approach: average the reported Z over ~10 detections,
compare to a hand-measured reference, back-solve the corrected mounting
height.

This is the ONLY place in this vision subsystem that runs a single-tag height
check (the runtime click-to-place workflow in backproject.py never uses
AprilTags — plain floor-plane back-projection — and the multi-tag auto-detect
anchor feature lives in apriltag_detect.py/run_localization.py). Run this to
validate a height you already trust, or to correct one you don't; it's
optional, not a prerequisite for either of those.

Usage:
    python height_calibration.py --tag-size 0.05 --actual-height 0.0
    python height_calibration.py --tag-size 0.05 --actual-height 0.0 --tag-id 3
"""
import sys
import time
import argparse
from pathlib import Path
from dataclasses import replace
from datetime import datetime
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.vision_config import VisionConfig, DEFAULT_CONFIG_PATH
from vision.kinect_capture import KinectCapture
from vision.backproject import mirror_correct
from vision.apriltag_detect import detect_tags


def detect_tag_z(frame_bgr, cfg: VisionConfig, tag_size_m: float,
                  tag_id: Optional[int] = None, tag_family: Optional[str] = None):
    """Detect one tag (optionally a specific tag_id, now that several
    physical tags can be visible at once), return its camera-frame Z
    (metres) or None. camera_z is independent of cfg.mounting_height_m —
    that's the value being calibrated here, so this must never depend on it.

    tag_family overrides cfg.apriltag_family for this probe only — the
    height-calibration tag is commonly a different physical print (e.g. a
    large tag36h11 sheet) than cfg.apriltag_family, which describes the
    anchor auto-detect feature's own tag set and must not be touched here."""
    corrected = mirror_correct(frame_bgr, cfg)
    probe_cfg = replace(cfg, apriltag_size_m=tag_size_m,
                         apriltag_family=tag_family or cfg.apriltag_family)
    detections = detect_tags(corrected, probe_cfg)
    if tag_id is not None:
        detections = [d for d in detections if d.tag_id == tag_id]
    if not detections:
        return None
    return detections[0].camera_z


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag-size', type=float, required=True,
                     help='AprilTag black-square size, metres (must match the print exactly)')
    ap.add_argument('--actual-height', type=float, required=True,
                     help='hand-measured height of the tag above the floor, metres '
                          '(0.0 if the tag is sitting on the floor)')
    ap.add_argument('--n-samples', type=int, default=10)
    ap.add_argument('--tag-id', type=int, default=None,
                     help='restrict to this specific AprilTag ID (needed once several '
                          'physical tags are visible at once — default: first accepted '
                          'detection, fine when only one tag is in frame)')
    ap.add_argument('--tag-family', type=str, default=None,
                     help='AprilTag family for THIS probe tag, e.g. tag36h11 — overrides '
                          "cfg.apriltag_family (which describes the anchor auto-detect "
                          'feature\'s own tags, usually a different physical print) '
                          'without touching the saved config. Default: use cfg.apriltag_family as-is.')
    ap.add_argument('--config', default=str(DEFAULT_CONFIG_PATH))
    args = ap.parse_args()

    cfg = VisionConfig.load(args.config)
    cap = KinectCapture()
    cap.start()

    print(f"Averaging {args.n_samples} detections of a {args.tag_size * 100:.1f} cm tag"
          + (f" (ID {args.tag_id})" if args.tag_id is not None else "") + "...")
    zs = []
    attempts = 0
    max_attempts = args.n_samples * 6
    try:
        while len(zs) < args.n_samples and attempts < max_attempts:
            frame = cap.get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            attempts += 1
            z = detect_tag_z(frame, cfg, args.tag_size, args.tag_id, args.tag_family)
            if z is not None:
                zs.append(z)
                print(f"  sample {len(zs)}/{args.n_samples}: z={z:.4f} m")
            time.sleep(0.1)
    finally:
        cap.stop()

    if len(zs) < 3:
        print("Too few detections — check lighting/tag placement/tag size")
        sys.exit(1)

    reported_z_mean = float(np.mean(zs))
    # reported_z (camera_z) is the RAW camera-to-tag distance along the
    # optical axis (PnP's pose_t[2]) — independent of mounting_height_m, see
    # TagDetection's docstring. For a straight-down camera:
    #   camera_to_tag_distance = mounting_height - actual_height
    # so the mounting height is recovered directly:
    #   mounting_height = camera_to_tag_distance + actual_height
    corrected_height = reported_z_mean + args.actual_height
    height_delta = corrected_height - cfg.mounting_height_m

    print(f"\nReported camera-to-tag distance: {reported_z_mean:.4f} m  (n={len(zs)})")
    print(f"Hand-measured tag height:        {args.actual_height:.4f} m")
    print(f"Current mounting_height_m:       {cfg.mounting_height_m:.4f} m")
    print(f"Corrected mounting_height_m:     {corrected_height:.4f} m"
          f"  ({height_delta*1000:+.1f} mm vs. current)")

    if corrected_height <= 0:
        print("\nREFUSING to save: a mounting height <= 0 is physically impossible "
              "for an overhead mount — this points at a real problem (wrong tag "
              "family/size, wrong intrinsics, or a bad detection), not a value to "
              "write. Not saved.")
        sys.exit(1)

    ans = input("\nWrite corrected height to config? [y/N] ").strip().lower()
    if ans == 'y':
        cfg.mounting_height_m = corrected_height
        cfg.height_calibrated_at = datetime.now().isoformat(timespec='seconds')
        cfg.height_z_error_m = height_delta
        cfg.save(args.config)
        print(f"Saved -> {args.config}")
    else:
        print("Not saved.")


if __name__ == '__main__':
    main()
