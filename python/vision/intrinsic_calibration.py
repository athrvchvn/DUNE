#!/usr/bin/env python3
"""
intrinsic_calibration.py — real OpenCV checkerboard intrinsic calibration for
the Kinect v2 color camera.

Fixes the MATLAB reference's fabricated reprojection error: its custom
calibration path (kinectAprilTagCalibration.m) computed
`norm(imgPts(j,:) - imgPts(j,:)) + randn()*0.5` — a point subtracted from
itself is always zero, so the printed "error" was pure Gaussian noise,
decoupled from the actual calibration. This script only ever reports the real
RMS returned by cv2.calibrateCamera.

Accuracy of the click-to-place feature (backproject.py) is directly
proportional to fx/fy here, so this is the single most important calibration
step — run it before trusting any placed anchor coordinate.

Usage:
    python intrinsic_calibration.py --images 'calib_images/*.png' \\
        --cols 9 --rows 6 --square 0.025
"""
import sys
import glob
import argparse
from datetime import datetime
from pathlib import Path

import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.vision_config import VisionConfig, DEFAULT_CONFIG_PATH


def calibrate_from_images(image_paths, cols, rows, square_size_m, fix_distortion=False):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_m

    objpoints, imgpoints = [], []
    image_size = None
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  skip (unreadable): {path}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if not found:
            print(f"  skip (no checkerboard found): {path}")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
        objpoints.append(objp)
        imgpoints.append(corners)

    if len(objpoints) < 5:
        raise RuntimeError(f"Only {len(objpoints)} usable checkerboard images found (need >=5)")

    # With few and/or small-in-frame detections, fitting a full 5-parameter
    # distortion model is prone to overfitting/destabilizing fx,fy (the
    # MATLAB reference this was ported from independently found Kinect v2's
    # color lens has negligible real distortion and got its best, validated
    # results FIXING distortion to zero rather than fitting it). Optional,
    # not default, since with enough good detections a real distortion fit
    # is more correct.
    dist_init = np.zeros(5) if fix_distortion else None
    flags = 0
    if fix_distortion:
        flags = (cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
                  | cv2.CALIB_ZERO_TANGENT_DIST)

    rms, K, dist, _rvecs, _tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, dist_init, flags=flags)

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])

    print(f"Used {len(objpoints)}/{len(image_paths)} images"
          + ("  (distortion fixed to zero)" if fix_distortion else ""))
    print(f"fx={fx:.2f}  fy={fy:.2f}  cx={cx:.2f}  cy={cy:.2f}")
    print(f"Distortion: {dist.ravel()}")
    print(f"REAL RMS reprojection error: {rms:.4f} px")
    return fx, fy, cx, cy, dist.ravel().tolist(), float(rms), image_size


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--images', nargs='+', required=True,
                     help='Glob pattern(s) for checkerboard image files')
    ap.add_argument('--cols', type=int, default=9, help='inner corners per row')
    ap.add_argument('--rows', type=int, default=6, help='inner corners per column')
    ap.add_argument('--square', type=float, required=True, help='checkerboard square size, metres')
    ap.add_argument('--fix-distortion', action='store_true',
                     help='fix distortion to zero instead of fitting it — more robust with few/small-in-frame detections')
    ap.add_argument('--config', default=str(DEFAULT_CONFIG_PATH))
    args = ap.parse_args()

    paths = []
    for pattern in args.images:
        paths.extend(sorted(glob.glob(pattern)))
    if not paths:
        print("No images matched")
        sys.exit(1)

    fx, fy, cx, cy, dist, rms, (w, h) = calibrate_from_images(
        paths, args.cols, args.rows, args.square, fix_distortion=args.fix_distortion)

    cfg = VisionConfig.load(args.config)
    cfg.fx, cfg.fy, cfg.cx, cfg.cy = fx, fy, cx, cy
    cfg.dist_coeffs = dist
    cfg.image_width, cfg.image_height = w, h
    cfg.intrinsics_calibrated_at = datetime.now().isoformat(timespec='seconds')
    cfg.intrinsics_rms_px = rms
    cfg.save(args.config)
    print(f"Saved -> {args.config}")


if __name__ == '__main__':
    main()
