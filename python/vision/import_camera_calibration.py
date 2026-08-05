#!/usr/bin/env python3
"""
import_camera_calibration.py — import a colleague's MATLAB camera_calibration.mat
into this project's kinect_vision.json, replacing this project's own
checkerboard/height-tag-derived intrinsics.

Source: vision/calibration_data/camera_calibration.mat (repo-root-relative —
that's a separate MATLAB codebase at the repo root, not matlab/vision/),
produced by vision/estimateCameraHeight.m / vision/calibrateOverheadCamera.m.
Fields consumed: norm_f (focal length, assumed fx==fy), k1/k2 (radial
distortion), dist_center (principal point), h (mounting height). nadir_px/
tilt_deg are carried over as diagnostic-only provenance — see VisionConfig's
docstring comment; they are never read by any transform math. Real tilt
correction comes from vision/registerWorldFrameClicks.m's world-registration
step (ported in world_registration.py), not from these two numbers.

load_camcal_mat()/apply_camcal() are also imported directly by
run_localization.py's GUI "Import camera_calibration.mat" button — this file
is the single source of truth for the import logic either way.

Usage:
    python import_camera_calibration.py
    python import_camera_calibration.py --mat /path/to/camera_calibration.mat --dry-run
"""
import sys
import argparse
from pathlib import Path

from scipy.io import loadmat

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.vision_config import VisionConfig, DEFAULT_CONFIG_PATH

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_MAT_PATH = REPO_ROOT / 'vision' / 'calibration_data' / 'camera_calibration.mat'

CAMCAL_SOURCE = 'camera_calibration.mat (Adersh M, MATLAB)'


def _scalar(d: dict, key: str) -> float:
    return float(d[key][0, 0])


def _pair(d: dict, key: str) -> list:
    return [float(v) for v in d[key][0]]


def load_camcal_mat(mat_path: Path | str) -> dict:
    """Load and extract the fields this project consumes from a
    camera_calibration.mat file."""
    d = loadmat(str(mat_path))
    return {
        'norm_f': _scalar(d, 'norm_f'),
        'k1': _scalar(d, 'k1'),
        'k2': _scalar(d, 'k2'),
        'dist_center': _pair(d, 'dist_center'),
        'h': _scalar(d, 'h'),
        'nadir_px': _pair(d, 'nadir_px') if 'nadir_px' in d else None,
        'tilt_deg': _scalar(d, 'tilt_deg') if 'tilt_deg' in d else None,
        'date': str(d['date'][0]) if 'date' in d else None,
    }


def apply_camcal(cfg: VisionConfig, camcal: dict, source: str = CAMCAL_SOURCE) -> dict:
    """Mutates cfg in place with the camcal fields (does not save). Returns
    {'before': {...}, 'after': {...}} for display/logging at the call site."""
    before = {'fx': cfg.fx, 'fy': cfg.fy, 'cx': cfg.cx, 'cy': cfg.cy,
              'mounting_height_m': cfg.mounting_height_m,
              'dist_coeffs': list(cfg.dist_coeffs)}

    cfg.fx, cfg.fy = camcal['norm_f'], camcal['norm_f']
    cfg.cx, cfg.cy = camcal['dist_center'][0], camcal['dist_center'][1]
    cfg.dist_coeffs = [camcal['k1'], camcal['k2'], 0.0, 0.0, 0.0]
    cfg.mounting_height_m = camcal['h']

    cfg.camcal_source = source
    cfg.camcal_date = camcal.get('date')
    cfg.nadir_px = camcal.get('nadir_px')
    cfg.tilt_deg_diagnostic = camcal.get('tilt_deg')

    # These describe THIS project's now-abandoned checkerboard/height-tag
    # intrinsics — leaving them populated would misdescribe the numbers above.
    cfg.intrinsics_calibrated_at = None
    cfg.intrinsics_rms_px = None
    cfg.height_calibrated_at = None
    cfg.height_z_error_m = None

    after = {'fx': cfg.fx, 'fy': cfg.fy, 'cx': cfg.cx, 'cy': cfg.cy,
             'mounting_height_m': cfg.mounting_height_m,
             'dist_coeffs': list(cfg.dist_coeffs)}
    return {'before': before, 'after': after}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mat', default=str(DEFAULT_MAT_PATH),
                     help='path to camera_calibration.mat')
    ap.add_argument('--config', default=str(DEFAULT_CONFIG_PATH))
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    mat_path = Path(args.mat)
    if not mat_path.exists():
        print(f"Not found: {mat_path}")
        sys.exit(1)

    camcal = load_camcal_mat(mat_path)
    cfg = VisionConfig.load(args.config)
    diff = apply_camcal(cfg, camcal)

    b, a = diff['before'], diff['after']
    print(f"fx/fy:   {b['fx']:.2f}/{b['fy']:.2f}  ->  {a['fx']:.2f}/{a['fy']:.2f}")
    print(f"cx/cy:   {b['cx']:.2f}/{b['cy']:.2f}  ->  {a['cx']:.2f}/{a['cy']:.2f}")
    print(f"height:  {b['mounting_height_m']:.4f} m  ->  {a['mounting_height_m']:.4f} m")
    print(f"dist_coeffs: {b['dist_coeffs']}  ->  {a['dist_coeffs']}")

    if args.dry_run:
        print("\nDry run — nothing saved.")
        return

    cfg.save(args.config)
    print(f"\nSaved -> {args.config}")
    print("\nNOTE: this alone does not correct camera tilt (~{:.2f}° per the "
          "source file's diagnostic tilt_deg) — run the ⑦REG click-based "
          "world registration step next.".format(camcal.get('tilt_deg') or 0.0))


if __name__ == '__main__':
    main()
