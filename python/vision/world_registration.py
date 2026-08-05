"""
world_registration.py — click-based camera->world rigid registration.

Port of vision/registerWorldFrameClicks.m (a colleague's MATLAB tool, at the
repo-root `vision/` folder — a separate codebase from this project's own
`matlab/`): click each already-placed UWB anchor's pixel position in a Kinect
snapshot, using anchors.json's coordinates as ground truth, and solve a full
rigid rotation + translation (R, t) via homography DLT + decomposition. This
is what actually corrects for camera tilt — backproject.pixel_to_world's
flat-plane assumption cannot, by design (see its own module docstring).

Two-mode dispatch mirrors vision/transformPoseToWorld.m: prefer this
module's registered transform when available, fall back to
backproject.pixel_to_world otherwise. compute_world() is the single entry
point callers should use instead of calling pixel_to_world directly.
"""
import sys
import json
import math
import itertools
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

from .vision_config import VisionConfig
from .backproject import undistort_pts, pixel_to_world

# Ported literals — registerWorldFrameClicks.m:210,226,346,195
MIN_ANCHORS  = 4
WARN_RMSE_M  = 0.03
BLOCK_RMSE_M = 0.05
HEIGHT_TOL_M = 0.25

# check_point_spread thresholds — NOT in the MATLAB source, added after
# loading vision/calibration_data/world_registration.mat.bad-20260707 and
# finding its 5-point worldPoints set had 3 points exactly collinear (a
# degenerate SUBSET the MATLAB rank(centered)<2 guard doesn't catch — that
# check only fires on TOTAL collinearity of the full point set) with the fit
# landing at rmse_m=1.68. These are soft warnings, not hard errors.
SPREAD_CONDITION_WARN = 6.0
TRIANGLE_AREA_FRAC_WARN = 0.05


@dataclass
class WorldRegistration:
    R: list                       # 3x3 camera->world rotation
    t: list                       # 3-vector camera->world translation (m)
    rmse_m: float
    max_residual_m: float
    residuals_m: list             # per anchor, same order as anchor_ids
    anchor_ids: list
    world_xy: list                # [[x,y],...], same order as anchor_ids
    pixel_clicks_uv: list         # [[u,v],...], same order — audit trail
    anchor_z_m: float
    implied_height_m: float
    implied_tilt_deg: float
    method: str = 'clicks'
    registered_at: str = ''
    gate_passed: bool = True
    gate_override: bool = False

    def save(self, path: Path | str) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w') as f:
            json.dump(asdict(self), f, indent=2)
            f.write('\n')


def load_registration(path: Path | str) -> Optional[WorldRegistration]:
    """None if the file doesn't exist — mirrors MATLAB's
    config.extrinsics.available=false. Deliberately not a classmethod
    default-constructing an identity transform: "never registered" must stay
    distinguishable from "registered as R=I"."""
    path = Path(path)
    if not path.exists():
        return None
    with open(path) as f:
        data = json.load(f)
    known = {k: v for k, v in data.items() if k in WorldRegistration.__dataclass_fields__}
    return WorldRegistration(**known)


# ── Hartley-normalized DLT homography ───────────────────────────────────────

def _hartley_normalize(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Centroid -> origin, mean distance -> sqrt(2). Returns (T, normalized_pts)."""
    mu = pts.mean(axis=0)
    d = float(np.mean(np.linalg.norm(pts - mu, axis=1)))
    if d < np.finfo(float).eps:
        d = 1.0
    s = math.sqrt(2) / d
    T = np.array([
        [s, 0.0, -s * mu[0]],
        [0.0, s, -s * mu[1]],
        [0.0, 0.0, 1.0],
    ])
    n = pts.shape[0]
    homo = np.hstack([pts, np.ones((n, 1))])
    normalized = (T @ homo.T).T
    return T, normalized[:, :2]


def dlt_homography(XY: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Kx2 world (X,Y) -> Kx2 normalized image (xn,yn). Hartley-normalized
    DLT, port of registerWorldFrameClicks.m:385-412. Returns 3x3 H such that
    [x,y,1]^T ~ H @ [X,Y,1]^T, scale fixed via H /= norm(H[:,0])."""
    n = XY.shape[0]
    Tw, XYn = _hartley_normalize(XY)
    Ti, xyn = _hartley_normalize(xy)

    A = np.zeros((2 * n, 9))
    for k in range(n):
        Xn, Yn = XYn[k]
        xn, yn = xyn[k]
        A[2 * k]     = [-Xn, -Yn, -1.0, 0.0, 0.0, 0.0, xn * Xn, xn * Yn, xn]
        A[2 * k + 1] = [0.0, 0.0, 0.0, -Xn, -Yn, -1.0, yn * Xn, yn * Yn, yn]

    _, _, Vt = np.linalg.svd(A)
    H_n = Vt[-1].reshape(3, 3)
    H = np.linalg.inv(Ti) @ H_n @ Tw
    H = H / np.linalg.norm(H[:, 0])
    return H


# ── Homography -> rigid (R, t), and residuals ───────────────────────────────

def homography_to_pose(H: np.ndarray, anchor_z_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Decompose a world-XY -> normalized-image homography into camera->world
    (R, t), given the plane z = anchor_z_m.

    Base math (lambda magnitude, r1/r2/r3, SVD-orthonormalization, t_wc) is a
    port of registerWorldFrameClicks.m:145-170. The overall SIGN of H is
    ambiguous — H and -H describe the same homography (SVD null-space
    vectors are only defined up to sign) — and the MATLAB source resolves it
    with a single heuristic (sign of H(3,3)). Ported that heuristic, then
    verified it against a synthetic self-test with a real ~3.8 deg tilt: it
    picked the WRONG branch, landing on a physically impossible negative
    camera height. Replaced with a directly physical check instead: this
    project's camera is always mounted ABOVE the floor, so the correct
    branch is whichever gives implied height t[2] > 0 — computed both
    candidates and picked by that constraint rather than trusting the
    intermediate-sign proxy.
    """
    lambda_mag = (np.linalg.norm(H[:, 0]) + np.linalg.norm(H[:, 1])) / 2.0

    def _decompose(lambda_: float) -> tuple[np.ndarray, np.ndarray]:
        r1 = H[:, 0] / lambda_
        r2 = H[:, 1] / lambda_
        r3 = np.cross(r1, r2)
        U, _, Vt = np.linalg.svd(np.column_stack([r1, r2, r3]))
        R_wc = U @ np.diag([1.0, 1.0, float(np.sign(np.linalg.det(U @ Vt)))]) @ Vt
        t_wc = H[:, 2] / lambda_ - R_wc[:, 2] * anchor_z_m
        return R_wc.T, -R_wc.T @ t_wc

    R, t = _decompose(lambda_mag)
    if t[2] > 0:
        return R, t
    return _decompose(-lambda_mag)


def _ray_plane_world(R: np.ndarray, t: np.ndarray, xn: float, yn: float, z_world: float) -> np.ndarray:
    """Intersect the camera ray through normalized pixel (xn, yn) with the
    world plane Z = z_world, given camera->world (R, t). Returns the 3-vector
    world point (z-component == z_world by construction)."""
    ray = np.array([xn, yn, 1.0])
    dirW = R @ ray
    s = (z_world - t[2]) / dirW[2]
    return R @ (s * ray) + t


def compute_residuals(R: np.ndarray, t: np.ndarray, world_xy: np.ndarray,
                       anchor_z_m: float, xn: np.ndarray, yn: np.ndarray):
    """Per-anchor residual via ray-plane intersection, port of
    registerWorldFrameClicks.m:172-186."""
    K = world_xy.shape[0]
    mapped_xy = np.zeros((K, 2))
    for k in range(K):
        pw = _ray_plane_world(R, t, xn[k], yn[k], anchor_z_m)
        mapped_xy[k] = pw[:2]
    residuals = np.linalg.norm(mapped_xy - world_xy, axis=1)
    rmse = float(math.sqrt(np.mean(residuals ** 2)))
    return mapped_xy, residuals, rmse


# ── Degeneracy checks ───────────────────────────────────────────────────────

def check_point_spread(world_xy, anchor_ids: Optional[list] = None) -> list[str]:
    """Non-fatal warnings about a poorly-conditioned anchor layout, beyond
    the hard collinearity check in solve_registration(). See the
    .bad-20260707 postmortem note at the top of this file for why this
    exists — the ported MATLAB rank check alone isn't enough."""
    warnings: list[str] = []
    world_xy = np.asarray(world_xy, dtype=float)
    n = len(world_xy)
    labels = list(anchor_ids) if anchor_ids is not None else list(range(n))

    centered = world_xy - world_xy.mean(axis=0)
    _, S, _ = np.linalg.svd(centered, full_matrices=False)
    if len(S) < 2 or S[1] <= 1e-9:
        warnings.append("anchor layout is degenerate (all points nearly collinear)")
    else:
        condition = float(S[0] / S[1])
        if condition > SPREAD_CONDITION_WARN:
            warnings.append(
                f"anchor layout is nearly collinear (spread ratio {condition:.1f}:1) "
                f"— registration will be poorly constrained along the short axis"
            )

    if n >= 3:
        xs, ys = world_xy[:, 0], world_xy[:, 1]
        bbox_area = (xs.max() - xs.min()) * (ys.max() - ys.min())
        if bbox_area > 1e-9:
            for combo in itertools.combinations(range(n), 3):
                p = world_xy[list(combo)]
                tri_area = abs((p[1, 0] - p[0, 0]) * (p[2, 1] - p[0, 1])
                               - (p[2, 0] - p[0, 0]) * (p[1, 1] - p[0, 1])) / 2.0
                if tri_area < TRIANGLE_AREA_FRAC_WARN * bbox_area:
                    ids_str = ', '.join(str(labels[i]) for i in combo)
                    warnings.append(
                        f"anchors {ids_str} are nearly collinear "
                        f"(triangle area {tri_area:.3f} m² vs bounding box {bbox_area:.3f} m²)"
                    )
                    break
    return warnings


# ── Top-level solve ──────────────────────────────────────────────────────────

def solve_registration(
    anchor_ids: list,
    world_xy: list,
    anchor_z_m: float,
    pixel_clicks_uv: list,
    cfg: VisionConfig,
) -> WorldRegistration:
    """Solve a camera->world rigid registration from clicked anchor pixels.
    Port of registerWorldFrameClicks.m's core solve (lines 94-186). Raises
    ValueError if there are too few anchors or the world points are (fully)
    collinear — see check_point_spread() for the softer, partial-degeneracy
    warnings this doesn't catch."""
    if len(anchor_ids) < MIN_ANCHORS:
        raise ValueError(f"Need >= {MIN_ANCHORS} anchors, got {len(anchor_ids)}")
    if not (len(anchor_ids) == len(world_xy) == len(pixel_clicks_uv)):
        raise ValueError("anchor_ids, world_xy, pixel_clicks_uv must be the same length")

    world_xy_arr = np.asarray(world_xy, dtype=float)
    centered = world_xy_arr - world_xy_arr.mean(axis=0)
    if np.linalg.matrix_rank(centered, tol=1e-6) < 2:
        raise ValueError("Anchor positions are collinear — spread them across the floor")

    pixels = np.asarray(pixel_clicks_uv, dtype=float)
    pixels_u = undistort_pts(pixels, cfg)
    xn = (pixels_u[:, 0] - cfg.cx) / cfg.fx
    yn = (pixels_u[:, 1] - cfg.cy) / cfg.fy

    H = dlt_homography(world_xy_arr, np.column_stack([xn, yn]))
    R, t = homography_to_pose(H, anchor_z_m)
    _mapped_xy, residuals, rmse = compute_residuals(R, t, world_xy_arr, anchor_z_m, xn, yn)

    implied_height_m = float(t[2])
    implied_tilt_deg = math.degrees(math.acos(float(np.clip(abs(R[2, 2]), -1.0, 1.0))))

    gate_passed = not (rmse > BLOCK_RMSE_M
                        or abs(implied_height_m - cfg.mounting_height_m) > HEIGHT_TOL_M)

    return WorldRegistration(
        R=R.tolist(), t=t.tolist(),
        rmse_m=rmse, max_residual_m=float(residuals.max()),
        residuals_m=residuals.tolist(),
        anchor_ids=list(anchor_ids),
        world_xy=world_xy_arr.tolist(),
        pixel_clicks_uv=pixels.tolist(),
        anchor_z_m=float(anchor_z_m),
        implied_height_m=implied_height_m,
        implied_tilt_deg=implied_tilt_deg,
        method='clicks',
        registered_at=datetime.now().isoformat(timespec='seconds'),
        gate_passed=gate_passed,
        gate_override=False,
    )


# ── Runtime use ──────────────────────────────────────────────────────────────

def pixel_to_world_registered(u: float, v: float, reg: WorldRegistration,
                               cfg: VisionConfig, z_world: float = 0.0) -> tuple[float, float, float]:
    """Ray-plane intersection using the registered (R, t) transform — the
    runtime replacement for backproject.pixel_to_world once a registration
    exists. z == z_world by construction, same as pixel_to_world."""
    uu, vv = undistort_pts(np.array([[u, v]]), cfg)[0]
    xn = (uu - cfg.cx) / cfg.fx
    yn = (vv - cfg.cy) / cfg.fy
    pw = _ray_plane_world(np.array(reg.R), np.array(reg.t), xn, yn, z_world)
    return (float(pw[0]), float(pw[1]), float(pw[2]))


def compute_world(u: float, v: float, cfg: VisionConfig,
                   registration: Optional[WorldRegistration], z_world: float = 0.0) -> tuple[float, float, float]:
    """Prefer the registered transform when available, else fall back to
    backproject.pixel_to_world's flat-plane nadir formula. Mirrors
    transformPoseToWorld.m's two-mode dispatch. Single entry point callers
    should use instead of calling pixel_to_world directly."""
    if registration is not None:
        return pixel_to_world_registered(u, v, registration, cfg, z_world)
    return pixel_to_world(u, v, cfg, z_world)


# ── Self-test ────────────────────────────────────────────────────────────────

def _self_test() -> bool:
    print("=" * 60)
    print("World Registration — Synthetic Self-Test")
    print("=" * 60)

    cfg = VisionConfig(fx=1050.0, fy=1050.0, cx=960.0, cy=540.0, mounting_height_m=3.87344)

    # Known ground truth: ~3.81 deg tilt (matches camera_calibration.mat's
    # diagnostic value) around a mostly-horizontal axis, real mount height.
    tilt = math.radians(3.81)
    axis = np.array([1.0, 0.3, 0.0])
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    R_true = np.eye(3) + math.sin(tilt) * K + (1 - math.cos(tilt)) * (K @ K)
    t_true = np.array([0.05, -0.03, 3.87344])

    anchor_ids = [1, 2, 3, 4]
    world_xy = np.array([[-1.5, 1.5], [1.5, 1.5], [1.5, -1.5], [-1.5, -1.5]])
    anchor_z = 0.30

    pixel_clicks = []
    for (X, Y) in world_xy:
        world_pt = np.array([X, Y, anchor_z])
        cam_pt = R_true.T @ (world_pt - t_true)
        xn, yn = cam_pt[0] / cam_pt[2], cam_pt[1] / cam_pt[2]
        pixel_clicks.append([xn * cfg.fx + cfg.cx, yn * cfg.fy + cfg.cy])

    reg = solve_registration(anchor_ids, world_xy.tolist(), anchor_z, pixel_clicks, cfg)

    R_rec = np.array(reg.R)
    t_rec = np.array(reg.t)
    r_err = float(np.max(np.abs(R_rec - R_true)))
    t_err = float(np.max(np.abs(t_rec - t_true)))

    print(f"\nRecovered vs true R: max abs diff = {r_err:.2e}")
    print(f"Recovered vs true t: recovered {t_rec}, true {t_true}, max abs diff = {t_err:.2e}")
    print(f"RMSE: {reg.rmse_m * 1000:.6f} mm  (noiseless synthetic case, expect ~0)")
    print(f"implied_height_m={reg.implied_height_m:.4f}  implied_tilt_deg={reg.implied_tilt_deg:.4f}")
    print(f"gate_passed: {reg.gate_passed}")

    ok1 = r_err < 1e-6 and t_err < 1e-6 and reg.rmse_m < 1e-6

    # Degenerate-layout case: reproduce the .bad-20260707 postmortem shape
    # (4 rectangle corners + a 5th point collinear with two of them).
    degenerate_xy = [[0, 0], [6.3, 0], [6.3, 3.1], [0, 3.1], [3.15, 3.1]]
    warnings = check_point_spread(degenerate_xy)
    print(f"\nDegenerate layout ({degenerate_xy}):")
    for w in warnings:
        print(f"  WARNING: {w}")
    ok2 = len(warnings) > 0

    passed = ok1 and ok2
    print(f"\n{'PASS' if passed else 'FAIL'}")
    return passed


if __name__ == '__main__':
    ok = _self_test()
    sys.exit(0 if ok else 1)
