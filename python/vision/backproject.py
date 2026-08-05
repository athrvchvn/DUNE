"""
backproject.py — pixel -> world back-projection onto an assumed Z plane.

The overhead Kinect looks straight down (no tilt). Given a pixel and the
calibrated intrinsics + mounting height, this recovers the world (x, y) that
pixel corresponds to on a horizontal plane at world Z = z_world (the floor by
default, z_world=0). No AprilTag/PnP is involved here — this is the
"click-to-place" path used for every anchor/tag placement at runtime; PnP is
used only once, offline, in height_calibration.py.

Mirror correction contract: `pixel_to_world()` expects (u, v) relative to the
ALREADY mirror-corrected frame (i.e. the same frame a human would see and
click on, post `mirror_correct()`) — it does not re-flip internally. Every
frame handed to a person (the GUI's Grab Frame preview) or fed to AprilTag
detection (height_calibration.py) must be run through `mirror_correct()`
first; from that point on all pixel coordinates — clicks, detections — are in
the corrected frame and this function's plain pinhole math applies directly.
This used to be split (frame shown raw, click coordinate un-flipped
internally) which meant a person had to click on a mirrored preview and trust
the math to silently compensate — confusing, and error-prone for telling
physically-similar objects apart by eye. Now the correction happens once, up
front, and everything downstream — human and code alike — works in the same
natural, un-mirrored frame.

Derivation (matches RoverControl.transformPositionToWorld's convention,
world = [cam_x, -cam_y, height - cam_z], with the PnP translation replaced by
a ray-cast to the assumed plane):

    x_n = (u - cx) / fx,  y_n = (v - cy) / fy      # normalized camera ray
    depth = mounting_height_m - z_world             # distance to the plane
    cam = depth * (x_n, y_n, 1)
    world = (cam_x, -cam_y, mounting_height_m - cam_z)

which collapses to world_z == z_world by construction (cam_z == depth).
"""
import numpy as np

from .vision_config import VisionConfig


def mirror_correct(frame: np.ndarray, cfg: VisionConfig) -> np.ndarray:
    """Undo the Kinect v2 color stream's known left-right mirror, if configured.

    Apply this ONCE to every raw captured frame before displaying it, clicking
    on it, or running detection on it — see the module docstring. A no-op
    (returns the frame unchanged) when cfg.mirror_flip is False.
    """
    if not cfg.mirror_flip:
        return frame
    return np.fliplr(frame).copy()


def undistort_pts(pts_uv: np.ndarray, cfg: VisionConfig) -> np.ndarray:
    """Closed-form radial undistortion, port of vision/undistortPts.m
    (the colleague's MATLAB model — radial k1/k2 only, no tangential term).

    pts_uv: (N, 2) raw pixel coordinates. Returns (N, 2) undistorted.
    No-op (returns pts_uv unchanged) when cfg.dist_coeffs[0]==[1]==0 — true
    for every intrinsics set this project has used so far (this project's
    own checkerboard fit and the colleague's box-hop fit both found
    negligible Kinect v2 lens distortion) — kept for completeness/
    future-proofing rather than because it currently changes anything.
    Assumes fx == fy (the colleague's model uses a single focal length,
    cc.norm_f); cfg.fx is used as that scale factor.
    """
    pts_uv = np.atleast_2d(np.asarray(pts_uv, dtype=float))
    k1, k2 = cfg.dist_coeffs[0], cfg.dist_coeffs[1]
    if k1 == 0.0 and k2 == 0.0:
        return pts_uv
    dc = np.array([cfg.cx, cfg.cy])
    d = (pts_uv - dc) / cfg.fx
    r2 = np.sum(d ** 2, axis=1, keepdims=True)
    s = 1 + k1 * r2 + k2 * r2 ** 2
    return dc + d * s * cfg.fx


def distort_pts(pts_uv: np.ndarray, cfg: VisionConfig, n_iter: int = 8) -> np.ndarray:
    """Iterative inverse of undistort_pts (fixed-point, 8 steps), port of
    vision/distortPts.m. Only needed to re-project model-derived points back
    onto the raw (still-distorted) image for overlay drawing — no core
    world_registration.py math needs this. Same no-op condition as
    undistort_pts.
    """
    pts_uv = np.atleast_2d(np.asarray(pts_uv, dtype=float))
    k1, k2 = cfg.dist_coeffs[0], cfg.dist_coeffs[1]
    if k1 == 0.0 and k2 == 0.0:
        return pts_uv
    dc = np.array([cfg.cx, cfg.cy])
    target = (pts_uv - dc) / cfg.fx
    d = target.copy()
    for _ in range(n_iter):
        r2 = np.sum(d ** 2, axis=1, keepdims=True)
        d = target / (1 + k1 * r2 + k2 * r2 ** 2)
    return dc + d * cfg.fx


def pixel_to_world(u: float, v: float, cfg: VisionConfig, z_world: float = 0.0) -> tuple[float, float, float]:
    """Back-project a pixel (u, v) from the mirror-corrected frame onto the
    world plane Z = z_world. (u, v) must already be in mirror_correct()'d
    frame coordinates — this does no flipping of its own. Undistorted via
    undistort_pts() first (a no-op at zero distortion, the only case so far)."""
    u, v = undistort_pts(np.array([[u, v]]), cfg)[0]

    x_n = (u - cfg.cx) / cfg.fx
    y_n = (v - cfg.cy) / cfg.fy

    depth = cfg.mounting_height_m - z_world
    cam_x = depth * x_n
    cam_y = depth * y_n
    cam_z = depth

    world_x = cam_x
    world_y = -cam_y
    world_z = cfg.mounting_height_m - cam_z
    return (world_x, world_y, world_z)
