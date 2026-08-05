"""Scoring an estimated trajectory against camera ground truth.

Ports of ``matlab/+dune/trajectoryMetrics.m``, ``alignTruth.m`` and
``readVisionLog.m``. Changes must be made in both.

The order matters: truth arrives on the Kinect's own clock at its own rate, so
it must be *interpolated onto the UWB timestamps* before any error is computed.
Doing that wrong is the classic way to manufacture accuracy — nearest-neighbour
matching across a dropout will happily pair a fix with truth from a second ago
and report the result as sub-centimetre.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# Same default as trajectoryMetrics.m — the testbed rectangle plus margin.
DEFAULT_BOUNDS = (-0.5, 6.8, -0.5, 3.6)


def prctile(x, p: float) -> float:
    """MATLAB's ``prctile``, which is *not* what ``np.percentile`` defaults to.

    MATLAB places the i-th of n sorted samples at percentile ``100*(i-0.5)/n``
    and interpolates linearly between those, clamping outside. NumPy's default
    ``'linear'`` method uses ``(i-1)/(n-1)`` instead, so the two disagree —
    ``prctile(1:10, 95)`` is 10 in MATLAB and 9.55 in NumPy. Every p95 in this
    project is compared against a MATLAB-produced number, so the definition has
    to match. ``'hazen'`` is that plotting position, verified against a direct
    implementation across sample sizes.
    """
    a = np.asarray(x, dtype=float)
    a = a[~np.isnan(a)]
    if a.size == 0:
        return float('nan')
    return float(np.percentile(a, p, method='hazen'))


# ── Vision log ────────────────────────────────────────────────────────────────

@dataclass
class VisionLog:
    t:       np.ndarray                       # posix seconds
    xy:      np.ndarray                       # [N, 2] metres
    z:       np.ndarray
    yaw_deg: np.ndarray
    rate_hz: float = 0.0
    file:    str = ''


def read_vision_log(csv_file: str | Path) -> VisionLog:
    """Read a Kinect AprilTag trajectory CSV (``aprilTagOdometry_3D`` export)."""
    path = Path(csv_file)
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f"{path} is empty")

    cols = rows[0].keys()
    for need in ('PosixTime_s', 'X_m', 'Y_m'):
        if need not in cols:
            raise ValueError(
                f"{path} has no {need} column — is this an aprilTagOdometry_3D export?")

    def col(name: str) -> np.ndarray:
        return np.array([float(r[name]) if r.get(name) not in (None, '') else np.nan
                         for r in rows])

    t = col('PosixTime_s')
    order = np.argsort(t, kind='stable')
    t = t[order]
    xy = np.column_stack([col('X_m')[order], col('Y_m')[order]])
    z = col('Z_m')[order] if 'Z_m' in cols else np.full(t.size, np.nan)
    yaw = col('Yaw_deg')[order] if 'Yaw_deg' in cols else np.full(t.size, np.nan)

    # Drop duplicate timestamps: interpolation needs a strictly increasing knot
    # vector, and the exporter occasionally repeats a frame time.
    keep = np.ones(t.size, dtype=bool)
    keep[1:] = np.diff(t) > 0
    t, xy, z, yaw = t[keep], xy[keep], z[keep], yaw[keep]

    span = max(float(t[-1] - t[0]), np.finfo(float).eps) if t.size > 1 else np.finfo(float).eps
    return VisionLog(t=t, xy=xy, z=z, yaw_deg=yaw,
                     rate_hz=float((t.size - 1) / span), file=str(path))


# ── Alignment ─────────────────────────────────────────────────────────────────

@dataclass
class AlignedTruth:
    xy:       np.ndarray                      # [N, 2], NaN where masked
    yaw_deg:  np.ndarray
    valid:    np.ndarray                      # [N] bool
    coverage: float = 0.0


def _interp(t_new: np.ndarray, t: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Linear interpolation that returns NaN outside the knot range."""
    out = np.interp(t_new, t, v, left=np.nan, right=np.nan)
    return out


def align_truth(vision: VisionLog, t_host: np.ndarray, *,
                time_offset: float = 0.0,
                lever: tuple[float, float] = (0.0, 0.0),
                max_gap_s: float = 0.2) -> AlignedTruth:
    """Interpolate the truth trajectory onto the UWB timestamps.

    `lever` is the tag antenna's offset from the AprilTag centre in the plate
    frame; it is rotated by the measured yaw, so it only helps if yaw is real.
    `max_gap_s` masks samples whose nearest actual camera frame is further away
    than that — occlusions and dropouts must become NaN, not interpolated
    fiction spanning the gap.
    """
    t_host = np.asarray(t_host, dtype=float).ravel()
    tv = vision.t + time_offset

    # A run where the camera never got a usable fix is a real outcome, not a
    # programming error: it should score as "no truth", not raise out of the
    # middle of a report.
    if tv.size < 2:
        nan_xy = np.full((t_host.size, 2), np.nan)
        return AlignedTruth(xy=nan_xy, yaw_deg=np.full(t_host.size, np.nan),
                            valid=np.zeros(t_host.size, dtype=bool), coverage=0.0)

    xy = np.column_stack([_interp(t_host, tv, vision.xy[:, 0]),
                          _interp(t_host, tv, vision.xy[:, 1])])

    # Yaw must be interpolated angle-aware or it swings the wrong way through
    # the ±180 wrap.
    yaw_u = np.unwrap(np.deg2rad(vision.yaw_deg))
    yaw = np.rad2deg(_wrap_to_pi(_interp(t_host, tv, yaw_u)))

    # Nearest actual frame time, for the dropout mask.
    idx = np.searchsorted(tv, t_host).clip(1, tv.size - 1)
    left, right = tv[idx - 1], tv[idx]
    nearest = np.where(np.abs(t_host - left) <= np.abs(right - t_host), left, right)
    gap = np.abs(nearest - t_host)

    mask = (gap > max_gap_s) | np.isnan(xy[:, 0])
    xy[mask, :] = np.nan
    yaw[mask] = np.nan

    if lever[0] != 0.0 or lever[1] != 0.0:
        c, s = np.cos(np.deg2rad(yaw)), np.sin(np.deg2rad(yaw))
        xy = xy + np.column_stack([c * lever[0] - s * lever[1],
                                   s * lever[0] + c * lever[1]])

    valid = ~mask
    return AlignedTruth(xy=xy, yaw_deg=yaw, valid=valid,
                        coverage=float(np.mean(valid)) if valid.size else 0.0)


def _wrap_to_pi(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


# ── Metrics ───────────────────────────────────────────────────────────────────

@dataclass
class TrajectoryMetrics:
    n:      int
    rmse:   float
    mean:   float
    median: float
    p95:    float
    max:    float
    err:    np.ndarray = field(default_factory=lambda: np.zeros(0))
    zone:   dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """JSON-safe summary (the per-sample error array is left out)."""
        return {'n': self.n, 'rmse': self.rmse, 'mean': self.mean,
                'median': self.median, 'p95': self.p95, 'max': self.max}


def trajectory_metrics(est_xy: np.ndarray, truth_xy: np.ndarray, *,
                       bounds: tuple = DEFAULT_BOUNDS,
                       cell_m: float = 1.0) -> TrajectoryMetrics:
    """2D error statistics, plus a per-zone RMSE map binned on truth position.

    Rows with NaN in either input are excluded — that is how masked dropouts
    from `align_truth` stay out of the statistics instead of scoring as zero.
    """
    est_xy = np.atleast_2d(np.asarray(est_xy, dtype=float))
    truth_xy = np.atleast_2d(np.asarray(truth_xy, dtype=float))
    if est_xy.shape != truth_xy.shape:
        raise ValueError(f"shape mismatch: {est_xy.shape} vs {truth_xy.shape}")

    d = est_xy - truth_xy
    err = np.hypot(d[:, 0], d[:, 1])
    ok = ~np.isnan(err)
    e = err[ok]

    if e.size == 0:
        return TrajectoryMetrics(0, float('nan'), float('nan'), float('nan'),
                                 float('nan'), float('nan'), err, {})

    xe = _edges(bounds[0], bounds[1], cell_m)
    ye = _edges(bounds[2], bounds[3], cell_m)
    nz = np.full((ye.size - 1, xe.size - 1), np.nan)
    rz = np.full_like(nz, np.nan)

    # np.digitize gives 0 below the first edge and len(edges) above the last;
    # shifting by one makes valid bins 0..n-1 and out-of-range negative or too
    # large, so the membership test below excludes them without extra masking.
    gx = np.digitize(truth_xy[:, 0], xe) - 1
    gy = np.digitize(truth_xy[:, 1], ye) - 1
    for iy in range(nz.shape[0]):
        for ix in range(nz.shape[1]):
            m = ok & (gx == ix) & (gy == iy)
            if np.any(m):
                nz[iy, ix] = int(np.count_nonzero(m))
                rz[iy, ix] = float(np.sqrt(np.mean(err[m] ** 2)))

    return TrajectoryMetrics(
        n=int(e.size),
        rmse=float(np.sqrt(np.mean(e ** 2))),
        mean=float(np.mean(e)),
        median=float(np.median(e)),
        p95=prctile(e, 95),
        max=float(np.max(e)),
        err=err,
        zone={'rmse': rz, 'n': nz, 'x_edges': xe, 'y_edges': ye},
    )


def _edges(lo: float, hi: float, step: float) -> np.ndarray:
    e = np.arange(lo, hi + 1e-9, step)
    if e.size == 0 or e[-1] < hi:
        e = np.append(e, hi)
    return e
