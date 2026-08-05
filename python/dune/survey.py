"""Anchor self-survey: pairwise ranges → constellation geometry.

Ports of ``matlab/scripts/build_survey_anchors.m`` and ``survey_fit.m``.
Changes must be made in both.

The anchors range to each other, and classical multidimensional scaling turns
that distance matrix back into a set of points. This is the **Kinect-free** path
to a geometry: no camera, no clicking, no tape measure.

Two things about the result are easy to misread:

* **The MDS frame is arbitrary.** A set of distances fixes shape but not
  absolute pose or handedness, so the points are pose+reflection aligned onto
  the existing `anchors.json` purely to get a readable frame and to resolve the
  one bit of handedness. That is *not* the same as taking geometry from the
  camera — the rover solve and the run report both re-fit the UWB→world
  transform, so only the anchor **shape** affects accuracy.
* **"Corrected" means delay-removed**, i.e. each measured pair has a per-anchor
  bias subtracted, fitted as ``meas_ij = true_ij + b_i + b_j`` against a
  reference geometry. Useful for seeing how much of the survey error is antenna
  delay, but it needs a reference — so it cannot be the Kinect-free path.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

TICK_M = 0.00469        # DW1000 antenna-delay tick, metres (survey_fit.m)


@dataclass
class SurveyCapture:
    ids:   np.ndarray
    D:     np.ndarray                       # [n, n] pairwise distances (m), NaN = missing
    n_pairs: int = 0
    quality: dict = field(default_factory=dict)   # (i,j) -> reported quality

    @property
    def complete(self) -> bool:
        off = ~np.eye(self.D.shape[0], dtype=bool)
        return bool(np.all(np.isfinite(self.D[off])))

    def missing_pairs(self) -> list[tuple[int, int]]:
        out = []
        n = self.D.shape[0]
        for i in range(n):
            for j in range(i + 1, n):
                if not np.isfinite(self.D[i, j]):
                    out.append((int(self.ids[i]), int(self.ids[j])))
        return out


def read_survey(path: str | Path, ids=None) -> SurveyCapture:
    """Parse a ``SURVEY,v1,<a>,<b>,<mm>,<quality>`` capture."""
    lines = Path(path).read_text(errors='replace').splitlines()
    rows = []
    for line in lines:
        t = [s.strip() for s in line.split(',')]
        if len(t) >= 6 and t[0] == 'SURVEY' and t[1] == 'v1':
            try:
                rows.append((int(t[2]), int(t[3]), float(t[4]) / 1000.0, float(t[5])))
            except ValueError:
                continue
    if not rows:
        raise ValueError(f"{Path(path).name} contains no SURVEY,v1 rows")

    if ids is None:
        ids = sorted({r[0] for r in rows} | {r[1] for r in rows})
    ids = np.asarray(list(ids), dtype=int)
    idx = {int(a): k for k, a in enumerate(ids)}

    n = ids.size
    D = np.full((n, n), np.nan)
    np.fill_diagonal(D, 0.0)
    quality = {}
    for a, b, d, q in rows:
        if a not in idx or b not in idx:
            continue
        i, j = idx[a], idx[b]
        D[i, j] = D[j, i] = d
        quality[(min(a, b), max(a, b))] = q
    return SurveyCapture(ids=ids, D=D, n_pairs=len(rows), quality=quality)


def fit_anchor_delays(D: np.ndarray, truth_xy: np.ndarray):
    """Per-anchor range bias from ``meas_ij = true_ij + b_i + b_j``.

    Port of the fit in `survey_fit.m` / the `corrected` branch of
    `build_survey_anchors.m`. Returns ``(b, b_ticks, residual_rms)``.

    The system is one equation per pair and one unknown per anchor, so with
    n ≥ 4 it is overdetermined and least squares is meaningful. It is also
    rank-deficient in a specific way worth knowing: adding a constant to every
    b and subtracting it from... nothing, cannot be absorbed — but a *common*
    bias shows up fully, so these numbers are only as good as `truth_xy`.
    """
    n = D.shape[0]
    M, rhs = [], []
    for i in range(n):
        for j in range(i + 1, n):
            if not np.isfinite(D[i, j]):
                continue
            row = np.zeros(n)
            row[i] = 1.0
            row[j] = 1.0
            M.append(row)
            true_ij = float(np.linalg.norm(truth_xy[i] - truth_xy[j]))
            rhs.append(D[i, j] - true_ij)
    M = np.asarray(M)
    rhs = np.asarray(rhs)
    b, *_ = np.linalg.lstsq(M, rhs, rcond=None)
    resid = M @ b - rhs
    rms = float(np.sqrt(np.mean(resid ** 2))) if resid.size else float('nan')
    return b, b / TICK_M, rms


def apply_delays(D: np.ndarray, b: np.ndarray) -> np.ndarray:
    out = D.copy()
    n = D.shape[0]
    for i in range(n):
        for j in range(i + 1, n):
            if np.isfinite(out[i, j]):
                c = out[i, j] - (b[i] + b[j])
                out[i, j] = out[j, i] = c
    return out


def mds_2d(D: np.ndarray) -> np.ndarray:
    """Classical MDS: a distance matrix back to planar points."""
    n = D.shape[0]
    J = np.eye(n) - np.ones((n, n)) / n
    B = -0.5 * J @ (D ** 2) @ J
    ev, V = np.linalg.eigh((B + B.T) / 2)
    order = np.argsort(ev)[::-1]
    lam = np.clip(ev[order[:2]], 0.0, None)
    return V[:, order[:2]] * np.sqrt(lam)


def align_to(X: np.ndarray, truth_xy: np.ndarray):
    """Pose+reflection align (NO scale) onto a reference. Returns (Xa, rms).

    Both reflections are tried and the better kept: MDS cannot determine
    handedness from distances alone, so that one bit has to come from somewhere.
    Scale is deliberately not fitted — the survey's job is to measure the shape
    at true size, and letting a scale term absorb error would hide exactly the
    ranging bias this is used to detect.
    """
    best, Xa = np.inf, X
    for flip in (1.0, -1.0):
        Xf = X.copy()
        Xf[:, 1] *= flip
        mx, my = Xf.mean(axis=0), truth_xy.mean(axis=0)
        U, _, Vt = np.linalg.svd((Xf - mx).T @ (truth_xy - my))
        R = Vt.T @ np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
        Xar = (R @ Xf.T).T + (my - (R @ mx.T).T)
        e = float(np.sqrt(np.mean(np.sum((Xar - truth_xy) ** 2, axis=1))))
        if e < best:
            best, Xa = e, Xar
    return Xa, best


def build_survey_anchors(capture: SurveyCapture, truth_xy: np.ndarray, *,
                         corrected: bool = False, z: float = 0.24,
                         layout_base: str = 'survey') -> dict:
    """Reconstruct the constellation and return an `anchors.json` document."""
    if not capture.complete:
        raise ValueError(f"incomplete survey — missing pairs: "
                         f"{capture.missing_pairs()}")

    D = capture.D
    tag = 'raw'
    if corrected:
        b, _, _ = fit_anchor_delays(D, truth_xy)
        D = apply_delays(D, b)
        tag = 'corrected'

    Xa, rms = align_to(mds_2d(D), truth_xy)

    return {
        'dim': 2,
        'bounds': [float(Xa[:, 0].min() - 0.5), float(Xa[:, 0].max() + 0.5),
                   float(Xa[:, 1].min() - 0.5), float(Xa[:, 1].max() + 0.5),
                   0.0, 3.0],
        'anchors': [{'id': int(capture.ids[k]),
                     'x': round(float(Xa[k, 0]), 4),
                     'y': round(float(Xa[k, 1]), 4),
                     'z': z} for k in range(capture.ids.size)],
        'layout': f'self_survey_{tag}_{layout_base}',
        '_rms_vs_reference_mm': round(1000 * rms, 1),
    }
