"""
Weighted Levenberg-Marquardt range multilateration (2D, fixed tag height).

Faithful port of ``matlab/+dune/multilaterate.m``. The two implementations are
checked against each other on recorded hardware sessions (see
``python/tests/test_equivalence.py``), so **any change here must be mirrored in
the MATLAB file and vice versa** or the cross-check will start failing.

Two details differ from the older ``rtls.multilaterator.Multilaterator`` and are
deliberate, not oversights:

*   The solve is 2D with the anchor/tag height difference folded into the
    residual (``dz2``), rather than a generic ``dim``-dimensional solve. Anchors
    and tag sit on near-parallel planes, so the vertical component is known and
    solving for it only injects noise.
*   Weights enter as ``JᵀW²J`` (applied per Jacobian row, then squared), which
    is what the MATLAB code does. ``Multilaterator`` used ``JᵀWJ``. Mixing the
    conventions changes the covariance the EKF is handed, so they are kept apart.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


# Require at least this much anchor spread (m) off the principal line before
# attempting a 2D fix. Near-collinear anchors (e.g. only the three top-edge ones
# answering) give a mirror ambiguity and a singular normal matrix: the geometry
# is genuinely ambiguous, so refusing to answer beats emitting a confident lie.
MIN_PERP_SPREAD_M = 0.3

# The MAD gate never trims residuals below this (m). Without a floor, a sweep
# where every anchor agrees to a few mm produces a tiny MAD and the gate starts
# discarding perfectly good ranges.
MAD_FLOOR_M = 0.02


@dataclass
class SolveInfo:
    """Per-anchor diagnostics, aligned to the input ``anchor_pos`` rows."""
    resid: np.ndarray = field(default_factory=lambda: np.array([]))   # signed pred-meas, NaN where unused
    used:  np.ndarray = field(default_factory=lambda: np.array([], dtype=bool))
    rmse:  float = float('nan')      # weighted residual RMSE of the final solve
    iters: int = 0
    cov:   np.ndarray = field(default_factory=lambda: np.full((2, 2), np.nan))


def multilaterate(
    anchor_pos: np.ndarray,
    ranges: np.ndarray,
    weights: Optional[np.ndarray] = None,
    tag_z: float = 0.22,
    x0: Optional[np.ndarray] = None,
    gate_k: float = 3.0,
    max_iter: int = 50,
) -> Tuple[np.ndarray, SolveInfo]:
    """
    Solve for tag x,y from ranges to anchors at known positions.

    anchor_pos  (M,3) anchor antenna positions, m
    ranges      (M,)  measured ranges, m; NaN marks an anchor absent this sweep
    weights     (M,)  per-anchor weights, default all ones
    tag_z       fixed tag antenna height, m
    x0          (2,)  warm start; default is the weighted anchor centroid
    gate_k      MAD outlier gate factor; 0 disables the re-solve

    Returns ``(pos, info)`` where pos is (2,) and is ``[nan, nan]`` when there
    are fewer than 3 usable ranges, the geometry is degenerate, or LM diverged.
    """
    anchor_pos = np.asarray(anchor_pos, dtype=float)
    ranges = np.asarray(ranges, dtype=float).ravel()
    m = ranges.size
    w_all = np.ones(m) if weights is None else np.asarray(weights, dtype=float).ravel()

    info = SolveInfo(
        resid=np.full(m, np.nan),
        used=np.zeros(m, dtype=bool),
    )
    pos = np.array([np.nan, np.nan])

    sel = ~np.isnan(ranges) & (w_all > 0)
    if np.count_nonzero(sel) < 3:
        return pos, info

    xy = anchor_pos[sel, :2]
    sv = np.linalg.svd(xy - xy.mean(axis=0), compute_uv=False)
    if sv.size < 2 or sv[1] < MIN_PERP_SPREAD_M:
        return pos, info

    p, r, it, cov = _lm_solve(
        anchor_pos[sel], ranges[sel], w_all[sel], tag_z, x0, max_iter
    )
    used = sel.copy()

    # Optional MAD gate, then re-solve on the survivors. Deliberately unweighted
    # so the gate (gross outliers) and the NLOS weights (soft de-emphasis) stay
    # independent defences -- one must not mask the other.
    if gate_k > 0 and np.count_nonzero(sel) > 3 and np.all(np.isfinite(p)):
        mad_sig = 1.4826 * np.median(np.abs(r - np.median(r)))
        keep = np.abs(r) <= max(gate_k * mad_sig, MAD_FLOOR_M)
        if (~keep).any() and np.count_nonzero(keep) >= 3:
            idx = np.flatnonzero(sel)
            used = np.zeros(m, dtype=bool)
            used[idx[keep]] = True
            p, r, it, cov = _lm_solve(
                anchor_pos[used], ranges[used], w_all[used], tag_z, p, max_iter
            )

    if np.all(np.isfinite(p)):
        pos = p
        info.resid[used] = r
        info.used = used
        w = w_all[used]
        info.rmse = float(np.sqrt(np.sum((w * r) ** 2) / np.sum(w ** 2)))
        info.iters = it
        info.cov = cov
    return pos, info


def _lm_solve(a, d, w, tag_z, x0, max_iter):
    """Minimise sum_i (w_i * (||p - a_i|| - d_i))^2 over p = [x, y]."""
    dz2 = (a[:, 2] - tag_z) ** 2

    if x0 is None or not np.all(np.isfinite(np.asarray(x0, dtype=float).ravel()[:2])):
        p = (a[:, :2] * w[:, None]).sum(axis=0) / w.sum()
    else:
        p = np.asarray(x0, dtype=float).ravel()[:2].copy()

    lam = 1e-3
    r, jac = _resid_jac(p, a, d, dz2)
    cost = np.sum((w * r) ** 2)
    it = 0
    for it in range(1, max_iter + 1):
        wj = jac * w[:, None]
        h = wj.T @ wj
        g = wj.T @ (w * r)
        try:
            step = -np.linalg.solve(h + lam * np.diag(np.diag(h)), g)
        except np.linalg.LinAlgError:
            # Singular even with damping: stop and let the caller see whatever
            # the last accepted iterate was, rather than propagating upward.
            break
        p_new = p + step
        r_new, jac_new = _resid_jac(p_new, a, d, dz2)
        cost_new = np.sum((w * r_new) ** 2)
        if cost_new < cost:
            p, r, jac, cost = p_new, r_new, jac_new, cost_new
            lam = max(lam / 3.0, 1e-9)
            if np.linalg.norm(step) < 1e-6:
                break
        else:
            lam *= 5.0
            if lam > 1e6:
                break

    dof = max(d.size - 2, 1)
    sigma2 = cost / dof
    wj = jac * w[:, None]
    hf = wj.T @ wj
    # rcond is 1/cond; MATLAB's guard is rcond > 1e-12.
    cond = np.linalg.cond(hf)
    if np.isfinite(cond) and cond > 0 and (1.0 / cond) > 1e-12:
        cov = sigma2 * np.linalg.inv(hf)
    else:
        cov = np.full((2, 2), np.nan)
    return p, r, it, cov


def _resid_jac(p, a, d, dz2):
    dx = p[0] - a[:, 0]
    dy = p[1] - a[:, 1]
    pred = np.sqrt(dx ** 2 + dy ** 2 + dz2)
    pred = np.maximum(pred, 1e-6)
    return pred - d, np.column_stack((dx / pred, dy / pred))
