"""
One ranging sweep -> tag x,y. The single solve path, shared by live and replay.

Port of ``matlab/+dune/solveSweep.m``.

The point of routing everything through here is that a bug found in replay is
the same bug that was live. The previous Python pipeline inlined this chain
inside ``RtlsPipeline.process()``, so offline analysis and the live path could
(and did) drift apart.

The sweep is protocol-agnostic: anything producing ``(anchor id, range, rx, fp)``
tuples for one instant can feed it, so swapping the round-robin schedule for
broadcast-POLL later is a firmware change only.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from .multilaterate import SolveInfo, multilaterate
from .nlos import DEFAULT_FLOOR_W, DEFAULT_THRESH_DB, gap_weights

# DW1000 diagnostic-read failure. The driver returns INT32_MIN rather than a
# power, and it has been observed on a specific anchor -- treating it as a real
# -2147483648 dBm would poison the NLOS gap and the weights.
SENTINEL = -2147483648


@dataclass
class SweepSolution:
    """Per-anchor rows are aligned to ``anchors.ids``, not to the sweep order."""
    pos:        np.ndarray = field(default_factory=lambda: np.array([np.nan, np.nan]))
    info:       Optional[SolveInfo] = None
    range_raw:  np.ndarray = field(default_factory=lambda: np.array([]))
    range_corr: np.ndarray = field(default_factory=lambda: np.array([]))
    rx:         np.ndarray = field(default_factory=lambda: np.array([]))
    fp:         np.ndarray = field(default_factory=lambda: np.array([]))
    gap:        np.ndarray = field(default_factory=lambda: np.array([]))
    w:          np.ndarray = field(default_factory=lambda: np.array([]))
    rejected:   np.ndarray = field(default_factory=lambda: np.array([], dtype=bool))

    @property
    def ok(self) -> bool:
        return bool(np.all(np.isfinite(self.pos)))


def solve_sweep(
    sweep,
    anchors,
    *,
    bias=None,
    range_corr=None,
    tag_z: float = 0.22,
    x0: Optional[np.ndarray] = None,
    use_gap_weights: bool = True,
    gap_thresh_db: float = DEFAULT_THRESH_DB,
    gap_floor_w: float = DEFAULT_FLOOR_W,
    gate_k: float = 3.0,
    w_scale: Optional[Sequence[float]] = None,
    nlos_k_bias: float = 0.0,
) -> SweepSolution:
    """
    sweep       object with ``.ids``, ``.dist`` (m), ``.rx``, ``.fp`` and
                optionally ``.tag_id`` -- a ``SweepPacket`` works directly.
    anchors     ``AnchorConfig``-like: ``.ids`` (M,) and ``.coords`` (M,3).
    bias        optional ``AnchorBias``; OFF by default, because the boards' NVS
                antenna delays are the source of truth and subtracting
                anchor_bias.json on top of them double-corrects.
    range_corr  optional ``RangeCorrection`` (Tier-2 power model).
    w_scale     per-sweep-entry weight multipliers, e.g. from RangeHold decay.
    nlos_k_bias m per dB of NLOS excess to subtract as an over-range correction.
    """
    ids = np.asarray(anchors.ids, dtype=int).ravel()
    apos = np.asarray(anchors.coords, dtype=float)
    m = ids.size

    rng = np.full(m, np.nan)
    rx = np.full(m, np.nan)
    fp = np.full(m, np.nan)
    rejected = np.zeros(m, dtype=bool)
    wscale = np.ones(m)

    s_ids = np.asarray(sweep.ids, dtype=int).ravel()
    s_dist = np.asarray(sweep.dist, dtype=float).ravel()
    s_rx = np.asarray(sweep.rx if hasattr(sweep, 'rx') else sweep.q, dtype=float).ravel()
    s_fp = np.asarray(sweep.fp, dtype=float).ravel() if len(getattr(sweep, 'fp', [])) else np.full(s_ids.size, np.nan)

    for k in range(s_ids.size):
        hit = np.flatnonzero(ids == s_ids[k])
        if hit.size == 0:
            continue                      # anchor answered but is not in the layout
        c = int(hit[0])
        if w_scale is not None and k < len(w_scale):
            wscale[c] = float(w_scale[k])
        rx[c] = s_rx[k] if k < s_rx.size else np.nan
        fp[c] = s_fp[k] if k < s_fp.size else np.nan
        bad = (
            (k < s_rx.size and s_rx[k] == SENTINEL)
            or (k < s_fp.size and s_fp[k] == SENTINEL)
            or not np.isfinite(s_dist[k])
            or s_dist[k] <= 0
        )
        if bad:
            rejected[c] = True            # range stays NaN -> excluded from the solve
            continue
        rng[c] = s_dist[k]

    corr = rng.copy()

    # Tier-2: per-anchor power bias.
    if range_corr is not None:
        for c in range(m):
            if np.isfinite(corr[c]) and np.isfinite(fp[c]):
                corr[c] -= range_corr.correction_mm(int(ids[c]), float(fp[c])) / 1000.0

    # Optional host-side additive device biases (default off, see docstring).
    if bias is not None:
        for c in range(m):
            j = np.flatnonzero(np.asarray(bias.anchor_ids, dtype=int) == ids[c])
            if j.size:
                corr[c] -= float(np.asarray(bias.anchor_bias)[int(j[0])])
        tag_id = getattr(sweep, 'tag_id', getattr(sweep, 'tag', None))
        if tag_id is not None and len(getattr(bias, 'tag_ids', [])):
            j = np.flatnonzero(np.asarray(bias.tag_ids, dtype=int) == int(tag_id))
            if j.size:
                tb = float(np.asarray(bias.tag_bias)[int(j[0])])
                if np.isfinite(tb):
                    corr -= tb

    gap = rx - fp
    w = gap_weights(gap, gap_thresh_db, gap_floor_w) if use_gap_weights else np.ones(m)
    w = w * wscale

    # NLOS over-range correction: an obstructed path reads long, roughly in
    # proportion to how far the gap exceeds the LOS threshold. Off by default
    # (k=0) -- it needs per-site calibration before it helps.
    if nlos_k_bias:
        excess = np.maximum(0.0, gap - gap_thresh_db)
        corr = np.where(np.isfinite(corr), corr - nlos_k_bias * np.nan_to_num(excess), corr)

    pos, info = multilaterate(
        apos, corr, weights=w, tag_z=tag_z, x0=x0, gate_k=gate_k
    )
    return SweepSolution(
        pos=pos, info=info, range_raw=rng, range_corr=corr,
        rx=rx, fp=fp, gap=gap, w=w, rejected=rejected,
    )
