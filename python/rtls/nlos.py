"""
NLOS soft weighting from the rx - first-path power gap.

Port of ``matlab/+dune/gapWeights.m``.

A direct line-of-sight arrival puts most energy in the first path, so total RX
power and first-path power are close. An obstruction spreads energy into later
multipath, widening the gap -- which makes ``rx - fp`` a usable per-anchor NLOS
score. This is only meaningful because the firmware reads both powers from the
*same* received frame (Phase 2.1); an earlier version compared two different
link directions, which was physically meaningless.

Soft weighting only: a suspect anchor is de-emphasised, never dropped. Hard
outliers are the MAD gate's job in ``multilaterate``, and keeping the two
independent stops either from masking the other.
"""

import numpy as np

DEFAULT_THRESH_DB = 3.0    # gap below this reads as line-of-sight
DEFAULT_FLOOR_W = 0.05     # a down-weighted anchor still contributes geometry


def gap_weights(
    gap_db,
    thresh_db: float = DEFAULT_THRESH_DB,
    floor_w: float = DEFAULT_FLOOR_W,
) -> np.ndarray:
    """
    Weights from the power gap (dB): unity up to ``thresh_db``, then
    ``10^(-excess/10)``, floored at ``floor_w``.

    A NaN gap means the firmware reported no diagnostics for that anchor, which
    is not evidence of NLOS -- those get weight 1 rather than being penalised.
    """
    gap = np.asarray(gap_db, dtype=float)
    excess = np.maximum(0.0, gap - thresh_db)
    w = np.maximum(floor_w, 10.0 ** (-excess / 10.0))
    return np.where(np.isnan(gap), 1.0, w)
