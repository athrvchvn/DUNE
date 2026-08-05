"""Per-tag antenna bias from a static capture.

Port of ``matlab/scripts/tag_bias_fit.m``. Changes must be made in both.

For a tag that is not moving, the position and the tag's *own* additive range
bias are estimated jointly:

    median_range_i − b_anchor_i = ‖p − a_i‖ + b_tag

solved by alternating — multilaterate `p` with the current `b_tag` removed, then
move `b_tag` by the weighted mean residual. It converges in a handful of
iterations because a common-mode offset maps directly onto `b_tag`.

The thing to understand before trusting the number: `b_tag` and the anchors'
*common* bias are not separable from ranges alone. Whatever offset every anchor
shares is indistinguishable from a tag bias, so this attributes all of it to the
tag. That is the right convention when the anchors have already been calibrated
against truth — and misleading if they have not.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, field

import numpy as np

from .multilaterate import multilaterate
from .nlos import gap_weights


@dataclass
class TagBiasResult:
    tag:       int
    pos:       np.ndarray
    bias_m:    float
    n_sweeps:  int
    resid_mm:  np.ndarray = field(default_factory=lambda: np.zeros(0))
    med_gap:   np.ndarray = field(default_factory=lambda: np.zeros(0))
    std_mm:    np.ndarray = field(default_factory=lambda: np.zeros(0))
    med_raw:   np.ndarray = field(default_factory=lambda: np.zeros(0))
    n_hit:     np.ndarray = field(default_factory=lambda: np.zeros(0, int))
    iters:     int = 0

    def to_dict(self) -> dict:
        return {
            'tag': int(self.tag),
            'pos': [float(self.pos[0]), float(self.pos[1])],
            'bias_mm': round(1000 * self.bias_m, 2),
            'n_sweeps': int(self.n_sweeps),
            'iters': int(self.iters),
            'resid_mm': [round(float(v), 2) for v in self.resid_mm],
            'med_gap_db': [round(float(v), 2) for v in self.med_gap],
            'std_mm': [round(float(v), 2) for v in self.std_mm],
            'med_raw_m': [round(float(v), 4) for v in self.med_raw],
            'n_hit': [int(v) for v in self.n_hit],
        }


def fit_tag_bias(sweeps, anchors, *, anchor_bias=None, tag_z: float = 0.22,
                 max_iter: int = 30, tol: float = 1e-5) -> list[TagBiasResult]:
    """Joint (position, tag bias) fit per tag over a static capture.

    `sweeps` is any iterable of parsed packets with `.tag_id`, `.ids`, `.dist`,
    `.rx`, `.fp`. `anchor_bias` maps anchor id → metres to subtract.
    """
    ids = np.asarray(anchors.ids, dtype=int)
    n = ids.size
    col = {int(a): k for k, a in enumerate(ids)}

    by_tag: dict[int, list] = {}
    for s in sweeps:
        by_tag.setdefault(int(s.tag_id), []).append(s)

    ab = np.zeros(n)
    for aid, b in (anchor_bias or {}).items():
        if int(aid) in col:
            ab[col[int(aid)]] = float(b)

    out: list[TagBiasResult] = []
    for tag_id in sorted(by_tag):
        S = by_tag[tag_id]
        N = len(S)
        R = np.full((N, n), np.nan)
        GAP = np.full((N, n), np.nan)
        for k, s in enumerate(S):
            s_ids = np.asarray(s.ids, dtype=int)
            s_d = np.asarray(s.dist, dtype=float)
            s_rx = np.asarray(getattr(s, 'rx', getattr(s, 'q', [])), dtype=float)
            s_fp = np.asarray(getattr(s, 'fp', []), dtype=float)
            for m, aid in enumerate(s_ids):
                c = col.get(int(aid))
                if c is None:
                    continue
                R[k, c] = s_d[m]
                if m < s_rx.size and m < s_fp.size:
                    GAP[k, c] = s_rx[m] - s_fp[m]

        # An anchor that never answered gives an all-NaN column, and one seen
        # once has no defined spread. Both are ordinary outcomes that MATLAB
        # handles silently, so the NaN they produce is the answer — not a
        # condition to warn about on every run. (errstate does not cover these:
        # they are RuntimeWarnings from the nan-functions, not FP errors.)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            med_r = np.nanmedian(R, axis=0)
            # ddof=1 matches MATLAB's std() default.
            std_r = np.nanstd(R, axis=0, ddof=1)
            med_gap = np.nanmedian(GAP, axis=0)
        n_hit = np.sum(~np.isnan(R), axis=0)

        corr = med_r - ab
        w = gap_weights(med_gap)

        b_tag = 0.0
        p = None
        it = 0
        for it in range(1, max_iter + 1):
            # gate_k=0 disables the MAD gate: with one median range per anchor
            # there is nothing to outlier-reject against, and gating here would
            # silently drop an anchor whose bias is exactly what we are measuring.
            p, info = multilaterate(anchors.coords, corr - b_tag, weights=w,
                                    tag_z=tag_z, x0=p, gate_k=0.0)
            if not np.all(np.isfinite(p)):
                break
            used = np.asarray(info.used, dtype=bool)
            if not np.any(used) or not np.any(w[used] > 0):
                break
            # resid = predicted − measured, so a range that reads long pushes
            # b_tag up.
            step = float(np.sum(w[used] * info.resid[used]) / np.sum(w[used]))
            b_tag -= step
            if abs(step) < tol:
                break

        if p is None or not np.all(np.isfinite(p)):
            out.append(TagBiasResult(tag=tag_id, pos=np.array([np.nan, np.nan]),
                                     bias_m=float('nan'), n_sweeps=N, iters=it))
            continue

        pred = np.sqrt(np.sum((anchors.coords[:, :2] - p) ** 2, axis=1)
                       + (anchors.coords[:, 2] - tag_z) ** 2)
        resid_mm = 1000.0 * (corr - b_tag - pred)

        out.append(TagBiasResult(
            tag=tag_id, pos=p, bias_m=b_tag, n_sweeps=N, iters=it,
            resid_mm=resid_mm, med_gap=med_gap, std_mm=1000.0 * std_r,
            med_raw=med_r, n_hit=n_hit))
    return out


def load_anchor_bias(path=None) -> dict:
    """`anchor_bias.json` → {anchor_id: bias_m}. Port of loadAnchorBias.m.

    Returns {} when absent, which is the correct default: the boards' NVS
    antenna delays are the source of truth, and subtracting this file on top of
    a rig that has already been delay-calibrated double-corrects. It belongs with
    the geometry it was fitted against — the committed file names the layout in
    its own `note` field.
    """
    import json
    from pathlib import Path

    if path is None:
        path = Path(__file__).resolve().parents[2] / 'matlab' / 'config' / 'anchor_bias.json'
    path = Path(path)
    if not path.exists():
        return {}
    doc = json.loads(path.read_text())
    return {int(a['id']): float(a.get('bias_m', 0.0))
            for a in doc.get('anchors', []) if 'id' in a}
