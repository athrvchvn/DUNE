"""
Last-known-range memory, so every solve sees the same anchor geometry.

Port of ``matlab/+dune/RangeHold.m``.

The problem it solves: when an anchor misses a sweep (scheduler backoff, a lost
frame), the solve silently switches to a different anchor subset. Each subset
averages the per-anchor residual biases differently, so the reported position
*jumps* by 13-30 mm even though the tag never moved. Measured on hardware,
holding the last known range cut that to 6-10 mm at the raw-fix level.

A held range is the **median of the last 5 real measurements**, not the single
most recent one, so a noisy final sample before a dropout does not become the
value that gets held.

Trust in a held value depends on whether the tag is moving:

*   parked  -- the range is still true, so it keeps full weight for up to 15 s
*   moving  -- it decays as ``exp(-(age/tau)^2)`` and is dropped after 3 s

Held entries are appended to the sweep with a ``w_scale`` multiplier rather than
being made indistinguishable from real measurements, so downstream diagnostics
can still tell which anchors actually answered.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

SENTINEL = -2147483648
MIN_HELD_WEIGHT = 0.05   # below this the held value is not worth injecting


@dataclass
class _Entry:
    hist: List[float] = field(default_factory=list)   # last N real ranges
    dist: float = float('nan')                        # median of hist = hold value
    rx:   float = float('nan')
    fp:   float = float('nan')
    t:    float = float('nan')                        # host time of last real measurement


@dataclass
class HeldSweep:
    """A sweep with held entries appended. Mirrors the MATLAB ``s2`` struct."""
    ids:     np.ndarray
    dist:    np.ndarray
    rx:      np.ndarray
    fp:      np.ndarray
    w_scale: np.ndarray
    tag_id:  int = 0
    n_held:  int = 0


class RangeHold:
    """Per-anchor last-known-range cache. One instance per tag."""

    def __init__(self, max_age: float = 3.0, max_age_still: float = 15.0,
                 tau: float = 1.0, hist_n: int = 5):
        self.max_age = max_age
        self.max_age_still = max_age_still
        self.tau = tau
        self.hist_n = hist_n
        self._cache: Dict[int, _Entry] = {}

    def apply(self, sweep, t_host: float, still: bool = False) -> HeldSweep:
        """
        Update the cache from this sweep's real measurements, then append held
        entries for anchors that did not answer.

        sweep    object with ``.ids``, ``.dist``, ``.rx``/``.q``, ``.fp``
        t_host   host clock for this sweep (s); ages are measured against it
        still    whether the tag is currently judged stationary
        """
        ids = np.asarray(sweep.ids, dtype=int).ravel()
        dist = np.asarray(sweep.dist, dtype=float).ravel()
        rx = np.asarray(getattr(sweep, 'rx', None) if getattr(sweep, 'rx', None) is not None
                        else sweep.q, dtype=float).ravel()
        fp_src = getattr(sweep, 'fp', [])
        fp = (np.asarray(fp_src, dtype=float).ravel()
              if len(fp_src) else np.full(ids.size, np.nan))

        for k in range(ids.size):
            r = rx[k] if k < rx.size else np.nan
            f = fp[k] if k < fp.size else np.nan
            if r == SENTINEL or f == SENTINEL:
                continue
            if not np.isfinite(dist[k]) or dist[k] <= 0:
                continue
            e = self._cache.setdefault(int(ids[k]), _Entry())
            e.hist.append(float(dist[k]))
            if len(e.hist) > self.hist_n:
                e.hist = e.hist[-self.hist_n:]
            e.dist = float(np.median(e.hist))   # noise-robust hold value
            e.rx, e.fp, e.t = float(r), float(f), float(t_host)

        out_ids = list(ids)
        out_dist = list(dist)
        out_rx = list(rx) + [np.nan] * max(0, ids.size - rx.size)
        out_fp = list(fp) + [np.nan] * max(0, ids.size - fp.size)
        out_w = [1.0] * ids.size
        n_held = 0

        cap = self.max_age_still if still else self.max_age
        present = set(int(i) for i in ids)
        for aid, e in self._cache.items():
            if aid in present or not np.isfinite(e.t):
                continue
            age = t_host - e.t
            if age > cap:
                continue
            w = 1.0 if still else float(np.exp(-((age / self.tau) ** 2)))
            if w < MIN_HELD_WEIGHT:
                continue
            out_ids.append(aid)
            out_dist.append(e.dist)
            out_rx.append(e.rx)
            out_fp.append(e.fp)
            out_w.append(w)
            n_held += 1

        return HeldSweep(
            ids=np.array(out_ids, dtype=int),
            dist=np.array(out_dist, dtype=float),
            rx=np.array(out_rx, dtype=float),
            fp=np.array(out_fp, dtype=float),
            w_scale=np.array(out_w, dtype=float),
            tag_id=int(getattr(sweep, 'tag_id', getattr(sweep, 'tag', 0)) or 0),
            n_held=n_held,
        )

    def reset(self) -> None:
        self._cache.clear()
