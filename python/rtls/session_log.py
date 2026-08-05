"""Read a JSONL session log into column arrays.

Port of ``matlab/+dune/readSessionLog.m``. Changes must be made in both.

Two log dialects land here and both must work, because the whole point of a
parity pipeline is that either side's recordings are readable by the other:

* **MATLAB** (``live_tag.m`` via ``dune.sweepRecord``) — one JSON object per
  solved sweep, nothing else.
* **Python** (``host.bus.SessionRecorder``) — *every* broadcast message, so the
  file interleaves ``tag_fix`` records with ``config``, ``robot``, ``diag`` and
  ``job_*`` messages. Sweeps are selected by the presence of ``tag_id``, which
  is exactly what MATLAB does, so the same rule covers both.

Three details are load-bearing and were each found by data disagreeing with an
assumption rather than by reading code:

1. ``jsonencode`` in MATLAB collapses a **single-element** struct array to a
   JSON *object*. A sweep with one diagnosed anchor therefore decodes as a dict
   where every other sweep gives a list.
2. ``t_ms`` is the tag's ``millis()`` and restarts every boot, so
   ``(tag_id, t_ms)`` is unique only within one session. Never pool sessions on
   it.
3. Anchor columns are keyed by id, not by position in the sweep. A sweep that
   drops an anchor shifts nothing; the missing column is NaN.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


@dataclass
class SessionLog:
    """Per-sweep columns. ``[N]`` vectors, ``[N, M]`` matrices over anchor ids."""

    anchor_ids: np.ndarray                      # [M] the column order
    tag:    np.ndarray = field(default_factory=lambda: np.zeros(0))
    t_ms:   np.ndarray = field(default_factory=lambda: np.zeros(0))
    t_host: np.ndarray = field(default_factory=lambda: np.zeros(0))
    n_used: np.ndarray = field(default_factory=lambda: np.zeros(0))
    x:      np.ndarray = field(default_factory=lambda: np.zeros(0))
    y:      np.ndarray = field(default_factory=lambda: np.zeros(0))
    ex:     np.ndarray = field(default_factory=lambda: np.zeros(0))
    ey:     np.ndarray = field(default_factory=lambda: np.zeros(0))
    lm_rmse: np.ndarray = field(default_factory=lambda: np.zeros(0))

    R:    np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))  # raw ranges (m)
    D:    np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))  # corrected (m)
    RX:   np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))  # dBm
    FP:   np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))  # dBm
    GAP:  np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))  # dB
    W:    np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    USED: np.ndarray = field(default_factory=lambda: np.zeros((0, 0), bool))

    imu_yaw:   np.ndarray = field(default_factory=lambda: np.zeros(0))
    imu_roll:  np.ndarray = field(default_factory=lambda: np.zeros(0))
    imu_pitch: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gyro_z:    np.ndarray = field(default_factory=lambda: np.zeros(0))

    # Filter state, when the writer recorded it. `live_rover.m` logs `still`;
    # `sweepRecord.m` does not, which is precisely the gap that makes the
    # Phase-2 ex/ey comparison harder than it looks. NaN = not recorded.
    still:     np.ndarray = field(default_factory=lambda: np.zeros(0))

    events: list = field(default_factory=list)
    meta:   dict = field(default_factory=dict)

    def __len__(self) -> int:
        return int(self.tag.size)

    @property
    def tags(self) -> list[int]:
        return sorted(int(t) for t in np.unique(self.tag)) if len(self) else []

    def for_tag(self, tag_id: int) -> "SessionLog":
        """A view restricted to one tag — dual-tag logs interleave both."""
        m = self.tag == tag_id
        out = SessionLog(anchor_ids=self.anchor_ids, events=self.events,
                         meta=dict(self.meta, tag=tag_id))
        for name in ('tag', 't_ms', 't_host', 'n_used', 'x', 'y', 'ex', 'ey',
                     'lm_rmse', 'imu_yaw', 'imu_roll', 'imu_pitch', 'gyro_z',
                     'still', 'R', 'D', 'RX', 'FP', 'GAP', 'W', 'USED'):
            setattr(out, name, getattr(self, name)[m])
        return out

    def column(self, anchor_id: int) -> int:
        """Column index for an anchor id, or -1 when the log never saw it."""
        hit = np.flatnonzero(self.anchor_ids == anchor_id)
        return int(hit[0]) if hit.size else -1


def _iter_records(path: Path) -> Iterable[tuple[dict, bool]]:
    """Yield (record, ok). A truncated final line ends the file, not the read."""
    with open(path, 'r', errors='replace') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except Exception:
                yield {}, False
                continue
            yield (rec, True) if isinstance(rec, dict) else ({}, False)


def _normalise_diag(diag) -> list[dict]:
    """MATLAB collapses a one-element struct array to an object, not a list."""
    if diag is None:
        return []
    if isinstance(diag, dict):
        return [diag]
    if isinstance(diag, list):
        return [d for d in diag if isinstance(d, dict)]
    return []


def discover_anchor_ids(path: str | Path, limit: int | None = None) -> np.ndarray:
    """Every anchor id the log mentions, sorted.

    MATLAB defaults to ``1:5`` and silently drops anything outside it. Reading
    the ids out of the data instead means a six-anchor or renumbered
    constellation does not quietly lose a column.
    """
    ids: set[int] = set()
    for n, (rec, ok) in enumerate(_iter_records(Path(path))):
        if limit is not None and n >= limit:
            break
        if not ok or 'tag_id' not in rec:
            continue
        for aid in rec.get('anchor_ids', []) or []:
            ids.add(int(aid))
        for d in _normalise_diag(rec.get('anchor_diag')):
            if 'id' in d:
                ids.add(int(d['id']))
        for key in rec:
            if len(key) > 1 and key[0] in 'rd' and key[1:].isdigit():
                ids.add(int(key[1:]))
    return np.array(sorted(ids), dtype=int)


def read_session_log(path: str | Path,
                     anchor_ids: Sequence[int] | None = None) -> SessionLog:
    """Read one JSONL session log into column arrays.

    `anchor_ids` fixes the column order; by default it is discovered from the
    file so no anchor is dropped.
    """
    path = Path(path)
    ids = (np.asarray(list(anchor_ids), dtype=int)
           if anchor_ids is not None else discover_anchor_ids(path))
    col = {int(a): i for i, a in enumerate(ids)}
    M = len(ids)

    rows: list[dict] = []
    events: list[dict] = []
    session_meta: dict | None = None
    n_bad = 0

    for rec, ok in _iter_records(path):
        if not ok:
            n_bad += 1
            continue
        if rec.get('type') == 'session_meta':
            # The settings the writer actually ran under. Before this existed,
            # replaying a log meant guessing them — see live_tag.m for why that
            # was expensive. Captured into meta so a consumer can configure a
            # replay from the file instead of from assumptions.
            session_meta = rec
            continue
        if 'tag_id' not in rec:
            # Non-sweep record: device events from MATLAB, or any of the other
            # broadcast families in a Python session log. Kept, not discarded —
            # a boot event mid-run explains a lot about the sweeps around it.
            if rec.get('type') in (None, 'event') or 'line' in rec:
                events.append({'t_host': rec.get('t_host'),
                               'src':    rec.get('src', ''),
                               'line':   rec.get('line', '')})
            continue
        rows.append(rec)

    N = len(rows)
    out = SessionLog(anchor_ids=ids)
    vec = lambda: np.full(N, np.nan)                              # noqa: E731
    mat = lambda: np.full((N, M), np.nan)                         # noqa: E731

    out.tag, out.t_ms, out.t_host = vec(), vec(), vec()
    out.n_used, out.x, out.y, out.ex, out.ey = vec(), vec(), vec(), vec(), vec()
    out.lm_rmse = vec()
    out.R, out.D, out.RX, out.FP, out.GAP, out.W = (mat(), mat(), mat(),
                                                    mat(), mat(), mat())
    out.USED = np.zeros((N, M), dtype=bool)
    out.imu_yaw, out.imu_roll = vec(), vec()
    out.imu_pitch, out.gyro_z = vec(), vec()
    out.still = vec()

    for i, rec in enumerate(rows):
        out.tag[i]  = rec['tag_id']
        out.t_ms[i] = rec.get('t_ms', np.nan)
        for src, dst in (('t_host', out.t_host), ('nUsed', out.n_used),
                         ('x', out.x), ('y', out.y), ('ex', out.ex),
                         ('ey', out.ey), ('rmse', out.lm_rmse)):
            if src in rec and rec[src] is not None:
                dst[i] = rec[src]

        for aid, c in col.items():
            rv = rec.get(f'r{aid}')
            dv = rec.get(f'd{aid}')
            if rv is not None:
                out.R[i, c] = rv
            if dv is not None:
                out.D[i, c] = dv

        for d in _normalise_diag(rec.get('anchor_diag')):
            c = col.get(int(d.get('id', -1)), None)
            if c is None:
                continue
            for src, dst in (('rx', out.RX), ('fp', out.FP),
                             ('gap', out.GAP), ('w', out.W)):
                if d.get(src) is not None:
                    dst[i, c] = d[src]
            # Python emits the corrected range inside the diag as well; take it
            # when the flat d<id> field is absent so both dialects fill D.
            if d.get('d') is not None and np.isnan(out.D[i, c]):
                out.D[i, c] = d['d']
            out.USED[i, c] = bool(d.get('used', False))

        imu = rec.get('imu')
        if isinstance(imu, dict):
            out.imu_yaw[i]   = imu.get('yaw', np.nan)
            out.imu_roll[i]  = imu.get('roll', np.nan)
            out.imu_pitch[i] = imu.get('pitch', np.nan)
        gyro = rec.get('imu_gyro')
        if isinstance(gyro, (list, tuple)) and len(gyro) == 3:
            out.gyro_z[i] = gyro[2]
        if rec.get('still') is not None:
            out.still[i] = float(bool(rec['still']))

    # GAP is the NLOS feature and both dialects sometimes omit it while still
    # logging rx and fp. Deriving it here keeps every consumer from having to.
    missing = np.isnan(out.GAP) & ~np.isnan(out.RX) & ~np.isnan(out.FP)
    out.GAP[missing] = (out.RX - out.FP)[missing]

    has_thost = bool(np.any(np.isfinite(out.t_host)))
    if has_thost:
        duration = float(np.nanmax(out.t_host) - np.nanmin(out.t_host))
    elif N:
        duration = float(np.nanmax(out.t_ms) - np.nanmin(out.t_ms)) / 1000.0
    else:
        duration = 0.0

    out.events = events
    out.meta = {
        'file':        str(path),
        'anchor_ids':  ids.tolist(),
        'tags':        out.tags,
        'n_records':   N,
        'n_bad_lines': n_bad,
        'has_thost':   has_thost,
        'duration_s':  duration,
        'rate_hz':     round(N / duration, 3) if duration > 0 else 0.0,
        # Which optional columns this file actually carries. Consumers should
        # branch on these rather than on the filename: three writers produce
        # this format (sweepRecord, live_rover, SessionRecorder) and they log
        # different subsets.
        'has_ranges':  bool(M and np.any(np.isfinite(out.R))),
        'has_ekf_out': bool(np.any(np.isfinite(out.ex))),
        'has_still':   bool(np.any(np.isfinite(out.still))),
        # None for every log written before live_tag started recording it.
        'session_meta': session_meta,
        'has_meta':     session_meta is not None,
    }
    return out
