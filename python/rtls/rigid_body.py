"""
rigid_body.py — authoritative rigid-body (robot) pose from the two rigidly
mounted tags (master 0xF0 + slave 0xF1, fixed ~0.50 m baseline).

The tags are localized independently every TDMA cycle; this module pairs the
two per-tag fixes back into one body pose and emits a typed ``robot`` message:

    {type:'robot', cycle, cx, cy, heading_deg, baseline_m, baseline_err_m,
     quality, both_present, vx, vy, speed, ts, master_id, slave_id}

Pairing strategy (robust to missing metadata):
  * prefer the firmware ``cycle`` id (both fixes of a TDMA cycle carry it);
  * fall back to a wall-clock window (``pair_window_s``) when cycle is absent
    (solo tag, old firmware, replayed single-tag logs).

Degraded mode is first-class: when only one tag is alive (partner stale) a
robot message is still emitted every fix with ``both_present=False``, the
heading **held** at its last paired value, and the body centre projected from
the present tag along the held baseline direction — the UI renders a "ghost
body" instead of blinking.

Conventions:
  * master = lowest tag_id seen (0xF0 < 0xF1 on this rig); override via ctor.
  * heading = atan2 of the master→slave vector, i.e. the baseline direction.
    Apply any body-frame mounting offset host-side later if the baseline is
    not the body X axis.
"""

from __future__ import annotations

import math
import time
from typing import Optional


class RigidBodyTracker:
    def __init__(self,
                 baseline_nominal_m: float = 0.50,
                 pair_window_s: float = 0.30,
                 stale_after_s: float = 1.0,
                 master_id: Optional[int] = None,
                 slave_id: Optional[int] = None):
        self.baseline_nominal_m = float(baseline_nominal_m)
        self.pair_window_s = float(pair_window_s)
        self.stale_after_s = float(stale_after_s)
        self._forced_master = master_id
        self._forced_slave = slave_id

        self._last: dict[int, tuple[dict, float]] = {}  # tag_id -> (fix, wall_ts)
        self._first_seen: dict[int, float] = {}         # tag_id -> first-fix wall_ts
        self._last_pair_key = None      # last emitted pair key (cycle or ts bucket)
        self._held_heading: Optional[float] = None  # deg, survives degraded mode

    # ── role assignment ──────────────────────────────────────────────────────

    @property
    def master_id(self) -> Optional[int]:
        if self._forced_master is not None:
            return self._forced_master
        return min(self._last.keys()) if self._last else None

    @property
    def slave_id(self) -> Optional[int]:
        if self._forced_slave is not None:
            return self._forced_slave
        ids = sorted(self._last.keys())
        return ids[1] if len(ids) >= 2 else None

    def roles(self) -> list[dict]:
        """[{id, role}] for the config message (empty until tags are seen)."""
        out = []
        if self.master_id is not None:
            out.append({'id': self.master_id, 'role': 'master'})
        if self.slave_id is not None:
            out.append({'id': self.slave_id, 'role': 'slave'})
        return out

    # ── main entry ───────────────────────────────────────────────────────────

    def add_fix(self, fix: dict) -> Optional[dict]:
        """Feed one per-tag fix (the result dict). Returns a robot message to
        broadcast, or None (e.g. first half of a pair — wait for the partner).
        """
        now = time.time()
        tid = fix.get('tag_id')
        if tid is None:
            return None
        self._last[tid] = (fix, now)
        self._first_seen.setdefault(tid, now)

        other = self._partner_of(tid)
        if other is None:
            # Only one tag ever seen. Grace period first: on a dual rig the
            # partner's first fix lands ~100 ms later — don't emit a spurious
            # degraded message during discovery. A true solo rig starts
            # emitting degraded poses after stale_after_s.
            if now - self._first_seen[tid] < self.stale_after_s:
                return None
            return self._degraded(fix, now)

        ofix, ots = self._last[other]

        if now - ots > self.stale_after_s:
            # Partner is stale/dead → degraded ghost-body message.
            return self._degraded(fix, now)

        if not self._compatible(fix, ofix, now, ots):
            # Partner's buffered fix is from a different cycle and too old to
            # time-pair; wait for its update (it will complete the pair).
            return None

        # Pair completed by THIS fix. Emit once per pair.
        key = self._pair_key(fix, ofix, now)
        if key == self._last_pair_key:
            return None
        self._last_pair_key = key
        return self._paired(fix, ofix, now)

    # ── internals ────────────────────────────────────────────────────────────

    def _partner_of(self, tid: int) -> Optional[int]:
        m, s = self.master_id, self.slave_id
        if m is None or s is None:
            return None
        if tid == m:
            return s
        if tid == s:
            return m
        return None  # third tag: not part of this body

    @staticmethod
    def _cycle(fix: dict):
        return fix.get('cycle')

    def _compatible(self, a: dict, b: dict, ta: float, tb: float) -> bool:
        ca, cb = self._cycle(a), self._cycle(b)
        if ca is not None and cb is not None:
            return ca == cb
        return abs(ta - tb) <= self.pair_window_s

    def _pair_key(self, a: dict, b: dict, now: float):
        ca, cb = self._cycle(a), self._cycle(b)
        if ca is not None and ca == cb:
            return ('cyc', ca)
        return ('ts', round(now, 1))

    def _tag_xy(self, fix: dict):
        return float(fix['ex']), float(fix['ey'])

    def _paired(self, fix: dict, ofix: dict, now: float) -> dict:
        m_id = self.master_id
        mfix = fix if fix['tag_id'] == m_id else ofix
        sfix = ofix if fix['tag_id'] == m_id else fix

        mx, my = self._tag_xy(mfix)
        sx, sy = self._tag_xy(sfix)
        dx, dy = sx - mx, sy - my
        baseline = math.hypot(dx, dy)
        heading = math.degrees(math.atan2(dy, dx))
        self._held_heading = heading

        err = abs(baseline - self.baseline_nominal_m)
        rms = 0.5 * (float(mfix.get('rmse', 0)) + float(sfix.get('rmse', 0)))
        # Quality heuristic in [0,1]: penalize baseline error (10 cm → 0) and
        # solver rms (15 cm → 0); both tags present keeps the ceiling at 1.
        quality = max(0.0, 1.0 - err / 0.10) * max(0.0, 1.0 - rms / 0.15)

        vx = 0.5 * (float(mfix.get('vx', 0)) + float(sfix.get('vx', 0)))
        vy = 0.5 * (float(mfix.get('vy', 0)) + float(sfix.get('vy', 0)))

        return {
            'type': 'robot',
            'cycle': self._cycle(mfix) if self._cycle(mfix) is not None else self._cycle(sfix),
            'cx': round((mx + sx) / 2, 4),
            'cy': round((my + sy) / 2, 4),
            'heading_deg': round(heading, 2),
            'baseline_m': round(baseline, 4),
            'baseline_err_m': round(err, 4),
            'quality': round(quality, 3),
            'both_present': True,
            'vx': round(vx, 4), 'vy': round(vy, 4),
            'speed': round(math.hypot(vx, vy), 4),
            'ts': round(now, 3),
            'master_id': m_id, 'slave_id': self.slave_id,
        }

    def _degraded(self, fix: dict, now: float) -> dict:
        """Single-tag robot message: heading held, centre projected if possible."""
        x, y = self._tag_xy(fix)
        cx, cy = x, y
        if self._held_heading is not None and self.slave_id is not None:
            # Project the body centre half a baseline along the held direction:
            # master sits at heading-origin end, slave at the far end.
            h = math.radians(self._held_heading)
            half = self.baseline_nominal_m / 2.0
            sign = +1.0 if fix['tag_id'] == self.master_id else -1.0
            cx = x + sign * half * math.cos(h)
            cy = y + sign * half * math.sin(h)

        return {
            'type': 'robot',
            'cycle': self._cycle(fix),
            'cx': round(cx, 4), 'cy': round(cy, 4),
            'heading_deg': round(self._held_heading, 2) if self._held_heading is not None else None,
            'baseline_m': None,
            'baseline_err_m': None,
            'quality': round(0.3 * max(0.0, 1.0 - float(fix.get('rmse', 0)) / 0.15), 3),
            'both_present': False,
            'present_id': fix['tag_id'],
            'vx': round(float(fix.get('vx', 0)), 4),
            'vy': round(float(fix.get('vy', 0)), 4),
            'speed': round(math.hypot(float(fix.get('vx', 0)), float(fix.get('vy', 0))), 4),
            'ts': round(now, 3),
            'master_id': self.master_id, 'slave_id': self.slave_id,
        }
