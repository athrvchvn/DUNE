"""One sweep in, one display fix out — the single pipeline for live and replay.

Port of the main loop body of ``matlab/scripts/live_tag.m``. Changes must be
made in both.

    stillness → RangeHold → solve_sweep → FusionEkf → output pin → output EMA

The last two stages are **output-stage only**: the estimator's own state is
never touched by them. That is deliberate and was learned the hard way — the
Phase-A3 experiment showed that absorbing the parked-tag "breathing" *inside*
the filter corrupts it, so the clamp lives strictly on the way out.

Why this replaces the previous Python pipeline wholesale: that one inlined its
solve inside the live loop, so offline analysis and live behaviour could drift
apart, and did. Everything here runs identically in both, which is what makes
the replay a trustworthy debugging tool rather than an approximation.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from rtls.ekf import FusionEkf
from rtls.system_health import SystemHealth
from rtls.range_hold import RangeHold
from rtls.solve_sweep import solve_sweep


@dataclass
class PipelineConfig:
    """Defaults mirror `live_tag.m`'s argument block. Same names, snake_case."""
    tag_z:        float = 0.24
    ekf:          bool = True
    hold:         bool = True
    nlos:         bool = True      # gap weights in the solve
    power_corr:   bool = True      # Tier-2 range_correction.json
    mode:         str = 'pos'      # 'pos' | 'ranges' (tightly coupled)
    pin:          bool = True
    pin_radius:   float = 0.05     # m, output deadband
    pin_resid:    float = 0.06     # m, fallback pin trigger on solve residual
    pin_vel:      float = 0.08     # m/s, EKF speed below this counts as parked
    smooth:       float = 0.6      # output EMA alpha while moving (1 = off)
    gate_k:       float = 3.0
    sigma_r:      float = 0.05     # m, per-range sigma in 'ranges' mode
    exclude:      tuple = ()       # anchor ids to drop (A/B testing a link)
    range_bias_states: bool = False

    # 'ekf' | 'mhe'. A real product option — live_tag.m offers the same choice —
    # not a migration flag. fusion_config has declared this "live-switchable via
    # GUI" since before the port; this is where it finally becomes true.
    estimator:    str = 'ekf'
    mhe_horizon:  int = 10
    mhe_model:    str = 'cv'       # 'cv' | 'unicycle'


# Stillness thresholds, from live_tag.m. The window is 8 sweeps (~1.5 s).
STILL_WIN = 8
STILL_ACC_MAX = 0.12      # m/s²
STILL_GYR_MAX = 0.05      # rad/s
STILL_COUNT = 10          # no-IMU fallback: calm sweeps before "still"
PIN_EXCURSION_SWEEPS = 8  # sustained excursion before re-pinning


@dataclass
class TagRuntime:
    """All mutable per-tag state. One per discovered tag."""
    tag_id: int
    ekf:    FusionEkf = field(default_factory=FusionEkf)
    hold:   RangeHold = field(default_factory=RangeHold)
    # Cumulative session health. The GUI's HEALTH panel reads this off every
    # fix, so it is part of the wire contract, not a debug extra.
    health: SystemHealth = field(default_factory=SystemHealth)

    hist_a: deque = field(default_factory=lambda: deque([np.nan] * STILL_WIN,
                                                        maxlen=STILL_WIN))
    hist_g: deque = field(default_factory=lambda: deque([np.nan] * STILL_WIN,
                                                        maxlen=STILL_WIN))
    still_cnt: int = 0

    pin_pos: Optional[np.ndarray] = None
    pin_out: int = 0
    smooth_pos: Optional[np.ndarray] = None

    mhe: object = None                         # built lazily on first use
    t_prev_mhe: Optional[float] = None

    prev_pos: Optional[np.ndarray] = None      # solver warm start
    t_prev_ekf: Optional[float] = None
    diverge_streak: int = 0
    n_sweeps: int = 0


@dataclass
class SweepResult:
    """What one sweep produced. `raw` is the solve, `out` is what to display."""
    tag_id:   int
    t_host:   float
    raw:      np.ndarray                       # p  — logged as x,y
    out:      np.ndarray                       # po — logged as ex,ey
    solution: object = None                    # SweepSolution
    still:    bool = False
    pinned:   bool = False
    n_held:   int = 0
    ekf_pos:  Optional[np.ndarray] = None
    nis:      float = float('nan')
    accepted: bool = True

    @property
    def ok(self) -> bool:
        return bool(np.all(np.isfinite(self.raw)))


class SweepPipeline:
    """Per-tag dispatch over the shared solve/filter chain."""

    def __init__(self, anchors, cfg: PipelineConfig | None = None,
                 range_corr=None, bias=None):
        self.anchors = anchors
        self.cfg = cfg or PipelineConfig()
        self.range_corr = range_corr if self.cfg.power_corr else None
        self.bias = bias
        self.tags: dict[int, TagRuntime] = {}

    # ── per-tag state ────────────────────────────────────────────────────────

    def runtime(self, tag_id: int) -> TagRuntime:
        rt = self.tags.get(tag_id)
        if rt is None:
            rt = TagRuntime(tag_id=tag_id)
            if self.cfg.range_bias_states:
                rt.ekf.enable_range_bias(len(self.anchors.ids))
            self.tags[tag_id] = rt
            print(f"[TAG]  New tag discovered: 0x{tag_id:02X}  (decimal {tag_id})")
        return rt

    # ── the chain ────────────────────────────────────────────────────────────

    def process(self, sweep, t_host: float, imu=None) -> SweepResult:
        cfg = self.cfg
        tag_id = int(getattr(sweep, 'tag_id', 0))
        rt = self.runtime(tag_id)
        rt.n_sweeps += 1

        still = self._stillness(rt, imu)

        n_held = 0
        if cfg.hold:
            held = rt.hold.apply(sweep, t_host, still)
            sweep = held.sweep if hasattr(held, 'sweep') else held
            n_held = getattr(held, 'n_held', 0)

        if cfg.exclude:
            sweep = _exclude(sweep, cfg.exclude)

        sol = solve_sweep(sweep, self.anchors, bias=self.bias,
                          range_corr=self.range_corr, tag_z=cfg.tag_z,
                          x0=rt.prev_pos, use_gap_weights=cfg.nlos,
                          gate_k=cfg.gate_k,
                          w_scale=getattr(sweep, 'w_scale', None))
        p = sol.pos

        pe = self._filter(rt, sol, p, t_host, still) if cfg.ekf else None
        if cfg.estimator == 'mhe':
            # The EKF keeps running underneath: it supplies the stillness
            # counter and the pin's velocity gate, exactly as live_tag.m does.
            # `x or y` would raise here — the truth value of a 2-element array
            # is ambiguous — so the fallback is spelled out.
            pm = self._mhe(rt, sol, p, t_host, still, imu)
            if pm is not None:
                pe = pm
        po = self._pin_and_smooth(rt, sol, p, pe, still)

        if sol.ok:
            rt.prev_pos = p.copy()

        self._update_health(rt, sol, imu, still)

        return SweepResult(
            tag_id=tag_id, t_host=t_host, raw=p, out=po, solution=sol,
            still=still, pinned=bool(rt.pin_pos is not None and np.array_equal(po, rt.pin_pos)),
            n_held=n_held,
            ekf_pos=rt.ekf.pos if rt.ekf.initialized else None,
            nis=rt.ekf.last_nis, accepted=rt.ekf.last_accepted)

    def _update_health(self, rt: TagRuntime, sol, imu, still: bool) -> None:
        """Keep the cumulative health counters the GUI reports."""
        h = rt.health
        h.imu_available = imu is not None
        if imu is not None:
            h.gyro_available = bool(len(getattr(imu, 'gyro', ()) or ()))
            h.orientation_confidence = int(getattr(imu, 'status', 0))
            h.orientation_valid = h.orientation_confidence >= 1
        h.uwb_available = bool(sol.ok)
        h.ekf_initialized = rt.ekf.initialized
        h.prediction_mode = 'CV'          # live_tag always predicts CV
        h.zupt_active = bool(still)
        h.nis_last = rt.ekf.last_nis if np.isfinite(rt.ekf.last_nis) else 0.0
        if sol.info is not None:
            h.n_anchors_used = int(np.sum(sol.info.used))
            h.n_anchors_rejected = int(np.sum(sol.rejected))
        h.correction_accepted = bool(rt.ekf.last_accepted)
        h.correction_rejected = not rt.ekf.last_accepted
        if sol.ok:
            if rt.ekf.last_accepted:
                h.n_uwb_accepted += 1
            else:
                h.n_uwb_rejected += 1
        h.compute_confidence()

    # ── stages ───────────────────────────────────────────────────────────────

    def _stillness(self, rt: TagRuntime, imu) -> bool:
        """IMU stillness when the tag has one, filter-inferred otherwise."""
        if imu is not None and getattr(imu, 'status', 0) >= 1:
            rt.hist_a.append(float(np.linalg.norm(imu.acc)))
            rt.hist_g.append(float(np.linalg.norm(imu.gyro)))
            a = np.asarray(rt.hist_a, dtype=float)
            g = np.asarray(rt.hist_g, dtype=float)
            return bool(np.all(np.isfinite(a)) and a.max() < STILL_ACC_MAX
                        and g.max() < STILL_GYR_MAX)
        # No IMU: calm innovations plus near-zero velocity for ~1.5 s. Motion
        # breaks it within a single sweep.
        return rt.still_cnt >= STILL_COUNT

    def _filter(self, rt: TagRuntime, sol, p, t_host: float, still: bool):
        cfg = self.cfg
        ekf = rt.ekf

        dt = 0.0
        if rt.t_prev_ekf is not None and np.isfinite(t_host):
            dt = min(max(t_host - rt.t_prev_ekf, 0.0), 1.0)
        rt.t_prev_ekf = t_host

        ekf.still_mode = still
        # live_tag always predicts constant-velocity: the IMU drives stillness
        # and the pin's velocity gate, not the propagation.
        ekf.predict(dt, None)

        if cfg.mode == 'ranges':
            if not ekf.initialized:
                if sol.ok:
                    ekf.update_position(p)
            else:
                ekf.update_ranges(self.anchors.coords, sol.range_corr, sol.w,
                                  cfg.tag_z, cfg.sigma_r)
                solid = (sol.ok and int(np.sum(sol.info.used)) >= 4
                         and sol.info.rmse < 0.10)
                if solid and np.linalg.norm(ekf.pos - p) > 0.5:
                    rt.diverge_streak += 1
                elif solid:
                    rt.diverge_streak = 0
                if ((ekf.consec_reject >= ekf.max_consec_reject
                     or rt.diverge_streak >= 5) and sol.ok):
                    ekf.reinit_from(p)
                    rt.diverge_streak = 0
        elif sol.ok:
            ekf.update_position(p, adaptive_r(sol, ekf.pos_sigma))
            if ekf.consec_reject >= ekf.max_consec_reject:
                ekf.reinit_from(p)

        if still and ekf.initialized:
            ekf.update_zupt()

        # The no-IMU stillness counter is driven by the filter itself.
        if ekf.initialized:
            if np.linalg.norm(ekf.vel) < 0.08 and ekf.last_nis < 3:
                rt.still_cnt += 1
            else:
                rt.still_cnt = 0

        return ekf.pos if ekf.initialized else None

    def _mhe(self, rt: TagRuntime, sol, p, t_host: float, still: bool, imu):
        """Sliding-window smoother; its output feeds the pin and the display.

        Falls back to the raw fix during warm-up rather than emitting nothing,
        so switching estimator mid-session does not blank the display for a
        window's worth of sweeps.
        """
        from rtls.mhe_estimator import MheEstimator

        if rt.mhe is None:
            rt.mhe = MheEstimator(self.anchors.coords)
            rt.mhe.model = self.cfg.mhe_model
            rt.mhe.horizon = self.cfg.mhe_horizon
            rt.mhe.tag_z = self.cfg.tag_z
            rt.mhe.build()

        dt = 0.0
        if rt.t_prev_mhe is not None and np.isfinite(t_host):
            dt = min(max(t_host - rt.t_prev_mhe, 0.0), 1.0)
        rt.t_prev_mhe = t_host

        omega = 0.0
        if imu is not None:
            from rtls.ekf import world_yaw_rate
            omega = world_yaw_rate(imu.quat, imu.gyro)

        pm, _, info = rt.mhe.push(sol.range_corr, sol.w, max(dt, 1e-3),
                                  p_hint=p if sol.ok else None, still=still,
                                  omega=omega)
        if not info.warmup and np.all(np.isfinite(pm)):
            return pm
        return p if sol.ok else None

    def _pin_and_smooth(self, rt: TagRuntime, sol, p, pe, still: bool):
        cfg = self.cfg
        po = p.copy()
        if cfg.ekf and pe is not None and np.all(np.isfinite(pe)):
            po = np.asarray(pe, dtype=float).copy()

        # The pin engages on IMU stillness OR — as a fallback for an IMU that
        # will not settle — a calm solve: low residual and near-zero EKF speed.
        # It releases the instant the solve gets noisy or the tag really moves.
        calm = (np.isfinite(sol.info.rmse) and sol.info.rmse < cfg.pin_resid
                and cfg.ekf and rt.ekf.initialized
                and np.linalg.norm(rt.ekf.vel) < cfg.pin_vel)
        pinnable = still or calm
        pinned = False

        if cfg.pin and pinnable and np.all(np.isfinite(po)):
            if rt.pin_pos is None or not np.all(np.isfinite(rt.pin_pos)):
                rt.pin_pos = po.copy()
                rt.pin_out = 0
            if np.linalg.norm(po - rt.pin_pos) < cfg.pin_radius:
                rt.pin_out = 0
                po = rt.pin_pos.copy()
                pinned = True
            else:
                rt.pin_out += 1
                if rt.pin_out >= PIN_EXCURSION_SWEEPS:
                    rt.pin_pos = po.copy()      # sustained — re-pin there
                    rt.pin_out = 0
                else:
                    po = rt.pin_pos.copy()      # brief excursion — hold
                pinned = True
        else:
            rt.pin_pos = None
            rt.pin_out = 0

        # Output smoother, moving only. Re-seeded to the pin point while pinned
        # so a release into motion starts without a lag jump.
        if pinned or not np.all(np.isfinite(po)):
            rt.smooth_pos = po.copy()
        elif cfg.smooth < 1 and rt.smooth_pos is not None and np.all(
                np.isfinite(rt.smooth_pos)):
            rt.smooth_pos = cfg.smooth * po + (1 - cfg.smooth) * rt.smooth_pos
            po = rt.smooth_pos.copy()
        else:
            rt.smooth_pos = po.copy()
        return po


# ── helpers ───────────────────────────────────────────────────────────────────

def adaptive_r(sol, pos_sigma: float) -> np.ndarray:
    """LM covariance inflated by residual RMSE, mean NLOS gap and a DOP proxy.

    Port of `adaptiveR` in live_tag.m. The floor of 0.02 m on the LM covariance
    matters: a very confident solve would otherwise let the filter ignore every
    subsequent measurement.
    """
    cov = np.asarray(sol.info.cov, dtype=float)
    if np.all(np.isfinite(cov)):
        Rb = cov + np.eye(2) * 0.02 ** 2
    else:
        Rb = np.eye(2) * pos_sigma ** 2

    rms_f = 1.0 + (sol.info.rmse / 0.05) ** 2 if np.isfinite(sol.info.rmse) else 1.0

    gap = np.asarray(sol.gap, dtype=float)
    rng = np.asarray(sol.range_raw, dtype=float)
    g = gap[np.isfinite(gap) & np.isfinite(rng)]
    nlos_f = 1.0 + float(np.mean(np.maximum(0.0, g / 6.0))) if g.size else 1.0

    dop = float(np.sqrt(np.trace(Rb)))
    dop_f = 1.0 + max(0.0, (dop - 0.05) / 0.05)

    return Rb * min(max(rms_f * nlos_f * dop_f, 1.0), 50.0)


def _exclude(sweep, ids):
    """Drop anchors by id, AFTER the hold so it cannot re-inject them."""
    keep = ~np.isin(np.asarray(sweep.ids), np.asarray(list(ids)))
    if bool(np.all(keep)):
        return sweep

    class _S:
        pass
    out = _S()
    for name in ('ids', 'dist', 'rx', 'fp', 'qual', 'w_scale', 'cfo_ppm', 'tex'):
        v = getattr(sweep, name, None)
        if v is not None and len(np.asarray(v)) == keep.size:
            setattr(out, name, np.asarray(v)[keep])
    out.tag_id = getattr(sweep, 'tag_id', 0)
    return out
