"""Build the `tag_fix` message from a `SweepResult`.

`tag_fix` is a **compatibility surface with two different strictnesses**, and
that is why this lives in its own file rather than inline:

* The GUI *renders* a subset of the fields. Changing any of those changes what
  the user sees, immediately.
* Every recorded NDJSON session carries all of them. Changing a field's meaning
  silently reinterprets every past recording, and replay renders it wrong with
  no error anywhere.

So the rule for this file is: **fields keep their names and their meanings; new
information gets new fields.** The GUI ignores keys it does not know, so
additions are free; redefinitions are not.
"""

from __future__ import annotations

import numpy as np

from .pipeline import SweepResult, TagRuntime


def tag_fix_message(res: SweepResult, rt: TagRuntime, pkt, anchors,
                    imu_state: dict | None = None,
                    log_full_gain: bool = False,
                    estimator: str = 'ekf') -> dict:
    """One solved sweep as the message the GUI and the session log expect."""
    sol = res.solution
    ekf = rt.ekf

    # Anchors are reported in the order the sweep answered in, matching what
    # the previous pipeline emitted; `sol` rows are aligned to anchors.ids.
    ids = np.asarray(anchors.ids, dtype=int)
    present = np.flatnonzero(np.isfinite(sol.range_raw))
    active = [int(ids[c]) for c in present]

    dists_filt = {f'd{ids[c]}': _r(sol.range_corr[c], 4) for c in present}
    dists_raw = {f'r{ids[c]}': _r(sol.range_raw[c], 4) for c in present}

    anchor_diag = [{
        'id':       int(ids[c]),
        'd':        _r(sol.range_corr[c], 4),
        'rx':       _r(sol.rx[c], 1),
        'fp':       _r(sol.fp[c], 1),
        'gap':      _r(sol.gap[c], 1),
        'w':        _r(sol.w[c], 3),
        'resid':    _r(sol.info.resid[c] if sol.info is not None else np.nan, 4),
        'used':     bool(sol.info.used[c]) if sol.info is not None else False,
        'rejected': bool(sol.rejected[c]),
    } for c in present]

    P = ekf.P
    pos_cov = [[_r(P[0, 0], 6), _r(P[0, 1], 6)],
               [_r(P[1, 0], 6), _r(P[1, 1], 6)]] if ekf.initialized else \
              [[None, None], [None, None]]

    msg: dict = {
        'type':   'tag_fix',
        'tag_id': res.tag_id,
        'pkt_n':  rt.n_sweeps,

        'cycle':  getattr(pkt, 'cycle', None),
        'mrx_us': getattr(pkt, 'mrx_us', None),
        't_ms':   getattr(pkt, 't_ms', None),

        # x,y is the raw solve; ex,ey is the display output. Same roles and
        # units as before — ex,ey is now pinned+smoothed, matching live_tag.m.
        'x':     _r(res.raw[0], 4),
        'y':     _r(res.raw[1], 4),
        'ex':    _r(res.out[0], 4),
        'ey':    _r(res.out[1], 4),
        'rmse':  _r(sol.info.rmse if sol.info is not None else np.nan, 4),
        'nUsed': int(np.sum(sol.info.used)) if sol.info is not None else 0,
        'ekfR':  ekf.n_reinit,

        'anchor_ids': active,
        **dists_filt,
        **dists_raw,
        'anchor_diag': anchor_diag,

        'ekf_pos':    [_r(ekf.pos[0], 4), _r(ekf.pos[1], 4)] if ekf.initialized else None,
        # Stays length-6 (the base states). Per-anchor bias states go in their
        # own field so nothing downstream mis-indexes this one.
        'ekf_P_diag': [_r(v, 6) for v in np.diag(P)[:6]],
        'pos_cov':    pos_cov,
        'vx':      _r(ekf.vel[0], 4),
        'vy':      _r(ekf.vel[1], 4),
        'speed':   _r(ekf.speed, 4),
        'heading': _r(np.degrees(np.arctan2(ekf.vel[1], ekf.vel[0])), 2),
        'bx':      _r(ekf.bias[0], 6),
        'by':      _r(ekf.bias[1], 6),

        # Restored after the Phase 2 swap dropped them. Every recorded session
        # before that carries these, and the GUI's HEALTH panel reads `health`
        # off each fix, so losing them broke both replay comparability and a
        # live panel.
        'innovation':     [_r(v, 5) for v in ekf.last_innovation],
        'K_diag':         [_r(v, 6) for v in ekf.last_k_diag],
        'correction_mag': _r(ekf.last_correction_mag, 5),
        'health':         rt.health.to_dict(),

        'zupt':          bool(res.still),
        'pred_mode':     'CV',        # live_tag always predicts constant-velocity
        'nis':           _r(res.nis, 4),
        'meas_accepted': bool(res.accepted),

        # ── new, additive ────────────────────────────────────────────────────
        'held':       int(res.n_held),
        'pinned':     bool(res.pinned),
        'still':      bool(res.still),
        'estimator':  estimator,
        'range_bias': [_r(v, 5) for v in ekf.range_bias] if ekf.n_rb else [],
        'rejected':   [bool(sol.rejected[c]) for c in present],
    }

    if imu_state:
        msg['imu'] = imu_state.get('angles')
        msg['imu_gyro'] = imu_state.get('gyro', [])
        msg['imu_orient_conf'] = imu_state.get('status', 0)
        msg['imu_ax_world'] = imu_state.get('ax_world', 0.0)
        msg['imu_ay_world'] = imu_state.get('ay_world', 0.0)
    else:
        msg['imu_gyro'] = []
        msg['imu_orient_conf'] = 0
        msg['imu_ax_world'] = 0.0
        msg['imu_ay_world'] = 0.0

    if log_full_gain:
        msg['K_full'] = None      # reserved; the EKF no longer surfaces K
    return msg


def _r(v, nd: int):
    """Round for the wire, mapping non-finite to None.

    None rather than NaN because NaN is not valid JSON — see host/bus.dumps.
    Doing it here as well keeps the recorded value and the rendered value the
    same thing rather than relying on the encoder to paper over it.
    """
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return round(f, nd) if np.isfinite(f) else None
