"""Moving Horizon Estimators, against synthetic truth and against MATLAB.

Two levels, because they answer different questions.

**Structural properties** are checked on synthetic data, where truth is known
exactly: the rigid model must make the inter-tag baseline exact by construction,
terrain pitch must shorten the projected baseline and split the tag heights, and
the jump guard must fire on a leap.

**Agreement with MATLAB** is checked on the eight recorded rover sessions, which
exist as both a raw wire tee and the pose track `live_rover.m` wrote from it.
That comparison is deliberately at *metric* level, not per-sample: MATLAB uses
the CasADi 3.7.0 MEX build and this uses the pip wheel, so the two IPOPT
instances need not take the same path to the same optimum. Chasing bit-equality
across two nonlinear solvers is a way to waste a week.

**Only two-tag sessions are compared.** With a single tag the body heading is
geometrically unobservable — a whole circle of poses fits the ranges equally —
so psi is driven only by gyro integration from an arbitrary seed and two
implementations will diverge legitimately. MATLAB agrees: it logs no `yawRaw`
for exactly those sessions. Comparing them would be measuring the degeneracy.
"""

from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

casadi = pytest.importorskip('casadi', reason="CasADi is required for the MHE")

from rtls.anchor_config import AnchorConfig          # noqa: E402
from rtls.ekf import quat_rpy, world_yaw_rate        # noqa: E402
from rtls.frame_parser import FrameParser            # noqa: E402
from rtls.mhe_estimator import MheEstimator          # noqa: E402
from rtls.mhe_rigid import MheRigid                  # noqa: E402
from rtls.range_correction import RangeCorrection    # noqa: E402
from rtls.raw_log import read_raw_tee                # noqa: E402
from rtls.solve_sweep import solve_sweep             # noqa: E402

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / 'logs'
CONFIG = REPO / 'config'
ANCHORS = CONFIG / 'anchors_kinect_clicked_20260724.json'

SQUARE = np.array([[0.0, 0.0, 0.0], [6.0, 0.0, 0.0],
                   [6.0, 3.0, 0.0], [0.0, 3.0, 0.0]])
FRONT_TAG, REAR_TAG = 241, 240
BASELINE = 0.527


def _ranges(p, anchors=SQUARE, tag_z=0.24):
    d = anchors[:, :2] - np.asarray(p, float)
    return np.hypot(np.hypot(d[:, 0], d[:, 1]), anchors[:, 2] - tag_z)


# ── MheEstimator ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('model', ['cv', 'unicycle'])
def test_mhe_tracks_a_moving_tag(model):
    rng = np.random.RandomState(0)
    m = MheEstimator(SQUARE)
    m.model = model
    m.build()

    errs = []
    for i in range(50):
        truth = np.array([1.0 + 0.02 * i, 1.5])
        z = _ranges(truth) + rng.randn(4) * 0.03
        p, v, info = m.push(z, np.ones(4), 0.2,
                            p_hint=truth + rng.randn(2) * 0.05)
        if not info.warmup:
            errs.append(np.linalg.norm(p - truth))

    assert errs, "never left warm-up"
    assert np.median(errs) < 0.10, f"{model}: median error {np.median(errs):.3f} m"


def test_mhe_reports_warmup_until_the_window_is_full():
    m = MheEstimator(SQUARE)
    m.horizon = 6
    for i in range(5):
        p, _, info = m.push(_ranges([2.0, 1.0]), np.ones(4), 0.2)
        assert info.warmup and np.all(np.isnan(p))
    _, _, info = m.push(_ranges([2.0, 1.0]), np.ones(4), 0.2)
    assert not info.warmup


def test_mhe_absent_anchors_are_neutralised_not_treated_as_zero_range():
    """A NaN range with weight 0 must not pull the solution toward the anchor."""
    m = MheEstimator(SQUARE)
    m.horizon = 6
    m.build()
    truth = np.array([2.0, 1.0])
    z = _ranges(truth)
    z[1] = np.nan
    w = np.array([1.0, 0.0, 1.0, 1.0])
    for _ in range(10):
        p, _, info = m.push(z, w, 0.2, p_hint=truth)
    assert np.linalg.norm(p - truth) < 0.15, f"pulled to {p}"


def test_mhe_reset_clears_the_window():
    m = MheEstimator(SQUARE)
    m.horizon = 4
    for _ in range(6):
        m.push(_ranges([2.0, 1.0]), np.ones(4), 0.2)
    m.reset()
    _, _, info = m.push(_ranges([2.0, 1.0]), np.ones(4), 0.2)
    assert info.warmup, "reset did not clear the buffer"


# ── MheRigid: the structural guarantees ───────────────────────────────────────

def test_the_inter_tag_baseline_is_exact_by_construction():
    """The whole point of solving one pose instead of two positions.

    Both tags are derived from the same pose, so the separation cannot drift —
    not "penalised toward L", but exactly L, to floating-point.
    """
    mr = MheRigid(SQUARE)
    mr.baseline = BASELINE
    for psi in np.linspace(-np.pi, np.pi, 17):
        pf, pr = mr.tag_positions([1.0, 2.0, psi, 0.3])
        assert np.linalg.norm(pf - pr) == pytest.approx(BASELINE, abs=1e-12)


def test_terrain_pitch_shortens_the_projected_baseline():
    mr = MheRigid(SQUARE)
    mr.baseline = BASELINE
    level = np.linalg.norm(np.subtract(*mr.tag_positions([0, 0, 0.0, 0], 0.0)))
    tilted = np.linalg.norm(np.subtract(*mr.tag_positions([0, 0, 0.0, 0], 0.4)))
    assert level == pytest.approx(BASELINE)
    assert tilted == pytest.approx(BASELINE * np.cos(0.4))


def test_pitch_raises_one_tag_and_drops_the_other():
    """Encoded in the buffered node, so check what push() stored."""
    mr = MheRigid(SQUARE)
    mr.baseline = BASELINE
    mr.tag_z = 0.24
    mr.push(_ranges([2.0, 1.0]), np.ones(4), True, 0.2, pitch=0.3)
    mr.push(_ranges([2.0, 1.0]), np.ones(4), False, 0.2, pitch=0.3)
    front, rear = mr.buf[-2], mr.buf[-1]
    assert front['tz'] > mr.tag_z > rear['tz']
    assert front['tz'] - mr.tag_z == pytest.approx((BASELINE / 2) * np.sin(0.3))
    assert front['lh'] == pytest.approx((BASELINE / 2) * np.cos(0.3))


def test_roll_is_deliberately_ignored():
    """The baseline lies along the axis roll rotates about, so it cannot move
    the tags. push() takes no roll argument at all — this documents that."""
    import inspect
    assert 'roll' not in inspect.signature(MheRigid.push).parameters


def test_rigid_mhe_tracks_a_turning_rover_and_holds_the_baseline():
    rng = np.random.RandomState(1)
    mr = MheRigid(SQUARE)
    mr.baseline = BASELINE
    mr.build()

    errs, bases = [], []
    for i in range(70):
        t = i * 0.15
        psi = 0.3 * t
        c = np.array([2.0 + 0.15 * t, 1.5 + 0.05 * t])
        is_front = (i % 2 == 0)
        u = (BASELINE / 2) * np.array([np.cos(psi), np.sin(psi)])
        p_tag = c + u if is_front else c - u
        z = _ranges(p_tag) + rng.randn(4) * 0.03
        pose, info = mr.push(z, np.ones(4), is_front, 0.15,
                             p_hint=p_tag + rng.randn(2) * 0.05, omega=0.3)
        if not info.warmup and np.all(np.isfinite(pose)):
            errs.append(np.linalg.norm(pose[:2] - c))
            pf, pr = mr.tag_positions(pose)
            bases.append(np.linalg.norm(pf - pr))

    assert errs, "never left warm-up"
    assert np.median(errs) < 0.20, f"centre error {np.median(errs):.3f} m"
    assert np.allclose(bases, BASELINE, atol=1e-9), "baseline drifted"


# ── Agreement with MATLAB, on real sessions ───────────────────────────────────

def _rover_pairs():
    for jp in sorted(LOGS.glob('rover_log_*.jsonl')):
        m = re.search(r'(\d{8}_\d{6})', jp.name)
        if not m:
            continue
        rp = LOGS / f"udp_raw_{m.group(1)}.log"
        if rp.exists():
            yield m.group(1), rp, jp


def _replay_rigid(raw: Path, jsonl: Path):
    """Drive MheRigid from the wire; compare against live_rover's pose track."""
    ref = {round(r['t_host'], 3): r
           for r in (json.loads(l) for l in jsonl.read_text().splitlines() if l.strip())
           if 'cx' in r}
    if len(ref) < 40:
        return None

    packets = [(t, FrameParser.parse(b)) for t, b in read_raw_tee(raw)]
    packets = [(t, p) for t, p in packets
               if p.valid and int(p.tag_id) in (FRONT_TAG, REAR_TAG)]
    tags = {int(p.tag_id) for _, p in packets}
    if len(tags) < 2:
        return {'two_tag': False, 'n': 0}

    cfg = AnchorConfig.from_json(str(ANCHORS))
    rc = RangeCorrection.load(str(CONFIG / 'range_correction.json'))
    mr = MheRigid(cfg.coords)
    mr.baseline = BASELINE
    mr.horizon = 12
    mr.tag_z = 0.24
    mr.build()

    prev = None
    om_hold = pitch_hold = 0.0       # persist across sweeps, as live_rover does
    dc, dpsi = [], []
    for t, pkt in packets:
        dt = 0.15 if prev is None else max(t - prev, 1e-3)
        prev = t
        if pkt.imu is not None:
            _, pitch_hold, _ = quat_rpy(pkt.imu.quat)
            om_hold = world_yaw_rate(pkt.imu.quat, pkt.imu.gyro)
        sol = solve_sweep(pkt, cfg, range_corr=rc, tag_z=0.24)
        pose, info = mr.push(sol.range_corr, sol.w, int(pkt.tag_id) == FRONT_TAG,
                             dt, p_hint=sol.pos if sol.ok else None,
                             omega=om_hold, pitch=pitch_hold)
        r = ref.get(round(t, 3))
        if r is not None and not info.warmup and np.all(np.isfinite(pose)):
            dc.append(np.hypot(pose[0] - r['cx'], pose[1] - r['cy']))
            dpsi.append(abs(((pose[2] - r['psi'] + np.pi) % (2 * np.pi)) - np.pi))
    if not dc:
        return None
    return {'two_tag': True, 'n': len(dc),
            'dc_med_mm': float(np.median(dc) * 1000),
            'dpsi_med_deg': float(np.degrees(np.median(dpsi)))}


_RIGID: list | None = None


def rigid_results() -> list:
    global _RIGID
    if _RIGID is None:
        _RIGID = []
        for stamp, raw, jsonl in _rover_pairs():
            r = _replay_rigid(raw, jsonl)
            if r:
                _RIGID.append(dict(r, stamp=stamp))
    return _RIGID


@pytest.mark.skipif(not ANCHORS.exists(), reason="no anchor config in this checkout")
def test_rigid_mhe_agrees_with_matlab_on_two_tag_sessions():
    res = [r for r in rigid_results() if r['two_tag'] and r['n'] > 50]
    if len(res) < 2:
        pytest.skip("fewer than two two-tag rover sessions available")

    for r in res:
        assert r['dc_med_mm'] < 50.0, (
            f"{r['stamp']}: centre differs by {r['dc_med_mm']:.1f} mm")
        assert r['dpsi_med_deg'] < 5.0, (
            f"{r['stamp']}: heading differs by {r['dpsi_med_deg']:.2f} deg")

    worst = max(r['dc_med_mm'] for r in res)
    assert worst < 30.0, f"worst two-tag session {worst:.1f} mm"


@pytest.mark.skipif(not ANCHORS.exists(), reason="no anchor config in this checkout")
def test_single_tag_sessions_are_identified_rather_than_compared():
    """Heading is unobservable with one tag; MATLAB logs no yawRaw for those.

    This asserts the corpus really does contain both kinds, so the exclusion
    above is a documented property rather than a convenient filter.
    """
    kinds = {r['two_tag'] for r in rigid_results()}
    if not kinds:
        pytest.skip("no rover session corpus in this checkout")
    assert kinds == {True, False}, f"corpus is not mixed: {kinds}"

    for stamp, _raw, jsonl in _rover_pairs():
        rows = [json.loads(l) for l in jsonl.read_text().splitlines() if l.strip()]
        match = [r for r in rigid_results() if r['stamp'] == stamp]
        if not match or match[0]['two_tag']:
            continue
        assert not any(r.get('yawRaw') is not None for r in rows), (
            f"{stamp} was judged single-tag but MATLAB logged a raw yaw")


# ── Live switching ────────────────────────────────────────────────────────────

def test_the_pipeline_can_run_either_estimator():
    """`estimator_mode` has been declared "live-switchable via GUI" in
    fusion_config since before the port, with nothing wiring it. This is it."""
    from host.pipeline import PipelineConfig, SweepPipeline

    line = ('RTLS,v3,4513,240,5,1,3223,-81,-83.9,383.11,2,3767,-98,-102.3,'
            '125.77,3,4274,-96,-98.1,164.67,4,3326,-92,-95.9,184.68,'
            '5,1971,-87,-89.0,278.39')
    cfg = AnchorConfig()

    outs = {}
    for est in ('ekf', 'mhe'):
        pipe = SweepPipeline(cfg, PipelineConfig(estimator=est, mhe_horizon=5))
        for i in range(12):
            res = pipe.process(FrameParser.parse(line), 1000.0 + 0.2 * i)
        outs[est] = res.out
        assert np.all(np.isfinite(res.out)), f"{est} produced no output"

    # Both must produce a fix; they need not agree, and the MHE having its own
    # answer is the point of offering it.
    assert not np.allclose(outs['ekf'], outs['mhe']), \
        "switching estimator changed nothing — is the MHE actually running?"


def test_switching_to_mhe_mid_session_does_not_blank_the_display():
    """During warm-up the MHE falls back to the raw fix rather than nothing."""
    from host.pipeline import PipelineConfig, SweepPipeline

    line = ('RTLS,v3,4513,240,5,1,3223,-81,-83.9,383.11,2,3767,-98,-102.3,'
            '125.77,3,4274,-96,-98.1,164.67,4,3326,-92,-95.9,184.68,'
            '5,1971,-87,-89.0,278.39')
    pipe = SweepPipeline(AnchorConfig(), PipelineConfig(mhe_horizon=8))
    for i in range(4):
        pipe.process(FrameParser.parse(line), 1000.0 + 0.2 * i)
    pipe.cfg.estimator = 'mhe'                      # switch mid-session
    for i in range(4, 9):
        res = pipe.process(FrameParser.parse(line), 1000.0 + 0.2 * i)
        assert np.all(np.isfinite(res.out)), f"display blanked at sweep {i}"
