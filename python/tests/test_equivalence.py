"""
Cross-language equivalence: the Python solve path vs DUNE's MATLAB pipeline.

This is the strongest check available that the Python rebuild is a faithful port
of ``matlab/+dune``. It does not need MATLAB installed: DUNE's recorded hardware
sessions in ``matlab/logs/`` contain, per sweep, both the raw wire line
(``serial_raw_*.log``) and what its MATLAB pipeline computed from that line
(``rtls_log_*.jsonl`` -- corrected ranges, NLOS gap and weight, used-anchor set,
solved x/y). Reproducing those numbers here pins both implementations together.

**Agreement is expected only where the fit is well conditioned.** Where a sweep's
ranges are mutually inconsistent (residual RMSE above ~100 mm) the weighted-LM
optimum sits in a shallow basin and lands tens of mm apart for any small
difference in damping schedule or stopping tolerance -- that is a property of the
data, not a defect. So the tolerance is stratified by DUNE's own reported
residual, which is also a useful signal in itself: if the well-conditioned bucket
ever stops matching, something real has broken.

Measured at the time of writing, over 17,643 sweeps:

    residual < 10 mm    median |dpos| 0.055 mm    (n=1525)
    residual 10-30 mm   median |dpos| 0.056 mm    (n=3259)
    residual 30-100 mm  median |dpos| 0.065 mm    (n=6832)

0.05 mm is the JSONL's own storage resolution (values are rounded to 4 dp), so
the well-conditioned buckets agree to the precision the reference can express.

Run:  python -m pytest python/tests/test_equivalence.py -v
"""

from __future__ import annotations

import glob
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from rtls.frame_parser import FrameParser              # noqa: E402
from rtls.multilaterate import multilaterate           # noqa: E402
from rtls.nlos import gap_weights                      # noqa: E402
from rtls.range_correction import RangeCorrection      # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
LOGS = os.path.join(REPO, 'matlab', 'logs')
CONFIG = os.path.join(REPO, 'matlab', 'config')

# The constellation these sessions were recorded against.
ANCHORS_JSON = os.path.join(CONFIG, 'anchors_kinect_clicked_20260724.json')
TAG_Z = 0.24

# JSONL stores ranges/positions rounded to 4 decimal places.
LOG_RESOLUTION_M = 1e-4


def _require(path):
    if not os.path.exists(path):
        pytest.skip(f"reference data not present: {path}")


def _anchors():
    _require(ANCHORS_JSON)
    d = json.load(open(ANCHORS_JSON))
    ids = np.array([a['id'] for a in d['anchors']], dtype=int)
    pos = np.array([[a['x'], a['y'], a['z']] for a in d['anchors']], dtype=float)
    return ids, pos


def _sessions():
    return sorted(glob.glob(os.path.join(LOGS, 'rtls_log_2026072*.jsonl')))


def _diag_rows(d):
    """
    anchor_diag entries, tolerating a MATLAB serialisation quirk: jsonencode
    collapses a SINGLE-element struct array into a JSON object rather than a
    one-element array, so a sweep with exactly one diagnosed anchor decodes as a
    dict. Iterating that blindly yields field-name strings.
    """
    ad = d.get('anchor_diag', [])
    if isinstance(ad, dict):
        return [ad]
    return [e for e in ad if isinstance(e, dict)]


def _session_records(path):
    """Yield (record, diag-by-anchor-id) for one session file."""
    for line in open(path, errors='replace'):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if d.get('x') is None:
            continue
        yield d, {e['id']: e for e in _diag_rows(d) if 'id' in e}


def _reference_rows():
    """Every solved sweep across all sessions."""
    for f in _sessions():
        yield from _session_records(f)


def _nlos_was_enabled(path) -> bool:
    """
    live_tag can run with `nlos=false`, which pins every weight to 1.0 and makes
    that session useless for checking the weighting rule. Detect it rather than
    hardcoding a filename: weighting was on if any weight differs from 1 while
    some gap exceeds the LOS threshold.
    """
    saw_gap = False
    for d, diag in _session_records(path):
        for e in diag.values():
            if e.get('gap') is None or e.get('w') is None:
                continue
            if e['gap'] > 3.0:
                saw_gap = True
                if abs(e['w'] - 1.0) > 1e-9:
                    return True
    return not saw_gap   # no informative gaps at all -> nothing to contradict


# ── parser ────────────────────────────────────────────────────────────────────

def test_parser_accepts_every_recorded_wire_line():
    """Every RTLS line DUNE's hardware actually emitted must parse (v3 and v4)."""
    files = sorted(glob.glob(os.path.join(LOGS, '*.log')))
    if not files:
        pytest.skip("no raw logs present")
    seen = {'v3': 0, 'v4': 0}
    failures = []
    for f in files:
        for raw in open(f, errors='replace'):
            i = raw.find('RTLS,')
            if i < 0:
                continue
            line = raw[i:].strip()
            pkt = FrameParser.parse(line)
            ver = line.split(',')[1]
            if ver in seen:
                seen[ver] += 1
            if not pkt.valid:
                failures.append(line[:120])
    assert not failures, f"{len(failures)} lines failed to parse, e.g. {failures[:3]}"
    assert seen['v3'] > 0 and seen['v4'] > 0, f"expected both versions, got {seen}"


def test_v4_tail_ordering_survives_an_imu_tail():
    """
    DIAG -> IMU -> CYC -> MRX is positional. A bug where the IMU branch failed to
    advance the token pointer made every tail after IMU unreachable, which
    silently dropped the TagLink cycle id on exactly the IMU-bearing master tag.
    """
    line = ("RTLS,v4,1009,240,1,2,3223,-86,-88.7,223.95,17,48,"
            "DIAG,-132.0,2.51,"
            "IMU,3,0.3986,0.0828,0.022,0.9131,0.01,-0.02,0.03,0.001,0.002,0.003,"
            "CYC,77,MRX,123456")
    p = FrameParser.parse(line)
    assert p.valid
    assert p.temp_c == -132.0 and p.vbat_v == 2.51
    assert p.imu is not None and p.imu.status == 3
    assert p.cycle == 77, "CYC lost after the IMU tail"
    assert p.mrx_us == 123456, "MRX lost after the IMU tail"
    assert p.tex_ms == [48.0]
    # cfo is converted to ppm, sign per dwt_readcarrierintegrator
    assert p.cfo_ppm[0] == pytest.approx(17 * FrameParser.CFO_PPM_PER_LSB)


def test_v1_stride_does_not_misread_as_v3():
    """v1 has 3 fields/anchor and no first-path power; fp must stay empty."""
    p = FrameParser.parse("RTLS,v1,500,240,2,1,1000,-80,2,2000,-85")
    assert p.valid and p.ids == [1, 2]
    assert p.dist == [1.0, 2.0]
    assert p.fp == [] and p.quality == []


def test_raw_ranges_match_matlab_exactly():
    """
    Parsed distances must equal what MATLAB recorded for the same sweep.

    Sessions are paired by filename stamp, NOT pooled: t_ms is the tag's
    millis() and restarts at every boot, so (tag_id, t_ms) is only unique within
    one session. Pooling silently matches sweeps from different runs.
    """
    compared = 0
    paired = 0
    for jf in _sessions():
        stamp = os.path.basename(jf)[len('rtls_log_'):-len('.jsonl')]
        sf = os.path.join(LOGS, f'serial_raw_{stamp}.log')
        if not os.path.exists(sf):
            continue
        paired += 1
        ref = {(d['tag_id'], d['t_ms']): d for d, _ in _session_records(jf)}
        for raw in open(sf, errors='replace'):
            i = raw.find('RTLS,')
            if i < 0:
                continue
            pkt = FrameParser.parse(raw[i:].strip())
            if not pkt.valid:
                continue
            d = ref.get((pkt.tag_id, pkt.t_ms))
            if d is None:
                continue
            for aid, dist in zip(pkt.ids, pkt.dist):
                jr = d.get(f'r{aid}')
                if jr is None:
                    continue
                compared += 1
                assert abs(jr - dist) <= LOG_RESOLUTION_M, (
                    f"{stamp} anchor {aid} t_ms={pkt.t_ms}: {dist} vs recorded {jr}")
    if paired == 0:
        pytest.skip("no matched serial_raw/jsonl session pairs present")
    assert compared > 1000, f"only {compared} ranges compared"


# ── Tier-2 correction and NLOS weighting ──────────────────────────────────────

def test_tier2_power_correction_matches_matlab():
    """corrected = raw - (slope*fp + intercept)/1000, fp clamped to the window."""
    rc = RangeCorrection.load(os.path.join(CONFIG, 'range_correction.json'))
    if rc is None:
        pytest.skip("range_correction.json not present")
    ids, _ = _anchors()
    compared = 0
    worst = 0.0
    for f in _sessions():
        rows = list(_session_records(f))
        # live_tag can run with powerCorr=false, in which case d == r and there
        # is no correction to verify. Skip those sessions rather than assert
        # against a model that was never applied.
        if not rows:
            continue
        applied = sum(1 for d, _ in rows
                      for aid in ids
                      if d.get(f'r{aid}') is not None and d.get(f'd{aid}') is not None
                      and abs(d[f'r{aid}'] - d[f'd{aid}']) > LOG_RESOLUTION_M)
        if applied == 0:
            continue
        for d, diag in rows:
            for aid in ids:
                raw_r, corr_r = d.get(f'r{aid}'), d.get(f'd{aid}')
                if raw_r is None or corr_r is None:
                    continue
                e = diag.get(int(aid))
                if not e or e.get('fp') is None:
                    continue
                mine = raw_r - rc.correction_mm(int(aid), float(e['fp'])) / 1000.0
                worst = max(worst, abs(mine - corr_r))
                compared += 1
    if compared < 1000:
        pytest.skip(f"only {compared} corrected ranges available")
    # 2x resolution: r and d are each rounded independently before storage.
    assert worst <= 2 * LOG_RESOLUTION_M, (
        f"worst Tier-2 disagreement {worst*1000:.4f} mm")


def test_nlos_gap_weights_match_matlab():
    """
    w = max(floor, 10^(-max(0, gap-thresh)/10)). Recorded weights may be scaled
    DOWN by RangeHold's decay on held entries, so the check is one-sided: a
    recorded weight above the gap-only value would mean the rule itself differs.
    """
    over = []
    total = 0
    for f in _sessions():
        if not _nlos_was_enabled(f):
            continue          # session ran with nlos=false; every w is pinned to 1
        for d, diag in _session_records(f):
            for e in diag.values():
                if e.get('gap') is None or e.get('w') is None:
                    continue
                pred = float(gap_weights(np.array([e['gap']]))[0])
                total += 1
                if e['w'] > pred + 2e-3:      # 2e-3 covers 3-dp log rounding
                    over.append((os.path.basename(f), e['gap'], e['w'], round(pred, 4)))
    if total < 1000:
        pytest.skip(f"only {total} weighted anchor rows available")
    assert not over, (
        f"{len(over)}/{total} recorded weights exceed the gap-only rule, "
        f"e.g. {over[:3]}")


# ── the solver ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("lo,hi,tol_mm", [
    (0.000, 0.010, 0.5),
    (0.010, 0.030, 0.5),
    (0.030, 0.100, 1.0),
])
def test_solver_matches_matlab_where_well_conditioned(lo, hi, tol_mm):
    """
    Feed MATLAB's own corrected ranges, weights and used-set into the Python
    solver with gating disabled, and require the same position back.

    Restricted to sweeps whose residual RMSE is below `hi`: above ~100 mm the
    ranges disagree with each other badly enough that the optimum is not
    well-determined, and the two implementations legitimately land apart.
    """
    ids, pos = _anchors()
    errs = []
    for d, diag in _reference_rows():
        r = d.get('rmse')
        if r is None or not (lo <= r < hi):
            continue
        rng = np.full(ids.size, np.nan)
        w = np.ones(ids.size)
        for k, aid in enumerate(ids):
            e = diag.get(int(aid))
            if e and e.get('used') and d.get(f'd{aid}') is not None:
                rng[k] = d[f'd{aid}']
                w[k] = e.get('w', 1.0)
        if np.count_nonzero(~np.isnan(rng)) < 3:
            continue
        p, _ = multilaterate(pos, rng, weights=w, tag_z=TAG_Z, gate_k=0)
        if np.all(np.isfinite(p)):
            errs.append(np.hypot(p[0] - d['x'], p[1] - d['y']))

    if len(errs) < 50:
        pytest.skip(f"only {len(errs)} sweeps in residual band [{lo},{hi})")
    errs = np.array(errs)
    med = np.median(errs) * 1000
    assert med <= tol_mm, (
        f"residual band [{lo},{hi}): median |dpos| {med:.4f} mm > {tol_mm} mm "
        f"over {errs.size} sweeps")


def test_collinear_geometry_is_refused():
    """
    Near-collinear anchors give a mirror ambiguity and a singular normal matrix.
    Returning a confident wrong answer is worse than returning nothing, so the
    solver must refuse rather than emit a fix.
    """
    pos = np.array([[0.0, 0.0, 0.24],
                    [1.0, 0.0, 0.24],
                    [2.0, 0.0, 0.24],
                    [3.0, 0.0, 0.24]])       # exactly collinear
    rng = np.array([1.5, 1.1, 1.1, 1.5])
    p, info = multilaterate(pos, rng, tag_z=0.24)
    assert not np.all(np.isfinite(p)), "collinear geometry should not produce a fix"


def test_sentinel_power_marks_range_rejected():
    """INT32_MIN is a DW1000 diagnostic-read failure, not a real power reading."""
    from rtls.solve_sweep import SENTINEL, solve_sweep

    class Sweep:
        ids = [1, 2, 3, 4]
        dist = [2.0, 2.0, 2.0, 2.0]
        rx = [-80.0, -80.0, SENTINEL, -80.0]
        fp = [-83.0, -83.0, -83.0, -83.0]
        tag_id = 240

    class Anch:
        ids = np.array([1, 2, 3, 4])
        coords = np.array([[0., 0., .24], [3., 0., .24], [3., 3., .24], [0., 3., .24]])

    sol = solve_sweep(Sweep(), Anch(), tag_z=0.24)
    assert sol.rejected[2], "sentinel rx power must reject that anchor's range"
    assert np.isnan(sol.range_raw[2])
    assert not sol.rejected[[0, 1, 3]].any()
