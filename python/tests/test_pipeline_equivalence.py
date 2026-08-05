"""`SweepPipeline` against MATLAB's own recorded output.

Every session in `matlab/logs/` exists twice: a raw serial/UDP tee of the wire
lines, and the JSONL that MATLAB's `live_tag.m` wrote while consuming them. That
pairing lets the Python port be driven with the same input and scored against
the same output, with no MATLAB needed at test time.

Two columns are compared, and they pin different things:

* **`x,y`** — the solve (`solve_sweep`). Reproduces to ~0.04 mm, which is the
  log's own storage resolution (4 decimal places), i.e. exactly.
* **`ex,ey`** — the display output, after `FusionEkf` + the pin deadband + the
  output EMA. This pins the entire new surface in one number.

## Why this does not assert on every session

`live_tag.m` takes a dozen options (`hold`, `pin`, `smooth`, `estimator`,
`tagZ`, `anchors`, …) and **records none of them in its output**. Sessions in
this corpus demonstrably ran with different settings — RangeHold is off in every
07-20 session and on from 07-21 onward, which is visible in the data (a held
sweep logs more anchors than the wire carried) and is how `hold` is recovered
below. Others differ in ways that are *not* recoverable.

So the assertion is: `x,y` must reproduce essentially everywhere, and a
substantial number of sessions must reproduce `ex,ey` **exactly**. Six currently
do, to 0.03-0.09 mm. That is a strict guard — any change to the EKF, the pin or
the smoother breaks those six immediately — without pretending the corpus is
homogeneous when it is not.

The permanent fix is for `sweepRecord.m` to log the options it ran under.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host.pipeline import PipelineConfig, SweepPipeline      # noqa: E402
from rtls.anchor_config import AnchorConfig                  # noqa: E402
from rtls.frame_parser import FrameParser                    # noqa: E402
from rtls.range_correction import RangeCorrection            # noqa: E402
from rtls.session_log import read_session_log                # noqa: E402

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / 'matlab' / 'logs'
CONFIG = REPO / 'matlab' / 'config'

# The geometry these sessions were recorded against. Established empirically:
# it reproduces the logged x,y to log resolution, where every other committed
# anchors file is off by metres.
ANCHORS = CONFIG / 'anchors_kinect_clicked_20260724.json'

# 4 decimal places in the log == 0.05 mm quantisation, so this is "exact".
LOG_RESOLUTION_MM = 0.1

pytestmark = pytest.mark.skipif(not LOGS.is_dir() or not ANCHORS.exists(),
                                reason="no paired session corpus in this checkout")


def _paired():
    """(stamp, raw tee, jsonl) for every session recorded both ways."""
    for jp in sorted(LOGS.glob('*.jsonl')):
        m = re.search(r'(\d{8}_\d{6})', jp.name)
        if not m:
            continue
        for prefix in ('serial_raw_', 'udp_raw_'):
            rp = LOGS / f"{prefix}{m.group(1)}.log"
            if rp.exists():
                yield m.group(1), rp, jp
                break


def _wire_packets(raw: Path):
    out = []
    for line in raw.read_text(errors='replace').splitlines():
        if '\t' not in line:
            continue
        stamp, body = line.split('\t', 1)
        if not body.startswith('RTLS,'):
            continue
        pkt = FrameParser.parse(body)
        if pkt.valid:
            try:
                out.append((float(stamp), pkt))
            except ValueError:
                pass
    return out


def _replay(stamp, raw, jsonl):
    """Drive the pipeline from the wire and score against MATLAB's output."""
    S = read_session_log(jsonl)
    if not len(S) or not S.meta['has_ranges']:
        return None
    ref = {(int(S.tag[i]), int(S.t_ms[i])):
           (S.x[i], S.y[i], S.ex[i], S.ey[i], int(np.isfinite(S.R[i]).sum()))
           for i in range(len(S))}

    common = [(t, p) for t, p in _wire_packets(raw)
              if (int(p.tag_id), int(p.t_ms)) in ref]
    if len(common) < 50:
        return None

    # Recover whether RangeHold was on: a held sweep logs MORE anchors than the
    # wire carried, because holding injects the missing ones before logging.
    extra = np.mean([ref[(int(p.tag_id), int(p.t_ms))][4] - len(p.ids)
                     for _, p in common])
    hold = bool(extra > 0.02)

    cfg = AnchorConfig.from_json(str(ANCHORS))
    rc = RangeCorrection.load(str(CONFIG / 'range_correction.json'))
    pipe = SweepPipeline(cfg, PipelineConfig(tag_z=0.24, hold=hold), range_corr=rc)

    d_raw, d_out = [], []
    for t, pkt in common:
        # dt is driven from the LOGGED host time, never the wall clock — a
        # replay that used real time would give the filter different dt and the
        # comparison would be meaningless.
        res = pipe.process(pkt, t, imu=pkt.imu)
        x, y, ex, ey, _ = ref[(int(pkt.tag_id), int(pkt.t_ms))]
        if res.ok and np.isfinite(x):
            d_raw.append(np.hypot(res.raw[0] - x, res.raw[1] - y))
        if np.all(np.isfinite(res.out)) and np.isfinite(ex):
            d_out.append(np.hypot(res.out[0] - ex, res.out[1] - ey))
    if not d_raw or not d_out:
        return None
    return {'stamp': stamp, 'hold': hold, 'n': len(d_out),
            'raw_med_mm': float(np.median(d_raw) * 1000),
            'out_med_mm': float(np.median(d_out) * 1000)}


# Replaying ~35 sessions costs a few seconds; do it once for the whole module.
_RESULTS: list | None = None


def results() -> list:
    global _RESULTS
    if _RESULTS is None:
        _RESULTS = [r for r in (_replay(*p) for p in _paired()) if r]
    return _RESULTS


def test_the_paired_corpus_is_present():
    r = results()
    assert len(r) >= 20, f"only {len(r)} replayable paired sessions"
    assert sum(x['n'] for x in r) > 10000


def test_the_solve_reproduces_matlab_to_log_resolution():
    """`x,y` is `solve_sweep`, and it must be exact almost everywhere.

    A handful of sessions were recorded against a different anchor layout, so
    the assertion is on the median session rather than on every one — a
    geometry mismatch is a property of the corpus, not a defect in the solver.
    """
    r = results()
    med = np.median([x['raw_med_mm'] for x in r])
    exact = [x for x in r if x['raw_med_mm'] < LOG_RESOLUTION_MM]
    assert med < LOG_RESOLUTION_MM, f"median session solve error {med:.3f} mm"
    assert len(exact) >= 0.8 * len(r), (
        f"only {len(exact)}/{len(r)} sessions solve to log resolution")


def test_a_substantial_set_of_sessions_reproduces_ex_ey_exactly():
    """The EKF + pin deadband + output EMA, pinned end to end.

    Exactness on any session at all is the strong claim: it means every stage
    matches MATLAB bit-for-bit given the same inputs and settings. Sessions that
    do not match ran under `live_tag` options the log does not record, so they
    are counted, not asserted on.
    """
    r = results()
    exact = sorted([x for x in r if x['out_med_mm'] < 1.0],
                   key=lambda x: -x['n'])
    detail = ', '.join(f"{x['stamp']}({x['n']})" for x in exact[:8])
    assert len(exact) >= 5, (
        f"only {len(exact)}/{len(r)} sessions reproduce ex,ey exactly. "
        f"Matching: {detail or 'none'}")
    assert sum(x['n'] for x in exact) >= 800, "too few exactly-matched sweeps"


def test_rangehold_state_is_recoverable_from_the_logs():
    """`hold` is not logged, but holding leaves a fingerprint in the data.

    A held sweep records more anchors than the wire carried. Both populations
    must exist in the corpus, or the recovery is untested and the equivalence
    result above would be resting on an assumption.
    """
    r = results()
    on = [x for x in r if x['hold']]
    off = [x for x in r if not x['hold']]
    assert on and off, f"corpus is not mixed: {len(on)} held, {len(off)} not"


def test_replaying_the_same_session_twice_is_deterministic():
    """No wall-clock or RNG dependence: the same input must give one answer."""
    first = next(iter(_paired()), None)
    if first is None:
        pytest.skip("no paired session")
    a, b = _replay(*first), _replay(*first)
    assert a is None or (a['raw_med_mm'] == b['raw_med_mm']
                         and a['out_med_mm'] == b['out_med_mm'])
