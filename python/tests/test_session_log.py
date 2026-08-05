"""`rtls.session_log` against every session log this repo actually contains.

The reader has to survive two dialects — MATLAB's `dune.sweepRecord` output and
Python's `SessionRecorder`, which interleaves every broadcast family — so the
useful test is not a handcrafted fixture but the 69 real logs sitting in
`matlab/logs/` and `python/logs/`. A parser that only works on invented input
is how the `anchor_diag`-as-dict bug survived the first time.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rtls.session_log import (discover_anchor_ids, read_session_log)  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
MATLAB_LOGS = sorted((REPO / 'matlab' / 'logs').glob('*.jsonl'))
PYTHON_LOGS = sorted((REPO / 'python' / 'logs').glob('*.jsonl'))


# ── Dialect quirks ────────────────────────────────────────────────────────────

def test_single_anchor_diag_decodes_as_a_dict_not_a_list(tmp_path):
    """MATLAB's jsonencode collapses a one-element struct array to an object.

    Every other sweep gives a list, so a reader that assumes a list silently
    drops the diagnostics of exactly the sweeps that had one anchor left — the
    worst ones, i.e. the ones being investigated.
    """
    p = tmp_path / 's.jsonl'
    p.write_text(
        json.dumps({'tag_id': 240, 't_ms': 10, 'r3': 1.5,
                    'anchor_diag': {'id': 3, 'rx': -80, 'fp': -84,
                                    'gap': 4.0, 'w': 0.4, 'used': True}}) + '\n'
        + json.dumps({'tag_id': 240, 't_ms': 20, 'r3': 1.6,
                      'anchor_diag': [{'id': 3, 'rx': -81, 'fp': -85,
                                       'gap': 4.0, 'w': 0.4, 'used': True}]}) + '\n')
    S = read_session_log(p)
    assert len(S) == 2
    c = S.column(3)
    assert np.allclose(S.RX[:, c], [-80, -81]), "dict-form diag was dropped"
    assert S.USED[:, c].all()


def test_anchor_columns_are_keyed_by_id_not_sweep_position(tmp_path):
    """A sweep that loses an anchor must leave a NaN hole, not shift columns."""
    p = tmp_path / 's.jsonl'
    p.write_text(
        json.dumps({'tag_id': 240, 't_ms': 1, 'r1': 1.0, 'r2': 2.0, 'r3': 3.0}) + '\n'
        + json.dumps({'tag_id': 240, 't_ms': 2, 'r1': 1.1, 'r3': 3.1}) + '\n')
    S = read_session_log(p)
    assert S.anchor_ids.tolist() == [1, 2, 3]
    assert np.allclose(S.R[0], [1.0, 2.0, 3.0])
    assert np.allclose(S.R[1], [1.1, np.nan, 3.1], equal_nan=True)


def test_gap_is_derived_when_absent_but_rx_and_fp_are_present(tmp_path):
    p = tmp_path / 's.jsonl'
    p.write_text(json.dumps({'tag_id': 240, 't_ms': 1,
                             'anchor_diag': [{'id': 1, 'rx': -80.0, 'fp': -86.5}]}) + '\n')
    S = read_session_log(p)
    assert S.GAP[0, 0] == pytest.approx(6.5)


def test_non_sweep_records_do_not_become_sweeps(tmp_path):
    """A Python session log interleaves config/job/robot messages with fixes."""
    p = tmp_path / 's.jsonl'
    p.write_text('\n'.join(json.dumps(r) for r in [
        {'type': 'config', 'anchors': [{'id': 1, 'x': 0, 'y': 0}]},
        {'type': 'job_started', 'job_id': 'job-1', 'kind': 'selftest'},
        {'type': 'tag_fix', 'tag_id': 240, 't_ms': 5, 'x': 1.0, 'y': 2.0, 'r1': 1.0},
        {'type': 'robot', 'cx': 1.0, 'cy': 2.0},
    ]) + '\n')
    S = read_session_log(p)
    assert len(S) == 1 and S.tag[0] == 240 and S.x[0] == 1.0


def test_truncated_final_line_ends_the_file_without_raising(tmp_path):
    """A killed process leaves a partial line; the rest must still be readable."""
    p = tmp_path / 's.jsonl'
    good = json.dumps({'tag_id': 240, 't_ms': 1, 'r1': 1.0})
    p.write_text(good + '\n' + good + '\n' + '{"tag_id": 240, "t_m')
    S = read_session_log(p)
    assert len(S) == 2 and S.meta['n_bad_lines'] == 1


def test_dual_tag_logs_split_cleanly_by_tag(tmp_path):
    p = tmp_path / 's.jsonl'
    p.write_text('\n'.join(json.dumps(r) for r in [
        {'tag_id': 240, 't_ms': 1, 'r1': 1.0},
        {'tag_id': 241, 't_ms': 2, 'r1': 2.0},
        {'tag_id': 240, 't_ms': 3, 'r1': 1.1},
    ]) + '\n')
    S = read_session_log(p)
    assert S.tags == [240, 241]
    a = S.for_tag(240)
    assert len(a) == 2 and np.allclose(a.R[:, 0], [1.0, 1.1])
    assert len(S.for_tag(241)) == 1


# ── Against the real corpus ───────────────────────────────────────────────────

@pytest.mark.skipif(not MATLAB_LOGS, reason="no matlab/logs/*.jsonl in this checkout")
def test_reads_every_matlab_log_without_error():
    """69 real logs, three writers, several dual-tag. None may raise."""
    total_sweeps = n_nonempty = n_ranged = 0
    for p in MATLAB_LOGS:
        S = read_session_log(p)                      # must not raise
        assert S.meta['n_bad_lines'] == 0, f"{p.name} had unparseable lines"
        total_sweeps += len(S)
        n_nonempty += 1 if len(S) else 0
        n_ranged += 1 if S.meta.get('has_ranges') else 0
        if len(S):
            assert np.all(np.isfinite(S.tag)), f"{p.name}: a sweep lost its tag id"
    assert n_nonempty >= 50 and total_sweeps > 20000, (
        f"corpus smaller than expected: {n_nonempty} logs, {total_sweeps} sweeps")
    assert n_ranged >= 40, f"only {n_ranged} logs carry ranges"


@pytest.mark.skipif(not MATLAB_LOGS, reason="no matlab/logs/*.jsonl in this checkout")
def test_position_only_rover_logs_are_read_as_such():
    """`live_rover.m` writes a third schema: solved poses, no ranges, no t_ms.

    It must not be mistaken for a truncated sweep log — and it is the only
    writer that records `still`, which Phase 2 needs and `sweepRecord.m` omits.
    """
    rover = [p for p in MATLAB_LOGS if p.name.startswith('rover_log_')]
    if not rover:
        pytest.skip("no rover_log_*.jsonl in this checkout")
    seen_still = False
    for p in rover:
        S = read_session_log(p)
        if not len(S):
            continue
        assert not S.meta['has_ranges'], f"{p.name} unexpectedly has ranges"
        assert S.meta['has_thost'], f"{p.name} must carry t_host to be alignable"
        # Not every row has a position: live_rover's coast-freeze guard emits a
        # row with no fix when the MHE declines to solve. 88-100% across the
        # corpus, so require a solid majority rather than all.
        have = float(np.mean(np.isfinite(S.x)))
        assert have > 0.8, f"{p.name}: only {have:.0%} of rows have a position"
        seen_still |= S.meta['has_still']
    assert seen_still, "no rover log recorded `still`"


@pytest.mark.skipif(not MATLAB_LOGS, reason="no matlab/logs/*.jsonl in this checkout")
def test_the_ekf_output_oracle_for_phase_2_is_present():
    """Phase 2 pins `ex/ey` against MATLAB. Check that data actually exists.

    `sweepRecord.m` only writes ex/ey when `live_tag` ran with the EKF enabled,
    so a corpus of raw-only logs would leave Phase 2 with no oracle. This asserts
    the corpus is usable rather than discovering it mid-port.
    """
    n_logs = n_sweeps = 0
    for p in MATLAB_LOGS:
        S = read_session_log(p)
        k = int(np.isfinite(S.ex).sum())
        if k:
            n_logs += 1
            n_sweeps += k
            assert S.meta['has_thost'], (
                f"{p.name} has ex/ey but no t_host — replay cannot drive dt "
                "from the log and the comparison would be meaningless")
    assert n_logs >= 40 and n_sweeps >= 15000, (
        f"only {n_sweeps} EKF-output sweeps in {n_logs} logs")


@pytest.mark.skipif(not PYTHON_LOGS, reason="no python/logs/*.jsonl in this checkout")
def test_reads_python_era_logs():
    for p in PYTHON_LOGS[:25]:
        read_session_log(p)                          # must not raise


@pytest.mark.skipif(not MATLAB_LOGS, reason="no matlab/logs/*.jsonl in this checkout")
def test_explicit_anchor_ids_fix_the_column_order():
    """Callers comparing against MATLAB need its 1:5 column order on demand."""
    p = next(q for q in MATLAB_LOGS if len(read_session_log(q)))
    S = read_session_log(p, anchor_ids=[1, 2, 3, 4, 5])
    assert S.anchor_ids.tolist() == [1, 2, 3, 4, 5]
    assert S.R.shape[1] == 5


def test_discover_anchor_ids_finds_ids_outside_matlabs_default_range(tmp_path):
    """readSessionLog.m defaults to 1:5 and silently drops anything else."""
    p = tmp_path / 's.jsonl'
    p.write_text(json.dumps({'tag_id': 240, 't_ms': 1, 'r7': 1.0, 'r9': 2.0}) + '\n')
    assert discover_anchor_ids(p).tolist() == [7, 9]
    assert read_session_log(p).R.shape == (1, 2)


# ── Raw wire tees ─────────────────────────────────────────────────────────────

def test_both_raw_tee_formats_are_read(tmp_path):
    """Serial writes 2 tab-separated fields, UDP writes 3 (it adds src_ip).

    The first version of the replay reader split on the FIRST tab and required
    the remainder to start with `RTLS,` — which is true for serial and false for
    UDP. Every udp_raw_*.log therefore read as empty, silently: no error, just a
    session that looked like it had no data.
    """
    from rtls.raw_log import is_raw_tee, read_raw_tee

    serial = tmp_path / 'serial_raw_x.log'
    serial.write_text("1784562647.555\tRTLS,v3,1,240,1,1,1000,-80,-84,10\n"
                      "1784562647.7\t[BOOT] reset_reason=POWERON\n")
    udp = tmp_path / 'udp_raw_x.log'
    udp.write_text("1784705187.033\t172.29.109.243\tRTLS,v4,1,240,1,1,1000,-80,-84,10,0,0\n"
                   "1784705187.3\t172.29.109.243\tDIAG,v1,1,2,3\n")

    for p, expect_t in ((serial, 1784562647.555), (udp, 1784705187.033)):
        assert is_raw_tee(p), f"{p.name} not recognised as a tee"
        rows = read_raw_tee(p)
        assert len(rows) == 1, f"{p.name}: expected 1 RTLS line, got {len(rows)}"
        assert rows[0][0] == pytest.approx(expect_t)
        assert rows[0][1].startswith('RTLS,')


def test_a_jsonl_is_not_mistaken_for_a_raw_tee(tmp_path):
    from rtls.raw_log import is_raw_tee
    p = tmp_path / 's.jsonl'
    p.write_text(json.dumps({'tag_id': 240, 't_ms': 1}) + '\n')
    assert not is_raw_tee(p)


@pytest.mark.skipif(not MATLAB_LOGS, reason="no corpus in this checkout")
def test_the_committed_udp_tees_are_readable():
    """The regression this guards: these all read as empty before the fix."""
    from rtls.raw_log import read_raw_tee
    udp = sorted((REPO / 'matlab' / 'logs').glob('udp_raw_*.log'))
    if not udp:
        pytest.skip("no udp_raw logs committed")
    total = sum(len(read_raw_tee(p)) for p in udp)
    assert total > 1000, f"only {total} RTLS lines across {len(udp)} udp tees"


# ── session_meta ──────────────────────────────────────────────────────────────

def test_session_meta_is_captured_when_present(tmp_path):
    """live_tag now records the options it ran under, on line one.

    This is the fix for the Phase 2 oracle problem: ex,ey depends on
    hold/pin/smooth/ekf/tagZ and on which anchors.json was loaded, and none of
    that used to be recorded. Of 35 paired sessions only 6 could be reproduced
    exactly, and `hold` had to be recovered indirectly from anchor counts.
    """
    p = tmp_path / 's.jsonl'
    p.write_text('\n'.join([
        json.dumps({'type': 'session_meta', 'schema': 'dune_session_meta_v1',
                    'anchors_layout': 'rect_6.3x3.1', 'anchors_n': 5,
                    'rangeCorr': True,
                    'opts': {'hold': True, 'pin': True, 'smooth': 0.6,
                             'tagZ': 0.24, 'ekf': True, 'estimator': 'ekf'}}),
        json.dumps({'tag_id': 240, 't_ms': 1, 'r1': 1.0, 'x': 1.0, 'y': 2.0,
                    'ex': 1.1, 'ey': 2.1}),
    ]) + '\n')

    S = read_session_log(p)
    assert len(S) == 1, "the meta line was counted as a sweep"
    assert S.meta['has_meta']
    o = S.meta['session_meta']['opts']
    assert o['hold'] is True and o['tagZ'] == 0.24 and o['smooth'] == 0.6
    assert S.meta['session_meta']['anchors_layout'] == 'rect_6.3x3.1'


def test_logs_written_before_session_meta_existed_still_read(tmp_path):
    """Every committed log predates it; none may start failing."""
    p = tmp_path / 'old.jsonl'
    p.write_text(json.dumps({'tag_id': 240, 't_ms': 1, 'r1': 1.0}) + '\n')
    S = read_session_log(p)
    assert len(S) == 1
    assert S.meta['has_meta'] is False and S.meta['session_meta'] is None


@pytest.mark.skipif(not MATLAB_LOGS, reason="no corpus in this checkout")
def test_the_whole_committed_corpus_reads_with_the_new_field():
    """has_meta must be False everywhere and nothing may have broken."""
    for p in MATLAB_LOGS:
        S = read_session_log(p)
        assert S.meta['has_meta'] is False, f"{p.name} unexpectedly has meta"
        assert S.meta['n_bad_lines'] == 0


def test_the_matlab_writer_emits_the_meta_line():
    """The MATLAB half of this cannot be executed here, so check the source.

    live_tag.m must write the record, and it must do so AFTER the variables it
    references are assigned — otherwise the script errors on the first run,
    which is a bad way to find out.
    """
    lt = REPO / 'matlab' / 'scripts' / 'live_tag.m'
    if not lt.exists():
        pytest.skip("live_tag.m not in this checkout")
    src = lt.read_text()
    assert 'dune_session_meta_v1' in src, "live_tag does not record its options"

    lines = src.splitlines()
    def first(pred):
        return next((i for i, l in enumerate(lines) if pred(l)), None)

    emit = first(lambda l: 'dune_session_meta_v1' in l)
    for name, pat in (('A', 'A = dune.loadAnchors'),
                      ('RC', 'RC = ')):
        assigned = first(lambda l, pat=pat: l.strip().startswith(pat))
        assert assigned is not None and assigned < emit, (
            f"live_tag.m references {name} at line {emit + 1} but assigns it "
            f"at {None if assigned is None else assigned + 1}")
