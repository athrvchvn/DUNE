"""Rover capture: does what we write come back out, and can MATLAB read it?

The whole value of this module is interoperability, so the central test is a
round trip — capture a synthetic run, then score it with the same
`rover_run_report` that consumes MATLAB's own captures. If the schema is wrong
in any dimension, transposition or field name, that fails.

The second theme is crash safety: the `.ndjson` is appended as the run happens,
so a run that dies mid-drive still leaves everything up to that moment.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip('h5py', reason="h5py is needed to write MATLAB v7.3")

from reports.matfile import load_mat                 # noqa: E402
from rover.capture import RunCapture                 # noqa: E402

ANCHORS = [1, 2, 3, 4, 5]
T0 = 1_800_000_000.0


class Pkt:
    """The minimal sweep shape the capture consumes."""

    def __init__(self, tag, ids, dist, rx=None, fp=None, imu=None, t_ms=0):
        self.tag_id = tag
        self.ids = ids
        self.dist = dist
        self.rx = rx if rx is not None else [-80.0] * len(ids)
        self.fp = fp if fp is not None else [-84.0] * len(ids)
        self.imu = imu
        self.t_ms = t_ms


class Imu:
    quat = [1.0, 0.0, 0.0, 0.0]
    acc = [0.1, 0.2, 9.8]
    gyro = [0.01, 0.02, 0.03]


def _capture(tmp_path, **kw):
    return RunCapture(tmp_path, anchor_ids=ANCHORS, front_tag=241, rear_tag=240,
                      baseline_m=0.527, apriltag_offset_body=(-0.1135, -0.10),
                      anchors_layout='test-layout', **kw)


def _simple_run(cap, n=20):
    for i in range(n):
        t = T0 + i * 0.1
        cap.add_sweep(Pkt(241 if i % 2 == 0 else 240, ANCHORS,
                          [1.0 + 0.01 * i] * 5, imu=Imu() if i % 2 == 0 else None),
                      t_host=t)
        if i % 2 == 0:
            cap.add_rover(v_cmd=0.05, omega_cmd=0.2,
                          truth_xy=[2.0 + 0.01 * i, 1.5, 0.24],
                          truth_yaw_deg=10.0 * i, t_host=t)


# ── the round trip ────────────────────────────────────────────────────────────

def test_a_captured_run_can_be_scored_by_the_report(tmp_path):
    """The point of the whole module: either pipeline can read either capture."""
    from pathlib import Path

    from reports.rover import rover_report
    from rtls.anchor_config import AnchorConfig

    # The SAME geometry the synthetic ranges are generated from. Passing None
    # here silently falls back to anchors.json — a different, 4-anchor layout —
    # and the baseline check then measures the mismatch instead of the capture.
    anchors_path = (Path(__file__).resolve().parents[2] / 'matlab' / 'config'
                    / 'anchors_kinect_clicked_20260724.json')
    cfg = AnchorConfig.from_json(str(anchors_path))
    A = cfg.coords
    rng = np.random.RandomState(0)

    cap = RunCapture(tmp_path, anchor_ids=cfg.ids, front_tag=241, rear_tag=240,
                     baseline_m=0.527, apriltag_offset_body=(-0.1135, -0.10))
    for i in range(120):
        t = T0 + i * 0.1
        psi = 0.2 * i * 0.1
        c = np.array([2.0 + 0.05 * i * 0.1, 1.5 + 0.03 * i * 0.1])
        front = (i % 2 == 0)
        u = 0.2635 * np.array([np.cos(psi), np.sin(psi)])
        p = c + u if front else c - u
        d = (np.hypot(np.hypot(*(A[:, :2] - p).T), A[:, 2] - 0.24)
             + rng.randn(len(cfg.ids)) * 0.02)
        cap.add_sweep(Pkt(241 if front else 240, list(cfg.ids), list(d)), t_host=t)
        if front:
            cap.add_rover(v_cmd=0.05, omega_cmd=0.2,
                          truth_xy=[c[0], c[1], 0.24],
                          truth_yaw_deg=float(np.degrees(psi)), t_host=t)
    res = cap.close()
    assert res['mat'], res

    r = rover_report(res['mat'], tmp_path / 'rep',
                     anchors_file=str(anchors_path), tag_z=0.24)
    assert set(r['tags']) == {'240', '241'}
    for v in r['tags'].values():
        assert v['scored'] > 20, "the report found almost no scorable samples"
    # Truth-free consistency: the inter-tag distance must come back near nominal.
    assert abs(r['baseline']['err_median_mm']) < 60, r['baseline']


# ── schema ────────────────────────────────────────────────────────────────────

def test_the_mat_has_the_fields_rover_run_report_reads(tmp_path):
    cap = _capture(tmp_path)
    _simple_run(cap)
    cap.close()

    m = load_mat(cap.path_mat)
    assert {'data', 'metadata'} <= set(m)
    d, meta = m['data'], m['metadata']
    for k in ('t_posix', 'time', 'V_cmd', 'omega_cmd', 'apriltag_pos',
              'apriltag_euler', 'uwb'):
        assert k in d, f"data.{k} missing"
    for k in ('t_posix', 'tag', 'anchorIds', 'R', 'RX', 'FP', 'quat', 'gyro'):
        assert k in d['uwb'], f"data.uwb.{k} missing"
    for k in ('baseline_m', 'frontTag', 'rearTag', 'apriltag_offset_body'):
        assert k in meta['uwb'], f"metadata.uwb.{k} missing"


def test_array_orientation_survives_the_hdf5_round_trip(tmp_path):
    """MATLAB is column-major, so everything is written transposed.

    Get this wrong and the data reads as a few very long samples instead of many
    short ones — with no error, just nonsense.
    """
    cap = _capture(tmp_path)
    _simple_run(cap, n=20)
    cap.close()

    d = load_mat(cap.path_mat)['data']
    n_uwb, n_rover = 20, 10
    assert np.asarray(d['uwb']['R']).shape == (n_uwb, len(ANCHORS))
    assert np.asarray(d['uwb']['quat']).shape == (n_uwb, 4)
    assert np.asarray(d['uwb']['gyro']).shape == (n_uwb, 3)
    assert np.asarray(d['apriltag_pos']).shape == (n_rover, 3)
    assert np.asarray(d['t_posix']).size == n_rover


def test_ranges_are_placed_by_anchor_id_not_sweep_order(tmp_path):
    """A sweep that drops an anchor must leave a NaN hole, not shift columns."""
    cap = _capture(tmp_path)
    cap.add_sweep(Pkt(240, [1, 3, 5], [1.0, 3.0, 5.0]), t_host=T0)
    cap.close()

    R = np.asarray(load_mat(cap.path_mat)['data']['uwb']['R'])
    assert R.shape == (1, 5)
    assert np.allclose(R[0], [1.0, np.nan, 3.0, np.nan, 5.0], equal_nan=True)


def test_strings_are_written_as_matlab_char_arrays(tmp_path):
    from reports.matfile import as_text
    cap = _capture(tmp_path)
    _simple_run(cap, n=4)
    cap.close()
    layout = load_mat(cap.path_mat)['metadata']['uwb']['anchors_layout']
    assert as_text(layout) == 'test-layout'


def test_one_clock_stamps_both_streams(tmp_path):
    """Alignment is exact rather than inferred, so both must share t_host."""
    cap = _capture(tmp_path)
    cap.add_sweep(Pkt(240, ANCHORS, [1.0] * 5), t_host=T0 + 1.0)
    cap.add_rover(v_cmd=0.0, omega_cmd=0.0, t_host=T0 + 1.0)
    cap.close()

    d = load_mat(cap.path_mat)['data']
    assert np.asarray(d['uwb']['t_posix']).ravel()[0] == pytest.approx(T0 + 1.0)
    assert np.asarray(d['t_posix']).ravel()[0] == pytest.approx(T0 + 1.0)


# ── crash safety ──────────────────────────────────────────────────────────────

def test_the_ndjson_is_written_as_the_run_happens(tmp_path):
    """A run that dies mid-drive must still leave everything up to that moment."""
    cap = _capture(tmp_path)
    for i in range(5):
        cap.add_sweep(Pkt(240, ANCHORS, [1.0] * 5), t_host=T0 + i)

    rows = [json.loads(l) for l in open(cap.path_ndjson) if l.strip()]
    assert rows[0]['type'] == 'capture_meta'
    assert sum(1 for r in rows if r.get('type') == 'uwb') == 5, (
        "sweeps were buffered instead of being flushed")
    cap.close()


def test_close_is_idempotent(tmp_path):
    cap = _capture(tmp_path)
    _simple_run(cap, n=4)
    a = cap.close()
    b = cap.close()
    assert a['stamp'] == b['stamp']


def test_recording_after_close_is_ignored_not_an_error(tmp_path):
    """A late sweep from the ingest thread must not raise into it."""
    cap = _capture(tmp_path)
    _simple_run(cap, n=4)
    cap.close()
    cap.add_sweep(Pkt(240, ANCHORS, [1.0] * 5))     # must not raise
    cap.add_rover(v_cmd=0.0, omega_cmd=0.0)
    assert cap.stats.n_uwb == 4, "a post-close sweep was recorded"
    assert cap.stats.n_rover == 2


def test_an_empty_run_still_produces_a_readable_file(tmp_path):
    """Stopping a capture that never saw a sweep must not leave a broken .mat."""
    cap = _capture(tmp_path)
    res = cap.close()
    assert res['mat'], res
    d = load_mat(cap.path_mat)['data']
    assert np.asarray(d['uwb']['R']).size == 0


def test_the_summary_is_json_safe(tmp_path):
    from host.bus import dumps
    cap = _capture(tmp_path)
    _simple_run(cap, n=6)
    text = dumps(cap.close())
    assert 'NaN' not in text
    json.loads(text)


# ── the whole loop: sweeps in, report out ────────────────────────────────────

import asyncio       # noqa: E402
import functools     # noqa: E402


def runs_async(fn):
    @functools.wraps(fn)
    def wrapper(*a, **kw):
        return asyncio.run(fn(*a, **kw))
    return wrapper


def _app(tmp_path):
    from pathlib import Path

    from host.app import App
    import host.jobs_offline                          # noqa: F401
    import host.rover_api as rapi
    from rtls.anchor_config import AnchorConfig

    app = App(log_path=tmp_path / 's.ndjson')
    app.set_config_msg(lambda: {'type': 'config'})
    app.register_handlers(rapi.HANDLERS)
    app.state.anchors = AnchorConfig.from_json(str(
        Path(__file__).resolve().parents[2] / 'matlab' / 'config'
        / 'anchors_kinect_clicked_20260724.json'))
    return app, rapi


def _rows(app):
    app.recorder._fh.flush()
    return [json.loads(l) for l in open(app.recorder.path) if l.strip()]


@runs_async
async def test_stopping_a_capture_scores_it_without_being_asked(tmp_path):
    """A run nobody scores is a run nobody learns from, and the moment the
    operator cares is right now, standing next to the rover."""
    app, rapi = _app(tmp_path)
    try:
        await rapi.h_capture_start(app, {'baseline': 0.527}, None)
        cap = app.state.capture
        assert cap is not None

        A = app.state.anchors.coords
        ids = list(app.state.anchors.ids)
        rng = np.random.RandomState(1)
        for i in range(120):
            t = T0 + i * 0.1
            psi = 0.2 * i * 0.1
            c = np.array([2.0 + 0.05 * i * 0.1, 1.5 + 0.03 * i * 0.1])
            front = (i % 2 == 0)
            u = 0.2635 * np.array([np.cos(psi), np.sin(psi)])
            p = c + u if front else c - u
            d = (np.hypot(np.hypot(*(A[:, :2] - p).T), A[:, 2] - 0.24)
                 + rng.randn(len(ids)) * 0.02)
            cap.add_sweep(Pkt(241 if front else 240, ids, list(d)), t_host=t)
            if front:
                cap.add_rover(v_cmd=0.05, omega_cmd=0.2,
                              truth_xy=[c[0], c[1], 0.24],
                              truth_yaw_deg=float(np.degrees(psi)), t_host=t)

        await rapi.h_capture_stop(app, {}, None)
        assert app.state.capture is None, "the capture was not cleared"

        for _ in range(300):
            await asyncio.sleep(0.05)
            done = [r for r in _rows(app) if r['type'] in ('job_done', 'job_failed')]
            if done:
                break
        assert done, "the report job never ran"
        assert done[-1]['type'] == 'job_done', done[-1].get('error')

        result = done[-1]['result']
        assert set(result['tags']) == {'240', '241'}
        assert any(a['path'].endswith('report.txt')
                   for a in done[-1]['artifacts']), done[-1]['artifacts']
    finally:
        await app.aclose()


@runs_async
async def test_a_second_capture_cannot_start_over_a_running_one(tmp_path):
    app, rapi = _app(tmp_path)
    try:
        await rapi.h_capture_start(app, {}, None)
        first = app.state.capture
        await rapi.h_capture_start(app, {}, None)
        assert app.state.capture is first, "the second start replaced the first"
        assert any(r['type'] == 'capture_error' for r in _rows(app))
    finally:
        if app.state.capture is not None:
            app.state.capture.close()
        await app.aclose()


@runs_async
async def test_stopping_without_a_capture_is_reported_not_ignored(tmp_path):
    app, rapi = _app(tmp_path)
    try:
        await rapi.h_capture_stop(app, {}, None)
        errs = [r for r in _rows(app) if r['type'] == 'capture_error']
        assert errs and 'no capture' in errs[-1]['error']
    finally:
        await app.aclose()


@runs_async
async def test_the_report_can_be_suppressed(tmp_path):
    """Useful when capturing back-to-back runs; the .mat is still written."""
    app, rapi = _app(tmp_path)
    try:
        await rapi.h_capture_start(app, {}, None)
        app.state.capture.add_sweep(Pkt(240, list(app.state.anchors.ids),
                                        [1.0] * len(app.state.anchors.ids)),
                                    t_host=T0)
        await rapi.h_capture_stop(app, {'report': False}, None)
        await asyncio.sleep(0.3)
        assert not [r for r in _rows(app) if r['type'] == 'job_started']
        status = [r for r in _rows(app) if r['type'] == 'capture_status'][-1]
        assert status['mat'], "the .mat was not written"
    finally:
        await app.aclose()
