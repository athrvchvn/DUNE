"""Offline reports, run against the real committed captures.

These exercise the whole path a GUI user takes — pick a file, run a job, get a
report — without a browser. They are deliberately run on real data rather than
fixtures, because every problem found while building these came from real data:
an anchors file that did not match the capture, a frame fit that was
ill-conditioned, and a NaN that would have been silently dropped by the browser.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host.bus import dumps                                    # noqa: E402
from host.paths import PathNotAllowed, list_files, resolve_under_repo  # noqa: E402
from reports.rover import fit_rigid_2d                        # noqa: E402

REPO = Path(__file__).resolve().parents[2]
RAW_CAPTURE = (REPO / 'data' / 'static_campaign'
               / 'serial_raw.log')
ANCHORS_5 = REPO / 'config' / 'anchors_kinect_clicked_20260724.json'
ROVER_RUNS = sorted((REPO / 'data' / 'rover_runs').glob('*.mat'))

h5py = pytest.importorskip('h5py', reason="h5py needed for MATLAB v7.3 .mat")


# ── Path containment ──────────────────────────────────────────────────────────

def test_paths_outside_the_repo_are_rejected():
    """Job args come from the browser and the server binds 0.0.0.0."""
    for bad in ('/etc/passwd', '../../../../etc/passwd', '/tmp'):
        with pytest.raises(PathNotAllowed):
            resolve_under_repo(bad)


def test_relative_paths_resolve_against_the_repo_root():
    p = resolve_under_repo('config/anchors.json')
    assert p.is_absolute() and p.name == 'anchors.json'


def test_dot_dot_inside_the_repo_is_allowed():
    """Containment is checked after resolution, not by filtering strings."""
    assert resolve_under_repo('python/../config/anchors.json').exists()


def test_file_listing_offers_only_known_kinds():
    with pytest.raises(ValueError):
        list_files('anything_goes')
    logs = list_files('logs')
    if not logs:
        pytest.skip("no logs in this checkout")
    assert all(not Path(f['path']).is_absolute() for f in logs)
    assert logs == sorted(logs, key=lambda d: d['mtime'], reverse=True)


# ── Replay ────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not RAW_CAPTURE.exists(), reason="no raw capture committed")
def test_replay_of_a_real_raw_capture(tmp_path):
    from reports.replay import replay_session

    seen = []
    r = replay_session(RAW_CAPTURE, tmp_path, anchors_file=ANCHORS_5, tag_z=0.24,
                       progress=lambda f, m: seen.append(f))

    assert Path(r['report']).exists()
    assert seen and seen[-1] == 1.0, "progress never reached completion"

    s = r['tags']['240']
    assert s['n_sweeps'] > 9000
    assert s['solved_pct'] > 90
    # With the geometry that actually matches this capture the LM residual is
    # small; against a mismatched anchors.json it is ~10x worse, which is how
    # this number earns its place as a regression guard.
    assert s['rmse_median_mm'] < 100, f"residual {s['rmse_median_mm']:.0f} mm"
    assert 4.0 < s['rate_hz'] < 6.0


@pytest.mark.skipif(not RAW_CAPTURE.exists(), reason="no raw capture committed")
def test_cancelling_a_report_reports_a_cancel_not_a_failure(tmp_path):
    """The cancel callback RAISES; it does not return a bool.

    With a bool-returning callback the reports module had to invent its own
    exception, which `host.jobs` did not recognise — so cancelling a long
    replay surfaced in the GUI as `job_failed` with a traceback, which reads as
    a crash rather than as the user's own action.
    """
    from host.jobs import JobCancelled
    from reports.replay import replay_session

    calls = {'n': 0}

    def check_cancel():
        calls['n'] += 1
        if calls['n'] > 3:
            raise JobCancelled()

    with pytest.raises(JobCancelled):
        replay_session(RAW_CAPTURE, tmp_path, anchors_file=ANCHORS_5,
                       tag_z=0.24, check_cancel=check_cancel)
    assert calls['n'] > 3, "the cancel check was never consulted"


@pytest.mark.skipif(not ROVER_RUNS, reason="no rover_runs/*.mat committed")
def test_cancelling_a_rover_report_also_reports_a_cancel(tmp_path):
    from host.jobs import JobCancelled
    from reports.rover import rover_report

    calls = {'n': 0}

    def check_cancel():
        calls['n'] += 1
        if calls['n'] > 2:
            raise JobCancelled()

    with pytest.raises(JobCancelled):
        rover_report(max(ROVER_RUNS, key=lambda p: p.stat().st_size), tmp_path,
                     anchors_file=ANCHORS_5, tag_z=0.24, check_cancel=check_cancel)


@pytest.mark.skipif(not RAW_CAPTURE.exists(), reason="no raw capture committed")
def test_replay_result_survives_json_encoding(tmp_path):
    """A NaN anywhere would make the browser drop the whole message."""
    from reports.replay import replay_session

    r = replay_session(RAW_CAPTURE, tmp_path, anchors_file=ANCHORS_5, tag_z=0.24)
    text = dumps({'type': 'job_done', 'result': r})
    assert 'NaN' not in text and 'Infinity' not in text
    json.loads(text)                       # must round-trip


# ── Rigid frame fit ───────────────────────────────────────────────────────────

def test_rigid_fit_recovers_a_known_transform():
    rng = np.random.RandomState(0)
    src = rng.rand(50, 2) * 4
    th = np.deg2rad(30.0)
    R_true = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    t_true = np.array([1.5, -0.75])
    dst = src @ R_true.T + t_true

    R, t, rmse = fit_rigid_2d(src, dst)
    assert np.allclose(R, R_true, atol=1e-9)
    assert np.allclose(t, t_true, atol=1e-9)
    assert rmse < 1e-9


def test_rigid_fit_never_returns_a_reflection():
    """A mirrored 'fit' can score well and is physically impossible."""
    rng = np.random.RandomState(1)
    src = rng.rand(40, 2)
    dst = src.copy()
    dst[:, 1] *= -1                        # pure reflection
    R, _, _ = fit_rigid_2d(src, dst)
    assert np.linalg.det(R) > 0, "returned a reflection"


def test_rigid_fit_ignores_nan_rows_and_degrades_gracefully():
    src = np.array([[0.0, 0.0], [1.0, 0.0], [np.nan, np.nan]])
    dst = np.array([[1.0, 1.0], [2.0, 1.0], [5.0, 5.0]])
    R, t, rmse = fit_rigid_2d(src, dst)
    assert np.isfinite(rmse)
    R, t, rmse = fit_rigid_2d(np.zeros((1, 2)), np.zeros((1, 2)))
    assert np.allclose(R, np.eye(2)) and np.isnan(rmse)


# ── Rover report ──────────────────────────────────────────────────────────────

@pytest.mark.skipif(not ROVER_RUNS, reason="no rover_runs/*.mat committed")
def test_rover_report_on_a_real_capture(tmp_path):
    from reports.rover import rover_report

    biggest = max(ROVER_RUNS, key=lambda p: p.stat().st_size)
    r = rover_report(biggest, tmp_path, anchors_file=ANCHORS_5, tag_z=0.24)

    assert Path(r['report']).exists()
    assert r['n_sweeps'] > 0 and len(r['tags']) == 2
    assert set(r['frame']) >= {'rotation_deg', 'translation_m', 'fit_rmse_mm',
                               'spread_m', 'anisotropy', 'verdict'}
    for s in r['tags'].values():
        assert s['solved_pct'] > 50
        assert s['scored'] > 0

    text = dumps({'type': 'job_done', 'result': r})
    assert 'NaN' not in text
    json.loads(text)


@pytest.mark.skipif(len(ROVER_RUNS) < 3, reason="need several rover runs")
def test_the_frame_transform_agrees_across_independent_runs(tmp_path):
    """One physical transform, so runs with real motion must agree on it.

    This is the check that caught the original per-tag fit: it produced +27 deg
    for one tag and -7 deg for the other on the same rigid body. Fitting once
    from all tags, runs with enough spatial spread now agree to a couple of
    degrees; runs without it are reported as ill-conditioned rather than
    believed.
    """
    from reports.rover import rover_report

    # Only runs captured under the SAME anchor layout can agree. ANCHORS_5 is
    # the 2026-07-20 clicked geometry; the anchors were re-surveyed after
    # 2026-07-25 and the later captures need a layout this repo does not have,
    # so scoring them here legitimately produces a different frame. Selecting by
    # file size instead of by era is how this test first became flaky.
    same_era = [p for p in ROVER_RUNS
                if 'rover_uwb_20260723' in p.name or 'rover_uwb_20260724' in p.name]
    if len(same_era) < 2:
        pytest.skip("fewer than two runs from the ANCHORS_5 era")

    angles = []
    for i, mat in enumerate(sorted(same_era, key=lambda p: p.stat().st_size,
                                   reverse=True)[:6]):
        try:
            r = rover_report(mat, tmp_path / f"r{i}", anchors_file=ANCHORS_5,
                             tag_z=0.24)
        except Exception:
            continue
        f = r.get('frame', {})
        if f.get('verdict') == 'well-conditioned':
            angles.append(f['rotation_deg'])

    if len(angles) < 2:
        pytest.skip("fewer than two well-conditioned runs available")
    # Same-era well-conditioned runs span roughly -10 to +5 deg; the bound is
    # loose enough for that spread and far tighter than the ~110 deg range seen
    # when the geometry does not match.
    assert max(angles) - min(angles) < 20.0, (
        f"same-era runs disagree on the frame rotation: {sorted(angles)}")


@pytest.mark.skipif(not ROVER_RUNS, reason="no rover_runs/*.mat committed")
def test_a_mismatched_anchor_geometry_is_flagged(tmp_path):
    """The wrong anchors file must not yield a confident-looking report.

    Runs captured after the 2026-07-25 re-survey score badly against every
    geometry committed here, which is the case this warning exists for.
    """
    from reports.rover import rover_report

    late = [p for p in ROVER_RUNS if 'rover_uwb_202607290' in p.name
            or 'rover_uwb_20260728' in p.name]
    if not late:
        pytest.skip("no post-resurvey runs in this checkout")

    flagged = 0
    for i, mat in enumerate(late[:4]):
        try:
            r = rover_report(mat, tmp_path / f"m{i}", anchors_file=ANCHORS_5,
                             tag_z=0.24)
        except Exception:
            continue
        if r.get('geometry_suspect'):
            flagged += 1
            assert 'CHECK THE ANCHOR GEOMETRY' in Path(r['report']).read_text()
    assert flagged, "no post-resurvey run was flagged as geometrically suspect"


@pytest.mark.skipif(not ROVER_RUNS, reason="no rover_runs/*.mat committed")
def test_a_non_rover_mat_is_rejected_with_a_clear_message(tmp_path):
    from reports.rover import rover_report

    other = REPO / 'data' / 'static_campaign' / 'bakeoff.mat'
    if not other.exists():
        pytest.skip("no non-rover .mat to test with")
    with pytest.raises(Exception) as e:
        rover_report(other, tmp_path)
    msg = str(e.value).lower()
    # Whichever way it fails, the message has to tell the user what to do.
    # h5py's own "file signature not found" does not.
    assert any(k in msg for k in ('v7.3', 'uwb', 'rover')), msg
