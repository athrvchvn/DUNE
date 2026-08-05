"""`rtls.metrics` — trajectory scoring, pinned to MATLAB's definitions.

These are small, exact tests rather than corpus tests, because the failure mode
here is not "crashes on real data" but "quietly computes a slightly different
number than MATLAB and nobody notices for a month". Every value below is
derivable by hand from the MATLAB source being ported.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rtls.metrics import (VisionLog, align_truth, prctile,       # noqa: E402
                          read_vision_log, trajectory_metrics)


# ── prctile: the definition MATLAB uses, which is not NumPy's default ─────────

def test_prctile_matches_matlabs_plotting_position():
    """MATLAB puts sample i of n at percentile 100*(i-0.5)/n and interpolates.

    Checked against a direct implementation of that rule, not against a value
    someone remembered — `prctile(1:10, 95)` is 10 in MATLAB but 9.55 under
    NumPy's default method, and every p95 in this project is compared against a
    MATLAB-produced number.
    """
    def reference(x, p):
        x = np.sort(np.asarray(x, float))
        pos = 100.0 * (np.arange(1, x.size + 1) - 0.5) / x.size
        return float(np.interp(p, pos, x))          # np.interp clamps, as MATLAB does

    rng = np.random.RandomState(0)
    for n in (1, 2, 7, 10, 33, 100, 501):
        x = rng.rand(n) * 10
        for p in (0, 1, 5, 25, 50, 75, 90, 95, 99, 100):
            assert prctile(x, p) == pytest.approx(reference(x, p), abs=1e-12), (n, p)

    assert prctile(np.arange(1, 11), 95) == pytest.approx(10.0)


def test_prctile_ignores_nans_and_survives_an_empty_input():
    assert prctile([1.0, np.nan, 3.0], 50) == pytest.approx(2.0)
    assert np.isnan(prctile([], 50))
    assert np.isnan(prctile([np.nan, np.nan], 50))


# ── trajectory_metrics ────────────────────────────────────────────────────────

def test_metrics_on_a_hand_computable_case():
    est = np.array([[0.0, 0.0], [3.0, 4.0], [1.0, 0.0]])
    truth = np.zeros((3, 2))                        # errors: 0, 5, 1
    X = trajectory_metrics(est, truth)
    assert X.n == 3
    assert X.rmse == pytest.approx(np.sqrt((0 + 25 + 1) / 3))
    assert X.mean == pytest.approx(2.0)
    assert X.median == pytest.approx(1.0)
    assert X.max == pytest.approx(5.0)


def test_rows_with_nan_in_either_input_are_excluded_not_scored_as_zero():
    """Masked dropouts must leave the statistics, not flatter them."""
    est = np.array([[0.0, 0.0], [3.0, 4.0], [10.0, 0.0]])
    truth = np.array([[0.0, 0.0], [0.0, 0.0], [np.nan, np.nan]])
    X = trajectory_metrics(est, truth)
    assert X.n == 2, "the masked sample was counted"
    assert X.max == pytest.approx(5.0), "the masked sample leaked into max"
    assert np.isnan(X.err[2])


def test_all_masked_input_reports_nan_rather_than_a_flattering_zero():
    X = trajectory_metrics(np.zeros((2, 2)), np.full((2, 2), np.nan))
    assert X.n == 0 and np.isnan(X.rmse) and np.isnan(X.p95)


def test_zone_map_bins_on_truth_position():
    """Zones answer "where is it bad", so binning must use truth, not estimate."""
    est = np.array([[0.1, 0.1], [5.0, 0.0]])
    truth = np.array([[0.0, 0.0], [5.0, 0.0]])
    X = trajectory_metrics(est, truth, bounds=(0.0, 6.0, 0.0, 2.0), cell_m=1.0)
    assert X.zone['rmse'].shape == (2, 6)
    assert X.zone['n'][0, 0] == 1 and X.zone['n'][0, 5] == 1
    assert X.zone['rmse'][0, 0] == pytest.approx(np.hypot(0.1, 0.1))
    assert np.isnan(X.zone['n'][1, 3]), "empty zones must be NaN, not 0"


def test_zone_edges_always_reach_the_upper_bound():
    """A non-integer span must not silently drop its last strip."""
    X = trajectory_metrics(np.zeros((1, 2)), np.zeros((1, 2)),
                           bounds=(0.0, 6.3, 0.0, 3.1), cell_m=1.0)
    assert X.zone['x_edges'][-1] == pytest.approx(6.3)
    assert X.zone['y_edges'][-1] == pytest.approx(3.1)


def test_shape_mismatch_is_an_error_not_a_broadcast():
    with pytest.raises(ValueError):
        trajectory_metrics(np.zeros((3, 2)), np.zeros((4, 2)))


# ── align_truth ───────────────────────────────────────────────────────────────

def _vision(t, xy, yaw=None):
    t = np.asarray(t, float)
    return VisionLog(t=t, xy=np.asarray(xy, float),
                     z=np.zeros(t.size),
                     yaw_deg=np.zeros(t.size) if yaw is None else np.asarray(yaw, float))


def test_truth_is_interpolated_onto_the_uwb_timestamps():
    # max_gap_s is generous here so the check is about interpolation alone; the
    # dropout mask gets its own test below. (The real camera runs ~30 Hz, so the
    # 0.2 s default would legitimately reject 1 Hz fixture data.)
    V = _vision([0.0, 1.0, 2.0], [[0.0, 0.0], [1.0, 2.0], [2.0, 4.0]])
    G = align_truth(V, np.array([0.5, 1.5]), max_gap_s=1.0)
    assert np.allclose(G.xy, [[0.5, 1.0], [1.5, 3.0]])
    assert G.coverage == pytest.approx(1.0)


def test_a_dropout_is_masked_rather_than_interpolated_across():
    """The failure this prevents: pairing a fix with truth from a second ago
    and reporting the result as sub-centimetre accuracy."""
    V = _vision([0.0, 3.0], [[0.0, 0.0], [3.0, 0.0]])
    G = align_truth(V, np.array([0.0, 1.5, 3.0]), max_gap_s=0.2)
    assert np.isfinite(G.xy[0, 0]) and np.isfinite(G.xy[2, 0])
    assert np.isnan(G.xy[1, 0]), "sample inside a 3 s camera gap was not masked"
    assert G.valid.tolist() == [True, False, True]
    assert G.coverage == pytest.approx(2 / 3)


def test_samples_outside_the_vision_span_are_nan():
    V = _vision([1.0, 2.0], [[0.0, 0.0], [1.0, 0.0]])
    G = align_truth(V, np.array([0.0, 1.5, 5.0]), max_gap_s=1.0)
    assert np.isnan(G.xy[0, 0]) and np.isnan(G.xy[2, 0])
    assert np.isfinite(G.xy[1, 0])


def test_yaw_interpolation_goes_the_short_way_around_the_wrap():
    """Naive interpolation from +170 to -170 sweeps 340 deg through zero."""
    V = _vision([0.0, 1.0], [[0.0, 0.0], [0.0, 0.0]], yaw=[170.0, -170.0])
    G = align_truth(V, np.array([0.5]), max_gap_s=1.0)
    assert abs(abs(G.yaw_deg[0]) - 180.0) < 1e-6, f"got {G.yaw_deg[0]}"


def test_lever_arm_is_rotated_into_the_world_frame():
    """A 1 m forward offset with the body at 90 deg must move +y, not +x."""
    V = _vision([0.0, 2.0], [[0.0, 0.0], [0.0, 0.0]], yaw=[90.0, 90.0])
    G = align_truth(V, np.array([1.0]), lever=(1.0, 0.0), max_gap_s=1.0)
    assert np.allclose(G.xy[0], [0.0, 1.0], atol=1e-9)


def test_time_offset_shifts_the_vision_clock():
    V = _vision([0.0, 1.0], [[0.0, 0.0], [1.0, 0.0]])
    G = align_truth(V, np.array([0.5]), time_offset=0.5, max_gap_s=1.0)
    assert G.xy[0, 0] == pytest.approx(0.0)


# ── read_vision_log ───────────────────────────────────────────────────────────

def test_vision_csv_is_sorted_and_deduplicated(tmp_path):
    """Interpolation needs strictly increasing knots; the exporter repeats frames."""
    p = tmp_path / 'v.csv'
    p.write_text("PosixTime_s,X_m,Y_m,Z_m,Yaw_deg\n"
                 "2.0,2.0,0.0,0.2,10\n"
                 "1.0,1.0,0.0,0.2,5\n"
                 "1.0,9.9,0.0,0.2,99\n")     # duplicate timestamp
    V = read_vision_log(p)
    assert V.t.tolist() == [1.0, 2.0]
    assert V.xy[0, 0] == pytest.approx(1.0), "kept the wrong row of the duplicate pair"
    assert V.rate_hz == pytest.approx(1.0)


def test_vision_csv_without_yaw_is_readable(tmp_path):
    p = tmp_path / 'v.csv'
    p.write_text("PosixTime_s,X_m,Y_m\n1.0,1.0,2.0\n2.0,3.0,4.0\n")
    V = read_vision_log(p)
    assert np.all(np.isnan(V.yaw_deg))


def test_a_non_apriltag_csv_is_rejected_with_a_useful_message(tmp_path):
    p = tmp_path / 'v.csv'
    p.write_text("time,a,b\n1,2,3\n")
    with pytest.raises(ValueError, match='PosixTime_s'):
        read_vision_log(p)


def test_the_default_gap_matches_a_real_camera_rate():
    """At the Kinect's ~30 Hz a 0.2 s gap really is a dropout, so the default
    must accept normal frames and reject only genuine holes."""
    t = np.arange(0.0, 2.0, 1 / 30)
    V = _vision(t, np.column_stack([t, np.zeros_like(t)]))
    G = align_truth(V, np.array([0.5, 1.0, 1.5]))
    assert G.valid.all() and G.coverage == pytest.approx(1.0)
