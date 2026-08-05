"""`rtls.ekf.FusionEkf` — the behaviours the session corpus cannot reach.

The equivalence test pins the EKF against MATLAB on real logs, which is the
stronger check but only covers what those sessions happened to do. None of them
ran with per-anchor range-bias states or the still-inversion noise budget, so
those are exercised here against the property each one exists to provide.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rtls.ekf import CHI2_95, FusionEkf, world_accel      # noqa: E402


def _ekf(**kw) -> FusionEkf:
    e = FusionEkf()
    for k, v in kw.items():
        setattr(e, k, v)
    return e


# ── basics ────────────────────────────────────────────────────────────────────

def test_initialize_sets_position_and_a_sane_covariance():
    e = _ekf()
    e.initialize([1.5, -2.0])
    assert np.allclose(e.pos, [1.5, -2.0])
    assert np.allclose(e.vel, 0) and np.allclose(e.bias, 0)
    assert np.allclose(np.diag(e.P), [0.25, 0.25, 1.0, 1.0, 0.01, 0.01])


def test_constant_velocity_prediction_moves_position_by_v_dt():
    e = _ekf()
    e.initialize([0.0, 0.0])
    e.x[2:4] = [1.0, -0.5]
    e.predict(0.2, None)
    assert np.allclose(e.pos, [0.2, -0.1])


def test_prediction_is_a_no_op_before_initialisation_or_with_dt_zero():
    e = _ekf()
    e.predict(0.1, None)
    assert not e.initialized
    e.initialize([1.0, 1.0])
    P0 = e.P.copy()
    e.predict(0.0, None)
    assert np.allclose(e.P, P0), "dt=0 must not inflate covariance"


def test_accel_bias_states_are_frozen_without_imu_input():
    """They are unobservable on a CV predict; growing their P is a lie."""
    e = _ekf()
    e.initialize([0.0, 0.0])
    before = e.P[4, 4]
    for _ in range(20):
        e.predict(0.1, None)
    assert e.P[4, 4] == pytest.approx(before), "bias covariance grew without IMU"


def test_imu_prediction_uses_acceleration_and_opens_the_bias_states():
    e = _ekf()
    e.initialize([0.0, 0.0])
    before = e.P[4, 4]
    e.predict(1.0, [2.0, 0.0])
    assert e.pos[0] == pytest.approx(1.0)      # ½·a·t²
    assert e.vel[0] == pytest.approx(2.0)
    assert e.P[4, 4] > before


# ── gating and robustness ─────────────────────────────────────────────────────

def test_a_gross_outlier_is_rejected():
    e = _ekf(robust=False)
    e.initialize([0.0, 0.0])
    assert e.update_position([0.05, 0.05], np.eye(2) * 0.01 ** 2)
    assert not e.update_position([50.0, 50.0], np.eye(2) * 0.01 ** 2)
    assert np.linalg.norm(e.pos) < 1.0, "a rejected fix still moved the state"


def test_huber_downweights_a_marginal_outlier_instead_of_discarding_it():
    """Bounded influence: the measurement is kept, its covariance inflated.

    1.5 m off a freshly initialised filter is NIS ~8.9 — past the 5.991 gate but
    well under the 59.91 hard-reject bound, which is precisely the band Huber
    exists for. (0.35 m is only NIS 0.5 and would be accepted outright.)
    """
    marginal = [1.5, 0.0]
    R = np.eye(2) * 0.05 ** 2

    hard = _ekf(robust=False)
    hard.initialize([0.0, 0.0])
    hard.update_position(marginal, R)

    soft = _ekf(robust=True)
    soft.initialize([0.0, 0.0])
    soft.update_position(marginal, R)

    assert not hard.last_accepted and hard.n_rejected == 1
    assert soft.last_accepted and soft.n_soft == 1
    assert 0 < soft.pos[0] < marginal[0], "soft update should move part-way"


def test_a_gross_outlier_is_hard_rejected_even_when_robust():
    """Beyond robust_hard_factor x gate there is no down-weighting to be had."""
    e = _ekf(robust=True)
    e.initialize([0.0, 0.0])
    e.update_position([5.0, 0.0], np.eye(2) * 0.05 ** 2)   # NIS ~99 > 59.91
    assert not e.last_accepted and e.n_soft == 0


def test_a_long_reject_streak_reanchors_the_filter():
    e = _ekf(robust=False, max_consec_reject=3)
    e.initialize([0.0, 0.0])
    for _ in range(3):
        e.update_position([80.0, 80.0], np.eye(2) * 0.01 ** 2)
    assert e.n_reinit == 1
    assert np.allclose(e.pos, [80.0, 80.0]), "did not re-anchor on the fixes"


def test_the_gate_widens_with_measurement_dimension():
    assert CHI2_95 == (3.841, 5.991, 7.815)


# ── stillness ─────────────────────────────────────────────────────────────────

def test_still_mode_and_zupt_together_let_a_parked_tag_pin():
    """Neither alone is enough, which is why the code uses both.

    still_mode caps the process noise, but position variance is dominated by
    the VELOCITY variance propagating through F, so on its own it only takes
    P00 from 5.96 to 4.26. ZUPT clamps the velocity, and the pair together give
    0.25 — a ~17x improvement over still_mode alone.
    """
    moving, still, pinned = _ekf(), _ekf(still_mode=True), _ekf(still_mode=True)
    for e in (moving, still, pinned):
        e.initialize([0.0, 0.0])
    for _ in range(10):
        moving.predict(0.2, None)
        still.predict(0.2, None)
        pinned.predict(0.2, None)
        pinned.update_zupt()

    assert still.P[0, 0] < moving.P[0, 0], "still mode did not damp position Q"
    assert pinned.P[0, 0] < still.P[0, 0] / 10, "ZUPT did not clamp the growth"
    assert pinned.P[2, 2] < 0.01, "velocity variance not clamped"


def test_still_inversion_moves_the_noise_budget_into_the_bias_states():
    """While parked the position is constant but the ranges keep breathing.

    Inverting the budget lets the breathing be absorbed by the per-anchor bias
    states instead of dragging the position around.
    """
    plain = _ekf(still_mode=True)
    plain.enable_range_bias(4)
    plain.initialize([0.0, 0.0])

    inv = _ekf(still_mode=True, still_invert=True)
    inv.enable_range_bias(4)
    inv.initialize([0.0, 0.0])

    for e in (plain, inv):
        for _ in range(10):
            e.predict(0.2, None)

    assert inv.P[0, 0] < plain.P[0, 0], "position Q was not clamped harder"
    assert inv.P[6, 6] > plain.P[6, 6], "bias random walk was not opened"


def test_zupt_pulls_velocity_toward_zero():
    e = _ekf()
    e.initialize([0.0, 0.0])
    e.x[2:4] = [0.02, 0.0]
    e.P[2, 2] = e.P[3, 3] = 1.0
    assert e.update_zupt()
    assert abs(e.vel[0]) < 0.02


# ── range-bias states ─────────────────────────────────────────────────────────

ANCHORS = np.array([[0.0, 0.0, 0.0], [6.0, 0.0, 0.0],
                    [6.0, 3.0, 0.0], [0.0, 3.0, 0.0]], dtype=float)


def _true_ranges(p, tag_z=0.24):
    d = ANCHORS[:, :2] - np.asarray(p, float)
    return np.hypot(np.hypot(d[:, 0], d[:, 1]), ANCHORS[:, 2] - tag_z)


# Deliberately NOT the centroid. At (3.0, 1.5) anchors 0 and 2 are exactly
# collinear with the tag, so a bias on one is indistinguishable from a position
# shift plus a bias on the other, and the filter splits it evenly between them.
# That degeneracy is real (it is why the delta-solve idea was deferred as
# "ill-conditioned near centroid"), but it makes for a test that measures the
# geometry rather than the filter.
TAG_XY = [2.0, 1.0]


def test_a_per_anchor_bias_is_learned_from_tightly_coupled_updates():
    e = _ekf()
    e.enable_range_bias(4)
    e.initialize(TAG_XY)
    truth = _true_ranges(TAG_XY)
    z = truth.copy()
    z[2] += 0.20                                # anchor 3 reads 20 cm long

    for _ in range(300):
        e.predict(0.1, None)
        e.update_ranges(ANCHORS, z, None, 0.24, 0.05)

    rb = e.range_bias
    # The biased anchor is clearly identified. It does not absorb the whole
    # 20 cm: with 4 ranges against 2 position + 4 bias unknowns the system is
    # underdetermined, so part of any offset is genuinely unobservable and
    # lands in position and in the diagonally-opposite anchor. That is the
    # documented behaviour ("common-mode bias stays in position"), not a defect.
    assert rb[2] > 0.05, f"biased anchor not learned: {rb}"
    assert rb[2] == max(rb), f"bias not attributed to the right anchor: {rb}"
    assert np.linalg.norm(e.pos - np.array(TAG_XY)) < 0.25, \
        f"position was dragged by the biased anchor: {e.pos}"


def test_a_learned_bias_is_remembered_while_its_anchor_is_silent():
    """The whole point: a changing anchor SUBSET must not jump the position."""
    e = _ekf()
    e.enable_range_bias(4)
    e.initialize(TAG_XY)
    z = _true_ranges(TAG_XY)
    z[2] += 0.20
    for _ in range(200):
        e.predict(0.1, None)
        e.update_ranges(ANCHORS, z, None, 0.24, 0.05)

    learned = e.range_bias[2]
    silent = z.copy()
    silent[2] = np.nan                          # that anchor stops answering
    for _ in range(20):
        e.predict(0.1, None)
        e.update_ranges(ANCHORS, silent, None, 0.24, 0.05)

    assert e.range_bias[2] == pytest.approx(learned, abs=0.02), "bias was forgotten"


def test_reinit_keeps_the_learned_biases_and_only_reopens_them():
    e = _ekf()
    e.enable_range_bias(4)
    e.initialize([1.0, 1.0])
    e.x[6:] = [0.01, -0.02, 0.03, 0.0]
    before = e.range_bias.copy()

    e.reinit_from([5.0, 2.0])

    assert np.allclose(e.range_bias, before), "reinit discarded learned biases"
    assert np.allclose(e.pos, [5.0, 2.0])
    assert e.P[6, 6] == pytest.approx((e.rb_sigma_init / 2) ** 2)
    assert e.n_reinit == 1


# ── world_accel ───────────────────────────────────────────────────────────────

def test_world_accel_identity_quaternion_passes_through():
    assert np.allclose(world_accel([1, 0, 0, 0], [1.0, 2.0, 3.0]), [1.0, 2.0])


def test_world_accel_rotates_by_yaw():
    s = np.sqrt(0.5)                            # 90 deg about z
    assert np.allclose(world_accel([s, 0, 0, s], [1.0, 0.0, 0.0]), [0.0, 1.0],
                       atol=1e-9)


def test_world_accel_rejects_unusable_input():
    """An unnormalised quaternion must not yield a plausible wrong answer."""
    assert world_accel([0, 0, 0, 0], [1, 2, 3]) is None
    assert world_accel([1, 0, 0], [1, 2, 3]) is None
    assert world_accel([1, 0, 0, 0], [np.nan, 2, 3]) is None
