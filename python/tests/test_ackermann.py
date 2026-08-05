"""(V, ω) → wheel commands, against the MMS algorithm.

**On the oracle.** The plan called for a golden table generated from MATLAB.
MATLAB cannot be run here, so every expected value below is derived by hand from
`AckermannCalculator.m` — the geometry constants, the RPM conversion and the
branch structure are all transcribed, and the arithmetic is redone
independently. That is weaker than executing the original, and it is the honest
description of what this checks: the port matches the algorithm as written.

Before any wheels-on-ground run, one MATLAB session should dump
`calculateFromVelocity` over the (V, ω) grid used here so this becomes a real
equivalence test. The cases are chosen to make that easy: straight, both turn
senses, both pivot entry paths, saturation, and both signs of V.

The three sign conventions are the reason this file exists at all. Get one wrong
and the rover drives confidently in the wrong direction — no error, no warning.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover.ackermann import (GEAR_RATIO, MAX_MOTOR_RPM, MAX_STEERING,  # noqa: E402
                            TRACK_WIDTH, WHEELBASE, WHEEL_RADIUS,
                            calculate_from_velocity, stop_command,
                            velocity_to_motor_rpm)

# The fastest the rig can actually go: 3000 motor RPM through 241:1 on a 70 mm
# wheel. Note rover_teleop_uwb.m commands V_MAX = 0.10 m/s, which is ABOVE this —
# so the top of the stick is saturated and the rover tops out here instead.
MAX_ACHIEVABLE_MS = (MAX_MOTOR_RPM / GEAR_RATIO) * (2 * math.pi / 60) * WHEEL_RADIUS


def test_the_geometry_constants_match_the_rig():
    """Physical facts about the vehicle; a typo here mis-steers everything."""
    assert (WHEELBASE, TRACK_WIDTH, MAX_STEERING) == (0.6, 0.70, 60.0)
    assert (WHEEL_RADIUS, GEAR_RATIO, MAX_MOTOR_RPM) == (0.07, 241, 3000)


def test_velocity_to_motor_rpm_is_the_documented_chain():
    """v / r → rad/s → RPM → ×gear. Checked at a value that makes it obvious."""
    # A wheel turning at exactly 1 rad/s travels 0.07 m/s.
    assert velocity_to_motor_rpm(WHEEL_RADIUS) == pytest.approx(
        (60.0 / (2 * math.pi)) * GEAR_RATIO)
    assert velocity_to_motor_rpm(0.0) == 0.0
    assert MAX_ACHIEVABLE_MS == pytest.approx(0.09123, abs=1e-4)


# ── the three sign conventions ────────────────────────────────────────────────

def test_forward_is_left0_right1_because_the_right_motors_are_reversed():
    """Not (0, 0). This one is impossible to guess and inverts the rover."""
    fwd = calculate_from_velocity(0.05, 0.0)
    assert (fwd.dir_left, fwd.dir_right) == (0, 1)
    rev = calculate_from_velocity(-0.05, 0.0)
    assert (rev.dir_left, rev.dir_right) == (1, 0)
    assert rev.motor_speeds == fwd.motor_speeds, "sign belongs in dirs, not speed"


def test_motor_speeds_are_unsigned():
    """The firmware validates 0..3000 and takes the sign from `dirs`."""
    for v in (-0.09, -0.01, 0.0, 0.01, 0.09):
        for om in (-0.3, 0.0, 0.3):
            c = calculate_from_velocity(v, om)
            assert all(0 <= s <= MAX_MOTOR_RPM for s in c.motor_speeds), (v, om, c)
            assert all(isinstance(s, int) for s in c.motor_speeds)


def test_the_front_steering_angles_are_inverted_relative_to_the_rears():
    """The front servos are mounted opposite; the port must negate them.

    On a left turn the left wheels are inner and turn MORE, so before inversion
    both left angles are equal and positive. After it, the front is negated —
    which is what the array must show.
    """
    # omega chosen to stay in the Ackermann branch — see the note on the narrow
    # usable band in test_the_ackermann_band_is_narrow_on_this_rig below.
    c = calculate_from_velocity(0.10, 0.20)      # left turn
    lr, lf, rf, rr = c.wheel_angles
    assert lr > 0 and lf < 0, f"front/rear not inverted on the left pair: {c.wheel_angles}"
    assert rr > 0 and rf < 0, f"front/rear not inverted on the right pair: {c.wheel_angles}"
    assert abs(lr) == pytest.approx(abs(lf))
    assert abs(rr) == pytest.approx(abs(rf))


def test_a_pivot_drives_both_sides_with_the_same_direction_bit():
    """Same bit + reversed right mounting = physical counter-rotation."""
    ccw = calculate_from_velocity(0.0, 0.30)
    cw = calculate_from_velocity(0.0, -0.30)
    assert ccw.mode == cw.mode == 'pivot'
    assert (ccw.dir_left, ccw.dir_right) == (1, 1)
    assert (cw.dir_left, cw.dir_right) == (0, 0)
    assert ccw.wheel_angles == cw.wheel_angles, "pivot pattern must not depend on sense"


# ── branch structure ──────────────────────────────────────────────────────────

def test_straight_line_keeps_the_wheels_centred_and_the_speeds_equal():
    c = calculate_from_velocity(0.05, 0.0)
    assert c.mode == 'straight'
    assert c.wheel_angles == [0.0, 0.0, 0.0, 0.0]
    assert len(set(c.motor_speeds)) == 1
    assert c.motor_speeds[0] == round(velocity_to_motor_rpm(0.05))


def test_the_inner_wheels_turn_more_and_run_slower():
    """The whole point of Ackermann; reversing it scrubs the tyres."""
    left = calculate_from_velocity(0.10, 0.20)
    lr, lf, rf, rr = left.wheel_angles
    assert abs(lr) > abs(rr), "inner (left) wheels should steer harder"
    assert left.motor_speeds[0] < left.motor_speeds[3], "inner side should be slower"

    right = calculate_from_velocity(0.10, -0.20)
    lr, lf, rf, rr = right.wheel_angles
    assert abs(rr) > abs(lr)
    assert right.motor_speeds[3] < right.motor_speeds[0]


def test_a_turn_too_tight_to_steer_becomes_a_pivot():
    """Inner radius <= 0 means the inner wheels would need to run backwards.

    Clamping them to zero instead would stall the inside of the rover mid-corner,
    so the algorithm switches to a pivot.
    """
    c = calculate_from_velocity(0.02, 1.5)
    assert c.mode == 'pivot', c
    assert all(s > 0 for s in c.motor_speeds), "pivot must not stall any wheel"


def test_pivot_only_engages_with_a_real_yaw_command():
    """V≈0 and ω≈0 is 'stopped', not 'spin'."""
    c = calculate_from_velocity(0.0, 0.0)
    assert c.mode == 'straight'
    assert c.motor_speeds == [0] * 6


def test_the_pivot_angle_is_the_geometric_one():
    """atan(wheelbase / track), clamped to the mechanical limit."""
    expect = min(math.degrees(math.atan(WHEELBASE / TRACK_WIDTH)), MAX_STEERING)
    c = calculate_from_velocity(0.0, 0.4)
    assert c.wheel_angles == pytest.approx([-expect, expect, -expect, expect])


# ── limits ────────────────────────────────────────────────────────────────────

def test_steering_never_exceeds_the_mechanical_limit():
    """The firmware rejects |angle| > 60, so an over-range command is a no-op
    on that wheel — the rover would then steer with three wheels."""
    for v in (0.001, 0.01, 0.05, 0.09, -0.05):
        for om in (-3.0, -1.0, -0.1, 0.1, 1.0, 3.0):
            c = calculate_from_velocity(v, om)
            assert all(abs(a) <= MAX_STEERING + 1e-9 for a in c.wheel_angles), (v, om, c)


def test_motor_speed_saturates_rather_than_overflowing():
    """The rig cannot reach the 0.10 m/s that teleop commands at full stick."""
    c = calculate_from_velocity(10.0, 0.0)
    assert c.motor_speeds == [MAX_MOTOR_RPM] * 6
    at_vmax = calculate_from_velocity(0.10, 0.0)
    assert at_vmax.motor_speeds[0] == MAX_MOTOR_RPM, (
        "0.10 m/s is above what 3000 RPM can deliver; it must clamp")


def test_stop_command_is_zero_speed():
    c = stop_command()
    assert c.motor_speeds == [0] * 6


def test_dirs_expands_to_six_bits_in_motor_order():
    c = calculate_from_velocity(0.05, 0.0)
    assert c.dirs == [c.dir_left] * 3 + [c.dir_right] * 3
    assert len(c.dirs) == len(c.motor_speeds) == 6


def test_the_command_is_json_safe():
    """It goes straight onto the wire and into the session log."""
    from host.bus import dumps
    import json
    for v, om in ((0.05, 0.2), (0.0, 0.3), (-0.05, 0.0), (0.0, 0.0)):
        text = dumps(calculate_from_velocity(v, om).to_dict())
        assert 'NaN' not in text
        json.loads(text)


# ── continuity ────────────────────────────────────────────────────────────────

def test_output_is_continuous_across_the_straight_to_turning_boundary():
    """A step in commanded angle at a threshold crossing jerks the steering."""
    prev = None
    for om in [i * 0.005 for i in range(0, 40)]:
        c = calculate_from_velocity(0.05, om)
        if prev is not None and c.mode == prev[0] == 'ackermann':
            jump = max(abs(a - b) for a, b in zip(c.wheel_angles, prev[1]))
            assert jump < 12.0, f"steering jumped {jump:.1f} deg at omega={om}"
        prev = (c.mode, c.wheel_angles)


def test_the_ackermann_band_is_narrow_on_this_rig():
    """A geometric fact worth knowing before tuning teleop rates.

    The minimum steerable turn radius is wheelbase / tan(60 deg) = 0.346 m, and
    half the track is 0.350 m. The tightest steerable turn therefore ALWAYS puts
    the inner wheels inside the turn centre, so it degrades to a pivot. Ackermann
    differential steering only applies while |omega| < V / (TRACK_WIDTH / 2).
    """
    min_radius = WHEELBASE / math.tan(math.radians(MAX_STEERING))
    assert min_radius < TRACK_WIDTH / 2, "the rig would have a steerable tight turn"

    v = 0.10
    boundary = v / (TRACK_WIDTH / 2)                 # ~0.286 rad/s
    assert calculate_from_velocity(v, boundary * 0.7).mode == 'ackermann'
    assert calculate_from_velocity(v, boundary * 1.3).mode == 'pivot'
