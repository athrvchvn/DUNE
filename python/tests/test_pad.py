"""The gamepad thread, against a fake pad.

The interesting cases are the failures, not the happy path: a pad that
disconnects mid-drive, a thread that dies, a stick left centred. Each must end
with the rover stopped, and none of them may depend on the control loop being
healthy — that is the whole reason the stick is server-side rather than in the
browser.
"""

from __future__ import annotations

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover.pad import (DEADZONE, PAD_V_MAX,  # noqa: E402
                       PadMapping, PadThread, _deadzone, _ramp)


class FakePad:
    """A pad whose axes and buttons the test drives directly."""

    name = 'fake-ds4'

    def __init__(self):
        self.v = 0.0
        self.omega = 0.0
        self.buttons = {'btn_stop': False, 'btn_center': False,
                        'btn_disarm': False, 'btn_pivot': False}
        self.fail = False
        self.closed = False
        self._lock = threading.Lock()

    def read(self):
        if self.fail:
            raise OSError('pad disconnected')
        with self._lock:
            return self.v, self.omega, dict(self.buttons)

    def close(self):
        self.closed = True


class FakeController:
    """Records what the pad asked for, without needing a link."""

    def __init__(self):
        self.intents = []
        self.stops = []
        self.disarms = 0
        self.centered = 0
        self.link = self

    def set_intent(self, v, omega):
        self.intents.append((v, omega))

    def emergency_stop(self, reason='stop'):
        self.stops.append(reason)

    def disarm(self):
        self.disarms += 1

    def center_wheels(self):
        self.centered += 1


def _running(pad_hz: float = 200.0):
    fake, ctrl = FakePad(), FakeController()
    pad = PadThread(ctrl, fake, poll_hz=pad_hz)
    pad.start()
    return pad, fake, ctrl


# ── shaping ───────────────────────────────────────────────────────────────────

def test_the_deadzone_rescales_so_the_stick_does_not_jump_at_its_edge():
    assert _deadzone(0.0) == 0.0
    assert _deadzone(DEADZONE * 0.9) == 0.0
    # Just outside the deadzone must be near zero, not a step to 10%.
    assert 0.0 < _deadzone(DEADZONE + 0.01) < 0.02
    assert _deadzone(1.0) == pytest.approx(1.0)
    assert _deadzone(-1.0) == pytest.approx(-1.0)


def test_the_ramp_limits_how_fast_a_command_can_change():
    """A step from a flicked stick would jerk the rig."""
    assert _ramp(0.0, 1.0, 0.1) == pytest.approx(0.1)
    assert _ramp(0.0, -1.0, 0.1) == pytest.approx(-0.1)
    assert _ramp(0.5, 0.52, 0.1) == pytest.approx(0.52)   # never overshoots


def test_a_full_stick_reaches_the_documented_ceiling():
    pad, fake, ctrl = _running()
    try:
        fake.v = 1.0
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and pad.state.v < PAD_V_MAX * 0.99:
            time.sleep(0.02)
        assert pad.state.v == pytest.approx(PAD_V_MAX, rel=0.02)
    finally:
        pad.stop()


def test_holding_pivot_zeroes_forward_speed():
    """R2 held is a spin in place, as MATLAB does it."""
    pad, fake, ctrl = _running()
    try:
        fake.v, fake.omega = 1.0, 1.0
        time.sleep(0.3)
        assert pad.state.v > 0
        fake.buttons['btn_pivot'] = True
        time.sleep(0.5)
        assert abs(pad.state.v) < 0.005, f"still driving forward: {pad.state.v}"
        assert pad.state.omega > 0, "pivot should keep the yaw command"
    finally:
        pad.stop()


# ── the safety-relevant paths ────────────────────────────────────────────────

def test_a_centred_stick_expresses_no_intent_at_all():
    """Not 'intent of zero' — NO intent, so the dead-man expires.

    Feeding a continuous zero would keep the rover armed and alive forever with
    nobody touching it.
    """
    pad, fake, ctrl = _running()
    try:
        time.sleep(0.3)
        assert not ctrl.intents, f"a centred stick sent {len(ctrl.intents)} intents"
    finally:
        pad.stop()


def test_the_stop_button_bypasses_the_intent_path():
    """A physical stop must not depend on the control loop being healthy."""
    pad, fake, ctrl = _running()
    try:
        fake.v = 1.0
        time.sleep(0.2)
        fake.buttons['btn_stop'] = True
        time.sleep(0.2)
        assert ctrl.stops, "the stop button did not reach the controller"
        assert 'pad' in ctrl.stops[0]
        assert pad.state.v == 0.0, "the pad kept its own command after a stop"
    finally:
        pad.stop()


def test_buttons_fire_on_the_press_edge_not_while_held():
    pad, fake, ctrl = _running()
    try:
        fake.buttons['btn_disarm'] = True
        time.sleep(0.3)                    # held for many polls
        assert ctrl.disarms == 1, f"fired {ctrl.disarms} times while held"
        fake.buttons['btn_disarm'] = False
        time.sleep(0.1)
        fake.buttons['btn_disarm'] = True
        time.sleep(0.1)
        assert ctrl.disarms == 2, "a second press did not register"
    finally:
        pad.stop()


def test_a_disconnected_pad_stops_expressing_intent():
    """The dead-man then stops the rover. The thread must NOT die silently and
    leave the last intent standing."""
    pad, fake, ctrl = _running()
    try:
        fake.v = 1.0
        time.sleep(0.2)
        assert ctrl.intents
        n = len(ctrl.intents)

        fake.fail = True
        time.sleep(0.3)
        assert len(ctrl.intents) == n, "kept sending intent after disconnection"
        assert pad.state.present is False, "a dead pad still reports present"
        assert pad._thread is not None and pad._thread.is_alive(), (
            "the pad thread died instead of reporting absence")
    finally:
        pad.stop()


def test_stopping_the_pad_clears_the_intent_immediately():
    """The dead-man would catch it within 300 ms; this makes it immediate."""
    pad, fake, ctrl = _running()
    fake.v = 1.0
    time.sleep(0.2)
    ctrl.intents.clear()
    pad.stop()
    assert ctrl.intents and ctrl.intents[-1] == (0.0, 0.0), (
        "stopping the pad left the last intent standing")
    assert fake.closed


def test_a_reconnecting_pad_resumes_without_a_restart():
    pad, fake, ctrl = _running()
    try:
        fake.fail = True
        time.sleep(0.2)
        assert pad.state.present is False
        fake.fail = False
        fake.v = 1.0
        time.sleep(0.3)
        assert pad.state.present is True
        assert pad.state.v > 0
    finally:
        pad.stop()


def test_the_mapping_is_data_not_hard_coded():
    """Pad axis codes differ between drivers and DS4 revisions; a wrong guess
    must be fixable without editing code at the bench."""
    m = PadMapping(axis_v='ABS_RY', btn_stop='BTN_WEST')
    assert m.axis_v == 'ABS_RY' and m.btn_stop == 'BTN_WEST'
    assert PadMapping().axis_v == 'ABS_Y'


def test_the_pad_state_is_json_safe():
    from host.bus import dumps
    import json
    pad, fake, ctrl = _running()
    try:
        fake.v, fake.omega = 0.5, -0.5
        time.sleep(0.2)
        text = dumps(pad.state.to_dict())
        assert 'NaN' not in text
        json.loads(text)
    finally:
        pad.stop()


def test_releasing_stop_with_the_stick_still_pushed_does_not_resume():
    """The interlock. Without it the next poll recomputes the target from the
    still-deflected stick and ramps straight back up, so the stop button stops
    the rover and immediately asks it to move again."""
    pad, fake, ctrl = _running()
    try:
        fake.v = 1.0
        time.sleep(0.2)
        assert pad.state.v > 0

        fake.buttons['btn_stop'] = True
        time.sleep(0.15)
        fake.buttons['btn_stop'] = False      # released, stick STILL pushed
        ctrl.intents.clear()
        time.sleep(0.3)

        assert pad.state.v == 0.0, f"resumed while the stick was held: {pad.state.v}"
        assert not ctrl.intents, "commanded motion after a stop without recentring"

        fake.v = 0.0                          # operator neutralises
        time.sleep(0.2)
        fake.v = 1.0                          # and pushes again
        time.sleep(0.3)
        assert pad.state.v > 0, "the latch never cleared after recentring"
    finally:
        pad.stop()
