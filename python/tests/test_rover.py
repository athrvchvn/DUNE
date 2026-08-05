"""The rover link and control loop, against a fake Teensy.

The fake speaks the real protocol from
`firmware/teensy_control_rebuild.ino`: newline JSON, `status:"ok"|"error"`,
unsolicited `{"event":...}` lines at arbitrary times, `emergencyStop` answering
with an event rather than a status, and the ESP32 relay interleaving replies
that belong to nobody.

What is worth testing here is not the byte format — it is every path by which
the rover stops. The firmware has no watchdog, so if the host fails to stop it,
nothing else will.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover.controller import (ARM_LEASE_S, INTENT_TIMEOUT_S,  # noqa: E402
                              OMEGA_MAX, V_MAX, RoverController)
from rover.teensy import TeensyError, TeensyLink                # noqa: E402


class FakeSerial:
    """A Teensy that answers, and also talks when nobody asked."""

    def __init__(self, *, noise: bool = False, fail: set | None = None,
                 silent: set | None = None):
        self.written: list[dict] = []
        self._out: list[str] = []
        self.noise = noise            # interleave unsolicited ESP32/event lines
        self.fail = fail or set()     # commands that answer status:"error"
        self.silent = silent or set() # commands that answer nothing at all
        self.motor_speeds = [0] * 6
        self.dirs = [0] * 6
        self.angles = {}
        self.stopped = 0
        self.emergency = 0
        self._lock = threading.Lock()

    # ── the serial API TeensyLink uses ───────────────────────────────────────

    def write(self, data: bytes) -> None:
        obj = json.loads(data.decode().strip())
        cmd = obj.get('cmd')
        with self._lock:
            self.written.append(obj)
            if self.noise:
                # An ESP32 relay landing between the request and its reply.
                self._out.append(json.dumps({'event': 'arm_reply',
                                             'message': 'unrelated'}))
            if cmd in self.silent:
                return
            if cmd in self.fail:
                self._out.append(json.dumps({'cmd': cmd, 'status': 'error',
                                             'error': 'simulated failure'}))
                return
            self._reply(cmd, obj)

    def _reply(self, cmd, obj):
        if cmd == 'setAllMotorSpeedsWithDir':
            self.motor_speeds = list(obj['speeds'])
            self.dirs = list(obj['dirs'])
            self._out.append(json.dumps({'cmd': cmd, 'status': 'ok',
                                         'data': {'enabled_first': True}}))
        elif cmd == 'setWheelAngle':
            if abs(obj['angle']) > 60:
                self._out.append(json.dumps({'cmd': cmd, 'status': 'error',
                                             'error': 'angle out of range'}))
                return
            self.angles[obj['wheel']] = obj['angle']
            self._out.append(json.dumps({'cmd': cmd, 'status': 'ok'}))
        elif cmd in ('stopAllMotors', 'stopRover'):
            self.motor_speeds = [0] * 6
            self.stopped += 1
            self._out.append(json.dumps({'cmd': cmd, 'status': 'ok'}))
        elif cmd == 'emergencyStop':
            self.motor_speeds = [0] * 6
            self.emergency += 1
            # An EVENT, not a status — the firmware really does this.
            self._out.append(json.dumps({'event': 'emergency_stop',
                                         'message': 'halted'}))
        elif cmd == 'getTeensyIMU':
            self._out.append(json.dumps({'cmd': cmd, 'status': 'ok', 'data': {
                'available': True, 'pitch': 1.5, 'roll': -0.5, 'yaw': 30.0}}))
        elif cmd == 'getAllMotorSpeedsFast':
            self._out.append(json.dumps({'cmd': cmd, 'status': 'ok', 'data': {
                'commanded_speeds': self.motor_speeds,
                'actual_speeds': self.motor_speeds}}))
        elif cmd == 'getVersion':
            self._out.append(json.dumps({'cmd': cmd, 'status': 'ok',
                                         'data': 'v0.5-fake'}))
        else:
            self._out.append(json.dumps({'cmd': cmd, 'status': 'error',
                                         'error': 'Unknown command'}))

    def readline(self) -> bytes:
        with self._lock:
            if self._out:
                return (self._out.pop(0) + '\n').encode()
        return b''

    def close(self) -> None:
        pass


def make_link(**kw) -> TeensyLink:
    fake = FakeSerial(**kw)
    link = TeensyLink(port='fake')
    link.open(boot_wait=0.0, serial_factory=lambda *_a: fake)
    link.fake = fake
    return link


# ── the link ──────────────────────────────────────────────────────────────────

def test_open_reads_the_version():
    link = make_link()
    assert link.version == 'v0.5-fake'


def test_a_reply_is_matched_to_its_own_command():
    """MATLAB accepts the first JSON line it sees; a relayed reply satisfies it.

    The fake interleaves an unrelated event with every write, so a reader that
    does not match on `cmd` will return the wrong object.
    """
    link = make_link(noise=True)
    r = link.request('getTeensyIMU')
    assert r['cmd'] == 'getTeensyIMU'
    assert r['data']['pitch'] == 1.5
    assert link.stats.events > 0, "the fake never produced the interleaved event"


def test_events_are_collected_not_mistaken_for_replies():
    link = make_link(noise=True)
    link.request('getVersion')
    assert any(e.get('event') == 'arm_reply' for e in link.events)


def test_emergency_stop_is_satisfied_by_an_event():
    """It answers with an event, so a status wait would time out — on the one
    command that must never time out."""
    link = make_link()
    r = link.emergency_stop()
    assert r['status'] == 'ok' and r['via'] == 'event'
    assert link.fake.emergency == 1


def test_an_error_status_raises_rather_than_being_ignored():
    link = make_link(fail={'setWheelAngle'})
    with pytest.raises(TeensyError, match='simulated failure'):
        link.set_wheel_angle(1, 10.0)


def test_a_silent_command_times_out_rather_than_hanging():
    link = make_link(silent={'getTeensyIMU'})
    t0 = time.monotonic()
    with pytest.raises(TeensyError, match='no reply'):
        link.request('getTeensyIMU', timeout=0.2)
    assert time.monotonic() - t0 < 1.0
    assert link.stats.timeouts == 1


def test_the_firmwares_argument_limits_are_enforced_before_sending():
    """Six speeds, six dirs, 0..3000. The firmware rejects anything else, and a
    rejected motor command means the rover keeps its PREVIOUS speed."""
    link = make_link()
    with pytest.raises(ValueError):
        link.set_all_motor_speeds_with_dir([100] * 5, [0] * 6)
    with pytest.raises(ValueError):
        link.set_all_motor_speeds_with_dir([5000] * 6, [0] * 6)
    with pytest.raises(ValueError):
        link.set_all_motor_speeds_with_dir([-1] * 6, [0] * 6)


def test_full_stop_never_raises():
    """It runs from `finally` blocks and signal handlers, where an exception
    would replace a stop attempt with a traceback."""
    link = make_link(fail={'stopAllMotors', 'stopRover'})
    link.full_stop()          # must not raise
    link._ser = None
    link.full_stop()          # not even with the port gone


# ── the control loop: every way the rover stops ──────────────────────────────

def _running_controller(**kw):
    link = make_link(**kw)
    c = RoverController(link, control_hz=50.0)
    c.start()
    return c, link


def test_an_unarmed_controller_never_commands_motion():
    c, link = _running_controller()
    try:
        c.set_intent(V_MAX, 0.0)          # intent without arming
        time.sleep(0.2)
        assert link.fake.motor_speeds == [0] * 6
    finally:
        c.stop()


def test_arming_plus_intent_moves_and_dropping_intent_stops():
    """The dead-man, which is the whole safety design in one assertion."""
    c, link = _running_controller()
    try:
        c.arm()
        c.set_intent(0.05, 0.0)
        time.sleep(0.2)
        assert any(s > 0 for s in link.fake.motor_speeds), "never started moving"

        time.sleep(INTENT_TIMEOUT_S + 0.2)   # stop refreshing intent
        assert link.fake.motor_speeds == [0] * 6, "dead-man did not fire"
        assert c.state.n_deadman_stops > 0
    finally:
        c.stop()


def test_disarming_stops_even_with_fresh_intent():
    c, link = _running_controller()
    try:
        c.arm()
        for _ in range(6):
            c.set_intent(0.05, 0.0)
            time.sleep(0.03)
        assert any(s > 0 for s in link.fake.motor_speeds)

        c.disarm()
        for _ in range(6):
            c.set_intent(0.05, 0.0)      # keep asking; it must be refused
            time.sleep(0.03)
        assert link.fake.motor_speeds == [0] * 6
    finally:
        c.stop()


def test_the_arm_lease_expires_on_its_own():
    """Arming is not a mode you can leave on and walk away from."""
    c, link = _running_controller()
    try:
        c.arm(lease_s=0.15)
        c.set_intent(0.05, 0.0)
        time.sleep(0.05)
        assert any(s > 0 for s in link.fake.motor_speeds)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            c.set_intent(0.05, 0.0)      # intent stays fresh; the LEASE expires
            time.sleep(0.02)
            if not c.armed:
                break
        time.sleep(0.1)
        assert link.fake.motor_speeds == [0] * 6, "expired lease still driving"
    finally:
        c.stop()


def test_losing_every_client_stops_the_rover():
    """A browser refresh stops it. That is correct, not a bug to work around."""
    c, link = _running_controller()
    try:
        c.arm()
        c.set_intent(0.05, 0.0)
        time.sleep(0.1)
        c.on_client_gone(0)
        time.sleep(0.1)
        assert link.fake.motor_speeds == [0] * 6
        assert not c.armed, "must not resume when a client reconnects"
        assert link.fake.emergency >= 1
    finally:
        c.stop()


def test_stopping_the_loop_stops_the_motors():
    c, link = _running_controller()
    c.arm()
    c.set_intent(0.05, 0.0)
    time.sleep(0.1)
    c.stop()
    assert link.fake.motor_speeds == [0] * 6
    assert link.fake.stopped > 0


def test_the_loop_stops_the_motors_even_if_it_dies_unexpectedly():
    """The `finally` in the run loop is the last line of host-side defence."""
    c, link = _running_controller()
    c.arm()
    c.set_intent(0.05, 0.0)
    time.sleep(0.1)
    c._stop_flag.set()                 # simulate the loop ending abruptly
    time.sleep(0.2)
    assert link.fake.motor_speeds == [0] * 6
    c.stop()


def test_a_link_failure_while_moving_stops_rather_than_continuing():
    c, link = _running_controller()
    try:
        c.arm()
        c.set_intent(0.05, 0.0)
        time.sleep(0.1)
        link.fake.fail.add('setAllMotorSpeedsWithDir')
        time.sleep(0.2)
        assert c.state.fault is not None, "a link error was swallowed"
        assert link.fake.stopped > 0, "did not stop on a link error"
    finally:
        c.stop()


# ── limits and rate ───────────────────────────────────────────────────────────

def test_intent_is_clamped_server_side():
    """A browser-supplied number is not a limit."""
    c, _ = _running_controller()
    try:
        c.set_intent(99.0, 99.0)
        assert c._intent[0] == pytest.approx(V_MAX)
        assert c._intent[1] == pytest.approx(OMEGA_MAX)
        c.set_intent(-99.0, -99.0)
        assert c._intent[0] == pytest.approx(-V_MAX)
    finally:
        c.stop()


def test_a_nudge_expires_by_itself():
    """This is what makes browser-initiated motion safe under latency: the
    command carries its own end, so a dead connection still stops on time."""
    c, link = _running_controller()
    try:
        c.arm()
        c.nudge(0.05, 0.0, duration_s=0.1)
        time.sleep(0.05)
        assert any(s > 0 for s in link.fake.motor_speeds)
        time.sleep(0.25)
        assert link.fake.motor_speeds == [0] * 6, "nudge outlived its duration"
    finally:
        c.stop()


def test_a_nudge_cannot_be_stretched_past_one_second():
    c, _ = _running_controller()
    try:
        c.arm()
        c.nudge(0.05, 0.0, duration_s=30.0)
        age_at_start = time.monotonic() - c._intent_ts
        assert age_at_start < INTENT_TIMEOUT_S
    finally:
        c.stop()


def test_steering_is_only_resent_when_a_wheel_actually_moves():
    """Five round-trips per tick caps the loop near 10 Hz; resending unchanged
    angles spends that budget for nothing."""
    c, link = _running_controller()
    try:
        c.arm()
        for _ in range(15):
            c.set_intent(0.05, 0.0)      # constant command
            time.sleep(0.02)
        n_angle = sum(1 for w in link.fake.written
                      if w.get('cmd') == 'setWheelAngle')
        n_motor = sum(1 for w in link.fake.written
                      if w.get('cmd') == 'setAllMotorSpeedsWithDir')
        assert n_motor > n_angle, (
            f"{n_angle} steering vs {n_motor} motor commands — steering is "
            "being re-sent unnecessarily")
    finally:
        c.stop()


def test_the_motor_command_is_sent_every_tick():
    """Its absence is what 'stopped' means, so it can never be skipped."""
    c, link = _running_controller()
    try:
        c.arm()
        c.set_intent(0.05, 0.0)
        time.sleep(0.2)
        n = sum(1 for w in link.fake.written
                if w.get('cmd') == 'setAllMotorSpeedsWithDir')
        assert n >= 5, f"only {n} motor commands in 0.2 s at 50 Hz"
    finally:
        c.stop()


# ── the WebSocket surface ─────────────────────────────────────────────────────

import asyncio       # noqa: E402
import functools     # noqa: E402

from host.app import App                       # noqa: E402
from host.jobs import JOB_KINDS                 # noqa: E402
import host.rover_api as rapi                   # noqa: E402,F401


def runs_async(fn):
    @functools.wraps(fn)
    def wrapper(*a, **kw):
        return asyncio.run(fn(*a, **kw))
    return wrapper


def _app_with_rover(tmp_path, **kw):
    app = App(log_path=tmp_path / 's.ndjson')
    app.set_config_msg(lambda: {'type': 'config'})
    app.register_handlers(rapi.HANDLERS)
    c, link = _running_controller(**kw)
    app.state.rover = c
    return app, c, link


def _msgs(app, kind=None):
    app.recorder._fh.flush()
    rows = [json.loads(l) for l in open(app.recorder.path) if l.strip()]
    return [r for r in rows if kind is None or r['type'] == kind]


@runs_async
async def test_arming_requires_a_token_from_a_recent_status(tmp_path):
    """A queued or replayed click must not arm a rover nobody is looking at."""
    app, c, link = _app_with_rover(tmp_path)
    try:
        await rapi.h_rover_arm(app, {}, None)                   # no token
        assert not c.armed
        await rapi.h_rover_arm(app, {'token': 'made-up'}, None)  # wrong token
        assert not c.armed
        errs = [m['error'] for m in _msgs(app, 'rover_error')]
        assert len(errs) == 2 and all('token' in e for e in errs)

        await rapi.h_rover_status(app, {}, None)
        token = _msgs(app, 'rover_status')[-1]['token']
        await rapi.h_rover_arm(app, {'token': token}, None)
        assert c.armed
    finally:
        c.stop()
        await app.aclose()


@runs_async
async def test_each_status_issues_a_fresh_token(tmp_path):
    """So a token cannot be hoarded and replayed later."""
    app, c, link = _app_with_rover(tmp_path)
    try:
        await rapi.h_rover_status(app, {}, None)
        first = _msgs(app, 'rover_status')[-1]['token']
        await rapi.h_rover_status(app, {}, None)
        second = _msgs(app, 'rover_status')[-1]['token']
        assert first != second

        await rapi.h_rover_arm(app, {'token': first}, None)      # stale
        assert not c.armed
    finally:
        c.stop()
        await app.aclose()


@runs_async
async def test_a_nudge_is_refused_unless_armed(tmp_path):
    app, c, link = _app_with_rover(tmp_path)
    try:
        await rapi.h_rover_nudge(app, {'v': 0.05, 'omega': 0.0}, None)
        assert link.fake.motor_speeds == [0] * 6
        assert any('not armed' in m['error'] for m in _msgs(app, 'rover_error'))
    finally:
        c.stop()
        await app.aclose()


@runs_async
async def test_stop_disarms_so_it_cannot_resume_by_itself(tmp_path):
    app, c, link = _app_with_rover(tmp_path)
    try:
        c.arm()
        c.set_intent(0.05, 0.0)
        time.sleep(0.1)
        await rapi.h_rover_stop(app, {'reason': 'test'}, None)
        assert not c.armed
        assert link.fake.motor_speeds == [0] * 6
    finally:
        c.stop()
        await app.aclose()


@runs_async
async def test_losing_the_last_client_stops_the_rover_through_the_app(tmp_path):
    """End to end: the App hook, not just the controller method."""
    app, c, link = _app_with_rover(tmp_path)
    try:
        app.set_on_client_gone(rapi._make_client_gone(app))
        c.arm()
        c.set_intent(0.05, 0.0)
        time.sleep(0.1)
        await app._on_client_gone(0)
        time.sleep(0.1)
        assert link.fake.motor_speeds == [0] * 6 and not c.armed
    finally:
        c.stop()
        await app.aclose()


# ── the bench job ─────────────────────────────────────────────────────────────

def test_wheels_test_is_a_dry_run_unless_confirmed():
    """Its entire purpose is to make the rover move, so it asks twice."""
    spec = JOB_KINDS['wheels_test']
    assert spec.args_schema['confirm']['default'] is False
    assert spec.mode == 'run' and 'rover' in spec.requires
    assert 'OFF THE GROUND' in spec.title


@runs_async
async def test_wheels_test_dry_run_sends_nothing(tmp_path):
    app, c, link = _app_with_rover(tmp_path)
    try:
        before = len(link.fake.written)
        job_id = await app.jobs.run('wheels_test', {'rpm': 150, 'seconds': 0.2})
        for _ in range(100):
            await asyncio.sleep(0.05)
            done = [m for m in _msgs(app) if m['type'] in
                    ('job_done', 'job_failed')]
            if done:
                break
        assert done[-1]['type'] == 'job_done', done[-1].get('error')
        assert done[-1]['result']['applied'] is False
        spins = [w for w in link.fake.written[before:]
                 if w.get('cmd') == 'setAllMotorSpeedsWithDir'
                 and any(w.get('speeds', []))]
        assert not spins, "a dry run commanded a non-zero speed"
    finally:
        c.stop()
        await app.aclose()


@runs_async
async def test_wheels_test_always_stops_and_leaves_it_disarmed(tmp_path):
    app, c, link = _app_with_rover(tmp_path)
    try:
        c.arm()
        await app.jobs.run('wheels_test',
                           {'rpm': 150, 'seconds': 0.2, 'confirm': True})
        for _ in range(200):
            await asyncio.sleep(0.05)
            done = [m for m in _msgs(app) if m['type'] in
                    ('job_done', 'job_failed')]
            if done:
                break
        assert done[-1]['type'] == 'job_done', done[-1].get('error')
        assert done[-1]['result']['applied'] is True
        assert link.fake.motor_speeds == [0] * 6, "did not stop after the spin"
        assert not c.armed, "must leave the rover disarmed"
        assert link.fake.stopped > 0
    finally:
        c.stop()
        await app.aclose()


# ── regressions: two bugs the first version of this file did not catch ───────

def test_a_stop_is_not_delayed_by_an_in_flight_request():
    """`request()` holds the link lock for its whole read-wait.

    Taking that lock in `send()` too let an in-flight request delay a stop by its
    full timeout — measured at 890 ms, on the one path that must be immediate.
    """
    link = make_link(silent={'getTeensyIMU'})

    def slow():
        try:
            link.request('getTeensyIMU', timeout=1.0)
        except TeensyError:
            pass

    t = threading.Thread(target=slow)
    t.start()
    time.sleep(0.1)                     # let it get into the wait
    t0 = time.monotonic()
    link.full_stop()
    blocked = time.monotonic() - t0
    t.join()

    assert blocked < 0.05, f"a stop waited {blocked * 1000:.0f} ms on a request"
    assert link.fake.stopped > 0, "the stop never reached the firmware"


def test_a_paused_controller_lets_a_job_drive_the_link():
    """The control loop writes a motor command EVERY tick, including zero.

    So merely disarming is not enough to hand the link to a job: the loop
    overwrites the job's command within ~100 ms and the job silently does
    nothing. The first wheels_test did exactly that, and its test passed because
    it only checked "applied" and "ends stopped" — both trivially true when the
    spin never happened.
    """
    link = make_link()
    c = RoverController(link, control_hz=20.0)
    c.start()
    try:
        time.sleep(0.15)
        c.pause()
        link.set_all_motor_speeds_with_dir([150] * 6, [0, 0, 0, 1, 1, 1])

        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.3:
            samples.append(any(link.fake.motor_speeds))
            time.sleep(0.02)
        assert all(samples), (
            f"the loop overwrote the job's command: only {sum(samples)}/"
            f"{len(samples)} samples were spinning")

        c.resume()
        time.sleep(0.15)
        assert link.fake.motor_speeds == [0] * 6, (
            "resuming did not put the loop back in charge")
    finally:
        c.stop()


def test_pausing_disarms_so_nothing_resumes_behind_the_job():
    link = make_link()
    c = RoverController(link, control_hz=20.0)
    c.start()
    try:
        c.arm()
        assert c.armed
        c.pause()
        assert not c.armed, "a paused controller must not stay armed"
        c.set_intent(0.05, 0.0)
        time.sleep(0.1)
        assert not c.armed
    finally:
        c.stop()


@runs_async
async def test_wheels_test_actually_commands_a_spin(tmp_path):
    """The assertion the original test was missing entirely."""
    app, c, link = _app_with_rover(tmp_path)
    try:
        await app.jobs.run('wheels_test',
                           {'rpm': 200, 'seconds': 0.3, 'confirm': True})
        for _ in range(200):
            await asyncio.sleep(0.05)
            done = [m for m in _msgs(app) if m['type'] in ('job_done', 'job_failed')]
            if done:
                break
        assert done[-1]['type'] == 'job_done', done[-1].get('error')

        spins = [w for w in link.fake.written
                 if w.get('cmd') == 'setAllMotorSpeedsWithDir'
                 and w.get('speeds') == [200] * 6]
        assert spins, "wheels_test never commanded the requested speed"
        assert link.fake.motor_speeds == [0] * 6, "did not stop afterwards"
        assert not c.paused, "the controller was left paused"
    finally:
        c.stop()
        await app.aclose()
