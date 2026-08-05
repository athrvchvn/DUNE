"""The rover's WebSocket surface: connect, arm, nudge, stop.

Deliberately small. The GUI can arm, ask for a bounded nudge, and stop — it
cannot stream continuous velocity. Three independent reasons:

* the browser Gamepad API only polls while the tab is focused and visible, so a
  backgrounded tab would freeze the stick **while the rover is moving**, with no
  error anywhere;
* the WebSocket adds a variable 5-50 ms and can queue behind a broadcast burst;
* if the browser crashes, a server-side input can still stop the rover.

So continuous teleop belongs to a server-side pad thread (Phase 6), and the
browser gets nudges that expire in the server. `rover_arm` carries a token from
the most recent `rover_status`, which makes arming an explicit act against a
state the operator has actually seen rather than a stale button press.
"""

from __future__ import annotations

import secrets
import time

from .jobs import job

# Arming is refused unless the client echoes the token from a recent status, so
# a queued or replayed click cannot arm a rover the operator is not looking at.
_TOKEN_TTL_S = 30.0


def _rover(app):
    r = getattr(app.state, 'rover', None)
    if r is None:
        raise RuntimeError("no rover connected — run rover_connect first")
    return r


async def _status(app, **extra) -> None:
    r = getattr(app.state, 'rover', None)
    if r is None:
        await app.bus.emit('rover_status', connected=False, **extra)
        return
    app.state.rover_token = secrets.token_hex(8)
    app.state.rover_token_ts = time.monotonic()
    await app.bus.emit('rover_status', connected=True,
                       token=app.state.rover_token,
                       port=getattr(r.link, 'port', None),
                       version=getattr(r.link, 'version', None),
                       **dict(r.state.to_dict(), **extra))


# ── handlers ──────────────────────────────────────────────────────────────────

async def h_rover_status(app, data, ws) -> None:
    await _status(app)


async def h_rover_connect(app, data, ws) -> None:
    """Open the link and start the control loop. Starts DISARMED."""
    from rover.controller import RoverController
    from rover.teensy import TeensyLink

    if getattr(app.state, 'rover', None) is not None:
        await _status(app, note='already connected')
        return

    port = str(data.get('port') or '/dev/ttyACM0')
    link = TeensyLink(port=port)
    try:
        link.open()
    except Exception as e:
        await app.bus.emit('rover_error', error=f"{type(e).__name__}: {e}",
                           port=port)
        return

    rover = RoverController(link)
    rover.start()
    app.state.rover = rover
    # No clients means nobody is watching, so anything that can move stands down.
    app.set_on_client_gone(_make_client_gone(app))
    await _status(app, note='connected')


def _make_client_gone(app):
    async def on_gone(n_clients: int) -> None:
        r = getattr(app.state, 'rover', None)
        if r is not None:
            r.on_client_gone(n_clients)
    return on_gone


async def h_rover_disconnect(app, data, ws) -> None:
    r = getattr(app.state, 'rover', None)
    if r is not None:
        r.stop()
        r.link.close()
        app.state.rover = None
    await app.bus.emit('rover_status', connected=False, note='disconnected')


async def h_rover_arm(app, data, ws) -> None:
    """Arm for a bounded lease, but only against a token the client has seen."""
    r = _rover(app)
    token = data.get('token')
    age = time.monotonic() - getattr(app.state, 'rover_token_ts', 0.0)
    if not token or token != getattr(app.state, 'rover_token', None):
        await app.bus.emit('rover_error', error='arm refused: stale or missing token')
        return
    if age > _TOKEN_TTL_S:
        await app.bus.emit('rover_error', error='arm refused: token expired')
        return
    lease = r.arm()
    await _status(app, note=f'armed for {lease:.0f}s')


async def h_rover_disarm(app, data, ws) -> None:
    _rover(app).disarm()
    await _status(app, note='disarmed')


async def h_rover_stop(app, data, ws) -> None:
    """Labelled STOP, never EMERGENCY STOP — see rover/controller.py."""
    r = getattr(app.state, 'rover', None)
    if r is not None:
        r.emergency_stop(str(data.get('reason') or 'operator stop'))
    await _status(app, note='stopped')


async def h_rover_nudge(app, data, ws) -> None:
    """A bounded move that expires in the server, not the browser."""
    r = _rover(app)
    if not r.armed:
        await app.bus.emit('rover_error', error='nudge refused: not armed')
        return
    r.nudge(float(data.get('v') or 0.0), float(data.get('omega') or 0.0),
            float(data.get('duration_s') or 0.3))
    await _status(app, note='nudge')


# ── capture ───────────────────────────────────────────────────────────────────

async def h_capture_start(app, data, ws) -> None:
    """Begin recording. UWB sweeps and rover samples share one host clock."""
    from rover.capture import RunCapture
    from host.paths import RESULTS_DIR

    if getattr(app.state, 'capture', None) is not None:
        await app.bus.emit('capture_error', error='a capture is already running')
        return

    cfg = getattr(app.state, 'anchors', None)
    if cfg is None:
        await app.bus.emit('capture_error', error='no anchor config loaded')
        return

    cap = RunCapture(
        RESULTS_DIR / 'rover_runs',
        anchor_ids=cfg.ids,
        front_tag=int(data.get('frontTag') or 241),
        rear_tag=int(data.get('rearTag') or 240),
        baseline_m=float(data.get('baseline') or 0.527),
        apriltag_offset_body=data.get('apriltagOffset') or (0.0, 0.0),
        tag_z=float(data.get('tagZ') or 0.24),
        anchors_layout=str(getattr(cfg, 'layout', '') or ''))
    app.state.capture = cap
    await app.bus.emit('capture_status', running=True, **cap.summary())


async def h_capture_stop(app, data, ws) -> None:
    """Stop recording, write the .mat, and score it — without being asked.

    The report is chained here rather than left as a separate click because a
    run nobody scores is a run nobody learns from, and the moment the operator
    cares is right now, standing next to the rover.
    """
    cap = getattr(app.state, 'capture', None)
    if cap is None:
        await app.bus.emit('capture_error', error='no capture running')
        return
    app.state.capture = None
    summary = cap.close()
    await app.bus.emit('capture_status', running=False, **summary)

    if not summary.get('mat'):
        await app.bus.emit('capture_error',
                           error=f"no .mat written: {summary.get('mat_error')}")
        return
    if data.get('report') is False:
        return
    from host.paths import rel
    await app.jobs.run('rover_run_report', {'matFile': rel(summary['mat']),
                                            'tagZ': cap.tag_z},
                       req_id='capture-auto')


async def h_capture_status(app, data, ws) -> None:
    cap = getattr(app.state, 'capture', None)
    if cap is None:
        await app.bus.emit('capture_status', running=False)
    else:
        await app.bus.emit('capture_status', running=True, **cap.summary())


HANDLERS = {
    'capture_start':    h_capture_start,
    'capture_stop':     h_capture_stop,
    'capture_status':   h_capture_status,
    'rover_status':     h_rover_status,
    'rover_connect':    h_rover_connect,
    'rover_disconnect': h_rover_disconnect,
    'rover_arm':        h_rover_arm,
    'rover_disarm':     h_rover_disarm,
    'rover_stop':       h_rover_stop,
    'rover_nudge':      h_rover_nudge,
}


# ── bench job ─────────────────────────────────────────────────────────────────

@job('wheels_test', 'Wheels test (OFF THE GROUND)', mode='run',
     requires=('rover',),
     args_schema={
         'rpm':     {'t': 'number', 'default': 150, 'min': 0, 'max': 600},
         'seconds': {'t': 'number', 'default': 1.0, 'min': 0.1, 'max': 5.0},
         'confirm': {'t': 'bool', 'default': False},
     })
def wheels_test(app, ctx, rpm: float = 150, seconds: float = 1.0,
                confirm: bool = False) -> dict:
    """Centre the wheels, then spin the motors briefly at a low speed.

    `confirm` defaults to off and the title says OFF THE GROUND, because this is
    the one job whose entire purpose is to make the rover move. A dry run reports
    what it would do.

    It drives the link directly rather than through the controller: the point is
    to check the wiring and the direction conventions before trusting the loop.
    """
    r = getattr(app.state, 'rover', None)
    if r is None:
        raise RuntimeError("no rover connected")

    rpm = int(round(rpm))
    plan = {'rpm': rpm, 'seconds': seconds}
    if not confirm:
        ctx.log("DRY RUN — tick `confirm` once the wheels are off the ground")
        ctx.progress(1.0, msg='dry run')
        return dict(plan, applied=False)

    link = r.link
    was_armed = r.armed
    # pause(), not just disarm(): a disarmed loop still WRITES a zero-speed
    # command every tick, which overwrites this job's within ~100 ms and makes
    # the whole test silently do nothing.
    r.pause()
    try:
        ctx.log('centering wheels')
        link.center_wheels()
        ctx.progress(0.3, msg='centered')

        ctx.log(f'spinning all six motors at {rpm} RPM for {seconds:.1f}s')
        link.set_all_motor_speeds_with_dir([rpm] * 6, [0, 0, 0, 1, 1, 1])
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            ctx.check_cancel()    # a cancel must reach the stop below
            time.sleep(0.05)
        ctx.progress(0.9, msg='stopping')
    finally:
        # Every exit path stops: normal end, cancel, or an exception mid-spin.
        link.full_stop()
        r.resume()
        if was_armed:
            ctx.log('left DISARMED — re-arm deliberately', level='warn')

    speeds = {}
    try:
        speeds = link.get_motor_speeds()
    except Exception:
        pass
    ctx.progress(1.0, msg='done')
    return dict(plan, applied=True, motors=speeds)
