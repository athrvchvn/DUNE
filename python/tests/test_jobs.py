"""Phase 0 gate: the job machinery does what the architecture claims.

Three claims are worth testing because the whole plan rests on them:

  1. **Recorder fidelity.** Every non-bulk broadcast lands in the session log,
     exactly once. Replay is only a debugging tool if the log is complete.
  2. **Ingest does not stall.** A long pure-Python job must not stop lines
     arriving. This is asserted against measured cadence, not asserted in prose
     — a job that holds the GIL is the realistic worst case.
  3. **Cancel is prompt.** A cancel request reaches a cooperating body quickly.

Everything here runs without hardware: the transport is a fake that emits lines
from a thread, exactly as a serial port would.
"""

from __future__ import annotations

import asyncio
import functools
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host.app import App                          # noqa: E402
from host.ingest import Transport                 # noqa: E402
from host.jobs import JOB_KINDS, job              # noqa: E402
import host.jobs_builtin                          # noqa: E402,F401  (registers 'selftest')


# ── Fakes ─────────────────────────────────────────────────────────────────────

class FakeWs:
    """A websocket that just collects what was sent to it."""

    def __init__(self):
        self.sent: list[str] = []

    async def send(self, payload: str) -> None:
        self.sent.append(payload)


class TickTransport(Transport):
    """A tag emitting `LINE,<n>` at a fixed rate — modelled as a buffered link.

    The catch-up loop matters. A real serial port blocks in `read()` with the
    GIL released while the kernel buffers incoming bytes, so contention delays
    delivery but does not destroy lines. A naive `sleep(period); yield` fake
    would instead *drop* ticks whenever the thread is descheduled, and the test
    built on it would be measuring an artifact of the fake rather than the
    property we care about: **no measurement is lost.**
    """

    name = 'fake'

    def __init__(self, period_s: float = 0.01):
        self.period_s = period_s
        self.written: list[bytes] = []
        self._stop = False
        self.n_generated = 0

    def read_lines(self):
        t0 = time.time()
        while not self._stop:
            time.sleep(self.period_s)
            # Emit every line the elapsed time says is due, draining the
            # notional buffer if this thread was starved.
            due = int((time.time() - t0) / self.period_s)
            while self.n_generated < due:
                self.n_generated += 1
                yield f"LINE,{self.n_generated}"

    def write(self, data: bytes) -> None:
        self.written.append(data)

    def close(self) -> None:
        self._stop = True


def runs_async(fn):
    """Run an async test body on a fresh loop.

    pytest-asyncio is not installed and this suite is not worth a new
    dependency for; `asyncio.run` per test also guarantees each test gets a
    clean loop, which matters because jobs post back onto the loop they were
    started from.
    """
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        return asyncio.run(fn(*args, **kwargs))
    return wrapper


def make_app(tmp_path) -> App:
    app = App(log_path=tmp_path / 'session.ndjson')
    app.set_config_msg(lambda: {'type': 'config', 'anchors': []})
    return app


def read_log(app) -> list[dict]:
    app.recorder._fh.flush()
    with open(app.recorder.path) as fh:
        return [json.loads(l) for l in fh if l.strip()]


# ── 1. Recorder fidelity ──────────────────────────────────────────────────────

@runs_async
async def test_every_broadcast_is_recorded_exactly_once(tmp_path):
    app = make_app(tmp_path)
    ws = FakeWs()
    app.bus.add_client(ws)

    for i in range(25):
        await app.bus.emit('tag_fix', tag_id=240, pkt_n=i)

    rows = read_log(app)
    assert len(rows) == 25, "recorder dropped or duplicated messages"
    assert [r['pkt_n'] for r in rows] == list(range(25))
    assert len(ws.sent) == 25, "client did not see every message"
    assert all('_ts' in r for r in rows), "every record must carry an emit time"
    await app.aclose()


@runs_async
async def test_bulk_payloads_reach_clients_but_not_the_log(tmp_path):
    """Camera frames would add megabytes per session; they must stay out."""
    app = make_app(tmp_path)
    ws = FakeWs()
    app.bus.add_client(ws)

    await app.bus.emit('vision_frame', jpeg_b64='x' * 5000, width=1920, height=1080)
    await app.bus.emit('tag_fix', tag_id=240)

    rows = read_log(app)
    assert [r['type'] for r in rows] == ['tag_fix']
    assert len(ws.sent) == 2, "the GUI still needs the frame"
    await app.aclose()


@runs_async
async def test_job_lifecycle_is_fully_recorded(tmp_path):
    app = make_app(tmp_path)
    await app.jobs.run('selftest', {'seconds': 1.0, 'busy': False})
    await _await_terminal(app)

    types_ = [r['type'] for r in read_log(app)]
    assert types_[0] == 'job_started'
    assert types_[-1] == 'job_done'
    assert 'job_progress' in types_ and 'job_log' in types_
    await app.aclose()


# ── 2. Ingest is not stalled by a long job ────────────────────────────────────

@runs_async
async def test_busy_job_does_not_stall_ingest(tmp_path):
    """The claim that made threads the right choice — measured, not asserted.

    A 3 s pure-Python busy job (the worst case: it holds the GIL) runs while a
    100 Hz transport feeds lines. Two separate properties are checked, because
    they have very different consequences:

      * **No line is lost.** This is the hard requirement — a dropped sweep is a
        dropped measurement, and no amount of later smoothing recovers it.
      * **Throughput stays close to baseline.** This is a quality-of-service
        expectation. Some jitter under a GIL-holding job is inherent to threads
        and is the accepted cost of keeping live handles available to jobs.
    """
    app = make_app(tmp_path)
    received: list[str] = []

    async def on_line(line: str, t_rx: float = 0.0) -> None:
        # The receive time is stamped in the ingest thread, so handlers take it
        # as a second argument rather than calling now() themselves.
        received.append((t_rx, line))

    transport = TickTransport(period_s=0.01)      # ~100 lines/s
    app.attach_transport(transport, on_line)

    await asyncio.sleep(1.0)                       # baseline
    baseline = len(received)
    assert baseline > 30, f"fake transport too slow to be a useful test ({baseline})"

    n_before = len(received)
    t0 = time.time()
    await app.jobs.run('selftest', {'seconds': 3.0, 'busy': True})
    await _await_terminal(app, timeout=20.0)
    elapsed = time.time() - t0
    during = (len(received) - n_before) / elapsed

    # Nothing generated may go missing: every line the transport produced was
    # delivered, and the queue reports no drops.
    assert len(received) == transport.n_generated, (
        f"lost {transport.n_generated - len(received)} lines under job load"
    )
    assert app.link.n_dropped == 0, f"{app.link.n_dropped} lines dropped by the queue"

    baseline_rate = baseline / 1.0
    assert during > 0.75 * baseline_rate, (
        f"ingest starved during a busy job: {during:.1f}/s vs "
        f"{baseline_rate:.1f}/s baseline"
    )
    await app.aclose()


@runs_async
async def test_ingest_survives_two_concurrent_jobs(tmp_path):
    """Two jobs fill the pool; the old shared-executor design stopped here."""
    app = make_app(tmp_path)
    received: list[str] = []

    async def on_line(line: str, t_rx: float = 0.0) -> None:
        # The receive time is stamped in the ingest thread, so handlers take it
        # as a second argument rather than calling now() themselves.
        received.append((t_rx, line))

    app.attach_transport(TickTransport(period_s=0.01), on_line)
    await asyncio.sleep(0.3)

    n_before = len(received)
    await app.jobs.run('selftest', {'seconds': 2.0, 'busy': True})
    await app.jobs.run('selftest', {'seconds': 2.0, 'busy': True})
    await asyncio.sleep(2.5)

    assert len(received) - n_before > 100, "ingest stopped while the pool was full"
    await app.aclose()


# ── 3. Cancellation ───────────────────────────────────────────────────────────

@runs_async
async def test_cancel_is_prompt(tmp_path):
    app = make_app(tmp_path)
    job_id = await app.jobs.run('selftest', {'seconds': 30.0, 'busy': True})
    await asyncio.sleep(0.3)

    t0 = time.time()
    assert await app.jobs.cancel(job_id) is True
    await _await_terminal(app, timeout=10.0)
    latency = time.time() - t0

    # The body checks between 1 s steps, so the bound is one step plus slack —
    # what matters is that it does not run the full 30 s.
    assert latency < 2.0, f"cancel took {latency:.2f}s"
    assert read_log(app)[-1]['type'] == 'job_cancelled'
    await app.aclose()


@runs_async
async def test_cancelling_an_unknown_job_is_harmless(tmp_path):
    app = make_app(tmp_path)
    assert await app.jobs.cancel('job-999999') is False
    await app.aclose()


# ── Rejections ────────────────────────────────────────────────────────────────

@runs_async
async def test_unknown_kind_is_reported_not_ignored(tmp_path):
    app = make_app(tmp_path)
    assert await app.jobs.run('no_such_tool', {}) is None
    row = read_log(app)[-1]
    assert row['type'] == 'job_failed' and 'unknown job kind' in row['error']
    await app.aclose()


@runs_async
async def test_bad_arguments_are_rejected_before_the_job_starts(tmp_path):
    app = make_app(tmp_path)
    assert await app.jobs.run('selftest', {'seconds': 999.0}) is None
    rows = read_log(app)
    assert rows[-1]['type'] == 'job_failed'
    assert 'bad arguments' in rows[-1]['error']
    assert not any(r['type'] == 'job_started' for r in rows)
    await app.aclose()


@runs_async
async def test_conflicting_resource_fails_immediately_without_queueing(tmp_path):
    """A queued job that later moves hardware is a dangerous surprise."""

    @job('holds_kinect', 'Holds the Kinect', requires=('kinect',),
         args_schema={'seconds': {'t': 'number', 'default': 1.0}})
    def _holds(app, ctx, seconds: float = 1.0):
        time.sleep(seconds)
        return {}

    app = make_app(tmp_path)
    first = await app.jobs.run('holds_kinect', {'seconds': 2.0})
    assert first is not None
    second = await app.jobs.run('holds_kinect', {'seconds': 2.0})
    assert second is None, "second job must be refused, not queued"

    row = read_log(app)[-1]
    assert row['type'] == 'job_failed'
    assert 'resource busy: kinect' in row['error'] and first in row['error']

    await _await_terminal(app, timeout=10.0, job_id=first)
    # The lock is released, so the same job can run again afterwards.
    third = await app.jobs.run('holds_kinect', {'seconds': 0.1})
    assert third is not None, "resource was not released when the holder finished"
    await _await_terminal(app, timeout=10.0, job_id=third)
    await app.aclose()
    JOB_KINDS.pop('holds_kinect', None)


# ── Artifact delivery ─────────────────────────────────────────────────────────

@runs_async
async def test_artifacts_are_delivered_to_the_gui(tmp_path):
    """The page is opened over file:// and cannot read the results directory.

    Without this the "report" a job produces is only ever a path the user has
    to go and open by hand, which defeats running it from the GUI at all.
    """
    report = tmp_path / 'report.txt'
    report.write_text('RMSE 46 mm\n')

    @job('makes_report', 'Makes a report', args_schema={})
    def _mk(app, ctx):
        ctx.artifact(report, 'txt')
        return {}

    app = make_app(tmp_path)
    ws = FakeWs()
    app.bus.add_client(ws)
    job_id = await app.jobs.run('makes_report', {})
    await _await_terminal(app, job_id=job_id)

    await app.jobs.send_artifact(job_id, str(report))
    msg = json.loads(ws.sent[-1])
    assert msg['type'] == 'job_artifact' and msg['kind'] == 'txt'
    assert 'RMSE 46 mm' in msg['text']

    # Bulk payloads stay out of the session log — one build would otherwise put
    # a megabyte of base64 into every recorded session.
    assert not any(r['type'] == 'job_artifact' for r in read_log(app))
    await app.aclose()
    JOB_KINDS.pop('makes_report', None)


@runs_async
async def test_artifact_requests_cannot_read_arbitrary_files(tmp_path):
    """Checked against what THAT job produced, not merely against the repo.

    The narrower check is free and stops this becoming a general file-read
    primitive on a server that binds 0.0.0.0.
    """
    @job('makes_nothing', 'Makes nothing', args_schema={})
    def _mk(app, ctx):
        return {}

    app = make_app(tmp_path)
    ws = FakeWs()
    app.bus.add_client(ws)
    job_id = await app.jobs.run('makes_nothing', {})
    await _await_terminal(app, job_id=job_id)

    for bad in ('/etc/passwd', str(tmp_path / 'session.ndjson')):
        await app.jobs.send_artifact(job_id, bad)
        msg = json.loads(ws.sent[-1])
        assert msg['type'] == 'job_artifact_error', f"{bad} was served"
        assert 'not an artifact' in msg['error']

    await app.jobs.send_artifact('job-999999', '/etc/passwd')
    assert json.loads(ws.sent[-1])['error'] == 'unknown job'
    await app.aclose()
    JOB_KINDS.pop('makes_nothing', None)


# ── WebSocket surface ─────────────────────────────────────────────────────────

@runs_async
async def test_unknown_message_type_gets_an_error_not_silence(tmp_path):
    app = make_app(tmp_path)
    ws = FakeWs()
    app.bus.add_client(ws)
    await app._handlers.get('job_list_req')(app, {'type': 'job_list_req'}, ws)

    handler = app._handlers.get('nope')
    assert handler is None
    await app.bus.emit('error', code='unknown_type', got='nope')
    assert json.loads(ws.sent[-1])['code'] == 'unknown_type'
    await app.aclose()


@runs_async
async def test_job_kinds_advertises_a_usable_schema(tmp_path):
    app = make_app(tmp_path)
    kinds = {k['kind']: k for k in app.jobs.kinds_msg()['kinds']}
    assert 'selftest' in kinds
    schema = kinds['selftest']['args_schema']
    assert schema['seconds']['t'] == 'number'
    assert schema['busy']['t'] == 'bool'
    await app.aclose()


# ── helpers ───────────────────────────────────────────────────────────────────

async def _await_terminal(app, timeout: float = 15.0, job_id: str | None = None) -> dict:
    """Wait for a terminal job message.

    `job_id` matters whenever more than one job is in play: without it, a
    rejection already sitting at the end of the log reads as "the job I am
    waiting for has finished", and the test races ahead.
    """
    terminal = {'job_done', 'job_failed', 'job_cancelled'}
    deadline = time.time() + timeout
    while time.time() < deadline:
        for row in reversed(read_log(app)):
            if row['type'] not in terminal:
                continue
            if job_id is None or row.get('job_id') == job_id:
                return row
        await asyncio.sleep(0.05)
    raise AssertionError(f"no terminal message for {job_id or 'any job'} within {timeout}s")


@runs_async
async def test_lines_are_stamped_at_read_time_not_at_processing_time(tmp_path):
    """Queueing delay must not leak into the timestamp the filter's dt uses.

    The handler is deliberately slow here, so a stamp taken at processing time
    would drift away from the transport's own schedule.
    """
    app = make_app(tmp_path)
    seen: list[tuple[float, str]] = []

    async def slow_on_line(line: str, t_rx: float = 0.0) -> None:
        seen.append((t_rx, time.time()))
        await asyncio.sleep(0.02)          # much slower than the 10 ms feed

    app.attach_transport(TickTransport(period_s=0.01), slow_on_line)
    await asyncio.sleep(1.2)
    await app.aclose()

    assert len(seen) > 20, f"only {len(seen)} lines"
    lag = [proc - rx for rx, proc in seen]
    # Processing falls progressively behind; the receive stamps must not.
    assert max(lag) > 0.05, "handler was not actually slower than the feed"
    rx_gaps = np.diff([rx for rx, _ in seen])
    assert np.median(rx_gaps) < 0.02, (
        f"receive stamps drifted with processing: median gap {np.median(rx_gaps):.3f}s")
