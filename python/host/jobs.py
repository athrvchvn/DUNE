"""Long-running operations, off the event loop, with progress and cancellation.

Why this exists: the GUI needs to drive ~29 tools, several of which run for
minutes (replay over a 26k-line log, an anchor delay sweep, a rover run report).
Previously every such operation ran inline in the WebSocket handler, so the
whole app — including the UWB reader — froze for its duration.

## Where jobs run

A `ThreadPoolExecutor`, not processes. Job bodies need live handles that do not
survive a fork or a pickle: the open serial port, the Kinect device, the running
pipeline, and this bus for streaming progress. Every heavy library in play
(numpy, scipy, OpenCV, CasADi) releases the GIL inside its C loops, so the
ingest threads keep getting scheduled.

The residual risk is a *pure-Python* hot loop, which holds the GIL in ~5 ms
slices and jitters (does not stall) ingest. The escape hatch is swapping this
executor for a `ProcessPoolExecutor` — which stays one line away only as long as
offline job bodies take **file paths in, JSON out, no live handles**. Enforce
that when writing them; retrofitting it later across a dozen jobs is expensive.

## Cancellation

Cooperative. A thread cannot be killed safely, so job bodies must call
`ctx.check_cancel()` between units of work (or poll `ctx.cancelled`). A body
that never checks simply runs to completion and reports `job_cancelled`
afterwards, which is honest rather than a lie about having stopped.

## Resources

Jobs declare `requires=('kinect',)`. A conflicting request fails immediately
rather than queueing: a queued job that later moves hardware — wheels, antenna
delays — is a genuinely dangerous surprise.
"""

from __future__ import annotations

import asyncio
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FuturesTimeout
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

# ── Argument schema ───────────────────────────────────────────────────────────
# Five types, deliberately: the GUI auto-generates its form from this, and a
# small closed set keeps that generator ~60 lines instead of a framework.
#   number  {"t":"number","default":0.24,"min":0,"max":2,"step":0.01}
#   bool    {"t":"bool","default":true}
#   string  {"t":"string","default":""}
#   enum    {"t":"enum","choices":["ekf","mhe"],"default":"ekf"}
#   path    {"t":"path","kind":"logs"|"rover_runs"|"results"|"config"}
SCHEMA_TYPES = frozenset({'number', 'bool', 'string', 'enum', 'path'})


class JobCancelled(Exception):
    """Raised by `JobContext.check_cancel()` when a cancel has been requested."""


class ResourceBusy(Exception):
    """A job asked for a resource another job is holding."""


@dataclass
class JobSpec:
    """One runnable tool, as advertised to the GUI in the `job_kinds` message."""
    kind:        str
    title:       str
    fn:          Callable
    mode:        str = 'lab'          # which GUI mode surfaces it: lab | cal | run
    requires:    tuple = ()           # resource names this job needs exclusively
    args_schema: dict = field(default_factory=dict)
    cancellable: bool = True

    def public(self) -> dict:
        return {
            'kind':        self.kind,
            'title':       self.title,
            'mode':        self.mode,
            'requires':    list(self.requires),
            'cancellable': self.cancellable,
            'args_schema': self.args_schema,
        }


JOB_KINDS: dict[str, JobSpec] = {}


def job(kind: str, title: str, *, mode: str = 'lab', requires: tuple = (),
        args_schema: dict | None = None, cancellable: bool = True):
    """Register a job body. The function is called as `fn(app, ctx, **args)`."""
    def deco(fn: Callable) -> Callable:
        for name, spec in (args_schema or {}).items():
            t = spec.get('t')
            if t not in SCHEMA_TYPES:
                raise ValueError(f"{kind}.{name}: bad arg type {t!r}")
            if t == 'enum' and not spec.get('choices'):
                raise ValueError(f"{kind}.{name}: enum needs choices")
        JOB_KINDS[kind] = JobSpec(
            kind=kind, title=title, fn=fn, mode=mode, requires=tuple(requires),
            args_schema=args_schema or {}, cancellable=cancellable,
        )
        return fn
    return deco


# ── Job context ───────────────────────────────────────────────────────────────

class JobContext:
    """Handed to a job body. Progress calls are safe from the worker thread.

    Every emit hops back onto the event loop via `run_coroutine_threadsafe` and
    does NOT wait for the result — a job must never block on the GUI being slow
    to drain, and a disconnected browser must not stall a calibration.
    """

    def __init__(self, job_id: str, kind: str, args: dict, loop, bus):
        self.job_id = job_id
        self.kind = kind
        self.args = args
        self._loop = loop
        self._bus = bus
        self._cancel = threading.Event()
        self.started_ts = time.time()
        self.artifacts: list[dict] = []
        self.frac = 0.0        # last reported progress, for `job_list`

    # ── cancellation ─────────────────────────────────────────────────────────

    def cancel(self) -> None:
        self._cancel.set()

    @property
    def cancelled(self) -> bool:
        return self._cancel.is_set()

    def check_cancel(self) -> None:
        """Call between units of work. Raises `JobCancelled` if cancel was asked."""
        if self._cancel.is_set():
            raise JobCancelled()

    # ── driving the event loop from a worker thread ──────────────────────────

    def run_coro(self, coro, timeout: float | None = None):
        """Run a coroutine on the event loop and wait for its result.

        Hardware jobs need this: the tag link, its subscriptions and its queues
        all live on the loop, but job bodies run on a worker thread. Scheduling
        the coroutine there and blocking *this* thread for the answer keeps the
        link single-threaded while leaving the job free to take as long as it
        needs — the loop is never blocked, only this worker is.

        Cancellation still works, because the coroutine polls `check_cancel`.

        The wait is deliberately CHUNKED rather than a single unbounded
        `result()`. If the loop stops while this worker is blocked — shutdown,
        or a test whose `asyncio.run` has returned — the scheduled coroutine can
        never complete, so an unbounded wait blocks forever. `concurrent.futures`
        threads are non-daemon, so the interpreter then hangs at exit with no
        traceback. Polling lets a dead loop and a cancel both surface.
        """
        if self._loop.is_closed():
            raise JobCancelled()

        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        deadline = None if timeout is None else time.monotonic() + timeout
        try:
            while True:
                if deadline is not None:
                    left = deadline - time.monotonic()
                    if left <= 0:
                        raise TimeoutError(f"run_coro timed out after {timeout}s")
                else:
                    left = None
                try:
                    return fut.result(min(0.25, left) if left is not None else 0.25)
                except FuturesTimeout:
                    if self._loop.is_closed() or not self._loop.is_running():
                        raise JobCancelled()
                    if self._cancel.is_set():
                        raise JobCancelled()
        except BaseException:
            fut.cancel()
            raise

    # ── reporting (thread-safe, fire-and-forget) ─────────────────────────────

    def _emit(self, type_: str, **fields) -> None:
        coro = self._bus.emit(type_, job_id=self.job_id, **fields)
        try:
            asyncio.run_coroutine_threadsafe(coro, self._loop)
        except RuntimeError:
            pass  # loop already closed (shutdown race) — nothing useful to do

    def progress(self, frac: float | None = None, *, msg: str = '',
                 step: int | None = None, steps: int | None = None,
                 eta_s: float | None = None) -> None:
        if frac is not None:
            self.frac = float(frac)
        self._emit('job_progress', frac=frac, msg=msg, step=step, steps=steps,
                   eta_s=eta_s)

    def log(self, msg: str, level: str = 'info') -> None:
        self._emit('job_log', level=level, msg=str(msg))

    def artifact(self, path, kind: str = 'file', nbytes: int | None = None) -> None:
        """Record a file this job produced. Reported in `job_done.artifacts`."""
        p = Path(path)
        if nbytes is None:
            try:
                nbytes = p.stat().st_size
            except OSError:
                nbytes = None
        self.artifacts.append({'path': str(p), 'kind': kind, 'bytes': nbytes})


# ── Resource locks ────────────────────────────────────────────────────────────

class ResourceTable:
    """Who holds what. Deliberately non-blocking: conflicts fail, never queue."""

    def __init__(self):
        self._held: dict[str, str] = {}   # resource -> job_id
        self._lock = threading.Lock()

    def acquire(self, names: tuple, job_id: str) -> None:
        with self._lock:
            for n in names:
                holder = self._held.get(n)
                if holder is not None:
                    raise ResourceBusy(f"resource busy: {n} (held by {holder})")
            for n in names:
                self._held[n] = job_id

    def release(self, names: tuple, job_id: str) -> None:
        with self._lock:
            for n in names:
                if self._held.get(n) == job_id:
                    self._held.pop(n, None)

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._held)


# ── Registry ──────────────────────────────────────────────────────────────────

@dataclass
class JobRecord:
    job_id:  str
    kind:    str
    title:   str
    ctx:     JobContext
    state:   str = 'running'       # running | done | failed | cancelled
    started_ts: float = 0.0


class JobRegistry:
    """Starts jobs, tracks them, and reports their lifecycle on the bus."""

    def __init__(self, app, max_workers: int = 2):
        self.app = app
        self._pool = ThreadPoolExecutor(max_workers=max_workers,
                                        thread_name_prefix='job')
        self._jobs: dict[str, JobRecord] = {}
        self._seq = 0
        self.resources = ResourceTable()

    # ── introspection ────────────────────────────────────────────────────────

    def kinds_msg(self) -> dict:
        return {'type': 'job_kinds',
                'kinds': [s.public() for s in JOB_KINDS.values()]}

    def list_msg(self) -> dict:
        return {'type': 'job_list', 'jobs': [
            {'job_id': r.job_id, 'kind': r.kind, 'title': r.title,
             'state': r.state, 'frac': r.ctx.frac, 'started_ts': r.started_ts}
            for r in self._jobs.values()
        ]}

    # ── argument coercion ────────────────────────────────────────────────────

    @staticmethod
    def coerce_args(spec: JobSpec, raw: dict) -> dict:
        """Validate and coerce GUI-supplied args. Raises ValueError on bad input.

        The GUI sends everything a form produces, i.e. strings. Coercing here —
        rather than in each job body — is what lets job bodies be plain typed
        Python functions that are also callable from a test.
        """
        out: dict = {}
        for name, sc in spec.args_schema.items():
            t = sc['t']
            if name not in raw or raw[name] is None or raw[name] == '':
                if 'default' in sc:
                    out[name] = sc['default']
                    continue
                if sc.get('required'):
                    raise ValueError(f"missing required argument: {name}")
                out[name] = None
                continue
            v = raw[name]
            try:
                if t == 'number':
                    v = float(v)
                    if 'min' in sc and v < sc['min']:
                        raise ValueError(f"{name} < {sc['min']}")
                    if 'max' in sc and v > sc['max']:
                        raise ValueError(f"{name} > {sc['max']}")
                elif t == 'bool':
                    v = v if isinstance(v, bool) else str(v).lower() in ('1', 'true', 'yes', 'on')
                elif t == 'enum':
                    v = str(v)
                    if v not in sc['choices']:
                        raise ValueError(f"{name}: {v!r} not in {sc['choices']}")
                else:
                    v = str(v)
            except ValueError:
                raise
            except Exception as e:
                raise ValueError(f"{name}: {e}") from e
            out[name] = v
        return out

    # ── lifecycle ────────────────────────────────────────────────────────────

    async def run(self, kind: str, raw_args: dict, req_id: str | None = None) -> str | None:
        """Start a job. Emits `job_started`, or `job_failed` if it cannot start.

        Returns the job_id, or None when the job was rejected. Rejection is
        reported as `job_failed` with a synthetic id so the GUI can attach the
        error to the request that caused it via `req_id`.
        """
        bus = self.app.bus
        spec = JOB_KINDS.get(kind)
        if spec is None:
            await bus.emit('job_failed', job_id='job-rejected', kind=kind,
                           req_id=req_id, elapsed_s=0.0,
                           error=f"unknown job kind: {kind}", trace='')
            return None

        try:
            args = self.coerce_args(spec, raw_args or {})
        except ValueError as e:
            await bus.emit('job_failed', job_id='job-rejected', kind=kind,
                           req_id=req_id, elapsed_s=0.0,
                           error=f"bad arguments: {e}", trace='')
            return None

        self._seq += 1
        job_id = f"job-{self._seq:06d}"

        try:
            self.resources.acquire(spec.requires, job_id)
        except ResourceBusy as e:
            await bus.emit('job_failed', job_id=job_id, kind=kind, req_id=req_id,
                           elapsed_s=0.0, error=str(e), trace='')
            return None

        loop = asyncio.get_running_loop()
        ctx = JobContext(job_id, kind, args, loop, bus)
        rec = JobRecord(job_id=job_id, kind=kind, title=spec.title, ctx=ctx,
                        started_ts=ctx.started_ts)
        self._jobs[job_id] = rec

        await bus.emit('job_started', job_id=job_id, kind=kind, title=spec.title,
                       req_id=req_id, cancellable=spec.cancellable,
                       started_ts=round(ctx.started_ts, 3))

        fut = self._pool.submit(self._run_body, spec, ctx)
        fut.add_done_callback(lambda f: self._schedule_finish(rec, spec, f, loop))
        return job_id

    def _schedule_finish(self, rec: JobRecord, spec: JobSpec, fut, loop) -> None:
        """Hand completion back to the event loop, tolerating shutdown.

        A job outliving the loop is normal, not exceptional: `shutdown()` does
        not wait for in-flight work, so on exit a body can finish after the loop
        is gone. Without this guard that raced into a `RuntimeError` traceback
        plus a "coroutine was never awaited" warning on every such exit.
        """
        coro = self._finish(rec, spec, fut)
        try:
            asyncio.run_coroutine_threadsafe(coro, loop)
        except RuntimeError:
            coro.close()
            self.resources.release(spec.requires, rec.job_id)

    def _run_body(self, spec: JobSpec, ctx: JobContext):
        """Executed on a worker thread."""
        return spec.fn(self.app, ctx, **ctx.args)

    async def _finish(self, rec: JobRecord, spec: JobSpec, fut):
        bus = self.app.bus
        ctx = rec.ctx
        elapsed = round(time.time() - ctx.started_ts, 3)
        self.resources.release(spec.requires, rec.job_id)
        try:
            result = fut.result()
        except JobCancelled:
            rec.state = 'cancelled'
            await bus.emit('job_cancelled', job_id=rec.job_id, elapsed_s=elapsed)
            return
        except Exception as e:
            rec.state = 'failed'
            await bus.emit('job_failed', job_id=rec.job_id, elapsed_s=elapsed,
                           error=f"{type(e).__name__}: {e}",
                           trace=traceback.format_exc(limit=12))
            return
        if ctx.cancelled:
            # Body finished without noticing the cancel. Say so plainly rather
            # than reporting a success the user asked us to abandon.
            rec.state = 'cancelled'
            await bus.emit('job_cancelled', job_id=rec.job_id, elapsed_s=elapsed)
            return
        rec.state = 'done'
        ctx.frac = 1.0
        await bus.emit('job_done', job_id=rec.job_id, elapsed_s=elapsed,
                       result=result if isinstance(result, dict) else {},
                       artifacts=ctx.artifacts)

    async def send_artifact(self, job_id: str, path: str) -> None:
        """Deliver one artifact's contents to the GUI.

        The page is opened over `file://` and cannot read the results directory
        itself, so without this a report is only ever a path the user has to go
        and open by hand — which defeats the point of running it from the GUI.

        `path` is checked against the artifacts THAT JOB actually produced, not
        merely against the repo root. The narrower check is free here and means
        this cannot become a general file-read primitive.
        """
        bus = self.app.bus
        rec = self._jobs.get(job_id)
        if rec is None:
            await bus.emit('job_artifact_error', job_id=job_id, path=path,
                           error='unknown job')
            return
        match = next((a for a in rec.ctx.artifacts if a['path'] == path), None)
        if match is None:
            await bus.emit('job_artifact_error', job_id=job_id, path=path,
                           error='not an artifact of this job')
            return

        p = Path(match['path'])
        try:
            if match.get('kind') == 'png':
                import base64
                payload = base64.b64encode(p.read_bytes()).decode('ascii')
                await bus.emit('job_artifact', job_id=job_id, path=path,
                               kind='png', b64=payload)
            else:
                # Bounded: a report is a few KB, but a job could name anything.
                text = p.read_text(errors='replace')[:200_000]
                await bus.emit('job_artifact', job_id=job_id, path=path,
                               kind='txt', text=text)
        except OSError as e:
            await bus.emit('job_artifact_error', job_id=job_id, path=path,
                           error=str(e))

    async def cancel(self, job_id: str) -> bool:
        rec = self._jobs.get(job_id)
        if rec is None or rec.state != 'running':
            return False
        rec.ctx.cancel()
        return True

    def shutdown(self) -> None:
        for rec in self._jobs.values():
            if rec.state == 'running':
                rec.ctx.cancel()
        self._pool.shutdown(wait=False)
