"""`App` — the context object that replaces the module-level globals.

Everything a handler or job body needs hangs off one `app`, passed as the first
argument. That is the whole point: with ~18 loose globals there was no way to
run two things at once, no way to test a handler, and no way to reason about
what a job could touch.

`App` deliberately does NOT know about the domain (anchors, vision, calibration).
It owns the plumbing — bus, jobs, ingest, WebSocket lifecycle — and holds domain
objects in `app.state`, a plain namespace the caller populates. Handlers are
registered rather than hard-coded, so the domain layer can migrate over module
by module without this file changing.
"""

from __future__ import annotations

import contextlib
import json
import sys
import types
from pathlib import Path
from typing import Awaitable, Callable, Optional

from .bus import Bus, SessionRecorder
from .ingest import TagLink, Transport
from .jobs import JobRegistry

Handler = Callable[["App", dict, object], Awaitable[None]]

# CPython hands the GIL between threads at most this often. The default 5 ms is
# tuned for throughput; here a job body that spends 5 ms in pure Python delays
# the ingest thread by that much, and measured line rate roughly halves under a
# GIL-holding job. Shortening the slice trades a little job throughput for
# markedly steadier ingest — the right side of that trade when the thing being
# delayed is a live measurement stream that cannot be recovered.
GIL_SWITCH_INTERVAL_S = 0.002


class App:
    """Holds the running host: transports, bus, jobs, handlers, domain state."""

    def __init__(self, *, log_path: Path | None = None, max_job_workers: int = 2):
        sys.setswitchinterval(GIL_SWITCH_INTERVAL_S)
        self.recorder = SessionRecorder(log_path) if log_path else None
        self.bus = Bus(self.recorder)
        self.jobs = JobRegistry(self, max_workers=max_job_workers)
        self.link: Optional[TagLink] = None

        # Domain objects live here (pipeline, cfg, calib_mgr, vision, rover…).
        # A namespace rather than fixed attributes so the domain layer can move
        # over incrementally without this file having to know about each piece.
        self.state = types.SimpleNamespace()

        self._handlers: dict[str, Handler] = {}
        self._config_msg: Callable[[], dict] | None = None
        self._on_client_gone: Callable[[int], Awaitable[None]] | None = None
        self._closing = False

        self.register_handlers({
            'job_run':          _h_job_run,
            'job_cancel':       _h_job_cancel,
            'job_list_req':     _h_job_list,
            'job_artifact_req': _h_job_artifact,
        })

    # ── registration ─────────────────────────────────────────────────────────

    def register_handlers(self, mapping: dict[str, Handler]) -> None:
        """Add inbound WebSocket message handlers. Later registrations win."""
        self._handlers.update(mapping)

    def set_config_msg(self, fn: Callable[[], dict]) -> None:
        """Supply the `config` keyframe builder (domain-owned)."""
        self._config_msg = fn

    def set_on_client_gone(self, fn: Callable[[int], Awaitable[None]]) -> None:
        """Called after each disconnect with the remaining client count.

        This is the hook rover arming uses: no clients means nobody is watching,
        so anything that can move must stand down.
        """
        self._on_client_gone = fn

    # ── ingest ───────────────────────────────────────────────────────────────

    def attach_transport(self, transport: Transport,
                         on_line: Callable[[str], Awaitable[None]]) -> TagLink:
        self.link = TagLink(transport)
        self.link.start(on_line)
        return self.link

    def send_to_tag(self, cmd: str) -> None:
        if self.link is None:
            raise RuntimeError("no transport attached")
        self.link.send(cmd)

    # ── WebSocket ────────────────────────────────────────────────────────────

    async def ws_handler(self, websocket) -> None:
        n = self.bus.add_client(websocket)
        print(f"[WS]  Browser connected  ({n} total)")

        # Handshake: the two keyframes a fresh client needs to render anything.
        # Both are also recorded once at startup, so replay reconstructs them.
        if self._config_msg is not None:
            await self.bus.send_one(websocket, self._config_msg())
        await self.bus.send_one(websocket, self.jobs.kinds_msg())

        try:
            async for msg in websocket:
                try:
                    data = json.loads(msg)
                except Exception:
                    continue
                mtype = data.get('type')
                handler = self._handlers.get(mtype)
                if handler is None:
                    # Previously unknown types were dropped in silence, which
                    # made a typo indistinguishable from a broken feature.
                    await self.bus.emit('error', code='unknown_type',
                                        got=mtype, req_id=data.get('req_id'))
                    continue
                try:
                    await handler(self, data, websocket)
                except Exception as e:
                    print(f"[WS] Error handling {mtype}: {e}")
                    await self.bus.emit('error', code='handler_failed',
                                        got=mtype, req_id=data.get('req_id'),
                                        error=f"{type(e).__name__}: {e}")
        finally:
            n = self.bus.discard_client(websocket)
            print(f"[WS]  Browser disconnected  ({n} total)")
            if self._on_client_gone is not None:
                with contextlib.suppress(Exception):
                    await self._on_client_gone(n)

    # ── startup / shutdown ───────────────────────────────────────────────────

    async def record_keyframes(self) -> None:
        """Write the replay keyframes into the session log exactly once."""
        if self.recorder is None:
            return
        if self._config_msg is not None:
            self.recorder.record(self._config_msg())
        self.recorder.record(self.jobs.kinds_msg())

    async def aclose(self) -> None:
        if self._closing:
            return
        self._closing = True
        self.jobs.shutdown()
        if self.link is not None:
            await self.link.stop()
        if self.recorder is not None:
            self.recorder.close()


# ── Built-in handlers ─────────────────────────────────────────────────────────

async def _h_job_run(app: App, data: dict, ws) -> None:
    await app.jobs.run(data.get('kind', ''), data.get('args') or {},
                       req_id=data.get('req_id'))


async def _h_job_cancel(app: App, data: dict, ws) -> None:
    await app.jobs.cancel(data.get('job_id', ''))


async def _h_job_list(app: App, data: dict, ws) -> None:
    await app.bus.broadcast(app.jobs.list_msg())


async def _h_job_artifact(app: App, data: dict, ws) -> None:
    await app.jobs.send_artifact(data.get('job_id', ''), data.get('path', ''))
