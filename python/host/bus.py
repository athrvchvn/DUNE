"""The outbound message bus and the NDJSON session recorder.

Every message the GUI sees goes through `Bus.broadcast`, and (unless explicitly
excluded) is written to the session log on the way out. That is what makes a
recorded session replay identically to the live run: the GUI's replay mode reads
the NDJSON back and re-dispatches it through the same code path.

Two rules keep that property true, and both are load-bearing:

  1. **No unicast.** Every client sees the same stream, so the recording is
     unambiguously "what the GUI saw". A per-client message would silently make
     the log a lie. If one ever becomes unavoidable, record it with a `_to`
     field rather than skipping the recorder.
  2. **record=False is only for bulk binary.** Camera frames (`vision_frame`,
     `vision_frame_live`, `vision_register_frame`, `job_artifact`) carry base64
     images that are meaningless for RTLS replay and would add megabytes per
     session. Everything else records.
"""

from __future__ import annotations

import asyncio
import json
import math
import time
from pathlib import Path
from typing import Any, Iterable


def dumps(obj) -> str:
    """`json.dumps`, but never emitting a payload a browser will reject.

    Python writes non-finite floats as bare ``NaN`` / ``Infinity``, which are
    not valid JSON. `JSON.parse` throws on them, and because the GUI's
    `dispatch()` swallows exceptions the whole message would vanish with no
    error anywhere — the worst possible failure mode. Statistics over a session
    produce NaN routinely (an all-masked interval, a tag that never solved), so
    this is a live hazard, not a theoretical one.

    Non-finite values become ``null``. The fast path costs one extra flag on the
    normal case and only pays for the cleaning walk when something is actually
    non-finite.
    """
    try:
        return json.dumps(obj, allow_nan=False)
    except ValueError:
        return json.dumps(_finite(obj), allow_nan=False)


def _finite(o):
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_finite(v) for v in o]
    return o


class SessionRecorder:
    """Append every broadcast message as one NDJSON line (the replay format).

    One JSON object per line, each stamped with `_ts` (emit wall time). Flushed
    per line so a killed process still leaves a readable log — replay tolerates a
    truncated final line by simply ending there.
    """

    def __init__(self, path: Path):
        self.path = Path(path)
        self._fh = open(self.path, 'w')
        self.n_records = 0

    def record(self, msg: str | dict) -> None:
        try:
            obj = json.loads(msg) if isinstance(msg, str) else dict(msg)
            obj.setdefault('_ts', round(time.time(), 3))
            self._fh.write(dumps(obj) + '\n')
            self._fh.flush()
            self.n_records += 1
        except Exception:
            pass  # recording must never take down the pipeline

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


class Bus:
    """Fan-out to every connected client, with recording on the same path."""

    # Message types whose payload is bulk binary (base64 imagery). Excluded from
    # the session log by default; see the module docstring.
    BULK_TYPES = frozenset({
        'vision_frame',
        'vision_frame_live',
        'vision_register_frame',
        'job_artifact',
    })

    def __init__(self, recorder: SessionRecorder | None = None):
        self.clients: set = set()
        self.recorder = recorder
        self.n_broadcasts = 0

    # ── client bookkeeping ───────────────────────────────────────────────────

    def add_client(self, ws) -> int:
        self.clients.add(ws)
        return len(self.clients)

    def discard_client(self, ws) -> int:
        self.clients.discard(ws)
        return len(self.clients)

    # ── sending ──────────────────────────────────────────────────────────────

    async def broadcast(self, payload: str | dict, record: bool | None = None) -> None:
        """Send one message to every client, recording it unless it is bulk.

        `record=None` (the default) decides from the message type, so callers
        never have to remember which families are bulk. Pass an explicit bool
        only to override that.
        """
        if isinstance(payload, dict):
            msg_type = payload.get('type')
            text = dumps(payload)
        else:
            text = payload
            msg_type = _peek_type(text)

        if record is None:
            record = msg_type not in self.BULK_TYPES

        if self.recorder is not None and record:
            self.recorder.record(text)

        self.n_broadcasts += 1

        if self.clients:
            await asyncio.gather(
                *[c.send(text) for c in list(self.clients)],
                return_exceptions=True,
            )

    async def emit(self, type_: str, **fields: Any) -> None:
        """Convenience wrapper: `await bus.emit('job_progress', job_id=..., frac=...)`."""
        await self.broadcast({'type': type_, **fields})

    async def send_one(self, ws, payload: str | dict) -> None:
        """Send to a single client WITHOUT recording.

        The only legitimate use is the per-connection handshake (`config`,
        `job_kinds`), where the same content is also recorded once as a replay
        keyframe. Never use this to give one client different information.
        """
        text = payload if isinstance(payload, str) else dumps(payload)
        try:
            await ws.send(text)
        except Exception:
            pass


def _peek_type(text: str) -> str | None:
    """Read the `type` field without paying for a full parse of a large payload.

    Camera-frame messages are hundreds of kilobytes of base64; parsing one just
    to learn it should not be recorded would defeat the point of skipping it.
    `json.dumps` writes `type` first for our messages, so a bounded prefix scan
    finds it; anything unusual falls back to a real parse.
    """
    head = text[:200]
    marker = '"type":'
    i = head.find(marker)
    if i >= 0:
        rest = head[i + len(marker):].lstrip()
        if rest.startswith('"'):
            end = rest.find('"', 1)
            if end > 0:
                return rest[1:end]
    try:
        return json.loads(text).get('type')
    except Exception:
        return None


def iter_clients(bus: Bus) -> Iterable:
    return list(bus.clients)
