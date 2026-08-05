"""Transport readers and shared access to the tag link.

Two problems this solves.

**Ingest must never be starved.** The old readers called
`loop.run_in_executor(None, ser.readline)`, sharing the default executor with
everything else. Two concurrent jobs would occupy it and the UWB stream would
simply stop — the exact failure mode a GUI that runs long tools invites. Here
each transport gets a dedicated thread that blocks on the OS read and hands
lines to the loop via `call_soon_threadsafe`, so executor saturation is
structurally unable to touch ingest.

**The tag link has more than one consumer.** The live pipeline reads sweeps from
it, but calibration also needs to inject commands and read specific replies.
Rather than a mutex that would stop positioning for the duration, `TagLink`
offers two mechanisms:

  * `subscribe(*prefixes)` — matching lines are diverted to the subscriber and
    everything else keeps flowing to the pipeline. Calibration runs *while you
    watch the live map*, which is what makes it usable.
  * `exclusive()` — dispatch pauses entirely and the caller drives the link.
    Only for operations where the firmware itself stops sweeping (`SURVEY`,
    CIR capture), so there is nothing to keep flowing anyway.

The port is opened once and never reopened: re-opening a CDC port can reset the
board, which would lose tag state mid-session.
"""

from __future__ import annotations

import asyncio
import contextlib
import socket
import threading
import time
from typing import Callable, Iterator, Optional

try:
    import serial as pyserial
except ImportError:                                   # pragma: no cover
    pyserial = None


# ── Transports ────────────────────────────────────────────────────────────────

class Transport:
    """A source of newline-delimited text from the tag, plus a way to talk back."""

    name = 'none'

    def read_lines(self) -> Iterator[str]:
        raise NotImplementedError

    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class SerialTransport(Transport):
    name = 'serial'

    def __init__(self, port: str, baud: int = 115200):
        if pyserial is None:
            raise RuntimeError("pyserial not installed: pip install pyserial")
        self.port = port
        self.baud = baud
        self._ser = pyserial.Serial(port, baud, timeout=1.0)

    def read_lines(self) -> Iterator[str]:
        while True:
            raw = self._ser.readline()          # blocks up to `timeout`
            if not raw:
                continue                        # timeout tick, not EOF
            yield raw.decode('utf-8', errors='replace').strip()

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self._ser.close()


class UdpTransport(Transport):
    name = 'udp'

    def __init__(self, port: int = 4100, tag_ip: str = '255.255.255.255',
                 tag_port: int = 4100):
        self.port = port
        self.tag_ip = tag_ip
        self.tag_port = tag_port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('', port))
        self._sock.settimeout(1.0)

    def read_lines(self) -> Iterator[str]:
        while True:
            try:
                data, _addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            # Normally one line per datagram, but never assume it: the master tag
            # forwards the slave's lines too, so split defensively.
            for line in data.decode('utf-8', errors='replace').splitlines():
                line = line.strip()
                if line:
                    yield line

    def write(self, data: bytes) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.sendto(data, (self.tag_ip, self.tag_port))

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self._sock.close()


# ── Subscriptions ─────────────────────────────────────────────────────────────

class Subscription:
    """Lines matching any of `prefixes`, diverted away from the pipeline."""

    def __init__(self, link: "TagLink", prefixes: tuple[str, ...]):
        self.link = link
        self.prefixes = prefixes
        self.queue: asyncio.Queue[str] = asyncio.Queue()

    def matches(self, line: str) -> bool:
        return any(line.startswith(p) for p in self.prefixes)

    async def get(self, timeout: float | None = None) -> str | None:
        """Next matching line, or None on timeout."""
        if timeout is None:
            return await self.queue.get()
        try:
            return await asyncio.wait_for(self.queue.get(), timeout)
        except asyncio.TimeoutError:
            return None

    def close(self) -> None:
        self.link._unsubscribe(self)


# ── The link ──────────────────────────────────────────────────────────────────

class TagLink:
    """Owns one transport: a reader thread, line routing, and the write path."""

    # Bounded so a stalled event loop cannot grow the queue without limit. At
    # ~45 sweeps/s this is several seconds of slack; past that the run is
    # already compromised and the honest thing is to drop and report.
    QUEUE_MAX = 512

    def __init__(self, transport: Transport):
        self.transport = transport
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._queue: Optional[asyncio.Queue] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._subs: list[Subscription] = []
        self._exclusive: Optional[asyncio.Queue] = None
        self._on_line: Optional[Callable] = None
        self._pump_task: Optional[asyncio.Task] = None

        # Counters, surfaced in DIAG
        self.n_lines = 0
        self.n_dropped = 0
        self.last_line_ts = 0.0

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self, on_line: Callable[[str], "asyncio.Future | None"]) -> None:
        """Begin reading. `on_line` is an async callable for unclaimed lines."""
        self._loop = asyncio.get_running_loop()
        self._queue = asyncio.Queue(maxsize=self.QUEUE_MAX)
        self._on_line = on_line
        self._thread = threading.Thread(target=self._read_thread,
                                        name=f'ingest-{self.transport.name}',
                                        daemon=True)
        self._thread.start()
        self._pump_task = asyncio.create_task(self._pump())

    def _read_thread(self) -> None:
        """Blocking reads, off the loop entirely.

        The receive time is taken HERE, not where the line is later processed:
        anything queued behind a slow handler would otherwise be stamped late,
        and that error goes straight into the filter's dt.
        """
        assert self._loop is not None and self._queue is not None
        loop, queue = self._loop, self._queue
        try:
            for line in self.transport.read_lines():
                if self._stop.is_set():
                    break
                loop.call_soon_threadsafe(self._offer, queue, (time.time(), line))
        except Exception as e:                        # pragma: no cover
            print(f"[INGEST] {self.transport.name} reader stopped: {e}")

    def _offer(self, queue: asyncio.Queue, item) -> None:
        """Runs on the loop. Drops the oldest line when the queue is full."""
        self.n_lines += 1
        self.last_line_ts = item[0]
        try:
            queue.put_nowait(item)
        except asyncio.QueueFull:
            with contextlib.suppress(asyncio.QueueEmpty):
                queue.get_nowait()
            self.n_dropped += 1
            with contextlib.suppress(asyncio.QueueFull):
                queue.put_nowait(item)

    async def _pump(self) -> None:
        """Route each line: exclusive holder, then subscribers, then pipeline."""
        assert self._queue is not None
        while True:
            t_rx, line = await self._queue.get()
            try:
                if self._exclusive is not None:
                    self._exclusive.put_nowait(line)
                    continue
                claimed = False
                for sub in list(self._subs):
                    if sub.matches(line):
                        sub.queue.put_nowait(line)
                        claimed = True
                        break
                if not claimed and self._on_line is not None:
                    await self._on_line(line, t_rx)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f"[INGEST] line dispatch error: {e}")

    async def stop(self) -> None:
        self._stop.set()
        if self._pump_task is not None:
            self._pump_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._pump_task
        self.transport.close()

    # ── sending ──────────────────────────────────────────────────────────────

    def send(self, cmd: str) -> None:
        """Write a command to the tag. Newline is the caller's responsibility."""
        self.transport.write(cmd.encode())

    # ── shared access ────────────────────────────────────────────────────────

    def subscribe(self, *prefixes: str) -> Subscription:
        """Divert matching lines to a private queue; everything else flows on."""
        sub = Subscription(self, prefixes)
        self._subs.append(sub)
        return sub

    def _unsubscribe(self, sub: Subscription) -> None:
        with contextlib.suppress(ValueError):
            self._subs.remove(sub)

    @contextlib.asynccontextmanager
    async def exclusive(self):
        """Take the whole link. Only for operations that stop the tag sweeping.

        Yields a queue of every line received while held. Normal dispatch —
        including the live pipeline — is suspended until the block exits.
        """
        if self._exclusive is not None:
            raise RuntimeError("tag link already held exclusively")
        q: asyncio.Queue[str] = asyncio.Queue()
        self._exclusive = q
        try:
            yield q
        finally:
            self._exclusive = None

    # ── diagnostics ──────────────────────────────────────────────────────────

    def stats(self) -> dict:
        return {
            'transport':  self.transport.name,
            'n_lines':    self.n_lines,
            'n_dropped':  self.n_dropped,
            'age_s':      round(time.time() - self.last_line_ts, 3)
                          if self.last_line_ts else None,
            'subs':       len(self._subs),
            'exclusive':  self._exclusive is not None,
        }
