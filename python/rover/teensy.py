"""Newline-delimited JSON link to the rover's Teensy 4.1.

Reimplemented from `firmware/teensy_control_rebuild/teensy_control_rebuild.ino`
(the dispatcher at lines 1050-1109 is the authoritative command list) and from
`matlab/hardware/RoverControl.m`, which is the only MATLAB code that touches the
port.

Transport: 115200 8N1, one JSON object per line out, one per line back.

Four things about this link are not obvious and each will bite:

1. **Unsolicited lines.** The firmware emits `{"event":...}` on its own schedule,
   and `updateArmCommunication()` relays the ESP32's replies to the USB port
   whenever they arrive. MATLAB copes by skipping up to ten lines that do not
   start with `{`, but never checks the reply belongs to the command it sent —
   so a well-timed event is silently accepted as a response. Here every reply is
   matched on `cmd`, and events go to their own queue.

2. **`emergencyStop` answers with an EVENT, not a status.** A naive
   request/response wait therefore times out on the one command that must never
   time out. It is special-cased.

3. **Boot takes over 4 s** and emits ~10 event lines. Opening the port and
   talking immediately gets you those events as your "reply".

4. **There is no watchdog in the firmware.** Nothing here can make the rover safe
   on its own; see `rover/__init__.py`.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

BAUD = 115200
BOOT_WAIT_S = 5.0          # firmware boot is >4 s and emits ~10 event lines
DEFAULT_TIMEOUT = 0.5

# Commands whose reply is an event rather than a `status` object.
EVENT_REPLY_COMMANDS = frozenset({'emergencyStop'})


class TeensyError(RuntimeError):
    """The firmware answered with status:"error", or did not answer."""


@dataclass
class LinkStats:
    sent: int = 0
    replies: int = 0
    events: int = 0
    mismatched: int = 0        # a reply whose cmd was not what we asked for
    timeouts: int = 0
    errors: int = 0

    def to_dict(self) -> dict:
        return dict(sent=self.sent, replies=self.replies, events=self.events,
                    mismatched=self.mismatched, timeouts=self.timeouts,
                    errors=self.errors)


@dataclass
class TeensyLink:
    """Request/response over the rover's USB serial link, with event demux."""

    port: str
    baud: int = BAUD
    timeout: float = DEFAULT_TIMEOUT
    _ser: object = None
    _lock: threading.Lock = field(default_factory=threading.Lock)
    events: list = field(default_factory=list)
    stats: LinkStats = field(default_factory=LinkStats)
    version: Optional[str] = None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def open(self, *, boot_wait: float = BOOT_WAIT_S, serial_factory=None) -> None:
        """Open the port, wait out the boot, drain its events, read the version.

        `serial_factory` exists so the tests can substitute a fake port; nothing
        in production passes it.
        """
        if serial_factory is not None:
            self._ser = serial_factory(self.port, self.baud, self.timeout)
        else:
            import serial
            # dsrdtr/rtscts pinned explicitly: a toggled control line is what
            # resets some boards on open, and the default varies by platform.
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout,
                                      dsrdtr=False, rtscts=False)

        if boot_wait:
            time.sleep(boot_wait)
        self.drain()

        # getVersion twice: the first often lands in the tail of the boot noise.
        for _ in range(2):
            try:
                r = self.request('getVersion', timeout=1.0)
                self.version = str(r.get('data', r.get('version', '')) or '') or None
                if self.version:
                    break
            except TeensyError:
                continue

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def drain(self) -> int:
        """Consume whatever is buffered; boot events land here."""
        n = 0
        while True:
            line = self._readline()
            if not line:
                return n
            self._classify(line)
            n += 1

    # ── low level ────────────────────────────────────────────────────────────

    def _readline(self) -> str:
        raw = self._ser.readline()
        if not raw:
            return ''
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', errors='replace')
        return raw.strip()

    def _classify(self, line: str) -> Optional[dict]:
        """Return a parsed reply, or None when the line was an event/noise."""
        if not line.startswith('{'):
            return None
        try:
            obj = json.loads(line)
        except ValueError:
            return None
        if 'event' in obj:
            self.stats.events += 1
            self.events.append(obj)
            return None
        return obj

    def send(self, cmd: str, **args) -> None:
        """Fire and forget. Used by the stop paths, where waiting is wrong.

        DELIBERATELY UNLOCKED. `request()` holds the lock for its whole
        read-wait, so taking it here would let an in-flight request delay a stop
        by its full timeout — measured at 890 ms, on the one path that must be
        immediate. A single `write()` of a short complete line is not split, so
        two threads cannot interleave *within* a line; the only cost is that
        this command's reply is consumed by the waiting request and counted as
        `mismatched`, which is harmless for a command whose reply we ignore.
        """
        payload = json.dumps({'cmd': cmd, **args}) + '\n'
        ser = self._ser
        if ser is None:
            return
        ser.write(payload.encode())
        self.stats.sent += 1

    def request(self, cmd: str, timeout: float | None = None, **args) -> dict:
        """Send and wait for the reply *to this command*.

        Reads until a reply whose `cmd` matches. Events and mismatched replies
        are consumed and counted rather than mistaken for the answer — the
        MATLAB implementation accepts the first JSON line it sees, which a
        well-timed ESP32 relay can satisfy.
        """
        timeout = self.timeout if timeout is None else timeout
        with self._lock:
            self._ser.write((json.dumps({'cmd': cmd, **args}) + '\n').encode())
            self.stats.sent += 1

            # emergencyStop replies with an event, so waiting for a status here
            # would time out on the one command that must always work.
            if cmd in EVENT_REPLY_COMMANDS:
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    line = self._readline()
                    if not line:
                        continue
                    self._classify(line)
                    if self.events and 'emergency' in str(
                            self.events[-1].get('event', '')).lower():
                        break
                return {'cmd': cmd, 'status': 'ok', 'via': 'event'}

            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                line = self._readline()
                if not line:
                    continue
                obj = self._classify(line)
                if obj is None:
                    continue
                if obj.get('cmd') != cmd:
                    self.stats.mismatched += 1
                    continue
                self.stats.replies += 1
                if obj.get('status') == 'error':
                    self.stats.errors += 1
                    raise TeensyError(f"{cmd}: {obj.get('error', 'unknown error')}")
                return obj

        self.stats.timeouts += 1
        raise TeensyError(f"{cmd}: no reply within {timeout:.2f}s")

    # ── the commands the drive path uses ─────────────────────────────────────

    def set_wheel_angle(self, wheel: int, angle: float) -> None:
        """`wheel` is 1..4; the firmware rejects |angle| > 60."""
        self.request('setWheelAngle', wheel=int(wheel), angle=round(float(angle), 2))

    def set_all_motor_speeds_with_dir(self, speeds, dirs) -> dict:
        speeds = [int(s) for s in speeds]
        dirs = [int(d) for d in dirs]
        if len(speeds) != 6 or len(dirs) != 6:
            raise ValueError("the firmware requires exactly 6 speeds and 6 dirs")
        if any(s < 0 or s > 3000 for s in speeds):
            raise ValueError(f"speed out of the firmware's 0..3000 range: {speeds}")
        return self.request('setAllMotorSpeedsWithDir', speeds=speeds, dirs=dirs)

    def stop_all_motors(self) -> None:
        self.send('stopAllMotors')

    def stop_rover(self) -> None:
        self.send('stopRover')

    def center_wheels(self) -> None:
        self.send('centerWheels')

    def emergency_stop(self) -> dict:
        return self.request('emergencyStop', timeout=1.0)

    def get_motor_speeds(self) -> dict:
        return self.request('getAllMotorSpeedsFast').get('data', {})

    def get_imu(self) -> dict:
        """pitch/roll/yaw in DEGREES, plus accel/gyro and an accuracy flag."""
        return self.request('getTeensyIMU').get('data', {})

    # ── safety ───────────────────────────────────────────────────────────────

    def full_stop(self) -> None:
        """Every stop the firmware offers, best-effort and in order.

        Never raises: this runs from `finally` blocks and signal handlers, where
        an exception would replace a stop attempt with a traceback.
        """
        for fn in (self.stop_all_motors, self.stop_rover):
            try:
                fn()
            except Exception:
                pass
