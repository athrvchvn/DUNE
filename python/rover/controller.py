"""The 10 Hz control loop, and every reason the rover stops.

**The governing fact: the firmware has no command watchdog.** If this host dies,
the motors keep running at whatever they were last told. So the stop logic is
entirely here, and it is built to fail closed.

The dead-man is the core of it. Each tick asks a single question — *has someone
expressed intent recently?* — and drives zero if not:

    if not (armed and now - last_intent < INTENT_TIMEOUT): v = omega = 0

A hung pad thread, a closed browser, a stalled event loop and a wedged solver
all converge on that same condition without any of them having to know the
others exist. That is why it is one check rather than four handlers.

Rate budget: each velocity tick costs up to five round-trips (four steering, one
motor), which caps the loop near 10 Hz. Steering is therefore only re-sent when a
wheel actually moves more than `ANGLE_EPSILON_DEG`, and telemetry runs at 3 Hz and
is skipped whenever the loop is behind and the rover is moving. **Control always
wins over telemetry** — a late stop is worse than a stale IMU reading.

None of this is safety-rated. A software stop over a 115200 link at ~10 Hz is
not an emergency stop; the only true one is an inline power switch.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .ackermann import DriveCommand, calculate_from_velocity

CONTROL_HZ = 10.0
INTENT_TIMEOUT_S = 0.30    # no fresh intent for this long -> drive zero
ARM_LEASE_S = 60.0         # arming expires; it is not a mode you can forget
TELEMETRY_HZ = 3.0
ANGLE_EPSILON_DEG = 0.5    # re-send a steering angle only if it moved this much

# Ceilings, from rover_teleop_uwb.m's CONFIGURATION block. Enforced here rather
# than trusted from the caller: a browser-supplied number is not a limit.
V_MAX = 0.10               # m/s
OMEGA_MAX = 0.107          # rad/s


@dataclass
class RoverState:
    armed: bool = False
    moving: bool = False
    v_cmd: float = 0.0
    omega_cmd: float = 0.0
    last_intent_age: float = float('inf')
    arm_expires_in: float = 0.0
    mode: str = 'straight'
    loop_hz: float = 0.0
    n_ticks: int = 0
    n_deadman_stops: int = 0
    imu: dict = field(default_factory=dict)
    motors: dict = field(default_factory=dict)
    link: dict = field(default_factory=dict)
    fault: Optional[str] = None

    def to_dict(self) -> dict:
        return {'armed': self.armed, 'moving': self.moving,
                'v': round(self.v_cmd, 4), 'omega': round(self.omega_cmd, 4),
                'intent_age_s': (None if self.last_intent_age == float('inf')
                                 else round(self.last_intent_age, 3)),
                'arm_expires_in_s': round(self.arm_expires_in, 1),
                'mode': self.mode, 'loop_hz': round(self.loop_hz, 1),
                'ticks': self.n_ticks, 'deadman_stops': self.n_deadman_stops,
                'imu': self.imu, 'motors': self.motors, 'link': self.link,
                'fault': self.fault}


class RoverController:
    """Owns the link. One thread, fixed rate, stop-by-default."""

    def __init__(self, link, *, control_hz: float = CONTROL_HZ,
                 on_state: Callable[[RoverState], None] | None = None):
        self.link = link
        self.period = 1.0 / control_hz
        self.on_state = on_state

        self._lock = threading.Lock()
        self._armed_until = 0.0
        self._intent = (0.0, 0.0)
        self._intent_ts = 0.0
        self._stop_flag = threading.Event()
        # Set while a job drives the link directly (wheels_test). The loop keeps
        # running — so the dead-man and the arm lease still tick — but it stops
        # WRITING, because otherwise its zero-speed command overwrites the job's
        # within one tick and the job silently does nothing.
        self._paused = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_angles = [None] * 4
        self._last_telemetry = 0.0
        self.state = RoverState()

    # ── arming ───────────────────────────────────────────────────────────────

    def arm(self, lease_s: float = ARM_LEASE_S) -> float:
        """Arm for a bounded time. Expiry is deliberate — see the module note."""
        with self._lock:
            self._armed_until = time.monotonic() + lease_s
        return lease_s

    def disarm(self) -> None:
        with self._lock:
            self._armed_until = 0.0
            self._intent = (0.0, 0.0)

    @property
    def armed(self) -> bool:
        return time.monotonic() < self._armed_until

    # ── intent ───────────────────────────────────────────────────────────────

    def set_intent(self, v: float, omega: float) -> None:
        """Latest desired velocity. Clamped here, not trusted from the caller."""
        v = max(-V_MAX, min(V_MAX, float(v)))
        omega = max(-OMEGA_MAX, min(OMEGA_MAX, float(omega)))
        with self._lock:
            self._intent = (v, omega)
            self._intent_ts = time.monotonic()

    def nudge(self, v: float, omega: float, duration_s: float) -> None:
        """A bounded move that expires in the SERVER, not the browser.

        This is what makes browser-initiated motion safe under arbitrary
        latency: the command carries its own end, so a connection that dies
        mid-nudge still stops on schedule.
        """
        duration_s = max(0.0, min(1.0, float(duration_s)))
        self.set_intent(v, omega)
        with self._lock:
            # Backdate the intent so the dead-man expires it after duration_s
            # rather than after its own timeout.
            self._intent_ts = time.monotonic() - (INTENT_TIMEOUT_S - duration_s)

    # ── yielding the link ────────────────────────────────────────────────────

    def pause(self) -> None:
        """Stop writing to the link, and disarm so nothing resumes behind it."""
        self.disarm()
        self._paused.set()

    def resume(self) -> None:
        self._paused.clear()

    @property
    def paused(self) -> bool:
        return self._paused.is_set()

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        self._stop_flag.clear()
        self._thread = threading.Thread(target=self._run, name='rover-control',
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the loop AND the rover. Safe to call more than once."""
        self._stop_flag.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.disarm()
        self.link.full_stop()

    def emergency_stop(self, reason: str = 'stop') -> None:
        """Immediate, from any thread. Disarms so it cannot resume by itself."""
        self.disarm()
        try:
            self.link.emergency_stop()
        except Exception:
            pass
        self.link.full_stop()
        self.state.fault = reason

    def on_client_gone(self, n_clients: int) -> None:
        """No browser attached means nobody is watching. Stand down.

        A page refresh therefore stops the rover. That is correct, and worth
        documenting rather than working around.
        """
        if n_clients == 0:
            self.emergency_stop('no clients connected')

    # ── the loop ─────────────────────────────────────────────────────────────

    def _run(self) -> None:
        next_tick = time.monotonic()
        try:
            while not self._stop_flag.is_set():
                now = time.monotonic()
                self._tick(now)
                next_tick += self.period
                behind = now - next_tick
                if behind > 0:
                    next_tick = now          # do not spiral after a slow tick
                time.sleep(max(0.0, next_tick - time.monotonic()))
        finally:
            # The loop owns the motors; if it exits for ANY reason they stop.
            self.link.full_stop()

    def _tick(self, now: float) -> None:
        with self._lock:
            v, omega = self._intent
            age = now - self._intent_ts if self._intent_ts else float('inf')
            armed_until = self._armed_until

        alive = bool(armed_until > now) and age < INTENT_TIMEOUT_S
        if not alive:
            if v or omega:
                self.state.n_deadman_stops += 1
            v = omega = 0.0

        if self._paused.is_set():
            # A job owns the link. Keep the state fresh so the GUI still shows
            # the truth, but write nothing.
            self.state.armed = False
            self.state.moving = False
            self.state.mode = 'paused'
            self.state.n_ticks += 1
            return

        cmd = calculate_from_velocity(v, omega)
        try:
            self._apply(cmd, moving=bool(v or omega))
            self.state.fault = None
        except Exception as e:
            # A link error while moving is the dangerous case: stop, then report.
            self.state.fault = f"{type(e).__name__}: {e}"
            self.link.full_stop()

        st = self.state
        st.armed = armed_until > now
        st.moving = bool(v or omega)
        st.v_cmd, st.omega_cmd = v, omega
        st.last_intent_age = age
        st.arm_expires_in = max(0.0, armed_until - now)
        st.mode = cmd.mode
        st.n_ticks += 1
        st.loop_hz = 1.0 / self.period
        st.link = self.link.stats.to_dict() if hasattr(self.link, 'stats') else {}

        self._telemetry(now, moving=st.moving)
        if self.on_state is not None:
            try:
                self.on_state(st)
            except Exception:
                pass

    def _apply(self, cmd: DriveCommand, *, moving: bool) -> None:
        """Four steering commands and one motor command, minus what is unchanged."""
        for i, angle in enumerate(cmd.wheel_angles):
            prev = self._last_angles[i]
            if prev is None or abs(angle - prev) >= ANGLE_EPSILON_DEG:
                self.link.set_wheel_angle(i + 1, angle)
                self._last_angles[i] = angle
        # Always sent, every tick: this is the command that actually moves the
        # rover, and the one whose absence must mean "stopped".
        self.link.set_all_motor_speeds_with_dir(cmd.motor_speeds, cmd.dirs)

    def _telemetry(self, now: float, *, moving: bool) -> None:
        if now - self._last_telemetry < 1.0 / TELEMETRY_HZ:
            return
        if moving and (time.monotonic() - now) > self.period * 0.5:
            return          # behind schedule and moving: control wins
        self._last_telemetry = now
        try:
            self.state.imu = self.link.get_imu()
            self.state.motors = self.link.get_motor_speeds()
        except Exception:
            pass            # telemetry is never worth failing a control tick
