"""Server-side gamepad. The stick is never in the browser.

Three independent reasons this runs here and not in the page, each sufficient on
its own:

* the browser Gamepad API only polls while the tab is **focused and visible**, so
  a backgrounded tab freezes the stick *while the rover is moving*, with no
  error anywhere;
* the WebSocket adds a variable 5-50 ms and can queue behind a broadcast burst;
* if the browser crashes, a server-side pad can still stop the rover.

Mapping is ported from `tests/system_tests/test_ps4_controller.m`, minus one for
zero-indexing. It is a *table*, not hard-coded: pad axis codes differ between
kernel drivers and DS4 firmware revisions, so a wrong guess would otherwise mean
editing code at the bench.

The stop button does **not** go through the intent path. It sets the controller's
stop directly, because the whole point of a physical stop is that it does not
depend on the loop being healthy.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

# From test_ps4_controller.m: MAX_LINEAR_VELOCITY / MAX_ANGULAR_VELOCITY.
PAD_V_MAX = 0.08           # m/s
PAD_OMEGA_MAX = 0.20       # rad/s
DEADZONE = 0.10
RAMP_PER_S = 0.025 / 0.02  # MATLAB ramps 0.025 per 20 ms tick
POLL_HZ = 50.0


@dataclass
class PadMapping:
    """evdev codes. Overridable because they vary by driver and firmware."""
    axis_v: str = 'ABS_Y'        # left stick Y, negated below
    axis_omega: str = 'ABS_RX'   # right stick X
    invert_v: bool = True
    btn_stop: str = 'BTN_EAST'   # Circle
    btn_center: str = 'BTN_NORTH'  # Triangle
    btn_disarm: str = 'BTN_MODE'   # PS
    btn_pivot: str = 'BTN_TR2'     # R2, hold for pivot


@dataclass
class PadState:
    present: bool = False
    v: float = 0.0
    omega: float = 0.0
    raw_v: float = 0.0
    raw_omega: float = 0.0
    buttons: dict = field(default_factory=dict)
    device: Optional[str] = None
    n_polls: int = 0

    def to_dict(self) -> dict:
        return {'present': self.present, 'v': round(self.v, 4),
                'omega': round(self.omega, 4),
                'raw_v': round(self.raw_v, 3), 'raw_omega': round(self.raw_omega, 3),
                'buttons': {k: bool(v) for k, v in self.buttons.items() if v},
                'device': self.device, 'polls': self.n_polls}


def _deadzone(x: float) -> float:
    """Rescale outside the deadzone so the stick does not jump at the edge."""
    if abs(x) < DEADZONE:
        return 0.0
    return (abs(x) - DEADZONE) / (1.0 - DEADZONE) * (1.0 if x > 0 else -1.0)


def _ramp(current: float, target: float, limit: float) -> float:
    """Move toward `target` by at most `limit`. A step command jerks the rig."""
    if target > current:
        return min(target, current + limit)
    return max(target, current - limit)


class EvdevBackend:
    """Reads a pad through /dev/input. Returns normalised axes in [-1, 1]."""

    def __init__(self, mapping: PadMapping, path: str | None = None):
        from evdev import InputDevice, ecodes, list_devices

        self.ecodes = ecodes
        self.mapping = mapping
        paths = [path] if path else list_devices()
        self.dev = None
        for p in paths:
            try:
                d = InputDevice(p)
            except Exception:
                continue
            caps = d.capabilities().get(ecodes.EV_ABS, [])
            codes = {c for c, _ in caps} if caps else set()
            if getattr(ecodes, mapping.axis_v, None) in codes:
                self.dev = d
                break
            d.close()
        if self.dev is None:
            raise RuntimeError("no gamepad found on /dev/input")

        self.name = self.dev.name
        self._range = {}
        for code, info in self.dev.capabilities().get(ecodes.EV_ABS, []):
            self._range[code] = (info.min, info.max)

    def read(self) -> tuple[float, float, dict]:
        """Current axis values and button states, from the device's own state."""
        ec = self.ecodes
        m = self.mapping

        def axis(name: str) -> float:
            code = getattr(ec, name, None)
            if code is None:
                return 0.0
            try:
                raw = self.dev.absinfo(code).value
            except Exception:
                return 0.0
            lo, hi = self._range.get(code, (-32768, 32767))
            mid = (lo + hi) / 2.0
            span = max((hi - lo) / 2.0, 1.0)
            return max(-1.0, min(1.0, (raw - mid) / span))

        active = set(self.dev.active_keys())
        buttons = {}
        for label in ('btn_stop', 'btn_center', 'btn_disarm', 'btn_pivot'):
            code = getattr(ec, getattr(m, label), None)
            buttons[label] = code is not None and code in active

        v = axis(m.axis_v)
        if m.invert_v:
            v = -v
        return v, axis(m.axis_omega), buttons

    def close(self) -> None:
        try:
            self.dev.close()
        except Exception:
            pass


class PadThread:
    """Polls a pad at 50 Hz and feeds the controller's intent."""

    def __init__(self, controller, backend, *, poll_hz: float = POLL_HZ,
                 on_state: Callable[[PadState], None] | None = None):
        self.controller = controller
        self.backend = backend
        self.period = 1.0 / poll_hz
        self.on_state = on_state
        self.state = PadState(present=True,
                              device=getattr(backend, 'name', None))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._prev_buttons: dict = {}
        # Latched by the stop button. Cleared only once the sticks are back at
        # centre, so releasing stop with the stick still pushed cannot resume at
        # full deflection — the operator has to physically neutralise first.
        self._latched = False

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='rover-pad',
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.state.present = False
        try:
            self.backend.close()
        except Exception:
            pass

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                self._poll()
                time.sleep(self.period)
        finally:
            # A dead pad thread must not leave a stale intent alive. The
            # controller's dead-man would catch it within 300 ms anyway; this
            # makes it immediate.
            self.state.present = False
            try:
                self.controller.set_intent(0.0, 0.0)
            except Exception:
                pass

    def _poll(self) -> None:
        try:
            raw_v, raw_omega, buttons = self.backend.read()
        except Exception:
            # A disconnected pad stops being a source of intent; the dead-man
            # then stops the rover. Do not raise — this thread must not die
            # silently and leave the last intent standing.
            self.state.present = False
            return

        self.state.present = True
        self.state.raw_v, self.state.raw_omega = raw_v, raw_omega
        self.state.buttons = buttons
        self.state.n_polls += 1

        self._edges(buttons)

        target_v = _deadzone(raw_v) * PAD_V_MAX
        target_omega = _deadzone(raw_omega) * PAD_OMEGA_MAX
        if buttons.get('btn_pivot'):
            target_v = 0.0        # R2 held: pivot in place, as MATLAB does

        # While latched, command nothing regardless of stick position. Without
        # this the next poll simply recomputes the target from the still-pushed
        # stick and ramps straight back up, so the stop button would stop the
        # rover and then immediately ask it to move again.
        if self._latched:
            if buttons.get('btn_stop') or target_v or target_omega:
                self.state.v = self.state.omega = 0.0
                self._report()
                return
            self._latched = False        # released AND sticks centred

        limit = RAMP_PER_S * self.period
        self.state.v = _ramp(self.state.v, target_v, limit)
        self.state.omega = _ramp(self.state.omega, target_omega, limit)

        # Only express intent while there IS intent: a centred stick must let
        # the dead-man expire rather than holding the rover armed at zero.
        if self.state.v or self.state.omega:
            self.controller.set_intent(self.state.v, self.state.omega)

        self._report()

    def _report(self) -> None:

        if self.on_state is not None:
            try:
                self.on_state(self.state)
            except Exception:
                pass

    def _edges(self, buttons: dict) -> None:
        """Act on press edges, not levels, so a held button fires once."""
        def pressed(name: str) -> bool:
            was = self._prev_buttons.get(name, False)
            self._prev_buttons[name] = buttons.get(name, False)
            return buttons.get(name, False) and not was

        if pressed('btn_stop'):
            # Straight to the stop, NOT through set_intent: a physical stop must
            # not depend on the control loop being healthy.
            self.state.v = self.state.omega = 0.0
            self._latched = True
            self.controller.emergency_stop('pad stop button')
        if pressed('btn_disarm'):
            self.state.v = self.state.omega = 0.0
            self.controller.disarm()
        if pressed('btn_center'):
            try:
                self.controller.link.center_wheels()
            except Exception:
                pass


def open_pad(controller, *, mapping: PadMapping | None = None,
             path: str | None = None, **kw) -> PadThread:
    """Find a pad and start feeding the controller. Raises if none is present."""
    backend = EvdevBackend(mapping or PadMapping(), path=path)
    pad = PadThread(controller, backend, **kw)
    pad.start()
    return pad
