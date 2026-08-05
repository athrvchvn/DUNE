"""Built-in jobs that exercise the job machinery itself.

`selftest` is not a toy: it is the fixture the GUI and the test suite both use to
check that a long operation reports progress, can be cancelled, and does not
stall ingest. It deliberately burns CPU in pure Python (the worst case for the
GIL) rather than sleeping, because a sleeping job would prove nothing.
"""

from __future__ import annotations

import time

from .jobs import job


@job('selftest', 'Self test (job machinery)', mode='lab', args_schema={
    'seconds': {'t': 'number', 'default': 5.0, 'min': 0.5, 'max': 120.0},
    'busy':    {'t': 'bool',   'default': True},
})
def selftest(app, ctx, seconds: float = 5.0, busy: bool = True) -> dict:
    """Count to `seconds`, reporting progress once per step."""
    steps = max(1, int(seconds))
    t0 = time.time()
    for i in range(steps):
        ctx.check_cancel()
        deadline = t0 + (i + 1) * (seconds / steps)
        if busy:
            # Pure-Python spin: holds the GIL in ~5 ms slices, which is exactly
            # the contention the ingest threads have to survive.
            x = 0
            while time.time() < deadline:
                x += 1
        else:
            time.sleep(max(0.0, deadline - time.time()))
        ctx.progress((i + 1) / steps, msg=f"step {i + 1}/{steps}",
                     step=i + 1, steps=steps)
        ctx.log(f"completed step {i + 1}")
    return {'steps': steps, 'seconds': round(time.time() - t0, 3), 'busy': busy}
