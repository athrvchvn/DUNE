"""Calibration and survey jobs that talk to the tag.

These are the first jobs that touch hardware, so two things about how they use
the link matter more than the code itself.

**Most of them run WITHOUT stopping live positioning.** `TagLink.subscribe`
diverts only the reply family a job is waiting for; every other line keeps
flowing to the pipeline. So delay calibration happens while the map is still
updating, which is what makes it usable — you can see the effect of a push
immediately, rather than calibrating blind and then restarting to look.

**`survey_capture` is the exception** and takes the link exclusively, because
the firmware's SURVEY genuinely stops the tag sweeping: there is nothing left to
keep flowing. That is a property of the firmware, not a convenience.

Every job here can move hardware state — antenna delays live in each anchor's
NVS — so each one reports what it will change before it changes it, and the
destructive ones are opt-in.
"""

from __future__ import annotations

import asyncio
import re
from datetime import datetime

from .jobs import job
from .paths import RESULTS_DIR

# DW1000 antenna-delay tick in metres, from survey_fit.m.
TICK_M = 0.00469

# The factory nominal each anchor ships with; anchor_delay_tool.m's default.
NOMINAL_TICKS = 16434


def _link(app):
    link = getattr(app, 'link', None)
    if link is None:
        raise RuntimeError("no tag link — start the host with --serial or --udp")
    return link


async def _await_line(sub, prefixes, timeout: float):
    """Next subscribed line starting with any of `prefixes`, or None."""
    deadline = asyncio.get_running_loop().time() + timeout
    while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            return None
        line = await sub.get(timeout=remaining)
        if line is None:
            return None
        if line.startswith(tuple(prefixes)):
            return line


# Every job here writes commands to the one tag link, so they must not overlap:
# two in flight would interleave their commands and each would read the other's
# ACKs. The resource table refuses the second rather than queueing it, which is
# the right call when the queued job would later move antenna delays.
@job('anchor_delay_push', 'Push antenna delays to anchors', mode='cal',
     requires=('tag_link',),
     args_schema={
         'ids':     {'t': 'string', 'default': '', },
         'ticks':   {'t': 'number', 'default': float(NOMINAL_TICKS),
                     'min': 0, 'max': 65535},
         'timeout': {'t': 'number', 'default': 10.0, 'min': 1.0, 'max': 60.0},
         'apply':   {'t': 'bool', 'default': False},
     })
def anchor_delay_push(app, ctx, ids: str = '', ticks: float = NOMINAL_TICKS,
                      timeout: float = 10.0, apply: bool = False) -> dict:
    """Write `ticks` into each listed anchor's NVS over UWB.

    Port of `anchor_delay_tool.m`. **`apply` is off by default**: this changes
    persistent state inside hardware that is not in front of you, and an
    accidental click should not silently recalibrate the rig. With it off the
    job reports exactly what it would send and stops.

    Runs via `subscribe`, so positioning keeps updating throughout.
    """
    target = [int(v) for v in re.findall(r'\d+', ids)] or [1, 2, 3, 4, 5]
    ticks = int(round(ticks))

    plan = {'ids': target, 'ticks': ticks,
            'implied_range_shift_mm': round(ticks * TICK_M * 1000, 1)}
    if not apply:
        ctx.log(f"DRY RUN — would send SETANTDELAY,<id>,{ticks} to {target}")
        ctx.progress(1.0, msg="dry run")
        return dict(plan, applied=False, results=[])

    return ctx.run_coro(_push_delays(app, ctx, target, ticks, timeout, plan))


async def _push_delays(app, ctx, target, ticks, timeout, plan) -> dict:
    link = _link(app)
    sub = link.subscribe('ANTDELAY_ACK', 'ANTDELAY_FAIL')
    results = []
    try:
        for n, aid in enumerate(target):
            ctx.check_cancel()
            ok = False
            for attempt in (1, 2):          # one retry, as MATLAB does
                ctx.log(f"SETANTDELAY,{aid},{ticks} (attempt {attempt})")
                link.send(f"SETANTDELAY,{aid},{ticks}\n")
                line = await _await_line(sub, (f'ANTDELAY_ACK,{aid},',
                                               f'ANTDELAY_FAIL,{aid},'), timeout)
                if line and line.startswith('ANTDELAY_ACK'):
                    ok = True
                    break
                ctx.log(f"anchor {aid}: {'no reply' if line is None else line}",
                        level='warn')
            results.append({'id': aid, 'ok': ok})
            ctx.progress((n + 1) / len(target), msg=f"anchor {aid}")
    finally:
        sub.close()

    n_ok = sum(r['ok'] for r in results)
    ctx.log(f"{n_ok}/{len(results)} anchors acknowledged")
    return dict(plan, applied=True, results=results, n_ok=n_ok)


@job('tag_delay', 'Read or set the tag antenna delay', mode='cal',
     requires=('tag_link',),
     args_schema={
         'tagId':   {'t': 'number', 'default': 240, 'min': 0, 'max': 65535},
         'ticks':   {'t': 'number', 'default': 0, 'min': 0, 'max': 65535},
         'timeout': {'t': 'number', 'default': 10.0, 'min': 1.0, 'max': 60.0},
         'apply':   {'t': 'bool', 'default': False},
     })
def tag_delay(app, ctx, tagId: float = 240, ticks: float = 0,
              timeout: float = 10.0, apply: bool = False) -> dict:
    """`GETMYDELAY` always; `SETMYDELAY` only when `apply` and ticks > 0."""
    return ctx.run_coro(_tag_delay(app, ctx, int(tagId), int(round(ticks)),
                                   timeout, apply))


async def _tag_delay(app, ctx, tag_id, ticks, timeout, apply) -> dict:
    link = _link(app)
    sub = link.subscribe('MYDELAY_ACK', 'MYDELAY_FAIL')
    out = {'tag_id': tag_id, 'set': False}
    try:
        if apply and ticks > 0:
            ctx.log(f"SETMYDELAY,{tag_id},{ticks}")
            link.send(f"SETMYDELAY,{tag_id},{ticks}\n")
            line = await _await_line(sub, ('MYDELAY_ACK', 'MYDELAY_FAIL'), timeout)
            out['set'] = bool(line and line.startswith('MYDELAY_ACK'))
            out['set_reply'] = line

        ctx.log(f"GETMYDELAY,{tag_id}")
        link.send(f"GETMYDELAY,{tag_id}\n")
        line = await _await_line(sub, ('MYDELAY_ACK', 'MYDELAY_FAIL'), timeout)
        out['reply'] = line
        m = re.search(r'MYDELAY_ACK,\s*(\d+)\s*,\s*(\d+)', line or '')
        if m:
            out['ticks'] = int(m.group(2))
        ctx.progress(1.0, msg=line or 'no reply')
    finally:
        sub.close()
    return out


@job('survey_capture', 'Run the anchor self-survey', mode='cal',
     requires=('tag_link',),
     args_schema={
         'timeout': {'t': 'number', 'default': 120.0, 'min': 10.0, 'max': 600.0},
         'stall':   {'t': 'number', 'default': 20.0, 'min': 1.0, 'max': 120.0},
     })
def survey_capture(app, ctx, timeout: float = 120.0, stall: float = 20.0) -> dict:
    """Trigger SURVEY and save the pairwise ranges.

    Takes the link EXCLUSIVELY: the firmware's SURVEY stops the tag sweeping, so
    there is no live stream to preserve. `stall` gives up when no new SURVEY line
    has arrived for that long, which is the failure mode when an anchor is off.
    """
    return ctx.run_coro(_survey_capture(app, ctx, timeout, stall))


async def _survey_capture(app, ctx, timeout, stall) -> dict:
    link = _link(app)
    loop = asyncio.get_running_loop()
    lines: list[str] = []
    expected = None

    async with link.exclusive() as q:
        while not q.empty():                 # flush anything stale
            q.get_nowait()
        ctx.log("SURVEY")
        link.send("SURVEY\n")

        t_end = loop.time() + timeout
        t_last = loop.time()
        while loop.time() < t_end:
            ctx.check_cancel()
            try:
                line = await asyncio.wait_for(q.get(), timeout=1.0)
            except asyncio.TimeoutError:
                if loop.time() - t_last > stall:
                    ctx.log(f"no SURVEY line for {stall:.0f}s — giving up",
                            level='warn')
                    break
                continue
            if line.startswith('SURVEY_BEGIN'):
                lines.append(line)
                t_last = loop.time()
                parts = line.split(',')
                if len(parts) >= 3 and parts[2].strip().isdigit():
                    expected = int(parts[2])
            elif line.startswith('SURVEY,'):
                lines.append(line)
                t_last = loop.time()
                n = sum(1 for x in lines if x.startswith('SURVEY,'))
                ctx.progress(min(n / expected, 0.99) if expected else None,
                             msg=f"{n}/{expected or '?'} pairs")
            elif line.startswith('SURVEY_DONE'):
                lines.append(line)
                break

    n_pairs = sum(1 for x in lines if x.startswith('SURVEY,'))
    if not n_pairs:
        raise RuntimeError("no SURVEY lines received — is the tag connected "
                           "and are all anchors powered?")

    out_dir = RESULTS_DIR / 'survey_runs'
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"survey_{datetime.now():%Y%m%d_%H%M%S}.txt"
    out.write_text('\n'.join(lines) + '\n')
    ctx.artifact(out, 'txt')
    ctx.progress(1.0, msg=f"{n_pairs} pairs")

    complete = expected is not None and n_pairs >= expected
    if not complete:
        ctx.log(f"INCOMPLETE: {n_pairs} of {expected} pairs — reconstruction "
                f"needs every pair", level='warn')
    return {'file': str(out), 'n_pairs': n_pairs, 'expected': expected,
            'complete': bool(complete)}
