"""Calibration jobs against a fake tag that speaks the real protocol.

The claim worth testing here is not "the bytes are right" — it is the one that
makes these usable at all: **`subscribe` diverts only the reply family, so live
positioning keeps running while you calibrate.** If that were wrong, every
calibration would blind the map for its duration and you would be tuning without
being able to see the effect.

`survey_capture` is the deliberate exception: it takes the link exclusively,
because the firmware's SURVEY stops the tag sweeping anyway.
"""

from __future__ import annotations

import asyncio
import functools
import json
import os
import sys
import threading
import time


sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host.app import App                                  # noqa: E402
from host.ingest import Transport                         # noqa: E402
import host.jobs_calib                                    # noqa: E402,F401
from host.jobs import JOB_KINDS                            # noqa: E402


def runs_async(fn):
    @functools.wraps(fn)
    def wrapper(*a, **kw):
        return asyncio.run(fn(*a, **kw))
    return wrapper


class FakeTag(Transport):
    """A tag that streams sweeps and answers calibration commands.

    Replies are queued from the writer's thread and emitted by the reader, which
    is how a real link behaves — the answer arrives interleaved with the ongoing
    sweep stream, not in a private channel.
    """

    name = 'faketag'

    def __init__(self, *, ack: bool = True, silent: bool = False,
                 survey_pairs: int = 6,
                 sweep_period: float = 0.01):
        self.ack = ack
        # silent = acknowledges NOTHING, which is different from acking a
        # failure: it exercises the timeout path, and it is what a
        # powered-down or out-of-range anchor actually looks like.
        self.silent = silent
        self.survey_pairs = survey_pairs
        self.sweep_period = sweep_period
        self.written: list[str] = []
        self.tag_ticks = 16400          # NVS-persisted, as on a real tag
        self._pending: list[str] = []
        self._lock = threading.Lock()
        self._stop = False
        self.n_sweeps = 0

    def read_lines(self):
        while not self._stop:
            time.sleep(self.sweep_period)
            with self._lock:
                pending, self._pending = self._pending, []
            for line in pending:
                yield line
            self.n_sweeps += 1
            yield (f"RTLS,v3,{self.n_sweeps},240,2,"
                   f"1,3223,-81,-83.9,383.11,2,3767,-98,-102.3,125.77")

    def write(self, data: bytes) -> None:
        cmd = data.decode().strip()
        self.written.append(cmd)
        if self.silent:
            return                      # acknowledges nothing at all
        with self._lock:
            if cmd.startswith('SETANTDELAY'):
                _, aid, ticks = cmd.split(',')
                self._pending.append(
                    f"ANTDELAY_{'ACK' if self.ack else 'FAIL'},{aid},{ticks}")
            elif cmd.startswith('SETMYDELAY'):
                parts = cmd.split(',')
                self.tag_ticks = int(parts[2])
                self._pending.append(f"MYDELAY_ACK,{parts[1]},{self.tag_ticks}")
            elif cmd.startswith('GETMYDELAY'):
                parts = cmd.split(',')
                self._pending.append(f"MYDELAY_ACK,{parts[1]},{self.tag_ticks}")
            elif cmd == 'SURVEY':
                self._pending.append(f"SURVEY_BEGIN,v1,{self.survey_pairs}")
                k = 0
                for i in range(1, 5):
                    for j in range(i + 1, 5):
                        if k >= self.survey_pairs:
                            break
                        self._pending.append(
                            f"SURVEY,v1,{i},{j},{1000 * (i + j)},51")
                        k += 1
                self._pending.append("SURVEY_DONE,v1")

    def close(self) -> None:
        self._stop = True


def make_app(tmp_path, tag: FakeTag, fixes: list):
    app = App(log_path=tmp_path / 's.ndjson')
    app.set_config_msg(lambda: {'type': 'config', 'anchors': []})

    async def on_line(line: str, t_rx: float = 0.0):
        if line.startswith('RTLS,'):
            fixes.append(line)

    app.attach_transport(tag, on_line)
    return app


async def _wait_done(app, timeout=30.0):
    deadline = time.time() + timeout
    terminal = {'job_done', 'job_failed', 'job_cancelled'}
    while time.time() < deadline:
        app.recorder._fh.flush()
        rows = [json.loads(l) for l in open(app.recorder.path) if l.strip()]
        for r in reversed(rows):
            if r['type'] in terminal:
                return r
        await asyncio.sleep(0.05)
    raise AssertionError("job never finished")


# ── the property that makes these usable ──────────────────────────────────────

@runs_async
async def test_positioning_keeps_running_during_a_delay_push(tmp_path):
    """`subscribe` diverts ONLY the ACK family; sweeps keep reaching the app."""
    tag = FakeTag()
    fixes: list[str] = []
    app = make_app(tmp_path, tag, fixes)
    await asyncio.sleep(0.3)

    before = len(fixes)
    await app.jobs.run('anchor_delay_push',
                                {'ids': '1 2 3', 'ticks': 16400, 'apply': True})
    row = await _wait_done(app)
    assert row['type'] == 'job_done', row.get('error')

    during = len(fixes) - before
    # The fake acks instantly, so the job is short — what matters is that sweeps
    # kept arriving throughout it, not how many.
    assert during > 0, "positioning stopped during calibration"
    assert row['result']['n_ok'] == 3
    assert [c for c in tag.written if c.startswith('SETANTDELAY')] == [
        'SETANTDELAY,1,16400', 'SETANTDELAY,2,16400', 'SETANTDELAY,3,16400']

    # And the diverted ACKs did NOT leak into the pipeline.
    assert not any('ANTDELAY' in f for f in fixes)
    await app.aclose()


@runs_async
async def test_a_delay_push_is_a_dry_run_unless_apply_is_set(tmp_path):
    """This writes persistent state into hardware that is not in front of you."""
    tag = FakeTag()
    app = make_app(tmp_path, tag, [])
    await asyncio.sleep(0.2)

    await app.jobs.run('anchor_delay_push', {'ids': '1 2', 'ticks': 16400})
    row = await _wait_done(app)
    assert row['type'] == 'job_done'
    assert row['result']['applied'] is False
    assert not tag.written, f"a dry run sent commands: {tag.written}"
    await app.aclose()


@runs_async
async def test_a_failing_anchor_is_retried_then_reported(tmp_path):
    tag = FakeTag(ack=False)
    app = make_app(tmp_path, tag, [])
    await asyncio.sleep(0.2)

    await app.jobs.run('anchor_delay_push',
                       {'ids': '7', 'ticks': 16400, 'apply': True,
                        'timeout': 1.0})
    row = await _wait_done(app)
    assert row['type'] == 'job_done'
    assert row['result']['n_ok'] == 0
    assert tag.written.count('SETANTDELAY,7,16400') == 2, "no retry"
    await app.aclose()


# ── tag delay ─────────────────────────────────────────────────────────────────

@runs_async
async def test_tag_delay_reads_without_writing_by_default(tmp_path):
    tag = FakeTag()
    app = make_app(tmp_path, tag, [])
    await asyncio.sleep(0.2)

    await app.jobs.run('tag_delay', {'tagId': 240, 'ticks': 16500})
    row = await _wait_done(app)
    assert row['type'] == 'job_done'
    assert row['result']['set'] is False
    assert not any(c.startswith('SETMYDELAY') for c in tag.written)
    assert any(c.startswith('GETMYDELAY') for c in tag.written)
    await app.aclose()


@runs_async
async def test_tag_delay_sets_then_reads_back_when_applied(tmp_path):
    tag = FakeTag()
    app = make_app(tmp_path, tag, [])
    await asyncio.sleep(0.2)

    await app.jobs.run('tag_delay',
                       {'tagId': 240, 'ticks': 16500, 'apply': True})
    row = await _wait_done(app)
    assert row['type'] == 'job_done'
    r = row['result']
    assert r['set'] is True and r.get('ticks') == 16500
    assert 'SETMYDELAY,240,16500' in tag.written
    assert 'GETMYDELAY,240' in tag.written, "did not read back after writing"
    await app.aclose()


# ── survey capture ────────────────────────────────────────────────────────────

@runs_async
async def test_survey_capture_writes_a_reconstructable_file(tmp_path):
    tag = FakeTag(survey_pairs=6)
    fixes: list[str] = []
    app = make_app(tmp_path, tag, fixes)
    await asyncio.sleep(0.2)

    await app.jobs.run('survey_capture', {'timeout': 20.0, 'stall': 5.0})
    row = await _wait_done(app)
    assert row['type'] == 'job_done', row.get('error')
    r = row['result']
    assert r['n_pairs'] == 6 and r['complete'] is True

    from pathlib import Path

    from dune.survey import read_survey
    cap = read_survey(Path(r['file']))
    assert cap.n_pairs == 6
    assert row['artifacts'], "no artifact recorded"
    Path(r['file']).unlink()                 # this test writes into results/
    await app.aclose()


@runs_async
async def test_an_incomplete_survey_is_reported_as_such(tmp_path):
    """An anchor being off is the normal failure, and must not look like success."""
    tag = FakeTag(survey_pairs=3)
    tag.survey_pairs = 3
    app = make_app(tmp_path, tag, [])
    await asyncio.sleep(0.2)

    # Announce 6 pairs but only send 3, then stall.
    orig = tag.write

    def announce_more(data: bytes):
        if data.decode().strip() == 'SURVEY':
            with tag._lock:
                tag._pending.append("SURVEY_BEGIN,v1,6")
                for k, (i, j) in enumerate(((1, 2), (1, 3), (1, 4))):
                    tag._pending.append(f"SURVEY,v1,{i},{j},{1000 * (i + j)},51")
            tag.written.append('SURVEY')
            return
        orig(data)

    tag.write = announce_more
    await app.jobs.run('survey_capture', {'timeout': 12.0, 'stall': 2.0})
    row = await _wait_done(app, timeout=30)
    assert row['type'] == 'job_done'
    assert row['result']['n_pairs'] == 3
    assert row['result']['complete'] is False, "a short survey looked complete"

    from pathlib import Path
    Path(row['result']['file']).unlink(missing_ok=True)
    await app.aclose()


@runs_async
async def test_survey_capture_holds_the_link_exclusively(tmp_path):
    """The firmware stops sweeping during SURVEY, so nothing should flow."""
    tag = FakeTag(survey_pairs=6)
    fixes: list[str] = []
    app = make_app(tmp_path, tag, fixes)
    await asyncio.sleep(0.3)

    await app.jobs.run('survey_capture', {'timeout': 20.0, 'stall': 5.0})
    # Count only what arrives once the hold is established: a few lines already
    # queued when it began are the pipeline's, not a leak.
    await asyncio.sleep(0.15)
    n_held = len(fixes)
    row = await _wait_done(app)
    during = len(fixes) - n_held
    assert row['type'] == 'job_done'
    assert during == 0, f"{during} sweeps leaked through an exclusive hold"

    from pathlib import Path
    Path(row['result']['file']).unlink(missing_ok=True)
    await app.aclose()


# ── registration ──────────────────────────────────────────────────────────────

def test_the_calibration_jobs_are_advertised_under_the_cal_mode():
    cal = {k for k, v in JOB_KINDS.items() if v.mode == 'cal'}
    assert {'anchor_delay_push', 'tag_delay', 'survey_capture'} <= cal


def test_hardware_writing_jobs_default_to_not_applying():
    for kind in ('anchor_delay_push', 'tag_delay'):
        schema = JOB_KINDS[kind].args_schema
        assert schema['apply']['default'] is False, (
            f"{kind} would change hardware on a bare run")


def test_every_tag_link_job_declares_the_resource():
    """Two jobs driving one link would interleave commands and cross their ACKs."""
    for kind in ('anchor_delay_push', 'tag_delay', 'survey_capture'):
        assert 'tag_link' in JOB_KINDS[kind].requires, (
            f"{kind} can run concurrently with another link job")


@runs_async
async def test_a_silent_anchor_is_reported_as_no_reply(tmp_path):
    """A powered-down anchor must be reported, not silently counted as done."""
    app = make_app(tmp_path, FakeTag(silent=True), [])
    await asyncio.sleep(0.2)
    await app.jobs.run('anchor_delay_push',
                       {'ids': '4', 'ticks': 16400, 'apply': True,
                        'timeout': 1.0})
    row = await _wait_done(app)
    assert row['type'] == 'job_done'
    assert row['result']['n_ok'] == 0
    assert row['result']['results'][0]['ok'] is False

    app.recorder._fh.flush()
    rows = [json.loads(l) for l in open(app.recorder.path) if l.strip()]
    logs = [r.get('msg', '') for r in rows if r['type'] == 'job_log']
    assert any('no reply' in m for m in logs), logs
    await app.aclose()


@runs_async
async def test_two_link_jobs_cannot_run_at_once(tmp_path):
    """Refused, not queued: a queued job would move delays later, unprompted."""
    # Silent, so the first job stays busy waiting out its timeout. `ack=False`
    # would NOT do — it replies FAIL immediately and the job finishes at once.
    tag = FakeTag(silent=True)
    app = make_app(tmp_path, tag, [])
    await asyncio.sleep(0.2)

    first = await app.jobs.run('anchor_delay_push',
                               {'ids': '1 2 3', 'ticks': 16400, 'apply': True,
                                'timeout': 5.0})
    assert first is not None
    await asyncio.sleep(0.2)
    second = await app.jobs.run('tag_delay', {'tagId': 240})
    assert second is None, "a second link job was allowed to start"

    app.recorder._fh.flush()
    rows = [json.loads(l) for l in open(app.recorder.path) if l.strip()]
    fail = [r for r in rows if r['type'] == 'job_failed']
    assert fail and 'resource busy: tag_link' in fail[-1]['error']

    await app.jobs.cancel(first)
    await app.aclose()
