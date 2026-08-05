"""Re-solve a recorded session from its raw ranges and report what happened.

Port of the analysis in ``matlab/scripts/replay_baseline.m``, with the same
report layout so the two pipelines' outputs can be compared by eye.

The point of a replay is that it uses **the same solve path as the live app** —
`rtls.solve_sweep` — so a bug found here is the bug that was live. The previous
Python pipeline inlined its solve inside the live loop, which meant offline
analysis and live behaviour could drift apart, and did.

Two input dialects, because both exist in this repo:

* a **JSONL session log** (`rtls_log_*.jsonl`), read via `rtls.session_log`
* a **raw serial tee** (`serial_raw_*.log`), tab-stamped `<t_host>\\t<line>`,
  parsed with `FrameParser` — the more faithful of the two, since it exercises
  the parser as well as the solver
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from rtls.anchor_config import AnchorConfig
from rtls.frame_parser import FrameParser
from rtls.metrics import prctile
from rtls.raw_log import is_raw_tee, read_raw_tee
from rtls.session_log import read_session_log
from rtls.solve_sweep import solve_sweep

from .writer import ReportWriter

Progress = Optional[Callable[[float, str], None]]

# A callable that RAISES to abort, rather than one that returns a bool. The job
# layer passes `JobContext.check_cancel` straight in, so its `JobCancelled`
# propagates and is reported as a cancel. The bool-returning version needed this
# module to invent its own exception type, which `host.jobs` did not recognise —
# so a cancelled report was reported as FAILED, with a traceback.
CheckCancel = Optional[Callable[[], None]]


@dataclass
class Sweep:
    """The minimal shape `solve_sweep` consumes, for the JSONL path."""
    ids:  np.ndarray
    dist: np.ndarray
    rx:   np.ndarray
    fp:   np.ndarray
    tag_id: int = 0




def replay_session(log: str | Path,
                   out_dir: str | Path,
                   *,
                   anchors_file: str | Path | None = None,
                   tag_z: float = 0.22,
                   use_gap_weights: bool = True,
                   gate_k: float = 3.0,
                   progress: Progress = None,
                   check_cancel: CheckCancel = None) -> dict:
    """Re-solve `log` and write `report.txt` plus per-tag trajectory figures."""
    log = Path(log)
    cfg = (AnchorConfig.from_json(str(anchors_file)) if anchors_file
           else AnchorConfig.from_json(str(_default_anchors())))

    rep = ReportWriter(out_dir, f"Replay — {log.stem}")
    rep.line(f"Log: {log}")
    rep.line(f"Anchors: {len(cfg.ids)} @ {anchors_file or _default_anchors()}")
    rep.line(f"tagZ {tag_z:.2f} m | gap weights {use_gap_weights} | gate_k {gate_k}")
    for i, aid in enumerate(cfg.ids):
        c = cfg.coords[i]
        rep.line(f"  A{aid}  [{c[0]:6.2f} {c[1]:6.2f} {c[2]:6.2f}]")

    per_tag = _solve_all(log, cfg, tag_z=tag_z, use_gap_weights=use_gap_weights,
                         gate_k=gate_k, progress=progress,
                         check_cancel=check_cancel)

    result: dict = {'log': str(log), 'tags': {}, 'n_tags': len(per_tag)}
    for tag_id, T in sorted(per_tag.items()):
        _report_tag(rep, cfg, tag_id, T)
        result['tags'][str(tag_id)] = T['summary']
        _plot_tag(rep, cfg, tag_id, T)

    rep.write()
    result['artifacts'] = rep.artifacts
    result['report'] = str(Path(out_dir) / 'report.txt')
    return result


# ── solving ───────────────────────────────────────────────────────────────────

def _solve_all(log: Path, cfg: AnchorConfig, *, tag_z: float,
               use_gap_weights: bool, gate_k: float,
               progress: Progress, check_cancel) -> dict:
    raw = is_raw_tee(log)
    if raw:
        records = read_raw_tee(log)
        n = len(records)
        iterator = _iter_raw(records)
    else:
        S = read_session_log(log)
        n = len(S)
        iterator = _iter_jsonl(S)

    per_tag: dict[int, dict] = {}
    last_pos: dict[int, np.ndarray] = {}

    for i, (tag_id, t_host, sweep, orig_xy) in enumerate(iterator):
        if check_cancel is not None:
            check_cancel()
        if progress is not None and n and i % 250 == 0:
            progress(i / n, f"sweep {i}/{n}")

        T = per_tag.setdefault(tag_id, _new_tag_acc(len(cfg.ids)))
        sol = solve_sweep(sweep, cfg, tag_z=tag_z, x0=last_pos.get(tag_id),
                          use_gap_weights=use_gap_weights, gate_k=gate_k)
        if sol.ok:
            last_pos[tag_id] = sol.pos

        T['t'].append(t_host)
        T['xy'].append(sol.pos if sol.ok else np.array([np.nan, np.nan]))
        T['rmse'].append(sol.info.rmse if (sol.ok and sol.info is not None)
                         else np.nan)
        T['n_used'].append(int(np.sum(sol.info.used)) if sol.info is not None else 0)
        T['orig'].append(orig_xy)
        T['range'].append(sol.range_raw)
        T['gap'].append(sol.gap)

    if progress is not None:
        progress(1.0, f"{n} sweeps")

    for T in per_tag.values():
        _finalise(T)
    return per_tag


def _new_tag_acc(m: int) -> dict:
    return {'t': [], 'xy': [], 'rmse': [], 'n_used': [], 'orig': [],
            'range': [], 'gap': [], 'm': m}


def _finalise(T: dict) -> None:
    for k in ('t', 'rmse', 'n_used'):
        T[k] = np.asarray(T[k], dtype=float)
    for k in ('xy', 'orig'):
        T[k] = np.asarray(T[k], dtype=float).reshape(-1, 2)
    for k in ('range', 'gap'):
        T[k] = (np.vstack(T[k]) if T[k] else np.zeros((0, T['m'])))

    solved = np.isfinite(T['xy'][:, 0])
    n = int(T['xy'].shape[0])

    # Interval statistics: the honest measure of update rate, since a mean over
    # a session that includes a reboot gap is meaningless.
    t = T['t'][np.isfinite(T['t'])]
    dt = np.diff(np.sort(t)) if t.size > 1 else np.array([])
    dt = dt[dt > 0]

    # Difference against whatever the original pipeline recorded, where present.
    both = solved & np.isfinite(T['orig'][:, 0])
    diff = (np.hypot(*(T['xy'][both] - T['orig'][both]).T)
            if np.any(both) else np.array([]))

    T['solved'] = solved
    T['summary'] = {
        'n_sweeps':      n,
        'solved_pct':    float(100.0 * solved.mean()) if n else 0.0,
        'rate_hz':       float(1.0 / np.median(dt)) if dt.size else float('nan'),
        'interval_mean_ms': float(1000 * np.mean(dt)) if dt.size else float('nan'),
        'interval_p95_ms':  float(1000 * prctile(dt, 95)) if dt.size else float('nan'),
        'rmse_median_mm':   float(1000 * np.nanmedian(T['rmse'])) if n else float('nan'),
        'rmse_p95_mm':      float(1000 * prctile(T['rmse'], 95)) if n else float('nan'),
        'n_vs_original':    int(diff.size),
        'diff_median_mm':   float(1000 * np.median(diff)) if diff.size else float('nan'),
        'diff_p95_mm':      float(1000 * prctile(diff, 95)) if diff.size else float('nan'),
    }


def _iter_raw(records):
    for t_host, line in records:
        pkt = FrameParser.parse(line)
        if not pkt.valid:
            continue
        yield (int(pkt.tag_id), float(t_host), pkt,
               np.array([np.nan, np.nan]))


def _iter_jsonl(S):
    """Rebuild a solvable sweep from the logged per-anchor columns."""
    for i in range(len(S)):
        present = np.isfinite(S.R[i])
        if not np.any(present):
            continue
        cols = np.flatnonzero(present)
        yield (int(S.tag[i]),
               float(S.t_host[i]) if np.isfinite(S.t_host[i]) else float(S.t_ms[i]) / 1000.0,
               Sweep(ids=S.anchor_ids[cols], dist=S.R[i, cols],
                     rx=S.RX[i, cols], fp=S.FP[i, cols], tag_id=int(S.tag[i])),
               np.array([S.ex[i], S.ey[i]]) if np.isfinite(S.ex[i])
               else np.array([S.x[i], S.y[i]]))


# ── reporting ─────────────────────────────────────────────────────────────────

def _report_tag(rep: ReportWriter, cfg: AnchorConfig, tag_id: int, T: dict) -> None:
    s = T['summary']
    rep.section(f"TAG {tag_id} (0x{tag_id:02X}) — {s['n_sweeps']} sweeps")
    rep.line(f"Update rate: median {s['rate_hz']:.2f} Hz  "
             f"(mean interval {s['interval_mean_ms']:.0f} ms, "
             f"p95 {s['interval_p95_ms']:.0f} ms)")

    rep.line("Per-anchor (raw ranges):")
    rep.line("  id   avail%   medGap dB   gap>3dB%   gap>6dB%   medRange m")
    R, G = T['range'], T['gap']
    for c, aid in enumerate(cfg.ids):
        if R.shape[0] == 0:
            continue
        avail = float(np.mean(np.isfinite(R[:, c]))) * 100
        g = G[:, c][np.isfinite(G[:, c])]
        med_gap = float(np.median(g)) if g.size else float('nan')
        p3 = float(np.mean(g > 3) * 100) if g.size else float('nan')
        p6 = float(np.mean(g > 6) * 100) if g.size else float('nan')
        r = R[:, c][np.isfinite(R[:, c])]
        med_r = float(np.median(r)) if r.size else float('nan')
        rep.line(f"  A{aid}  {avail:6.1f}   {_f(med_gap, 8, 2)}   "
                 f"{_f(p3, 8, 1)}   {_f(p6, 8, 1)}   {_f(med_r, 9, 2)}")

    rep.line(f"Replay solve: {s['solved_pct']:.1f}% of sweeps solved  "
             f"(median LM residual RMSE {s['rmse_median_mm']:.0f} mm, "
             f"p95 {s['rmse_p95_mm']:.0f} mm)")
    if s['n_vs_original']:
        rep.line(f"Difference vs original pipeline output: "
                 f"median {s['diff_median_mm']:.0f} mm, "
                 f"p95 {s['diff_p95_mm']:.0f} mm  (n={s['n_vs_original']})")
    else:
        rep.line("Difference vs original pipeline output: "
                 "n/a (log carries no pipeline output)")


def _f(v: float, width: int, dp: int) -> str:
    return ('NaN'.rjust(width) if not np.isfinite(v)
            else f"{v:.{dp}f}".rjust(width))


def _plot_tag(rep: ReportWriter, cfg: AnchorConfig, tag_id: int, T: dict) -> None:
    xy, solved = T['xy'], T['solved']
    if not np.any(solved):
        return
    with rep.figure(f"trajectory_tag{tag_id}.png") as fig:
        ax = fig.add_subplot(1, 2, 1)
        ax.plot(xy[solved, 0], xy[solved, 1], '-', lw=0.6, alpha=0.5, color='#00d4ff')
        ax.plot(xy[solved, 0], xy[solved, 1], '.', ms=2, color='#00d4ff')
        ax.plot(cfg.coords[:, 0], cfg.coords[:, 1], 's', ms=9,
                mfc='none', color='#ff9d00')
        for i, aid in enumerate(cfg.ids):
            ax.annotate(f"A{aid}", cfg.coords[i, :2], textcoords='offset points',
                        xytext=(6, 5), fontsize=8)
        ax.set_aspect('equal', 'box')
        ax.grid(alpha=0.25)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_title(f"tag {tag_id} — replay trajectory "
                     f"({int(solved.sum())}/{solved.size} solved)")

        ax2 = fig.add_subplot(2, 2, 2)
        g = T['gap'][np.isfinite(T['gap'])]
        if g.size:
            ax2.hist(g, bins=60, color='#a78bfa')
            ax2.axvline(3, color='#ff9d00', lw=1, label='3 dB')
            ax2.axvline(6, color='#ff3d5a', lw=1, label='6 dB')
            ax2.legend(fontsize=7)
        ax2.set_title('NLOS gap (rx − fp)', fontsize=9)
        ax2.set_xlabel('dB')

        ax3 = fig.add_subplot(2, 2, 4)
        r = T['rmse'][np.isfinite(T['rmse'])] * 1000
        if r.size:
            ax3.hist(r, bins=60, color='#00ff9d')
        ax3.set_title('LM residual RMSE', fontsize=9)
        ax3.set_xlabel('mm')


def _default_anchors() -> Path:
    return (Path(__file__).resolve().parents[2] / 'matlab' / 'config' / 'anchors.json')
