"""Offline job kinds: analysis over recorded files, no hardware involved.

These are the safest tools to expose first — nothing here can move an antenna
delay or a wheel — and they are what makes "get a report" reachable from the
browser before any rover code exists.

Every body follows the rule from `jobs.py`: **file paths in, JSON out, no live
handles.** The `app` argument is used only to resolve paths, never to reach into
the running pipeline. That is what keeps the `ProcessPoolExecutor` option open.
"""

from __future__ import annotations

from datetime import datetime

from .jobs import job
from .paths import CONFIG_DIR, RESULTS_DIR, resolve_under_repo

# Argument fragments reused across the replay-shaped jobs.
_LOG_ARG = {'t': 'path', 'kind': 'logs', 'required': True}
_ANCHORS_ARG = {'t': 'path', 'kind': 'config'}
_TAGZ_ARG = {'t': 'number', 'default': 0.22, 'min': 0.0, 'max': 3.0, 'step': 0.01}


@job('replay_rtls', 'Replay a session log', mode='lab', args_schema={
    'log':      _LOG_ARG,
    'anchors':  _ANCHORS_ARG,
    'tagZ':     _TAGZ_ARG,
    'gapWeights': {'t': 'bool', 'default': True},
    'gateK':    {'t': 'number', 'default': 3.0, 'min': 0.0, 'max': 10.0, 'step': 0.5},
})
def replay_rtls(app, ctx, log: str, anchors: str | None = None,
                tagZ: float = 0.22, gapWeights: bool = True,
                gateK: float = 3.0) -> dict:
    """Re-solve a recorded session through the live solve path and report it."""
    from reports.replay import replay_session

    log_path = resolve_under_repo(log)
    out_dir = RESULTS_DIR / f"replay_{log_path.stem}_{_stamp()}"
    ctx.log(f"replaying {log_path.name}")

    result = replay_session(
        log_path, out_dir,
        anchors_file=resolve_under_repo(anchors) if anchors else None,
        tag_z=tagZ, use_gap_weights=gapWeights, gate_k=gateK,
        progress=lambda f, m: ctx.progress(f, msg=m),
        check_cancel=ctx.check_cancel,
    )
    for a in result.pop('artifacts', []):
        ctx.artifact(a['path'], a['kind'], a.get('bytes'))
    ctx.log(f"report written to {out_dir.name}")
    return result


@job('rover_run_report', 'Rover run report', mode='lab', args_schema={
    'matFile': {'t': 'path', 'kind': 'rover_runs', 'required': True},
    'anchors': _ANCHORS_ARG,
    'tagZ':    {'t': 'number', 'default': 0.24, 'min': 0.0, 'max': 3.0, 'step': 0.01},
})
def rover_run_report(app, ctx, matFile: str, anchors: str | None = None,
                     tagZ: float = 0.24) -> dict:
    """Score a `rover_teleop_uwb` capture against its AprilTag ground truth."""
    from reports.rover import rover_report

    mat = resolve_under_repo(matFile)
    out_dir = RESULTS_DIR / f"roverreport_{mat.stem}_{_stamp()}"
    ctx.log(f"loading {mat.name}")

    result = rover_report(
        mat, out_dir,
        anchors_file=resolve_under_repo(anchors) if anchors else None,
        tag_z=tagZ,
        progress=lambda f, m: ctx.progress(f, msg=m),
        check_cancel=ctx.check_cancel,
    )
    for a in result.pop('artifacts', []):
        ctx.artifact(a['path'], a['kind'], a.get('bytes'))
    return result


@job('session_summary', 'Summarise a session log', mode='lab', args_schema={
    'log': _LOG_ARG,
})
def session_summary(app, ctx, log: str) -> dict:
    """Cheap first look at a log: what is in it, before spending a full replay."""
    from rtls.session_log import read_session_log

    p = resolve_under_repo(log)
    S = read_session_log(p)
    ctx.progress(1.0, msg=f"{len(S)} sweeps")
    return dict(S.meta, n_events=len(S.events))


def _stamp() -> str:
    return datetime.now().strftime('%Y%m%d_%H%M%S')


# ── Survey / geometry ─────────────────────────────────────────────────────────

@job('survey_reconstruct', 'Reconstruct anchors from a self-survey', mode='lab',
     args_schema={
         'survey':    {'t': 'path', 'kind': 'survey', 'required': True},
         'reference': _ANCHORS_ARG,
         'corrected': {'t': 'bool', 'default': False},
         'z':         {'t': 'number', 'default': 0.24, 'min': 0.0, 'max': 3.0},
         'write':     {'t': 'bool', 'default': False},
     })
def survey_reconstruct(app, ctx, survey: str, reference: str | None = None,
                       corrected: bool = False, z: float = 0.24,
                       write: bool = False) -> dict:
    """Turn anchor-to-anchor ranges into a constellation, Kinect-free.

    `write` is opt-in and off by default: this produces an anchors.json, and
    silently replacing the layout every other tool reads would be a surprising
    side effect of pressing "run".
    """
    import json


    from rtls.anchor_config import AnchorConfig
    from rtls.survey import (build_survey_anchors, fit_anchor_delays,
                             read_survey)

    path = resolve_under_repo(survey)
    ref_path = resolve_under_repo(reference) if reference else (
        CONFIG_DIR / 'anchors.json')
    A = AnchorConfig.from_json(str(ref_path))

    cap = read_survey(path, ids=list(A.ids))
    ctx.progress(0.4, msg=f"{cap.n_pairs} pairs, {cap.ids.size} anchors")
    if not cap.complete:
        raise ValueError(f"incomplete survey — missing pairs {cap.missing_pairs()}")

    doc = build_survey_anchors(cap, A.coords[:, :2], corrected=corrected, z=z,
                               layout_base=path.stem)
    b, ticks, fit_rms = fit_anchor_delays(cap.D, A.coords[:, :2])
    ctx.progress(0.8, msg="reconstructed")

    out_dir = RESULTS_DIR / 'survey_runs'
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = 'corrected' if corrected else 'raw'
    out = out_dir / f"{path.stem}_anchors_{tag}_py.json"
    out.write_text(json.dumps(doc, indent=2))
    ctx.artifact(out, 'json')

    if write:
        target = RESULTS_DIR.parent / 'config' / 'anchors.json'
        target.write_text(json.dumps(doc, indent=2))
        ctx.log(f"WROTE {target} — restart or reload to pick it up")
        ctx.artifact(target, 'json')

    return {
        'survey': str(path),
        'reference': str(ref_path),
        'n_anchors': int(cap.ids.size),
        'n_pairs': int(cap.n_pairs),
        'rms_vs_reference_mm': doc['_rms_vs_reference_mm'],
        'delay_fit': {
            'per_anchor_mm': [round(float(v) * 1000, 1) for v in b],
            'per_anchor_ticks': [int(round(float(v))) for v in ticks],
            'residual_rms_mm': round(float(fit_rms) * 1000, 1),
            'ids': [int(i) for i in cap.ids],
        },
        'wrote_anchors_json': bool(write),
    }


@job('survey_fit', 'Per-anchor delay fit from a self-survey', mode='lab',
     args_schema={
         'survey':    {'t': 'path', 'kind': 'survey', 'required': True},
         'reference': _ANCHORS_ARG,
     })
def survey_fit(app, ctx, survey: str, reference: str | None = None) -> dict:
    """Fit `meas_ij = true_ij + b_i + b_j` — how much error is antenna delay.

    Reported in millimetres and in DW1000 ticks, because ticks are what gets
    pushed to an anchor's NVS.
    """
    from rtls.anchor_config import AnchorConfig
    from rtls.survey import fit_anchor_delays, read_survey

    path = resolve_under_repo(survey)
    ref_path = resolve_under_repo(reference) if reference else (
        CONFIG_DIR / 'anchors.json')
    A = AnchorConfig.from_json(str(ref_path))
    cap = read_survey(path, ids=list(A.ids))
    b, ticks, rms = fit_anchor_delays(cap.D, A.coords[:, :2])
    ctx.progress(1.0, msg=f"residual RMS {rms * 1000:.1f} mm")
    return {
        'ids': [int(i) for i in cap.ids],
        'bias_mm': [round(float(v) * 1000, 1) for v in b],
        'bias_ticks': [int(round(float(v))) for v in ticks],
        'residual_rms_mm': round(float(rms) * 1000, 1),
        'reference': str(ref_path),
    }


@job('tag_bias_fit', 'Fit per-tag antenna bias', mode='lab', args_schema={
    'capture': {'t': 'path', 'kind': 'captures', 'required': True},
    'anchors': _ANCHORS_ARG,
    'tagZ':    {'t': 'number', 'default': 0.22, 'min': 0.0, 'max': 3.0, 'step': 0.01},
    'anchorBias': {'t': 'bool', 'default': True},
})
def tag_bias_fit(app, ctx, capture: str, anchors: str | None = None,
                 tagZ: float = 0.22, anchorBias: bool = True) -> dict:
    """Joint (position, tag bias) fit from a static capture.

    `anchorBias` applies `anchor_bias.json`, matching MATLAB's default. It is
    only meaningful with the geometry it was fitted against — the committed file
    names that layout in its own `note` field — so a mismatch shows up as a large
    residual rather than a wrong-but-plausible bias.
    """
    from rtls.anchor_config import AnchorConfig
    from rtls.frame_parser import FrameParser
    from rtls.tag_bias import fit_tag_bias as _fit, load_anchor_bias

    cap = resolve_under_repo(capture)
    cfg = AnchorConfig.from_json(str(resolve_under_repo(anchors))) if anchors \
        else AnchorConfig.from_json(str(CONFIG_DIR / 'anchors.json'))

    sweeps = [s for s in (FrameParser.parse(l) for l in
                          cap.read_text(errors='replace').splitlines()) if s.valid]
    if not sweeps:
        raise ValueError(f"{cap.name} has no parseable RTLS lines")
    ctx.progress(0.3, msg=f"{len(sweeps)} sweeps")

    ab = load_anchor_bias(CONFIG_DIR / 'anchor_bias.json') if anchorBias else {}
    results = _fit(sweeps, cfg, anchor_bias=ab, tag_z=tagZ)
    ctx.progress(1.0, msg=f"{len(results)} tags")

    for r in results:
        ctx.log(f"tag {r.tag}: pos [{r.pos[0]:.3f} {r.pos[1]:.3f}] "
                f"bias {1000 * r.bias_m:+.1f} mm ({r.n_sweeps} sweeps)")
    return {'capture': str(cap), 'n_sweeps': len(sweeps),
            'anchor_bias_applied': bool(ab),
            'tags': [r.to_dict() for r in results]}
