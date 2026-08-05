"""Score a rover run: UWB estimate against AprilTag ground truth.

Port of the analysis in ``matlab/scripts/rover_run_report.m``.

A rover capture carries, on one POSIX clock, the AprilTag pose (truth), every
UWB sweep both tags produced, and the drive commands. This re-solves the sweeps
offline through the live solve path, lines them up against truth, and reports
the moving accuracy.

**The two worlds are not assumed to be the same.** The Kinect world and the UWB
anchor world are independently defined, so a 2D rigid transform between them is
*fitted* (rotation and translation only — fitting scale too would absorb genuine
range error and flatter the result). Everything downstream is reported in the
anchor frame.

The rigid two-tag MHE that `rover_run_report.m` also runs is Phase 3 work; per-
tag fixes are reported here and the report says so rather than implying the
comparison is complete.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

import numpy as np

from dune.anchor_config import AnchorConfig
from dune.paths import config
from dune.metrics import prctile, trajectory_metrics
from dune.solve_sweep import solve_sweep

from .matfile import as_text, load_mat, scalar
from .replay import Sweep
from .writer import ReportWriter

Progress = Optional[Callable[[float, str], None]]
CheckCancel = Optional[Callable[[], None]]   # raises to abort; see reports/replay.py


def rover_report(mat_file: str | Path,
                 out_dir: str | Path,
                 *,
                 anchors_file: str | Path | None = None,
                 tag_z: float = 0.24,
                 progress: Progress = None,
                 check_cancel: CheckCancel = None) -> dict:
    mat_file = Path(mat_file)
    S = load_mat(mat_file)
    data, meta = S.get('data', {}), S.get('metadata', {})
    uwb = data.get('uwb')
    if not uwb:
        raise ValueError(f"{mat_file.name} has no data.uwb — not a rover_teleop_uwb capture")

    cfg = AnchorConfig.from_json(str(anchors_file)) if anchors_file else \
        AnchorConfig.from_json(str(_default_anchors()))

    mu = meta.get('uwb', {})
    front_tag = int(scalar(mu.get('frontTag'), 241))
    rear_tag = int(scalar(mu.get('rearTag'), 240))
    baseline = scalar(mu.get('baseline_m'), float('nan'))
    offset_body = np.asarray(mu.get('apriltag_offset_body', [0.0, 0.0]),
                             dtype=float).ravel()[:2]

    rep = ReportWriter(out_dir, f"Rover run report — {mat_file.stem}")
    rep.line(f"Capture: {mat_file}")
    rep.line(f"Anchors: {anchors_file or _default_anchors()} "
             f"({as_text(mu.get('anchors_layout', ''))})")
    rep.line(f"Tags: front {front_tag}, rear {rear_tag} | "
             f"baseline {baseline:.3f} m | tagZ {tag_z:.2f} m")
    rep.line(f"AprilTag offset in body frame: "
             f"[{offset_body[0]:+.4f} {offset_body[1]:+.4f}] m")

    # ── truth ────────────────────────────────────────────────────────────────
    t_truth = np.asarray(data.get('t_posix'), dtype=float).ravel()
    apos = np.asarray(data.get('apriltag_pos'), dtype=float)
    if apos.ndim == 2 and apos.shape[1] == 3:
        truth_xy = apos[:, :2]
    else:
        truth_xy = apos.reshape(-1, 3)[:, :2]
    ok_truth = np.isfinite(truth_xy[:, 0]) & np.isfinite(t_truth[:truth_xy.shape[0]])
    rep.line(f"Rover samples: {t_truth.size}  "
             f"({int(ok_truth.sum())} with an AprilTag fix, "
             f"{100 * ok_truth.mean():.0f}%)")

    # ── solve every sweep ────────────────────────────────────────────────────
    n_sweeps = int(np.asarray(uwb['t_posix']).size)
    rep.line(f"UWB sweeps: {n_sweeps}")
    rep.line('')

    per_tag = _solve_sweeps(uwb, cfg, tag_z=tag_z, progress=progress,
                            check_cancel=check_cancel)

    result: dict = {'mat': str(mat_file), 'n_sweeps': n_sweeps,
                    'front_tag': front_tag, 'rear_tag': rear_tag,
                    'baseline_nominal_m': baseline, 'tags': {}}

    # Body yaw, needed to place each tag relative to the AprilTag (below).
    aeul = np.asarray(data.get('apriltag_euler'), dtype=float)
    yaw_deg = (aeul[:, 2] if aeul.ndim == 2 and aeul.shape[1] == 3
               else aeul.reshape(-1, 3)[:, 2])
    finite_yaw = yaw_deg[np.isfinite(yaw_deg)]
    if finite_yaw.size and np.max(np.abs(finite_yaw)) <= 2 * np.pi:
        # A full-circle heading that never exceeds 2*pi is radians, not degrees.
        yaw_deg = np.degrees(yaw_deg)

    t_t = t_truth[:truth_xy.shape[0]]

    # Each tag sits at its own place on the body, so each has its own truth
    # track. Comparing both against the AprilTag centre inflates the error and
    # lets the frame fit absorb the lever arm as a false rotation.
    levers, truths = {}, {}
    for tag_id in per_tag:
        levers[tag_id] = _tag_lever(tag_id, front_tag, rear_tag, baseline, offset_body)
        truths[tag_id] = _apply_lever(truth_xy, yaw_deg[:truth_xy.shape[0]],
                                      levers[tag_id])

    # ONE transform, fitted from every tag's points at once. There is physically
    # a single Kinect->anchor transform, and fitting it per tag on a short run
    # is badly conditioned: the first version of this report produced +27 deg
    # for one tag and -7 deg for the other, which cannot both be true. Pooling
    # the points constrains it far better, and `condition` below makes a fit
    # that still cannot be trusted visible instead of merely wrong.
    Rf, tf, fit_rmse, cond = _fit_shared_frame(per_tag, truths, t_t)
    ang = float(np.degrees(np.arctan2(Rf[1, 0], Rf[0, 0])))
    rep.section("KINECT <-> ANCHOR FRAME (one transform, fitted from all tags)")
    rep.line(f"Rotation {ang:+.2f} deg | translation [{tf[0]:+.3f} {tf[1]:+.3f}] m")
    rep.line(f"Fit RMSE {1000 * fit_rmse:.0f} mm over {cond['n']} paired points")
    rep.line(f"Conditioning: truth spread {cond['spread_m']:.2f} m, "
             f"anisotropy {cond['anisotropy']:.2f}  -> {cond['verdict']}")
    if cond['verdict'] != 'well-conditioned':
        rep.line("  A poorly-conditioned fit cannot separate rotation from")
        rep.line("  translation; treat the rotation above as indicative only.")
    result['frame'] = {'rotation_deg': ang,
                       'translation_m': [float(tf[0]), float(tf[1])],
                       'fit_rmse_mm': float(1000 * fit_rmse), **cond}

    for tag_id, T in sorted(per_tag.items()):
        summary = _report_tag(rep, tag_id, T, t_t, truths[tag_id],
                              levers[tag_id], Rf, tf)
        result['tags'][str(tag_id)] = summary

    _report_baseline(rep, per_tag, front_tag, rear_tag, baseline, result)

    _report_sanity(rep, result)

    rep.line('')
    rep.line("NOTE: per-tag fixes only. The rigid two-tag MHE that MATLAB's")
    rep.line("rover_run_report also scores is not yet ported (Phase 3), so these")
    rep.line("numbers are the raw solve, not the best this data can support.")

    _plot(rep, cfg, per_tag, t_t, truth_xy)
    rep.write()
    result['artifacts'] = rep.artifacts
    result['report'] = str(Path(out_dir) / 'report.txt')
    return result


# ── solving ───────────────────────────────────────────────────────────────────

def _solve_sweeps(uwb: dict, cfg: AnchorConfig, *, tag_z: float,
                  progress: Progress, check_cancel) -> dict:
    ids = np.asarray(uwb['anchorIds'], dtype=float).ravel().astype(int)
    R = np.asarray(uwb['R'], dtype=float)
    RX = np.asarray(uwb['RX'], dtype=float)
    FP = np.asarray(uwb['FP'], dtype=float)
    t = np.asarray(uwb['t_posix'], dtype=float).ravel()
    tags = np.asarray(uwb['tag'], dtype=float).ravel().astype(int)

    # After load_mat's transpose these are [n_sweeps x n_anchors].
    if R.shape[0] != t.size and R.shape[1] == t.size:
        R, RX, FP = R.T, RX.T, FP.T

    per_tag: dict[int, dict] = {}
    last: dict[int, np.ndarray] = {}
    n = t.size

    for i in range(n):
        if check_cancel is not None:
            check_cancel()
        if progress is not None and i % 200 == 0:
            progress(i / max(n, 1), f"sweep {i}/{n}")

        present = np.isfinite(R[i]) & (R[i] > 0)
        if not np.any(present):
            continue
        c = np.flatnonzero(present)
        sweep = Sweep(ids=ids[c], dist=R[i, c], rx=RX[i, c], fp=FP[i, c],
                      tag_id=int(tags[i]))
        sol = solve_sweep(sweep, cfg, tag_z=tag_z, x0=last.get(int(tags[i])))
        if sol.ok:
            last[int(tags[i])] = sol.pos

        T = per_tag.setdefault(int(tags[i]), {'t': [], 'xy': [], 'rmse': [],
                                              'n_used': []})
        T['t'].append(t[i])
        T['xy'].append(sol.pos if sol.ok else np.array([np.nan, np.nan]))
        T['rmse'].append(sol.info.rmse if (sol.ok and sol.info is not None) else np.nan)
        T['n_used'].append(int(np.sum(sol.info.used)) if sol.info is not None else 0)

    for T in per_tag.values():
        T['t'] = np.asarray(T['t'], dtype=float)
        T['xy'] = np.asarray(T['xy'], dtype=float).reshape(-1, 2)
        T['rmse'] = np.asarray(T['rmse'], dtype=float)
        T['n_used'] = np.asarray(T['n_used'], dtype=float)
    if progress is not None:
        progress(1.0, f"{n} sweeps")
    return per_tag


# ── frame fitting ─────────────────────────────────────────────────────────────

def fit_rigid_2d(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Least-squares rotation+translation taking `src` onto `dst` (no scale).

    Scale is deliberately excluded. The Kinect and anchor frames differ by a
    rigid motion; allowing a scale term would let the fit absorb systematic
    range error and report a flattering RMSE that says nothing about ranging.

    Returns (R, t, rmse). Kabsch/Umeyama with a reflection guard — without it a
    degenerate or noisy set can produce a mirrored "fit" that scores well and is
    physically impossible.
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    ok = np.isfinite(src[:, 0]) & np.isfinite(dst[:, 0])
    src, dst = src[ok], dst[ok]
    if src.shape[0] < 2:
        return np.eye(2), np.zeros(2), float('nan')

    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = cd - R @ cs
    resid = (src @ R.T + t) - dst
    return R, t, float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))


# ── reporting ─────────────────────────────────────────────────────────────────

def _tag_lever(tag_id: int, front: int, rear: int, baseline: float,
               offset_body: np.ndarray) -> np.ndarray:
    """Where this tag sits relative to the AprilTag, in body axes [fwd, left].

    Body geometry from `rover_teleop_uwb.m`: the two tags straddle the body
    centre along the forward axis, half a baseline each way, and the AprilTag is
    mounted at `apriltag_offset_body` from that same centre. So the vector from
    AprilTag to a tag is (tag offset from centre) - (AprilTag offset from centre).
    """
    if not np.isfinite(baseline):
        baseline = 0.0
    half = baseline / 2.0
    fwd = half if tag_id == front else (-half if tag_id == rear else 0.0)
    return np.array([fwd, 0.0]) - np.asarray(offset_body, float)[:2]


def _apply_lever(truth_xy: np.ndarray, yaw_deg: np.ndarray,
                 lever: np.ndarray) -> np.ndarray:
    """Rotate a body-frame offset into the world and add it to the truth track."""
    if not np.any(lever):
        return truth_xy
    c = np.cos(np.deg2rad(yaw_deg))
    s = np.sin(np.deg2rad(yaw_deg))
    return truth_xy + np.column_stack([c * lever[0] - s * lever[1],
                                       s * lever[0] + c * lever[1]])


def _fit_shared_frame(per_tag: dict, truths: dict, t_truth: np.ndarray):
    """Fit the single Kinect->anchor transform using every tag's paired points."""
    src, dst = [], []
    for tag_id, T in per_tag.items():
        at = _interp_truth(t_truth, truths[tag_id], T['t'])
        ok = np.isfinite(T['xy'][:, 0]) & np.isfinite(at[:, 0])
        src.append(T['xy'][ok])
        dst.append(at[ok])
    src = np.vstack(src) if src else np.zeros((0, 2))
    dst = np.vstack(dst) if dst else np.zeros((0, 2))
    R, t, rmse = fit_rigid_2d(src, dst)

    # Conditioning: a rigid fit needs the points to span two dimensions. Along a
    # straight drive, rotation and translation trade off almost freely and the
    # reported angle is close to meaningless.
    cond = {'n': int(dst.shape[0]), 'spread_m': 0.0, 'anisotropy': 0.0,
            'verdict': 'insufficient'}
    if dst.shape[0] >= 4:
        sv = np.linalg.svd(dst - dst.mean(0), compute_uv=False)
        sv = sv / np.sqrt(max(dst.shape[0] - 1, 1))
        cond['spread_m'] = float(sv[0])
        cond['anisotropy'] = float(sv[1] / sv[0]) if sv[0] > 0 else 0.0
        cond['verdict'] = ('well-conditioned'
                           if cond['spread_m'] > 0.5 and cond['anisotropy'] > 0.25
                           else 'ill-conditioned (near-collinear or small motion)')
    return R, t, rmse, cond


def _report_tag(rep: ReportWriter, tag_id: int, T: dict,
                t_truth: np.ndarray, truth_xy: np.ndarray,
                lever: np.ndarray | None = None,
                Rf: np.ndarray | None = None,
                tf: np.ndarray | None = None) -> dict:
    solved = np.isfinite(T['xy'][:, 0])
    rep.section(f"TAG {tag_id} (0x{tag_id:02X}) — {T['xy'].shape[0]} sweeps")
    rep.line(f"Solved: {100 * solved.mean():.1f}%  "
             f"(median residual {1000 * np.nanmedian(T['rmse']):.0f} mm, "
             f"median anchors used {np.nanmedian(T['n_used']):.1f})")
    if lever is not None:
        rep.line(f"Truth lever arm from AprilTag (body axes): "
                 f"[{lever[0]:+.4f} {lever[1]:+.4f}] m")

    truth_at_uwb = _interp_truth(t_truth, truth_xy, T['t'])
    both = solved & np.isfinite(truth_at_uwb[:, 0])
    if not np.any(both):
        rep.line("No overlapping truth — cannot score this tag.")
        return {'n_sweeps': int(T['xy'].shape[0]),
                'solved_pct': float(100 * solved.mean()), 'scored': 0}

    # Scored under the SHARED transform, not a per-tag one refitted here — a
    # per-tag fit would quietly absorb that tag's own error into the frame and
    # report a better number than the system deserves.
    if Rf is None or tf is None:
        raise ValueError("_report_tag needs the shared frame transform")
    aligned = T['xy'] @ Rf.T + tf
    X = trajectory_metrics(aligned, truth_at_uwb)

    rep.line(f"Moving accuracy vs AprilTag truth (n={X.n}, shared frame):")
    rep.line(f"  RMSE {1000 * X.rmse:.0f} mm | median {1000 * X.median:.0f} | "
             f"p95 {1000 * X.p95:.0f} | max {1000 * X.max:.0f} mm")

    return {'n_sweeps': int(T['xy'].shape[0]),
            'solved_pct': float(100 * solved.mean()),
            'scored': int(X.n),
            'residual_median_mm': float(1000 * np.nanmedian(T['rmse'])),
            'rmse_mm': float(1000 * X.rmse),
            'median_mm': float(1000 * X.median),
            'p95_mm': float(1000 * X.p95),
            'max_mm': float(1000 * X.max)}


def _report_baseline(rep: ReportWriter, per_tag: dict, front: int, rear: int,
                     nominal: float, result: dict) -> None:
    """Inter-tag distance is a truth-free check on the whole chain.

    The two tags are bolted to one rigid body, so their separation is known
    without any camera. A solve that cannot reproduce it is wrong in a way no
    frame fit can hide.
    """
    if front not in per_tag or rear not in per_tag:
        return
    A, B = per_tag[front], per_tag[rear]
    if A['t'].size == 0 or B['t'].size == 0:
        return

    idx = np.searchsorted(B['t'], A['t']).clip(0, B['t'].size - 1)
    dt = np.abs(B['t'][idx] - A['t'])
    pair = dt < 0.25                       # same cycle, generously
    d = np.hypot(*(A['xy'] - B['xy'][idx]).T)
    d = d[pair & np.isfinite(d)]

    rep.section("INTER-TAG BASELINE (truth-free consistency check)")
    if d.size == 0:
        rep.line("No paired sweeps within 250 ms — cannot check.")
        return
    err = d - nominal
    rep.line(f"Paired sweeps: {d.size}   nominal {nominal:.3f} m")
    rep.line(f"Measured: median {np.median(d):.3f} m | "
             f"error median {1000 * np.median(err):+.0f} mm | "
             f"p95 |err| {1000 * prctile(np.abs(err), 95):.0f} mm | "
             f"std {1000 * np.std(d):.0f} mm")
    result['baseline'] = {
        'n_paired': int(d.size),
        'median_m': float(np.median(d)),
        'err_median_mm': float(1000 * np.median(err)),
        'err_p95_mm': float(1000 * prctile(np.abs(err), 95)),
        'std_mm': float(1000 * np.std(d)),
    }


# Thresholds for the "does this anchors file belong to this capture" hint.
# Chosen from the committed corpus rather than invented: across 42 runs whose
# geometry matches, frame fit RMSE has p90 ~0.70 m and |baseline error| p90
# ~0.20 m; across 16 runs captured after the anchors were moved, the medians are
# ~0.82 m and ~0.20 m. The distributions OVERLAP, so these are set high enough
# to fire only on the unambiguous cases and the wording is a prompt to check,
# not a verdict. Detecting mismatch reliably needs the anchors file to carry the
# capture date it belongs to.
_FIT_RMSE_SUSPECT_M = 1.0
_BASELINE_SUSPECT_MM = 300.0


def _report_sanity(rep: ReportWriter, result: dict) -> None:
    """Flag results that most likely come from the wrong anchor geometry.

    Without this a mismatched anchors file produces a confident-looking report
    with metre-scale errors and no explanation — which is exactly what happens
    with the runs in this repo captured after the anchors were re-surveyed.
    """
    frame = result.get('frame', {})
    base = result.get('baseline', {})
    fit_m = (frame.get('fit_rmse_mm') or 0.0) / 1000.0
    base_mm = abs(base.get('err_median_mm') or 0.0)

    reasons = []
    if fit_m > _FIT_RMSE_SUSPECT_M:
        reasons.append(f"frame fit RMSE {fit_m:.2f} m (> {_FIT_RMSE_SUSPECT_M:.1f} m)")
    if base_mm > _BASELINE_SUSPECT_MM:
        reasons.append(f"inter-tag baseline off by {base_mm:.0f} mm "
                       f"(> {_BASELINE_SUSPECT_MM:.0f} mm)")
    if not reasons:
        return

    rep.section("CHECK THE ANCHOR GEOMETRY")
    for r in reasons:
        rep.line(f"  - {r}")
    rep.line("These are larger than this rig normally produces, which usually")
    rep.line("means the anchors file does not describe the layout this capture")
    rep.line("was recorded with. Re-run with the anchors.json from that session")
    rep.line("before reading anything into the accuracy numbers above.")
    result['geometry_suspect'] = reasons


def _interp_truth(t_truth: np.ndarray, truth_xy: np.ndarray,
                  t_uwb: np.ndarray, max_gap_s: float = 0.35) -> np.ndarray:
    ok = np.isfinite(t_truth) & np.isfinite(truth_xy[:, 0])
    if np.count_nonzero(ok) < 2:
        return np.full((t_uwb.size, 2), np.nan)
    tt, xy = t_truth[ok], truth_xy[ok]
    out = np.column_stack([np.interp(t_uwb, tt, xy[:, 0], left=np.nan, right=np.nan),
                           np.interp(t_uwb, tt, xy[:, 1], left=np.nan, right=np.nan)])
    idx = np.searchsorted(tt, t_uwb).clip(1, tt.size - 1)
    nearest = np.where(np.abs(t_uwb - tt[idx - 1]) <= np.abs(tt[idx] - t_uwb),
                       tt[idx - 1], tt[idx])
    out[np.abs(nearest - t_uwb) > max_gap_s] = np.nan
    return out


def _plot(rep: ReportWriter, cfg: AnchorConfig, per_tag: dict,
          t_truth: np.ndarray, truth_xy: np.ndarray) -> None:
    if not per_tag:
        return
    with rep.figure('rover_report.png') as fig:
        ax = fig.add_subplot(1, 1, 1)
        ok = np.isfinite(truth_xy[:, 0])
        if np.any(ok):
            ax.plot(truth_xy[ok, 0], truth_xy[ok, 1], '-', lw=2, color='#00ff9d',
                    label='AprilTag truth')
        for colour, (tag_id, T) in zip(('#00d4ff', '#a78bfa'), sorted(per_tag.items())):
            s = np.isfinite(T['xy'][:, 0])
            ax.plot(T['xy'][s, 0], T['xy'][s, 1], '.', ms=3, color=colour,
                    label=f'tag {tag_id} (raw solve)')
        ax.plot(cfg.coords[:, 0], cfg.coords[:, 1], 's', ms=10, mfc='none',
                color='#ff9d00', label='anchors')
        ax.set_aspect('equal', 'box')
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('Rover run — UWB vs AprilTag truth (Kinect frame)')


def _default_anchors() -> Path:
    return config('anchors.json')
