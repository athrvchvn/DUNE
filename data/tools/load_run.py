#!/usr/bin/env python3
"""Load one DUNE dataset run.

Usage:
    python3 tools/load_run.py runs/run_01_20260725_184427

As a library:
    from load_run import load_run
    r = load_run('runs/run_01_20260725_184427')
    r['uwb']['R']            # (A, S) ranges in metres, NaN where absent
    r['uwb']['gap']          # (A, S) RX - FP in dB, the NLOS indicator
    r['gt']['pos']           # (3, N) ground-truth position in metres
    r['meta']                # the run's metadata.json

Requires: numpy, h5py.  run.mat is MATLAB v7.3 (HDF5) — scipy.io.loadmat cannot read it.
"""
from __future__ import annotations

import json
import os
import sys

import h5py
import numpy as np


def _str(h5obj) -> str:
    """MATLAB char arrays come back as uint16 codepoint columns."""
    return "".join(chr(int(c)) for c in np.array(h5obj).flatten())


def _rows(arr: np.ndarray, n_rows: int) -> np.ndarray:
    """Return arr oriented with n_rows rows, transposing if HDF5 stored it flipped."""
    a = np.array(arr)
    if a.ndim == 2 and a.shape[0] != n_rows and a.shape[1] == n_rows:
        a = a.T
    return a


def load_run(run_dir: str) -> dict:
    run_dir = os.path.abspath(run_dir)
    with open(os.path.join(run_dir, "metadata.json")) as fh:
        meta = json.load(fh)

    with h5py.File(os.path.join(run_dir, "run.mat"), "r") as h:
        d, m = h["data"], h["metadata"]
        n_a = int(np.array(d["uwb/anchorIds"]).size)

        R = _rows(d["uwb/R"], n_a).astype(float)
        RX = _rows(d["uwb/RX"], n_a).astype(float)
        FP = _rows(d["uwb/FP"], n_a).astype(float)
        R[R <= 0] = np.nan                       # sentinel for "no measurement this sweep"

        uwb = dict(
            anchor_ids=np.array(d["uwb/anchorIds"]).flatten().astype(int),
            R=R, RX=RX, FP=FP,
            gap=RX - FP,                          # NLOS indicator used by the paper's weighting
            tag=np.array(d["uwb/tag"]).flatten().astype(int),
            t_rel=np.array(d["uwb/t_rel"]).flatten(),
            t_posix=np.array(d["uwb/t_posix"]).flatten(),
            quat=_rows(d["uwb/quat"], 4),
            acc=_rows(d["uwb/acc"], 3),
            gyro=_rows(d["uwb/gyro"], 3),
        )

        gt = dict(
            pos=_rows(d["apriltag_pos"], 3),
            euler=_rows(d["apriltag_euler"], 3),
            t_posix=np.array(d["t_posix"]).flatten(),
            t_rel=np.array(d["time"]).flatten(),
        )

        ctrl = dict(
            imu_euler=_rows(d["imu_euler"], 3),
            imu_accel=_rows(d["imu_accel"], 3),
            imu_gyro=_rows(d["imu_gyro"], 3),
            wheel_rpm=_rows(d["wheel_rpm"], 6),
            motor_rpm=_rows(d["motor_rpm"], 6),
            commanded_motor_rpm=_rows(d["commanded_motor_rpm"], 6),
            steering_angles=_rows(d["steering_angles"], 4),
            V_cmd=np.array(d["V_cmd"]).flatten(),
            omega_cmd=np.array(d["omega_cmd"]).flatten(),
            segment_id=np.array(d["segment_id"]).flatten().astype(int),
        )

        rover = dict(
            wheelbase_m=float(np.array(m["wheelbase"]).squeeze()),
            track_width_m=float(np.array(m["track_width"]).squeeze()),
            wheel_radius_m=float(np.array(m["wheel_radius"]).squeeze()),
            control_rate_hz=float(np.array(m["control_rate"]).squeeze()),
            anchors_layout=_str(m["uwb/anchors_layout"]),
            front_tag=int(np.array(m["uwb/frontTag"]).squeeze()),
            rear_tag=int(np.array(m["uwb/rearTag"]).squeeze()),
            baseline_m=float(np.array(m["uwb/baseline_m"]).squeeze()),
            apriltag_offset_body=np.array(m["uwb/apriltag_offset_body"]).flatten(),
            frame_note=_str(m["uwb/frame_note"]),
        )

    return dict(meta=meta, uwb=uwb, gt=gt, ctrl=ctrl, rover=rover)


def split_by_tag(uwb: dict) -> dict:
    """Sweeps interleave the two tags. Split before treating either as a time series."""
    return {int(t): {k: (v[..., uwb["tag"] == t] if getattr(v, "ndim", 0) and k != "anchor_ids"
                         else v)
                     for k, v in uwb.items()}
            for t in np.unique(uwb["tag"])}


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    r = load_run(sys.argv[1])
    m, u = r["meta"], r["uwb"]

    print(f"{m['run_id']}   {m['duration_s']} s   {m['path_length_m']} m driven")
    print(f"  layout      {r['rover']['anchors_layout']}")
    print(f"  anchors     {[int(a) for a in u['anchor_ids']]}")
    print(f"  tags        front={r['rover']['front_tag']} rear={r['rover']['rear_tag']} "
          f"baseline={r['rover']['baseline_m']} m")
    print(f"  sweeps      {u['R'].shape[1]}  ({m['mean_anchors_per_sweep']} anchors/sweep, "
          f"{100 * m['frac_sweeps_ge4']:.1f}% with >=4)")
    print(f"  ranges      {np.nanmin(u['R']):.2f} .. {np.nanmax(u['R']):.2f} m")
    print(f"  NLOS gap    median {np.nanmedian(u['gap']):.1f} dB, "
          f"p95 {np.nanpercentile(u['gap'], 95):.1f} dB")

    g = r["gt"]["pos"]
    ok = np.isfinite(g).all(axis=0)
    print(f"  ground truth {ok.sum()}/{g.shape[1]} samples, "
          f"x {g[0, ok].min():.2f}..{g[0, ok].max():.2f}  "
          f"y {g[1, ok].min():.2f}..{g[1, ok].max():.2f} m")

    for tag, s in split_by_tag(u).items():
        dur = s["t_rel"][-1] - s["t_rel"][0]
        print(f"  tag {tag}: {s['R'].shape[1]} sweeps, {s['R'].shape[1] / dur:.1f} Hz")

    print(f"\n  NOTE: {r['rover']['frame_note']}")
    print("        Fit an SE(2) transform before comparing estimates to ground truth —")
    print("        see docs/coordinate_frames.md.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
