#!/usr/bin/env python3
"""
run_survey.py — Anchor self-survey: collect pairwise ranges, run classical
                MDS, and write matlab/config/anchors.json.

Python equivalent of matlab/runSurvey.m.

Usage:
    python run_survey.py --port /dev/ttyUSB0
    python run_survey.py --port COM5 --anchors 1,2,3 --dim 2 --z 1.5

Steps:
    1. Flash all anchors with Anchor.ino and the tag with Tag.ino.
    2. Connect the tag via USB serial.
    3. Run this script — it sends "SURVEY\\n" and waits for results.
    4. anchors.json is written automatically to matlab/config/anchors.json.
    5. Run run_localization.py to start localization with the new anchor map.

Wire protocol from tag:
    SURVEY_BEGIN,v1,<n_pairs>
    SURVEY,v1,<src_id>,<dst_id>,<avg_dist_mm>,<ok_samples>   (one per pair)
    SURVEY_DONE,v1
"""

import sys
import json
import argparse
import time
import math
from pathlib import Path
from datetime import datetime

try:
    import numpy as np
except ImportError:
    print("Missing dependency: pip install numpy")
    sys.exit(1)

try:
    import serial as pyserial
except ImportError:
    print("Missing dependency: pip install pyserial")
    sys.exit(1)


SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = SCRIPT_DIR.parent / 'matlab' / 'config'


# ── MDS ──────────────────────────────────────────────────────────────────────

def classical_mds(D: np.ndarray, dim: int) -> np.ndarray:
    """
    Classical (metric) MDS from a symmetric N×N distance matrix D.
    Returns N×dim coordinate array.
    """
    N = D.shape[0]
    D2 = D ** 2
    J = np.eye(N) - np.ones((N, N)) / N
    B = -0.5 * J @ D2 @ J
    eigvals, eigvecs = np.linalg.eigh(B)
    # sort descending
    idx = np.argsort(eigvals)[::-1]
    eigvals = eigvals[idx]
    eigvecs = eigvecs[:, idx]
    # clamp small negatives from measurement noise
    eigvals = np.maximum(eigvals, 0.0)
    d = min(dim, N - 1)
    X = eigvecs[:, :d] * np.sqrt(eigvals[:d])
    return X


def fix_frame(X: np.ndarray) -> np.ndarray:
    """
    Canonical coordinate frame:
      - Anchor 0 at origin.
      - Anchor 1 on the +X axis.
      - Anchor 2 in the +Y half-plane (if present).
    """
    N, d = X.shape
    X = X - X[0]                           # anchor 0 → origin
    if N >= 2 and np.linalg.norm(X[1]) > 1e-6:
        theta = math.atan2(float(X[1, 1]), float(X[1, 0]))
        c, s = math.cos(-theta), math.sin(-theta)
        R = np.array([[c, -s], [s, c]])
        X = (R @ X.T).T                    # rotate → anchor 1 on +X
    if d >= 2 and N >= 3 and float(X[2, 1]) < 0:
        X[:, 1] = -X[:, 1]                 # flip → anchor 2 in +Y half-plane
    return X


# ── Survey receiver ──────────────────────────────────────────────────────────

def run_survey(port: str, baud: int, anchor_ids: list[int], dim: int,
               z_height: float, timeout_s: float, samples_override: int | None):
    """Open serial port, trigger survey, collect pairwise distances."""

    print(f"Opening {port} at {baud} baud...")
    ser = pyserial.Serial(port, baud, timeout=2.0)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Send survey trigger
    ser.write(b'SURVEY\n')
    ser.flush()
    print("Sent SURVEY command. Waiting for results...\n")

    n_anchors = len(anchor_ids)
    n_pairs_expected = n_anchors * (n_anchors - 1) // 2
    pairs = []   # list of {'src': int, 'dst': int, 'dist_m': float, 'ok': int}

    t0 = time.monotonic()
    done = False

    while time.monotonic() - t0 < timeout_s:
        try:
            raw = ser.readline()
        except Exception as e:
            print(f"  [serial error] {e}")
            continue
        if not raw:
            continue
        line = raw.decode('utf-8', errors='replace').strip()
        if not line:
            continue
        print(f"  {line}")

        if line.startswith('SURVEY_DONE'):
            done = True
            break
        elif line.startswith('SURVEY,v1,'):
            tok = line.split(',')
            if len(tok) >= 6:
                try:
                    src_id  = int(tok[2])
                    dst_id  = int(tok[3])
                    dist_mm = float(tok[4])
                    ok_cnt  = int(tok[5])
                    if ok_cnt > 0 and dist_mm > 0:
                        pairs.append({'src': src_id, 'dst': dst_id,
                                      'dist_m': dist_mm / 1000.0, 'ok': ok_cnt})
                except (ValueError, IndexError) as e:
                    print(f"  [parse error] {e}")

    ser.close()

    if not done:
        print(f"\nWARNING: Did not receive SURVEY_DONE within {timeout_s:.0f} s.")

    print(f"\nCollected {len(pairs)} / {n_pairs_expected} pairs.")

    if not pairs:
        print("ERROR: No survey pairs received. Check serial port and tag firmware.")
        sys.exit(1)

    return pairs


# ── MDS + JSON writer ─────────────────────────────────────────────────────────

def compute_and_write(pairs, anchor_ids, dim, z_height):
    N = len(anchor_ids)
    id_to_idx = {aid: i for i, aid in enumerate(anchor_ids)}

    # Build symmetric distance matrix
    D = np.zeros((N, N))
    for p in pairs:
        if p['src'] in id_to_idx and p['dst'] in id_to_idx:
            i = id_to_idx[p['src']]
            j = id_to_idx[p['dst']]
            D[i, j] = p['dist_m']
            D[j, i] = p['dist_m']

    # Warn about missing pairs
    missing = []
    for i in range(N):
        for j in range(i + 1, N):
            if D[i, j] == 0.0:
                missing.append((anchor_ids[i], anchor_ids[j]))
    if missing:
        pairs_str = ', '.join(f'({a},{b})' for a, b in missing)
        print(f"WARNING: missing pairs — {pairs_str}")
        print("  MDS result will be degraded.")

    # Classical MDS
    X = classical_mds(D, dim)

    # Canonical frame
    X = fix_frame(X)

    # Pad to 2 columns minimum
    if X.shape[1] < 2:
        X = np.hstack([X, np.zeros((N, 2 - X.shape[1]))])

    # Report
    print("\nAnchor positions (MDS result):")
    for i, aid in enumerate(anchor_ids):
        if dim == 2:
            print(f"  Anchor 0x{aid:02X}  id={aid} : "
                  f"x={X[i,0]:.3f}  y={X[i,1]:.3f}  z={z_height:.3f} m")
        else:
            z = float(X[i, 2]) if X.shape[1] > 2 else z_height
            print(f"  Anchor 0x{aid:02X}  id={aid} : "
                  f"x={X[i,0]:.3f}  y={X[i,1]:.3f}  z={z:.3f} m")

    # Auto-bounds (20 % pad)
    pad = 0.5
    xs, ys = X[:, 0], X[:, 1]
    bounds = [float(xs.min()) - pad, float(xs.max()) + pad,
              float(ys.min()) - pad, float(ys.max()) + pad,
              0.0, 3.0]

    # Build output dict
    anchors_list = []
    for i, aid in enumerate(anchor_ids):
        z = z_height if dim == 2 else (float(X[i, 2]) if X.shape[1] > 2 else z_height)
        anchors_list.append({
            'id': aid,
            'x':  round(float(X[i, 0]), 6),
            'y':  round(float(X[i, 1]), 6),
            'z':  round(float(z), 6),
        })

    out = {
        'dim':     dim,
        'bounds':  [round(v, 4) for v in bounds],
        'anchors': anchors_list,
        'survey_timestamp': datetime.now().isoformat(timespec='seconds'),
    }

    # Write
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    out_path = CONFIG_DIR / 'anchors.json'
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)
        f.write('\n')

    print(f"\nWritten to {out_path}")
    print("Run run_localization.py to start localization with the new anchor map.")
    return out


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='UWB Anchor Self-Survey — Python equivalent of runSurvey.m')
    p.add_argument('--port',    required=True,
                   help='Serial port connected to the tag (e.g. /dev/ttyUSB0 or COM5)')
    p.add_argument('--baud',    type=int, default=115200,
                   help='Serial baud rate (default: 115200)')
    p.add_argument('--anchors', default='1,2,3',
                   help='Comma-separated anchor IDs, must match ANCHORS[] in Tag.ino '
                        '(default: 1,2,3)')
    p.add_argument('--dim',     type=int, default=2, choices=[2, 3],
                   help='Solve in 2D or 3D (default: 2)')
    p.add_argument('--z',       type=float, default=1.5,
                   help='Anchor height above floor in metres, applied in 2D mode '
                        '(default: 1.5)')
    p.add_argument('--timeout', type=float, default=120.0,
                   help='Max seconds to wait for all pairs (default: 120)')
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()
    anchor_ids = [int(x.strip()) for x in args.anchors.split(',')]

    print("=" * 60)
    print("UWB Anchor Self-Survey")
    print(f"  Port    : {args.port} @ {args.baud} baud")
    print(f"  Anchors : {anchor_ids}")
    print(f"  Dim     : {args.dim}D   Z_height: {args.z} m")
    print(f"  Timeout : {args.timeout} s")
    print("=" * 60)

    pairs = run_survey(
        port=args.port,
        baud=args.baud,
        anchor_ids=anchor_ids,
        dim=args.dim,
        z_height=args.z,
        timeout_s=args.timeout,
        samples_override=None,
    )

    compute_and_write(pairs, anchor_ids, args.dim, args.z)
