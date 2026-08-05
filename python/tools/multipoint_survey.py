#!/usr/bin/env python3
"""
multipoint_survey.py — Tag-mediated anchor position survey.

Instead of anchor-to-anchor ranging (which fails at distance), the user
places the tag at several known grid positions and captures averaged
distances to each anchor.  An optimizer jointly solves for anchor AND
temporary tag positions, then keeps only the anchor coordinates.

Usage (standalone self-test):
    python multipoint_survey.py

Called from run_localization.py:
    from multipoint_survey import solve_anchor_positions, write_anchors_json
"""

import sys
import json
import math
import numpy as np
from pathlib import Path
from datetime import datetime

import warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore", UserWarning)
    try:
        from scipy.optimize import least_squares
    except ImportError:
        print("Missing dependency: pip install scipy")
        sys.exit(1)


SCRIPT_DIR = Path(__file__).resolve().parent
# Running as a script from anywhere: make `python/` importable.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dune.paths import CONFIG_DIR

# 3×3 calibration grid for 4×4 m workspace (anchors at corners ±2 m)
SURVEY_GRID = [
    (-1.5,  1.5), ( 0.0,  1.5), ( 1.5,  1.5),
    (-1.5,  0.0), ( 0.0,  0.0), ( 1.5,  0.0),
    (-1.5, -1.5), ( 0.0, -1.5), ( 1.5, -1.5),
]


# ── Optimizer ────────────────────────────────────────────────────────────────

def solve_anchor_positions(
    captures: list[dict],
    anchor_ids: list[int],
    dim: int = 2,
) -> tuple[np.ndarray, dict]:
    """
    Jointly solve for N anchor positions given K tag-position captures.

    Parameters
    ----------
    captures : list of dict
        Each dict has:
            'dists': {anchor_id: averaged_distance_m, ...}
        Anchors not seen in a capture are skipped.
    anchor_ids : list of int
        Ordered anchor IDs (e.g. [1, 2, 3]).
    dim : int
        Solve in 2D (default) or 3D.

    Returns
    -------
    positions : np.ndarray, shape (N, dim)
        Anchor positions in canonical frame (anchor 0 at origin, anchor 1 on +X).
    info : dict
        'residual_rms': float, 'n_captures': int, 'n_anchors': int
    """
    N = len(anchor_ids)
    K = len(captures)
    aid_to_idx = {aid: i for i, aid in enumerate(anchor_ids)}

    # Build observation list: (capture_idx, anchor_idx, distance)
    obs = []
    for ci, cap in enumerate(captures):
        for aid, dist in cap['dists'].items():
            aid_int = int(aid)
            if aid_int in aid_to_idx and dist > 0:
                obs.append((ci, aid_to_idx[aid_int], float(dist)))

    if len(obs) < N + K:
        raise ValueError(
            f"Not enough observations ({len(obs)}) for {N} anchors + {K} captures. "
            f"Need at least {N + K}."
        )

    # ── Initial guess ────────────────────────────────────────────────────
    # Use average distances to seed rough anchor positions.
    # Place anchors in an equilateral triangle as a starting guess,
    # then tag positions at the centroid.

    # Estimate pairwise anchor distances from triangulation:
    # For each pair of captures, the anchor distances give geometric constraints.
    # Simple seed: equilateral triangle scaled by median distance.
    median_dist = np.median([d for _, _, d in obs])
    anchor_radius = median_dist  # rough scale

    # Seed anchors in an equilateral polygon
    x0_anchors = np.zeros((N, dim))
    for i in range(N):
        angle = 2 * math.pi * i / N + math.pi / 2  # start at top
        x0_anchors[i, 0] = anchor_radius * math.cos(angle)
        x0_anchors[i, 1] = anchor_radius * math.sin(angle)

    # Seed tag positions at centroid
    x0_tags = np.zeros((K, dim))
    centroid = x0_anchors.mean(axis=0)
    for ci in range(K):
        x0_tags[ci] = centroid

    # ── Pack into parameter vector ───────────────────────────────────────
    # Parameters: [anchor_free_coords..., tag_coords...]
    #
    # To remove the 3-DOF ambiguity (translation + rotation + reflection):
    #   - Anchor 0 is fixed at origin → 0 free params
    #   - Anchor 1 is fixed on +X axis → 1 free param (its x-coordinate)
    #   - All other anchors → dim free params each
    #   - All tag positions → dim free params each
    #
    # Total free params = 1 + (N-2)*dim + K*dim

    def pack(a_pos, t_pos):
        """Pack anchor and tag positions into the free parameter vector."""
        parts = []
        # Anchor 1: only x-coordinate is free (y=0 forced)
        if N >= 2:
            parts.append(a_pos[1, 0:1])  # just x
        # Anchors 2..N-1: all coords free
        for i in range(2, N):
            parts.append(a_pos[i, :dim])
        # All tag positions
        for ci in range(K):
            parts.append(t_pos[ci, :dim])
        return np.concatenate(parts) if parts else np.array([])

    def unpack(p):
        """Unpack the free parameter vector into anchor and tag positions."""
        a_pos = np.zeros((N, dim))
        # Anchor 0: fixed at origin (all zeros)
        idx = 0
        # Anchor 1: x from params, y=0
        if N >= 2:
            a_pos[1, 0] = p[idx]
            idx += 1
        # Anchors 2..N-1: full coords
        for i in range(2, N):
            a_pos[i, :dim] = p[idx:idx+dim]
            idx += dim
        # Tag positions
        t_pos = np.zeros((K, dim))
        for ci in range(K):
            t_pos[ci, :dim] = p[idx:idx+dim]
            idx += dim
        return a_pos, t_pos

    p0 = pack(x0_anchors, x0_tags)

    # ── Residual function ────────────────────────────────────────────────
    def residuals(p):
        a_pos, t_pos = unpack(p)
        r = np.empty(len(obs))
        for k, (ci, ai, d_meas) in enumerate(obs):
            d_model = np.linalg.norm(t_pos[ci] - a_pos[ai])
            r[k] = d_model - d_meas
        return r

    # ── Solve ────────────────────────────────────────────────────────────
    result = least_squares(
        residuals, p0,
        method='trf',
        loss='soft_l1',    # robust to outlier measurements
        f_scale=0.10,      # ~10cm expected noise
        max_nfev=5000,
    )

    a_final, t_final = unpack(result.x)

    # ── Canonical frame ──────────────────────────────────────────────────
    # Already mostly canonical (anchor 0 at origin, anchor 1 on +X),
    # but enforce anchor 1 on +X (flip if negative) and anchor 2 in +Y.
    if N >= 2 and a_final[1, 0] < 0:
        a_final[:, 0] = -a_final[:, 0]
    if N >= 3 and dim >= 2 and a_final[2, 1] < 0:
        a_final[:, 1] = -a_final[:, 1]

    # ── Info ─────────────────────────────────────────────────────────────
    r_final = residuals(result.x)
    rms = float(np.sqrt(np.mean(r_final ** 2)))

    info = {
        'residual_rms': rms,
        'n_captures': K,
        'n_anchors': N,
        'cost': float(result.cost),
        'success': bool(result.success),
        'message': str(result.message),
    }

    return a_final[:, :dim], info


# ── JSON writer ──────────────────────────────────────────────────────────────

def write_anchors_json(
    positions: np.ndarray,
    anchor_ids: list[int],
    dim: int = 2,
    z_height: float = 0.0,
    path: Path | str | None = None,
) -> dict:
    """Write computed anchor positions to anchors.json."""
    if path is None:
        path = CONFIG_DIR / 'anchors.json'
    path = Path(path)

    N = len(anchor_ids)
    xs = positions[:, 0]
    ys = positions[:, 1] if positions.shape[1] >= 2 else np.zeros(N)

    pad = 0.5
    bounds = [
        float(xs.min()) - pad, float(xs.max()) + pad,
        float(ys.min()) - pad, float(ys.max()) + pad,
        0.0, 3.0,
    ]

    anchors_list = []
    for i, aid in enumerate(anchor_ids):
        z = z_height if dim == 2 else (float(positions[i, 2]) if positions.shape[1] > 2 else z_height)
        anchors_list.append({
            'id': aid,
            'x': round(float(positions[i, 0]), 4),
            'y': round(float(positions[i, 1]), 4),
            'z': round(float(z), 4),
        })

    out = {
        'dim': dim,
        'bounds': [round(v, 4) for v in bounds],
        'anchors': anchors_list,
        'survey_timestamp': datetime.now().isoformat(timespec='seconds'),
        'survey_method': 'multipoint_tag_mediated',
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(out, f, indent=2)
        f.write('\n')

    print(f"[SURVEY] Written anchors.json → {path}")
    return out


def update_anchor_json(
    anchor_id: int,
    x: float,
    y: float,
    z: float = 0.0,
    path: Path | str | None = None,
    apriltag_id: int | None = None,
) -> dict:
    """Upsert one anchor's position into anchors.json, preserving the rest.

    Used by the Kinect click-to-place workflow: unlike write_anchors_json
    (which replaces the whole anchor list from a joint RF-survey solve), this
    updates a single anchor in place — or appends it if the id is new —
    without touching any other anchor. Creates the file with sensible
    defaults if it doesn't exist yet.

    apriltag_id (optional): provenance for the auto-detect path — which
    physical AprilTag ID produced this position, stored on the anchor entry
    so a later detect can flag "anchor 2 came from a different tag than last
    time". Omitted entirely (not even a null) for the manual click-to-place
    path, which has no such ID.
    """
    if path is None:
        path = CONFIG_DIR / 'anchors.json'
    path = Path(path)

    if path.exists():
        with open(path) as f:
            doc = json.load(f)
    else:
        doc = {'dim': 2, 'bounds': [-5.0, 5.0, -5.0, 5.0, 0.0, 3.0], 'anchors': []}

    anchors = doc.setdefault('anchors', [])
    for a in anchors:
        if a['id'] == anchor_id:
            a['x'], a['y'], a['z'] = round(x, 4), round(y, 4), round(z, 4)
            if apriltag_id is not None:
                a['apriltag_id'] = apriltag_id
            break
    else:
        entry = {'id': anchor_id, 'x': round(x, 4), 'y': round(y, 4), 'z': round(z, 4)}
        if apriltag_id is not None:
            entry['apriltag_id'] = apriltag_id
        anchors.append(entry)

    doc['vision_updated_at'] = datetime.now().isoformat(timespec='seconds')

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(doc, f, indent=2)
        f.write('\n')

    print(f"[VISION] Anchor {anchor_id} -> ({x:.3f}, {y:.3f}, {z:.3f})  written to {path}")
    return doc


# ── Self-test ────────────────────────────────────────────────────────────────

def _self_test():
    """
    Synthetic test: place 4 anchors at known positions, simulate tag
    captures at calibration grid points with noise, verify recovery.
    """
    np.random.seed(42)

    # True anchor positions (4-anchor square, 4 m side, centred at origin)
    true_anchors = np.array([
        [-2.0, -2.0],
        [ 2.0, -2.0],
        [ 2.0,  2.0],
        [-2.0,  2.0],
    ])
    anchor_ids = [1, 2, 3, 4]
    N = len(anchor_ids)

    # Use calibration grid as tag positions
    tag_positions = np.array(SURVEY_GRID)
    K = len(tag_positions)

    # Simulate measurements with noise
    noise_sigma = 0.05  # 5cm noise
    captures = []
    for ci in range(K):
        dists = {}
        for ai, aid in enumerate(anchor_ids):
            true_dist = np.linalg.norm(tag_positions[ci] - true_anchors[ai])
            noisy_dist = true_dist + np.random.randn() * noise_sigma
            dists[aid] = max(0.01, noisy_dist)
        captures.append({'dists': dists})

    print("=" * 60)
    print("Multi-Point Survey — Synthetic Self-Test")
    print(f"  Anchors : {N}")
    print(f"  Captures: {K}")
    print(f"  Noise σ : {noise_sigma*100:.0f} cm")
    print("=" * 60)

    positions, info = solve_anchor_positions(captures, anchor_ids, dim=2)

    print(f"\nOptimizer: success={info['success']}  "
          f"residual_rms={info['residual_rms']:.4f} m  "
          f"cost={info['cost']:.6f}")
    print(f"  {info['message']}")

    # Transform true anchors to the same canonical frame for comparison:
    # anchor 0 at origin, anchor 1 on +X axis, anchor 2 in +Y half-plane
    true_canon = true_anchors.copy()
    true_canon -= true_canon[0]  # anchor 0 → origin
    if np.linalg.norm(true_canon[1]) > 1e-6:
        theta = math.atan2(float(true_canon[1, 1]), float(true_canon[1, 0]))
        c, s = math.cos(-theta), math.sin(-theta)
        R = np.array([[c, -s], [s, c]])
        true_canon = (R @ true_canon.T).T
    if true_canon.shape[0] >= 3 and true_canon[2, 1] < 0:
        true_canon[:, 1] = -true_canon[:, 1]

    print("\nRecovered vs. True anchor positions (canonical frame):")
    max_err = 0.0
    for i, aid in enumerate(anchor_ids):
        err = np.linalg.norm(positions[i] - true_canon[i])
        max_err = max(max_err, err)
        print(f"  A{aid}:  recovered=({positions[i,0]:+.3f}, {positions[i,1]:+.3f})  "
              f"true=({true_canon[i,0]:+.3f}, {true_canon[i,1]:+.3f})  "
              f"error={err*100:.1f} cm")

    # Test JSON writing
    out = write_anchors_json(positions, anchor_ids, dim=2, z_height=0.0,
                              path=SCRIPT_DIR / '_test_anchors.json')
    # Clean up test file
    test_path = SCRIPT_DIR / '_test_anchors.json'
    if test_path.exists():
        test_path.unlink()
        print(f"  (test file cleaned up)")

    print(f"\n{'PASS' if max_err < 0.15 else 'FAIL'}: "
          f"max anchor error = {max_err*100:.1f} cm "
          f"(threshold: 15 cm)")

    return max_err < 0.15


if __name__ == '__main__':
    ok = _self_test()
    sys.exit(0 if ok else 1)
