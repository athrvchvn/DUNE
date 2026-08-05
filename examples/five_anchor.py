#!/usr/bin/env python3
"""Solve a synthetic five-anchor geometry — the smallest useful thing DUNE does.

No hardware, no logs, no configuration. Run it on a fresh clone to confirm the
install works and to see what a fix actually costs in accuracy.

    python examples/five_anchor.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))

import numpy as np                                      # noqa: E402

from dune.multilaterate import multilaterate            # noqa: E402

# Five anchors around a 5 x 3 m arena at 0.48 m — the reported deployment's shape.
ANCHORS = np.array([
    [0.0, 0.0, 0.48],
    [5.0, 0.0, 0.48],
    [5.0, 3.0, 0.48],
    [0.0, 3.0, 0.48],
    [2.5, 3.0, 0.48],
])
TAG_Z = 0.48        # co-planar with the anchors: the vertical term vanishes exactly


def ranges_to(p_xy, noise_m=0.0, rng=None):
    """True ranges from a planar position to every anchor, optionally noisy."""
    d = ANCHORS[:, :2] - np.asarray(p_xy, float)
    r = np.hypot(np.hypot(d[:, 0], d[:, 1]), ANCHORS[:, 2] - TAG_Z)
    if noise_m and rng is not None:
        r = r + rng.randn(len(r)) * noise_m
    return r


def spread(pos, sigma_m, rng, n=200):
    """Median and 95th-percentile error over n noisy solves, in mm."""
    errs = []
    for _ in range(n):
        p, _ = multilaterate(ANCHORS, ranges_to(pos, sigma_m, rng), tag_z=TAG_Z)
        if np.all(np.isfinite(p)):
            errs.append(np.linalg.norm(p - np.asarray(pos, float)))
    e = np.array(errs) * 1000
    return np.median(e), np.percentile(e, 95), len(errs)


def main():
    rng = np.random.RandomState(0)
    truth = np.array([2.0, 1.5])

    print(f"{len(ANCHORS)} anchors, tag at {truth} m, height {TAG_Z} m\n")

    # 1. Noise-free — exact to solver tolerance. If this is not ~0, something is wrong.
    p, info = multilaterate(ANCHORS, ranges_to(truth), tag_z=TAG_Z)
    print(f"  noise-free       {p}   error {np.linalg.norm(p - truth) * 1000:6.3f} mm"
          f"   ({info.iters} LM iterations)")

    # 2. Realistic per-range noise, repeated — one sample would demonstrate nothing.
    print("\n  per-range noise -> position error:")
    for sigma_mm in (10, 30, 100):
        med, p95, n = spread(truth, sigma_mm / 1000, rng)
        print(f"    sigma {sigma_mm:4d} mm    median {med:6.1f} mm    95th {p95:6.1f} mm")

    # 3. Geometry matters, and it degrades with distance rather than falling off a
    #    cliff at the hull boundary. Identical 30 mm ranges give a steadily worse
    #    fix as the anchors subtend a narrower angle from where you stand.
    print("\n  same 30 mm range noise, different places relative to the arena:")
    for name, pos in [("centre", [2.5, 1.5]),
                      ("corner", [0.3, 0.3]),
                      ("1.5 m outside", [6.5, 1.5]),
                      ("5 m outside", [10.0, 1.5]),
                      ("10 m outside", [15.0, 1.5])]:
        med, p95, n = spread(pos, 0.030, rng, n=400)
        print(f"    {name:15s} median {med:6.1f} mm    95th {p95:6.1f} mm")

    print("\n  Note it degrades smoothly, not abruptly at the hull edge — just outside is")
    print("  barely worse than the corner. It is the *angle* the anchors subtend that")
    print("  matters, so error grows with distance from the constellation. Compare with")
    print("  the noise table above: at 10 m outside, 30 mm ranges give what 100 mm ranges")
    print("  give in the middle. Anchor placement buys more than range precision does.")

    # 4. Anchors drop out. The solver degrades rather than failing — but a changing
    #    responding subset shifts the fix even when every range is good, which is
    #    what range hold exists to smooth. See docs/estimators.md.
    print("\n  dropping anchors (NaN marks one absent this sweep, 30 mm noise):")
    for drop in ([], [4], [4, 3], [4, 3, 2]):
        errs = []
        for _ in range(200):
            z = ranges_to(truth, 0.030, rng)
            z[drop] = np.nan
            p, _ = multilaterate(ANCHORS, z, tag_z=TAG_Z)
            if np.all(np.isfinite(p)):
                errs.append(np.linalg.norm(p - truth))
        label = f"{len(ANCHORS) - len(drop)} anchors"
        if errs:
            print(f"    {label:12s} median {np.median(np.array(errs)) * 1000:6.1f} mm"
                  f"   solved {len(errs)}/200")
        else:
            print(f"    {label:12s} no solution (as it should be)")


if __name__ == '__main__':
    main()
