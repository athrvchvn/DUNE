#!/usr/bin/env python3
"""Degenerate geometry — and why DUNE refuses to solve it.

Three anchors is the minimum for a planar fix. What matters is not the count but
the *shape*: a near-collinear set is mirror-ambiguous, and two reflected
positions fit the ranges equally well. Returning one of them at random looks like
a working system right up until the robot drives into a wall.

So the solver refuses. This shows the refusal firing.

    python examples/three_anchor.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))

import numpy as np                                      # noqa: E402

from dune.multilaterate import multilaterate            # noqa: E402

TAG_Z = 0.48


def ranges(anchors, p_xy, noise_m=0.0, rng=None):
    d = anchors[:, :2] - np.asarray(p_xy, float)
    r = np.hypot(np.hypot(d[:, 0], d[:, 1]), anchors[:, 2] - TAG_Z)
    if noise_m and rng is not None:
        r = r + rng.randn(len(r)) * noise_m
    return r


def trial(name, anchors, truth, rng, sigma=0.030, n=200):
    errs, failed = [], 0
    for _ in range(n):
        p, _ = multilaterate(anchors, ranges(anchors, truth, sigma, rng), tag_z=TAG_Z)
        if np.all(np.isfinite(p)):
            errs.append(np.linalg.norm(p - np.asarray(truth, float)))
        else:
            failed += 1
    if errs:
        print(f"  {name:26s} median {np.median(np.array(errs)) * 1000:7.1f} mm"
              f"    refused {failed:3d}/{n}")
    else:
        print(f"  {name:26s} {'no solution':>14s}    refused {failed:3d}/{n}")


def main():
    rng = np.random.RandomState(0)
    truth = [2.0, 1.5]

    print("Three anchors, 30 mm range noise, tag at [2.0, 1.5] m\n")

    # A well-spread triangle: fine.
    good = np.array([[0.0, 0.0, 0.48], [5.0, 0.0, 0.48], [2.5, 3.0, 0.48]])
    trial("well-spread triangle", good, truth, rng)

    # Squashing the triangle toward a line. The anchors are still three distinct
    # points, and a naive solver would happily return something.
    for y in (2.0, 1.0, 0.5, 0.2, 0.05, 0.0):
        anchors = np.array([[0.0, 0.0, 0.48], [5.0, 0.0, 0.48], [2.5, y, 0.48]])
        trial(f"apex {y:.2f} m off the line", anchors, truth, rng)

    print("\n  As the triangle flattens, the reflection about the anchor line becomes an")
    print("  equally good fit. DUNE applies a singular-value test to the centred anchor")
    print("  positions and returns no position rather than a coin flip.")
    print("\n  Read the middle row carefully. The guard is a threshold, so there is a band")
    print("  just above it where the geometry is bad enough to give metre-scale errors but")
    print("  not bad enough to be refused. The refusal catches the hopeless case; it is not")
    print("  a substitute for placing anchors well.")
    print("\n  Four anchors do not save you either — what matters is the shape:")

    line4 = np.array([[0.0, 0.0, 0.48], [1.7, 0.0, 0.48],
                      [3.3, 0.0, 0.48], [5.0, 0.0, 0.48]])
    trial("4 anchors, all collinear", line4, truth, rng)

    box4 = np.array([[0.0, 0.0, 0.48], [5.0, 0.0, 0.48],
                     [5.0, 3.0, 0.48], [0.0, 3.0, 0.48]])
    trial("4 anchors, rectangle", box4, truth, rng)

    print("\n  Practical consequence: spread anchors in two dimensions. A row of")
    print("  anchors along one wall of a corridor cannot localize across it.")


if __name__ == '__main__':
    main()
