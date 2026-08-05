#!/usr/bin/env python3
"""Parse a recorded session and solve every sweep — the smallest end-to-end path.

This is what the live host does, minus the transport and the GUI: read wire
lines, apply the Tier-2 correction, weight by the NLOS gap, solve. The identical
`solve_sweep` call runs live, which is why offline replay is a faithful
reproduction rather than an approximation.

    python examples/replay_log.py                    # bundled example log
    python examples/replay_log.py path/to/serial_raw.log
    python examples/replay_log.py path/to/log.jsonl --anchors config/anchors.json
"""

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'python'))

import numpy as np                                      # noqa: E402

from dune.anchor_config import AnchorConfig             # noqa: E402
from dune.frame_parser import FrameParser               # noqa: E402
from dune.range_correction import RangeCorrection       # noqa: E402
from dune.solve_sweep import solve_sweep                # noqa: E402

DEFAULT_LOG = REPO / 'examples' / 'logs' / 'example_session.log'
DEFAULT_ANCHORS = REPO / 'config' / 'anchors_kinect_clicked_20260724.json'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', nargs='?', default=str(DEFAULT_LOG))
    ap.add_argument('--anchors', default=str(DEFAULT_ANCHORS))
    ap.add_argument('--tag-z', type=float, default=0.24)
    ap.add_argument('--no-correction', action='store_true',
                    help="skip the Tier-2 correction, to see what it is worth")
    args = ap.parse_args()

    log = Path(args.log)
    if not log.exists():
        used_default = (str(log) == str(DEFAULT_LOG))
        print(f"No log at {log}.\n")
        if used_default:
            # Expected on a fresh clone: the bundled log has not been added yet.
            # Not an error — say so and exit cleanly, so "all examples run" stays
            # a check you can actually run.
            print("The bundled example log has not been added yet — see\n"
                  "  examples/logs/README.md   and   docs/datasets.md\n\n"
                  "Point this at any capture you have:\n"
                  "  python examples/replay_log.py path/to/serial_raw.log\n\n"
                  "Or run examples/five_anchor.py, which needs no data at all.")
            return 0
        print("Check the path.")
        return 1

    anchors = AnchorConfig.from_json(args.anchors)
    corr = None if args.no_correction else RangeCorrection.load()
    print(f"log     {log.name}")
    print(f"anchors {Path(args.anchors).name}  ({len(anchors.ids)} anchors)")
    print(f"Tier-2  {'disabled' if corr is None else 'loaded'}\n")

    per_tag: dict[int, list] = {}
    n_lines = n_parsed = 0

    for raw in log.read_text(errors='replace').splitlines():
        i = raw.find('RTLS,')
        if i < 0:
            continue
        n_lines += 1
        pkt = FrameParser.parse(raw[i:])
        if pkt is None:
            continue
        n_parsed += 1

        sol = solve_sweep(pkt, anchors, range_corr=corr, tag_z=args.tag_z)
        if sol is not None and np.all(np.isfinite(sol.pos)):
            per_tag.setdefault(pkt.tag_id, []).append((sol.pos, sol.info.rmse))

    print(f"{n_lines} RTLS lines, {n_parsed} parsed\n")
    if not per_tag:
        print("No sweeps solved. Wrong anchors file for this capture?")
        return 1

    for tag_id, rows in sorted(per_tag.items()):
        pos = np.array([r[0] for r in rows])
        rmse = np.array([r[1] for r in rows]) * 1000
        print(f"  tag {tag_id}")
        print(f"    solved sweeps    {len(rows)}")
        print(f"    residual RMSE    median {np.median(rmse):.1f} mm, "
              f"95th {np.percentile(rmse, 95):.1f} mm")
        print(f"    x  {pos[:, 0].min():6.2f} .. {pos[:, 0].max():6.2f} m")
        print(f"    y  {pos[:, 1].min():6.2f} .. {pos[:, 1].max():6.2f} m")

    print("\n  Residual RMSE is how well the ranges agree with each other given the")
    print("  geometry — not distance from truth. Scoring against truth needs")
    print("  reports.rover. See docs/python_pipeline.md.")

    worst = max(np.median([r[1] for r in rows]) for rows in per_tag.values()) * 1000
    if worst > 300:
        print(f"\n  ⚠  Residual is high ({worst:.0f} mm). The ranges disagree with each")
        print("     other by far more than the radio's noise, which almost always means")
        print("     the anchors file does not describe the constellation this log was")
        print("     recorded in. Try --anchors with the layout from that session's era.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
