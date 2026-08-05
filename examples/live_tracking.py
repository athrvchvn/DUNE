#!/usr/bin/env python3
"""A live tag position, in about forty lines.

`run_localization.py` is the real host — GUI, jobs, capture, multi-tag state,
reconnection. This is the same measurement path with all of that stripped away,
so you can see what it costs to get a position out of DUNE: read a line, parse
it, solve it.

    python examples/live_tracking.py                    # UDP :4100
    python examples/live_tracking.py --serial /dev/ttyUSB0

Ctrl-C to stop.
"""

import argparse
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'python'))

import numpy as np                                      # noqa: E402

from dune.anchor_config import AnchorConfig             # noqa: E402
from dune.frame_parser import FrameParser               # noqa: E402
from dune.paths import config                           # noqa: E402
from dune.range_correction import RangeCorrection       # noqa: E402
from dune.solve_sweep import solve_sweep                # noqa: E402


def lines_from_udp(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    print(f"listening on UDP :{port}")
    while True:
        data, _ = sock.recvfrom(4096)
        # One datagram can carry several lines — the dual-tag master forwards
        # both tags' lines, and they can share a packet.
        for line in data.decode(errors='replace').splitlines():
            yield line


def lines_from_serial(port, baud):
    import serial
    ser = serial.Serial(port, baud, timeout=1.0)
    print(f"reading {port} at {baud}")
    while True:
        line = ser.readline().decode(errors='replace').strip()
        if line:
            yield line


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--serial')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--udp-port', type=int, default=4100)
    ap.add_argument('--anchors', default=str(config('anchors.json')))
    ap.add_argument('--tag-z', type=float, default=0.24)
    args = ap.parse_args()

    if not Path(args.anchors).exists():
        print(f"No anchor geometry at {args.anchors}.\n"
              f"Copy config/anchors.example.json to config/anchors.json and edit it —\n"
              f"see docs/getting_started.md.")
        return 1

    anchors = AnchorConfig.from_json(args.anchors)
    corr = RangeCorrection.load()          # None if not fitted yet; that is fine
    print(f"{len(anchors.ids)} anchors, Tier-2 {'loaded' if corr else 'absent'}\n")

    source = (lines_from_serial(args.serial, args.baud) if args.serial
              else lines_from_udp(args.udp_port))

    last = {}
    t0 = time.time()
    n = 0
    try:
        for line in source:
            i = line.find('RTLS,')
            if i < 0:
                continue
            pkt = FrameParser.parse(line[i:])
            if pkt is None:
                continue

            # Warm-starting from this tag's previous fix is what makes the solve
            # a few iterations instead of a search.
            sol = solve_sweep(pkt, anchors, range_corr=corr, tag_z=args.tag_z,
                              x0=last.get(pkt.tag_id))
            if sol is None or not np.all(np.isfinite(sol.pos)):
                print(f"tag {pkt.tag_id}  no fix "
                      f"({len(pkt.ids)} anchors responded)")
                continue

            last[pkt.tag_id] = sol.pos
            n += 1
            print(f"tag {pkt.tag_id}  x={sol.pos[0]:7.3f}  y={sol.pos[1]:7.3f} m   "
                  f"resid {sol.info.rmse * 1000:5.1f} mm   "
                  f"{int(sol.info.used.sum())}/{len(pkt.ids)} anchors used   "
                  f"{n / max(time.time() - t0, 1e-6):4.1f} Hz")
    except KeyboardInterrupt:
        print(f"\n{n} fixes in {time.time() - t0:.1f} s")
    return 0


if __name__ == '__main__':
    sys.exit(main())
