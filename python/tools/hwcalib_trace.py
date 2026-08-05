#!/usr/bin/env python3
"""
hwcalib_trace.py — dual-serial capture + merged timeline for HWCALIB root-cause
investigation (see the plan: instrumentation only, no fix).

Standalone diagnostic tool. Does NOT import or touch run_localization.py, dune/,
or any part of the live RTLS pipeline — safe to run alongside (or instead of) the
host, with zero risk to the production path.

Opens up to two serial ports at once — the calibrating tag, and (optionally) the
anchor under test wired via its own USB cable for the duration of one test — each
in its own reader thread. Every line is timestamped the instant it's read from the
OS serial buffer, using ONE shared host clock (time.perf_counter()). This sidesteps
cross-board clock drift entirely: the two boards' millis()/micros() are never
compared to each other, only the host's single monotonic clock orders events. USB
latency (a few ms at most) is negligible against the ~2000 ms timeout windows the
calibration path uses, so host-arrival order is an accurate proxy for wire order.

Usage:
    python3 hwcalib_trace.py --tag-port /dev/ttyUSB0 --anchor-port /dev/ttyUSB1
    python3 hwcalib_trace.py --tag-port /dev/ttyUSB0                # tag-only pass
    python3 hwcalib_trace.py --tag-port COM5 --anchor-port COM6 --out trace.log

Output: a live-printed, time-ordered merged log (also written to --out) like:
    [+0.000s] TAG     [CAL-TX] anchor=0x03 attempt=1/3 seq=12 delay=16420 ... sending...
    [+0.041s] ANCH-03 [CAL-RX] MSG_ANT_DELAY received: newDelay=16420 src=0xF0 seq=12 ...
    [+0.043s] ANCH-03 [CAL] Antenna delay updated: 16420 (from 0xF0)  configure+startRx took 1850 us
    [+0.044s] ANCH-03 [CAL-RX] ACK sent-OK seq=12 ...
    [+2.041s] TAG     [CAL-TX] anchor=0x03 seq=12 NO-REPLY (waitReceived(2000) timeout ...) ...
"""

import argparse
import queue
import sys
import threading
import time
from pathlib import Path

try:
    import serial as pyserial
except ImportError:
    pyserial = None


def reader_thread(port: str, baud: int, label: str, q: "queue.Queue", stop: threading.Event):
    """Read lines from one serial port, timestamp on arrival, push to the shared queue."""
    if pyserial is None:
        q.put((time.perf_counter(), label, "ERROR: pip install pyserial"))
        return
    try:
        ser = pyserial.Serial(port, baud, timeout=0.2)
    except Exception as e:
        q.put((time.perf_counter(), label, f"ERROR: could not open {port}: {e}"))
        return
    with ser:
        while not stop.is_set():
            try:
                raw = ser.readline()
            except Exception as e:
                q.put((time.perf_counter(), label, f"ERROR: read failed: {e}"))
                break
            if not raw:
                continue   # readline() timeout with no data — keep polling
            line = raw.decode('utf-8', errors='replace').rstrip('\r\n')
            if line:
                q.put((time.perf_counter(), label, line))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--tag-port', required=True, help='Serial port for the calibrating tag')
    p.add_argument('--anchor-port', default=None,
                    help='Serial port for the anchor under test (optional — tag-only pass if omitted)')
    p.add_argument('--anchor-label', default='ANCHOR',
                    help='Label for the anchor stream in the merged log, e.g. A3 (default: ANCHOR)')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--out', default=None,
                    help='Output log file (default: hwcalib_trace_<timestamp>.log)')
    args = p.parse_args()

    if pyserial is None:
        print("ERROR: pyserial is required — pip install pyserial")
        sys.exit(1)

    out_path = Path(args.out) if args.out else Path(f"hwcalib_trace_{time.strftime('%Y%m%d_%H%M%S')}.log")
    q: "queue.Queue" = queue.Queue()
    stop = threading.Event()

    threads = [threading.Thread(target=reader_thread,
                                 args=(args.tag_port, args.baud, "TAG", q, stop), daemon=True)]
    if args.anchor_port:
        threads.append(threading.Thread(target=reader_thread,
                                         args=(args.anchor_port, args.baud, args.anchor_label, q, stop),
                                         daemon=True))
    for t in threads:
        t.start()

    print(f"[hwcalib_trace] tag={args.tag_port}"
          + (f" anchor={args.anchor_port}" if args.anchor_port else " (tag-only)")
          + f" baud={args.baud}")
    print(f"[hwcalib_trace] writing merged trace to {out_path}")
    print("[hwcalib_trace] Ctrl-C to stop\n")

    t0 = None
    try:
        with open(out_path, 'w') as f:
            while True:
                try:
                    ts, label, line = q.get(timeout=0.5)
                except queue.Empty:
                    continue
                if t0 is None:
                    t0 = ts
                out_line = f"[+{ts - t0:7.3f}s] {label:<8}{line}"
                print(out_line)
                f.write(out_line + "\n")
                f.flush()
    except KeyboardInterrupt:
        print("\n[hwcalib_trace] stopped.")
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=1.0)


if __name__ == '__main__':
    main()
