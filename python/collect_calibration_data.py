#!/usr/bin/env python3
"""
collect_calibration_data.py — Capture per-anchor ranging data for bias characterisation.

Supports UDP (WiFi) or serial (USB) transport.  Filters for one anchor ID and
writes raw measurements to CSV.

Usage:
    # UDP (WiFi) — tag must have HOST_IP set to this machine
    python collect_calibration_data.py --anchor 1 --samples 500

    # Serial (USB) — no WiFi needed, more reliable for lab work
    python collect_calibration_data.py --anchor 1 --samples 500 --serial /dev/ttyUSB0

    # Custom output file
    python collect_calibration_data.py --anchor 1 --output my_run.csv

CSV columns:
    session, true_distance_m, sample_n, t_ms, d_mm, q_dbm

Workflow:
    1. Set up tag + anchor at the first test distance.
    2. Run this script with --serial or --udp (default).
    3. Enter session number (1) and true distance when prompted.
    4. Wait for N samples — progress prints every 50.
    5. Move boards to next distance; script auto-increments session.
    6. Repeatability test: power-cycle + re-place at same distance,
       keep same true_distance_m, enter same or incremented session number.
    7. Type 'q' at any prompt to quit and save.
"""

import sys
import csv
import socket
import argparse
import statistics
import threading
import queue
from pathlib import Path
from datetime import datetime
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rtls import FrameParser

DEFAULT_UDP_PORT     = 5005
DEFAULT_BAUD         = 115200
DEFAULT_SAMPLES      = 500
REPORT_EVERY         = 50
UDP_TIMEOUT_S        = 5.0
MAX_SILENT_TIMEOUTS  = 12      # 60 s of silence → abort session


# ── Line sources ───────────────────────────────────────────────────────────────

def _udp_lines(port: int, line_queue: queue.Queue, stop: threading.Event) -> None:
    """Background thread: receive UDP packets and push lines onto line_queue."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', port))
    sock.settimeout(UDP_TIMEOUT_S)
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(512)
            line_queue.put(data.decode('utf-8', errors='replace').strip())
        except socket.timeout:
            line_queue.put(None)   # sentinel: timed out
        except OSError:
            break
    sock.close()


def _serial_lines(port: str, baud: int,
                  line_queue: queue.Queue, stop: threading.Event) -> None:
    """Background thread: read serial lines and push onto line_queue."""
    try:
        import serial as pyserial
    except ImportError:
        print("ERROR: pip install pyserial")
        stop.set()
        return
    try:
        ser = pyserial.Serial(port, baud, timeout=UDP_TIMEOUT_S)
    except Exception as e:
        print(f"[SER] Cannot open {port}: {e}")
        stop.set()
        return
    while not stop.is_set():
        try:
            raw = ser.readline()
            if raw:
                line_queue.put(raw.decode('utf-8', errors='replace').strip())
            else:
                line_queue.put(None)   # timeout sentinel
        except Exception:
            break
    ser.close()


# ── Collection ─────────────────────────────────────────────────────────────────

def collect_session(line_queue: queue.Queue, anchor_id: int, n_samples: int,
                    session: int, true_dist_m: float,
                    writer: 'csv.writer') -> int:
    """
    Pull lines from line_queue until n_samples for anchor_id are collected.
    None entries from the queue are timeout sentinels (no data for one interval).
    Returns the number of samples collected.
    """
    collected    = 0
    silent_count = 0
    d_list: list = []
    sum_d        = 0.0
    sum_q        = 0.0
    width        = len(str(n_samples))

    print(f"\n[COLLECT] Session {session}  dist={true_dist_m:.3f} m  "
          f"anchor=0x{anchor_id:02X}  target={n_samples} samples")

    while collected < n_samples:
        try:
            item = line_queue.get(timeout=UDP_TIMEOUT_S + 1.0)
        except queue.Empty:
            item = None

        if item is None:
            silent_count += 1
            if silent_count == 1:
                print("[WARN]  No data for 5 s. "
                      "Check transport (--serial / --udp) and that tag is running.")
            if silent_count >= MAX_SILENT_TIMEOUTS:
                print("[ERROR] No signal for 60 s. Aborting session.")
                break
            continue

        silent_count = 0
        pkt = FrameParser.parse(item)
        if not pkt.valid:
            continue

        for i, aid in enumerate(pkt.ids):
            if int(aid) != anchor_id:
                continue

            d_mm  = int(round(pkt.dist[i] * 1000.0))
            q_dbm = round(float(pkt.q[i]), 1)

            writer.writerow([session, true_dist_m, collected + 1,
                             pkt.t_ms, d_mm, q_dbm])
            collected += 1
            sum_d     += d_mm
            sum_q     += q_dbm
            d_list.append(d_mm)

            if collected % REPORT_EVERY == 0 or collected == n_samples:
                print(f"  [{collected:{width}d}/{n_samples}]  "
                      f"d={d_mm:5d} mm  q={q_dbm:+6.1f} dBm")
            break

    if collected > 0:
        mean_d  = sum_d / collected
        std_d   = statistics.stdev(d_list) if len(d_list) > 1 else 0.0
        bias_mm = mean_d - true_dist_m * 1000.0
        mean_q  = sum_q / collected
        print(f"[DONE]   mean={mean_d:.0f} mm  std={std_d:.1f} mm  "
              f"bias={bias_mm:+.0f} mm  q={mean_q:.1f} dBm  n={collected}")
    else:
        print("[DONE]   No samples collected in this session.")

    return collected


# ── Prompts ────────────────────────────────────────────────────────────────────

def _prompt_float(label: str) -> Optional[float]:
    while True:
        raw = input(f"{label}: ").strip()
        if raw.lower() == 'q':
            return None
        try:
            v = float(raw)
            if v > 0:
                return v
            print("  Must be a positive number.")
        except ValueError:
            print("  Enter a number (e.g. 1.0) or 'q' to quit.")


def _prompt_int(label: str, default: int) -> int:
    while True:
        raw = input(f"{label} [{default}]: ").strip()
        if raw == '':
            return default
        try:
            v = int(raw)
            if v >= 1:
                return v
            print("  Must be ≥ 1.")
        except ValueError:
            print("  Enter an integer.")


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description='Capture UWB ranging data for bias characterisation'
    )
    ap.add_argument('--anchor',  type=int, required=True,
                    help='Anchor ID (e.g. 1 for 0x01)')
    ap.add_argument('--samples', type=int, default=DEFAULT_SAMPLES,
                    help=f'Measurements per distance (default {DEFAULT_SAMPLES})')
    ap.add_argument('--output',  type=str, default=None,
                    help='CSV output path (default: bias_anchor<N>_<timestamp>.csv)')

    transport = ap.add_mutually_exclusive_group()
    transport.add_argument('--udp',    type=int, nargs='?', const=DEFAULT_UDP_PORT,
                           metavar='PORT',
                           help=f'UDP port (default {DEFAULT_UDP_PORT}) [default transport]')
    transport.add_argument('--serial', type=str, metavar='PORT',
                           help='Serial port (e.g. /dev/ttyUSB0 or COM3)')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                    help=f'Serial baud rate (default {DEFAULT_BAUD})')

    args = ap.parse_args()

    anchor_id = args.anchor
    n_samples = args.samples

    # Default to UDP if neither transport flag given
    use_serial = args.serial is not None
    udp_port   = args.udp if args.udp is not None else DEFAULT_UDP_PORT

    if args.output:
        out_path = Path(args.output)
    else:
        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = Path(__file__).resolve().parent / f'bias_anchor{anchor_id}_{ts}.csv'

    file_exists = out_path.exists() and out_path.stat().st_size > 0
    open_mode   = 'a' if file_exists else 'w'

    transport_str = f"serial:{args.serial}@{args.baud}" if use_serial else f"UDP:{udp_port}"
    print(f"\n[INIT]  Anchor 0x{anchor_id:02X}  transport={transport_str}  "
          f"samples/distance={n_samples}")
    print(f"[INIT]  Output → {out_path}"
          + ("  (appending)" if file_exists else "  (new file)"))
    print("[INIT]  Type 'q' at any prompt to quit.\n")

    line_queue: queue.Queue = queue.Queue(maxsize=256)
    stop_event              = threading.Event()

    if use_serial:
        t = threading.Thread(target=_serial_lines,
                             args=(args.serial, args.baud, line_queue, stop_event),
                             daemon=True)
        print(f"[SER]   Reading from {args.serial} @ {args.baud} baud...\n")
    else:
        t = threading.Thread(target=_udp_lines,
                             args=(udp_port, line_queue, stop_event),
                             daemon=True)
        print(f"[UDP]   Listening on port {udp_port}...\n")

    t.start()

    session = 1
    try:
        with open(out_path, open_mode, newline='') as fh:
            writer = csv.writer(fh)
            if not file_exists:
                writer.writerow(['session', 'true_distance_m', 'sample_n',
                                 't_ms', 'd_mm', 'q_dbm'])

            while True:
                session = _prompt_int("Session number", session)
                dist    = _prompt_float("True distance (m)")
                if dist is None:
                    break

                n = collect_session(line_queue, anchor_id, n_samples,
                                    session, dist, writer)
                fh.flush()
                if n > 0:
                    print(f"  → {n} rows saved to {out_path}\n")

                session += 1
    finally:
        stop_event.set()

    print(f"\n[EXIT]  Data saved to {out_path}")


if __name__ == '__main__':
    main()
