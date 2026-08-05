#!/usr/bin/env python3
"""
collect_bias_data.py — Systematic bias characterisation data collection tool.

Records raw UWB ranging samples at known distances for one anchor at a time.
Supports v1 and v2 wire formats (v2 adds fp_power_dbm and quality columns).

CRITICAL DATA INTEGRITY RULES:
  - Recording never starts automatically.  The user must press ENTER (or type OK)
    to confirm that the boards are stable at the stated distance.
  - After confirmation, a 2-second flush window discards in-flight stale packets
    before any sample is written to CSV.
  - All counters, sample arrays, and packet buffers are reset before each session.

Usage (UDP — tag sends to broadcast/host IP):
    python collect_bias_data.py --anchor 1

Usage (Serial — more reliable in lab):
    python collect_bias_data.py --anchor 1 --serial /dev/ttyUSB0

Custom distances (cm, comma-separated):
    python collect_bias_data.py --anchor 1 --distances "50,100,200,300"

CSV columns (16):
    wall_time_iso, anchor_id, session, distance_cm, true_distance_m,
    sample_n, t_ms, d_mm, rx_power_dbm, fp_power_dbm, quality,
    antenna_delay, transport,
    packets_received, packets_accepted, packets_discarded
"""

import sys
import csv
import time
import socket
import argparse
import statistics
import threading
import queue
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rtls import FrameParser

# ── Constants ─────────────────────────────────────────────────────────────────

DEFAULT_UDP_PORT    = 4100
DEFAULT_BAUD        = 115200
DEFAULT_SAMPLES     = 500
DEFAULT_SESSIONS    = 2
DEFAULT_ANTENNA_DELAY = 16385
FLUSH_DURATION_S    = 2.0      # seconds to discard packets after user confirms
UDP_RX_TIMEOUT_S    = 3.0      # socket/serial readline timeout
REPORT_EVERY        = 100      # print progress every N samples

DEFAULT_DISTANCES_CM = [5, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600]

CSV_HEADER = [
    'wall_time_iso', 'anchor_id', 'session', 'distance_cm', 'true_distance_m',
    'sample_n', 't_ms', 'd_mm', 'rx_power_dbm', 'fp_power_dbm', 'quality',
    'antenna_delay', 'transport',
    'packets_received', 'packets_accepted', 'packets_discarded',
]


# ── State machine ─────────────────────────────────────────────────────────────

class State(Enum):
    CONFIRM_WAIT = auto()   # waiting for user to press ENTER
    FLUSHING     = auto()   # 2-second discard window
    RECORDING    = auto()   # writing samples to CSV
    SESSION_DONE = auto()   # target reached; session complete


# ── Line sources (background threads) ─────────────────────────────────────────

class LineSource(ABC):
    """Background thread that pushes decoded text lines onto a queue."""

    def __init__(self) -> None:
        self._q: queue.Queue = queue.Queue(maxsize=512)
        self._stop           = threading.Event()
        self._thread         = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def get(self, timeout: float = 1.0) -> Optional[str]:
        """Return next line, or None on timeout."""
        try:
            return self._q.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self) -> int:
        """Discard all lines currently in the queue. Returns count drained."""
        count = 0
        while True:
            try:
                self._q.get_nowait()
                count += 1
            except queue.Empty:
                break
        return count

    @abstractmethod
    def _run(self) -> None: ...


class UdpLineSource(LineSource):
    def __init__(self, port: int) -> None:
        super().__init__()
        self._port = port

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # SO_REUSEPORT lets multiple processes share the port (e.g. run_localization.py
        # running alongside).  SO_BROADCAST lets us receive tag's 255.255.255.255 datagrams.
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass   # Windows / older kernels: ignore gracefully
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', self._port))
        sock.settimeout(UDP_RX_TIMEOUT_S)
        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(2048)
                line = data.decode('utf-8', errors='replace').strip()
                if line:
                    try:
                        self._q.put_nowait(line)
                    except queue.Full:
                        pass
            except socket.timeout:
                pass
            except OSError:
                break
        sock.close()


class SerialLineSource(LineSource):
    def __init__(self, port: str, baud: int) -> None:
        super().__init__()
        self._port = port
        self._baud = baud

    def _run(self) -> None:
        try:
            import serial as pyserial
        except ImportError:
            print("ERROR: pyserial not installed — run: pip install pyserial")
            self._stop.set()
            return
        try:
            ser = pyserial.Serial(self._port, self._baud, timeout=UDP_RX_TIMEOUT_S)
        except Exception as e:
            print(f"[SER] Cannot open {self._port}: {e}")
            self._stop.set()
            return
        while not self._stop.is_set():
            try:
                raw = ser.readline()
                if raw:
                    line = raw.decode('utf-8', errors='replace').strip()
                    if line:
                        try:
                            self._q.put_nowait(line)
                        except queue.Full:
                            pass
            except Exception:
                break
        ser.close()


# ── Session stats (per-session, reset before each session) ────────────────────

@dataclass
class SessionStats:
    samples:    List[float] = field(default_factory=list)   # d_mm per sample
    q_list:     List[float] = field(default_factory=list)
    fp_list:    List[float] = field(default_factory=list)
    pkt_rx:     int = 0   # packets received from source (after flush)
    pkt_ok:     int = 0   # packets that parsed and matched anchor_id
    pkt_discard: int = 0  # packets that parsed but did NOT match anchor_id

    def reset(self) -> None:
        self.samples.clear()
        self.q_list.clear()
        self.fp_list.clear()
        self.pkt_rx = self.pkt_ok = self.pkt_discard = 0


# ── Prompt helpers ─────────────────────────────────────────────────────────────

def _await_confirmation(distance_cm: int, session: int, run_n: int, total_runs: int) -> bool:
    """Print the per-run banner and block until the user presses ENTER or types OK.
    Returns False if user signals quit (q/quit/exit)."""
    print()
    print("━" * 50)
    print(f"  Run {run_n}/{total_runs}  │  dist={distance_cm:4d} cm  │  session={session}")
    print("━" * 50)
    if session > 1:
        print(f"  Power-cycle boards if needed, then place at {distance_cm} cm.")
    else:
        print(f"  Place boards at {distance_cm} cm and wait for stable ranging.")
    print()
    while True:
        try:
            raw = input("  Type OK or press ENTER when stable: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return False
        if raw in ('', 'ok', 'y', 'yes'):
            return True
        if raw in ('q', 'quit', 'exit'):
            return False
        print("  (Press ENTER or type OK to start, 'q' to quit)")


# ── Main collection function ───────────────────────────────────────────────────

def collect_run(
    source: LineSource,
    anchor_id: int,
    n_samples: int,
    distance_cm: int,
    session: int,
    antenna_delay: int,
    transport_str: str,
    writer: 'csv.writer',
    fileobj,            # underlying file handle for per-row flush
) -> SessionStats:
    """
    Execute one CONFIRM_WAIT → FLUSHING → RECORDING → SESSION_DONE cycle.
    Returns SessionStats for the completed session.
    """
    stats = SessionStats()
    true_dist_m = distance_cm / 100.0
    width = len(str(n_samples))

    # ── FLUSHING: drain queue + discard incoming for 2 s ─────────────────────
    source.drain()
    flush_start = time.monotonic()
    print(f"  [FLUSH]  Discarding stale packets for {FLUSH_DURATION_S:.0f} s …", end='', flush=True)
    flushed = 0
    while time.monotonic() - flush_start < FLUSH_DURATION_S:
        line = source.get(timeout=0.1)
        if line:
            flushed += 1
    source.drain()
    print(f" done ({flushed} packets discarded)")
    print(f"  [REC]    Recording {n_samples} samples …\n")

    # ── RECORDING ─────────────────────────────────────────────────────────────
    silent_streak = 0
    mismatched_warn = 0       # avoid flooding terminal with per-packet warnings
    while len(stats.samples) < n_samples:
        line = source.get(timeout=UDP_RX_TIMEOUT_S + 0.5)
        if line is None:
            silent_streak += 1
            if silent_streak == 3:
                print(f"\n  [WARN]  No data for {silent_streak * UDP_RX_TIMEOUT_S:.0f} s."
                      "  Check transport and that tag is running.")
            if silent_streak >= 10:
                print("  [ERROR] No signal for too long. Stopping this session.")
                break
            continue
        silent_streak = 0
        stats.pkt_rx += 1

        pkt = FrameParser.parse(line)
        if not pkt.valid:
            if stats.pkt_rx <= 3:
                print(f"  [DBG] pkt_rx={stats.pkt_rx} parse failed: {line[:80]!r}")
            continue

        matched = False
        for i, aid in enumerate(pkt.ids):
            if int(aid) != anchor_id:
                continue
            matched = True

            d_mm    = int(round(pkt.dist[i] * 1000.0))
            rx_dbm  = round(float(pkt.q[i]), 1)
            fp_dbm  = round(float(pkt.fp[i]),      1) if i < len(pkt.fp)      else ''
            quality_val = round(float(pkt.quality[i]), 3) if i < len(pkt.quality) else ''

            sample_n = len(stats.samples) + 1
            wall_iso = datetime.now().isoformat(timespec='milliseconds')

            writer.writerow([
                wall_iso, anchor_id, session, distance_cm, true_dist_m,
                sample_n, pkt.t_ms, d_mm, rx_dbm, fp_dbm, quality_val,
                antenna_delay, transport_str,
                stats.pkt_rx, sample_n, stats.pkt_rx - sample_n,
            ])
            fileobj.flush()   # ensure every row hits disk immediately

            stats.samples.append(d_mm)
            stats.q_list.append(rx_dbm)
            if fp_dbm != '':
                stats.fp_list.append(float(fp_dbm))

            if sample_n % REPORT_EVERY == 0 or sample_n == n_samples:
                bias = d_mm - true_dist_m * 1000.0
                print(f"  [{sample_n:{width}d}/{n_samples}]"
                      f"  d={d_mm:5d} mm  bias={bias:+5.0f} mm"
                      f"  rx={rx_dbm:+6.1f} dBm"
                      + (f"  fp={float(fp_dbm):+6.1f} dBm" if fp_dbm != '' else ''))
            break

        if not matched:
            stats.pkt_discard += 1
            mismatched_warn += 1
            # Print first few mismatches so user can see which anchors ARE in the sweep
            if mismatched_warn <= 3:
                seen = [int(a) for a in pkt.ids]
                print(f"  [WARN]  Pkt has anchor IDs {seen} — not anchor {anchor_id}."
                      f"  (Check --anchor flag.  Suppressing after 3 warnings.)"
                      if mismatched_warn == 1 else
                      f"  [WARN]  Still no anchor {anchor_id} in sweep. IDs seen: {seen}")

    # ── SESSION_DONE summary ───────────────────────────────────────────────────
    n = len(stats.samples)
    if n > 0:
        mean_d  = statistics.mean(stats.samples)
        std_d   = statistics.stdev(stats.samples) if n > 1 else 0.0
        bias_mm = mean_d - true_dist_m * 1000.0
        mean_q  = statistics.mean(stats.q_list) if stats.q_list else float('nan')
        fp_str  = (f"  fp_mean={statistics.mean(stats.fp_list):+.1f} dBm"
                   if stats.fp_list else '')
        print()
        print(f"  ✓ Session done:  n={n}  mean={mean_d:.0f} mm"
              f"  bias={bias_mm:+.0f} mm  std={std_d:.1f} mm"
              f"  rx_mean={mean_q:.1f} dBm{fp_str}")
        print(f"    pkt_rx={stats.pkt_rx}  pkt_ok={n}  pkt_discard={stats.pkt_discard}")
    else:
        print("  ✗ No samples collected in this session.")

    return stats


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description='Collect UWB bias-characterisation data with explicit confirmation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('--anchor', type=int, required=True,
                    help='Anchor ID to collect from (e.g. 1 for 0x01)')
    ap.add_argument('--sessions', type=int, default=DEFAULT_SESSIONS,
                    help=f'Sessions per distance (default {DEFAULT_SESSIONS})')
    ap.add_argument('--samples', type=int, default=DEFAULT_SAMPLES,
                    help=f'Samples per session (default {DEFAULT_SAMPLES})')
    ap.add_argument('--antenna-delay', type=int, default=DEFAULT_ANTENNA_DELAY,
                    help=f'Antenna delay value to log in CSV (default {DEFAULT_ANTENNA_DELAY})')
    ap.add_argument('--distances', type=str, default=None,
                    help='Comma-separated distances in cm '
                         '(default: 5,50,100,...,600)')
    ap.add_argument('--output', type=str, default=None,
                    help='CSV output path (default: bias_anchor<N>_<ts>.csv)')

    transport = ap.add_mutually_exclusive_group()
    transport.add_argument('--udp', type=int, nargs='?', const=DEFAULT_UDP_PORT,
                           metavar='PORT',
                           help=f'UDP port (default {DEFAULT_UDP_PORT}) [default transport]')
    transport.add_argument('--serial', type=str, metavar='PORT',
                           help='Serial port (e.g. /dev/ttyUSB0 or COM3)')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                    help=f'Serial baud rate (default {DEFAULT_BAUD})')

    args = ap.parse_args()

    anchor_id     = args.anchor
    n_sessions    = args.sessions
    n_samples     = args.samples
    antenna_delay = args.antenna_delay

    if args.distances:
        try:
            distances = [int(x.strip()) for x in args.distances.split(',') if x.strip()]
        except ValueError:
            ap.error("--distances must be comma-separated integers (cm)")
    else:
        distances = DEFAULT_DISTANCES_CM

    use_serial    = args.serial is not None
    udp_port      = args.udp if args.udp is not None else DEFAULT_UDP_PORT
    transport_str = (f"serial:{args.serial}@{args.baud}" if use_serial
                     else f"udp:{udp_port}")

    if args.output:
        out_path = Path(args.output)
    else:
        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        data_dir = Path(__file__).resolve().parent / 'data'
        data_dir.mkdir(exist_ok=True)
        out_path = data_dir / f'bias_anchor{anchor_id}_{ts}.csv'

    # ── Session plan ───────────────────────────────────────────────────────────
    total_runs = len(distances) * n_sessions
    sweep_hz   = 10.0   # approximate firmware sweep rate
    eta_s      = total_runs * n_samples / sweep_hz
    eta_min    = eta_s / 60.0

    print(f"\n{'─'*55}")
    print(f"  Session plan — anchor 0x{anchor_id:02X}")
    print(f"{'─'*55}")
    run_n = 0
    for dist_cm in distances:
        for sess in range(1, n_sessions + 1):
            run_n += 1
            print(f"  Run {run_n:3d}/{total_runs}:  "
                  f"dist={dist_cm:4d} cm  session={sess}  samples={n_samples}")
    print(f"{'─'*55}")
    print(f"  Transport:      {transport_str}")
    print(f"  Antenna delay:  {antenna_delay}")
    print(f"  Output:         {out_path}")
    print(f"  Estimated time: ~{eta_min:.0f} min at {sweep_hz:.0f} Hz")
    print(f"{'─'*55}\n")

    # ── Start source ───────────────────────────────────────────────────────────
    if use_serial:
        source: LineSource = SerialLineSource(args.serial, args.baud)
        print(f"[SER]  Opening {args.serial} @ {args.baud} baud …")
    else:
        source = UdpLineSource(udp_port)
        print(f"[UDP]  Listening on port {udp_port} …")
    source.start()

    # ── Open CSV ───────────────────────────────────────────────────────────────
    file_exists = out_path.exists() and out_path.stat().st_size > 0
    open_mode   = 'a' if file_exists else 'w'
    if file_exists:
        print(f"[CSV]  Appending to existing file {out_path}")
    else:
        print(f"[CSV]  Creating {out_path}")

    run_n = 0
    try:
        with open(out_path, open_mode, newline='') as fh:
            writer = csv.writer(fh)
            if not file_exists:
                writer.writerow(CSV_HEADER)

            for dist_cm in distances:
                for sess in range(1, n_sessions + 1):
                    run_n += 1
                    confirmed = _await_confirmation(dist_cm, sess, run_n, total_runs)
                    if not confirmed:
                        print("\n[EXIT]  User quit.")
                        return

                    collect_run(
                        source=source,
                        anchor_id=anchor_id,
                        n_samples=n_samples,
                        distance_cm=dist_cm,
                        session=sess,
                        antenna_delay=antenna_delay,
                        transport_str=transport_str,
                        writer=writer,
                        fileobj=fh,
                    )
                    fh.flush()

    except KeyboardInterrupt:
        print("\n\n[CTRL+C]  Interrupted — partial data already written to CSV.")
        print(f"[EXIT]    {out_path}  (keep this file; rows already on disk are valid)")
    finally:
        source.stop()

    print(f"\n[DONE]  Output: {out_path}")


if __name__ == '__main__':
    main()
