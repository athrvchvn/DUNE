"""
Parse one RTLS host-link line into a typed packet.

Wire format (see firmware HostLink.h / TagWrover.ino):

  RTLS frames (localization sweeps):
    v1: RTLS,v1,<t_ms>,<tag_id>,<n>,<id1>,<d1_mm>,<rx1_dbm>,...
    v2: RTLS,v2,<t_ms>,<tag_id>,<n>,<id1>,<d1_mm>,<rx1_dbm>,<fp1_dbm>,<q1>,...
         [,IMU,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>]
    v3: RTLS,v3,...  (same anchor fields as v2)
         [,IMU,<status>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>]
         [,CYC,<cycle_id>][,MRX,<master_rx_us>]
    v4: RTLS,v4,<t_ms>,<tag_id>,<n>,{...v3 anchor fields...,<cfo>,<tex>}*n
         [,DIAG,<die_temp_c>,<vbat_v>][,IMU,...][,CYC,...][,MRX,...]
    v1 = 3 fields/anchor (id, d_mm, rx_dbm)
    v2/v3 = 5 fields/anchor (id, d_mm, rx_dbm, fp_dbm, quality)
    v4 = 7 fields/anchor (adds cfo, tex)

  v4 per-anchor radio diagnostics (Phase-C wobble investigation):
    cfo = raw DW1000 carrier integrator of the RANGE_REPORT RX. Converted here
          to ppm; positive means the remote (anchor) crystal runs faster.
    tex = realised exchange start, ms from the start of the sweep. Ranges in one
          sweep are sequential, so this is what lets the host reason about
          intra-sweep motion instead of treating a sweep as instantaneous.
  DIAG tail = the tag's own DW1000 die temperature (C) and battery voltage (V).
    NOTE this is the in-line DIAG *tail* on an RTLS line, which is a different
    thing from the standalone `DIAG,v1,...` TDMA-budget line that
    run_localization.py parses separately.

  Tail order is strictly positional: DIAG -> IMU -> CYC -> MRX.

  Dual-tag (TagLink) tails, appended by the master tag:
    CYC = master cycle number (joins the two tags' lines + DIAG records)
    MRX = master receive time (µs, master clock) of the slave's data — present
          only on forwarded slave lines. t_ms always stays the OWNING tag's
          clock (originals are never rewritten); the host has three time
          references per measurement (slave t_ms, MRX, DIAG cycle_start_us).

  Survey frames (anchor self-survey):
    SURVEY_BEGIN,v1,<n_pairs>
    SURVEY,v1,<src_id>,<dst_id>,<avg_dist_mm>,<ok_samples>
    SURVEY_DONE,v1
"""

from dataclasses import dataclass, field
from typing import List, Optional
import math


# ── IMU / Sweep data-classes ──────────────────────────────────────────────────

@dataclass
class ImuData:
    t_ms:   int                                     # firmware timestamp of sample
    quat:   List[float]                             # [w, x, y, z]
    acc:    List[float]                             # [ax, ay, az] m/s², body frame
    gyro:   List[float] = field(default_factory=list)  # [gx, gy, gz] rad/s; [] if v2
    status: int = 0                                 # BNO085 accuracy 0–3; 0 if v2


@dataclass
class SweepPacket:
    valid: bool = False
    t_ms:  int = 0
    tag_id: int = 0
    ids:    List[int]   = field(default_factory=list)
    dist:   List[float] = field(default_factory=list)
    q:      List[float] = field(default_factory=list)
    fp:     List[float] = field(default_factory=list)   # first-path power dBm (v2+)
    quality: List[float] = field(default_factory=list)  # receive quality (v2+)
    imu:    Optional[ImuData] = None
    cycle:  Optional[int] = None   # TagLink master cycle id (CYC tail)
    mrx_us: Optional[int] = None   # master receive time µs (MRX tail; forwarded slave lines)
    # ── v4 only; empty / None on older versions ──────────────────────────────
    cfo_ppm: List[float] = field(default_factory=list)  # per-anchor carrier offset, ppm
    tex_ms:  List[float] = field(default_factory=list)  # per-anchor exchange start, ms into sweep
    temp_c:  Optional[float] = None                     # tag die temperature, C (DIAG tail)
    vbat_v:  Optional[float] = None                     # tag battery, V (DIAG tail)
    version: str = ''                                   # 'v1'..'v4' as seen on the wire


# ── Survey data-classes ───────────────────────────────────────────────────────

@dataclass
class SurveyPair:
    """One pairwise measurement from a SURVEY,v1,… line."""
    src:    int
    dst:    int
    dist_m: float
    ok:     int


@dataclass
class SurveyPacket:
    """
    Represents a single parsed survey line.

    kind: 'begin' | 'pair' | 'done' | None  (None = not a survey line)
    """
    kind:     Optional[str] = None   # 'begin', 'pair', 'done'
    n_pairs:  int = 0                # set on 'begin'
    pair:     Optional[SurveyPair] = None   # set on 'pair'


# ── Parser ────────────────────────────────────────────────────────────────────

class FrameParser:
    """
    Static parser for all host-link frame types emitted by the tag firmware.

    Usage::

        pkt = FrameParser.parse(line)          # SweepPacket — RTLS data
        spkt = FrameParser.parse_survey(line)  # SurveyPacket — survey data
    """

    # Raw DW1000 carrier integrator -> ppm. Decawave constants for 110 kbps /
    # channel 5: Fs/2 correction 998.4e6/2/8192/131072 Hz per LSB against a
    # 6489.6 MHz carrier. The negative sign follows dwt_readcarrierintegrator's
    # convention, so positive ppm = the remote crystal runs faster than ours.
    # Kept identical to matlab/+dune/parseRtlsLine.m so both hosts agree.
    CFO_PPM_PER_LSB = (998.4e6 / 2 / 8192 / 131072) * (-1e6 / 6489.6e6)

    _STRIDE = {'v1': 3, 'v2': 5, 'v3': 5, 'v4': 7}

    @staticmethod
    def parse(line: str) -> SweepPacket:
        """Parse an RTLS v1/v2/v3/v4 sweep line. Returns SweepPacket(valid=False) on error."""
        pkt = SweepPacket()
        if not line:
            return pkt
        tok = line.strip().split(',')
        if len(tok) < 5 or tok[0] != 'RTLS':
            return pkt

        version = tok[1]
        # Unknown/newer versions are rejected outright rather than silently
        # decoded with the v1 layout -- misreading a future format as valid data
        # is worse than dropping the line.
        if version not in FrameParser._STRIDE:
            return pkt
        pkt.version = version
        fields_per_anchor = FrameParser._STRIDE[version]

        try:
            pkt.t_ms   = int(tok[2])
            pkt.tag_id = int(tok[3])
            n          = int(tok[4])
        except (ValueError, IndexError):
            return pkt
        if n < 0:
            return pkt

        need = 5 + fields_per_anchor * n
        if len(tok) < need:
            return pkt

        ids, dist, q, fp, quality = [], [], [], [], []
        cfo_ppm, tex_ms = [], []
        k = 5
        for _ in range(n):
            try:
                ids.append(int(tok[k]))
                dist.append(float(tok[k + 1]) / 1000.0)   # mm → m
                q.append(float(tok[k + 2]))
                if fields_per_anchor >= 5:                # v1 carries no fp/quality
                    fp.append(float(tok[k + 3]))
                    quality.append(float(tok[k + 4]))
                if fields_per_anchor == 7:                 # v4 radio diagnostics
                    cfo_ppm.append(float(tok[k + 5]) * FrameParser.CFO_PPM_PER_LSB)
                    tex_ms.append(float(tok[k + 6]))
            except (ValueError, IndexError):
                return pkt
            k += fields_per_anchor

        pkt.ids     = ids
        pkt.dist    = dist
        pkt.q       = q
        pkt.fp      = fp
        pkt.quality = quality
        pkt.cfo_ppm = cfo_ppm
        pkt.tex_ms  = tex_ms

        # Optional DIAG tail (v4) — tag die temperature + battery. Must be read
        # before IMU: the firmware emits the tails in a fixed order.
        if len(tok) >= k + 3 and tok[k] == 'DIAG':
            try:
                pkt.temp_c = float(tok[k + 1])
                pkt.vbat_v = float(tok[k + 2])
                k += 3
            except ValueError:
                pass

        # Optional IMU tail. v3 and v4 share the 11-value status+quat+acc+gyro
        # form; v2 is the older 7-float quat+acc form.
        if len(tok) > k and tok[k] == 'IMU':
            try:
                if version in ('v3', 'v4') and len(tok) >= k + 12:
                    # IMU,<status>,<qw,qx,qy,qz>,<ax,ay,az>,<gx,gy,gz>
                    status = int(tok[k + 1])
                    vals   = [float(tok[k + 2 + i]) for i in range(10)]
                    pkt.imu = ImuData(
                        t_ms=pkt.t_ms,
                        quat=vals[0:4],
                        acc=vals[4:7],
                        gyro=vals[7:10],
                        status=status,
                    )
                    k += 12
                elif len(tok) >= k + 8:
                    # v2: IMU,<qw,qx,qy,qz>,<ax,ay,az>  (7 floats, no status/gyro)
                    vals = [float(tok[k + 1 + i]) for i in range(7)]
                    pkt.imu = ImuData(
                        t_ms=pkt.t_ms,
                        quat=vals[0:4],
                        acc=vals[4:7],
                    )
                    k += 8
            except (ValueError, IndexError):
                pass

        # Optional TagLink tails (CYC/MRX marker-value pairs after the IMU tail).
        while k < len(tok):
            try:
                if tok[k] == 'CYC' and k + 1 < len(tok):
                    pkt.cycle = int(tok[k + 1])
                    k += 2
                    continue
                if tok[k] == 'MRX' and k + 1 < len(tok):
                    pkt.mrx_us = int(tok[k + 1])
                    k += 2
                    continue
            except ValueError:
                pass
            k += 1

        # Validate: no NaN anywhere
        if any(math.isnan(v) for v in dist + q + fp + quality):
            return pkt
        pkt.valid = True
        return pkt

    @staticmethod
    def parse_survey(line: str) -> SurveyPacket:
        """
        Parse a SURVEY_BEGIN / SURVEY / SURVEY_DONE line.
        Returns SurveyPacket(kind=None) if the line is not a survey frame.
        """
        spkt = SurveyPacket()
        if not line:
            return spkt
        s = line.strip()

        if s.startswith('SURVEY_BEGIN,v1,'):
            tok = s.split(',')
            try:
                spkt.kind    = 'begin'
                spkt.n_pairs = int(tok[2])
            except (IndexError, ValueError):
                pass
            return spkt

        if s.startswith('SURVEY_DONE'):
            spkt.kind = 'done'
            return spkt

        if s.startswith('SURVEY,v1,'):
            tok = s.split(',')
            if len(tok) >= 6:
                try:
                    src    = int(tok[2])
                    dst    = int(tok[3])
                    dist_m = float(tok[4]) / 1000.0
                    ok     = int(tok[5])
                    if ok > 0 and dist_m > 0:
                        spkt.kind = 'pair'
                        spkt.pair = SurveyPair(src=src, dst=dst, dist_m=dist_m, ok=ok)
                except (IndexError, ValueError):
                    pass
            return spkt

        return spkt   # kind remains None — not a survey line
