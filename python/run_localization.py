#!/usr/bin/env python3
"""
run_localization.py — UWB RTLS host: receive ranges, solve, filter, serve GUI.

Python replacement for run_localization.m.  Receives RTLS packets from one or
more tags over UDP (or serial), runs per-tag multilateration + EKF + EMA, then
pushes results to a browser GUI via WebSocket.

Each tag that produces a packet automatically gets its own independent pipeline
state (TagState). No tag IDs are hard-coded; the system scales to N tags.

Multi-point survey: the GUI can request distance captures at known grid
positions.  After enough captures (≥4), the optimizer jointly solves for
anchor positions and writes anchors.json.

Usage:
    python run_localization.py                  # UDP on port 4100
    python run_localization.py --serial /dev/ttyUSB0
    python run_localization.py --udp-port 4100 --ws-port 8765
"""

import sys
import json
import time
import argparse
import asyncio
import math
import subprocess
import webbrowser
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional

# Add parent so `dune` is importable when run as a script
sys.path.insert(0, str(Path(__file__).resolve().parent))

from host.app       import App
from host.ingest    import SerialTransport, UdpTransport
import host.jobs_builtin                      # noqa: F401  (registers 'selftest')
import host.jobs_offline                      # noqa: F401  (registers the LAB jobs)
from host.rover_api import HANDLERS as ROVER_HANDLERS  # also registers wheels_test
import host.jobs_calib                        # noqa: F401  (registers the CAL jobs)
from host.paths import list_files

from dune import FrameParser, AnchorConfig
from dune.paths          import CONFIG_DIR
from dune.ekf            import quat_rpy, world_accel
from dune.fusion_config  import FUSION_DEFAULTS, TAG_PALETTE
from dune.range_correction import RangeCorrection
from dune.rigid_body     import RigidBodyTracker
from host.pipeline       import PipelineConfig, SweepPipeline
from host.tag_fix        import tag_fix_message
from tools.multipoint_survey import (solve_anchor_positions, write_anchors_json,
                                     update_anchor_json, SURVEY_GRID)
from tools.calibration_manager import CalibrationManager
from vision.vision_config import VisionConfig
from vision.backproject import mirror_correct
from vision.kinect_capture import KinectCapture
from vision.apriltag_detect import detect_tags, draw_overlay, parse_anchor_role
from vision.world_registration import (
    solve_registration, load_registration, compute_world, check_point_spread,
    WorldRegistration, MIN_ANCHORS, WARN_RMSE_M, BLOCK_RMSE_M, HEIGHT_TOL_M,
)
from vision.import_camera_calibration import load_camcal_mat, apply_camcal, DEFAULT_MAT_PATH
import numpy as np

try:
    import websockets
    import websockets.server
except ImportError:
    print("Missing dependency: pip install websockets")
    sys.exit(1)

try:
    import serial as pyserial
except ImportError:
    pyserial = None

try:
    import cv2
except ImportError:
    cv2 = None

# ── Defaults ──────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR    = SCRIPT_DIR / 'logs'
LOG_DIR.mkdir(exist_ok=True)

DEFAULT_UDP_PORT  = 4100
DEFAULT_WS_PORT   = 8765
DEFAULT_BAUD      = 115200

# Rigid dual-tag body: nominal distance between the two tag antennas (metres).
# Override per rig with --baseline.
BASELINE_NOMINAL_M = 0.50

# Multilaterator constants (not in fusion_config since they are solver-level)
RANGE_SIGMA = 0.20
GATE_K      = 2.5


# ── Per-tag state ─────────────────────────────────────────────────────────────

# ── Live pipeline ─────────────────────────────────────────────────────────────
# The estimator chain now lives in host/pipeline.py (a port of live_tag.m) and
# is shared byte-for-byte with replay. What remains here is the adapter that
# turns one SweepResult into the `tag_fix` message the GUI and every recorded
# session log expect. See host/tag_fix.py for why that schema is load-bearing.


class LivePipeline:
    """Adapts SweepPipeline to the wire: parse -> process -> tag_fix."""

    def __init__(self, cfg: AnchorConfig, fusion_cfg: dict | None = None):
        self.cfg = cfg
        self._fcfg = dict(FUSION_DEFAULTS)
        if fusion_cfg:
            self._fcfg.update(fusion_cfg)

        rc = RangeCorrection.load(CONFIG_DIR / 'range_correction.json')
        if rc is not None:
            # `or []` on a numpy array raises: truth value of an array is
            # ambiguous. Ask for the length directly.
            n_corr = np.size(getattr(rc, 'ids', ()))
            print(f"[TIER2] range_correction.json loaded ({n_corr} anchors)")
        else:
            print("[TIER2] no range_correction.json — ranges used uncorrected")

        self.pipe = SweepPipeline(
            cfg,
            PipelineConfig(tag_z=float(self._fcfg.get('tag_z', 0.24)),
                           gate_k=float(self._fcfg.get('gate_k', 3.0))),
            range_corr=rc)
        self.pkt_count = 0

    # ── live tuning ──────────────────────────────────────────────────────────

    def update_tuning(self, params: dict) -> dict:
        """Apply GUI tuning and RETURN the resulting state.

        The old version was fire-and-forget with no acknowledgement, so the
        sliders could disagree with the server and nothing showed it. Callers
        broadcast the returned dict as `tune_state`.
        """
        c = self.pipe.cfg
        for wire, apply in (
            ('emaAlpha',   lambda v: setattr(c, 'smooth', float(v))),
            ('gateK',      lambda v: setattr(c, 'gate_k', float(v))),
            ('tagZ',       lambda v: setattr(c, 'tag_z', float(v))),
            ('pinRadius',  lambda v: setattr(c, 'pin_radius', float(v))),
            ('hold',       lambda v: setattr(c, 'hold', bool(v))),
            ('pin',        lambda v: setattr(c, 'pin', bool(v))),
            ('nlos',       lambda v: setattr(c, 'nlos', bool(v))),
            # Finally satisfies fusion_config's long-dangling declaration that
            # estimator_mode is "live-switchable via GUI".
            ('estimator',  lambda v: setattr(c, 'estimator',
                                             'mhe' if str(v).lower() == 'mhe' else 'ekf')),
            ('mheModel',   lambda v: setattr(c, 'mhe_model',
                                             'unicycle' if str(v) == 'unicycle' else 'cv')),
        ):
            if wire in params and params[wire] is not None:
                apply(params[wire])

        # These reach the already-built filters, which the previous
        # implementation did not do — only ZUPT was ever rebuilt, so most
        # sliders silently had no effect on a running session.
        for key, attr in (('sigmaAccel', 'sigma_accel_cv'),
                          ('ekfQAccel', 'sigma_accel_cv'),
                          ('posSigma', 'pos_sigma'),
                          ('zuptSigma', 'zupt_sigma')):
            if key in params and params[key] is not None:
                for rt in self.pipe.tags.values():
                    setattr(rt.ekf, attr, float(params[key]))
        return self.tuning_state()

    def tuning_state(self) -> dict:
        c = self.pipe.cfg
        any_ekf = next(iter(self.pipe.tags.values()), None)
        return {'emaAlpha': c.smooth, 'gateK': c.gate_k, 'tagZ': c.tag_z,
                'pinRadius': c.pin_radius, 'hold': c.hold, 'pin': c.pin,
                'nlos': c.nlos, 'estimator': c.estimator,
                'mheModel': c.mhe_model,
                'sigmaAccel': (any_ekf.ekf.sigma_accel_cv if any_ekf else None)}

    @property
    def tag_ids(self) -> list:
        return list(self.pipe.tags.keys())

    # ── main entry ───────────────────────────────────────────────────────────

    def process(self, line: str, t_host: float | None = None) -> dict | None:
        """`t_host` is the ingest thread's receive time when available.

        Falling back to now() would fold any queueing delay into the filter's
        dt, so the caller passes the stamp taken at read time.
        """
        pkt = FrameParser.parse(line)
        if not pkt.valid:
            return None
        self.pkt_count += 1

        imu_state = None
        if pkt.imu is not None:
            aw = world_accel(pkt.imu.quat, pkt.imu.acc)
            roll, pitch, yaw = quat_rpy(pkt.imu.quat)
            imu_state = {
                'status': pkt.imu.status,
                'gyro': [round(float(v), 4) for v in pkt.imu.gyro],
                'ax_world': round(float(aw[0]), 4) if aw is not None else 0.0,
                'ay_world': round(float(aw[1]), 4) if aw is not None else 0.0,
                # The GUI's ORIENTATION panel reads fix.imu.roll/pitch/yaw
                # directly, so this must be the angles, not a placeholder.
                'angles': {
                    'roll':  round(float(np.degrees(roll)), 2),
                    'pitch': round(float(np.degrees(pitch)), 2),
                    'yaw':   round(float(np.degrees(yaw)), 2)
                             if np.isfinite(yaw) else 0.0,
                    'ax': round(float(pkt.imu.acc[0]), 3),
                    'ay': round(float(pkt.imu.acc[1]), 3),
                    'az': round(float(pkt.imu.acc[2]), 3),
                },
            }

        res = self.pipe.process(pkt, t_host if t_host is not None else time.time(),
                                imu=pkt.imu)
        rt = self.pipe.tags[res.tag_id]
        if not res.ok:
            return None
        return tag_fix_message(res, rt, pkt, self.cfg, imu_state=imu_state,
                               estimator=self.pipe.cfg.estimator)


# ── Domain state ──────────────────────────────────────────────────────────────
# The plumbing (clients, recorder, transport, job registry) now lives on `app`
# — see python/host/. What remains here is the domain layer: the pipeline, the
# anchor config, calibration, vision. Those move into host/handlers.py next;
# keeping them as globals for one step keeps this migration reviewable, since
# every message shape stays byte-identical.
app: App = None  # type: ignore
pipeline: LivePipeline = None   # type: ignore
cfg_global: AnchorConfig = None  # type: ignore
calib_mgr: CalibrationManager = None  # type: ignore
rigid_body = RigidBodyTracker(BASELINE_NOMINAL_M)

# ── Overhead Kinect click-to-place (optional, --vision) ───────────────────────
vision_cap: "KinectCapture | None" = None
vision_cfg: "VisionConfig | None" = None

# ── World registration (click-based tilt correction, see world_registration.py)
# None until kinect_world_registration.json exists — compute_world() then
# falls back to pixel_to_world's flat-plane assumption automatically.
world_registration: "WorldRegistration | None" = None

# ── AprilTag auto-detect: pending review state (detect -> review -> confirm) ──
# Nothing in anchors.json changes until _handle_vision_detect_confirm() runs;
# these hold whatever the last _handle_vision_detect() proposed.
_vision_pending: dict = {}          # anchor_id -> {'x','y','z','apriltag_id',...}
_vision_pending_frames: dict = {}   # {'raw': jpeg_bytes, 'overlay': jpeg_bytes}

# ── World registration: pending review state (start -> click -> solve -> confirm)
_reg_pending: dict = {}             # {'anchor_ids','world_xy','anchor_z_m','clicks':{aid:[u,v]},'solved':WorldRegistration|None}
_reg_pending_frame: "bytes | None" = None

# ── Multi-point survey state ──────────────────────────────────────────────────
_survey_captures: list = []
_survey_capture_active: bool = False
_survey_capture_buf: list = []
_survey_capture_target: int = 50
_survey_capture_point_idx: int = -1

# ── Error-map (Tier-2) capture state ──────────────────────────────────────────
# The Tier-2 SPATIAL error map is retired: DUNE replaced it with the per-anchor
# power model in range_correction.json, which SweepPipeline applies inside the
# solve. Keeping both would mean two corrections fighting over the same error.
# The errormap_* message family is gone; CAL step 4 now points at the power model.

# ── Hardware calibration state ────────────────────────────────────────────────
# Sends HWCALIB,<id>,<dist_mm> to the tag one anchor at a time. The tag blocks
# during calibration, so the next command is sent only after HWCALIB_DONE/FAIL
# is received for the current one — paced, and backstopped by a stall watchdog
# in case a command or reply is lost (see _hwcalib_watchdog).
HWCALIB_PACE_S        = 0.4   # settle time after DONE/FAIL before sending the next anchor
HWCALIB_STALL_S       = 8.0   # no HWCALIB_* line (incl. PROG) for this long => stalled
HWCALIB_WATCHDOG_POLL_S = 1.0  # how often the watchdog checks for staleness


@dataclass
class HwCalibSession:
    """All 'Calibrate All Anchors' run state in one place (was several loose
    globals) — scales cleanly if Tier-2/replay calibration is added later."""
    queue:      list            = field(default_factory=list)  # remaining anchor ids
    distances:  dict            = field(default_factory=dict)  # anchor_id -> true dist (mm)
    current:    Optional[int]   = None    # anchor id currently in flight
    retries:    int             = 0       # retransmits used for `current`
    last_msg_ts: float          = 0.0     # last HWCALIB_* line received (any type incl. PROG)
    active:     bool            = False   # a run is in progress
    tag_xy:     Optional[tuple] = None    # (x, y) m the distances were computed from, if given
    git_commit: Optional[str]   = None    # captured once per run, for provenance
    iteration_history: dict     = field(default_factory=dict)  # anchor_id -> [{it,delay,mean_mm,err_mm}]


hwcalib = HwCalibSession()


def send_to_tag(cmd: str):
    """Send a command string to the tag (UDP broadcast or serial)."""
    app.send_to_tag(cmd)


def _config_msg() -> dict:
    """The (single) config message: sent to each new client, re-broadcast when
    anchors change, and written as the recorder's replay keyframe."""
    return ({
        'type': 'config',
        'dim':  cfg_global.dim,
        'bounds': cfg_global.bounds,
        'anchors': [
            {'id': cfg_global.ids[i],
             'x': float(cfg_global.coords[i, 0]),
             'y': float(cfg_global.coords[i, 1])}
            for i in range(len(cfg_global.ids))
        ],
        'calibration': calib_mgr.status_msg() if calib_mgr else {},
        'error_map': {'loaded': False, 'retired': True},
        # Tag palette for the GUI — colour assignments are data-driven
        'tag_palette': TAG_PALETTE,
        # Rigid dual-tag body (roles fill in once tags are discovered)
        'baseline_nominal_m': rigid_body.baseline_nominal_m,
        'tags': rigid_body.roles(),
    })


# ── Inbound message table ─────────────────────────────────────────────────────
# Was a 21-branch elif chain inside the WebSocket handler. As a table it is
# introspectable, testable one entry at a time, and — the reason it had to
# change — an unknown type now produces an `error` message instead of silence.
# `app.ws_handler` does the dispatch; these adapters keep each domain handler's
# existing signature so their bodies are untouched.

async def _h_tune(app, data, ws):
    state = pipeline.update_tuning(data)
    # Echo what the server actually holds. Without this the sliders and the
    # server could disagree indefinitely with nothing to show it.
    await broadcast({'type': 'tune_state', 'params': state})


async def _h_survey_clear(app, data, ws):
    _survey_captures.clear()
    await broadcast({'type': 'survey_cleared'})
    print("[SURVEY] Cleared all captures")


async def _h_fs_list(app, data, ws):
    """Files the GUI may offer for a `path` job argument of this kind.

    Paths are repo-relative and every job re-validates containment on the way
    in — the picker is a convenience, not the security boundary.
    """
    kind = data.get('kind', 'logs')
    try:
        files = list_files(kind)
    except ValueError as e:
        await app.bus.emit('fs_error', kind=kind, error=str(e))
        return
    await app.bus.emit('fs_listing', kind=kind, files=files)


HANDLERS = {
    'fs_list':               _h_fs_list,
    'tune':                  _h_tune,
    'survey_capture':        lambda a, d, w: _start_survey_capture(d),
    'survey_compute':        lambda a, d, w: _do_survey_compute(),
    'survey_clear':          _h_survey_clear,
    'HWCALIB_REQ':           lambda a, d, w: _handle_hwcalib_req(d),
    'vision_grab':           lambda a, d, w: _handle_vision_grab(),
    'vision_grab_live':      lambda a, d, w: _handle_vision_live_grab(),
    'vision_place':          lambda a, d, w: _handle_vision_place(d),
    'vision_detect':         lambda a, d, w: _handle_vision_detect(),
    'vision_detect_confirm': lambda a, d, w: _handle_vision_detect_confirm(),
    'vision_detect_cancel':  lambda a, d, w: _handle_vision_detect_cancel(),
    'vision_register_start': lambda a, d, w: _handle_vision_register_start(d),
    'vision_register_click': lambda a, d, w: _handle_vision_register_click(d),
    'vision_register_solve': lambda a, d, w: _handle_vision_register_solve(),
    'vision_register_confirm': lambda a, d, w: _handle_vision_register_confirm(d),
    'vision_register_cancel':  lambda a, d, w: _handle_vision_register_cancel(),
    'vision_import_camcal':    lambda a, d, w: _handle_vision_import_camcal(d),
}


async def broadcast(payload, record: bool = True):
    """Domain-side alias for the bus.

    Kept so the ~40 existing call sites stay untouched during the migration.
    The choke-point semantics (record then fan out, bulk payloads excluded) now
    live in host/bus.py.
    """
    # record=False stays an explicit opt-out; the default defers to the bus,
    # which excludes bulk camera payloads by message type so a caller cannot
    # accidentally put a megabyte of base64 into the session log.
    await app.bus.broadcast(payload, False if record is False else None)


async def _start_survey_capture(data: dict):
    global _survey_capture_active, _survey_capture_buf
    global _survey_capture_target, _survey_capture_point_idx

    if _survey_capture_active:
        print("[SURVEY] Capture already in progress")
        return

    point_idx = data.get('point_idx', -1)
    n_samples = data.get('n_samples', 50)

    _survey_capture_point_idx = point_idx
    _survey_capture_target = max(10, n_samples)
    _survey_capture_buf = []
    _survey_capture_active = True

    grid_pt = SURVEY_GRID[point_idx] if 0 <= point_idx < len(SURVEY_GRID) else None
    label = f"({grid_pt[0]}, {grid_pt[1]})" if grid_pt else f"idx={point_idx}"
    print(f"[SURVEY] Capture started at {label}  (collecting {n_samples} sweeps)")
    await broadcast(json.dumps({
        'type': 'survey_capture_started',
        'point_idx': point_idx,
        'n_samples': n_samples,
    }))


def _process_survey_sweep(result: dict):
    """Called for each RTLS fix during an active capture."""
    global _survey_capture_active

    if not _survey_capture_active:
        return None

    dists = {}
    for key, val in result.items():
        if key.startswith('d') and key[1:].isdigit():
            aid = int(key[1:])
            dists[aid] = val

    if dists:
        _survey_capture_buf.append(dists)

    n = len(_survey_capture_buf)
    progress_msg = {
        'type': 'survey_capture_progress',
        'point_idx': _survey_capture_point_idx,
        'n': n,
        'target': _survey_capture_target,
    }

    if n >= _survey_capture_target:
        _survey_capture_active = False
        all_aids = set()
        for d in _survey_capture_buf:
            all_aids.update(d.keys())

        avg_dists = {}
        for aid in sorted(all_aids):
            vals = [d[aid] for d in _survey_capture_buf if aid in d]
            if vals:
                avg_dists[aid] = float(np.median(vals))

        capture = {'dists': avg_dists, 'point_idx': _survey_capture_point_idx}
        _survey_captures.append(capture)

        grid_pt = SURVEY_GRID[_survey_capture_point_idx] \
            if 0 <= _survey_capture_point_idx < len(SURVEY_GRID) else None
        label = f"({grid_pt[0]}, {grid_pt[1]})" if grid_pt else "?"
        dists_str = '  '.join(f'A{k}={v:.3f}m' for k, v in avg_dists.items())
        print(f"[SURVEY] Capture done at {label}: {dists_str}")

        done_msg = {
            'type': 'survey_capture_done',
            'point_idx': _survey_capture_point_idx,
            'avg_dists': {str(k): round(v, 4) for k, v in avg_dists.items()},
            'n_total_captures': len(_survey_captures),
        }
        return done_msg

    return progress_msg


async def _do_survey_compute():
    global cfg_global, pipeline

    if len(_survey_captures) < 4:
        msg = f"Need ≥4 captures, have {len(_survey_captures)}"
        print(f"[SURVEY] {msg}")
        await broadcast(json.dumps({'type': 'survey_error', 'msg': msg}))
        return

    anchor_ids = sorted(cfg_global.ids) if cfg_global else [1, 2, 3]
    dim = cfg_global.dim if cfg_global else 2
    z_height = 0.0
    if cfg_global and cfg_global.coords.shape[1] >= 3:
        z_height = float(cfg_global.coords[0, 2])

    print(f"[SURVEY] Computing positions from {len(_survey_captures)} captures...")
    await broadcast(json.dumps({'type': 'survey_computing'}))

    try:
        positions, info = solve_anchor_positions(
            _survey_captures, anchor_ids, dim=dim
        )
        write_anchors_json(
            positions, anchor_ids, dim=dim, z_height=z_height,
            path=CONFIG_DIR / 'anchors.json'
        )
    except Exception as e:
        msg = f"Optimizer failed: {e}"
        print(f"[SURVEY] {msg}")
        await broadcast(json.dumps({'type': 'survey_error', 'msg': msg}))
        return

    anchors_result = []
    for i, aid in enumerate(anchor_ids):
        anchors_result.append({
            'id': aid,
            'x': round(float(positions[i, 0]), 4),
            'y': round(float(positions[i, 1]), 4),
        })

    result_msg = json.dumps({
        'type': 'survey_result',
        'anchors': anchors_result,
        'residual_rms': round(info['residual_rms'], 4),
        'n_captures': info['n_captures'],
        'success': info['success'],
    })
    print(f"[SURVEY] Done — residual RMS={info['residual_rms']:.4f} m")
    for a in anchors_result:
        print(f"  A{a['id']}: ({a['x']:+.3f}, {a['y']:+.3f})")
    await broadcast(result_msg)

    # Hot-reload config and pipeline (per-tag state is discarded intentionally —
    # anchor positions changed, so all EKFs must reinitialise from scratch)
    anchors_path = CONFIG_DIR / 'anchors.json'
    cfg_global = AnchorConfig.from_json(str(anchors_path))
    pipeline = LivePipeline(cfg_global)
    if calib_mgr:
        calib_mgr.on_anchors_updated(cfg_global)
    print("[SURVEY] Pipeline reloaded with new anchor positions")

    await broadcast(_config_msg())


async def _handle_vision_grab():
    """GUI clicked 'Grab frame'. Encode the latest Kinect color frame as JPEG
    and send it to the click-to-place panel."""
    if vision_cap is None:
        await broadcast(json.dumps({'type': 'vision_error',
                                    'msg': 'Vision not enabled — start with --vision'}))
        return
    if cv2 is None:
        await broadcast(json.dumps({'type': 'vision_error', 'msg': 'opencv-python not installed'}))
        return

    frame = vision_cap.get_latest_frame()
    if frame is None:
        await broadcast(json.dumps({'type': 'vision_error', 'msg': 'No frame available yet'}))
        return

    # Show the mirror-corrected (natural, un-mirrored) frame — a person can
    # then click on it exactly as they'd expect, matching the real room, and
    # the click coordinates need no further flip in pixel_to_world().
    frame = mirror_correct(frame, vision_cfg)

    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        await broadcast(json.dumps({'type': 'vision_error', 'msg': 'JPEG encode failed'}))
        return

    import base64
    jpeg_b64 = base64.b64encode(buf.tobytes()).decode('ascii')
    await broadcast(json.dumps({
        'type':   'vision_frame',
        'jpeg_b64': jpeg_b64,
        'width':  int(frame.shape[1]),
        'height': int(frame.shape[0]),
        'ts':     time.time(),
    }), record=False)


async def _handle_vision_live_grab():
    """gui.html's LiveFeed panel calls this repeatedly on a timer (see
    Vision.grab() for the one-off equivalent). Deliberately silent on
    failure — a polling loop hitting an unavailable Kinect shouldn't spam
    error broadcasts every interval; the panel just shows nothing until
    frames start arriving. Own message type (vision_frame_live, not
    vision_frame) so it never collides with the CAL-mode click-to-place
    panel's own grab/display state. Lower JPEG quality than the one-off
    grab (75 vs 85) since this is a repeatedly-polled preview, not a frame
    a person needs to click precisely on."""
    if vision_cap is None or vision_cfg is None or cv2 is None:
        return
    frame = vision_cap.get_latest_frame()
    if frame is None:
        return
    frame = mirror_correct(frame, vision_cfg)
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ok:
        return

    import base64
    await broadcast(json.dumps({
        'type': 'vision_frame_live',
        'jpeg_b64': base64.b64encode(buf.tobytes()).decode('ascii'),
        'width': int(frame.shape[1]), 'height': int(frame.shape[0]), 'ts': time.time(),
    }), record=False)


async def _handle_vision_place(data: dict):
    """GUI clicked a pixel on the grabbed frame and hit Assign. Back-project
    to world (x, y, z) on the assumed plane at z_override, then either upsert
    into anchors.json (target='anchor') or just echo the coordinate back
    (target='tag', for the HWCALIB tag-position inputs)."""
    global cfg_global, pipeline

    # Gate on vision_cap (the live capture), not just vision_cfg: a pixel
    # placement is only meaningful if it came from a real grabbed frame, and
    # vision_cap is None whenever the Kinect never actually started —
    # matching _handle_vision_grab's gate so the two can't disagree.
    if vision_cap is None or vision_cfg is None:
        await broadcast(json.dumps({'type': 'vision_error',
                                    'msg': 'Vision not enabled — start with --vision'}))
        return

    try:
        u = float(data['u'])
        v = float(data['v'])
    except (KeyError, TypeError, ValueError):
        await broadcast(json.dumps({'type': 'vision_error', 'msg': 'Bad pixel coordinates'}))
        return

    target     = data.get('target')
    z_override = float(data.get('z_override', 0.0))
    x, y, z = compute_world(u, v, vision_cfg, world_registration, z_world=z_override)
    mode = 'registered' if world_registration is not None else 'flat-plane fallback'

    anchor_id = data.get('anchor_id')
    if target == 'anchor':
        anchor_id = int(anchor_id)
        update_anchor_json(anchor_id, x, y, z, path=CONFIG_DIR / 'anchors.json')

        # Hot-reload config and pipeline (same pattern as _do_survey_compute).
        anchors_path = CONFIG_DIR / 'anchors.json'
        cfg_global = AnchorConfig.from_json(str(anchors_path))
        pipeline = LivePipeline(cfg_global)
        if calib_mgr:
            calib_mgr.on_anchors_updated(cfg_global)
        await broadcast(_config_msg())

    await broadcast(json.dumps({
        'type':      'vision_place_result',
        'target':    target,
        'anchor_id': anchor_id,
        'x': round(x, 4), 'y': round(y, 4), 'z': round(z, 4),
        'mode':      mode,
    }))


async def _handle_vision_detect():
    """GUI clicked 'Auto-detect tags'. Grab a frame, detect every visible
    AprilTag in one pass, map each to a role via vision_cfg.tag_roles.
    'anchorN' roles become CANDIDATE anchor writes (validated, but NOT yet
    written — see _handle_vision_detect_confirm); any other role (e.g.
    'tagA') immediately re-emits the existing target='tag' vision_place_result
    shape so the GUI's current HWCALIB-field-filling code just works, with no
    file write involved."""
    if vision_cap is None or vision_cfg is None:
        await broadcast(json.dumps({'type': 'vision_error',
                                    'msg': 'Vision not enabled — start with --vision'}))
        return
    if cv2 is None:
        await broadcast(json.dumps({'type': 'vision_error', 'msg': 'opencv-python not installed'}))
        return

    frame = vision_cap.get_latest_frame()
    if frame is None:
        await broadcast(json.dumps({'type': 'vision_error', 'msg': 'No frame available yet'}))
        return
    # Single mirror-correction, reused for detection, overlay drawing, and
    # the archived raw snapshot (avoids re-flipping the same frame 3x).
    frame = mirror_correct(frame, vision_cfg)

    try:
        detections = detect_tags(frame, vision_cfg)
    except RuntimeError as e:
        await broadcast(json.dumps({'type': 'vision_error', 'msg': str(e)}))
        return

    # Duplicate tag_id in one frame: can't safely tell which is real, drop both.
    by_id: dict = {}
    duplicate_ids: set = set()
    for d in detections:
        if d.tag_id in by_id:
            duplicate_ids.add(d.tag_id)
        else:
            by_id[d.tag_id] = d
    for tid in duplicate_ids:
        by_id.pop(tid, None)

    proposed: dict = {}   # anchor_id -> candidate write
    tag_hits  = []        # non-anchor role hits (tagA/tagB/...)
    unknown_ids = []
    rejected  = []

    for tid, d in by_id.items():
        role = vision_cfg.tag_roles.get(str(tid))
        if role is None:
            unknown_ids.append(tid)
            continue

        aid = parse_anchor_role(role)
        if aid is None:
            tag_hits.append({'role': role, 'apriltag_id': tid,
                              'x': round(d.world_x, 4), 'y': round(d.world_y, 4),
                              'z': round(d.world_z, 4)})
            continue

        # Plausibility gate: reject a glitched detection before it can ever
        # reach anchors.json, rather than trusting it because a tag was seen.
        if cfg_global is not None and cfg_global.bounds:
            xmin, xmax, ymin, ymax = cfg_global.bounds[0], cfg_global.bounds[1], \
                                      cfg_global.bounds[2], cfg_global.bounds[3]
            pad = vision_cfg.bounds_padding_m
            if not (xmin - pad <= d.world_x <= xmax + pad and
                    ymin - pad <= d.world_y <= ymax + pad):
                rejected.append({'anchor_id': aid, 'apriltag_id': tid,
                                  'x': round(d.world_x, 4), 'y': round(d.world_y, 4),
                                  'reason': 'outside workspace bounds'})
                continue

        jump_m = None
        if cfg_global is not None and aid in cfg_global.ids:
            idx = cfg_global.ids.index(aid)
            prev = cfg_global.coords[idx]
            jump_m = float(np.hypot(d.world_x - prev[0], d.world_y - prev[1]))

        proposed[aid] = {
            'anchor_id': aid, 'apriltag_id': tid,
            'x': round(d.world_x, 4), 'y': round(d.world_y, 4), 'z': round(d.world_z, 4),
            'decision_margin': round(d.decision_margin, 1),
            'jump_m': round(jump_m, 3) if jump_m is not None else None,
            'jump_warn': jump_m is not None and jump_m > vision_cfg.anchor_jump_warn_m,
        }

    # "missing" means never detected at all — an anchor that WAS detected but
    # rejected (bounds) already has its own distinct entry in `rejected` and
    # must not also be double-reported here.
    handled_anchor_ids = set(proposed.keys()) | {r['anchor_id'] for r in rejected}
    missing_roles = [role for role in vision_cfg.tag_roles.values()
                      if parse_anchor_role(role) is not None
                      and parse_anchor_role(role) not in handled_anchor_ids]

    # Nothing written yet — stash for the confirm step.
    global _vision_pending, _vision_pending_frames
    _vision_pending = proposed

    overlay = draw_overlay(frame, detections)
    import base64
    ok_o, obuf = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 85])
    ok_r, rbuf = cv2.imencode('.jpg', frame,   [cv2.IMWRITE_JPEG_QUALITY, 90])
    _vision_pending_frames = {
        'overlay': obuf.tobytes() if ok_o else None,
        'raw':     rbuf.tobytes() if ok_r else None,
    }

    if ok_o:
        await broadcast(json.dumps({
            'type': 'vision_frame', 'jpeg_b64': base64.b64encode(obuf.tobytes()).decode('ascii'),
            'width': int(overlay.shape[1]), 'height': int(overlay.shape[0]), 'ts': time.time(),
        }), record=False)

    # Non-anchor role hits: immediate, no confirm needed (no file write).
    for hit in tag_hits:
        await broadcast(json.dumps({
            'type': 'vision_place_result', 'target': 'tag', 'anchor_id': hit['apriltag_id'],
            'x': hit['x'], 'y': hit['y'], 'z': hit['z'],
        }))

    await broadcast(json.dumps({
        'type': 'vision_detect_result',
        'pending': True,
        'proposed': list(proposed.values()),
        'rejected': rejected,
        'missing_roles': missing_roles,
        'unknown_tag_ids': unknown_ids,
        'duplicate_tag_ids': sorted(duplicate_ids),
        'tags': tag_hits,
        'n_detected': len(detections),
    }))


async def _handle_vision_detect_confirm():
    """GUI clicked 'Confirm & write' on the auto-detect review panel. Commits
    whatever _handle_vision_detect proposed last — nothing before this point
    ever touched anchors.json."""
    global cfg_global, pipeline, _vision_pending, _vision_pending_frames

    if not _vision_pending:
        await broadcast(json.dumps({'type': 'vision_error',
                                    'msg': 'Nothing to confirm — run Auto-detect first'}))
        return

    for aid, item in _vision_pending.items():
        update_anchor_json(aid, item['x'], item['y'], item['z'],
                            path=CONFIG_DIR / 'anchors.json', apriltag_id=item['apriltag_id'])

    cfg_global = AnchorConfig.from_json(str(CONFIG_DIR / 'anchors.json'))
    pipeline = LivePipeline(cfg_global)
    if calib_mgr:
        calib_mgr.on_anchors_updated(cfg_global)

    # Audit trail: archive exactly what produced this write (reproducibility
    # — if an anchor coordinate looks wrong months later, the frame that
    # produced it is still on disk).
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    snap_dir = LOG_DIR / 'vision_snapshots' / ts
    snap_dir.mkdir(parents=True, exist_ok=True)
    if _vision_pending_frames.get('raw'):
        (snap_dir / 'vision_snapshot.jpg').write_bytes(_vision_pending_frames['raw'])
    if _vision_pending_frames.get('overlay'):
        (snap_dir / 'vision_overlay.jpg').write_bytes(_vision_pending_frames['overlay'])
    (snap_dir / 'vision_metadata.json').write_text(json.dumps({
        'ts': ts, 'written': list(_vision_pending.values()),
    }, indent=2))

    written = list(_vision_pending.values())
    print(f"[VISION] Auto-detect committed {len(written)} anchor(s) -> {snap_dir}")

    await broadcast(_config_msg())
    await broadcast(json.dumps({'type': 'vision_detect_committed', 'written': written}))

    _vision_pending = {}
    _vision_pending_frames = {}


async def _handle_vision_detect_cancel():
    """GUI clicked 'Cancel' on the auto-detect review panel — discard the
    pending proposal without writing anything."""
    global _vision_pending, _vision_pending_frames
    _vision_pending = {}
    _vision_pending_frames = {}
    await broadcast(json.dumps({'type': 'vision_detect_cancelled'}))


def _anchors_freshness_warning(anchors_path: Path) -> Optional[str]:
    """Non-blocking warning if anchors.json doesn't look like fresh RF-survey
    ground truth. Registering the camera against its own vision-derived (and
    possibly tilt-squeezed) output would just re-encode that error instead
    of fixing it — the whole point of ①ANCH being camera-independent."""
    try:
        with open(anchors_path) as f:
            doc = json.load(f)
    except (OSError, json.JSONDecodeError):
        return None

    survey_method = doc.get('survey_method')
    survey_ts     = doc.get('survey_timestamp')
    vision_ts     = doc.get('vision_updated_at')

    if survey_method != 'multipoint_tag_mediated':
        return ("anchors.json wasn't written by the RF self-survey (①ANCH) — "
                "registering against these coordinates may just re-encode "
                "whatever placed them (e.g. vision click-to-place)")
    if vision_ts and survey_ts and vision_ts > survey_ts:
        return ("anchors.json was touched by vision click-to-place AFTER the "
                "last RF survey — re-run ①ANCH first so registration has "
                "camera-independent ground truth")
    return None


async def _handle_vision_import_camcal(data: dict):
    """GUI clicked 'Import camera_calibration.mat'. Same logic as
    import_camera_calibration.py's CLI (shared, not duplicated — see
    load_camcal_mat/apply_camcal), just triggered from the GUI and applied to
    the live vision_cfg instead of a config path passed on the command line."""
    if vision_cfg is None:
        await broadcast(json.dumps({'type': 'vision_camcal_error',
                                    'msg': 'Vision not enabled — start with --vision'}))
        return

    mat_path = Path(data.get('mat_path')) if data.get('mat_path') else DEFAULT_MAT_PATH
    if not mat_path.exists():
        await broadcast(json.dumps({'type': 'vision_camcal_error',
                                    'msg': f'Not found: {mat_path}'}))
        return

    try:
        camcal = load_camcal_mat(mat_path)
        diff = apply_camcal(vision_cfg, camcal)
    except Exception as e:
        await broadcast(json.dumps({'type': 'vision_camcal_error', 'msg': f'Import failed: {e}'}))
        return

    vision_cfg.save()

    print(f"[VISION] Imported {mat_path.name}: "
          f"fx/fy {diff['before']['fx']:.1f}/{diff['before']['fy']:.1f} -> "
          f"{diff['after']['fx']:.1f}/{diff['after']['fy']:.1f}, "
          f"height {diff['before']['mounting_height_m']:.4f} -> "
          f"{diff['after']['mounting_height_m']:.4f} m")

    await broadcast(json.dumps({
        'type': 'vision_camcal_imported',
        'before': diff['before'], 'after': diff['after'],
        'camcal_date': vision_cfg.camcal_date,
        'tilt_deg_diagnostic': vision_cfg.tilt_deg_diagnostic,
    }))


async def _handle_vision_register_start(data: dict):
    """GUI clicked 'Start' on the ⑦REG world-registration step. Reads
    anchors.json for the click order + ground truth (fixes anchor_ids/
    world_xy for the rest of this round), grabs a frame, and begins a new
    click-collection round. Nothing is solved or written yet."""
    global _reg_pending, _reg_pending_frame

    if vision_cap is None or vision_cfg is None:
        await broadcast(json.dumps({'type': 'vision_register_error',
                                    'msg': 'Vision not enabled — start with --vision'}))
        return
    if cv2 is None:
        await broadcast(json.dumps({'type': 'vision_register_error', 'msg': 'opencv-python not installed'}))
        return

    anchors_path = CONFIG_DIR / 'anchors.json'
    try:
        with open(anchors_path) as f:
            doc = json.load(f)
        anchors = doc['anchors']
        if len(anchors) < MIN_ANCHORS:
            raise ValueError(f"Only {len(anchors)} anchors in anchors.json (need >= {MIN_ANCHORS})")
        anchor_ids = [a['id'] for a in anchors]
        world_xy   = [[a['x'], a['y']] for a in anchors]
    except (OSError, KeyError, ValueError) as e:
        await broadcast(json.dumps({'type': 'vision_register_error', 'msg': f'anchors.json: {e}'}))
        return

    try:
        anchor_z_m = float(data.get('anchor_z_m', 0.0))
    except (TypeError, ValueError):
        await broadcast(json.dumps({'type': 'vision_register_error', 'msg': 'Bad anchor_z_m'}))
        return

    warnings = []
    freshness = _anchors_freshness_warning(anchors_path)
    if freshness:
        warnings.append(freshness)
    warnings.extend(check_point_spread(world_xy, anchor_ids))

    frame = vision_cap.get_latest_frame()
    if frame is None:
        await broadcast(json.dumps({'type': 'vision_register_error', 'msg': 'No frame available yet'}))
        return
    frame = mirror_correct(frame, vision_cfg)

    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        await broadcast(json.dumps({'type': 'vision_register_error', 'msg': 'JPEG encode failed'}))
        return
    _reg_pending_frame = buf.tobytes()

    _reg_pending = {
        'anchor_ids': anchor_ids, 'world_xy': world_xy,
        'anchor_z_m': anchor_z_m, 'clicks': {}, 'solved': None,
    }

    import base64
    await broadcast(json.dumps({
        'type': 'vision_register_frame',
        'jpeg_b64': base64.b64encode(_reg_pending_frame).decode('ascii'),
        'width': int(frame.shape[1]), 'height': int(frame.shape[0]), 'ts': time.time(),
        'anchor_ids': anchor_ids, 'world_xy': world_xy, 'anchor_z_m': anchor_z_m,
        'warnings': warnings,
    }), record=False)


async def _handle_vision_register_click(data: dict):
    """Records one anchor's clicked pixel position during a registration round."""
    if not _reg_pending:
        await broadcast(json.dumps({'type': 'vision_register_error',
                                    'msg': 'No registration in progress — click Start first'}))
        return
    try:
        anchor_id = int(data['anchor_id'])
        u = float(data['u'])
        v = float(data['v'])
    except (KeyError, TypeError, ValueError):
        await broadcast(json.dumps({'type': 'vision_register_error', 'msg': 'Bad click data'}))
        return
    if anchor_id not in _reg_pending['anchor_ids']:
        await broadcast(json.dumps({'type': 'vision_register_error',
                                    'msg': f'Anchor {anchor_id} is not in this registration round'}))
        return

    _reg_pending['clicks'][anchor_id] = [u, v]
    clicked = list(_reg_pending['clicks'].keys())
    remaining = [aid for aid in _reg_pending['anchor_ids'] if aid not in _reg_pending['clicks']]
    await broadcast(json.dumps({
        'type': 'vision_register_progress', 'clicked': clicked, 'remaining': remaining,
    }))


async def _handle_vision_register_solve():
    """All anchors clicked — solve the rigid registration. Does not write
    anything; see _handle_vision_register_confirm for that."""
    if not _reg_pending:
        await broadcast(json.dumps({'type': 'vision_register_error',
                                    'msg': 'No registration in progress — click Start first'}))
        return
    anchor_ids = _reg_pending['anchor_ids']
    clicks = _reg_pending['clicks']
    remaining = [aid for aid in anchor_ids if aid not in clicks]
    if remaining:
        await broadcast(json.dumps({'type': 'vision_register_error',
                                    'msg': f'Still need clicks for anchors: {remaining}'}))
        return

    pixel_clicks_uv = [clicks[aid] for aid in anchor_ids]
    try:
        reg = solve_registration(anchor_ids, _reg_pending['world_xy'], _reg_pending['anchor_z_m'],
                                  pixel_clicks_uv, vision_cfg)
    except ValueError as e:
        await broadcast(json.dumps({'type': 'vision_register_error', 'msg': str(e)}))
        return

    _reg_pending['solved'] = reg
    await broadcast(json.dumps({
        'type': 'vision_register_result',
        'R': reg.R, 't': reg.t,
        'rmse_m': round(reg.rmse_m, 5), 'max_residual_m': round(reg.max_residual_m, 5),
        'residuals_m': [round(r, 5) for r in reg.residuals_m],
        'anchor_ids': reg.anchor_ids,
        'implied_height_m': round(reg.implied_height_m, 4),
        'implied_tilt_deg': round(reg.implied_tilt_deg, 3),
        'gate_passed': reg.gate_passed,
        'warn_rmse_m': WARN_RMSE_M, 'block_rmse_m': BLOCK_RMSE_M, 'height_tol_m': HEIGHT_TOL_M,
    }))


async def _handle_vision_register_confirm(data: dict):
    """GUI clicked 'Confirm & save' (or 'Override & save anyway'). Direct
    port of registerWorldFrameClicks.m's save-time sanity gate (lines
    344-357): refuses to save a failed-gate solve unless explicitly
    overridden."""
    global world_registration, _reg_pending, _reg_pending_frame

    reg: Optional[WorldRegistration] = _reg_pending.get('solved') if _reg_pending else None
    if reg is None:
        await broadcast(json.dumps({'type': 'vision_register_error',
                                    'msg': 'Nothing to confirm — solve a registration first'}))
        return

    override = bool(data.get('override'))
    if not reg.gate_passed and not override:
        await broadcast(json.dumps({
            'type': 'vision_register_error',
            'msg': (f'RMSE {reg.rmse_m*1000:.0f} mm / implied height '
                    f'{reg.implied_height_m:.2f} m failed the sanity check — '
                    f'confirm with override to save anyway'),
        }))
        return

    reg.gate_override = override and not reg.gate_passed
    reg_path = CONFIG_DIR / 'kinect_world_registration.json'
    reg.save(reg_path)
    world_registration = reg

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    snap_dir = LOG_DIR / 'vision_snapshots' / ts
    snap_dir.mkdir(parents=True, exist_ok=True)
    if _reg_pending_frame:
        (snap_dir / 'registration_frame.jpg').write_bytes(_reg_pending_frame)
    reg.save(snap_dir / 'registration_result.json')

    print(f"[REG] World registration saved -> {reg_path}  "
          f"(RMSE {reg.rmse_m*1000:.1f} mm, height {reg.implied_height_m:.3f} m, "
          f"tilt {reg.implied_tilt_deg:.2f}°)  archived -> {snap_dir}")

    await broadcast(json.dumps({'type': 'vision_register_committed',
                                'rmse_m': reg.rmse_m, 'implied_height_m': reg.implied_height_m,
                                'implied_tilt_deg': reg.implied_tilt_deg,
                                'gate_override': reg.gate_override}))
    _reg_pending = {}
    _reg_pending_frame = None


async def _handle_vision_register_cancel():
    """Discard the in-progress registration round without saving anything."""
    global _reg_pending, _reg_pending_frame
    _reg_pending = {}
    _reg_pending_frame = None
    await broadcast(json.dumps({'type': 'vision_register_cancelled'}))


def _hwcalib_git_commit() -> Optional[str]:
    """Short git hash for provenance in the calibration record. None outside a repo."""
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'], cwd=str(SCRIPT_DIR),
            stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return None


async def _handle_hwcalib_req(data: dict):
    """GUI clicked 'Calibrate All'. Start the per-anchor queue.

    Per-anchor true distance: if the GUI supplies the tag's known (x, y), each
    anchor's true distance is the exact geometric distance to it (anchors are
    almost never equidistant from the tag in a real layout, so forcing one
    shared distance on all of them was itself a source of bad calibration).
    Falls back to a single shared distance if no tag point is given.
    """
    ids = list(data.get('anchor_ids', []))
    if not ids:
        return

    hwcalib.queue             = ids
    hwcalib.current           = None
    hwcalib.retries           = 0
    hwcalib.active            = True
    hwcalib.last_msg_ts       = time.time()
    hwcalib.iteration_history = {}
    hwcalib.git_commit        = _hwcalib_git_commit()

    # Only compute/echo distances for the anchors actually being calibrated
    # (the requested subset), not every configured anchor.
    id_to_idx = {int(aid): i for i, aid in enumerate(cfg_global.ids)}
    req_ids   = [int(aid) for aid in ids if int(aid) in id_to_idx]

    tag_x, tag_y = data.get('tag_x'), data.get('tag_y')
    if tag_x is not None and tag_y is not None:
        hwcalib.tag_xy    = (float(tag_x), float(tag_y))
        hwcalib.distances = {
            aid: int(round(1000.0 * math.hypot(float(tag_x) - cfg_global.coords[id_to_idx[aid], 0],
                                                float(tag_y) - cfg_global.coords[id_to_idx[aid], 1])))
            for aid in req_ids
        }
    else:
        hwcalib.tag_xy    = None
        dist_mm           = int(data.get('true_dist_mm', 2828))
        hwcalib.distances = {aid: dist_mm for aid in req_ids}

    # Echo computed distances immediately so the GUI can show them per-anchor
    # before the first HWCALIB_PROG arrives.
    await broadcast(json.dumps({'type': 'HWCALIB_REQ_ACK', 'distances': hwcalib.distances}))

    _hwcalib_send_current()
    asyncio.create_task(_hwcalib_watchdog())


def _hwcalib_send_current():
    """Send the HWCALIB command for the anchor at the head of the queue."""
    if not hwcalib.queue:
        return
    aid = hwcalib.queue[0]
    hwcalib.current     = aid
    hwcalib.last_msg_ts = time.time()
    dist_mm = hwcalib.distances.get(aid, 2828)
    send_to_tag(f"HWCALIB,{aid},{dist_mm}\n")
    print(f"[HWCAL] -> anchor {aid} at {dist_mm} mm  ({len(hwcalib.queue)} anchors queued)")


async def _handle_hwcalib_line(line: str):
    """Forward HWCALIB_PROG/DONE/FAIL from tag to GUI; advance queue on completion.

    Any HWCALIB_* line (PROG included) refreshes the stall watchdog's
    liveness clock — a healthy calibration emits ~14 PROG lines per anchor, so
    silence on that channel, not just on DONE/FAIL, means the tag is stuck.
    """
    hwcalib.last_msg_ts = time.time()
    # [CAL-HOST] diagnostic (HWCALIB root-cause investigation): the only prior
    # console output on this path was the outgoing command in
    # _hwcalib_send_current(); this gives a host-clock arrival record for every
    # incoming line, on the same clock hwcalib_trace.py uses for its merged trace.
    print(f"[CAL-HOST] {time.time():.3f}  {line}")

    parts    = line.split(',')
    msg_type = parts[0]

    await broadcast(json.dumps({'type': msg_type, 'raw': line}))

    if msg_type == 'HWCALIB_PROG' and len(parts) >= 6:
        try:
            aid = int(parts[1])
            hwcalib.iteration_history.setdefault(aid, []).append({
                'it': int(parts[2]), 'delay': int(parts[3]),
                'mean_mm': int(parts[4]), 'err_mm': int(parts[5]),
            })
        except (ValueError, IndexError):
            pass
        return

    if msg_type in ('HWCALIB_DONE', 'HWCALIB_FAIL'):
        await _hwcalib_advance(msg_type, parts)


async def _hwcalib_advance(msg_type: str, parts: list):
    """Record the just-finished anchor's result (if any) and move the queue on
    — called for a real tag reply AND for the watchdog's synthetic timeout."""
    try:
        aid = int(parts[1]) if len(parts) > 1 else None
    except ValueError:
        aid = None

    if msg_type == 'HWCALIB_DONE' and aid is not None and len(parts) >= 3:
        final_delay = int(parts[2])
        final_mean  = int(parts[3]) if len(parts) >= 4 else None
        residual    = int(parts[4]) if len(parts) >= 5 else None
        true_mm     = hwcalib.distances.get(aid)
        old_delay   = (calib_mgr._level1_hw.get(aid, {}).get('antenna_delay_ticks')
                       if calib_mgr else None)
        delta       = (final_delay - old_delay) if old_delay is not None else None

        print(f"[CAL] Anchor A{aid}  old {old_delay if old_delay is not None else 'factory'} "
              f"-> new {final_delay}" + (f"  (Δ{delta:+d})" if delta is not None else "")
              + (f"  true {true_mm} mm" if true_mm is not None else "")
              + (f"  final {final_mean} mm" if final_mean is not None else "")
              + (f"  resid {residual:+d} mm" if residual is not None else ""))

        if calib_mgr:
            try:
                calib_mgr.record_hwcalib_result(
                    aid, final_delay,
                    true_mm=true_mm, final_mean_mm=final_mean, residual_mm=residual,
                    git_commit=hwcalib.git_commit,
                    iteration_history=hwcalib.iteration_history.get(aid),
                )
            except (ValueError, IndexError):
                pass
    elif aid is not None:
        print(f"[HWCAL] Anchor {aid} FAIL  reason={parts[2] if len(parts) > 2 else '?'}")

    if hwcalib.queue and hwcalib.queue[0] == aid:
        hwcalib.queue.pop(0)
    hwcalib.current = None
    hwcalib.retries = 0

    if hwcalib.queue:
        await asyncio.sleep(HWCALIB_PACE_S)   # let the tag/anchor settle before the next one
        _hwcalib_send_current()
    else:
        hwcalib.active = False
        if calib_mgr:
            calib_mgr.save_level1()
        await broadcast(json.dumps({'type': 'HWCALIB_ALL_DONE'}))
        print("[HWCAL] All anchors calibrated")


async def _hwcalib_watchdog():
    """Backstop for a lost command/reply: while a run is active, if the current
    anchor goes silent (no HWCALIB_* line, PROG included) for HWCALIB_STALL_S,
    retransmit its command once; a second stall synthesizes a FAIL so the
    queue always drains instead of hanging forever."""
    while hwcalib.active:
        await asyncio.sleep(HWCALIB_WATCHDOG_POLL_S)
        if not hwcalib.active or hwcalib.current is None:
            continue
        silent = time.time() - hwcalib.last_msg_ts
        if silent <= HWCALIB_STALL_S:
            continue
        if hwcalib.retries < 1:
            hwcalib.retries += 1
            hwcalib.last_msg_ts = time.time()
            aid = hwcalib.current
            dist_mm = hwcalib.distances.get(aid, 2828)
            print(f"[HWCAL] anchor {aid} silent {silent:.1f}s — retransmitting")
            send_to_tag(f"HWCALIB,{aid},{dist_mm}\n")
        else:
            aid = hwcalib.current
            print(f"[HWCAL] anchor {aid} stalled after retry — failing and advancing")
            await _hwcalib_advance('HWCALIB_FAIL', ['HWCALIB_FAIL', str(aid), 'timeout'])


async def _handle_diag_line(line: str):
    """Parse the master tag's per-cycle time budget and broadcast it typed
    (Diagnostics view: TDMA budget bar, rate, sweep history).

    DIAG,v1,<cycle>,<cycle_start_us>,<master_sweep_us>,<slave_sweep_us>,
            <uart_wait_us>,<proc_us>,<idle_us>,<total_us>
    Broadcast (and therefore recorded); console summary at a low rate.
    """
    global _diag_count
    p = line.split(',')
    if len(p) < 10:
        return
    try:
        msg = {
            'type': 'diag',
            'cycle':          int(p[2]),
            'cycle_start_us': int(p[3]),
            'master_us':      int(p[4]),
            'slave_us':       int(p[5]),
            'uart_us':        int(p[6]),
            'proc_us':        int(p[7]),
            'idle_us':        int(p[8]),
            'total_us':       int(p[9]),
        }
    except ValueError:
        return
    msg['rate_hz'] = round(1e6 / msg['total_us'], 2) if msg['total_us'] else 0.0
    await broadcast(json.dumps(msg))

    _diag_count += 1
    if _diag_count % 50 == 1:
        print(f"[DIAG] cycle={msg['cycle']} master={msg['master_us']/1e3:.1f}ms"
              f" slave={msg['slave_us']/1e3:.1f}ms uart={msg['uart_us']/1e3:.1f}ms"
              f" proc={msg['proc_us']/1e3:.1f}ms idle={msg['idle_us']/1e3:.1f}ms"
              f" total={msg['total_us']/1e3:.1f}ms ({msg['rate_hz']:.1f} Hz)")


_diag_count = 0


async def on_line(line: str, t_rx: float | None = None) -> None:
    """One raw line from the tag: route it, solve it, publish it.

    Previously this body existed twice — once in `udp_reader`, once in
    `serial_reader` — as near-verbatim copies that were already drifting. There
    is one copy now; the transport difference is entirely inside host/ingest.py.
    """
    if line.startswith('HWCALIB_'):
        await _handle_hwcalib_line(line)
        return

    if line.startswith('DIAG,'):
        await _handle_diag_line(line)
        return

    # A run in progress records every sweep on the host clock, before any
    # solving — the report re-solves offline, so raw ranges are what matter.
    cap = getattr(getattr(app, 'state', None), 'capture', None)
    if cap is not None:
        pkt = FrameParser.parse(line)
        if pkt.valid:
            cap.add_sweep(pkt, t_rx)

    result = pipeline.process(line, t_rx)
    if result:
        await broadcast(result)
        print(f"[FIX] tag=0x{result['tag_id']:02X}"
              f"  ({result['ex']:+.3f}, {result['ey']:+.3f})"
              f"  rms={result['rmse']:.3f}"
              f"  pkt#{result['pkt_n']}")

        # Rigid-body pairing: emit the authoritative robot pose once the
        # cycle's partner fix is in (or degraded solo).
        robot = rigid_body.add_fix(result)
        if robot:
            await broadcast(robot)

        if _survey_capture_active:
            survey_msg = _process_survey_sweep(result)
            if survey_msg:
                await broadcast(survey_msg)



async def main(args):
    global app, pipeline, cfg_global, calib_mgr, vision_cap, vision_cfg, world_registration

    rigid_body.baseline_nominal_m = float(args.baseline)

    if args.vision:
        vision_cfg = VisionConfig.load()
        world_registration = load_registration(CONFIG_DIR / 'kinect_world_registration.json')
        if world_registration is not None:
            print(f"[REG]  World registration loaded  "
                  f"(RMSE {world_registration.rmse_m*1000:.1f} mm, "
                  f"height {world_registration.implied_height_m:.3f} m, "
                  f"tilt {world_registration.implied_tilt_deg:.2f}°)")
        else:
            print("[REG]  No world registration found — using flat-plane fallback "
                  "(run the ⑦REG GUI step to correct camera tilt)")
        vision_cap = KinectCapture()
        try:
            vision_cap.start()
            print(f"[VISION] Kinect capture started  "
                  f"(mounting_height={vision_cfg.mounting_height_m:.3f} m, "
                  f"fx={vision_cfg.fx:.1f})")
        except Exception as e:
            print(f"[VISION] Failed to start Kinect capture: {e}")
            vision_cap = None
            vision_cfg = None

    anchors_path = CONFIG_DIR / 'anchors.json'
    if anchors_path.exists():
        cfg_global = AnchorConfig.from_json(str(anchors_path))
        print(f"[CFG] Loaded {len(cfg_global.ids)} anchors from {anchors_path}")
    else:
        cfg_global = AnchorConfig()
        print("[CFG] Using built-in 4-anchor example (no anchors.json found)")

    pipeline = LivePipeline(cfg_global)

    calib_mgr = CalibrationManager(cfg_global, CONFIG_DIR / 'calibration.json')
    calib_mgr.try_load()   # Tier-1 hardware-delay traceability only (no software bias)

    # The app owns the plumbing: session recorder, client set, job registry,
    # ingest threads. One NDJSON line per broadcast message is the replay file.
    rec_path = LOG_DIR / f"session_{datetime.now():%Y%m%d_%H%M%S}.ndjson"
    app = App(log_path=rec_path)
    app.state.pipeline = pipeline
    app.state.anchors = cfg_global
    app.register_handlers(HANDLERS)
    app.register_handlers(ROVER_HANDLERS)
    app.set_config_msg(_config_msg)
    await app.record_keyframes()     # config + job_kinds, so replay can render
    print(f"[REC] {rec_path}")

    print(f"[WS]  WebSocket server on ws://localhost:{args.ws_port}")

    async with websockets.serve(app.ws_handler, '0.0.0.0', args.ws_port):
        # Server is listening now (serve() has already bound by the time
        # `async with` enters the block) — safe to open the browser here
        # rather than racing it against server startup.
        gui_path = SCRIPT_DIR / 'gui' / 'gui.html'
        if not args.no_browser:
            print(f"[GUI]  Opening {gui_path}")
            webbrowser.open(gui_path.as_uri())
        else:
            print(f"      Open {gui_path} in your browser\n")

        # Ingest runs on its own thread, so a long job can never starve it.
        if args.serial:
            transport = SerialTransport(args.serial, args.baud)
            print(f"[SER] Listening on {args.serial} @ {args.baud} baud...")
        else:
            tag_ip   = calib_mgr._tag_ip   if calib_mgr else '255.255.255.255'
            tag_port = calib_mgr._tag_port if calib_mgr else 4100
            transport = UdpTransport(args.udp_port, tag_ip, tag_port)
            print(f"[UDP] Listening on port {args.udp_port}...")
        app.attach_transport(transport, on_line)

        await asyncio.Event().wait()      # run until interrupted


def parse_args():
    p = argparse.ArgumentParser(description='UWB RTLS Python host')
    p.add_argument('--udp-port', type=int, default=DEFAULT_UDP_PORT)
    p.add_argument('--ws-port',  type=int, default=DEFAULT_WS_PORT)
    p.add_argument('--serial',   type=str, default=None,
                   help='Serial port for RTLS data (e.g. /dev/ttyUSB0)')
    p.add_argument('--baud',     type=int, default=DEFAULT_BAUD)
    p.add_argument('--baseline', type=float, default=BASELINE_NOMINAL_M,
                   help='Nominal dual-tag baseline in metres (default %(default)s)')
    p.add_argument('--vision', action='store_true',
                   help='Enable the overhead Kinect click-to-place GUI panel '
                        '(requires python/vision/ deps — see docs/kinect_vision_setup.md)')
    p.add_argument('--no-browser', action='store_true',
                   help="Don't auto-open gui.html in a browser on startup")
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        # The rover stops before anything else: the firmware has no watchdog, so
        # a host that exits without stopping leaves the motors running.
        rover = getattr(getattr(app, 'state', None), 'rover', None)
        if rover is not None:
            try:
                rover.stop()
                rover.link.close()
            except Exception:
                pass
        if app:
            asyncio.run(app.aclose())
        if vision_cap:
            vision_cap.stop()
