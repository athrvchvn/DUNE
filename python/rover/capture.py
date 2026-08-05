"""Record a rover run: UWB sweeps, drive commands and truth on one clock.

Writes both artifacts a run needs:

* **`.ndjson`** — appended as the run happens, so a crash mid-run still leaves
  everything up to that moment. This is the safety net.
* **`.mat`** — written at the end, in the *exact* schema
  `matlab/scripts/rover_run_report.m` reads, so a Python-captured run can be
  scored by either pipeline. That is the point: the two pipelines stay peers.

**Everything is stamped with one clock.** `time.time()` is read once per record
on the host, for UWB sweeps and rover samples alike, so alignment is exact
rather than inferred. The tag's own `t_ms` is kept alongside but never used to
align — it restarts every boot.

The schema is transcribed from a real capture rather than from the MATLAB
source, because the source builds it incrementally and the shapes are easier to
get wrong than to read off. HDF5 stores MATLAB arrays transposed, so everything
is written `.T`; `reports/matfile.py` transposes back on the way in, and a test
round-trips through it.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SCHEMA = 'dune_rover_capture_v1'


@dataclass
class CaptureStats:
    n_uwb: int = 0
    n_rover: int = 0
    n_truth: int = 0
    started: float = 0.0
    duration_s: float = 0.0

    def to_dict(self) -> dict:
        return {'n_uwb': self.n_uwb, 'n_rover': self.n_rover,
                'n_truth': self.n_truth,
                'duration_s': round(self.duration_s, 2)}


class RunCapture:
    """Collects a run in memory and on disk, then writes the MATLAB `.mat`."""

    def __init__(self, out_dir, *, anchor_ids, front_tag: int = 241,
                 rear_tag: int = 240, baseline_m: float = 0.527,
                 apriltag_offset_body=(0.0, 0.0), tag_z: float = 0.24,
                 anchors_file: str = '', anchors_layout: str = '',
                 stamp: str | None = None):
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.stamp = stamp or time.strftime('%Y%m%d_%H%M%S')
        self.anchor_ids = np.asarray(list(anchor_ids), dtype=int)
        self.front_tag, self.rear_tag = int(front_tag), int(rear_tag)
        self.baseline_m = float(baseline_m)
        self.apriltag_offset_body = np.asarray(apriltag_offset_body, dtype=float)
        self.tag_z = float(tag_z)
        self.anchors_file = str(anchors_file)
        self.anchors_layout = str(anchors_layout)

        self.path_ndjson = self.dir / f'rover_uwb_{self.stamp}.ndjson'
        self.path_mat = self.dir / f'rover_uwb_{self.stamp}.mat'

        self._uwb: list[dict] = []
        self._rover: list[dict] = []
        self._lock = threading.Lock()
        self._fh = open(self.path_ndjson, 'w')
        self.stats = CaptureStats(started=time.time())
        self._closed = False

        self._write({'type': 'capture_meta', 'schema': SCHEMA,
                     'stamp': self.stamp, 't_host': self.stats.started,
                     'anchor_ids': self.anchor_ids.tolist(),
                     'front_tag': self.front_tag, 'rear_tag': self.rear_tag,
                     'baseline_m': self.baseline_m, 'tag_z': self.tag_z,
                     'apriltag_offset_body': self.apriltag_offset_body.tolist(),
                     'anchors_file': self.anchors_file,
                     'anchors_layout': self.anchors_layout})

    # ── recording ────────────────────────────────────────────────────────────

    def _write(self, rec: dict) -> None:
        try:
            self._fh.write(json.dumps(rec) + '\n')
            self._fh.flush()
        except Exception:
            pass          # a capture must never take down the run it records

    def add_sweep(self, pkt, t_host: float | None = None) -> None:
        """One UWB sweep, from either tag."""
        if self._closed:
            return
        t_host = time.time() if t_host is None else t_host
        ids = np.asarray(pkt.ids, dtype=int).ravel()
        dist = np.asarray(pkt.dist, dtype=float).ravel()
        rx = np.asarray(getattr(pkt, 'rx', getattr(pkt, 'q', [])), dtype=float).ravel()
        fp = np.asarray(getattr(pkt, 'fp', []), dtype=float).ravel()

        row = {'t_host': t_host, 'tag': int(pkt.tag_id),
               't_ms': getattr(pkt, 't_ms', None),
               'ids': ids.tolist(), 'R': dist.tolist(),
               'RX': rx.tolist(), 'FP': fp.tolist()}
        imu = getattr(pkt, 'imu', None)
        if imu is not None:
            row['quat'] = [float(v) for v in imu.quat]
            row['acc'] = [float(v) for v in imu.acc]
            row['gyro'] = [float(v) for v in (imu.gyro or [])]

        with self._lock:
            self._uwb.append(row)
            self.stats.n_uwb += 1
        self._write(dict(row, type='uwb'))

    def add_rover(self, *, v_cmd: float, omega_cmd: float,
                  truth_xy=None, truth_yaw_deg=None, imu: dict | None = None,
                  motors: dict | None = None, t_host: float | None = None) -> None:
        """One control-loop sample: what was commanded, and truth if available."""
        if self._closed:
            return
        t_host = time.time() if t_host is None else t_host
        row = {'t_host': t_host, 'V_cmd': float(v_cmd),
               'omega_cmd': float(omega_cmd)}
        if truth_xy is not None and np.all(np.isfinite(truth_xy)):
            row['apriltag_pos'] = [float(truth_xy[0]), float(truth_xy[1]),
                                   float(truth_xy[2]) if len(truth_xy) > 2 else 0.0]
            with self._lock:
                self.stats.n_truth += 1
        if truth_yaw_deg is not None and np.isfinite(truth_yaw_deg):
            row['apriltag_yaw_deg'] = float(truth_yaw_deg)
        if imu:
            row['imu_euler'] = [float(imu.get('roll', 0.0)),
                                float(imu.get('pitch', 0.0)),
                                float(imu.get('yaw', 0.0))]
            row['imu_accuracy'] = float(imu.get('accuracy', 0.0))
        if motors:
            row['motor_rpm'] = [float(x) for x in
                                (motors.get('actual_speeds') or [0] * 6)]
            row['commanded_motor_rpm'] = [float(x) for x in
                                          (motors.get('commanded_speeds') or [0] * 6)]

        with self._lock:
            self._rover.append(row)
            self.stats.n_rover += 1
        self._write(dict(row, type='rover'))

    # ── finishing ────────────────────────────────────────────────────────────

    def close(self) -> dict:
        """Stop recording and write the `.mat`. Safe to call more than once."""
        if self._closed:
            return self.summary()
        self._closed = True
        self.stats.duration_s = time.time() - self.stats.started
        self._write({'type': 'capture_end', **self.stats.to_dict()})
        try:
            self._fh.close()
        except Exception:
            pass
        try:
            self.write_mat()
        except Exception as e:
            # The ndjson is already safe on disk; say so rather than losing the
            # run because the .mat writer failed.
            return dict(self.summary(), mat_error=f"{type(e).__name__}: {e}")
        return self.summary()

    def summary(self) -> dict:
        return {'stamp': self.stamp, 'ndjson': str(self.path_ndjson),
                'mat': str(self.path_mat) if self.path_mat.exists() else None,
                **self.stats.to_dict()}

    def write_mat(self) -> Path:
        """Write the v7.3 `.mat` in the schema rover_run_report.m reads."""
        import h5py

        with self._lock:
            uwb = list(self._uwb)
            rover = list(self._rover)

        n_anch = self.anchor_ids.size
        col = {int(a): k for k, a in enumerate(self.anchor_ids)}

        nu = len(uwb)
        R = np.full((nu, n_anch), np.nan)
        RX = np.full((nu, n_anch), np.nan)
        FP = np.full((nu, n_anch), np.nan)
        t_u = np.zeros(nu)
        tag = np.zeros(nu)
        quat = np.full((nu, 4), np.nan)
        acc = np.full((nu, 3), np.nan)
        gyro = np.full((nu, 3), np.nan)

        for i, s in enumerate(uwb):
            t_u[i] = s['t_host']
            tag[i] = s['tag']
            for k, aid in enumerate(s['ids']):
                c = col.get(int(aid))
                if c is None:
                    continue
                R[i, c] = s['R'][k]
                if k < len(s['RX']):
                    RX[i, c] = s['RX'][k]
                if k < len(s['FP']):
                    FP[i, c] = s['FP'][k]
            for name, arr in (('quat', quat), ('acc', acc), ('gyro', gyro)):
                v = s.get(name)
                if v and len(v) == arr.shape[1]:
                    arr[i, :] = v

        nr = len(rover)
        t_r = np.zeros(nr)
        v_cmd = np.zeros(nr)
        om_cmd = np.zeros(nr)
        apos = np.full((nr, 3), np.nan)
        aeul = np.full((nr, 3), np.nan)
        imu_eul = np.full((nr, 3), np.nan)
        motor = np.full((nr, 6), np.nan)
        cmd_motor = np.full((nr, 6), np.nan)
        for i, r in enumerate(rover):
            t_r[i] = r['t_host']
            v_cmd[i] = r['V_cmd']
            om_cmd[i] = r['omega_cmd']
            if 'apriltag_pos' in r:
                apos[i, :] = r['apriltag_pos']
            if 'apriltag_yaw_deg' in r:
                aeul[i, 2] = r['apriltag_yaw_deg']
            for name, arr in (('imu_euler', imu_eul), ('motor_rpm', motor),
                              ('commanded_motor_rpm', cmd_motor)):
                if name in r and len(r[name]) == arr.shape[1]:
                    arr[i, :] = r[name]

        t0 = t_r[0] if nr else (t_u[0] if nu else 0.0)

        with h5py.File(str(self.path_mat), 'w') as f:
            # HDF5 stores MATLAB arrays transposed, so everything goes in as .T.
            d = f.create_group('data')
            d.create_dataset('t_posix', data=t_r.reshape(-1, 1).T)
            d.create_dataset('time', data=(t_r - t0).reshape(-1, 1).T)
            d.create_dataset('V_cmd', data=v_cmd.reshape(-1, 1).T)
            d.create_dataset('omega_cmd', data=om_cmd.reshape(-1, 1).T)
            d.create_dataset('apriltag_pos', data=apos.T)
            d.create_dataset('apriltag_euler', data=aeul.T)
            d.create_dataset('imu_euler', data=imu_eul.T)
            d.create_dataset('motor_rpm', data=motor.T)
            d.create_dataset('commanded_motor_rpm', data=cmd_motor.T)

            u = d.create_group('uwb')
            u.create_dataset('t_posix', data=t_u.reshape(-1, 1).T)
            u.create_dataset('t_rel', data=(t_u - t0).reshape(-1, 1).T)
            u.create_dataset('tag', data=tag.reshape(-1, 1).T)
            u.create_dataset('anchorIds',
                             data=self.anchor_ids.astype(float).reshape(1, -1).T)
            u.create_dataset('R', data=R.T)
            u.create_dataset('RX', data=RX.T)
            u.create_dataset('FP', data=FP.T)
            u.create_dataset('quat', data=quat.T)
            u.create_dataset('acc', data=acc.T)
            u.create_dataset('gyro', data=gyro.T)

            m = f.create_group('metadata')
            m.create_dataset('test_start_posix', data=np.array([[t0]]))
            mu = m.create_group('uwb')
            mu.create_dataset('baseline_m', data=np.array([[self.baseline_m]]))
            mu.create_dataset('frontTag', data=np.array([[float(self.front_tag)]]))
            mu.create_dataset('rearTag', data=np.array([[float(self.rear_tag)]]))
            mu.create_dataset('apriltag_offset_body',
                              data=self.apriltag_offset_body.reshape(-1, 1))
            mu.create_dataset('enabled', data=np.array([[1]], dtype=np.uint8))
            _put_str(mu, 'anchors_file', self.anchors_file)
            _put_str(mu, 'anchors_layout', self.anchors_layout)
            _put_str(mu, 'transport', 'python')
        return self.path_mat


def _put_str(group, name: str, text: str) -> None:
    """MATLAB char arrays are uint16 UTF-16 code units, stored as a column."""
    data = np.array([ord(c) for c in (text or ' ')], dtype=np.uint16)
    ds = group.create_dataset(name, data=data.reshape(-1, 1))
    ds.attrs['MATLAB_class'] = np.bytes_(b'char')
