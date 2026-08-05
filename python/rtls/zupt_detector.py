"""
ZuptDetector — sliding-window zero-velocity detection for ZUPT injection.

Why variance-based (not just instantaneous threshold)
-----------------------------------------------------
A single low-magnitude sample could be a vibration zero-crossing rather than
true stationarity. Requiring both low mean AND low variance over ~800 ms of
data (8 packets at 10 Hz) guards against transient zero-crossings.

Why include EKF velocity
------------------------
If the filter says the tag is moving (speed > threshold), do not ZUPT even
if the accelerometer momentarily looks small — this prevents false ZUPT
during constant-velocity phases where linear acceleration is near zero.
"""

from collections import deque
import numpy as np


class ZuptDetector:

    def __init__(self, cfg: dict):
        n = max(2, int(cfg.get('zupt_window', 8)))
        self._buf         = deque(maxlen=n)
        self._thresh_mean = cfg.get('zupt_accel_mean', 0.10)    # m/s²
        self._thresh_var  = cfg.get('zupt_accel_var',  0.005)   # (m/s²)²
        self._thresh_gyro = cfg.get('zupt_gyro',       0.087)   # rad/s
        self._thresh_spd  = cfg.get('zupt_speed',      0.05)    # m/s
        self._debounce    = max(1, int(cfg.get('zupt_debounce', 3)))
        self._streak      = 0
        self.active       = False

    def update(self,
               a_world_xy,          # (2,) ndarray or None if IMU unavailable
               gyro,                 # list [gx,gy,gz] rad/s, or [] / None
               v_ekf: np.ndarray,   # (2,) current EKF velocity estimate
               ) -> bool:
        """
        Update the detector with a new sample. Returns True when ZUPT should
        be injected (stationary for long enough to pass debounce).
        """
        # Use world-XY acceleration magnitude; fall back to 0 if no IMU
        if a_world_xy is not None:
            mag = float(np.linalg.norm(a_world_xy))
        else:
            mag = 0.0
        self._buf.append(mag)

        if len(self._buf) < 2:
            self.active = False
            return False

        mean_a = float(np.mean(self._buf))
        var_a  = float(np.var(self._buf))

        # Gyro check — use only Z axis (yaw rate) as primary rotation indicator
        if gyro and len(gyro) >= 3:
            gyro_ok = abs(gyro[2]) < self._thresh_gyro
        else:
            gyro_ok = True  # no gyro → don't use it as a veto

        speed     = float(np.linalg.norm(v_ekf))
        speed_ok  = speed < self._thresh_spd

        stationary = (
            mean_a < self._thresh_mean
            and var_a  < self._thresh_var
            and gyro_ok
            and speed_ok
        )

        self._streak = (self._streak + 1) if stationary else 0
        self.active  = self._streak >= self._debounce
        return self.active

    def reset(self) -> None:
        """Force-clear the detector (e.g. after an EKF reset)."""
        self._buf.clear()
        self._streak = 0
        self.active  = False
