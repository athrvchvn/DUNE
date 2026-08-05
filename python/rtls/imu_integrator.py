"""
ImuIntegrator — body-frame to world-frame acceleration transform.

Coordinate conventions
----------------------
BODY frame:  BNO085 sensor frame, fixed to the tag hardware.
WORLD frame: Fixed, ENU (East-North-Up). The BNO085 rotation vector encodes
             the body→world rotation (quaternion q such that v_world = R(q) @ v_body).
             Z is aligned with gravity-up.

The BNO085 SH2_LINEAR_ACCELERATION report already removes gravity in the body
frame, so the output is pure kinematic acceleration in body coordinates.
We rotate it to world frame, then take the XY components for the 2D EKF.
"""

import math
import numpy as np
from .frame_parser import ImuData


def _quat_to_rot(quat) -> np.ndarray:
    """Quaternion [w, x, y, z] → 3×3 rotation matrix (body→world)."""
    w, x, y, z = quat
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=float)


class ImuIntegrator:
    """Transform BNO085 IMU data into world-frame quantities for the EKF."""

    def transform_accel(self, imu: ImuData) -> np.ndarray | None:
        """
        Rotate body-frame linear acceleration to world frame and return the
        XY components as a (2,) array, or None if orientation is unreliable.

        Returns None when imu.status == 0 (BNO085 reports unreliable fusion).
        In that case the EKF falls back to constant-velocity prediction.
        """
        if imu.status == 0:
            return None
        R = _quat_to_rot(imu.quat)
        a_world = R @ np.array(imu.acc, dtype=float)
        return a_world[:2]

    def yaw_deg(self, imu: ImuData) -> float:
        """Extract world-frame yaw angle in degrees from the quaternion (ZYX Euler)."""
        w, x, y, z = imu.quat
        return math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

    def roll_pitch_yaw_deg(self, imu: ImuData) -> tuple[float, float, float]:
        """Return (roll, pitch, yaw) in degrees from the quaternion."""
        w, x, y, z = imu.quat
        roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
        sinp  = max(-1.0, min(1.0, 2*(w*y - z*x)))
        pitch = math.degrees(math.asin(sinp))
        yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
        return roll, pitch, yaw
