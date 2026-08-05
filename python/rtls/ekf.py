"""2D UWB + IMU EKF with optional per-anchor range-bias states.

Faithful port of ``matlab/+dune/FusionEkf.m``. Changes must be made in both.

Base state ``x = [px py vx vy bax bay]``: world position (m), world velocity
(m/s), horizontal accelerometer bias in the world frame (m/s²).

Two things here are not in the older `fusion_ekf.py` this replaces, and both
exist to fix specific measured failures:

* **Per-anchor range-bias states** (`enable_range_bias`). With tightly-coupled
  range updates each anchor's residual bias is *learned while it answers and
  remembered while it is skipped*, so the position no longer jumps when the
  answering anchor **subset** changes. Common-mode bias stays in position — it
  is unobservable — but the per-anchor differences are held.
* **`still_mode`**, which caps process noise so a parked tag can actually pin.
  ZUPT alone clamps velocity, but constant-velocity Q keeps re-injecting
  position noise, so a stationary tag still wanders.

Updates are chi-squared gated with Huber-style down-weighting: a marginal
outlier has its covariance inflated so the effective NIS sits at the gate,
rather than being thrown away. Only gross outliers past
``robust_hard_factor × gate`` are rejected outright.
"""

from __future__ import annotations

import numpy as np

# chi2(dof, 0.95) for dof 1..3 — the acceptance gate for each update.
CHI2_95 = (3.841, 5.991, 7.815)


class FusionEkf:
    """Port of dune.FusionEkf. Parameters are attributes, as in MATLAB."""

    def __init__(self):
        self.sigma_accel = 0.35        # m/s², accel white noise (IMU predict)
        self.sigma_accel_cv = 0.8      # m/s², CV-fallback model uncertainty
        self.sigma_bias_rw = 0.01      # m/s²/√s, accel-bias random walk
        self.pos_sigma = 0.08          # m, fallback position-measurement sigma
        self.zupt_sigma = 0.05         # m/s, ZUPT pseudo-measurement sigma
        self.max_consec_reject = 10    # reinit from the fix after this many
        self.still_mode = False
        self.sigma_accel_still = 0.05
        self.rb_sigma_init = 0.05      # m, initial per-anchor bias uncertainty
        self.rb_sigma_rw = 0.003       # m/√s, range-bias random walk
        self.still_invert = False
        self.sigma_accel_still_inv = 0.005
        self.rb_sigma_rw_still = 0.02
        self.robust = True
        self.robust_hard_factor = 10.0

        self.x = np.zeros(6)
        self.P = np.eye(6) * 1e4
        self.n_rb = 0
        self.initialized = False
        self.last_nis = float('nan')
        self.last_accepted = True
        # Diagnostics from the most recent accepted update. Recorded sessions
        # carry these, so they are part of the wire contract, not decoration.
        self.last_innovation = np.zeros(2)
        self.last_k_diag = np.zeros(6)
        self.last_correction_mag = 0.0
        self.n_rejected = 0
        self.n_soft = 0                # updates applied with inflated R
        self.n_reinit = 0
        self.consec_reject = 0

    # ── setup ────────────────────────────────────────────────────────────────

    def enable_range_bias(self, m: int) -> None:
        """Add `m` per-anchor bias states. Call BEFORE initialize.

        Ranges passed to `update_ranges` must use this same anchor order — the
        k-th range is tied to the k-th bias state by index, not by anchor id.
        """
        self.n_rb = int(m)
        self.x = np.zeros(6 + self.n_rb)
        self.P = np.eye(6 + self.n_rb) * 1e4
        self.initialized = False

    def initialize(self, pos) -> None:
        n = 6 + self.n_rb
        self.x = np.zeros(n)
        self.x[:2] = np.asarray(pos, dtype=float).ravel()[:2]
        self.P = np.zeros((n, n))
        self.P[:6, :6] = np.diag([0.25, 0.25, 1.0, 1.0, 0.01, 0.01])
        if self.n_rb:
            self.P[6:, 6:] = np.eye(self.n_rb) * self.rb_sigma_init ** 2
        self.initialized = True

    # ── accessors (MATLAB names on the right) ────────────────────────────────

    @property
    def pos(self) -> np.ndarray:
        return self.x[:2].copy()

    @property
    def vel(self) -> np.ndarray:
        return self.x[2:4].copy()

    @property
    def bias(self) -> np.ndarray:
        return self.x[4:6].copy()

    @property
    def range_bias(self) -> np.ndarray:
        return self.x[6:].copy()

    @property
    def speed(self) -> float:
        return float(np.hypot(*self.x[2:4]))

    # ── predict ──────────────────────────────────────────────────────────────

    def predict(self, dt: float, a_world=None) -> None:
        """IMU-driven when a world-frame acceleration is given, CV otherwise."""
        if not self.initialized or dt <= 0:
            return

        a = np.asarray(a_world, dtype=float).ravel() if a_world is not None else None
        use_imu = a is not None and a.size == 2 and bool(np.all(np.isfinite(a)))

        n = 6 + self.n_rb
        I2 = np.eye(2)
        F = np.eye(n)

        if use_imu:
            # p' = p + v·dt + ½(a − ba)dt² ;  v' = v + (a − ba)dt
            F[0:2, 2:4] = dt * I2
            F[0:2, 4:6] = -0.5 * dt * dt * I2
            F[2:4, 4:6] = -dt * I2
            self.x[0:2] = self.x[0:2] + self.x[2:4] * dt + 0.5 * (a - self.x[4:6]) * dt * dt
            self.x[2:4] = self.x[2:4] + (a - self.x[4:6]) * dt
            sa = self.sigma_accel
        else:
            F[0, 2] = dt
            F[1, 3] = dt
            self.x[0:2] = self.x[0:2] + self.x[2:4] * dt
            sa = self.sigma_accel_cv

        rb_rw = self.rb_sigma_rw
        if self.still_mode:
            if self.still_invert and self.n_rb > 0:
                # Noise-budget inversion: while still, the position is known to
                # be constant but the ranges keep breathing. Clamp position Q to
                # ~0 and open the per-anchor bias random walk, so the breathing
                # is absorbed by the bias states instead of dragging position.
                sa = min(sa, self.sigma_accel_still_inv)
                rb_rw = self.rb_sigma_rw_still
            else:
                sa = min(sa, self.sigma_accel_still)

        # Accel-bias states are unobservable without IMU input — freeze them.
        sb = self.sigma_bias_rw if use_imu else 0.0

        Q = np.zeros((n, n))
        Q[0:2, 0:2] = sa * sa * (dt ** 3 / 3.0) * I2
        Q[0:2, 2:4] = sa * sa * (dt ** 2 / 2.0) * I2
        Q[2:4, 0:2] = sa * sa * (dt ** 2 / 2.0) * I2
        Q[2:4, 2:4] = sa * sa * dt * I2
        Q[4:6, 4:6] = sb * sb * dt * I2
        if self.n_rb:
            Q[6:, 6:] = np.eye(self.n_rb) * rb_rw * rb_rw * dt

        self.P = F @ self.P @ F.T + Q

    # ── generic update ───────────────────────────────────────────────────────

    def update(self, z, H, R, zhat=None) -> bool:
        """Gated EKF update in Joseph form. Gate is chi2(dof, 0.95)."""
        z = np.atleast_1d(np.asarray(z, dtype=float)).ravel()
        H = np.atleast_2d(np.asarray(H, dtype=float))
        R = np.atleast_2d(np.asarray(R, dtype=float))

        pred = (H @ self.x) if zhat is None else np.atleast_1d(
            np.asarray(zhat, dtype=float)).ravel()
        y = z - pred
        S = H @ self.P @ H.T + R
        nis = float(y @ np.linalg.solve(S, y))
        self.last_nis = nis

        gate = CHI2_95[min(z.size, 3) - 1]
        if nis > gate and self.robust and nis <= self.robust_hard_factor * gate:
            # Huber: keep the measurement, inflate its covariance so the
            # effective NIS sits at the gate. Bounded influence beats a hard
            # reject, which throws away a marginal but real measurement.
            R = R * (nis / gate)
            S = H @ self.P @ H.T + R
            self.n_soft += 1
            ok = True
        else:
            ok = nis <= gate

        self.last_accepted = ok
        if not ok:
            self.n_rejected += 1
            return False

        K = self.P @ H.T @ np.linalg.inv(S)
        correction = K @ y
        self.x = self.x + correction

        # Keep the shapes the old pipeline published: a 2-vector innovation and
        # a length-6 gain diagonal, whatever this particular update's dimension.
        inn = np.zeros(2)
        inn[:min(2, y.size)] = y[:2]
        self.last_innovation = inn
        kd = np.zeros(6)
        for i in range(min(6, K.shape[0])):
            kd[i] = K[i, i % max(K.shape[1], 1)]
        self.last_k_diag = kd
        self.last_correction_mag = float(np.linalg.norm(correction[:2]))

        IKH = np.eye(self.x.size) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = (self.P + self.P.T) / 2.0
        return True

    # ── measurement types ────────────────────────────────────────────────────

    def update_position(self, pos, R=None) -> bool:
        if not self.initialized:
            self.initialize(pos)
            return True
        if R is None or not np.all(np.isfinite(np.asarray(R, dtype=float))):
            R = np.eye(2) * self.pos_sigma ** 2
        H = np.zeros((2, 6 + self.n_rb))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        ok = self.update(np.asarray(pos, dtype=float).ravel()[:2], H, R)

        # Divergence recovery: a healthy filter does not reject a long streak of
        # fixes. When it does, re-anchor on the measurements.
        if ok:
            self.consec_reject = 0
        else:
            self.consec_reject += 1
            if self.consec_reject >= self.max_consec_reject:
                self.reinit_from(pos)
        return ok

    def update_ranges(self, anchor_pos, ranges, weights=None,
                      tag_z: float = 0.24, sigma_r: float = 0.05) -> int:
        """Tightly-coupled sequential range updates, each chi2(1)-gated.

        With bias states enabled the model is ``z_k = ||p − a_k|| + b_k``, which
        is what lets a skipped anchor's bias be remembered rather than relearned.
        """
        if not self.initialized:
            return 0
        anchor_pos = np.asarray(anchor_pos, dtype=float)
        ranges = np.asarray(ranges, dtype=float).ravel()
        weights = (np.ones_like(ranges) if weights is None
                   else np.asarray(weights, dtype=float).ravel())

        n = 6 + self.n_rb
        n_acc = 0
        for k in np.flatnonzero(np.isfinite(ranges) & (weights > 0)):
            dx = self.x[0] - anchor_pos[k, 0]
            dy = self.x[1] - anchor_pos[k, 1]
            dz2 = (anchor_pos[k, 2] - tag_z) ** 2
            pred = max(float(np.sqrt(dx * dx + dy * dy + dz2)), 1e-6)
            H = np.zeros((1, n))
            H[0, 0] = dx / pred
            H[0, 1] = dy / pred
            zhat = pred
            if self.n_rb > 0 and k < self.n_rb:
                H[0, 6 + k] = 1.0
                zhat = pred + self.x[6 + k]
            Rk = np.array([[sigma_r ** 2 / weights[k]]])
            n_acc += int(self.update([ranges[k]], H, Rk, [zhat]))

        if n_acc > 0:
            self.consec_reject = 0
        else:
            self.consec_reject += 1
        return n_acc

    def update_zupt(self) -> bool:
        if not self.initialized:
            return False
        H = np.zeros((2, 6 + self.n_rb))
        H[0, 2] = 1.0
        H[1, 3] = 1.0
        return self.update([0.0, 0.0], H, np.eye(2) * self.zupt_sigma ** 2)

    def reinit_from(self, pos) -> None:
        """Re-anchor the pose but KEEP the learned range biases.

        Their memory is the entire point of having them; only their uncertainty
        is reopened, and only moderately.
        """
        rb = self.x[6:].copy()
        self.initialize(pos)
        if self.n_rb:
            self.x[6:] = rb
            self.P[6:, 6:] = np.eye(self.n_rb) * (self.rb_sigma_init / 2.0) ** 2
        self.n_reinit += 1
        self.consec_reject = 0


# ── static helper ─────────────────────────────────────────────────────────────

def world_yaw_rate(quat, gyro) -> float:
    """World-vertical component of the body angular rate (rad/s).

    Port of `worldYawRate` in live_rover.m. Deliberately **delta-only**: the
    absolute rotated heading is never trusted, only the turn rate, because the
    BNO085's yaw drifts while its rate does not.

    Note this is NOT the raw gyro z. On a rover that pitches or rolls, body-z
    and world-z diverge, and feeding raw gyro-z to the heading model as if it
    were a yaw rate is a real error.
    """
    q = np.asarray(quat, dtype=float).ravel()
    g = np.asarray(gyro, dtype=float).ravel()
    if q.size != 4 or g.size != 3:
        return 0.0
    if not (np.all(np.isfinite(q)) and np.all(np.isfinite(g))):
        return 0.0
    nq = float(np.linalg.norm(q))
    if nq < 0.5:
        return 0.0
    a, b, c, d = q / nq
    # Third row of the body->world rotation: the world-z axis in body terms.
    R3 = np.array([2 * (b * d - c * a), 2 * (c * d + b * a),
                   1 - 2 * (b * b + c * c)])
    return float(R3 @ g)


def quat_rpy(quat):
    """BNO085 quaternion [qw qx qy qz] -> (roll, pitch, yaw) in radians."""
    q = np.asarray(quat, dtype=float).ravel()
    if q.size != 4:
        return 0.0, 0.0, float('nan')
    nq = float(np.linalg.norm(q))
    if nq < 0.5:
        return 0.0, 0.0, float('nan')
    w, x, y, z = q / nq
    roll = float(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    sp = float(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    pitch = float(np.arcsin(sp))
    yaw = float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def world_accel(quat, acc):
    """Rotate BNO085 body-frame LINEAR acceleration into the world frame.

    `quat` is ``[qw qx qy qz]`` in HostLink order. Returns ``[ax ay]`` or None
    when the inputs are unusable — an unnormalised quaternion is the common
    case and must not silently produce a plausible-looking wrong answer.
    """
    q = np.asarray(quat, dtype=float).ravel()
    a = np.asarray(acc, dtype=float).ravel()
    if q.size != 4 or a.size != 3:
        return None
    if not (np.all(np.isfinite(q)) and np.all(np.isfinite(a))):
        return None
    nq = float(np.linalg.norm(q))
    if nq < 0.5:
        return None
    w, xq, yq, zq = q / nq
    R = np.array([
        [1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * w),     2 * (xq * zq + yq * w)],
        [2 * (xq * yq + zq * w),      1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * w)],
        [2 * (xq * zq - yq * w),      2 * (yq * zq + xq * w),     1 - 2 * (xq * xq + yq * yq)],
    ])
    return (R @ a)[:2]
