"""Moving Horizon Estimator for 2D UWB positioning (CasADi / IPOPT).

Faithful port of ``matlab/+dune/MheEstimator.m``. Changes must be made in both.

Where the EKF propagates a single state forward, this keeps a sliding window of
the last N sweeps and each step re-solves for the **whole trajectory** over that
window: the one that best fits every anchor range (robust pseudo-Huber), the
motion model, ZUPT while still, and an arrival cost tying the window start to
the previous solution. The output is the newest node, so it behaves as a filter.

Two motion models:

* ``cv`` — state ``[px py vx vy]``. Constant velocity, with the gyro rotating
  the velocity vector (coordinated turn). Holonomic: motion in any direction.
* ``unicycle`` — state ``[px py theta speed]``. A **non-holonomic car**: gyro
  yaw rate drives heading (a change-in-yaw, not an absolute compass), velocity
  is forced along the heading so there is no sideways slip, and speed is free.
  The right model for front-steering; UWB anchors the absolute heading and the
  gyro smooths it.

The NLP is built once as a *parametric* problem and reused for every push — the
build costs seconds, the solve costs milliseconds, and rebuilding per sweep
would make this unusable live.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class MheInfo:
    warmup: bool = True
    cost:   float = float('nan')
    iters:  int = 0
    jumped: bool = False


class MheEstimator:
    """Port of dune.MheEstimator. Parameters are attributes, as in MATLAB."""

    def __init__(self, anchor_pos):
        self.A = np.asarray(anchor_pos, dtype=float)
        self.M = int(self.A.shape[0])

        self.horizon = 10          # sweeps in the window (~2 s at 5 Hz)
        self.model = 'cv'          # 'cv' | 'unicycle'
        self.sigma_r = 0.05        # range measurement sigma (m)
        self.sigma_accel = 0.8     # process accel white noise (m/s²)
        self.sigma_theta = 0.5     # unicycle: heading process noise (rad/s)
        self.tag_z = 0.24
        self.arrival_pos = 0.15    # arrival-cost sigma, position (m)
        self.arrival_vel = 0.6     # arrival-cost sigma, velocity/speed (m/s)
        self.arrival_theta = 0.4   # unicycle: arrival-cost sigma, heading (rad)
        self.huber_delta = 0.15    # robust range-residual threshold (m)
        self.zupt_sigma = 0.03     # zero-velocity pseudo-measurement sigma
        self.max_jump = 0.20       # m, jump guard against the CV prediction
        self.max_iter = 60

        self.solver = None
        self.built = False
        self.buf: list[dict] = []
        self.Xw: Optional[np.ndarray] = None      # last window solution, flat
        self.prior: Optional[np.ndarray] = None   # arrival target, 4x1
        self.last_p = np.array([np.nan, np.nan])
        self.last_v = np.array([0.0, 0.0])
        self.n_solves = 0
        self.n_jumps = 0

    @property
    def _uni(self) -> bool:
        return self.model == 'unicycle'

    # ── build ────────────────────────────────────────────────────────────────

    def build(self) -> None:
        """Construct the parametric NLP once; reused by every push()."""
        import casadi as ca

        N, Ma = self.horizon, self.M
        Ax = ca.DM(self.A[:, 0])
        Ay = ca.DM(self.A[:, 1])
        dz = ca.DM(self.A[:, 2] - self.tag_z)
        uni = self._uni

        X = ca.MX.sym('X', 4, N)
        Z = ca.MX.sym('Z', Ma, N)
        W = ca.MX.sym('W', Ma, N)
        DT = ca.MX.sym('DT', N - 1, 1)
        OMEGA = ca.MX.sym('OMEGA', N - 1, 1)
        STILL = ca.MX.sym('STILL', N, 1)
        PRIOR = ca.MX.sym('PRIOR', 4, 1)

        f = ca.MX(0)
        d = self.huber_delta
        sR2 = self.sigma_r ** 2
        zs2 = self.zupt_sigma ** 2

        for k in range(N):
            pred = ca.sqrt((X[0, k] - Ax) ** 2 + (X[1, k] - Ay) ** 2 + dz ** 2 + 1e-9)
            r = pred - Z[:, k]
            # Pseudo-Huber: quadratic near zero, linear far out, and smooth
            # everywhere — a hard switch would make the NLP non-differentiable
            # exactly where outliers put it.
            ph = d ** 2 * (ca.sqrt(1 + (r / d) ** 2) - 1)
            f = f + ca.sum1(W[:, k] * ph) / sR2
            if uni:
                f = f + STILL[k] * X[3, k] ** 2 / zs2          # speed -> 0
            else:
                f = f + STILL[k] * ca.sum1(X[2:4, k] ** 2) / zs2   # |v| -> 0

        sig_a, s_th = self.sigma_accel, self.sigma_theta
        for k in range(N - 1):
            dt = DT[k]
            qp = 0.5 * sig_a * dt ** 2 + 1e-6
            qv = sig_a * dt + 1e-6
            if uni:
                # Velocities implied by [theta, speed]: no sideways slip.
                vk = ca.vertcat(X[3, k] * ca.cos(X[2, k]),
                                X[3, k] * ca.sin(X[2, k]))
                vk1 = ca.vertcat(X[3, k + 1] * ca.cos(X[2, k + 1]),
                                 X[3, k + 1] * ca.sin(X[2, k + 1]))
                dp = X[0:2, k + 1] - X[0:2, k] - 0.5 * (vk + vk1) * dt
                dth = X[2, k + 1] - X[2, k] - OMEGA[k] * dt    # gyro drives heading
                dsp = X[3, k + 1] - X[3, k]                    # speed random walk
                qth = s_th * dt + 1e-6
                f = f + ca.sum1(dp ** 2) / qp ** 2 + dth ** 2 / qth ** 2 \
                    + dsp ** 2 / qv ** 2
            else:
                dp = X[0:2, k + 1] - X[0:2, k] - 0.5 * (X[2:4, k] + X[2:4, k + 1]) * dt
                th = OMEGA[k] * dt
                cth, sth = ca.cos(th), ca.sin(th)
                vrot = ca.vertcat(cth * X[2, k] - sth * X[3, k],
                                  sth * X[2, k] + cth * X[3, k])
                dv = X[2:4, k + 1] - vrot
                f = f + ca.sum1(dp ** 2) / qp ** 2 + ca.sum1(dv ** 2) / qv ** 2

        # Arrival cost: what the window start remembers of everything before it.
        f = f + ca.sum1((X[0:2, 0] - PRIOR[0:2]) ** 2) / self.arrival_pos ** 2
        if uni:
            f = f + (X[2, 0] - PRIOR[2]) ** 2 / self.arrival_theta ** 2 \
                  + (X[3, 0] - PRIOR[3]) ** 2 / self.arrival_vel ** 2
        else:
            f = f + ca.sum1((X[2:4, 0] - PRIOR[2:4]) ** 2) / self.arrival_vel ** 2

        P = ca.vertcat(ca.reshape(Z, -1, 1), ca.reshape(W, -1, 1),
                       DT, OMEGA, STILL, PRIOR)
        nlp = {'x': ca.reshape(X, -1, 1), 'f': f, 'p': P}
        opts = {
            'print_time': False,
            'ipopt': {'print_level': 0, 'sb': 'yes', 'max_iter': self.max_iter,
                      'tol': 1e-5, 'acceptable_tol': 1e-4,
                      'mu_strategy': 'adaptive'},
        }
        self.solver = ca.nlpsol('mhe', 'ipopt', nlp, opts)
        self.built = True

    def reset(self) -> None:
        self.buf = []
        self.Xw = None
        self.prior = None
        self.last_p = np.array([np.nan, np.nan])
        self.last_v = np.array([0.0, 0.0])

    # ── step ─────────────────────────────────────────────────────────────────

    def push(self, z, w, dt: float, p_hint=None, still: bool = False,
             omega: float = 0.0):
        """One sweep in; returns ``(p, v, info)``.

        `p` is NaN during warm-up, i.e. until the window is full.
        `z`/`w` are per-anchor corrected ranges and weights (NaN or w<=0 means
        the anchor is absent). `p_hint` is the raw fix, used to seed the
        optimiser and the heading. `omega` is gyro yaw rate in rad/s.
        """
        z = np.asarray(z, dtype=float).ravel().copy()
        w = np.asarray(w, dtype=float).ravel().copy()
        p_hint = (np.array([np.nan, np.nan]) if p_hint is None
                  else np.asarray(p_hint, dtype=float).ravel()[:2])
        if not np.isfinite(omega):
            omega = 0.0

        bad = ~np.isfinite(z) | ~np.isfinite(w) | (w <= 0)
        z[bad] = 0.0
        w[bad] = 0.0

        self.buf.append({'z': z, 'w': w, 'dt': max(float(dt), 1e-3),
                         'still': float(bool(still)), 'omega': float(omega),
                         'p_hint': p_hint})

        N, uni = self.horizon, self._uni
        info = MheInfo()
        nan2 = np.array([np.nan, np.nan])
        if len(self.buf) < N:
            return nan2, nan2, info
        if len(self.buf) > N:
            self.buf.pop(0)
        if not self.built:
            self.build()

        Zm = np.zeros((self.M, N))
        Wm = np.zeros((self.M, N))
        DTv = np.zeros(N - 1)
        OMv = np.zeros(N - 1)
        STv = np.zeros(N)
        Ph = np.full((N, 2), np.nan)
        tw = np.zeros(N)
        for k in range(N):
            b = self.buf[k]
            Zm[:, k] = b['z']
            Wm[:, k] = b['w']
            STv[k] = b['still']
            Ph[k, :] = b['p_hint']
            if k >= 1:
                DTv[k - 1] = b['dt']
                # Trapezoidal: the yaw rate over the interval, not at an end.
                OMv[k - 1] = 0.5 * (self.buf[k - 1]['omega'] + b['omega'])
                tw[k] = tw[k - 1] + DTv[k - 1]

        if self.Xw is None:
            X0, pr = self._seed_first(Ph, tw, uni)
        else:
            X0, pr = self._warm_start(DTv, OMv, uni)

        Pv = np.concatenate([Zm.ravel(order='F'), Wm.ravel(order='F'),
                             DTv, OMv, STv, pr])

        r = self.solver(x0=X0, p=Pv)
        X = np.asarray(r['x']).ravel()
        self.n_solves += 1
        Xm = X.reshape((4, N), order='F')
        p = Xm[0:2, -1].copy()
        v = self._state_vel(Xm[:, -1], uni)

        st = self.solver.stats()
        solve_ok = bool(st.get('success', True))

        info.warmup = False
        info.cost = float(r['f'])
        info.iters = int(st.get('iter_count', 0) or 0)

        # Jump guard: an output that leaps away from the constant-velocity
        # prediction — or a non-converged solve — is rejected in favour of the
        # raw fix, and the window is rebuilt so one bad solve cannot poison the
        # next warm start.
        predicted = self.last_p + self.last_v * dt
        if np.all(np.isfinite(self.last_p)) and (
                not solve_ok or np.linalg.norm(p - predicted) > self.max_jump):
            p = p_hint.copy() if np.all(np.isfinite(p_hint)) else predicted
            v = self.last_v.copy()
            self._reseed_window(p, v, DTv, uni)
            self.n_jumps += 1
            info.jumped = True
        else:
            self.Xw = X
            self.prior = Xm[:, 1].copy()

        self.last_p = p
        self.last_v = v
        return p, v, info

    # ── seeding ──────────────────────────────────────────────────────────────

    @staticmethod
    def _state_vel(xcol, uni: bool) -> np.ndarray:
        if uni:
            return xcol[3] * np.array([np.cos(xcol[2]), np.sin(xcol[2])])
        return xcol[2:4].copy()

    def _seed_first(self, Ph, tw, uni):
        """Seed the first full window from the buffered raw fixes."""
        fin = np.flatnonzero(np.all(np.isfinite(Ph), axis=1))
        if fin.size >= 2:
            span = max(tw[fin[-1]] - tw[fin[0]], 1e-3)
            vseed = (Ph[fin[-1]] - Ph[fin[0]]) / span
            seed_pos = Ph[fin[0]]
        else:
            vseed = np.zeros(2)
            seed_pos = self.A[:, :2].mean(axis=0)
            if fin.size:
                seed_pos = Ph[fin[0]]

        N = self.horizon
        Xm = np.zeros((4, N))
        for k in range(N):
            pk = Ph[k]
            if not np.all(np.isfinite(pk)):
                pk = seed_pos
            Xm[0:2, k] = pk
            if uni:
                Xm[2, k] = np.arctan2(vseed[1], vseed[0])
                Xm[3, k] = np.linalg.norm(vseed)
            else:
                Xm[2:4, k] = vseed
        # First window: the arrival cost targets the seed, so it is ~free.
        return Xm.ravel(order='F'), Xm[:, 0].copy()

    def _warm_start(self, DTv, OMv, uni):
        """Shift the previous solution forward and extrapolate the new node."""
        N = self.horizon
        Xp = self.Xw.reshape((4, N), order='F')
        last = Xp[:, -1].copy()
        if uni:
            th = last[2] + OMv[-1] * DTv[-1]
            sp = last[3]
            last = np.array([last[0] + sp * np.cos(th) * DTv[-1],
                             last[1] + sp * np.sin(th) * DTv[-1], th, sp])
        else:
            last = last + np.array([last[2] * DTv[-1], last[3] * DTv[-1], 0.0, 0.0])
        X0 = np.concatenate([Xp[:, 1:].ravel(order='F'), last])
        return X0, self.prior.copy()

    def _reseed_window(self, p, v, DTv, uni) -> None:
        """Rebuild the window as a clean constant-motion track ending at (p,v)."""
        N = self.horizon
        dt = float(np.mean(DTv)) if DTv.size else 0.2
        Xm = np.zeros((4, N))
        th = np.arctan2(v[1], v[0])
        sp = float(np.linalg.norm(v))
        for k in range(N):
            Xm[0:2, k] = np.asarray(p) - np.asarray(v) * (N - 1 - k) * dt
            if uni:
                Xm[2, k] = th
                Xm[3, k] = sp
            else:
                Xm[2:4, k] = v
        self.Xw = Xm.ravel(order='F')
        self.prior = Xm[:, 1].copy()
