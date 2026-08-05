"""Rigid dual-tag Moving Horizon Estimator (CasADi / IPOPT).

Faithful port of ``matlab/+dune/MheRigid.m``. Changes must be made in both.

One rigid body carries **two** tags a fixed distance L apart, with the
non-holonomic unicycle model. Window node state is the *body pose*:

    x = [cx, cy, psi, speed]

The two antennas are never solved separately — both are derived from that one
pose, so the rig's physical facts hold **structurally** rather than as penalties
the optimiser can trade away:

    front tag = c + (Lh/2)·[cos ψ, sin ψ]
    rear  tag = c − (Lh/2)·[cos ψ, sin ψ]

which means the inter-tag distance is exact by construction; "both tags still,
or both moving" is automatic (there is one speed state, and a rigid link cannot
let them disagree); and yaw is observed by the UWB geometry *and* driven by the
gyro, which beats either source alone.

**Asynchronous by design.** The tags sweep at different instants, so each
incoming sweep — from either tag — becomes its own window node observing the
body at its own timestamp, tagged ``sgn = +1`` (front) or ``−1`` (rear). There
is no nearest-time pairing of the two streams, and therefore no pairing error.

**Terrain** comes from the IMU rather than being estimated: the horizontal half
baseline shrinks as ``(L/2)·cos(pitch)``, and each tag's antenna height moves by
``±(L/2)·sin(pitch)`` — the front rises as the rear drops. Roll is deliberately
excluded: the baseline lies along the body x-axis, which roll rotates *about*,
so it cannot move the tags. It would matter only for a laterally-offset tag.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class RigidInfo:
    warmup: bool = True
    cost:   float = float('nan')
    iters:  int = 0
    jumped: bool = False


class MheRigid:
    """Port of dune.MheRigid. Parameters are attributes, as in MATLAB."""

    def __init__(self, anchor_pos):
        self.A = np.asarray(anchor_pos, dtype=float)
        self.M = int(self.A.shape[0])

        self.horizon = 12          # nodes (~2 s: two tags at ~3 Hz each)
        self.baseline = 0.50       # L, antenna centre to antenna centre — MEASURE IT
        self.sigma_r = 0.05
        self.sigma_accel = 0.8
        self.sigma_theta = 0.5
        self.tag_z = 0.24
        self.arrival_pos = 0.15
        self.arrival_vel = 0.6
        self.arrival_theta = 0.4
        self.huber_delta = 0.15
        self.zupt_sigma = 0.03
        self.max_jump = 0.20
        self.max_iter = 80

        self.solver = None
        self.built = False
        self.buf: list[dict] = []
        self.Xw: Optional[np.ndarray] = None
        self.prior: Optional[np.ndarray] = None
        self.last_c = np.array([np.nan, np.nan])
        self.last_v = np.array([0.0, 0.0])
        self.last_psi = 0.0
        self.n_solves = 0
        self.n_jumps = 0

    # ── build ────────────────────────────────────────────────────────────────

    def build(self) -> None:
        import casadi as ca

        N, Ma = self.horizon, self.M
        Ax = ca.DM(self.A[:, 0])
        Ay = ca.DM(self.A[:, 1])
        Az = ca.DM(self.A[:, 2])

        X = ca.MX.sym('X', 4, N)          # [cx; cy; psi; speed]
        Z = ca.MX.sym('Z', Ma, N)         # ranges of the OBSERVING tag
        W = ca.MX.sym('W', Ma, N)
        SGN = ca.MX.sym('SGN', N, 1)      # +1 front, -1 rear
        LH = ca.MX.sym('LH', N, 1)        # horizontal HALF baseline
        TZ = ca.MX.sym('TZ', N, 1)        # observing tag's antenna height
        DT = ca.MX.sym('DT', N - 1, 1)
        OMEGA = ca.MX.sym('OMEGA', N - 1, 1)
        STILL = ca.MX.sym('STILL', N, 1)
        PRIOR = ca.MX.sym('PRIOR', 4, 1)

        f = ca.MX(0)
        d = self.huber_delta
        sR2 = self.sigma_r ** 2
        zs2 = self.zupt_sigma ** 2

        for k in range(N):
            psi = X[2, k]
            off = SGN[k] * LH[k]                       # signed lever arm
            tx = X[0, k] + off * ca.cos(psi)
            ty = X[1, k] + off * ca.sin(psi)
            dz = TZ[k] - Az
            pred = ca.sqrt((tx - Ax) ** 2 + (ty - Ay) ** 2 + dz ** 2 + 1e-9)
            r = pred - Z[:, k]
            ph = d ** 2 * (ca.sqrt(1 + (r / d) ** 2) - 1)
            f = f + ca.sum1(W[:, k] * ph) / sR2
            f = f + STILL[k] * X[3, k] ** 2 / zs2      # ZUPT on the body speed

        sig_a, s_th = self.sigma_accel, self.sigma_theta
        for k in range(N - 1):
            dt = DT[k]
            qp = 0.5 * sig_a * dt ** 2 + 1e-6
            qv = sig_a * dt + 1e-6
            qth = s_th * dt + 1e-6
            vk = ca.vertcat(X[3, k] * ca.cos(X[2, k]),
                            X[3, k] * ca.sin(X[2, k]))
            vk1 = ca.vertcat(X[3, k + 1] * ca.cos(X[2, k + 1]),
                             X[3, k + 1] * ca.sin(X[2, k + 1]))
            dp = X[0:2, k + 1] - X[0:2, k] - 0.5 * (vk + vk1) * dt   # trapezoidal
            dth = X[2, k + 1] - X[2, k] - OMEGA[k] * dt              # gyro drives psi
            dsp = X[3, k + 1] - X[3, k]
            f = f + ca.sum1(dp ** 2) / qp ** 2 + dth ** 2 / qth ** 2 \
                + dsp ** 2 / qv ** 2

        f = f + ca.sum1((X[0:2, 0] - PRIOR[0:2]) ** 2) / self.arrival_pos ** 2 \
              + (X[2, 0] - PRIOR[2]) ** 2 / self.arrival_theta ** 2 \
              + (X[3, 0] - PRIOR[3]) ** 2 / self.arrival_vel ** 2

        P = ca.vertcat(ca.reshape(Z, -1, 1), ca.reshape(W, -1, 1),
                       SGN, LH, TZ, DT, OMEGA, STILL, PRIOR)
        nlp = {'x': ca.reshape(X, -1, 1), 'f': f, 'p': P}
        opts = {
            'print_time': False,
            'ipopt': {'print_level': 0, 'sb': 'yes', 'max_iter': self.max_iter,
                      'tol': 1e-5, 'acceptable_tol': 1e-4,
                      'mu_strategy': 'adaptive'},
        }
        self.solver = ca.nlpsol('mheRigid', 'ipopt', nlp, opts)
        self.built = True

    def reset(self) -> None:
        self.buf = []
        self.Xw = None
        self.prior = None
        self.last_c = np.array([np.nan, np.nan])
        self.last_v = np.array([0.0, 0.0])
        self.last_psi = 0.0

    # ── step ─────────────────────────────────────────────────────────────────

    def push(self, z, w, is_front: bool, dt: float, p_hint=None,
             still: bool = False, omega: float = 0.0, pitch: float = 0.0):
        """One sweep from EITHER tag; returns ``(pose, info)``.

        `pose` is ``[cx, cy, psi, speed]``, NaN until the window fills. `omega`
        is the gyro yaw rate — hold the last value on the tag that has no IMU.
        """
        z = np.asarray(z, dtype=float).ravel().copy()
        w = np.asarray(w, dtype=float).ravel().copy()
        p_hint = (np.array([np.nan, np.nan]) if p_hint is None
                  else np.asarray(p_hint, dtype=float).ravel()[:2])
        if not np.isfinite(omega):
            omega = 0.0
        if not np.isfinite(pitch):
            pitch = 0.0

        bad = ~np.isfinite(z) | ~np.isfinite(w) | (w <= 0)
        z[bad] = 0.0
        w[bad] = 0.0

        sgn = 1.0 if is_front else -1.0
        L2 = self.baseline / 2.0
        self.buf.append({
            'z': z, 'w': w, 'sgn': sgn,
            'lh': L2 * np.cos(pitch),                   # horizontal lever arm
            'tz': self.tag_z + sgn * L2 * np.sin(pitch),  # this tag's height
            'dt': max(float(dt), 1e-3), 'still': float(bool(still)),
            'omega': float(omega), 'p_hint': p_hint,
        })

        N = self.horizon
        info = RigidInfo()
        if len(self.buf) < N:
            return np.full(4, np.nan), info
        if len(self.buf) > N:
            self.buf.pop(0)
        if not self.built:
            self.build()

        Zm = np.zeros((self.M, N))
        Wm = np.zeros((self.M, N))
        SG = np.zeros(N)
        LHv = np.zeros(N)
        TZv = np.zeros(N)
        STv = np.zeros(N)
        DTv = np.zeros(N - 1)
        OMv = np.zeros(N - 1)
        for k in range(N):
            b = self.buf[k]
            Zm[:, k] = b['z']
            Wm[:, k] = b['w']
            SG[k], LHv[k], TZv[k], STv[k] = b['sgn'], b['lh'], b['tz'], b['still']
            if k >= 1:
                DTv[k - 1] = b['dt']
                OMv[k - 1] = 0.5 * (self.buf[k - 1]['omega'] + b['omega'])

        if self.Xw is None:
            X0, pr = self._seed_first()
        else:
            Xp = self.Xw.reshape((4, N), order='F')
            last = Xp[:, -1].copy()
            th = last[2] + OMv[-1] * DTv[-1]
            sp = last[3]
            last = np.array([last[0] + sp * np.cos(th) * DTv[-1],
                             last[1] + sp * np.sin(th) * DTv[-1], th, sp])
            X0 = np.concatenate([Xp[:, 1:].ravel(order='F'), last])
            pr = self.prior.copy()

        Pv = np.concatenate([Zm.ravel(order='F'), Wm.ravel(order='F'),
                             SG, LHv, TZv, DTv, OMv, STv, pr])

        r = self.solver(x0=X0, p=Pv)
        X = np.asarray(r['x']).ravel()
        self.n_solves += 1
        Xm = X.reshape((4, N), order='F')
        c = Xm[0:2, -1].copy()
        psi = float(Xm[2, -1])
        spd = float(Xm[3, -1])
        v = spd * np.array([np.cos(psi), np.sin(psi)])

        st = self.solver.stats()
        solve_ok = bool(st.get('success', True))
        info.warmup = False
        info.cost = float(r['f'])
        info.iters = int(st.get('iter_count', 0) or 0)

        predicted = self.last_c + self.last_v * dt
        if np.all(np.isfinite(self.last_c)) and (
                not solve_ok or np.linalg.norm(c - predicted) > self.max_jump):
            c_fall = predicted
            if np.all(np.isfinite(p_hint)):
                # The observing tag's own fix, de-levered back to the centre.
                lh = self.buf[-1]['lh']
                c_fall = p_hint - sgn * lh * np.array([np.cos(self.last_psi),
                                                       np.sin(self.last_psi)])
            c = c_fall
            psi = self.last_psi
            spd = float(np.linalg.norm(self.last_v))
            v = self.last_v.copy()
            self._reseed_window(c, psi, spd, DTv)
            self.n_jumps += 1
            info.jumped = True
        else:
            self.Xw = X
            self.prior = Xm[:, 1].copy()

        self.last_c, self.last_v, self.last_psi = c, v, psi
        return np.array([c[0], c[1], psi, spd]), info

    # ── derived geometry ─────────────────────────────────────────────────────

    def tag_positions(self, pose, pitch: float = 0.0):
        """Front/rear tag xy implied by a pose — the inverse of the model."""
        lh = (self.baseline / 2.0) * np.cos(pitch)
        u = lh * np.array([np.cos(pose[2]), np.sin(pose[2])])
        c = np.asarray(pose[:2], dtype=float)
        return c + u, c - u

    # ── seeding ──────────────────────────────────────────────────────────────

    def _seed_first(self):
        """Centre and heading from the newest front/rear raw fixes in the window."""
        N = self.horizon
        pF = np.array([np.nan, np.nan])
        pR = np.array([np.nan, np.nan])
        for k in range(N - 1, -1, -1):
            b = self.buf[k]
            if not np.all(np.isfinite(b['p_hint'])):
                continue
            if b['sgn'] > 0 and not np.all(np.isfinite(pF)):
                pF = b['p_hint']
            if b['sgn'] < 0 and not np.all(np.isfinite(pR)):
                pR = b['p_hint']

        if np.all(np.isfinite(pF)) and np.all(np.isfinite(pR)):
            c0 = 0.5 * (pF + pR)
            psi0 = float(np.arctan2(pF[1] - pR[1], pF[0] - pR[0]))
        elif np.all(np.isfinite(pF)):
            c0, psi0 = pF, 0.0
        elif np.all(np.isfinite(pR)):
            c0, psi0 = pR, 0.0
        else:
            c0, psi0 = self.A[:, :2].mean(axis=0), 0.0

        Xm = np.tile(np.array([c0[0], c0[1], psi0, 0.0]).reshape(4, 1), (1, N))
        return Xm.ravel(order='F'), Xm[:, 0].copy()

    def _reseed_window(self, c, psi, spd, DTv) -> None:
        N = self.horizon
        dt = float(np.mean(DTv)) if DTv.size else 0.2
        v = spd * np.array([np.cos(psi), np.sin(psi)])
        Xm = np.zeros((4, N))
        for k in range(N):
            Xm[0:2, k] = np.asarray(c) - v * (N - 1 - k) * dt
            Xm[2, k] = psi
            Xm[3, k] = spd
        self.Xw = Xm.ravel(order='F')
        self.prior = Xm[:, 1].copy()
