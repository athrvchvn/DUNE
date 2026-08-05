"""
Centralized configuration for the IMU+UWB fusion pipeline.

All noise parameters and pipeline constants live here. Pass a copy of
FUSION_DEFAULTS (possibly overriding individual keys) to FusionEKF,
ZuptDetector, and RtlsPipeline instead of scattering magic numbers.
"""

FUSION_DEFAULTS: dict = {
    # ── IMU-driven prediction ─────────────────────────────────────────────────
    # Accelerometer white-noise standard deviation [m/s²].
    # Drives the Q matrix via the Singer model; increase if position drifts
    # between UWB corrections when stationary.
    'sigma_accel':          0.15,

    # Accelerometer bias random-walk rate [m/s³].
    # Small constant; keeps the bias state from becoming over-confident.
    'sigma_bias_drift':     1e-3,

    # Q inflation factor applied when BNO085 orientation confidence (status) == 0.
    # At confidence 0 the prediction reverts to a constant-velocity model.
    'imu_conf_inflate':     4.0,

    # ── UWB measurement covariance (adaptive R) ───────────────────────────────
    # Fallback position sigma used when multilaterator covariance is unavailable [m].
    'sigma_pos_nominal':    0.10,

    # Expected multilateration RMSE at good geometry [m].
    # R is scaled by (1 + (rms/rms_nominal)²).
    'rms_nominal':          0.05,

    # Expected DOP-proxy (sqrt(trace(cov))) at good geometry [m].
    # R is further scaled when DOP exceeds this value.
    'dop_nominal':          0.10,

    # ── Innovation gate (NIS chi-squared test) ────────────────────────────────
    # Threshold for the Normalised Innovation Squared (NIS).
    # chi²(2 DOF, p=0.95) = 5.991 — measurements with NIS above this are rejected.
    'nis_gate':             5.991,

    # EKF is re-initialised to the raw multilat position after this many
    # consecutive rejected measurements (possible filter divergence).
    'max_consecutive_rej':  5,

    # ── ZUPT (Zero-Velocity Update) ───────────────────────────────────────────
    # Sliding window length for stationary detection [packets at ~10 Hz].
    'zupt_window':          8,

    # Mean acceleration magnitude threshold for stationary detection [m/s²].
    'zupt_accel_mean':      0.10,

    # Variance threshold for stationary detection [(m/s²)²].
    'zupt_accel_var':       0.005,

    # Angular-rate threshold for stationary detection [rad/s] (≈5°/s).
    'zupt_gyro':            0.087,

    # EKF speed below which the velocity gate passes [m/s].
    'zupt_speed':           0.05,

    # Consecutive stationary packets required before ZUPT fires (debounce).
    'zupt_debounce':        3,

    # Noise applied to the zero-velocity pseudo-measurement [m/s].
    'sigma_zupt':           0.01,

    # ── Pipeline constants (mirrors prior hard-coded values) ──────────────────
    # Median filter window per anchor [packets].
    'range_filt_n':         8,

    # Systematic range bias applied uniformly to all anchors [m].
    'range_bias_m':         0.010,

    # Age gate: drop an anchor's measurement if unseen for this long [s].
    'age_gate_sec':         3.0,

    # EMA smoothing factor for final reported position.
    'ema_alpha':            0.15,

    # Last-resort divergence guard: EKF is reset if estimated position
    # jumps more than this distance from the multilat solution [m].
    'ekf_jump_thresh':      2.0,

    # Last-resort divergence guard: EKF is reset if trace(P) exceeds this.
    'ekf_trace_thresh':     25.0,

    # Watchdog: log a warning if no packet arrives within this period [s].
    'watchdog_sec':         10.0,

    # ── NLOS handling (Phase 2) ───────────────────────────────────────────────
    # Per-anchor NLOS score = rx_dbm − fp_dbm (same-frame, from firmware). A gap
    # below this is treated as line-of-sight and incurs no penalty [dB].
    'nlos_thresh_db':       3.0,

    # Multilateration down-weighting: score = max(0, gap − nlos_thresh_db);
    # weight wᵢ = 10^(−score/10), floored at nlos_weight_min so a suspect anchor is
    # de-emphasised but never silently dropped by weighting alone (the MAD gate is
    # the independent hard-reject path).
    'nlos_weight_min':      0.05,

    # NLOS range-bias correction (through-wall over-range), metres per dB of score:
    #   d_corr = d − nlos_k_bias · max(0, gap − nlos_thresh_db)
    # Opt-in (0 = off until tuned). Plan range 0–0.06 m/dB.
    'nlos_k_bias':          0.0,

    # ── Logging ───────────────────────────────────────────────────────────────
    # When True, log the full 6×2 Kalman gain matrix instead of just the diagonal.
    'log_full_kalman_gain': False,

    # ── Estimator mode (EKF vs MHE, live-switchable via GUI) ──────────────────
    # Which filter's output feeds ex/ey and downstream EMA. Both estimators are
    # always kept "hot" (fed every packet) regardless of this setting, so
    # switching is instantaneous and track-continuous — see RtlsPipeline.
    'estimator_mode':       'EKF',   # 'EKF' | 'MHE'

    # ── MHE (Moving Horizon Estimator) — minimal v1 ───────────────────────────
    # Sliding window length [sweeps]. Tightly-coupled batch solve over raw
    # ranges from the last N sweeps; no hard room-bound/speed constraints in v1.
    'mhe_window':           6,

    # Huber loss f_scale on range residuals [m] — residuals within this margin
    # are treated as ~quadratic (normal LS), beyond it the loss grows linearly
    # (NLOS/outlier robustness) rather than quadratically.
    'mhe_huber_delta':      0.15,

    # Weight on the position dynamics-smoothness residual between consecutive
    # window steps (constant-velocity penalty).
    'mhe_dyn_pos_weight':   5.0,

    # Weight on the velocity dynamics-smoothness residual between consecutive
    # window steps.
    'mhe_dyn_vel_weight':   2.0,

    # Nominal range sigma [m] used to convert the batch solve's Gauss-Newton
    # Hessian into a position covariance estimate (pos_cov) — mirrors
    # Multilaterator's range_sigma convention (cov = range_sigma² · inv(JᵀWJ)).
    'mhe_range_sigma':      0.10,

    # Last-resort divergence guard: MHE is reset (cold-started) if its solved
    # position jumps more than this distance from the raw multilat solution [m].
    'mhe_jump_thresh':      2.0,

    # Cap on least_squares iterations per solve — latency safety valve at ~10 Hz.
    'mhe_max_nfev':         50,
}

# ── Tag color palette ─────────────────────────────────────────────────────────
# Colors assigned in order to each newly discovered tag_id (CSS hex strings).
# The GUI cycles through this list. Add more colours to support more tags.
TAG_PALETTE: list[str] = [
    '#00d4ff',   # cyan   — first tag
    '#e63946',   # red    — second tag
    '#a3e635',   # lime   — third tag
    '#a78bfa',   # violet — fourth tag
    '#ff9d00',   # orange — fifth tag
    '#ff3d5a',   # rose   — sixth tag
]
