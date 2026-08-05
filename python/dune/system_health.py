"""
SystemHealth dataclass — one instance per pipeline fix, included in the
result dict and JSONL log. The GUI reads it to populate the health strip.
"""

from dataclasses import dataclass, asdict


@dataclass
class SystemHealth:
    # ── Sensor availability ───────────────────────────────────────────────────
    imu_available:           bool  = False
    gyro_available:          bool  = False
    orientation_valid:       bool  = False
    orientation_confidence:  int   = 0       # 0–3, BNO085 status field
    uwb_available:           bool  = False

    # ── EKF state ─────────────────────────────────────────────────────────────
    ekf_initialized:         bool  = False
    prediction_mode:         str   = 'CV'    # 'IMU' or 'CV'
    imu_prediction_age_ms:   float = 0.0     # ms since last successful IMU prediction

    # ── Correction state (per-packet) ─────────────────────────────────────────
    correction_accepted:     bool  = False
    correction_rejected:     bool  = False
    n_uwb_accepted:          int   = 0       # cumulative accepted corrections
    n_uwb_rejected:          int   = 0       # cumulative rejected corrections

    # ── Measurement quality ───────────────────────────────────────────────────
    nis_last:                float = 0.0     # NIS from most recent UWB update
    zupt_active:             bool  = False
    n_anchors_used:          int   = 0       # anchors used by multilaterator
    n_anchors_rejected:      int   = 0       # anchors rejected (NLOS / MAD gate)

    # ── Composite confidence [0, 1] ───────────────────────────────────────────
    # Rough product of geometry quality × acceptance rate × orientation confidence.
    # Intended for GUI colour-coding, not for filter decisions.
    localization_confidence: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)

    def compute_confidence(self) -> None:
        total = self.n_uwb_accepted + self.n_uwb_rejected
        acceptance = self.n_uwb_accepted / total if total > 0 else 1.0
        orient_c = self.orientation_confidence / 3.0 if self.imu_available else 0.5
        geom_ok = 1.0 if self.n_anchors_used >= 3 else (self.n_anchors_used / 3.0)
        self.localization_confidence = round(acceptance * orient_c * geom_ok, 3)
