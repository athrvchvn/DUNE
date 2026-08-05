"""
Tier-2 range correction: per-anchor DW1000 power bias, linear in first-path power.

Port of ``matlab/+dune/loadRangeCorrection.m`` (loader) plus the fitter that
DUNE never committed.

**Why this and not a spatial error map.** The DW1000's leading-edge detector
reads long when the signal is strong, so range error correlates with received
first-path power: ``corr_mm = slope * fp_dBm + intercept``. Fitted per anchor on
a 19-spot camera-truth campaign this took floor-wide static RMSE from 199 mm to
94 mm (leave-one-spot-out validated).

The alternative -- a spatial error map keyed on position -- was tried in this
repo and retired: the campaign showed the residual is *time-varying* (slow
multipath "breathing" of 5-7 cm at a fixed position), and an oracle test bounded
even a perfect per-spot constant correction at ~72 mm RMSE. A static function of
position cannot absorb a signal that moves while the tag stands still, whereas a
function of measured power tracks it. See ``docs/phase_a_findings.md``.

Correction is applied as ``corrected = raw - corr_mm/1000``, with ``fp`` clamped
to the fitted validity window so an out-of-range power cannot extrapolate the
line into nonsense.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from .paths import config

import numpy as np

SCHEMA = "dune_range_correction_v1"


@dataclass
class RangeCorrection:
    """Per-anchor linear-in-first-path-power range correction."""
    ids:    np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    slope:  np.ndarray = field(default_factory=lambda: np.array([]))   # mm per dB
    icept:  np.ndarray = field(default_factory=lambda: np.array([]))   # mm
    fp_min: np.ndarray = field(default_factory=lambda: np.array([]))   # dBm
    fp_max: np.ndarray = field(default_factory=lambda: np.array([]))   # dBm
    file:   str = ""

    @classmethod
    def load(cls, path: Optional[str | Path] = None) -> Optional["RangeCorrection"]:
        """
        Load ``range_correction.json``; returns ``None`` when absent so callers
        can invoke this unconditionally (matching the MATLAB ``[]`` contract).
        """
        if path is None:
            path = config("range_correction.json")
        path = Path(path)
        if not path.exists():
            return None
        raw = json.loads(path.read_text())
        a = raw["anchors"]
        return cls(
            ids=np.array([int(x["id"]) for x in a], dtype=int),
            slope=np.array([float(x["slope_mm_per_dB"]) for x in a]),
            icept=np.array([float(x["intercept_mm"]) for x in a]),
            fp_min=np.array([float(x["fp_valid_dBm"][0]) for x in a]),
            fp_max=np.array([float(x["fp_valid_dBm"][1]) for x in a]),
            file=str(path),
        )

    def correction_mm(self, anchor_id: int, fp_dbm: float) -> float:
        """Correction in mm to SUBTRACT from the raw range. 0.0 when unknown."""
        j = np.flatnonzero(self.ids == anchor_id)
        if j.size == 0 or not np.isfinite(fp_dbm):
            return 0.0
        j = int(j[0])
        f = min(max(fp_dbm, self.fp_min[j]), self.fp_max[j])
        return float(self.slope[j] * f + self.icept[j])


# ── Fitter ────────────────────────────────────────────────────────────────────
# DUNE shipped range_correction.json but no code that produces it, so the model
# could not be refitted for different hardware or a moved constellation. This is
# that missing step.

def fit_range_correction(
    samples: Sequence[Tuple[int, float, float, float]],
    min_samples: int = 30,
    min_fp_span_db: float = 3.0,
) -> Dict:
    """
    Fit the per-anchor model from truth-referenced samples.

    samples: iterable of ``(anchor_id, fp_dbm, measured_m, true_m)``.

    Only anchors with enough samples *and* enough spread in first-path power get
    a slope -- fitting a line through a narrow power band produces a wild slope
    that then extrapolates badly. Those fall back to a constant offset (slope 0,
    intercept = median error), which is still better than no correction.

    Returns the ``range_correction.json`` document.
    """
    by_anchor: Dict[int, List[Tuple[float, float]]] = {}
    for aid, fp, meas, true in samples:
        if not (np.isfinite(fp) and np.isfinite(meas) and np.isfinite(true)):
            continue
        by_anchor.setdefault(int(aid), []).append((float(fp), (meas - true) * 1000.0))

    anchors = []
    for aid in sorted(by_anchor):
        arr = np.array(by_anchor[aid])
        fp, err_mm = arr[:, 0], arr[:, 1]
        n = fp.size
        if n < min_samples:
            continue
        span = float(fp.max() - fp.min())
        if span >= min_fp_span_db:
            slope, icept = np.polyfit(fp, err_mm, 1)
        else:
            slope, icept = 0.0, float(np.median(err_mm))
        anchors.append({
            "id": aid,
            "slope_mm_per_dB": round(float(slope), 3),
            "intercept_mm": round(float(icept), 2),
            "fp_valid_dBm": [round(float(fp.min()), 1), round(float(fp.max()), 1)],
            "n_samples": int(n),
            "fp_span_dB": round(span, 1),
            "resid_rms_mm": round(float(np.sqrt(np.mean(
                (err_mm - (slope * fp + icept)) ** 2))), 1),
        })

    return {
        "schema": SCHEMA,
        "date": datetime.now().strftime("%Y-%m-%d"),
        "method": (
            "per-anchor linear in first-path power (dBm): corr_mm = slope*fp + intercept; "
            "corrected_range = raw - corr_mm/1000. fp clamped to fp_valid_dBm before "
            "evaluating (extrapolation guard)."
        ),
        "fitted_by": "python/dune/range_correction.py:fit_range_correction",
        "anchors": anchors,
    }


def loso_validate(
    samples: Sequence[Tuple[int, float, float, float, int]],
) -> Dict[str, float]:
    """
    Leave-one-spot-out check: refit with each spot held out and score the model
    on that spot. In-sample residuals always look good, so this is what tells you
    whether the correction generalises to a position it was not fitted on.

    samples: ``(anchor_id, fp_dbm, measured_m, true_m, spot_id)``.
    """
    spots = sorted({int(s[4]) for s in samples})
    before, after = [], []
    for held in spots:
        train = [s[:4] for s in samples if int(s[4]) != held]
        test = [s for s in samples if int(s[4]) == held]
        if not train or not test:
            continue
        doc = fit_range_correction(train)
        rc = RangeCorrection(
            ids=np.array([a["id"] for a in doc["anchors"]], dtype=int),
            slope=np.array([a["slope_mm_per_dB"] for a in doc["anchors"]]),
            icept=np.array([a["intercept_mm"] for a in doc["anchors"]]),
            fp_min=np.array([a["fp_valid_dBm"][0] for a in doc["anchors"]]),
            fp_max=np.array([a["fp_valid_dBm"][1] for a in doc["anchors"]]),
        )
        for aid, fp, meas, true, _ in test:
            before.append((meas - true) * 1000.0)
            after.append((meas - rc.correction_mm(int(aid), fp) / 1000.0 - true) * 1000.0)

    def rms(v):
        return float(np.sqrt(np.mean(np.square(v)))) if len(v) else float("nan")

    return {
        "n_spots": len(spots),
        "n_samples": len(before),
        "range_rms_mm_before": round(rms(before), 1),
        "range_rms_mm_after": round(rms(after), 1),
    }


def write_range_correction(doc: Dict, path: str | Path) -> None:
    path = Path(path)
    path.write_text(json.dumps(doc, indent=2) + "\n")
