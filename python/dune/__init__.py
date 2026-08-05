"""RTLS pipeline modules — the Python side of the DUNE parity port.

Each module here mirrors one file in ``matlab/+dune/`` and changes must be made
in both. The live app and offline replay share every one of them, which is the
property that makes a replay a trustworthy reproduction rather than an
approximation of what ran live.

Superseded and removed (recoverable from git):
  ``fusion_ekf.py``     → ``ekf.py`` — adds per-anchor range-bias states,
                          stillMode, Huber down-weighting, still-inversion.
  ``multilaterator.py`` → ``multilaterate.py`` — weighted LM + collinearity gate.
  ``position_ekf.py``   → ``ekf.py``.
  ``error_map.py``      → ``range_correction.py``. The Tier-2 *spatial* map was
                          replaced by the per-anchor *power* model; running both
                          would mean two corrections fighting over one error.
  ``mhe.py``            → Phase 3 ports ``+dune/MheEstimator.m`` properly. The
                          old file was Python-era code, not a port of it.
"""

from .frame_parser     import FrameParser, SweepPacket, ImuData, SurveyPacket, SurveyPair
from .anchor_config    import AnchorConfig
from .multilaterate    import multilaterate, SolveInfo
from .solve_sweep      import solve_sweep, SweepSolution
from .range_hold       import RangeHold
from .range_correction import RangeCorrection
from .nlos             import gap_weights
from .ekf              import FusionEkf, world_accel
from .imu_integrator   import ImuIntegrator
from .zupt_detector    import ZuptDetector
from .system_health    import SystemHealth
from .fusion_config    import FUSION_DEFAULTS
from .session_log      import read_session_log, SessionLog
from .metrics          import trajectory_metrics, align_truth, prctile

__all__ = [
    'FrameParser', 'SweepPacket', 'ImuData', 'SurveyPacket', 'SurveyPair',
    'AnchorConfig', 'multilaterate', 'SolveInfo', 'solve_sweep',
    'SweepSolution', 'RangeHold', 'RangeCorrection', 'gap_weights',
    'FusionEkf', 'world_accel', 'ImuIntegrator', 'ZuptDetector',
    'SystemHealth', 'FUSION_DEFAULTS', 'read_session_log', 'SessionLog',
    'trajectory_metrics', 'align_truth', 'prctile',
]
