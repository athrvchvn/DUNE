"""
vision_config.py — single source of truth for the overhead Kinect vision config.

Mirrors visionSystemConfig.m's role in the MATLAB reference: every script and
the WS server import this one object instead of hardcoding fx/fy/height
anywhere. The reference's own history is exactly what this avoids — it shipped
two disagreeing intrinsic sets (fx=800 in the "production" class vs. fx=1050
in the config, the former explicitly documented as rejected) and a mounting
height that drifted 4.0 -> 3.32 -> 3.43 m across files that never got
resynced.
"""
import json
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Optional

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR.parent.parent / 'matlab' / 'config' / 'kinect_vision.json'


@dataclass
class VisionConfig:
    fx: float = 1050.0
    fy: float = 1050.0
    cx: float = 960.0
    cy: float = 540.0
    dist_coeffs: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0])
    image_width:  int = 1920
    image_height: int = 1080

    # Straight-down overhead mount, no tilt (see docs/kinect_vision_setup.md).
    mounting_height_m: float = 3.4
    mirror_flip: bool = True

    intrinsics_calibrated_at: Optional[str] = None
    intrinsics_rms_px:        Optional[float] = None
    height_calibrated_at:     Optional[str] = None
    height_z_error_m:         Optional[float] = None

    # ── Provenance when fx/fy/cx/cy/mounting_height_m came from an external
    # calibration (e.g. import_camera_calibration.py) rather than this
    # project's own checkerboard/height-tag scripts. nadir_px/tilt_deg here
    # are diagnostic only — carried over for reference, never read by any
    # transform math (world_registration.py's registered transform is what
    # actually corrects tilt; see its own implied_tilt_deg for the real,
    # solved value once a registration exists).
    camcal_source:       Optional[str]   = None
    camcal_date:         Optional[str]   = None
    nadir_px:             Optional[list]  = None
    tilt_deg_diagnostic:  Optional[float] = None

    # ── AprilTag auto-detect (physical markers on anchors/tag boards) ────────
    apriltag_family: str   = 'tag16h5'
    apriltag_size_m: float = 0.05

    # Confidence gating: tag16h5's short code is more false-positive-prone
    # than tag36h11, so reject low-confidence reads outright.
    apriltag_min_decision_margin: float = 30.0

    # Plausibility gating for the auto-detect -> anchors.json path: how far
    # outside the current workspace bounds a detection may land before it's
    # rejected as a glitch, and how far a re-detected anchor may move from
    # its current stored position before the GUI flags (not blocks) it.
    bounds_padding_m:    float = 2.0
    anchor_jump_warn_m:  float = 0.5

    # ID -> role mapping for the physical print set, e.g. {"1": "anchor1",
    # "5": "tagA"}. "anchorN" roles map that AprilTag ID to anchor N in
    # anchors.json; any other role string is a non-anchor tag position (its
    # coordinates fill the HWCALIB tag-position fields instead of writing a
    # file). Purely config-driven — adding more anchors/tags is a JSON edit,
    # never a code change.
    tag_roles: dict = field(default_factory=lambda: {
        "1": "anchor1", "2": "anchor2", "3": "anchor3", "4": "anchor4",
        "5": "tagA",    "6": "tagB",
    })

    @classmethod
    def load(cls, path: Path | str | None = None) -> 'VisionConfig':
        path = Path(path) if path else DEFAULT_CONFIG_PATH
        if not path.exists():
            return cls()
        with open(path) as f:
            data = json.load(f)
        known = {k: v for k, v in data.items() if k in cls.__dataclass_fields__}
        return cls(**known)

    def save(self, path: Path | str | None = None) -> None:
        path = Path(path) if path else DEFAULT_CONFIG_PATH
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w') as f:
            json.dump(asdict(self), f, indent=2)
            f.write('\n')
