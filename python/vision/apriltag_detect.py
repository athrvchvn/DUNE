"""
apriltag_detect.py — multi-tag AprilTag detection + true world-position PnP.

Unlike backproject.py's pixel_to_world() (a floor-plane-assumption ray-cast,
no physical marker needed — used by the manual click-to-place flow), this
requires a real printed AprilTag of known size and gets genuine (x, y, z)
straight from pupil_apriltags' pose solve (it runs PnP internally given
camera_params + tag_size) — no Z-plane assumption needed.

Camera -> world convention matches backproject.py exactly:
    world = (cam_x, -cam_y, mounting_height_m - cam_z)

Role-mapping (which AprilTag ID means which anchor/tag), duplicate-ID
handling, bounds/plausibility checks, and the confirm-before-write workflow
are business logic tied to anchors/roles, not generic vision math — they
live in run_localization.py, not here. This module stays reusable/
role-agnostic: it only knows how to turn a frame into a list of tag
detections with world positions.
"""
import re
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .vision_config import VisionConfig

_ANCHOR_ROLE_RE = re.compile(r'^anchor(\d+)$')


def parse_anchor_role(role: str) -> Optional[int]:
    """'anchor3' -> 3; any other role string (e.g. 'tagA') -> None."""
    m = _ANCHOR_ROLE_RE.match(role)
    return int(m.group(1)) if m else None


@dataclass
class TagDetection:
    tag_id: int
    # Raw camera-frame pose — independent of mounting_height_m. Used by
    # height_calibration.py, which must stay non-circular (it's what
    # calibrates mounting_height_m in the first place).
    camera_x: float
    camera_y: float
    camera_z: float
    # Derived world position — depends on cfg.mounting_height_m.
    world_x: float
    world_y: float
    world_z: float
    corners: list             # 4 [u, v] pixel points, for overlay drawing
    decision_margin: float
    hamming: int


def detect_tags(frame_bgr: np.ndarray, cfg: VisionConfig) -> list[TagDetection]:
    """Detect every cfg.apriltag_family/apriltag_size_m tag in frame_bgr.

    Contract: frame_bgr must ALREADY be mirror_correct()'d (same contract as
    backproject.pixel_to_world) — callers that need one frame for detection,
    overlay drawing, AND display/archival should correct once and reuse it,
    not re-flip per use.

    Only hamming==0 (exact, non-error-corrected) detections at or above
    cfg.apriltag_min_decision_margin are returned: tag16h5's short code is
    more false-positive-prone than tag36h11, so low-confidence and
    error-corrected reads are rejected outright rather than trusted.
    """
    try:
        from pupil_apriltags import Detector
    except ImportError as e:
        raise RuntimeError("pip install pupil-apriltags") from e
    import cv2

    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    detector = Detector(families=cfg.apriltag_family)
    raw = detector.detect(
        gray, estimate_tag_pose=True,
        camera_params=(cfg.fx, cfg.fy, cfg.cx, cfg.cy),
        tag_size=cfg.apriltag_size_m,
    )

    out = []
    for d in raw:
        if d.hamming != 0:
            continue
        if float(d.decision_margin) < cfg.apriltag_min_decision_margin:
            continue
        cam_x, cam_y, cam_z = float(d.pose_t[0]), float(d.pose_t[1]), float(d.pose_t[2])
        out.append(TagDetection(
            tag_id=int(d.tag_id),
            camera_x=cam_x, camera_y=cam_y, camera_z=cam_z,
            world_x=cam_x, world_y=-cam_y, world_z=cfg.mounting_height_m - cam_z,
            corners=[[float(x), float(y)] for x, y in d.corners],
            decision_margin=float(d.decision_margin), hamming=int(d.hamming),
        ))
    return out


def draw_overlay(frame_bgr: np.ndarray, detections: list[TagDetection]) -> np.ndarray:
    """Copy of frame_bgr with each tag's outline + ID label drawn — for the
    review-step preview sent back after an auto-detect action."""
    import cv2
    out = frame_bgr.copy()
    for d in detections:
        pts = np.array(d.corners, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        c = pts.mean(axis=0).astype(int).ravel()
        cv2.putText(out, str(d.tag_id), (int(c[0]) - 10, int(c[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return out
