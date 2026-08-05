# Coordinate frames

## Arena frame (`W`)

Right-handed, `z` up, origin and axes defined by the anchor layout in
`calibration/anchors/`. All anchor coordinates, all AprilTag ground-truth positions and all
solved positions are expressed in this frame. Anchors are treated as 2-D (`dim: 2`) in the
published layouts; the `z` column is present but the solve is planar.

## Rover body frame (`B`)

`x` forward, `y` left, `z` up, origin at the geometric centre of the rover. Rover geometry
from `/metadata`: wheelbase 0.60 m, track width 0.70 m, wheel radius 0.07 m.

## Tags

Two UWB tags on a rigid longitudinal baseline of **0.527 m** (`/metadata/uwb/baseline_m`):

- **front tag, ID 241** — TagLink master (has WiFi and the IMU)
- **rear tag, ID 240** — TagLink slave

The rigid-baseline constraint between them is what the paper's rigid-body MHE exploits.

## Ground-truth marker

The AprilTag observed by the ceiling camera is **not** at the body origin. Its offset in the
body frame is `/metadata/uwb/apriltag_offset_body` = `[-0.1135, -0.1000]` m. Subtract this
lever arm (rotated by the ground-truth yaw) before comparing an estimate of the body origin
against `apriltag_pos`.

## The frame caveat that matters

`/metadata/uwb/frame_note` records:

> *"apriltag and uwb frames logged raw; transform fitted offline"*

The AprilTag ground truth and the UWB anchor geometry were **not** registered to a common
origin at capture time. Both are internally consistent, but a rigid `SE(2)` transform
(rotation + translation, **no scale**) between them must be fitted per layout before error
metrics mean anything. This is deliberate — it keeps the raw observations unmodified — but a
naive `‖estimate − apriltag_pos‖` without fitting the transform first will report a large
constant offset that is an artefact, not error.

The DUNE code repo performs this fit in its rover-report path; see `matlab/scripts/` and
`docs/matlab_pipeline.md` there.

## Time

Two clocks, both present:

- `t_rel` / `time` — seconds from capture start, per stream, and **not** shared between the
  UWB and control streams.
- `t_posix` — absolute POSIX seconds, present on both streams. **Use this to join them.**

Tag-side sweeps additionally carry `CYC` (dual-tag cycle id) and `MRX` (master receive
timestamp) in the raw wire format; these are collapsed during `.mat` export.
