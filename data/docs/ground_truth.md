# Ground truth

## The rig

A **Kinect v2** mounted at the ceiling looking straight down (nadir), detecting an
**AprilTag** carried by the rover. Configuration is published verbatim in
`calibration/camera/kinect_vision.json`; the MATLAB camera calibration it derives from is
`calibration/camera/camera_calibration.mat`.

| Parameter | Value |
|---|---|
| Image | 1920 × 1080 |
| Intrinsics | `fx = fy = 1050`, `cx = 960`, `cy = 540` |
| Distortion | zeros — **see caveat below** |
| Mounting height | 3.8734 m |
| Nadir pixel | (890.78, 529.51) |
| Residual tilt (diagnostic) | 3.81° |
| Mirror flip | true |
| Tag family | `tag16h5` |
| Tag size | 0.05 m |
| Min decision margin | 30.0 |

`tag_roles` in that file maps tag IDs to physical roles: IDs 1–4 mark anchors (used for the
click-truth anchor layout), IDs 5–6 are the rover tags.

Ground truth lands in `run.mat` as `/data/apriltag_pos` (m) and `/data/apriltag_euler` (rad),
sampled on the 20 Hz control stream.

## Accuracy — what we can and cannot claim

Be careful here, because the calibration file is explicit about what was *not* done:

- `intrinsics_calibrated_at` is **null** and `intrinsics_rms_px` is **null**. The intrinsics
  above are the imported MATLAB calibration, not a re-verified in-situ fit.
- `dist_coeffs` are all zero, i.e. lens distortion is **not** modelled. For a nadir camera
  observing a tag near image centre this is a small effect; near the frame edges it is not
  negligible.
- `height_calibrated_at` and `height_z_error_m` are **null** — the 3.8734 m mounting height
  is a measurement, not a residual-verified fit.
- A 3.81° tilt from true nadir is recorded as a *diagnostic*, meaning it was observed but not
  compensated in the stored model.

Consequently this dataset describes its reference as a **sub-decimetre-class planar
reference**, and does not publish a calibrated ground-truth uncertainty figure. Anyone using
these runs to claim centimetre-level absolute accuracy should first re-derive the camera
model from `camera_calibration.mat` and the checkerboard captures, and re-verify the height.
Treat the AprilTag track as a high-quality *relative* trajectory reference — which is what
the paper's RMSE comparisons actually require after the `SE(2)` fit described in
[`coordinate_frames.md`](coordinate_frames.md).

## Coverage

Every published run has ≥ 90 % ground-truth coverage, and 17/17 in fact have 100 % — see the
`gt_coverage` column in `manifest.csv`. Runs that lost tag lock were filtered out at
selection time.

## Independent cross-checks in this dataset

- `survey/*_compare.png` — camera-free anchor self-survey (MDS) solutions plotted against the
  tape-measured geometry, an independent check on the anchor layout the ground truth is
  registered to.
- `calibration/anchors/anchor_bias.json` — records a 33.6 mm RMS fit residual over 10 anchor
  pairs against the tape geometry, which bounds how well the two agree.
- `static/static_accuracy_*/spots.json` — surveyed static positions with `report.txt` giving
  measured error at each.
