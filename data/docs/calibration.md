# Calibration files

Everything in `calibration/` is **configuration**, not data: it is what the DUNE pipelines
read to turn the raw ranges in `runs/` into positions. All of it is plain JSON.

## `calibration/anchors/`

| File | What it is |
|---|---|
| `anchors_kinect_clicked_20260724.json` | **5-anchor Kinect click-truth layout** (`clicked_kinect_2026-07-20_z0.24`) — the geometry for 10 of the 17 runs |
| `anchors.json` | The pipeline's *current* default at export time — a 4-anchor 2026-07-10 layout. **Not** the layout the published runs used; kept for provenance only |
| `anchors.example.json` | Template showing the schema |
| `anchors_rect6p3x3p0_2026-07-01.json` | Tape-measured rectangular layout, 6.3 × 3.0 m plus a top-edge anchor at x = 3.3 |
| `anchors_dune_selfsurvey_20260725.json` | A camera-free self-survey solution (4 anchors, IDs 2–5) |
| `anchor_bias.json` | Additive per-anchor and per-tag range bias in metres, from self-survey; 33.6 mm RMS fit residual over 10 pairs |
| `tag_truth.json` | Surveyed tag reference positions |
| `layout_index.md` | **Start here** — maps each run's `anchors_layout` string to the file that defines it |

## `calibration/antenna_delay/` — Tier 1

`calibration.json` (schema `uwb_rtls_calibration_v4`) records the per-board antenna delay
found by the on-tag `HWCALIB` binary search, with the full iteration history, the true
reference distance, the final residual, and the firmware git commit for each board.

Read the file's own `note` carefully:

> *"Antenna delay stored in anchor NVS (set by HWCALIB). Traceability only — NOT applied by
> the host pipeline."*

Tier 1 is **already baked into the hardware** that produced these captures. It is published
for traceability. Do not apply it again in software — that is the exact double-correction bug
the DUNE calibration redesign removed.

`delay_calibration.json`, `delay_tuning_sweep.json` and `delay_tuning_tag241.json` are the
supporting sweeps.

## `calibration/range_correction/` — Tier 2

`range_correction.json` (schema `dune_range_correction_v1`, 2026-07-20) is the per-anchor
correction that **is** applied host-side at solve time:

```
corr_mm          = slope_mm_per_dB · fp + intercept_mm     (fp clamped to fp_valid_dBm)
corrected_range  = raw_range − corr_mm / 1000
```

`fp` is the first-path power in dBm — the `FP` array in `run.mat`. The clamp to
`fp_valid_dBm` is an extrapolation guard and must not be dropped.

Fitted on `static_accuracy_20260720_195637` (19 surveyed spots, Kinect click-truth), with a
**leave-one-spot-out position RMSE of 100 mm** recorded in the file. The corresponding static
capture is published under `static/` so the fit can be reproduced.

`error_map.json` is the earlier spatial k-NN/IDW error-map correction. It is superseded by
the first-path-power model above and is included for reference; the two are alternatives, not
to be stacked.

## `calibration/camera/`

`kinect_vision.json` and `camera_calibration.mat` — the ground-truth rig. Its limitations are
documented candidly in [`ground_truth.md`](ground_truth.md); read that before using this
dataset for absolute-accuracy claims.

## Order of operations

```
raw R  →  Tier-2 first-path-power correction  →  MAD outlier gate
       →  NLOS weighting from (RX − FP)       →  weighted LM multilateration
       →  EKF  or  rigid-body dual-tag MHE
```

Tier 1 sits before all of it, in the radio. Anchor bias (`anchor_bias.json`) is an
*alternative* static correction from the self-survey era; it and the Tier-2 model both target
the same systematic error, so apply one or the other.
