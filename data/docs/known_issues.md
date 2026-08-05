# Known issues

An honest list. Every item here is a real limitation of the published data, not a caveat
about how to use it correctly.

## 1. Anchor geometry for `run_17` is not published

`run_17` records `anchors_layout = self_survey_raw_survey_20260725_233419`. The `survey/`
folder contains `survey_20260725_233226` and `survey_20260725_233304`, but **not** `_233419`,
and no matching anchor JSON was found in the archive. The nearest available 4-anchor
self-survey solution is `calibration/anchors/anchors_dune_selfsurvey_20260725.json`
(`survey_20260725_234541`, same session, same anchor IDs 2–5), which is a close but **not
identical** geometry.

Use `run_17` for geometry-degradation and relative-consistency analysis; do not use it for
absolute-accuracy numbers until the exact layout is recovered.

## 2. Ground-truth camera model is not residual-verified

`kinect_vision.json` has null `intrinsics_calibrated_at`, null `intrinsics_rms_px`, null
`height_calibrated_at`, zero distortion coefficients, and a 3.81° tilt recorded as a
diagnostic rather than compensated. See [`ground_truth.md`](ground_truth.md). No calibrated
ground-truth uncertainty figure is published.

## 3. AprilTag and UWB frames are not co-registered in the files

By design — both are logged raw and an `SE(2)` transform is fitted offline. A direct
comparison without that fit reports a constant offset that is an artefact. See
[`coordinate_frames.md`](coordinate_frames.md).

## 4. Machine-specific absolute paths inside `run.mat`

`/metadata/uwb/anchors_file` and `/metadata/ported_from` contain Windows paths from the
capture machine (`D:\UWB_3\...`, `D:\MMS_Codebase\...`), and `/metadata/uwb/tag_com` records
the serial port it was captured on (`COM12`). They are inert provenance strings — nothing
reads them — but they are not portable and should not be interpreted as instructions to load
anything. No WiFi credentials or host IPs appear anywhere in this dataset; the published runs
were captured over serial.

## 5. Six archive captures were unreadable

Six `.mat` files in the source archive (2026-08-04 and 2026-08-05 sessions) fail to open with
the expected group structure — truncated or written by an interrupted export. They were
excluded at the eligibility stage and are not part of the 17.

## 6. `anchors.json` is not the runs' layout

The pipeline default shipped as `calibration/anchors/anchors.json` is a 4-anchor 2026-07-10
layout that **no published run uses**. It is included for provenance. Resolve geometry via
each run's `anchors_layout` field and `calibration/anchors/layout_index.md`.

## 7. Two anchor-layout families across the 17 runs

10 runs use the Kinect click-truth layout, 6 use a 2026-07-24 self-survey solution, 1 uses an
unpublished 2026-07-25 self-survey. Aggregating error metrics across all 17 without
accounting for the layout change mixes two different geometry sources. Group by
`anchors_layout` when reporting.

## 8. UWB sweep rate is per-tag, not per-sample

`manifest.csv`'s `mean_range_hz` counts **all** sweeps, both tags interleaved. Per-tag rate is
roughly half that. `metadata/data_quality/avg_rate_hz` inside the `.mat` refers to the control
stream and is a different quantity again.

## 9. Placeholder folders are empty

`environment/` and `raw/` currently contain only `README.md` files describing what belongs
there. They must be filled before the dataset DOI is minted.
