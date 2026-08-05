# Collection protocol

## Hardware

| Item | Detail |
|---|---|
| UWB modules | Makerfabs *ESP32 UWB Pro with Display* (Decawave DW1000) |
| Anchors | 5 (4 in `run_17`), static, floor/mast mounted |
| Tags | 2, rigidly mounted on the rover, 0.527 m longitudinal baseline |
| Tag roles | ID **241** front = TagLink master (WiFi + IMU); ID **240** rear = slave, no WiFi |
| Tag link | UART2 cross-wire, 921600 baud, binary framed master/slave TDMA |
| Ranging | Double-sided two-way ranging (DS-TWR), round-robin over the anchor list |
| Radio | 110 kbps, 64 MHz PRF accuracy mode |
| Rover | 6-wheel skid-steer, wheelbase 0.60 m, track 0.70 m, wheel radius 0.07 m |
| Rover control | 20 Hz, PS4-gamepad driven |
| Ground truth | ceiling Kinect v2 + AprilTag — see [`ground_truth.md`](ground_truth.md) |
| Host transport | serial (`/metadata/uwb/transport`) for the published runs |

## Architecture note

The tag streams **raw ranges only**. No position is computed on the microcontroller — every
solve happens host-side. This is why the dataset is useful independently of the DUNE code:
the published `R`/`RX`/`FP` arrays are the unprocessed sensor output, so any estimator can be
run against them without re-flashing anything.

## Per-run procedure

1. Power the anchors; confirm all expected IDs are ranging.
2. Load the anchor layout for the session — either the Kinect click-truth layout
   (`clicked_kinect_2026-07-20_z0.24`) or a fresh camera-free self-survey run. Which one each
   published run used is recorded in its `metadata.json` `anchors_layout` field, and resolved
   to a file by [`../calibration/anchors/layout_index.md`](../calibration/anchors/layout_index.md).
3. Start the ceiling-camera AprilTag detector.
4. Start synchronised capture: UWB sweep stream + rover control stream, both stamped with
   POSIX time.
5. Drive a random-excitation trajectory across the arena for ~1–3 minutes.
6. Stop capture; the report generator writes the `.mat` and the `report.png` automatically.

Runs were captured over five sessions between **2026-07-23 and 2026-07-25**; the archive
extends to 2026-08-05 but those later captures did not survive the eligibility filters.

## Calibration state during capture

Tier-1 (hardware antenna delay) was applied on-device via the on-tag `HWCALIB` binary search;
Tier-2 (first-path-power range correction) is applied **host-side at solve time** and is
therefore *not* baked into the published `R` values. The published ranges are raw. See
[`calibration.md`](calibration.md).

## Environment

Indoor arena, mixed line-of-sight and obstructed geometry. Site map, anchor mounting drawing
and deployment photos are **pending** — see `environment/README.md`.
