# File formats

## `runs/*/run.mat`

MATLAB **v7.3**, i.e. an HDF5 container. `scipy.io.loadmat` will **not** read it — use
`h5py` (or `tools/load_run.py`, which wraps this).

> HDF5 stores MATLAB arrays transposed. A MATLAB `3×N` array reads back as `(3, N)` in
> h5py here because the writer emitted it that way; the loaders normalise orientation, but
> if you read raw, always check `shape` rather than assuming.

### `/data/uwb` — the UWB sweep stream (one column per sweep)

| Field | Shape | Units | Meaning |
|---|---|---|---|
| `R` | `A×S` | m | Two-way-ranging distance, anchor *a* to the ranging tag. `NaN`/`≤0` = no measurement this sweep |
| `RX` | `A×S` | dBm | Total receive power of the range report frame |
| `FP` | `A×S` | dBm | First-path power of the **same** frame. `RX − FP` is the NLOS gap the paper's weighting uses |
| `anchorIds` | `A×1` | — | Anchor short addresses, row order of `R`/`RX`/`FP` |
| `tag` | `1×S` | — | Which tag produced this sweep (`240` rear, `241` front) |
| `t_rel` | `1×S` | s | Time from run start. May start slightly negative |
| `t_posix` | `1×S` | s | Absolute POSIX time |
| `quat` | `4×S` | — | Tag IMU orientation, `[w x y z]` |
| `acc` | `3×S` | m/s² | Tag IMU acceleration |
| `gyro` | `3×S` | rad/s | Tag IMU angular rate |

`A` is 5 for most runs and 4 for `run_17`. `S` is the sweep count.

**Sweeps interleave the two tags.** Filter by `tag` before treating the stream as one
sensor's time series — this is the single most common mistake with this data.

### `/data` — the rover control stream (one column per control sample, 20 Hz nominal)

| Field | Shape | Units | Meaning |
|---|---|---|---|
| `apriltag_pos` | `3×N` | m | **Ground-truth position** in the arena frame |
| `apriltag_euler` | `3×N` | rad | Ground-truth orientation, roll-pitch-yaw |
| `time`, `t_posix` | `1×N` | s | Relative and absolute timestamps |
| `imu_euler` | `3×N` | rad | Rover IMU orientation |
| `imu_accel` | `3×N` | m/s² | Rover IMU acceleration |
| `imu_gyro` | `3×N` | rad/s | Rover IMU angular rate |
| `imu_accuracy` | `1×N` | 0–3 | BNO085 calibration-accuracy flag |
| `wheel_rpm`, `motor_rpm` | `6×N` | rpm | Measured, per wheel / per motor |
| `commanded_motor_rpm` | `6×N` | rpm | Commanded |
| `steering_angles` | `4×N` | rad | Corner steering |
| `V_cmd`, `omega_cmd` | `1×N` | m/s, rad/s | Commanded body twist |
| `segment_id` | `1×N` | — | Excitation-segment index within the run |

The UWB stream and the control stream have **different rates and independent clocks**;
join them on `t_posix`, not by index.

### `/metadata`

Rover geometry (`wheelbase` 0.60 m, `track_width` 0.70 m, `wheel_radius` 0.07 m,
`gear_ratio`), the capture's `test_start_posix`, `data_quality` counters, and a `/metadata/uwb`
block giving `frontTag`/`rearTag` IDs, the dual-tag `baseline_m` (0.527 m), the
`apriltag_offset_body` lever arm, `anchors_layout`, and `frame_note` recording that the
AprilTag and UWB frames are logged **raw** — the rigid transform between them is fitted
offline.

## `runs/*/metadata.json`

The same run summarised for filtering without opening the HDF5 file: duration, path length,
sweep count, anchor count, mean anchors per sweep, fraction of sweeps with ≥4 anchors,
ground-truth coverage, and the quality score used for ranking. `manifest.csv` is these
17 files flattened into one table.

## `calibration/**/*.json`

Plain JSON, read directly by both DUNE pipelines. See [`calibration.md`](calibration.md).

## `static/`, `cir/`, `survey/`

- `static/static_accuracy_*/` — `bakeoff.mat` (estimator comparison), `spots.json` (surveyed
  test positions), `serial_raw.log` (the raw capture line stream), `report.txt`, plots.
- `cir/cir_*/` — `cir_A<n>.mat`, the accumulator dump for one anchor link, plus its waterfall plot.
- `survey/survey_*` — `.txt` capture logs, `_anchors_raw.json` / `_anchors_corrected.json`
  MDS solutions, `_compare.png` against the surveyed truth.

## Raw wire format

`serial_raw.log` lines are the DUNE v1 wire format:

```
RTLS,v1,<t_ms>,<tag_id>,<n>,<id,d_mm,q>...[,IMU,<qw,qx,qy,qz,ax,ay,az>][,CYC,<n>][,MRX,<us>]
```

`q` is receive power in dBm. Full grammar, including the `DIAG,v1` line and the dual-tag
UART framing, is in the code repo's `docs/protocol.md`.
