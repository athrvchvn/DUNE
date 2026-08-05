# Datasets

> ## ⬜ Status: pending curation
>
> The datasets described here **are not yet published**. They exist — every number in the
> paper comes from them — but they have not been curated into a citable release. Until then
> the repository ships code and configuration only, and `data/` holds just its README.
>
> This is a known gap and it is tracked in [release_checklist.md](release_checklist.md).

## Why they are not in the repository

Git stores every version of every file forever. The recorded campaigns run to hundreds of
megabytes, and a clone of the source should not cost that. Data therefore ships as a
separate versioned archive with its own DOI — which is also what makes it *citable*
independently of the code.

## What the drop will contain

All of it recorded on a single clock, so the streams align exactly rather than by inference.

### Rover runs

28 teleoperated driven sessions, each with:

- UWB sweeps from **both** tags with full per-anchor radio diagnostics — range, received
  power, first-path power, receive quality, carrier frequency offset
- AprilTag fiducial ground truth from the overhead Kinect
- inertial data (quaternion, acceleration, gyro rate)
- wheel encoder / command data

Includes the two runs analysed in the paper: the estimator comparison (126 mm rigid MHE vs
171 mm per-tag EKF) and the best-scoring run (95 mm).

Format: MATLAB v7.3 `.mat` plus a streaming `.ndjson` written as the run happens, so a
session that dies mid-drive still leaves everything up to that moment. Both pipelines read
both.

### Static accuracy campaign

20 surveyed positions, **2471 sweeps**, 20 s dwell per position, with camera-derived truth
for each. This is the data the Tier-2 correction was fitted and cross-validated on
(leave-one-position-out), and the data behind the 199 mm → 94 mm result and the 72 mm oracle
bound.

### Channel impulse response captures

Raw DW1000 accumulator readouts supporting the multipath analysis — the measurements showing
the direct path ≈20 dB below the strongest reflection and only 8–10 dB above noise, which is
what sets the accuracy floor. Captured with `matlab/scripts/cir_capture.m`.

### Anchor self-survey captures

Raw inter-anchor pairwise ranging sessions and the reconstructed geometries, behind the
49 mm raw / 42 mm shape survey result.

## Configuration is already here

The `config/` directory in this repository holds the deployment's actual parameters — anchor
geometry, per-anchor Tier-2 coefficients, Tier-1 traceability records, camera calibration
and world registration. That is configuration rather than data, it is small, and the code
does not run without it, so it is tracked.

| File | What |
|---|---|
| `anchors.json` | the active anchor geometry |
| `anchors_kinect_clicked_20260724.json` | the 5-anchor layout the reported results use |
| `anchors_dune_selfsurvey_20260725.json` | the same constellation, recovered by self-survey |
| `range_correction.json` | Tier-2 per-anchor coefficients, LOPO-fitted |
| `calibration.json` | Tier-1 antenna-delay traceability, with iteration history |
| `kinect_vision.json` | overhead camera intrinsics and pose |
| `error_map.json` | the retired spatial map, kept as a recorded result |

> Note that `config/anchors.json` currently holds a **4-anchor** layout. The 5-anchor
> deployment the paper reports is `anchors_kinect_clicked_20260724.json` — pass it
> explicitly when replaying those sessions, as the tests and examples do.

## Using the drop

Unpack into `data/`, matching the layout in [`data/README.md`](../data/README.md). Then:

```bash
python -m pytest python/tests -q     # the 44 skipped tests now run
```

```python
from reports.rover import rover_report
rover_report('data/rover_runs/rover_uwb_20260726_131929.mat', 'results/out',
             anchors_file='config/anchors_kinect_clicked_20260724.json', tag_z=0.24)
```

## Recording your own

Nothing here is special to our arena. A capture is one command:

```bash
python python/run_localization.py       # GUI → RUN → start / stop; scoring is automatic
```

```matlab
rover_teleop_uwb                        % MATLAB equivalent
```

Ground truth is optional — without it you still get sweeps, diagnostics and estimator
output, which is enough for everything except absolute scoring. See
[vision_ground_truth.md](vision_ground_truth.md) if you want truth too.
