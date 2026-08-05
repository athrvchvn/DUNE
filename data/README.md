# DUNE Dataset — Dual-Tag UWB Navigation Engine

Companion dataset for **_DUNE: A Dual-Tag UWB Navigation Engine for Indoor Ground Robots_**.

Ultra-wideband (DW1000) two-way-ranging captures from a dual-tag skid-steer rover in an
indoor arena, with **synchronised AprilTag/Kinect ground truth**, wheel and IMU telemetry,
the full calibration state used to produce the paper's numbers, and supporting static,
channel-impulse-response and anchor-self-survey captures.

- **Code:** <https://github.com/athrvchvn/DUNE>
- **Paper:** *(citation + DOI — TODO before release)*
- **Dataset DOI:** *(Zenodo — TODO before release)*
- **Data licence:** CC BY 4.0 (see `LICENSE-DATA.md`). The code in `tools/` is MIT.

---

## What is in here

| Folder | Contents | Status |
|---|---|---|
| `runs/` | **17 curated driven rover runs**, one folder each, with ground truth | ✅ populated |
| `static/` | Static-accuracy campaign — fixed positions, long dwells | ✅ populated |
| `cir/` | Channel-impulse-response captures (per-anchor waterfalls) | ✅ populated |
| `survey/` | Camera-free anchor self-survey captures and MDS solutions | ✅ populated |
| `calibration/` | Anchor geometry, antenna delays, range correction, camera intrinsics | ✅ populated |
| `environment/` | Site map, anchor layout drawing, deployment photos | 📥 **to be added** |
| `raw/` | Unparsed serial captures and host session logs | 📥 **to be added** |
| `tools/` | Loaders for Python and MATLAB, checksum verifier | ✅ populated |
| `docs/` | File formats, frames, ground-truth rig, protocol, known issues | ✅ populated |

Folders marked 📥 exist with a `README.md` stating exactly what belongs there; they are
placeholders to be filled before the DOI is minted.

## The 17 runs

Every run is a self-contained folder:

```
runs/run_01_20260725_184427/
├── run.mat          # MATLAB v7.3 (HDF5) — UWB ranges, ground truth, IMU, wheels
├── metadata.json    # machine-readable summary of this run
└── report.png       # rendered trajectory / error report
```

Runs are **ranked by capture quality**, not chronology: `run_01` is the best. They were
selected from 67 candidate captures by the criteria in
[`docs/run_selection.md`](docs/run_selection.md) — briefly: ≥25 s duration, ≥90 % ground-truth
coverage, both tags present, then scored on duration, driven path length and the fraction of
sweeps with ≥4 anchors ranging.

`run_17` is deliberately a **4-anchor** capture rather than the next-best 5-anchor one, so the
set supports geometry-degradation analysis instead of only reporting the easy case.

[`manifest.csv`](manifest.csv) is the machine-readable index over all 17 — load it first.

## Quickstart

```bash
python3 tools/load_run.py runs/run_01_20260725_184427
```

```matlab
addpath('tools');
r = load_run('runs/run_01_20260725_184427');
plot(r.gt.pos(1,:), r.gt.pos(2,:));
```

To replay a run through the full DUNE estimator, clone the code repo and see its
`docs/matlab_pipeline.md` (`mhe_replay`) or `docs/python_pipeline.md` (replay jobs).

## Verify your download

```bash
sha256sum -c checksums.sha256
```

## Citing

If you use this dataset, please cite the DUNE paper *(BibTeX — TODO)* and the dataset DOI.

## Documentation

| Doc | What it answers |
|---|---|
| [`docs/file_formats.md`](docs/file_formats.md) | Every field in `run.mat`, with units |
| [`docs/coordinate_frames.md`](docs/coordinate_frames.md) | Arena, body and tag frames; the offsets between them |
| [`docs/ground_truth.md`](docs/ground_truth.md) | The AprilTag + Kinect v2 reference rig and its accuracy |
| [`docs/collection_protocol.md`](docs/collection_protocol.md) | Hardware, site, and how a run was recorded |
| [`docs/run_selection.md`](docs/run_selection.md) | Why these 17 and not the other 50 |
| [`docs/calibration.md`](docs/calibration.md) | What each calibration file is and when it was measured |
| [`docs/known_issues.md`](docs/known_issues.md) | Honest list of the dataset's defects |
