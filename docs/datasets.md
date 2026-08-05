# Datasets

**The dataset ships with the code, in [`data/`](../data/).** Clone the repository and you
have it — 14 MB, 17 curated rover runs with ground truth, the static-accuracy campaign, the
channel-impulse-response captures and the anchor self-survey captures.

> **Licence differs from the code.** `data/` is **CC BY 4.0**; the rest of the repository is
> MIT. See [`data/LICENSE-DATA.md`](../data/LICENSE-DATA.md).

[`data/README.md`](../data/README.md) is the authority on layout and contents. This page is
the short version and how it connects to the pipelines.

## Layout

```
data/
├── runs/          17 driven rover runs, ranked best-first
├── static/        static-accuracy campaign, fixed positions and long dwells
├── cir/           channel-impulse-response captures
├── survey/        anchor self-survey captures + MDS solutions
├── calibration/   the calibration state used for the paper's numbers
├── environment/   site map, layout drawing, photos     📥 to be added
├── raw/           unparsed serial and session logs      📥 to be added
├── tools/         loaders for Python and MATLAB, checksum verifier
├── docs/          file formats, frames, ground truth, known issues
├── manifest.csv   machine-readable index over all 17 runs — load this first
└── checksums.sha256
```

Each run is self-contained:

```
runs/run_01_20260725_184427/
├── run.mat        MATLAB v7.3 — UWB ranges, ground truth, IMU, wheels
├── metadata.json  machine-readable summary
└── report.png     rendered trajectory / error report
```

Runs are ranked by **capture quality, not date** — `run_01` is the best of 67 candidates.
`run_17` is deliberately a 4-anchor capture so the set supports geometry-degradation
analysis rather than only the easy case. The criteria are in
[`data/docs/run_selection.md`](../data/docs/run_selection.md).

## Verify your copy

```bash
cd data && sha256sum -c checksums.sha256      # 177 files
```

## Using it

Quickest look, no DUNE install needed:

```bash
python3 data/tools/load_run.py data/runs/run_01_20260725_184427
```

Score a run through the full pipeline:

```python
from reports.rover import rover_report
r = rover_report('data/runs/run_01_20260725_184427/run.mat', 'results/out',
                 anchors_file='config/anchors_kinect_clicked_20260724.json', tag_z=0.24)
```

```matlab
rover_run_report(fullfile(dune.dataDir(), 'runs', 'run_01_20260725_184427', 'run.mat'))
mhe_replay(...)   % EKF vs rigid MHE on identical sweeps
```

Or from the browser: **GUI → LAB → Rover run report**, which lists all 17 in the file picker.

## What it unlocks

The dataset is what makes the test suite complete:

| | Passed | Skipped |
|---|---|---|
| Without `data/` | 209 | 44 |
| With `data/` | **225** | 28 |

The remaining 28 skips need `data/raw/` (session logs in `.jsonl`) and two static-campaign
artefacts, both still to be added — see the 📥 rows above.

## Two directories that sound alike

`config/` and `data/calibration/` are not the same thing:

- **`config/`** — the **live** parameters the pipelines read right now. Edit these.
- **`data/calibration/`** — a **snapshot** of the calibration state as it was when the runs
  were captured. Provenance, not configuration. Read it to understand a run; do not point
  the pipeline at it.

## Recording your own

Nothing here is specific to our arena:

```bash
python python/run_localization.py       # GUI → RUN → start / stop; scoring is automatic
```

```matlab
rover_teleop_uwb
```

Ground truth is optional — without it you still get sweeps, diagnostics and estimator
output, which covers everything except absolute scoring. See
[vision_ground_truth.md](vision_ground_truth.md).

## Citing

The dataset is covered by the paper's citation ([`CITATION.cff`](../CITATION.cff)). A
separate Zenodo DOI for the data is still to be minted — see
[release_checklist.md](release_checklist.md).
