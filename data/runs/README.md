# Runs

17 curated driven rover runs, **ranked by capture quality** — `run_01` is the best. See
`../docs/run_selection.md` for how they were chosen from 67 candidates, and `../manifest.csv`
for a machine-readable index.

```
run_NN_<YYYYMMDD_HHMMSS>/
├── run.mat         # MATLAB v7.3 (HDF5) — see ../docs/file_formats.md
├── metadata.json   # this run's summary and provenance
└── report.png      # rendered trajectory / error report
```

Before you compute error metrics, read two things:

1. `../docs/coordinate_frames.md` — the AprilTag and UWB frames are logged **raw**; fit an
   SE(2) transform first.
2. `../docs/known_issues.md` — in particular §1 (`run_17`'s exact anchor layout is not
   published) and §7 (the 17 runs span two anchor-layout families; group by `anchors_layout`
   before aggregating).

Load one with `python3 ../tools/load_run.py run_01_20260725_184427`.
