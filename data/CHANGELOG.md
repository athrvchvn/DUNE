# Changelog

All notable changes to this dataset. Format follows Keep a Changelog; the dataset is
versioned independently of the code repository.

## [Unreleased]

### To do before v1.0.0 / DOI
- Populate `environment/` (site map, anchor layout drawing, deployment photos)
- Populate `raw/` (per-run serial captures, scrubbed session logs)
- Recover the exact anchor layout for `run_17` (`survey_20260725_233419`) — see
  `docs/known_issues.md` §1
- Fill in the paper citation, BibTeX and dataset DOI in `README.md` and `LICENSE-DATA.md`
- Re-run `tools/make_checksums.sh` after the above

## [0.9.0] — 2026-08-06

Initial curated release, pre-DOI.

### Added
- 17 curated dual-tag rover runs selected from 67 archive captures, ranked by quality score
  (`docs/run_selection.md`), each with `run.mat`, `metadata.json` and `report.png`
- `manifest.csv` — machine-readable index over all 17 runs
- Static-accuracy campaign, CIR captures, and anchor self-survey captures
- Full calibration state: anchor layouts, Tier-1 antenna delays, Tier-2 range correction,
  camera intrinsics, with `calibration/anchors/layout_index.md` resolving each run's geometry
- Python and MATLAB loaders, checksum tooling
- Documentation: file formats, coordinate frames, ground truth, collection protocol,
  run selection, calibration, known issues
- Placeholder trees for `environment/` and `raw/`
