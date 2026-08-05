# Anchor layout index

Each run's `metadata.json` carries an `anchors_layout` string. This table resolves it to the
file that defines the geometry.

| `anchors_layout` | Defined by | Anchors | Used by |
|---|---|---|---|
| `clicked_kinect_2026-07-20_z0.24` | `anchors_kinect_clicked_20260724.json` | 5 (IDs 1–5) | runs 01, 03, 06, 07, 08, 11, 12, 14, 15, 16 |
| `self_survey_raw_survey_20260724_164356` | `../../survey/survey_20260724_164356_anchors_raw.json` (and `_anchors_corrected.json`) | 5 (IDs 1–5) | runs 02, 04, 05, 09, 10, 13 |
| `self_survey_raw_survey_20260725_233419` | **not published** — nearest available is `anchors_dune_selfsurvey_20260725.json` (`survey_20260725_234541`) | 4 (IDs 2–5) | run 17 — see `../../docs/known_issues.md` §1 |

Not referenced by any published run, included for provenance:

| File | Note |
|---|---|
| `anchors.json` | Pipeline default at export time, 4 anchors, 2026-07-10. No published run uses it |
| `anchors_rect6p3x3p0_2026-07-01.json` | Tape-measured 6.3 × 3.0 m rectangle plus top-edge anchor at x = 3.3 |
| `anchors.example.json` | Schema template |

## Schema

```json
{
  "dim": 2,
  "bounds": [xmin, xmax, ymin, ymax, zmin, zmax],
  "anchors": [{"id": 1, "x": 0.0, "y": 0.0, "z": 0.0}, ...],
  "layout": "<the anchors_layout string>"
}
```

`dim: 2` means the solve is planar; the `z` field is carried but not estimated.
