# Anchor self-survey captures

Camera-free anchor self-survey: anchors range to each other, the pairwise distance matrix is
solved by multidimensional scaling, and the result is aligned (rotation + translation, **no
scale**) to give an anchor layout without any external measurement.

| Pattern | Contents |
|---|---|
| `survey_<ts>.txt` | The capture log — pairwise ranges, sample counts per pair |
| `survey_<ts>_anchors_raw.json` | The raw MDS solution |
| `survey_<ts>_anchors_corrected.json` | After per-anchor bias correction |
| `survey_<ts>_anchors_raw_py.json` | The Python pipeline's solution, for cross-check |
| `survey_<ts>_compare.png` | Solution plotted against the tape-measured geometry |

Six of the 17 published runs use `survey_20260724_164356` as their anchor geometry — see
`../calibration/anchors/layout_index.md`.

The `_compare.png` plots are the honest measure of how well this works: they show the
camera-free solution against tape truth for the same anchors.
