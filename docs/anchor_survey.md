# Anchor self-survey

Recover the anchor constellation **without a tape measure and without a camera** — the
anchors range to each other, and classical multidimensional scaling turns that distance
matrix back into a set of points.

Recovers geometry to **49 mm raw** and **42 mm in shape** with no external measurement of
any kind.

## Why it works without the tag's calibration

The firmware supports a survey exchange in which a tag *instructs* one anchor to range to
another. The tag orchestrates but does not participate, so **the tag's own antenna delay
never enters the measurement**. What comes back is the full set of pairwise anchor
distances.

## Running it

```bash
# 1. Capture — the tag drives every anchor pair, 100 samples each, averaged
python python/tools/run_survey.py --serial /dev/ttyUSB0
```

or send `SURVEY\n` to the tag on its serial link, or press the survey button in the GUI's
LAB panel. MATLAB: `survey_capture(transport="serial", com="COM12")`.

The capture emits:

```
SURVEY_BEGIN,v1,<n_pairs>
SURVEY,v1,<src_id>,<dst_id>,<avg_dist_mm>,<ok_samples>
...
SURVEY_DONE,v1
```

```bash
# 2. Reconstruct — MDS, then write config/anchors.json
python python/run_localization.py    # GUI → LAB → "Reconstruct anchors from a self-survey"
```

MATLAB: `survey_reconstruct` / `build_survey_anchors`.

Reconstruction is **opt-in to writing**. It produces an `anchors.json` and shows you the
RMS against your existing geometry first; silently replacing the layout every other tool
reads would be a surprising side effect of pressing "run".

## What the result is, and is not

Two things are easy to misread.

**The MDS frame is arbitrary.** A set of distances fixes *shape* but not absolute pose or
handedness. The reconstructed points are pose- and reflection-aligned onto your existing
`anchors.json` purely to land in a readable frame and to resolve the one bit of handedness.
That is **not** the same as taking geometry from a camera — the rover solve and the run
report both re-fit the UWB→world transform, so only the anchor *shape* affects accuracy.

The alignment is performed **without scale**. A free scale factor would absorb exactly the
global error the survey exists to expose, and you would get a beautiful fit that told you
nothing.

**"Corrected" means delay-removed.** The reconstruction can optionally subtract a per-anchor
bias fitted as `meas_ij = true_ij + b_i + b_j` against a reference geometry. That is useful
for seeing *how much* of the survey error is antenna delay — but it needs a reference, so it
is not the camera-free path. Leave `corrected` off for a genuine cold survey.

## Delay fitting

The same pairwise data supports a per-anchor antenna-delay fit, reported in millimetres and
in **DW1000 ticks** — ticks being what gets written to an anchor's NVS:

```
GUI → LAB → "Per-anchor delay fit from a self-survey"
```

MATLAB: `survey_fit`.

This is a cross-check on Tier-1, not a replacement for it. Tier-1 measures against a known
distance; this infers from a self-consistent set with no absolute reference, so it can tell
you the delays are inconsistent with each other without telling you which one is wrong.

## When to re-survey

- After moving any anchor.
- After changing anchor heights.
- If static accuracy degrades across the whole floor at once — that pattern points at
  geometry rather than at any single link.

Re-surveying costs a couple of minutes and needs no reflash: it writes `config/anchors.json`
and the next solve picks it up.

## Limits

- Needs every anchor pair to range successfully. An incomplete matrix is rejected with the
  missing pairs listed, rather than silently reconstructed from partial data.
- Anchors that cannot hear each other — across a wall, at opposite ends of a long corridor —
  will not produce a complete matrix. That is a deployment constraint, not a bug.
- Recovers a *planar* configuration. Anchor heights come from your configuration, not from
  the survey.

`python/dune/survey.py` · `matlab/scripts/survey_capture.m`, `survey_fit.m`,
`build_survey_anchors.m`
