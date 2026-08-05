# How the 17 runs were selected

67 driven rover captures exist in the research archive. Publishing all of them would
publish mostly noise: aborted starts, 2-second captures, runs where the ground-truth camera
lost the tag. This dataset ships a curated 17. The procedure is recorded here so the
selection is reproducible and auditable rather than a matter of taste.

## Stage 1 — eligibility (hard filters)

A capture is eligible only if **all** of these hold:

| Filter | Threshold | Rejects |
|---|---|---|
| Duration | > 25 s | aborted and test captures (some were 1.9 s) |
| Ground-truth coverage | > 90 % of control samples have a finite AprilTag pose | runs where the ceiling camera lost lock |
| Tags present | exactly 2 | single-tag captures, which cannot exercise the rigid-body MHE |
| Readable | opens as MATLAB v7.3 with the expected groups | 6 truncated/corrupt files |

67 candidates → **51 eligible**.

## Stage 2 — quality score (ranking)

Eligible runs are ranked by

```
score = 0.30 · min(duration, 180 s)/180
      + 0.35 · fraction of sweeps with ≥ 4 anchors ranging
      + 0.35 · min(path_length, 60 m)/60
```

The three terms are what makes a run *useful* rather than merely valid:

- **duration** — longer runs let filter transients settle and expose drift;
- **fraction of sweeps with ≥4 anchors** — the geometric conditioning actually available to
  the solver, which matters more than the nominal anchor count;
- **path length** — a rover that barely moved gives a near-static run, which the `static/`
  campaign already covers better.

Each term saturates so that one very long or very long-pathed run cannot dominate.

## Stage 3 — one deliberate substitution

The top 16 by score are taken as-is. Slot 17 is **not** the 17th-ranked run
(`20260724_222427`, 5 anchors) but `20260725_234343`, the best-scoring **4-anchor** capture
(rank 26 overall). Shipping only 5-anchor runs would make the set unable to answer "what
happens when one anchor drops out?" — a question the paper's geometry discussion raises. The
substitution is recorded in that run's `metadata.json` `notes` field.

## Reproducing the ranking

`manifest.csv` carries every input to the score, so the ranking can be recomputed from the
published data alone:

```python
import csv
rows = list(csv.DictReader(open('manifest.csv')))
rows.sort(key=lambda r: -float(r['quality_score']))
```

## What the excluded runs were

Nothing was excluded for being *unflattering*. The rejected majority are short, ground-truth-
less, or single-tag captures. The full 67-capture archive is retained by the authors and can
be made available on request for anyone who wants to re-do the curation differently.
