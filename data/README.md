# Datasets

⬜ **This directory is empty on a fresh clone.** The recorded data behind the published
results is distributed separately and unpacks here.

See [docs/datasets.md](../docs/datasets.md) for what each dataset contains, how it was
recorded, and where to get it.

## Expected layout once unpacked

```
data/
├── rover_runs/         *.mat + *.ndjson  — 28 teleoperated driven sessions
├── static_campaign/    20 surveyed positions, 2471 sweeps, camera truth
├── cir/                raw DW1000 accumulator readouts
└── survey/             raw inter-anchor pairwise ranging + reconstructions
```

Contents are gitignored (`data/*`), so nothing large can be committed here by accident.

## What depends on it

- **44 tests skip** without it and run with it. That is expected on a fresh clone; see
  `python/tests/` and the skip reasons.
- `examples/replay_log.py` uses a small trimmed log from `examples/logs/` instead, so the
  examples work without the full drop.
- The paper's figures and tables are reproducible only with it.
