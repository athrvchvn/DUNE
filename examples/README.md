# Examples

Small, runnable, and honest about what they show. Most people run these before reading
anything, which is why they exist.

| Example | Needs | Shows |
|---|---|---|
| [`five_anchor.py`](five_anchor.py) | nothing | The core solve. How range noise and anchor geometry each turn into position error. |
| [`three_anchor.py`](three_anchor.py) | nothing | Degenerate geometry, and the refusal that stops DUNE returning a mirrored position. |
| [`replay_log.py`](replay_log.py) | a log | The full offline path: parse → correct → weight → solve. |
| [`replay_log.m`](replay_log.m) | a log, MATLAB | The same program in MATLAB, for comparing the two pipelines. |
| [`live_tracking.py`](live_tracking.py) | hardware | A live position in ~40 lines, no GUI. |

## Run them

```bash
pip install -e "python/[all]"

python examples/five_anchor.py       # start here — no hardware, no data
python examples/three_anchor.py
python examples/replay_log.py        # explains itself if no log is present
python examples/live_tracking.py --serial /dev/ttyUSB0
```

MATLAB:

```matlab
cd matlab
dune_setup
addpath(fullfile(dune.rootDir(), 'examples'))
replay_log("path/to/serial_raw.log")
```

The two `replay_log` implementations are deliberately the same program twice. Run both on
one log; the numbers should match digit for digit. That is the two-pipeline claim, checkable
in thirty seconds.

## What you will notice

**`five_anchor.py`** — anchor placement buys more than range precision does. At 10 m outside
the constellation, 30 mm ranges give roughly what 100 mm ranges give in the middle.

**`three_anchor.py`** — the collinearity guard fires on hopeless geometry, but it is a
threshold: just above it there is a band that solves and is metres wrong. The guard is not a
substitute for placing anchors well.

**`replay_log.py`** — residual RMSE measures whether the ranges agree *with each other*, not
distance from truth. A high residual almost always means the anchors file does not match the
constellation the log came from.

## Notes

- `examples/logs/` ⬜ is not populated yet; the replay examples say so and exit cleanly. See
  [docs/datasets.md](../docs/datasets.md).
- Every example works from a fresh clone without configuration. The two that need data
  degrade to an explanation rather than a traceback.
- These are teaching code. The real host is `python/run_localization.py` — it adds the GUI,
  the job system, multi-tag state, capture and reconnection.
