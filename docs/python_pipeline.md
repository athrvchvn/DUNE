# The Python pipeline

The default host. Installs with pip, needs no licence, carries the browser operator GUI.

## Install

```bash
pip install -e "python/[all]"      # library + MHE + reports + dev tools
pip install -e python/             # core only: parse, solve, filter, serve
pip install -e "python/[vision]"   # + overhead-camera ground truth
```

Editable (`-e`) is recommended even if you are not developing: the library resolves
`config/` relative to the repository, so an editable install keeps configuration and code
together.

Verify:

```bash
python -c "import dune; print(dune.__file__)"
python -m pytest python/tests -q       # expect 209 passed, 44 skipped on a fresh clone
```

The 44 skips are data-dependent tests. A fresh clone ships no datasets; they run once the
dataset drop is unpacked into `data/`.

## Running live

```bash
python python/run_localization.py                      # UDP :4100, opens the GUI
python python/run_localization.py --serial /dev/ttyUSB0
python python/run_localization.py --no-browser         # headless
python python/run_localization.py --baseline 0.527     # dual-tag rig
python python/run_localization.py --vision             # + overhead camera panel
```

Every tag that sends a packet gets its own pipeline state automatically. No tag ids are
hard-coded and none need pre-registering — the system scales to N tags by discovery.

## The operator GUI

A single self-contained `python/gui/gui.html`, served over WebSocket. No build step, no
framework, no npm.

| Panel | For |
|---|---|
| Plan view | anchors, tags, trails, covariance |
| Tuning | live estimator parameters |
| NLOS diagnostics | per-anchor gap, weight, residual, used/unused |
| CAL | guided calibration flows |
| LAB | run any offline tool and read its report in the browser |
| RUN | start/stop a synchronized rover capture, auto-scored |

**LAB and CAL are the job system.** A job is a named function with a declared argument
schema; the GUI renders the form from that schema, so adding a job adds a UI. Jobs run on
worker threads, report progress, and can be cancelled.

```python
from host.jobs import job

@job('my_analysis', 'Analyse something', mode='lab', args_schema={
    'log':  {'t': 'path', 'kind': 'logs', 'required': True},
    'tagZ': {'t': 'number', 'default': 0.24, 'min': 0.0, 'max': 3.0},
})
def my_analysis(app, ctx, log: str, tagZ: float = 0.24) -> dict:
    p = resolve_under_repo(log)          # always: paths from the browser are untrusted
    ctx.progress(0.5, msg="working")
    return {'answer': 42}
```

Rules that keep the job system safe and testable: **file paths in, JSON out, no live
handles.** The `app` argument resolves paths; it is not a way to reach into the running
pipeline. And every path from the browser goes through `resolve_under_repo` — the server
binds `0.0.0.0` and job arguments are attacker-controlled in any shared-network deployment.

## Offline replay

The same solve path as live, which is what makes estimator comparisons honest.

```bash
# From the GUI: LAB → "Replay a session log"
```

```python
from reports.replay import replay_session
r = replay_session('logs/serial_raw_20260726.log', 'results/out',
                   anchors_file='config/anchors_kinect_clicked_20260724.json',
                   tag_z=0.24)
print(r['tags']['240']['rmse_median_mm'])
```

Rover captures scored against AprilTag truth:

```python
from reports.rover import rover_report
r = rover_report('data/rover_runs/rover_uwb_20260726_131929.mat', 'results/out',
                 anchors_file='config/anchors_kinect_clicked_20260724.json', tag_z=0.24)
```

## Operator tools

One task each, run from the repository root:

| Tool | Does |
|---|---|
| `tools/run_survey.py` | anchor self-survey capture → `config/anchors.json` |
| `tools/multipoint_survey.py` | joint anchor solve from known grid positions |
| `tools/collect_bias_data.py` | capture ranges at a known distance |
| `tools/collect_calibration_data.py` | Tier-2 campaign capture |
| `tools/analyze_calibration_data.py` | fit the Tier-2 coefficients |
| `tools/calibration_manager.py` | Tier-1 traceability records |
| `tools/hwcalib_trace.py` | trace an antenna-delay binary search |

All take `--help`.

## Using it as a library

```python
from dune.anchor_config import AnchorConfig
from dune.frame_parser import FrameParser
from dune.solve_sweep import solve_sweep
from dune.paths import config

cfg = AnchorConfig.from_json(str(config('anchors.json')))
pkt = FrameParser.parse("RTLS,v1,500,240,2,1,1000,-80,2,2000,-85")
fix = solve_sweep(pkt, cfg)
```

See [examples/](../examples/) for runnable versions.

## Module map

| Module | Role |
|---|---|
| `dune/frame_parser.py` | wire line → typed packet, all versions |
| `dune/anchor_config.py` | anchor geometry |
| `dune/range_correction.py` | Tier-2 correction |
| `dune/nlos.py` | gap → weight |
| `dune/multilaterate.py` | weighted LM |
| `dune/solve_sweep.py` | the whole per-sweep chain, with guards |
| `dune/range_hold.py` | absent-anchor hold |
| `dune/ekf.py` | per-tag constant-velocity EKF |
| `dune/mhe_estimator.py` | single-tag MHE |
| `dune/mhe_rigid.py` | dual-tag rigid-body MHE |
| `dune/survey.py` | MDS reconstruction, delay fit |
| `dune/session_log.py`, `raw_log.py` | log reading |
| `dune/metrics.py` | trajectory metrics, percentiles |
| `dune/paths.py` | where things live |

`host/` is the application (transports, bus, jobs, path policy); `reports/` is offline
analysis; `rover/` is control and capture; `vision/` is ground truth. The layering rule —
`dune/` never imports from `host/` — is in [architecture.md](architecture.md).

## Testing

```bash
python -m pytest python/tests -q
python -m pytest python/tests/test_mhe.py -q -k rigid
```

Two conventions before you add tests:

- **Data-dependent tests skip rather than fail.** Match the existing idiom.
- **Never write into the checkout.** `tests/conftest.py` redirects `RESULTS_DIR` to a
  scratch directory removed per test. Read the reason in that file — debris from one run
  silently changed the outcome of the next, and the suite passed once then failed forever.
