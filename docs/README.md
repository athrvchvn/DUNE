# DUNE documentation

## Start here

| | |
|---|---|
| [getting_started.md](getting_started.md) | Bare boards → first fix. Start here if you have hardware. |
| [architecture.md](architecture.md) | What owns what, and where a change belongs. Start here if you have the code. |
| [choosing_a_pipeline.md](choosing_a_pipeline.md) | Python or MATLAB? |

## Deploying

| | |
|---|---|
| [hardware.md](hardware.md) | BOM, pin map, board roles, radio config, charge mode, antenna placement |
| [calibration.md](calibration.md) | Tier-1 antenna delay, Tier-2 power correction, the interlock |
| [anchor_survey.md](anchor_survey.md) | Recovering anchor geometry without a tape measure |
| [dual_tag_link.md](dual_tag_link.md) | The wired link that makes two tags possible |
| [vision_ground_truth.md](vision_ground_truth.md) | Optional overhead-camera reference rig |
| [troubleshooting.md](troubleshooting.md) | When it does not work |

## Reference

| | |
|---|---|
| [protocol.md](protocol.md) | Over-air frames, the `RTLS,…` wire format, survey frames |
| [estimators.md](estimators.md) | NLOS weighting, multilateration, guards, EKF vs rigid MHE |
| [python_pipeline.md](python_pipeline.md) | Install, run, jobs, replay, module map |
| [matlab_pipeline.md](matlab_pipeline.md) | `dune_setup`, live scripts, offline analysis |
| [compatibility.md](compatibility.md) | Verified versions, and what is merely expected |
| [datasets.md](datasets.md) | What the recorded campaigns contain ⬜ *pending* |
| [release_checklist.md](release_checklist.md) | Gate before publishing a release |
| [design/](design/) | The long-form mathematical design documents |

## The one thing to know

> **The tag streams raw ranges. All position solving happens on the host.**

The firmware never computes a position. Anchor geometry, corrections and the estimator are
host-side configuration — so moving an anchor or swapping estimators needs no reflash, and
every recorded session replays through the identical solve path used live.

## Reading order for a new contributor

1. [architecture.md](architecture.md) — the map
2. [protocol.md](protocol.md) — what actually crosses the wire
3. [estimators.md](estimators.md) — the maths
4. [python_pipeline.md](python_pipeline.md) or [matlab_pipeline.md](matlab_pipeline.md)
5. `examples/` — runnable, small, and honest about what they show
