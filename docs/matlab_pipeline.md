# The MATLAB pipeline

A complete second implementation over the same firmware, the same wire format and the same
`config/`. Use it for analysis, for the figures, or as an independent check on a result.

## Setup

```matlab
cd matlab
dune_setup
```

`dune_setup` adds `matlab/`, `matlab/scripts` and `matlab/vision` to the path, then reports
what it found — repository root, config directory, how many anchors loaded, whether CasADi
is available. Run it once per session, or add it to your `startup.m`.

```
DUNE MATLAB pipeline
  repo root : /path/to/DUNE
  config    : /path/to/DUNE/config
  anchors   : 5 loaded from anchors.json
  CasADi    : available (MHE enabled)
```

### CasADi

Needed only for the moving-horizon estimators. Without it, everything else — parsing,
calibration correction, NLOS weighting, multilateration, the EKF — still runs.

```matlab
dune.addCasadi                  % locates an installed CasADi
dune.addCasadi_available        % true/false
```

If it is not found, download CasADi for MATLAB and `addpath` the folder. The MEX build is
faster than the Python wheel; the two agree at metric level, not bit level (see
[choosing_a_pipeline.md](choosing_a_pipeline.md)).

## Live

```matlab
live_tag("/dev/ttyUSB0")           % single tag, serial
live_tag("COM7", bias=true)        % also subtract config/anchor_bias.json
live_tag                           % UDP

live_rover                         % dual-tag rigid MHE, both tags over UDP:4100
live_rover(transport="serial")     % both tags on one port via the master
live_rover(baseline=0.527)         % MEASURE YOUR RIG
```

`live_rover` feeds **every** sweep from either tag as its own node to `dune.MheRigid`,
estimating one rigid body pose `[cx cy psi speed]`. Inter-tag distance is exact, yaw is
fused from UWB geometry and gyro rate, roll/pitch come from the IMU and drive the terrain
geometry.

> **Display guard.** The markers move only on a fresh valid fix. If both tags go quiet for
> `coastLimit` seconds the display **freezes** and the title says COASTING. It never follows
> a dead-reckoned estimate away from reality — a previous version did, and a frozen display
> is far less dangerous than a confident wrong one.

Rover teleoperation with synchronized capture:

```matlab
rover_teleop_uwb                   % set COM_PORT / UWB_TAG_COM for your machine first
```

## Offline

```matlab
replay_rtls("logs/serial_raw_20260726.log")
mhe_replay("logs/serial_raw_20260726.log")        % EKF vs MHE A/B on identical sweeps
rover_run_report("data/rover_runs/rover_uwb_20260726_131929.mat")
rover_pose_report(...)
static_accuracy("COM12")                          % static campaign capture
static_verify(...)
```

`mhe_replay` is the one that produced the estimator comparison: it runs the identical
`solveSweep` chain, then feeds each sweep to **both** `dune.FusionEkf` and the MHE, and
compares path smoothness, lag proxy, static wander and per-sweep IPOPT solve time.

## Calibration and survey

```matlab
calibrate_delays("COM12")              % Tier-1, guided
calibrate_delays("COM12", dryRun=true) % show the plan, touch no hardware
tune_tag_delay("COM7")
tune_delays_sweep("COM12")
anchor_delay_tool("COM7", ids=[3 5])
restore_anchor_delays("COM12")         % roll back to a previous backup

survey_capture(transport="serial", com="COM12")
survey_reconstruct(...)
survey_fit(...)
build_survey_anchors(...)

cir_capture("COM12", anchor=1, n=30)   % channel impulse response
```

`dryRun` on `calibrate_delays` is worth using the first time. It prints exactly what it
would write, and antenna delays are the one thing here that changes hardware state.

## Vision / ground truth

```matlab
calibrateOverheadCamera(...)
registerWorldFrame(...)                % or registerWorldFrameClicks
clickAnchorsWorld                      % click anchors → config/anchors.json
clickTagTruth                          % click tag truth → config/tag_truth.json
```

Needs the Computer Vision Toolbox. Full procedure in
[vision_ground_truth.md](vision_ground_truth.md).

## The `dune` package

| Function | Role |
|---|---|
| `dune.parseRtlsLine` | wire line → struct |
| `dune.loadAnchors` | anchor geometry from `config/` |
| `dune.loadRangeCorrection` | Tier-2 coefficients |
| `dune.loadAnchorBias` | per-anchor bias |
| `dune.gapWeights` | NLOS weighting |
| `dune.multilaterate` | weighted LM |
| `dune.solveSweep` | the per-sweep chain |
| `dune.RangeHold` | absent-anchor hold |
| `dune.FusionEkf` | EKF |
| `dune.MheEstimator` / `dune.MheRigid` | moving-horizon estimators |
| `dune.TagSerial` / `dune.TagUdp` | transports |
| `dune.LivePlotter` | live display |
| `dune.readSessionLog` / `readVisionLog` | log reading |
| `dune.alignTruth` / `trajectoryMetrics` | scoring |
| `dune.rootDir` / `configDir` / `dataDir` / `logsDir` / `resultsDir` | locations |

## Per-machine settings

Serial port defaults are marked `← PER MACHINE` in the scripts. Windows uses `COM8`; Linux
`/dev/ttyUSB0`; macOS `/dev/tty.usbserial-*`. They are `arguments`-block defaults, so pass
your own rather than editing the file:

```matlab
live_tag("/dev/ttyUSB0")
```

## Keeping the two pipelines in step

`matlab/+dune/` and `python/dune/` implement the same algorithms. When you change one,
change the other — several files say so in their header comments, and
`python/tests/test_equivalence.py` will fail if the numeric behaviour diverges on recorded
sessions.
