# Python or MATLAB?

DUNE ships two host pipelines. They are not a port and a fallback — both are complete, both
run live, both replay the same logs, and both read the same `config/`. This page says which
to pick.

## Short answer

**Use Python** unless you need MATLAB. It installs with `pip`, needs no licence, has the
browser operator GUI and the guided calibration flows, and is what you want on the robot.

**Use MATLAB** if you are doing analysis, want the figures, already live in MATLAB, or want
a second independent implementation to check a result against.

## Side by side

| | Python | MATLAB |
|---|---|---|
| Licence | none needed | MATLAB licence |
| Install | `pip install -e "python/[all]"` | `dune_setup` |
| Live single tag | ✅ `run_localization.py` | ✅ `live_tag` |
| Live dual-tag rigid MHE | ✅ | ✅ `live_rover` |
| Browser operator GUI | ✅ | — |
| Guided calibration flows | ✅ (in the GUI) | ✅ (scripts) |
| Anchor self-survey | ✅ capture + reconstruct | ✅ capture + reconstruct |
| Offline replay | ✅ | ✅ `replay_rtls` |
| Rover run scoring vs truth | ✅ | ✅ `rover_run_report` |
| Antenna-delay tuning tools | partial | ✅ the fuller set |
| Overhead-camera ground truth | ✅ | ✅ (registration lives here) |
| Rover control / capture | ✅ | ✅ `rover_teleop_uwb` |
| Automated tests | ✅ 253 | — |
| CasADi needed for MHE | pip wheel | MEX build |

Where the table says "partial", the other pipeline is the fuller one and the docs point you
there rather than pretending parity.

## Which produced the published numbers

Both, deliberately. The reported figures come from logged sessions replayed offline, and
the two implementations are cross-checked against each other on those same logs:

- `python/tests/test_equivalence.py` — every recorded wire line must parse, and corrected
  ranges, NLOS gaps and weights must match what MATLAB wrote for the same sweeps.
- `python/tests/test_pipeline_equivalence.py` — full solve path, sweep by sweep.
- `python/tests/test_mhe.py` — the two rigid MHE implementations compared at metric level
  on eight recorded rover sessions.

That cross-check is the point of having two. A result that survives two independent
implementations of the same equations is worth more than a result from one.

They agree at *metric* level, not bit level, for the MHE: MATLAB uses the CasADi MEX build
and Python the pip wheel, and two IPOPT instances need not take the same path to the same
optimum. Chasing bit-equality across two nonlinear solvers is a way to lose a week.

### Check it yourself

The two `replay_log` examples are deliberately the same program in two languages. Point
both at one log:

```bash
python examples/replay_log.py path/to/serial_raw.log
```
```matlab
replay_log("path/to/serial_raw.log")
```

They report solved-sweep counts, residual percentiles and position bounds that agree to the
printed precision — on the log used during development, both give 484 solved sweeps and a
761.2 mm median residual, digit for digit. If you change one pipeline and this diverges,
you have changed behaviour, not just implementation.

## They interoperate

The same log replays in either. The same `config/anchors.json` drives both. A capture from
`rover_teleop_uwb` (MATLAB) is scored by `reports.rover` (Python) and vice versa — that
round trip is what `python/tests/test_capture.py` exists to protect.

So a normal working pattern is: run Python live on the robot, then open the resulting `.mat`
in MATLAB for analysis. Nothing needs converting.

## Getting started with each

- [python_pipeline.md](python_pipeline.md)
- [matlab_pipeline.md](matlab_pipeline.md)
