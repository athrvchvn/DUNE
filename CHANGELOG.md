# Changelog

Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versioning is
[semantic](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Pending before v1.0.0 is tagged

- ⬜ Datasets — rover runs, static campaign, CIR captures, self-survey captures — curated
  and published with a DOI (`docs/datasets.md`)
- ⬜ CAD, datasheets and board photos (`hardware/`)
- ⬜ Zenodo DOI in `CITATION.cff`, `README.md` and the paper

## [1.0.0] — unreleased

First public release: the reference implementation behind *DUNE: A Dual-Tag UWB Navigation
Engine for Indoor Ground Robots*.

The code is not new — it is the system the paper's results were measured on. What is new is
that it is arranged for someone else to use.

### The stack

- **Firmware** — single-source Arduino library over a vendored DW1000 driver. Asymmetric
  DS-TWR, four-byte application framing (so anchor count is bounded by the address space,
  not by 802.15.4 discovery), round-robin scheduler with per-anchor exponential backoff,
  jumper-detected deep-sleep charge mode. Five board roles.
- **Dual-tag synchronization** — wired master/slave TDMA over UART (`TagLink`), CRC-framed,
  cycle-numbered, with an adaptive slot budget. At most one radio is ever un-idled, so
  collision is structurally impossible rather than merely unlikely.
- **Two-tier calibration** — Tier-1 antenna delay by closed-loop binary search on the
  *production sweep path*; Tier-2 per-anchor correction linear in first-path power, fitted
  leave-one-position-out. Interlocked so a bias cannot be corrected twice.
- **Camera-free anchor self-survey** — pairwise inter-anchor ranging, classical MDS, and a
  reflection-aware rigid alignment performed *without scale*.
- **Same-frame NLOS weighting** — both power readings taken from one received frame, which
  is what makes the score a property of a single path. Continuous weighting, floored rather
  than zeroed.
- **Robust solving** — weighted Levenberg–Marquardt with three independent guards: a MAD
  outlier gate, a collinearity refusal, and an age-decayed range hold.
- **Rigid-body MHE** — one body pose per window node with both antenna positions derived
  from it, so the inter-tag baseline is exact by construction and heading falls out.
- **Two host pipelines** — Python and MATLAB, over the same firmware, wire format and
  `config/`, cross-checked against each other on recorded sessions.
- **Operator GUI** — single self-contained HTML file over WebSocket. No build step.

### Reported performance

94 mm static RMSE (199 mm with Tier-1 alone); 60–65 mm filtered at rest; 126 mm dynamic for
the rigid MHE against 171 mm for per-tag EKF on identical logged sweeps, with path-length
inflation reduced from 1.9× to 1.3×. Heading 3.9°. Anchor self-survey 49 mm raw / 42 mm in
shape. All against an overhead-camera reference registered to 13.5 mm RMSE.

### Changed for this release

- Repository restructured into `firmware/`, `python/`, `matlab/`, `config/`, `docs/`,
  `examples/`, `hardware/`, `data/`.
- `config/` promoted to the repository root: both pipelines read the same deployment
  parameters. Configuration describes the deployment, not one implementation of it.
- Python package renamed `rtls` → `dune`, matching the MATLAB `+dune` namespace, and made
  installable (`pip install -e python/`).
- `pyproject.toml` is the single source of dependency truth; `requirements.txt` is generated
  from it.
- Firmware sketches carry placeholder network credentials rather than a deployment's own.
- Full documentation set, runnable examples, and CI.

### Fixed

- **The test suite dirtied its own checkout.** Synthetic captures were written into
  `matlab/results/`, where the next run's `skipif` guards mistook them for the real rover
  corpus. The suite passed once on a fresh clone and failed every time after — invisible to
  CI, which always starts clean. Output now goes to a scratch directory removed per test,
  and CI runs the suite twice and then asserts the working tree is clean.
- **`run_localization.py` could not import**, and the whole suite still passed, because
  nothing imports the entry point. Added a test that starts every entry point and
  distinguishes a missing third-party package from an import left pointing at a module's
  old location.
- Configuration was located by walking up from the *output* directory
  (`RESULTS_DIR.parent / 'config'`) in three places. Now asks for `CONFIG_DIR`.
- Two tests asserted on a corpus that is empty in a data-free checkout; they now skip,
  matching the idiom the other 44 data-dependent tests use.
- Cleared the standing lint debt so CI starts green.

### Known limitations

Stated because they bound what the system can do, not because they are bugs:

- A stationary tag wanders a few centimetres. The cause is physical — the direct path sits
  ≈20 dB below the strongest reflection and the ground reflection merges with it at 0.48 m
  antenna height. **Raise the antennas**; no filter substitutes for it.
- A static, position-dependent correction is bounded at 72 mm RMSE by an oracle test, so a
  finer spatial survey is not the lever it appears to be.
- The NLOS weighting is reported as implemented and physically well-founded. An ablation
  isolating its accuracy contribution is future work.
- Ground truth runs at ≈1.25 Hz against ~4 Hz sweeps, and the platform's sub-0.1 m/s speed is
  unfavourable for moving-accuracy metrics.
- Arduino compilation is not in CI.
