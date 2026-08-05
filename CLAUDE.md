# DUNE — project guide for agents

Open-source indoor localization for ground robots on Decawave DW1000 radios. Reference
implementation for the RA-P paper *DUNE: A Dual-Tag UWB Navigation Engine for Indoor Ground
Robots*.

## The rule that explains the layout

> **The tag streams raw ranges. All position solving happens on the host.**

Firmware never computes a position, applies a correction, or knows an anchor coordinate. So
adding/moving anchors or changing the estimator is a host-side change — no reflash. Read
[docs/architecture.md](docs/architecture.md) before making structural changes; it has a
"where to make a change" table.

## Layout

| | |
|---|---|
| `firmware/libraries/UwbRtls/src/` | shared firmware library, **single-source** — edit once |
| `firmware/libraries/UwbRtls/src/dw1000/` | vendored Decawave/thotro driver, Apache-2.0. Avoid gratuitous edits. |
| `firmware/sketches/` | the active flash targets, five board roles |
| `python/dune/` | the library — never imports from `host/` |
| `python/host/` | the running application: transports, bus, jobs, path policy |
| `python/reports/`, `rover/`, `vision/`, `tools/` | offline analysis, control, ground truth, CLI tools |
| `matlab/+dune/`, `matlab/scripts/` | the second pipeline |
| `config/` | deployment parameters, read by **both** pipelines |
| `logs/`, `results/`, `data/` | runtime output and datasets — all gitignored |

## Gotchas

1. **Dual sketch copies.** Every sketch exists in `firmware/sketches/` (active) *and*
   `firmware/libraries/UwbRtls/examples/` (IDE Examples menu). They have intentionally
   diverged. **Edit both** for sketch-level changes. Headers under `src/` are single-source.
2. **Two pipelines stay in step.** `python/dune/` and `matlab/+dune/` implement the same
   algorithms. Change one, change the other; `test_equivalence.py` fails if they diverge
   numerically on recorded sessions.
3. **Antenna delay is per-board and PRF-dependent.** Any `UwbConfig.h` radio change
   invalidates every board's calibration. Recalibrate all of them.
4. **GPIO 13 & 14 are the charge-mode jumper.** Do not reuse. 22/35 are the dual-tag UART.
5. **Wire format: append, never insert.** Update `HostLink.h`,
   `python/dune/frame_parser.py` and `matlab/+dune/parseRtlsLine.m` together. `RTLS,`
   (uppercase) is the protocol literal and is never renamed.
6. **Machine-specific values** (serial ports, WiFi SSID) are placeholders marked per-machine.
   Do not "fix" them to a particular machine's values.

## Testing

```bash
python -m pytest python/tests -q       # 209 passed, 44 skipped on a fresh clone
python -m pytest python/tests -q       # run twice — must be identical
git status --porcelain                 # must be empty afterwards
```

The 44 skips are data-dependent; a data-free clone is *supposed* to skip them.

Two hard rules:

- **A test must not write into the checkout.** `python/tests/conftest.py` redirects
  `RESULTS_DIR` per test. Debris once changed the *outcome* of the following run.
- **A test must fail when the behaviour breaks.** Break the code deliberately and confirm
  it fails. This repository has shipped tests that passed against the bug they were
  written for.

## Current state

Code-complete for v1.0.0. Outstanding before the repository is "released":

- ⬜ Datasets curated and published (`docs/datasets.md`)
- ⬜ CAD, datasheets, board photos (`hardware/`)
- ⬜ Zenodo DOI in `CITATION.cff` and `README.md`

Gate is [docs/release_checklist.md](docs/release_checklist.md).
