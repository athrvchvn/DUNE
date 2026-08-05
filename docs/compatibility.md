# Compatibility

What DUNE is **verified** on, versus what it is merely expected to work on. The distinction
is deliberate: a table that quietly lists untested versions as supported wastes more time
than it saves.

## Verified

Measured on the development machine at the time of the v1.0.0 release.

| Component | Version |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Python | 3.10.12 |
| NumPy | 2.2.6 |
| SciPy | 1.15.3 |
| CasADi (Python) | 3.7.2 |
| websockets | 16.0 |
| pyserial | 3.5 |
| matplotlib | 3.10.9 |
| pandas | 2.3.3 |
| h5py | 3.16.0 |
| pytest | 6.2.5 |
| MATLAB | R2026a |
| CasADi (MATLAB MEX) | 3.7.0 |

Result on this configuration: **209 passed, 44 skipped** (the 44 are data-dependent — see
[datasets.md](datasets.md)), and `dune_setup` resolves `config/` and loads the anchor
geometry, Tier-2 coefficients and per-anchor bias.

## Expected, untested — reports welcome

| Component | Declared support | Status |
|---|---|---|
| Python | ≥3.9 (`pyproject.toml`) | only 3.10 exercised |
| NumPy | ≥1.21 | only 2.2 exercised |
| SciPy | ≥1.7 | only 1.15 exercised |
| CasADi | ≥3.6 | only 3.7 exercised |
| MATLAB | R2021b+ expected — the scripts use `arguments` blocks (R2019b) and string types throughout | only R2026a exercised |
| Windows 11 / macOS | should work; serial port names differ | untested for this release |

The lower bounds in `pyproject.toml` are *declared* support, chosen from the APIs the code
actually uses. They are not all separately exercised. If you run DUNE on something outside
the verified table, an issue saying so — working or not — is genuinely useful.

## MATLAB toolboxes

| Toolbox | Needed for |
|---|---|
| base MATLAB | the whole `dune` package, live scripts, replay |
| **Computer Vision Toolbox** | `matlab/vision/` only — camera calibration, AprilTag pose, world registration |

The localization pipeline needs no toolboxes. Only the optional ground-truth rig does.

CasADi is required only for the moving-horizon estimators; `dune.addCasadi_available`
reports whether it is usable, and everything else runs without it.

## Firmware toolchain

| Component | Notes |
|---|---|
| Arduino IDE | 2.x, or `arduino-cli` |
| ESP32 board package | Espressif, via Boards Manager |
| Board profile | ESP32 Dev Module, or the Makerfabs profile |
| Adafruit SSD1306 + GFX | only if you enable `UWB_USE_OLED` |

Arduino compilation is **not** in CI — `arduino-cli` plus the ESP32 core is slow and brittle
to set up on a runner, and a green Python CI that takes twenty minutes because of a
toolchain download is a CI nobody keeps. It is a known follow-up.

The vendored DW1000 driver is not a dependency you install; it ships in
`firmware/libraries/UwbRtls/src/dw1000/` under its own Apache-2.0 licence.

## Hardware

| | |
|---|---|
| Verified board | Makerfabs "ESP32 UWB Pro with Display" (DW1000 / BU01 with PA) |
| IMU | BNO085 |
| Ground-truth camera | Kinect v2 (optional) |

**Not supported:** the DW3000 boards, despite the near-identical product name. Porting is
confined to the driver layer by design — see [architecture.md](architecture.md) — but it has
not been done.

## Reporting a mismatch

Open an issue with your OS, Python/MATLAB version, `pip list` output, and the failure. If it
is a version-compatibility problem specifically, the fix is usually a lower bound in
`pyproject.toml` and a row in this table.
