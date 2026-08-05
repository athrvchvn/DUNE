# Hardware

## Bill of materials

| Qty | Item | Notes |
|---|---|---|
| 4–7 | Makerfabs **ESP32 UWB Pro with Display** | ESP32 + Decawave **DW1000** (BU01 module with PA) + 1.3" SSD1306 OLED |
| 1 | BNO085 IMU | on the master tag; optional for a single-tag setup |
| — | USB cables, or LiPo batteries | serial or untethered |
| — | 2× jumper wires + common ground | only for a dual-tag rig |

The reported deployment uses **5 anchors + 2 tags**. The minimum useful configuration is
4 anchors + 1 tag; anchor count is bounded by the address space and sweep time, not by the
protocol.

> ⚠️ **DW1000, not DW3000.** Makerfabs sells a near-identically named DW3000 board that this
> firmware does not support. Every board in one network must be the same chip. Check before
> ordering.

## Pin map

Already encoded in `firmware/libraries/UwbRtls/src/UwbConfig.h` — listed here for wiring
checks and for anyone adding a peripheral.

| Function | Pin |
|---|---|
| SPI SCK | 18 |
| SPI MISO | 19 |
| SPI MOSI | 23 |
| DW1000 CS (SS) | 21 |
| DW1000 RST | 27 |
| DW1000 IRQ | 34 |
| OLED SDA | 4 |
| OLED SCL | 5 |

The DW1000 driver calls `SPI.begin()` on the ESP32 default VSPI pins (18/19/23), which match
this board. **Do not call `SPI.begin()` in your sketch.**

### Reserved pins — do not reuse

| Pins | Why |
|---|---|
| **13 & 14** | charge-mode jumper (below) |
| 16 & 17 | WROVER PSRAM |
| 12 | strapping pin |
| 22 & 35 | dual-tag UART link (if used) |

If you must reuse 13/14, override `PIN_CHARGE_SENSE` / `PIN_CHARGE_DRIVE` **before**
`#include <UwbRtls.h>`.

## Board roles

Five sketches under `firmware/sketches/`. Each has a `PER-BOARD CONFIGURATION` block at the
top — that block is the only part you edit.

| Sketch | Role |
|---|---|
| `Anchor/` | fixed anchor. Set a unique `ANCHOR_ID` and its calibrated `ANTENNA_DELAY`. |
| `AnchorWrover/` | same, for WROVER boards |
| `Tag/` | mobile tag. Set `TAG_ID`, `ANTENNA_DELAY`, the `ANCHORS[]` list, transport. Defaults to SOLO. |
| `TagWrover/` | the dual-tag rig. `TAG_LINK_MASTER` / `TAG_LINK_SLAVE` / `TAG_LINK_SOLO`. |
| `AntennaCalibration/` | bench utility for per-board delay tuning |

> Every sketch exists **twice**: in `firmware/sketches/` (the active flash targets) and in
> `firmware/libraries/UwbRtls/examples/` (the library's bundled reference copies, which
> appear in the Arduino IDE's Examples menu). They have intentionally diverged — the
> examples omit OLED and use demo constants. **For a sketch-level change, edit both.**
> Shared headers under `firmware/libraries/UwbRtls/src/` are single-source: edit once.

## Radio configuration

`UwbConfig.h`, single-source across every board.

| | |
|---|---|
| Channel | 5 |
| Data rate | 110 kbit/s |
| Pulse repetition frequency | 64 MHz |
| Preamble | 2048 symbols |
| Reply delay | 5000 µs |
| Timestamp quantum | 15.65 ps (4.69 mm) |
| Sweep rate | 3.9–4.25 Hz per tag |

> **Changing any of this invalidates every antenna delay.** Antenna delay depends on PRF and
> data rate. After a radio-mode change, recalibrate every board — see
> [calibration.md](calibration.md).

A faster mode (6.8 Mbps, 128-symbol preamble, 1500 µs reply delay, ~35–45 Hz) was tried and
**reverted**: it was incompatible with these boards in a way that was never root-caused. The
current configuration is the one that works.

## Charge mode

Shorting **GPIO 13 ↔ 14** puts a board into deep-sleep charge mode: DW1000 held in reset,
WiFi off, OLED shows `CHARGING`, ESP32 at ~10 µA. It wakes every 3 s to re-test the jumper,
so removing the short brings the board back within ~3 s. No reflash, no reboot button.

Detection is a debounced two-phase short test — drive low, read low; drive high, read high —
so a pin merely stuck at one level cannot trigger it. Neither pin is a strapping pin, so
flashing is unaffected.

Wired into all four roles in both `setup()` and `loop()`, so the jumper can be inserted while
running. `AntennaCalibration` is deliberately excluded (bench utility).

## Antenna placement — the thing that actually limits accuracy

From the reported deployment, anchors and tags at 0.48 m: impulse-response captures show the
direct path **≈20 dB below the strongest reflection** and only 8–10 dB above noise.

With both antennas at height *h* and separation *d*, the ground reflection travels
√(d² + 4h²), exceeding the direct path by 10–20 cm at h = 0.48 m — well inside the ~60 cm
the channel resolves. The two arrivals merge, and the leading-edge estimate wanders by a few
centimetres even with the tag standing still.

**Raise your antennas.** It is the single highest-value physical change available, and no
amount of filtering substitutes for it.

Other placement advice:

- Spread anchors out; avoid near-collinear layouts. The solver **refuses** to solve a
  mirror-ambiguous geometry rather than return a plausible reflected position.
- Keep anchors and tags roughly co-planar if you are solving in 2D — then the vertical term
  in the predicted range vanishes identically and the planar reduction is exact.
- Vary anchor heights if you want Z.

## Rover platform

The reported deployment uses a rocker–bogie rover:

| | |
|---|---|
| Type | rocker–bogie, 6 wheels, passive suspension |
| DOF | 10 (base) + 3 (arm) |
| Wheelbase | 0.6 m |
| Track width | 0.7 m |
| Wheel radius | 0.07 m |
| Max speed | 0.1 m/s |
| Tag baseline | 0.550 m |
| Tag / anchor height | 0.48 m |

Control and capture code is in `python/rover/` and `matlab/scripts/rover_teleop_uwb.m`.
DUNE does not require this platform — it is what the numbers were measured on.

## CAD, datasheets and photos

See [`hardware/`](../hardware/):

- [`hardware/cad/`](../hardware/cad/) — mounts, masts, rover
- [`hardware/datasheets/`](../hardware/datasheets/) — DW1000, ESP32, BNO085
- [`hardware/images/`](../hardware/images/) — board and deployment photos

⬜ **These are pending.** Each directory's README says exactly what belongs in it.
