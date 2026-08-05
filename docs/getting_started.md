# Getting started — bare boards to a first fix

Budget about two hours the first time, most of it calibration. The order below matters:
each step's output is the next step's input.

```
1. flash        → boards answer on serial
2. geometry     → config/anchors.json
3. Tier-1       → per-board antenna delay
4. first fix    → a moving dot
5. Tier-2       → 94 mm instead of 199 mm
```

---

## 0. What you need

Minimum: **four anchors + one tag**, five boards. The reported deployment uses five anchors
and two tags. Full bill of materials in [hardware.md](hardware.md).

> ⚠️ Confirm your boards carry the **DW1000**. Makerfabs sells a near-identically named
> **DW3000** board that this firmware does not support. Every board in one network must be
> the same chip.

Host software:

```bash
git clone https://github.com/AdershM-2/DUNE
cd DUNE
pip install -e "python/[all]"
```

Try it before touching hardware — this needs no radios at all:

```bash
python examples/five_anchor.py
python examples/replay_log.py
```

---

## 1. Flash the boards

Arduino IDE with the **ESP32 board package** installed. Then make the library visible:

```bash
# Linux / macOS
ln -s "$PWD/firmware/libraries/UwbRtls" ~/Arduino/libraries/UwbRtls
```

or copy `firmware/libraries/UwbRtls` into your sketchbook `libraries/` folder. Restart the
IDE; the examples appear under **File ▸ Examples ▸ UwbRtls**.

Board: **ESP32 Dev Module** (or the Makerfabs profile). The DW1000 driver calls
`SPI.begin()` on the ESP32 default VSPI pins, which match this board — **do not** call
`SPI.begin()` in your sketch.

### Per-board settings

Each sketch has a `PER-BOARD CONFIGURATION` block near the top. This is the only part you
edit.

**Anchors** — `firmware/sketches/Anchor/Anchor.ino`:

```cpp
#define ANCHOR_ID      0x01     // unique per board: 0x01, 0x02, 0x03, ...
#define ANTENNA_DELAY  16384    // placeholder until step 3
```

**Tag** — `firmware/sketches/Tag/Tag.ino`:

```cpp
#define TAG_ID         240
#define ANTENNA_DELAY  16384
static const uint8_t ANCHORS[] = {0x01, 0x02, 0x03, 0x04, 0x05};

// Transport: serial is simplest to start with.
static const char*   WIFI_SSID = "your-wifi-ssid";
static const char*   WIFI_PASS = "";
static IPAddress     HOST_IP(255, 255, 255, 255);   // broadcast, or your PC's IP
```

Start on **serial** (`UWB_HOSTLINK_SERIAL`). It removes WiFi from the list of things that
can be wrong. Move to UDP once you have a fix.

### First light

Open Serial Monitor at **115200**. You want a line containing `DECA`:

```
DW1000 device id: DECA0130
```

No `DECA` means SPI is not talking — wrong board profile, wrong port, or a hardware fault.
Nothing downstream will work until this line appears.

Anchors then print nothing until a tag polls them. The tag prints one `RTLS,…` line per
sweep once anchors respond.

---

## 2. Anchor geometry

Every position DUNE computes is relative to the anchor coordinates in
`config/anchors.json`. Wrong coordinates produce confidently wrong positions.

```bash
cp config/anchors.example.json config/anchors.json
```

```json
{
  "dim": 2,
  "bounds": [-0.5, 5.5, -0.5, 3.5, 0.0, 3.0],
  "anchors": [
    {"id": 1, "x": 0.00, "y": 0.00, "z": 0.48},
    {"id": 2, "x": 5.00, "y": 0.00, "z": 0.48},
    {"id": 3, "x": 5.00, "y": 3.00, "z": 0.48},
    {"id": 4, "x": 0.00, "y": 3.00, "z": 0.48}
  ],
  "layout": "my_lab_4anchor"
}
```

`dim: 2` solves in the plane (z is still used to predict range); `dim: 3` solves full XYZ.
`bounds` sets plot limits only. `id` must match each board's `ANCHOR_ID`.

Three ways to fill it in, in increasing order of convenience:

1. **Tape measure.** Fine to start. Your position accuracy cannot beat your geometry
   accuracy, so measure carefully.
2. **Anchor self-survey** — the anchors range to each other and the geometry is
   reconstructed. No camera, no tape. See [anchor_survey.md](anchor_survey.md). Recovers
   geometry to 49 mm raw, 42 mm in shape.
3. **Overhead camera** — if you are also setting up ground truth.
   See [vision_ground_truth.md](vision_ground_truth.md).

**Geometry advice that matters more than it sounds:** spread the anchors out, and do not put
them in a line. A near-collinear constellation is mirror-ambiguous, and DUNE will *refuse*
to solve rather than return a plausible-looking reflected position. Different heights help
if you want Z.

---

## 3. Tier-1 calibration

Do not skip this. Uncalibrated, per-link errors span 43–514 mm — the system will look
broken, because it is.

Full procedure in [calibration.md](calibration.md). The short version: put each board at a
**measured** distance from a reference board and run the `HWCALIB` binary search from the
tag's serial link or from the GUI's CAL panel. Expect convergence to a few millimetres.

Record the resulting `ANTENNA_DELAY` in that board's sketch so a reflash does not lose it.

---

## 4. First fix

```bash
python python/run_localization.py --serial /dev/ttyUSB0
```

The browser GUI opens automatically (`--no-browser` to suppress). You should see the anchor
constellation, and a tag marker that moves when you walk.

Check the per-anchor diagnostics panel: every anchor should be responding, with a small
residual. An anchor that never appears is unreachable — check its ID and that it is in the
tag's `ANCHORS[]` list.

MATLAB instead:

```matlab
cd matlab
dune_setup
live_tag("/dev/ttyUSB0")     % or "COM7" on Windows
```

**Expect ~199 mm RMSE at this point.** That is the correct number for Tier-1 only.

---

## 5. Tier-2 calibration

This is the step that takes 199 mm to 94 mm — the largest single improvement in the
pipeline. It needs truth at a spread of positions; see
[calibration.md](calibration.md#tier-2--first-path-power-correction).

---

## 6. Dual tags (optional)

A second tag buys you **heading**, which a single tag cannot observe. It needs a wired
link between the two tags — not just a second tag switched on, which starves one of the
pair through a capture effect. See [dual_tag_link.md](dual_tag_link.md).

---

## When it does not work

[troubleshooting.md](troubleshooting.md) covers the common failures. The three that catch
almost everyone:

1. **No `DECA` on boot** — SPI. Nothing else matters until this is fixed.
2. **Positions off by a metre or more, consistently** — anchor geometry or Tier-1.
3. **Changed something in `UwbConfig.h`** — every board needs recalibrating. Antenna delay
   depends on PRF and data rate.
