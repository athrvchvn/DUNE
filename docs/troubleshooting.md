# Troubleshooting

Ordered by how often each one is the answer.

---

## No `DECA` on boot

```
DW1000 device id: DECA0130      ← what you want, at 115200 baud
```

Nothing downstream works until this appears. SPI is not talking to the radio.

- Wrong board profile — select **ESP32 Dev Module** or the Makerfabs profile.
- Wrong serial port.
- **You called `SPI.begin()` in your sketch.** The driver does it on the ESP32 default VSPI
  pins. Remove yours.
- The board is a **DW3000**, not a DW1000. Unsupported; check the module marking.

---

## Positions are off by a metre or more, consistently

Almost always geometry or Tier-1, in that order.

1. **Does `config/anchors.json` describe the constellation you are standing in?** Check the
   ids match each board's `ANCHOR_ID` and that x/y are in metres. This is the most common
   single cause.
2. **Is Tier-1 done on every board?** Uncalibrated per-link errors span 43–514 mm. See
   [calibration.md](calibration.md).
3. Are you passing the right anchors file? The repository's default `anchors.json` is a
   4-anchor layout; the 5-anchor deployment is `anchors_kinect_clicked_20260724.json`.

---

## Everything got worse after I changed the radio config

Expected. **Antenna delay depends on PRF and data rate.** Any change to `UwbConfig.h` —
radio mode, PRF, preamble — invalidates every board's calibration. The old numbers do not
degrade gracefully; they are silently wrong.

Recalibrate every board. There is no shortcut.

---

## Static RMSE is ~200 mm

That is the correct number for **Tier-1 only**. Tier-2 takes it to 94 mm and is the largest
single improvement in the pipeline — see
[calibration.md](calibration.md#tier-2--first-path-power-correction).

Check `config/range_correction.json` exists and that the anchor ids in it match yours.

---

## A stationary tag wanders by a few centimetres

Physical, not algorithmic, and not fixable in software.

Impulse-response captures show the direct path ≈20 dB below the strongest reflection and
only 8–10 dB above noise. With antennas at height *h* and separation *d*, the ground
reflection travels √(d² + 4h²) — 10–20 cm longer than the direct path at h = 0.48 m, well
inside the ~60 cm the channel resolves. The arrivals merge and the leading-edge estimate
wanders.

**Raise the antennas.** Filtering reaches 60–65 mm at rest and cannot do better, because a
static correction cannot track a signal that moves while the tag stands still.

---

## An anchor never responds

- Its `ANCHOR_ID` is not in the tag's `ANCHORS[]` list.
- Duplicate `ANCHOR_ID` on two boards — they will interfere confusingly.
- It is out of range or occluded. Watch the `[SCHED]` serial prints: after two consecutive
  failures the scheduler skips it for 5 sweeps, doubling to a cap of 40, resetting on
  success. Regular skip/recover messages mean a marginal link, not a broken one.

The solver degrades rather than fails when an anchor drops — but a *changing* responding
subset shifts the fix even when every range is good, which is what range hold exists to
smooth. See [estimators.md](estimators.md).

---

## The solver refuses to produce a position

This is deliberate. A singular-value test on the centred anchor positions detects a
near-collinear responding subset, which is **mirror-ambiguous** — two reflected solutions fit
equally well. Returning one of them at random is worse than returning nothing.

Spread your anchors out, or work out why only a collinear subset is responding.

---

## A board shows `CHARGING` and does nothing

GPIO 13 and 14 are shorted — that is charge mode. Remove the jumper; it returns to normal
within ~3 s. See [hardware.md](hardware.md#charge-mode).

---

## Dual-tag: one tag starves

If the two tags are not wired together, this is exactly the capture effect the wired link
exists to prevent. Two tags cannot simply be switched on. See
[dual_tag_link.md](dual_tag_link.md).

If they *are* wired:

- **`crc_err` climbing** → the cable. Nearly always the cable. Check the common ground.
- **Slave never joins** → cross-wiring (TX→RX both ways), or a build-id mismatch, which the
  master warns about.
- **Frequent slot timeouts** → watch the adaptive budget converge; it should settle at
  ≈1.5× the slave's sweep time.

Expected sweep split is 48–52%.

---

## Heading is wrong or drifts

- **Single tag?** Heading is geometrically unobservable from one tag. A whole circle of
  poses fits the ranges equally. You need two.
- **`baseline` set to your actual rig?** It defaults to 0.50 m. The entire rigid constraint
  is built on this number; a wrong one biases every pose. Measure antenna centre to antenna
  centre.
- There is no magnetometer and no absolute yaw reference anywhere in the filter — heading is
  observed by the radios alone, with gyro rate fused as a *differential* quantity. Expect
  ≈3.9°; the geometric budget for a single pair of fixes is ≈4.4°.

---

## MHE unavailable / CasADi errors

The MHE needs CasADi. Without it the EKF path still runs.

```bash
pip install casadi
```

```matlab
dune.addCasadi_available     % false means MATLAB cannot find it
```

MATLAB needs the MEX build on its path — download CasADi for MATLAB and `addpath` the
folder.

---

## Tests fail on a fresh clone

Expected result is **209 passed, 44 skipped**. The 44 skips are data-dependent; a fresh
clone ships no datasets. See [datasets.md](datasets.md).

If tests *fail* rather than skip:

- Missing optional dependency — `pip install -e "python/[all]"`.
- If they passed once and now fail, something wrote into your checkout. `git status` should
  be clean after a test run; `tests/conftest.py` exists to guarantee that.

---

## Still stuck

Open an issue with: board type and count, `UwbConfig.h` radio settings, your
`config/anchors.json`, a handful of raw `RTLS,` lines, and what you expected versus what you
saw. Raw wire lines are the single most useful thing you can attach.
