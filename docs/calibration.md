# Calibration

Calibration is the single largest accuracy lever in DUNE. Uncalibrated, per-link errors span
43–514 mm. After Tier-1 they are 199 mm RMSE across the floor; after Tier-2, **94 mm**.

Two tiers, deliberately non-overlapping so a bias cannot be corrected twice.

| | What it fixes | Where it lives | How often |
|---|---|---|---|
| **Tier-1** | per-board antenna delay | the board's NVS / sketch | once per board, redo after any radio-mode change |
| **Tier-2** | residual power-dependent range error | `config/range_correction.json` | once per deployment |

---

## Why there is anything to calibrate

The DW1000 timestamps at an internal reference, not at the antenna phase centre. The
difference is a constant per-board delay of order **16 000 ticks** (~2.4 m of apparent
range) that differs between devices.

Two properties make it nasty:

- It is **constant**, so averaging does not remove it.
- **Both participants contribute**, so it cannot be removed by one global offset — the error
  on link A↔B is the sum of two per-board delays.

That is why Tier-1 is per board and lives in hardware.

---

## Tier-1 — antenna delay

A closed-loop binary search against a surveyed reference distance. The tag runs it; the
result is written to the board's NVS.

### Running it

Place the board at a **measured** distance from a calibrated reference board. Tape-measure
accuracy is the floor of everything downstream, so measure antenna-face to antenna-face and
write the number down.

```bash
# Over the tag's serial link
HWCALIB <target_id> <true_distance_mm>
```

or drive it from the operator GUI (CAL panel), which is the easier path — it shows the
search converging.

Convergence to **4.4 mm** at the reference point is typical. Results are appended to
`config/calibration.json` (`level1_hardware`) with the full iteration history, the git
commit, and the residual — traceability, not something the host applies.

### The part that is easy to get wrong

> **Calibrate on the production sweep path, not in a rapid burst.**

A tight measurement loop and the real round-robin sweep disagree by a constant per-anchor
offset. Calibrating on the burst path leaves the path the system actually uses biased. The
`HWCALIB` flow measures the way the deployed scheduler measures, which is the whole reason
it exists rather than a simple ranging loop.

### After a radio-mode change

Antenna delay depends on PRF and data rate. **Change anything in `UwbConfig.h` — radio mode,
PRF, preamble — and every board must be recalibrated.** There is no shortcut; the old
numbers are silently wrong rather than obviously wrong.

---

## Tier-2 — first-path-power correction

Tier-1 leaves a residual that is not constant: the DW1000's leading-edge detector reads
**long when the signal is strong**. So the residual correlates with first-path power, and is
correctable as a per-anchor line:

```
corr_mm       = slope · P_fp + intercept
corrected     = raw − corr_mm / 1000
```

with `P_fp` **clamped to the fitted validity window** before evaluating, so an out-of-range
power can never extrapolate the line into nonsense.

Coefficients live in `config/range_correction.json`:

```json
{
  "id": 1,
  "slope_mm_per_dB": 9.46,
  "intercept_mm": 839.9,
  "fp_valid_dBm": [-114.7, -74.7],
  "n_spots": 17
}
```

Applied by `python/dune/range_correction.py` and `matlab/+dune/loadRangeCorrection.m`.

### Fitting it

You need ground truth at a spread of positions — the shipped coefficients come from 19–20
surveyed spots with camera truth, 20 s dwell each. Then:

```bash
python python/tools/analyze_calibration_data.py   # see --help for the fit options
```

Coefficients are estimated **leave-one-position-out**, so the reported accuracy is
out-of-sample rather than a fit to its own training data. This is not pedantry: an in-sample
number here would be optimistic by a wide margin, and the whole point of the tier is a
correction that generalises to positions you did not stand on.

### Why not a spatial error map

A map keyed on position was implemented and then retired in favour of the power correction
above. The captured map is still in `config/error_map.json` and the loader hooks remain in
`run_localization.py`, but the module that evaluated it is no longer part of the tree — treat
the map as a recorded result, not a supported code path.

The campaign showed the residual is **time-varying**: slow multipath "breathing" of 5–7 cm
at a *fixed* position. An oracle that subtracts the mean residual measured at each position
— the best any time-invariant, position-dependent map could do — still leaves **72 mm RMSE**.
Temporal filtering reaches 60–65 mm, *below* that bound, because it averages out the
zero-mean high-frequency part that no static map can capture.

A function of measured power tracks a signal that moves while the tag stands still. A
function of position cannot. The practical consequence: **a finer spatial survey is not the
lever it appears to be.**

---

## The interlock

The two tiers must not both correct the same bias. Concretely:

- Tier-1 lives in the **firmware**, in the board's antenna delay. The host never applies it —
  `calibration.json` records it for traceability only.
- Tier-2 lives in the **host**, in `range_correction.json`. The firmware knows nothing of it.

An earlier design broke this: it pushed a per-anchor antenna delay derived from a measured
bias **and** subtracted that same bias in software — double-correcting — and the delay push
had an inverted sign. Both are gone. If you add a third correction, the question to answer
first is which of the two existing tiers it overlaps.

Similarly, if you load a spatial error map, the global `range_bias_m` is forced to zero and
the GUI's BIAS slider is disabled, for the same reason.

---

## Verifying a calibration

```bash
python python/tools/collect_bias_data.py --help   # capture at a known distance
```

or, offline, replay a session and look at the residual:

```bash
python python/run_localization.py     # GUI → LAB → "Replay a session log"
```

What to expect after both tiers, from the reported deployment:

| | |
|---|---|
| Antenna-delay convergence at the reference point | 4.4 mm |
| Static single-reading RMSE across 20 positions | 94 mm |
| Median | 69 mm |
| 95th percentile | 160 mm |
| Filtered, at rest | 60–65 mm |

If your static RMSE is near 200 mm, Tier-2 is not loaded or not fitted. If it is far worse
than 200 mm, suspect Tier-1, the anchor geometry, or an anchors.json that does not match the
constellation you are standing in.

---

## Where the remaining error comes from

Worth knowing, so you do not chase it in software. A stationary tag wanders by a few
centimetres, and the cause is physical: impulse-response captures show the direct path
**≈20 dB below the strongest reflection** and only 8–10 dB above noise. With both antennas at
height *h* and separation *d*, the ground reflection travels √(d² + 4h²) — exceeding the
direct path by 10–20 cm at h = 0.48 m. That is well inside the ~60 cm the channel resolves,
so the two arrivals merge and the leading-edge estimate wanders.

**The remedy is to raise the antennas, not to improve the filter.**

---

Next: [anchor_survey.md](anchor_survey.md) — getting the anchor coordinates without a tape
measure.
