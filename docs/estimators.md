# Solving and estimation

From a sweep of ranges to a pose. Four stages: weight, solve, guard, estimate.

---

## NLOS weighting

When the direct path is attenuated relative to its reflections, most received energy arrives
*after* the first detected arrival. So the gap

```
g = P_rx − P_fp
```

between total received power and first-path power is a per-measurement confidence score,
computable on every exchange at **no airtime cost**. It is the vendor-recommended NLOS
indicator, and DUNE uses it continuously rather than as a binary decision.

```
w_i = max(w_min, 10^(−max(0, g_i − g₀)/10))
        g₀ = 3 dB      w_min = 0.05
```

Unity below the threshold, so ordinary measurements are undisturbed. Exponential decay
beyond it. And a **floor rather than zero**, because dropping an anchor changes the solve
geometry and therefore introduces a discontinuity in the output — a down-weighted anchor
still contributes geometry.

`python/dune/nlos.py` · `matlab/+dune/gapWeights.m`

> ### The firmware change that makes this valid
> The obvious implementation compares the anchor's reported received power against the
> tag's own first-path reading. Those are measured **on different links, in different
> directions** — their difference is not a property of any single path, and the score is
> meaningless.
>
> DUNE reads both quantities from the **same received frame**, on the tag side, and streams
> the raw ingredients rather than a computed score. The policy stays host-side and
> replayable; the measurement is physically sound. See [protocol.md](protocol.md).

The weighting is reported as implemented and physically well-founded. An ablation isolating
its accuracy contribution is future work — it is not claimed as a measured improvement.

---

## Weighted multilateration

Each sweep is a weighted nonlinear least-squares problem

```
J(p) = Σᵢ wᵢ (‖p − aᵢ‖ − dᵢ)²
```

minimised by Levenberg–Marquardt, warm-started from the previous fix. Works for any
N ≥ dim+1 anchors.

The covariance returned to the estimator is the weighted GLS form, so a fix built from
down-weighted measurements honestly reports larger uncertainty rather than pretending
confidence it does not have.

`python/dune/multilaterate.py` · `matlab/+dune/multilaterate.m`

---

## Three guards, three failure modes

These are what let the solver **degrade** rather than fail when anchors drop out. They are
deliberately independent — each catches something the others cannot.

| Guard | Catches | Behaviour |
|---|---|---|
| **MAD gate** | gross outliers | drops residuals beyond k·MAD, with a 2 cm floor so a very consistent sweep does not gate its own good measurements |
| **Collinearity refusal** | mirror-ambiguous geometry | a singular-value test on the centred anchor positions; **refuses to solve** rather than return a reflected position |
| **Range hold** | a changing responding *subset* | injects an absent anchor's last range with an age-decayed weight |

The MAD gate and NLOS weighting are independent on purpose: the gate handles indefensible
residuals, the weighting handles soft NLOS that passes the gate. Either alone would mask
the other.

Range hold addresses something easy to miss: a change in *which* anchors respond shifts the
fix even when every contributing range is good, because the geometry of the solve changed.
Holding the missing one briefly, at decaying weight, keeps that from looking like motion.

`python/dune/solve_sweep.py` · `python/dune/range_hold.py`

---

## Estimators

### Per-tag EKF

Constant-velocity, dimension-parameterised, with a divergence guard and chi-squared gating.
One filter per tag. Fast (<1 ms), simple, and the right default for a single tag.

`python/dune/ekf.py` · `matlab/+dune/FusionEkf.m`

### Rigid-body MHE

For a two-tag rig, the per-tag EKF discards the strongest prior available: **the two
antennas are bolted to one body a known distance apart.**

A penalty term encouraging that separation is a soft suggestion with a weight to tune, and
the data can always override it. DUNE instead makes the constraint structural.

Estimate one planar body pose per window node, `x = [c_x, c_y, ψ, v]`, and *derive* both
antenna positions from it:

```
t_x = c_x + s·ℓ·cos ψ
t_y = c_y + s·ℓ·sin ψ          ℓ = (L/2)·cos θ,   s = ±1 (front/rear)
```

The tag positions are **never free variables**, so the inter-tag distance is exact by
construction — an estimate violating the rig's geometry is not representable in the state.
Heading falls out for free.

Terrain enters through inertial pitch θ, which foreshortens the baseline and offsets the two
tag heights by ±(L/2)·sin θ. Roll is deliberately excluded: the baseline lies along the axis
roll rotates about, so roll cannot move the tags. (`test_roll_is_deliberately_ignored`
asserts `push()` takes no roll argument, so this stays true.)

Because the two tags sweep in alternating slots, their measurements never coincide in time.
Rather than pairing nearest-in-time sweeps — which injects an artificial timing error —
**each incoming sweep becomes its own window node at its own timestamp**, and the rigid-body
model connects them.

```
min  Σₖ Σⱼ w_kj · ρ_δ( ‖t(xₖ,sₖ) − aⱼ‖ − d̂_kj )        pseudo-Huber on range residuals
  +  Σₖ ‖x_{k+1} − f(xₖ, ωₖ)‖²_Q⁻¹                      unicycle motion, gyro-driven heading
  +  ‖x₁ − x̄‖²_P⁻¹                                       arrival cost
  +  Σₖ σ_z⁻² · 1_still(k) · vₖ²                         zero-velocity when stationary
                                   s.t.  |vₖ| ≤ v_max
```

The speed box suppresses the unphysical velocity spikes an unconstrained optimiser uses to
explain a single noisy sweep. An optional forward-only bound `0 ≤ v` removes the sign
ambiguity by which (v, ψ) and (−v, ψ+180°) describe the same velocity.

Solved with CasADi/IPOPT over a **12-node horizon** (≈2 s at the deployed sweep rate). A
divergence guard rejects and reseeds when the solver fails or the body centre departs from a
constant-velocity prediction by more than 0.20 m.

The IMU contributes only **differential** quantities — yaw rate, and gravity-referenced roll
and pitch — never an absolute heading. There is no magnetometer and no yaw reference in the
filter: **heading is observed by the radios alone.**

`python/dune/mhe_rigid.py` · `matlab/+dune/MheRigid.m`

---

## Which to use

| | Per-tag EKF | Rigid MHE |
|---|---|---|
| Tags needed | 1 or more | 2, rigidly mounted |
| Heading | not observable | ✅ |
| Position RMSE (same data) | 171 mm | **126 mm** |
| Path-length inflation | 1.9× | **1.3×** |
| Cost per update | <1 ms | 3 ms median, 30 ms p95 |
| Needs CasADi | no | yes |

Both sit far inside the 235–255 ms sweep interval, so cost is not a real-time constraint at
this rate.

The 26% accuracy gain is real but the path-length column is the more informative one. A
ratio of 1.9 means the filtered trajectory is nearly twice as long as the path actually
driven: the estimate is not lagging or diverging but **wandering** along the route. Each
per-tag filter observes only its own tag and has no representation of the other antenna, so
per-tag measurement noise passes straight into the midpoint. The rigid estimator cannot
produce that wander.

Measured heading accuracy is **3.9°**. The useful comparison is not zero but the geometric
budget σ_ψ ≈ √2·σ_d/L ≈ 4.4° for a single pair of independent fixes — it beats that because
the estimator accumulates the constraint across a window and fuses gyro rate.

---

## Tuning

`python/dune/fusion_config.py` holds the defaults; the GUI exposes the ones worth touching
live. Start by changing nothing — the defaults are what produced the reported numbers.

| Parameter | Default | Effect |
|---|---|---|
| `nlos_thresh_db` | 3.0 | gap below this reads as line-of-sight |
| `nlos_weight_min` | 0.05 | floor weight for a suspect anchor |
| `nlos_k_bias` | 0.0 | optional range de-biasing, mm/dB; off by default |
| `ekf_jump_thresh` | 2.0 m | EKF reseed threshold |

The rigid MHE carries its own defaults on the class (`python/dune/mhe_rigid.py`), not in
`fusion_config.py` — these are the ones the reported results use:

| Parameter | Default | Effect |
|---|---|---|
| `horizon` | 12 nodes | ≈2 s at two tags × ~3 Hz. Longer = smoother, slower, laggier |
| `baseline` | 0.50 m | **measure your own rig**, antenna centre to antenna centre |
| `huber_delta` | 0.15 | pseudo-Huber knee on range residuals |
| `max_jump` | 0.20 m | reseed when the centre departs a CV prediction by more than this |

> `baseline` is the one you must not leave at its default. It is a physical property of
> your rig, the whole rigid constraint is built on it, and a wrong value biases every pose.
