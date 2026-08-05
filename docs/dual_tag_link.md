# Dual-tag link (TagLink)

A second tag buys **heading**, which a single tag cannot observe — no arrangement of ranges
to a point tells you which way that point is facing. But two tags cannot simply be switched
on together.

## The problem

Two tags ranging independently do not interleave gracefully. An anchor answers whichever
poll it receives, so overlapping polls produce a **capture effect**: one tag monopolises the
anchors and the other starves. That is the worst possible outcome for a rig whose heading
depends on both ends of its baseline.

An over-air token ring was tried first and is documented here only so nobody rebuilds it: it
crashed (interrupt-watchdog panics, one tag reboot-looping) and coordinating over the same
lossy shared channel you are trying to arbitrate is circular.

## The solution: arbitrate over a wire

The two tags are rigidly mounted ~50 cm apart on one platform, so the coordination channel
does not need to be the UWB air at all. A dedicated UART between the two ESP32s arbitrates.

**At any instant exactly one tag's DW1000 may transmit; the other's transceiver is off
(TRXOFF).** It neither disturbs nor even hears the active slot. Collision is *structurally
impossible* rather than merely unlikely.

Measured sweep split: **48–52%** — direct evidence the arbitration prevents the starvation it
was built to avoid.

## Roles

| Role | Address | Behaviour |
|---|---|---|
| **MASTER** | 0xF0 | Has the IMU. Owns the schedule and the host link. Sweeps first, grants the slave its slot, publishes both tags' lines plus diagnostics. |
| **SLAVE** | 0xF1 | **No WiFi at all.** Radio parked outside its slot; on `GO` it sweeps, returns binary results over the wire, parks again. |
| **SOLO** | — | Classic single-tag free-run, TagLink inert. A master also degrades to this automatically while the slave is absent. |

> **The slave's slot is master CPU time, not wait time.** `GO` is issued the moment the
> master's own sweep ends, and the master's IMU reads, line formatting, UDP transmit, DIAG
> emission and OLED updates all execute *while the slave is sweeping*. Blocking wait is the
> fallback, not the schedule.

## Wiring

| Signal | Board A | Board B |
|---|---|---|
| A TX → B RX | GPIO 22 | GPIO 35 |
| B TX → A RX | GPIO 35 | GPIO 22 |
| Ground | GND | GND (**required**) |

Cross-wire TX→RX in both directions plus a common ground. Keep leads short and twisted.
UART2 at **921600 baud** (~11 µs/byte; a full DATA frame ≈ 1 ms).

GPIO 22 (output-capable) and 35 (input-only, fine for RX) are free on both board maps.

> **Do not use:** 16/17 (WROVER PSRAM), **13/14** (the charge-mode jumper — see
> [hardware.md](hardware.md)), or 12 (strapping).

## Wire protocol

Binary, framed, with the protocol version in **every** frame so mismatched firmware is
rejected at the parser rather than misinterpreted:

```
0xA5 | proto_ver | type | len | payload[len] | crc16-ccitt-false(ver,type,len,payload)
```

| Type | Direction | Purpose |
|---|---|---|
| `HELLO` (1) | slave→master | Join + liveness. Every 100 ms until ACK, then a 500 ms heartbeat. Carries a build id, so the master warns on a mixed fleet. |
| `HELLO_ACK` (2) | master→slave | Confirms join. |
| `GO` (3) | master→slave | Grants the air slot: `cycle_id`, `budget_ms`. |
| `DATA` (4) | slave→master | Sweep results. Only **valid** anchors ship. |
| `DONE` (5) | slave→master | Explicit end-of-slot, separate from DATA so a slot can carry several DATA frames later. |

**Cycle number is the scheduler's primary key.** The master owns a monotonic `cycle_id`;
every message and every published line references it, and frames with a stale cycle are
discarded and counted. `slave_seq` (u32 — multi-hour runs never wrap) is an independent
counter, so UART loss or duplication is distinguishable from cycle mismatch.

Both ends keep link-health counters — `crc_err` (meaning: bad cable), `resync`,
`stale_cycle`, sequence gap/dup, slot timeouts — surfaced on the master's periodic `[TIME]`
serial line and on the OLED.

### Adaptive slot budget

```
budget = clamp(1.5 × EMA(slave sweep time), 150 ms, 700 ms)
```

The EMA grows instantly on a slower sweep and shrinks slowly (α = 0.1), so a single lucky
fast sweep can never starve a legitimate worst-case one into being declared absent. Starts
at 700 ms until the first measurement. Sent inside `GO` — the master is authoritative.

### Failure model

Three consecutive slot timeouts declares the slave absent; the master then skips its slot
and runs solo but keeps listening. The slave hot-rejoins via `HELLO` within ~100 ms of
power-up — **neither board reboots**. Pulling the UART wire behaves identically.

## Cycle timeline

```
startCycle ─ own sweep ─ RADIO_IDLE ─ GO ──────────────┐  (slave sweeping)
                                     IMU / format / UDP own line (,CYC) /
                                     OLED / [idle: pollImu]
                                     ◄─ DATA ◄─ DONE ──┘
             forward slave line (,CYC,MRX) ─ DIAG ─ next cycle
```

**Three clocks, originals never destroyed.** The slave's own `t_ms` rides inside its line
untouched; the master appends `,CYC,<cycle>` to its own lines and `,CYC,<cycle>,MRX,<µs>` to
forwarded slave lines; the per-cycle DIAG record carries `cycle_start_us`. Joining on `CYC`
gives every measurement a slave-clock, a master-receive and a cycle-start reference.

All firmware-internal timing uses `esp_timer_get_time()` (µs), never `millis()`. The `t_ms`
wire field stays in milliseconds — changing its unit would be a wire-format break.

### Per-cycle diagnostics

```
DIAG,v1,<cycle>,<cycle_start_us>,<master_sweep_us>,<slave_sweep_us>,<uart_wait_us>,<proc_us>,<idle_us>,<total_us>
```

Logged verbatim by the host. **UART latency ≈ `uart_wait_us` − `slave_sweep_us`**, which
answers "is the UART on the critical path?" without a firmware change.

> This standalone `DIAG,v1` *line* is a different thing from the `,DIAG,` *tail* on an RTLS
> line, which carries die temperature and battery voltage. See [protocol.md](protocol.md).

## Flashing

Both tags use `firmware/sketches/TagWrover/TagWrover.ino`, differing only in the
`PER-BOARD CONFIGURATION` block:

1. **Master**: `#define TAG_LINK_MASTER`, `TAG_ID 0xF0`, its own calibrated `ANTENNA_DELAY`.
2. **Slave**: `#define TAG_LINK_SLAVE`, `TAG_ID 0xF1`, its own calibrated `ANTENNA_DELAY`.
3. **Anchors**: unchanged sketch.

Bench utilities (`SURVEY`, `HWCALIB`, `SETANTDELAY`) still work on either tag over USB
serial, but they transmit **outside any slot** — run them with the other tag powered off.

## Bringing it up

1. **Master solo**, no wire: should behave exactly like single-tag firmware. `[TIME]` prints;
   the host shows tag 0xF0 with the IMU tail and `,CYC` tails.
2. **Dual**: wire it up. The slave should join within ~100 ms (`[LINK] slave 0xF1 joined`).
   The host shows both tag ids; forwarded lines carry the original slave `t_ms` plus
   `,CYC,MRX`; link counters stay at zero on a clean cable.
3. **Failure drill**: kill slave power mid-run — the master should go solo within three
   cycles. Repower — it should rejoin without a master reboot. Pull the UART wire — same.
   Watch the adaptive budget converge to ≈1.5× the slave sweep without false timeouts.
4. **Soak, ≥30 min continuous**: zero panics, and RTLS timestamps never reset.

Non-zero `crc_err` means the cable. That is almost always what it means.

## Host side

No architecture change is needed: both pipelines demux by `tag_id` regardless of source, so
the master forwarding both tags' lines from one socket works as-is. `FrameParser` exposes
`pkt.cycle` and `pkt.mrx_us`; `DIAG,v1` lines go to the session log.

Then point the estimator at the rig — [estimators.md](estimators.md#rigid-body-mhe). Set the
`baseline` to **your** measured antenna-to-antenna distance; it is the foundation of the
rigid constraint.
