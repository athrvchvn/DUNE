# Wire protocol

Three protocols matter in DUNE, at different layers:

1. **Over the air** — the UWB frames anchors and tags exchange (DS-TWR).
2. **Tag → host** — the ASCII `RTLS,…` line every sweep produces.
3. **Tag ↔ tag** — the wired dual-tag link (see [dual_tag_link.md](dual_tag_link.md)).

The authoritative definitions live in `firmware/libraries/UwbRtls/src/HostLink.h` and
`UwbFrame.h`, and the reference decoder is `python/dune/frame_parser.py`. If this document
and that parser ever disagree, the parser wins.

---

## 1. Over the air

### Addressing

One-byte short addresses — DUNE's own scheme, **not** 802.15.4:

| Range | Meaning |
|---|---|
| `0x01`–`0xEF` | anchors |
| `0xF0`–`0xFE` | tags |
| `0xFF` | broadcast (reserved) |

Filtering is done in software against the destination byte, so the anchor count is bounded
by the address space and by sweep time — not by the protocol. This is the change that
removes the fixed small-network ceiling that 802.15.4 discovery state machines impose in
libraries derived from `arduino-dw1000`.

### Frame

Every UWB payload starts with a four-byte header:

```
[0] type   [1] src   [2] dst   [3] seq   [4..] payload
```

The DW1000 hardware CRC stays enabled, so corrupt frames never reach application code.

Message types 10 and 11 are **permanently unassigned**. They were `MSG_ANNOUNCE` and
`MSG_SLOT_GRANT` in a retired over-air token ring; leaving them unallocated means a board
still running that firmware cannot be misread by current firmware.

### Ranging — asymmetric double-sided TWR

Four messages per anchor. Only the addressed anchor replies, so exchanges within a sweep
never collide.

```
TAG                                    ANCHOR
 │  POLL ────────────────────────────►  │   t: pollSent          r: pollReceived
 │  ◄──────────────────────  POLL_ACK   │   r: pollAckReceived   t: pollAckSent
 │  RANGE (carries 3 tag stamps) ─────►  │   t: rangeSent         r: rangeReceived
 │                                       │   → computes ToF
 │  ◄─────────────────── RANGE_REPORT   │   (distance, RX power, first-path power)
```

The RANGE message embeds the tag's three timestamps using DW1000 **delayed transmit**, so
the tag knows its exact send time before sending it. The anchor then holds all six
timestamps and computes:

```
round1 = pollAckReceived − pollSent        (tag clock)
reply1 = pollAckSent     − pollReceived    (anchor clock)
round2 = rangeReceived   − pollAckSent     (anchor clock)
reply2 = rangeSent       − pollAckReceived (tag clock)

ToF = (round1·round2 − reply1·reply2) / (round1 + round2 + reply1 + reply2)
```

This form cancels crystal frequency offset to first order and needs no symmetry between
the two reply delays — which is what lets the scheduled reply delay be tuned for throughput
without biasing range.

Timestamp quantum is 15.65 ps, or **4.69 mm** of range. That is the resolution floor;
everything above it is calibration and multipath.

### Scheduling

`UwbScheduler` walks the anchor list one exchange at a time and emits one host line per
completed sweep. Each anchor carries a failure streak: **two consecutive failures** causes
it to be skipped for 5 sweeps, doubling on each further failure to a cap of 40, and
resetting immediately on success. An unreachable anchor therefore costs almost nothing,
while a briefly obstructed one rejoins at once.

---

## 2. Tag → host

One newline-terminated ASCII line per sweep, over UDP or USB serial. Versioned, and the
host parser accepts **all** versions — a mixed-firmware fleet works.

```
RTLS,v<N>,<t_ms>,<tag_id>,<n>,<per-anchor fields × n>[,tails]
```

| Field | Meaning |
|---|---|
| `t_ms` | the **owning** tag's millisecond clock at sweep end |
| `tag_id` | source tag address (e.g. 240, 241) |
| `n` | number of **valid** anchor measurements that follow |

### Per-anchor fields, by version

| Version | Fields per anchor | |
|---|---|---|
| v1 | 3 | `id, d_mm, rx_dbm` |
| v2 | 5 | `id, d_mm, rx_dbm, fp_dbm, quality` |
| v3 | 5 | as v2 |
| v4 | 7 | `id, d_mm, rx_dbm, fp_dbm, quality, cfo, tex` |

`d_mm` is millimetres. `rx_dbm` is total received power; `fp_dbm` is first-path power.

**Both are read from the same received frame, on the tag side.** This is not an
implementation detail — it is what makes the NLOS score valid. The obvious implementation
compares the anchor's reported received power against the tag's own first-path reading, but
those are measured on different links in different directions, so their difference is not a
property of any single path. See [estimators.md](estimators.md).

v4 additions:

- `cfo` — raw DW1000 carrier integrator of the RANGE_REPORT reception, converted to ppm by
  the host. Positive means the anchor's crystal runs faster.
- `tex` — realised exchange start, in ms from the start of the sweep. Ranges within a sweep
  are sequential, not simultaneous; `tex` is what lets the host reason about motion *within*
  a sweep instead of pretending it was instantaneous.

### Tails

Strictly positional, in this order: **`DIAG` → `IMU` → `CYC` → `MRX`**. Each is optional.

```
,DIAG,<die_temp_c>,<vbat_v>
,IMU,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>                          (v2)
,IMU,<status>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>  (v3+)
,CYC,<cycle_id>
,MRX,<master_rx_us>
```

The `IMU` tail appears only when a sample is present, so a tag without an IMU emits a valid
line that older and newer parsers both accept. `status` is the BNO085 accuracy field (0–3).

> **Two different things are called DIAG.** The `,DIAG,` *tail* above carries the tag's own
> die temperature and battery voltage. A standalone `DIAG,v1,…` *line* carries the dual-tag
> TDMA time budget and is parsed separately. They are unrelated.

### Dual-tag tails

Appended by the master tag:

- `CYC` — the master's cycle number. It appears on both tags' lines, which is what joins
  them into one cycle.
- `MRX` — master receive time (µs, master clock) of the slave's data. Present **only** on
  forwarded slave lines.

`t_ms` always remains the owning tag's own clock; forwarded lines are never rewritten. The
host therefore has three time references per measurement — the slave's `t_ms`, the master's
`MRX`, and the `DIAG` line's cycle start — and can choose which to trust.

---

## 3. Anchor self-survey

The same link carries the survey exchange, in which a tag instructs one anchor to range to
another. The tag's own antenna delay never enters those measurements.

```
SURVEY_BEGIN,v1,<n_pairs>
SURVEY,v1,<src_id>,<dst_id>,<avg_dist_mm>,<ok_samples>
SURVEY_DONE,v1
```

Trigger it by sending `SURVEY\n` to the tag. See [anchor_survey.md](anchor_survey.md).

---

## Adding a field

The rule that keeps this format extensible: **append, never insert.** Add a new tail, or a
new version with more per-anchor fields, and leave every earlier version parseable. Both
host parsers dispatch on the version token and the tails are order-checked positionally, so
an old host reading a new tag degrades to ignoring what it does not know rather than
misreading it.

When you add a version, update `python/dune/frame_parser.py`, `matlab/+dune/parseRtlsLine.m`
and `firmware/libraries/UwbRtls/src/HostLink.h` together — and add a line of that version
to `python/tests/test_equivalence.py`, which asserts every version still parses.
