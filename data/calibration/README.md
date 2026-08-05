# Calibration

Configuration, not data — this is what the DUNE pipelines read to turn raw ranges into
positions. Full explanation in [`../docs/calibration.md`](../docs/calibration.md).

| Folder | Contents |
|---|---|
| `anchors/` | Anchor geometry for every layout used, plus `layout_index.md` mapping each run to its file. **Start here.** |
| `antenna_delay/` | Tier-1 per-board antenna delay (already baked into the hardware — traceability only, do **not** re-apply in software) |
| `range_correction/` | Tier-2 first-path-power correction, applied host-side at solve time |
| `camera/` | Ground-truth camera model — read `../docs/ground_truth.md` for its limitations |
