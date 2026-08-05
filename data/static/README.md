# Static accuracy campaign

Fixed-position captures with long dwells at surveyed spots — the counterpart to the driven
runs in `runs/`. This is what the Tier-2 first-path-power range correction was fitted on.

```
static_accuracy_<YYYYMMDD_HHMMSS>/
├── serial_raw.log   # raw DUNE v1 wire-format capture (see ../docs/file_formats.md)
├── spots.json       # surveyed test positions
├── bakeoff.mat      # estimator comparison over the campaign
├── pinn_model.mat   # learned correction model trialled against the analytic one
├── report.txt       # per-spot measured error
├── map.png, ekf_replay_serial_raw.png
```

`static_accuracy_20260720_195637` is the campaign cited by
`calibration/range_correction/range_correction.json` — 19 spots, Kinect click-truth,
leave-one-spot-out position RMSE 100 mm. Publishing it means that fit is reproducible rather
than asserted.
