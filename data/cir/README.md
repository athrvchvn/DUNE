# Channel impulse response captures

Per-anchor CIR (channel impulse response) accumulator dumps from the DW1000, captured
2026-07-21. These are the raw multipath evidence behind the NLOS discussion: the first-path
peak and the later reflections are directly visible.

```
cir_<YYYYMMDD_HHMMSS>/
├── cir_A<n>.mat            # accumulator dump for the link to anchor n
└── cir_waterfall_A<n>.png  # waterfall plot over the capture
```

Empty capture folders present in the research archive were dropped; 9 captures remain.

These are diagnostic captures, not synchronised with any run in `runs/` — they were taken in
a separate session to characterise the channel, and are published so the first-path-power
correction in `calibration/range_correction/` can be judged against the actual channel
response rather than taken on trust.
