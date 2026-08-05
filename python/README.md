# dune-uwb

Python host pipeline for **DUNE**, an open-source indoor localization stack for ground
robots built on Decawave DW1000 radios: firmware, ranging protocol, dual-tag
synchronization, two-tier calibration, camera-free anchor survey, robust solving, and a
rigid-body moving-horizon estimator.

```bash
pip install -e "python/[all]"
python python/run_localization.py --serial /dev/ttyUSB0
```

```python
from dune.anchor_config import AnchorConfig
from dune.frame_parser import FrameParser
from dune.solve_sweep import solve_sweep
from dune.paths import config

cfg = AnchorConfig.from_json(str(config('anchors.json')))
pkt = FrameParser.parse("RTLS,v1,500,240,2,1,1000,-80,2,2000,-85")
fix = solve_sweep(pkt, cfg)
```

**Full documentation, hardware guide and examples are in the repository root** —
see the top-level `README.md` and `docs/`.

Extras: `[mhe]` (CasADi), `[reports]` (offline analysis), `[vision]` (overhead-camera
ground truth), `[dev]`, `[all]`.

MIT licensed, except the vendored DW1000 driver under
`firmware/libraries/UwbRtls/src/dw1000/`, which is Apache-2.0.
