# tools/

| File | Purpose |
|---|---|
| `load_run.py` | Load one run in Python (`numpy`, `h5py`). Run it directly for a summary of any run |
| `load_run.m` | The MATLAB equivalent, same returned structure |
| `make_checksums.sh` | Regenerate `checksums.sha256` after changing the dataset |
| `verify.sh` | Verify a downloaded copy |

These are MIT licensed (the data is CC BY 4.0).

```bash
python3 tools/load_run.py runs/run_01_20260725_184427
bash    tools/verify.sh
```
