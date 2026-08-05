# Raw captures — TO BE ADDED

The `runs/` folder ships processed `.mat` files. This folder is for the *unprocessed* streams
behind them, so the full processing chain is reproducible from the wire up. Placeholder: not
yet populated.

## `serial/`
Raw serial line captures in the DUNE v1 wire format, one file per published run, named to
match the run folder (e.g. `run_01_20260725_184427.log`):

```
RTLS,v1,<t_ms>,<tag_id>,<n>,<id,d_mm,q>...[,IMU,...][,CYC,<n>][,MRX,<us>]
DIAG,v1,...
```

One example of this format is already published: `../static/static_accuracy_*/serial_raw.log`.

## `session_logs/`
Host-side `session_*.ndjson` logs from the Python pipeline — one JSON object per line
recording every packet, solve, and GUI event. ~305 MB exists in the research archive; select
only the sessions covering the 17 published runs, and strip any WiFi credentials or host IPs
before publishing.

## Before adding anything here
- [ ] Confirm no SSID, password or host IP appears in any line
- [ ] Confirm filenames map 1:1 onto `runs/`
- [ ] Re-run `tools/make_checksums.sh`
