# Release checklist

Run before tagging and pushing. It exists because "check it's clean" as a habit fails
exactly once, in public, permanently — git history cannot be quietly fixed after a push.

## Secrets and machine-specific values

```bash
grep -rn "ssid\|SSID\|password\|PASSWORD" --include=*.ino --include=*.h firmware/
grep -rEn "([0-9]{1,3}\.){3}[0-9]{1,3}" --include=*.ino --include=*.h --include=*.py firmware/ python/
grep -rn "D:\\\\\|C:\\\\Users\|/home/\|/media/" --include=*.py --include=*.m --include=*.ino .
grep -rn "COM[0-9]\|/dev/ttyUSB" --include=*.py --include=*.m . | grep -v "docs/\|%\|#"
```

- [ ] No real WiFi SSID or password in any sketch — placeholders only
- [ ] No hardcoded host IPs beyond `255.255.255.255` (broadcast) or `0.0.0.0`
- [ ] No absolute paths from anyone's machine, in code **or** docstrings
- [ ] Serial-port defaults are commented as per-machine and overridable

## Repository hygiene

```bash
git status --porcelain            # must be empty after a full test run
du -sh . && git count-objects -vH
```

- [ ] No generated logs, `.mat`, `__pycache__`, LaTeX artefacts, or `.pytest-results/`
- [ ] Repository under 50 MB
- [ ] `git status` clean immediately after `pytest` — a test that dirties the checkout is a
      bug (see `python/tests/conftest.py`)
- [ ] No paper sources, no sibling-repo clones, no third-party downloads

## Licensing

- [ ] `LICENSE` present
- [ ] Vendored `firmware/libraries/UwbRtls/src/dw1000/` retains its upstream Apache-2.0
      headers and its own `LICENSE.md`
- [ ] Any datasheet in `hardware/datasheets/` is redistributable, or is a link instead
- [ ] `CITATION.cff` parses and names the right authors

## It works

```bash
python -m pytest python/tests -q                    # 209 passed, 44 skipped
python -m pytest python/tests -q                    # run twice — identical
python python/run_localization.py --help
for f in examples/*.py; do python "$f" || echo "FAILED $f"; done
```

- [ ] Test suite green, and **identical on a second consecutive run**
- [ ] Every entry point starts
- [ ] Every example runs to completion
- [ ] MATLAB: `dune_setup` reports the right paths and loads anchors

## Fresh clone

The one that catches what all of the above misses.

```bash
git clone . /tmp/dune-clone && cd /tmp/dune-clone
python3 -m venv .venv && .venv/bin/pip install -e "python/[all]"
.venv/bin/python -m pytest python/tests -q
```

- [ ] Clones, installs and tests green in a clean venv
- [ ] `docs/getting_started.md` followed verbatim — every path it names exists
- [ ] Every relative link in `README.md` and `docs/` resolves

## Documentation honesty

- [ ] No claim in `README.md` that the repository does not support
- [ ] Pending items marked ⬜ rather than silently omitted
- [ ] Numbers match the paper
- [ ] `docs/compatibility.md` lists only versions actually tested; everything else is marked
      untested

## Outstanding for v1.0.0

- [x] ✅ **Datasets** — 17 curated rover runs, static campaign, CIR and self-survey captures
      now ship in `data/` (CC BY 4.0, checksummed). Still outstanding within it:
  - [ ] ⬜ `data/environment/` — site map, anchor layout drawing, deployment photos
  - [ ] ⬜ `data/raw/` — unparsed serial captures and host session logs (28 tests still skip
        without these)
  - [ ] ⬜ a Zenodo DOI for the data, added to `data/README.md`
- [ ] ⬜ **CAD** — `hardware/cad/`
- [ ] ⬜ **Datasheets** — `hardware/datasheets/`
- [ ] ⬜ **Board photos** — `hardware/images/`

> The repository should not be described as "released" while the paper's supplementary
> material table points at nothing. Code-complete and release-complete are different
> states, and it is worth being honest about which one you are in.

## Tag and publish

```bash
git tag -a v1.0.0 -m "DUNE v1.0.0 — reference implementation for the RA-P paper"
git push origin main --tags
```

- [ ] Tagged `v1.0.0` (not `0.x` — this is the validated system behind published numbers)
- [ ] `CHANGELOG.md` updated
- [ ] Zenodo GitHub integration enabled **before** pushing the tag, so the release mints a
      DOI
- [ ] DOI added to `CITATION.cff`, `README.md`, and the paper's `\repoLink`

A DOI is a permanent snapshot; a branch URL is not, and reviewers follow the link.
