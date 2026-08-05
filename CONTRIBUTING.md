# Contributing

Bug reports, hardware reports and pull requests are all welcome. This page says what will
make yours easy to act on.

## Reporting a problem

The most useful bug report for a radio system includes **raw wire lines**. Paste a handful
of `RTLS,…` lines along with:

- board type and count, and which are anchors vs tags
- your `config/anchors.json`
- the radio settings from `firmware/libraries/UwbRtls/src/UwbConfig.h`
- what you expected and what you saw

Before filing, check [docs/troubleshooting.md](docs/troubleshooting.md) — the three most
common causes are anchor geometry, missing Tier-1 calibration, and a radio-config change
that invalidated every antenna delay.

**Hardware reports are valuable even when nothing is wrong.** If you run DUNE on a platform
outside [docs/compatibility.md](docs/compatibility.md)'s verified table, saying so — working
or not — is a genuine contribution.

## Development setup

```bash
git clone https://github.com/athrvchvn/DUNE
cd DUNE
python3 -m venv .venv && source .venv/bin/activate
pip install -e "python/[all]"
python -m pytest python/tests -q         # expect 209 passed, 44 skipped
```

The 44 skips are data-dependent tests; a fresh clone ships no datasets. That is correct, not
a failure.

## Before you open a pull request

```bash
python -m pytest python/tests -q         # green
python -m pytest python/tests -q         # run it twice — the result must be identical
git status --porcelain                   # must be empty
ruff check python/
```

The "run it twice" step is not superstition. A test that writes into the checkout can change
the *outcome* of the next run, and that failure mode is invisible to CI, which always starts
clean.

## Things worth knowing before you change code

### The architectural rule

> **The tag streams raw ranges. All position solving happens on the host.**

If a change would put a solve, a calibration correction, or an anchor coordinate into
firmware, it is going the wrong way. [docs/architecture.md](docs/architecture.md) explains
why and lists where each kind of change belongs.

### Two pipelines stay in step

`python/dune/` and `matlab/+dune/` implement the same algorithms. Change one, change the
other. Several files say so in their headers, and `python/tests/test_equivalence.py` fails if
the numeric behaviour diverges on recorded sessions.

### Two copies of every sketch

Every sketch exists in `firmware/sketches/` (the active flash targets) **and** in
`firmware/libraries/UwbRtls/examples/` (the IDE's Examples menu). They have intentionally
diverged. For a sketch-level change, **edit both**. Shared headers under
`firmware/libraries/UwbRtls/src/` are single-source: edit once.

### The layering rule

`python/dune/` must never import from `python/host/`. The library does not depend on the
application. If you want something from `host/` inside `dune/`, it probably belongs in
`dune/`.

### Wire-format changes

Append, never insert. Add a tail or a new version; leave every earlier version parseable.
Update `HostLink.h`, `python/dune/frame_parser.py` and `matlab/+dune/parseRtlsLine.m`
together, and add a line of the new version to `test_equivalence.py`.

## Tests

A test should fail when the behaviour it describes breaks. That sounds obvious and is the
thing most easily got wrong: this repository has shipped tests that asserted trivially-true
outcomes and passed against the very bug they were written for.

**So: break the code deliberately and confirm your test fails.** If it does not, the test is
decoration.

Two local conventions:

- **Data-dependent tests skip, they do not fail.** Follow the existing `skipif` idiom so a
  data-free clone stays green.
- **Never write into the checkout.** `python/tests/conftest.py` redirects `RESULTS_DIR` to a
  scratch directory removed per test. Read the comment there; it explains a real bug this
  caused.

Name tests after the behaviour, not the function — `test_the_inter_tag_baseline_is_exact_by_construction`
tells a reader what broke; `test_mhe_rigid_2` does not.

## Style

Match the surrounding code. The prose comments explaining *why* something is the way it is
are deliberate and load-bearing — several encode a bug that was expensive to find. Keep them,
and add your own when the reason for a choice is not obvious from the code.

`ruff` config is in `python/pyproject.toml` and is deliberately permissive; it is there to
catch mistakes, not to enforce taste.

## Commits

Explain **why**, not what — the diff already shows what. If a change fixes a subtle bug, say
what the bug did and how you know it is fixed.

## Licence

Contributions are accepted under the MIT licence. The vendored DW1000 driver in
`firmware/libraries/UwbRtls/src/dw1000/` is Apache-2.0 and keeps its upstream headers —
avoid gratuitous edits there.
