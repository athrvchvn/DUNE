"""Test-wide guards.

The one rule enforced here: **a test run must not leave anything behind in the
checkout.**
"""

from __future__ import annotations

import shutil
import sys

import pytest


@pytest.fixture(autouse=True)
def results_dir_is_disposable(tmp_path, monkeypatch):
    """Redirect every binding of ``RESULTS_DIR`` at a scratch directory.

    Capture and job code writes its output under ``host.paths.RESULTS_DIR`` —
    correct in production, corrosive in a test. Left alone, running the suite
    drops synthetic ``rover_runs/*.mat``, ``survey_runs/`` and
    ``roverreport_*/`` into the working tree. On the *next* run the
    module-level ``skipif(not ROVER_RUNS)`` guards in test_reports.py see that
    debris, conclude the real rover corpus is present, and fail comparing
    synthetic captures against expectations derived from real ones.

    So the suite would pass once on a fresh clone and fail every time after —
    the worst shape a failure can take, since a clean CI runner never
    reproduces it and whoever does hit it has no reason to suspect their own
    earlier test run.

    The scratch directory lives **inside the repository**, not in ``/tmp``.
    ``resolve_under_repo`` rejects paths outside the repo root, and that check
    guards untrusted paths arriving from the browser — it is not something to
    relax for the convenience of a test. So the tests respect it and clean up
    after themselves instead.

    Patching ``host.paths`` alone is not enough: ``jobs_offline`` and
    ``jobs_calib`` bind the name at import time and keep the old value. Every
    already-imported module carrying the attribute is redirected, so a new
    module that imports it is covered without editing this fixture.
    """
    import host.paths

    scratch = host.paths.REPO_ROOT / '.pytest-results' / tmp_path.name
    scratch.mkdir(parents=True, exist_ok=True)

    monkeypatch.setattr(host.paths, 'RESULTS_DIR', scratch)
    for name, mod in list(sys.modules.items()):
        if name.startswith('host.') and hasattr(mod, 'RESULTS_DIR'):
            monkeypatch.setattr(mod, 'RESULTS_DIR', scratch)

    yield scratch

    shutil.rmtree(scratch, ignore_errors=True)
