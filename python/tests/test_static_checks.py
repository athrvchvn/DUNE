"""Static checks that catch a whole bug class the unit tests structurally miss.

Twice now a job body has referenced an undefined name — `Path` in `host/jobs.py`,
`CONFIG_DIR` in `host/jobs_offline.py` — and both times the tests passed. The
reason is structural, not carelessness: a job body only executes inside a worker
thread when that specific job runs with that specific combination of arguments,
so a name used on one branch is invisible to a test that calls the underlying
function directly. `NameError` then surfaces at the worst moment: on real
hardware, mid-calibration.

pyflakes finds all of it in under a second, so it belongs in the suite rather
than in a habit.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

PY = Path(__file__).resolve().parents[1]

# The packages this port owns. `analysis/` and the standalone calibration scripts
# predate it and carry their own lint debt; adding them here would mean either a
# noisy failure or a stale ignore list, so they are deliberately out of scope.
OWNED = ('host', 'rtls', 'reports', 'tests')


def _pyflakes(paths):
    proc = subprocess.run([sys.executable, '-m', 'pyflakes', *paths],
                          capture_output=True, text=True, cwd=PY, timeout=120)
    if 'No module named' in proc.stderr:
        pytest.skip("pyflakes not installed")
    return [l for l in proc.stdout.splitlines() if l.strip()]


def test_no_undefined_names_anywhere_in_the_port():
    """The check that would have caught both NameError bugs immediately."""
    findings = [l for l in _pyflakes(OWNED) if 'undefined name' in l]
    assert not findings, "undefined names:\n  " + "\n  ".join(findings)


def test_no_unused_imports_in_the_port():
    """Kept separate from the above: cosmetic, but it is how dead code hides.

    An import left behind after a refactor is usually the last trace of a code
    path that was removed — worth noticing while the context is still fresh.
    """
    ignored = ('imported but unused',)
    findings = [l for l in _pyflakes(OWNED)
                if any(k in l for k in ignored)
                # A module imported purely to register its jobs has no name to
                # use; those are marked with an explicit noqa at the import.
                and 'noqa' not in l]
    unexplained = []
    for line in findings:
        loc = line.split(':')[0:2]
        try:
            src = (PY / loc[0]).read_text().splitlines()[int(loc[1]) - 1]
        except Exception:
            unexplained.append(line)
            continue
        if 'noqa' not in src:
            unexplained.append(line)
    assert not unexplained, "unused imports:\n  " + "\n  ".join(unexplained)


def test_every_registered_job_body_has_resolvable_globals():
    """Complements the pyflakes check by actually importing every job module.

    A job whose module fails to import is advertised to the GUI and then fails
    the moment it is clicked.
    """
    sys.path.insert(0, str(PY))
    import host.jobs_builtin      # noqa: F401
    import host.jobs_calib        # noqa: F401
    import host.jobs_offline      # noqa: F401
    from host.jobs import JOB_KINDS

    assert len(JOB_KINDS) >= 8, f"only {len(JOB_KINDS)} jobs registered"
    for kind, spec in JOB_KINDS.items():
        assert callable(spec.fn), kind
        assert spec.args_schema is not None, kind
