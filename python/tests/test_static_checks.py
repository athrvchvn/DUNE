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

import re
import subprocess
import sys
from pathlib import Path

import pytest

PY = Path(__file__).resolve().parents[1]

# The packages this port owns. `analysis/` and the standalone calibration scripts
# predate it and carry their own lint debt; adding them here would mean either a
# noisy failure or a stale ignore list, so they are deliberately out of scope.
OWNED = ('host', 'dune', 'reports', 'tests')


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


def _is_ours(top: str) -> bool:
    """Does `top` name a module that ships in this repository?

    Searched over the whole of `python/`, not just its top level: the point of
    the check is to catch an import left pointing at a module's *old* location
    after it moved into a subpackage, which is precisely the case a top-level
    lookup misses.
    """
    if any(p.name == top and (p / '__init__.py').exists()
           for p in PY.rglob(top) if p.is_dir()):
        return True
    return any(p.parent.name != '__pycache__' for p in PY.rglob(f'{top}.py'))


def test_the_entry_points_still_start():
    """Nothing else imports `run_localization.py` or the operator tools.

    The whole suite passed with `run_localization.py` unable to import at all,
    because a module nobody imports is a module nobody type-checks. The live
    host is the one program every user runs first, so its import graph is
    exactly what must not be allowed to rot silently.

    `--help` is used rather than an import because these are scripts: argparse
    runs, the module body executes top to bottom, and nothing binds a socket.
    """
    scripts = [PY / 'run_localization.py', *sorted((PY / 'tools').glob('*.py'))]
    broken = []
    for s in scripts:
        if s.name == '__init__.py':
            continue
        proc = subprocess.run([sys.executable, str(s), '--help'],
                              capture_output=True, text=True,
                              cwd=PY.parent, timeout=120)
        if proc.returncode == 0 or 'usage:' in proc.stdout:
            continue

        # A missing *third-party* package is the reader's environment and not
        # this tree's problem. A missing module that is sitting right there in
        # python/ is a stale import — the bug this test exists for. Deciding by
        # looking on disk keeps the test honest when modules move again.
        missing = re.findall(r"No module named '([\w.]+)'", proc.stderr)
        ours = [m for m in missing if _is_ours(m.split('.')[0])]
        if ours:
            broken.append(f"{s.name}: stale import of {', '.join(ours)}")
        elif not missing:
            broken.append(f"{s.name}: exit {proc.returncode}: "
                          f"{proc.stderr.strip()[-200:]}")

    assert not broken, "entry points that will not start:\n  " + "\n  ".join(broken)


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
