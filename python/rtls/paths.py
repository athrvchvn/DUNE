"""Where things live.

One module owns the answer, because the alternative — every caller recomputing
``Path(__file__).parents[N] / ...`` — is how a tree ends up with the same
directory spelled four different ways and one of them wrong after a move.

The split is deliberate: this module states *locations* and nothing else, so
the library can find its configuration without importing the host application.
``host.paths`` builds on it and adds *policy* — which directories the browser
is allowed to reach, and the containment check that keeps an untrusted path
from escaping the repository.

``config/`` sits at the repository root rather than under ``matlab/``: both the
Python and the MATLAB pipeline read the same anchor geometry, the same
correction coefficients and the same camera calibration. Configuration
describes the *deployment*, not one implementation of it.
"""

from __future__ import annotations

from pathlib import Path

# python/rtls/paths.py -> python/rtls -> python -> repo root
REPO_ROOT = Path(__file__).resolve().parents[2]

CONFIG_DIR = REPO_ROOT / 'config'      # tracked; the deployment's parameters
DATA_DIR = REPO_ROOT / 'data'          # datasets, distributed separately
LOGS_DIR = REPO_ROOT / 'logs'          # session output, gitignored
RESULTS_DIR = REPO_ROOT / 'results'    # report output, gitignored

PYTHON_DIR = REPO_ROOT / 'python'
MATLAB_DIR = REPO_ROOT / 'matlab'
FIRMWARE_DIR = REPO_ROOT / 'firmware'


def config(name: str) -> Path:
    """Path to a file in ``config/``. Existence is the caller's problem."""
    return CONFIG_DIR / name
