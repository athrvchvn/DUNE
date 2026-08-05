"""Where the GUI is allowed to read and write.

The WebSocket server binds ``0.0.0.0`` and job arguments arrive from the
browser, so a file path in a message is **untrusted input**. Without a
containment check, ``{"type":"job_run","kind":"session_summary",
"args":{"log":"/etc/passwd"}}`` reads that file and puts an excerpt in the
session log; a write-shaped job would be worse. Everything therefore resolves
against the repository root and is rejected if it escapes.

`..` is not filtered — it is *resolved* and then the result checked, because
filtering strings is a game you lose to symlinks and encodings.
"""

from __future__ import annotations

from pathlib import Path

# Locations come from the library (rtls.paths); this module adds only policy.
from rtls.paths import (  # noqa: E402,F401
    CONFIG_DIR, DATA_DIR, LOGS_DIR, MATLAB_DIR, PYTHON_DIR, REPO_ROOT,
    RESULTS_DIR,
)

# Browsable collections, keyed by the `kind` a `path` argument declares. Order
# matters: the first existing directory is the default the picker opens on.
#
# `logs/` is the current location; `matlab/logs` and `python/logs` are where
# sessions recorded before the layout was unified still sit, and are kept so an
# existing working copy does not lose sight of its own history.
BROWSABLE: dict[str, tuple[tuple[Path, str], ...]] = {
    'logs':       ((LOGS_DIR, '*.jsonl'),
                   (LOGS_DIR, '*.ndjson'),
                   (LOGS_DIR, 'serial_raw_*.log'),
                   (LOGS_DIR, 'udp_raw_*.log'),
                   (MATLAB_DIR / 'logs', '*.jsonl'),
                   (MATLAB_DIR / 'logs', 'serial_raw_*.log'),
                   (MATLAB_DIR / 'logs', 'udp_raw_*.log'),
                   (PYTHON_DIR / 'logs', '*.jsonl'),
                   (PYTHON_DIR / 'logs', '*.ndjson')),
    'rover_runs': ((RESULTS_DIR / 'rover_runs', '*.mat'),
                   (DATA_DIR / 'rover_runs', '*.mat')),
    'results':    ((RESULTS_DIR, '*/report.txt'),),
    'config':     ((CONFIG_DIR, 'anchors*.json'),),
    'captures':   ((RESULTS_DIR, 'static_capture.txt'),
                   (RESULTS_DIR, '*/serial_raw.log'),
                   (DATA_DIR / 'static_campaign', '*.txt'),
                   (LOGS_DIR, 'serial_raw_*.log'),
                   (LOGS_DIR, 'udp_raw_*.log')),
    'survey':     ((RESULTS_DIR / 'survey_runs', 'survey_*.txt'),
                   (DATA_DIR / 'survey', 'survey_*.txt')),
}


class PathNotAllowed(ValueError):
    """A supplied path resolved outside the repository."""


def resolve_under_repo(path: str | Path) -> Path:
    """Resolve `path` and confirm it stays inside the repo.

    Relative paths are taken against the repo root, so the GUI can send the
    short form it displays.
    """
    p = Path(path)
    if not p.is_absolute():
        p = REPO_ROOT / p
    p = p.resolve()
    try:
        p.relative_to(REPO_ROOT)
    except ValueError:
        raise PathNotAllowed(f"path outside the repository: {path}") from None
    return p


def rel(path: str | Path) -> str:
    """Repo-relative form — what the GUI shows and sends back."""
    p = Path(path).resolve()
    try:
        return str(p.relative_to(REPO_ROOT))
    except ValueError:
        return str(p)


def list_files(kind: str, limit: int = 400) -> list[dict]:
    """Files the GUI may offer for a `path` argument of this kind, newest first."""
    specs = BROWSABLE.get(kind)
    if specs is None:
        raise ValueError(f"unknown file kind: {kind}")

    seen: dict[str, dict] = {}
    for directory, pattern in specs:
        if not directory.is_dir():
            continue
        for f in directory.glob(pattern):
            if not f.is_file():
                continue
            try:
                st = f.stat()
            except OSError:
                continue
            key = str(f.resolve())
            seen.setdefault(key, {'path': rel(f), 'name': f.name,
                                  'bytes': st.st_size,
                                  'mtime': round(st.st_mtime, 1)})
    out = sorted(seen.values(), key=lambda d: d['mtime'], reverse=True)
    return out[:limit]
