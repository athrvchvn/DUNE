"""Report text and figure plumbing.

`report.txt` is the primary artifact — it is what gets read, diffed against a
previous run, and pasted into a note. Figures are secondary. The MATLAB scripts
produce the same pair, and keeping the layout recognisably similar means the two
pipelines' outputs can be eyeballed side by side.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import numpy as np


def fmt_mm(v: float, width: int = 0, signed: bool = False) -> str:
    """Millimetres, or `NaN` — never `nan mm` or a misleading 0."""
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return 'NaN'.rjust(width)
    s = f"{v * 1000:+.0f}" if signed else f"{v * 1000:.0f}"
    return s.rjust(width)


def fmt_pct(v: float, width: int = 0) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return 'NaN'.rjust(width)
    return f"{v * 100:.1f}".rjust(width)


class ReportWriter:
    """Accumulate report lines, then write them out with any figures."""

    def __init__(self, out_dir: str | Path, title: str):
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.lines: list[str] = [title]
        self.artifacts: list[dict] = []

    # ── text ─────────────────────────────────────────────────────────────────

    def line(self, text: str = '') -> None:
        self.lines.append(text)

    def lines_(self, texts: Iterable[str]) -> None:
        self.lines.extend(texts)

    def section(self, title: str) -> None:
        self.lines.append('')
        self.lines.append(f"================ {title} ================")

    def table(self, header: str, rows: Iterable[str]) -> None:
        self.lines.append(header)
        self.lines.extend(rows)

    def write(self, name: str = 'report.txt') -> Path:
        p = self.dir / name
        p.write_text('\n'.join(self.lines) + '\n')
        self.artifacts.append({'path': str(p), 'kind': 'txt',
                               'bytes': p.stat().st_size})
        return p

    @property
    def text(self) -> str:
        return '\n'.join(self.lines) + '\n'

    # ── figures ──────────────────────────────────────────────────────────────

    def figure(self, name: str):
        """Context manager yielding a matplotlib Figure, saved on exit.

        matplotlib is imported lazily and forced onto the Agg backend: these run
        on a worker thread inside a server process, where any GUI backend would
        at best warn and at worst deadlock.
        """
        return _FigureContext(self, name)


class _FigureContext:
    def __init__(self, writer: ReportWriter, name: str):
        self.writer = writer
        self.name = name
        self.fig = None

    def __enter__(self):
        import matplotlib
        matplotlib.use('Agg', force=False)
        import matplotlib.pyplot as plt
        self.fig = plt.figure(figsize=(11, 7), dpi=110)
        return self.fig

    def __exit__(self, exc_type, exc, tb):
        import matplotlib.pyplot as plt
        try:
            if exc_type is None and self.fig is not None:
                p = self.writer.dir / self.name
                self.fig.savefig(p, bbox_inches='tight')
                self.writer.artifacts.append(
                    {'path': str(p), 'kind': 'png', 'bytes': p.stat().st_size})
        finally:
            if self.fig is not None:
                plt.close(self.fig)
        return False


def safe_stats(values) -> dict:
    """median/p95/max of a possibly all-NaN array, without NumPy warnings."""
    from rtls.metrics import prctile
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return {'n': 0, 'median': float('nan'), 'p95': float('nan'),
                'max': float('nan'), 'mean': float('nan')}
    return {'n': int(a.size), 'median': float(np.median(a)),
            'p95': prctile(a, 95), 'max': float(np.max(a)),
            'mean': float(np.mean(a))}
