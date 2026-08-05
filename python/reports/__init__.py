"""Offline analysis reports.

Each module here takes **file paths in and returns JSON out**, writing its
artifacts to a directory. No live handles, no bus, no app — that discipline is
what keeps the option of moving these onto a `ProcessPoolExecutor` open (see
`host/jobs.py`), and it is also what makes them callable from a test without
standing up a server.

Progress is reported through an optional `progress` callback rather than by
importing anything from `host`, so the dependency only ever points one way.
"""

from .writer import ReportWriter, fmt_mm, fmt_pct

__all__ = ['ReportWriter', 'fmt_mm', 'fmt_pct']
