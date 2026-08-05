"""Reading the raw wire tees that `dune.TagSerial` / `dune.TagUdp` write.

There are **two** formats, and assuming the wrong one fails silently rather than
loudly — which is exactly what happened: the first version of the replay reader
handled only the serial form, so every `udp_raw_*.log` produced an empty result
and looked like a session with no data.

    serial:  <t_host>\\t<line>
    udp:     <t_host>\\t<src_ip>\\t<line>

Both are tab-separated with the host receive time first, so the rule is simply
"take the last tab-separated field as the payload, the first as the timestamp".
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterator

PREFIXES = ('RTLS,', 'DIAG,', 'HWCALIB')


def iter_raw_lines(path: str | Path, prefixes=('RTLS,',)) -> Iterator[tuple[float, str]]:
    """Yield ``(t_host, line)`` for payloads matching any of `prefixes`."""
    with open(path, 'r', errors='replace') as fh:
        for raw in fh:
            parts = raw.rstrip('\n').rstrip('\r').split('\t')
            if len(parts) < 2:
                continue
            body = parts[-1]
            if prefixes and not body.startswith(tuple(prefixes)):
                continue
            try:
                yield float(parts[0]), body
            except ValueError:
                continue


def read_raw_tee(path: str | Path, prefixes=('RTLS,',)) -> list[tuple[float, str]]:
    return list(iter_raw_lines(path, prefixes))


def is_raw_tee(path: str | Path, probe_lines: int = 400) -> bool:
    """Whether this file looks like a raw tee rather than a JSONL log."""
    p = Path(path)
    if p.suffix.lower() in ('.jsonl', '.ndjson'):
        return False
    with open(p, 'r', errors='replace') as fh:
        for _ in range(probe_lines):
            line = fh.readline()
            if not line:
                break
            parts = line.rstrip('\n').split('\t')
            if len(parts) >= 2 and parts[-1].startswith(PREFIXES):
                return True
    return False
