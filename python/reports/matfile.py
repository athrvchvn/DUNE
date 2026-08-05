"""Reading MATLAB v7.3 ``.mat`` files, which are HDF5 underneath.

`scipy.io.loadmat` cannot read v7.3, and `rover_teleop_uwb.m` writes `-v7.3`,
so h5py it is. Two conversions are needed every single time and are easy to get
subtly wrong:

* **Transposition.** MATLAB is column-major and HDF5 is row-major, so a MATLAB
  ``[N x 3]`` array arrives as ``(3, N)``. Every array must be transposed back
  or the data silently reads as three very long samples instead of N triples.
* **Strings.** Character arrays arrive as ``uint16`` UTF-16 code units, not as
  text.

Both are handled here so no caller has to remember.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


class NotV73Error(ValueError):
    """The file is a `.mat`, but an older format h5py cannot open."""


def load_mat(path: str | Path) -> dict:
    """Load a v7.3 ``.mat`` into nested dicts, with MATLAB orientation restored."""
    import h5py

    path = Path(path)
    try:
        f = h5py.File(str(path), 'r')
    except OSError as e:
        # Pre-v7.3 .mat files are a proprietary container h5py cannot read, and
        # it says so as "file signature not found" — accurate but useless to
        # someone who just picked a file in the GUI. Note that sniffing the
        # magic bytes is NOT the way to detect this: a v7.3 file begins with a
        # 128-byte text header and only then the HDF5 signature, so a naive
        # check rejects perfectly good files. Attempting the open is the
        # reliable test.
        raise NotV73Error(
            f"{path.name} is not a MATLAB v7.3 file ({e}). Only v7.3 (HDF5) is "
            f"readable here; re-save it in MATLAB with save(..., '-v7.3').") from None

    with f:
        return {k: _convert(f[k], f) for k in f
                if not k.startswith(('#refs#', '#subsystem#'))}


def _convert(node, root) -> Any:
    import h5py

    if isinstance(node, h5py.Group):
        return {k: _convert(node[k], root) for k in node}

    arr = node[()]

    # MATLAB char arrays come through as uint16 UTF-16 code units.
    if isinstance(arr, np.ndarray) and arr.dtype in (np.uint16, np.uint8):
        if node.attrs.get('MATLAB_class', b'') == b'char':
            return _to_str(arr)

    if isinstance(arr, np.ndarray) and arr.ndim >= 2:
        arr = arr.T                        # column-major -> row-major
    return arr


def _to_str(arr: np.ndarray) -> str:
    return ''.join(chr(int(c)) for c in np.asarray(arr).ravel() if int(c) != 0)


def scalar(v, default: float = float('nan')) -> float:
    """First element of a 1x1 MATLAB array, as a float."""
    a = np.asarray(v, dtype=float).ravel()
    return float(a[0]) if a.size else default


def vector(v) -> np.ndarray:
    return np.asarray(v, dtype=float).ravel()


def as_text(v) -> str:
    """A MATLAB string field that may already have been decoded."""
    if isinstance(v, str):
        return v
    try:
        return _to_str(np.asarray(v))
    except Exception:
        return ''
