"""
Anchor coordinates + room geometry.

Reads config/anchors.json (same format as the MATLAB version).
"""

import json
import numpy as np
from typing import Tuple, List


class AnchorConfig:
    def __init__(self, ids=None, coords=None, dim=2, bounds=None):
        if ids is None:
            # Built-in default: 4-anchor square, 4 m side, centred at origin.
            # A1=(-2,-2), A2=(2,-2), A3=(2,2), A4=(-2,2) — all on z=0 ground plane.
            self.dim    = 2
            self.ids    = [1, 2, 3, 4]
            self.coords = np.array([
                [-2.0, -2.0, 0.0],
                [ 2.0, -2.0, 0.0],
                [ 2.0,  2.0, 0.0],
                [-2.0,  2.0, 0.0],
            ])
            self.bounds = [-2.5, 2.5, -2.5, 2.5, 0.0, 3.0]
        else:
            self.dim    = dim
            self.ids    = list(ids)
            self.coords = np.array(coords, dtype=float)
            self.bounds = bounds if bounds else [0, 5, 0, 5, 0, 3]

    @classmethod
    def from_json(cls, path: str) -> 'AnchorConfig':
        with open(path) as f:
            j = json.load(f)
        anchors = j['anchors']
        ids    = [a['id'] for a in anchors]
        coords = [[a['x'], a['y'], a.get('z', 0.0)] for a in anchors]
        dim    = j.get('dim', 2)
        bounds = j.get('bounds', [0, 5, 0, 5, 0, 3])
        return cls(ids, coords, dim, bounds)

    def coords_for(self, query_ids: List[int]) -> Tuple[np.ndarray, List[bool]]:
        """Map anchor ids → coordinates. Returns (Nxdim array, found mask)."""
        A     = np.zeros((len(query_ids), self.dim))
        found = [False] * len(query_ids)
        for i, qid in enumerate(query_ids):
            if qid in self.ids:
                idx = self.ids.index(qid)
                A[i] = self.coords[idx, :self.dim]
                found[i] = True
        return A, found

    def centroid(self) -> np.ndarray:
        return np.mean(self.coords[:, :self.dim], axis=0)
