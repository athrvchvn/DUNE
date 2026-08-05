"""
calibration_manager.py — Tier-1 hardware antenna-delay traceability + loader.

After the 2-tier redesign (see docs/calibration.md):

  - Tier-1 (hardware antenna delay) is set on the tag via the HWCALIB binary search and
    stored in each anchor's NVS. This manager only RECORDS the resulting delays in
    calibration.json (`level1_hardware`) for traceability. It does NOT apply any software
    correction and never pushes delays itself.

  - Tier-2 (systematic per-anchor error) is handled by the first-path-power
    correction (`dune/range_correction.py` + `config/range_correction.json`), NOT here.
    The spatial error map it replaced is retired; only the captured
    `config/error_map.json` survives, as a recorded result rather than a code path.

The old Level-2 software bias (per-anchor constant/linear) and the b0→Δticks antenna-delay
auto-push were removed: together they caused a double-correction, and the push had an
inverted sign. Old v2/v3 calibration.json files still load, but their bias blocks are
ignored (logged once).
"""

import json
import hashlib
from pathlib import Path
from datetime import datetime


class CalibrationManager:
    """Tier-1 hardware-delay traceability + calibration.json loader (no software bias)."""

    def __init__(self, cfg, cal_path, tag_ip: str = '255.255.255.255', tag_port: int = 4100):
        self.cfg       = cfg
        self.cal_path  = Path(cal_path)
        self._tag_ip   = tag_ip     # used by run_localization.send_to_tag()
        self._tag_port = tag_port
        # {aid: {'antenna_delay_ticks': int, 'calibrated_at': iso8601}}
        self._level1_hw: dict[int, dict] = {}
        self.cal_loaded = False

    # ── Anchor-config hash (shared convention with the error map) ─────────────────────

    def anchor_config_hash(self) -> str:
        """sha256(anchors.json bytes)[:16]; '' if the file is missing."""
        anchors_path = self.cal_path.parent / 'anchors.json'
        if not anchors_path.exists():
            return ''
        return hashlib.sha256(anchors_path.read_bytes()).hexdigest()[:16]

    # ── Load ──────────────────────────────────────────────────────────────────────────

    def try_load(self) -> bool:
        """
        Load calibration.json (v2/v3/v4). Consumes ONLY `level1_hardware`. Old Level-2
        software biases (`tier1` in v2, `level2_systematic` in v3) are parsed-then-ignored.
        Returns True if any Level-1 delays were loaded.
        """
        if not self.cal_path.exists():
            return False
        try:
            data = json.loads(self.cal_path.read_text())
        except Exception as e:
            print(f"[CAL] Failed to parse calibration.json: {e}")
            return False

        ver = data.get('version', 0)
        boards = data.get('level1_hardware', {}).get('boards', {})
        for key, binfo in boards.items():
            try:
                aid = int(key, 0)          # "0x01" or "1" → int
            except (TypeError, ValueError):
                continue
            # Preserve every field verbatim (delay/timestamp plus, if present,
            # true/final/residual mm, git_commit, iteration_history) so a
            # load-then-save round-trip never silently drops provenance.
            entry = dict(binfo)
            entry['antenna_delay_ticks'] = int(binfo.get('antenna_delay_ticks', 0))
            entry['calibrated_at']       = binfo.get('calibrated_at', '')
            self._level1_hw[aid] = entry

        if ('tier1' in data) or ('level2_systematic' in data):
            print("[CAL] Legacy calibration.json detected — Level-2 software bias ignored "
                  "(now handled by the spatial error map).")

        self.cal_loaded = bool(self._level1_hw)
        if self.cal_loaded:
            print(f"[CAL] Level-1 antenna-delay traceability loaded for anchors "
                  f"{sorted(self._level1_hw.keys())} (v{ver})")
        return self.cal_loaded

    # ── HWCALIB result recording (Tier-1) ─────────────────────────────────────────────

    def record_hwcalib_result(self, anchor_id: int, delay_ticks: int, *,
                              true_mm: int | None = None,
                              final_mean_mm: int | None = None,
                              residual_mm: int | None = None,
                              git_commit: str | None = None,
                              iteration_history: list | None = None) -> None:
        """Stamp a per-anchor antenna delay produced by the on-tag HWCALIB binary
        search, plus (when available) the verification residual and provenance
        needed to reproduce the run later — the calibration's own QA record."""
        entry = {
            'antenna_delay_ticks': int(delay_ticks),
            'calibrated_at':       datetime.now().isoformat(timespec='seconds'),
        }
        if true_mm is not None:            entry['true_mm'] = int(true_mm)
        if final_mean_mm is not None:       entry['final_mean_mm'] = int(final_mean_mm)
        if residual_mm is not None:         entry['residual_mm'] = int(residual_mm)
        if git_commit is not None:          entry['git_commit'] = git_commit
        if iteration_history is not None:   entry['iteration_history'] = iteration_history
        self._level1_hw[int(anchor_id)] = entry
        self.cal_loaded = True

    def save_level1(self) -> dict:
        """Persist accumulated Level-1 delays to calibration.json v4."""
        try:
            self._write_v4()
            print(f"[CAL] Saved Level-1 traceability → {self.cal_path}")
            return {'type': 'CALIB_SAVED', 'level1_hardware': self._level1_summary()}
        except Exception as e:
            print(f"[CAL] Save failed: {e}")
            return {'type': 'CALIB_ERROR', 'msg': str(e)}

    def _write_v4(self) -> None:
        em_path = self.cal_path.parent / 'error_map.json'
        em_hash = (hashlib.sha256(em_path.read_bytes()).hexdigest()[:16]
                   if em_path.exists() else '')

        boards = {
            f"0x{aid:02X}": info    # already carries delay/timestamp + any QA/provenance fields
            for aid, info in sorted(self._level1_hw.items())
        }

        doc = {
            'version':            4,
            'schema':             'uwb_rtls_calibration_v4',
            'created_at':         datetime.now().isoformat(timespec='seconds'),
            'anchor_config_hash': self.anchor_config_hash(),
            'level1_hardware': {
                'method': 'hwcalib_binary_search',
                'note':   ('Antenna delay stored in anchor NVS (set by HWCALIB). '
                           'Traceability only — NOT applied by the host pipeline.'),
                'boards': boards,
            },
            'level2_spatial': {
                'note':           'Per-anchor systematic error handled by the spatial error map.',
                'error_map_path': 'error_map.json',
                'error_map_hash': em_hash,
            },
        }
        self.cal_path.write_text(json.dumps(doc, indent=2) + "\n")

    # ── Anchor layout change ──────────────────────────────────────────────────────────

    def on_anchors_updated(self, cfg) -> None:
        """Anchors moved. Antenna delays are per-board (layout-independent) so they are
        kept; the spatial error map is dropped by the pipeline and must be recaptured."""
        self.cfg = cfg
        print("[CAL] Anchor positions updated — recapture the spatial error map.")

    # ── Status (for the WS 'config' message) ──────────────────────────────────────────

    def status_msg(self) -> dict:
        return {
            'cal_loaded':      self.cal_loaded,
            'level1_hardware': self._level1_summary(),
        }

    def _level1_summary(self) -> dict:
        return {f"0x{aid:02X}": info['antenna_delay_ticks']
                for aid, info in sorted(self._level1_hw.items())}
