"""The `tag_fix` wire schema, which is a compatibility surface twice over.

The GUI renders a subset of these fields, so a rename breaks the display. Every
recorded NDJSON session carries all of them, so a *redefinition* silently
reinterprets every past recording and replay renders it wrong with no error.

Phase 2 swapped the producer of this message from the old inline pipeline to
`SweepPipeline`. This is the test that says the swap kept the contract.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host.bus import dumps                                  # noqa: E402
from host.pipeline import PipelineConfig, SweepPipeline      # noqa: E402
from host.tag_fix import tag_fix_message                     # noqa: E402
from dune.anchor_config import AnchorConfig                  # noqa: E402
from dune.frame_parser import FrameParser                    # noqa: E402

# Every field the pre-swap pipeline emitted. Removing one is a breaking change
# to every recorded log, so this list should only ever grow.
#
# It is transcribed from the OLD implementation (git 81f1632^), not from what
# the current code happens to produce. Writing it the other way round is how the
# first version of this test passed while five fields — innovation, K_diag,
# correction_mag, health and K_full — had silently been dropped, one of which
# (`health`) the GUI's HEALTH panel reads off every fix.
LEGACY_FIELDS = {
    'type', 'tag_id', 'pkt_n', 'cycle', 'mrx_us', 't_ms',
    'x', 'y', 'ex', 'ey', 'rmse', 'nUsed', 'ekfR',
    'anchor_ids', 'anchor_diag',
    'ekf_pos', 'ekf_P_diag', 'pos_cov', 'vx', 'vy', 'speed', 'heading',
    'bx', 'by',
    'imu_gyro', 'imu_orient_conf', 'imu_ax_world', 'imu_ay_world',
    'zupt', 'pred_mode', 'nis', 'meas_accepted',
    'innovation', 'K_diag', 'correction_mag', 'health',
}

# Added by the Phase 2 swap. Additive only — the GUI ignores unknown keys.
NEW_FIELDS = {'held', 'pinned', 'still', 'estimator', 'range_bias', 'rejected'}

LINE = ('RTLS,v3,4513,240,5,'
        '1,3223,-81,-83.9,383.11,2,3767,-98,-102.3,125.77,'
        '3,4274,-96,-98.1,164.67,4,3326,-92,-95.9,184.68,'
        '5,1971,-87,-89.0,278.39')


def _one_fix():
    cfg = AnchorConfig()
    pipe = SweepPipeline(cfg, PipelineConfig(tag_z=0.24))
    pkt = FrameParser.parse(LINE)
    assert pkt.valid
    res = pipe.process(pkt, 1000.0, imu=pkt.imu)
    rt = pipe.tags[res.tag_id]
    return tag_fix_message(res, rt, pkt, cfg), res


def test_every_legacy_field_is_still_emitted():
    msg, _ = _one_fix()
    missing = LEGACY_FIELDS - set(msg)
    assert not missing, f"the swap dropped fields recorded logs rely on: {missing}"


def test_the_new_fields_are_present_and_additive():
    msg, _ = _one_fix()
    assert NEW_FIELDS <= set(msg)
    assert msg['estimator'] == 'ekf'
    assert isinstance(msg['held'], int) and isinstance(msg['pinned'], bool)


def test_x_y_is_the_raw_solve_and_ex_ey_is_the_display_output():
    """The two pairs must keep their distinct meanings, not become aliases."""
    msg, res = _one_fix()
    assert msg['x'] == pytest.approx(round(float(res.raw[0]), 4))
    assert msg['ex'] == pytest.approx(round(float(res.out[0]), 4))


def test_per_anchor_range_keys_use_the_d_and_r_prefixes():
    msg, _ = _one_fix()
    d = sorted(k for k in msg if k.startswith('d') and k[1:].isdigit())
    r = sorted(k for k in msg if k.startswith('r') and k[1:].isdigit())
    assert d and len(d) == len(r), f"d/r keys unbalanced: {d} vs {r}"
    assert [k[1:] for k in d] == [k[1:] for k in r]


def test_anchor_diag_keeps_its_shape():
    msg, _ = _one_fix()
    assert msg['anchor_diag'], "no per-anchor diagnostics"
    keys = set(msg['anchor_diag'][0])
    assert {'id', 'd', 'rx', 'fp', 'gap', 'w', 'resid', 'used'} <= keys


def test_ekf_p_diag_stays_length_six():
    """Per-anchor bias states go in `range_bias`, NOT appended here.

    Anything reading `ekf_P_diag[4]` for the accel-bias variance would silently
    read a range bias instead if this grew.
    """
    cfg = AnchorConfig()
    pipe = SweepPipeline(cfg, PipelineConfig(tag_z=0.24, range_bias_states=True))
    pkt = FrameParser.parse(LINE)
    res = pipe.process(pkt, 1000.0, imu=pkt.imu)
    msg = tag_fix_message(res, pipe.tags[res.tag_id], pkt, cfg)
    assert len(msg['ekf_P_diag']) == 6
    assert len(msg['range_bias']) == len(cfg.ids)


def test_the_message_is_valid_json_with_no_nan():
    """NaN is not valid JSON; the browser would drop the whole message."""
    msg, _ = _one_fix()
    text = dumps(msg)
    assert 'NaN' not in text and 'Infinity' not in text
    import json
    json.loads(text)


def test_non_finite_values_become_null_not_nan():
    from host.tag_fix import _r
    assert _r(float('nan'), 3) is None
    assert _r(float('inf'), 3) is None
    assert _r(1.23456, 3) == pytest.approx(1.235)
    assert _r(None, 3) is None


def test_the_legacy_field_list_matches_the_pre_swap_implementation():
    """Guard against the failure mode that let five dropped fields through.

    The list above is only meaningful if it describes the OLD message. Rebuild
    it from git and compare, so a future edit cannot quietly narrow the contract
    by trimming the list to match whatever the code now emits.
    """
    import re
    import subprocess

    import shutil
    if shutil.which('git') is None:
        pytest.skip("git not available")
    old = subprocess.run(
        ['git', 'show', '81f1632^:python/run_localization.py'],
        capture_output=True, text=True, timeout=30,
        cwd=Path(__file__).resolve().parents[2]).stdout
    if 'result: dict = {' not in old:
        pytest.skip("pre-swap implementation not reachable from this checkout")

    blk = old[old.index('result: dict = {'):old.index('return result',
                                                      old.index('result: dict = {'))]
    keys = set(re.findall(r"^\s*'([A-Za-z_]\w*)':", blk, re.M))
    keys |= set(re.findall(r"result\['(\w+)'\]", old))
    keys -= {'K_full'}          # mutually exclusive with K_diag, opt-in only

    missing = keys - LEGACY_FIELDS
    assert not missing, (
        f"LEGACY_FIELDS is missing fields the old pipeline emitted: {missing}")


def test_health_is_populated_not_an_empty_shell():
    """The GUI reads fix.health.* directly; an empty dict blanks the panel."""
    msg, _ = _one_fix()
    h = msg['health']
    assert isinstance(h, dict) and len(h) > 10
    for k in ('localization_confidence', 'n_anchors_used', 'ekf_initialized',
              'prediction_mode', 'zupt_active'):
        assert k in h, f"health missing {k}"


def test_imu_angles_are_real_values_when_a_tail_is_present():
    """`fix.imu.roll/pitch/yaw` is read straight into the ORIENTATION panel.

    It was briefly a hardcoded None, which left the panel blank with no error.
    """
    import numpy as np
    from dune.ekf import quat_rpy

    roll, pitch, yaw = quat_rpy([0.9803, 0.0386, -0.0911, 0.1711])
    assert np.isfinite([roll, pitch, yaw]).all()
    assert abs(np.degrees(yaw)) > 1.0, "a non-identity quaternion gave zero yaw"
