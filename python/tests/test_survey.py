"""Anchor self-survey reconstruction, against MATLAB's committed output.

`matlab/results/survey_runs/` holds captures **and** the `anchors_*.json` that
`build_survey_anchors.m` produced from them — a self-contained oracle needing no
MATLAB at test time.

The comparison is on **shape**, not position, and that is the whole point rather
than a concession. A set of pairwise distances fixes shape but not absolute pose
or handedness, so MDS output is aligned onto whatever `anchors.json` was current
when MATLAB ran — a reference this repo does not always still contain. Comparing
inter-anchor distances is invariant to that choice, so it tests the
reconstruction instead of testing which file happened to be on disk.

That distinction is not academic: aligned against the wrong reference these
reconstructions differ by *metres*, while their shapes are identical to 0.000 mm.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rtls.anchor_config import AnchorConfig                       # noqa: E402
from rtls.survey import (align_to, apply_delays, build_survey_anchors,  # noqa: E402
                         fit_anchor_delays, mds_2d, read_survey)

REPO = Path(__file__).resolve().parents[2]
SURVEY_DIR = REPO / 'matlab' / 'results' / 'survey_runs'
CONFIG = REPO / 'matlab' / 'config'

pytestmark = pytest.mark.skipif(not SURVEY_DIR.is_dir(),
                                reason="no survey_runs/ in this checkout")


def _pairwise(pts: np.ndarray) -> np.ndarray:
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    return d[np.triu_indices(len(pts), 1)]


def _cases(tag: str):
    for out in sorted(SURVEY_DIR.glob(f'*_anchors_{tag}.json')):
        capture = SURVEY_DIR / f"{out.name.split('_anchors_')[0]}.txt"
        if capture.exists():
            yield capture, out


# ── parsing ───────────────────────────────────────────────────────────────────

def test_a_capture_parses_into_a_complete_distance_matrix():
    caps = sorted(SURVEY_DIR.glob('survey_*.txt'))
    assert caps, "no survey captures committed"
    c = read_survey(caps[0])
    assert c.ids.size >= 4
    assert c.complete, f"missing pairs: {c.missing_pairs()}"
    assert np.allclose(c.D, c.D.T), "distance matrix is not symmetric"
    assert np.allclose(np.diag(c.D), 0.0)
    # n(n-1)/2 pairs for n anchors
    n = c.ids.size
    assert c.n_pairs >= n * (n - 1) // 2


def test_an_incomplete_survey_is_refused_rather_than_guessed(tmp_path):
    """A missing pair makes MDS silently invent geometry; refuse instead."""
    p = tmp_path / 'partial.txt'
    p.write_text("SURVEY_BEGIN,v1,3\n"
                 "SURVEY,v1,1,2,1000,50\n"
                 "SURVEY,v1,1,3,2000,50\n"
                 "SURVEY_DONE,v1\n")
    c = read_survey(p)
    assert not c.complete and (2, 3) in c.missing_pairs()
    with pytest.raises(ValueError, match='incomplete'):
        build_survey_anchors(c, np.zeros((3, 2)))


def test_a_file_with_no_survey_rows_is_rejected(tmp_path):
    p = tmp_path / 'empty.txt'
    p.write_text("SURVEY_BEGIN,v1,4\nSURVEY_DONE,v1\n")
    with pytest.raises(ValueError, match='no SURVEY'):
        read_survey(p)


# ── reconstruction, against MATLAB ────────────────────────────────────────────

def test_raw_reconstruction_matches_matlabs_shape_exactly():
    cases = list(_cases('raw'))
    assert len(cases) >= 5, f"only {len(cases)} raw cases"

    for capture, out in cases:
        ref = json.load(open(out))
        ids = [a['id'] for a in ref['anchors']]
        R = np.array([[a['x'], a['y']] for a in ref['anchors']])

        c = read_survey(capture, ids=ids)
        doc = build_survey_anchors(c, R, layout_base=capture.stem)
        X = np.array([[a['x'], a['y']] for a in doc['anchors']])

        err = float(np.abs(_pairwise(X) - _pairwise(R)).max())
        assert err < 1e-4, f"{out.name}: shape differs by {err * 1000:.3f} mm"


def test_reconstruction_reproduces_matlabs_positions_given_its_reference():
    """Exact to the last decimal place when aligned onto the same file.

    Only some cases can be checked: MATLAB aligned each run onto whichever
    anchors.json was current at the time, and the layouts used after the
    2026-07-25 re-survey are not in this repo.
    """
    exact = 0
    for capture, out in list(_cases('raw')) + list(_cases('corrected')):
        ref = json.load(open(out))
        ids = [a['id'] for a in ref['anchors']]
        R = np.array([[a['x'], a['y']] for a in ref['anchors']])
        corrected = out.name.endswith('_corrected.json')

        for anchors_file in sorted(CONFIG.glob('anchors*.json')):
            try:
                A = AnchorConfig.from_json(str(anchors_file))
            except Exception:
                continue
            if list(A.ids) != list(ids):
                continue
            c = read_survey(capture, ids=ids)
            doc = build_survey_anchors(c, A.coords[:, :2], corrected=corrected,
                                       layout_base=capture.stem)
            X = np.array([[a['x'], a['y']] for a in doc['anchors']])
            if np.abs(X - R).max() < 1e-4:
                exact += 1
                break

    assert exact >= 4, f"only {exact} runs reproduced MATLAB's positions exactly"


def test_the_output_is_a_loadable_anchors_document(tmp_path):
    capture, out = next(_cases('raw'))
    ref = json.load(open(out))
    ids = [a['id'] for a in ref['anchors']]
    R = np.array([[a['x'], a['y']] for a in ref['anchors']])

    doc = build_survey_anchors(read_survey(capture, ids=ids), R, z=0.24,
                               layout_base=capture.stem)
    p = tmp_path / 'anchors.json'
    p.write_text(json.dumps(doc))
    A = AnchorConfig.from_json(str(p))          # must round-trip
    assert list(A.ids) == ids
    assert A.dim == 2
    assert all(a['z'] == 0.24 for a in doc['anchors'])
    assert 'self_survey' in doc['layout']


# ── the maths ─────────────────────────────────────────────────────────────────

def test_mds_recovers_a_known_constellation_up_to_pose():
    truth = np.array([[0.0, 0.0], [6.0, 0.0], [6.0, 3.0], [0.0, 3.0]])
    D = np.linalg.norm(truth[:, None, :] - truth[None, :, :], axis=2)
    X, rms = align_to(mds_2d(D), truth)
    assert rms < 1e-9, f"exact distances did not reconstruct exactly: {rms}"


def test_alignment_resolves_handedness():
    """Distances cannot tell a shape from its mirror; the flip search must."""
    truth = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [0.0, 2.0]])
    D = np.linalg.norm(truth[:, None, :] - truth[None, :, :], axis=2)
    mirrored = mds_2d(D)
    mirrored[:, 1] *= -1
    X, rms = align_to(mirrored, truth)
    assert rms < 1e-9, "a mirrored reconstruction was not corrected"


def test_alignment_does_not_fit_scale():
    """Fitting scale would absorb ranging bias and flatter the result.

    A constellation measured 10% too large must stay 10% too large.
    """
    truth = np.array([[0.0, 0.0], [6.0, 0.0], [6.0, 3.0], [0.0, 3.0]])
    big = truth * 1.10
    D = np.linalg.norm(big[:, None, :] - big[None, :, :], axis=2)
    X, rms = align_to(mds_2d(D), truth)
    assert rms > 0.1, "scale was silently absorbed"
    assert np.abs(_pairwise(X) / _pairwise(truth) - 1.10).max() < 1e-9


def test_per_anchor_delay_fit_recovers_injected_biases():
    truth = np.array([[0.0, 0.0], [6.0, 0.0], [6.0, 3.0], [0.0, 3.0]])
    D = np.linalg.norm(truth[:, None, :] - truth[None, :, :], axis=2)
    b_true = np.array([0.05, -0.02, 0.10, 0.00])
    Dm = D.copy()
    n = len(truth)
    for i in range(n):
        for j in range(i + 1, n):
            Dm[i, j] = Dm[j, i] = D[i, j] + b_true[i] + b_true[j]

    b, ticks, rms = fit_anchor_delays(Dm, truth)
    assert np.allclose(b, b_true, atol=1e-9), f"got {b}"
    assert rms < 1e-9
    assert np.allclose(apply_delays(Dm, b), D, atol=1e-9)
    # Reported in DW1000 ticks as well, since that is what gets pushed to NVS.
    assert np.allclose(ticks, b_true / 0.00469, rtol=1e-9)


# ── tag bias ──────────────────────────────────────────────────────────────────

STATIC_CAPTURE = REPO / 'matlab' / 'results' / 'static_capture.txt'
TAG_BIAS_MAT = REPO / 'matlab' / 'results' / 'tag_bias_fit.mat'
# The geometry anchor_bias.json was fitted against — it names the layout in its
# own `note` field ("rect 6.3x3.0, A5 top edge at x=3.3").
ANCHORS_RECT = REPO / 'matlab' / 'config' / 'anchors_rect6p3x3p0_2026-07-01.json'


@pytest.mark.skipif(not (STATIC_CAPTURE.exists() and TAG_BIAS_MAT.exists()
                        and ANCHORS_RECT.exists()),
                    reason="static capture / committed result not in this checkout")
def test_tag_bias_fit_reproduces_matlab_exactly():
    """Both input and MATLAB's output are committed, so this is exact.

    The alternating fit is deterministic, so unlike the MHE there is no reason to
    settle for a metric-level match here.
    """
    scipy_io = pytest.importorskip('scipy.io')
    import numpy as np

    from rtls.anchor_config import AnchorConfig
    from rtls.frame_parser import FrameParser
    from rtls.tag_bias import fit_tag_bias, load_anchor_bias

    ref = {int(o.tag): (np.asarray(o.pos, float), float(o.biasM))
           for o in np.atleast_1d(
               scipy_io.loadmat(str(TAG_BIAS_MAT), squeeze_me=True,
                                struct_as_record=False)['out'])}
    assert len(ref) == 2, f"expected two tags in the reference, got {list(ref)}"

    sweeps = [s for s in (FrameParser.parse(l) for l in
                          STATIC_CAPTURE.read_text(errors='replace').splitlines())
              if s.valid]
    cfg = AnchorConfig.from_json(str(ANCHORS_RECT))
    results = fit_tag_bias(sweeps, cfg, anchor_bias=load_anchor_bias(), tag_z=0.22)

    assert {r.tag for r in results} == set(ref)
    for r in results:
        ref_pos, ref_bias = ref[r.tag]
        assert np.linalg.norm(r.pos - ref_pos) < 1e-4, (
            f"tag {r.tag}: position off by "
            f"{1000 * np.linalg.norm(r.pos - ref_pos):.3f} mm")
        assert abs(r.bias_m - ref_bias) < 1e-6, (
            f"tag {r.tag}: bias off by {1000 * (r.bias_m - ref_bias):+.3f} mm")


@pytest.mark.skipif(not STATIC_CAPTURE.exists(), reason="no static capture")
def test_the_anchor_bias_file_is_what_makes_it_match():
    """Without it the fit is ~250 mm and ~40 mm out — a plausible wrong answer.

    Worth pinning: it is the difference between reproducing MATLAB and quietly
    reporting a bias that looks reasonable and is not.
    """

    from rtls.anchor_config import AnchorConfig
    from rtls.frame_parser import FrameParser
    from rtls.tag_bias import fit_tag_bias, load_anchor_bias

    ab = load_anchor_bias()
    assert ab, "anchor_bias.json did not load"
    assert set(ab) >= {1, 2, 3, 4, 5}

    sweeps = [s for s in (FrameParser.parse(l) for l in
                          STATIC_CAPTURE.read_text(errors='replace').splitlines())
              if s.valid]
    cfg = AnchorConfig.from_json(str(ANCHORS_RECT))
    with_ab = {r.tag: r.bias_m for r in
               fit_tag_bias(sweeps, cfg, anchor_bias=ab, tag_z=0.22)}
    without = {r.tag: r.bias_m for r in
               fit_tag_bias(sweeps, cfg, anchor_bias=None, tag_z=0.22)}
    assert any(abs(with_ab[t] - without[t]) > 0.02 for t in with_ab), (
        "anchor bias made no difference — is it being applied at all?")


@pytest.mark.skipif(not STATIC_CAPTURE.exists(), reason="no static capture")
def test_tag_bias_result_is_json_safe():
    from host.bus import dumps
    from rtls.anchor_config import AnchorConfig
    from rtls.frame_parser import FrameParser
    from rtls.tag_bias import fit_tag_bias, load_anchor_bias

    sweeps = [s for s in (FrameParser.parse(l) for l in
                          STATIC_CAPTURE.read_text(errors='replace').splitlines())
              if s.valid]
    cfg = AnchorConfig.from_json(str(ANCHORS_RECT))
    payload = {'type': 'job_done', 'result': {
        'tags': [r.to_dict() for r in
                 fit_tag_bias(sweeps, cfg, anchor_bias=load_anchor_bias())]}}
    text = dumps(payload)
    assert 'NaN' not in text and 'Infinity' not in text
    json.loads(text)
