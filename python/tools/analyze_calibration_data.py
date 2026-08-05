#!/usr/bin/env python3
"""
analyze_calibration_data.py — Phase 1 bias characterisation analysis.

Reads a CSV produced by collect_calibration_data.py and:
  1. Computes per-session statistics with outlier rejection
  2. Evaluates Phase 1 go/no-go Criteria 1, 2, 3
  3. Generates a 4-panel diagnostic figure
  4. Prints a written report with verdicts

Usage:
    python analyze_calibration_data.py bias_anchor1_*.csv
    python analyze_calibration_data.py --output-dir plots/
"""

import sys
import csv
import math
import argparse
import textwrap
from pathlib import Path
from collections import defaultdict
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
import matplotlib.ticker as mticker

# ── Constants ────────────────────────────────────────────────────────────────

IQR_MULTIPLIER   = 3.0    # samples outside median ± 3×IQR are outliers
PLACEMENT_ERR_MM = 5.0    # tape-measure placement uncertainty (±mm)
SIGMA_SINGLE_MM  = 40.0   # expected single-measurement noise (mm) for DW1000 110kbps
CRITERIA3_SWING  = 20.0   # Criterion 3: bias swing > 20 mm required
CRITERIA2_AGREE  = 5.0    # Criterion 2: session-to-session agreement < 5 mm
REPORT_WIDTH     = 72

PALETTE = {
    'S1_1m':  '#1f77b4',
    'S2_1m':  '#aec7e8',
    'S3_3m':  '#d62728',
    'S4_3m':  '#f5a9a9',
}
DIST_COLORS = {'1.0': '#1f77b4', '3.0': '#d62728'}


# ── Data loading & cleaning ──────────────────────────────────────────────────

def load_csv(path: Path) -> dict:
    """
    Load CSV; return dict keyed by (session, true_dist_m) → {'d_mm': [], 'q': []}.
    """
    raw = defaultdict(lambda: {'d_mm': [], 'q': []})
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            key = (int(row['session']), float(row['true_distance_m']))
            raw[key]['d_mm'].append(int(row['d_mm']))
            raw[key]['q'].append(float(row['q_dbm']))
    return dict(raw)


def reject_outliers(vals: list, k: float = IQR_MULTIPLIER):
    """Return cleaned array after IQR-based outlier rejection."""
    a = np.array(vals, dtype=float)
    q1, q3 = np.percentile(a, 25), np.percentile(a, 75)
    iqr = q3 - q1
    if iqr < 1:          # degenerate distribution (all same value)
        return a
    lo, hi = q1 - k * iqr, q3 + k * iqr
    return a[(a >= lo) & (a <= hi)]


def compute_stats(d_mm_raw: list, q_raw: list, true_dist_mm: float) -> dict:
    d = reject_outliers(d_mm_raw)
    q = np.array(q_raw, dtype=float)
    n_raw = len(d_mm_raw)
    n_clean = len(d)
    n_out = n_raw - n_clean

    mean  = float(np.mean(d))
    std   = float(np.std(d, ddof=1))
    bias  = mean - true_dist_mm
    sigma_mean = std / math.sqrt(n_clean)
    ci95  = 1.96 * math.sqrt(sigma_mean**2 + PLACEMENT_ERR_MM**2)

    return {
        'n_raw': n_raw,
        'n_clean': n_clean,
        'n_outliers': n_out,
        'mean_mm': mean,
        'std_mm': std,
        'bias_mm': bias,
        'sigma_mean_mm': sigma_mean,
        'ci95_mm': ci95,
        'mean_q_dbm': float(np.mean(q)),
        'std_q_dbm': float(np.std(q, ddof=1)),
        'd_clean': d,
    }


# ── Criteria evaluation ──────────────────────────────────────────────────────

def evaluate_criteria(stats_by_key: dict) -> dict:
    """
    Evaluate Phase 1 Criteria 1, 2, 3.

    stats_by_key : {(session, true_dist_m): stats_dict}
    Returns dict with criterion verdicts and supporting numbers.
    """
    # Group by distance
    by_dist: dict = defaultdict(list)
    for (sess, dist), s in stats_by_key.items():
        by_dist[dist].append({'session': sess, **s})

    results = {}

    # ── Criterion 2 — Repeatability ─────────────────────────────────────────
    c2 = {}
    for dist, sessions in by_dist.items():
        biases = [s['bias_mm'] for s in sessions]
        max_diff = max(biases) - min(biases)
        c2[dist] = {
            'biases': biases,
            'max_diff_mm': max_diff,
            'pass': max_diff <= CRITERIA2_AGREE,
        }
    results['criterion2'] = c2

    all_c2_pass = all(v['pass'] for v in c2.values())
    results['criterion2_overall'] = all_c2_pass

    # ── Criterion 1 — Statistical significance ──────────────────────────────
    mean_bias_by_dist = {}
    ci95_by_dist = {}
    for dist, sessions in by_dist.items():
        # Combined stats from all sessions at this distance
        all_d = np.concatenate([s['d_clean'] for s in sessions])
        true_mm = dist * 1000.0
        mean_b = float(np.mean(all_d)) - true_mm
        sigma_m = float(np.std(all_d, ddof=1)) / math.sqrt(len(all_d))
        ci95 = 1.96 * math.sqrt(sigma_m**2 + PLACEMENT_ERR_MM**2)
        mean_bias_by_dist[dist] = mean_b
        ci95_by_dist[dist] = ci95

    dists = sorted(mean_bias_by_dist.keys())
    if len(dists) >= 2:
        bias_diff = abs(mean_bias_by_dist[dists[0]] - mean_bias_by_dist[dists[1]])
        # Combined uncertainty: sqrt(ci95_A^2 + ci95_B^2)
        combined_unc = math.sqrt(ci95_by_dist[dists[0]]**2 + ci95_by_dist[dists[1]]**2)
        snr = bias_diff / combined_unc if combined_unc > 0 else 0.0
        c1_pass = snr >= 3.0
    else:
        bias_diff = 0.0
        combined_unc = 0.0
        snr = 0.0
        c1_pass = False

    results['criterion1'] = {
        'mean_bias_by_dist': mean_bias_by_dist,
        'ci95_by_dist': ci95_by_dist,
        'bias_diff_mm': bias_diff,
        'combined_unc_mm': combined_unc,
        'snr': snr,
        'pass': c1_pass,
    }

    # ── Criterion 3 — Bias swing vs noise floor ──────────────────────────────
    biases = list(mean_bias_by_dist.values())
    swing = max(biases) - min(biases) if biases else 0.0
    results['criterion3'] = {
        'swing_mm': swing,
        'threshold_mm': CRITERIA3_SWING,
        'pass': swing >= CRITERIA3_SWING,
    }

    return results


# ── Plotting ─────────────────────────────────────────────────────────────────

def make_figure(stats_by_key: dict, criteria: dict, out_path: Path, csv_path: Path):
    fig = plt.figure(figsize=(14, 10))
    fig.patch.set_facecolor('#fafafa')
    gs = GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38,
                  left=0.07, right=0.97, top=0.90, bottom=0.08)

    ax_box  = fig.add_subplot(gs[0, 0])   # distribution violins
    ax_bias = fig.add_subplot(gs[0, 1])   # bias summary
    ax_rep  = fig.add_subplot(gs[0, 2])   # repeatability
    ax_ts   = fig.add_subplot(gs[1, :2])  # time series
    ax_q    = fig.add_subplot(gs[1, 2])   # rxPower box

    label_map = {
        (1, 1.0): 'S1 · 1 m', (2, 1.0): 'S2 · 1 m',
        (3, 3.0): 'S3 · 3 m', (4, 3.0): 'S4 · 3 m',
    }
    color_map = {
        (1, 1.0): PALETTE['S1_1m'], (2, 1.0): PALETTE['S2_1m'],
        (3, 3.0): PALETTE['S3_3m'], (4, 3.0): PALETTE['S4_3m'],
    }

    sorted_keys = sorted(stats_by_key.keys())

    # ── Panel A: Violin / Distribution ──────────────────────────────────────
    positions = list(range(1, len(sorted_keys) + 1))
    for pos, key in zip(positions, sorted_keys):
        s = stats_by_key[key]
        d = s['d_clean']
        true_mm = key[1] * 1000.0
        bias_vals = d - true_mm
        parts = ax_box.violinplot([bias_vals], positions=[pos], widths=0.6,
                                  showmedians=True, showextrema=False)
        for pc in parts['bodies']:
            pc.set_facecolor(color_map[key])
            pc.set_alpha(0.65)
        parts['cmedians'].set_color('black')
        parts['cmedians'].set_linewidth(1.5)

    ax_box.axhline(0, color='grey', lw=0.8, ls='--')
    ax_box.set_xticks(positions)
    ax_box.set_xticklabels([label_map[k] for k in sorted_keys], fontsize=8)
    ax_box.set_ylabel('Bias (mm)')
    ax_box.set_title('A — Bias distributions', fontsize=10, fontweight='bold')
    ax_box.grid(axis='y', alpha=0.3)

    # ── Panel B: Mean bias summary with CI ──────────────────────────────────
    c1 = criteria['criterion1']
    by_dist = c1['mean_bias_by_dist']
    ci95 = c1['ci95_by_dist']
    dists = sorted(by_dist.keys())
    xs = np.arange(len(dists))
    for i, d in enumerate(dists):
        col = DIST_COLORS[f'{d:.1f}']
        ax_bias.errorbar(i, by_dist[d], yerr=ci95[d], fmt='o', color=col,
                         capsize=5, capthick=1.5, markersize=8, lw=1.5,
                         label=f'{d:.1f} m  ({by_dist[d]:+.1f} mm)')

    # Per-session points (lighter)
    for key, s in stats_by_key.items():
        sess, dist = key
        xi = dists.index(dist)
        col = color_map[key]
        ax_bias.plot(xi + (0.12 if sess % 2 == 0 else -0.12), s['bias_mm'],
                     's', color=col, alpha=0.7, markersize=5)

    ax_bias.axhline(0, color='grey', lw=0.8, ls='--')
    ax_bias.set_xticks(xs)
    ax_bias.set_xticklabels([f'{d:.1f} m' for d in dists])
    ax_bias.set_ylabel('Bias (mm)')
    ax_bias.set_title('B — Mean bias ± 95% CI', fontsize=10, fontweight='bold')
    ax_bias.legend(fontsize=8, loc='best')
    ax_bias.grid(alpha=0.3)

    # Annotate pass/fail
    verdict = '✓ PASS' if criteria['criterion1']['pass'] else '✗ FAIL'
    ax_bias.text(0.98, 0.04, f'Crit.1: {verdict}',
                 transform=ax_bias.transAxes, ha='right', fontsize=8,
                 color='green' if criteria['criterion1']['pass'] else 'red',
                 fontweight='bold')

    # ── Panel C: Repeatability ───────────────────────────────────────────────
    c2 = criteria['criterion2']
    for dist_idx, dist in enumerate(sorted(c2.keys())):
        info = c2[dist]
        xs_rep = [dist_idx - 0.15, dist_idx + 0.15]
        col = DIST_COLORS[f'{dist:.1f}']
        for i, (x, b) in enumerate(zip(xs_rep, info['biases'])):
            ax_rep.plot(x, b, 'o', color=col, markersize=9,
                        label=f'S{i+1+dist_idx*2} · {dist:.1f} m')
        ax_rep.plot(xs_rep, info['biases'], '-', color=col, lw=1.5, alpha=0.6)

        # Annotate diff
        mid_y = sum(info['biases']) / 2
        ax_rep.annotate(f"Δ={info['max_diff_mm']:.1f} mm",
                        xy=(dist_idx, mid_y),
                        fontsize=8.5, ha='center', color=col,
                        bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.7))

    ax_rep.axhline(0, color='grey', lw=0.8, ls='--')
    ax_rep.set_xticks(range(len(c2)))
    ax_rep.set_xticklabels([f'{d:.1f} m' for d in sorted(c2.keys())])
    ax_rep.set_ylabel('Bias (mm)')
    ax_rep.set_title('C — Session repeatability', fontsize=10, fontweight='bold')
    ax_rep.grid(alpha=0.3)

    c2_ok = criteria['criterion2_overall']
    ax_rep.text(0.98, 0.04, f"Crit.2: {'✓ PASS' if c2_ok else '✗ FAIL'}",
                transform=ax_rep.transAxes, ha='right', fontsize=8,
                color='green' if c2_ok else 'red', fontweight='bold')

    # ── Panel D: Time series ─────────────────────────────────────────────────
    offset = 0
    for key in sorted_keys:
        s = stats_by_key[key]
        d_raw_all = stats_by_key[key]['d_clean']
        true_mm = key[1] * 1000.0
        bias_ts = d_raw_all - true_mm
        xs_ts = np.arange(offset, offset + len(bias_ts))
        col = color_map[key]
        ax_ts.plot(xs_ts, bias_ts, '.', color=col, alpha=0.25, markersize=2)
        # rolling mean (window=50)
        rm = np.convolve(bias_ts, np.ones(50)/50, mode='valid')
        rm_xs = xs_ts[49:]
        ax_ts.plot(rm_xs, rm, '-', color=col, lw=1.8, label=label_map[key])
        offset += len(bias_ts) + 50  # gap between sessions

    ax_ts.axhline(0, color='grey', lw=0.8, ls='--')
    ax_ts.set_xlabel('Sample index (within session)')
    ax_ts.set_ylabel('Bias (mm)')
    ax_ts.set_title('D — Raw bias time series (dots) + 50-sample rolling mean (line)',
                    fontsize=10, fontweight='bold')
    ax_ts.legend(fontsize=8, loc='upper right', ncol=4)
    ax_ts.grid(alpha=0.3)

    # ── Panel E: rxPower ────────────────────────────────────────────────────
    for pos, key in zip(positions, sorted_keys):
        q_vals = np.array(stats_by_key[key]['q_raw'])
        parts = ax_q.violinplot([q_vals], positions=[pos], widths=0.6,
                                showmedians=True, showextrema=False)
        for pc in parts['bodies']:
            pc.set_facecolor(color_map[key])
            pc.set_alpha(0.65)
        parts['cmedians'].set_color('black')
        parts['cmedians'].set_linewidth(1.5)

    ax_q.set_xticks(positions)
    ax_q.set_xticklabels([label_map[k] for k in sorted_keys], fontsize=8)
    ax_q.set_ylabel('RX power (dBm)')
    ax_q.set_title('E — RX power distribution', fontsize=10, fontweight='bold')
    ax_q.grid(axis='y', alpha=0.3)

    # ── Title & footer ───────────────────────────────────────────────────────
    anchor_id = 1  # from filename
    swing = criteria['criterion3']['swing_mm']
    swing_pass = criteria['criterion3']['pass']
    c3_str = f"Crit.3 swing={swing:.1f} mm  {'✓' if swing_pass else '✗'}"
    fig.suptitle(
        f'UWB Anchor-1 Bias Characterisation  ·  {csv_path.name}\n'
        f'{c3_str}   |   Analysis: {datetime.now():%Y-%m-%d %H:%M}',
        fontsize=11, fontweight='bold', y=0.97
    )

    plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'[PLOT]  Saved → {out_path}')


# ── Report ───────────────────────────────────────────────────────────────────

def make_report(stats_by_key: dict, criteria: dict, csv_path: Path) -> str:
    lines = []
    W = REPORT_WIDTH
    sep = '─' * W

    def h(title):
        lines.append('')
        lines.append(title)
        lines.append('─' * len(title))

    lines.append('=' * W)
    lines.append('UWB RTLS — Phase 1 Bias Characterisation Report')
    lines.append(f'  Source : {csv_path.name}')
    lines.append(f'  Date   : {datetime.now():%Y-%m-%d %H:%M:%S}')
    lines.append('=' * W)

    # ── Session summary ──────────────────────────────────────────────────────
    h('1. Session Summary')
    hdr = f"{'Session':>7}  {'Dist (m)':>8}  {'N_raw':>6}  {'N_clean':>7}  "
    hdr += f"{'N_out':>5}  {'Mean (mm)':>9}  {'Bias (mm)':>9}  {'Std (mm)':>8}  "
    hdr += f"{'σ_mean':>6}  {'q̄ (dBm)':>7}"
    lines.append(hdr)
    lines.append('─' * len(hdr))
    for key in sorted(stats_by_key.keys()):
        sess, dist = key
        s = stats_by_key[key]
        row = (f"{sess:>7}  {dist:>8.1f}  {s['n_raw']:>6d}  {s['n_clean']:>7d}  "
               f"{s['n_outliers']:>5d}  {s['mean_mm']:>9.1f}  "
               f"{s['bias_mm']:>+9.1f}  {s['std_mm']:>8.1f}  "
               f"{s['sigma_mean_mm']:>6.2f}  {s['mean_q_dbm']:>7.1f}")
        lines.append(row)

    # ── Criterion 2 ──────────────────────────────────────────────────────────
    h('2. Criterion 2 — Repeatability  (pass: session-to-session Δ < 5 mm)')
    c2 = criteria['criterion2']
    for dist in sorted(c2.keys()):
        info = c2[dist]
        biases_str = '  '.join(f'{b:+.1f}' for b in info['biases'])
        verdict = 'PASS ✓' if info['pass'] else 'FAIL ✗'
        lines.append(f'  {dist:.1f} m  session biases = [{biases_str}] mm')
        lines.append(f'          max Δ = {info["max_diff_mm"]:.1f} mm   → {verdict}')

    lines.append('')
    overall = 'PASS ✓' if criteria['criterion2_overall'] else 'FAIL ✗'
    lines.append(f'  CRITERION 2 OVERALL: {overall}')

    # ── Criterion 1 ──────────────────────────────────────────────────────────
    h('3. Criterion 1 — Distance-Dependent Bias (statistically significant?)')
    c1 = criteria['criterion1']
    for dist in sorted(c1['mean_bias_by_dist'].keys()):
        b = c1['mean_bias_by_dist'][dist]
        ci = c1['ci95_by_dist'][dist]
        lines.append(f'  {dist:.1f} m  pooled mean bias = {b:+.1f} ± {ci:.1f} mm  (95% CI)')

    lines.append(f'\n  Bias difference between distances = {c1["bias_diff_mm"]:.1f} mm')
    lines.append(f'  Combined uncertainty             = {c1["combined_unc_mm"]:.1f} mm')
    lines.append(f'  SNR (diff / unc)                 = {c1["snr"]:.1f}×')
    lines.append(f'  Pass threshold: SNR ≥ 3×')
    verdict = 'PASS ✓' if c1['pass'] else 'FAIL ✗'
    lines.append(f'\n  CRITERION 1: {verdict}')

    # ── Criterion 3 ──────────────────────────────────────────────────────────
    h('4. Criterion 3 — Bias Swing vs Noise Floor  (pass: swing > 20 mm)')
    c3 = criteria['criterion3']
    lines.append(f'  Bias swing = {c3["swing_mm"]:.1f} mm  (threshold: {c3["threshold_mm"]:.0f} mm)')
    verdict = 'PASS ✓' if c3['pass'] else 'FAIL ✗'
    lines.append(f'\n  CRITERION 3: {verdict}')

    # ── Criterion 4 & 5 ──────────────────────────────────────────────────────
    h('5. Criteria 4 & 5 — Not Yet Evaluable')
    lines.append(
        '  Criterion 4 (common across anchors): requires data from all 3 anchors.\n'
        '  Criterion 5 (>20% model improvement on held-out data): requires ≥5\n'
        '  test distances and all-anchor data.\n'
        '  → Pending full 8-distance experiment.'
    )

    # ── Overall decision ─────────────────────────────────────────────────────
    h('6. Phase 1 Gate Decision')
    c1p = criteria['criterion1']['pass']
    c2p = criteria['criterion2_overall']
    c3p = criteria['criterion3']['pass']
    lines.append(f'  Criterion 1 (significant distance bias) : {"PASS ✓" if c1p else "FAIL ✗"}')
    lines.append(f'  Criterion 2 (repeatability)             : {"PASS ✓" if c2p else "FAIL ✗"}')
    lines.append(f'  Criterion 3 (bias > noise floor)        : {"PASS ✓" if c3p else "FAIL ✗"}')
    lines.append(f'  Criterion 4 (common across anchors)     : PENDING — more data needed')
    lines.append(f'  Criterion 5 (model improvement > 20%)   : PENDING — more data needed')

    if c1p and c2p and c3p:
        lines.append(textwrap.fill(
            '\nVERDICT — Criteria 1, 2, 3 all PASS. The distance-dependent bias is '
            'real, reproducible, and large enough to correct. Proceed to the full '
            '8-distance, 3-anchor experiment to evaluate Criteria 4 and 5.',
            width=W, initial_indent='  ', subsequent_indent='  '))
    elif not c2p:
        lines.append(textwrap.fill(
            '\nVERDICT — Criterion 2 FAILS. Session-to-session repeatability is '
            'insufficient. The bias is not reproducible enough to build a stable '
            'correction table. Investigate positioning jitter or multipath before '
            'proceeding.', width=W, initial_indent='  ', subsequent_indent='  '))
    elif not c1p:
        lines.append(textwrap.fill(
            '\nVERDICT — Criterion 1 FAILS. No statistically significant '
            'distance-dependent bias detected across the two test distances. '
            'The existing constant RANGE_BIAS_M correction is sufficient.',
            width=W, initial_indent='  ', subsequent_indent='  '))
    else:
        lines.append(textwrap.fill(
            '\nVERDICT — Criterion 3 FAILS. The bias swing is below the noise floor. '
            'A correction table would not yield meaningful improvement.',
            width=W, initial_indent='  ', subsequent_indent='  '))

    # ── Observations ─────────────────────────────────────────────────────────
    h('7. Key Observations')

    # Compute per-distance pooled stats
    by_dist: dict = defaultdict(list)
    for (sess, dist), s in stats_by_key.items():
        by_dist[dist].append(s)

    for dist in sorted(by_dist.keys()):
        sessions = by_dist[dist]
        all_d = np.concatenate([s['d_clean'] for s in sessions])
        all_q = np.concatenate([np.array(s['q_raw']) for s in sessions])
        true_mm = dist * 1000.0
        bias = float(np.mean(all_d)) - true_mm
        std = float(np.std(all_d, ddof=1))
        q_mean = float(np.mean(all_q))
        lines.append(f'  {dist:.1f} m: bias={bias:+.1f} mm, σ={std:.1f} mm, '
                     f'q̄={q_mean:.1f} dBm  (n={len(all_d)})')

    lines.append('\n  RX power decreases with distance (as expected for LOS):')
    q_1m = np.concatenate([np.array(s['q_raw']) for s in by_dist[1.0]])
    q_3m = np.concatenate([np.array(s['q_raw']) for s in by_dist[3.0]])
    lines.append(f'    1.0 m: {np.mean(q_1m):.1f} dBm   3.0 m: {np.mean(q_3m):.1f} dBm   '
                 f'Δ = {np.mean(q_3m)-np.mean(q_1m):.1f} dBm')

    lines.append('\n  Bias sign interpretation:')
    for dist in sorted(by_dist.keys()):
        b = c1['mean_bias_by_dist'][dist]
        if b > 0:
            sign_text = 'reads LONGER (positive bias)'
        elif b < 0:
            sign_text = 'reads SHORTER (negative bias)'
        else:
            sign_text = 'no bias'
        lines.append(f'    {dist:.1f} m  → {b:+.1f} mm  ({sign_text})')

    # ── Next steps ────────────────────────────────────────────────────────────
    h('8. Recommended Next Steps')
    lines.append(
        '  1. Run the full 8-distance experiment (0.5, 1.0, 1.5, 2.0, 3.0, 4.0,\n'
        '     5.0, 6.0 m) for all 3 anchors — 2 sessions per distance per anchor.\n'
        '     This enables evaluation of Criteria 4 and 5 and selection of the\n'
        '     best correction model (polynomial vs lookup-table vs spline).\n'
        '\n'
        '  2. Use serial transport (--serial /dev/ttyUSBx) to avoid AP isolation\n'
        '     issues on the "UWB" WiFi network.\n'
        '\n'
        '  3. Check whether the bias at 1.0 m (+208 mm nominal) is consistent with\n'
        '     the current ANTENNA_DELAY calibration — a large positive bias at all\n'
        '     distances suggests the antenna delay may be slightly off, independent\n'
        '     of the distance-dependent term.'
    )

    lines.append('')
    lines.append('=' * W)
    return '\n'.join(lines)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?',
                    default='bias_anchor1_20260617_201942.csv',
                    help='CSV file from collect_calibration_data.py')
    ap.add_argument('--output-dir', default=None,
                    help='Directory for plots and report (default: next to CSV)')
    args = ap.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.is_absolute():
        base = Path(__file__).resolve().parent
        # try data/ subdir first, fall back to script dir for bare filenames
        candidate = base / 'data' / csv_path
        csv_path = candidate if candidate.exists() else base / csv_path
    if not csv_path.exists():
        print(f'ERROR: {csv_path} not found')
        sys.exit(1)

    out_dir = Path(args.output_dir) if args.output_dir else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = csv_path.stem
    plot_path  = out_dir / f'{stem}_analysis.png'
    report_path = out_dir / f'{stem}_report.txt'

    print(f'\n[LOAD]  {csv_path}')
    raw_data = load_csv(csv_path)
    print(f'[LOAD]  {len(raw_data)} sessions/distances: '
          + ', '.join(f'({s},{d:.1f}m)' for s, d in sorted(raw_data.keys())))

    # Build stats and keep q_raw for plotting
    stats = {}
    for key, data in raw_data.items():
        s = compute_stats(data['d_mm'], data['q'], key[1] * 1000.0)
        s['q_raw'] = np.array(data['q'])
        stats[key] = s

    criteria = evaluate_criteria(stats)

    make_figure(stats, criteria, plot_path, csv_path)

    report = make_report(stats, criteria, csv_path)
    print('\n' + report)
    with open(report_path, 'w') as f:
        f.write(report)
    print(f'\n[REPORT] Saved → {report_path}')


if __name__ == '__main__':
    main()
