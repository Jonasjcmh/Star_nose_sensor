"""
analyze_ramp_collector.py — Star-Nose Sensor | Ramp-Collector Analysis
======================================================================
Reads a CSV produced by capacitance_ramp_collector.py (N samples/rounds × 19
points, phases locate → press → hold → retract → post, logged at ~100 Hz) and
produces a full analysis, mirroring analyze_capacitance_dataset.py but adapted
to the fixed-ramp / tail-averaging design of the collector.

Two families of figures
------------------------
  A. Per-round overlay grids  — FOR EACH ROUND (sample): a 4×5 grid of 19 panels
     (one per point) overlaying Cp (left axis) + force (right axis) vs time, with
     the background shaded by phase. Lets you inspect every ramp individually.

  B. Dataset-level summary (all rounds together):
       1. Time-series per point   — Cp + force vs time, all rounds overlaid.
       2. Summary bar chart       — mean ± std settled ΔCp per point.
       3. Sensor-map heatmap      — 19-point layout coloured by mean ΔCp.
       4. Force–ΔCp scatter       — settled ΔCp vs force, one point per indent.
       5. Phase boxplots          — Cp distribution across the five phases.
       6. Stats table (printed)   — per-point n / mean / std / CV of ΔCp.

Why ΔCp (baseline-corrected)?
-----------------------------
The LCR probes are re-wired to each pad between points, so the *absolute* Cp
baseline differs from point to point. The meaningful, comparable response of an
indentation is the settled change:
        ΔCp = mean(Cp over hold tail) − mean(Cp over locate baseline)
computed from the last SETTLE_WINDOW_S of each phase (≈ √N less noise than a
single sample), exactly as the collector reports it live.

Usage
-----
  python analyze_ramp_collector.py                       # newest ramp_collector_*.csv
  python analyze_ramp_collector.py logs/ramp_collector_20260625_181557.csv
  python analyze_ramp_collector.py --round 0             # only round 1's grid
  python analyze_ramp_collector.py --no-grids            # summary figures only
  python analyze_ramp_collector.py FILE.csv --no-show    # save only, don't display
"""

import os
import csv
import sys
import glob
import math
import argparse
from collections import defaultdict
from datetime import datetime

import numpy as np

_HERE    = os.path.dirname(os.path.abspath(__file__))
LOG_DIR  = os.path.join(_HERE, 'logs')
PLOT_DIR = os.path.join(_HERE, 'plots')

# Must match capacitance_ramp_collector.SETTLE_WINDOW_S — the tail window over
# which each phase's settled reading is averaged.
SETTLE_WINDOW_S = 1.0

PHASE_ORDER  = ['locate', 'press', 'hold', 'retract', 'post']
PHASE_COLORS = {
    'locate':  '#cfe8ff',
    'press':   '#ffe2c2',
    'hold':    '#c8f5c8',
    'retract': '#ffd6d6',
    'post':    '#e6e6e6',
}

# ── Sensor layout (mm, relative to reference pose) ───────────────────────────────
POINTS_XY = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}
ALL_POINTS = list(range(1, 20))

# ── Data loading ────────────────────────────────────────────────────────────────

def newest_csv():
    files = sorted(glob.glob(os.path.join(LOG_DIR, 'ramp_collector_*.csv')))
    if not files:
        raise SystemExit(f'[error] No ramp_collector_*.csv found in {LOG_DIR}')
    return files[-1]

def _f(v, default=float('nan')):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default

def _i(v, default=0):
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return default

def load_rows(path):
    """Load CSV into a list of light dicts with the fields we analyse."""
    rows = []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            rows.append({
                'timestamp':   _f(r.get('timestamp')),
                'round_idx':   _i(r.get('round_idx', 0)),
                'sample_idx':  _i(r.get('sample_idx', 0)),
                'point':       _i(r.get('point', 0)),
                'depth_mm':    _f(r.get('depth_mm')),
                'phase':       r.get('phase', ''),
                'fz':          _f(r.get('fz')),
                'load_cell_N': _f(r.get('load_cell_N')),
                'Cp_pF':       _f(r.get('Cp_pF')),
                'Rp_Ohm':      _f(r.get('Rp_Ohm')),
                'lcr_ok':      _i(r.get('lcr_ok', 0)),
            })
    if not rows:
        raise SystemExit(f'[error] No data rows in {path}')
    return rows

def group_indentations(rows):
    """Group rows into indentations keyed by (point, round_idx, sample_idx),
    each sorted by timestamp."""
    groups = defaultdict(list)
    for r in rows:
        groups[(r['point'], r['round_idx'], r['sample_idx'])].append(r)
    for key in groups:
        groups[key].sort(key=lambda r: r['timestamp'])
    return groups

# ── Settled tail-averaging (matches the collector's live readings) ───────────────

def _phase_rows(rows_g, phase):
    return [r for r in rows_g if r['phase'] == phase]

def tail_mean(rows_g, phase, window_s=SETTLE_WINDOW_S):
    """Mean Cp_pF (LCR-ok, non-NaN) and load_cell_N over the LAST `window_s`
    seconds of `phase` within one indentation. Returns (cp_mean, load_mean, n)."""
    prows = _phase_rows(rows_g, phase)
    if not prows:
        return float('nan'), float('nan'), 0
    t_end = prows[-1]['timestamp']
    tail  = [r for r in prows if r['timestamp'] >= t_end - window_s]
    cps   = [r['Cp_pF'] for r in tail if not math.isnan(r['Cp_pF'])]
    lds   = [r['load_cell_N'] for r in tail if not math.isnan(r['load_cell_N'])]
    cp = float(np.mean(cps)) if cps else float('nan')
    ld = float(np.mean(lds)) if lds else float('nan')
    return cp, ld, len(tail)

def indentation_responses(groups, window_s=SETTLE_WINDOW_S):
    """One settled response per indentation. Returns list of dicts with
    baseline (locate), hold, ΔCp = hold − locate, hold force, and hold peaks."""
    out = []
    for (pt, rnd, smp), rows_g in groups.items():
        cp_base, _,       _ = tail_mean(rows_g, 'locate', window_s)
        cp_hold, f_hold,  n = tail_mean(rows_g, 'hold',   window_s)
        if math.isnan(cp_hold):
            continue
        dcp = (cp_hold - cp_base) if not math.isnan(cp_base) else float('nan')
        hold_rows = [r for r in _phase_rows(rows_g, 'hold')
                     if not math.isnan(r['Cp_pF'])]
        peak_cp = max((r['Cp_pF'] for r in hold_rows), default=float('nan'))
        peak_f  = max((r['load_cell_N'] for r in _phase_rows(rows_g, 'hold')
                       if not math.isnan(r['load_cell_N'])), default=float('nan'))
        out.append({
            'point': pt, 'round': rnd, 'sample': smp,
            'cp_base': cp_base, 'cp_hold': cp_hold, 'dcp': dcp,
            'force': f_hold, 'peak_cp': peak_cp, 'peak_force': peak_f,
        })
    return out

def per_point_dcp(responses):
    """{pt: [ΔCp, ...]} across rounds (drops NaN)."""
    d = defaultdict(list)
    for r in responses:
        if not math.isnan(r['dcp']):
            d[r['point']].append(r['dcp'])
    return d

# ── Phase shading helper (per-point time-series) ─────────────────────────────────

def phase_spans(t, phases):
    """Yield (phase, t_start, t_end) for each consecutive run of the same phase."""
    spans = []
    if not phases:
        return spans
    start = 0
    for i in range(1, len(phases) + 1):
        if i == len(phases) or phases[i] != phases[start]:
            t_end = t[i] if i < len(phases) else t[i - 1]
            spans.append((phases[start], t[start], t_end))
            start = i
    return spans

# ── A. Per-round overlay grid ────────────────────────────────────────────────────

def plot_round_grid(rows, round_idx, out_dir, show=True):
    """One figure per round: 4×5 grid of 19 per-point Cp+force overlay panels."""
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    rrows = [r for r in rows if r['round_idx'] == round_idx]
    if not rrows:
        print(f'[grid] No data for round {round_idx + 1}.')
        return None

    by_pt = defaultdict(list)
    for r in rrows:
        by_pt[r['point']].append(r)

    ncols, nrows = 5, 4
    C_CP, C_FORCE = '#1f77b4', '#d62728'

    fig, axs = plt.subplots(nrows, ncols, figsize=(22, 13))
    fig.suptitle(f'Round {round_idx + 1} — Cp (blue) + force (red) vs time, per point',
                 fontsize=14, fontweight='bold')

    for i, pt in enumerate(ALL_POINTS):
        ax = axs[i // ncols][i % ncols]
        prows = sorted(by_pt.get(pt, []), key=lambda r: r['timestamp'])
        if not prows:
            ax.set_title(f'P{pt:02d}  (no data)', fontsize=9)
            ax.set_xticks([]); ax.set_yticks([])
            continue
        t0 = prows[0]['timestamp']
        t  = [r['timestamp'] - t0 for r in prows]
        cp = [r['Cp_pF'] for r in prows]
        ld = [r['load_cell_N'] for r in prows]
        phases = [r['phase'] for r in prows]
        for phase, ta, tb in phase_spans(t, phases):
            ax.axvspan(ta, tb, color=PHASE_COLORS.get(phase, '#ffffff'), alpha=0.5, lw=0)
        ax.plot(t, cp, color=C_CP, lw=1.0)
        ax.tick_params(axis='y', labelcolor=C_CP, labelsize=7)
        ax.tick_params(axis='x', labelsize=7)
        ax.grid(alpha=0.25)
        ax.set_title(f'P{pt:02d}', fontsize=10, fontweight='bold')
        axf = ax.twinx()
        axf.plot(t, ld, color=C_FORCE, lw=0.9, alpha=0.85)
        axf.tick_params(axis='y', labelcolor=C_FORCE, labelsize=7)

    for j in range(len(ALL_POINTS), nrows * ncols):
        axs[j // ncols][j % ncols].axis('off')

    sig = [Line2D([0], [0], color=C_CP,    lw=1.4, label='Cp (pF, left)'),
           Line2D([0], [0], color=C_FORCE, lw=1.4, label='force (N, right)')]
    ph  = [Patch(color=c, alpha=0.5, label=p) for p, c in PHASE_COLORS.items()]
    fig.legend(handles=sig + ph, loc='lower center', ncol=7, fontsize=10,
               frameon=False, bbox_to_anchor=(0.5, 0.005))
    fig.tight_layout(rect=[0, 0.04, 1, 0.95])

    path = os.path.join(out_dir, f'round{round_idx + 1:02d}_grid.png')
    fig.savefig(path, dpi=120, bbox_inches='tight')
    print(f'  [grid] Round {round_idx + 1} → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B1. Time-series per point (all rounds overlaid) ──────────────────────────────

def plot_timeseries(groups, out_dir, show=True):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    ncols, nrows = 5, 4
    C_CP, C_FORCE = 'steelblue', 'tomato'
    fig, axes = plt.subplots(nrows, ncols, figsize=(22, 13))
    fig.suptitle('Cp (blue) + force (red) vs time — per point, all rounds overlaid',
                 fontsize=14, fontweight='bold')

    by_pt = defaultdict(list)
    for key, rows_g in groups.items():
        by_pt[key[0]].append(rows_g)

    for i, pt in enumerate(ALL_POINTS):
        ax  = axes[i // ncols][i % ncols]
        ax2 = ax.twinx()
        any_data = False
        for rows_g in by_pt.get(pt, []):
            if not rows_g:
                continue
            t0 = rows_g[0]['timestamp']
            ts = np.array([r['timestamp'] - t0 for r in rows_g])
            cp = np.array([r['Cp_pF'] for r in rows_g])
            ld = np.array([r['load_cell_N'] for r in rows_g])
            ax.plot(ts, cp, color=C_CP, alpha=0.55, lw=0.8)
            ax2.plot(ts, ld, color=C_FORCE, alpha=0.35, lw=0.8, ls='--')
            any_data = True
        ax.set_title(f'P{pt:02d}  ({POINTS_XY[pt][0]:+.0f},{POINTS_XY[pt][1]:+.0f})',
                     fontsize=9, fontweight='bold')
        ax.tick_params(axis='y', labelcolor=C_CP, labelsize=7)
        ax2.tick_params(axis='y', labelcolor=C_FORCE, labelsize=7)
        ax.tick_params(axis='x', labelsize=7)
        ax.grid(alpha=0.25)
        if not any_data:
            ax.set_xticks([]); ax.set_yticks([])

    for j in range(len(ALL_POINTS), nrows * ncols):
        axes[j // ncols][j % ncols].axis('off')

    handles = [Line2D([0], [0], color=C_CP, lw=1.5, label='Cp (pF, left)'),
               Line2D([0], [0], color=C_FORCE, lw=1.5, ls='--', label='force (N, right)')]
    fig.legend(handles=handles, loc='lower center', ncol=2, fontsize=10,
               frameon=False, bbox_to_anchor=(0.5, 0.005))
    fig.tight_layout(rect=[0, 0.04, 1, 0.95])
    path = os.path.join(out_dir, 'timeseries_per_point.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    print(f'  [fig1] Time-series → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B2. Summary bar — mean ± std ΔCp per point ──────────────────────────────────

def plot_summary_bar(dcp_by_pt, out_dir, show=True):
    import matplotlib.pyplot as plt
    pts   = ALL_POINTS
    means = [np.mean(dcp_by_pt[p]) if dcp_by_pt.get(p) else 0.0 for p in pts]
    stds  = [np.std(dcp_by_pt[p])  if dcp_by_pt.get(p) else 0.0 for p in pts]
    ns    = [len(dcp_by_pt.get(p, [])) for p in pts]

    fig, ax = plt.subplots(figsize=(13, 5))
    x = np.arange(len(pts))
    bars = ax.bar(x, means, yerr=stds, capsize=4, color='steelblue',
                  alpha=0.75, ecolor='navy', error_kw={'lw': 1.4})
    top = max((m + s for m, s in zip(means, stds)), default=1.0)
    for bar, n in zip(bars, ns):
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.02 * top,
                f'n={n}', ha='center', va='bottom', fontsize=7)
    ax.set_xticks(x)
    ax.set_xticklabels([f'P{p:02d}' for p in pts], rotation=45, fontsize=8)
    ax.set_ylabel('Settled ΔCp = hold − locate  (pF)')
    ax.set_title('Mean ± Std settled ΔCp per Sensor Point  (tail-averaged)')
    ax.axhline(0, color='k', lw=0.6)
    ax.grid(axis='y', alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'summary_bar_dcp.png')
    fig.savefig(path, dpi=150)
    print(f'  [fig2] Summary bar → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B3. Sensor-map heatmap of mean ΔCp ───────────────────────────────────────────

def plot_sensor_map(dcp_by_pt, out_dir, show=True):
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    from matplotlib.cm import ScalarMappable

    means = {p: (float(np.mean(dcp_by_pt[p])) if dcp_by_pt.get(p) else float('nan'))
             for p in ALL_POINTS}
    valid = [v for v in means.values() if not math.isnan(v)]
    vmin, vmax = (min(valid), max(valid)) if valid else (0.0, 1.0)
    if vmin == vmax:
        vmax = vmin + 1.0
    norm = Normalize(vmin=vmin, vmax=vmax)
    cmap = plt.cm.plasma

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_aspect('equal')
    r_circle = 3.5
    for pt in ALL_POINTS:
        x, y  = POINTS_XY[pt]
        val   = means[pt]
        color = cmap(norm(val)) if not math.isnan(val) else (0.82, 0.82, 0.82, 1.0)
        ax.add_patch(plt.Circle((x, y), r_circle, color=color, ec='white', lw=0.8))
        label = f'P{pt}\n{val:.1f}' if not math.isnan(val) else f'P{pt}\n—'
        ax.text(x, y, label, ha='center', va='center',
                fontsize=6.5, color='white', fontweight='bold')
    sm = ScalarMappable(cmap=cmap, norm=norm); sm.set_array([])
    fig.colorbar(sm, ax=ax, shrink=0.7).set_label('Mean settled ΔCp (pF)')
    ax.set_xlim(-22, 22); ax.set_ylim(-20, 20)
    ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')
    ax.set_title('Sensor Layout — Mean settled ΔCp per Point')
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    path = os.path.join(out_dir, 'sensor_map_dcp.png')
    fig.savefig(path, dpi=150)
    print(f'  [fig3] Sensor map → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B4. Force–ΔCp scatter ────────────────────────────────────────────────────────

def plot_force_cp_scatter(responses, out_dir, show=True):
    import matplotlib.pyplot as plt
    pts = [r for r in responses
           if not math.isnan(r['dcp']) and not math.isnan(r['force'])]
    if not pts:
        print('  [fig4] No settled response data to plot.')
        return None

    uniq  = sorted({r['point'] for r in pts})
    cmap  = plt.cm.tab20
    color = {p: cmap(i / max(len(uniq) - 1, 1)) for i, p in enumerate(uniq)}

    fig, ax = plt.subplots(figsize=(9, 6))
    for p in uniq:
        sel = [r for r in pts if r['point'] == p]
        ax.scatter([r['dcp'] for r in sel], [r['force'] for r in sel],
                   color=color[p], s=40, alpha=0.75, label=f'P{p:02d}',
                   edgecolors='none')

    dcp = np.array([r['dcp'] for r in pts])
    frc = np.array([r['force'] for r in pts])
    if len(dcp) >= 3:
        a, b = np.polyfit(dcp, frc, 1)
        xs   = np.linspace(dcp.min(), dcp.max(), 100)
        ax.plot(xs, a * xs + b, 'k--', lw=1.5, label=f'fit slope={a:.3f}')
        r = np.corrcoef(dcp, frc)[0, 1]
        ax.text(0.97, 0.97, f'r = {r:.3f}', transform=ax.transAxes,
                ha='right', va='top', fontsize=10)

    ax.set_xlabel('Settled ΔCp (pF)  — hold − locate')
    ax.set_ylabel('Settled force (N)  — hold tail')
    ax.set_title('Force vs settled ΔCp per Indentation')
    ax.legend(ncol=4, fontsize=7, loc='upper left')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'force_dcp_scatter.png')
    fig.savefig(path, dpi=150)
    print(f'  [fig4] Force–ΔCp scatter → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B5. Phase boxplots ───────────────────────────────────────────────────────────

def plot_phase_boxplots(rows, out_dir, show=True):
    import matplotlib.pyplot as plt
    data = {ph: [] for ph in PHASE_ORDER}
    for r in rows:
        if r['phase'] in data and not math.isnan(r['Cp_pF']):
            data[r['phase']].append(r['Cp_pF'])

    fig, ax = plt.subplots(figsize=(9, 5))
    bp = ax.boxplot([data[ph] for ph in PHASE_ORDER], tick_labels=PHASE_ORDER,
                    patch_artist=True, medianprops={'color': 'black', 'lw': 2},
                    showfliers=False)
    for patch, ph in zip(bp['boxes'], PHASE_ORDER):
        patch.set_facecolor(PHASE_COLORS[ph])
    ax.set_ylabel('Cp (pF)')
    ax.set_title('Cp Distribution by Phase — all points & rounds')
    ax.grid(axis='y', alpha=0.3)
    for i, ph in enumerate(PHASE_ORDER):
        ax.text(i + 1, ax.get_ylim()[0], f'n={len(data[ph])}',
                ha='center', va='bottom', fontsize=7)
    fig.tight_layout()
    path = os.path.join(out_dir, 'phase_boxplots.png')
    fig.savefig(path, dpi=150)
    print(f'  [fig5] Phase boxplots → {path}')
    if not show:
        plt.close(fig)
    return path

# ── Stats table ──────────────────────────────────────────────────────────────────

def print_stats_table(dcp_by_pt):
    print()
    print('  Point  |  n  |  Mean ΔCp (pF)  |   Std (pF)   |  CV (%)')
    print('  ' + '-' * 56)
    for pt in ALL_POINTS:
        vals = dcp_by_pt.get(pt, [])
        if not vals:
            print(f'  P{pt:02d}    |  0  |       —         |      —       |    —')
            continue
        mu, sd = float(np.mean(vals)), float(np.std(vals))
        cv = (sd / mu * 100) if mu != 0 else float('nan')
        print(f'  P{pt:02d}    | {len(vals):3d} | {mu:13.3f}   | {sd:10.3f}   | {cv:6.2f}')
    print()

# ── Main ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Analyse a ramp_collector CSV (N rounds × 19 points)',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('csv', nargs='?', default=None,
                   help='CSV path (default: newest ramp_collector_*.csv in logs/)')
    p.add_argument('--round', type=int, default=None,
                   help='Only draw this round index (0-based) for the per-round grid')
    p.add_argument('--no-grids', action='store_true',
                   help='Skip the per-round overlay grids (summary figures only)')
    p.add_argument('--no-summary', action='store_true',
                   help='Skip the dataset-level summary figures (grids only)')
    p.add_argument('--out', default=None, help='Output directory (default: plots/dataset_<name>)')
    p.add_argument('--no-show', action='store_true', help='Save figures only, do not display')
    return p.parse_args()

def main():
    args = parse_args()
    path = args.csv or newest_csv()
    if not os.path.exists(path):
        raise SystemExit(f'[error] File not found: {path}')

    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')

    print(f'[load] {path}')
    rows = load_rows(path)

    out_dir = args.out or os.path.join(
        PLOT_DIR, 'dataset_' + os.path.splitext(os.path.basename(path))[0])
    os.makedirs(out_dir, exist_ok=True)

    groups    = group_indentations(rows)
    responses = indentation_responses(groups)
    dcp_by_pt = per_point_dcp(responses)

    rounds  = sorted({r['round_idx'] for r in rows})
    n_pts   = len({r['point'] for r in rows})
    phases  = {r['phase'] for r in rows}
    depth   = rows[0]['depth_mm']
    lcr_pct = 100 * sum(1 for r in rows if r['lcr_ok']) / max(len(rows), 1)

    print(f'[info] {len(rows)} rows  |  rounds (samples): {[r + 1 for r in rounds]}  '
          f'|  points: {n_pts}  |  indentations: {len(groups)}')
    print(f'[info] depth: {depth:.2f} mm  |  phases: {sorted(phases)}  '
          f'|  settle window: {SETTLE_WINDOW_S:.1f}s')
    if lcr_pct < 1.0:
        print('[note] lcr_ok flag is ~0% for this collector (known unreliable) — '
              'Cp values are valid, so analysis filters on NaN only, not lcr_ok.')
    print_stats_table(dcp_by_pt)
    print(f'[out]  Saving figures to: {out_dir}')

    show = not args.no_show

    # ── B. Dataset-level summary ────────────────────────────────────────────────
    if not args.no_summary:
        plot_timeseries(groups, out_dir, show=show)
        plot_summary_bar(dcp_by_pt, out_dir, show=show)
        plot_sensor_map(dcp_by_pt, out_dir, show=show)
        plot_force_cp_scatter(responses, out_dir, show=show)
        plot_phase_boxplots(rows, out_dir, show=show)

    # ── A. Per-round overlay grids ──────────────────────────────────────────────
    if not args.no_grids:
        grid_rounds = rounds
        if args.round is not None:
            if args.round not in rounds:
                raise SystemExit(f'[error] Round {args.round} not in data (have {rounds})')
            grid_rounds = [args.round]
        for ri in grid_rounds:
            plot_round_grid(rows, ri, out_dir, show=show)

    if show:
        import matplotlib.pyplot as plt
        print('\n[show] Displaying figures — close windows to exit.')
        plt.show()
    print('[done]')


if __name__ == '__main__':
    main()
