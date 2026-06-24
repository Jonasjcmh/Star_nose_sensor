"""
analyze_capacitance_dataset.py — Star-Nose Sensor | Dataset Analysis
=====================================================================
Loads a CSV produced by capacitance_dataset_collector.py and generates:

  1. Time-series overlay  — Cp_pF and load_cell_N vs time, one subplot per
                            point, all samples overlaid.
  2. Summary bar chart    — Mean ± std Cp_pF per point (hold phase only).
  3. Sensor map heatmap   — 19-point layout coloured by mean Cp_pF.
  4. Force–Capacitance scatter — peak Cp vs peak force per indentation.
  5. Phase boxplots        — Cp_pF distribution across phases for all points.

Usage
-----
  python analyze_capacitance_dataset.py logs/capacitance_dataset_20260623_123456.csv
  python analyze_capacitance_dataset.py          # opens file dialog
"""

import os
import sys
import csv
import math
import argparse
from datetime import datetime
from collections import defaultdict

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable

# ── Sensor layout ──────────────────────────────────────────────────────────────
POINTS_XY = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

SENSOR_MAP_ROWS = [
    [1, 2, 3],
    [4, 5, 6, 7],
    [8, 9, 10, 11, 12],
    [13, 14, 15, 16],
    [17, 18, 19],
]


# ── Data loading ───────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append({
                    'timestamp':  float(row['timestamp']),
                    'round_idx':  int(row['round_idx']),
                    'sample_idx': int(row['sample_idx']),
                    'point':      int(row['point']),
                    'depth_mm':   float(row['depth_mm']),
                    'phase':      row['phase'],
                    'tcp_x':      float(row['tcp_x']),
                    'tcp_y':      float(row['tcp_y']),
                    'tcp_z':      float(row['tcp_z']),
                    'fz':         float(row['fz']),
                    'load_cell_N':float(row['load_cell_N']),
                    'Cp_pF':      float(row['Cp_pF']) if row['Cp_pF'] not in ('', 'nan') else float('nan'),
                    'Rp_Ohm':     float(row['Rp_Ohm']) if row['Rp_Ohm'] not in ('', 'nan') else float('nan'),
                    'lcr_ok':     int(row.get('lcr_ok', 0)),
                })
            except (ValueError, KeyError):
                continue
    return rows


def group_indentations(rows):
    """
    Group rows into individual indentations keyed by (point, round_idx, sample_idx).
    Returns dict → list of rows sorted by timestamp.
    """
    groups = defaultdict(list)
    for r in rows:
        key = (r['point'], r['round_idx'], r['sample_idx'])
        groups[key].append(r)
    for key in groups:
        groups[key].sort(key=lambda r: r['timestamp'])
    return groups


def hold_phase_stats(groups):
    """
    For each point, collect Cp_pF values from 'hold' phase rows across all samples.
    Returns: {pt: {'Cp_pF': [values], 'load_cell_N': [values]}}
    """
    per_point = defaultdict(lambda: {'Cp_pF': [], 'load_cell_N': []})
    for (pt, round_idx, sample_idx), rows_g in groups.items():
        for r in rows_g:
            if r['phase'] == 'hold' and not math.isnan(r['Cp_pF']):
                per_point[pt]['Cp_pF'].append(r['Cp_pF'])
                per_point[pt]['load_cell_N'].append(r['load_cell_N'])
    return per_point


def peak_per_indentation(groups):
    """
    For each indentation, find peak Cp_pF and peak load_cell_N during 'hold' phase.
    Returns list of (pt, peak_Cp_pF, peak_load_N).
    """
    results = []
    for (pt, _, _), rows_g in groups.items():
        hold_rows = [r for r in rows_g if r['phase'] == 'hold'
                     and not math.isnan(r['Cp_pF'])]
        if not hold_rows:
            continue
        peak_Cp = max(r['Cp_pF'] for r in hold_rows)
        peak_F  = max(r['load_cell_N'] for r in hold_rows)
        results.append((pt, peak_Cp, peak_F))
    return results


# ── Figure 1: Time-series per point ───────────────────────────────────────────

def plot_timeseries(groups, out_dir):
    """5×4 grid (20 subplots, 19 used): Cp and force vs relative time."""
    n_cols = 5
    n_rows = 4
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(20, 14), sharex=False)
    fig.suptitle('Cp and Load-Cell vs Time — Per Point (all samples overlaid)',
                 fontsize=13)

    ax_flat = axes.flatten()
    for ax in ax_flat:
        ax.set_visible(False)

    sorted_pts = sorted(POINTS_XY.keys())
    for ax, pt in zip(ax_flat, sorted_pts):
        ax.set_visible(True)
        ax2 = ax.twinx()
        plotted_Cp = False

        for (pt2, round_idx, sample_idx), rows_g in groups.items():
            if pt2 != pt:
                continue
            if not rows_g:
                continue
            t0 = rows_g[0]['timestamp']
            ts  = np.array([r['timestamp'] - t0 for r in rows_g])
            Cp  = np.array([r['Cp_pF']      for r in rows_g])
            lc  = np.array([r['load_cell_N'] for r in rows_g])
            ok  = np.array([bool(r['lcr_ok']) for r in rows_g])

            Cp_clean = np.where(ok & ~np.isnan(Cp), Cp, np.nan)
            ax.plot(ts, Cp_clean, alpha=0.5, linewidth=0.8, color='steelblue')
            ax2.plot(ts, lc, alpha=0.35, linewidth=0.8, color='tomato',
                     linestyle='--')
            plotted_Cp = True

        ax.set_title(f'P{pt:02d}  ({POINTS_XY[pt][0]:+.0f},{POINTS_XY[pt][1]:+.0f})',
                     fontsize=8)
        ax.set_xlabel('t (s)', fontsize=7)
        ax.set_ylabel('Cp (pF)', fontsize=7, color='steelblue')
        ax2.set_ylabel('Force (N)', fontsize=7, color='tomato')
        ax.tick_params(labelsize=6)
        ax2.tick_params(labelsize=6)

    handles = [
        matplotlib.lines.Line2D([0], [0], color='steelblue', lw=1.5,
                                 label='Cp (pF)'),
        matplotlib.lines.Line2D([0], [0], color='tomato', lw=1.5,
                                 ls='--', label='Load cell (N)'),
    ]
    fig.legend(handles=handles, loc='lower right', fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(out_dir, 'timeseries_per_point.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [fig1] Saved: {path}')
    return path


# ── Figure 2: Mean ± std bar chart ────────────────────────────────────────────

def plot_summary_bar(per_point, out_dir):
    pts   = sorted(per_point.keys())
    means = [np.mean(per_point[p]['Cp_pF']) if per_point[p]['Cp_pF'] else 0.0
             for p in pts]
    stds  = [np.std(per_point[p]['Cp_pF'])  if per_point[p]['Cp_pF'] else 0.0
             for p in pts]
    ns    = [len(per_point[p]['Cp_pF']) for p in pts]

    fig, ax = plt.subplots(figsize=(12, 5))
    x = np.arange(len(pts))
    bars = ax.bar(x, means, yerr=stds, capsize=4, color='steelblue',
                  alpha=0.7, ecolor='navy', error_kw={'lw': 1.5})

    for i, (bar, n) in enumerate(zip(bars, ns)):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + max(stds)*0.05,
                f'n={n}', ha='center', va='bottom', fontsize=7)

    ax.set_xticks(x)
    ax.set_xticklabels([f'P{p:02d}' for p in pts], rotation=45, fontsize=8)
    ax.set_ylabel('Cp (pF)  [hold phase, LCR ok]')
    ax.set_title('Mean ± Std Capacitance per Sensor Point (hold phase)')
    ax.grid(axis='y', alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'summary_bar.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [fig2] Saved: {path}')
    return path


# ── Figure 3: Sensor map heatmap ──────────────────────────────────────────────

def plot_sensor_map(per_point, out_dir):
    pts   = sorted(POINTS_XY.keys())
    means = {p: (np.mean(per_point[p]['Cp_pF'])
                 if per_point.get(p, {}).get('Cp_pF') else float('nan'))
             for p in pts}

    valid = [v for v in means.values() if not math.isnan(v)]
    vmin  = min(valid) if valid else 0
    vmax  = max(valid) if valid else 1
    norm  = Normalize(vmin=vmin, vmax=vmax)
    cmap  = plt.cm.plasma

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_aspect('equal')

    r_circle = 3.5   # mm — display radius for each node

    for pt in pts:
        x, y  = POINTS_XY[pt]
        val   = means[pt]
        color = cmap(norm(val)) if not math.isnan(val) else (0.8, 0.8, 0.8, 1.0)
        circle = plt.Circle((x, y), r_circle, color=color, ec='white', lw=0.8)
        ax.add_patch(circle)
        label = f'P{pt}\n{val:.1f}' if not math.isnan(val) else f'P{pt}\n—'
        ax.text(x, y, label, ha='center', va='center',
                fontsize=6.5, color='white', fontweight='bold')

    sm = ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, shrink=0.7)
    cbar.set_label('Mean Cp (pF) — hold phase')

    ax.set_xlim(-22, 22)
    ax.set_ylim(-20, 20)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('Sensor Layout — Mean Cp per Point (hold phase)')
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    path = os.path.join(out_dir, 'sensor_map_heatmap.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [fig3] Saved: {path}')
    return path


# ── Figure 4: Force–Capacitance scatter ───────────────────────────────────────

def plot_force_cp_scatter(peaks, out_dir):
    if not peaks:
        print('  [fig4] No peak data to plot.')
        return None

    pts_all  = [p[0] for p in peaks]
    Cp_all   = [p[1] for p in peaks]
    F_all    = [p[2] for p in peaks]
    unique_pts = sorted(set(pts_all))
    cmap     = plt.cm.tab20
    colors   = {p: cmap(i / max(len(unique_pts) - 1, 1))
                for i, p in enumerate(unique_pts)}

    fig, ax = plt.subplots(figsize=(9, 6))
    for pt in unique_pts:
        mask = [i for i, p in enumerate(pts_all) if p == pt]
        ax.scatter([Cp_all[i] for i in mask],
                   [F_all[i]  for i in mask],
                   color=colors[pt], s=40, alpha=0.7,
                   label=f'P{pt:02d}', edgecolors='none')

    # Linear fit
    Cp_arr = np.array(Cp_all)
    F_arr  = np.array(F_all)
    if len(Cp_arr) >= 3:
        coeffs = np.polyfit(Cp_arr, F_arr, 1)
        x_fit  = np.linspace(Cp_arr.min(), Cp_arr.max(), 100)
        ax.plot(x_fit, np.polyval(coeffs, x_fit),
                'k--', lw=1.5, label=f'Linear fit (slope={coeffs[0]:.3f})')
        r     = np.corrcoef(Cp_arr, F_arr)[0, 1]
        ax.text(0.97, 0.97, f'r = {r:.3f}',
                transform=ax.transAxes, ha='right', va='top', fontsize=10)

    ax.set_xlabel('Peak Cp (pF)  — hold phase')
    ax.set_ylabel('Peak Load Cell Force (N)  — hold phase')
    ax.set_title('Force vs Capacitance per Indentation')
    ax.legend(ncol=4, fontsize=7, loc='upper left')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'force_cp_scatter.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [fig4] Saved: {path}')
    return path


# ── Figure 5: Phase boxplots ───────────────────────────────────────────────────

def plot_phase_boxplots(rows, out_dir):
    phases  = ['locate', 'press', 'hold', 'retract', 'post']
    data    = {ph: [] for ph in phases}
    for r in rows:
        if r['phase'] in data and not math.isnan(r['Cp_pF']):
            data[r['phase']].append(r['Cp_pF'])

    fig, ax = plt.subplots(figsize=(9, 5))
    box_data = [data[ph] for ph in phases]
    bp = ax.boxplot(box_data, labels=phases, patch_artist=True,
                    medianprops={'color': 'black', 'lw': 2})

    colors = ['#AED6F1', '#F0B27A', '#A9DFBF', '#F1948A', '#D7BDE2']
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)

    ax.set_ylabel('Cp (pF)  [LCR ok only]')
    ax.set_title('Cp Distribution by Phase — All Points & Samples')
    ax.grid(axis='y', alpha=0.3)

    for i, ph in enumerate(phases):
        n = len(data[ph])
        ax.text(i + 1, ax.get_ylim()[0], f'n={n}',
                ha='center', va='bottom', fontsize=7)

    fig.tight_layout()
    path = os.path.join(out_dir, 'phase_boxplots.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [fig5] Saved: {path}')
    return path


# ── Figure 6: Per-point Cp statistics table (text summary) ────────────────────

def print_stats_table(per_point):
    pts = sorted(per_point.keys())
    print()
    print('  Point  |  n  |   Mean Cp (pF)   |   Std (pF)   |  CV (%)')
    print('  ' + '-' * 58)
    for pt in pts:
        vals = per_point[pt]['Cp_pF']
        if not vals:
            print(f'  P{pt:02d}    |  0  |       —          |      —       |    —')
            continue
        mu  = np.mean(vals)
        sd  = np.std(vals)
        cv  = (sd / mu * 100) if mu != 0 else float('nan')
        print(f'  P{pt:02d}    | {len(vals):3d} | {mu:14.3f}   | {sd:10.3f}   | {cv:6.2f}')
    print()


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Analyze capacitance dataset from capacitance_dataset_collector.py')
    p.add_argument('csv_path', nargs='?', default=None,
                   help='Path to dataset CSV file')
    p.add_argument('--out', default=None,
                   help='Output directory for plots (default: plots/ next to CSV)')
    p.add_argument('--no-show', action='store_true',
                   help='Do not display plots interactively')
    return p.parse_args()


def _pick_file():
    """If tkinter is available, open a file dialog; otherwise ask for path."""
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    csvs    = sorted(glob.glob(os.path.join(log_dir, 'capacitance_dataset_*.csv')))

    if csvs:
        print('\n  Available dataset files:')
        for i, p in enumerate(csvs):
            print(f'    [{i}]  {os.path.basename(p)}')
        try:
            idx = int(input(f'  Select file [0–{len(csvs)-1}] > ').strip())
            if 0 <= idx < len(csvs):
                return csvs[idx]
        except (ValueError, EOFError, KeyboardInterrupt):
            pass

    return input('  Enter full path to CSV file: ').strip()


def main():
    import glob as _glob
    global glob
    glob = _glob

    args = parse_args()

    csv_path = args.csv_path or _pick_file()
    if not os.path.exists(csv_path):
        print(f'File not found: {csv_path}')
        sys.exit(1)

    out_dir = args.out or os.path.join(
        os.path.dirname(csv_path), '..', 'plots',
        'dataset_' + os.path.splitext(os.path.basename(csv_path))[0])
    out_dir = os.path.normpath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    print(f'\n  Loading: {csv_path}')
    rows = load_csv(csv_path)
    print(f'  Rows loaded: {len(rows)}')

    if not rows:
        print('  No valid rows found — check the CSV format.')
        sys.exit(1)

    groups    = group_indentations(rows)
    per_point = hold_phase_stats(groups)
    peaks     = peak_per_indentation(groups)

    n_pts  = len({r['point'] for r in rows})
    n_inds = len(groups)
    phases_present = {r['phase'] for r in rows}
    depth  = rows[0]['depth_mm']

    print(f'  Points in dataset : {n_pts}')
    print(f'  Indentations      : {n_inds}')
    print(f'  Phases present    : {sorted(phases_present)}')
    print(f'  Indentation depth : {depth:.2f} mm')
    lcr_ok_pct = 100 * sum(1 for r in rows if r['lcr_ok']) / max(len(rows), 1)
    print(f'  LCR ok rows       : {lcr_ok_pct:.1f}%')

    print_stats_table(per_point)

    if not args.no_show:
        matplotlib.use('TkAgg')   # adjust if using a headless environment

    print(f'\n  Saving plots to: {out_dir}')
    plot_timeseries(groups, out_dir)
    plot_summary_bar(per_point, out_dir)
    plot_sensor_map(per_point, out_dir)
    plot_force_cp_scatter(peaks, out_dir)
    plot_phase_boxplots(rows, out_dir)

    if not args.no_show:
        print('\n  Opening plots ...')
        for f in os.listdir(out_dir):
            if f.endswith('.png'):
                img = plt.imread(os.path.join(out_dir, f))
                fig, ax = plt.subplots()
                ax.imshow(img)
                ax.axis('off')
                ax.set_title(f)
        plt.show()

    print('\n[done]')


if __name__ == '__main__':
    main()
