#!/usr/bin/env python3
"""
Temporary analysis script for capacitance_dataset_20260624_163916.csv

Root cause of empty figures in analyze_capacitance_dataset.py:
  lcr_ok == 0 for every row in this dataset, so the lcr_ok=1 filter
  yields zero rows. All Cp_pF values are valid (1.50–2.17 pF), so we
  simply skip the lcr_ok filter here.

Output: Capacitance_measurement/plots2/
"""

import os
import csv
import math
import collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import Normalize, LinearSegmentedColormap
from matplotlib.patches import RegularPolygon
from matplotlib.cm import ScalarMappable

# ── Paths ─────────────────────────────────────────────────────────────────────
HERE    = os.path.dirname(os.path.abspath(__file__))
CSV     = os.path.join(HERE, 'logs', 'capacitance_dataset_20260624_163916.csv')
OUT_DIR = os.path.join(HERE, 'plots2')
os.makedirs(OUT_DIR, exist_ok=True)

# ── Sensor layout (same as friction_mode scripts) ─────────────────────────────
POINTS_MM = [
    ( -8, +14), (  0, +14), ( +8, +14),
    (-12,  +7), ( -4,  +7), ( +4,  +7), (+12,  +7),
    (-16,   0), ( -8,   0), (  0,   0), ( +8,   0), (+16,   0),
    (-12,  -7), ( -4,  -7), ( +4,  -7), (+12,  -7),
    ( -8, -14), (  0, -14), ( +8, -14),
]
# UR5 point (1-19) → POINTS_MM index (120° rotation between frames)
UR5_TO_IDX = {
    1:16,  2:12,  3:7,
    4:17,  5:13,  6:8,   7:3,
    8:18,  9:14,  10:9,  11:4,  12:0,
    13:15, 14:10, 15:5,  16:1,
    17:11, 18:6,  19:2,
}
IDX_TO_UR5 = {v: k for k, v in UR5_TO_IDX.items()}

CMAP = LinearSegmentedColormap.from_list(
    'star_nose', ['#2ab5a0', '#33e666', '#ffe619', '#ff7300', '#dc0000'])

HEX_R = 4.2  # hex radius in mm

matplotlib.rcParams.update({
    'figure.facecolor': 'white',
    'axes.facecolor':   'white',
    'font.family':      'DejaVu Sans',
    'axes.spines.top':  False,
    'axes.spines.right':False,
})

PHASE_ORDER  = ['locate', 'press', 'hold', 'retract', 'post']
PHASE_COLORS = {
    'locate':  '#5b9bd5',
    'press':   '#ed7d31',
    'hold':    '#a9d18e',
    'retract': '#9e5fb5',
    'post':    '#c9c9c9',
}


# ── Data loading ──────────────────────────────────────────────────────────────
def load_data(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            # parse numeric fields
            for col in ('timestamp', 'round_idx', 'sample_idx', 'depth_mm',
                        'Cp_pF', 'Cp_F', 'Rp_Ohm', 'load_cell_N', 'fz',
                        'tcp_x', 'tcp_y', 'tcp_z', 'ai0'):
                if col in row:
                    try:
                        row[col] = float(row[col])
                    except (ValueError, TypeError):
                        row[col] = float('nan')
            row['point']     = int(row['point'])
            row['round_idx'] = int(row['round_idx'])
            row['sample_idx']= int(row['sample_idx'])
            row['lcr_ok']    = int(row.get('lcr_ok', 0))
            rows.append(row)
    return rows


def valid_cp(v):
    return not (math.isnan(v) or math.isinf(v))


# ── 1. Summary statistics ─────────────────────────────────────────────────────
def compute_stats(rows):
    hold = [r for r in rows if r['phase'] == 'hold' and valid_cp(r['Cp_pF'])]

    by_point  = collections.defaultdict(list)
    by_round  = collections.defaultdict(list)
    by_phase  = collections.defaultdict(list)

    for r in rows:
        if valid_cp(r['Cp_pF']):
            by_phase[r['phase']].append(r['Cp_pF'])

    for r in hold:
        by_point[r['point']].append(r['Cp_pF'])
        by_round[r['round_idx']].append(r['Cp_pF'])

    return hold, by_point, by_round, by_phase


# ── 2. Figure: Sensor map heatmap ─────────────────────────────────────────────
def plot_sensor_map(by_point, outdir):
    means = {p: np.mean(v) for p, v in by_point.items()}
    stds  = {p: np.std(v)  for p, v in by_point.items()}

    vals = np.array([means.get(IDX_TO_UR5.get(i, -1), 0.0)
                     for i in range(len(POINTS_MM))])
    vmin, vmax = vals.min(), vals.max()
    norm = Normalize(vmin, vmax)

    fig, ax = plt.subplots(figsize=(7, 6))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('#111111')

    for i, (xmm, ymm) in enumerate(POINTS_MM):
        ur5_pt  = IDX_TO_UR5.get(i)
        cp_mean = means.get(ur5_pt, float('nan')) if ur5_pt else float('nan')
        cp_std  = stds.get(ur5_pt, 0.0) if ur5_pt else 0.0

        if ur5_pt and not math.isnan(cp_mean):
            col = CMAP(norm(cp_mean))
            label_cp = f'{cp_mean:.3f}'
        else:
            col = '#333333'
            label_cp = 'N/A'

        h = RegularPolygon((xmm, ymm), numVertices=6, radius=HEX_R,
                           facecolor=col, edgecolor='white',
                           linewidth=0.8, zorder=2)
        ax.add_patch(h)
        ax.text(xmm, ymm + 1.1, f'P{ur5_pt}',
                ha='center', va='center', fontsize=6.5,
                color='white', fontweight='bold', zorder=3)
        ax.text(xmm, ymm - 1.4, label_cp,
                ha='center', va='center', fontsize=5.5,
                color='white', zorder=3)

    sm = ScalarMappable(cmap=CMAP, norm=norm)
    sm.set_array([])
    cb = plt.colorbar(sm, ax=ax, fraction=0.035, pad=0.02)
    cb.set_label('Mean Cp (pF) — hold phase', fontsize=9)
    cb.ax.tick_params(labelsize=8)

    ax.set_xlim(-24, 24)
    ax.set_ylim(-22, 22)
    ax.set_aspect('equal')
    ax.set_xlabel('X (mm, sensor frame)', fontsize=9)
    ax.set_ylabel('Y (mm, sensor frame)', fontsize=9)
    ax.set_title('Sensor map — mean Cp_pF per cell (hold phase)\n'
                 'dataset: 20260624_163916  |  depth = 9 mm',
                 fontsize=10, fontweight='bold')

    out = os.path.join(outdir, 'sensor_map.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── 3. Figure: Summary bar chart ──────────────────────────────────────────────
def plot_summary_bar(by_point, outdir):
    points = sorted(by_point.keys())
    means  = [np.mean(by_point[p]) for p in points]
    stds   = [np.std(by_point[p])  for p in points]
    ns     = [len(by_point[p])     for p in points]

    # colour bars by mean value
    norm = Normalize(min(means), max(means))
    colors = [CMAP(norm(m)) for m in means]

    fig, ax = plt.subplots(figsize=(13, 5))
    x = np.arange(len(points))
    bars = ax.bar(x, means, yerr=stds, capsize=4,
                  color=colors, edgecolor='#333', linewidth=0.7,
                  error_kw=dict(elinewidth=1.2, ecolor='#555'))

    ax.set_xticks(x)
    ax.set_xticklabels([f'P{p}' for p in points], fontsize=9)
    ax.set_xlabel('Sensor point (UR5 numbering)', fontsize=10)
    ax.set_ylabel('Cp (pF)', fontsize=10)
    ax.set_title('Mean Cp_pF ± std per sensor cell  (hold phase, n≈495/cell)\n'
                 'dataset: 20260624_163916  |  depth = 9 mm',
                 fontsize=10, fontweight='bold')

    # annotate n and mean
    for i, (bar, m, s, n) in enumerate(zip(bars, means, stds, ns)):
        ax.text(bar.get_x() + bar.get_width() / 2, m + s + 0.005,
                f'{m:.3f}', ha='center', va='bottom', fontsize=6.5, color='#222')

    ax.set_ylim(0, max(means) + max(stds) * 2.5)
    ax.grid(axis='y', alpha=0.3)
    ax.axhline(np.mean(means), color='steelblue', linewidth=1.2,
               linestyle='--', alpha=0.7, label=f'Grand mean = {np.mean(means):.3f} pF')
    ax.legend(fontsize=9)

    out = os.path.join(outdir, 'summary_bar.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── 4. Figure: Phase boxplots ─────────────────────────────────────────────────
def plot_phase_boxplots(by_phase, outdir):
    phases  = [p for p in PHASE_ORDER if p in by_phase and by_phase[p]]
    data    = [by_phase[p] for p in phases]
    colors  = [PHASE_COLORS[p] for p in phases]

    fig, ax = plt.subplots(figsize=(9, 5))
    bp = ax.boxplot(data, patch_artist=True, notch=False,
                    medianprops=dict(color='black', linewidth=2),
                    flierprops=dict(marker='o', markersize=2,
                                   markerfacecolor='#aaa', alpha=0.4))
    for patch, col in zip(bp['boxes'], colors):
        patch.set_facecolor(col)
        patch.set_alpha(0.85)

    ax.set_xticks(range(1, len(phases) + 1))
    ax.set_xticklabels(
        [f'{p}\n(n={len(by_phase[p])})' for p in phases], fontsize=9)
    ax.set_ylabel('Cp (pF)', fontsize=10)
    ax.set_title('Cp_pF distribution by measurement phase\n'
                 'dataset: 20260624_163916  |  depth = 9 mm',
                 fontsize=10, fontweight='bold')
    ax.grid(axis='y', alpha=0.3)

    out = os.path.join(outdir, 'phase_boxplots.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── 5. Figure: Per-round variability ──────────────────────────────────────────
def plot_round_variability(by_round, outdir):
    rounds = sorted(by_round.keys())
    data   = [by_round[r] for r in rounds]

    fig, ax = plt.subplots(figsize=(8, 4))
    bp = ax.boxplot(data, patch_artist=True,
                    medianprops=dict(color='black', linewidth=2),
                    flierprops=dict(marker='o', markersize=2,
                                   markerfacecolor='#aaa', alpha=0.4))
    for patch in bp['boxes']:
        patch.set_facecolor('#5b9bd5')
        patch.set_alpha(0.7)

    ax.set_xticks(range(1, len(rounds) + 1))
    ax.set_xticklabels([f'Round {r}' for r in rounds], fontsize=9)
    ax.set_ylabel('Cp (pF)', fontsize=10)
    ax.set_title('Cp_pF variability across measurement rounds  (hold phase)\n'
                 'dataset: 20260624_163916',
                 fontsize=10, fontweight='bold')
    ax.grid(axis='y', alpha=0.3)

    out = os.path.join(outdir, 'round_variability.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── 6. Figure: Force vs Cp scatter ────────────────────────────────────────────
def plot_force_cp(rows, outdir):
    hold = [r for r in rows
            if r['phase'] == 'hold'
            and valid_cp(r['Cp_pF'])
            and valid_cp(r.get('load_cell_N', float('nan')))]
    if not hold:
        print('  [skip] force_cp: no hold rows with load_cell_N')
        return

    forces = np.array([r['load_cell_N'] for r in hold])
    cp     = np.array([r['Cp_pF']       for r in hold])
    pts    = np.array([r['point']        for r in hold])

    # only show rows with non-trivial force variation
    if forces.std() < 0.01:
        print('  [skip] force_cp: load_cell_N has no variation (all ~constant)')
        return

    fig, ax = plt.subplots(figsize=(8, 5))
    sc = ax.scatter(forces, cp, c=pts, cmap='tab20',
                    s=4, alpha=0.45, linewidths=0)
    cb = plt.colorbar(sc, ax=ax)
    cb.set_label('Point (UR5 #)', fontsize=9)
    ax.set_xlabel('Load cell force (N)', fontsize=10)
    ax.set_ylabel('Cp (pF)', fontsize=10)
    ax.set_title('Force vs Cp_pF scatter  (hold phase)\n'
                 'dataset: 20260624_163916',
                 fontsize=10, fontweight='bold')
    ax.grid(alpha=0.25)

    out = os.path.join(outdir, 'force_cp_scatter.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── 7. Figure: Cp_pF vs time — full timeline per round ───────────────────────
PHASE_BG = {
    'locate':  ('#d0e8ff', 0.45),   # light blue
    'press':   ('#ffe0b2', 0.60),   # orange
    'hold':    ('#c8f0c8', 0.60),   # green
    'retract': ('#e8d0f0', 0.60),   # purple
    'post':    ('#e8e8e8', 0.40),   # grey
}

def plot_timeseries_per_round(rows, outdir):
    """
    One subplot per round (5 total).
    x-axis  : time in seconds relative to round start
    y-axis  : Cp_pF
    19 lines: one per sensor point, coloured by point number
    Phase bands drawn as background shading.
    """
    import matplotlib.cm as mcm

    rounds = sorted(set(r['round_idx'] for r in rows))
    points = sorted(set(r['point']     for r in rows))

    cmap19  = mcm.get_cmap('tab20', len(points))
    pt_color = {pt: cmap19(i) for i, pt in enumerate(points)}

    # global Cp range for shared y-axis
    cp_all = [r['Cp_pF'] for r in rows if valid_cp(r['Cp_pF'])]
    ymin   = min(cp_all) - 0.05
    ymax   = max(cp_all) + 0.05

    fig, axes = plt.subplots(len(rounds), 1,
                             figsize=(22, 4.5 * len(rounds)),
                             sharex=False)
    if len(rounds) == 1:
        axes = [axes]

    for ax, rnd in zip(axes, rounds):
        rnd_rows = sorted([r for r in rows if r['round_idx'] == rnd],
                          key=lambda r: r['timestamp'])
        t0 = rnd_rows[0]['timestamp']

        # ── phase background bands ──────────────────────────────────────────
        prev_phase = None
        band_start = 0.0
        phase_seq  = []   # (t_start, t_end, phase)
        for r in rnd_rows:
            t_rel = r['timestamp'] - t0
            if r['phase'] != prev_phase:
                if prev_phase is not None:
                    phase_seq.append((band_start, t_rel, prev_phase))
                band_start = t_rel
                prev_phase = r['phase']
        phase_seq.append((band_start, rnd_rows[-1]['timestamp'] - t0, prev_phase))

        drawn_phases = set()
        for t_s, t_e, ph in phase_seq:
            col, alpha = PHASE_BG.get(ph, ('#ffffff', 0.0))
            label = ph if ph not in drawn_phases else None
            ax.axvspan(t_s, t_e, color=col, alpha=alpha,
                       label=label, zorder=0)
            drawn_phases.add(ph)

        # ── one line per point ──────────────────────────────────────────────
        by_point_time = collections.defaultdict(lambda: ([], []))
        for r in rnd_rows:
            if valid_cp(r['Cp_pF']):
                t_list, cp_list = by_point_time[r['point']]
                t_list.append(r['timestamp'] - t0)
                cp_list.append(r['Cp_pF'])

        for pt in points:
            t_list, cp_list = by_point_time[pt]
            if not t_list:
                continue
            ax.plot(t_list, cp_list,
                    color=pt_color[pt], linewidth=0.9, alpha=0.85,
                    label=f'P{pt}', zorder=2)

            # label the point at its hold-phase midpoint
            pt_rnd_rows = [r for r in rnd_rows
                           if r['point'] == pt and valid_cp(r['Cp_pF'])]
            hold_pts = [(r['timestamp'] - t0, r['Cp_pF'])
                        for r in pt_rnd_rows if r['phase'] == 'hold']
            if hold_pts:
                mid_i   = len(hold_pts) // 2
                ht, hcp = hold_pts[mid_i]
                ax.annotate(f'P{pt}', xy=(ht, hcp),
                            fontsize=5.5, color=pt_color[pt],
                            fontweight='bold', zorder=5,
                            xytext=(0, 5), textcoords='offset points',
                            ha='center')

        ax.set_ylim(ymin, ymax)
        ax.set_ylabel('Cp (pF)', fontsize=9)
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_title(f'Round {rnd}  —  Cp_pF vs time  '
                     f'(19 sensor cells, full sequence from positioning to release)',
                     fontsize=9, fontweight='bold')
        ax.grid(axis='y', alpha=0.22, zorder=1)
        ax.tick_params(labelsize=8)

        # phase legend on first subplot only, point legend on all
        phase_handles = [plt.Rectangle((0,0),1,1,
                         facecolor=PHASE_BG[ph][0], alpha=0.7, label=ph)
                         for ph in PHASE_ORDER if ph in drawn_phases]
        pt_handles = [plt.Line2D([0],[0], color=pt_color[pt],
                      linewidth=1.5, label=f'P{pt}') for pt in points]

        leg1 = ax.legend(handles=phase_handles, title='Phase',
                         fontsize=7, loc='upper left',
                         ncol=len(drawn_phases), framealpha=0.9)
        ax.add_artist(leg1)
        ax.legend(handles=pt_handles, title='Point',
                  fontsize=6, loc='upper right',
                  ncol=5, framealpha=0.9)

    fig.suptitle('Capacitance vs time — 5 rounds  (20260624_163916, depth = 9 mm)\n'
                 'Each coloured line = one sensor cell  |  Background = measurement phase',
                 fontsize=11, fontweight='bold', y=1.002)
    fig.tight_layout()

    out = os.path.join(outdir, 'timeseries_per_round.png')
    fig.savefig(out, dpi=130, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── 8. Figure: Capacitance + Force + Z-position waveform per point per round ──
def plot_cp_waveforms(rows, outdir):
    """
    One figure per round (5 figures).
    Each figure: 19 subplots (4 rows × 5 cols), one per sensor point.
    Left y-axis          (blue)  : Cp_pF
    Right y-axis 1st     (orange): load_cell_N zeroed to locate baseline
    Right y-axis 2nd     (green) : tcp_z depth in mm (0 = surface, +9 = full press)
    Background shading by measurement phase.
    """
    rounds = sorted(set(r['round_idx'] for r in rows))
    points = sorted(set(r['point']     for r in rows))

    # ── global y ranges (shared across all subplots) ──────────────────────────
    cp_all = [r['Cp_pF']       for r in rows if valid_cp(r['Cp_pF'])]
    lc_all = [r['load_cell_N'] for r in rows if valid_cp(r['load_cell_N'])]
    cp_min, cp_max = min(cp_all) - 0.04, max(cp_all) + 0.04

    lc_baseline_global = np.percentile(lc_all, 5)
    lc_zeroed = np.array(lc_all) - lc_baseline_global
    lc_min = lc_zeroed.min() - 0.3
    lc_max = lc_zeroed.max() + 0.3

    # tcp_z: convert to depth in mm (0 = surface, positive = pressed deeper)
    # depth = (locate_baseline_z - tcp_z) * 1000
    # global range: ~0 to ~9.1 mm
    z_min, z_max = -0.5, 10.0

    ph_fill = {
        'locate':  '#cce5ff',
        'press':   '#ffe0b2',
        'hold':    '#c3f0c3',
        'retract': '#e8d0f0',
        'post':    '#efefef',
    }

    ncols, nrows_grid = 5, 4

    for rnd in rounds:
        rnd_rows = sorted([r for r in rows if r['round_idx'] == rnd],
                          key=lambda r: r['timestamp'])

        by_pt = collections.defaultdict(list)
        for r in rnd_rows:
            by_pt[r['point']].append(r)

        fig, axes = plt.subplots(nrows_grid, ncols,
                                 figsize=(26, nrows_grid * 3.6),
                                 sharey=False)
        # leave right margin for the extra axis
        fig.subplots_adjust(right=0.88, left=0.06,
                            hspace=0.45, wspace=0.55)
        axes_flat = axes.flatten()

        for ax_i, pt in enumerate(points):
            ax  = axes_flat[ax_i]
            pts = sorted(by_pt[pt], key=lambda r: r['timestamp'])
            if not pts:
                ax.set_visible(False)
                continue

            t0_pt  = pts[0]['timestamp']
            t_rel  = np.array([r['timestamp'] - t0_pt for r in pts])
            cp     = np.array([r['Cp_pF']       for r in pts])
            lc_raw = np.array([r['load_cell_N'] for r in pts])
            tz_raw = np.array([r['tcp_z']        for r in pts])
            phases = [r['phase'] for r in pts]

            locate_mask = np.array([ph == 'locate' for ph in phases])
            hold_mask   = np.array([ph == 'hold'   for ph in phases])

            # zero force to locate baseline
            lc_base = lc_raw[locate_mask].mean() if locate_mask.any() else lc_raw[0]
            lc      = lc_raw - lc_base

            # depth: 0 at surface, positive going deeper
            z_base = tz_raw[locate_mask].mean() if locate_mask.any() else tz_raw[0]
            depth  = (z_base - tz_raw) * 1000   # m → mm, positive = pressed

            # ── phase background bands ────────────────────────────────────
            drawn = set()
            prev_ph, t_band = phases[0], t_rel[0]
            for ti, ph in zip(t_rel[1:], phases[1:]):
                if ph != prev_ph:
                    ax.axvspan(t_band, ti,
                               color=ph_fill.get(prev_ph, '#fff'),
                               alpha=0.50, zorder=0,
                               label=prev_ph if prev_ph not in drawn else None)
                    drawn.add(prev_ph)
                    t_band, prev_ph = ti, ph
            ax.axvspan(t_band, t_rel[-1],
                       color=ph_fill.get(prev_ph, '#fff'), alpha=0.50,
                       zorder=0,
                       label=prev_ph if prev_ph not in drawn else None)

            # ── LEFT axis : Cp_pF ─────────────────────────────────────────
            ax.plot(t_rel, cp, color='#1a6eb5', linewidth=1.0,
                    alpha=0.95, zorder=3, label='Cp (pF)')
            if hold_mask.any():
                mu_cp = cp[hold_mask].mean()
                ax.axhline(mu_cp, color='#1a6eb5', linewidth=0.7,
                           linestyle='--', alpha=0.65, zorder=4,
                           label=f'Cp μ={mu_cp:.3f}')
            ax.set_ylim(cp_min, cp_max)
            ax.set_xlim(t_rel[0], t_rel[-1])
            ax.tick_params(axis='y', labelcolor='#1a6eb5', labelsize=6)
            ax.tick_params(axis='x', labelsize=6)
            ax.spines['left'].set_color('#1a6eb5')
            ax.grid(axis='y', alpha=0.18, zorder=1, color='#1a6eb5',
                    linestyle=':')
            if ax_i % ncols == 0:
                ax.set_ylabel('Cp (pF)', color='#1a6eb5', fontsize=7.5)
            if ax_i >= (nrows_grid - 1) * ncols:
                ax.set_xlabel('Time (s)', fontsize=7.5)

            # ── RIGHT axis 1 : Force ──────────────────────────────────────
            ax2 = ax.twinx()
            ax2.plot(t_rel, lc, color='#e07000', linewidth=1.0,
                     alpha=0.88, zorder=3, label='Force (N)')
            if hold_mask.any():
                mu_f = lc[hold_mask].mean()
                ax2.axhline(mu_f, color='#e07000', linewidth=0.7,
                            linestyle='--', alpha=0.65, zorder=4,
                            label=f'F μ={mu_f:.2f} N')
            ax2.set_ylim(lc_min, lc_max)
            ax2.tick_params(axis='y', labelcolor='#e07000', labelsize=6)
            ax2.spines['right'].set_color('#e07000')
            ax2.spines['left'].set_color('#1a6eb5')
            ax2.set_ylabel('Force (N)', color='#e07000', fontsize=7.5)

            # ── RIGHT axis 2 : Z depth — offset spine outward ────────────
            ax3 = ax.twinx()
            ax3.spines['right'].set_position(('outward', 48))
            ax3.spines['right'].set_color('#1a9934')
            ax3.plot(t_rel, depth, color='#1a9934', linewidth=1.0,
                     alpha=0.88, zorder=3, label='Z depth (mm)')
            if hold_mask.any():
                mu_z = depth[hold_mask].mean()
                ax3.axhline(mu_z, color='#1a9934', linewidth=0.7,
                            linestyle='--', alpha=0.65, zorder=4,
                            label=f'Z μ={mu_z:.2f} mm')
            ax3.set_ylim(z_min, z_max)
            ax3.tick_params(axis='y', labelcolor='#1a9934', labelsize=6)
            ax3.set_ylabel('Depth (mm)', color='#1a9934', fontsize=7.5)

            # ── compact combined legend ───────────────────────────────────
            h1, l1 = ax.get_legend_handles_labels()
            h2, l2 = ax2.get_legend_handles_labels()
            h3, l3 = ax3.get_legend_handles_labels()
            sig_h, sig_l = [], []
            for h, l in zip(h1 + h2 + h3, l1 + l2 + l3):
                if any(kw in l for kw in ('Cp', 'Force', 'Z depth')):
                    sig_h.append(h); sig_l.append(l)
            ax.legend(sig_h, sig_l, fontsize=5, loc='upper left',
                      handlelength=1.0, framealpha=0.80,
                      ncol=1, borderpad=0.3)

            ax.set_title(f'P{pt}', fontsize=9, fontweight='bold', pad=3)

        # hide unused slots
        for ax_i in range(len(points), len(axes_flat)):
            axes_flat[ax_i].set_visible(False)

        # shared phase legend
        phase_handles = [
            plt.Rectangle((0, 0), 1, 1,
                           facecolor=ph_fill[ph], alpha=0.65, label=ph)
            for ph in PHASE_ORDER
        ]
        fig.legend(handles=phase_handles, title='Phase', fontsize=8,
                   loc='lower center', ncol=len(PHASE_ORDER),
                   bbox_to_anchor=(0.5, 0.0), framealpha=0.9)

        fig.suptitle(
            f'Capacitance, Force & Z-depth — Round {rnd}  '
            f'(20260624_163916, depth = 9 mm)\n'
            'Blue = Cp_pF  |  Orange = Load cell force  |  Green = End-effector Z depth',
            fontsize=11, fontweight='bold')

        out = os.path.join(outdir, f'cp_waveforms_round{rnd}.png')
        fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close(fig)
        print(f'  saved: {os.path.relpath(out, HERE)}')


# ── 9. Combined overview figure ───────────────────────────────────────────────
def plot_overview(by_point, by_phase, outdir):
    """One page: sensor map + bar chart side by side + phase boxplot below."""
    means_pt = {p: np.mean(v) for p, v in by_point.items()}
    stds_pt  = {p: np.std(v)  for p, v in by_point.items()}
    points   = sorted(by_point.keys())
    all_means = [means_pt[p] for p in points]
    all_stds  = [stds_pt[p]  for p in points]

    vmin, vmax = min(all_means), max(all_means)
    norm_map   = Normalize(vmin, vmax)
    norm_bar   = Normalize(vmin, vmax)

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle('Capacitance dataset 20260624_163916 — overview\n'
                 'depth = 9 mm  |  5 rounds × 5 samples × 19 points',
                 fontsize=13, fontweight='bold', y=0.99)

    gs = gridspec.GridSpec(2, 2, figure=fig,
                           height_ratios=[1.4, 1],
                           wspace=0.35, hspace=0.45)

    ax_map = fig.add_subplot(gs[0, 0])
    ax_bar = fig.add_subplot(gs[0, 1])
    ax_box = fig.add_subplot(gs[1, :])

    # --- sensor map ---
    ax_map.set_facecolor('#111111')
    for i, (xmm, ymm) in enumerate(POINTS_MM):
        ur5_pt  = IDX_TO_UR5.get(i)
        cp_mean = means_pt.get(ur5_pt, float('nan')) if ur5_pt else float('nan')
        col     = CMAP(norm_map(cp_mean)) if not math.isnan(cp_mean) else '#333'
        h = RegularPolygon((xmm, ymm), numVertices=6, radius=HEX_R,
                           facecolor=col, edgecolor='white',
                           linewidth=0.8, zorder=2)
        ax_map.add_patch(h)
        if ur5_pt:
            ax_map.text(xmm, ymm + 1.0, f'P{ur5_pt}',
                        ha='center', va='center', fontsize=6, color='white',
                        fontweight='bold', zorder=3)
            if not math.isnan(cp_mean):
                ax_map.text(xmm, ymm - 1.5, f'{cp_mean:.3f}',
                            ha='center', va='center', fontsize=5,
                            color='white', zorder=3)
    sm_map = ScalarMappable(cmap=CMAP, norm=norm_map)
    sm_map.set_array([])
    cb_map = plt.colorbar(sm_map, ax=ax_map, fraction=0.035, pad=0.02)
    cb_map.set_label('Mean Cp (pF)', fontsize=8)
    cb_map.ax.tick_params(labelsize=7)
    ax_map.set_xlim(-24, 24); ax_map.set_ylim(-22, 22)
    ax_map.set_aspect('equal')
    ax_map.set_title('Sensor map (hold phase mean)', fontsize=9, fontweight='bold')
    ax_map.set_xlabel('X (mm)', fontsize=8)
    ax_map.set_ylabel('Y (mm)', fontsize=8)

    # --- bar chart ---
    x      = np.arange(len(points))
    colors = [CMAP(norm_bar(m)) for m in all_means]
    ax_bar.bar(x, all_means, yerr=all_stds, capsize=3,
               color=colors, edgecolor='#333', linewidth=0.5,
               error_kw=dict(elinewidth=1.0, ecolor='#444'))
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels([f'P{p}' for p in points], fontsize=7, rotation=45)
    ax_bar.set_ylabel('Cp (pF)', fontsize=9)
    ax_bar.set_title('Mean Cp ± std per cell (hold phase)', fontsize=9, fontweight='bold')
    ax_bar.axhline(np.mean(all_means), color='steelblue', linewidth=1.0,
                   linestyle='--', alpha=0.7,
                   label=f'Grand mean {np.mean(all_means):.3f} pF')
    ax_bar.set_ylim(0, max(all_means) + max(all_stds) * 3)
    ax_bar.grid(axis='y', alpha=0.25)
    ax_bar.legend(fontsize=8)
    ax_bar.tick_params(labelsize=8)

    # --- phase boxplot ---
    phases_present = [p for p in PHASE_ORDER if p in by_phase and by_phase[p]]
    data_box = [by_phase[p] for p in phases_present]
    box_colors = [PHASE_COLORS[p] for p in phases_present]
    bp = ax_box.boxplot(data_box, patch_artist=True, notch=False,
                        medianprops=dict(color='black', linewidth=1.8),
                        flierprops=dict(marker='o', markersize=2,
                                        markerfacecolor='#999', alpha=0.4))
    for patch, col in zip(bp['boxes'], box_colors):
        patch.set_facecolor(col); patch.set_alpha(0.80)
    ax_box.set_xticks(range(1, len(phases_present) + 1))
    ax_box.set_xticklabels(
        [f'{p}\n(n={len(by_phase[p])})' for p in phases_present], fontsize=9)
    ax_box.set_ylabel('Cp (pF)', fontsize=9)
    ax_box.set_title('Cp_pF distribution by measurement phase', fontsize=9, fontweight='bold')
    ax_box.grid(axis='y', alpha=0.25)

    out = os.path.join(outdir, 'overview.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── Print statistics ──────────────────────────────────────────────────────────
def print_stats(rows, by_point, by_round, by_phase):
    all_hold_cp = [v for vals in by_point.values() for v in vals]

    print('\n' + '='*60)
    print('CAPACITANCE DATASET  20260624_163916')
    print('='*60)
    print(f'  Total rows    : {len(rows)}')
    print(f'  lcr_ok == 0   : ALL rows (device flag not set — values still valid)')
    print(f'  Depth         : 9 mm')
    print(f'  Points        : 19  (P1–P19)')
    print(f'  Rounds        : 5   (0–4)')
    print(f'  Samples/round : 5   (0–4)')
    print()
    print('Hold-phase Cp_pF (pF):')
    print(f'  n             : {len(all_hold_cp)}')
    print(f'  Grand mean    : {np.mean(all_hold_cp):.4f}')
    print(f'  Grand std     : {np.std(all_hold_cp):.4f}')
    print(f'  Min  /  Max   : {min(all_hold_cp):.4f}  /  {max(all_hold_cp):.4f}')
    print()

    sorted_pts = sorted(by_point.items(), key=lambda kv: np.mean(kv[1]))
    print('Per-cell summary (hold phase, sorted by mean):')
    print(f'  {"Point":>5}  {"n":>5}  {"mean":>7}  {"std":>7}  {"min":>7}  {"max":>7}')
    for pt, vals in sorted_pts:
        a = np.array(vals)
        print(f'  P{pt:>2d}   {len(a):>5}  {a.mean():>7.4f}  {a.std():>7.4f}'
              f'  {a.min():>7.4f}  {a.max():>7.4f}')

    print()
    print('Top 5 highest mean Cp (most capacitive cells):')
    for pt, vals in sorted_pts[-5:][::-1]:
        print(f'  P{pt:>2d}: {np.mean(vals):.4f} pF  (Δ from min: '
              f'{np.mean(vals)-np.mean(sorted_pts[0][1]):+.4f} pF)')

    print()
    print('Per-round variability (hold phase):')
    for rnd in sorted(by_round.keys()):
        a = np.array(by_round[rnd])
        print(f'  Round {rnd}: mean={a.mean():.4f}  std={a.std():.4f}')

    print()
    print('Phase comparison:')
    for ph in PHASE_ORDER:
        if ph in by_phase:
            a = np.array(by_phase[ph])
            print(f'  {ph:>8}: n={len(a):>5}  mean={a.mean():.4f}  std={a.std():.4f}')
    print('='*60 + '\n')


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print(f'Loading {os.path.relpath(CSV, HERE)} ...')
    rows = load_data(CSV)

    hold, by_point, by_round, by_phase = compute_stats(rows)
    print(f'  {len(rows)} rows  |  {len(hold)} hold-phase rows with valid Cp_pF')

    print_stats(rows, by_point, by_round, by_phase)

    print(f'Generating figures → {os.path.relpath(OUT_DIR, HERE)}/')
    plot_sensor_map(by_point, OUT_DIR)
    plot_summary_bar(by_point, OUT_DIR)
    plot_phase_boxplots(by_phase, OUT_DIR)
    plot_round_variability(by_round, OUT_DIR)
    plot_force_cp(rows, OUT_DIR)
    plot_timeseries_per_round(rows, OUT_DIR)
    plot_cp_waveforms(rows, OUT_DIR)
    plot_overview(by_point, by_phase, OUT_DIR)

    print('\nDone.')


if __name__ == '__main__':
    main()