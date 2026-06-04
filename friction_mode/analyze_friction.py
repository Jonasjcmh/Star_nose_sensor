"""
analyze_friction.py  —  Star-Nose Sensor  |  Friction / Sliding Session Analysis
==================================================================================
Loads a friction-mode CSV log and produces six figure panels:

  1. Capacitive sensor  — all 19 cells time-series + heatmap
  2. UR5 force/torque   — Fx Fy Fz Tx Ty Tz time-series + box plots
  3. Load cell (FUTEK)  — AI0 → N time-series + histogram
  4. Force comparison   — robot Fz vs load cell overlay + scatter
  5. TCP trajectory     — XY sliding path coloured by max sensor value
  6. Correlation panel  — sensor vs force scatter + cell correlation matrix

Usage
-----
  python analyze_friction.py                          # latest session in logs/
  python analyze_friction.py friction_disp_line_h     # partial filename match
  python analyze_friction.py --save                   # save PNGs to plots/
  python analyze_friction.py --list                   # list available sessions
"""

import os
import sys
import glob
import argparse
from datetime import datetime

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import RegularPolygon
from matplotlib.cm import ScalarMappable
from matplotlib.collections import LineCollection

# ── Paths ─────────────────────────────────────────────────────────────────────
FRICTION_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR     = os.path.join(FRICTION_DIR, 'logs')
PLOTS_DIR    = os.path.join(FRICTION_DIR, 'plots')

# ── Sensor layout (19 active cells, hexagonal grid) ──────────────────────────
POINTS_MM = [
    (-8, +14), ( 0, +14), (+8, +14),
    (-12, +7), (-4, +7),  (+4, +7),  (+12, +7),
    (-16,  0), (-8,  0),  ( 0,  0),  ( +8,  0), (+16, 0),
    (-12, -7), (-4, -7),  (+4, -7),  (+12, -7),
    (-8, -14), ( 0, -14), (+8, -14),
]
N_CELLS = 19
CELL_COLS = [f'cell_{i+1}' for i in range(N_CELLS)]

# UR5 point index → sensor array index (120° rotation correction)
UR5_TO_IDX = {
    1:16,  2:12,  3:7,
    4:17,  5:13,  6:8,   7:3,
    8:18,  9:14,  10:9,  11:4,  12:0,
    13:15, 14:10, 15:5,  16:1,
    17:11, 18:6,  19:2,
}
POINT_ORDER = [UR5_TO_IDX[p] for p in range(1, N_CELLS + 1)]

# ── FUTEK load cell calibration ───────────────────────────────────────────────
AI0_ZERO_V       = 5.0
LOADCELL_MAX_N   = 10.0 * 4.44822   # 44.482 N
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0

def ai0_to_newtons(v):
    return -(np.asarray(v, dtype=float) - AI0_ZERO_V) * LOADCELL_N_PER_V

# ── Colour map ────────────────────────────────────────────────────────────────
CMAP = LinearSegmentedColormap.from_list(
    'star_nose', ['#2ab5a0', '#33e666', '#ffe619', '#ff7300', '#dc0000'])

matplotlib.rcParams.update({
    'figure.facecolor':  'white',
    'axes.facecolor':    'white',
    'font.family':       'DejaVu Sans',
    'axes.spines.top':   False,
    'axes.spines.right': False,
})


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description='Friction session signal analyser',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('file',   nargs='?',
                   help='CSV filename or partial name (default: latest)')
    p.add_argument('--save', action='store_true',
                   help='Save figures to friction_mode/plots/')
    p.add_argument('--list', action='store_true',
                   help='List available sessions and exit')
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# File helpers
# ─────────────────────────────────────────────────────────────────────────────
def find_all_csvs():
    return sorted(glob.glob(os.path.join(LOGS_DIR, '*.csv')))


def find_csv(arg=None):
    files = find_all_csvs()
    if not files:
        sys.exit(f'[analyze] No CSV files found in {LOGS_DIR}')
    if arg is None:
        return files[-1]
    matches = [f for f in files
               if os.path.basename(f) == arg or arg in os.path.basename(f)]
    return matches[-1] if matches else files[-1]


def session_label(path):
    base = os.path.basename(path).replace('.csv', '')
    if '_session_' in base:
        return base.split('_session_', 1)[0]
    return base


def save_dir(path):
    d = os.path.join(PLOTS_DIR,
                     os.path.basename(path).replace('.csv', ''))
    os.makedirs(d, exist_ok=True)
    return d


def savefig(fig, csv_path, suffix, do_save):
    if do_save:
        p = os.path.join(save_dir(csv_path), f'{suffix}.png')
        fig.savefig(p, dpi=150, bbox_inches='tight', facecolor='white')
        print(f'[analyze] Saved  → {p}')


# ─────────────────────────────────────────────────────────────────────────────
# Data loading
# ─────────────────────────────────────────────────────────────────────────────
def load_session(path):
    print(f'[analyze] Loading : {os.path.basename(path)}')
    df = pd.read_csv(path)

    df['t'] = df['timestamp'] - df['timestamp'].iloc[0]

    for c in CELL_COLS:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce').fillna(0.0)

    for c in ['fx', 'fy', 'fz', 'tx', 'ty', 'tz',
              'tcp_x', 'tcp_y', 'tcp_z', 'ai0',
              'ur5_pressing']:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce').fillna(0.0)

    if 'ai0' in df.columns:
        df['lc_N'] = ai0_to_newtons(df['ai0'])
        lc_min = df['lc_N'].min()
        df['lc_N'] = df['lc_N'] - lc_min

    if 'fz' in df.columns:
        fz_min = df['fz'].min()
        df['fz_c'] = df['fz'] - fz_min

    dur  = df['t'].iloc[-1]
    rate = len(df) / dur if dur > 0 else 0
    t0   = datetime.fromtimestamp(df['timestamp'].iloc[0])

    print(f'[analyze] Label   : {session_label(path)}')
    print(f'[analyze] Start   : {t0.strftime("%Y-%m-%d %H:%M:%S")}')
    print(f'[analyze] Duration: {dur:.1f} s  |  {len(df):,} rows  |  {rate:.1f} Hz')
    print(f'[analyze] Columns : {list(df.columns)}')
    return df


# ─────────────────────────────────────────────────────────────────────────────
# Hex map helper
# ─────────────────────────────────────────────────────────────────────────────
def draw_hex_map(ax, values_19, title='', vmax=None, unit=''):
    if vmax is None:
        vmax = max(float(np.max(values_19)), 1e-6)
    for i, (xmm, ymm) in enumerate(POINTS_MM):
        si  = POINT_ORDER[i]
        v   = float(values_19[si]) / vmax if si < len(values_19) else 0.0
        v   = max(0.0, min(1.0, v))
        col = CMAP(v)
        h   = RegularPolygon((xmm, ymm), numVertices=6, radius=4.5,
                             facecolor=col, edgecolor='white', linewidth=0.6)
        ax.add_patch(h)
        val = values_19[si] if si < len(values_19) else 0
        ax.text(xmm, ymm, f'{float(val):.2f}' if abs(float(val)) > 0.01 else '',
                ha='center', va='center', fontsize=5,
                color='white' if v > 0.45 else '#222')
    ax.set_xlim(-22, 22)
    ax.set_ylim(-20, 20)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title(title, fontsize=9, fontweight='bold', pad=4)
    sm = ScalarMappable(cmap=CMAP, norm=Normalize(0, vmax))
    sm.set_array([])
    plt.colorbar(sm, ax=ax, shrink=0.55, pad=0.02,
                 label=f'0 – {vmax:.3f}{unit}')


# ─────────────────────────────────────────────────────────────────────────────
# Figure 1 — Capacitive sensor signals
# ─────────────────────────────────────────────────────────────────────────────
def fig_sensor(df, csv_path, save):
    label    = session_label(csv_path)
    t        = df['t'].to_numpy()
    cell_arr = df[CELL_COLS].to_numpy()    # (frames, 19)
    pressing = (df.get('ur5_pressing', pd.Series(0, index=df.index)) == 1).to_numpy()

    fig = plt.figure(figsize=(20, 11))
    fig.suptitle(f'Capacitive sensor — {label}', fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.50, wspace=0.38)

    # ── Timeline heatmap (all 19 cells, ordered P1→P19) ──────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    data = cell_arr.T[POINT_ORDER, :]
    im   = ax1.imshow(data, aspect='auto', cmap=CMAP, vmin=0, vmax=1,
                      extent=[0, t[-1], N_CELLS + 0.5, 0.5])
    ax1.fill_between(t, N_CELLS + 0.5, 0.5,
                     where=pressing, alpha=0.08, color='steelblue')
    ax1.set_xlabel('Time (s)', fontsize=9)
    ax1.set_ylabel('Cell (P1 → P19)', fontsize=9)
    ax1.set_title('All 19 cells — full session', fontsize=10)
    ax1.set_yticks(range(1, N_CELLS + 1))
    ax1.set_yticklabels([f'P{p}' for p in range(1, N_CELLS + 1)], fontsize=6)
    plt.colorbar(im, ax=ax1, label='Pressure (norm.)', shrink=0.9)

    # ── Individual cell time-series (first 6 cells for clarity) ──────────────
    ax2 = fig.add_subplot(gs[1, :2])
    colors6 = plt.cm.tab10(np.linspace(0, 0.6, 6))
    for i in range(6):
        ax2.plot(t, cell_arr[:, i], linewidth=0.7, alpha=0.85,
                 label=f'P{i+1}', color=colors6[i])
    ax2.fill_between(t, 0, 1.05, where=pressing, alpha=0.08,
                     color='steelblue', label='Sliding')
    ax2.set_xlabel('Time (s)', fontsize=9)
    ax2.set_ylabel('Pressure (norm.)', fontsize=9)
    ax2.set_title('Cells P1–P6 time-series', fontsize=10)
    ax2.set_ylim(-0.02, 1.05)
    ax2.legend(fontsize=7, ncol=3, loc='upper right')
    ax2.grid(alpha=0.25)

    # ── Max cell activity over time ───────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 2])
    max_cell = cell_arr.max(axis=1)
    mean_cell = cell_arr.mean(axis=1)
    ax3.plot(t, max_cell,  linewidth=0.9, color='#dc0000', label='Max cell')
    ax3.plot(t, mean_cell, linewidth=0.7, color='#2ab5a0', alpha=0.8,
             label='Mean all cells')
    ax3.fill_between(t, 0, 1.05, where=pressing, alpha=0.10,
                     color='steelblue')
    ax3.set_xlabel('Time (s)', fontsize=9)
    ax3.set_ylabel('Pressure (norm.)', fontsize=9)
    ax3.set_title('Max / mean sensor activity', fontsize=10)
    ax3.set_ylim(-0.02, 1.05)
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.25)

    # ── Mean activation hex map ───────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 0])
    draw_hex_map(ax4, cell_arr.mean(axis=0),
                 title='Mean activation (full session)')

    # ── Peak activation hex map ───────────────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 1])
    draw_hex_map(ax5, cell_arr.max(axis=0),
                 title='Peak activation (full session)')

    # ── Cell activity distribution ────────────────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 2])
    means = cell_arr.mean(axis=0)
    colors_bar = [CMAP(v / max(means.max(), 1e-6)) for v in means]
    ax6.bar(range(N_CELLS), means, color=colors_bar, edgecolor='white', linewidth=0.4)
    ax6.set_xticks(range(N_CELLS))
    ax6.set_xticklabels([f'P{i+1}' for i in range(N_CELLS)],
                        rotation=90, fontsize=6)
    ax6.set_ylabel('Mean activation', fontsize=9)
    ax6.set_title('Mean activation per cell', fontsize=10)
    ax6.grid(axis='y', alpha=0.25)

    plt.tight_layout()
    savefig(fig, csv_path, '1_sensor', save)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# Figure 2 — UR5 force / torque
# ─────────────────────────────────────────────────────────────────────────────
def fig_force(df, csv_path, save):
    if 'fz' not in df.columns:
        print('[analyze] No force data — skipping force figure')
        return

    label    = session_label(csv_path)
    t        = df['t'].to_numpy()
    pressing = (df.get('ur5_pressing', pd.Series(0, index=df.index)) == 1).to_numpy()
    ft_cols  = [c for c in ['fx', 'fy', 'fz', 'tx', 'ty', 'tz'] if c in df.columns]

    fig = plt.figure(figsize=(20, 11))
    fig.suptitle(f'UR5 force / torque — {label}', fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.50, wspace=0.38)

    # ── Force time-series ─────────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    force_colors = {'fx': '#2ab5a0', 'fy': '#EF9F27', 'fz': '#dc0000',
                    'tx': '#2ab5a0', 'ty': '#EF9F27', 'tz': '#dc0000'}
    force_ls     = {'fx': '-', 'fy': '-', 'fz': '-',
                    'tx': '--', 'ty': '--', 'tz': '--'}
    for c in ft_cols:
        ax1.plot(t, df[c].to_numpy(), linewidth=0.8, alpha=0.85,
                 color=force_colors.get(c, 'gray'),
                 linestyle=force_ls.get(c, '-'), label=c)
    fmin = df[ft_cols].min().min()
    fmax = df[ft_cols].max().max()
    ax1.fill_between(t, fmin, fmax, where=pressing, alpha=0.09,
                     color='steelblue', label='Sliding')
    ax1.axhline(0, color='black', linewidth=0.5, linestyle=':', alpha=0.5)
    ax1.set_xlabel('Time (s)', fontsize=9)
    ax1.set_ylabel('Force (N) / Torque (Nm)', fontsize=9)
    ax1.set_title('TCP force & torque — full session', fontsize=10)
    ax1.legend(fontsize=8, ncol=4, loc='upper right')
    ax1.grid(alpha=0.25)

    # ── Fz time-series zoomed ─────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, df['fz'].to_numpy(), linewidth=0.9, color='#dc0000', label='Fz')
    if 'fz_c' in df.columns:
        ax2.plot(t, df['fz_c'].to_numpy(), linewidth=0.7, color='#ff7300',
                 linestyle='--', alpha=0.7, label='Fz (zeroed)')
    ax2.fill_between(t, df['fz'].min(), df['fz'].max(),
                     where=pressing, alpha=0.09, color='steelblue')
    ax2.set_xlabel('Time (s)', fontsize=9)
    ax2.set_ylabel('Fz (N)', fontsize=9)
    ax2.set_title('Contact force Fz', fontsize=10)
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.25)

    # ── Fz rolling statistics ─────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 1])
    window = max(5, int(0.5 / (df['t'].diff().median() or 0.05)))
    fz_roll_mean = df['fz'].rolling(window, center=True).mean()
    fz_roll_std  = df['fz'].rolling(window, center=True).std()
    ax3.plot(t, fz_roll_mean.to_numpy(), linewidth=1.0, color='#dc0000',
             label=f'Rolling mean ({window} frames)')
    ax3.fill_between(t,
                     (fz_roll_mean - fz_roll_std).to_numpy(),
                     (fz_roll_mean + fz_roll_std).to_numpy(),
                     alpha=0.25, color='#dc0000', label='±1 std')
    ax3.fill_between(t, fz_roll_mean.min(), fz_roll_mean.max(),
                     where=pressing, alpha=0.09, color='steelblue')
    ax3.set_xlabel('Time (s)', fontsize=9)
    ax3.set_ylabel('Fz (N)', fontsize=9)
    ax3.set_title('Fz rolling statistics', fontsize=10)
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.25)

    # ── Box plots (all components) ────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 0])
    slide_df = df[df.get('ur5_pressing', 0) == 1] if pressing.any() else df
    bdata  = [slide_df[c].abs().dropna().values for c in ft_cols]
    bp = ax4.boxplot(bdata, labels=ft_cols, patch_artist=True,
                     medianprops={'color': 'white', 'linewidth': 1.5})
    box_colors = ['#2ab5a0', '#EF9F27', '#dc0000', '#2ab5a0', '#EF9F27', '#dc0000']
    for patch, col in zip(bp['boxes'], box_colors[:len(ft_cols)]):
        patch.set_facecolor(col)
        patch.set_alpha(0.75)
    ax4.set_ylabel('|Value|', fontsize=9)
    ax4.set_title('Force/torque distribution (sliding)', fontsize=10)
    ax4.grid(axis='y', alpha=0.25)

    # ── Force magnitude over time ─────────────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 1])
    force_only = [c for c in ['fx', 'fy', 'fz'] if c in df.columns]
    if force_only:
        fmag = np.sqrt(sum(df[c].to_numpy()**2 for c in force_only))
        ax5.plot(t, fmag, linewidth=0.9, color='#9b59b6', label='|F| magnitude')
        ax5.fill_between(t, 0, fmag.max(), where=pressing,
                         alpha=0.09, color='steelblue')
        ax5.set_xlabel('Time (s)', fontsize=9)
        ax5.set_ylabel('|F| (N)', fontsize=9)
        ax5.set_title('Total force magnitude', fontsize=10)
        ax5.legend(fontsize=8)
        ax5.grid(alpha=0.25)

    plt.tight_layout()
    savefig(fig, csv_path, '2_force', save)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# Figure 3 — Load cell (FUTEK, AI0)
# ─────────────────────────────────────────────────────────────────────────────
def fig_loadcell(df, csv_path, save):
    has_ai0 = 'ai0' in df.columns and df['ai0'].abs().max() > 1e-6
    if not has_ai0:
        print('[analyze] No AI0 load cell data — skipping load cell figure')
        return

    label    = session_label(csv_path)
    t        = df['t'].to_numpy()
    lc       = df['lc_N'].to_numpy()
    ai0      = df['ai0'].to_numpy()
    pressing = (df.get('ur5_pressing', pd.Series(0, index=df.index)) == 1).to_numpy()

    fig = plt.figure(figsize=(18, 10))
    fig.suptitle(
        f'FUTEK load cell (AI0 → N) — {label}\n'
        f'Calibration: {AI0_ZERO_V:.1f} V = 0 N  |  '
        f'{LOADCELL_N_PER_V:.3f} N/V  |  max {LOADCELL_MAX_N:.1f} N',
        fontsize=11, fontweight='bold')
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38)

    # ── Load cell (N) time-series ─────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, lc, linewidth=0.9, color='#9b59b6', label='Load cell (N, baseline removed)')
    ax1.fill_between(t, lc.min(), lc.max(), where=pressing,
                     alpha=0.10, color='steelblue', label='Sliding')
    ax1.axhline(0, color='black', linewidth=0.5, linestyle=':', alpha=0.5)
    ax1b = ax1.twinx()
    ax1b.plot(t, ai0, linewidth=0.6, color='#bdc3c7', alpha=0.5,
              linestyle='--', label='AI0 raw (V)')
    ax1b.set_ylabel('AI0 (V)', fontsize=9, color='#999')
    ax1b.tick_params(axis='y', labelcolor='#999', labelsize=7)
    lines1, lbl1 = ax1.get_legend_handles_labels()
    lines2, lbl2 = ax1b.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, lbl1 + lbl2, fontsize=8, loc='upper right')
    ax1.set_xlabel('Time (s)', fontsize=9)
    ax1.set_ylabel('Force (N)', fontsize=9)
    ax1.set_title('Load cell force — full session', fontsize=10)
    ax1.grid(alpha=0.25)

    # ── AI0 voltage distribution ──────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.hist(ai0, bins=60, color='#9b59b6', edgecolor='white', alpha=0.85)
    ax2.axvline(AI0_ZERO_V, color='red', linewidth=1.2, linestyle='--',
                label=f'Zero = {AI0_ZERO_V} V')
    ax2.axvline(float(np.mean(ai0)), color='orange', linewidth=1.0, linestyle='--',
                label=f'Mean = {np.mean(ai0):.3f} V')
    ax2.set_xlabel('AI0 (V)', fontsize=9)
    ax2.set_ylabel('Count', fontsize=9)
    ax2.set_title('AI0 voltage distribution', fontsize=10)
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.25)

    # ── Load cell force distribution ──────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.hist(lc, bins=60, color='#e056b6', edgecolor='white', alpha=0.85)
    ax3.axvline(float(np.mean(lc)), color='red', linewidth=1.0, linestyle='--',
                label=f'Mean = {np.mean(lc):.2f} N')
    ax3.axvline(float(np.max(lc)), color='orange', linewidth=1.0, linestyle=':',
                label=f'Max = {np.max(lc):.2f} N')
    ax3.set_xlabel('Force (N)', fontsize=9)
    ax3.set_ylabel('Count', fontsize=9)
    ax3.set_title('Load cell force distribution', fontsize=10)
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.25)

    # ── Load cell summary stats ───────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 2])
    ax4.axis('off')
    lc_slide = lc[pressing] if pressing.any() else lc
    stats = [
        ('Duration',      f'{t[-1]:.1f} s'),
        ('Sample rate',   f'{len(df) / t[-1]:.1f} Hz'),
        ('AI0 zero ref',  f'{AI0_ZERO_V:.2f} V'),
        ('Sensitivity',   f'{LOADCELL_N_PER_V:.3f} N/V'),
        ('Rated capacity',f'{LOADCELL_MAX_N:.1f} N'),
        ('',              ''),
        ('Full session',  ''),
        ('  Mean LC',     f'{np.mean(lc):.3f} N'),
        ('  Max  LC',     f'{np.max(lc):.3f} N'),
        ('  Std  LC',     f'{np.std(lc):.3f} N'),
        ('',              ''),
        ('Sliding only',  ''),
        ('  Mean LC',     f'{np.mean(lc_slide):.3f} N'),
        ('  Max  LC',     f'{np.max(lc_slide):.3f} N'),
    ]
    ax4.text(0.05, 0.97, 'Load cell statistics',
             fontweight='bold', fontsize=11, transform=ax4.transAxes)
    y = 0.88
    for lbl, val in stats:
        if lbl == '':
            y -= 0.04
            continue
        ax4.text(0.05, y, lbl, fontsize=8, color='gray',
                 transform=ax4.transAxes)
        ax4.text(0.60, y, val, fontsize=8, fontweight='bold',
                 transform=ax4.transAxes)
        y -= 0.075

    plt.tight_layout()
    savefig(fig, csv_path, '3_loadcell', save)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# Figure 4 — Force comparison: robot Fz vs FUTEK load cell
# ─────────────────────────────────────────────────────────────────────────────
def fig_force_comparison(df, csv_path, save):
    has_fz  = 'fz'  in df.columns
    has_lc  = 'lc_N' in df.columns
    if not has_fz or not has_lc:
        print('[analyze] Missing Fz or AI0 — skipping force comparison figure')
        return

    label    = session_label(csv_path)
    t        = df['t'].to_numpy()
    fz_c     = df['fz_c'].to_numpy()
    lc       = df['lc_N'].to_numpy()
    pressing = (df.get('ur5_pressing', pd.Series(0, index=df.index)) == 1).to_numpy()

    fig = plt.figure(figsize=(18, 10))
    fig.suptitle(f'Robot Fz vs FUTEK load cell — {label}',
                 fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38)

    # ── Overlay time-series ───────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, fz_c, linewidth=0.9, color='#dc0000', label='Robot Fz (N, zeroed)')
    ax1.plot(t, lc,   linewidth=0.9, color='#9b59b6', alpha=0.85,
             label='FUTEK load cell (N, zeroed)')
    fmin = min(fz_c.min(), lc.min())
    fmax = max(fz_c.max(), lc.max())
    ax1.fill_between(t, fmin, fmax, where=pressing,
                     alpha=0.09, color='steelblue', label='Sliding')
    ax1.axhline(0, color='black', linewidth=0.5, linestyle=':', alpha=0.4)
    ax1.set_xlabel('Time (s)', fontsize=9)
    ax1.set_ylabel('Force (N)', fontsize=9)
    ax1.set_title('Force sensor overlay — full session', fontsize=10)
    ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(alpha=0.25)

    # ── Scatter correlation ───────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 0])
    sc = ax2.scatter(lc, fz_c, alpha=0.25, s=6,
                     c=df['t'].to_numpy(), cmap='viridis')
    plt.colorbar(sc, ax=ax2, label='Time (s)', shrink=0.85)
    lim = [min(lc.min(), fz_c.min()) * 1.05,
           max(lc.max(), fz_c.max()) * 1.05]
    ax2.plot(lim, lim, 'k--', linewidth=0.8, alpha=0.5, label='1:1 line')
    if len(lc) > 10:
        r = float(np.corrcoef(lc, fz_c)[0, 1])
        ax2.text(0.05, 0.93, f'r = {r:.3f}', transform=ax2.transAxes,
                 fontsize=9, fontweight='bold', color='#333')
    ax2.set_xlabel('FUTEK (N)', fontsize=9)
    ax2.set_ylabel('Robot Fz (N)', fontsize=9)
    ax2.set_title('Correlation (all frames)', fontsize=10)
    ax2.legend(fontsize=7)
    ax2.grid(alpha=0.25)

    # ── Residuals ─────────────────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 1])
    residuals = fz_c - lc
    ax3.plot(t, residuals, linewidth=0.7, color='#e67e22', alpha=0.8)
    ax3.axhline(0,                  color='black', linewidth=0.8, linestyle='--')
    ax3.axhline(residuals.mean(),   color='red',   linewidth=0.9, linestyle=':',
                label=f'Mean = {residuals.mean():.2f} N')
    ax3.fill_between(t, residuals.min(), residuals.max(), where=pressing,
                     alpha=0.09, color='steelblue')
    ax3.set_xlabel('Time (s)', fontsize=9)
    ax3.set_ylabel('Robot Fz − Load cell (N)', fontsize=9)
    ax3.set_title('Residuals (robot − load cell)', fontsize=10)
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.25)

    # ── Bland–Altman ──────────────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 2])
    mean_sig = (lc + fz_c) / 2.0
    diff_sig = fz_c - lc
    md  = float(diff_sig.mean())
    std = float(diff_sig.std())
    ax4.scatter(mean_sig, diff_sig, alpha=0.25, s=6, color='#2980b9')
    ax4.axhline(md,             color='red',  linewidth=1.0,
                label=f'Bias = {md:.2f} N')
    ax4.axhline(md + 1.96*std,  color='gray', linewidth=0.8, linestyle='--',
                label=f'+1.96σ = {md+1.96*std:.2f} N')
    ax4.axhline(md - 1.96*std,  color='gray', linewidth=0.8, linestyle='--',
                label=f'−1.96σ = {md-1.96*std:.2f} N')
    ax4.set_xlabel('Mean of two sensors (N)', fontsize=9)
    ax4.set_ylabel('Robot − Load cell (N)', fontsize=9)
    ax4.set_title('Bland–Altman agreement', fontsize=10)
    ax4.legend(fontsize=7)
    ax4.grid(alpha=0.25)

    plt.tight_layout()
    savefig(fig, csv_path, '4_force_comparison', save)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# Figure 5 — TCP trajectory
# ─────────────────────────────────────────────────────────────────────────────
def fig_trajectory(df, csv_path, save):
    if 'tcp_x' not in df.columns:
        print('[analyze] No TCP pose data — skipping trajectory figure')
        return

    label    = session_label(csv_path)
    t        = df['t'].to_numpy()
    tx       = df['tcp_x'].to_numpy() * 1000   # m → mm
    ty       = df['tcp_y'].to_numpy() * 1000
    tz       = df['tcp_z'].to_numpy() * 1000
    pressing = (df.get('ur5_pressing', pd.Series(0, index=df.index)) == 1).to_numpy()
    max_cell = df[CELL_COLS].max(axis=1).to_numpy()

    fig = plt.figure(figsize=(18, 10))
    fig.suptitle(f'TCP trajectory — {label}', fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.50, wspace=0.42)

    # ── XY path coloured by max sensor value ─────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :2])
    pts  = np.array([tx, ty]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc_col = LineCollection(segs, cmap=CMAP,
                            norm=Normalize(0, max(max_cell.max(), 1e-6)),
                            linewidth=1.5, alpha=0.85)
    lc_col.set_array(max_cell[:-1])
    ax1.add_collection(lc_col)
    plt.colorbar(lc_col, ax=ax1, label='Max cell activation')
    ax1.scatter(tx[0],  ty[0],  s=80, color='green',  zorder=5, label='Start')
    ax1.scatter(tx[-1], ty[-1], s=80, color='red',    zorder=5, label='End')
    ax1.set_xlabel('TCP X (mm)', fontsize=9)
    ax1.set_ylabel('TCP Y (mm)', fontsize=9)
    ax1.set_title('XY sliding path — coloured by sensor max', fontsize=10)
    ax1.set_aspect('equal')
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.25)

    # ── Z depth over time ─────────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(t, tz, linewidth=0.9, color='#2ab5a0')
    ax2.fill_between(t, tz.min(), tz.max(), where=pressing,
                     alpha=0.12, color='steelblue', label='Sliding')
    ax2.set_xlabel('Time (s)', fontsize=9)
    ax2.set_ylabel('TCP Z (mm)', fontsize=9)
    ax2.set_title('TCP Z — indentation depth', fontsize=10)
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.25)

    # ── X over time ───────────────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t, tx, linewidth=0.9, color='#dc0000', label='TCP X')
    ax3.fill_between(t, tx.min(), tx.max(), where=pressing,
                     alpha=0.10, color='steelblue')
    ax3.set_xlabel('Time (s)', fontsize=9)
    ax3.set_ylabel('X (mm)', fontsize=9)
    ax3.set_title('TCP X over time', fontsize=10)
    ax3.grid(alpha=0.25)

    # ── Y over time ───────────────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, ty, linewidth=0.9, color='#EF9F27', label='TCP Y')
    ax4.fill_between(t, ty.min(), ty.max(), where=pressing,
                     alpha=0.10, color='steelblue')
    ax4.set_xlabel('Time (s)', fontsize=9)
    ax4.set_ylabel('Y (mm)', fontsize=9)
    ax4.set_title('TCP Y over time', fontsize=10)
    ax4.grid(alpha=0.25)

    # ── XY path coloured by Fz (if available) ────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 2])
    if 'fz_c' in df.columns:
        fz_c = df['fz_c'].to_numpy()
        lc2  = LineCollection(segs, cmap='RdYlGn_r',
                              norm=Normalize(0, max(fz_c.max(), 1e-6)),
                              linewidth=1.5, alpha=0.85)
        lc2.set_array(fz_c[:-1])
        ax5.add_collection(lc2)
        plt.colorbar(lc2, ax=ax5, label='|Fz| (N)')
        ax5.scatter(tx[0],  ty[0],  s=60, color='green', zorder=5)
        ax5.scatter(tx[-1], ty[-1], s=60, color='red',   zorder=5)
        ax5.set_xlabel('TCP X (mm)', fontsize=9)
        ax5.set_ylabel('TCP Y (mm)', fontsize=9)
        ax5.set_title('XY path — coloured by Fz', fontsize=10)
        ax5.set_aspect('equal')
        ax5.grid(alpha=0.25)
    else:
        ax5.axis('off')
        ax5.text(0.5, 0.5, 'No Fz data', ha='center', va='center',
                 transform=ax5.transAxes, fontsize=11, color='gray')

    plt.tight_layout()
    savefig(fig, csv_path, '5_trajectory', save)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# Figure 6 — Cross-correlation: sensor vs force
# ─────────────────────────────────────────────────────────────────────────────
def fig_correlation(df, csv_path, save):
    label    = session_label(csv_path)
    cell_arr = df[CELL_COLS].to_numpy()
    max_cell = cell_arr.max(axis=1)
    pressing = (df.get('ur5_pressing', pd.Series(0, index=df.index)) == 1).to_numpy()

    fig = plt.figure(figsize=(18, 10))
    fig.suptitle(f'Cross-correlation analysis — {label}', fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.48, wspace=0.40)

    # ── Cell-to-cell correlation matrix ──────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :2])
    slide_cells = cell_arr[pressing] if pressing.any() else cell_arr
    if len(slide_cells) > 10:
        corr = np.corrcoef(slide_cells.T)
        im = ax1.imshow(corr, cmap='RdBu_r', vmin=-1, vmax=1)
        ax1.set_xticks(range(N_CELLS))
        ax1.set_yticks(range(N_CELLS))
        ax1.set_xticklabels([f'P{i+1}' for i in range(N_CELLS)],
                            rotation=90, fontsize=6)
        ax1.set_yticklabels([f'P{i+1}' for i in range(N_CELLS)], fontsize=6)
        plt.colorbar(im, ax=ax1, label='Pearson r', shrink=0.85)
        ax1.set_title('Cell-to-cell correlation (sliding frames)', fontsize=10)

    # ── Mean cell activation hex map during sliding ───────────────────────────
    ax2 = fig.add_subplot(gs[0, 2])
    slide_mean = (slide_cells.mean(axis=0)
                  if len(slide_cells) > 0 else cell_arr.mean(axis=0))
    draw_hex_map(ax2, slide_mean, title='Mean activation (sliding)')

    # ── Sensor max vs Fz scatter ──────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    if 'fz_c' in df.columns:
        fz_c = df['fz_c'].to_numpy()
        sc = ax3.scatter(fz_c, max_cell, alpha=0.25, s=6,
                         c=df['t'].to_numpy(), cmap='viridis')
        plt.colorbar(sc, ax=ax3, label='Time (s)', shrink=0.85)
        mask = (fz_c < fz_c.mean() + 3*fz_c.std()) & (max_cell < 1.05)
        if mask.sum() > 20:
            coef = np.polyfit(fz_c[mask], max_cell[mask], 1)
            xf   = np.linspace(fz_c[mask].min(), fz_c[mask].max(), 100)
            r2   = np.corrcoef(fz_c[mask], max_cell[mask])[0, 1]**2
            ax3.plot(xf, np.polyval(coef, xf), 'r--', linewidth=1.4,
                     label=f'slope={coef[0]:.4f}  r²={r2:.3f}')
            ax3.legend(fontsize=7)
        ax3.set_xlabel('Robot Fz (N, zeroed)', fontsize=9)
        ax3.set_ylabel('Max sensor activation', fontsize=9)
        ax3.set_title('Fz vs sensor', fontsize=10)
        ax3.grid(alpha=0.25)
    else:
        ax3.axis('off')
        ax3.text(0.5, 0.5, 'No Fz data', ha='center', va='center',
                 transform=ax3.transAxes, fontsize=11, color='gray')

    # ── Sensor max vs load cell scatter ──────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    if 'lc_N' in df.columns:
        lc = df['lc_N'].to_numpy()
        sc4 = ax4.scatter(lc, max_cell, alpha=0.25, s=6,
                          c=df['t'].to_numpy(), cmap='plasma')
        plt.colorbar(sc4, ax=ax4, label='Time (s)', shrink=0.85)
        mask4 = (lc < lc.mean() + 3*lc.std()) & (max_cell < 1.05)
        if mask4.sum() > 20:
            coef4 = np.polyfit(lc[mask4], max_cell[mask4], 1)
            xf4   = np.linspace(lc[mask4].min(), lc[mask4].max(), 100)
            r2_4  = np.corrcoef(lc[mask4], max_cell[mask4])[0, 1]**2
            ax4.plot(xf4, np.polyval(coef4, xf4), 'r--', linewidth=1.4,
                     label=f'slope={coef4[0]:.4f}  r²={r2_4:.3f}')
            ax4.legend(fontsize=7)
        ax4.set_xlabel('Load cell (N, zeroed)', fontsize=9)
        ax4.set_ylabel('Max sensor activation', fontsize=9)
        ax4.set_title('Load cell vs sensor', fontsize=10)
        ax4.grid(alpha=0.25)
    else:
        ax4.axis('off')
        ax4.text(0.5, 0.5, 'No AI0 data', ha='center', va='center',
                 transform=ax4.transAxes, fontsize=11, color='gray')

    # ── Mean activation per cell (bar) ────────────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 2])
    means = slide_mean
    colors_bar = [CMAP(v / max(means.max(), 1e-6)) for v in means]
    ax5.bar(range(N_CELLS), means, color=colors_bar, edgecolor='white', linewidth=0.4)
    ax5.set_xticks(range(N_CELLS))
    ax5.set_xticklabels([f'P{i+1}' for i in range(N_CELLS)],
                        rotation=90, fontsize=6)
    ax5.set_ylabel('Mean activation', fontsize=9)
    ax5.set_title('Mean cell activation (sliding)', fontsize=10)
    ax5.grid(axis='y', alpha=0.25)

    plt.tight_layout()
    savefig(fig, csv_path, '6_correlation', save)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    if args.list:
        files = find_all_csvs()
        if not files:
            print(f'No sessions found in {LOGS_DIR}')
            return
        print(f'\nSessions in {LOGS_DIR}:\n')
        for f in files:
            size = os.path.getsize(f) / 1024
            print(f'  {os.path.basename(f)}  ({size:.0f} KB)')
        print()
        return

    csv_path = find_csv(args.file)
    print(f'\n{"=" * 62}')
    print(f'  Friction Session Analyser')
    print(f'{"=" * 62}')

    df = load_session(csv_path)

    print(f'\n[analyze] Generating figures ...')
    print(f'[analyze] Output: {"saved to plots/" if args.save else "screen only"}')
    print()

    fig_sensor(df,            csv_path, args.save)
    fig_force(df,             csv_path, args.save)
    fig_loadcell(df,          csv_path, args.save)
    fig_force_comparison(df,  csv_path, args.save)
    fig_trajectory(df,        csv_path, args.save)
    fig_correlation(df,       csv_path, args.save)

    if args.save:
        print(f'\n[analyze] All figures saved → {save_dir(csv_path)}')
    print('[analyze] Done!')


if __name__ == '__main__':
    try:
        import pandas, numpy, matplotlib
    except ImportError:
        print('Installing dependencies ...')
        os.system(f'{sys.executable} -m pip install matplotlib pandas numpy')
    main()
