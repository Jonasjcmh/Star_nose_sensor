#!/usr/bin/env python3
"""
trajectory_comparison.py
─────────────────────────────────────────────────────────────────────────────
For every session in friction_mode/logs/ produces one figure:

  TOP-LEFT  — Real TCP XY path overlaid on the hex sensor map
               (cells coloured by mean activation; bold red border on
               activated cells; path coloured by elapsed time)
  TOP-RIGHT — Peak activation hex map (full session)
  BOTTOM    — Z-force: UR5 Fz (red) vs FUTEK load cell (purple)

Output: friction_mode/trajectory_analysis/<session_label>.png

Usage
-----
  python trajectory_comparison.py             # all sessions
  python trajectory_comparison.py line_1mm    # sessions matching substring
"""

import os
import sys
import glob
import argparse

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import RegularPolygon
from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable

# ── Paths ─────────────────────────────────────────────────────────────────────
FRICTION_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR     = os.path.join(FRICTION_DIR, 'logs')
OUT_DIR      = os.path.join(FRICTION_DIR, 'trajectory_analysis')

# ── Sensor layout (UR5 point positions in mm, robot frame) ───────────────────
POINTS_MM = [
    ( -8, +14), (  0, +14), ( +8, +14),
    (-12,  +7), ( -4,  +7), ( +4,  +7), (+12,  +7),
    (-16,   0), ( -8,   0), (  0,   0), ( +8,   0), (+16,   0),
    (-12,  -7), ( -4,  -7), ( +4,  -7), (+12,  -7),
    ( -8, -14), (  0, -14), ( +8, -14),
]
N_CELLS   = 19
CELL_COLS = [f'cell_{i+1}' for i in range(N_CELLS)]

# UR5 point (1-19) → sensor array index (120° rotation correction)
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
LOADCELL_MAX_N   = 10.0 * 4.44822
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0

def ai0_to_newtons(v):
    return -(np.asarray(v, dtype=float) - AI0_ZERO_V) * LOADCELL_N_PER_V

# ── Style ─────────────────────────────────────────────────────────────────────
CMAP = LinearSegmentedColormap.from_list(
    'star_nose', ['#2ab5a0', '#33e666', '#ffe619', '#ff7300', '#dc0000'])

matplotlib.rcParams.update({
    'figure.facecolor': 'white',
    'axes.facecolor':   'white',
    'font.family':      'DejaVu Sans',
    'axes.spines.top':  False,
    'axes.spines.right':False,
})

ACT_THRESHOLD = 0.08   # cells above this mean are "activated"
HEX_RADIUS_MM = 4.2    # display radius per hexagon


# ─────────────────────────────────────────────────────────────────────────────
def session_label(path):
    base = os.path.basename(path).replace('.csv', '')
    if '_session_' in base:
        return base.split('_session_', 1)[0]
    return base


def load_session(path):
    df = pd.read_csv(path)
    df['t'] = df['timestamp'] - df['timestamp'].iloc[0]

    for c in CELL_COLS:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce').fillna(0.0)

    for c in ['fx', 'fy', 'fz', 'tcp_x', 'tcp_y', 'tcp_z', 'ai0', 'ur5_pressing']:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce').fillna(0.0)

    if 'ai0' in df.columns:
        df['lc_N'] = ai0_to_newtons(df['ai0'])
        df['lc_N'] -= df['lc_N'].min()

    if 'fz' in df.columns:
        df['fz_c'] = df['fz'] - df['fz'].min()

    return df


# ─────────────────────────────────────────────────────────────────────────────
def _draw_hex_cells(ax, cell_values, vmax=None, show_labels=True):
    """
    Draw the 19 hexagonal cells on ax.
    cell_values: array indexed by sensor index (19 values).
    Cells above ACT_THRESHOLD get a bold red border.
    Returns the ScalarMappable for colorbar.
    """
    if vmax is None:
        vmax = max(float(np.max(cell_values)), 1e-6)
    norm = Normalize(0, vmax)

    for i, (xmm, ymm) in enumerate(POINTS_MM):
        si  = POINT_ORDER[i]
        val = float(cell_values[si]) if si < len(cell_values) else 0.0
        v   = max(0.0, min(1.0, val / vmax))
        col = CMAP(v)
        activated = val > ACT_THRESHOLD

        h = RegularPolygon(
            (xmm, ymm), numVertices=6, radius=HEX_RADIUS_MM,
            facecolor=col,
            edgecolor='#e74c3c' if activated else 'white',
            linewidth=2.2 if activated else 0.6,
            zorder=2,
        )
        ax.add_patch(h)

        if show_labels:
            ax.text(xmm, ymm, f'P{i+1}',
                    ha='center', va='center', fontsize=5,
                    color='white' if v > 0.45 else '#333',
                    fontweight='bold', zorder=3)

    ax.set_xlim(-24, 24)
    ax.set_ylim(-22, 22)
    ax.set_aspect('equal')

    sm = ScalarMappable(cmap=CMAP, norm=norm)
    sm.set_array([])
    return sm


# ─────────────────────────────────────────────────────────────────────────────
def make_figure(df, csv_path):
    label    = session_label(csv_path)
    t        = df['t'].to_numpy()
    pressing = (df.get('ur5_pressing', pd.Series(0, index=df.index)) == 1).to_numpy()

    cell_arr  = df[CELL_COLS].to_numpy()
    cell_mean = cell_arr.mean(axis=0)
    cell_peak = cell_arr.max(axis=0)

    # ── TCP relative position (centred on sensor) ─────────────────────────────
    has_tcp = ('tcp_x' in df.columns and df['tcp_x'].abs().max() > 1e-6)
    if has_tcp:
        ref = pressing if pressing.any() else np.ones(len(df), bool)
        cx  = df['tcp_x'].to_numpy()[ref].mean()
        cy  = df['tcp_y'].to_numpy()[ref].mean()
        tx  = (df['tcp_x'].to_numpy() - cx) * 1000   # m → mm, sensor-centred
        ty  = (df['tcp_y'].to_numpy() - cy) * 1000

    has_fz = 'fz_c' in df.columns and df['fz_c'].abs().max() > 1e-6
    has_lc = 'lc_N' in df.columns and df['lc_N'].abs().max() > 1e-6

    # ── Layout ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f'Trajectory vs Sensor Cells — {label}',
                 fontsize=13, fontweight='bold', y=0.99)

    gs_outer = gridspec.GridSpec(2, 1, figure=fig,
                                 height_ratios=[3, 1], hspace=0.40)
    gs_top   = gs_outer[0].subgridspec(1, 2, wspace=0.28, width_ratios=[1.5, 1])

    ax_traj = fig.add_subplot(gs_top[0])
    ax_hex  = fig.add_subplot(gs_top[1])
    ax_fz   = fig.add_subplot(gs_outer[1])

    # ── Panel 1 : TCP path overlaid on hex cells ──────────────────────────────
    sm_mean = _draw_hex_cells(ax_traj, cell_mean,
                              vmax=max(float(cell_mean.max()), 1e-6))
    cb_mean = plt.colorbar(sm_mean, ax=ax_traj,
                           fraction=0.030, pad=0.02, label='Mean activation')
    cb_mean.ax.tick_params(labelsize=7)

    if has_tcp:
        pts  = np.array([tx, ty]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc_path = LineCollection(segs, cmap='cool',
                                 norm=Normalize(t[0], t[-1]),
                                 linewidth=2.2, alpha=0.92, zorder=4)
        lc_path.set_array(t[:-1])
        ax_traj.add_collection(lc_path)
        cb_time = plt.colorbar(lc_path, ax=ax_traj,
                               fraction=0.030, pad=0.10, label='Time (s)')
        cb_time.ax.tick_params(labelsize=7)

        ax_traj.scatter(tx[0],  ty[0],  s=100, color='lime', zorder=6,
                        label='Start', edgecolors='black', linewidth=0.8)
        ax_traj.scatter(tx[-1], ty[-1], s=100, color='red',  zorder=6,
                        label='End',   edgecolors='black', linewidth=0.8)

        # direction arrow at mid-point
        mid = len(tx) // 2
        i0, i1 = max(mid - 3, 0), min(mid + 3, len(tx) - 1)
        if i1 > i0:
            ax_traj.annotate(
                '', xy=(tx[i1], ty[i1]), xytext=(tx[i0], ty[i0]),
                arrowprops=dict(arrowstyle='->', color='cyan', lw=2.0),
                zorder=7)

        ax_traj.legend(fontsize=8, loc='lower right')

    ax_traj.set_xlabel('X (mm, sensor frame)', fontsize=9)
    ax_traj.set_ylabel('Y (mm, sensor frame)', fontsize=9)
    ax_traj.set_title(
        'TCP trajectory vs hex cells\n'
        'cells: mean activation   |   bold border: activated   |   path: time',
        fontsize=8.5, fontweight='bold')
    ax_traj.grid(alpha=0.18, zorder=1)
    ax_traj.set_xlim(-24, 24)
    ax_traj.set_ylim(-22, 22)
    ax_traj.set_aspect('equal')

    # ── Panel 2 : Peak activation hex map ─────────────────────────────────────
    sm_peak = _draw_hex_cells(ax_hex, cell_peak,
                              vmax=max(float(cell_peak.max()), 1e-6))
    cb_peak = plt.colorbar(sm_peak, ax=ax_hex,
                           fraction=0.040, pad=0.02, label='Peak activation')
    cb_peak.ax.tick_params(labelsize=7)
    ax_hex.set_xlabel('X (mm)', fontsize=9)
    ax_hex.set_ylabel('Y (mm)', fontsize=9)
    ax_hex.set_title('Peak cell activation\n(full session)',
                     fontsize=8.5, fontweight='bold')
    ax_hex.grid(alpha=0.18)

    # ── Panel 3 : Z force strip — cropped to contact window only ─────────────
    # Find the time window where the tip was in contact with the surface
    if pressing.any():
        contact_idx = np.where(pressing)[0]
        i0, i1 = contact_idx[0], contact_idx[-1]
    else:
        i0, i1 = 0, len(t) - 1

    t_crop       = t[i0:i1 + 1]
    press_crop   = pressing[i0:i1 + 1]

    all_force_vals = []
    if has_fz:
        fz_vals = df['fz_c'].to_numpy()[i0:i1 + 1]
        ax_fz.plot(t_crop, fz_vals, linewidth=1.1, color='#dc0000',
                   alpha=0.90, label='UR5 Fz (N, zeroed)', zorder=3)
        all_force_vals.append(fz_vals)
    if has_lc:
        lc_vals = df['lc_N'].to_numpy()[i0:i1 + 1]
        ax_fz.plot(t_crop, lc_vals, linewidth=1.1, color='#9b59b6',
                   alpha=0.85, label='Load cell (N, zeroed)', zorder=3)
        all_force_vals.append(lc_vals)

    if all_force_vals:
        combined = np.concatenate(all_force_vals)
        fmin, fmax = combined.min(), combined.max()
        margin = max((fmax - fmin) * 0.07, 0.1)
        ax_fz.set_ylim(fmin - margin, fmax + margin)
        if press_crop.any():
            ax_fz.fill_between(t_crop, fmin - margin, fmax + margin,
                               where=press_crop, alpha=0.10,
                               color='steelblue', label='Contact', zorder=1)

    ax_fz.axhline(0, color='black', linewidth=0.6, linestyle=':', alpha=0.5)
    ax_fz.set_xlabel('Time (s)', fontsize=9)
    ax_fz.set_ylabel('Force (N)', fontsize=9)
    ax_fz.set_title('Z-force during contact: UR5 Fz vs FUTEK load cell',
                    fontsize=9, fontweight='bold')
    ax_fz.legend(fontsize=8, loc='upper right', ncol=3)
    ax_fz.grid(alpha=0.25)
    ax_fz.set_xlim(t_crop[0], t_crop[-1])

    return fig


# ─────────────────────────────────────────────────────────────────────────────
def process_session(csv_path):
    label    = session_label(csv_path)
    out_path = os.path.join(OUT_DIR, f'{label}.png')
    print(f'[traj] {os.path.basename(csv_path)}', end='  ...  ', flush=True)
    try:
        df  = load_session(csv_path)
        fig = make_figure(df, csv_path)
        fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close(fig)
        print(f'saved → {os.path.relpath(out_path, FRICTION_DIR)}')
    except Exception as exc:
        print(f'ERROR: {exc}')
        import traceback
        traceback.print_exc()


def main():
    p = argparse.ArgumentParser(
        description='TCP trajectory vs capacitive sensor comparison',
        epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('filter', nargs='?', default=None,
                   help='Substring filter on CSV filename (default: all)')
    args = p.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)

    csvs = sorted(glob.glob(os.path.join(LOGS_DIR, '*.csv')))
    if args.filter:
        csvs = [f for f in csvs if args.filter in os.path.basename(f)]
    if not csvs:
        sys.exit(f'[traj] No CSV files found in {LOGS_DIR}')

    print(f'[traj] {len(csvs)} session(s) found')
    print(f'[traj] Output → {OUT_DIR}\n')

    for csv_path in csvs:
        process_session(csv_path)

    print(f'\n[traj] Done — {len(csvs)} figure(s) saved.')


if __name__ == '__main__':
    main()
