"""
analyze_interdome.py — Interdome_touch dataset analysis & visualization
=========================================================================
Reads a CSV produced by main.py (and its companion *_meta.json, if present)
and reconstructs:

  1. The UR5 TCP trajectory — XY path over the sensor grid, and Z (depth)
     vs time so the press/hold/retract waveform is visible.
  2. The pressing plan actually executed — depth x iteration x point x phase.
  3. The 19-cell hexagonal schematic (same geometry as
     Integration_2/plot_rigid.py, view-rotated upright about the anchor
     point), colored by:
       - capacitive sensor intensity: mean of each point's OWN expected
         cell (via raw_sensor_cell -> cell_N) during the 'hold' phase
       - FUTEK force: mean of futek_force_N during the 'hold' phase
     one hexagon grid per depth, plus an all-depths summary figure.

Usage
-----
  python3 analyze_interdome.py                          # latest logs/*.csv
  python3 analyze_interdome.py logs/interdome_<ts>.csv
  python3 analyze_interdome.py logs/interdome_<ts>.csv --save
  python3 analyze_interdome.py logs/interdome_<ts>.csv --no-show
"""

import os
import sys
import json
import glob
import math
import argparse

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')   # headless-safe; all figures are saved to plots/, not shown
import matplotlib.pyplot as plt
from matplotlib.patches import RegularPolygon
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
LOG_DIR      = os.path.join(_HERE, 'logs')
PLOTS_DIR    = os.path.join(_HERE, 'plots')

DEFAULT_POINTS_JSON = os.path.join(
    _INTEGRATION, 'calib_points_supposed_rigid_transformed.json')

HEX_RADIUS   = 8.0 / math.sqrt(3)   # ~4.6188 mm, matches plot_rigid.py
ANCHOR_POINT = 10

# Point -> raw sensor cell (must match main.py / Integration_2/ur5_control.py)
UR5_TO_SENSOR = {
    1: 24,  2: 12,  3: 0,
    4: 37,  5: 25,  6: 13,  7: 1,
    8: 50,  9: 38,  10: 26, 11: 14, 12: 2,
    13: 51, 14: 39, 15: 27, 16: 15,
    17: 52, 18: 40, 19: 28,
}
# Serial frame index (0..18) -> raw sensor cell, i.e. cell_{i+1} in the CSV
# corresponds to USED_CELLS[i] (must match Integration_2/sensor.py)
USED_CELLS = [
     2, 15, 28,
     1, 14, 27, 40,
     0, 13, 26, 39, 52,
    12, 25, 38, 51,
    24, 37, 50,
]

def own_cell_column(pt):
    """CSV column name (cell_1..cell_19) for point pt's own expected sensor cell."""
    raw = UR5_TO_SENSOR.get(pt)
    if raw is None or raw not in USED_CELLS:
        return None
    idx = USED_CELLS.index(raw)
    return f'cell_{idx + 1}'

# ── Loading ──────────────────────────────────────────────────────────────────
def find_latest_csv():
    files = sorted(glob.glob(os.path.join(LOG_DIR, 'interdome_*.csv')))
    files = [f for f in files if not f.endswith('_meta.json')]
    if not files:
        raise FileNotFoundError(f'No interdome_*.csv files found in {LOG_DIR}')
    return files[-1]

def load_meta(csv_path):
    meta_path = csv_path.replace('.csv', '_meta.json')
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            return json.load(f), meta_path
    print(f'[warn] No companion meta file at {meta_path} — using defaults')
    return {}, None

def load_points(meta):
    """{pt: (x_mm, y_mm)}, rotation_deg, anchor — from the meta's recorded
    points_source if available, else the default rigid-transformed json."""
    path = meta.get('points_source', DEFAULT_POINTS_JSON)
    if not os.path.exists(path):
        path = DEFAULT_POINTS_JSON
    with open(path) as f:
        d = json.load(f)
    points = {int(k): (v['x_mm'], v['y_mm']) for k, v in d['points'].items()}
    rotation_deg = d.get('rotation_deg', 0.0)
    anchor = d.get('anchor_point', ANCHOR_POINT)
    return points, rotation_deg, anchor

def load_dataset(csv_path):
    df = pd.read_csv(csv_path)
    return df

# ── Geometry helpers (mirrors Integration_2/plot_rigid.py) ────────────────────
def rotate_point(px, py, pivot_x, pivot_y, angle_deg):
    theta = math.radians(angle_deg)
    dx, dy = px - pivot_x, py - pivot_y
    c, s = math.cos(theta), math.sin(theta)
    return pivot_x + dx * c - dy * s, pivot_y + dx * s + dy * c

def rotate_points(points, pivot_x, pivot_y, angle_deg):
    return {k: rotate_point(x, y, pivot_x, pivot_y, angle_deg)
            for k, (x, y) in points.items()}

def view_upright(points, rotation_deg, anchor):
    """Rotate the whole point set by -rotation_deg around the anchor point so
    hexagons can be drawn axis-aligned (orientation=0) and still tile
    correctly — display-only, doesn't change any underlying measurement."""
    if not points or rotation_deg == 0.0:
        return points
    pivot_x, pivot_y = points[anchor]
    return rotate_points(points, pivot_x, pivot_y, -rotation_deg)

# ── Trajectory reconstruction ─────────────────────────────────────────────────
def plot_trajectory(df, points, out_path):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6.5))

    phase_colors = {
        'locate': '#1f77b4', 'press': '#d62728', 'hold': '#2ca02c',
        'retract': '#ff7f0e', 'post': '#9467bd',
    }

    # ── Left: XY path over the sensor grid ────────────────────────────────────
    for pid, (x, y) in points.items():
        hexagon = RegularPolygon((x, y), numVertices=6, radius=HEX_RADIUS,
                                  orientation=0, facecolor='none',
                                  edgecolor='#bbbbbb', linewidth=1.0, zorder=1)
        ax1.add_patch(hexagon)
        ax1.annotate(str(pid), (x, y), ha='center', va='center',
                     fontsize=7, color='#888888', zorder=2)

    tcp_x_mm = (df['tcp_x'] - df['tcp_x'].iloc[0]) * 1000.0 \
        if 'tcp_x' in df else None
    for phase, color in phase_colors.items():
        sub = df[df['phase'] == phase]
        if len(sub) == 0:
            continue
        ax1.scatter(sub['point_x_mm'], sub['point_y_mm'], s=8, alpha=0.35,
                    color=color, label=phase, zorder=3)

    ax1.set_title('Reconstructed trajectory — points visited (by phase)',
                   fontsize=11, fontweight='bold')
    ax1.set_xlabel('x (mm)')
    ax1.set_ylabel('y (mm)')
    ax1.set_aspect('equal')
    ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(alpha=0.2)

    # ── Right: Z (depth) vs time — press/hold/retract waveform ────────────────
    t = df['timestamp'] - df['timestamp'].iloc[0]
    for phase, color in phase_colors.items():
        sub = df[df['phase'] == phase]
        if len(sub) == 0:
            continue
        ax2.scatter(t.loc[sub.index], sub['tcp_z'] * 1000.0, s=4, alpha=0.5,
                    color=color, label=phase)
    ax2.set_title('TCP height vs time (mm)', fontsize=11, fontweight='bold')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('tcp_z (mm)')
    ax2.legend(fontsize=8, loc='upper right')
    ax2.grid(alpha=0.2)

    fig.suptitle('Interdome_touch — trajectory reconstruction',
                  fontsize=13, fontweight='bold')
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {out_path}')

# ── Per-point / per-depth aggregation ─────────────────────────────────────────
def point_stats(df, points, depths_mm, value_fn, phase='hold'):
    """{depth: {pt: aggregated_value}} using only rows where phase==phase."""
    hold = df[df['phase'] == phase]
    out = {}
    for depth in depths_mm:
        d_rows = hold[np.isclose(hold['depth_mm'], depth)]
        per_pt = {}
        for pt in points:
            pt_rows = d_rows[d_rows['point'] == pt]
            per_pt[pt] = value_fn(pt_rows, pt) if len(pt_rows) else float('nan')
        out[depth] = per_pt
    return out

def capacitive_value_fn(rows, pt):
    col = own_cell_column(pt)
    if col is None or col not in rows:
        return float('nan')
    return float(rows[col].mean())

def futek_value_fn(rows, pt):
    if 'futek_force_N' not in rows:
        return float('nan')
    return float(rows['futek_force_N'].mean())

# ── Hexagonal schematic plotting ──────────────────────────────────────────────
def plot_hex_grid(ax, points, values, cmap, norm, title):
    for pid, (x, y) in points.items():
        val = values.get(pid, float('nan'))
        color = cmap(norm(val)) if val == val else '#eeeeee'   # nan -> grey
        hexagon = RegularPolygon((x, y), numVertices=6, radius=HEX_RADIUS,
                                  orientation=0, facecolor=color,
                                  edgecolor='#333333', linewidth=1.0, zorder=2)
        ax.add_patch(hexagon)
        label = f'{val:.3f}' if val == val else 'n/a'
        ax.annotate(f'P{pid}\n{label}', (x, y), ha='center', va='center',
                    fontsize=6.5, fontweight='bold', zorder=3)
    xs = [x for x, y in points.values()]
    ys = [y for x, y in points.values()]
    pad = HEX_RADIUS * 2
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=9, fontweight='bold')
    ax.set_xticks([]); ax.set_yticks([])

def plot_hex_by_depth(stats_by_depth, points, out_path, suptitle, unit_label, cmap_name):
    depths = sorted(stats_by_depth.keys())
    ncols = min(3, len(depths) + 1)
    nrows = -(-(len(depths) + 1) // ncols)   # +1 for the all-depths summary panel

    all_vals = [v for d in stats_by_depth.values() for v in d.values() if v == v]
    if not all_vals:
        print(f'  [warn] no data for "{suptitle}" — skipping plot')
        return
    vmin, vmax = min(all_vals), max(all_vals)
    if vmin == vmax:
        vmax = vmin + 1e-6
    norm = Normalize(vmin=vmin, vmax=vmax)
    cmap = matplotlib.colormaps[cmap_name]

    fig, axes = plt.subplots(nrows, ncols, figsize=(ncols * 4.2, nrows * 4.2))
    axes_flat = np.atleast_1d(axes).flatten()

    for i, depth in enumerate(depths):
        plot_hex_grid(axes_flat[i], points, stats_by_depth[depth], cmap, norm,
                      f'{depth:.1f} mm')

    # ── all-depths summary (mean across depths per point) ─────────────────────
    summary = {}
    for pt in points:
        vals = [stats_by_depth[d][pt] for d in depths if stats_by_depth[d][pt] == stats_by_depth[d][pt]]
        summary[pt] = float(np.mean(vals)) if vals else float('nan')
    plot_hex_grid(axes_flat[len(depths)], points, summary, cmap, norm, 'All depths (mean)')

    for j in range(len(depths) + 1, len(axes_flat)):
        axes_flat[j].set_visible(False)

    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    fig.colorbar(sm, ax=axes_flat.tolist(), shrink=0.7, label=unit_label)

    fig.suptitle(suptitle, fontsize=13, fontweight='bold')
    fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {out_path}')

# ── Summary table ──────────────────────────────────────────────────────────────
def print_summary(df, cap_stats, futek_stats, depths_mm):
    print()
    print('=' * 70)
    print('  SUMMARY (hold-phase means)')
    print('=' * 70)
    print(f'  {"depth_mm":>9}  {"mean_cap":>9}  {"max_cap":>8}  '
          f'{"mean_futek_N":>13}  {"max_futek_N":>12}')
    for depth in depths_mm:
        cap_vals   = [v for v in cap_stats[depth].values() if v == v]
        futek_vals = [v for v in futek_stats[depth].values() if v == v]
        mc = np.mean(cap_vals) if cap_vals else float('nan')
        xc = np.max(cap_vals) if cap_vals else float('nan')
        mf = np.mean(futek_vals) if futek_vals else float('nan')
        xf = np.max(futek_vals) if futek_vals else float('nan')
        print(f'  {depth:>9.1f}  {mc:>9.3f}  {xc:>8.3f}  {mf:>13.3f}  {xf:>12.3f}')
    print('=' * 70)

# ── Main ───────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description='Analyze an Interdome_touch dataset: trajectory + hex schematic',
        epilog=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('csv', nargs='?', default=None, help='Path to interdome_*.csv')
    return p.parse_args()

def main():
    args = parse_args()

    csv_path = args.csv or find_latest_csv()
    if not os.path.exists(csv_path):
        print(f'[error] File not found: {csv_path}')
        sys.exit(1)

    print(f'[load] {csv_path}')
    meta, meta_path = load_meta(csv_path)
    points_raw, rotation_deg, anchor = load_points(meta)
    points = view_upright(points_raw, rotation_deg, anchor)
    df = load_dataset(csv_path)

    depths_mm = meta.get('depths_mm') or sorted(df['depth_mm'].unique().tolist())
    iterations = meta.get('iterations', df['iteration'].max() + 1 if 'iteration' in df else '?')

    print(f'  rows       : {len(df):,}')
    print(f'  depths     : {depths_mm} mm')
    print(f'  iterations : {iterations}')
    print(f'  points     : {sorted(points.keys())}')

    os.makedirs(PLOTS_DIR, exist_ok=True)
    base = os.path.splitext(os.path.basename(csv_path))[0]

    # 1) Trajectory
    plot_trajectory(df, points, os.path.join(PLOTS_DIR, f'{base}_trajectory.png'))

    # 2) Hex schematic — capacitive sensor
    cap_stats = point_stats(df, points, depths_mm, capacitive_value_fn, phase='hold')
    plot_hex_by_depth(
        cap_stats, points, os.path.join(PLOTS_DIR, f'{base}_hex_capacitive.png'),
        suptitle='Interdome_touch — capacitive sensor intensity (own cell, hold-phase mean)',
        unit_label='normalized capacitive value (0-1)', cmap_name='viridis')

    # 3) Hex schematic — FUTEK force
    futek_stats = point_stats(df, points, depths_mm, futek_value_fn, phase='hold')
    plot_hex_by_depth(
        futek_stats, points, os.path.join(PLOTS_DIR, f'{base}_hex_futek.png'),
        suptitle='Interdome_touch — FUTEK load cell force (hold-phase mean)',
        unit_label='force (N)', cmap_name='plasma')

    print_summary(df, cap_stats, futek_stats, depths_mm)
    print(f'\nFigures saved in: {PLOTS_DIR}/')

if __name__ == '__main__':
    main()
