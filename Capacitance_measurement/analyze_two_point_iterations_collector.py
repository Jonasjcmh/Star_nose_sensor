"""
analyze_two_point_iterations_collector.py — Star-Nose Sensor | Multi-Point
Depth-Sweep Iterations Analysis
============================================================================
Reads a CSV produced by capacitance_two_point_iterations_collector.py (N
user-chosen points × depths × iterations, phases locate → press → hold →
retract → post, logged at ~100 Hz) and produces a full analysis, mirroring
analyze_ramp_collector.py but adapted to the fact that here the swept
variable is DEPTH (per point) rather than point (at one fixed depth).

Two families of figures
------------------------
  A. Per-point detail grid  — ONE FIGURE PER POINT: a grid of panels
     (rows = iterations, cols = depths) each showing ONE indentation's Cp
     (left axis) + force (right axis) vs time, background shaded by phase.
     Lets you inspect every single indentation individually.

  B. Dataset-level summary (all iterations together):
       1. Overlay grid          — ONE FIGURE: rows = points, cols = depths,
                                   each panel overlays Cp + force vs time for
                                   all iterations at that (point, depth).
       2. Depth-response curve  — mean ± std settled ΔCp vs depth, one line
                                   per point (the calibration curve).
       3. Force-vs-depth curve  — mean ± std settled hold force vs depth,
                                   one line per point.
       4. Sensor-map heatmap    — 19-point layout coloured by mean ΔCp at a
                                   reference depth (default: deepest tested);
                                   untested points shown greyed out.
       5. Force–ΔCp scatter     — settled ΔCp vs force, one point per indent,
                                   coloured by pad, marker by depth.
       6. Stats table (printed) — per (point, depth) n / mean / std / CV of ΔCp.

  (No box plots in this variant.)

Why ΔCp (baseline-corrected)?
-----------------------------
The LCR probes are re-wired to each pad between points, so the *absolute* Cp
baseline differs from point to point. The meaningful, comparable response of
an indentation is the settled change:
        ΔCp = mean(Cp over hold tail) − mean(Cp over locate baseline)
computed from the last SETTLE_WINDOW_S of each phase (≈ √N less noise than a
single sample), exactly as the collector reports it live.

Usage
-----
  python analyze_two_point_iterations_collector.py                # newest CSV
  python analyze_two_point_iterations_collector.py logs/two_point_iterations_P05_P12_20260720_101500.csv
  python analyze_two_point_iterations_collector.py --point 5       # only P05's detail grid
  python analyze_two_point_iterations_collector.py --map-depth 4   # sensor map at 4mm
  python analyze_two_point_iterations_collector.py --no-grids      # summary figures only
  python analyze_two_point_iterations_collector.py FILE.csv --no-show
"""

import os
import csv
import sys
import glob
import math
import argparse
from collections import defaultdict

import numpy as np

_HERE    = os.path.dirname(os.path.abspath(__file__))
LOG_DIR  = os.path.join(_HERE, 'logs')
PLOT_DIR = os.path.join(_HERE, 'plots')

# Must match capacitance_two_point_iterations_collector.SETTLE_WINDOW_S — the
# tail window over which each phase's settled reading is averaged.
SETTLE_WINDOW_S = 1.0

PHASE_ORDER  = ['locate', 'press', 'hold', 'retract', 'post']
PHASE_COLORS = {
    'locate':  '#cfe8ff',
    'press':   '#ffe2c2',
    'hold':    '#c8f5c8',
    'retract': '#ffd6d6',
    'post':    '#e6e6e6',
}
DEPTH_MARKERS = ['o', 's', '^', 'D', 'v', 'P', 'X', '*']

# ── Sensor layout (mm, relative to reference pose) — all 19 pads, for the
# sensor-map figure (only the tested subset gets coloured in) ───────────────
POINTS_XY = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

# ── Data loading ────────────────────────────────────────────────────────────────

def newest_csv():
    files = sorted(glob.glob(os.path.join(LOG_DIR, 'two_point_iterations_*.csv')))
    if not files:
        raise SystemExit(f'[error] No two_point_iterations_*.csv found in {LOG_DIR}')
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
                'point_idx':   _i(r.get('point_idx', 0)),
                'point':       _i(r.get('point', 0)),
                'depth_mm':    _f(r.get('depth_mm')),
                'iter_idx':    _i(r.get('iter_idx', 0)),
                'phase':       r.get('phase', ''),
                'load_cell_N': _f(r.get('load_cell_N')),
                'Cp_pF':       _f(r.get('Cp_pF')),
                'Rp_Ohm':      _f(r.get('Rp_Ohm')),
                'lcr_ok':      _i(r.get('lcr_ok', 0)),
            })
    if not rows:
        raise SystemExit(f'[error] No data rows in {path}')
    return rows

def group_indentations(rows):
    """Group rows into indentations keyed by (point, depth_mm, iter_idx),
    each sorted by timestamp."""
    groups = defaultdict(list)
    for r in rows:
        groups[(r['point'], r['depth_mm'], r['iter_idx'])].append(r)
    for key in groups:
        groups[key].sort(key=lambda r: r['timestamp'])
    return groups

def point_list(rows):
    """Points in testing order (by point_idx), not numeric pad order."""
    first_idx = {}
    for r in rows:
        first_idx.setdefault(r['point'], r['point_idx'])
    return sorted(first_idx, key=lambda p: first_idx[p])

def depth_list(rows):
    return sorted({r['depth_mm'] for r in rows})

# ── Settled tail-averaging (matches the collector's live readings) ───────────────

def _phase_rows(rows_g, phase):
    return [r for r in rows_g if r['phase'] == phase]

def tail_mean(rows_g, phase, window_s=SETTLE_WINDOW_S):
    """Mean Cp_pF (non-NaN) and load_cell_N over the LAST `window_s` seconds
    of `phase` within one indentation. Returns (cp_mean, load_mean, n)."""
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
    baseline (locate), hold, ΔCp = hold − locate, hold force."""
    out = []
    for (pt, depth_mm, iter_idx), rows_g in groups.items():
        cp_base, _,      _ = tail_mean(rows_g, 'locate', window_s)
        cp_hold, f_hold, n = tail_mean(rows_g, 'hold',   window_s)
        if math.isnan(cp_hold):
            continue
        dcp = (cp_hold - cp_base) if not math.isnan(cp_base) else float('nan')
        out.append({
            'point': pt, 'depth_mm': depth_mm, 'iter': iter_idx,
            'cp_base': cp_base, 'cp_hold': cp_hold, 'dcp': dcp, 'force': f_hold,
        })
    return out

def dcp_by_point_depth(responses):
    """{(point, depth_mm): [ΔCp, ...]} across iterations (drops NaN)."""
    d = defaultdict(list)
    for r in responses:
        if not math.isnan(r['dcp']):
            d[(r['point'], r['depth_mm'])].append(r['dcp'])
    return d

def force_by_point_depth(responses):
    """{(point, depth_mm): [force, ...]} across iterations (drops NaN)."""
    d = defaultdict(list)
    for r in responses:
        if not math.isnan(r['force']):
            d[(r['point'], r['depth_mm'])].append(r['force'])
    return d

# ── Phase shading helper (per-indentation time-series) ───────────────────────────

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

# ── A. Per-point detail grid (rows = iterations, cols = depths) ──────────────────

def plot_point_detail_grid(rows, pt, out_dir, show=True):
    """One figure per point: grid of (iteration × depth) panels, each ONE
    indentation trace, phase-shaded. Lets you inspect every indentation."""
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    prows = [r for r in rows if r['point'] == pt]
    if not prows:
        print(f'[grid] No data for P{pt:02d}.')
        return None

    depths = sorted({r['depth_mm'] for r in prows})
    iters  = sorted({r['iter_idx'] for r in prows})
    ncols, nrows = len(depths), len(iters)

    by_di = defaultdict(list)
    for r in prows:
        by_di[(r['depth_mm'], r['iter_idx'])].append(r)

    C_CP, C_FORCE = '#1f77b4', '#d62728'
    fig, axs = plt.subplots(nrows, ncols, figsize=(3.6 * ncols, 2.6 * nrows), squeeze=False)
    fig.suptitle(f'P{pt:02d} — Cp (blue) + force (red) vs time, per depth × iteration',
                 fontsize=14, fontweight='bold')

    for ri, iter_idx in enumerate(iters):
        for ci, depth_mm in enumerate(depths):
            ax = axs[ri][ci]
            drows = sorted(by_di.get((depth_mm, iter_idx), []), key=lambda r: r['timestamp'])
            if not drows:
                ax.set_xticks([]); ax.set_yticks([])
                continue
            t0 = drows[0]['timestamp']
            t  = [r['timestamp'] - t0 for r in drows]
            cp = [r['Cp_pF'] for r in drows]
            ld = [r['load_cell_N'] for r in drows]
            phases = [r['phase'] for r in drows]
            for phase, ta, tb in phase_spans(t, phases):
                ax.axvspan(ta, tb, color=PHASE_COLORS.get(phase, '#ffffff'), alpha=0.5, lw=0)
            ax.plot(t, cp, color=C_CP, lw=1.0)
            ax.tick_params(axis='y', labelcolor=C_CP, labelsize=6.5)
            ax.tick_params(axis='x', labelsize=6.5)
            ax.grid(alpha=0.25)
            if ri == 0:
                ax.set_title(f'{depth_mm:.1f} mm', fontsize=9, fontweight='bold')
            if ci == 0:
                ax.set_ylabel(f'iter {iter_idx + 1}', fontsize=8)
            axf = ax.twinx()
            axf.plot(t, ld, color=C_FORCE, lw=0.9, alpha=0.85)
            axf.tick_params(axis='y', labelcolor=C_FORCE, labelsize=6.5)

    sig = [Line2D([0], [0], color=C_CP,    lw=1.4, label='Cp (pF, left)'),
           Line2D([0], [0], color=C_FORCE, lw=1.4, label='force (N, right)')]
    ph  = [Patch(color=c, alpha=0.5, label=p) for p, c in PHASE_COLORS.items()]
    fig.legend(handles=sig + ph, loc='lower center', ncol=7, fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, 0.005))
    fig.tight_layout(rect=[0, 0.04, 1, 0.95])

    path = os.path.join(out_dir, f'point{pt:02d}_detail_grid.png')
    fig.savefig(path, dpi=120, bbox_inches='tight')
    print(f'  [grid] P{pt:02d} → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B1. Overlay grid (rows = points, cols = depths, iterations overlaid) ────────

def plot_overlay_grid(groups, points, depths, out_dir, show=True):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    ncols, nrows = len(depths), len(points)
    C_CP, C_FORCE = 'steelblue', 'tomato'
    fig, axes = plt.subplots(nrows, ncols, figsize=(3.6 * ncols, 2.6 * nrows), squeeze=False)
    fig.suptitle('Cp (blue) + force (red) vs time — per point × depth, all iterations overlaid',
                 fontsize=14, fontweight='bold')

    by_pd = defaultdict(list)
    for (pt, depth_mm, iter_idx), rows_g in groups.items():
        by_pd[(pt, depth_mm)].append(rows_g)

    for ri, pt in enumerate(points):
        for ci, depth_mm in enumerate(depths):
            ax  = axes[ri][ci]
            ax2 = ax.twinx()
            any_data = False
            for rows_g in by_pd.get((pt, depth_mm), []):
                if not rows_g:
                    continue
                t0 = rows_g[0]['timestamp']
                ts = np.array([r['timestamp'] - t0 for r in rows_g])
                cp = np.array([r['Cp_pF'] for r in rows_g])
                ld = np.array([r['load_cell_N'] for r in rows_g])
                ax.plot(ts, cp, color=C_CP, alpha=0.55, lw=0.8)
                ax2.plot(ts, ld, color=C_FORCE, alpha=0.35, lw=0.8, ls='--')
                any_data = True
            if ri == 0:
                ax.set_title(f'{depth_mm:.1f} mm', fontsize=9, fontweight='bold')
            if ci == 0:
                ax.set_ylabel(f'P{pt:02d}', fontsize=9, fontweight='bold')
            ax.tick_params(axis='y', labelcolor=C_CP, labelsize=6.5)
            ax2.tick_params(axis='y', labelcolor=C_FORCE, labelsize=6.5)
            ax.tick_params(axis='x', labelsize=6.5)
            ax.grid(alpha=0.25)
            if not any_data:
                ax.set_xticks([]); ax.set_yticks([])

    handles = [Line2D([0], [0], color=C_CP, lw=1.5, label='Cp (pF, left)'),
               Line2D([0], [0], color=C_FORCE, lw=1.5, ls='--', label='force (N, right)')]
    fig.legend(handles=handles, loc='lower center', ncol=2, fontsize=10,
               frameon=False, bbox_to_anchor=(0.5, 0.005))
    fig.tight_layout(rect=[0, 0.04, 1, 0.95])
    path = os.path.join(out_dir, 'overlay_grid.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    print(f'  [fig1] Overlay grid → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B2. Depth-response curve — ΔCp vs depth, one line per point ─────────────────

def plot_depth_response(dcp_by_pd, points, depths, out_dir, show=True):
    import matplotlib.pyplot as plt
    cmap  = plt.cm.tab10 if len(points) <= 10 else plt.cm.tab20
    color = {p: cmap(i / max(len(points) - 1, 1)) for i, p in enumerate(points)}

    fig, ax = plt.subplots(figsize=(8, 5.5))
    for pt in points:
        means = [np.mean(dcp_by_pd[(pt, d)]) if dcp_by_pd.get((pt, d)) else float('nan')
                 for d in depths]
        stds  = [np.std(dcp_by_pd[(pt, d)]) if dcp_by_pd.get((pt, d)) else 0.0
                 for d in depths]
        ax.errorbar(depths, means, yerr=stds, marker='o', capsize=4,
                    color=color[pt], label=f'P{pt:02d}', lw=1.6)
    ax.set_xlabel('Indentation depth (mm)')
    ax.set_ylabel('Settled ΔCp = hold − locate  (pF)')
    ax.set_title('Depth-Response Curve — Mean ± Std settled ΔCp per Point')
    ax.axhline(0, color='k', lw=0.6)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=min(len(points), 6))
    fig.tight_layout()
    path = os.path.join(out_dir, 'depth_response_dcp.png')
    fig.savefig(path, dpi=150)
    print(f'  [fig2] Depth-response ΔCp → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B3. Force-vs-depth curve — settled hold force vs depth, one line per point ──

def plot_force_depth(force_by_pd, points, depths, out_dir, show=True):
    import matplotlib.pyplot as plt
    cmap  = plt.cm.tab10 if len(points) <= 10 else plt.cm.tab20
    color = {p: cmap(i / max(len(points) - 1, 1)) for i, p in enumerate(points)}

    fig, ax = plt.subplots(figsize=(8, 5.5))
    for pt in points:
        means = [np.mean(force_by_pd[(pt, d)]) if force_by_pd.get((pt, d)) else float('nan')
                 for d in depths]
        stds  = [np.std(force_by_pd[(pt, d)]) if force_by_pd.get((pt, d)) else 0.0
                 for d in depths]
        ax.errorbar(depths, means, yerr=stds, marker='o', capsize=4,
                    color=color[pt], label=f'P{pt:02d}', lw=1.6)
    ax.set_xlabel('Indentation depth (mm)')
    ax.set_ylabel('Settled hold force (N)')
    ax.set_title('Depth-Response Curve — Mean ± Std settled force per Point')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=min(len(points), 6))
    fig.tight_layout()
    path = os.path.join(out_dir, 'depth_response_force.png')
    fig.savefig(path, dpi=150)
    print(f'  [fig3] Depth-response force → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B4. Sensor-map heatmap at a reference depth ─────────────────────────────────

def plot_sensor_map(dcp_by_pd, points, map_depth, out_dir, show=True):
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    from matplotlib.cm import ScalarMappable

    means = {p: (float(np.mean(dcp_by_pd[(p, map_depth)]))
                 if dcp_by_pd.get((p, map_depth)) else float('nan'))
             for p in points}
    valid = [v for v in means.values() if not math.isnan(v)]
    vmin, vmax = (min(valid), max(valid)) if valid else (0.0, 1.0)
    if vmin == vmax:
        vmax = vmin + 1.0
    norm = Normalize(vmin=vmin, vmax=vmax)
    cmap = plt.cm.plasma

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_aspect('equal')
    r_circle = 3.5
    for pt in range(1, 20):
        x, y = POINTS_XY[pt]
        if pt in means and not math.isnan(means[pt]):
            val   = means[pt]
            color = cmap(norm(val))
            label = f'P{pt}\n{val:.1f}'
        else:
            color = (0.85, 0.85, 0.85, 1.0)
            label = f'P{pt}\n—' if pt not in points else f'P{pt}\nn/a'
        ax.add_patch(plt.Circle((x, y), r_circle, color=color, ec='white', lw=0.8))
        ax.text(x, y, label, ha='center', va='center',
                fontsize=6.5, color='white' if pt in means else '#555555', fontweight='bold')
    sm = ScalarMappable(cmap=cmap, norm=norm); sm.set_array([])
    fig.colorbar(sm, ax=ax, shrink=0.7).set_label('Mean settled ΔCp (pF)')
    ax.set_xlim(-22, 22); ax.set_ylim(-20, 20)
    ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')
    ax.set_title(f'Sensor Layout — Mean settled ΔCp @ {map_depth:.1f} mm (tested points only)')
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    path = os.path.join(out_dir, 'sensor_map_dcp.png')
    fig.savefig(path, dpi=150)
    print(f'  [fig4] Sensor map @ {map_depth:.1f} mm → {path}')
    if not show:
        plt.close(fig)
    return path

# ── B5. Force–ΔCp scatter (coloured by point, marker by depth) ─────────────────

def plot_force_cp_scatter(responses, points, depths, out_dir, show=True):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    pts = [r for r in responses
           if not math.isnan(r['dcp']) and not math.isnan(r['force'])]
    if not pts:
        print('  [fig5] No settled response data to plot.')
        return None

    cmap  = plt.cm.tab10 if len(points) <= 10 else plt.cm.tab20
    color = {p: cmap(i / max(len(points) - 1, 1)) for i, p in enumerate(points)}
    marker = {d: DEPTH_MARKERS[i % len(DEPTH_MARKERS)] for i, d in enumerate(depths)}

    fig, ax = plt.subplots(figsize=(9, 6.5))
    for pt in points:
        for depth_mm in depths:
            sel = [r for r in pts if r['point'] == pt and r['depth_mm'] == depth_mm]
            if not sel:
                continue
            ax.scatter([r['dcp'] for r in sel], [r['force'] for r in sel],
                       color=color[pt], marker=marker[depth_mm], s=40, alpha=0.75,
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

    pt_handles = [Line2D([0], [0], marker='o', color='none', markerfacecolor=color[p],
                         markersize=7, label=f'P{p:02d}') for p in points]
    d_handles  = [Line2D([0], [0], marker=marker[d], color='none', markerfacecolor='grey',
                         markersize=7, label=f'{d:.1f} mm') for d in depths]

    ax.set_xlabel('Settled ΔCp (pF)  — hold − locate')
    ax.set_ylabel('Settled force (N)  — hold tail')
    ax.set_title('Force vs settled ΔCp per Indentation  (colour = point, marker = depth)')
    leg1 = ax.legend(handles=pt_handles, fontsize=7, loc='upper left', ncol=min(len(points), 4),
                     title='Point', frameon=False)
    ax.add_artist(leg1)
    ax.legend(handles=d_handles + [Line2D([0], [0], color='k', ls='--', lw=1.5,
              label='fit')], fontsize=7, loc='lower right', title='Depth', frameon=False)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'force_dcp_scatter.png')
    fig.savefig(path, dpi=150)
    print(f'  [fig5] Force–ΔCp scatter → {path}')
    if not show:
        plt.close(fig)
    return path

# ── Stats table ──────────────────────────────────────────────────────────────────

def print_stats_table(dcp_by_pd, points, depths):
    print()
    print('  Point  |  Depth (mm)  |  n  |  Mean ΔCp (pF)  |   Std (pF)   |  CV (%)')
    print('  ' + '-' * 72)
    for pt in points:
        for depth_mm in depths:
            vals = dcp_by_pd.get((pt, depth_mm), [])
            if not vals:
                print(f'  P{pt:02d}    | {depth_mm:10.2f}   |  0  |       —         |      —       |    —')
                continue
            mu, sd = float(np.mean(vals)), float(np.std(vals))
            cv = (sd / mu * 100) if mu != 0 else float('nan')
            print(f'  P{pt:02d}    | {depth_mm:10.2f}   | {len(vals):3d} | '
                  f'{mu:13.3f}   | {sd:10.3f}   | {cv:6.2f}')
        print()

# ── Main ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Analyse a two_point_iterations CSV (N points × depths × iterations)',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('csv', nargs='?', default=None,
                   help='CSV path (default: newest two_point_iterations_*.csv in logs/)')
    p.add_argument('--point', type=int, default=None,
                   help='Only draw this pad number\'s detail grid (default: all tested points)')
    p.add_argument('--map-depth', type=float, default=None,
                   help='Depth (mm) to use for the sensor-map figure (default: deepest tested)')
    p.add_argument('--no-grids', action='store_true',
                   help='Skip the per-point detail grids (summary figures only)')
    p.add_argument('--no-summary', action='store_true',
                   help='Skip the dataset-level summary figures (grids only)')
    p.add_argument('--out', default=None, help='Output directory (default: plots/twopt_<name>)')
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
        PLOT_DIR, 'twopt_' + os.path.splitext(os.path.basename(path))[0])
    os.makedirs(out_dir, exist_ok=True)

    points = point_list(rows)
    depths = depth_list(rows)

    groups     = group_indentations(rows)
    responses  = indentation_responses(groups)
    dcp_by_pd  = dcp_by_point_depth(responses)
    force_by_pd = force_by_point_depth(responses)

    phases  = {r['phase'] for r in rows}
    lcr_pct = 100 * sum(1 for r in rows if r['lcr_ok']) / max(len(rows), 1)

    print(f'[info] {len(rows)} rows  |  points: {[f"P{p:02d}" for p in points]}  '
          f'|  depths: {depths} mm  |  indentations: {len(groups)}')
    print(f'[info] phases: {sorted(phases)}  |  settle window: {SETTLE_WINDOW_S:.1f}s')
    if lcr_pct < 1.0:
        print('[note] lcr_ok flag is ~0% for this collector (known unreliable) — '
              'Cp values are valid, so analysis filters on NaN only, not lcr_ok.')
    print_stats_table(dcp_by_pd, points, depths)
    print(f'[out]  Saving figures to: {out_dir}')

    show = not args.no_show
    map_depth = args.map_depth if args.map_depth is not None else max(depths)

    # ── B. Dataset-level summary ────────────────────────────────────────────────
    if not args.no_summary:
        plot_overlay_grid(groups, points, depths, out_dir, show=show)
        plot_depth_response(dcp_by_pd, points, depths, out_dir, show=show)
        plot_force_depth(force_by_pd, points, depths, out_dir, show=show)
        plot_sensor_map(dcp_by_pd, points, map_depth, out_dir, show=show)
        plot_force_cp_scatter(responses, points, depths, out_dir, show=show)

    # ── A. Per-point detail grids ────────────────────────────────────────────────
    if not args.no_grids:
        grid_points = points
        if args.point is not None:
            if args.point not in points:
                raise SystemExit(f'[error] Point P{args.point:02d} not in data (have {points})')
            grid_points = [args.point]
        for pt in grid_points:
            plot_point_detail_grid(rows, pt, out_dir, show=show)

    if show:
        import matplotlib.pyplot as plt
        print('\n[show] Displaying figures — close windows to exit.')
        plt.show()
    print('[done]')


if __name__ == '__main__':
    main()
