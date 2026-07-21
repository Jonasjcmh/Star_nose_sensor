"""
analyze_two_point_p08_p19.py — Star-Nose Sensor | P08 & P19 Two-Point-Iteration
Solid / Hollow / Flat Comparison
================================================================================
Same three figure types as data_analysis_ws_*/frames_v2.py, frames2.py and
frames3.py, adapted to the schema produced by
capacitance_two_point_iterations_collector.py (an `iter_idx` column instead of
`round_idx`, and every depth already living in ONE csv per surface instead of
one file per depth) — restricted to P08 and P19, across the three surfaces
collected on 2026-07-20 (solid dome, hollow dome, flat).

Three figures per (point, surface):
  A. Depth grids        — one combined ΔC (top row) / ΔForce (bottom row)
                           figure per depth, columns = iterations. Mirrors
                           frames_v2.TouchSensorAnalyzer.plot_combined_grid.
  B. All-depths overlay — ΔC + ΔForce, all depths overlaid per iteration
                           column. Mirrors frames2.MultiDepthComparison.
  C. Iterations overlaid — ΔC, ΔForce(fz), ΔForce(load_cell) per depth
                           column, iterations overlaid. Mirrors
                           frames3.DepthColumnComparison.

Surfaces / source files
------------------------
  solid  : logs/two_point_iterations_P08_P19_20260720_133940_solid.csv
  hollow : logs/two_point_iterations_P08_P19_20260720_122906_hollow.csv
  flat   : logs/two_point_iterations_P08_20260720_141402_flat.csv
           + logs/two_point_iterations_P19_20260720_140639_flat.csv
           (flat was captured as two single-point runs; concatenated here
           since they cover disjoint points)

Usage
-----
  python analyze_two_point_p08_p19.py                    # everything
  python analyze_two_point_p08_p19.py --surface solid     # one surface only
  python analyze_two_point_p08_p19.py --point 8           # P08 only
  python analyze_two_point_p08_p19.py --no-depth-grids    # skip type A (fastest)
"""

import os
import math
import argparse

import pandas as pd
import matplotlib
matplotlib.use('Agg')  # batch-save many figures without needing a display
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib.lines import Line2D
from scipy.signal import savgol_filter

_HERE    = os.path.dirname(os.path.abspath(__file__))
LOG_DIR  = os.path.join(_HERE, 'logs')
PLOT_DIR = os.path.join(_HERE, 'plots')

POINTS = [8, 19]

SURFACE_FILES = {
    'solid':  [os.path.join(LOG_DIR, 'two_point_iterations_P08_P19_20260720_133940_solid.csv')],
    'hollow': [os.path.join(LOG_DIR, 'two_point_iterations_P08_P19_20260720_122906_hollow.csv')],
    'flat':   [os.path.join(LOG_DIR, 'two_point_iterations_P08_20260720_141402_flat.csv'),
               os.path.join(LOG_DIR, 'two_point_iterations_P19_20260720_140639_flat.csv')],
}


def load_surface_df(surface):
    paths = SURFACE_FILES[surface]
    for p in paths:
        if not os.path.exists(p):
            raise SystemExit(f'[error] Missing source file for surface={surface}: {p}')
    dfs = [pd.read_csv(p) for p in paths]
    return pd.concat(dfs, ignore_index=True) if len(dfs) > 1 else dfs[0]


# ── A. Per-(point, depth) combined grid — mirrors frames_v2.TouchSensorAnalyzer ──

class TouchSensorAnalyzer:
    """One combined figure per (point, surface, depth): 2 rows x N iteration
    columns. Row 0: ΔC (baseline-subtracted Cp, smoothed per phase).
    Row 1: ΔForce, load_cell_N (solid) vs fz (dashed)."""

    KEEP_PHASES = ['locate', 'hold', 'post']
    PHASE_COLORS = {'locate': 'tab:gray', 'hold': 'tab:green', 'post': 'tab:gray'}
    CAP_YLIM = (-0.09, 0.01)
    SAVGOL_POLYORDER = 3
    SAVGOL_WINDOW_BY_PHASE = {'locate': 21, 'hold': 101, 'post': 21}

    def __init__(self, df, point, depth_mm, surface=''):
        self.point = point
        self.depth_mm = depth_mm
        self.surface = surface

        mask_full = (df['point'] == point)
        if depth_mm is not None:
            mask_full &= (df['depth_mm'] == depth_mm)
        self.data_full = df[mask_full].copy()

        mask_c = mask_full & (df['phase'].isin(self.KEEP_PHASES))
        self.data_c = df[mask_c].copy()

        self.rounds = sorted(self.data_full['iter_idx'].unique())
        if not self.rounds:
            available_points = sorted(df['point'].unique().tolist())
            msg = f"No rows matched point={point}"
            if depth_mm is not None:
                available_depths = sorted(df['depth_mm'].unique().tolist())
                msg += f", depth_mm={depth_mm}. File has depth_mm values={available_depths}"
            msg += f". File has points={available_points}."
            raise ValueError(msg)

        self.baselines_c = {}
        self.baselines_f = {}
        self.corrected_c = {}
        self.corrected_f = {}

    def _smooth_by_phase(self, sub, value_col, out_col):
        sub[out_col] = sub[value_col].copy()
        phase_change = (sub['phase'] != sub['phase'].shift()).cumsum()
        for _, block_idx in sub.groupby(phase_change).groups.items():
            block = sub.loc[block_idx]
            phase = block['phase'].iloc[0]
            window = self.SAVGOL_WINDOW_BY_PHASE.get(phase, 21)
            window = min(window, len(block) - (1 - len(block) % 2))
            if window % 2 == 0:
                window -= 1
            if window >= self.SAVGOL_POLYORDER + 2:
                sub.loc[block_idx, out_col] = savgol_filter(block[value_col], window, self.SAVGOL_POLYORDER)
        return sub

    def compute_baseline_subtraction(self):
        for r in self.rounds:
            sub_c = self.data_c[self.data_c['iter_idx'] == r].sort_values('timestamp').copy()
            c_locate = sub_c.loc[sub_c['phase'] == 'locate', 'Cp_pF'].mean()
            c_post = sub_c.loc[sub_c['phase'] == 'post', 'Cp_pF'].mean()
            baseline_c = (c_locate + c_post) / 2

            sub_c['Cp_corrected'] = sub_c['Cp_pF'] - baseline_c
            sub_c['t0'] = sub_c['timestamp'] - sub_c['timestamp'].iloc[0]
            sub_c = self._smooth_by_phase(sub_c, 'Cp_corrected', 'Cp_smoothed')
            self.baselines_c[r] = baseline_c
            self.corrected_c[r] = sub_c

            sub_f = self.data_full[self.data_full['iter_idx'] == r].sort_values('timestamp').copy()
            f_locate = sub_f.loc[sub_f['phase'] == 'locate', 'load_cell_N'].mean()
            f_post = sub_f.loc[sub_f['phase'] == 'post', 'load_cell_N'].mean()
            baseline_f = (f_locate + f_post) / 2

            fz_locate = sub_f.loc[sub_f['phase'] == 'locate', 'fz'].mean()
            fz_post = sub_f.loc[sub_f['phase'] == 'post', 'fz'].mean()
            baseline_fz = (fz_locate + fz_post) / 2

            sub_f['Force_corrected'] = sub_f['load_cell_N'] - baseline_f
            sub_f['fz_corrected'] = sub_f['fz'] - baseline_fz
            sub_f['t0'] = sub_f['timestamp'] - sub_f['timestamp'].iloc[0]
            sub_f = self._smooth_by_phase(sub_f, 'Force_corrected', 'Force_smoothed')
            sub_f = self._smooth_by_phase(sub_f, 'fz_corrected', 'fz_smoothed')

            self.baselines_f[r] = baseline_f
            self.corrected_f[r] = sub_f
        return self.baselines_c, self.baselines_f

    def _shade_phases(self, ax, sub):
        phase_change = (sub['phase'] != sub['phase'].shift()).cumsum()
        for _, block in sub.groupby(phase_change):
            phase = block['phase'].iloc[0]
            t_start, t_end = block['t0'].iloc[0], block['t0'].iloc[-1]
            ax.axvspan(t_start, t_end, color=self.PHASE_COLORS.get(phase, 'lightgray'), alpha=0.15)

    def _shared_ylim(self, frames, column):
        all_vals = pd.concat([f[column] for f in frames.values()])
        y_min, y_max = all_vals.min(), all_vals.max()
        margin = (y_max - y_min) * 0.1
        return y_min - margin, y_max + margin

    def _integer_ticks_up_to(self, upper_value):
        top = max(1, math.ceil(upper_value))
        return list(range(0, top + 1))

    def plot_combined_grid(self, save_path=None):
        if not self.corrected_c:
            self.compute_baseline_subtraction()

        n = len(self.rounds)
        fig, axes = plt.subplots(2, n, figsize=(3.4 * n, 7.2), sharex=False)
        if n == 1:
            axes = axes.reshape(2, 1)

        ylim_f = self._shared_ylim(self.corrected_f, 'Force_corrected')
        force_ticks = self._integer_ticks_up_to(ylim_f[1])

        for col, r in enumerate(self.rounds):
            sub_c = self.corrected_c[r]
            ax_c = axes[0, col]
            self._shade_phases(ax_c, sub_c)

            hold_c = sub_c.loc[sub_c['phase'] == 'hold', 'Cp_smoothed']
            mean_c = hold_c.mean()
            co_val = self.baselines_c[r]

            ax_c.plot(sub_c['t0'], sub_c['Cp_corrected'], color='lightgray', lw=0.8)
            ax_c.plot(sub_c['t0'], sub_c['Cp_smoothed'], color='black', lw=1.2)
            ax_c.axhline(mean_c, color='red', lw=0.8, ls='--')
            ax_c.axhline(0, color='black', lw=0.5, ls=':')
            ax_c.set_ylim(self.CAP_YLIM)
            ax_c.set_yticks([0, -0.04, -0.08])
            ax_c.set_title(f'Iteration {r + 1}\nCo={co_val:.4f} pF', fontsize=10)
            ax_c.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_c.set_ylabel('ΔC (pF)')
            ax_c.text(0.5, -0.16, f'mean ΔC = {mean_c:.4f} pF',
                      transform=ax_c.transAxes, ha='center', va='top', fontsize=8, color='red')

            sub_f = self.corrected_f[r]
            hold_f = sub_f.loc[sub_f['phase'] == 'hold', 'Force_smoothed']
            settled_force = hold_f.mean()
            peak_force = hold_f.max()

            ax_f = axes[1, col]
            ax_f.plot(sub_f['t0'], sub_f['Force_corrected'], color='lightgray', lw=0.8)
            ax_f.plot(sub_f['t0'], sub_f['fz_corrected'], color='mistyrose', lw=0.8)
            ax_f.plot(sub_f['t0'], sub_f['Force_smoothed'], color='black', lw=1.2, ls='-',
                     marker='o', markersize=2, markevery=15)
            ax_f.plot(sub_f['t0'], sub_f['fz_smoothed'], color='tab:red', lw=1.2, ls='--', alpha=0.8,
                     marker='^', markersize=2, markevery=15)
            ax_f.axhline(settled_force, color='blue', lw=0.8, ls='--')
            ax_f.axhline(peak_force, color='green', lw=0.8, ls=':')
            ax_f.axhline(0, color='black', lw=0.5, ls=':')
            ax_f.set_ylim(ylim_f)
            ax_f.set_yticks(force_ticks)
            ax_f.set_xlabel('Time (s)')
            ax_f.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_f.set_ylabel('ΔForce (N)')
            ax_f.text(0.5, -0.32, f'settled = {settled_force:.2f} N   |   peak = {peak_force:.2f} N',
                      transform=ax_f.transAxes, ha='center', va='top', fontsize=8)

        legend_handles = [
            Line2D([0], [0], color='lightgray', lw=1.5, label='raw'),
            Line2D([0], [0], color='black', lw=1.5, label='smoothed'),
            Line2D([0], [0], color='black', lw=1.5, marker='o', markersize=5, label='load_cell_N'),
            Line2D([0], [0], color='tab:red', lw=1.5, ls='--', marker='^', markersize=5, label='fz'),
        ]
        fig.legend(handles=legend_handles, loc='upper center', ncol=4,
                  bbox_to_anchor=(0.5, 0.995), fontsize=9, frameon=False)

        depth_label = f", depth={self.depth_mm:.0f}mm" if self.depth_mm is not None else ""
        surf_label = f" [{self.surface}]" if self.surface else ""
        fig.suptitle(f'P{self.point:02d}{surf_label}{depth_label} — ΔC (top) and ΔForce: load_cell_N vs fz (bottom)',
                    y=1.05)

        plt.tight_layout(rect=[0, 0, 1, 0.93])
        fig.subplots_adjust(hspace=0.6)

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f'  [A] saved: {save_path}')
        else:
            plt.show()


# ── B. All-depths overlay — mirrors frames2.MultiDepthComparison ────────────────

class MultiDepthComparison:
    """For ONE (point, surface): overlays all depths present in the surface's
    df on the same figure. 2 rows x N iteration columns."""

    DEPTH_COLORS = plt.cm.viridis
    CAP_YLIM = (-0.09, 0.01)

    def __init__(self, df, point, surface=''):
        self.point = point
        self.surface = surface
        depths = sorted(df.loc[df['point'] == point, 'depth_mm'].unique())
        self.analyzers = {}
        for d in depths:
            analyzer = TouchSensorAnalyzer(df, point=point, depth_mm=d, surface=surface)
            analyzer.compute_baseline_subtraction()
            self.analyzers[d] = analyzer

        n_depths = len(self.analyzers)
        self._colors = [self.DEPTH_COLORS(i / max(1, n_depths - 1)) for i in range(n_depths)]

    def _shared_ylim_across_depths(self, column, corrected_attr):
        all_vals = []
        for analyzer in self.analyzers.values():
            for sub in getattr(analyzer, corrected_attr).values():
                all_vals.append(sub[column])
        all_vals = pd.concat(all_vals)
        y_min, y_max = all_vals.min(), all_vals.max()
        margin = (y_max - y_min) * 0.1
        return y_min - margin, y_max + margin

    def plot_depth_overlay_grid(self, save_path=None):
        rounds = sorted(next(iter(self.analyzers.values())).rounds)
        n = len(rounds)

        fig, axes = plt.subplots(2, n, figsize=(3.4 * n, 7.0), sharex=False)
        if n == 1:
            axes = axes.reshape(2, 1)

        ylim_f = self._shared_ylim_across_depths('Force_smoothed', 'corrected_f')

        legend_handles, legend_labels = [], []

        for col, r in enumerate(rounds):
            ax_c = axes[0, col]
            ax_f = axes[1, col]

            for i, depth_mm in enumerate(sorted(self.analyzers.keys())):
                analyzer = self.analyzers[depth_mm]
                color = self._colors[i]

                sub_c = analyzer.corrected_c[r]
                line_c, = ax_c.plot(sub_c['t0'], sub_c['Cp_smoothed'], color=color, lw=1.3)

                sub_f = analyzer.corrected_f[r]
                ax_f.plot(sub_f['t0'], sub_f['Force_smoothed'], color=color, lw=1.3)

                if col == 0:
                    legend_handles.append(line_c)
                    legend_labels.append(f'{depth_mm:.0f}mm')

            ax_c.axhline(0, color='black', lw=0.5, ls=':')
            ax_c.set_ylim(self.CAP_YLIM)
            ax_c.set_yticks([0, -0.04, -0.08])
            ax_c.set_title(f'Iteration {r + 1}', fontsize=10)
            ax_c.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_c.set_ylabel('ΔC (pF)')

            ax_f.axhline(0, color='black', lw=0.5, ls=':')
            ax_f.set_ylim(ylim_f)
            ax_f.set_xlabel('Time (s)')
            ax_f.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_f.set_ylabel('ΔForce (N)')

        fig.legend(legend_handles, legend_labels, loc='upper center',
                  bbox_to_anchor=(0.5, 0.995), ncol=len(legend_labels), fontsize=9,
                  title='Depth', frameon=False)

        surf_label = f' [{self.surface}]' if self.surface else ''
        fig.suptitle(f'P{self.point:02d}{surf_label} — ΔC (top) and ΔForce (bottom), all depths overlaid per iteration',
                    y=1.08)
        plt.tight_layout(rect=[0, 0, 1, 0.90])

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f'  [B] saved: {save_path}')
        else:
            plt.show()


# ── C. Iterations overlaid per depth — mirrors frames3.DepthColumnComparison ────

class DepthColumnComparison:
    """For ONE (point, surface): 3 rows x N depth columns. Row 0: ΔC,
    iterations overlaid. Row 1: ΔForce from fz. Row 2: ΔForce from load_cell."""

    ITERATION_COLORS = plt.cm.plasma
    CAP_YLIM = (-0.09, 0.01)

    def __init__(self, df, point, surface=''):
        self.point = point
        self.surface = surface
        depths = sorted(df.loc[df['point'] == point, 'depth_mm'].unique())
        self.analyzers = {}
        for d in depths:
            analyzer = TouchSensorAnalyzer(df, point=point, depth_mm=d, surface=surface)
            analyzer.compute_baseline_subtraction()
            self.analyzers[d] = analyzer

    def _shared_ylim_across_depths(self, column, corrected_attr):
        all_vals = []
        for analyzer in self.analyzers.values():
            for sub in getattr(analyzer, corrected_attr).values():
                all_vals.append(sub[column])
        all_vals = pd.concat(all_vals)
        y_min, y_max = all_vals.min(), all_vals.max()
        margin = (y_max - y_min) * 0.1
        return y_min - margin, y_max + margin

    def plot_iteration_overlay_grid(self, save_path=None):
        depths_sorted = sorted(self.analyzers.keys())
        n = len(depths_sorted)
        rounds = sorted(next(iter(self.analyzers.values())).rounds)

        fig, axes = plt.subplots(3, n, figsize=(3.4 * n, 10.0), sharex=False)
        if n == 1:
            axes = axes.reshape(3, 1)

        ylim_fz = self._shared_ylim_across_depths('fz_smoothed', 'corrected_f')
        ylim_lc = self._shared_ylim_across_depths('Force_smoothed', 'corrected_f')

        colors = [self.ITERATION_COLORS(i / max(1, len(rounds) - 1)) for i in range(len(rounds))]
        legend_handles, legend_labels = [], []

        for col, depth_mm in enumerate(depths_sorted):
            analyzer = self.analyzers[depth_mm]

            ax_c, ax_fz, ax_lc = axes[0, col], axes[1, col], axes[2, col]

            for i, r in enumerate(rounds):
                sub_c = analyzer.corrected_c[r]
                line_c, = ax_c.plot(sub_c['t0'], sub_c['Cp_smoothed'], color=colors[i], lw=1.3)

                sub_f = analyzer.corrected_f[r]
                ax_fz.plot(sub_f['t0'], sub_f['fz_smoothed'], color=colors[i], lw=1.3)
                ax_lc.plot(sub_f['t0'], sub_f['Force_smoothed'], color=colors[i], lw=1.3)

                if col == 0:
                    legend_handles.append(line_c)
                    legend_labels.append(f'Iteration {r + 1}')

            ax_c.axhline(0, color='black', lw=0.5, ls=':')
            ax_c.set_ylim(self.CAP_YLIM)
            ax_c.set_yticks([0, -0.04, -0.08])
            ax_c.set_title(f'Depth {depth_mm:.0f}mm', fontsize=10)
            ax_c.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_c.set_ylabel('ΔC (pF)')

            ax_fz.axhline(0, color='black', lw=0.5, ls=':')
            ax_fz.set_ylim(ylim_fz)
            ax_fz.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_fz.set_ylabel('ΔF, fz (N)\n[UR estimate]')

            ax_lc.axhline(0, color='black', lw=0.5, ls=':')
            ax_lc.set_ylim(ylim_lc)
            ax_lc.set_xlabel('Time (s)')
            ax_lc.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_lc.set_ylabel('ΔF, load_cell (N)\n[ground truth]')

        fig.legend(legend_handles, legend_labels, loc='upper center',
                  bbox_to_anchor=(0.5, 0.995), ncol=len(legend_labels), fontsize=9,
                  title='Iteration', frameon=False)

        surf_label = f' [{self.surface}]' if self.surface else ''
        fig.suptitle(f'P{self.point:02d}{surf_label} — ΔC, ΔF(fz/UR), ΔF(load_cell) — iterations overlaid per depth',
                    y=1.06)
        plt.tight_layout(rect=[0, 0, 1, 0.92])

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f'  [C] saved: {save_path}')
        else:
            plt.show()


# ── Main ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Generate depth-grid / all-depths-overlay / iterations-overlaid '
                    'figures for P08 and P19, across solid / hollow / flat surfaces.')
    p.add_argument('--surface', choices=['solid', 'hollow', 'flat'], default=None,
                   help='Only this surface (default: all three)')
    p.add_argument('--point', type=int, choices=POINTS, default=None,
                   help='Only this point (default: both P08 and P19)')
    p.add_argument('--no-depth-grids', action='store_true',
                   help='Skip figure type A (per-depth combined grids) — the slowest set')
    p.add_argument('--no-overlay', action='store_true', help='Skip figure type B')
    p.add_argument('--no-iter-overlay', action='store_true', help='Skip figure type C')
    return p.parse_args()


def main():
    args = parse_args()
    surfaces = [args.surface] if args.surface else list(SURFACE_FILES.keys())
    points = [args.point] if args.point else POINTS

    for surface in surfaces:
        print(f'\n[surface] {surface}')
        df = load_surface_df(surface)
        out_base = os.path.join(PLOT_DIR, f'twopt_p08_p19_{surface}')
        os.makedirs(out_base, exist_ok=True)

        for pt in points:
            if pt not in df['point'].unique():
                print(f'  [skip] P{pt:02d} not present in {surface} data')
                continue
            print(f'  [point] P{pt:02d}')

            if not args.no_depth_grids:
                grid_dir = os.path.join(out_base, 'depth_grids')
                os.makedirs(grid_dir, exist_ok=True)
                depths = sorted(df.loc[df['point'] == pt, 'depth_mm'].unique())
                for d in depths:
                    analyzer = TouchSensorAnalyzer(df, point=pt, depth_mm=d, surface=surface)
                    analyzer.compute_baseline_subtraction()
                    save_path = os.path.join(grid_dir, f'P{pt:02d}_depth{int(d)}mm.svg')
                    analyzer.plot_combined_grid(save_path=save_path)

            if not args.no_overlay:
                comp_b = MultiDepthComparison(df, point=pt, surface=surface)
                save_path = os.path.join(out_base, f'P{pt:02d}_all_depths.svg')
                comp_b.plot_depth_overlay_grid(save_path=save_path)

            if not args.no_iter_overlay:
                comp_c = DepthColumnComparison(df, point=pt, surface=surface)
                save_path = os.path.join(out_base, f'P{pt:02d}_iterations_overlaid.png')
                comp_c.plot_iteration_overlay_grid(save_path=save_path)

    print('\n[done]')


if __name__ == '__main__':
    main()
