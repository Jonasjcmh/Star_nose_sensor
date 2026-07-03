import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from frames_v2 import TouchSensorAnalyzer   # reuse baseline + smoothing logic


class DepthColumnComparison:
    """
    For ONE point: 3 rows x N columns (N = number of depths, e.g. 0,1,2,3,4mm).
    Row 0: ΔC, all 5 iterations overlaid per depth column.
    Row 1: ΔForce from fz (robot's own internal force estimate), iterations overlaid.
    Row 2: ΔForce from load_cell_N (ground-truth external sensor), iterations overlaid.

    Internally builds one TouchSensorAnalyzer per depth (reusing baseline
    subtraction + Savitzky-Golay smoothing, all previously built), then
    plots iterations overlaid within each depth's column.
    """

    ITERATION_COLORS = plt.cm.plasma  # colormap; sampled per iteration

    def __init__(self, depth_dataframes: dict, point: int):
        """
        depth_dataframes: dict mapping RAW depth_mm (e.g. 5.0, 6.0, 7.0, 8.0, 9.0)
                           -> the dataframe loaded from that depth's CSV file.
        point: the sensor point number to analyze across all these depths.
        """
        self.point = point
        self.analyzers = {}   # raw_depth_mm -> TouchSensorAnalyzer (already computed)

        for raw_depth in sorted(depth_dataframes.keys()):
            df = depth_dataframes[raw_depth]
            analyzer = TouchSensorAnalyzer(df, point=point, depth_mm=raw_depth)
            analyzer.compute_baseline_subtraction()
            self.analyzers[raw_depth] = analyzer

    def __repr__(self):
        depths_actual = [a.depth_actual for a in self.analyzers.values()]
        return f"DepthColumnComparison(point={self.point}, depths_actual={depths_actual})"

    def _shared_ylim_across_depths(self, column, corrected_attr):
        """Compute one y-range across ALL depths and ALL rounds, for a given column name."""
        all_vals = []
        for analyzer in self.analyzers.values():
            frames = getattr(analyzer, corrected_attr)
            for sub in frames.values():
                all_vals.append(sub[column])
        all_vals = pd.concat(all_vals)
        y_min, y_max = all_vals.min(), all_vals.max()
        margin = (y_max - y_min) * 0.1
        return y_min - margin, y_max + margin

    def plot_iteration_overlay_grid(self, save_path=None):
        """
        3 rows x N columns, N = number of depths.
        Row 0: ΔC, iterations 1-5 overlaid per depth column.
        Row 1: ΔForce from fz (robot's own internal force estimate), iterations overlaid.
        Row 2: ΔForce from load_cell_N (ground-truth external sensor), iterations overlaid.
        """
        depths_sorted = sorted(self.analyzers.keys())
        n = len(depths_sorted)
        rounds = sorted(next(iter(self.analyzers.values())).rounds)

        fig, axes = plt.subplots(3, n, figsize=(3.4 * n, 9.5), sharex=False)

        ylim_c = self._shared_ylim_across_depths('Cp_smoothed', 'corrected_c')
        ylim_fz = self._shared_ylim_across_depths('fz_smoothed', 'corrected_f')
        ylim_lc = self._shared_ylim_across_depths('Force_smoothed', 'corrected_f')

        colors = [self.ITERATION_COLORS(i / max(1, len(rounds) - 1)) for i in range(len(rounds))]

        legend_handles = []
        legend_labels = []

        for col, raw_depth in enumerate(depths_sorted):
            analyzer = self.analyzers[raw_depth]
            depth_actual = analyzer.depth_actual

            ax_c = axes[0, col]
            ax_fz = axes[1, col]
            ax_lc = axes[2, col]

            for i, r in enumerate(rounds):
                sub_c = analyzer.corrected_c[r]
                line_c, = ax_c.plot(sub_c['t0'], sub_c['Cp_smoothed'], color=colors[i], lw=1.3)

                sub_f = analyzer.corrected_f[r]
                ax_fz.plot(sub_f['t0'], sub_f['fz_smoothed'], color=colors[i], lw=1.3)
                ax_lc.plot(sub_f['t0'], sub_f['Force_smoothed'], color=colors[i], lw=1.3)

                if col == 0:  # only collect legend handles once
                    legend_handles.append(line_c)
                    legend_labels.append(f'Iteration {r + 1}')

            ax_c.axhline(0, color='black', lw=0.5, ls=':')
            ax_c.set_ylim(ylim_c)
            ax_c.set_yticks([0, -0.04, -0.08])
            ax_c.set_title(f'Depth {depth_actual:.0f}mm')
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
                   bbox_to_anchor=(0.5, 1.03), ncol=len(legend_labels), fontsize=9, title='Iteration')

        fig.suptitle(f'P{self.point} — ΔC, ΔF(fz/UR), ΔF(load_cell) — iterations overlaid per depth', y=1.07)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f'saved: {save_path}')
        else:
            plt.show()


if __name__ == '__main__':
    import os

    # ---- EDIT THESE PATHS to your actual files ----
    depth_files = {
        5: '/home/divuthejo/Downloads/ramp_collector_20260630_180318_5mm.csv',
        6: '/home/divuthejo/Downloads/ramp_collector_20260626_155329_6mm.csv',
        7: '/home/divuthejo/Downloads/ramp_collector_20260626_171140_7mm.csv',
        8: '/home/divuthejo/Downloads/ramp_collector_20260627_122027_8mm.csv',
        9: '/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv',   # actual 4mm
    }
    output_dir = '/home/divuthejo/Documents/data_analysis_ws/plots'
    # -------------------------------------------------

    os.makedirs(output_dir, exist_ok=True)
    depth_dataframes = {raw_depth: pd.read_csv(path) for raw_depth, path in depth_files.items()}

    # get all point numbers from any one of the loaded files
    all_points = sorted(next(iter(depth_dataframes.values()))['point'].unique())

    for pt in all_points:
        comparison = DepthColumnComparison(depth_dataframes, point=pt)
        save_path = os.path.join(output_dir, f'P{pt}_iterations_overlaid.png')
        comparison.plot_iteration_overlay_grid(save_path=save_path)