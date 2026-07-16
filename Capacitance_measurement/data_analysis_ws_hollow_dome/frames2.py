import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from frames_v2 import TouchSensorAnalyzer  # reuse baseline + smoothing logic


class MultiDepthComparison:
    """
    For ONE point, overlays multiple depths on the same figure.
    2 rows x N columns (N = number of iterations/rounds).
    Row 0: ΔC, all depths overlaid per column.
    Row 1: ΔForce (load_cell_N), all depths overlaid per column.

    Internally builds one TouchSensorAnalyzer per depth (reusing all the
    baseline-subtraction + Savitzky-Golay smoothing already built there),
    then plots them together instead of one-figure-per-depth.
    """

    DEPTH_COLORS = plt.cm.viridis  # colormap; sampled per depth in __init__
    CAP_YLIM = (-0.09, 0.01)  # fixed ΔC range so every plot is directly comparable

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

        # one distinct color per depth, sampled evenly from a colormap
        n_depths = len(self.analyzers)
        self._colors = [self.DEPTH_COLORS(i / max(1, n_depths - 1)) for i in range(n_depths)]

    def __repr__(self):
        depths_actual = [a.depth_actual for a in self.analyzers.values()]
        return f"MultiDepthComparison(point={self.point}, depths_actual={depths_actual})"

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

    def plot_depth_overlay_grid(self, save_path=None):
        # all depths should share the same round numbers (0..4 -> iterations 1..5)
        rounds = sorted(next(iter(self.analyzers.values())).rounds)
        n = len(rounds)

        fig, axes = plt.subplots(2, n, figsize=(3.4 * n, 6.5), sharex=False)

        ylim_f = self._shared_ylim_across_depths('Force_smoothed', 'corrected_f')

        legend_handles = []
        legend_labels = []

        for col, r in enumerate(rounds):
            ax_c = axes[0, col]
            ax_f = axes[1, col]

            for i, raw_depth in enumerate(sorted(self.analyzers.keys())):
                analyzer = self.analyzers[raw_depth]
                color = self._colors[i]
                depth_actual = analyzer.depth_actual

                sub_c = analyzer.corrected_c[r]
                line_c, = ax_c.plot(sub_c['t0'], sub_c['Cp_smoothed'], color=color, lw=1.3)

                sub_f = analyzer.corrected_f[r]
                ax_f.plot(sub_f['t0'], sub_f['Force_smoothed'], color=color, lw=1.3)

                if col == 0:  # only collect legend handles once
                    legend_handles.append(line_c)
                    legend_labels.append(f'{depth_actual:.0f}mm')

            ax_c.axhline(0, color='black', lw=0.5, ls=':')
            ax_c.set_ylim(self.CAP_YLIM)
            ax_c.set_yticks([0, -0.04, -0.08])
            ax_c.set_title(f'Iteration {r + 1}')
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
                   bbox_to_anchor=(0.5, 1.04), ncol=len(legend_labels), fontsize=9, title='Depth')

        fig.suptitle(f'P{self.point} — ΔC (top) and ΔForce (bottom), all depths overlaid per iteration', y=1.1)
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
        9: '/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv',  # actual 4mm
    }
    output_dir = '/home/divuthejo/Documents/data_analysis_ws/plots_comparison'
    # -------------------------------------------------

    os.makedirs(output_dir, exist_ok=True)
    depth_dataframes = {raw_depth: pd.read_csv(path) for raw_depth, path in depth_files.items()}

    # get all point numbers from any one of the loaded files
    all_points = sorted(next(iter(depth_dataframes.values()))['point'].unique())

    for pt in all_points:
        comparison = MultiDepthComparison(depth_dataframes, point=pt)
        save_path = os.path.join(output_dir, f'P{pt}_all_depths.svg')
        comparison.plot_depth_overlay_grid(save_path=save_path)