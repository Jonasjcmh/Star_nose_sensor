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

    Legend: the depth->color mapping is the same in every column, so it's
    collected once (col == 0) and drawn as a single figure-level legend
    above the grid -- not repeated inside any subplot.
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

        empty_depths = []  # collect problems instead of failing deep inside a later plot call

        for raw_depth in sorted(depth_dataframes.keys()):
            df = depth_dataframes[raw_depth]
            analyzer = TouchSensorAnalyzer(df, point=point, depth_mm=raw_depth)
            analyzer.compute_baseline_subtraction()

            if not analyzer.rounds:
                # nothing matched (df['point'] == point) & (df['depth_mm'] == raw_depth)
                available_points = sorted(df['point'].unique().tolist())
                available_depths = sorted(df['depth_mm'].unique().tolist())
                empty_depths.append(
                    f"  depth_dataframes key={raw_depth}: point={point} not found, or "
                    f"depth_mm={raw_depth} not in file. "
                    f"File actually has points={available_points}, depth_mm values={available_depths}."
                )

            self.analyzers[raw_depth] = analyzer

        if empty_depths:
            raise ValueError(
                "No matching rows for point={} in the following depth_dataframes entries:\n{}\n"
                "Check that your dict keys match the real 'depth_mm' values in each CSV."
                .format(point, "\n".join(empty_depths))
            )

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

        if not all_vals:
            raise ValueError(
                f"No data available across any depth/round for column='{column}' "
                f"(attr='{corrected_attr}'). Check that depth_dataframes keys match "
                f"actual depth_mm values, and that point={self.point} exists in every file."
            )

        all_vals = pd.concat(all_vals)
        y_min, y_max = all_vals.min(), all_vals.max()
        margin = (y_max - y_min) * 0.1
        return y_min - margin, y_max + margin

    def plot_depth_overlay_grid(self, save_path=None):
        # all depths should share the same round numbers (0..4 -> iterations 1..5)
        rounds = sorted(next(iter(self.analyzers.values())).rounds)
        n = len(rounds)

        fig, axes = plt.subplots(2, n, figsize=(3.4 * n, 7.0), sharex=False)
        if n == 1:
            axes = axes.reshape(2, 1)  # keep 2D indexing consistent for a single iteration

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

                if col == 0:  # only collect legend handles once -- same mapping in every column
                    legend_handles.append(line_c)
                    legend_labels.append(f'{depth_actual:.0f}mm')

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

        # ---- ONE legend for the whole figure, drawn above the grid ----
        # Positioned and spaced the same way as in the single-depth script, so the
        # legend and the suptitle don't collide or get clipped on save.
        fig.legend(legend_handles, legend_labels, loc='upper center',
                   bbox_to_anchor=(0.5, 0.995), ncol=len(legend_labels),
                   fontsize=9, title='Depth', frameon=False)

        fig.suptitle(f'P{self.point} — ΔC (top) and ΔForce (bottom), all depths overlaid per iteration',
                     y=1.08)

        plt.tight_layout(rect=[0, 0, 1, 0.90])

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f'saved: {save_path}')
        else:
            plt.show()


if __name__ == '__main__':
    import os

    # ---- EDIT THIS LIST to your actual files ----
    # No more guessing keys: the real depth_mm value is read directly out of
    # each CSV below, so a mismatched label can't silently produce zero rows.
    csv_paths = [
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260709_144320_solidD_4mm.csv',
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260709_161711_solidD_2mm.csv',
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260710_140020_solidD_3mm.csv',
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260710_152938_solidD_1mm.csv',
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260710_164240_solidD_0mm.csv',
    ]
    output_dir = '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/data_analysis_ws_solid_dome/plots_comparison'
    # -----------------------------------------------

    os.makedirs(output_dir, exist_ok=True)

    depth_dataframes = {}
    for path in csv_paths:
        df = pd.read_csv(path)
        found_depths = df['depth_mm'].unique()

        if len(found_depths) != 1:
            raise ValueError(
                f"{path} contains more than one depth_mm value: {sorted(found_depths.tolist())}. "
                f"Expected exactly one depth per file."
            )

        raw_depth = found_depths[0]
        if raw_depth in depth_dataframes:
            raise ValueError(
                f"depth_mm={raw_depth} appears in more than one file (duplicate found at "
                f"{path}). Each depth should only be represented once."
            )

        depth_dataframes[raw_depth] = df

    # get all point numbers from any one of the loaded files
    all_points = sorted(next(iter(depth_dataframes.values()))['point'].unique())

    for pt in all_points:
        comparison = MultiDepthComparison(depth_dataframes, point=pt)
        save_path = os.path.join(output_dir, f'P{pt}_all_depths.svg')
        comparison.plot_depth_overlay_grid(save_path=save_path)
