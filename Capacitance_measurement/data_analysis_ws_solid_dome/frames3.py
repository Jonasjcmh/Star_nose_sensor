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
    CAP_YLIM = (-0.09, 0.01)  # fixed ΔC range so every plot is directly comparable

    def __init__(self, depth_dataframes: dict, point: int):
        """
        depth_dataframes: dict mapping RAW depth_mm (e.g. 5.0, 6.0, 7.0, 8.0, 9.0)
                           -> the dataframe loaded from that depth's CSV file.
                           NOTE: these dict keys must match the actual values in
                           each dataframe's 'depth_mm' column -- they are used
                           directly as a filter (df['depth_mm'] == raw_depth).
                           If a key doesn't match what's really in that file,
                           the analyzer for that depth silently gets zero rows.
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
                # -- record what WAS actually available in this file so the mismatch
                # is diagnosable instead of a bare pandas crash three calls later.
                available_points = sorted(df['point'].unique().tolist())
                available_depths = sorted(df['depth_mm'].unique().tolist())
                empty_depths.append(
                    f"  depth_files key={raw_depth}: point={point} not found, or "
                    f"depth_mm={raw_depth} not in file. "
                    f"File actually has points={available_points}, depth_mm values={available_depths}."
                )

            self.analyzers[raw_depth] = analyzer

        if empty_depths:
            raise ValueError(
                "No matching rows for point={} in the following depth_files entries:\n{}\n"
                "Check that your depth_files dict keys match the real 'depth_mm' values "
                "in each CSV -- they're currently used directly as a filter, so a key "
                "that doesn't match the file's actual depth_mm silently produces zero rows."
                .format(point, "\n".join(empty_depths))
            )

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

        if not all_vals:
            # Should not normally get here since __init__ already checks for empty
            # analyzers, but guard anyway so this never surfaces as a bare
            # "No objects to concatenate" crash from inside pandas.
            raise ValueError(
                f"No data available across any depth/round for column='{column}' "
                f"(attr='{corrected_attr}'). Check depth_files keys match actual "
                f"depth_mm values, and that point={self.point} exists in every file."
            )

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

        fig, axes = plt.subplots(3, n, figsize=(3.4 * n, 10.0), sharex=False)
        if n == 1:
            axes = axes.reshape(3, 1)  # keep 2D indexing consistent for a single depth

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

                if col == 0:  # only collect legend handles once -- same mapping in every column
                    legend_handles.append(line_c)
                    legend_labels.append(f'Iteration {r + 1}')

            ax_c.axhline(0, color='black', lw=0.5, ls=':')
            ax_c.set_ylim(self.CAP_YLIM)
            ax_c.set_yticks([0, -0.04, -0.08])
            ax_c.set_title(f'Depth {depth_actual:.0f}mm', fontsize=10)
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

        # ---- ONE legend for the whole figure, drawn above the grid ----
        # (same iteration->color mapping every column, so collected once above)
        fig.legend(legend_handles, legend_labels, loc='upper center',
                   bbox_to_anchor=(0.5, 0.995), ncol=len(legend_labels),
                   fontsize=9, title='Iteration', frameon=False)

        fig.suptitle(f'P{self.point} — ΔC, ΔF(fz/UR), ΔF(load_cell) — iterations overlaid per depth',
                     y=1.06)

        plt.tight_layout(rect=[0, 0, 1, 0.92])

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
    # each CSV below, so a mismatched label can't silently produce zero rows
    # anymore.
    csv_paths = [
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260709_144320_solidD_4mm.csv',
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260709_161711_solidD_2mm.csv',
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260710_140020_solidD_3mm.csv',
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260710_152938_solidD_1mm.csv',
        '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260710_164240_solidD_0mm.csv',
    ]
    output_dir = '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/data_analysis_ws_solid_dome/plots'
    # -----------------------------------------------

    os.makedirs(output_dir, exist_ok=True)

    depth_dataframes = {}
    for path in csv_paths:
        df = pd.read_csv(path)
        found_depths = df['depth_mm'].unique()

        if len(found_depths) != 1:
            # a file mixing multiple depth_mm values would silently corrupt the
            # per-depth grouping below, so fail loudly here instead
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
        comparison = DepthColumnComparison(depth_dataframes, point=pt)
        save_path = os.path.join(output_dir, f'P{pt}_iterations_overlaid.png')
        comparison.plot_iteration_overlay_grid(save_path=save_path)