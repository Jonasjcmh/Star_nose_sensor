import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib.lines import Line2D
from scipy.signal import savgol_filter


class TouchSensorAnalyzer:
    """
    One combined figure per point: 2 rows x N columns (N = number of rounds/iterations).
    Row 0: delta-C (Cp_corrected, baseline-subtracted) per iteration.
    Row 1: delta-force (Force_corrected, baseline-subtracted) per iteration,
           load_cell_N (solid) vs fz (dashed).
    Both rows share the same y-axis range across all iterations.

    Legend handling: line-style meaning (raw/smoothed, load_cell_N/fz) is the same
    in every subplot, so it's drawn ONCE as a figure-level legend above the whole
    grid instead of being repeated inside each plot. Per-plot numbers (mean delta-C,
    settled force, peak force) are still specific to each subplot, so they're kept,
    but as small text below each subplot rather than a legend box sitting on top
    of the data.
    """

    KEEP_PHASES = ['locate', 'hold', 'post']  # press/retract excluded from delta-C row
    PHASE_COLORS = {
        'locate': 'tab:gray',
        'hold':   'tab:green',
        'post':   'tab:gray',
    }
    DEPTH_OFFSET = 5  # raw depth_mm includes a 5mm offset; actual indentation = raw - 5
    CAP_YLIM = (-0.09, 0.01)  # fixed ΔC range so every plot is directly comparable
    SAVGOL_POLYORDER = 3  # polynomial order fit within each phase block
    # separate smoothing window per phase (in samples) -- hold gets a bigger window
    # than locate/post since it's a longer region
    SAVGOL_WINDOW_BY_PHASE = {
        'locate': 21,
        'hold':   101,
        'post':   21,
    }

    def __init__(self, df: pd.DataFrame, point: int, depth_mm: float = None):
        self.point = point
        self.depth_mm = depth_mm                      # raw value, e.g. 9.0
        self.depth_actual = None                       # converted value, e.g. 4.0
        if depth_mm is not None:
            self.depth_actual = depth_mm - self.DEPTH_OFFSET

        # full data (all phases) for this point -- used for the force row, which keeps ramps
        mask_full = (df['point'] == point)
        if depth_mm is not None:
            mask_full &= (df['depth_mm'] == depth_mm)
        self.data_full = df[mask_full].copy()

        # restricted data (locate/hold/post only) -- used for the delta-C row
        mask_c = mask_full & (df['phase'].isin(self.KEEP_PHASES))
        self.data_c = df[mask_c].copy()

        self.rounds = sorted(self.data_full['round_idx'].unique())
        self.baselines_c = {}   # round -> capacitance baseline (pF)
        self.baselines_f = {}   # round -> force baseline (N), from load_cell_N
        self.corrected_c = {}   # round -> dataframe (locate/hold/post) with Cp_corrected, t0
        self.corrected_f = {}   # round -> dataframe (full trace) with Force_corrected, fz_corrected, t0

    def __repr__(self):
        return f"TouchSensorAnalyzer(point={self.point}, depth_mm={self.depth_mm}, rounds={self.rounds})"

    def _smooth_by_phase(self, sub, value_col, out_col):
        """
        Apply Savitzky-Golay smoothing separately to each contiguous phase block
        (locate/press/hold/retract/post), using SAVGOL_WINDOW_BY_PHASE for the
        window size per phase. Writes result into out_col; falls back to raw
        values for phases/blocks too short to smooth safely.
        """
        sub[out_col] = sub[value_col].copy()
        phase_change = (sub['phase'] != sub['phase'].shift()).cumsum()
        for _, block_idx in sub.groupby(phase_change).groups.items():
            block = sub.loc[block_idx]
            phase = block['phase'].iloc[0]
            window = self.SAVGOL_WINDOW_BY_PHASE.get(phase, 21)
            window = min(window, len(block) - (1 - len(block) % 2))  # clip to block length, force odd
            if window % 2 == 0:
                window -= 1
            if window >= self.SAVGOL_POLYORDER + 2:
                sub.loc[block_idx, out_col] = savgol_filter(block[value_col], window, self.SAVGOL_POLYORDER)
            # else: too few points in this block, leave as raw (already copied above)
        return sub

    def compute_baseline_subtraction(self):
        """
        Capacitance: baseline = average of (mean Cp during locate, mean Cp during post),
                     applied only to locate/hold/post samples.
        Force:       baseline = average of (mean load_cell_N during locate, mean during post),
                     applied to the FULL trace (including press/retract) so the ramps
                     still show, just referenced to a zero resting point.
                     fz gets the same treatment using its own locate/post mean, so both
                     lines start at 0 and are directly comparable.
        """
        for r in self.rounds:
            # --- capacitance (locate/hold/post only) ---
            sub_c = self.data_c[self.data_c['round_idx'] == r].sort_values('timestamp').copy()
            c_locate = sub_c.loc[sub_c['phase'] == 'locate', 'Cp_pF'].mean()
            c_post = sub_c.loc[sub_c['phase'] == 'post', 'Cp_pF'].mean()
            baseline_c = (c_locate + c_post) / 2

            sub_c['Cp_corrected'] = sub_c['Cp_pF'] - baseline_c
            sub_c['t0'] = sub_c['timestamp'] - sub_c['timestamp'].iloc[0]

            # Savitzky-Golay smoothing, applied SEPARATELY per phase block (locate/hold/post)
            sub_c = self._smooth_by_phase(sub_c, 'Cp_corrected', 'Cp_smoothed')

            self.baselines_c[r] = baseline_c
            self.corrected_c[r] = sub_c

            # --- force (full trace, including press/retract) ---
            sub_f = self.data_full[self.data_full['round_idx'] == r].sort_values('timestamp').copy()

            f_locate = sub_f.loc[sub_f['phase'] == 'locate', 'load_cell_N'].mean()
            f_post = sub_f.loc[sub_f['phase'] == 'post', 'load_cell_N'].mean()
            baseline_f = (f_locate + f_post) / 2

            fz_locate = sub_f.loc[sub_f['phase'] == 'locate', 'fz'].mean()
            fz_post = sub_f.loc[sub_f['phase'] == 'post', 'fz'].mean()
            baseline_fz = (fz_locate + fz_post) / 2

            sub_f['Force_corrected'] = sub_f['load_cell_N'] - baseline_f
            sub_f['fz_corrected'] = sub_f['fz'] - baseline_fz
            sub_f['t0'] = sub_f['timestamp'] - sub_f['timestamp'].iloc[0]

            # smoothing on force too -- same function, same per-phase logic as capacitance
            sub_f = self._smooth_by_phase(sub_f, 'Force_corrected', 'Force_smoothed')
            sub_f = self._smooth_by_phase(sub_f, 'fz_corrected', 'fz_smoothed')

            self.baselines_f[r] = baseline_f
            self.corrected_f[r] = sub_f

        return self.baselines_c, self.baselines_f

    def _shade_phases(self, ax, sub):
        """Shade contiguous locate/hold/post blocks on a given axis."""
        phase_change = (sub['phase'] != sub['phase'].shift()).cumsum()
        for _, block in sub.groupby(phase_change):
            phase = block['phase'].iloc[0]
            t_start, t_end = block['t0'].iloc[0], block['t0'].iloc[-1]
            ax.axvspan(t_start, t_end, color=self.PHASE_COLORS.get(phase, 'lightgray'), alpha=0.15)

    def _shared_ylim(self, frames, column):
        """Compute a common (min, max) with 10% margin across all rounds for one column name."""
        all_vals = pd.concat([f[column] for f in frames.values()])
        y_min, y_max = all_vals.min(), all_vals.max()
        margin = (y_max - y_min) * 0.1
        return y_min - margin, y_max + margin

    def _integer_ticks_up_to(self, upper_value):
        """
        Build a list of integer ticks [0, 1, 2, ..., ceil(upper_value)].
        Used for the force row so the tick range auto-adapts to whatever
        the actual peak force is for this point/depth, instead of a
        hardcoded range that's wrong for shallower depths.
        """
        import math
        top = max(1, math.ceil(upper_value))
        return list(range(0, top + 1))

    def plot_combined_grid(self, save_path=None):
        """
        One figure, 2 rows x N columns.
        Row 0: Cp_corrected (delta-C), locate/hold/post only, phase-shaded, shared y-axis.
        Row 1: Force_corrected (load_cell_N, solid) vs fz_corrected (dashed), full trace,
               shared y-axis with ticks auto-computed from the actual peak force.

        Legends: ONE shared legend for the whole figure (line-style meaning: raw vs
        smoothed, load_cell_N vs fz), drawn above the grid, not inside any subplot.
        Per-plot numbers (mean delta-C / settled / peak) are written as small text
        underneath each subplot instead of a per-subplot legend, so nothing sits on
        top of the traces.

        If save_path is given, the figure is saved to that path instead of being shown
        interactively -- useful when generating many figures in a loop.
        """
        if not self.corrected_c:
            self.compute_baseline_subtraction()

        n = len(self.rounds)
        fig, axes = plt.subplots(2, n, figsize=(3.4 * n, 7.2), sharex=False)
        if n == 1:
            axes = axes.reshape(2, 1)  # keep 2D indexing consistent for a single iteration

        # compute shared y-limits once, across all rounds
        ylim_f = self._shared_ylim(self.corrected_f, 'Force_corrected')
        force_ticks = self._integer_ticks_up_to(ylim_f[1])

        for col, r in enumerate(self.rounds):
            # ---------- row 0: delta C (locate/hold/post only) ----------
            sub_c = self.corrected_c[r]
            ax_c = axes[0, col]
            self._shade_phases(ax_c, sub_c)

            hold_c = sub_c.loc[sub_c['phase'] == 'hold', 'Cp_smoothed']
            mean_c = hold_c.mean()
            co_val = self.baselines_c[r]

            # no per-line labels here -- meaning is covered once by the figure-level legend
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

            # per-plot value -> text below the subplot, not a legend on top of the data
            ax_c.text(0.5, -0.16, f'mean ΔC = {mean_c:.4f} pF',
                       transform=ax_c.transAxes, ha='center', va='top',
                       fontsize=8, color='red')

            # ---------- row 1: force comparison (full trace, ramps included, baseline-subtracted) ----------
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
            ax_f.set_yticks(force_ticks)  # auto-adapts to the actual peak force for this point/depth
            ax_f.set_xlabel('Time (s)')
            ax_f.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_f.set_ylabel('ΔForce (N)')

            # per-plot values -> text below the subplot
            ax_f.text(0.5, -0.32,
                      f'settled = {settled_force:.2f} N   |   peak = {peak_force:.2f} N',
                      transform=ax_f.transAxes, ha='center', va='top', fontsize=8)

        # ---------- ONE shared legend for the whole figure ----------
        # raw/smoothed and load_cell_N/fz mean the same thing in every subplot,
        # so state it once here, above the grid, instead of repeating it in every plot.
        legend_handles = [
            Line2D([0], [0], color='lightgray', lw=1.5, label='raw'),
            Line2D([0], [0], color='black', lw=1.5, label='smoothed'),
            Line2D([0], [0], color='black', lw=1.5, marker='o', markersize=5, label='load_cell_N'),
            Line2D([0], [0], color='tab:red', lw=1.5, ls='--', marker='^', markersize=5, label='fz'),
        ]
        fig.legend(handles=legend_handles, loc='upper center', ncol=4,
                   bbox_to_anchor=(0.5, 0.995), fontsize=9, frameon=False)

        depth_label = f", depth={self.depth_actual:.0f}mm" if self.depth_actual is not None else ""
        fig.suptitle(f'P{self.point}{depth_label} — ΔC (top row) and ΔForce: load_cell_N vs fz (bottom row)',
                     y=1.05)

        plt.tight_layout(rect=[0, 0, 1, 0.93])
        fig.subplots_adjust(hspace=0.6)  # extra room so the text under each subplot doesn't collide

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close(fig)   # free memory -- important when looping over many points
            print(f'saved: {save_path}')
        else:
            plt.show()


if __name__ == '__main__':
    import os

    df = pd.read_csv('/home/divuthejo/Star_nose_sensor/Capacitance_measurement/logs/ramp_collector_20260710_164240_solidD_0mm.csv')
    output_dir = '/home/divuthejo/Star_nose_sensor/Capacitance_measurement/data_analysis_ws_solid_dome/plots_0mm'
    os.makedirs(output_dir, exist_ok=True)

    depth = 5.0  # change this to the raw depth_mm value in whatever file you're running

    all_points = sorted(df['point'].unique())
    for pt in all_points:
        analyzer = TouchSensorAnalyzer(df, point=pt, depth_mm=depth)
        analyzer.compute_baseline_subtraction()

        depth_actual = int(depth - TouchSensorAnalyzer.DEPTH_OFFSET)
        filename = f'P{pt}_depth{depth_actual}mm.svg'
        save_path = os.path.join(output_dir, filename)

        analyzer.plot_combined_grid(save_path=save_path)