import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from scipy.signal import savgol_filter


class TouchSensorAnalyzer:
    """
    One combined figure per point: 2 rows x N columns (N = number of rounds/iterations).
    Row 0: delta-C (Cp_corrected, baseline-subtracted, Savitzky-Golay smoothed per phase) per iteration.
    Row 1: delta-force (Force_corrected, baseline-subtracted) per iteration,
           load_cell_N (solid) vs fz (dashed).
    Both rows share the same y-axis range across all iterations.
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
    # separate smoothing window per phase (in samples) -- hold gets a much bigger
    # window since it's a long flat region we want to look square/flat, while
    # locate/post get a lighter touch since they're short and already fairly flat
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
        self.corrected_c = {}   # round -> dataframe (locate/hold/post) with Cp_corrected, Cp_smoothed, t0
        self.corrected_f = {}   # round -> dataframe (full trace) with Force_corrected, fz_corrected, t0

    def __repr__(self):
        return f"TouchSensorAnalyzer(point={self.point}, depth_mm={self.depth_mm}, rounds={self.rounds})"

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

            # Savitzky-Golay smoothing, applied SEPARATELY per phase block (locate/hold/post),
            # each with its own window. This lets 'hold' get a much heavier window so the
            # noisy dwell region flattens out, without over-smoothing the shorter locate/post
            # segments or blurring across phase boundaries.
            sub_c['Cp_smoothed'] = sub_c['Cp_corrected'].copy()
            phase_change = (sub_c['phase'] != sub_c['phase'].shift()).cumsum()
            for _, block_idx in sub_c.groupby(phase_change).groups.items():
                block = sub_c.loc[block_idx]
                phase = block['phase'].iloc[0]
                window = self.SAVGOL_WINDOW_BY_PHASE.get(phase, 21)
                window = min(window, len(block) - (1 - len(block) % 2))  # clip to block length, force odd
                if window % 2 == 0:
                    window -= 1
                if window >= self.SAVGOL_POLYORDER + 2:
                    smoothed = savgol_filter(block['Cp_corrected'], window, self.SAVGOL_POLYORDER)
                    sub_c.loc[block_idx, 'Cp_smoothed'] = smoothed
                # else: too few points in this block, leave as raw (already copied above)

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

    def plot_combined_grid(self):
        """
        One figure, 2 rows x N columns.
        Row 0: Cp_corrected (raw, faint gray) + Cp_smoothed (black), locate/hold/post only,
               phase-shaded, shared y-axis, integer x-ticks.
        Row 1: Force_corrected (load_cell_N, solid) vs fz_corrected (dashed), full trace,
               shared y-axis, ticks 0-5.
        """
        if not self.corrected_c:
            self.compute_baseline_subtraction()

        n = len(self.rounds)
        fig, axes = plt.subplots(2, n, figsize=(3.4 * n, 6.5), sharex=False)

        # compute shared y-limits once, across all rounds
        ylim_f = self._shared_ylim(self.corrected_f, 'Force_corrected')

        for col, r in enumerate(self.rounds):
            # ---------- row 0: delta C (locate/hold/post only) ----------
            sub_c = self.corrected_c[r]
            ax_c = axes[0, col]
            self._shade_phases(ax_c, sub_c)

            hold_c = sub_c.loc[sub_c['phase'] == 'hold', 'Cp_smoothed']
            mean_c = hold_c.mean()
            co_val = self.baselines_c[r]

            ax_c.plot(sub_c['t0'], sub_c['Cp_corrected'], color='lightgray', lw=0.8, label='raw')
            ax_c.plot(sub_c['t0'], sub_c['Cp_smoothed'], color='black', lw=1.2, label='smoothed')
            ax_c.axhline(mean_c, color='red', lw=0.8, ls='--', label=f'mean ΔC={mean_c:.4f} pF')
            ax_c.axhline(0, color='black', lw=0.5, ls=':')
            ax_c.set_ylim(self.CAP_YLIM)
            ax_c.set_yticks([0, -0.04, -0.08])
            ax_c.set_title(f'Iteration {r + 1}\nCo={co_val:.4f} pF')
            ax_c.legend(loc='upper right', fontsize=9)
            ax_c.xaxis.set_major_locator(MaxNLocator(integer=True))
            if col == 0:
                ax_c.set_ylabel('ΔC (pF)')

            # ---------- row 1: force comparison (full trace, ramps included, baseline-subtracted) ----------
            sub_f = self.corrected_f[r]

            hold_f = sub_f.loc[sub_f['phase'] == 'hold', 'Force_corrected']
            settled_force = hold_f.mean()
            peak_force = hold_f.max()

            ax_f = axes[1, col]
            ax_f.plot(sub_f['t0'], sub_f['Force_corrected'], color='black', lw=1.2, ls='-',
                      marker='o', markersize=2, markevery=15, label='load_cell_N')
            ax_f.plot(sub_f['t0'], sub_f['fz_corrected'], color='tab:red', lw=1.2, ls='--', alpha=0.8,
                      marker='^', markersize=2, markevery=15, label='fz')
            ax_f.axhline(settled_force, color='blue', lw=0.8, ls='--',
                         label=f'settled={settled_force:.2f} N')
            ax_f.axhline(peak_force, color='green', lw=0.8, ls=':',
                         label=f'peak={peak_force:.2f} N')
            ax_f.axhline(0, color='black', lw=0.5, ls=':')
            ax_f.set_ylim(ylim_f)
            ax_f.set_yticks(range(0, 6))  # force axis ticks 0..5
            ax_f.set_xlabel('Time (s)')
            ax_f.legend(loc='upper right', fontsize=9)
            if col == 0:
                ax_f.set_ylabel('ΔForce (N)')

        depth_label = f", depth={self.depth_actual:.0f}mm" if self.depth_actual is not None else ""
        fig.suptitle(f'P{self.point}{depth_label} — ΔC (top row) and ΔForce: load_cell_N vs fz (bottom row)')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    df = pd.read_csv('/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv')

    # change point= for a different sensor location
    analyzer = TouchSensorAnalyzer(df, point=10, depth_mm=9.0)
    analyzer.compute_baseline_subtraction()
    analyzer.plot_combined_grid()