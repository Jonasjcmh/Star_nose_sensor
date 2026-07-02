import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

SETTLE_WINDOW_S = 1.0  # matches capacitance_ramp_collector.SETTLE_WINDOW_S


class TouchSensorAnalyzer:
    KEEP_PHASES = ['locate', 'hold', 'post']  # press/retract excluded from analysis
    PHASE_COLORS = {
        'locate': 'tab:gray',
        'hold':   'tab:green',
        'post':   'tab:gray',
    }

    def __init__(self, df: pd.DataFrame, point: int, force_col: str = 'load_cell_N'):
        self.point = point
        self.force_col = force_col
        self.data = df[(df['point'] == point) & (df['phase'].isin(self.KEEP_PHASES))].copy()
        self.rounds = sorted(self.data['round_idx'].unique())
        self.baselines = {}
        self.corrected = {}

    def __repr__(self):
        return f"TouchSensorAnalyzer(point={self.point}, rounds={self.rounds})"

    @staticmethod
    def _tail_mean(sub, phase, col, window_s=SETTLE_WINDOW_S):
        """Mean of `col` over the last `window_s` seconds of `phase`."""
        prows = sub[sub['phase'] == phase]
        if prows.empty:
            return float('nan')
        t_end = prows['timestamp'].iloc[-1]
        tail = prows[prows['timestamp'] >= t_end - window_s]
        return tail[col].mean()

    def compute_baseline_subtraction(self):
        for r in self.rounds:
            sub = self.data[self.data['round_idx'] == r].sort_values('timestamp').copy()

            base_locate = sub.loc[sub['phase'] == 'locate', 'Cp_pF'].mean()
            base_post = sub.loc[sub['phase'] == 'post', 'Cp_pF'].mean()
            baseline = (base_locate + base_post) / 2

            sub['Cp_corrected'] = sub['Cp_pF'] - baseline
            sub['t0'] = sub['timestamp'] - sub['timestamp'].iloc[0]

            self.baselines[r] = baseline
            self.corrected[r] = sub

        return self.baselines

    def plot_no_ramps_with_force(self, share_yaxis=True, n_force_ticks=8):
        """
        Per-round subplots, ramps excluded. Cp_corrected (black, left axis) +
        force (blue, right axis, with n_force_ticks tick marks for finer
        resolution). locate/hold/post shaded. Legend shows Co, settled ΔCp,
        and settled/peak force.
        """
        if not self.corrected:
            self.compute_baseline_subtraction()

        n = len(self.rounds)
        fig, axes = plt.subplots(n, 1, figsize=(9, 2.6 * n), sharex=True, sharey=share_yaxis)
        if n == 1:
            axes = [axes]

        for i, r in enumerate(self.rounds):
            sub = self.corrected[r]
            ax = axes[i]
            ax_force = ax.twinx()

            # shade each contiguous phase block (locate/hold/post)
            phase_change = (sub['phase'] != sub['phase'].shift()).cumsum()
            for _, block in sub.groupby(phase_change):
                phase = block['phase'].iloc[0]
                t_start, t_end = block['t0'].iloc[0], block['t0'].iloc[-1]
                ax.axvspan(t_start, t_end, color=self.PHASE_COLORS.get(phase, 'lightgray'), alpha=0.15)

            co_val = self.baselines[r]

            cp_hold_settled = self._tail_mean(sub, 'hold', 'Cp_pF')
            dcp_settled = cp_hold_settled - co_val
            force_settled = self._tail_mean(sub, 'hold', self.force_col)
            force_peak = sub.loc[sub['phase'] == 'hold', self.force_col].max()

            l1, = ax.plot(sub['t0'], sub['Cp_corrected'], color='black', lw=1,
                           label=f'Co={co_val:.4f} pF')
            l2 = ax.axhline(dcp_settled, color='red', lw=0.8, ls='--',
                             label=f'ΔCp(settled)={dcp_settled:.4f} pF')
            ax.axhline(0, color='black', lw=0.5, ls=':')

            l3, = ax_force.plot(sub['t0'], sub[self.force_col], color='tab:blue', lw=1, alpha=0.7,
                                 label=f'force(settled)={force_settled:.3f}, peak={force_peak:.3f} N')

            # force axis: more, evenly-spaced ticks
            ax_force.yaxis.set_major_locator(MaxNLocator(nbins=n_force_ticks))

            ax.set_ylabel(f'R{r}\nCp (pF)', rotation=0, ha='right', va='center')
            ax_force.set_ylabel('Force (N)', color='tab:blue')
            ax_force.tick_params(axis='y', labelcolor='tab:blue')

            lines = [l1, l2, l3]
            ax.legend(lines, [line.get_label() for line in lines], loc='upper right', fontsize=7)

        axes[-1].set_xlabel('Time (s) — press/retract removed')
        fig.suptitle(f'P{self.point} — Cp (black) & Force (blue), per round')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    df = pd.read_csv('/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv')

    analyzer = TouchSensorAnalyzer(df, point=10, force_col='load_cell_N')
    analyzer.compute_baseline_subtraction()
    analyzer.plot_no_ramps_with_force()