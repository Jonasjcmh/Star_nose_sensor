import pandas as pd
import matplotlib.pyplot as plt


class TouchSensorAnalyzer:
    KEEP_PHASES = ['locate', 'hold', 'post']  # press/retract excluded from analysis
    PHASE_COLORS = {
        'locate': 'tab:gray',
        'hold':   'tab:green',
        'post':   'tab:gray',
    }

    def __init__(self, df: pd.DataFrame, point: int):
        self.point = point
        self.data = df[(df['point'] == point) & (df['phase'].isin(self.KEEP_PHASES))].copy()
        self.rounds = sorted(self.data['round_idx'].unique())
        self.baselines = {}
        self.corrected = {}

    def __repr__(self):
        return f"TouchSensorAnalyzer(point={self.point}, rounds={self.rounds})"

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

    def plot_no_ramps_separate(self, share_yaxis=True):
        """
        Per-round subplots, ramps (press/retract) excluded.
        locate/hold/post phases shaded; legend on each subplot shows
        that round's baseline (Co) and hold-phase mean of Cp_corrected.
        """
        if not self.corrected:
            self.compute_baseline_subtraction()

        n = len(self.rounds)
        fig, axes = plt.subplots(n, 1, figsize=(9, 2.4 * n), sharex=True, sharey=share_yaxis)
        if n == 1:
            axes = [axes]

        for i, r in enumerate(self.rounds):
            sub = self.corrected[r]
            ax = axes[i]

            # shade each contiguous phase block (locate/hold/post)
            phase_change = (sub['phase'] != sub['phase'].shift()).cumsum()
            for _, block in sub.groupby(phase_change):
                phase = block['phase'].iloc[0]
                t_start, t_end = block['t0'].iloc[0], block['t0'].iloc[-1]
                ax.axvspan(t_start, t_end, color=self.PHASE_COLORS.get(phase, 'lightgray'), alpha=0.15)

            hold_vals = sub.loc[sub['phase'] == 'hold', 'Cp_corrected']
            mean_val = hold_vals.mean()
            co_val = self.baselines[r]

            ax.plot(sub['t0'], sub['Cp_corrected'], color='black', lw=1,
                    label=f'Co={co_val:.4f} pF')
            ax.axhline(mean_val, color='red', lw=0.8, ls='--',
                       label=f'mean={mean_val:.4f} pF')
            ax.axhline(0, color='black', lw=0.5, ls=':')

            ax.set_ylabel(f'R{r}', rotation=0, ha='right', va='center')
            ax.legend(loc='upper right', fontsize=8)

        axes[-1].set_xlabel('Time (s) — press/retract removed')
        fig.suptitle(f'P{self.point} — locate/hold/post, baseline-subtracted, per round')
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    df = pd.read_csv('/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv')

    analyzer = TouchSensorAnalyzer(df, point=10)
    analyzer.compute_baseline_subtraction()
    analyzer.plot_no_ramps_separate()