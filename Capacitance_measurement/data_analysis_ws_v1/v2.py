import pandas as pd
import matplotlib.pyplot as plt


class TouchSensorAnalyzer:
    KEEP_PHASES = ['locate', 'hold', 'post']  # used for baseline computation
    PHASE_COLORS = {
        'locate':  'tab:gray',
        'press':   'tab:orange',
        'hold':    'tab:green',
        'retract': 'tab:orange',
        'post':    'tab:gray',
    }

    def __init__(self, df: pd.DataFrame, point: int):
        self.point = point
        self.full_data = df[df['point'] == point].copy()                          # all phases
        self.data = self.full_data[self.full_data['phase'].isin(self.KEEP_PHASES)].copy()  # for baseline
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

    def plot_all_phases_separate(self, share_yaxis=True):
        """
        Per-round subplots, ALL phases included (locate/press/hold/retract/post),
        each shaded its own color. Red dashed line + label shows the hold-phase
        mean for EACH round.
        """
        if not self.corrected:
            self.compute_baseline_subtraction()

        n = len(self.rounds)
        fig, axes = plt.subplots(n, 1, figsize=(9, 2.4 * n), sharex=True, sharey=share_yaxis)
        if n == 1:
            axes = [axes]

        for i, r in enumerate(self.rounds):
            sub = self.full_data[self.full_data['round_idx'] == r].sort_values('timestamp').copy()
            sub['t0'] = sub['timestamp'] - sub['timestamp'].iloc[0]
            sub['Cp_corrected'] = sub['Cp_pF'] - self.baselines[r]

            ax = axes[i]

            phase_change = (sub['phase'] != sub['phase'].shift()).cumsum()
            for _, block in sub.groupby(phase_change):
                phase = block['phase'].iloc[0]
                t_start, t_end = block['t0'].iloc[0], block['t0'].iloc[-1]
                ax.axvspan(t_start, t_end, color=self.PHASE_COLORS.get(phase, 'lightgray'), alpha=0.15)

            ax.plot(sub['t0'], sub['Cp_corrected'], color='black', lw=1)

            hold_vals = sub.loc[sub['phase'] == 'hold', 'Cp_corrected']
            mean_val = hold_vals.mean()
            ax.axhline(mean_val, color='red', lw=0.8, ls='--', label=f'mean={mean_val:.4f}')
            ax.axhline(0, color='black', lw=0.5, ls=':')

            ax.set_ylabel(f'R{r}', rotation=0, ha='right', va='center')
            ax.legend(loc='upper right', fontsize=8)  # every subplot gets its own mean label

        axes[-1].set_xlabel('Time (s)')
        fig.suptitle(f'P{self.point} — all phases shaded, baseline-subtracted, per round')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    df = pd.read_csv('/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv')

    analyzer = TouchSensorAnalyzer(df, point=10)
    analyzer.compute_baseline_subtraction()
    analyzer.plot_all_phases_separate()