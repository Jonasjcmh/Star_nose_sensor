import pandas as pd
import matplotlib.pyplot as plt


class TouchSensorAnalyzer:
    """
    Analyzes ramp-collector capacitance data for a single sensor point.
    Handles baseline subtraction, delta-C, and repeatability across rounds.
    """

    KEEP_PHASES = ['locate', 'hold', 'post']  # press/retract excluded from analysis

    def __init__(self, df: pd.DataFrame, point: int):
        self.point = point
        # keep only rows for this point, and only the phases we care about
        self.data = df[(df['point'] == point) & (df['phase'].isin(self.KEEP_PHASES))].copy()
        self.rounds = sorted(self.data['round_idx'].unique())
        self.baselines = {}      # round -> baseline Cp value (pF)
        self.corrected = {}      # round -> dataframe with Cp_corrected column added

    def __repr__(self):
        return f"TouchSensorAnalyzer(point={self.point}, rounds={self.rounds})"

    def compute_baseline_subtraction(self):
        """
        For each round:
          1. take mean Cp during 'locate'
          2. take mean Cp during 'post'
          3. baseline = average of those two means
          4. subtract baseline from every sample in that round -> Cp_corrected
        Fills self.baselines and self.corrected.
        """
        for r in self.rounds:
            sub = self.data[self.data['round_idx'] == r].sort_values('timestamp').copy()

            base_locate = sub.loc[sub['phase'] == 'locate', 'Cp_pF'].mean()
            base_post = sub.loc[sub['phase'] == 'post', 'Cp_pF'].mean()
            baseline = (base_locate + base_post) / 2

            sub['Cp_corrected'] = sub['Cp_pF'] - baseline
            sub['t0'] = sub['timestamp'] - sub['timestamp'].iloc[0]  # reset clock to round start

            self.baselines[r] = baseline
            self.corrected[r] = sub

        return self.baselines

    def plot_baseline_subtracted(self):
        """Overlay Cp_corrected vs time for all rounds on one axis."""
        if not self.corrected:
            self.compute_baseline_subtraction()

        fig, ax = plt.subplots(figsize=(9, 5))
        colors = plt.cm.tab10.colors

        for i, r in enumerate(self.rounds):
            sub = self.corrected[r]
            ax.plot(sub['t0'], sub['Cp_corrected'], color=colors[i % 10], lw=1, label=f'Round {r}')

        ax.axhline(0, color='black', lw=0.8, ls='--')
        ax.set_xlabel('Time (s) — press/retract removed')
        ax.set_ylabel('Cp − baseline (pF)')
        ax.set_title(f'P{self.point} — Baseline-subtracted Cp, all rounds overlaid')
        ax.legend()
        plt.tight_layout()
        plt.show()

    def plot_baseline_subtracted_separate(self):
        """Plot each round in its own subplot (not overlaid), with per-round y-scaling."""
        if not self.corrected:
            self.compute_baseline_subtraction()

        n = len(self.rounds)
        fig, axes = plt.subplots(n, 1, figsize=(9, 3 * n))

        # in case there's only 1 round, axes won't be a list
        if n == 1:
            axes = [axes]

        for i, r in enumerate(self.rounds):
            sub = self.corrected[r]
            ax = axes[i]
            ax.plot(sub['t0'], sub['Cp_corrected'], color='tab:blue', lw=1)
            ax.axhline(0, color='black', lw=0.8, ls='--')

            # scale y-axis to this round's own data, with a small margin
            y_min, y_max = sub['Cp_corrected'].min(), sub['Cp_corrected'].max()
            margin = (y_max - y_min) * 0.1
            ax.set_ylim(y_min - margin, y_max + margin)

            ax.set_title(f'Round {r}')
            ax.set_ylabel('Cp − baseline (pF)')

        axes[-1].set_xlabel('Time (s) — press/retract removed')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    df = pd.read_csv('/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv')

    analyzer = TouchSensorAnalyzer(df, point=10)
    analyzer.compute_baseline_subtraction()
    analyzer.plot_baseline_subtracted_separate()