import numpy as np
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

    def plot_baseline_subtracted(self, hold_window=None):
        if not self.corrected:
            self.compute_baseline_subtraction()

        fig, (ax_full, ax_zoom) = plt.subplots(1, 2, figsize=(14, 5))
        colors = plt.cm.tab10.colors

        for i, r in enumerate(self.rounds):
            sub = self.corrected[r]
            ax_full.plot(sub['t0'], sub['Cp_corrected'], color=colors[i % 10], lw=1, label=f'Round {r}')
            ax_zoom.plot(sub['t0'], sub['Cp_corrected'], color=colors[i % 10], lw=1)

        ax_full.axhline(0, color='black', lw=0.8, ls='--')
        ax_full.set_title(f'P{self.point} — full trace')
        ax_full.legend()

        # auto-zoom into the "hold" plateau, or pass explicit (tmin, tmax)
        if hold_window is None:
            hold_window = (4.5, 8.5)  # tweak based on your phase timing
        ax_zoom.set_xlim(*hold_window)
        # auto-scale y to just the visible data
        ax_zoom.set_title('Zoomed: hold plateau')
        plt.tight_layout()
        plt.show()



if __name__ == '__main__':
    df = pd.read_csv('/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv')

    analyzer = TouchSensorAnalyzer(df, point=10)
    analyzer.compute_baseline_subtraction()
    analyzer.plot_baseline_subtracted()
