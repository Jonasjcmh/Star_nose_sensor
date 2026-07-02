import pandas as pd
import matplotlib.pyplot as plt


class ForceComparisonAnalyzer:
    """
    Compares fz (robot's own force estimate) vs load_cell_N (ground truth)
    for a SINGLE point, SINGLE depth file, as time-series line plots.
    """

    def __init__(self, df: pd.DataFrame, point: int):
        self.point = point
        self.data = df[df['point'] == point].copy()
        self.rounds = sorted(self.data['round_idx'].unique())

    def __repr__(self):
        return f"ForceComparisonAnalyzer(point={self.point}, rounds={self.rounds})"

    def plot_force_timeseries(self):
        """
        One subplot per round. load_cell_N (solid, black) and fz (dashed, red)
        overlaid vs time, full trace including ramps.
        """
        n = len(self.rounds)
        fig, axes = plt.subplots(n, 1, figsize=(9, 2.6 * n), sharex=True, sharey=True)
        if n == 1:
            axes = [axes]

        for i, r in enumerate(self.rounds):
            sub = self.data[self.data['round_idx'] == r].sort_values('timestamp').copy()
            sub['t0'] = sub['timestamp'] - sub['timestamp'].iloc[0]

            ax = axes[i]
            ax.plot(sub['t0'], sub['load_cell_N'], color='black', lw=1.2, ls='-', label='load_cell_N')
            ax.plot(sub['t0'], sub['fz'], color='tab:red', lw=1.2, ls='--', alpha=0.8, label='fz')

            ax.set_ylabel(f'R{r}\nForce (N)', rotation=0, ha='right', va='center')
            ax.legend(loc='upper right', fontsize=7)
            ax.grid(alpha=0.3)

        axes[-1].set_xlabel('Time (s)')
        fig.suptitle(f'P{self.point} — load_cell_N (solid) vs fz (dashed), per round, 9mm depth')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    df = pd.read_csv('/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv')

    fc = ForceComparisonAnalyzer(df, point=10)
    fc.plot_force_timeseries()