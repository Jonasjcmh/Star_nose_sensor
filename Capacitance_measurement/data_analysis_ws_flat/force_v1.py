import pandas as pd
import matplotlib.pyplot as plt

DEPTH_COLORS = ['#c2185b', '#7b1fa2', '#2e7d32', '#c62828', '#1565c0']  # dark pink, purple, green, red, blue


class ForceDepthComparisonAnalyzer:
    """
    Compares fz (robot's own force estimate) vs load_cell_N (ground truth)
    for a SINGLE point, across MULTIPLE depth files, as time-series line plots.
    """

    def __init__(self, depth_files: dict, point: int):
        self.point = point
        self.depths = sorted(depth_files.keys())
        self.data = {}
        for depth, path in depth_files.items():
            df = pd.read_csv(path)
            self.data[depth] = df[df['point'] == point].copy()

        all_round_sets = [set(d['round_idx'].unique()) for d in self.data.values()]
        self.rounds = sorted(set.intersection(*all_round_sets))

    def __repr__(self):
        return f"ForceDepthComparisonAnalyzer(point={self.point}, depths={self.depths})"

    def plot_force_timeseries_by_depth(self):
        """
        One subplot per round. For each depth: load_cell_N (solid line) and
        fz (dashed line) overlaid vs time, both color-coded by depth.
        """
        n = len(self.rounds)
        fig, axes = plt.subplots(n, 1, figsize=(9, 2.6 * n), sharex=True, sharey=True)
        if n == 1:
            axes = [axes]

        for i, r in enumerate(self.rounds):
            ax = axes[i]
            for depth, color in zip(self.depths, DEPTH_COLORS):
                sub = self.data[depth][self.data[depth]['round_idx'] == r].sort_values('timestamp').copy()
                sub['t0'] = sub['timestamp'] - sub['timestamp'].iloc[0]

                ax.plot(sub['t0'], sub['load_cell_N'], color=color, lw=1.2, ls='-',
                        label=f'{depth}mm load_cell')
                ax.plot(sub['t0'], sub['fz'], color=color, lw=1.2, ls='--', alpha=0.7,
                        label=f'{depth}mm fz')

            ax.set_ylabel(f'R{r}\nForce (N)', rotation=0, ha='right', va='center')
            ax.legend(loc='upper right', fontsize=6, ncol=2)
            ax.grid(alpha=0.3)

        axes[-1].set_xlabel('Time (s)')
        fig.suptitle(f'P{self.point} — load_cell_N (solid) vs fz (dashed), per round, across depths')
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    depth_files = {
        5: '/home/divuthejo/Downloads/ramp_collector_20260630_180318_5mm.csv',
        6: '/home/divuthejo/Downloads/ramp_collector_20260626_155329_6mm.csv',
        7: '/home/divuthejo/Downloads/ramp_collector_20260626_171140_7mm.csv',
        8: '/home/divuthejo/Downloads/ramp_collector_20260627_122027_8mm.csv',
        9: '/home/divuthejo/Downloads/ramp_collector_20260630_103911_9mm.csv',
    }

    fdc = ForceDepthComparisonAnalyzer(depth_files, point=10)
    fdc.plot_force_timeseries_by_depth()