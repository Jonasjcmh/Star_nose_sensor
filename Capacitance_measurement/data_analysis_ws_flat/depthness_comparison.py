import pandas as pd
import matplotlib.pyplot as plt

SETTLE_WINDOW_S = 1.0  # matches capacitance_ramp_collector.SETTLE_WINDOW_S


class TouchSensorAnalyzer:
    KEEP_PHASES = ['locate', 'hold', 'post']  # used for baseline computation
    PHASE_COLORS = {
        'locate':  'tab:gray',
        'press':   'tab:orange',
        'hold':    'tab:green',
        'retract': 'tab:orange',
        'post':    'tab:gray',
    }

    def __init__(self, df: pd.DataFrame, point: int, force_col: str = 'load_cell_N'):
        self.point = point
        self.force_col = force_col
        self.full_data = df[df['point'] == point].copy()                                    # ALL phases
        self.data = self.full_data[self.full_data['phase'].isin(self.KEEP_PHASES)].copy()    # for baseline calc
        self.rounds = sorted(self.data['round_idx'].unique())
        self.baselines = {}
        self.corrected = {}       # locate/hold/post only, baseline-subtracted
        self.corrected_full = {}  # ALL phases (incl. ramps), baseline-subtracted

    def __repr__(self):
        return f"TouchSensorAnalyzer(point={self.point}, rounds={self.rounds})"

    @staticmethod
    def _tail_mean(sub, phase, col, window_s=SETTLE_WINDOW_S):
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

            # same baseline applied to the FULL trace, including ramps
            full_sub = self.full_data[self.full_data['round_idx'] == r].sort_values('timestamp').copy()
            full_sub['t0'] = full_sub['timestamp'] - full_sub['timestamp'].iloc[0]
            full_sub['Cp_corrected'] = full_sub['Cp_pF'] - baseline
            self.corrected_full[r] = full_sub

        return self.baselines


class DepthComparisonAnalyzer:
    """
    Loads one TouchSensorAnalyzer per depth (each from its own CSV) for a
    single point, and compares baseline-subtracted Cp across depths, round by
    round — FULL trace including ramps, all phases shaded. Legend shows Co
    and settled ΔCp per depth.
    """

    def __init__(self, depth_files: dict, point: int, force_col: str = 'load_cell_N'):
        self.point = point
        self.depths = sorted(depth_files.keys())
        self.analyzers = {}
        for depth, path in depth_files.items():
            df = pd.read_csv(path)
            analyzer = TouchSensorAnalyzer(df, point=point, force_col=force_col)
            analyzer.compute_baseline_subtraction()
            self.analyzers[depth] = analyzer

        all_round_sets = [set(a.rounds) for a in self.analyzers.values()]
        self.rounds = sorted(set.intersection(*all_round_sets))

    def plot_depth_comparison(self, share_yaxis=True):
        """
        One subplot per round. Full trace (locate→press→hold→retract→post,
        ramps untouched) overlaid for every depth, color-coded, with all
        five phase regions shaded using the reference depth's timing.
        Legend per subplot shows Co and settled ΔCp per depth.
        """
        n = len(self.rounds)
        fig, axes = plt.subplots(n, 1, figsize=(9, 2.6 * n), sharex=True, sharey=share_yaxis)
        if n == 1:
            axes = [axes]

        depth_colors = ['#c2185b', '#7b1fa2', '#2e7d32', '#c62828', '#1565c0']  # dark pink, purple, green, red, blue
        colors = [depth_colors[i % len(depth_colors)] for i in range(len(self.depths))]

        for i, r in enumerate(self.rounds):
            ax = axes[i]

            ref_sub = self.analyzers[self.depths[0]].corrected_full[r]
            phase_change = (ref_sub['phase'] != ref_sub['phase'].shift()).cumsum()
            for _, block in ref_sub.groupby(phase_change):
                phase = block['phase'].iloc[0]
                t_start, t_end = block['t0'].iloc[0], block['t0'].iloc[-1]
                ax.axvspan(t_start, t_end,
                           color=TouchSensorAnalyzer.PHASE_COLORS.get(phase, 'lightgray'), alpha=0.15)

            for d, color in zip(self.depths, colors):
                sub = self.analyzers[d].corrected_full[r]

                co_val = self.analyzers[d].baselines[r]
                hold_sub = self.analyzers[d].corrected[r]
                cp_hold_settled = TouchSensorAnalyzer._tail_mean(hold_sub, 'hold', 'Cp_pF')
                dcp_settled = cp_hold_settled - co_val

                ax.plot(sub['t0'], sub['Cp_corrected'], color=color, lw=1,
                        label=f'{d}mm: Co={co_val:.4f}, \u0394Cp={dcp_settled:.4f} pF')

            ax.axhline(0, color='black', lw=0.5, ls=':')
            ax.set_ylabel(f'R{r}\nCp (pF)', rotation=0, ha='right', va='center')
            ax.legend(loc='upper right', fontsize=6, ncol=2)

        axes[-1].set_xlabel('Time (s)')
        fig.suptitle(f'P{self.point} — Cp corrected across depths, per round (full trace)')
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

    comp = DepthComparisonAnalyzer(depth_files, point=10)
    comp.plot_depth_comparison()