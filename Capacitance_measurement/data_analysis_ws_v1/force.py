import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

DEPTH_COLORS = ['#c2185b', '#7b1fa2', '#2e7d32', '#c62828', '#1565c0']  # dark pink, purple, green, red, blue


class ForceDepthComparisonAnalyzer:
    """
    Compares fz (robot's own force estimate) vs load_cell_N (ground truth)
    for a SINGLE point, across MULTIPLE depth files. Lets you see whether
    the fz/load_cell relationship is consistent across depths, or whether
    it shifts as applied force increases with depth.
    """

    def __init__(self, depth_files: dict, point: int):
        """
        depth_files: dict mapping depth label -> csv path, e.g.
            {5: '...5mm.csv', 6: '...6mm.csv', 7: '...7mm.csv',
             8: '...8mm.csv', 9: '...9mm.csv'}
        """
        self.point = point
        self.depths = sorted(depth_files.keys())
        self.data = {}  # depth -> dataframe filtered to this point
        for depth, path in depth_files.items():
            df = pd.read_csv(path)
            self.data[depth] = df[df['point'] == point].copy()

    def __repr__(self):
        return f"ForceDepthComparisonAnalyzer(point={self.point}, depths={self.depths})"

    def _combined(self, depth, phase_filter=None):
        d = self.data[depth]
        if phase_filter is not None:
            d = d[d['phase'].isin(phase_filter)]
        return d.dropna(subset=['fz', 'load_cell_N'])

    def plot_scatter_by_depth(self, phase_filter=('hold',)):
        """
        Scatter of fz vs load_cell_N, one color per depth, with a per-depth
        linear fit AND an overall pooled fit for reference.
        """
        fig, ax = plt.subplots(figsize=(8, 7))
        all_x, all_y = [], []

        for depth, color in zip(self.depths, DEPTH_COLORS):
            d = self._combined(depth, phase_filter)
            if d.empty:
                continue
            x = d['load_cell_N'].values
            y = d['fz'].values
            all_x.append(x); all_y.append(y)

            ax.scatter(x, y, s=6, alpha=0.4, color=color, label=f'{depth}mm (n={len(x)})')

            if len(x) >= 3:
                slope, intercept = np.polyfit(x, y, 1)
                r_val = np.corrcoef(x, y)[0, 1]
                xs = np.linspace(x.min(), x.max(), 50)
                ax.plot(xs, slope * xs + intercept, color=color, lw=1.5, ls='--')
                print(f'{depth}mm: fz = {slope:.3f}*load_cell + {intercept:.3f}, r={r_val:.3f}, n={len(x)}')

        # pooled fit across all depths
        all_x = np.concatenate(all_x)
        all_y = np.concatenate(all_y)
        slope, intercept = np.polyfit(all_x, all_y, 1)
        r_val = np.corrcoef(all_x, all_y)[0, 1]
        xs = np.linspace(all_x.min(), all_x.max(), 100)
        ax.plot(xs, slope * xs + intercept, color='black', lw=2,
                label=f'pooled fit: fz={slope:.3f}·load_cell+{intercept:.3f}, r={r_val:.3f}')
        ax.plot(xs, xs, color='gray', lw=1, ls=':', label='y = x (perfect agreement)')

        ax.set_xlabel('load_cell_N (ground truth, N)')
        ax.set_ylabel('fz (robot estimate, N)')
        ax.set_title(f'P{self.point} — fz vs load_cell_N across depths ({"/".join(phase_filter)} phase)')
        ax.legend(fontsize=7, loc='upper left')
        ax.grid(alpha=0.3)
        plt.tight_layout()
        plt.show()

    def plot_fit_params_vs_depth(self, phase_filter=('hold',)):
        """
        Per-depth fit slope, intercept, and r, plotted against depth —
        answers directly: does the fz/load_cell relationship DRIFT as
        depth (force magnitude) increases?
        """
        depths_list, slopes, intercepts, rs = [], [], [], []
        for depth in self.depths:
            d = self._combined(depth, phase_filter)
            if len(d) < 3:
                continue
            x = d['load_cell_N'].values
            y = d['fz'].values
            slope, intercept = np.polyfit(x, y, 1)
            r_val = np.corrcoef(x, y)[0, 1]
            depths_list.append(depth); slopes.append(slope); intercepts.append(intercept); rs.append(r_val)

        fig, axes = plt.subplots(1, 3, figsize=(13, 4))
        axes[0].plot(depths_list, slopes, 'o-', color='#1565c0')
        axes[0].axhline(1.0, color='gray', ls=':', lw=1)
        axes[0].set_title('Fit slope vs depth'); axes[0].set_xlabel('Depth (mm)'); axes[0].set_ylabel('slope')

        axes[1].plot(depths_list, intercepts, 'o-', color='#c62828')
        axes[1].axhline(0.0, color='gray', ls=':', lw=1)
        axes[1].set_title('Fit intercept vs depth'); axes[1].set_xlabel('Depth (mm)'); axes[1].set_ylabel('intercept (N)')

        axes[2].plot(depths_list, rs, 'o-', color='#2e7d32')
        axes[2].set_ylim(0, 1.05)
        axes[2].set_title('Correlation (r) vs depth'); axes[2].set_xlabel('Depth (mm)'); axes[2].set_ylabel('r')

        for ax in axes:
            ax.grid(alpha=0.3)
        fig.suptitle(f'P{self.point} — fz vs load_cell_N fit parameters across depths')
        plt.tight_layout()
        plt.show()

    def plot_bland_altman_by_depth(self, phase_filter=('hold',)):
        """
        Bland-Altman (mean vs difference), color-coded by depth — shows
        whether bias/agreement changes as force magnitude (depth) increases.
        """
        fig, ax = plt.subplots(figsize=(9, 5))
        for depth, color in zip(self.depths, DEPTH_COLORS):
            d = self._combined(depth, phase_filter)
            if d.empty:
                continue
            mean_force = (d['fz'] + d['load_cell_N']) / 2
            diff = d['fz'] - d['load_cell_N']
            ax.scatter(mean_force, diff, s=6, alpha=0.4, color=color, label=f'{depth}mm')

        ax.axhline(0, color='black', lw=0.8, ls=':')
        ax.set_xlabel('Mean of fz & load_cell_N (N)')
        ax.set_ylabel('fz − load_cell_N (N)')
        ax.set_title(f'P{self.point} — Bland-Altman across depths ({"/".join(phase_filter)})')
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
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
    fdc.plot_scatter_by_depth(phase_filter=('hold',))
    fdc.plot_fit_params_vs_depth(phase_filter=('hold',))
    fdc.plot_bland_altman_by_depth(phase_filter=('hold',))