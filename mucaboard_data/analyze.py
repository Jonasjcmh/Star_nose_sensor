"""
analyze.py — Star-Nose Sensor | Muca-Board Ramp Analysis (launcher)
===================================================================
Thin launcher that runs Integration_2/analyze_session.py's OWN dashboard on the
CSVs produced by mucaboard_ramp_collector.py. It reuses that analyzer's exact
functions (overview · per-point · hex maps · force · analog · loadcell-vs-robot),
so the analysis is identical to `main.py --analyze` — the only reason this file
exists is that analyze_session.py hard-codes its file search to Integration_2/logs,
whereas our logs live in mucaboard_data/logs.

Both LOGS_DIR and PLOTS_DIR are pointed back into this folder, so nothing is
read from or written to Integration_2.

Usage
-----
  python analyze.py                                   # newest CSV in logs/
  python analyze.py ecoflex_domes                     # partial-name match in logs/
  python analyze.py logs/ecoflex_domes_session_XX.csv # explicit path
  python analyze.py --save                            # save figures to plots/
  python analyze.py --force                           # force analysis only
  python analyze.py --loadcell                        # load cell vs robot only
"""

import os
import sys
import glob
import argparse

import numpy as np

_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
LOG_DIR      = os.path.join(_HERE, 'logs')
PLOT_DIR     = os.path.join(_HERE, 'plots')

sys.path.insert(0, _INTEGRATION)
import analyze_session as A   # noqa: E402  (reuse its dashboard functions)

# Redirect the reused module's I/O into THIS folder so it never touches
# Integration_2/logs or Integration_2/plots.
A.LOGS_DIR  = LOG_DIR
A.PLOTS_DIR = PLOT_DIR


def _resolve(arg):
    """Find the CSV to analyse within mucaboard_data/logs (or an explicit path)."""
    if arg and os.path.isfile(arg):
        return os.path.abspath(arg)
    files = sorted(glob.glob(os.path.join(LOG_DIR, '*.csv')))
    if not files:
        raise SystemExit(f'[analyze] No CSV files found in {LOG_DIR}')
    if not arg:
        return files[-1]
    matches = [f for f in files
               if os.path.basename(f) == arg or arg in os.path.basename(f)]
    return matches[-1] if matches else files[-1]


# ── Depth grouping (on top of analyze_session's press events) ─────────────────
# analyze_session.get_press_events() doesn't record the press depth. The muca
# collector writes a depth_mm column, so we tag each event with the depth held
# during its press window, then add two depth-resolved figures.

def attach_depth(df, events):
    """Tag every press event with the depth_mm active during its press window.
    Returns the sorted list of distinct depths (empty if no depth column)."""
    if 'depth_mm' not in df.columns:
        for e in events:
            e['depth'] = float('nan')
        return []
    press = df[df['ur5_pressing'] == 1]
    for e in events:
        t0 = e['start']
        t1 = t0 + e['n_frames'] * 0.05
        win = press[(press['t'] >= t0 - 1e-3) & (press['t'] <= t1 + 1e-3)]
        if len(win):
            e['depth'] = float(round(win['depth_mm'].median(), 3))
        else:
            e['depth'] = float('nan')
    return sorted({e['depth'] for e in events if e['depth'] == e['depth']})


def plot_response_vs_depth(events, depths, csv_path, save=False):
    """Mean peak target-cell sensor response (left) and mean |Fz| (right) vs
    press depth, one line per point — the core depth-sweep view."""
    import matplotlib.pyplot as plt
    if len(depths) < 2:
        print('[depth] Single depth in dataset — skipping response-vs-depth.')
        return None

    pts  = sorted({e['point'] for e in events if e['point'] > 0})
    cmap = plt.cm.tab20
    fig, (axS, axF) = plt.subplots(1, 2, figsize=(15, 6))
    for i, p in enumerate(pts):
        xs, ys, fs = [], [], []
        for d in depths:
            sel = [e for e in events if e['point'] == p and e['depth'] == d]
            if not sel:
                continue
            xs.append(d)
            ys.append(float(np.mean([e['target_peak'] for e in sel])))
            fs.append(float(np.mean([abs(e['fz_mean']) for e in sel])))
        if xs:
            c = cmap(i / max(len(pts) - 1, 1))
            axS.plot(xs, ys, '-o', color=c, lw=1.2, ms=4, label=f'P{p:02d}')
            axF.plot(xs, fs, '-o', color=c, lw=1.2, ms=4)

    axS.set_xlabel('Press depth (mm)'); axS.set_ylabel('Mean peak sensor (target cell)')
    axS.set_title('Sensor response vs depth'); axS.grid(alpha=0.3)
    axF.set_xlabel('Press depth (mm)'); axF.set_ylabel('Mean |Fz| (N)')
    axF.set_title('Force vs depth'); axF.grid(alpha=0.3)
    axS.legend(ncol=2, fontsize=7, loc='upper left')
    fig.suptitle('Depth grouping — sensor response & force vs depth, per point',
                 fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    A.savefig(fig, csv_path, 'response_vs_depth', save)
    return fig


def plot_hexmaps_by_depth(events, depths, csv_path, save=False):
    """One hex map per depth (reusing analyze_session._hex_map), coloured by the
    mean peak target response per point at that depth (shared colour scale)."""
    import matplotlib.pyplot as plt
    if not depths:
        print('[depth] No depth column — skipping per-depth hex maps.')
        return None

    arrs, vmax = {}, 1e-6
    for d in depths:
        arr = [0.0] * A.N
        cnt = [0] * A.N
        for e in events:
            if e['depth'] != d:
                continue
            ti = e['target_idx']
            if 0 <= ti < A.N:
                arr[ti] += e['target_peak']; cnt[ti] += 1
        for i in range(A.N):
            if cnt[i]:
                arr[i] /= cnt[i]
        arrs[d] = arr
        vmax = max(vmax, max(arr))

    n = len(depths)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 5), squeeze=False)
    for j, d in enumerate(depths):
        A._hex_map(axes[0][j], arrs[d], target_idx=-1,
                   title=f'{d:g} mm', vmax=vmax, unit='')
    fig.suptitle('Depth grouping — mean peak sensor per point, by depth',
                 fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    A.savefig(fig, csv_path, 'hexmaps_by_depth', save)
    return fig


def parse_args():
    p = argparse.ArgumentParser(
        description="Run Integration_2's analyze_session dashboard on mucaboard_data logs",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('file', nargs='?', default=None,
                   help='CSV path or partial name (default: newest in logs/)')
    p.add_argument('--save',     action='store_true', help='Save figures to plots/')
    p.add_argument('--force',    action='store_true', help='Force analysis only')
    p.add_argument('--loadcell', action='store_true',
                   help='FUTEK load cell vs robot force only')
    p.add_argument('--no-depth', action='store_true',
                   help='Skip the depth-grouping figures')
    return p.parse_args()


def main():
    args = parse_args()

    import matplotlib
    matplotlib.rcParams.update({
        'figure.facecolor':  'white',
        'axes.facecolor':    'white',
        'font.family':       'DejaVu Sans',
        'axes.spines.top':   False,
        'axes.spines.right': False,
    })

    path   = _resolve(args.file)
    label  = A.get_dataset_label(path)
    df     = A.load_session(path)
    events = A.get_press_events(df)

    print(f"\n[analyze] Dataset : {label}")
    print(f"[analyze] Figures → "
          f"{A.get_save_dir(path) if args.save else 'screen only'}")

    if args.force:
        A.plot_force(df, events, path, save=args.save)
    elif args.loadcell:
        A.plot_loadcell_vs_robot(df, events, path, save=args.save)
    else:
        A.plot_overview(df,   events, path, save=args.save)
        A.plot_per_point(df,  events, path, save=args.save)
        A.plot_hex_detail(df, events, path, save=args.save)
        A.plot_force(df,      events, path, save=args.save)
        A.plot_analog(df,     events, path, save=args.save)
        A.plot_loadcell_vs_robot(df, events, path, save=args.save)

        # ── Depth grouping (mucaboard extra) ──────────────────────────────
        if not args.no_depth:
            depths = attach_depth(df, events)
            print(f"[analyze] Depths  : {[f'{d:g}' for d in depths] or 'none'}")
            plot_response_vs_depth(events, depths, path, save=args.save)
            plot_hexmaps_by_depth(events, depths, path, save=args.save)

    if args.save:
        print(f"\n[analyze] Figures saved to: {A.get_save_dir(path)}")
    else:
        import matplotlib.pyplot as plt
        print('\n[analyze] Displaying — close windows to exit.')
        plt.show()
    print('[analyze] Done!')


if __name__ == '__main__':
    main()
