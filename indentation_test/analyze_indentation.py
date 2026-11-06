"""
analyze_indentation.py — plot a single-point indentation run
============================================================
Reads an indentation_<timestamp>.csv (+ its _meta.json, if present) produced by
indentation_test.py and draws the requested result curves:

  • indentation vs force      (FUTEK load cell, with UR Fz and XJC Fz overlaid)
  • indentation vs impedance  (LCR field0 — |Z| in Z-θ mode, Cp in Cp-Rp mode)
  • indentation vs phase      (LCR field1 — θ in Z-θ mode, Rp in Cp-Rp mode)
  • cycles over time          (multi-panel time series: indentation / force /
                               |Z| / phase vs elapsed time, cycle 1,2,3 … in
                               sequence with iteration boundaries marked)

The indentation-vs-* x-axis is the ACTUAL indentation (mm) from the TCP z
relative to the surface-contact height; each ramp iteration is its own trace.
Figures are saved per sample: plots/<sample-name>/ (the sample/point name from
the run metadata), so repeats of the same sample collect in one folder.

Usage
-----
  python3 analyze_indentation.py logs/indentation_<timestamp>.csv
  python3 analyze_indentation.py logs/indentation_<timestamp>.csv --no-show
"""

import os
import sys
import csv
import json
import argparse

_HERE    = os.path.dirname(os.path.abspath(__file__))
PLOT_DIR = os.path.join(_HERE, 'plots')


def _load_meta(meta_path):
    if meta_path and os.path.exists(meta_path):
        try:
            with open(meta_path) as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def _read_csv(csv_path):
    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def _f(row, key):
    try:
        v = float(row.get(key, 'nan'))
        return v
    except (TypeError, ValueError):
        return float('nan')


def plot(csv_path, meta_path=None, show=True):
    import numpy as np
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if meta_path is None:
        guess = csv_path.replace('.csv', '_meta.json')
        meta_path = guess if os.path.exists(guess) else None
    meta = _load_meta(meta_path)
    lcr_meta = meta.get('lcr', {})
    z_label  = lcr_meta.get('primary_label',   'LCR field 0')
    ph_label = lcr_meta.get('secondary_label', 'LCR field 1')
    lcr_on   = lcr_meta.get('enabled', True)

    rows = _read_csv(csv_path)
    if not rows:
        print('[plot] empty CSV — nothing to draw')
        return

    iters = sorted({int(float(r['iteration'])) for r in rows
                    if r.get('iteration') not in (None, '', '-1')})

    # Save the figures NEXT TO the dataset, so each test's CSV + meta + plots
    # live together in one folder (e.g. results/<material>/). Falls back to the
    # plots/ dir only if the CSV path has no usable directory.
    out_dir = os.path.dirname(os.path.abspath(csv_path)) or PLOT_DIR
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(csv_path))[0]

    # Group rows by iteration, keeping only the press+hold+retract (indentation ≥ ~0).
    def series(rows_i, ykey):
        x = np.array([_f(r, 'indentation_mm') for r in rows_i])
        y = np.array([_f(r, ykey) for r in rows_i])
        return x, y

    by_iter = {it: [r for r in rows if r.get('iteration') not in (None, '')
                    and int(float(r['iteration'])) == it] for it in iters}

    # Which force channels are present / non-trivial.
    has_xjc = meta.get('xjc', {}).get('enabled', True)

    # ── Figure 1: indentation vs force ─────────────────────────────────────────
    fig1, ax1 = plt.subplots(figsize=(7.5, 5.5))
    cmap = plt.get_cmap('viridis')
    for j, it in enumerate(iters):
        ri = by_iter[it]
        c  = cmap(j / max(1, len(iters) - 1))
        x, fu = series(ri, 'futek_force_N')
        ax1.plot(x, fu, '-', color=c, lw=1.4, alpha=0.9,
                 label=f'FUTEK iter{it + 1}' if it == iters[0] else None)
        if has_xjc:
            _, fz = series(ri, 'xjc_Fz')
            ax1.plot(x, fz, '--', color=c, lw=1.0, alpha=0.6,
                     label='XJC Fz' if it == iters[0] else None)
    ax1.set_xlabel('indentation (mm)')
    ax1.set_ylabel('force (N)')
    ax1.set_title(f'Indentation vs force   ({base})')
    ax1.grid(alpha=0.3)
    ax1.legend(loc='best', fontsize=8)
    p1 = os.path.join(out_dir, f"{base}_force.png")
    fig1.tight_layout(); fig1.savefig(p1, dpi=130)
    print(f'[plot] {p1}')

    saved = [p1]
    # ── Figures 2 & 3: indentation vs impedance / phase ────────────────────────
    # Count finite LCR samples so we can warn instead of drawing an empty axes.
    n_fin = sum(1 for r in rows if np.isfinite(_f(r, 'lcr_primary')))
    if lcr_on and n_fin == 0:
        print('[plot] ⚠ LCR was enabled but every lcr_primary is NaN — the meter '
              'returned no numbers. Skipping the impedance/phase plots.')

    if lcr_on and n_fin > 0:
        for ykey, ylabel, suffix in (('lcr_primary',   z_label,  'impedance'),
                                     ('lcr_secondary', ph_label, 'phase')):
            fig, ax = plt.subplots(figsize=(7.5, 5.5))
            for j, it in enumerate(iters):
                ri = by_iter[it]
                c  = cmap(j / max(1, len(iters) - 1))
                x, y = series(ri, ykey)
                # Plot every FINITE reading. The meter's lcr_ok bundles a pass/
                # fail (NG) bit that can be 0 for perfectly valid measurements,
                # so gating on it would wrongly blank the curve — mark not-ok
                # points faintly instead of dropping them.
                good = np.isfinite(x) & np.isfinite(y)
                ax.plot(x[good], y[good], '-', color=c, lw=1.4, alpha=0.9,
                        label=f'iter{it + 1}')
            ax.set_xlabel('indentation (mm)')
            ax.set_ylabel(ylabel)
            ax.set_title(f'Indentation vs {ylabel}   ({base})')
            ax.grid(alpha=0.3); ax.legend(loc='best', fontsize=8)
            p = os.path.join(out_dir, f'{base}_{suffix}.png')
            fig.tight_layout(); fig.savefig(p, dpi=130)
            print(f'[plot] {p}'); saved.append(p)

    # ── Figure 4: time series — the cycles one after another ───────────────────
    # Everything vs elapsed time, so you see cycle 1, 2, 3 … in sequence. The
    # one-time 'approach'/'idle' rows (tip at the safe height) are dropped so the
    # panels start at the first press; iteration boundaries are marked.
    cyc = [r for r in rows if r.get('phase') in ('press', 'hold', 'retract', 'recover')]
    if cyc:
        ts = np.array([_f(r, 'timestamp') for r in cyc])
        if np.isfinite(ts).sum() >= 2 and ts[np.isfinite(ts)][0] > 0:
            t = ts - np.nanmin(ts)
        else:
            t = np.arange(len(cyc), dtype=float)   # fallback: row index
        it_arr = np.array([int(float(r['iteration'])) if r.get('iteration') not in
                           (None, '', '-1') else -1 for r in cyc])

        panels = [('indentation_mm', 'indent (mm)', None)]
        panels.append(('__force__', 'force (N)', None))
        if lcr_on and n_fin > 0:
            panels.append(('lcr_primary',   z_label))
            panels.append(('lcr_secondary', ph_label))

        fig, axes = plt.subplots(len(panels), 1, figsize=(11, 2.1 * len(panels)),
                                 sharex=True)
        if len(panels) == 1:
            axes = [axes]
        # iteration-boundary markers (start time of each iteration)
        bounds = []
        for it in iters:
            idx = np.where(it_arr == it)[0]
            if len(idx):
                bounds.append((t[idx[0]], it))

        for ax, spec in zip(axes, panels):
            key = spec[0]
            if key == '__force__':
                ax.plot(t, [_f(r, 'futek_force_N') for r in cyc], '-', lw=0.8,
                        color='#1f77b4', label='FUTEK')
                if has_xjc:
                    ax.plot(t, [_f(r, 'xjc_Fz') for r in cyc], '-', lw=0.8,
                            color='#d62728', alpha=0.7, label='XJC Fz')
                ax.legend(loc='upper right', fontsize=7)
            else:
                y = np.array([_f(r, key) for r in cyc])
                ax.plot(t, y, '-', lw=0.8, color='#2ca02c')
            ax.set_ylabel(spec[1], fontsize=9)
            ax.grid(alpha=0.3)
            for tb, it in bounds:
                ax.axvline(tb, color='0.6', lw=0.7, ls='--', alpha=0.6)
        # label the iterations across the top panel
        for tb, it in bounds:
            axes[0].text(tb, axes[0].get_ylim()[1], f' c{it + 1}', fontsize=7,
                         color='0.3', va='top')
        axes[-1].set_xlabel('time (s)')
        axes[0].set_title(f'Cycles over time — {len(iters)} cycle(s)   ({base})')
        pt = os.path.join(out_dir, f'{base}_timeseries.png')
        fig.tight_layout(); fig.savefig(pt, dpi=130)
        print(f'[plot] {pt}'); saved.append(pt)

    if show:
        plt.show()
    else:
        plt.close('all')
    return saved


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', help='indentation_<timestamp>.csv')
    ap.add_argument('--meta', default=None, help='meta.json (default: alongside the CSV)')
    ap.add_argument('--no-show', action='store_true', help='save PNGs only, do not display')
    args = ap.parse_args()
    if not os.path.exists(args.csv):
        print(f'[plot] not found: {args.csv}')
        sys.exit(1)
    plot(args.csv, args.meta, show=not args.no_show)


if __name__ == '__main__':
    main()
