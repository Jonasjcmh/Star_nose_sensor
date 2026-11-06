"""
analyze_indentation.py — plot a single-point indentation run
============================================================
Reads an indentation_<timestamp>.csv (+ its _meta.json, if present) produced by
indentation_test.py and draws the requested result curves:

  • indentation vs force      (FUTEK load cell, with UR Fz and XJC Fz overlaid)
  • indentation vs impedance  (LCR field0 — |Z| in Z-θ mode, Cp in Cp-Rp mode)
  • indentation vs phase      (LCR field1 — θ in Z-θ mode, Rp in Cp-Rp mode)

The x-axis is the ACTUAL indentation (mm), measured from the TCP z relative to
the surface-contact height. Each ramp iteration is drawn as its own faint trace;
the press phase (0→max) is emphasised. Figures are saved to plots/.

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
    os.makedirs(PLOT_DIR, exist_ok=True)
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
    p1 = os.path.join(PLOT_DIR, f'{base}_force.png')
    fig1.tight_layout(); fig1.savefig(p1, dpi=130)
    print(f'[plot] {p1}')

    saved = [p1]
    # ── Figures 2 & 3: indentation vs impedance / phase ────────────────────────
    if lcr_on:
        fig2, ax2 = plt.subplots(figsize=(7.5, 5.5))
        for j, it in enumerate(iters):
            ri = by_iter[it]
            c  = cmap(j / max(1, len(iters) - 1))
            x, z = series(ri, 'lcr_primary')
            # only plot rows where the LCR reading was valid
            ok = np.array([_f(r, 'lcr_ok') for r in ri]) > 0.5
            ax2.plot(x[ok], z[ok], '-', color=c, lw=1.4, alpha=0.9,
                     label=f'iter{it + 1}')
        ax2.set_xlabel('indentation (mm)')
        ax2.set_ylabel(z_label)
        ax2.set_title(f'Indentation vs {z_label}   ({base})')
        ax2.grid(alpha=0.3); ax2.legend(loc='best', fontsize=8)
        p2 = os.path.join(PLOT_DIR, f'{base}_impedance.png')
        fig2.tight_layout(); fig2.savefig(p2, dpi=130)
        print(f'[plot] {p2}'); saved.append(p2)

        fig3, ax3 = plt.subplots(figsize=(7.5, 5.5))
        for j, it in enumerate(iters):
            ri = by_iter[it]
            c  = cmap(j / max(1, len(iters) - 1))
            x, ph = series(ri, 'lcr_secondary')
            ok = np.array([_f(r, 'lcr_ok') for r in ri]) > 0.5
            ax3.plot(x[ok], ph[ok], '-', color=c, lw=1.4, alpha=0.9,
                     label=f'iter{it + 1}')
        ax3.set_xlabel('indentation (mm)')
        ax3.set_ylabel(ph_label)
        ax3.set_title(f'Indentation vs {ph_label}   ({base})')
        ax3.grid(alpha=0.3); ax3.legend(loc='best', fontsize=8)
        p3 = os.path.join(PLOT_DIR, f'{base}_phase.png')
        fig3.tight_layout(); fig3.savefig(p3, dpi=130)
        print(f'[plot] {p3}'); saved.append(p3)

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
