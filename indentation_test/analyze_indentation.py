"""
analyze_indentation.py — plot a single-point indentation run
============================================================
Reads an indentation_<timestamp>.csv (+ its _meta.json, if present) produced by
indentation_test.py and draws the requested result curves:

  • indentation vs force      (FUTEK load cell, with UR Fz and XJC Fz overlaid)
  • indentation vs impedance  (LCR field0 — |Z| in Z-θ mode, Cp in Cp-Rp mode)
  • indentation vs phase      (LCR field1 — θ in Z-θ mode, Rp in Cp-Rp mode)
  • impedance / phase vs force (force = XJC |Fz|, since the FUTEK saturates)
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

# ── SDU (University of Southern Denmark) brand palette ─────────────────────────
# Best-effort hex values from the SDU visual identity — edit here if your design
# guide differs; everything below uses these constants.
SDU_RED    = '#B5123E'   # primary SDU bordeaux-red
SDU_BLUE   = '#114B5F'   # deep blue (secondary)
SDU_GREEN  = '#0E7C61'   # green accent
SDU_SAND   = '#E8A33D'   # sand/orange accent
SDU_PURPLE = '#5A2A82'   # purple accent
SDU_GREY   = '#6E6E6E'   # neutral grey (spines / ticks / labels)
SDU_GRID   = '#E7E2DD'   # very light warm grey — the replacement gridline colour
SDU_SEQ    = ['#EBC4CE', SDU_RED, '#4A0A1C']   # light tint → SDU red → dark (per-cycle ramp)
SDU_CYCLE  = [SDU_RED, SDU_BLUE, SDU_GREEN, SDU_SAND, SDU_PURPLE, SDU_GREY]


def _movavg(a, win):
    """Centred moving average over `win` samples (edges padded), NaN-aware.
    Returns the array unchanged if win <= 1. Smooths the robot's µm position
    jitter and the LCR's ~11 Hz staircase without shifting the curve."""
    import numpy as np
    a = np.asarray(a, dtype=float)
    win = int(win)
    if win <= 1 or a.size < 2:
        return a
    win = min(win, a.size)
    # NaN-aware running mean: sum valid samples / count of valid samples
    v = np.isfinite(a)
    af = np.where(v, a, 0.0)
    k = np.ones(win)
    num = np.convolve(af, k, mode='same')
    den = np.convolve(v.astype(float), k, mode='same')
    with np.errstate(invalid='ignore', divide='ignore'):
        out = num / den
    return out


def _style(ax):
    """Apply the SDU look: drop the old crosshatch grid for a subtle light
    horizontal grid, hide the top/right spines, and tint spines/ticks grey."""
    ax.set_axisbelow(True)
    ax.grid(False)
    ax.grid(True, axis='y', color=SDU_GRID, linewidth=0.8, zorder=0)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(SDU_GREY)
    ax.tick_params(colors=SDU_GREY, labelcolor='#333333')
    ax.title.set_color(SDU_RED)


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


def plot(csv_path, meta_path=None, show=True, smooth=0, lcr_smooth=None):
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
    # Smoothed figures get a _sm<N> suffix so they sit alongside the raw ones.
    # Force/indentation curves follow `smooth` (0 = raw by default). The LCR is
    # updated at only ~10 Hz so its traces look stepped — smooth those BY DEFAULT
    # with a window auto-sized to the staircase (override/disable with lcr_smooth).
    smooth = int(smooth or 0)
    tag = f'_sm{smooth}' if smooth > 1 else ''
    lcr_vals = np.array([_f(r, 'lcr_primary') for r in rows])
    if lcr_smooth is None:
        fin = lcr_vals[np.isfinite(lcr_vals)]
        step = 1
        if fin.size > 10:
            ch = np.where(np.diff(fin) != 0)[0]
            if ch.size > 2:
                step = int(np.median(np.diff(ch)))
        lcr_win = int(np.clip(3 * step, 51, 601))
        if smooth > 1:
            lcr_win = smooth                 # an explicit global --smooth wins
    else:
        lcr_win = int(lcr_smooth)
    sm     = (lambda a: _movavg(a, smooth))
    sm_lcr = (lambda a: _movavg(a, lcr_win))
    if lcr_win > 1:
        print(f'[plot] LCR traces smoothed with a {lcr_win}-sample window '
              f'(~{lcr_win} ms) — set --lcr-smooth 0 for raw')

    # Group rows by iteration, keeping only the press+hold+retract (indentation ≥ ~0).
    def series(rows_i, ykey):
        x = np.array([_f(r, 'indentation_mm') for r in rows_i])
        y = np.array([_f(r, ykey) for r in rows_i])
        return x, y

    by_iter = {it: [r for r in rows if r.get('iteration') not in (None, '')
                    and int(float(r['iteration'])) == it] for it in iters}

    # Which force channels are present / non-trivial.
    has_xjc = meta.get('xjc', {}).get('enabled', True)

    # Per-cycle colour ramp in SDU red (light tint → SDU red → dark).
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list('sdu_seq', SDU_SEQ)

    # ── Figure 1: indentation vs force ─────────────────────────────────────────
    fig1, ax1 = plt.subplots(figsize=(7.5, 5.5))
    for j, it in enumerate(iters):
        ri = by_iter[it]
        c  = cmap(j / max(1, len(iters) - 1))
        x, fu = series(ri, 'futek_force_N')
        xs = sm(x)
        ax1.plot(xs, sm(fu), '-', color=c, lw=1.4, alpha=0.9,
                 label=f'FUTEK iter{it + 1}' if it == iters[0] else None)
        if has_xjc:
            _, fz = series(ri, 'xjc_Fz')
            ax1.plot(xs, sm(fz), '--', color=c, lw=1.0, alpha=0.6,
                     label='XJC Fz' if it == iters[0] else None)
    ax1.set_xlabel('indentation (mm)')
    ax1.set_ylabel('force (N)')
    ax1.set_title(f'Indentation vs force   ({base}){"  [smoothed]" if tag else ""}')
    _style(ax1)
    ax1.legend(loc='best', fontsize=8, frameon=False)
    p1 = os.path.join(out_dir, f"{base}_force{tag}.png")
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
                ax.plot(sm_lcr(x[good]), sm_lcr(y[good]), '-', color=c, lw=1.4,
                        alpha=0.9, label=f'iter{it + 1}')
            ax.set_xlabel('indentation (mm)')
            ax.set_ylabel(ylabel)
            ax.set_title(f'Indentation vs {ylabel}   ({base}){"  [smoothed]" if tag else ""}')
            _style(ax); ax.legend(loc='best', fontsize=8, frameon=False)
            p = os.path.join(out_dir, f'{base}_{suffix}{tag}.png')
            fig.tight_layout(); fig.savefig(p, dpi=130)
            print(f'[plot] {p}'); saved.append(p)

    # ── Figures: impedance / phase vs FORCE (force from the XJC) ────────────────
    # The FUTEK saturates (~19 N), so the force axis uses the XJC's normal force
    # |Fz| (compression magnitude), which stays valid to its ±50 N range.
    has_xjc_fin = any(np.isfinite(_f(r, 'xjc_Fz')) for r in rows)
    if lcr_on and n_fin > 0 and has_xjc_fin:
        for ykey, ylabel, suffix in (('lcr_primary',   z_label,  'impedance_vs_force'),
                                     ('lcr_secondary', ph_label, 'phase_vs_force')):
            fig, ax = plt.subplots(figsize=(7.5, 5.5))
            for j, it in enumerate(iters):
                ri = by_iter[it]
                c  = cmap(j / max(1, len(iters) - 1))
                fx = np.array([abs(_f(r, 'xjc_Fz')) for r in ri])   # |Fz| (N)
                yy = np.array([_f(r, ykey) for r in ri])
                good = np.isfinite(fx) & np.isfinite(yy)
                ax.plot(sm_lcr(fx[good]), sm_lcr(yy[good]), '-', color=c, lw=1.2,
                        alpha=0.9, label=f'iter{it + 1}')
            ax.set_xlabel('force  |XJC Fz|  (N)')
            ax.set_ylabel(ylabel)
            ax.set_title(f'{ylabel} vs force (XJC)   ({base}){"  [smoothed]" if tag else ""}')
            _style(ax); ax.legend(loc='best', fontsize=8, frameon=False)
            p = os.path.join(out_dir, f'{base}_{suffix}{tag}.png')
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

        panel_color = {'indentation_mm': SDU_BLUE, 'lcr_primary': SDU_GREEN,
                       'lcr_secondary': SDU_PURPLE}
        for ax, spec in zip(axes, panels):
            key = spec[0]
            if key == '__force__':
                ax.plot(t, sm(np.array([_f(r, 'futek_force_N') for r in cyc])), '-',
                        lw=0.8, color=SDU_RED, label='FUTEK')
                if has_xjc:
                    ax.plot(t, sm(np.array([_f(r, 'xjc_Fz') for r in cyc])), '-',
                            lw=0.8, color=SDU_BLUE, alpha=0.8, label='XJC Fz')
                ax.legend(loc='upper right', fontsize=7, frameon=False)
            else:
                # LCR panels (lcr_*) get the default LCR smoothing; indentation
                # follows the global --smooth.
                raw = np.array([_f(r, key) for r in cyc])
                y = sm_lcr(raw) if key.startswith('lcr_') else sm(raw)
                ax.plot(t, y, '-', lw=0.8, color=panel_color.get(key, SDU_RED))
            ax.set_ylabel(spec[1], fontsize=9)
            _style(ax)
            for tb, it in bounds:
                ax.axvline(tb, color=SDU_GREY, lw=0.7, ls='--', alpha=0.5)
        # label the iterations across the top panel
        for tb, it in bounds:
            axes[0].text(tb, axes[0].get_ylim()[1], f' c{it + 1}', fontsize=7,
                         color=SDU_GREY, va='top')
        axes[-1].set_xlabel('time (s)')
        axes[0].set_title(f'Cycles over time — {len(iters)} cycle(s)   ({base})'
                          f'{"  [smoothed]" if tag else ""}')
        pt = os.path.join(out_dir, f'{base}_timeseries{tag}.png')
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
    ap.add_argument('--smooth', type=int, default=0, metavar='N',
                    help='Moving-average window in samples for the FORCE/indentation '
                         'curves (0=off). Try ~101-301 at 1 kHz. Smoothed figures get '
                         'a _sm<N> suffix.')
    ap.add_argument('--lcr-smooth', type=int, default=None, metavar='N',
                    help='Moving-average window for the LCR traces (default: auto, '
                         'sized to the ~10 Hz staircase; use 0 for raw).')
    args = ap.parse_args()
    if not os.path.exists(args.csv):
        print(f'[plot] not found: {args.csv}')
        sys.exit(1)
    plot(args.csv, args.meta, show=not args.no_show,
         smooth=args.smooth, lcr_smooth=args.lcr_smooth)


if __name__ == '__main__':
    main()
