#!/usr/bin/env python3
"""
analyze_actuator_session.py — plots + summary for an actuator_lcr_logger run
===========================================================================

Called automatically at the end of every actuator_lcr_logger.py run, or by
hand on any session folder:

    python analyze_actuator_session.py logs/first_test_out1_20261110_150638
    python analyze_actuator_session.py            # newest folder in logs/
    python analyze_actuator_session.py <dir> --show

Writes into the session folder:
    timeseries.png      pressure (set-point + measured), LCR primary (Z or Cp)
                        and LCR secondary (theta or D) vs time
    lcr_vs_pressure.png LCR primary / secondary vs measured pressure,
                        inflation and deflation coloured separately
                        (hysteresis), hold averages as points with error bars
    summary.csv         one row per hold: mean/std pressure and LCR values,
                        change vs the 0-mbar baseline
"""

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# SDU-inspired palette (shared with actuator_gui.py).  SDU's design guide
# lists dark green #789D4A (Pantone 576 C) and light green #AEB862
# (Pantone 2302 C); yellow and brown here are close matches — replace them
# with the official values from designguide.sdu.dk if needed.
SDU_GREEN = '#789D4A'
SDU_LIGHT_GREEN = '#AEB862'
SDU_YELLOW = '#F0B323'
SDU_YELLOW_DEEP = '#D9940B'     # darker yellow: readable lines on light paper
SDU_BROWN = '#5B3B24'
SDU_BEIGE = '#EFE9DF'

C_INFLATE = SDU_GREEN
C_DEFLATE = SDU_YELLOW_DEEP
C_HOLD = '#B3A796'
C_SETPOINT = '#9C8470'
C_PRESSURE = SDU_BROWN
C_TEXT = '#1D1D1B'
C_TEXT2 = '#6B5A4C'
C_GRID = '#E6DED2'              # axis lines + shading (no grid lines are drawn)
C_SURFACE = '#FBF8F3'

HOLD_PHASES = ('hold', 'hold_high', 'hold_low', 'rest')
SETTLE_FRACTION = 0.4      # ignore the first 40 % of each hold (transient)

PRETTY = {'theta': 'θ', 'Cp': 'Cp', 'Cs': 'Cs', 'Z': '|Z|', 'D': 'D', 'Q': 'Q',
          'Rp': 'Rp', 'Rs': 'Rs', 'Lp': 'Lp', 'Ls': 'Ls', 'X': 'X', 'R': 'R',
          'DCR': 'DCR'}


# ─────────────────────────────────────────────────────────────────────────
#  Loading
# ─────────────────────────────────────────────────────────────────────────

def load_session(folder):
    with open(os.path.join(folder, 'data.csv'), newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f'{folder}/data.csv is empty')
    meta = {}
    try:
        with open(os.path.join(folder, 'meta.json')) as fh:
            meta = json.load(fh)
    except (OSError, ValueError):
        pass
    cols = list(rows[0].keys())

    def num(c):
        out = np.empty(len(rows))
        for i, r in enumerate(rows):
            try:
                out[i] = float(r[c])
            except (TypeError, ValueError):
                out[i] = np.nan
        return out

    d = {c: num(c) for c in ('t_s', 'setpoint_mbar', 'pressure_mbar', 'step',
                             'cycle', 'rep', 'segment')}
    d['phase'] = np.array([r['phase'] for r in rows])
    d['label'] = np.array([r['label'] for r in rows])
    d['valves'] = np.array([r['valves_open'] for r in rows])
    lcr_cols = [c for c in cols if c.startswith('lcr_')
                and c not in ('lcr_ok', 'lcr_n', 'lcr_age_s')]
    d['lcr_cols'] = lcr_cols
    for c in lcr_cols + (['lcr_ok', 'lcr_n'] if lcr_cols else []):
        d[c] = num(c)
    return d, meta


def quantity(col):
    """'lcr_Z_Ohm' -> ('Z', 'Ohm');  'lcr_D' -> ('D', '')."""
    parts = col[4:].split('_')
    return parts[0], ('_'.join(parts[1:]) if len(parts) > 1 else '')


def display_scale(name, unit, values):
    """Pick a readable unit: F -> pF/nF, H -> uH, Ohm -> Ohm/kOhm/MOhm."""
    v = values[np.isfinite(values)]
    mag = np.median(np.abs(v)) if v.size else 1.0
    if unit == 'F':
        return (1e12, 'pF') if mag < 1e-9 else (1e9, 'nF')
    if unit == 'H':
        return (1e6, 'µH') if mag < 1e-3 else (1e3, 'mH')
    if unit == 'Ohm':
        if mag >= 1e6:
            return 1e-6, 'MΩ'
        if mag >= 1e3:
            return 1e-3, 'kΩ'
        return 1.0, 'Ω'
    if unit == 'deg':
        return 1.0, '°'
    return 1.0, unit


# ─────────────────────────────────────────────────────────────────────────
#  Summary
# ─────────────────────────────────────────────────────────────────────────

def hold_windows(d):
    """Yield (key, index array) for the settled part of every hold segment."""
    seg = d['segment']
    rep = d['rep']
    keys = []
    for i in range(len(seg)):
        k = (rep[i], seg[i], d['phase'][i])      # phase splits off the shutdown vent
        if not keys or keys[-1][0] != k:
            keys.append([k, i, i])
        else:
            keys[-1][2] = i
    for (r, s, _), a, b in keys:
        if d['phase'][a] not in HOLD_PHASES:
            continue
        idx = np.arange(a, b + 1)
        t = d['t_s'][idx]
        t_cut = t[0] + SETTLE_FRACTION * (t[-1] - t[0])
        yield (r, s), idx[t >= t_cut]


def summarize(d):
    lcr = d['lcr_cols']
    ok = d.get('lcr_ok')
    out = []
    for (r, s), idx in hold_windows(d):
        if idx.size == 0:
            continue
        i0 = idx[0]
        row = {'rep': int(r), 'segment': int(s), 'step': int(d['step'][i0]),
               'cycle': int(d['cycle'][i0]), 'phase': d['phase'][i0],
               'label': d['label'][i0], 'valves': d['valves'][i0],
               't_start_s': round(float(d['t_s'][idx[0]]), 2),
               'setpoint_mbar': round(float(d['setpoint_mbar'][i0]), 2),
               'pressure_mean_mbar': round(float(np.nanmean(d['pressure_mbar'][idx])), 3),
               'pressure_std_mbar': round(float(np.nanstd(d['pressure_mbar'][idx])), 3)}
        if lcr:
            good = idx[ok[idx] > 0] if ok is not None else idx
            row['lcr_ok_frac'] = round(good.size / idx.size, 3)
            use = good if good.size else idx
            for c in lcr:
                row[c + '_mean'] = float(np.nanmean(d[c][use]))
                row[c + '_std'] = float(np.nanstd(d[c][use]))
        out.append(row)

    # change of the LCR primary vs the first 0-mbar hold (baseline)
    if lcr and out:
        c = lcr[0] + '_mean'
        base = next((r[c] for r in out if r['setpoint_mbar'] == 0), None)
        for r in out:
            r['primary_change_pct'] = (round(100.0 * (r[c] - base) / base, 4)
                                       if base not in (None, 0) else float('nan'))
    return out


def write_summary(rows, path):
    if not rows:
        return
    keys = list(rows[0].keys())
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def print_summary(rows, lcr_cols):
    if not rows:
        return
    pc = lcr_cols[0] if lcr_cols else None
    print('\n  Hold summary (settled part of each hold)')
    hdr = f"  {'step':>4} {'phase':<9} {'valves':<10} {'sp':>6} {'P mean':>8} {'±':>5}"
    if pc:
        hdr += f"  {quantity(pc)[0] + ' mean':>12} {'Δ%':>8} {'ok':>5}"
    print(hdr)
    for r in rows:
        line = (f"  {r['step']:>4} {r['phase']:<9} {r['valves']:<10} "
                f"{r['setpoint_mbar']:6.0f} {r['pressure_mean_mbar']:8.2f} "
                f"{r['pressure_std_mbar']:5.2f}")
        if pc:
            line += (f"  {r[pc + '_mean']:12.5g} {r['primary_change_pct']:8.3f} "
                     f"{r['lcr_ok_frac']:5.0%}")
        print(line)


# ─────────────────────────────────────────────────────────────────────────
#  Plots
# ─────────────────────────────────────────────────────────────────────────

def _style(ax):
    ax.set_facecolor(C_SURFACE)
    ax.grid(False)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color('#CBBFAE')
    ax.tick_params(colors=C_TEXT2, labelsize=9)
    ax.yaxis.label.set_color(C_TEXT2)
    ax.xaxis.label.set_color(C_TEXT2)


def _lcr_samples(d):
    """Indices of rows carrying a NEW LCR reading (one per lcr_n)."""
    n = d['lcr_n']
    keep = np.ones(n.size, bool)
    keep[1:] = n[1:] != n[:-1]
    return np.where(keep)[0]


def _shade_pressurised(ax, d):
    sp = d['setpoint_mbar']
    on = sp > 0.5
    t = d['t_s']
    i = 0
    while i < on.size:
        if on[i]:
            j = i
            while j + 1 < on.size and on[j + 1]:
                j += 1
            ax.axvspan(t[i], t[j], color=C_GRID, alpha=0.45, linewidth=0)
            i = j + 1
        else:
            i += 1


def _figure(**kw):
    # pyplot-free Figure: safe inside the Tk GUI and in worker threads
    from matplotlib.figure import Figure
    return Figure(layout='constrained', **kw)


def plot_timeseries(d, meta, folder):
    lcr = d['lcr_cols']
    n_rows = 1 + len(lcr)
    fig = _figure(figsize=(11, 2.6 + 2.3 * n_rows))
    axes = fig.subplots(n_rows, 1, sharex=True)
    axes = np.atleast_1d(axes)
    fig.patch.set_facecolor(C_SURFACE)

    ax = axes[0]
    _style(ax)
    _shade_pressurised(ax, d)
    ax.plot(d['t_s'], d['setpoint_mbar'], color=C_SETPOINT, lw=1.2, ls='--',
            label='set-point')
    ax.plot(d['t_s'], d['pressure_mbar'], color=C_PRESSURE, lw=2, label='measured')
    ax.set_ylabel('Pressure (mbar)')
    ax.legend(loc='upper right', frameon=False, fontsize=9, labelcolor=C_TEXT2)
    ax.set_title('Pressure vs time', loc='left', color=C_TEXT, fontsize=11)

    if lcr:
        li = _lcr_samples(d)
        ok = d['lcr_ok'][li] > 0
        for ax, c in zip(axes[1:], lcr):
            _style(ax)
            _shade_pressurised(ax, d)
            name, unit = quantity(c)
            k, u = display_scale(name, unit, d[c][li])
            y = d[c][li] * k
            t = d['t_s'][li]
            mostly_out = ok.size and ok.mean() < 0.5
            ax.plot(t, y, color=C_HOLD if mostly_out else C_INFLATE, lw=1.6)
            if (~ok).any() and not mostly_out:
                ax.plot(t[~ok], y[~ok], ls='none', marker='o', ms=3,
                        color=C_DEFLATE, alpha=0.6, label='out of range (OUT)')
                ax.legend(loc='upper right', frameon=False, fontsize=9,
                          labelcolor=C_TEXT2)
            ax.set_ylabel(f'{PRETTY.get(name, name)} ({u})' if u else PRETTY.get(name, name))
            title = f'{PRETTY.get(name, name)} vs time'
            if mostly_out:
                title += f'   —   {1 - ok.mean():.0%} of readings OUT of range: check probe wiring'
            ax.set_title(title, loc='left', color=C_TEXT, fontsize=11)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle(_suptitle(folder, meta), x=0.01, ha='left', color=C_TEXT2, fontsize=9)
    path = os.path.join(folder, 'timeseries.png')
    fig.savefig(path, dpi=150, facecolor=C_SURFACE)
    return fig, path


def plot_lcr_vs_pressure(d, meta, summary, folder):
    lcr = d['lcr_cols']
    if not lcr:
        return None, None
    li = _lcr_samples(d)
    P = d['pressure_mbar'][li]
    ph = d['phase'][li]
    fig = _figure(figsize=(5.6 * len(lcr), 4.6))
    axes = fig.subplots(1, len(lcr))
    axes = np.atleast_1d(axes)
    fig.patch.set_facecolor(C_SURFACE)
    for ax, c in zip(axes, lcr):
        _style(ax)
        name, unit = quantity(c)
        k, u = display_scale(name, unit, d[c][li])
        y = d[c][li] * k
        hold = np.isin(ph, HOLD_PHASES)
        ax.plot(P[hold], y[hold], ls='none', marker='o', ms=2.5, color=C_HOLD,
                alpha=0.35, label='hold samples')
        for phase, col, lab in (('inflate', C_INFLATE, 'inflation'),
                                ('deflate', C_DEFLATE, 'deflation')):
            m = ph == phase
            ax.plot(P[m], y[m], ls='none', marker='o', ms=3, color=col,
                    alpha=0.8, label=lab)
        # settled hold means
        if summary:
            px = np.array([r['pressure_mean_mbar'] for r in summary])
            pe = np.array([r['pressure_std_mbar'] for r in summary])
            my = np.array([r[c + '_mean'] for r in summary]) * k
            me = np.array([r[c + '_std'] for r in summary]) * k
            ax.errorbar(px, my, xerr=pe, yerr=me, ls='none', marker='D', ms=6,
                        color=C_TEXT, mfc=C_SURFACE, mew=1.5, elinewidth=1,
                        capsize=3, label='hold mean ± sd', zorder=5)
        ax.set_xlabel('Measured pressure (mbar)')
        lab = PRETTY.get(name, name)
        ax.set_ylabel(f'{lab} ({u})' if u else lab)
        ax.set_title(f'{lab} vs pressure', loc='left', color=C_TEXT, fontsize=11)
        ax.legend(loc='best', frameon=False, fontsize=8, labelcolor=C_TEXT2)
    fig.suptitle(_suptitle(folder, meta), x=0.01, ha='left', color=C_TEXT2, fontsize=9)
    path = os.path.join(folder, 'lcr_vs_pressure.png')
    fig.savefig(path, dpi=150, facecolor=C_SURFACE)
    return fig, path


def _suptitle(folder, meta):
    s = os.path.basename(os.path.normpath(folder))
    lcr = (meta.get('lcr') or {}).get('describe')
    if lcr:
        s += '   ·   LCR ' + lcr.replace('Mode: ', '')
    if meta.get('status') and meta['status'] != 'completed':
        s += f"   ·   {meta['status']}"
    return s


# ─────────────────────────────────────────────────────────────────────────
#  Entry points
# ─────────────────────────────────────────────────────────────────────────

def analyze(folder, show=False, verbose=True):
    """Make plots + summary for one session folder.  Returns list of files."""
    d, meta = load_session(folder)
    summary = summarize(d)
    files = []
    sp = os.path.join(folder, 'summary.csv')
    write_summary(summary, sp)
    if summary:
        files.append(sp)
    f1, p1 = plot_timeseries(d, meta, folder)
    files.append(p1)
    f2, p2 = plot_lcr_vs_pressure(d, meta, summary, folder)
    if p2:
        files.append(p2)

    if verbose:
        print_summary(summary, d['lcr_cols'])
        if d['lcr_cols']:
            okf = np.nanmean(d['lcr_ok'])
            if okf < 0.5:
                print(f'\n  WARNING: only {okf:.0%} of LCR readings were in range '
                      '(OUT) — check the probe wiring.')
        print('\n  Analysis files:')
        for f in files:
            print('    ' + f)
    if show:
        open_files([f for f in files if f.endswith('.png')])
    return files


def open_files(paths):
    """Open files with the OS viewer (Preview on macOS)."""
    import subprocess
    for p in paths:
        try:
            if sys.platform == 'darwin':
                subprocess.Popen(['open', p])
            elif sys.platform.startswith('win'):
                os.startfile(p)
            else:
                subprocess.Popen(['xdg-open', p])
        except Exception as e:
            print(f'  could not open {p}: {e}')


def newest_session(root=os.path.join(HERE, 'logs')):
    dirs = [p for p in glob.glob(os.path.join(root, '*')) if
            os.path.isfile(os.path.join(p, 'data.csv'))]
    if not dirs:
        raise SystemExit(f'no session folders in {root}')
    return max(dirs, key=os.path.getmtime)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', nargs='?', help='session folder [newest in logs/]')
    ap.add_argument('--show', action='store_true', help='open the figures')
    a = ap.parse_args()
    analyze(a.folder or newest_session(), show=a.show)


if __name__ == '__main__':
    main()
