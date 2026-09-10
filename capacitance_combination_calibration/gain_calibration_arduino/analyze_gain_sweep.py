"""
analyze_gain_sweep.py — Star-Nose Sensor | which FT5316 gain should we use?
===========================================================================
Reads a gain_sweep_collector.py CSV (and, optionally, the older
../logs/*.csv files from combination_calibration_collector.py, which are
treated as a single `native`-gain sweep) and answers one question:

    at which analog gain does the muca board measure capacitance best?

"Best" is not "biggest number". Four things are computed per gain, and a gain
has to survive all four:

  1. SENSITIVITY   |slope| in counts per pF, from a straight-line fit of the
                   responding cell's raw counts against the LCR's Cp.
                   Higher is better — but only if 2-4 also hold.

  2. LINEARITY     R^2 of that fit. A high slope with a poor R^2 means the AFE
                   is bending, not measuring.

  3. HEADROOM      distance from both rails (0 and 65,535) over the whole
                   capacitance range, including the extrapolated Cp=0
                   intercept. A gain whose intercept lies outside [0, 65535]
                   is CLIPPING: part of your range is unrecoverable, and no
                   software calibration gets it back. This is the check the
                   2026-08-26 session fails — its intercept extrapolates to
                   ~65,770 against a ceiling of 65,535.

  4. RESOLUTION    frame-to-frame noise divided by |slope|, i.e. the smallest
                   capacitance change the board can actually resolve, in pF.
                   This is the number that matters for the sensor, and it is
                   the tie-breaker: raising the gain raises the slope AND the
                   noise, so more gain is not automatically better.

The responding cell for each point is found from the data, not from a table:
it is the cell whose median moves most across the capacitor combinations.
That matches how the collectors work — the operator names the points at the
bench and there is no fixed name-to-channel map.

Usage
-----
  python analyze_gain_sweep.py logs/gain_sweep_session_20260911_101500.csv
  python analyze_gain_sweep.py logs/*.csv --legacy ../logs/flat_calibration_muca_lcr_session_20260826_225636.csv
  python analyze_gain_sweep.py logs/my_sweep.csv --outdir results --no-plots
"""

import os
import sys
import csv
import glob
import math
import argparse
import statistics as st
from collections import defaultdict

RAW_FULL_SCALE = 65535
N_CELLS        = 19

# ---------------------------------------------------------------------------
# Palette — light chart surface. Gain is an ORDERED quantity, so it gets a
# single-hue sequential ramp (blue), never a cycled categorical set. The
# ordinal floor on a light surface is step 250, so the ramp runs 250 -> 700.
# ---------------------------------------------------------------------------
SURFACE      = '#fcfcfb'
INK_PRIMARY  = '#0b0b0b'
INK_SECOND   = '#52514e'
INK_MUTED    = '#898781'
GRIDLINE     = '#e1e0d9'
BASELINE_INK = '#c3c2b7'
SERIES_BLUE  = '#2a78d6'
STATUS_GOOD  = '#0ca30c'
STATUS_CRIT  = '#d03b3b'
BLUE_RAMP    = ['#86b6ef', '#6da7ec', '#5598e7', '#3987e5',
                '#2a78d6', '#256abf', '#1c5cab', '#184f95',
                '#104281', '#0d366b']


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def load_rows(paths, legacy_paths):
    """Rows from every CSV. Legacy files (no `gain` column) become gain='native'."""
    rows = []
    for p in paths:
        with open(p, newline='') as f:
            for r in csv.DictReader(f):
                r['_source'] = os.path.basename(p)
                r.setdefault('gain', '')
                if not r.get('gain'):
                    r['gain'] = 'native'
                rows.append(r)
    for p in legacy_paths:
        with open(p, newline='') as f:
            for r in csv.DictReader(f):
                r['_source'] = os.path.basename(p) + ' (legacy)'
                r['gain'] = 'native'
                rows.append(r)
    if not rows:
        print("No rows loaded — check the file paths.")
        sys.exit(1)
    return rows


def gain_key(g):
    """Sort 'native' first, then numerically."""
    try:
        return (1, int(g))
    except (TypeError, ValueError):
        return (0, 0)


def gain_label(g):
    return str(g)


def cells_of(row):
    out = []
    for i in range(1, N_CELLS + 1):
        v = row.get(f'cell_{i}', '')
        if v == '' or v is None:
            return None
        try:
            out.append(float(v))
        except ValueError:
            return None
    return out


# ---------------------------------------------------------------------------
# Reshaping
# ---------------------------------------------------------------------------

def build_index(rows):
    """
    truth[combo]                       -> median Cp in pF
    frames[gain][point][combo]         -> list of 19-cell frames
    baseline[gain]                     -> list of 19-cell frames (no load)
    """
    truth    = defaultdict(list)
    frames   = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    baseline = defaultdict(list)

    for r in rows:
        phase = (r.get('phase') or '').lower()
        combo = r.get('combination_label') or r.get('combination_index') or '?'

        if phase == 'lcr':
            try:
                cp = float(r.get('Cp_pF'))
            except (TypeError, ValueError):
                continue
            if math.isfinite(cp) and 0 < cp < 1e6:
                truth[combo].append(cp)
            continue

        cells = cells_of(r)
        if cells is None:
            continue

        if phase == 'baseline':
            baseline[r['gain']].append(cells)
        else:
            point = r.get('point_label') or 'point'
            frames[r['gain']][point][combo].append(cells)

    return ({k: st.median(v) for k, v in truth.items()}, frames, baseline)


def responding_cell(per_combo_medians):
    """Cell index whose median moves most across combinations."""
    combos = list(per_combo_medians)
    if len(combos) < 2:
        # Only one combination — fall back to the cell furthest from the pack.
        med = per_combo_medians[combos[0]]
        centre = st.median(med)
        return max(range(len(med)), key=lambda i: abs(med[i] - centre))
    spread = []
    for i in range(N_CELLS):
        vals = [per_combo_medians[c][i] for c in combos]
        spread.append(max(vals) - min(vals))
    return spread.index(max(spread))


def linfit(x, y):
    """Least squares. Returns (intercept, slope, r2) or None if degenerate."""
    n = len(x)
    if n < 2:
        return None
    mx, my = sum(x) / n, sum(y) / n
    sxx = sum((a - mx) ** 2 for a in x)
    if sxx == 0:
        return None
    slope = sum((a - mx) * (b - my) for a, b in zip(x, y)) / sxx
    inter = my - slope * mx
    sst = sum((b - my) ** 2 for b in y)
    sse = sum((b - (inter + slope * a)) ** 2 for a, b in zip(x, y))
    r2 = 1.0 - sse / sst if sst > 0 else float('nan')
    return inter, slope, r2


# ---------------------------------------------------------------------------
# Per-gain metrics
# ---------------------------------------------------------------------------

def analyse(truth, frames, baseline):
    results = []

    for gain in sorted(frames, key=gain_key):
        for point in sorted(frames[gain]):
            per_combo = {}
            per_combo_noise = {}
            per_combo_clip = {}

            for combo, fr in frames[gain][point].items():
                if combo not in truth or not fr:
                    continue
                med = [st.median([f[i] for f in fr]) for i in range(N_CELLS)]
                per_combo[combo] = med
                per_combo_noise[combo] = fr
                per_combo_clip[combo] = fr

            if len(per_combo) < 2:
                continue

            cell = responding_cell(per_combo)

            xs, ys = [], []
            noises, clipped, total = [], 0, 0
            for combo, med in per_combo.items():
                xs.append(truth[combo])
                ys.append(med[cell])
                series = [f[cell] for f in per_combo_noise[combo]]
                if len(series) > 2:
                    noises.append(st.pstdev(series))
                for v in series:
                    total += 1
                    if v >= RAW_FULL_SCALE - 1 or v <= 0:
                        clipped += 1

            fit = linfit(xs, ys)
            if fit is None:
                continue
            inter, slope, r2 = fit

            noise = st.median(noises) if noises else float('nan')
            resolution = abs(noise / slope) if (slope and noise == noise and slope != 0) \
                else float('nan')

            base = float('nan')
            if gain in baseline and baseline[gain]:
                base = st.median([f[cell] for f in baseline[gain]])

            lo, hi = min(ys), max(ys)
            # Where the fit says the reading sits with no capacitor at all.
            intercept_ok = 0 <= inter <= RAW_FULL_SCALE
            rail_margin = min(lo, RAW_FULL_SCALE - hi)

            results.append({
                'gain':            gain,
                'point':           point,
                'cell':            cell + 1,
                'n_combos':        len(xs),
                'cp_min':          min(xs),
                'cp_max':          max(xs),
                'raw_min':         lo,
                'raw_max':         hi,
                'span':            hi - lo,
                'span_pct':        100.0 * (hi - lo) / RAW_FULL_SCALE,
                'slope':           slope,
                'sensitivity':     abs(slope),
                'intercept':       inter,
                'intercept_ok':    intercept_ok,
                'r2':              r2,
                'noise':           noise,
                'resolution_pF':   resolution,
                'baseline':        base,
                'rail_margin':     rail_margin,
                'clip_frac':       (clipped / total) if total else 0.0,
                'usable':          intercept_ok and rail_margin > 0 and r2 >= 0.95,
            })

    return results


def summarise_by_gain(results):
    """Collapse the per-point rows into one row per gain (median across points)."""
    by_gain = defaultdict(list)
    for r in results:
        by_gain[r['gain']].append(r)

    out = []
    for gain, rs in by_gain.items():
        def med(k):
            vals = [r[k] for r in rs if r[k] == r[k]]
            return st.median(vals) if vals else float('nan')
        out.append({
            'gain':          gain,
            'n_points':      len(rs),
            'sensitivity':   med('sensitivity'),
            'slope':         med('slope'),
            'r2':            med('r2'),
            'span_pct':      med('span_pct'),
            'intercept':     med('intercept'),
            'rail_margin':   med('rail_margin'),
            'noise':         med('noise'),
            'resolution_pF': med('resolution_pF'),
            'clip_frac':     med('clip_frac'),
            'intercept_ok':  all(r['intercept_ok'] for r in rs),
            'usable':        all(r['usable'] for r in rs),
        })
    return sorted(out, key=lambda r: gain_key(r['gain']))


def rank(summary):
    """Usable gains first, then finest resolution."""
    def key(r):
        res = r['resolution_pF']
        if res != res:
            res = float('inf')
        return (0 if r['usable'] else 1, res)
    return sorted(summary, key=key)


# ---------------------------------------------------------------------------
# Text report (this is the table view — it always exists, plots or not)
# ---------------------------------------------------------------------------

def print_report(summary, results):
    print()
    print("=" * 100)
    print("  PER-GAIN SUMMARY   (median across points; 16-bit full scale ="
          f" {RAW_FULL_SCALE})")
    print("=" * 100)
    head = (f"{'gain':>7} {'pts':>4} {'counts/pF':>11} {'R^2':>7} {'span%':>7} "
            f"{'Cp=0 fit':>10} {'rail gap':>9} {'noise':>8} {'res [pF]':>9}  verdict")
    print(head)
    print("-" * 100)
    for r in summary:
        verdict = "OK" if r['usable'] else "CLIPPING / NONLINEAR"
        if r['clip_frac'] > 0.01:
            verdict = "RAILED"
        print(f"{gain_label(r['gain']):>7} {r['n_points']:>4} "
              f"{r['sensitivity']:>11.1f} {r['r2']:>7.4f} {r['span_pct']:>7.2f} "
              f"{r['intercept']:>10.0f} {r['rail_margin']:>9.0f} "
              f"{r['noise']:>8.1f} {r['resolution_pF']:>9.4f}  {verdict}")

    print()
    ranked = rank(summary)
    best = ranked[0] if ranked else None
    if best and best['usable']:
        print(f"  RECOMMENDED GAIN: {gain_label(best['gain'])}")
        print(f"    {best['sensitivity']:.1f} counts/pF, R^2 = {best['r2']:.4f}, "
              f"resolves ~{best['resolution_pF']:.4f} pF,")
        print(f"    uses {best['span_pct']:.1f}% of the 16-bit range with "
              f"{best['rail_margin']:.0f} counts of margin to the nearest rail.")
    else:
        print("  NO GAIN IN THIS SWEEP IS CLEAN.")
        print("    Every gain either clips a rail or fits poorly. Widen the sweep,")
        print("    or reduce the fixed capacitance the cell sees (shorter leads,")
        print("    guarding) before turning the gain down further.")

    print()
    print("-" * 100)
    print("  PER-POINT DETAIL")
    print("-" * 100)
    print(f"{'gain':>7} {'point':>10} {'cell':>5} {'n':>3} {'counts/pF':>11} "
          f"{'R^2':>7} {'baseline':>9} {'raw min':>9} {'raw max':>9}  flag")
    for r in sorted(results, key=lambda r: (gain_key(r['gain']), r['point'])):
        flag = "" if r['usable'] else ("railed" if r['clip_frac'] > 0.01 else "check")
        base = f"{r['baseline']:.0f}" if r['baseline'] == r['baseline'] else "-"
        print(f"{gain_label(r['gain']):>7} {r['point']:>10} {r['cell']:>5} "
              f"{r['n_combos']:>3} {r['sensitivity']:>11.1f} {r['r2']:>7.4f} "
              f"{base:>9} {r['raw_min']:>9.0f} {r['raw_max']:>9.0f}  {flag}")
    print()


def write_summary_csv(summary, results, outdir):
    os.makedirs(outdir, exist_ok=True)
    p1 = os.path.join(outdir, 'gain_summary.csv')
    with open(p1, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    p2 = os.path.join(outdir, 'gain_per_point.csv')
    with open(p2, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)
    print(f"  Table view written: {p1}")
    print(f"                      {p2}")
    return p1, p2


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def style_axes(ax, title, xlabel, ylabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK_PRIMARY, fontsize=11, loc='left', pad=10)
    ax.set_xlabel(xlabel, color=INK_SECOND, fontsize=9)
    ax.set_ylabel(ylabel, color=INK_SECOND, fontsize=9)
    ax.grid(True, color=GRIDLINE, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(BASELINE_INK)
        ax.spines[side].set_linewidth(1.0)
    ax.tick_params(colors=INK_MUTED, labelsize=8, length=3)


def make_plots(summary, results, truth, frames, outdir, stem):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib not available — skipping plots "
              "(the tables above have everything).")
        return None

    os.makedirs(outdir, exist_ok=True)
    gains = [r['gain'] for r in summary]
    ramp = [BLUE_RAMP[min(len(BLUE_RAMP) - 1,
                          int(round(i * (len(BLUE_RAMP) - 1) / max(1, len(gains) - 1))))]
            for i in range(len(gains))]
    colour = dict(zip(gains, ramp))

    fig, axes = plt.subplots(2, 2, figsize=(13, 9.5))
    fig.patch.set_facecolor(SURFACE)

    # (a) Response curves — one line per gain, sequential ramp, ordered.
    ax = axes[0][0]
    style_axes(ax, 'Response over the full 16-bit scale\n'
                   '(a flat line pinned at the top means almost none of the '
                   'range is in use)',
               'Cp from LCR-6100  [pF]', 'raw counts (responding cell)')
    ax.axhline(RAW_FULL_SCALE, color=STATUS_CRIT, linewidth=1.5, linestyle='--', zorder=2)
    ax.annotate('16-bit ceiling 65,535 — anything here is clipped',
                xy=(0.02, RAW_FULL_SCALE), xycoords=('axes fraction', 'data'),
                va='bottom', fontsize=8, color=STATUS_CRIT)
    ax.axhline(0, color=BASELINE_INK, linewidth=1.0, zorder=2)

    drawn = 0
    for g in gains:
        pts = [r for r in results if r['gain'] == g]
        if not pts:
            continue
        r = pts[0]
        pairs = []
        for combo, fr in frames[g][r['point']].items():
            if combo in truth and fr:
                pairs.append((truth[combo],
                              st.median([f[r['cell'] - 1] for f in fr])))
        pairs.sort()
        if len(pairs) < 2:
            continue
        xs = [p[0] for p in pairs]
        ys = [p[1] for p in pairs]
        ax.plot(xs, ys, marker='o', markersize=5, linewidth=2,
                color=colour[g], label=f'gain {gain_label(g)}', zorder=3)
        drawn += 1
    # Legend for >=2 series so identity is never colour-alone; a single
    # series is named by the title instead of a one-row legend box.
    if drawn >= 2:
        ax.legend(frameon=False, fontsize=8, labelcolor=INK_SECOND,
                  ncol=2 if drawn > 6 else 1, loc='center right')
    elif drawn == 1:
        ax.legend(frameon=False, fontsize=8, labelcolor=INK_SECOND,
                  loc='center right')

    # (b) Sensitivity — single series, one axis.
    ax = axes[0][1]
    style_axes(ax, 'Sensitivity: how many counts one pF moves',
               'gain (register 0x07)', 'counts per pF  (|slope|)')
    labels = [gain_label(r['gain']) for r in summary]
    vals   = [r['sensitivity'] for r in summary]
    bar_w  = 0.62 if len(labels) > 2 else 0.28
    bars = ax.bar(labels, vals, color=SERIES_BLUE, zorder=3, width=bar_w)
    ax.margins(x=0.35 if len(labels) < 3 else 0.05)
    ax.set_ylim(0, max(vals) * 1.18 if vals else 1)
    for b, v, r in zip(bars, vals, summary):
        if not r['usable']:
            b.set_color(STATUS_CRIT)
        ax.text(b.get_x() + b.get_width() / 2, v, f'{v:.0f}',
                ha='center', va='bottom', fontsize=7.5, color=INK_SECOND)
    import matplotlib.patches as mpatches
    ax.legend(handles=[mpatches.Patch(color=SERIES_BLUE, label='usable'),
                       mpatches.Patch(color=STATUS_CRIT,
                                      label='clipping or nonlinear')],
              frameon=False, fontsize=8, labelcolor=INK_SECOND, loc='upper right')

    # (c) Headroom to the rails.
    ax = axes[1][0]
    style_axes(ax, 'Headroom: margin to the nearest rail\n'
                   '(at or below zero, part of the range is unrecoverable)',
               'gain (register 0x07)', 'counts to nearest rail')
    margins = [r['rail_margin'] for r in summary]
    cols = [STATUS_GOOD if r['usable'] else STATUS_CRIT for r in summary]
    ax.bar(labels, margins, color=cols, zorder=3, width=bar_w)
    ax.margins(x=0.35 if len(labels) < 3 else 0.05)
    ax.axhline(0, color=BASELINE_INK, linewidth=1.2, zorder=4)
    top = max(margins) if margins else 1
    ax.set_ylim(min(0, min(margins) * 1.2 if margins else 0), top * 1.25)
    for i, r in enumerate(summary):
        if not r['intercept_ok']:
            # Status colour never carries the meaning alone — it is labelled.
            ax.annotate('clipped', (i, max(0, margins[i])), ha='center',
                        va='bottom', fontsize=8, color=STATUS_CRIT,
                        textcoords='offset points', xytext=(0, 6))

    # (d) Resolution — the decision chart.
    ax = axes[1][1]
    style_axes(ax, 'Resolution: smallest capacitance change the board can see',
               'gain (register 0x07)', 'noise-limited resolution  [pF]')
    res = [r['resolution_pF'] for r in summary]
    ax.plot(labels, res, marker='o', markersize=8, linewidth=2,
            color=SERIES_BLUE, zorder=3)
    ax.margins(x=0.35 if len(labels) < 3 else 0.05, y=0.25)
    finite = [(i, v) for i, v in enumerate(res)
              if v == v and summary[i]['usable']]
    if finite:
        bi, bv = min(finite, key=lambda t: t[1])
        ax.plot([labels[bi]], [bv], marker='o', markersize=11,
                color=STATUS_GOOD, zorder=4)
        ax.annotate(f'best usable: gain {labels[bi]}\n{bv:.4f} pF',
                    (bi, bv), textcoords='offset points', xytext=(8, 10),
                    fontsize=8.5, color=STATUS_GOOD)
    ax.set_yscale('log')

    fig.suptitle('FT5316 analog-gain calibration — star-nose capacitive sensor',
                 color=INK_PRIMARY, fontsize=13, x=0.008, ha='left')
    fig.tight_layout(rect=[0, 0, 1, 0.965])

    out = os.path.join(outdir, f'{stem}_gain_analysis.png')
    fig.savefig(out, dpi=160, facecolor=SURFACE)
    plt.close(fig)
    print(f"  Figure written:     {out}")
    return out


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Compare FT5316 analog gains from a gain sweep CSV.")
    ap.add_argument('csv', nargs='*', default=[],
                    help="gain-sweep CSV(s) from gain_sweep_collector.py. May be "
                         "omitted if --legacy is given, to characterise the old "
                         "single-gain datasets on their own.")
    ap.add_argument('--legacy', nargs='*', default=[],
                    help="older combination_calibration_collector.py CSV(s); "
                         "logged as gain='native' so they can be compared directly")
    ap.add_argument('--outdir', default='results')
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()

    paths = []
    for pattern in args.csv:
        paths.extend(sorted(glob.glob(pattern)) or [pattern])
    legacy = []
    for pattern in args.legacy:
        legacy.extend(sorted(glob.glob(pattern)) or [pattern])
    if not paths and not legacy:
        ap.error("give at least one CSV, either positionally or via --legacy")

    print("=" * 100)
    print("  FT5316 GAIN ANALYSIS")
    print("=" * 100)
    for p in paths:
        print(f"  sweep : {p}")
    for p in legacy:
        print(f"  legacy: {p}   (treated as gain='native')")

    rows = load_rows(paths, legacy)
    truth, frames, baseline = build_index(rows)

    print(f"\n  {len(rows)} rows, {len(truth)} capacitor combinations, "
          f"{len(frames)} gain settings.")
    if truth:
        print("  Ground truth (LCR-6100): " +
              ", ".join(f"{k}={v:.3f} pF" for k, v in sorted(truth.items(),
                                                             key=lambda kv: kv[1])))
    if not baseline:
        print("  NOTE: no 'baseline' phase in these files — the Cp=0 intercept is")
        print("        extrapolated from the fit rather than measured directly.")

    results = analyse(truth, frames, baseline)
    if not results:
        print("\n  Not enough data: at least two capacitor combinations are needed")
        print("  at the same gain and point to fit a line.")
        sys.exit(1)

    summary = summarise_by_gain(results)
    print_report(summary, results)
    write_summary_csv(summary, results, args.outdir)

    if not args.no_plots:
        stem = os.path.splitext(os.path.basename((paths or legacy)[0]))[0]
        make_plots(summary, results, truth, frames, args.outdir, stem)

    print()


if __name__ == '__main__':
    main()
