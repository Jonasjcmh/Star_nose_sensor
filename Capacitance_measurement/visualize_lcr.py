"""
visualize_lcr.py — RS Pro LCR-6100 | Live + Post-Collection Visualizer
=======================================================================

Two modes
---------
  LIVE  : Connect to the LCR-6100 and show scrolling Cp/Rp plots updated
          in real time. Useful for verifying probe contact before/during
          collection and for standalone bench measurements.

  FILE  : Load a dataset CSV from capacitance_dataset_collector.py and
          generate a detailed set of LCR-focused plots:
            • Cp and Rp time series per point (all samples overlaid, phase-coloured)
            • Cp and Rp per-point summary (mean ± std, CV)
            • Cp vs Rp phase-plane scatter
            • Sample drift — does Cp change across the collection session?
            • Rp heatmap on the sensor layout
            • Per-point Cp histogram

Usage
-----
  Live mode:
    python visualize_lcr.py --live
    python visualize_lcr.py --live --port /dev/ttyUSB0

  File mode:
    python visualize_lcr.py --file logs/capacitance_dataset_20260623_123456.csv

  Both (live window + file analysis):
    python visualize_lcr.py --live --file logs/capacitance_dataset_*.csv
"""

import os
import sys
import csv
import math
import time
import argparse
import glob
import threading
from collections import deque, defaultdict
from datetime import datetime

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as mticker
from matplotlib.animation import FuncAnimation
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.patches import FancyBboxPatch

# ── Sensor layout ──────────────────────────────────────────────────────────────
POINTS_XY = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

PHASE_COLORS = {
    'locate':  '#4A90D9',
    'press':   '#E67E22',
    'hold':    '#27AE60',
    'retract': '#9B59B6',
    'post':    '#1ABC9C',
}

PHASE_ORDER = ['locate', 'press', 'hold', 'retract', 'post']


# ═══════════════════════════════════════════════════════════════════════════════
#  LIVE MODE
# ═══════════════════════════════════════════════════════════════════════════════

LIVE_WINDOW_S = 30.0   # seconds of history shown in scrolling plot
LIVE_MAXPTS   = 3000   # circular buffer size

class LiveLCRWindow:
    """
    Scrolling real-time plot of Cp_pF and Rp_kΩ from the LCR-6100.

    Layout
    ------
      Top    : Cp (pF) vs time — solid=OK, dimmed=OUT/NG
      Middle : Rp (kΩ) vs time — same colouring
      Bottom : Stats bar (current, mean, std, rate, ok%)

    Fixes vs original
    -----------------
    • Always store finite Cp/Rp values regardless of ok status — out-of-range
      readings are plotted dimmed so the axes always have data to display.
    • Title built from live FREQ? / FUNC? queries, not hardcoded strings.
    • ylim set from all finite values, with a minimum visible span.
    """

    def __init__(self, port, baud=None, window_s=LIVE_WINDOW_S):
        from lcr6100 import LCR6100
        self._lcr    = LCR6100(port, baud=baud)
        self._window = window_s

        self._ts  = deque(maxlen=LIVE_MAXPTS)
        self._Cp  = deque(maxlen=LIVE_MAXPTS)   # pF  — always finite or NaN (parse error only)
        self._Rp  = deque(maxlen=LIVE_MAXPTS)   # kΩ  — same
        self._ok  = deque(maxlen=LIVE_MAXPTS)   # bool

        self._n_total = 0
        self._n_ok    = 0
        self._lock    = threading.Lock()

        self._fig  = None
        self._anim = None

    # ── Background sampler ────────────────────────────────────────────────────

    def _sample_loop(self):
        while not self._stop.is_set():
            try:
                Cp_F, Rp_Ohm, ok = self._lcr.measure()
                Cp_pF   = Cp_F   * 1e12
                Rp_kOhm = Rp_Ohm / 1e3
                # Store raw values always — NaN only for genuine parse failures.
                # Out-of-range (ok=False) values are stored and plotted dimmed.
                Cp_store = Cp_pF   if math.isfinite(Cp_pF)   else float('nan')
                Rp_store = Rp_kOhm if math.isfinite(Rp_kOhm) else float('nan')
                with self._lock:
                    self._ts.append(time.time())
                    self._Cp.append(Cp_store)
                    self._Rp.append(Rp_store)
                    self._ok.append(ok)
                    self._n_total += 1
                    if ok:
                        self._n_ok += 1
            except Exception:
                time.sleep(0.05)

    # ── Query actual device settings for title ────────────────────────────────

    def _query_device_info(self):
        """Return (func_str, freq_str) read from the meter."""
        try:
            raw = self._lcr._query('FUNC?').strip()
            # Validate: should be short, no digits only, no embedded CR
            if raw and len(raw) <= 20 and '\r' not in raw and raw.replace('-', '').replace('/', '').isalnum():
                func = raw
            else:
                func = 'Cp-Rp'   # fallback — device is configured to Cp-Rp
        except Exception:
            func = 'Cp-Rp'
        try:
            raw_freq = self._lcr._query('FREQ?').strip()
            freq_hz  = float(raw_freq)
            freq_str = f'{freq_hz/1e3:.0f} kHz' if freq_hz >= 1000 else f'{freq_hz:.0f} Hz'
        except Exception:
            freq_str = '20 kHz'   # fallback — device is configured to 20 kHz
        return func, freq_str

    # ── Figure construction ───────────────────────────────────────────────────

    def _build_fig(self, func, freq_str):
        self._fig = plt.figure(figsize=(13, 8))
        self._fig.patch.set_facecolor('#1A1A2E')

        gs = gridspec.GridSpec(3, 1, height_ratios=[4, 4, 1.2],
                               hspace=0.38, figure=self._fig)
        self._ax_Cp = self._fig.add_subplot(gs[0])
        self._ax_Rp = self._fig.add_subplot(gs[1])
        self._ax_st = self._fig.add_subplot(gs[2])

        for ax in (self._ax_Cp, self._ax_Rp):
            ax.set_facecolor('#0D1117')
            ax.tick_params(colors='#CCCCCC', labelsize=9)
            for spine in ax.spines.values():
                spine.set_color('#444444')
            ax.grid(True, alpha=0.15, color='#FFFFFF')

        self._ax_Cp.set_ylabel('Cp  (pF)', color='#7EB8F7', fontsize=11)
        self._ax_Rp.set_ylabel('Rp  (kΩ)', color='#F0A500', fontsize=11)
        self._ax_Rp.set_xlabel('Time  (s)', color='#CCCCCC', fontsize=10)

        # Two lines per channel: solid = OK reading, dimmed = OUT/NG reading
        self._line_Cp_ok,  = self._ax_Cp.plot([], [], color='#7EB8F7', lw=1.5,
                                               label='OK')
        self._line_Cp_out, = self._ax_Cp.plot([], [], color='#7EB8F7', lw=1.0,
                                               alpha=0.3, ls='--', label='OUT')
        self._line_Rp_ok,  = self._ax_Rp.plot([], [], color='#F0A500', lw=1.5)
        self._line_Rp_out, = self._ax_Rp.plot([], [], color='#F0A500', lw=1.0,
                                               alpha=0.3, ls='--')

        self._ax_Cp.legend(loc='upper left', fontsize=8,
                           facecolor='#1A1A2E', labelcolor='#CCCCCC')

        self._ax_st.set_facecolor('#0D1117')
        self._ax_st.axis('off')
        self._stats_text = self._ax_st.text(
            0.5, 0.5, 'Waiting for data…',
            transform=self._ax_st.transAxes,
            fontsize=9, color='#EEEEEE', ha='center', va='center',
            fontfamily='monospace')

        self._status_dot = self._ax_st.text(
            0.01, 0.5, '●', transform=self._ax_st.transAxes,
            fontsize=14, va='center', color='#888888')

        self._fig.suptitle(
            f'LCR-6100  |  {func}  |  {freq_str}  |  1 V  |  FAST'
            f'  |  {self._lcr._port}',
            color='#EEEEEE', fontsize=11)
        self._fig.canvas.manager.set_window_title('LCR-6100 Live')

    # ── Animation update ──────────────────────────────────────────────────────

    def _update(self, frame):
        now = time.time()

        with self._lock:
            ts  = np.array(self._ts)
            Cp  = np.array(self._Cp)
            Rp  = np.array(self._Rp)
            ok  = np.array(self._ok, dtype=bool)
            nt  = self._n_total
            nok = self._n_ok

        if not len(ts):
            return

        t_rel = ts - now                       # negative, 0 = now
        win   = t_rel >= -self._window

        tr   = t_rel[win]
        Cp_w = Cp[win];  ok_w = ok[win]
        Rp_w = Rp[win]

        def _split(arr, mask):
            """Return two arrays: values where mask=True, NaN elsewhere."""
            a_ok  = np.where(mask,  arr, np.nan)
            a_out = np.where(~mask, arr, np.nan)
            return a_ok, a_out

        Cp_ok, Cp_out = _split(Cp_w, ok_w)
        Rp_ok, Rp_out = _split(Rp_w, ok_w)

        self._line_Cp_ok.set_data(tr, Cp_ok)
        self._line_Cp_out.set_data(tr, Cp_out)
        self._line_Rp_ok.set_data(tr, Rp_ok)
        self._line_Rp_out.set_data(tr, Rp_out)

        # Set ylim from ALL finite values (ok + out), with a minimum span
        for ax, arr in ((self._ax_Cp, Cp_w), (self._ax_Rp, Rp_w)):
            finite = arr[np.isfinite(arr)]
            if len(finite):
                lo, hi = finite.min(), finite.max()
                span   = hi - lo
                margin = max(span * 0.2, abs(np.mean(finite)) * 0.1, 0.5)
                ax.set_ylim(lo - margin, hi + margin)
            ax.set_xlim(-self._window, 0.5)

        # Stats bar
        all_Cp = Cp_w[np.isfinite(Cp_w)]
        all_Rp = Rp_w[np.isfinite(Rp_w)]
        rate   = nt / max(now - float(ts[0]), 1e-3) if len(ts) else 0
        ok_pct = 100 * nok / max(nt, 1)
        last_ok = bool(ok_w[-1]) if len(ok_w) else False

        def _fmt(arr, unit):
            if not len(arr):
                return f'—  {unit}'
            return (f'now={arr[-1]:.3f}  μ={np.mean(arr):.3f}  '
                    f'σ={np.std(arr):.3f}  [{arr.min():.3f}–{arr.max():.3f}]  {unit}')

        self._stats_text.set_text(
            f'Cp: {_fmt(all_Cp, "pF")}     '
            f'Rp: {_fmt(all_Rp, "kΩ")}     '
            f'rate={rate:.1f} Hz   ok={ok_pct:.0f}%   n={nt}')

        self._status_dot.set_color('#27AE60' if last_ok else '#E74C3C')

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self):
        self._stop = threading.Event()
        print(f'[live] Connecting to LCR on {self._lcr._port} ...')
        self._lcr.connect()

        # Query actual device settings for the title
        func, freq_str = self._query_device_info()
        print(f'[live] Device: func={func}  freq={freq_str}')
        print('[live] Starting live plot ...')

        self._sampler = threading.Thread(target=self._sample_loop,
                                         daemon=True, name='lcr-sampler')
        self._sampler.start()

        self._build_fig(func, freq_str)
        self._anim = FuncAnimation(
            self._fig, self._update, interval=100,
            blit=False, cache_frame_data=False)

        try:
            plt.show()
        finally:
            self._stop.set()
            self._lcr.disconnect()
            print('[live] Disconnected.')


def run_live(port, baud=None):
    win = LiveLCRWindow(port, baud=baud)
    win.run()


# ═══════════════════════════════════════════════════════════════════════════════
#  POST-COLLECTION FILE MODE
# ═══════════════════════════════════════════════════════════════════════════════

# ── Data loading ───────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                Cp_pF = float(row['Cp_pF']) if row.get('Cp_pF', '') not in ('', 'nan') else float('nan')
                Rp    = float(row['Rp_Ohm']) if row.get('Rp_Ohm', '') not in ('', 'nan') else float('nan')
                rows.append({
                    'timestamp':  float(row['timestamp']),
                    'round_idx':  int(row['round_idx']),
                    'sample_idx': int(row['sample_idx']),
                    'point':      int(row['point']),
                    'depth_mm':   float(row['depth_mm']),
                    'phase':      row['phase'],
                    'load_cell_N':float(row.get('load_cell_N', 0)),
                    'Cp_pF':      Cp_pF,
                    'Rp_kOhm':    Rp / 1e3 if math.isfinite(Rp) else float('nan'),
                    'lcr_ok':     int(row.get('lcr_ok', 0)),
                })
            except (ValueError, KeyError):
                continue
    return rows


def _group_indentations(rows):
    groups = defaultdict(list)
    for r in rows:
        groups[(r['point'], r['round_idx'], r['sample_idx'])].append(r)
    for k in groups:
        groups[k].sort(key=lambda r: r['timestamp'])
    return groups


def _hold_stats(rows):
    """Per-point stats from hold phase (valid LCR readings only)."""
    data = defaultdict(lambda: {'Cp': [], 'Rp': []})
    for r in rows:
        if r['phase'] == 'hold' and r['lcr_ok'] and not math.isnan(r['Cp_pF']):
            data[r['point']]['Cp'].append(r['Cp_pF'])
            data[r['point']]['Rp'].append(r['Rp_kOhm'])
    return data


# ── Plot A: Cp & Rp time series per point ─────────────────────────────────────

def plot_lcr_timeseries(groups, out_dir):
    """
    5×4 grid of subplots. Each cell = one sensor point.
    Traces for every sample overlaid, colour-coded by phase.
    Top half of cell = Cp_pF, bottom half = Rp_kΩ.
    """
    n_cols = 5
    n_rows = 4
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(22, 16))
    fig.suptitle('Cp (pF) and Rp (kΩ) vs Time — per Point, all Samples, phase-coloured',
                 fontsize=12)
    fig.patch.set_facecolor('#F9F9F9')

    ax_flat = axes.flatten()
    for ax in ax_flat:
        ax.set_visible(False)

    for ax, pt in zip(ax_flat, sorted(POINTS_XY)):
        ax.set_visible(True)
        ax2 = ax.twinx()

        for (pt2, _, _), rows_g in groups.items():
            if pt2 != pt or not rows_g:
                continue
            t0 = rows_g[0]['timestamp']
            for phase in PHASE_ORDER:
                pr = [r for r in rows_g if r['phase'] == phase]
                if not pr:
                    continue
                ts  = [r['timestamp'] - t0 for r in pr]
                Cp  = [r['Cp_pF']    if (r['lcr_ok'] and not math.isnan(r['Cp_pF']))    else float('nan') for r in pr]
                Rp  = [r['Rp_kOhm'] if (r['lcr_ok'] and not math.isnan(r['Rp_kOhm'])) else float('nan') for r in pr]
                col = PHASE_COLORS.get(phase, '#888888')
                ax.plot(ts, Cp, color=col, lw=0.9, alpha=0.6)
                ax2.plot(ts, Rp, color=col, lw=0.6, alpha=0.35, ls='--')

        ax.set_title(f'P{pt:02d}  ({POINTS_XY[pt][0]:+.0f},{POINTS_XY[pt][1]:+.0f})',
                     fontsize=7.5)
        ax.set_xlabel('t (s)', fontsize=6)
        ax.set_ylabel('Cp (pF)', fontsize=6.5, color='#2B6CB0')
        ax2.set_ylabel('Rp (kΩ)', fontsize=6.5, color='#975A16')
        ax.tick_params(labelsize=5.5)
        ax2.tick_params(labelsize=5.5)

    # Legend for phases
    handles = [matplotlib.lines.Line2D([0], [0], color=PHASE_COLORS[p], lw=2, label=p)
               for p in PHASE_ORDER]
    fig.legend(handles=handles, loc='lower center', ncol=5,
               fontsize=9, bbox_to_anchor=(0.5, 0.0))

    fig.tight_layout(rect=[0, 0.04, 1, 0.96])
    path = os.path.join(out_dir, 'lcr_timeseries_per_point.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [A] Saved: {path}')


# ── Plot B: Mean ± std Cp and Rp per point ────────────────────────────────────

def plot_lcr_summary(stats, out_dir):
    pts = sorted(stats.keys())

    def _bars(ax, vals, color, ylabel, title):
        means = [np.mean(stats[p][vals]) if stats[p][vals] else 0.0 for p in pts]
        stds  = [np.std(stats[p][vals])  if stats[p][vals] else 0.0 for p in pts]
        x     = np.arange(len(pts))
        ax.bar(x, means, yerr=stds, capsize=3, color=color,
               alpha=0.75, ecolor='#333333', error_kw={'lw': 1.2})
        ax.set_xticks(x)
        ax.set_xticklabels([f'P{p:02d}' for p in pts], rotation=45, fontsize=8)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis='y', alpha=0.3)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9), sharex=False)
    _bars(ax1, 'Cp', '#3182CE', 'Cp (pF)', 'Mean ± Std Cp — hold phase')
    _bars(ax2, 'Rp', '#D97706', 'Rp (kΩ)', 'Mean ± Std Rp — hold phase')
    fig.suptitle('LCR Summary per Sensor Point (hold phase, LCR ok only)', fontsize=12)
    fig.tight_layout()
    path = os.path.join(out_dir, 'lcr_summary_bars.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [B] Saved: {path}')


# ── Plot C: Cp vs Rp phase-plane scatter ──────────────────────────────────────

def plot_cp_rp_scatter(rows, out_dir):
    fig, axes = plt.subplots(1, len(PHASE_ORDER), figsize=(20, 5), sharey=False)
    fig.suptitle('Cp vs Rp Scatter — by Phase (each dot = one sample row)',
                 fontsize=11)

    for ax, phase in zip(axes, PHASE_ORDER):
        pr = [r for r in rows if r['phase'] == phase
              and r['lcr_ok']
              and not math.isnan(r['Cp_pF'])
              and not math.isnan(r['Rp_kOhm'])]
        if not pr:
            ax.set_title(phase)
            ax.text(0.5, 0.5, 'no data', ha='center', va='center',
                    transform=ax.transAxes)
            continue

        pts_arr = [r['point'] for r in pr]
        Cp_arr  = [r['Cp_pF']   for r in pr]
        Rp_arr  = [r['Rp_kOhm'] for r in pr]

        cmap   = plt.cm.tab20
        unique = sorted(set(pts_arr))
        cdict  = {p: cmap(i / max(len(unique) - 1, 1)) for i, p in enumerate(unique)}
        colors = [cdict[p] for p in pts_arr]

        ax.scatter(Cp_arr, Rp_arr, c=colors, s=8, alpha=0.6, edgecolors='none')
        ax.set_xlabel('Cp (pF)', fontsize=9)
        ax.set_ylabel('Rp (kΩ)', fontsize=9)
        ax.set_title(phase, fontsize=10,
                     color=PHASE_COLORS.get(phase, '#333333'))
        ax.grid(alpha=0.3)

    # Common colour legend for points
    unique_pts = sorted({r['point'] for r in rows})
    cmap = plt.cm.tab20
    handles = [matplotlib.lines.Line2D([0], [0], marker='o', color='w', ms=6,
                markerfacecolor=cmap(i / max(len(unique_pts)-1, 1)),
                label=f'P{p:02d}')
               for i, p in enumerate(unique_pts)]
    fig.legend(handles=handles, ncol=6, fontsize=7,
               loc='lower center', bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=[0, 0.06, 1, 0.95])
    path = os.path.join(out_dir, 'lcr_cp_rp_scatter.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [C] Saved: {path}')


# ── Plot D: Sample drift — Cp mean vs session order ───────────────────────────

def plot_sample_drift(groups, out_dir):
    """
    For each point, plot the mean Cp (hold phase) vs sample_idx.
    Shows whether capacitance drifts over the measurement session.
    """
    fig, ax = plt.subplots(figsize=(12, 6))

    pts    = sorted(POINTS_XY.keys())
    cmap   = plt.cm.tab20
    cdict  = {p: cmap(i / max(len(pts)-1, 1)) for i, p in enumerate(pts)}

    for pt in pts:
        sample_means = defaultdict(list)
        for (pt2, _, sample_idx), rows_g in groups.items():
            if pt2 != pt:
                continue
            hold_Cp = [r['Cp_pF'] for r in rows_g
                       if r['phase'] == 'hold' and r['lcr_ok']
                       and not math.isnan(r['Cp_pF'])]
            if hold_Cp:
                sample_means[sample_idx].append(np.mean(hold_Cp))

        if not sample_means:
            continue
        xs = sorted(sample_means.keys())
        ys = [np.mean(sample_means[x]) for x in xs]
        ax.plot(xs, ys, marker='o', ms=5, lw=1.2,
                color=cdict[pt], label=f'P{pt:02d}', alpha=0.8)

    ax.set_xlabel('Sample index (collection order)', fontsize=10)
    ax.set_ylabel('Mean Cp (pF)  — hold phase', fontsize=10)
    ax.set_title('Cp Drift Over Session — per Point', fontsize=11)
    ax.legend(ncol=4, fontsize=7, loc='upper right')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, 'lcr_sample_drift.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [D] Saved: {path}')


# ── Plot E: Coefficient of variation (CV) heatmap on sensor layout ────────────

def plot_cv_map(stats, out_dir):
    """
    Two side-by-side sensor-layout maps:
      Left:  Mean Cp (pF) — same as analyze script but LCR-focused
      Right: CV of Cp (std/mean × 100%) — measurement stability
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle('Sensor Layout — Cp Mean and Stability (CV)', fontsize=12)

    pts = sorted(POINTS_XY.keys())

    means = {p: (np.mean(stats[p]['Cp']) if stats.get(p, {}).get('Cp') else float('nan'))
             for p in pts}
    cvs   = {p: (np.std(stats[p]['Cp']) / np.mean(stats[p]['Cp']) * 100
                 if stats.get(p, {}).get('Cp') and np.mean(stats[p]['Cp']) != 0
                 else float('nan'))
             for p in pts}

    def _draw_map(ax, val_dict, cmap_name, label, fmt='.1f'):
        valid = [v for v in val_dict.values() if not math.isnan(v)]
        if not valid:
            ax.text(0.5, 0.5, 'no data', ha='center', va='center',
                    transform=ax.transAxes)
            return
        norm = Normalize(vmin=min(valid), vmax=max(valid))
        cmap = plt.get_cmap(cmap_name)
        for pt in pts:
            x, y  = POINTS_XY[pt]
            v     = val_dict[pt]
            color = cmap(norm(v)) if not math.isnan(v) else (0.85, 0.85, 0.85, 1)
            circ  = plt.Circle((x, y), 3.3, color=color, ec='white', lw=0.8)
            ax.add_patch(circ)
            txt   = f'P{pt}\n{v:{fmt}}' if not math.isnan(v) else f'P{pt}\n—'
            ax.text(x, y, txt, ha='center', va='center',
                    fontsize=6.5, color='white', fontweight='bold')
        sm   = ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        cbar = fig.colorbar(sm, ax=ax, shrink=0.65)
        cbar.set_label(label)
        ax.set_xlim(-22, 22); ax.set_ylim(-20, 20)
        ax.set_aspect('equal')
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')
        ax.grid(True, alpha=0.15)

    _draw_map(ax1, means, 'plasma', 'Mean Cp (pF)', fmt='.2f')
    ax1.set_title('Mean Cp (pF) — hold phase')
    _draw_map(ax2, cvs, 'RdYlGn_r', 'CV (%)', fmt='.1f')
    ax2.set_title('CV (%) = σ/μ×100 — hold phase  (lower = more stable)')

    fig.tight_layout()
    path = os.path.join(out_dir, 'lcr_cv_map.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [E] Saved: {path}')


# ── Plot F: Cp histogram per point ────────────────────────────────────────────

def plot_cp_histograms(stats, out_dir):
    n_cols = 5
    n_rows = 4
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(20, 13))
    fig.suptitle('Cp Distribution per Point — hold phase (LCR ok only)', fontsize=12)

    ax_flat = axes.flatten()
    for ax in ax_flat:
        ax.set_visible(False)

    for ax, pt in zip(ax_flat, sorted(POINTS_XY)):
        ax.set_visible(True)
        vals = stats.get(pt, {}).get('Cp', [])
        if not vals:
            ax.set_title(f'P{pt:02d} — no data', fontsize=8)
            continue
        ax.hist(vals, bins=max(5, len(vals)//3), color='steelblue',
                edgecolor='white', linewidth=0.5, alpha=0.8)
        mu  = np.mean(vals)
        sd  = np.std(vals)
        ax.axvline(mu, color='#E63946', lw=1.5, label=f'μ={mu:.2f}')
        ax.axvline(mu - sd, color='#F4A261', lw=1.0, ls='--')
        ax.axvline(mu + sd, color='#F4A261', lw=1.0, ls='--')
        ax.set_title(f'P{pt:02d}  n={len(vals)}\nμ={mu:.2f}  σ={sd:.2f} pF',
                     fontsize=7.5)
        ax.tick_params(labelsize=6)
        ax.set_xlabel('Cp (pF)', fontsize=6.5)
        ax.set_ylabel('count',   fontsize=6.5)
        ax.legend(fontsize=6, handlelength=1)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(out_dir, 'lcr_cp_histograms.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [F] Saved: {path}')


# ── Plot G: Rp heatmap on sensor layout ───────────────────────────────────────

def plot_rp_map(stats, out_dir):
    pts   = sorted(POINTS_XY.keys())
    means = {p: (np.mean(stats[p]['Rp']) if stats.get(p, {}).get('Rp') else float('nan'))
             for p in pts}
    valid = [v for v in means.values() if not math.isnan(v)]
    if not valid:
        print('  [G] No Rp data to plot.')
        return

    norm = Normalize(vmin=min(valid), vmax=max(valid))
    cmap = plt.cm.coolwarm

    fig, ax = plt.subplots(figsize=(8, 7))
    for pt in pts:
        x, y  = POINTS_XY[pt]
        v     = means[pt]
        color = cmap(norm(v)) if not math.isnan(v) else (0.85, 0.85, 0.85, 1)
        circ  = plt.Circle((x, y), 3.3, color=color, ec='white', lw=0.8)
        ax.add_patch(circ)
        txt = f'P{pt}\n{v:.0f}' if not math.isnan(v) else f'P{pt}\n—'
        ax.text(x, y, txt, ha='center', va='center',
                fontsize=6.5, color='white', fontweight='bold')

    sm = ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, shrink=0.7)
    cbar.set_label('Mean Rp (kΩ) — hold phase')

    ax.set_xlim(-22, 22); ax.set_ylim(-20, 20)
    ax.set_aspect('equal')
    ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')
    ax.set_title('Sensor Layout — Mean Rp (kΩ) per Point (hold phase)')
    ax.grid(True, alpha=0.15)
    fig.tight_layout()
    path = os.path.join(out_dir, 'lcr_rp_map.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'  [G] Saved: {path}')


# ── Post-collection main ───────────────────────────────────────────────────────

def run_file(csv_path, no_show=False):
    print(f'\n  Loading: {csv_path}')
    rows = load_csv(csv_path)
    if not rows:
        print('  No valid rows — check the CSV path and format.')
        return

    print(f'  Rows: {len(rows)}')
    lcr_ok_pct = 100 * sum(1 for r in rows if r['lcr_ok']) / max(len(rows), 1)
    print(f'  LCR ok: {lcr_ok_pct:.1f}%')

    groups = _group_indentations(rows)
    stats  = _hold_stats(rows)

    base    = os.path.splitext(os.path.basename(csv_path))[0]
    out_dir = os.path.normpath(
        os.path.join(os.path.dirname(csv_path), '..', 'plots', f'lcr_{base}'))
    os.makedirs(out_dir, exist_ok=True)
    print(f'  Output directory: {out_dir}\n')

    plot_lcr_timeseries(groups, out_dir)
    plot_lcr_summary(stats, out_dir)
    plot_cp_rp_scatter(rows, out_dir)
    plot_sample_drift(groups, out_dir)
    plot_cv_map(stats, out_dir)
    plot_cp_histograms(stats, out_dir)
    plot_rp_map(stats, out_dir)

    if not no_show:
        for fname in sorted(os.listdir(out_dir)):
            if fname.endswith('.png'):
                img = plt.imread(os.path.join(out_dir, fname))
                fig, ax = plt.subplots()
                ax.imshow(img)
                ax.axis('off')
                ax.set_title(fname)
        plt.show()

    print('\n  All plots saved.')


# ═══════════════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════════════

def _pick_port():
    try:
        from lcr6100 import list_ports
    except ImportError:
        return input('  LCR serial port > ').strip()

    ports = list_ports()
    if not ports:
        return input('  No ports detected. Enter port path > ').strip()
    print('\n  Available serial ports:')
    for i, (dev, desc) in enumerate(ports):
        print(f'    [{i}]  {dev}  —  {desc}')
    while True:
        raw = input(f'  Select [0–{len(ports)-1}] or type path > ').strip()
        if raw.startswith('/') or raw.upper().startswith('COM'):
            return raw
        try:
            return ports[int(raw)][0]
        except (ValueError, IndexError):
            pass


def _pick_csv():
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    csvs    = sorted(glob.glob(os.path.join(log_dir, 'capacitance_dataset_*.csv')))
    if csvs:
        print('\n  Dataset files:')
        for i, p in enumerate(csvs):
            print(f'    [{i}]  {os.path.basename(p)}')
        try:
            idx = int(input(f'  Select [0–{len(csvs)-1}] > ').strip())
            if 0 <= idx < len(csvs):
                return csvs[idx]
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
    return input('  Path to CSV > ').strip()


def parse_args():
    p = argparse.ArgumentParser(
        description='LCR-6100 live + post-collection visualizer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--live',     action='store_true',
                   help='Run live scrolling plot (requires LCR connected)')
    p.add_argument('--port',     default=None,
                   help='Serial port for live mode (default: interactive)')
    p.add_argument('--baud',     type=int, default=None,
                   help='Baud rate override (default: 9600)')
    p.add_argument('--file',     default=None, metavar='CSV',
                   help='Dataset CSV for post-collection analysis')
    p.add_argument('--no-show',  action='store_true',
                   help='Save plots but do not display them interactively')
    p.add_argument('--window',   type=float, default=LIVE_WINDOW_S,
                   help=f'Live scrolling window (s, default: {LIVE_WINDOW_S})')
    return p.parse_args()


def main():
    args = parse_args()

    if not args.live and args.file is None:
        # Interactive mode selection
        print('=' * 60)
        print('  LCR-6100 Visualizer')
        print('=' * 60)
        print('  [1]  Live real-time plot')
        print('  [2]  Post-collection analysis from CSV')
        print('  [3]  Both')
        choice = input('\n  Select [1/2/3] > ').strip()
        if choice == '1':
            args.live = True
        elif choice == '2':
            args.file = _pick_csv()
        elif choice == '3':
            args.live = True
            args.file = _pick_csv()
        else:
            print('  Invalid choice.')
            return

    if args.file and not args.live:
        # File-only: can run without display backend issues
        if not args.no_show:
            matplotlib.use('TkAgg')
        run_file(args.file, no_show=args.no_show)

    elif args.live and not args.file:
        # Live-only
        matplotlib.use('TkAgg')
        port = args.port or _pick_port()
        run_live(port, baud=args.baud)

    else:
        # Both: run file analysis first, then live
        if not args.no_show:
            matplotlib.use('TkAgg')
        if args.file:
            run_file(args.file, no_show=args.no_show)
        port = args.port or _pick_port()
        run_live(port, baud=args.baud)


if __name__ == '__main__':
    main()
