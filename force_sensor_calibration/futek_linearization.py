#!/usr/bin/env python3
"""
futek_linearization.py
──────────────────────────────────────────────────────────────────────────────
Analyses the fzcal_futek_direct_* calibration datasets to determine the
correct voltage-to-force coefficient for the FUTEK load cell (AI0 channel).

Datasets: 5 g, 10 g, 20 g, 50 g, 100 g, 200 g known weights applied
directly on top of the sensor.

Outputs (saved to force_sensor_calibration/plots/):
  1. voltage_vs_time.png   — ai0 vs time for every weight run
  2. voltage_vs_force.png  — ΔV vs applied force, linear fit vs old coeff
  3. force_comparison.png  — true vs estimated force (new fit & old formula)
  4. residuals.png         — estimation error per method per weight

Usage
-----
  python futek_linearization.py
"""

import os
import csv
import json
import glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import stats

# ── Paths ─────────────────────────────────────────────────────────────────────
HERE    = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, 'logs')
OUT_DIR = os.path.join(HERE, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

# ── Physical constants ─────────────────────────────────────────────────────────
G = 9.80665  # m/s²

# ── Existing (old) coefficient from the codebase ──────────────────────────────
# LOADCELL_MAX_N = 10 * 4.44822 N,  supply span = 5 V
# Formula: F = -(ai0 - 5.0) * K_OLD  →  for ΔV: ΔF = -ΔV * K_OLD
K_OLD     = (10.0 * 4.44822) / 5.0   # 8.8964 N/V
V_ZERO_OLD = 5.0                       # assumed zero-force voltage

matplotlib.rcParams.update({
    'figure.facecolor': 'white',
    'axes.facecolor':   'white',
    'font.family':      'DejaVu Sans',
    'axes.spines.top':  False,
    'axes.spines.right':False,
})

WEIGHT_COLORS = {
    5:   '#1f77b4',
    10:  '#ff7f0e',
    20:  '#2ca02c',
    50:  '#d62728',
    100: '#9467bd',
    200: '#8c564b',
}


# ── Data loading ──────────────────────────────────────────────────────────────
def load_dataset(meta_path):
    with open(meta_path) as f:
        meta = json.load(f)
    csv_path = meta_path.replace('_meta.json', '.csv')
    rows = []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            rows.append({
                'timestamp': float(row['timestamp']),
                'loaded':    int(row['loaded']),
                'ai0':       float(row['ai0']),
                'fz':        float(row['fz']),
                'weight_g':  float(row['weight_g']),
            })
    rows.sort(key=lambda r: r['timestamp'])
    t0 = rows[0]['timestamp']
    for r in rows:
        r['t_rel'] = r['timestamp'] - t0
    return meta, rows


def per_dataset_stats(meta, rows):
    """Return baseline voltage, loaded voltage, ΔV, true force, old-formula force."""
    ai0_unloaded = np.array([r['ai0'] for r in rows if r['loaded'] == 0])
    ai0_loaded   = np.array([r['ai0'] for r in rows if r['loaded'] == 1])
    fz_loaded    = np.array([r['fz']  for r in rows if r['loaded'] == 1])

    V_base   = ai0_unloaded.mean()
    V_load   = ai0_loaded.mean()
    dV       = V_load - V_base
    tilt_rad = np.deg2rad(meta['tilt_from_vertical_deg'])
    F_true   = meta['weight_g'] * G / 1000.0 * np.cos(tilt_rad)

    # old formula: offset from 5 V, negative sign
    F_old_base = -(V_base - V_ZERO_OLD) * K_OLD
    F_old_load = -(V_load - V_ZERO_OLD) * K_OLD
    dF_old     = F_old_load - F_old_base   # delta using old coefficient

    return {
        'weight_g':  meta['weight_g'],
        'V_base':    V_base,
        'V_load':    V_load,
        'dV':        dV,
        'F_true_N':  F_true,
        'dF_old':    dF_old,         # signed; old formula gives negative dF
        'fz_mean':   fz_loaded.mean(),
        'fz_std':    fz_loaded.std(),
        'ai0_std':   ai0_loaded.std(),
    }


# ── Linear regression ─────────────────────────────────────────────────────────
def fit_sensitivity(stats_list):
    """
    Fit F_true = k * dV  (through origin, and with intercept).
    Returns slope k, intercept b, R².
    """
    dV     = np.array([s['dV']       for s in stats_list])
    F_true = np.array([s['F_true_N'] for s in stats_list])

    # through origin
    k_origin = np.dot(dV, F_true) / np.dot(dV, dV)

    # with intercept
    slope, intercept, r, p, se = stats.linregress(dV, F_true)

    # R² for through-origin fit
    ss_res = np.sum((F_true - k_origin * dV) ** 2)
    ss_tot = np.sum((F_true - F_true.mean()) ** 2)
    r2_origin = 1 - ss_res / ss_tot

    return k_origin, slope, intercept, r**2, r2_origin


# ── Plot 1: Voltage vs time ───────────────────────────────────────────────────
def plot_voltage_vs_time(datasets, outdir):
    n     = len(datasets)
    ncols = 3
    nrows = -(-n // ncols)   # ceil division

    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(ncols * 6, nrows * 3.5),
                             sharey=True)
    axes_flat = axes.flatten()

    for ax_i, (meta, rows) in enumerate(datasets):
        ax   = axes_flat[ax_i]
        wg   = meta['weight_g']
        col  = WEIGHT_COLORS.get(int(wg), '#333')
        t    = np.array([r['t_rel'] for r in rows])
        ai0  = np.array([r['ai0']   for r in rows])
        load = np.array([r['loaded'] for r in rows])

        # shade loaded window
        in_load = False
        t_start = 0.0
        for i, (ti, li) in enumerate(zip(t, load)):
            if li == 1 and not in_load:
                t_start = ti
                in_load = True
            elif li == 0 and in_load:
                ax.axvspan(t_start, ti, color='#ffecb3', alpha=0.6,
                           label='Loaded' if t_start == t[load == 1][0] else None)
                in_load = False
        if in_load:
            ax.axvspan(t_start, t[-1], color='#ffecb3', alpha=0.6)

        ax.plot(t, ai0, color=col, linewidth=0.9, alpha=0.95)

        # mean lines
        V_base = np.array([r['ai0'] for r in rows if r['loaded'] == 0]).mean()
        V_load = np.array([r['ai0'] for r in rows if r['loaded'] == 1]).mean()
        ax.axhline(V_base, color='steelblue', linewidth=1.0,
                   linestyle='--', alpha=0.8, label=f'V_base={V_base:.4f} V')
        ax.axhline(V_load, color='#e07000', linewidth=1.0,
                   linestyle='--', alpha=0.8, label=f'V_load={V_load:.4f} V')

        dV    = V_load - V_base
        F_exp = meta['weight_g'] * G / 1000
        ax.set_title(f'{int(wg)} g  —  ΔV = {dV*1000:.2f} mV  '
                     f'(F = {F_exp:.4f} N)',
                     fontsize=9, fontweight='bold')
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.set_ylabel('ai0 (V)', fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=6.5, loc='lower right', framealpha=0.85)

    for ax_i in range(n, len(axes_flat)):
        axes_flat[ax_i].set_visible(False)

    fig.suptitle('FUTEK load cell — AI0 voltage vs time per calibration run\n'
                 'Yellow band = weight applied  |  Dashed = phase mean',
                 fontsize=11, fontweight='bold')
    fig.tight_layout()
    out = os.path.join(outdir, 'voltage_vs_time.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── Plot 2: ΔV vs applied force ───────────────────────────────────────────────
def plot_voltage_vs_force(stats_list, k_origin, slope, intercept, r2, r2_origin, outdir):
    dV     = np.array([s['dV']       for s in stats_list])
    F_true = np.array([s['F_true_N'] for s in stats_list])
    wg     = np.array([s['weight_g'] for s in stats_list])
    ai0_std = np.array([s['ai0_std']  for s in stats_list])

    fig, ax = plt.subplots(figsize=(8, 5.5))

    # scatter per weight
    for s in stats_list:
        c = WEIGHT_COLORS.get(int(s['weight_g']), '#333')
        ax.errorbar(s['dV'], s['F_true_N'],
                    xerr=s['ai0_std'], fmt='o', color=c, markersize=8,
                    capsize=4, label=f"{int(s['weight_g'])} g", zorder=4)

    # fit lines
    dV_range = np.linspace(0, dV.max() * 1.05, 200)
    ax.plot(dV_range, k_origin * dV_range,
            color='#1a6eb5', linewidth=2.0, linestyle='-',
            label=f'New fit (origin): k = {k_origin:.4f} N/V  R²={r2_origin:.5f}',
            zorder=3)
    ax.plot(dV_range, slope * dV_range + intercept,
            color='#2ca02c', linewidth=1.5, linestyle='--',
            label=f'New fit (intercept): k = {slope:.4f}, b = {intercept:.5f}  R²={r2:.5f}',
            zorder=3)

    # old coefficient line (negative because old formula: ΔF = -ΔV * K_OLD)
    ax.plot(dV_range, K_OLD * dV_range,
            color='#d62728', linewidth=1.5, linestyle=':',
            label=f'Old coefficient: k = {K_OLD:.4f} N/V (overestimates by ~{K_OLD/k_origin:.2f}×)',
            zorder=2)

    ax.set_xlabel('ΔV = V_loaded − V_baseline  (V)', fontsize=10)
    ax.set_ylabel('Applied force (N)', fontsize=10)
    ax.set_title('Voltage change vs applied force — FUTEK linearization\n'
                 'Error bars = std(ai0) during loaded phase',
                 fontsize=10, fontweight='bold')
    ax.legend(fontsize=8, loc='upper left', framealpha=0.9)
    ax.grid(alpha=0.25)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)

    out = os.path.join(outdir, 'voltage_vs_force.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── Plot 3: Force comparison ──────────────────────────────────────────────────
def plot_force_comparison(stats_list, k_origin, slope, intercept, outdir):
    weights  = np.array([s['weight_g'] for s in stats_list])
    F_true   = np.array([s['F_true_N'] for s in stats_list])
    dV       = np.array([s['dV']       for s in stats_list])

    F_new    = k_origin * dV                   # through-origin fit
    F_new_ic = slope * dV + intercept          # with intercept
    F_old    = K_OLD * dV                      # old magnitude (|ΔF_old|)

    x  = np.arange(len(weights))
    bw = 0.2

    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.bar(x - bw,   F_true,   bw, label='True force',           color='#555', zorder=3)
    ax.bar(x,        F_new,    bw, label=f'New fit (origin) k={k_origin:.3f} N/V',
           color='#1a6eb5', zorder=3)
    ax.bar(x + bw,   F_old,    bw, label=f'Old coeff k={K_OLD:.3f} N/V',
           color='#d62728', alpha=0.85, zorder=3)

    ax.set_xticks(x)
    ax.set_xticklabels([f'{int(w)} g' for w in weights], fontsize=9)
    ax.set_ylabel('Force (N)', fontsize=10)
    ax.set_title('Force comparison: true vs estimated (new fit vs old coefficient)\n'
                 'FUTEK direct-weight calibration',
                 fontsize=10, fontweight='bold')
    ax.legend(fontsize=8, framealpha=0.9)
    ax.grid(axis='y', alpha=0.25)

    out = os.path.join(outdir, 'force_comparison.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── Plot 4: Residuals / error ─────────────────────────────────────────────────
def plot_residuals(stats_list, k_origin, slope, intercept, outdir):
    weights = np.array([s['weight_g'] for s in stats_list])
    F_true  = np.array([s['F_true_N'] for s in stats_list])
    dV      = np.array([s['dV']       for s in stats_list])

    err_new = (k_origin * dV - F_true) * 1000       # mN
    err_new_pct = (k_origin * dV - F_true) / F_true * 100
    err_old = (K_OLD * dV - F_true) * 1000
    err_old_pct = (K_OLD * dV - F_true) / F_true * 100

    x  = np.arange(len(weights))
    bw = 0.3

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # absolute error
    ax1.bar(x - bw/2, err_new, bw, label=f'New fit k={k_origin:.3f} N/V',
            color='#1a6eb5', zorder=3)
    ax1.bar(x + bw/2, err_old, bw, label=f'Old coeff k={K_OLD:.3f} N/V',
            color='#d62728', alpha=0.85, zorder=3)
    ax1.axhline(0, color='black', linewidth=0.8, linestyle=':')
    ax1.set_xticks(x)
    ax1.set_xticklabels([f'{int(w)} g' for w in weights], fontsize=9)
    ax1.set_ylabel('Error (mN)', fontsize=10)
    ax1.set_title('Absolute error  F_estimated − F_true', fontsize=9, fontweight='bold')
    ax1.legend(fontsize=8)
    ax1.grid(axis='y', alpha=0.25)

    # relative error
    ax2.bar(x - bw/2, err_new_pct, bw, label=f'New fit',
            color='#1a6eb5', zorder=3)
    ax2.bar(x + bw/2, err_old_pct, bw, label=f'Old coeff',
            color='#d62728', alpha=0.85, zorder=3)
    ax2.axhline(0, color='black', linewidth=0.8, linestyle=':')
    ax2.set_xticks(x)
    ax2.set_xticklabels([f'{int(w)} g' for w in weights], fontsize=9)
    ax2.set_ylabel('Error (%)', fontsize=10)
    ax2.set_title('Relative error  (F_estimated − F_true) / F_true × 100', fontsize=9, fontweight='bold')
    ax2.legend(fontsize=8)
    ax2.grid(axis='y', alpha=0.25)

    fig.suptitle('FUTEK calibration — estimation error\n'
                 'New fitted coefficient vs old hardcoded coefficient',
                 fontsize=11, fontweight='bold')
    fig.tight_layout()

    out = os.path.join(outdir, 'residuals.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f'  saved: {os.path.relpath(out, HERE)}')


# ── Print summary ─────────────────────────────────────────────────────────────
def print_summary(stats_list, k_origin, slope, intercept, r2, r2_origin):
    print()
    print('=' * 65)
    print('FUTEK LOAD CELL LINEARIZATION SUMMARY')
    print('=' * 65)
    print(f'  Old formula:  F = -(ai0 - {V_ZERO_OLD}) × {K_OLD:.4f}')
    print(f'  Old coeff:    K_OLD = {K_OLD:.4f} N/V')
    print()
    print(f'  New fit (through origin):')
    print(f'    F = (ai0 - V_baseline) × {k_origin:.4f} N/V')
    print(f'    R² = {r2_origin:.6f}')
    print()
    print(f'  New fit (with intercept):')
    print(f'    F = (ai0 - V_baseline) × {slope:.4f} + {intercept:.6f}')
    print(f'    R² = {r2:.6f}')
    print()
    print(f'  Old / New ratio: {K_OLD / k_origin:.3f}× '
          f'(old overestimates force by {(K_OLD/k_origin - 1)*100:.1f}%)')
    print()
    print(f'  {"weight_g":>8}  {"dV (mV)":>9}  {"F_true (N)":>11}  '
          f'{"F_new (N)":>10}  {"F_old (N)":>10}  '
          f'{"err_new%":>9}  {"err_old%":>9}')
    print('  ' + '-'*80)
    for s in stats_list:
        F_new = k_origin * s['dV']
        F_old = K_OLD   * s['dV']
        e_new = (F_new - s['F_true_N']) / s['F_true_N'] * 100
        e_old = (F_old - s['F_true_N']) / s['F_true_N'] * 100
        print(f'  {int(s["weight_g"]):>8}  {s["dV"]*1000:>9.2f}  '
              f'{s["F_true_N"]:>11.5f}  {F_new:>10.5f}  {F_old:>10.5f}  '
              f'{e_new:>+9.2f}  {e_old:>+9.2f}')

    V_baseline_mean = np.mean([s['V_base'] for s in stats_list])
    V_baseline_std  = np.std( [s['V_base'] for s in stats_list])
    print()
    print(f'  Baseline voltage across all runs:')
    print(f'    mean = {V_baseline_mean:.5f} V   std = {V_baseline_std:.5f} V')
    print()
    print('  ► Recommended constants for code:')
    print(f'    AI0_ZERO_V        = {V_baseline_mean:.4f}  # mean unloaded baseline')
    print(f'    LOADCELL_N_PER_V  = {k_origin:.4f}  # fitted sensitivity (N/V)')
    print(f'    # Usage: F = (ai0 - AI0_ZERO_V) * LOADCELL_N_PER_V')
    print('=' * 65)
    print()


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    meta_files = sorted(glob.glob(os.path.join(LOG_DIR, '*_meta.json')))
    if not meta_files:
        raise FileNotFoundError(f'No *_meta.json files found in {LOG_DIR}')

    print(f'Found {len(meta_files)} calibration runs')
    datasets   = [load_dataset(mf) for mf in meta_files]
    stats_list = [per_dataset_stats(meta, rows) for meta, rows in datasets]

    # sort by weight
    order      = np.argsort([s['weight_g'] for s in stats_list])
    stats_list = [stats_list[i] for i in order]
    datasets   = [datasets[i]   for i in order]

    k_origin, slope, intercept, r2, r2_origin = fit_sensitivity(stats_list)

    print_summary(stats_list, k_origin, slope, intercept, r2, r2_origin)

    print(f'Generating figures → {os.path.relpath(OUT_DIR, HERE)}/')
    plot_voltage_vs_time(datasets, OUT_DIR)
    plot_voltage_vs_force(stats_list, k_origin, slope, intercept, r2, r2_origin, OUT_DIR)
    plot_force_comparison(stats_list, k_origin, slope, intercept, OUT_DIR)
    plot_residuals(stats_list, k_origin, slope, intercept, OUT_DIR)
    print('\nDone.')


if __name__ == '__main__':
    main()
