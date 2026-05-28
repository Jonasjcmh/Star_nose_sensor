"""
analyze_session.py
Post-processing dashboard for KYWO sensor sessions.

Usage:
  python3.10 analyze_session.py                          # latest session
  python3.10 analyze_session.py ecoflex_flat_layer       # partial name match
  python3.10 analyze_session.py --all                    # compare all sessions
  python3.10 analyze_session.py --save                   # save figures
  python3.10 analyze_session.py --force                  # force plots only
"""

import os
import sys
import glob
import argparse
import math
from datetime import datetime

import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import RegularPolygon
from matplotlib.cm import ScalarMappable

# ── Paths ─────────────────────────────────────────────────────
INTEGRATION_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR        = os.path.join(INTEGRATION_DIR, "logs")
PLOTS_DIR       = os.path.join(INTEGRATION_DIR, "plots")
LEGACY_DATASETS_DIR = os.path.join(INTEGRATION_DIR, "datasets")
LEGACY_LOG_DIR  = os.path.expanduser("~/sofa-projects/logs")

# ── Sensor layout ─────────────────────────────────────────────
POINTS_MM = [
    (-8,  +14), ( 0, +14), (+8, +14),
    (-12,  +7), (-4,  +7), (+4,  +7), (+12, +7),
    (-16,   0), (-8,   0), ( 0,   0), (+8,   0), (+16, 0),
    (-12,  -7), (-4,  -7), (+4,  -7), (+12, -7),
    (-8,  -14), ( 0, -14), (+8, -14),
]
RAW_CELLS  = [2,15,28,1,14,27,40,0,13,26,39,52,12,25,38,51,24,37,50]
N          = 19
# Corrected: sensor mounted 120° CCW → UR5 point i fires cell at rotated position
UR5_TO_IDX = {
    1:16,  2:12,  3:7,
    4:17,  5:13,  6:8,   7:3,
    8:18,  9:14,  10:9,  11:4,  12:0,
    13:15, 14:10, 15:5,  16:1,
    17:11, 18:6,  19:2,
}
IDX_TO_UR5 = {v: k for k, v in UR5_TO_IDX.items()}
# Unique points in robot visit order (for force plot aggregation)
VISIT_ORDER    = [10,1,2,3,7,6,5,4,8,9,11,12,16,15,14,13,17,18,19]
# Full sequence including repeated P10 visits, as (point, visit_index) tuples
VISIT_SEQUENCE = [
    (10,0),(1,0),(2,0),(3,0),(7,0),(6,0),(5,0),(4,0),
    (8,0),(9,0),(10,1),(11,0),(12,0),(16,0),(15,0),(14,0),(13,0),
    (17,0),(18,0),(19,0),(10,2),
]

CMAP = LinearSegmentedColormap.from_list('star_nose', [
    '#2ab5a0', '#33e666', '#ffe619', '#ff7300', '#dc0000'
])

# ── Args ──────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('file',    nargs='?',
                   help='CSV filename or partial label name')
    p.add_argument('--all',   action='store_true',
                   help='Compare all sessions')
    p.add_argument('--save',  action='store_true',
                   help='Save figures to plots folder')
    p.add_argument('--force', action='store_true',
                   help='Show force analysis only')
    return p.parse_args()

# ── File helpers ──────────────────────────────────────────────
def find_all_csvs():
    """Find all CSVs in logs folder + legacy locations."""
    files = sorted(glob.glob(
        os.path.join(LOGS_DIR, '*.csv')))
    legacy_datasets = sorted(glob.glob(
        os.path.join(LEGACY_DATASETS_DIR, '*.csv')))
    legacy = sorted(glob.glob(
        os.path.join(LEGACY_LOG_DIR, '*.csv')))
    seen = set(os.path.basename(f) for f in files)
    for f in legacy_datasets + legacy:
        if os.path.basename(f) not in seen:
            files.append(f)
            seen.add(os.path.basename(f))
    return sorted(files)

def find_csv(arg=None):
    files = find_all_csvs()
    if not files:
        print(f"[analyze] No CSV files found in {LOGS_DIR}")
        sys.exit(1)
    if arg is None:
        return files[-1]
    matches = [f for f in files
               if os.path.basename(f) == arg
               or arg in os.path.basename(f)]
    return matches[-1] if matches else files[-1]

def get_save_dir(csv_path):
    """
    Save figures in:
      plots/<session_name>/
    Creates folder if needed.
    """
    name     = os.path.basename(csv_path).replace('.csv', '')
    save_dir = os.path.join(PLOTS_DIR, name)
    os.makedirs(save_dir, exist_ok=True)
    return save_dir

def get_dataset_label(csv_path):
    """Extract human label from filename."""
    base = os.path.basename(csv_path).replace('.csv', '')
    if '_session_' in base:
        return base.split('_session_', 1)[0]
    # Format: session_YYYYMMDD_HHMMSS_label
    parts = base.split('_', 3)
    if len(parts) >= 4:
        return parts[3]          # user label
    elif len(parts) == 3:
        return parts[2]          # legacy: just timestamp
    return base

# ── Data loading ──────────────────────────────────────────────
def load_session(path):
    print(f"[analyze] Loading : {os.path.basename(path)}")
    df = pd.read_csv(path)

    df['t']        = df['timestamp'] - df['timestamp'].iloc[0]
    df['datetime'] = pd.to_datetime(df['timestamp'], unit='s')

    df['ur5_point']    = pd.to_numeric(
        df['ur5_point'], errors='coerce')
    df['ur5_pressing'] = pd.to_numeric(
        df['ur5_pressing'], errors='coerce').fillna(0).astype(int)
    df['ur5_done']     = pd.to_numeric(
        df['ur5_done'], errors='coerce').fillna(0).astype(int)

    cell_cols = [f'cell_{i+1}' for i in range(N)]
    for c in cell_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(
                df[c], errors='coerce').fillna(0)

    for c in ['fx','fy','fz','tx','ty','tz',
              'tcp_x','tcp_y','tcp_z','ai0']:
        if c in df.columns:
            df[c] = pd.to_numeric(
                df[c], errors='coerce').fillna(0)

    has_force = 'fz' in df.columns
    has_tcp   = 'tcp_z' in df.columns
    dur       = df['t'].iloc[-1]
    rate      = len(df) / dur if dur > 0 else 0
    label     = get_dataset_label(path)

    t_start = datetime.fromtimestamp(df['timestamp'].iloc[0])
    print(f"[analyze] Label   : {label}")
    print(f"[analyze] Start   : {t_start.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"[analyze] Duration: {dur:.1f}s")
    print(f"[analyze] Rows    : {len(df):,}  |  Rate: {rate:.1f} Hz")
    print(f"[analyze] Force   : {'YES' if has_force else 'NO'}")
    print(f"[analyze] TCP pose: {'YES' if has_tcp else 'NO'}")
    return df

def get_press_events(df):
    cell_cols   = [f'cell_{i+1}' for i in range(N)]
    events      = []
    in_press    = False
    rows        = []
    pt          = None
    visit_count = {}   # how many times each point has been pressed so far

    for _, row in df.iterrows():
        if row['ur5_pressing'] == 1:
            if not in_press:
                in_press = True
                pt       = row['ur5_point']
                rows     = []
            rows.append(row)
        else:
            if in_press and rows:
                arr  = np.array([r[cell_cols].values.astype(float)
                                 for r in rows])
                peak = arr.max(axis=0)
                mean = arr.mean(axis=0)
                pt_i = int(pt) if not (
                    isinstance(pt, float) and math.isnan(pt)) else -1
                ti    = UR5_TO_IDX.get(pt_i, -1)
                visit = visit_count.get(pt_i, 0)
                visit_count[pt_i] = visit + 1

                fz_v = ([r['fz'] for r in rows]
                        if 'fz' in df.columns else [])
                fx_v = ([r['fx'] for r in rows]
                        if 'fx' in df.columns else [])
                fy_v = ([r['fy'] for r in rows]
                        if 'fy' in df.columns else [])
                ai0_v = ([float(r['ai0']) for r in rows]
                         if 'ai0' in df.columns else [])

                events.append({
                    'point':        pt_i,
                    'visit':        visit,
                    'start':        rows[0]['t'],
                    'duration':     len(rows) * 0.05,
                    'n_frames':     len(rows),
                    'peak':         peak,
                    'mean':         mean,
                    'peak_max':     float(peak.max()),
                    'mean_max':     float(mean.max()),
                    'target_idx':   ti,
                    'target_peak':  float(peak[ti])
                                    if 0 <= ti < N else 0.0,
                    'fz_mean':      float(np.mean(np.abs(fz_v)))
                                    if fz_v else 0.0,
                    'fz_peak':      float(np.max(np.abs(fz_v)))
                                    if fz_v else 0.0,
                    'fx_mean':      float(np.mean(np.abs(fx_v)))
                                    if fx_v else 0.0,
                    'fy_mean':      float(np.mean(np.abs(fy_v)))
                                    if fy_v else 0.0,
                    'ai0_mean':     float(np.mean(ai0_v))
                                    if ai0_v else 0.0,
                    'ai0_peak':     float(np.max(ai0_v))
                                    if ai0_v else 0.0,
                })
            in_press = False
            rows     = []

    print(f"[analyze] Events  : {len(events)} press events found")
    return events

# ── Save helper ───────────────────────────────────────────────
def savefig(fig, csv_path, suffix, save):
    if save:
        save_dir = get_save_dir(csv_path)
        path     = os.path.join(save_dir, f"{suffix}.png")
        fig.savefig(path, dpi=150, bbox_inches='tight',
                    facecolor='white')
        print(f"[analyze] Saved   → {path}")

# ── Hex map helper ────────────────────────────────────────────
def _hex_map(ax, values_19, target_idx=-1, title='',
             vmax=1.0, unit=''):
    vmax = max(vmax, 1e-6)
    for i, (xmm, ymm) in enumerate(POINTS_MM):
        v   = float(values_19[i]) / vmax \
              if i < len(values_19) else 0.0
        v   = max(0.0, min(1.0, v))
        col = CMAP(v)
        lw  = 2.5 if i == target_idx else 0.5
        ec  = 'red' if i == target_idx else 'white'
        h   = RegularPolygon(
            (xmm, ymm), numVertices=6, radius=4.5,
            facecolor=col, edgecolor=ec, linewidth=lw)
        ax.add_patch(h)
        val = values_19[i] if i < len(values_19) else 0
        txt = (f"{val:.2f}{unit}"
               if abs(float(val)) > 0.01 else "")
        ax.text(xmm, ymm, txt,
                ha='center', va='center', fontsize=5,
                color='white' if v > 0.45 else '#222')
    ax.set_xlim(-22, 22)
    ax.set_ylim(-20, 20)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title(title, fontsize=9, fontweight='bold', pad=4)
    sm = ScalarMappable(cmap=CMAP,
                        norm=Normalize(0, vmax))
    sm.set_array([])
    plt.colorbar(sm, ax=ax, shrink=0.55, pad=0.02,
                 label=f'0 – {vmax:.2f}{unit}')

# ── Plot 1: Overview ──────────────────────────────────────────
def plot_overview(df, events, csv_path, save=False):
    cell_cols = [f'cell_{i+1}' for i in range(N)]
    name      = os.path.basename(csv_path).replace('.csv', '')
    label     = get_dataset_label(csv_path)
    dur       = df['t'].iloc[-1]
    rate      = len(df) / dur if dur > 0 else 0
    t_start   = datetime.fromtimestamp(
        df['timestamp'].iloc[0])

    fig = plt.figure(figsize=(20, 12))
    fig.suptitle(
        f"Session overview — {label}\n"
        f"{t_start.strftime('%Y-%m-%d %H:%M:%S')}  "
        f"duration={dur:.1f}s  "
        f"rows={len(df):,}  "
        f"rate={rate:.1f}Hz",
        fontsize=12, fontweight='bold', y=0.99)

    gs = gridspec.GridSpec(3, 3, figure=fig,
                           hspace=0.50, wspace=0.38)

    # Timeline heatmap
    ax1 = fig.add_subplot(gs[0, :])
    data = df[cell_cols].values.T
    im   = ax1.imshow(data, aspect='auto', cmap=CMAP,
                      vmin=0, vmax=1,
                      extent=[0, dur, N+0.5, 0.5])
    ax1.set_xlabel('Time (s)', fontsize=9)
    ax1.set_ylabel('Sensor cell', fontsize=9)
    ax1.set_title('All 19 cells — full session', fontsize=10)
    ax1.set_yticks(range(1, N+1))
    ax1.set_yticklabels(
        [f'P{i} S{RAW_CELLS[i-1]}' for i in range(1, N+1)],
        fontsize=6)
    plt.colorbar(im, ax=ax1, label='Pressure', shrink=0.9)
    valid = [e for e in events if e['point'] > 0]
    for ev in valid:
        ax1.axvline(ev['start'], color='white',
                   alpha=0.5, linewidth=0.8, linestyle='--')
        ax1.text(ev['start']+0.1, 0.4,
                f"P{ev['point']}", color='white',
                fontsize=5, rotation=90, va='bottom')

    # Peak per event
    ax2 = fig.add_subplot(gs[1, 0])
    if valid:
        x     = range(len(valid))
        peaks = [e['peak_max']    for e in valid]
        tgts  = [e['target_peak'] for e in valid]
        ax2.bar(x, peaks, color='#2ab5a0', alpha=0.85,
                label='Max any cell')
        ax2.bar(x, tgts, color='#dc0000', alpha=0.9,
                label='Target cell')
        ax2.set_xticks(list(x))
        ax2.set_xticklabels(
            [f"P{e['point']}" for e in valid],
            rotation=60, fontsize=6)
        ax2.set_ylabel('Peak pressure')
        ax2.set_title('Peak per press event')
        ax2.legend(fontsize=7)
        ax2.set_ylim(0, 1.1)
        ax2.grid(axis='y', alpha=0.3)

    # Hex map peak
    ax3 = fig.add_subplot(gs[1, 1])
    peak_per_cell = np.zeros(N)
    for ev in valid:
        ti = ev['target_idx']
        if ti >= 0:
            peak_per_cell[ti] = max(
                peak_per_cell[ti], ev['target_peak'])
    _hex_map(ax3, peak_per_cell,
             title='Peak pressure per cell')

    # Hex map mean
    ax4 = fig.add_subplot(gs[1, 2])
    mean_pc  = np.zeros(N)
    count_pc = np.zeros(N)
    for ev in valid:
        ti = ev['target_idx']
        if ti >= 0:
            mean_pc[ti]  += ev['target_peak']
            count_pc[ti] += 1
    mask = count_pc > 0
    mean_pc[mask] /= count_pc[mask]
    _hex_map(ax4, mean_pc, title='Mean pressure per cell')

    # Correlation matrix
    ax5 = fig.add_subplot(gs[2, 0])
    press_df = df[df['ur5_pressing']==1][cell_cols]
    if len(press_df) > 10:
        corr = press_df.corr()
        im5  = ax5.imshow(corr, cmap='RdBu_r',
                          vmin=-1, vmax=1)
        ax5.set_title('Cell correlation (pressing)',
                      fontsize=9)
        ax5.set_xticks(range(N))
        ax5.set_yticks(range(N))
        ax5.set_xticklabels(
            [f'P{i+1}' for i in range(N)],
            rotation=90, fontsize=5)
        ax5.set_yticklabels(
            [f'P{i+1}' for i in range(N)], fontsize=5)
        plt.colorbar(im5, ax=ax5, shrink=0.7)

    # Duration histogram
    ax6 = fig.add_subplot(gs[2, 1])
    if valid:
        durs = [e['duration'] for e in valid]
        ax6.hist(durs, bins=min(15, len(durs)),
                 color='#2ab5a0', edgecolor='white',
                 alpha=0.85)
        ax6.axvline(np.mean(durs), color='#dc0000',
                   linestyle='--',
                   label=f'Mean={np.mean(durs):.2f}s')
        ax6.set_xlabel('Duration (s)', fontsize=9)
        ax6.set_ylabel('Count', fontsize=9)
        ax6.set_title('Press duration distribution',
                      fontsize=9)
        ax6.legend(fontsize=8)
        ax6.grid(alpha=0.3)

    # Summary stats
    ax7 = fig.add_subplot(gs[2, 2])
    ax7.axis('off')
    has_ft  = 'fz' in df.columns
    has_ai0 = 'ai0' in df.columns and df['ai0'].abs().max() > 1e-6
    avg_pk  = np.mean([e['peak_max']    for e in valid]) if valid else 0
    avg_tg  = np.mean([e['target_peak'] for e in valid]) if valid else 0
    avg_fz  = (np.mean([e['fz_mean'] for e in valid])
               if (valid and has_ft)  else None)
    avg_ai0 = (np.mean([e.get('ai0_mean', 0.0) for e in valid])
               if (valid and has_ai0) else None)

    stats = [
        ("Dataset",       label[:28]),
        ("Start",         t_start.strftime('%Y-%m-%d %H:%M:%S')),
        ("Duration",      f"{dur:.1f} s"),
        ("Frames",        f"{len(df):,}"),
        ("Sample rate",   f"{rate:.1f} Hz"),
        ("Press events",  str(len(valid))),
        ("Avg peak",      f"{avg_pk:.3f}"),
        ("Avg target",    f"{avg_tg:.3f}"),
        ("Target ratio",  f"{avg_tg/avg_pk*100:.1f}%"
                          if avg_pk > 0 else "N/A"),
        ("Force data",    "YES" if has_ft  else "NO"),
        ("AI0 data",      "YES" if has_ai0 else "NO"),
    ]
    if avg_fz is not None:
        stats.append(("Avg |Fz|",  f"{avg_fz:.2f} N"))
    if avg_ai0 is not None:
        stats.append(("Avg AI0",   f"{avg_ai0:.4f} V"))

    y = 0.96
    ax7.text(0.05, y, "Summary", fontweight='bold',
             fontsize=11, transform=ax7.transAxes)
    y -= 0.08
    for lbl, val in stats:
        ax7.text(0.05, y, lbl, fontsize=8,
                color='gray', transform=ax7.transAxes)
        ax7.text(0.55, y, str(val), fontsize=8,
                fontweight='bold', transform=ax7.transAxes)
        y -= 0.082

    savefig(fig, csv_path, 'overview', save)
    plt.show()

# ── Plot 2: Per-point bar charts ──────────────────────────────
def plot_per_point(df, events, csv_path, save=False):
    valid = [e for e in events if e['point'] > 0]
    if not valid:
        print("[analyze] No press events"); return

    label    = get_dataset_label(csv_path)
    seen     = {(e['point'], e['visit']) for e in valid}
    pts      = [(p, v) for (p, v) in VISIT_SEQUENCE if (p, v) in seen]
    cols  = 4
    rows  = math.ceil(len(pts) / cols)
    fig, axes = plt.subplots(rows, cols,
                             figsize=(cols*5, rows*3.8))
    fig.suptitle(
        f"Per-point cell response — {label}",
        fontsize=12, fontweight='bold')
    axes = np.array(axes).flatten()

    for idx, (pt, visit) in enumerate(pts):
        ax  = axes[idx]
        evs = [e for e in valid if e['point'] == pt and e['visit'] == visit]
        avg = np.mean([e['peak'] for e in evs], axis=0)
        ti  = UR5_TO_IDX.get(pt, -1)

        colors = [CMAP(v) for v in avg]
        bars   = ax.bar(range(N), avg, color=colors,
                        edgecolor='white', linewidth=0.4)
        if 0 <= ti < N:
            bars[ti].set_edgecolor('red')
            bars[ti].set_linewidth(2.5)
            ax.axvspan(ti-0.5, ti+0.5, alpha=0.12,
                      color='red')

        raw    = RAW_CELLS[ti] if 0 <= ti < N else '?'
        v_tag  = f" #{visit+1}" if visit > 0 else ""
        fz_s   = (f"  |Fz|={np.mean([e['fz_mean'] for e in evs]):.1f}N"
                  if evs[0]['fz_mean'] > 0 else "")
        ax.set_title(
            f"P{pt}{v_tag} → S{raw} "
            f"({len(evs)} press){fz_s}",
            fontsize=8, fontweight='bold')
        ax.set_xticks(range(N))
        ax.set_xticklabels(
            [f'P{IDX_TO_UR5.get(i, "?")}' for i in range(N)],
            rotation=90, fontsize=5)
        ax.set_ylim(0, 1.05)
        ax.set_ylabel('Pressure', fontsize=7)
        ax.grid(axis='y', alpha=0.3)
        ax.set_facecolor('#fafafa')

    for idx in range(len(pts), len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()
    savefig(fig, csv_path, 'perpoint', save)
    plt.show()

# ── Plot 3: Hex maps per point ────────────────────────────────
def plot_hex_detail(df, events, csv_path, save=False):
    valid = [e for e in events if e['point'] > 0]
    if not valid:
        print("[analyze] No press events"); return

    label    = get_dataset_label(csv_path)
    seen     = {(e['point'], e['visit']) for e in valid}
    pts      = [(p, v) for (p, v) in VISIT_SEQUENCE if (p, v) in seen]
    cols  = 5
    rows  = math.ceil(len(pts) / cols)
    fig, axes = plt.subplots(rows, cols,
                             figsize=(cols*3.2, rows*3.2))
    fig.suptitle(
        f"Hex pressure maps per point — {label}",
        fontsize=12, fontweight='bold')
    axes = np.array(axes).flatten()

    for idx, (pt, visit) in enumerate(pts):
        evs   = [e for e in valid if e['point'] == pt and e['visit'] == visit]
        avg   = np.mean([e['peak'] for e in evs], axis=0)
        ti    = UR5_TO_IDX.get(pt, -1)
        v_tag = f" #{visit+1}" if visit > 0 else ""
        _hex_map(axes[idx], avg, target_idx=ti,
                 title=f"P{pt}{v_tag} (n={len(evs)})")

    for idx in range(len(pts), len(axes)):
        axes[idx].set_visible(False)

    plt.tight_layout()
    savefig(fig, csv_path, 'hexmaps', save)
    plt.show()

# ── Plot 4: Force analysis ────────────────────────────────────
def plot_force(df, events, csv_path, save=False):
    if 'fz' not in df.columns:
        print("[analyze] No force data in this session")
        print("[analyze] Run a new session — force is now logged")
        return

    label = get_dataset_label(csv_path)
    valid = [e for e in events if e['point'] > 0]
    fig   = plt.figure(figsize=(20, 12))
    fig.suptitle(
        f"Force / torque analysis — {label}",
        fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(3, 3, figure=fig,
                           hspace=0.48, wspace=0.38)

    # Force timeline
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(df['t'].to_numpy(), df['fz'].to_numpy(), color='#dc0000',
            linewidth=0.8, label='Fz (contact)')
    ax1.plot(df['t'].to_numpy(), df['fx'].to_numpy(), color='#2ab5a0',
            linewidth=0.6, alpha=0.7, label='Fx')
    ax1.plot(df['t'].to_numpy(), df['fy'].to_numpy(), color='#EF9F27',
            linewidth=0.6, alpha=0.7, label='Fy')
    pressing = (df['ur5_pressing'] == 1).to_numpy()
    fmin = df[['fx','fy','fz']].min().min()
    fmax = df[['fx','fy','fz']].max().max()
    ax1.fill_between(df['t'].to_numpy(), fmin, fmax,
                    where=pressing, alpha=0.12,
                    color='red', label='Pressing')
    for ev in valid:
        ax1.axvline(ev['start'], color='gray',
                   alpha=0.4, linewidth=0.7)
        ax1.text(ev['start']+0.1, fmax*0.85,
                f"P{ev['point']}", fontsize=5,
                color='gray', rotation=90)
    ax1.set_xlabel('Time (s)', fontsize=9)
    ax1.set_ylabel('Force (N)', fontsize=9)
    ax1.set_title(
        'TCP force components over session', fontsize=10)
    has_ai0_f = 'ai0' in df.columns and df['ai0'].abs().max() > 1e-6
    if has_ai0_f:
        ax1b = ax1.twinx()
        ax1b.plot(df['t'].to_numpy(), df['ai0'].to_numpy(),
                 color='#9b59b6', linewidth=0.7, linestyle='--',
                 alpha=0.9, label='AI0 (V)')
        ax1b.set_ylabel('AI0 (V)', fontsize=9, color='#9b59b6')
        ax1b.tick_params(axis='y', colors='#9b59b6', labelsize=7)
        lines1, lbl1 = ax1.get_legend_handles_labels()
        lines2, lbl2 = ax1b.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, lbl1 + lbl2,
                  fontsize=8, loc='upper right')
    else:
        ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(alpha=0.3)

    # Mean |Fz| per point
    ax2 = fig.add_subplot(gs[1, 0])
    if valid:
        seen  = set(e['point'] for e in valid)
        pts   = [p for p in VISIT_ORDER if p in seen]
        fz_pt = {p: [] for p in pts}
        for _, row in df[df['ur5_pressing']==1].iterrows():
            pt = row.get('ur5_point')
            if pd.notna(pt) and int(pt) in fz_pt:
                fz_pt[int(pt)].append(abs(row['fz']))
        fz_m = [np.mean(fz_pt[p]) if fz_pt[p] else 0
                for p in pts]
        fz_s = [np.std(fz_pt[p])  if fz_pt[p] else 0
                for p in pts]
        ax2.bar(range(len(pts)), fz_m, yerr=fz_s,
               color='#dc0000', alpha=0.85,
               edgecolor='white', capsize=3)
        ax2.set_xticks(range(len(pts)))
        ax2.set_xticklabels(
            [f'P{p}' for p in pts],
            rotation=60, fontsize=6)
        ax2.set_ylabel('|Fz| (N)', fontsize=9)
        ax2.set_title('Mean contact force per point',
                      fontsize=9)
        ax2.grid(axis='y', alpha=0.3)

    # Force vs sensor scatter
    ax3 = fig.add_subplot(gs[1, 1])
    press_df  = df[df['ur5_pressing']==1].copy()
    cell_cols = [f'cell_{i+1}' for i in range(N)]
    if len(press_df) > 10:
        max_s = press_df[cell_cols].max(axis=1)
        fz_a  = press_df['fz'].abs()
        sc    = ax3.scatter(fz_a, max_s,
                           c=fz_a, cmap=CMAP,
                           alpha=0.3, s=8)
        plt.colorbar(sc, ax=ax3,
                    label='|Fz| (N)', shrink=0.8)
        mask = (fz_a < 50) & (max_s < 1.05)
        if mask.sum() > 20:
            coef = np.polyfit(
                fz_a[mask], max_s[mask], 1)
            xf   = np.linspace(
                fz_a[mask].min(),
                fz_a[mask].max(), 100)
            r2   = np.corrcoef(
                fz_a[mask],
                max_s[mask])[0,1]**2
            ax3.plot(xf, np.polyval(coef, xf),
                    'r--', linewidth=1.5,
                    label=f'slope={coef[0]:.4f}  '
                          f'r²={r2:.3f}')
            ax3.legend(fontsize=7)
        ax3.set_xlabel('|Fz| (N)', fontsize=9)
        ax3.set_ylabel('Max sensor value', fontsize=9)
        ax3.set_title('Force vs sensor correlation',
                      fontsize=9)
        ax3.grid(alpha=0.3)

    # TCP Z trajectory
    ax4 = fig.add_subplot(gs[1, 2])
    if 'tcp_z' in df.columns:
        ax4.plot(df['t'].to_numpy(), df['tcp_z'].to_numpy()*1000,
                color='#2ab5a0', linewidth=0.8)
        ax4.fill_between(
            df['t'].to_numpy(),
            df['tcp_z'].min()*1000,
            df['tcp_z'].max()*1000,
            where=pressing, alpha=0.2,
            color='red', label='Pressing')
        ax4.set_xlabel('Time (s)', fontsize=9)
        ax4.set_ylabel('TCP Z (mm)', fontsize=9)
        ax4.set_title('TCP Z — indentation depth',
                      fontsize=9)
        ax4.legend(fontsize=8)
        ax4.grid(alpha=0.3)

    # Box plot force components
    ax5 = fig.add_subplot(gs[2, 0])
    press_df = df[df['ur5_pressing']==1]
    if len(press_df) > 0:
        ft_cols = [c for c in
                   ['fx','fy','fz','tx','ty','tz']
                   if c in press_df.columns]
        data = [press_df[c].abs().values
                for c in ft_cols]
        bp   = ax5.boxplot(data, labels=ft_cols,
                          patch_artist=True,
                          medianprops={
                              'color':'red',
                              'linewidth':1.5})
        colors = ['#2ab5a0','#2ab5a0','#dc0000',
                  '#EF9F27','#EF9F27','#EF9F27']
        for patch, col in zip(
                bp['boxes'], colors[:len(ft_cols)]):
            patch.set_facecolor(col)
            patch.set_alpha(0.7)
        ax5.set_ylabel('|Value|', fontsize=9)
        ax5.set_title(
            'Force/torque distribution (pressing)',
            fontsize=9)
        ax5.grid(axis='y', alpha=0.3)

    # Fz hex map
    ax6 = fig.add_subplot(gs[2, 1])
    fz_hex = np.zeros(N)
    cnt    = np.zeros(N)
    for _, row in df[df['ur5_pressing']==1].iterrows():
        pt = row.get('ur5_point')
        if pd.notna(pt):
            idx = UR5_TO_IDX.get(int(pt), -1)
            if idx >= 0:
                fz_hex[idx] += abs(row['fz'])
                cnt[idx]    += 1
    mask = cnt > 0
    fz_hex[mask] /= cnt[mask]
    _hex_map(ax6, fz_hex,
             title='Mean |Fz| per cell',
             vmax=max(fz_hex.max(), 1.0), unit='N')

    # Force vs sensor per event scatter
    ax7 = fig.add_subplot(gs[2, 2])
    if valid and any(e['fz_mean'] > 0 for e in valid):
        fz_ev = [e['fz_mean']     for e in valid]
        s_ev  = [e['target_peak'] for e in valid]
        pts_l = [f"P{e['point']}" for e in valid]
        sc = ax7.scatter(fz_ev, s_ev,
                        c=range(len(valid)),
                        cmap='viridis', s=60,
                        edgecolors='white',
                        linewidth=0.5)
        for i, lbl in enumerate(pts_l):
            ax7.annotate(
                lbl, (fz_ev[i], s_ev[i]),
                fontsize=6, alpha=0.8,
                xytext=(3, 3),
                textcoords='offset points')
        ax7.set_xlabel('|Fz| mean (N)', fontsize=9)
        ax7.set_ylabel('Target cell pressure',
                       fontsize=9)
        ax7.set_title(
            'Force vs target sensor (per event)',
            fontsize=9)
        ax7.grid(alpha=0.3)

    savefig(fig, csv_path, 'force', save)
    plt.show()

# ── Plot 5b: Analog input (AI0) ───────────────────────────────
def plot_analog(df, events, csv_path, save=False):
    has_ai0 = 'ai0' in df.columns and df['ai0'].abs().max() > 1e-6
    if not has_ai0:
        print("[analyze] No AI0 analog data in this session")
        return

    label    = get_dataset_label(csv_path)
    valid    = [e for e in events if e['point'] > 0]

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        f"UR Robot Analog Input (AI0) — {label}",
        fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(2, 2, figure=fig,
                           hspace=0.45, wspace=0.35)

    # AI0 timeline
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(df['t'].to_numpy(), df['ai0'].to_numpy(),
            color='#9b59b6', linewidth=0.8, label='AI0 (V)')
    pressing = (df['ur5_pressing'] == 1).to_numpy()
    ai0_arr  = df['ai0'].to_numpy()
    ai0_min, ai0_max = float(ai0_arr.min()), float(ai0_arr.max())
    span = ai0_max - ai0_min
    if span > 1e-6:
        ax1.fill_between(df['t'].to_numpy(), ai0_min, ai0_max,
                        where=pressing, alpha=0.15,
                        color='red', label='Pressing')
    for ev in valid:
        ax1.axvline(ev['start'], color='gray',
                   alpha=0.4, linewidth=0.7)
        ax1.text(ev['start'] + 0.1,
                ai0_min + span * 0.9 if span > 1e-6 else ai0_min,
                f"P{ev['point']}", fontsize=5,
                color='gray', rotation=90)
    ax1.set_xlabel('Time (s)', fontsize=9)
    ax1.set_ylabel('AI0 (V)', fontsize=9)
    ax1.set_title('Analog Input 0 — full session', fontsize=10)
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    # Mean / peak AI0 per press event
    ax2 = fig.add_subplot(gs[1, 0])
    if valid:
        ai0_means = [e.get('ai0_mean', 0.0) for e in valid]
        ai0_peaks = [e.get('ai0_peak', 0.0) for e in valid]
        x = list(range(len(valid)))
        ax2.bar([i - 0.2 for i in x], ai0_means, width=0.35,
               color='#9b59b6', alpha=0.85,
               edgecolor='white', label='Mean')
        ax2.bar([i + 0.2 for i in x], ai0_peaks, width=0.35,
               color='#e056b6', alpha=0.85,
               edgecolor='white', label='Peak')
        ax2.set_xticks(x)
        ax2.set_xticklabels(
            [f"P{e['point']}" for e in valid],
            rotation=60, fontsize=6)
        ax2.set_ylabel('AI0 (V)', fontsize=9)
        ax2.set_title('AI0 per press event', fontsize=9)
        ax2.legend(fontsize=7)
        ax2.grid(axis='y', alpha=0.3)

    # AI0 vs Force scatter (pressing frames)
    ax3 = fig.add_subplot(gs[1, 1])
    press_df = df[df['ur5_pressing'] == 1].copy()
    has_ft   = 'fz' in df.columns
    if has_ft and len(press_df) > 10:
        sc = ax3.scatter(
            press_df['ai0'].to_numpy(),
            press_df['fz'].abs().to_numpy(),
            alpha=0.3, s=8,
            c=press_df['t'].to_numpy(), cmap=CMAP)
        plt.colorbar(sc, ax=ax3, label='Time (s)', shrink=0.8)
        ax3.set_xlabel('AI0 (V)', fontsize=9)
        ax3.set_ylabel('|Fz| (N)', fontsize=9)
        ax3.set_title('AI0 vs Contact Force (pressing)',
                      fontsize=9)
        ax3.grid(alpha=0.3)
    else:
        msg = 'No force data' if not has_ft else 'Not enough data'
        ax3.text(0.5, 0.5, msg,
                ha='center', va='center',
                transform=ax3.transAxes,
                fontsize=11, color='gray')
        ax3.axis('off')

    savefig(fig, csv_path, 'analog', save)
    plt.show()

# ── Plot 5: Session comparison ────────────────────────────────
def plot_comparison(save=False):
    files = find_all_csvs()
    if not files:
        print("[analyze] No sessions found"); return

    records = []
    for f in files:
        try:
            df     = load_session(f)
            events = get_press_events(df)
            valid  = [e for e in events
                      if e['point'] > 0]
            if not valid:
                continue
            label = get_dataset_label(f)
            t_s   = datetime.fromtimestamp(
                df['timestamp'].iloc[0])
            records.append({
                'path':      f,
                'name':      label,
                'datetime':  t_s,
                'duration':  df['t'].iloc[-1],
                'n_events':  len(valid),
                'avg_peak':  np.mean(
                    [e['peak_max'] for e in valid]),
                'avg_tgt':   np.mean(
                    [e['target_peak'] for e in valid]),
                'avg_fz':    np.mean(
                    [e['fz_mean'] for e in valid])
                             if valid[0]['fz_mean'] > 0
                             else 0,
                'has_force': 'fz' in df.columns,
            })
        except Exception as e:
            print(f"[analyze] Skip {f}: {e}")

    if not records:
        print("[analyze] No valid sessions"); return

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Session comparison — all datasets",
                fontsize=13, fontweight='bold')

    names = [r['name'][:20] for r in records]
    x     = range(len(records))

    # Avg peak
    ax = axes[0, 0]
    vals = [r['avg_peak'] for r in records]
    ax.bar(x, vals, color=[CMAP(v) for v in vals],
          edgecolor='white')
    ax.set_xticks(list(x))
    ax.set_xticklabels(names, rotation=45,
                       ha='right', fontsize=7)
    ax.set_ylabel('Avg peak pressure')
    ax.set_title('Average peak per dataset')
    ax.set_ylim(0, 1.05)
    ax.grid(axis='y', alpha=0.3)

    # Target vs all
    ax = axes[0, 1]
    w  = 0.35
    ax.bar([i-w/2 for i in x],
          [r['avg_peak'] for r in records],
          width=w, color='#2ab5a0',
          label='All cells', edgecolor='white')
    ax.bar([i+w/2 for i in x],
          [r['avg_tgt'] for r in records],
          width=w, color='#dc0000',
          label='Target cell', edgecolor='white')
    ax.set_xticks(list(x))
    ax.set_xticklabels(names, rotation=45,
                       ha='right', fontsize=7)
    ax.set_ylabel('Pressure')
    ax.set_title('All cells vs target cell')
    ax.legend(fontsize=8)
    ax.set_ylim(0, 1.05)
    ax.grid(axis='y', alpha=0.3)

    # Duration
    ax = axes[0, 2]
    ax.bar(x, [r['duration'] for r in records],
          color='#2ab5a0', edgecolor='white')
    ax.set_xticks(list(x))
    ax.set_xticklabels(names, rotation=45,
                       ha='right', fontsize=7)
    ax.set_ylabel('Duration (s)')
    ax.set_title('Session duration')
    ax.grid(axis='y', alpha=0.3)

    # Force
    ax = axes[1, 0]
    fr = [r for r in records if r['avg_fz'] > 0]
    if fr:
        fn   = [r['name'][:20] for r in fr]
        fz_v = [r['avg_fz']    for r in fr]
        ax.bar(range(len(fr)), fz_v,
              color='#dc0000', edgecolor='white',
              alpha=0.85)
        ax.set_xticks(range(len(fr)))
        ax.set_xticklabels(fn, rotation=45,
                          ha='right', fontsize=7)
        ax.set_ylabel('|Fz| (N)')
        ax.set_title('Avg contact force per dataset')
        ax.grid(axis='y', alpha=0.3)
    else:
        ax.text(0.5, 0.5,
               'No force data yet\nRun new session',
               ha='center', va='center',
               transform=ax.transAxes,
               fontsize=11, color='gray')
        ax.axis('off')

    # Force vs sensor scatter
    ax = axes[1, 1]
    fr = [r for r in records if r['avg_fz'] > 0]
    if fr:
        ax.scatter(
            [r['avg_fz']   for r in fr],
            [r['avg_peak'] for r in fr],
            c=[r['avg_fz'] for r in fr],
            cmap=CMAP, s=80,
            edgecolors='white', linewidth=0.5)
        for r in fr:
            ax.annotate(r['name'][:14],
                       (r['avg_fz'],
                        r['avg_peak']),
                       fontsize=6, alpha=0.7)
        ax.set_xlabel('Avg |Fz| (N)')
        ax.set_ylabel('Avg peak pressure')
        ax.set_title(
            'Force vs sensor (across datasets)')
        ax.grid(alpha=0.3)
    else:
        ax.axis('off')

    # Summary table
    ax = axes[1, 2]
    ax.axis('off')
    tdata = [
        [r['name'][:18],
         r['datetime'].strftime('%m-%d %H:%M'),
         f"{r['duration']:.0f}s",
         str(r['n_events']),
         f"{r['avg_peak']:.3f}",
         f"{r['avg_fz']:.1f}N"
         if r['avg_fz'] > 0 else "—"]
        for r in records
    ]
    tbl = ax.table(
        cellText=tdata,
        colLabels=['Dataset', 'Time', 'Dur',
                   'Events', 'Avg peak', 'Avg Fz'],
        loc='center', cellLoc='center')
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7)
    tbl.scale(1, 1.4)
    ax.set_title('All datasets',
                fontsize=9, fontweight='bold',
                pad=12)

    plt.tight_layout()

    if save:
        save_dir = os.path.join(PLOTS_DIR,
                                'comparison')
        os.makedirs(save_dir, exist_ok=True)
        path = os.path.join(save_dir,
                           'comparison_all.png')
        fig.savefig(path, dpi=150,
                   bbox_inches='tight',
                   facecolor='white')
        print(f"[analyze] Saved → {path}")

    plt.show()

# ── Main ──────────────────────────────────────────────────────
def main():
    args = parse_args()

    matplotlib.rcParams.update({
        'figure.facecolor':  'white',
        'axes.facecolor':    'white',
        'font.family':       'DejaVu Sans',
        'axes.spines.top':   False,
        'axes.spines.right': False,
    })

    if args.all:
        plot_comparison(save=args.save)
        return

    path   = find_csv(args.file)
    label  = get_dataset_label(path)
    df     = load_session(path)
    events = get_press_events(df)

    print(f"\n[analyze] Dataset : {label}")
    print(f"[analyze] Figures → "
          f"{get_save_dir(path) if args.save else 'screen only'}")

    if args.force:
        plot_force(df, events, path, save=args.save)
        return

    plot_overview(df,   events, path, save=args.save)
    plot_per_point(df,  events, path, save=args.save)
    plot_hex_detail(df, events, path, save=args.save)
    plot_force(df,      events, path, save=args.save)
    plot_analog(df,     events, path, save=args.save)

    if args.save:
        save_dir = get_save_dir(path)
        print(f"\n[analyze] All figures saved to:")
        print(f"          {save_dir}")
    print("\n[analyze] Done!")

if __name__ == "__main__":
    try:
        import pandas, numpy, matplotlib
    except ImportError:
        print("Installing dependencies...")
        os.system(f"{sys.executable} -m pip install "
                 "matplotlib pandas numpy")
    main()
