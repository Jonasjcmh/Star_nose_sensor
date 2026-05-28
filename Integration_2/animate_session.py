"""
animate_session.py
Hexmap animation of a KYWO sensor session.

Usage:
  python3 animate_session.py                       # latest session
  python3 animate_session.py ecoflex_flat          # partial name match
  python3 animate_session.py --save                # save as MP4
  python3 animate_session.py --save --gif          # save as GIF
  python3 animate_session.py --speed 2.0           # 2x playback speed
  python3 animate_session.py --step 3              # use every 3rd frame
"""

import os, sys, glob, argparse, math
import pandas as pd
import numpy as np
import platform, matplotlib
matplotlib.use("MacOSX" if platform.system() == "Darwin" else "TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import RegularPolygon
from matplotlib.cm import ScalarMappable
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

# ── Constants (mirrors analyze_session.py) ────────────────────────────────────
INTEGRATION_DIR     = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR            = os.path.join(INTEGRATION_DIR, "logs")
PLOTS_DIR           = os.path.join(INTEGRATION_DIR, "plots")
LEGACY_DATASETS_DIR = os.path.join(INTEGRATION_DIR, "datasets")
LEGACY_LOG_DIR      = os.path.expanduser("~/sofa-projects/logs")

POINTS_MM = [
    (-8, +14), ( 0, +14), (+8, +14),
    (-12, +7), (-4, +7),  (+4, +7),  (+12, +7),
    (-16,  0), (-8,  0),  ( 0,  0),  (+8,  0),  (+16, 0),
    (-12, -7), (-4, -7),  (+4, -7),  (+12, -7),
    (-8, -14), ( 0, -14), (+8, -14),
]
RAW_CELLS = [2,15,28,1,14,27,40,0,13,26,39,52,12,25,38,51,24,37,50]
N = 19
UR5_TO_IDX = {
    1:16,  2:12,  3:7,
    4:17,  5:13,  6:8,   7:3,
    8:18,  9:14,  10:9,  11:4,  12:0,
    13:15, 14:10, 15:5,  16:1,
    17:11, 18:6,  19:2,
}
IDX_TO_UR5 = {v: k for k, v in UR5_TO_IDX.items()}

CMAP = LinearSegmentedColormap.from_list('star_nose', [
    '#2ab5a0', '#33e666', '#ffe619', '#ff7300', '#dc0000'
])

BG   = '#111111'
MID  = '#222222'
EDGE = '#444444'

# ── Args ──────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('file',    nargs='?',              help='CSV file or partial label')
    p.add_argument('--save',  action='store_true',    help='Save animation to file')
    p.add_argument('--gif',   action='store_true',    help='Save as GIF (default: MP4)')
    p.add_argument('--speed', type=float, default=1.0, help='Playback speed multiplier')
    p.add_argument('--step',  type=int,   default=1,   help='Use every Nth row')
    return p.parse_args()

# ── File helpers ──────────────────────────────────────────────────────────────
def find_all_csvs():
    files = sorted(glob.glob(os.path.join(LOGS_DIR, '*.csv')))
    for extra_dir in [LEGACY_DATASETS_DIR, LEGACY_LOG_DIR]:
        seen = set(os.path.basename(f) for f in files)
        for f in sorted(glob.glob(os.path.join(extra_dir, '*.csv'))):
            if os.path.basename(f) not in seen:
                files.append(f)
                seen.add(os.path.basename(f))
    return sorted(files)

def find_csv(arg=None):
    files = find_all_csvs()
    if not files:
        print(f"[animate] No CSV files found in {LOGS_DIR}")
        sys.exit(1)
    if arg is None:
        return files[-1]
    matches = [f for f in files
               if os.path.basename(f) == arg or arg in os.path.basename(f)]
    return matches[-1] if matches else files[-1]

def get_dataset_label(csv_path):
    base = os.path.basename(csv_path).replace('.csv', '')
    if '_session_' in base:
        return base.split('_session_', 1)[0]
    parts = base.split('_', 3)
    if len(parts) >= 4:
        return parts[3]
    elif len(parts) == 3:
        return parts[2]
    return base

# ── Load ──────────────────────────────────────────────────────────────────────
def load_session(path):
    print(f"[animate] Loading : {os.path.basename(path)}")
    df = pd.read_csv(path)
    df['t'] = df['timestamp'] - df['timestamp'].iloc[0]
    df['ur5_point']    = pd.to_numeric(df['ur5_point'],    errors='coerce')
    df['ur5_pressing'] = pd.to_numeric(df['ur5_pressing'], errors='coerce').fillna(0).astype(int)
    for c in [f'cell_{i+1}' for i in range(N)]:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce').fillna(0)
    for c in ['fx', 'fy', 'fz', 'ai0']:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce').fillna(0)
    dur  = df['t'].iloc[-1]
    rate = len(df) / dur if dur > 0 else 0
    print(f"[animate] Duration: {dur:.1f}s  |  Rate: {rate:.1f} Hz  |  Rows: {len(df):,}")
    return df

# ── Build animation ───────────────────────────────────────────────────────────
def build_animation(df, label, step=1, speed=1.0):
    cell_cols  = [f'cell_{i+1}' for i in range(N)]
    has_force  = 'fz' in df.columns
    frames_df  = df.iloc[::step].reset_index(drop=True)
    n_frames   = len(frames_df)
    total_t    = frames_df['t'].iloc[-1]

    # ms per animation frame
    avg_dt_s = (df['t'].iloc[-1] / max(len(df) - 1, 1))
    interval = max(10.0, avg_dt_s * step * 1000.0 / speed)

    # ── Layout ──
    has_ai0 = 'ai0' in df.columns and df['ai0'].abs().max() > 1e-6
    fig = plt.figure(figsize=(13, 9), facecolor=BG)
    gs  = gridspec.GridSpec(
        4, 2, figure=fig,
        height_ratios=[10, 3, 2, 0.6],
        width_ratios=[6, 4],
        hspace=0.15, wspace=0.15,
        left=0.04, right=0.97, top=0.93, bottom=0.06,
    )

    ax_hex  = fig.add_subplot(gs[0, 0])   # hexmap
    ax_bar  = fig.add_subplot(gs[0, 1])   # per-cell bar chart
    ax_hist = fig.add_subplot(gs[1, :])   # rolling history strip
    ax_ai0  = fig.add_subplot(gs[2, :])   # AI0 analog input trace
    ax_prog = fig.add_subplot(gs[3, :])   # progress bar

    for ax in [ax_hex, ax_bar, ax_hist, ax_ai0, ax_prog]:
        ax.set_facecolor(BG)
        for sp in ax.spines.values():
            sp.set_edgecolor(EDGE)

    # ── Hex map ──────────────────────────────────────────────────────────────
    hex_patches = []
    hex_texts   = []
    for xmm, ymm in POINTS_MM:
        h = RegularPolygon(
            (xmm, ymm), numVertices=6, radius=4.5,
            facecolor=CMAP(0.0), edgecolor=EDGE, linewidth=0.8,
        )
        ax_hex.add_patch(h)
        hex_patches.append(h)
        t = ax_hex.text(xmm, ymm, '', ha='center', va='center',
                        fontsize=5.5, color='white')
        hex_texts.append(t)

    ax_hex.set_xlim(-22, 22)
    ax_hex.set_ylim(-20, 20)
    ax_hex.set_aspect('equal')
    ax_hex.axis('off')

    sm = ScalarMappable(cmap=CMAP, norm=Normalize(0, 1))
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax_hex, shrink=0.55, pad=0.02, label='Pressure')
    cb.ax.yaxis.label.set_color('white')
    cb.ax.tick_params(colors='white')

    hex_title = ax_hex.set_title(
        '', fontsize=11, fontweight='bold', color='white', pad=6)

    # ── Bar chart ─────────────────────────────────────────────────────────────
    x_pos    = np.arange(N)
    bar_rects = ax_bar.bar(x_pos, np.zeros(N),
                           color=[CMAP(0.0)] * N,
                           edgecolor=EDGE, linewidth=0.4)
    target_vline = ax_bar.axvline(-1, color='red', linewidth=1.5, alpha=0.75)

    ax_bar.set_xlim(-0.5, N - 0.5)
    ax_bar.set_ylim(0, 1.05)
    ax_bar.set_xticks(x_pos)
    ax_bar.set_xticklabels(
        [f'P{IDX_TO_UR5.get(i, "?")}' for i in range(N)],
        rotation=90, fontsize=5, color='#aaaaaa')
    ax_bar.tick_params(axis='y', colors='#aaaaaa', labelsize=7)
    ax_bar.set_ylabel('Pressure', fontsize=8, color='#aaaaaa')
    ax_bar.set_title('Cell values', fontsize=9, color='white', pad=4)
    ax_bar.grid(axis='y', color=EDGE, alpha=0.5, linewidth=0.5)

    # ── Rolling history strip ─────────────────────────────────────────────────
    HIST_WIN = 200   # frames shown in the strip
    hist_buf = np.zeros((N, HIST_WIN))
    hist_img = ax_hist.imshow(
        hist_buf, aspect='auto', cmap=CMAP,
        vmin=0, vmax=1,
        extent=[0, HIST_WIN, N + 0.5, 0.5],
        interpolation='nearest',
    )
    ax_hist.set_yticks(range(1, N + 1))
    ax_hist.set_yticklabels(
        [f'P{IDX_TO_UR5.get(i, "?")}' for i in range(N)],
        fontsize=5, color='#aaaaaa')
    ax_hist.set_xticks([])
    ax_hist.set_title('Recent history (last ~200 frames)', fontsize=8,
                       color='white', pad=3)
    press_vline = ax_hist.axvline(HIST_WIN - 1, color='white',
                                   linewidth=0.8, alpha=0.5)

    # ── AI0 analog input trace ────────────────────────────────────────────────
    AI0_WIN  = HIST_WIN
    ai0_buf  = np.zeros(AI0_WIN)
    ai0_rng  = max(float(df['ai0'].abs().max()), 0.1) if has_ai0 else 1.0
    ai0_line, = ax_ai0.plot(range(AI0_WIN), ai0_buf,
                             color='#9b59b6', linewidth=1.0)
    ax_ai0.set_xlim(0, AI0_WIN)
    ax_ai0.set_ylim(-ai0_rng * 1.1, ai0_rng * 1.1)
    ax_ai0.set_xticks([])
    ax_ai0.set_ylabel('V', fontsize=7, color='#aaaaaa')
    ax_ai0.tick_params(axis='y', colors='#aaaaaa', labelsize=6)
    ax_ai0.set_title('Analog Input 0 (AI0)', fontsize=8,
                     color='white', pad=3)
    ax_ai0.axhline(0, color=EDGE, linewidth=0.5)
    ax_ai0.grid(axis='y', color=EDGE, alpha=0.4, linewidth=0.5)
    if not has_ai0:
        ax_ai0.text(AI0_WIN / 2, 0, 'no AI0 data',
                    ha='center', va='center',
                    fontsize=7, color='#666666')

    # ── Progress bar ──────────────────────────────────────────────────────────
    (prog_rect,) = ax_prog.barh([0], [0], height=0.8, color='#2ab5a0')
    ax_prog.set_xlim(0, total_t)
    ax_prog.set_ylim(-0.5, 0.5)
    ax_prog.axis('off')
    prog_label = ax_prog.text(
        0.01 * total_t, 0, '0.0 s',
        va='center', ha='left', fontsize=8, color='white')
    ax_prog.text(total_t, 0, f'{total_t:.0f} s',
                 va='center', ha='right', fontsize=7, color='#888888')

    fig.suptitle(f'Hexmap animation — {label}',
                 fontsize=13, fontweight='bold', color='white', y=0.98)

    # ── Update function ───────────────────────────────────────────────────────
    def update(fi):
        row      = frames_df.iloc[fi]
        vals     = row[cell_cols].values.astype(float)
        pt       = row['ur5_point']
        pressing = int(row['ur5_pressing']) == 1
        t_now    = row['t']

        pt_i = int(pt) if pd.notna(pt) else -1
        ti   = UR5_TO_IDX.get(pt_i, -1)

        # Hex patches
        for i, (patch, txt) in enumerate(zip(hex_patches, hex_texts)):
            v   = float(np.clip(vals[i], 0.0, 1.0))
            col = CMAP(v)
            patch.set_facecolor(col)
            if i == ti and pressing:
                patch.set_edgecolor('red')
                patch.set_linewidth(2.5)
            else:
                patch.set_edgecolor(EDGE)
                patch.set_linewidth(0.8)
            txt.set_text(f'{vals[i]:.2f}' if vals[i] > 0.02 else '')
            txt.set_color('white' if v > 0.45 else '#cccccc')

        # Bar chart
        for i, rect in enumerate(bar_rects):
            v = float(np.clip(vals[i], 0.0, 1.0))
            rect.set_height(v)
            rect.set_facecolor(CMAP(v))
            if i == ti and pressing:
                rect.set_edgecolor('red')
                rect.set_linewidth(2.0)
            else:
                rect.set_edgecolor(EDGE)
                rect.set_linewidth(0.4)
        target_vline.set_xdata([ti, ti] if (ti >= 0 and pressing) else [-2, -2])

        # Rolling history
        hist_buf[:, :-1] = hist_buf[:, 1:]
        hist_buf[:, -1]  = vals
        hist_img.set_data(hist_buf)

        # AI0 trace
        ai0_val = float(row['ai0']) if (has_ai0 and pd.notna(row.get('ai0'))) else 0.0
        ai0_buf[:-1] = ai0_buf[1:]
        ai0_buf[-1]  = ai0_val
        ai0_line.set_ydata(ai0_buf)

        # Title
        fz_str = ''
        if has_force and pressing and pd.notna(row.get('fz')):
            fz_str = f'   |Fz| = {abs(row["fz"]):.1f} N'
        if pressing and pt_i > 0:
            hex_title.set_text(f't = {t_now:.2f} s   ▶  PRESSING P{pt_i}{fz_str}')
            hex_title.set_color('#ff5555')
        else:
            hex_title.set_text(f't = {t_now:.2f} s')
            hex_title.set_color('white')

        # Progress
        prog_rect.set_width(t_now)
        prog_label.set_text(f'{t_now:.1f} s')

        return (hex_patches + hex_texts +
                list(bar_rects) + [target_vline, hist_img,
                ai0_line, prog_rect, prog_label, hex_title])

    anim = FuncAnimation(fig, update, frames=n_frames,
                         interval=interval, blit=True)
    return fig, anim, interval

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    matplotlib.rcParams.update({
        'figure.facecolor': BG,
        'text.color':       'white',
        'axes.facecolor':   BG,
        'axes.edgecolor':   EDGE,
    })

    path  = find_csv(args.file)
    label = get_dataset_label(path)
    df    = load_session(path)

    print(f"[animate] Label   : {label}")
    print(f"[animate] Speed   : {args.speed}x  |  Step: every {args.step} frame(s)")

    fig, anim, interval = build_animation(df, label, step=args.step, speed=args.speed)
    fps = max(1, min(60, int(1000.0 / interval)))

    if args.save:
        save_dir = os.path.join(PLOTS_DIR, label)
        os.makedirs(save_dir, exist_ok=True)

        if args.gif:
            out = os.path.join(save_dir, 'hexmap_animation.gif')
            print(f"[animate] Saving GIF @ {fps} fps → {out}")
            anim.save(out, writer=PillowWriter(fps=fps))
        else:
            out = os.path.join(save_dir, 'hexmap_animation.mp4')
            print(f"[animate] Saving MP4 @ {fps} fps → {out}")
            anim.save(out, writer=FFMpegWriter(fps=fps, bitrate=2000))

        print(f"[animate] Done → {out}")
    else:
        plt.show()

if __name__ == '__main__':
    try:
        import pandas, numpy, matplotlib
    except ImportError:
        print("Installing dependencies...")
        os.system(f"{sys.executable} -m pip install matplotlib pandas numpy")
    main()
