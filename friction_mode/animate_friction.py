"""
animate_friction.py  —  Star-Nose Sensor  |  Friction Session Animation
========================================================================
Animated dashboard for a friction / sliding session log.

Layout
------
  Top-left  : Hex map — live capacitive sensor values
  Top-right : TCP XY sliding trajectory — current position marker
  Mid       : Rolling cell history strip
  Lower-1   : Force rolling window — Robot Fz  +  FUTEK load cell (N)
  Lower-2   : Torque rolling window — Tx  Ty  Tz  +  |τ| magnitude
  Bottom    : Progress bar

Usage
-----
  python animate_friction.py                    # latest session, screen
  python animate_friction.py ecoflex_raster     # partial filename match
  python animate_friction.py --save             # save as MP4
  python animate_friction.py --save --gif       # save as GIF
  python animate_friction.py --speed 2.0        # 2× playback speed
  python animate_friction.py --step 3           # use every 3rd row
"""

import os
import sys
import glob
import argparse
import platform

import shutil

import numpy as np
import pandas as pd
import matplotlib
if '--save' in sys.argv:
    matplotlib.use('Agg')
elif platform.system() == 'Darwin':
    matplotlib.use('MacOSX')
# on Linux: let matplotlib auto-detect (TkAgg / Qt5Agg / GTK3Agg)
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import RegularPolygon, FancyArrowPatch
from matplotlib.cm import ScalarMappable
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

_HAS_FFMPEG = shutil.which('ffmpeg') is not None
from matplotlib.collections import LineCollection

# ── Paths ─────────────────────────────────────────────────────────────────────
FRICTION_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR     = os.path.join(FRICTION_DIR, 'logs')
PLOTS_DIR    = os.path.join(FRICTION_DIR, 'plots')

# ── Sensor layout ─────────────────────────────────────────────────────────────
POINTS_MM = [
    (-8, +14), ( 0, +14), (+8, +14),
    (-12, +7), (-4, +7),  (+4, +7),  (+12, +7),
    (-16,  0), (-8,  0),  ( 0,  0),  (+8,  0),  (+16, 0),
    (-12, -7), (-4, -7),  (+4, -7),  (+12, -7),
    (-8, -14), ( 0, -14), (+8, -14),
]
N = 19
UR5_TO_IDX = {
    1:16,  2:12,  3:7,
    4:17,  5:13,  6:8,   7:3,
    8:18,  9:14,  10:9,  11:4,  12:0,
    13:15, 14:10, 15:5,  16:1,
    17:11, 18:6,  19:2,
}
IDX_TO_UR5    = {v: k for k, v in UR5_TO_IDX.items()}
POS_TO_SENSOR = [UR5_TO_IDX[i + 1] for i in range(N)]
POINT_ORDER   = [UR5_TO_IDX[p] for p in range(1, N + 1)]

# ── FUTEK load cell ────────────────────────────────────────────────────────────
AI0_ZERO_V       = 5.0
LOADCELL_MAX_N   = 10.0 * 4.44822
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0

def _ai0_to_n(v):
    return -(np.asarray(v, dtype=float) - AI0_ZERO_V) * LOADCELL_N_PER_V

# ── Colours ────────────────────────────────────────────────────────────────────
CMAP = LinearSegmentedColormap.from_list(
    'star_nose', ['#2ab5a0', '#33e666', '#ffe619', '#ff7300', '#dc0000'])
BG   = '#111111'
MID  = '#222222'
EDGE = '#444444'

TQ_COLORS = {'tx': '#3498db', 'ty': '#9b59b6', 'tz': '#e67e22'}


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description='Friction session animator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('file',    nargs='?',
                   help='CSV filename or partial label (default: latest)')
    p.add_argument('--save',  action='store_true', help='Save animation to file')
    p.add_argument('--gif',   action='store_true', help='Save as GIF (default: MP4)')
    p.add_argument('--speed', type=float, default=1.0,
                   help='Playback speed multiplier (default: 1.0)')
    p.add_argument('--step',  type=int,   default=1,
                   help='Use every Nth row (default: 1)')
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# File helpers
# ─────────────────────────────────────────────────────────────────────────────
def find_all_csvs():
    return sorted(glob.glob(os.path.join(LOGS_DIR, '*.csv')))


def find_csv(arg=None):
    files = find_all_csvs()
    if not files:
        sys.exit(f'[animate] No CSV files in {LOGS_DIR}')
    if arg is None:
        return files[-1]
    if os.path.isfile(arg):
        return arg
    matches = [f for f in files
               if os.path.basename(f) == arg or arg in os.path.basename(f)]
    return matches[-1] if matches else files[-1]


def session_label(path):
    base = os.path.basename(path).replace('.csv', '')
    if '_session_' in base:
        return base.split('_session_', 1)[0]
    return base


# ─────────────────────────────────────────────────────────────────────────────
# Data loading
# ─────────────────────────────────────────────────────────────────────────────
def load_session(path):
    print(f'[animate] Loading : {os.path.basename(path)}')
    df = pd.read_csv(path)
    df['t'] = df['timestamp'] - df['timestamp'].iloc[0]

    for c in [f'cell_{i+1}' for i in range(N)]:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce').fillna(0.0)

    for c in ['fx', 'fy', 'fz', 'tx', 'ty', 'tz',
              'tcp_x', 'tcp_y', 'tcp_z', 'ai0', 'ur5_pressing']:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce').fillna(0.0)

    dur  = df['t'].iloc[-1]
    rate = len(df) / dur if dur > 0 else 0
    print(f'[animate] Duration: {dur:.1f} s  |  {len(df):,} rows  |  {rate:.1f} Hz')
    return df


# ─────────────────────────────────────────────────────────────────────────────
# Build animation
# ─────────────────────────────────────────────────────────────────────────────
def build_animation(df, label, step=1, speed=1.0):
    cell_cols = [f'cell_{i+1}' for i in range(N)]
    has_force = 'fz'    in df.columns
    has_ai0   = 'ai0'   in df.columns and df['ai0'].abs().max() > 1e-6
    has_tcp   = 'tcp_x' in df.columns
    has_tq    = all(c in df.columns for c in ['tx', 'ty', 'tz'])

    # Zero baselines (subtract global min so rest = 0)
    fz_base = float(df['fz'].min())           if has_force else 0.0
    lc_base = float(_ai0_to_n(df['ai0']).min()) if has_ai0 else 0.0
    print(f'[animate] Zero ref — Fz: {fz_base:+.3f} N   LC: {lc_base:+.3f} N')

    frames_df = df.iloc[::step].reset_index(drop=True)
    n_frames  = len(frames_df)
    total_t   = frames_df['t'].iloc[-1]

    avg_dt_s = df['t'].iloc[-1] / max(len(df) - 1, 1)
    interval = max(10.0, avg_dt_s * step * 1000.0 / speed)

    # ── Layout ────────────────────────────────────────────────────────────────
    # 5 rows: [hex+traj, history, force, torque, progress]
    fig = plt.figure(figsize=(14, 12), facecolor=BG)
    gs  = gridspec.GridSpec(
        5, 2, figure=fig,
        height_ratios=[9, 2.5, 2, 2, 0.5],
        width_ratios=[6, 4],
        hspace=0.18, wspace=0.15,
        left=0.04, right=0.97, top=0.93, bottom=0.04,
    )

    ax_hex   = fig.add_subplot(gs[0, 0])   # hex map
    ax_traj  = fig.add_subplot(gs[0, 1])   # TCP XY trajectory
    ax_hist  = fig.add_subplot(gs[1, :])   # rolling history
    ax_force = fig.add_subplot(gs[2, :])   # Fz + load cell
    ax_tq    = fig.add_subplot(gs[3, :])   # Tx Ty Tz + |τ|
    ax_prog  = fig.add_subplot(gs[4, :])   # progress bar

    for ax in [ax_hex, ax_traj, ax_hist, ax_force, ax_tq, ax_prog]:
        ax.set_facecolor(BG)
        for sp in ax.spines.values():
            sp.set_edgecolor(EDGE)

    # ── Hex map ───────────────────────────────────────────────────────────────
    hex_patches, hex_texts = [], []
    for xmm, ymm in POINTS_MM:
        h = RegularPolygon((xmm, ymm), numVertices=6, radius=4.5,
                           facecolor=CMAP(0.0), edgecolor=EDGE, linewidth=0.8)
        ax_hex.add_patch(h)
        hex_patches.append(h)
        txt = ax_hex.text(xmm, ymm, '', ha='center', va='center',
                          fontsize=5.5, color='white')
        hex_texts.append(txt)

    for i, (xmm, ymm) in enumerate(POINTS_MM):
        ax_hex.text(xmm, ymm + 2.2, f"P{IDX_TO_UR5.get(i, '?')}",
                    ha='center', va='center', fontsize=4.5,
                    color='#bbbbbb', alpha=0.8)

    ax_hex.set_xlim(-22, 22)
    ax_hex.set_ylim(-20, 20)
    ax_hex.set_aspect('equal')
    ax_hex.axis('off')

    sm = ScalarMappable(cmap=CMAP, norm=Normalize(0, 1))
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax_hex, shrink=0.55, pad=0.02, label='Pressure')
    cb.ax.yaxis.label.set_color('white')
    cb.ax.tick_params(colors='white')

    hex_title = ax_hex.set_title('', fontsize=10, fontweight='bold',
                                  color='white', pad=5)

    # ── TCP XY trajectory ─────────────────────────────────────────────────────
    if has_tcp:
        tx_all = df['tcp_x'].to_numpy() * 1000   # m → mm
        ty_all = df['tcp_y'].to_numpy() * 1000
        max_s_all = df[cell_cols].max(axis=1).to_numpy()

        # Static full-path background (grey)
        ax_traj.plot(tx_all, ty_all, color='#333333', linewidth=0.8,
                     alpha=0.6, zorder=1)

        # Start / end markers
        ax_traj.scatter(tx_all[0],  ty_all[0],  s=50, color='#2ecc71',
                        zorder=4, label='Start')
        ax_traj.scatter(tx_all[-1], ty_all[-1], s=50, color='#e74c3c',
                        zorder=4, label='End')

        # Travelled path line (dynamic, coloured by sensor max)
        traj_seg_data = np.array([tx_all, ty_all]).T.reshape(-1, 1, 2)
        traj_segs     = np.concatenate([traj_seg_data[:-1], traj_seg_data[1:]], axis=1)
        traj_lc = LineCollection([], cmap=CMAP,
                                 norm=Normalize(0, max(max_s_all.max(), 1e-6)),
                                 linewidth=1.8, zorder=2, alpha=0.9)
        ax_traj.add_collection(traj_lc)
        cb2 = fig.colorbar(traj_lc, ax=ax_traj, shrink=0.55, pad=0.02,
                           label='Sensor max')
        cb2.ax.yaxis.label.set_color('white')
        cb2.ax.tick_params(colors='white')

        # Current position dot
        pos_dot, = ax_traj.plot([], [], 'o', ms=8, color='white',
                                markeredgecolor='#dc0000',
                                markeredgewidth=1.5, zorder=5)

        pad_x = max((tx_all.max() - tx_all.min()) * 0.15, 2.0)
        pad_y = max((ty_all.max() - ty_all.min()) * 0.15, 2.0)
        ax_traj.set_xlim(tx_all.min() - pad_x, tx_all.max() + pad_x)
        ax_traj.set_ylim(ty_all.min() - pad_y, ty_all.max() + pad_y)
    else:
        ax_traj.text(0.5, 0.5, 'No TCP data', ha='center', va='center',
                     transform=ax_traj.transAxes, fontsize=10, color='#666666')
        pos_dot = ax_traj.plot([], [])[0]
        traj_lc = None
        traj_segs = None
        max_s_all = np.zeros(len(df))

    ax_traj.set_xlabel('TCP X (mm)', fontsize=8, color='#aaaaaa')
    ax_traj.set_ylabel('TCP Y (mm)', fontsize=8, color='#aaaaaa')
    ax_traj.tick_params(colors='#aaaaaa', labelsize=7)
    ax_traj.set_title('TCP sliding trajectory', fontsize=9, color='white', pad=4)
    ax_traj.set_aspect('equal')
    ax_traj.grid(color=EDGE, alpha=0.4, linewidth=0.4)
    if has_tcp:
        ax_traj.legend(fontsize=6, facecolor=BG, labelcolor='white',
                       edgecolor=EDGE, loc='upper right')

    # ── Rolling history strip ─────────────────────────────────────────────────
    HIST_WIN = 200
    hist_buf = np.zeros((N, HIST_WIN))
    hist_img = ax_hist.imshow(
        hist_buf, aspect='auto', cmap=CMAP, vmin=0, vmax=1,
        extent=[0, HIST_WIN, N + 0.5, 0.5], interpolation='nearest')
    ax_hist.set_yticks(range(1, N + 1))
    ax_hist.set_yticklabels([f'P{p}' for p in range(1, N + 1)],
                             fontsize=5, color='#aaaaaa')
    ax_hist.set_xticks([])
    ax_hist.set_title('Cell history (last ~200 frames)', fontsize=8,
                      color='white', pad=3)

    # ── Force rolling window: Fz + load cell ─────────────────────────────────
    FW = HIST_WIN
    fz_buf = np.zeros(FW)
    lc_buf = np.zeros(FW)

    all_fv = []
    if has_force:
        all_fv.append(df['fz'].to_numpy() - fz_base)
    if has_ai0:
        all_fv.append(_ai0_to_n(df['ai0'].to_numpy()) - lc_base)
    if all_fv:
        combined = np.concatenate(all_fv)
        f_pad = max((combined.max() - combined.min()) * 0.15, 0.5)
        f_ymin, f_ymax = combined.min() - f_pad, combined.max() + f_pad
    else:
        f_ymin, f_ymax = -1.0, 1.0

    fz_line, = ax_force.plot(range(FW), fz_buf, color='#dc0000',
                              linewidth=1.1, label='Robot Fz (N)')
    lc_line, = ax_force.plot(range(FW), lc_buf, color='#9b59b6',
                              linewidth=1.0, label='Load cell (N)')
    ax_force.set_xlim(0, FW)
    ax_force.set_ylim(f_ymin, f_ymax)
    ax_force.set_xticks([])
    ax_force.set_ylabel('N', fontsize=7, color='#aaaaaa')
    ax_force.tick_params(axis='y', colors='#aaaaaa', labelsize=6)
    ax_force.set_title('Force — Robot Fz  vs  FUTEK load cell',
                       fontsize=8, color='white', pad=3)
    ax_force.axhline(0, color=EDGE, linewidth=0.5, linestyle='--')
    ax_force.grid(axis='y', color=EDGE, alpha=0.4, linewidth=0.5)
    ax_force.legend(fontsize=6, facecolor=BG, labelcolor='white',
                    edgecolor=EDGE, loc='upper left')

    # ── Torque rolling window: Tx Ty Tz + |τ| ────────────────────────────────
    tq_bufs  = {c: np.zeros(FW) for c in ['tx', 'ty', 'tz']}
    tmag_buf = np.zeros(FW)

    if has_tq:
        all_tq = np.concatenate([df[c].to_numpy() for c in ['tx', 'ty', 'tz']])
        t_pad  = max((all_tq.max() - all_tq.min()) * 0.15, 0.01)
        t_ymin, t_ymax = all_tq.min() - t_pad, all_tq.max() + t_pad
        tmag_max = np.sqrt(sum(df[c].to_numpy()**2 for c in ['tx', 'ty', 'tz'])).max()
    else:
        t_ymin, t_ymax = -0.1, 0.1
        tmag_max = 0.1

    tq_lines = {}
    for c in ['tx', 'ty', 'tz']:
        ln, = ax_tq.plot(range(FW), tq_bufs[c],
                         color=TQ_COLORS[c], linewidth=1.0, label=c)
        tq_lines[c] = ln

    tmag_line, = ax_tq.plot(range(FW), tmag_buf, color='white',
                             linewidth=0.8, linestyle='--', alpha=0.6,
                             label='|τ|')
    ax_tq_r = ax_tq.twinx()
    ax_tq_r.set_facecolor(BG)
    ax_tq_r.set_ylim(0, max(tmag_max * 1.2, 0.01))
    ax_tq_r.set_ylabel('|τ| (Nm)', fontsize=6, color='#888888')
    ax_tq_r.tick_params(axis='y', colors='#888888', labelsize=5)
    for sp in ax_tq_r.spines.values():
        sp.set_edgecolor(EDGE)

    ax_tq.set_xlim(0, FW)
    ax_tq.set_ylim(t_ymin, t_ymax)
    ax_tq.set_xticks([])
    ax_tq.set_ylabel('Nm', fontsize=7, color='#aaaaaa')
    ax_tq.tick_params(axis='y', colors='#aaaaaa', labelsize=6)
    ax_tq.set_title('Torque — Tx  Ty  Tz  (dashed = |τ| magnitude)',
                    fontsize=8, color='white', pad=3)
    ax_tq.axhline(0, color=EDGE, linewidth=0.5, linestyle='--')
    ax_tq.grid(axis='y', color=EDGE, alpha=0.4, linewidth=0.5)
    ax_tq.legend(fontsize=6, facecolor=BG, labelcolor='white',
                 edgecolor=EDGE, loc='upper left', ncol=4)

    # ── Progress bar ──────────────────────────────────────────────────────────
    (prog_rect,) = ax_prog.barh([0], [0], height=0.8, color='#2ab5a0')
    ax_prog.set_xlim(0, total_t)
    ax_prog.set_ylim(-0.5, 0.5)
    ax_prog.axis('off')
    prog_lbl = ax_prog.text(0.01 * total_t, 0, '0.0 s',
                            va='center', ha='left', fontsize=8, color='white')
    ax_prog.text(total_t, 0, f'{total_t:.0f} s',
                 va='center', ha='right', fontsize=7, color='#888888')

    fig.suptitle(f'Friction animation — {label}',
                 fontsize=13, fontweight='bold', color='white', y=0.98)

    # ── Cached numpy arrays for speed ─────────────────────────────────────────
    cell_arr    = frames_df[[f'cell_{i+1}' for i in range(N)]].to_numpy()
    t_arr       = frames_df['t'].to_numpy()
    pressing_arr = frames_df['ur5_pressing'].to_numpy().astype(int)
    fz_arr      = (frames_df['fz'].to_numpy() - fz_base) if has_force else np.zeros(n_frames)
    lc_arr      = (_ai0_to_n(frames_df['ai0'].to_numpy()) - lc_base) if has_ai0 else np.zeros(n_frames)
    tx_frame    = (frames_df['tcp_x'].to_numpy() * 1000) if has_tcp else np.zeros(n_frames)
    ty_frame    = (frames_df['tcp_y'].to_numpy() * 1000) if has_tcp else np.zeros(n_frames)
    tq_arr      = {c: frames_df[c].to_numpy() if has_tq else np.zeros(n_frames)
                   for c in ['tx', 'ty', 'tz']}
    max_s_frame = cell_arr.max(axis=1)

    # ── Update ────────────────────────────────────────────────────────────────
    def update(fi):
        vals     = cell_arr[fi]
        t_now    = t_arr[fi]
        pressing = pressing_arr[fi] == 1

        # Hex patches
        for i, (patch, txt) in enumerate(zip(hex_patches, hex_texts)):
            si  = POS_TO_SENSOR[i]
            v   = float(np.clip(vals[si], 0.0, 1.0))
            patch.set_facecolor(CMAP(v))
            patch.set_edgecolor('#ff3333' if pressing else EDGE)
            patch.set_linewidth(2.0 if pressing else 0.8)
            txt.set_text(f'{vals[si]:.2f}' if vals[si] > 0.02 else '')
            txt.set_color('white' if v > 0.45 else '#cccccc')

        # Hex title
        slide_str = '▶  SLIDING' if pressing else ''
        fz_val    = float(fz_arr[fi])
        lc_val    = float(lc_arr[fi])
        parts     = []
        if has_force:
            parts.append(f'Fz={fz_val:.1f} N')
        if has_ai0:
            parts.append(f'LC={lc_val:.1f} N')
        force_tag = ('   ' + '  |  '.join(parts)) if (parts and pressing) else ''
        hex_title.set_text(f't = {t_now:.2f} s   {slide_str}{force_tag}')
        hex_title.set_color('#ff5555' if pressing else 'white')

        # TCP trajectory
        if has_tcp and traj_lc is not None:
            segs_to  = traj_segs[:fi] if fi > 0 else traj_segs[:1]
            vals_to  = max_s_frame[:fi] if fi > 0 else max_s_frame[:1]
            traj_lc.set_segments(segs_to)
            traj_lc.set_array(vals_to)
            pos_dot.set_data([tx_frame[fi]], [ty_frame[fi]])
            dot_col = CMAP(float(np.clip(max_s_frame[fi], 0, 1)))
            pos_dot.set_markerfacecolor(dot_col)

        # Rolling history
        hist_buf[:, :-1] = hist_buf[:, 1:]
        hist_buf[:, -1]  = [vals[POINT_ORDER[j]] for j in range(N)]
        hist_img.set_data(hist_buf)

        # Force buffers
        fz_buf[:-1] = fz_buf[1:];  fz_buf[-1] = float(fz_arr[fi])
        lc_buf[:-1] = lc_buf[1:];  lc_buf[-1] = float(lc_arr[fi])
        fz_line.set_ydata(fz_buf)
        lc_line.set_ydata(lc_buf)

        # Torque buffers
        tmag = 0.0
        for c in ['tx', 'ty', 'tz']:
            v = float(tq_arr[c][fi])
            tq_bufs[c][:-1] = tq_bufs[c][1:]
            tq_bufs[c][-1]  = v
            tq_lines[c].set_ydata(tq_bufs[c])
            tmag += v ** 2
        tmag = np.sqrt(tmag)
        tmag_buf[:-1] = tmag_buf[1:]
        tmag_buf[-1]  = tmag
        tmag_line.set_ydata(tmag_buf)
        ax_tq_r.set_ylim(0, max(tmag_buf.max() * 1.2, 0.01))

        # Progress
        prog_rect.set_width(t_now)
        prog_lbl.set_text(f'{t_now:.1f} s')

        artists = (hex_patches + hex_texts +
                   [hex_title, hist_img, fz_line, lc_line,
                    tmag_line, prog_rect, prog_lbl, pos_dot] +
                   list(tq_lines.values()))
        if traj_lc is not None:
            artists.append(traj_lc)
        return artists

    anim = FuncAnimation(fig, update, frames=n_frames,
                         interval=interval, blit=True)
    return fig, anim, interval


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    matplotlib.rcParams.update({
        'figure.facecolor': BG,
        'text.color':       'white',
        'axes.facecolor':   BG,
        'axes.edgecolor':   EDGE,
    })

    path  = find_csv(args.file)
    label = session_label(path)
    df    = load_session(path)

    print(f'[animate] Label : {label}')
    print(f'[animate] Speed : {args.speed}x  |  Step: every {args.step} row(s)')

    fig, anim, interval = build_animation(df, label,
                                          step=args.step, speed=args.speed)
    fps = max(1, min(60, int(1000.0 / interval)))

    if args.save:
        out_dir = os.path.join(PLOTS_DIR, label)
        os.makedirs(out_dir, exist_ok=True)

        use_gif = args.gif or not _HAS_FFMPEG
        if use_gif:
            out = os.path.join(out_dir, 'friction_animation.gif')
            print(f'[animate] Saving GIF @ {fps} fps → {out}')
            anim.save(out, writer=PillowWriter(fps=fps))
        else:
            out = os.path.join(out_dir, 'friction_animation.mp4')
            print(f'[animate] Saving MP4 @ {fps} fps → {out}')
            anim.save(out, writer=FFMpegWriter(fps=fps, bitrate=2000))

        print(f'[animate] Done → {out}')
    else:
        plt.show()


if __name__ == '__main__':
    try:
        import pandas, numpy, matplotlib
    except ImportError:
        print('Installing dependencies ...')
        os.system(f'{sys.executable} -m pip install matplotlib pandas numpy')
    main()
