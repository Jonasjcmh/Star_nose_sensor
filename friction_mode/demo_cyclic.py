"""
demo_cyclic.py  —  Star-Nose Sensor  |  Cyclic Trajectory Demo
==============================================================
Synthetic animated dashboard that cycles through the real test trajectories
defined in trajectories.py.  No CSV file needed — physics are simulated.

For each trajectory the script synthesises:
  • Capacitive sensor activation  (Gaussian hot-spot on the leading edge)
  • Robot F/T  (Fz normal load + friction-induced Fx, Fy, Tx, Ty, Tz)
  • FUTEK load-cell signal  (AI0 voltage → N)

Layout  (same panels as animate_friction.py)
---------------------------------------------------------------------------
  Top-left  : Hex map — live capacitive sensor values
  Top-right : TCP XY trajectory — coloured by trajectory, animated position
  Mid       : Rolling cell history strip
  Lower-1   : Force rolling window — Robot Fz  vs  FUTEK load cell
  Lower-2   : Torque rolling window — Tx  Ty  Tz  + |τ| magnitude
  Bottom    : Progress bar + trajectory label

Usage
-----
  python demo_cyclic.py                     # screen, all trajectories, loop
  python demo_cyclic.py --speed 2.0         # 2× playback
  python demo_cyclic.py --traj raster circle spiral   # subset of trajectories
  python demo_cyclic.py --list              # print available trajectory names
  python demo_cyclic.py --cycles 2          # repeat the full cycle N times
  python demo_cyclic.py --step 2            # use every 2nd frame
  python demo_cyclic.py --save              # save MP4 (1 cycle)
  python demo_cyclic.py --gif               # save GIF instead of MP4
"""

import os
import sys
import argparse
import platform
import shutil

import numpy as np
import pandas as pd
import matplotlib

if '--save' in sys.argv or '--gif' in sys.argv:
    matplotlib.use('Agg')
elif platform.system() == 'Darwin':
    matplotlib.use('MacOSX')

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import RegularPolygon
from matplotlib.cm import ScalarMappable
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
from matplotlib.collections import LineCollection

_HAS_FFMPEG = shutil.which('ffmpeg') is not None

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
PLOTS_DIR = os.path.join(_HERE, 'plots')

# ── Sensor layout (identical to animate_friction.py) ──────────────────────────
POINTS_MM = [
    (-8, +14), ( 0, +14), (+8, +14),
    (-12, +7), (-4, +7),  (+4, +7),  (+12, +7),
    (-16,  0), (-8,  0),  ( 0,  0),  (+8,  0),  (+16, 0),
    (-12, -7), (-4, -7),  (+4, -7),  (+12, -7),
    (-8, -14), ( 0, -14), (+8, -14),
]
N = 19
UR5_TO_IDX = {
    1: 16,  2: 12,  3:  7,
    4: 17,  5: 13,  6:  8,  7:  3,
    8: 18,  9: 14, 10:  9, 11:  4, 12:  0,
   13: 15, 14: 10, 15:  5, 16:  1,
   17: 11, 18:  6, 19:  2,
}
IDX_TO_UR5    = {v: k for k, v in UR5_TO_IDX.items()}
POS_TO_SENSOR = [UR5_TO_IDX[i + 1] for i in range(N)]
POINT_ORDER   = [UR5_TO_IDX[p] for p in range(1, N + 1)]

# ── FUTEK constants ────────────────────────────────────────────────────────────
AI0_ZERO_V       = 5.0
LOADCELL_MAX_N   = 10.0 * 4.44822
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0


def _ai0_to_n(v):
    return -(np.asarray(v, dtype=float) - AI0_ZERO_V) * LOADCELL_N_PER_V


# ── Appearance ─────────────────────────────────────────────────────────────────
CMAP = LinearSegmentedColormap.from_list(
    'star_nose', ['#2ab5a0', '#33e666', '#ffe619', '#ff7300', '#dc0000'])
BG   = '#111111'
EDGE = '#444444'
TQ_COLORS = {'tx': '#3498db', 'ty': '#9b59b6', 'tz': '#e67e22'}

TRAJ_PALETTE = [
    '#e74c3c', '#3498db', '#2ecc71', '#f39c12',
    '#9b59b6', '#1abc9c', '#e67e22', '#ecf0f1', '#e91e63',
]

# ── Physics parameters ─────────────────────────────────────────────────────────
FZ_PRESS_N    = 8.2      # N, normal load while pressing
MU_FRICTION   = 0.33     # kinetic friction coefficient
ARM_M         = 0.055    # m, wrist → contact moment arm
SIGMA_HOT_MM  = 6.0      # mm, Gaussian hot-spot σ on sensor array
HOT_REACH_MM  = 11.0     # mm, hot-spot centre offset along velocity direction
NOISE_SIGMA   = 0.025    # sensor cell noise std
V_SLIDE_MMS   = 12.0     # mm/s, default sliding speed


# ─────────────────────────────────────────────────────────────────────────────
# Trajectory helpers
# ─────────────────────────────────────────────────────────────────────────────
DT = 0.02   # seconds — 50 Hz simulation


def _waypoints_to_series(pts_mm, v_mm_s=V_SLIDE_MMS, pause_s=0.5):
    """
    Convert a list of (x_mm, y_mm) waypoints to dense (t, x, y, pressing)
    arrays at DT resolution.  Approach and retract phases add non-pressing
    segments at the start and end.
    """
    t_wpts = [0.0]
    x_wpts = [float(pts_mm[0][0])]
    y_wpts = [float(pts_mm[0][1])]
    p_wpts = [0.0]

    # Press-down at first waypoint
    t_wpts.append(pause_s)
    x_wpts.append(float(pts_mm[0][0]))
    y_wpts.append(float(pts_mm[0][1]))
    p_wpts.append(1.0)

    t = pause_s
    for i in range(1, len(pts_mm)):
        dx = float(pts_mm[i][0]) - x_wpts[-1]
        dy = float(pts_mm[i][1]) - y_wpts[-1]
        ds = np.hypot(dx, dy)
        t += (ds / v_mm_s) if ds > 1e-6 else DT
        t_wpts.append(t)
        x_wpts.append(float(pts_mm[i][0]))
        y_wpts.append(float(pts_mm[i][1]))
        p_wpts.append(1.0)

    # Retract
    t += pause_s
    t_wpts.append(t)
    x_wpts.append(x_wpts[-1])
    y_wpts.append(y_wpts[-1])
    p_wpts.append(0.0)

    ta = np.array(t_wpts)
    xa = np.array(x_wpts)
    ya = np.array(y_wpts)
    pa = np.array(p_wpts)

    t_dense = np.arange(0.0, ta[-1] + DT * 0.5, DT)
    x_dense = np.interp(t_dense, ta, xa)
    y_dense = np.interp(t_dense, ta, ya)
    p_dense = np.interp(t_dense, ta, pa) > 0.5
    return t_dense, x_dense, y_dense, p_dense


# ─────────────────────────────────────────────────────────────────────────────
# Physics simulation
# ─────────────────────────────────────────────────────────────────────────────

def _cell_activation(px, py, vnx, vny, spd_frac, rng):
    """Activation (0-1) for one sensor cell at (px, py) mm."""
    hx = vnx * HOT_REACH_MM
    hy = vny * HOT_REACH_MM
    d  = np.hypot(px - hx, py - hy)
    hot = np.exp(-d ** 2 / (2.0 * SIGMA_HOT_MM ** 2))
    return float(np.clip(
        0.27 + 0.62 * hot * spd_frac + rng.normal(0.0, NOISE_SIGMA),
        0.0, 1.0))


def simulate_physics(t_raw, x_mm, y_mm, pressing, rng):
    """
    Produce a complete synthetic DataFrame for one trajectory segment.
    Columns match those expected by build_animation().
    """
    n  = len(t_raw)
    vx = np.gradient(x_mm, t_raw)
    vy = np.gradient(y_mm, t_raw)
    spd = np.hypot(vx, vy)

    moving  = spd > 1.5
    vn_x    = np.where(moving, vx / np.maximum(spd, 1e-9), 0.0)
    vn_y    = np.where(moving, vy / np.maximum(spd, 1e-9), 0.0)
    spd_frac = np.clip(spd / 20.0, 0.0, 1.0)   # 20 mm/s = full intensity

    # ── Sensor activations ────────────────────────────────────────────────────
    act_by_pos = np.zeros((n, N))   # [frame, POINTS_MM index]
    prev = np.zeros(N)
    for fi in range(n):
        if not pressing[fi]:
            prev = prev * 0.78
            act_by_pos[fi] = prev
            continue
        sf  = float(spd_frac[fi])
        vnx = float(vn_x[fi])
        vny = float(vn_y[fi])
        for pi, (px, py) in enumerate(POINTS_MM):
            prev[pi] = _cell_activation(px, py, vnx, vny, sf, rng)
        act_by_pos[fi] = prev.copy()

    # Remap to cell_{k} columns (k=1..19):
    # act[i] (at POINTS_MM[i]) → column cell_{UR5_TO_IDX[i+1]+1}
    cell_dict = {}
    for i in range(N):
        col = f'cell_{UR5_TO_IDX[i + 1] + 1}'
        cell_dict[col] = act_by_pos[:, i]

    # ── Forces ────────────────────────────────────────────────────────────────
    press_f = pressing.astype(float)
    fz = np.where(pressing,
                  FZ_PRESS_N + rng.normal(0.0, 0.10, n),
                  rng.normal(0.0, 0.015, n))
    fx = -MU_FRICTION * fz * vn_x * press_f + rng.normal(0.0, 0.05, n)
    fy = -MU_FRICTION * fz * vn_y * press_f + rng.normal(0.0, 0.05, n)

    # ── Torques ───────────────────────────────────────────────────────────────
    tx = fy * ARM_M + rng.normal(0.0, 0.003, n)
    ty = -fx * ARM_M + rng.normal(0.0, 0.003, n)
    tz = rng.normal(0.0, 0.002, n) * press_f

    # ── Load cell ─────────────────────────────────────────────────────────────
    lc_n = fz + rng.normal(0.0, 0.15, n)
    ai0  = AI0_ZERO_V - lc_n / LOADCELL_N_PER_V

    data = {
        'timestamp':    t_raw,
        't':            t_raw,
        'tcp_x':        x_mm / 1000.0,   # → metres, matches real data
        'tcp_y':        y_mm / 1000.0,
        'tcp_z':        np.where(pressing, -0.004, 0.0),
        'fz': fz, 'fx': fx, 'fy': fy,
        'tx': tx, 'ty': ty, 'tz': tz,
        'ai0':          ai0,
        'ur5_pressing': pressing.astype(int),
    }
    data.update(cell_dict)
    return pd.DataFrame(data)


# ─────────────────────────────────────────────────────────────────────────────
# Demo dataset builder
# ─────────────────────────────────────────────────────────────────────────────
TRANSITION_S = 1.8   # gap (not-pressing) inserted between trajectories

# Human-readable labels for the trajectory registry keys
TRAJ_LABELS = {
    'line_h':      'Horizontal Line',
    'line_v':      'Vertical Line',
    'diagonal_lr': 'Diagonal ↗',
    'diagonal_rl': 'Diagonal ↘',
    'circle':      'Circle',
    'raster':      'Raster Scan',
    'cross':       'Cross (+)',
    'spiral':      'Archimedean Spiral',
    'star':        'Star Path (all 19 pts)',
}

DEFAULT_ORDER = ['raster', 'circle', 'spiral', 'cross',
                 'diagonal_lr', 'diagonal_rl', 'line_h', 'line_v', 'star']


def build_demo_df(traj_keys, cycles=1, step=1, seed=7):
    """
    Assemble one (or more) cycle of synthetic data for the requested
    trajectory sequence.  Returns (DataFrame, list_of_(key, label, colour)).
    """
    from trajectories import TRAJECTORIES   # local import keeps the namespace clean

    rng = np.random.default_rng(seed)
    invalid = [k for k in traj_keys if k not in TRAJECTORIES]
    if invalid:
        print(f'[demo] Unknown trajectories: {invalid}  — ignoring.')
    keys = [k for k in traj_keys if k in TRAJECTORIES]
    if not keys:
        keys = DEFAULT_ORDER

    traj_meta = [(k, TRAJ_LABELS.get(k, k), TRAJ_PALETTE[i % len(TRAJ_PALETTE)])
                 for i, k in enumerate(keys)]

    segments   = []
    t_offset   = 0.0
    seg_bounds = []   # (t_start, t_end, traj_idx) for overlay annotation

    for cycle_idx in range(cycles):
        for ti, (key, label, colour) in enumerate(traj_meta):
            pts  = TRAJECTORIES[key]()
            t_r, x_mm, y_mm, pressing = _waypoints_to_series(pts)
            df_s = simulate_physics(t_r, x_mm, y_mm, pressing, rng)

            df_s = df_s.iloc[::step].reset_index(drop=True)
            df_s['t']         += t_offset
            df_s['timestamp'] += t_offset
            df_s['traj_idx']  = ti
            df_s['traj_name'] = label

            seg_bounds.append((float(df_s['t'].iloc[0]),
                               float(df_s['t'].iloc[-1]),
                               ti))
            segments.append(df_s)
            t_offset += t_r[-1] + TRANSITION_S

    df = pd.concat(segments, ignore_index=True)
    df['t'] = df['t'] - df['t'].iloc[0]

    dur  = df['t'].iloc[-1]
    rate = len(df) / dur
    print(f'[demo] {len(keys)} trajectory/ies  ×  {cycles} cycle(s)  '
          f'→  {len(df):,} frames  |  {dur:.1f} s  |  {rate:.1f} Hz  |  step={step}')

    return df, traj_meta, seg_bounds


# ─────────────────────────────────────────────────────────────────────────────
# Animation
# ─────────────────────────────────────────────────────────────────────────────

def build_animation(df, traj_meta, seg_bounds, step=1, speed=1.0):
    """Build and return (fig, FuncAnimation)."""
    cell_cols = [f'cell_{i+1}' for i in range(N)]
    has_force = 'fz'    in df.columns
    has_ai0   = 'ai0'   in df.columns and df['ai0'].abs().max() > 1e-6
    has_tcp   = 'tcp_x' in df.columns
    has_tq    = all(c in df.columns for c in ['tx', 'ty', 'tz'])

    fz_base = float(df['fz'].min())               if has_force else 0.0
    lc_base = float(_ai0_to_n(df['ai0']).min())   if has_ai0  else 0.0

    frames_df   = df.reset_index(drop=True)
    n_frames    = len(frames_df)
    total_t     = frames_df['t'].iloc[-1]

    avg_dt_s = total_t / max(n_frames - 1, 1)
    interval = max(10.0, avg_dt_s * 1000.0 / speed)

    # ── Layout ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 12), facecolor=BG)
    gs  = gridspec.GridSpec(
        5, 2, figure=fig,
        height_ratios=[9, 2.5, 2, 2, 0.5],
        width_ratios=[6, 4],
        hspace=0.18, wspace=0.15,
        left=0.04, right=0.97, top=0.93, bottom=0.04,
    )
    ax_hex   = fig.add_subplot(gs[0, 0])
    ax_traj  = fig.add_subplot(gs[0, 1])
    ax_hist  = fig.add_subplot(gs[1, :])
    ax_force = fig.add_subplot(gs[2, :])
    ax_tq    = fig.add_subplot(gs[3, :])
    ax_prog  = fig.add_subplot(gs[4, :])

    for ax in (ax_hex, ax_traj, ax_hist, ax_force, ax_tq, ax_prog):
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
        ax_hex.text(xmm, ymm + 2.2, f'P{IDX_TO_UR5.get(i, "?")}',
                    ha='center', va='center', fontsize=4.5,
                    color='#bbbbbb', alpha=0.8)

    ax_hex.set_xlim(-22, 22); ax_hex.set_ylim(-20, 20)
    ax_hex.set_aspect('equal'); ax_hex.axis('off')

    sm = ScalarMappable(cmap=CMAP, norm=Normalize(0, 1))
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax_hex, shrink=0.55, pad=0.02, label='Pressure')
    cb.ax.yaxis.label.set_color('white')
    cb.ax.tick_params(colors='white')

    hex_title = ax_hex.set_title('', fontsize=10, fontweight='bold',
                                  color='white', pad=5)

    # Trajectory legend (bottom of hex panel)
    legend_texts = []
    n_traj = len(traj_meta)
    for li, (_, label, colour) in enumerate(traj_meta):
        lx = -21 + li * (42 / max(n_traj - 1, 1))
        lt = ax_hex.text(lx, -19.5, f'{li+1}', ha='center', va='bottom',
                         fontsize=5.5, color='#555555',
                         fontweight='bold')
        legend_texts.append(lt)

    # ── TCP XY trajectory ─────────────────────────────────────────────────────
    # Pre-compute per-trajectory path data for colouring
    tx_all = df['tcp_x'].to_numpy() * 1000 if has_tcp else np.zeros(len(df))
    ty_all = df['tcp_y'].to_numpy() * 1000 if has_tcp else np.zeros(len(df))
    tidx_all = df['traj_idx'].to_numpy().astype(int)
    max_s_all = df[cell_cols].max(axis=1).to_numpy()

    if has_tcp:
        # Draw faint full-path background per trajectory
        for ti, (key, label, colour) in enumerate(traj_meta):
            mask = tidx_all == ti
            if mask.sum() < 2:
                continue
            ax_traj.plot(tx_all[mask], ty_all[mask],
                         color=colour, linewidth=1.0, alpha=0.18, zorder=1)
            # Label at centroid
            cx = tx_all[mask].mean()
            cy = ty_all[mask].mean()
            ax_traj.text(cx, cy, f'{ti+1}', fontsize=6,
                         color=colour, ha='center', va='center',
                         alpha=0.55, zorder=2)

        ax_traj.scatter(tx_all[0],  ty_all[0],  s=40, color='#2ecc71', zorder=5, label='Start')
        ax_traj.scatter(tx_all[-1], ty_all[-1], s=40, color='#e74c3c', zorder=5, label='End')

        traj_seg_data = np.array([tx_all, ty_all]).T.reshape(-1, 1, 2)
        traj_segs     = np.concatenate([traj_seg_data[:-1], traj_seg_data[1:]], axis=1)
        traj_lc = LineCollection([], cmap=CMAP,
                                 norm=Normalize(0, max(max_s_all.max(), 1e-6)),
                                 linewidth=1.8, zorder=3, alpha=0.9)
        ax_traj.add_collection(traj_lc)
        cb2 = fig.colorbar(traj_lc, ax=ax_traj, shrink=0.55, pad=0.02,
                           label='Sensor max')
        cb2.ax.yaxis.label.set_color('white')
        cb2.ax.tick_params(colors='white')

        pos_dot, = ax_traj.plot([], [], 'o', ms=8, color='white',
                                markeredgecolor='#dc0000', markeredgewidth=1.5, zorder=6)

        pad_x = max((tx_all.max() - tx_all.min()) * 0.15, 2.0)
        pad_y = max((ty_all.max() - ty_all.min()) * 0.15, 2.0)
        ax_traj.set_xlim(tx_all.min() - pad_x, tx_all.max() + pad_x)
        ax_traj.set_ylim(ty_all.min() - pad_y, ty_all.max() + pad_y)
    else:
        ax_traj.text(0.5, 0.5, 'No TCP data', ha='center', va='center',
                     transform=ax_traj.transAxes, fontsize=10, color='#666666')
        pos_dot  = ax_traj.plot([], [])[0]
        traj_lc  = None
        traj_segs = None

    ax_traj.set_xlabel('TCP X (mm)', fontsize=8, color='#aaaaaa')
    ax_traj.set_ylabel('TCP Y (mm)', fontsize=8, color='#aaaaaa')
    ax_traj.tick_params(colors='#aaaaaa', labelsize=7)
    ax_traj.set_title('TCP sliding trajectories', fontsize=9, color='white', pad=4)
    ax_traj.set_aspect('equal')
    ax_traj.grid(color=EDGE, alpha=0.4, linewidth=0.4)
    if has_tcp:
        ax_traj.legend(fontsize=6, facecolor=BG, labelcolor='white',
                       edgecolor=EDGE, loc='upper right')

    traj_name_text = ax_traj.text(
        0.02, 0.97, '', transform=ax_traj.transAxes,
        fontsize=9, fontweight='bold', color='white',
        va='top', ha='left',
        bbox=dict(facecolor='#222222', edgecolor='#555555', pad=3, alpha=0.85))

    # ── Rolling cell history ───────────────────────────────────────────────────
    HIST_WIN  = 200
    hist_buf  = np.zeros((N, HIST_WIN))
    hist_img  = ax_hist.imshow(
        hist_buf, aspect='auto', cmap=CMAP, vmin=0, vmax=1,
        extent=[0, HIST_WIN, N + 0.5, 0.5], interpolation='nearest')
    ax_hist.set_yticks(range(1, N + 1))
    ax_hist.set_yticklabels([f'P{p}' for p in range(1, N + 1)],
                            fontsize=5, color='#aaaaaa')
    ax_hist.set_xticks([])
    ax_hist.set_title('Cell history (last ~200 frames)', fontsize=8,
                      color='white', pad=3)

    # ── Force rolling window ───────────────────────────────────────────────────
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
        fp = max((combined.max() - combined.min()) * 0.15, 0.5)
        f_ymin, f_ymax = combined.min() - fp, combined.max() + fp
    else:
        f_ymin, f_ymax = -1.0, 1.0

    fz_line, = ax_force.plot(range(FW), fz_buf, color='#dc0000',
                              linewidth=1.1, label='Robot Fz (N)')
    lc_line, = ax_force.plot(range(FW), lc_buf, color='#9b59b6',
                              linewidth=1.0, label='Load cell (N)')
    ax_force.set_xlim(0, FW); ax_force.set_ylim(f_ymin, f_ymax)
    ax_force.set_xticks([])
    ax_force.set_ylabel('N', fontsize=7, color='#aaaaaa')
    ax_force.tick_params(axis='y', colors='#aaaaaa', labelsize=6)
    ax_force.set_title('Force — Robot Fz  vs  FUTEK load cell',
                       fontsize=8, color='white', pad=3)
    ax_force.axhline(0, color=EDGE, linewidth=0.5, linestyle='--')
    ax_force.grid(axis='y', color=EDGE, alpha=0.4, linewidth=0.5)
    ax_force.legend(fontsize=6, facecolor=BG, labelcolor='white',
                    edgecolor=EDGE, loc='upper left')

    # ── Torque rolling window ──────────────────────────────────────────────────
    tq_bufs  = {c: np.zeros(FW) for c in ['tx', 'ty', 'tz']}
    tmag_buf = np.zeros(FW)

    if has_tq:
        all_tq = np.concatenate([df[c].to_numpy() for c in ['tx', 'ty', 'tz']])
        tp = max((all_tq.max() - all_tq.min()) * 0.15, 0.01)
        t_ymin, t_ymax = all_tq.min() - tp, all_tq.max() + tp
        tmag_max = np.sqrt(sum(df[c].to_numpy() ** 2 for c in ['tx', 'ty', 'tz'])).max()
    else:
        t_ymin, t_ymax = -0.1, 0.1
        tmag_max = 0.1

    tq_lines = {}
    for c in ['tx', 'ty', 'tz']:
        ln, = ax_tq.plot(range(FW), tq_bufs[c],
                         color=TQ_COLORS[c], linewidth=1.0, label=c)
        tq_lines[c] = ln

    tmag_line, = ax_tq.plot(range(FW), tmag_buf, color='white',
                             linewidth=0.8, linestyle='--', alpha=0.6, label='|τ|')
    ax_tq_r = ax_tq.twinx()
    ax_tq_r.set_facecolor(BG)
    ax_tq_r.set_ylim(0, max(tmag_max * 1.2, 0.01))
    ax_tq_r.set_ylabel('|τ| (Nm)', fontsize=6, color='#888888')
    ax_tq_r.tick_params(axis='y', colors='#888888', labelsize=5)
    for sp in ax_tq_r.spines.values():
        sp.set_edgecolor(EDGE)

    ax_tq.set_xlim(0, FW); ax_tq.set_ylim(t_ymin, t_ymax)
    ax_tq.set_xticks([])
    ax_tq.set_ylabel('Nm', fontsize=7, color='#aaaaaa')
    ax_tq.tick_params(axis='y', colors='#aaaaaa', labelsize=6)
    ax_tq.set_title('Torque — Tx  Ty  Tz  (dashed = |τ| magnitude)',
                    fontsize=8, color='white', pad=3)
    ax_tq.axhline(0, color=EDGE, linewidth=0.5, linestyle='--')
    ax_tq.grid(axis='y', color=EDGE, alpha=0.4, linewidth=0.5)
    ax_tq.legend(fontsize=6, facecolor=BG, labelcolor='white',
                 edgecolor=EDGE, loc='upper left', ncol=4)

    # ── Progress bar ───────────────────────────────────────────────────────────
    (prog_rect,) = ax_prog.barh([0], [0], height=0.8, color='#2ab5a0')
    ax_prog.set_xlim(0, total_t); ax_prog.set_ylim(-0.5, 0.5)
    ax_prog.axis('off')
    prog_lbl = ax_prog.text(0.01 * total_t, 0, '0.0 s',
                            va='center', ha='left', fontsize=8, color='white')
    ax_prog.text(total_t, 0, f'{total_t:.0f} s',
                 va='center', ha='right', fontsize=7, color='#888888')

    # Trajectory segment markers on progress bar
    for t_s, t_e, ti in seg_bounds:
        c = traj_meta[ti % len(traj_meta)][2]
        ax_prog.axvspan(t_s, t_e, ymin=0.05, ymax=0.95,
                        color=c, alpha=0.18, zorder=0)
        ax_prog.axvline(t_s, color=c, linewidth=0.6, alpha=0.6, ymin=0.05, ymax=0.95)

    fig.suptitle('Star-Nose Sensor — Cyclic Trajectory Demo',
                 fontsize=13, fontweight='bold', color='white', y=0.98)

    # ── Cached numpy arrays ────────────────────────────────────────────────────
    cell_arr     = frames_df[cell_cols].to_numpy()
    t_arr        = frames_df['t'].to_numpy()
    pressing_arr = frames_df['ur5_pressing'].to_numpy().astype(int)
    traj_idx_arr = frames_df['traj_idx'].to_numpy().astype(int)
    traj_name_arr = frames_df['traj_name'].to_numpy()
    fz_arr       = (frames_df['fz'].to_numpy() - fz_base) if has_force else np.zeros(n_frames)
    lc_arr       = (_ai0_to_n(frames_df['ai0'].to_numpy()) - lc_base) if has_ai0 else np.zeros(n_frames)
    tcp_x_arr    = (frames_df['tcp_x'].to_numpy() * 1000) if has_tcp else np.zeros(n_frames)
    tcp_y_arr    = (frames_df['tcp_y'].to_numpy() * 1000) if has_tcp else np.zeros(n_frames)
    tq_arr       = {c: frames_df[c].to_numpy() if has_tq else np.zeros(n_frames)
                    for c in ['tx', 'ty', 'tz']}
    max_s_frame  = cell_arr.max(axis=1)

    _prev_traj_idx = [-1]   # mutable closure state to detect trajectory change

    # ── Update function ────────────────────────────────────────────────────────
    def update(fi):
        vals     = cell_arr[fi]
        t_now    = t_arr[fi]
        pressing = pressing_arr[fi] == 1
        ti_now   = int(traj_idx_arr[fi])
        tname    = traj_name_arr[fi]
        colour   = traj_meta[ti_now % len(traj_meta)][2]

        # ── Hex patches ───────────────────────────────────────────────────────
        for i, (patch, txt) in enumerate(zip(hex_patches, hex_texts)):
            si = POS_TO_SENSOR[i]
            v  = float(np.clip(vals[si], 0.0, 1.0))
            patch.set_facecolor(CMAP(v))
            patch.set_edgecolor('#ff3333' if pressing else EDGE)
            patch.set_linewidth(2.0 if pressing else 0.8)
            txt.set_text(f'{vals[si]:.2f}' if vals[si] > 0.02 else '')
            txt.set_color('white' if v > 0.45 else '#cccccc')

        # ── Hex title ─────────────────────────────────────────────────────────
        slide_str = '▶  SLIDING' if pressing else '  paused'
        parts = []
        if has_force:
            parts.append(f'Fz={fz_arr[fi]:.1f} N')
        if has_ai0:
            parts.append(f'LC={lc_arr[fi]:.1f} N')
        force_tag = ('   ' + '  |  '.join(parts)) if (parts and pressing) else ''
        hex_title.set_text(
            f't = {t_now:.2f} s   {slide_str}{force_tag}\n'
            f'[{ti_now + 1}/{len(traj_meta)}]  {tname}')
        hex_title.set_color(colour if pressing else '#888888')

        # ── Trajectory legend highlighting ─────────────────────────────────────
        for li, lt in enumerate(legend_texts):
            lt.set_color(traj_meta[li][2] if li == ti_now else '#444444')
            lt.set_fontsize(7 if li == ti_now else 5.5)

        # ── TCP trajectory ─────────────────────────────────────────────────────
        if has_tcp and traj_lc is not None:
            segs_to = traj_segs[:fi] if fi > 0 else traj_segs[:1]
            vals_to = max_s_frame[:fi] if fi > 0 else max_s_frame[:1]
            traj_lc.set_segments(segs_to)
            traj_lc.set_array(vals_to)
            pos_dot.set_data([tcp_x_arr[fi]], [tcp_y_arr[fi]])
            dot_col = CMAP(float(np.clip(max_s_frame[fi], 0.0, 1.0)))
            pos_dot.set_markerfacecolor(dot_col)

        traj_name_text.set_text(f'{ti_now + 1} / {len(traj_meta)}\n{tname}')
        traj_name_text.set_color(colour)

        # ── Rolling cell history ───────────────────────────────────────────────
        hist_buf[:, :-1] = hist_buf[:, 1:]
        hist_buf[:, -1]  = [vals[POINT_ORDER[j]] for j in range(N)]
        hist_img.set_data(hist_buf)

        # ── Force buffers ──────────────────────────────────────────────────────
        fz_buf[:-1] = fz_buf[1:]; fz_buf[-1] = float(fz_arr[fi])
        lc_buf[:-1] = lc_buf[1:]; lc_buf[-1] = float(lc_arr[fi])
        fz_line.set_ydata(fz_buf)
        lc_line.set_ydata(lc_buf)

        # ── Torque buffers ─────────────────────────────────────────────────────
        tmag = 0.0
        for c in ['tx', 'ty', 'tz']:
            v2 = float(tq_arr[c][fi])
            tq_bufs[c][:-1] = tq_bufs[c][1:]
            tq_bufs[c][-1]  = v2
            tq_lines[c].set_ydata(tq_bufs[c])
            tmag += v2 ** 2
        tmag = np.sqrt(tmag)
        tmag_buf[:-1] = tmag_buf[1:]; tmag_buf[-1] = tmag
        tmag_line.set_ydata(tmag_buf)
        ax_tq_r.set_ylim(0, max(tmag_buf.max() * 1.2, 0.01))

        # ── Progress bar ───────────────────────────────────────────────────────
        prog_rect.set_width(t_now)
        prog_lbl.set_text(f'{t_now:.1f} s')

        artists = (hex_patches + hex_texts + legend_texts +
                   [hex_title, hist_img, fz_line, lc_line, tmag_line,
                    prog_rect, prog_lbl, pos_dot, traj_name_text] +
                   list(tq_lines.values()))
        if traj_lc is not None:
            artists.append(traj_lc)
        return artists

    anim = FuncAnimation(fig, update, frames=n_frames,
                         interval=interval, blit=True, repeat=True)
    return fig, anim, interval


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description='Star-Nose cyclic trajectory demo (synthetic)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--traj',   nargs='+', default=DEFAULT_ORDER,
                   metavar='NAME',
                   help='Trajectory names to include (default: all)')
    p.add_argument('--list',   action='store_true',
                   help='Print available trajectory names and exit')
    p.add_argument('--cycles', type=int, default=1,
                   help='Number of full cycles to render (default: 1)')
    p.add_argument('--speed',  type=float, default=1.0,
                   help='Playback speed multiplier (default: 1.0)')
    p.add_argument('--step',   type=int,   default=1,
                   help='Use every Nth frame (default: 1)')
    p.add_argument('--save',   action='store_true',
                   help='Save animation to file')
    p.add_argument('--gif',    action='store_true',
                   help='Save as GIF (default: MP4)')
    p.add_argument('--seed',   type=int, default=7,
                   help='RNG seed for reproducibility (default: 7)')
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    if args.list:
        print('Available trajectories:')
        for k, lbl in TRAJ_LABELS.items():
            marker = ' *' if k in DEFAULT_ORDER else ''
            print(f'  {k:<18} {lbl}{marker}')
        print('  (* included in default cycle)')
        return

    matplotlib.rcParams.update({
        'figure.facecolor': BG,
        'text.color':       'white',
        'axes.facecolor':   BG,
        'axes.edgecolor':   EDGE,
    })

    df, traj_meta, seg_bounds = build_demo_df(
        traj_keys=args.traj,
        cycles=args.cycles,
        step=args.step,
        seed=args.seed,
    )

    print(f'[demo] Speed: {args.speed}×   Step: every {args.step} frame(s)')

    fig, anim, interval = build_animation(
        df, traj_meta, seg_bounds,
        step=args.step, speed=args.speed)
    fps = max(1, min(60, int(1000.0 / interval)))

    if args.save or args.gif:
        os.makedirs(PLOTS_DIR, exist_ok=True)
        use_gif = args.gif or not _HAS_FFMPEG
        if use_gif:
            out = os.path.join(PLOTS_DIR, 'demo_cyclic.gif')
            print(f'[demo] Saving GIF @ {fps} fps → {out}')
            anim.save(out, writer=PillowWriter(fps=fps))
        else:
            out = os.path.join(PLOTS_DIR, 'demo_cyclic.mp4')
            print(f'[demo] Saving MP4 @ {fps} fps → {out}')
            anim.save(out, writer=FFMpegWriter(fps=fps, bitrate=2500))
        print(f'[demo] Done → {out}')
    else:
        plt.show()


if __name__ == '__main__':
    try:
        import pandas, numpy, matplotlib
    except ImportError:
        print('Installing dependencies …')
        os.system(f'{sys.executable} -m pip install matplotlib pandas numpy')
    main()
