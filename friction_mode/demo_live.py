"""
demo_live.py  —  Star-Nose Sensor  |  Live Cyclic Trajectory Demo
=================================================================
Executes each trajectory on the real UR5 and displays live data from
the actual devices (capacitance sensor, UR5 F/T, FUTEK load cell).

Calibration : Integration_2/calib_short_6mm.json  (x=-2.5, y=+3.0, z=-11.0 mm)
Depth       : 7 mm  (displacement mode, override with --depth)
Sensor      : /dev/ttyACM0 — 19-cell capacitive array
F/T + AI0   : UR5 RTDE @ 177.22.22.2  (override: UR_ROBOT_IP env var)

Dashboard panels
----------------
  Top-left  : Hex map — live 19-cell capacitive activation
  Top-right : TCP XY — planned path outlines + growing live trail
  Mid       : Rolling cell activation history strip
  Lower-1   : Force rolling window — Robot Fz  vs  FUTEK load cell
  Lower-2   : Torque rolling window — Tx  Ty  Tz  + |τ| magnitude
  Bottom    : Trajectory progress bar

Usage
-----
  python demo_live.py                        # all 9 trajectories, 7 mm, 1 cycle
  python demo_live.py --depth 5              # 5 mm depth
  python demo_live.py --speed 6              # 6 mm/s sliding speed
  python demo_live.py --traj raster circle   # only raster and circle
  python demo_live.py --cycles 3             # repeat the full set 3 times
  python demo_live.py --no-robot             # sensor + display, no motion
  python demo_live.py --no-sensor            # robot + F/T display only
  python demo_live.py --list                 # print available trajectory names
"""

import os
import sys
import json
import argparse
import threading
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import RegularPolygon
from matplotlib.cm import ScalarMappable
from matplotlib.animation import FuncAnimation

# ── Path setup ────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
sys.path.insert(0, _INTEGRATION)
sys.path.insert(0, _HERE)

CALIB_FILE = os.path.join(_INTEGRATION, 'calib_short_6mm.json')

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
    1:16, 2:12,  3:7,
    4:17, 5:13,  6:8,  7:3,
    8:18, 9:14, 10:9, 11:4, 12:0,
   13:15,14:10, 15:5, 16:1,
   17:11,18:6,  19:2,
}
IDX_TO_UR5    = {v: k for k, v in UR5_TO_IDX.items()}
POS_TO_SENSOR = [UR5_TO_IDX[i + 1] for i in range(N)]
POINT_ORDER   = [UR5_TO_IDX[p]     for p in range(1, N + 1)]

# ── FUTEK load cell ────────────────────────────────────────────────────────────
AI0_ZERO_V       = 5.0
LOADCELL_MAX_N   = 10.0 * 4.44822   # 10 lb → N
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0

def _ai0_to_n(v):
    return -(float(v) - AI0_ZERO_V) * LOADCELL_N_PER_V

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

# ── Trajectory registry ────────────────────────────────────────────────────────
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

# ── Shared live state ─────────────────────────────────────────────────────────
HIST_WIN  = 200
TRAIL_MAX = 800

_lock = threading.Lock()
_live = {
    'cells':     np.zeros(N),
    'ft':        np.zeros(6),
    'ai0':       float(AI0_ZERO_V),
    'tcp':       np.zeros(6),
    'pressing':  False,
    'traj_idx':  0,
    'traj_name': '',
    'elapsed':   0.0,
    'done':      False,
}

# Rolling display buffers — written by sampler thread, read by animation.
# Occasional race condition on numpy slice ops is acceptable for display-only data.
_cell_hist = np.zeros((N, HIST_WIN))
_fz_buf    = np.zeros(HIST_WIN)
_lc_buf    = np.zeros(HIST_WIN)
_tq_bufs   = {c: np.zeros(HIST_WIN) for c in ['tx', 'ty', 'tz']}
_tmag_buf  = np.zeros(HIST_WIN)
_trail_x   = []
_trail_y   = []

_t0       = [time.time()]
_stop_evt = threading.Event()


# ─────────────────────────────────────────────────────────────────────────────
# Background threads
# ─────────────────────────────────────────────────────────────────────────────

def _sampler_loop(use_sensor, use_robot):
    """20 Hz — reads devices and pushes into rolling display buffers."""
    global _cell_hist, _fz_buf, _lc_buf, _tmag_buf

    if use_robot:
        import ur5_friction as ur5
    if use_sensor:
        import sensor as snsr

    while not _stop_evt.is_set():
        t0 = time.perf_counter()

        cells = np.array(snsr.get_values(), dtype=float) if use_sensor else np.zeros(N)

        if use_robot:
            st       = ur5.get_state()
            ft       = np.array(st['ft'],  dtype=float)
            tcp      = np.array(st['tcp'], dtype=float)
            ai0      = float(st['ai0'])
            pressing = bool(st['pressing'])
        else:
            ft       = np.zeros(6)
            tcp      = np.zeros(6)
            ai0      = float(AI0_ZERO_V)
            pressing = False

        fz_val = float(ft[2])
        lc_val = _ai0_to_n(ai0)
        tx_v   = float(ft[3])
        ty_v   = float(ft[4])
        tz_v   = float(ft[5])
        tmag   = float(np.sqrt(tx_v**2 + ty_v**2 + tz_v**2))

        # Shift rolling buffers left, append new sample on the right
        _cell_hist[:, :-1] = _cell_hist[:, 1:]
        _cell_hist[:, -1]  = [cells[POINT_ORDER[j]] for j in range(N)]

        _fz_buf[:-1] = _fz_buf[1:];  _fz_buf[-1] = fz_val
        _lc_buf[:-1] = _lc_buf[1:];  _lc_buf[-1] = lc_val
        for c, v in [('tx', tx_v), ('ty', ty_v), ('tz', tz_v)]:
            _tq_bufs[c][:-1] = _tq_bufs[c][1:]
            _tq_bufs[c][-1]  = v
        _tmag_buf[:-1] = _tmag_buf[1:];  _tmag_buf[-1] = tmag

        # Append to TCP trail only while the robot is actively sliding
        if pressing and np.linalg.norm(tcp[:3]) > 0.001:
            _trail_x.append(float(tcp[0]) * 1000.0)   # m → mm
            _trail_y.append(float(tcp[1]) * 1000.0)
            if len(_trail_x) > TRAIL_MAX:
                _trail_x.pop(0)
                _trail_y.pop(0)

        with _lock:
            _live['cells']    = cells.copy()
            _live['ft']       = ft.copy()
            _live['ai0']      = ai0
            _live['tcp']      = tcp.copy()
            _live['pressing'] = pressing
            _live['elapsed']  = time.time() - _t0[0]

        rem = 0.05 - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)


def _trajectory_runner(traj_keys, depth_mm, speed_mps, cycles, use_robot):
    """Runs each trajectory in sequence; updates _live traj_idx / traj_name."""
    import trajectories as traj_lib

    if use_robot:
        import ur5_friction as ur5
        with open(CALIB_FILE) as f:
            cal = json.load(f)
        ur5.set_calibration(cal['x_mm'], cal['y_mm'], cal['z_mm'])
        print(f'[demo] Calibration: X={cal["x_mm"]:+.1f}  Y={cal["y_mm"]:+.1f}  '
              f'Z={cal["z_mm"]:+.1f} mm')

    _t0[0] = time.time()
    cycle_iter = iter(range(cycles)) if cycles > 0 else iter(int, 1)  # infinite when cycles=0

    for cycle in cycle_iter:
        if _stop_evt.is_set():
            break
        for ti, key in enumerate(traj_keys):
            if _stop_evt.is_set():
                break

            label = TRAJ_LABELS.get(key, key)
            with _lock:
                _live['traj_idx']  = ti
                _live['traj_name'] = label

            pts = traj_lib.TRAJECTORIES[key]()
            n_pts = len(pts)
            print(f'\n[demo] cycle {cycle+1}/{cycles}  traj {ti+1}/{len(traj_keys)}'
                  f'  {label}  {n_pts} pts  depth={depth_mm:.1f}mm  '
                  f'{speed_mps*1000:.1f}mm/s')

            if use_robot:
                ur5.run_displacement_trajectory(pts, depth_mm, speed_mps)
            else:
                # Simulate wait time proportional to path length
                seg_len = sum(
                    np.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
                    for i in range(n_pts - 1))
                with _lock:
                    _live['pressing'] = True
                time.sleep(max(3.0, seg_len / (speed_mps * 1000.0)))
                with _lock:
                    _live['pressing'] = False

    with _lock:
        _live['done'] = True
    print('\n[demo] All trajectories complete')


# ─────────────────────────────────────────────────────────────────────────────
# Dashboard
# ─────────────────────────────────────────────────────────────────────────────

def build_animation(traj_keys, planned_pts, depth_mm):
    n_traj    = len(traj_keys)
    traj_meta = [(k, TRAJ_LABELS.get(k, k), TRAJ_PALETTE[i % len(TRAJ_PALETTE)])
                 for i, k in enumerate(traj_keys)]

    matplotlib.rcParams.update({
        'figure.facecolor': BG, 'text.color': 'white',
        'axes.facecolor':   BG, 'axes.edgecolor': EDGE,
    })

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

    ax_hex.set_xlim(-22, 22);  ax_hex.set_ylim(-20, 20)
    ax_hex.set_aspect('equal'); ax_hex.axis('off')
    sm = ScalarMappable(cmap=CMAP, norm=Normalize(0, 1))
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax_hex, shrink=0.55, pad=0.02, label='Pressure')
    cb.ax.yaxis.label.set_color('white');  cb.ax.tick_params(colors='white')

    hex_title = ax_hex.set_title('', fontsize=10, fontweight='bold',
                                  color='white', pad=5)

    legend_texts = []
    for li, (_, label, colour) in enumerate(traj_meta):
        lx = -21 + li * (42 / max(n_traj - 1, 1))
        lt = ax_hex.text(lx, -19.5, f'{li+1}', ha='center', va='bottom',
                         fontsize=5.5, color='#555555', fontweight='bold')
        legend_texts.append(lt)

    # ── TCP XY trajectory panel ────────────────────────────────────────────────
    for ti, (key, label, colour) in enumerate(traj_meta):
        pts = planned_pts[key]
        xs  = [p[0] for p in pts];  ys = [p[1] for p in pts]
        ax_traj.plot(xs, ys, color=colour, linewidth=0.9, alpha=0.2, zorder=1)
        ax_traj.text(float(np.mean(xs)), float(np.mean(ys)), f'{ti+1}',
                     fontsize=6, color=colour, ha='center', va='center',
                     alpha=0.6, zorder=2)

    trail_line, = ax_traj.plot([], [], linewidth=1.6, alpha=0.85, zorder=3)
    pos_dot,    = ax_traj.plot([], [], 'o', ms=8, color='white',
                                markeredgecolor='#dc0000', markeredgewidth=1.5,
                                zorder=5)

    all_xs = [p[0] for pts in planned_pts.values() for p in pts]
    all_ys = [p[1] for pts in planned_pts.values() for p in pts]
    pad_x  = max((max(all_xs) - min(all_xs)) * 0.18, 3.0)
    pad_y  = max((max(all_ys) - min(all_ys)) * 0.18, 3.0)
    ax_traj.set_xlim(min(all_xs) - pad_x, max(all_xs) + pad_x)
    ax_traj.set_ylim(min(all_ys) - pad_y, max(all_ys) + pad_y)
    ax_traj.set_xlabel('TCP X (mm)', fontsize=8, color='#aaaaaa')
    ax_traj.set_ylabel('TCP Y (mm)', fontsize=8, color='#aaaaaa')
    ax_traj.tick_params(colors='#aaaaaa', labelsize=7)
    ax_traj.set_title('TCP sliding trajectories', fontsize=9, color='white', pad=4)
    ax_traj.set_aspect('equal')
    ax_traj.grid(color=EDGE, alpha=0.4, linewidth=0.4)

    traj_name_text = ax_traj.text(
        0.02, 0.97, '', transform=ax_traj.transAxes,
        fontsize=9, fontweight='bold', color='white', va='top', ha='left',
        bbox=dict(facecolor='#222222', edgecolor='#555555', pad=3, alpha=0.85))

    # ── Rolling cell history strip ─────────────────────────────────────────────
    hist_img = ax_hist.imshow(
        _cell_hist, aspect='auto', cmap=CMAP, vmin=0, vmax=1,
        extent=[0, HIST_WIN, N + 0.5, 0.5], interpolation='nearest')
    ax_hist.set_yticks(range(1, N + 1))
    ax_hist.set_yticklabels([f'P{p}' for p in range(1, N + 1)],
                            fontsize=5, color='#aaaaaa')
    ax_hist.set_xticks([])
    ax_hist.set_title('Cell history (last ~200 frames)', fontsize=8,
                      color='white', pad=3)

    # ── Force rolling window ───────────────────────────────────────────────────
    fz_line, = ax_force.plot(range(HIST_WIN), _fz_buf, color='#dc0000',
                              linewidth=1.1, label='Robot Fz (N)')
    lc_line, = ax_force.plot(range(HIST_WIN), _lc_buf, color='#9b59b6',
                              linewidth=1.0, label='Load cell (N)')
    ax_force.set_xlim(0, HIST_WIN);  ax_force.set_ylim(-15.0, 15.0)
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
    tq_lines = {}
    for c in ['tx', 'ty', 'tz']:
        ln, = ax_tq.plot(range(HIST_WIN), _tq_bufs[c],
                         color=TQ_COLORS[c], linewidth=1.0, label=c)
        tq_lines[c] = ln
    tmag_line, = ax_tq.plot(range(HIST_WIN), _tmag_buf, color='white',
                             linewidth=0.8, linestyle='--', alpha=0.6, label='|τ|')

    ax_tq_r = ax_tq.twinx()
    ax_tq_r.set_facecolor(BG)
    ax_tq_r.set_ylim(0, 0.5)
    ax_tq_r.set_ylabel('|τ| (Nm)', fontsize=6, color='#888888')
    ax_tq_r.tick_params(axis='y', colors='#888888', labelsize=5)
    for sp in ax_tq_r.spines.values():
        sp.set_edgecolor(EDGE)

    ax_tq.set_xlim(0, HIST_WIN);  ax_tq.set_ylim(-0.5, 0.5)
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
    ax_prog.set_xlim(0, n_traj);  ax_prog.set_ylim(-0.5, 0.5)
    ax_prog.axis('off')
    prog_lbl = ax_prog.text(0.5, 0, '', va='center', ha='center',
                            fontsize=9, color='white',
                            transform=ax_prog.transAxes)
    for ti, (_, label, colour) in enumerate(traj_meta):
        ax_prog.axvspan(ti, ti + 1, ymin=0.05, ymax=0.95,
                        color=colour, alpha=0.18, zorder=0)
        ax_prog.axvline(ti, color=colour, linewidth=0.6, alpha=0.6,
                        ymin=0.05, ymax=0.95)

    fig.suptitle(
        f'Star-Nose Sensor — Live Demo  '
        f'(calib: short_6mm  |  depth: {depth_mm:.0f} mm)',
        fontsize=12, fontweight='bold', color='white', y=0.98)

    # ── Animation update ───────────────────────────────────────────────────────
    def update(_frame):
        with _lock:
            cells    = _live['cells'].copy()
            ft       = _live['ft'].copy()
            ai0      = _live['ai0']
            pressing = _live['pressing']
            ti_now   = int(_live['traj_idx'])
            tname    = _live['traj_name']
            elapsed  = _live['elapsed']

        colour = TRAJ_PALETTE[ti_now % len(TRAJ_PALETTE)]

        # Hex patches
        for i, (patch, txt) in enumerate(zip(hex_patches, hex_texts)):
            si = POS_TO_SENSOR[i]
            v  = float(np.clip(cells[si], 0.0, 1.0))
            patch.set_facecolor(CMAP(v))
            patch.set_edgecolor('#ff3333' if pressing else EDGE)
            patch.set_linewidth(2.0 if pressing else 0.8)
            txt.set_text(f'{cells[si]:.2f}' if cells[si] > 0.02 else '')
            txt.set_color('white' if v > 0.45 else '#cccccc')

        fz_val    = float(ft[2])
        lc_val    = _ai0_to_n(ai0)
        slide_str = '▶  SLIDING' if pressing else '  paused  '
        force_tag = f'   Fz={fz_val:+.1f} N  |  LC={lc_val:.1f} N' if pressing else ''
        hex_title.set_text(
            f't = {elapsed:.1f} s   {slide_str}{force_tag}\n'
            f'[{ti_now + 1}/{n_traj}]  {tname}')
        hex_title.set_color(colour if pressing else '#888888')

        for li, lt in enumerate(legend_texts):
            lt.set_color(TRAJ_PALETTE[li % len(TRAJ_PALETTE)] if li == ti_now else '#444444')
            lt.set_fontsize(7 if li == ti_now else 5.5)

        # TCP trail
        trail_line.set_data(_trail_x, _trail_y)
        trail_line.set_color(colour)
        if _trail_x:
            pos_dot.set_data([_trail_x[-1]], [_trail_y[-1]])
            pos_dot.set_markerfacecolor(colour)
        else:
            pos_dot.set_data([], [])

        traj_name_text.set_text(f'{ti_now + 1}/{n_traj}\n{tname}')
        traj_name_text.set_color(colour)

        # Rolling history strip
        hist_img.set_data(_cell_hist)

        # Force panel
        fz_line.set_ydata(_fz_buf)
        lc_line.set_ydata(_lc_buf)
        fv  = np.concatenate([_fz_buf, _lc_buf])
        fp  = max(abs(fv).max() * 0.15, 0.5)
        ax_force.set_ylim(fv.min() - fp, fv.max() + fp)

        # Torque panel
        for c in ['tx', 'ty', 'tz']:
            tq_lines[c].set_ydata(_tq_bufs[c])
        tmag_line.set_ydata(_tmag_buf)
        ax_tq_r.set_ylim(0, max(_tmag_buf.max() * 1.2, 0.01))
        tq_all = np.concatenate(list(_tq_bufs.values()))
        tp     = max(abs(tq_all).max() * 0.15, 0.01)
        ax_tq.set_ylim(tq_all.min() - tp, tq_all.max() + tp)

        # Progress bar
        prog_rect.set_width(ti_now + (1.0 if pressing else 0.0))
        prog_lbl.set_text(f'Trajectory  {ti_now + 1} / {n_traj}  —  {tname}')

    anim = FuncAnimation(fig, update, interval=50, blit=False,
                         cache_frame_data=False)
    return fig, anim


# ─────────────────────────────────────────────────────────────────────────────
# CLI / Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Star-Nose Sensor — live cyclic trajectory demo',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--traj',   nargs='+', default=DEFAULT_ORDER, metavar='NAME',
                   help='Trajectory names to run (default: all 9)')
    p.add_argument('--depth',  type=float, default=7.0,
                   help='Indentation depth in mm (default: 7)')
    p.add_argument('--speed',  type=float, default=8.0,
                   help='Sliding speed in mm/s (default: 8)')
    p.add_argument('--cycles', type=int,   default=1,
                   help='Number of full cycles (default: 1)')
    p.add_argument('--loop',   action='store_true',
                   help='Loop forever (overrides --cycles)')
    p.add_argument('--no-robot',  action='store_true', help='Skip robot motion')
    p.add_argument('--no-sensor', action='store_true', help='Skip tactile sensor')
    p.add_argument('--list', action='store_true',
                   help='Print available trajectory names and exit')
    return p.parse_args()


def main():
    args = parse_args()

    if args.list:
        print('Available trajectories:')
        for k, lbl in TRAJ_LABELS.items():
            marker = ' *' if k in DEFAULT_ORDER else ''
            print(f'  {k:<18} {lbl}{marker}')
        print('  (* included in default cycle)')
        return

    use_robot  = not args.no_robot
    use_sensor = not args.no_sensor
    speed_mps  = args.speed / 1000.0
    cycles     = 0 if args.loop else args.cycles  # 0 = infinite

    traj_keys = [k for k in args.traj if k in TRAJ_LABELS]
    if not traj_keys:
        print('[demo] No valid trajectory names — using default order')
        traj_keys = DEFAULT_ORDER

    print('=' * 64)
    print('  Star-Nose Sensor — Live Cyclic Demo')
    print('=' * 64)
    print(f'  Calibration  : calib_short_6mm  (x=-2.5  y=+3.0  z=-11.0 mm)')
    print(f'  Depth        : {args.depth:.1f} mm')
    print(f'  Speed        : {args.speed:.1f} mm/s')
    print(f'  Trajectories : {len(traj_keys)}  × {"∞" if args.loop else args.cycles} cycle(s)')
    print(f'  Robot        : {"ON — " + os.environ.get("UR_ROBOT_IP","177.22.22.2") if use_robot else "OFF (--no-robot)"}')
    print(f'  Sensor       : {"ON — /dev/ttyACM0" if use_sensor else "OFF (--no-sensor)"}')
    print('=' * 64)

    # Start capacitive sensor (non-blocking serial reader)
    if use_sensor:
        import sensor as snsr
        snsr.start()
        print('[demo] Waiting for sensor calibration...')
        if not snsr.wait_until_ready(timeout=60):
            print('[demo] ERROR: sensor not ready — check /dev/ttyACM0 '
                  'or use --no-sensor')
            return
        print('[demo] Sensor ready')

    # Pre-compute planned waypoints for trajectory panel background outlines
    import trajectories as traj_lib
    planned_pts = {k: traj_lib.TRAJECTORIES[k]() for k in traj_keys}

    # Trajectory runner — moves robot through each trajectory in sequence
    traj_thread = threading.Thread(
        target=_trajectory_runner,
        args=(traj_keys, args.depth, speed_mps, cycles, use_robot),
        daemon=True)
    traj_thread.start()

    # Sampler — reads devices at 20 Hz into rolling display buffers
    sampler_thread = threading.Thread(
        target=_sampler_loop,
        args=(use_sensor, use_robot),
        daemon=True)
    sampler_thread.start()

    # Build and display live dashboard
    fig, anim = build_animation(traj_keys, planned_pts, args.depth)
    print('\n[demo] Dashboard open — close window or Ctrl+C to stop\n')

    try:
        plt.show()
    except KeyboardInterrupt:
        print('\n[demo] Stopped by user')
    finally:
        _stop_evt.set()
        if use_robot:
            import ur5_friction as ur5
            ur5.request_stop()
        traj_thread.join(timeout=20)
        print('[demo] Done')


if __name__ == '__main__':
    main()
