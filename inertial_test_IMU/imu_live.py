"""
imu_live.py  —  Inertial (IMU) Test  |  Live Trajectory Viewer + Controller
==========================================================================
Game-style live driver for the IMU trajectories, in the spirit of
friction_mode/friction_live.py but on a matplotlib dashboard and with NO
sensors (no capacitance, no FUTEK, no calibration press). You fly the robot
through position-controlled paths and steer everything from the keyboard.

Keyboard (window focused)
-------------------------
  , / .        previous / next trajectory in the library (JSON + generated)
  [ / ]        manual YAW offset − / +   ( { / } for big steps )
  y            reset the yaw offset to 0
  ← → ↑ ↓      move the whole path (central point), live
  shift+arrow  move the central point in big steps (×5)
  - / =        smaller / larger central-point step
  n            reset central point to the reference origin (0,0)
  space        RUN the selected trajectory  /  STOP if already running
  c            CONFIRM the calibrated yaw → rise to work height and run
  backspace    stop and return home
  r            re-scan the JSON library folder
  mouse        click / drag on the XY panel to place the central point

The robot ALWAYS starts at the trajectory's own first waypoint (fixed start).
The manual yaw offset sets the initial orientation — previewed as the red
heading arrow at the start point while idle — and every waypoint's yaw is then
applied relative to it. Switching trajectory while running restarts the motion;
centre and yaw changes apply live without restarting.

Yaw calibration at the start (default ON, disable with --no-confirm)
--------------------------------------------------------------------
At the start of every run the robot travels to the first waypoint, descends to
the CALIBRATION plane (-20 mm, below the reference) and zeroes WRIST 3 so yaw
starts from a repeatable mechanical zero. It then PAUSES there: adjust the yaw
with [ / ] ({ / } for big steps) — the tool re-points live so you can verify /
align it. Press `c` (or type "confirm") to accept; only then does the tool rise
to the commanded work height and slide into the trajectory. Waypoint yaws are
applied relative to the calibrated start, so the path begins without a jump.

Library
-------
  Calibrated UR-frame JSONs from trajectories_ur/ (run at their true table
  position) PLUS the generated shapes (circle, spiral, raster, ...). Override
  the folder with --lib-dir, preselect with --json FILE or --traj NAME.

Usage
-----
  python imu_live.py                       # library viewer, --no-robot to simulate
  python imu_live.py --no-robot            # simulate motion, no robot
  python imu_live.py --traj spiral --size 120
  python imu_live.py --json trajectories_ur/star_5_arm_hand_drawn.json
  python imu_live.py --lib-dir imported_trajectories
  python imu_live.py --list
"""
import os
import sys
import glob
import time
import argparse
import threading

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_trajectories as traj

_HERE = os.path.dirname(os.path.abspath(__file__))

# ── Appearance ─────────────────────────────────────────────────────────────────
BG   = '#111111'
EDGE = '#444444'
ACCENT   = '#2ab5a0'
TRAIL_C  = '#33e666'
PLAN_C   = '#3498db'

TRAIL_MAX = 4000
HIST_WIN  = 400

# ── Shared live state ──────────────────────────────────────────────────────────
_lock = threading.Lock()
_live = {
    'x_mm': 0.0, 'y_mm': 0.0, 'z_mm': 0.0, 'yaw': None,
    'wp': 0, 'wp_total': 0, 'moving': False, 'done': False, 'elapsed': 0.0,
    'awaiting_confirm': False,
}
_trail_x, _trail_y = [], []
_x_buf = np.zeros(HIST_WIN)
_y_buf = np.zeros(HIST_WIN)

# Live central-point offset (mm), driven by arrow keys / mouse.
_center_offset = [0.0, 0.0]

# Live manual yaw offset (deg) — the initial orientation, set by hand; the tool
# then follows each waypoint's yaw relative to it.
_yaw_offset = [0.0]
YAW_STEP_DEG = 5.0

# Height (mm, below reference) at which the yaw is calibrated before each run;
# mirrors ur5_imu.CALIB_HEIGHT_MM for the simulated path. On confirm the tool
# rises to the commanded work height.
CALIB_HEIGHT_MM = -20.0

# Interactive control state (all mutated from the key handler).
_ctrl = {
    'sel': 0,
    'run_req': False, 'running': False, 'restart': False,
    'step': 5.0, 'label': '',
}
# Current selected trajectory drawn as the plan (offset frame). The robot ALWAYS
# starts at waypoint 0 (the trajectory's own fixed start); yaw0 is that start
# waypoint's yaw, used to preview the initial orientation before running.
_plan = {'base': [(0.0, 0.0)], 'yaw0': None, 'label': '', 'absolute': False}

# Library + config, filled by main().
_LIB = []
_cfg = {'use_robot': False, 'size': traj.DEFAULT_SPAN_MM, 'lib_dir': '',
        'speed_mps': 0.03, 'height_mm': 30.0, 'confirm': True}

# Signals the operator's "confirm start orientation" in simulation (--no-robot).
_sim_confirm = threading.Event()

# Typed command console ("pseudo terminal") — Enter opens it, type e.g.
# "speed 40" / "height 25", Enter submits, Esc closes.
_console = {'on': False, 'buf': '', 'msg': ''}

_t0       = [time.time()]
_stop_evt = threading.Event()


def _clamp_center(x, y):
    """Keep the central-point NUDGE within the working-area window."""
    half = traj.WORKAREA_MAX_MM / 2.0
    return max(-half, min(half, x)), max(-half, min(half, y))


# ─────────────────────────────────────────────────────────────────────────────
# Library / trajectory building
# ─────────────────────────────────────────────────────────────────────────────

def _build_library(lib_dir, extra_json=None):
    """Combined library: calibrated JSON files in lib_dir + generated shapes."""
    lib = []
    if lib_dir and os.path.isdir(lib_dir):
        for f in sorted(glob.glob(os.path.join(lib_dir, '*.json'))):
            lib.append({'kind': 'json', 'path': f,
                        'label': 'JSON: ' + os.path.basename(f)})
    for name in traj.TRAJECTORIES:
        lib.append({'kind': 'gen', 'name': name,
                    'label': 'gen: ' + traj.LABELS.get(name, name)})
    if extra_json:
        ap = os.path.abspath(extra_json)
        if not any(e.get('path') and os.path.abspath(e['path']) == ap
                   for e in lib):
            lib.insert(0, {'kind': 'json', 'path': extra_json,
                           'label': 'JSON: ' + os.path.basename(extra_json)})
    return lib


def _build_entry(entry, size):
    """
    Build a library entry into (pts, absolute) in the reference-offset frame.
    The path is kept as authored — the robot always starts at waypoint 0.
    """
    import import_trajectory as imp
    import ur5_imu as ur5
    if entry['kind'] == 'json':
        pts, absolute = imp.load_trajectory(entry['path'], span_mm=size, fill=True)
        if absolute:
            pts = [(ur5.ur_to_offset(p[0], p[1]) + tuple(p[2:])) for p in pts]
    else:
        pts = traj.build(entry['name'], size_mm=size)
        absolute = False
    return pts, absolute


def _refresh_plan():
    """Rebuild the previewed plan for the current selection."""
    with _lock:
        sel = _ctrl['sel']
    entry = _LIB[sel]
    try:
        pts, absolute = _build_entry(entry, _cfg['size'])
    except Exception as e:                                   # bad JSON, etc.
        print(f"[imu-live] Could not load {entry.get('label')}: {e}")
        pts, absolute = [(0.0, 0.0)], False
    with _lock:
        _plan['base'] = [(p[0], p[1]) for p in pts]
        _plan['yaw0'] = (pts[0][2] if len(pts[0]) >= 3 else None)
        _plan['label'] = entry['label']
        _plan['absolute'] = absolute
        _ctrl['label'] = entry['label']


def _workarea_polygon():
    """Return (closed polygon, label) of the real working area in offset coords.
    Prefers the calibrated 23 cm square; falls back to the axis-aligned box."""
    try:
        import ur_calibration as cal
        import ur5_imu as ur5
        poly = [ur5.ur_to_offset(x, y) for (x, y) in cal.corners()]
        poly.append(poly[0])
        side = cal.SCALE * 230.0
        return poly, f'{side / 10:.0f} cm working area (calibrated)'
    except Exception:
        h = traj.WORKAREA_MAX_MM / 2.0
        return ([(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)],
                f'{traj.WORKAREA_MAX_MM / 10:.0f} cm working area')


def _lib_extent(size):
    """Max |x|,|y| over every library entry + the working area — for axis limits."""
    m = traj.WORKAREA_MAX_MM / 2.0
    for e in _LIB:
        try:
            pts, _ = _build_entry(e, size)
        except Exception:
            continue
        for p in pts:
            m = max(m, abs(p[0]), abs(p[1]))
    for p in _workarea_polygon()[0]:
        m = max(m, abs(p[0]), abs(p[1]))
    return m * 1.10 + 8.0


# ─────────────────────────────────────────────────────────────────────────────
# Control actions (called from the key/mouse handlers)
# ─────────────────────────────────────────────────────────────────────────────

def _apply_center(x, y):
    x, y = _clamp_center(x, y)
    _center_offset[0] = x
    _center_offset[1] = y
    if _cfg['use_robot']:
        import ur5_imu as ur5
        ur5.set_center_offset(x, y)


def _maybe_restart():
    """If a run is in progress, restart it with the new selection/start."""
    with _lock:
        running = _ctrl['running'] or _ctrl['run_req']
    if running:
        with _lock:
            _ctrl['restart'] = True
        if _cfg['use_robot']:
            import ur5_imu as ur5
            ur5.request_stop()


def _select(delta):
    with _lock:
        _ctrl['sel'] = (_ctrl['sel'] + delta) % len(_LIB)
    _refresh_plan()
    _maybe_restart()


def _apply_yaw(deg):
    """Set the manual initial-yaw offset (deg); applies live, no restart."""
    deg = ((deg + 180.0) % 360.0) - 180.0        # wrap to [-180, 180)
    _yaw_offset[0] = deg
    if _cfg['use_robot']:
        import ur5_imu as ur5
        ur5.set_yaw_offset(deg)


def _set_speed(mm_s):
    """Set the live trajectory speed (mm/s); applies mid-run on the robot."""
    v = max(1.0, min(400.0, float(mm_s)))
    _cfg['speed_mps'] = v / 1000.0
    if _cfg['use_robot']:
        import ur5_imu as ur5
        ur5.set_speed(v / 1000.0)
    return v


def _set_height(mm):
    """Set the live work-plane height (mm); applies mid-run on the robot.
    Range ±50 mm about the reference pose (negative = below reference)."""
    v = max(-50.0, min(50.0, float(mm)))
    _cfg['height_mm'] = v
    if _cfg['use_robot']:
        import ur5_imu as ur5
        ur5.set_height_mm(v)
    return v


def _run_command(text):
    """Parse a console command line. Returns a short status message."""
    s = text.strip()
    if not s:
        return ''
    p = s.split()
    c = p[0].lower()
    try:
        if c in ('speed', 'spd', 's') and len(p) > 1:
            return f'speed = {_set_speed(p[1]):.0f} mm/s'
        if c in ('height', 'hgt', 'h', 'z') and len(p) > 1:
            return f'height = {_set_height(p[1]):.0f} mm'
        if c == 'step' and len(p) > 1:
            _ctrl['step'] = max(0.5, min(40.0, float(p[1])))
            return f'centre step = {_ctrl["step"]:.1f} mm'
        if c == 'yaw' and len(p) > 1:
            _apply_yaw(float(p[1]))
            return f'yaw offset = {_yaw_offset[0]:.0f} deg'
        if c in ('run', 'go', 'start'):
            with _lock:
                active = _ctrl['run_req'] or _ctrl['running']
            if not active:
                _toggle_run()
            return 'running'
        if c in ('stop', 'halt', 'home'):
            _stop_home()
            return 'stopped'
        if c in ('confirm', 'ok', 'c'):
            _confirm_start()
            return 'start orientation confirmed'
        if c in ('sel', 'select', 'traj') and len(p) > 1:
            arg = p[1]
            if arg.isdigit():
                with _lock:
                    _ctrl['sel'] = max(0, min(len(_LIB) - 1, int(arg) - 1))
            else:
                import re
                a = arg.lower()
                def _toks(lbl):
                    return re.split(r'[^a-z0-9]+', lbl.lower())
                # Prefer a whole-token match ("star" won't match "start"),
                # then fall back to a substring match.
                idx = next((i for i, e in enumerate(_LIB)
                            if a in _toks(e['label'])), None)
                if idx is None:
                    idx = next((i for i, e in enumerate(_LIB)
                                if a in e['label'].lower()), None)
                if idx is None:
                    return f'no traj matches "{arg}"'
                with _lock:
                    _ctrl['sel'] = idx
            _refresh_plan(); _maybe_restart()
            return f'-> {_ctrl["label"]}'
        return f'? unknown: {s}'
    except ValueError:
        return f'? bad number in: {s}'


def _toggle_run():
    with _lock:
        active = _ctrl['run_req'] or _ctrl['running']
    if active:
        _stop_home()
    else:
        with _lock:
            _ctrl['run_req'] = True
            _ctrl['restart'] = False


def _confirm_start():
    """Confirm the start orientation so a paused run proceeds into the path."""
    _sim_confirm.set()
    if _cfg['use_robot']:
        import ur5_imu as ur5
        ur5.confirm_start()


def _stop_home():
    with _lock:
        _ctrl['run_req'] = False
        _ctrl['restart'] = False
    _sim_confirm.set()            # release a pending sim confirmation wait
    if _cfg['use_robot']:
        import ur5_imu as ur5
        ur5.request_stop()


def _rescan():
    global _LIB
    _LIB = _build_library(_cfg['lib_dir'])
    with _lock:
        _ctrl['sel'] = min(_ctrl['sel'], len(_LIB) - 1)
    _refresh_plan()
    print(f"[imu-live] Library re-scanned — {len(_LIB)} trajectories")


# ─────────────────────────────────────────────────────────────────────────────
# Background threads
# ─────────────────────────────────────────────────────────────────────────────

def _sampler_loop(use_robot):
    """20 Hz — read TCP pose (or simulate) into the rolling display buffers."""
    ur5 = None
    if use_robot:
        import ur5_imu as ur5

    while not _stop_evt.is_set():
        t0 = time.perf_counter()

        if use_robot:
            st  = ur5.get_state()
            tcp = st['tcp']
            x_mm = (tcp[0] - ur5.REFERENCE_POSE[0]) * 1000.0
            y_mm = (tcp[1] - ur5.REFERENCE_POSE[1]) * 1000.0
            z_mm = (tcp[2] - ur5.REFERENCE_POSE[2]) * 1000.0
            moving   = st['moving']; wp = st['wp']
            wp_total = st['wp_total']; done = st['done']; yaw = st.get('yaw')
            awaiting = st.get('awaiting_confirm', False)
        else:
            x_mm = _live['x_mm']; y_mm = _live['y_mm']; z_mm = _live['z_mm']
            moving = _live['moving']; wp = _live['wp']
            wp_total = _live['wp_total']; done = _live['done']; yaw = _live['yaw']
            awaiting = _live['awaiting_confirm']

        _x_buf[:-1] = _x_buf[1:];  _x_buf[-1] = x_mm
        _y_buf[:-1] = _y_buf[1:];  _y_buf[-1] = y_mm

        if moving:
            _trail_x.append(x_mm); _trail_y.append(y_mm)
            if len(_trail_x) > TRAIL_MAX:
                _trail_x.pop(0); _trail_y.pop(0)

        with _lock:
            _live.update(x_mm=x_mm, y_mm=y_mm, z_mm=z_mm, yaw=yaw,
                         moving=moving, wp=wp, wp_total=wp_total, done=done,
                         awaiting_confirm=awaiting,
                         elapsed=time.time() - _t0[0])

        rem = 0.05 - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)


def _sim_run(pts):
    """Simulate motion through pts, cancellable on stop / switch.
    Reads live speed / height from _cfg each waypoint."""
    # Calibrate the yaw at the -20 mm plane and wait for the operator to confirm
    # (mirrors the robot: wrist-3 zero origin, yaw measured from 0). On confirm
    # the tool rises to the commanded work height and the path runs.
    if _cfg['confirm']:
        sx, sy = _clamp_center(pts[0][0] + _center_offset[0],
                               pts[0][1] + _center_offset[1])
        with _lock:
            _live['x_mm'] = sx; _live['y_mm'] = sy
            _live['z_mm'] = CALIB_HEIGHT_MM; _live['wp'] = 0
            _live['yaw'] = _yaw_offset[0]
            _live['moving'] = False; _live['awaiting_confirm'] = True
        _sim_confirm.clear()
        while not (_sim_confirm.is_set() or _stop_evt.is_set()
                   or _ctrl['restart'] or not _ctrl['run_req']):
            with _lock:
                _live['yaw'] = _yaw_offset[0]       # live calibration preview
            time.sleep(0.05)
        with _lock:
            _live['awaiting_confirm'] = False
        if (_stop_evt.is_set() or _ctrl['restart'] or not _ctrl['run_req']):
            return
    # Waypoint yaws are applied relative to the start waypoint's authored yaw so
    # the path begins exactly at the calibrated orientation (matches the robot).
    base0 = pts[0][2] if len(pts[0]) >= 3 else None
    b0 = base0 if base0 is not None else 0.0
    with _lock:
        _live['moving'] = True
    for i, wp in enumerate(pts):
        if _stop_evt.is_set() or _ctrl['restart'] or not _ctrl['run_req']:
            break
        x, y = wp[0], wp[1]
        bw = wp[2] if len(wp) >= 3 else None
        rel = (bw - b0) if bw is not None else 0.0
        yaw = rel + _yaw_offset[0]
        ox, oy = _clamp_center(x + _center_offset[0], y + _center_offset[1])
        with _lock:
            _live['x_mm'] = ox; _live['y_mm'] = oy
            _live['z_mm'] = _cfg['height_mm']; _live['wp'] = i + 1
            _live['yaw'] = yaw
        if i + 1 < len(pts):
            seg = np.hypot(pts[i + 1][0] - x, pts[i + 1][1] - y)
            time.sleep(min(0.3, max(0.005, seg / (_cfg['speed_mps'] * 1000.0))))
    with _lock:
        _live['moving'] = False


def _runner_loop(use_robot):
    """Interactive runner — runs the selected trajectory on demand, restarts
    on switch, idles otherwise. Speed / height are read live from _cfg."""
    ur5 = None
    if use_robot:
        import ur5_imu as ur5

    while not _stop_evt.is_set():
        with _lock:
            run = _ctrl['run_req']; sel = _ctrl['sel']
        if not run:
            with _lock:
                _ctrl['running'] = False
            time.sleep(0.05)
            continue

        entry = _LIB[sel]
        try:
            pts, absolute = _build_entry(entry, _cfg['size'])
        except Exception as e:
            print(f"[imu-live] Cannot run {entry.get('label')}: {e}")
            with _lock:
                _ctrl['run_req'] = False
            continue

        _trail_x.clear(); _trail_y.clear()
        _t0[0] = time.time()
        _apply_yaw(0.0)          # every run calibrates yaw from the wrist-3 zero
        with _lock:
            _ctrl['running'] = True; _ctrl['restart'] = False
            _live['wp_total'] = len(pts); _live['done'] = False

        if use_robot:
            span = max((max(abs(p[0]), abs(p[1])) for p in pts), default=0.0)
            try:
                ur5.run_trajectory(
                    pts, height_mm=_cfg['height_mm'],
                    speed_mps=_cfg['speed_mps'], fit=not absolute,
                    clamp_mm=(2.0 * span + 40.0) if absolute
                             else traj.WORKAREA_MAX_MM)
            except Exception as e:
                print(f"[imu-live] Robot run error: {e}")
        else:
            _sim_run(pts)

        with _lock:
            restarting = _ctrl['restart']
            if not restarting:
                _ctrl['run_req'] = False
                _ctrl['running'] = False
                _live['done'] = True


# ─────────────────────────────────────────────────────────────────────────────
# Dashboard
# ─────────────────────────────────────────────────────────────────────────────

def build_dashboard(lim, size_mm, height_mm, use_robot=False):
    matplotlib.rcParams.update({
        'figure.facecolor': BG, 'text.color': 'white',
        'axes.facecolor':   BG, 'axes.edgecolor': EDGE,
    })

    fig = plt.figure(figsize=(13, 10), facecolor=BG)
    gs  = gridspec.GridSpec(
        3, 2, figure=fig,
        height_ratios=[8, 2.5, 0.5], width_ratios=[7, 3],
        hspace=0.22, wspace=0.16,
        left=0.06, right=0.97, top=0.92, bottom=0.05)

    ax_xy   = fig.add_subplot(gs[0, 0])
    ax_read = fig.add_subplot(gs[0, 1])
    ax_hist = fig.add_subplot(gs[1, :])
    ax_prog = fig.add_subplot(gs[2, :])
    for ax in (ax_xy, ax_read, ax_hist, ax_prog):
        ax.set_facecolor(BG)
        for sp in ax.spines.values():
            sp.set_edgecolor(EDGE)

    # ── XY plane ───────────────────────────────────────────────────────────────
    half = traj.WORKAREA_MAX_MM / 2.0
    wa_poly, wa_label = _workarea_polygon()
    ax_xy.plot([p[0] for p in wa_poly], [p[1] for p in wa_poly],
               '--', color='#663333', linewidth=1.2)
    wx = min(p[0] for p in wa_poly); wy = max(p[1] for p in wa_poly)
    ax_xy.text(wx, wy + 4, wa_label, fontsize=7, color='#996666')

    plan_line,  = ax_xy.plot([], [], color=PLAN_C, linewidth=1.0, alpha=0.4,
                             label='planned path', zorder=1)
    start_dot,  = ax_xy.plot([], [], 's', color='#f1c40f', ms=9, alpha=0.9,
                             zorder=2, label='start waypoint')
    center_dot, = ax_xy.plot([0], [0], '+', color='#ffbf00', ms=16,
                             markeredgewidth=2.0, zorder=6, label='central point')
    trail_line, = ax_xy.plot([], [], color=TRAIL_C, linewidth=1.8, alpha=0.9,
                             zorder=3, label='end-effector trail')
    heading_line, = ax_xy.plot([], [], color='#ff5555', linewidth=2.2, zorder=4,
                               label='yaw heading')
    pos_dot,    = ax_xy.plot([], [], 'o', ms=11, color='white',
                             markeredgecolor='#dc0000', markeredgewidth=2.0, zorder=5)

    ax_xy.set_xlim(-lim, lim);  ax_xy.set_ylim(-lim, lim)
    ax_xy.set_aspect('equal')
    ax_xy.set_xlabel('X (mm, rel. reference)', fontsize=9, color='#aaaaaa')
    ax_xy.set_ylabel('Y (mm, rel. reference)', fontsize=9, color='#aaaaaa')
    ax_xy.tick_params(colors='#aaaaaa', labelsize=8)
    ax_xy.grid(color=EDGE, alpha=0.35, linewidth=0.4)
    ax_xy.axhline(0, color=EDGE, linewidth=0.5)
    ax_xy.axvline(0, color=EDGE, linewidth=0.5)
    xy_title = ax_xy.set_title('', fontsize=11, color='white', pad=6)
    ax_xy.legend(fontsize=7, facecolor=BG, labelcolor='white',
                 edgecolor=EDGE, loc='upper right')

    # Degree label that rides the yaw arrow tip.
    heading_txt = ax_xy.text(0, 0, '', color='#ff5555', fontsize=9,
                             fontweight='bold', ha='left', va='center', zorder=7)

    # Compass rose inset (lower-left of the XY panel) — a live yaw needle.
    cax = ax_xy.inset_axes([0.015, 0.015, 0.19, 0.19])
    cax.set_aspect('equal'); cax.axis('off')
    cax.set_xlim(-1.6, 1.6); cax.set_ylim(-1.6, 1.6)
    _th = np.linspace(0, 2 * np.pi, 120)
    cax.plot(np.cos(_th), np.sin(_th), color='#666666', linewidth=1.0)
    for _a, _lbl in [(0, '0°'), (90, '90'), (180, '180'), (270, '270')]:
        _r = np.radians(_a)
        cax.plot([0.82 * np.cos(_r), np.cos(_r)],
                 [0.82 * np.sin(_r), np.sin(_r)], color='#666666', linewidth=0.8)
        cax.text(1.32 * np.cos(_r), 1.32 * np.sin(_r), _lbl,
                 color='#999999', fontsize=6, ha='center', va='center')
    cax.plot([0], [0], 'o', color='#888888', ms=3)
    compass_needle, = cax.plot([0, 1], [0, 0], color='#ff5555', linewidth=2.2,
                               solid_capstyle='round')
    compass_txt = cax.text(0, -1.45, '', color='#ff5555', fontsize=7.5,
                           ha='center', va='center', fontweight='bold')
    cax.set_title('yaw heading', color='#aaaaaa', fontsize=6, pad=2)

    # ── Readout + key help ─────────────────────────────────────────────────────
    ax_read.axis('off')
    read_txt = ax_read.text(
        0.03, 0.98, '', transform=ax_read.transAxes, fontsize=12,
        family='monospace', color='white', va='top', ha='left', linespacing=1.55,
        bbox=dict(facecolor='#1a1a1a', edgecolor=EDGE, pad=8))
    ax_read.text(
        0.03, 0.34,
        ', / .   prev / next traj\n'
        '[ / ]   yaw offset -/+  (shift = big)\n'
        'y       reset yaw offset\n'
        '←→↑↓   move centre  (shift=x5)\n'
        '- / =   centre step -/+\n'
        'n       reset centre\n'
        'space   run / stop\n'
        'c       confirm yaw -> rise + run\n'
        'backspace   stop + home\n'
        'r       rescan JSON folder\n'
        'enter   console: speed / height ...\n'
        'mouse   click/drag = place centre',
        transform=ax_read.transAxes, fontsize=8.5, family='monospace',
        color='#ffbf00', va='top', ha='left', linespacing=1.5,
        bbox=dict(facecolor='#161616', edgecolor='#553311', pad=6))

    # ── X(t)/Y(t) rolling strip ────────────────────────────────────────────────
    xline, = ax_hist.plot(range(HIST_WIN), _x_buf, color='#e74c3c',
                          linewidth=1.1, label='X (mm)')
    yline, = ax_hist.plot(range(HIST_WIN), _y_buf, color='#f1c40f',
                          linewidth=1.1, label='Y (mm)')
    ax_hist.set_xlim(0, HIST_WIN);  ax_hist.set_ylim(-lim, lim)
    ax_hist.set_xticks([])
    ax_hist.set_ylabel('mm', fontsize=8, color='#aaaaaa')
    ax_hist.tick_params(axis='y', colors='#aaaaaa', labelsize=7)
    ax_hist.axhline(0, color=EDGE, linewidth=0.5, linestyle='--')
    ax_hist.grid(axis='y', color=EDGE, alpha=0.35, linewidth=0.4)
    ax_hist.set_title('Position history — X and Y vs time', fontsize=8,
                      color='white', pad=3)
    ax_hist.legend(fontsize=7, facecolor=BG, labelcolor='white',
                   edgecolor=EDGE, loc='upper left', ncol=2)

    # ── Progress bar ───────────────────────────────────────────────────────────
    (prog_rect,) = ax_prog.barh([0], [0], height=0.8, color=ACCENT)
    ax_prog.set_xlim(0, 1);  ax_prog.set_ylim(-0.5, 0.5); ax_prog.axis('off')
    prog_lbl = ax_prog.text(0.5, 0, '', va='center', ha='center', fontsize=9,
                            color='white', transform=ax_prog.transAxes)

    # Console line ("pseudo terminal") along the very bottom of the figure.
    console_txt = fig.text(0.06, 0.008, '', family='monospace', fontsize=9.5,
                           color='#33e666', va='bottom', ha='left')

    fig.suptitle(
        f'Inertial (IMU) Test — Live Controller   '
        f'(size {size_mm:.0f} mm  |  height +{height_mm:.0f} mm  |  no sensors)',
        fontsize=12, fontweight='bold', color='white', y=0.975)

    # ── Event handlers ─────────────────────────────────────────────────────────
    _drag = {'on': False}

    def _on_key(event):
        k = event.key
        if k is None:
            return

        # ── Console ("pseudo terminal") mode captures all typing ──────────────
        if _console['on']:
            if k in ('enter', 'return'):
                _console['msg'] = _run_command(_console['buf'])
                _console['buf'] = ''
            elif k == 'escape':
                _console['on'] = False; _console['buf'] = ''
            elif k == 'backspace':
                _console['buf'] = _console['buf'][:-1]
            elif k == ' ':
                _console['buf'] += ' '
            elif len(k) == 1:
                _console['buf'] += k
            return

        if k in ('enter', 'return'):          # open the console
            _console['on'] = True; _console['msg'] = ''
            return

        step = _ctrl['step']
        cx, cy = _center_offset
        if k == 'left':          _apply_center(cx - step, cy)
        elif k == 'right':       _apply_center(cx + step, cy)
        elif k == 'up':          _apply_center(cx, cy + step)
        elif k == 'down':        _apply_center(cx, cy - step)
        elif k == 'shift+left':  _apply_center(cx - 5 * step, cy)
        elif k == 'shift+right': _apply_center(cx + 5 * step, cy)
        elif k == 'shift+up':    _apply_center(cx, cy + 5 * step)
        elif k == 'shift+down':  _apply_center(cx, cy - 5 * step)
        elif k == 'n':           _apply_center(0.0, 0.0)
        elif k == ',':           _select(-1)
        elif k == '.':           _select(+1)
        elif k == '[':           _apply_yaw(_yaw_offset[0] - YAW_STEP_DEG)
        elif k == ']':           _apply_yaw(_yaw_offset[0] + YAW_STEP_DEG)
        elif k == '{':           _apply_yaw(_yaw_offset[0] - 3 * YAW_STEP_DEG)
        elif k == '}':           _apply_yaw(_yaw_offset[0] + 3 * YAW_STEP_DEG)
        elif k == 'y':           _apply_yaw(0.0)
        elif k == '-':           _ctrl['step'] = max(0.5, step / 2.0)
        elif k in ('=', '+'):    _ctrl['step'] = min(40.0, step * 2.0)
        elif k == ' ':           _toggle_run()
        elif k == 'c':           _confirm_start()
        elif k == 'backspace':   _stop_home()
        elif k == 'r':           _rescan()

    def _on_press(event):
        if event.inaxes is ax_xy and event.xdata is not None:
            _drag['on'] = True
            _apply_center(event.xdata, event.ydata)

    def _on_motion(event):
        if _drag['on'] and event.inaxes is ax_xy and event.xdata is not None:
            _apply_center(event.xdata, event.ydata)

    def _on_release(_event):
        _drag['on'] = False

    fig.canvas.mpl_connect('key_press_event', _on_key)
    fig.canvas.mpl_connect('button_press_event', _on_press)
    fig.canvas.mpl_connect('motion_notify_event', _on_motion)
    fig.canvas.mpl_connect('button_release_event', _on_release)

    # ── Animation update ───────────────────────────────────────────────────────
    def update(_frame):
        with _lock:
            x = _live['x_mm']; y = _live['y_mm']; z = _live['z_mm']
            yaw = _live['yaw']; wp = _live['wp']; wp_total = _live['wp_total']
            moving = _live['moving']; done = _live['done']; elapsed = _live['elapsed']
            awaiting = _live['awaiting_confirm']
            base = list(_plan['base']); plabel = _plan['label']
            absolute = _plan['absolute']; yaw0 = _plan['yaw0']
            sel = _ctrl['sel']; step = _ctrl['step']
            run_req = _ctrl['run_req']
        yaw_off = _yaw_offset[0]

        ox, oy = _center_offset
        if base:
            plan_line.set_data([bx + ox for bx, _ in base],
                               [by + oy for _, by in base])
            start_dot.set_data([base[0][0] + ox], [base[0][1] + oy])
        center_dot.set_data([ox], [oy])

        trail_line.set_data(_trail_x, _trail_y)
        px, py = (_trail_x[-1], _trail_y[-1]) if _trail_x else (x, y)
        pos_dot.set_data([px], [py])

        # Yaw heading arrow. While moving it tracks the live effective yaw at the
        # tool; while calibrating it shows the yaw measured from the wrist-3 zero
        # (starts at 0); while idle it previews the start-point orientation.
        hl = 14.0
        if moving:
            ang, hx, hy = yaw, px, py
        elif awaiting:
            ang = yaw_off                          # calibrated from wrist-3 zero
            hx = (base[0][0] + ox) if base else ox
            hy = (base[0][1] + oy) if base else oy
        else:
            ang = (yaw0 + yaw_off) if yaw0 is not None else (
                yaw_off if abs(yaw_off) > 1e-9 else None)
            hx = (base[0][0] + ox) if base else ox
            hy = (base[0][1] + oy) if base else oy
        if ang is not None:
            ca, sa = np.cos(np.radians(ang)), np.sin(np.radians(ang))
            heading_line.set_data([hx, hx + hl * ca], [hy, hy + hl * sa])
            disp = ang % 360.0
            heading_txt.set_position((hx + hl * ca + 3, hy + hl * sa))
            heading_txt.set_text(f'{disp:.0f}°')
            compass_needle.set_data([0, ca], [0, sa])
            compass_txt.set_text(f'{disp:.0f}°')
        else:
            heading_line.set_data([], [])
            heading_txt.set_text('')
            compass_needle.set_data([0, 0], [0, 0])
            compass_txt.set_text('--')

        xline.set_ydata(_x_buf);  yline.set_ydata(_y_buf)

        state = (f'⏸ CALIBRATE YAW @ {CALIB_HEIGHT_MM:+.0f}mm — [ ], c=go up'
                 if awaiting else
                 ('▶ RUNNING' if moving else
                  ('… starting' if run_req else ('✓ done' if done else '· idle'))))
        frac = (wp / wp_total) if wp_total else 0.0
        xy_title.set_text(f'[{sel + 1}/{len(_LIB)}]  {plabel}'
                          f'{"   (UR frame)" if absolute else ""}')
        xy_title.set_color(TRAIL_C if moving else 'white')

        read_txt.set_text(
            f'{state}\n\n'
            f'traj [{sel + 1}/{len(_LIB)}]\n'
            f'speed  = {_cfg["speed_mps"] * 1000:>5.0f} mm/s\n'
            f'height = {_cfg["height_mm"]:>5.0f} mm\n'
            f'yaw off = {yaw_off:+6.1f} deg\n'
            f'step    = {step:>4.1f} mm\n\n'
            f'X = {x:+8.1f} mm\n'
            f'Y = {y:+8.1f} mm\n'
            f'Z = {z:+8.1f} mm\n'
            f'yaw = {("%+7.1f" % yaw) if yaw is not None else "   --":>8} deg\n\n'
            f'centre = ({ox:+.1f}, {oy:+.1f})\n'
            f'wp {wp:>4d}/{wp_total}   t={elapsed:5.1f}s')
        read_txt.set_color('#ffbf00' if awaiting else
                           (TRAIL_C if moving else ('#888888' if not done else ACCENT)))

        if _console['on']:
            console_txt.set_text(f'cmd> {_console["buf"]}█    {_console["msg"]}')
            console_txt.set_color('#33e666')
        else:
            hint = _console['msg'] or ('Enter = console  (e.g.  speed 40   '
                                       'height 25)')
            console_txt.set_text(hint)
            console_txt.set_color('#888888')

        prog_rect.set_width(frac)
        prog_rect.set_color(TRAIL_C if moving else ACCENT)
        prog_lbl.set_text(f'{wp} / {wp_total} waypoints  ({frac * 100:.0f} %)')

    anim = FuncAnimation(fig, update, interval=50, blit=False,
                         cache_frame_data=False)
    return fig, anim


# ─────────────────────────────────────────────────────────────────────────────
# CLI / Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Inertial (IMU) test — live trajectory controller',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('--traj',   default=None, metavar='NAME',
                   help='preselect a generated shape')
    p.add_argument('--json',   default=None, metavar='FILE',
                   help='preselect / add a JSON trajectory')
    p.add_argument('--lib-dir', default=os.path.join(_HERE, 'trajectories_ur'),
                   help='JSON library folder (default: trajectories_ur/)')
    p.add_argument('--size',   type=float, default=traj.DEFAULT_SPAN_MM,
                   help='size for generated shapes in mm (max 230)')
    p.add_argument('--speed',  type=float, default=30.0,
                   help='trajectory speed in mm/s (default 30)')
    p.add_argument('--height', type=float, default=None,
                   help='work-plane height above reference in mm (default 30)')
    p.add_argument('--no-robot', action='store_true',
                   help='simulate motion (display only, no robot)')
    p.add_argument('--no-confirm', action='store_true',
                   help='skip the start-orientation confirmation pause')
    p.add_argument('--list', action='store_true',
                   help='list the library and exit')
    return p.parse_args()


def main():
    global _LIB
    args = parse_args()

    _LIB = _build_library(args.lib_dir, extra_json=args.json)

    if args.list:
        print(f'Library ({len(_LIB)} trajectories):')
        for i, e in enumerate(_LIB):
            print(f'  {i + 1:>2d}. {e["label"]}')
        return
    if not _LIB:
        print('[imu-live] Empty library — check --lib-dir or add generated shapes.')
        sys.exit(1)

    use_robot = not args.no_robot
    import ur5_imu as ur5
    height = args.height if args.height is not None else ur5.DEFAULT_HEIGHT_MM
    confirm = not args.no_confirm
    _cfg.update(use_robot=use_robot, size=args.size, lib_dir=args.lib_dir,
                speed_mps=args.speed / 1000.0, height_mm=height, confirm=confirm)
    ur5.set_speed(args.speed / 1000.0)
    ur5.set_height_mm(height)
    ur5.set_confirm_enabled(confirm)

    # Initial selection
    sel = 0
    if args.json:
        ap = os.path.abspath(args.json)
        sel = next((i for i, e in enumerate(_LIB)
                    if e.get('path') and os.path.abspath(e['path']) == ap), 0)
    elif args.traj:
        sel = next((i for i, e in enumerate(_LIB)
                    if e['kind'] == 'gen' and e['name'] == args.traj), 0)
    _ctrl['sel'] = sel
    _refresh_plan()

    lim = _lib_extent(args.size)

    print('=' * 62)
    print('  Inertial (IMU) Test — Live Controller')
    print('=' * 62)
    print(f'  Library    : {len(_LIB)} trajectories  (JSON dir: {args.lib_dir})')
    print(f'  Selected   : {_ctrl["label"]}')
    print(f'  Speed      : {args.speed:.0f} mm/s   Height: +{height:.0f} mm')
    print(f'  Yaw calib  : {"ON — wrist-3 zero @ %+.0f mm, press c to rise + run" % CALIB_HEIGHT_MM if confirm else "OFF (--no-confirm)"}')
    print(f'  Robot      : {"ON — " + os.environ.get("UR_ROBOT_IP", ur5.ROBOT_IP) if use_robot else "OFF (--no-robot, simulated)"}')
    print('  Keys       : , . prev/next   [ ] yaw-offset   arrows centre   '
          'space run/stop   backspace home   r rescan')
    print('  Console    : Enter opens it — e.g.  "speed 40"  "height 25"  '
          '"stop"')
    print('=' * 62)

    threading.Thread(target=_runner_loop, args=(use_robot,),
                     daemon=True).start()
    threading.Thread(target=_sampler_loop, args=(use_robot,),
                     daemon=True).start()

    fig, anim = build_dashboard(lim, args.size, height, use_robot)
    print('\n[imu-live] Dashboard open — SPACE to run, close window to quit\n')
    try:
        plt.show()
    except KeyboardInterrupt:
        print('\n[imu-live] Stopped by user')
    finally:
        _stop_evt.set()
        if use_robot:
            ur5.request_stop()
        print('[imu-live] Done')


if __name__ == '__main__':
    main()
