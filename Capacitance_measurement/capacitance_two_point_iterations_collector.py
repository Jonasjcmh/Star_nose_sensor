"""
capacitance_two_point_iterations_collector.py — Star-Nose Sensor | Multi-Point Depth-Sweep Collector
======================================================================================================
A variant of capacitance_ramp_collector.py for a small, focused depth sweep on
N user-chosen points (instead of all 19), with the fixed-ramp
trapezoidal press and tail-averaging of single_point_ramp_test.py.

This script ONLY collects and saves data. Plotting/analysis is done separately.

What it does
------------
  • You pick however many points (pad numbers 1–19) you want at the start,
    or choose "all" to sweep every pad 1–19 in a random order.
  • For each point in turn: for each depth in DEPTHS_MM (default 0,1,2,3,4 mm),
    run ITERATIONS (default 5) repeat indentations at that depth — so all
    depths × iterations for that point happen back-to-back, in order.
  • Then the script pauses so you can re-wire the LCR-6100 probes to the
    NEXT point, and repeats the same depth × iteration sweep there.
  • Each indentation: locate → press(ramp_s) → hold → retract(ramp_s) → post,
    with press/retract speed & accel derived so each move takes ~ramp_s
    seconds regardless of depth (trapezoidal profile).
  • All raw samples go to ONE combined CSV.

Fixed-ramp motion (per move of distance d, in time ramp_s, accel fraction f):
        cruise v = d / ((1−f)·ramp_s)     accel a = d / (f·(1−f)·ramp_s²)
  v and a scale with d, so the move always takes ramp_s for any depth
  (depth = 0 mm is a no-op move — the press/retract phases just log at the
  surface, which is useful as a same-cycle baseline).

Usage
-----
  python capacitance_two_point_iterations_collector.py
  python capacitance_two_point_iterations_collector.py --points 5,12,9 \
         --depths 0,1,2,3,4 --iterations 5 --ramp 2 --hold 5 --locate 5 --post 5
  python capacitance_two_point_iterations_collector.py --points all   # all 19, random order
"""

import os
import sys
import csv
import time
import random
import argparse
import threading
from datetime import datetime

import rtde_control
import rtde_receive
from lcr6100 import LCR6100, list_ports

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
LOG_DIR      = os.path.join(_HERE, 'logs')

# Point numbering/geometry and hex-grid labels (a1..e5) are the single source
# of truth in Integration_2/ur5_control.py — imported directly so this
# collector can't drift from the current physical point layout.
sys.path.insert(0, _INTEGRATION)
import ur5_control   # noqa: E402  (Integration_2/ur5_control.py)

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_IP   = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
VEL_TRAVEL = 0.05    # m/s — travel between points
VEL_PRESS  = 0.004   # m/s — fallback press speed (press/retract use a ramp time)
ACCEL      = 0.3     # m/s²
SAFE_HOME_Z = 30.0   # mm above surface at home

# Base press depth (mm) added to every requested depth. Must be 0 with the
# current calibration: the chosen calib's global Z (gz, e.g. -10.8 mm) is
# measured at TRUE surface contact — extra_z=0 already sits on the surface, so
# depth label D presses exactly D mm (matching capacitance_ramp_collector.py and
# Interdome_touch/main.py, where SURFACE_STANDOFF_MM=0). A non-zero base would
# press that many mm DEEPER than every other script. Only raise it if a tip is
# genuinely calibrated with a standoff gap above the surface.
BASE_DEPTH_MM = 0.0

# ── FUTEK load cell ────────────────────────────────────────────────────────────
AI0_ZERO_V       = 5.0
LOADCELL_MAX_N   = 10.0 * 4.44822
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0

def _ai0_to_n(v):
    return -(float(v) - AI0_ZERO_V) * LOADCELL_N_PER_V

# ── Sensor points (mm, relative to reference pose) ───────────────────────────
# Hex-grid labelled layout (P1=a1, P2=a2, ... P10=c3 center, ... P19=e5) —
# same POINTS/labels/pose as Integration_2/ur5_control.py, so a point pressed
# here is the same physical pad the muca-board pipeline and visualizer call by
# that label.
POINTS         = ur5_control.POINTS
POINT_TO_LABEL = ur5_control.POINT_TO_LABEL   # pt -> 'a1'..'e5'

REFERENCE_POSE = ur5_control.REFERENCE_POSE

# Point ids laid out as the physical hex rows in the SENSOR/DISPLAY frame
# (constant display-y lines, top -> bottom) under the NEW a1..e5 numbering —
# NOT sequential P#. Kept in sync with Interdome_touch/main.py SENSOR_MAP_ROWS.
# The operator hand-wires the LCR probe to the highlighted pad, so this MUST
# reflect the true physical layout (P1=a1 is the middle-row left, not top-left).
#   disp y=+14 : a3 b4 c5           -> P3  P7  P12
#   disp y=+7  : a2 b3 c4 d5        -> P2  P6  P11 P16
#   disp y=0   : a1 b2 c3 d4 e5     -> P1  P5  P10 P15 P19
#   disp y=-7  : b1 c2 d3 e4        -> P4  P9  P14 P18
#   disp y=-14 : c1 d2 e3           -> P8  P13 P17
SENSOR_MAP_ROWS = [
    [3, 7, 12],
    [2, 6, 11, 16],
    [1, 5, 10, 15, 19],
    [4, 9, 14, 18],
    [8, 13, 17],
]

# ── Calibration globals ────────────────────────────────────────────────────────
CALIB_X_MM    = 0.0
CALIB_Y_MM    = 0.0
CALIB_Z_MM    = 0.0
POINT_OFFSETS = {}     # pt → (dx_mm, dy_mm)

# ── Shared robot state (background thread) ────────────────────────────────────
_state      = {'ft': [0.0]*6, 'tcp': [0.0]*6, 'ai0': AI0_ZERO_V}
_state_lock = threading.Lock()
_ft_stop    = threading.Event()

def _ft_reader(rtde_r):
    while not _ft_stop.is_set():
        try:
            ft  = rtde_r.getActualTCPForce()
            tcp = rtde_r.getActualTCPPose()
            ai0 = rtde_r.getStandardAnalogInput0()
            with _state_lock:
                _state['ft']  = list(ft)
                _state['tcp'] = list(tcp)
                _state['ai0'] = float(ai0)
        except Exception:
            pass
        time.sleep(0.004)   # ~250 Hz

def get_robot_state():
    with _state_lock:
        return {k: list(v) if isinstance(v, list) else v
                for k, v in _state.items()}

# ── Fixed-ramp motion (trapezoidal profile) ────────────────────────────────────

# Fraction of the ramp time spent accelerating (and the same decelerating); the
# remaining middle is a constant-velocity cruise. 0.25 → 25/50/25. Use 0.5 for a
# pure triangular profile (no cruise).
RAMP_ACCEL_FRAC = 0.25

def vel_accel_for_ramp(depth_mm, ramp_s, accel_frac=RAMP_ACCEL_FRAC):
    """Speed & accel so a moveL over depth_mm completes in ~ramp_s as a
    trapezoid: v = d/((1−f)·ramp_s), a = d/(f·(1−f)·ramp_s²). v & a scale with
    depth, so the move takes ramp_s for any depth. Returns (vel_m_s, accel_m_s2)."""
    d = abs(depth_mm) / 1000.0
    f = min(max(accel_frac, 1e-3), 0.5)
    if d <= 0.0 or ramp_s <= 0.0:
        return VEL_PRESS, ACCEL
    vel   = d / ((1.0 - f) * ramp_s)
    accel = d / (f * (1.0 - f) * ramp_s * ramp_s)
    return vel, accel

# ── Dataset log ───────────────────────────────────────────────────────────────
_log_rows = []
_log_lock = threading.Lock()

FIELDNAMES = [
    'timestamp', 'datetime',
    'point_idx', 'point', 'label', 'depth_mm', 'iter_idx', 'phase',
    'tcp_x', 'tcp_y', 'tcp_z',
    'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
    'ai0', 'load_cell_N',
    'Cp_F', 'Cp_pF', 'Rp_Ohm', 'lcr_ok',
]

def _log_row(pt, depth_mm, phase, point_idx, iter_idx, lcr):
    st  = get_robot_state()
    ft  = st['ft']
    tcp = st['tcp']
    ai0 = st['ai0']
    Cp, Rp, lcr_ok = lcr.get_latest()
    row = {
        'timestamp':   round(time.time(), 4),
        'datetime':    datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'point_idx':   point_idx,
        'point':       pt,
        'label':       POINT_TO_LABEL.get(pt, str(pt)),
        'depth_mm':    depth_mm,
        'iter_idx':    iter_idx,
        'phase':       phase,
        'tcp_x':       round(tcp[0], 6),
        'tcp_y':       round(tcp[1], 6),
        'tcp_z':       round(tcp[2], 6),
        'fx':          round(ft[0], 4),
        'fy':          round(ft[1], 4),
        'fz':          round(ft[2], 4),
        'tx':          round(ft[3], 4),
        'ty':          round(ft[4], 4),
        'tz':          round(ft[5], 4),
        'ai0':         round(ai0, 5),
        'load_cell_N': round(_ai0_to_n(ai0), 4),
        'Cp_F':        Cp,
        'Cp_pF':       Cp * 1e12 if Cp == Cp else float('nan'),
        'Rp_Ohm':      Rp,
        'lcr_ok':      int(lcr_ok),
    }
    with _log_lock:
        _log_rows.append(row)

def _log_timed(pt, depth_mm, phase, point_idx, iter_idx, lcr, rate_hz, duration_s):
    interval = 1.0 / rate_hz
    t_end    = time.time() + duration_s
    while time.time() < t_end:
        t0 = time.perf_counter()
        _log_row(pt, depth_mm, phase, point_idx, iter_idx, lcr)
        rem = interval - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)

# Averaging window (s) for the settled per-phase readings reported at each dwell.
SETTLE_WINDOW_S = 1.0

def _phase_tail_mean(phase, window_s=SETTLE_WINDOW_S, rate_hz=100):
    """Average Cp (pF) and load-cell force (N) over the LAST window_s of the most
    recent run of `phase` (the current indentation, since indentations are done
    in series). Settled mean instead of one sample → ~√N less noise. Returns
    (cp_pF_mean, load_N_mean, n_used)."""
    n = max(1, int(window_s * rate_hz))
    with _log_lock:
        rows = [r for r in _log_rows if r['phase'] == phase][-n:]
    if not rows:
        return float('nan'), float('nan'), 0
    cps = [r['Cp_pF'] for r in rows if r['Cp_pF'] == r['Cp_pF']]   # drop NaN
    lds = [r['load_cell_N'] for r in rows]
    cp_mean = sum(cps) / len(cps) if cps else float('nan')
    ld_mean = sum(lds) / len(lds) if lds else float('nan')
    return cp_mean, ld_mean, len(rows)

def _move_logged(rtde_c, target, vel, accel, pt, depth_mm, phase,
                 point_idx, iter_idx, lcr, rate_hz):
    """Run a blocking moveL while logging `phase` continuously from a helper
    thread, so the press/retract ramp spans real time (LCR + force captured the
    whole move). Returns the actual ramp seconds."""
    stop = threading.Event()
    interval = 1.0 / rate_hz

    def _logger():
        while not stop.is_set():
            t0 = time.perf_counter()
            _log_row(pt, depth_mm, phase, point_idx, iter_idx, lcr)
            rem = interval - (time.perf_counter() - t0)
            if rem > 0:
                time.sleep(rem)

    th = threading.Thread(target=_logger, daemon=True)
    th.start()
    t0 = time.perf_counter()
    try:
        rtde_c.moveL(target, vel, accel)   # blocking
    finally:
        stop.set()
        th.join()
    return time.perf_counter() - t0

def save_dataset(points):
    with _log_lock:
        rows = list(_log_rows)
    if not rows:
        print('[log] Nothing to save.')
        return None
    os.makedirs(LOG_DIR, exist_ok=True)
    ts     = datetime.now().strftime('%Y%m%d_%H%M%S')
    # Keep the filename short when many points are swept (e.g. 'all'); the exact
    # per-point order is preserved in the CSV via point_idx/point anyway.
    p_tag  = (f'ALL{len(points)}' if len(points) >= len(POINTS)
              else '_'.join(f'P{p:02d}' for p in points))
    path   = os.path.join(LOG_DIR, f'two_point_iterations_{p_tag}_{ts}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)
    print(f'[log] Saved {len(rows)} rows → {path}')
    return path

# ── Calibration ───────────────────────────────────────────────────────────────

def list_calib_files():
    import glob, json
    pattern = os.path.join(_INTEGRATION, 'calib_*.json')
    files   = sorted(glob.glob(pattern))
    results = []
    for path in files:
        base = os.path.basename(path)
        if base.startswith('calib_points_'):
            continue
        tip = base[len('calib_'):-len('.json')]
        pts = os.path.join(_INTEGRATION, f'calib_points_{tip}.json')
        results.append((tip, path, pts if os.path.exists(pts) else None))
    plain = os.path.join(_INTEGRATION, 'calib.json')
    if os.path.exists(plain):
        pts = os.path.join(_INTEGRATION, 'calib_points.json')
        results.insert(0, ('(default)', plain, pts if os.path.exists(pts) else None))
    return results

def select_calibration():
    import json
    global CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM, POINT_OFFSETS

    files = list_calib_files()
    if not files:
        print(f'[calib] No calibration files found in {_INTEGRATION}')
        print('[calib] Using zero offsets — robot may not hit the point correctly!')
        return

    print()
    print('  Available calibration profiles:')
    for i, (tip, gpath, ppath) in enumerate(files):
        with open(gpath) as f:
            d = json.load(f)
        pts_info = f'  + per-point ({os.path.basename(ppath)})' if ppath else ''
        print(f'    [{i}]  {tip:20s}  '
              f'X={d.get("x_mm",0):+.3f}  Y={d.get("y_mm",0):+.3f}  '
              f'Z={d.get("z_mm",0):+.3f} mm{pts_info}')

    while True:
        try:
            raw = input(f'\n  Select calibration [0–{len(files)-1}] > ').strip()
            idx = int(raw)
            if 0 <= idx < len(files):
                break
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
        print(f'  Please enter a number between 0 and {len(files)-1}')

    tip, gpath, ppath = files[idx]
    with open(gpath) as f:
        d = json.load(f)
    CALIB_X_MM = d.get('x_mm', 0.0)
    CALIB_Y_MM = d.get('y_mm', 0.0)
    CALIB_Z_MM = d.get('z_mm', 0.0)

    if ppath:
        with open(ppath) as f:
            pd = json.load(f)
        POINT_OFFSETS = {int(k): (v.get('dx_mm', 0.0), v.get('dy_mm', 0.0))
                         for k, v in pd.get('per_point', {}).items()}
        # Single source of truth: the calib_points file's own `global` block is
        # authoritative for X/Y/Z — Interdome_touch/main.py reads the global from
        # there too. Prefer it so this collector can't drift from Interdome when a
        # re-calibration updates only the calib_points file. Warn if the
        # standalone calib_<tip>.json disagrees.
        g = pd.get('global') or {}
        if g:
            gx = g.get('x_mm', 0.0); gy = g.get('y_mm', 0.0); gz = g.get('z_mm', 0.0)
            if (abs(gx - CALIB_X_MM) > 1e-6 or abs(gy - CALIB_Y_MM) > 1e-6
                    or abs(gz - CALIB_Z_MM) > 1e-6):
                print(f'  [calib] NOTE: {os.path.basename(gpath)} global '
                      f'(X={CALIB_X_MM:+.3f} Y={CALIB_Y_MM:+.3f} Z={CALIB_Z_MM:+.3f}) '
                      f'differs from {os.path.basename(ppath)} — using the '
                      f'calib_points global (matches Interdome).')
            CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM = gx, gy, gz
        print(f'  [calib] Per-point offsets loaded for {len(POINT_OFFSETS)} points')
    else:
        POINT_OFFSETS = {}
        print('  [calib] No per-point file — global offset only')

    print(f'\n  [calib] Profile "{tip}": '
          f'X={CALIB_X_MM:+.3f}  Y={CALIB_Y_MM:+.3f}  Z={CALIB_Z_MM:+.3f} mm')

    try:
        ans = input('\n  Correct tip mounted? Confirm calibration? [y/N] > ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        raise SystemExit(1)
    if ans != 'y':
        print('[calib] Aborted — re-run to select a different calibration.')
        raise SystemExit(1)

# ── Pose builders ─────────────────────────────────────────────────────────────

def _build_pose(pt, extra_z_mm=0.0):
    dx, dy   = POINTS[pt]
    pdx, pdy = POINT_OFFSETS.get(pt, (0.0, 0.0))
    pose     = list(REFERENCE_POSE)
    pose[0] += (dx + CALIB_X_MM + pdx) / 1000.0
    pose[1] += (dy + CALIB_Y_MM + pdy) / 1000.0
    pose[2] += (extra_z_mm + CALIB_Z_MM) / 1000.0
    return pose

def _home_pose():
    return _build_pose(10, SAFE_HOME_Z)

# ── Plan ───────────────────────────────────────────────────────────────────────

def generate_plan(points, depths_mm, iterations):
    """For each point (in order): for each depth (in order): `iterations`
    repeat indentations. Returns list of (point_idx, pt, depth_mm, iter_idx)."""
    plan = []
    for point_idx, pt in enumerate(points):
        for depth_mm in depths_mm:
            for iter_idx in range(iterations):
                plan.append((point_idx, pt, depth_mm, iter_idx))
    return plan

# ── Indentation ───────────────────────────────────────────────────────────────

def do_indentation(rtde_c, pt, depth_mm, point_idx, iter_idx, lcr,
                   locate_s, hold_s, post_s, ramp_s, rate_hz=100,
                   base_depth_mm=BASE_DEPTH_MM):
    """One step-impulse indentation logged at rate_hz:
      locate → press(ramp_s) → hold → retract(ramp_s) → post.
    Press/retract speed & accel are derived so each takes ~ramp_s for any depth.
    The physical press is depth_mm + base_depth_mm (the surface pose sits above
    real contact); depth_mm stays the recorded label.
    """
    press_depth_mm = depth_mm + base_depth_mm
    surface = _build_pose(pt, 0.0)
    pressed = _build_pose(pt, -press_depth_mm)
    v_press, a_press = vel_accel_for_ramp(press_depth_mm, ramp_s)

    # ── Locate ───────────────────────────────────────────────────────────────
    print(f'     → locate  (moving to surface P{pt:02d}) ...')
    rtde_c.moveL(surface, VEL_TRAVEL, ACCEL)
    _log_row(pt, depth_mm, 'locate', point_idx, iter_idx, lcr)
    _log_timed(pt, depth_mm, 'locate', point_idx, iter_idx, lcr, rate_hz, locate_s)
    cp_m, lc_m, n = _phase_tail_mean('locate', SETTLE_WINDOW_S, rate_hz)
    print(f'     [locate]   LC={lc_m:.2f} N   Cp={cp_m:.2f} pF   (mean of last {n})')

    # ── Press (ramp_s) ─────────────────────────────────────────────────────────
    print(f'     → press   (depth {depth_mm:.2f} mm → {press_depth_mm:.2f} mm physical, '
          f'target ramp {ramp_s:.1f}s) ...')
    t_press = _move_logged(rtde_c, pressed, v_press, a_press,
                           pt, depth_mm, 'press', point_idx, iter_idx, lcr, rate_hz)
    print(f'     [press]    target ramp = {ramp_s:.1f}s   actual = {t_press:.2f}s')

    # ── Hold ──────────────────────────────────────────────────────────────────
    _log_timed(pt, depth_mm, 'hold', point_idx, iter_idx, lcr, rate_hz, hold_s)
    cp_m, lc_m, n = _phase_tail_mean('hold', SETTLE_WINDOW_S, rate_hz)
    print(f'     [hold]     LC={lc_m:.2f} N   Cp={cp_m:.2f} pF   (mean of last {n})')

    # ── Retract (ramp_s) ───────────────────────────────────────────────────────
    print(f'     → retract  (back to surface, target ramp {ramp_s:.1f}s) ...')
    t_ret = _move_logged(rtde_c, surface, v_press, a_press,
                         pt, depth_mm, 'retract', point_idx, iter_idx, lcr, rate_hz)
    print(f'     [retract]  target ramp = {ramp_s:.1f}s   actual = {t_ret:.2f}s')

    # ── Post ─────────────────────────────────────────────────────────────────
    _log_timed(pt, depth_mm, 'post', point_idx, iter_idx, lcr, rate_hz, post_s)
    cp_m, lc_m, n = _phase_tail_mean('post', SETTLE_WINDOW_S, rate_hz)
    print(f'     [post]     LC={lc_m:.2f} N   Cp={cp_m:.2f} pF   (mean of last {n})')

    with _log_lock:
        n_rows = len(_log_rows)
    print(f'     Done — dataset rows so far: {n_rows}')

# ── Display / input helpers ────────────────────────────────────────────────────

def print_sensor_map(highlight=None):
    print()
    for row in SENSOR_MAP_ROWS:
        indent = ' ' * (4 * (5 - len(row)))
        cells = []
        for pt in row:
            label = POINT_TO_LABEL.get(pt, '??')
            cell  = f'P{pt:02d}/{label:<2}'          # e.g. 'P01/a1'
            cells.append(f'[{cell}]' if pt == highlight else f' {cell} ')
        print('  ' + indent + ' '.join(cells))
    print()

def _select_lcr_port():
    ports = list_ports()
    if not ports:
        print('\n  [LCR] No serial ports found — is the USB cable connected?')
        return input('  Enter port manually (e.g. /dev/ttyUSB0): ').strip()
    print('\n  Available serial ports:')
    for i, (dev, desc) in enumerate(ports):
        print(f'    [{i}]  {dev}  —  {desc}')
    while True:
        try:
            raw = input(f'  Select LCR port [0–{len(ports)-1}] or type path > ').strip()
            if raw.startswith('/') or raw.upper().startswith('COM'):
                return raw
            idx = int(raw)
            if 0 <= idx < len(ports):
                return ports[idx][0]
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
        print('  Invalid selection — try again')

def _ask_int(prompt, default, minimum, maximum):
    while True:
        try:
            raw = input(prompt).strip()
            if raw == '' and default is not None:
                return default
            val = int(raw)
            if minimum <= val <= maximum:
                return val
            print(f'  Must be between {minimum} and {maximum}')
        except (ValueError, EOFError, KeyboardInterrupt):
            if default is not None:
                return default
            print('  Please enter a number')

def _ask_float(prompt, default, minimum, maximum):
    while True:
        try:
            raw = input(prompt).strip()
            if raw == '' and default is not None:
                return default
            val = float(raw)
            if minimum <= val <= maximum:
                return val
            print(f'  Must be between {minimum:.2f} and {maximum:.2f}')
        except (ValueError, EOFError, KeyboardInterrupt):
            if default is not None:
                return default
            print('  Please enter a number')

def _ask_point(prompt):
    while True:
        try:
            raw = input(prompt).strip()
            val = int(raw)
            if val in POINTS:
                return val
            print('  Must be a valid pad number, 1–19')
        except (ValueError, EOFError, KeyboardInterrupt):
            print('  Please enter a number, 1–19')

def _parse_depths(raw):
    return [float(x) for x in raw.split(',') if x.strip() != '']

def _parse_points(raw):
    pts = []
    for x in raw.split(','):
        x = x.strip()
        if x == '':
            continue
        val = int(x)
        if val not in POINTS:
            raise ValueError(f'{val} is not a valid pad number, 1–19')
        pts.append(val)
    return pts

def _all_points_random():
    """Every pad 1–19 in a fresh random order (re-wiring happens per point)."""
    pts = list(POINTS)
    random.shuffle(pts)
    return pts

def _ask_points():
    while True:
        try:
            raw = input('  Test [S]eparate chosen points or [A]ll points '
                        '(random order)? [S] > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            raw = ''
        if raw in ('', 's', 'sep', 'separate'):
            break
        if raw in ('a', 'all'):
            pts = _all_points_random()
            print(f'  All {len(pts)} points, random order: '
                  + ', '.join(f'P{p:02d}/{POINT_TO_LABEL.get(p, "?")}' for p in pts))
            return pts
        print("  Please enter 'S' for separate points or 'A' for all")

    n = _ask_int(f'  How many points? [2] > ', 2, 1, len(POINTS))
    points = []
    for i in range(n):
        while True:
            pt = _ask_point(f'  Point {i + 1}/{n} [1–19] > ')
            if pt in points:
                print(f'  P{pt:02d} already selected — pick a different point')
                continue
            break
        points.append(pt)
    return points

# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Multi-point depth-sweep capacitance collector (data only)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--points',     type=str,   default=None,
                   help="Comma-separated pad numbers 1–19 (e.g. 5,12,9), or "
                        "'all' for every pad in random order [ask]")
    p.add_argument('--depths',     type=str,   default='0,1,2,3,4',
                   help='Comma-separated indentation depths in mm [0,1,2,3,4]')
    p.add_argument('--iterations', type=int,   default=5, help='Repeats per depth [5]')
    p.add_argument('--base-depth', type=float, default=BASE_DEPTH_MM,
                   help=f'Base mm added to every depth; keep 0 (calib gz is at true '
                        f'contact) unless the tip has a standoff gap [{BASE_DEPTH_MM}]')
    p.add_argument('--ramp',       type=float, default=None, help='Ramp time s per press/retract [ask, default 2]')
    p.add_argument('--hold',       type=float, default=None, help='Hold dwell s at full depth [ask, default 5]')
    p.add_argument('--locate',     type=float, default=None, help='Locate dwell s at surface [ask, default 5]')
    p.add_argument('--post',       type=float, default=None, help='Post dwell s after release [ask, default 5]')
    p.add_argument('--rate',       type=int,   default=100,  help='Logging rate Hz [100]')
    p.add_argument('--port',       default=None, help='LCR serial port [interactive]')
    return p.parse_args()

def main():
    args = parse_args()

    print('=' * 65)
    print('  Multi-Point Depth-Sweep Capacitance Collector — Star-Nose Sensor')
    print('=' * 65)

    print_sensor_map()
    if args.points:
        if args.points.strip().lower() == 'all':
            points = _all_points_random()
            print('  All points, random order: '
                  + ', '.join(f'P{p:02d}/{POINT_TO_LABEL.get(p, "?")}' for p in points))
        else:
            points = _parse_points(args.points)
    else:
        points = _ask_points()

    depths_mm  = _parse_depths(args.depths)
    iterations = args.iterations

    ramp_s = args.ramp if args.ramp is not None else float(_ask_int(
        '  Ramp time per press & retract (s) [2] > ', 2, 1, 30))
    hold_s = args.hold if args.hold is not None else _ask_float(
        '  Hold dwell (s, at full depth) [5.0] > ', 5.0, 0.1, 120.0)
    locate_s = args.locate if args.locate is not None else _ask_float(
        '  Locate dwell (s, at surface before press) [5.0] > ', 5.0, 0.1, 120.0)
    post_s = args.post if args.post is not None else _ask_float(
        '  Post dwell (s, at surface after release) [5.0] > ', 5.0, 0.1, 120.0)

    rate_hz = args.rate

    # Hold must outlast the ramp so the at-depth reading is taken settled.
    min_hold = ramp_s * 1.2
    if hold_s < min_hold:
        print(f'  [warn] Hold ({hold_s:.2f}s) does not exceed the ramp '
              f'({ramp_s:.2f}s) — raising hold to {min_hold:.2f}s.')
        hold_s = min_hold

    per_cycle_s   = locate_s + ramp_s + hold_s + ramp_s + post_s
    per_point_n   = len(depths_mm) * iterations
    total         = per_point_n * len(points)

    points_str = ', then '.join(f'P{p:02d}' for p in points)
    print(f'\n  Points            : {points_str}')
    print(f'  Depths (mm)       : {depths_mm}  (+{args.base_depth:.1f} mm base = '
          f'{[d + args.base_depth for d in depths_mm]} physical)')
    print(f'  Iterations/depth  : {iterations}')
    print(f'  Indentations      : {per_point_n}/point  ×  {len(points)} points  =  {total}')
    print(f'  Ramp (press/retr) : {ramp_s:.1f} s each')
    print(f'  Hold dwell        : {hold_s:.2f} s')
    print(f'  Locate dwell      : {locate_s:.2f} s')
    print(f'  Post dwell        : {post_s:.2f} s')
    print(f'  Log rate          : {rate_hz} Hz')
    print(f'  ~Cycle time       : {per_cycle_s:.1f} s/indentation (excl. wiring & travel)')

    # ── Calibration ───────────────────────────────────────────────────────────
    select_calibration()

    # ── LCR port ──────────────────────────────────────────────────────────────
    lcr_port = args.port or _select_lcr_port()

    # ── Plan ──────────────────────────────────────────────────────────────────
    plan = generate_plan(points, depths_mm, iterations)

    # ── LCR ───────────────────────────────────────────────────────────────────
    print('\n[LCR] Connecting ...')
    lcr = LCR6100(lcr_port)
    try:
        lcr.connect()
    except Exception as e:
        print(f'[LCR] Connection failed: {e}'); sys.exit(1)
    lcr.start_polling()
    time.sleep(0.5)
    Cp0, Rp0, ok0 = lcr.get_latest()
    print(f'[LCR] Initial reading: Cp={Cp0*1e12:.2f} pF  Rp={Rp0/1e3:.1f} kΩ  ok={ok0}')

    # ── Robot ─────────────────────────────────────────────────────────────────
    print(f'\n[robot] Connecting to {ROBOT_IP} ...')
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        rtde_c = rtde_control.RTDEControlInterface(
            ROBOT_IP, frequency=500.0,
            flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
    except Exception as e:
        print(f'[robot] Connection failed: {e}')
        lcr.disconnect(); sys.exit(1)
    print('[robot] Connected')

    ft_thread = threading.Thread(target=_ft_reader, args=(rtde_r,), daemon=True)
    ft_thread.start()
    time.sleep(0.3)

    print('[robot] Moving to home ...')
    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
    print('[robot] At home\n')

    # ── Main collection loop ───────────────────────────────────────────────────
    completed       = 0
    start_time      = time.time()
    current_point   = None
    try:
        for step, (point_idx, pt, depth_mm, iter_idx) in enumerate(plan):
            # Pause to re-wire only when the point changes.
            if pt != current_point:
                current_point = pt
                px, py = POINTS[pt]
                lbl = POINT_TO_LABEL.get(pt, '??')
                print('═' * 65)
                print(f'  Point {point_idx + 1}/{len(points)}  |  P{pt:02d} ({lbl})  ({px:+.0f}, {py:+.0f}) mm')
                print_sensor_map(highlight=pt)
                print(f'  ┌─────────────────────────────────────────────────────┐')
                print(f'  │  Wire the LCR-6100 probes to point  P{pt:02d} ({lbl:^3})      │')
                print(f'  │  Freq: 20 kHz | Mode: Cp-Rp | Volt: 1 V | FAST     │')
                print(f'  └─────────────────────────────────────────────────────┘')
                Cp_now, _, _ = lcr.get_latest()
                print(f'  Current LCR reading: Cp = {Cp_now * 1e12:.2f} pF')
                try:
                    input('  Press ENTER when wired and ready ... ')
                except (EOFError, KeyboardInterrupt):
                    print('\n  Interrupted — saving partial dataset ...')
                    break

            print('─' * 65)
            print(f'  Step {step + 1}/{total}  |  P{pt:02d}  |  depth {depth_mm:.2f} mm  '
                  f'|  iteration {iter_idx + 1}/{iterations}')

            try:
                do_indentation(rtde_c, pt, depth_mm, point_idx, iter_idx, lcr,
                               locate_s=locate_s, hold_s=hold_s, post_s=post_s,
                               ramp_s=ramp_s, rate_hz=rate_hz,
                               base_depth_mm=args.base_depth)
            except KeyboardInterrupt:
                print('\n  Interrupted during indentation — saving partial dataset ...')
                break
            except Exception as e:
                print(f'\n  [error] {e}')
                print('  Moving to home for safety ...')
                try:
                    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
                except Exception:
                    pass

            completed += 1
            elapsed = time.time() - start_time
            avg_s   = elapsed / completed
            remain  = (total - completed) * avg_s
            print(f'  Progress: {completed}/{total}  |  Elapsed: {elapsed/60:.1f} min  '
                  f'|  Est. remaining: {remain/60:.1f} min')

            # Autosave every 10 completed indentations (crash-safety on long runs)
            if completed % 10 == 0:
                print('  [autosave]')
                save_dataset(points)

            # End of a depth's iterations → note progress
            if step == total - 1 or plan[step + 1][2] != depth_mm or plan[step + 1][0] != point_idx:
                print(f'\n  ✓ P{pt:02d} @ {depth_mm:.2f} mm — {iterations} iterations complete.')
    except KeyboardInterrupt:
        print('\n  Interrupted')
    finally:
        _ft_stop.set()
        print('\n[robot] Returning to home ...')
        try:
            rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
            rtde_c.stopScript()
        except Exception:
            pass
        lcr.disconnect()
        path = save_dataset(points)
        print(f'\n  Completed {completed}/{total} indentations.')
        if path:
            print(f'  Data: {path}')
    print('[done]')


if __name__ == '__main__':
    main()
