"""
main.py — Interdome_touch  |  Star-Nose Sensor + UR5 + FUTEK  |  long-hold collector
======================================================================================
Combines, in one session (like Integration_2/main.py combines robot + sensor +
logging), three data sources while the UR5 presses every point of the 19-point
star-nose grid:

  1. UR5 robot control (position, TCP force/torque, AI0)
  2. The 19-cell capacitive skin sensor (Integration_2/sensor.py)
  3. The FUTEK load cell (AI0 channel), converted to Newtons with the
     direct voltage->force calibration confirmed on 2026-07-23 (see
     "FUTEK load cell calibration" below)

Calibration
-----------
At startup you choose a TCP calibration profile (calib_<tip>.json /
calib.json) from Integration_2/, exactly like the interactive selector in
Capacitance_measurement/capacitance_dataset_collector_long_hold.py.

Points
------
The 19 pressing points are NOT the hardcoded nominal grid used elsewhere —
they are read directly from
    Integration_2/calib_points_supposed_rigid_transformed.json
(the rigid rotation+translation fit of the "supposed" grid onto the actual
robot-calibrated positions). Point numbering matches ur5_control.py /
capacitance_dataset_collector_long_hold.py (point 10 = center/anchor).

Pushing schema (same as capacitance_dataset_collector_long_hold.py)
---------------------------------------------------------------------
Each indentation is a 5-phase step impulse:
    locate  (positioning) : move to point surface, hold hold_s_eff
    press   (ramp down)   : press to depth at VEL_PRESS m/s     (ramp_s)
    hold    (holding)     : hold at depth, hold_s_eff (>= ramp_s)
    retract (ramp up)     : retract to surface at VEL_PRESS m/s (ramp_s)
    post    (releasing)   : hold at surface, hold_s_eff
where hold_s_eff = max(user_hold_s, ramp_s * hold_mult), so the dwell at
depth is never shorter than the press/retract ramp itself. High-rate
logging (rate_hz) runs continuously during the three hold phases; press/
retract only get a single before/after snapshot row (moveL is blocking),
matching the reference script exactly.

Test matrix
-----------
5 indentation depths (mm) x N iterations x 19 points, where N is chosen
interactively (or via --iterations). Each iteration presses every point
once, in a random order, so every point gets N samples per depth spread
uniformly across the session.

Output
------
  logs/interdome_<timestamp>.csv        — one row per logged sample
  logs/interdome_<timestamp>_meta.json  — depths, iterations, calibration,
                                           points source, FUTEK coefficients,
                                           hold/ramp parameters, seed

Usage
-----
  python3 main.py
  python3 main.py --iterations 5 --depths 1 2 3 4 5
  python3 main.py --hold 1.5 --hold-mult 1.2 --rate 100
  python3 main.py --no-sensor           # robot + FUTEK only, skip capacitive sensor
  UR_ROBOT_IP=127.0.0.1 python3 main.py # URSim

Analysis
--------
  python3 analyze_interdome.py logs/interdome_<timestamp>.csv
"""

import os
import sys
import json
import csv
import glob
import time
import random
import threading
import argparse
import math
from datetime import datetime

import rtde_control
import rtde_receive

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
LOG_DIR      = os.path.join(_HERE, 'logs')
sys.path.insert(0, _INTEGRATION)

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_IP    = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
VEL_TRAVEL  = 0.05     # m/s — travel between points
VEL_PRESS   = 0.004    # m/s — slow press / retract (matches capacitance long-hold collector)
ACCEL       = 0.3      # m/s^2
SAFE_HOME_Z = 30.0     # mm above surface at home

REFERENCE_POSE = [
    -0.03746 + 0.0005,
    -0.50066 + 0.0016,
     0.06054,
    -2.35063, 2.08341, -0.00009,
]

# ── FUTEK load cell calibration ────────────────────────────────────────────────
# Direct AI0 (V) -> force (N), confirmed with the user 2026-07-23.
# Source: force_sensor_calibration/Matlab calibration/step1_loadcell_calibration.json
#   F_futek_N = FUTEK_SLOPE_N_PER_V * ai0 + FUTEK_OFFSET_N
#   R^2 = 0.999964   RMSE = 0.0158 N   (n=44, fzcal_futek_direct_* v2 dataset)
# Chosen over the calib_fz_lc_pattern.json "fz_signed" correction because that
# formula recovers a load-cell-equivalent force from the ROBOT's own F/T sensor
# for rigs WITHOUT a physical load cell in the loop — here the FUTEK is
# physically wired to AI0, so the direct fit is the accurate, simpler choice.
FUTEK_SLOPE_N_PER_V = 4.113951054770791
FUTEK_OFFSET_N       = -19.28418747084478

def ai0_to_futek_n(ai0_v):
    return FUTEK_SLOPE_N_PER_V * float(ai0_v) + FUTEK_OFFSET_N

# ── Point -> raw sensor cell (same rig/numbering as Integration_2/ur5_control.py) ─
UR5_TO_SENSOR = {
    1:24,  2:12,  3:0,
    4:37,  5:25,  6:13,  7:1,
    8:50,  9:38,  10:26, 11:14, 12:2,
    13:51, 14:39, 15:27, 16:15,
    17:52, 18:40, 19:28,
}

SENSOR_MAP_ROWS = [
    [1, 2, 3],
    [4, 5, 6, 7],
    [8, 9, 10, 11, 12],
    [13, 14, 15, 16],
    [17, 18, 19],
]

ANCHOR_POINT = 10

DEFAULT_DEPTHS_MM = [1.0, 2.0, 3.0, 4.0, 5.0]

# ── Points (loaded from calib_points_supposed_rigid_transformed.json) ─────────
POINTS      = {}   # pt -> (x_mm, y_mm)
POINTS_META = {}

def load_points():
    path = os.path.join(_INTEGRATION, 'calib_points_supposed_rigid_transformed.json')
    if not os.path.exists(path):
        raise FileNotFoundError(f'Points file not found: {path}')
    with open(path) as f:
        d = json.load(f)
    pts = {int(k): (v['x_mm'], v['y_mm']) for k, v in d['points'].items()}
    meta = {
        'source':         path,
        'anchor_point':   d.get('anchor_point', ANCHOR_POINT),
        'rotation_deg':   d.get('rotation_deg', 0.0),
        'rotation_matrix':d.get('rotation_matrix'),
        'translation_mm': d.get('translation_mm'),
        'residual_mm':    d.get('residual_mm'),
    }
    return pts, meta

# ── TCP calibration (global X/Y/Z offset, chosen interactively) ───────────────
CALIB_X_MM = CALIB_Y_MM = CALIB_Z_MM = 0.0
CALIB_TIP  = '(none)'

def list_calib_files():
    """[(tip_name, path), ...] for every calib_*.json in Integration_2, excluding
    the per-point calib_points_*.json files."""
    pattern = os.path.join(_INTEGRATION, 'calib_*.json')
    results = []
    for path in sorted(glob.glob(pattern)):
        base = os.path.basename(path)
        if base.startswith('calib_points_'):
            continue
        tip = '(default)' if base == 'calib.json' else base[len('calib_'):-len('.json')]
        results.append((tip, path))
    return results

def select_calibration():
    """Interactively list & select a TCP calibration profile, then confirm."""
    global CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM, CALIB_TIP

    files = list_calib_files()
    if not files:
        print(f'[calib] No calibration files found in {_INTEGRATION}')
        print('[calib] Using zero TCP offset — robot may not hit points correctly!')
        return

    print()
    print('  Available TCP calibration profiles (Integration_2/calib_*.json):')
    for i, (tip, path) in enumerate(files):
        with open(path) as f:
            d = json.load(f)
        print(f'    [{i}]  {tip:20s}  '
              f'X={d.get("x_mm", 0):+.3f}  Y={d.get("y_mm", 0):+.3f}  '
              f'Z={d.get("z_mm", 0):+.3f} mm')

    while True:
        try:
            raw = input(f'\n  Select calibration [0-{len(files) - 1}] > ').strip()
            idx = int(raw)
            if 0 <= idx < len(files):
                break
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
        print(f'  Please enter a number between 0 and {len(files) - 1}')

    tip, path = files[idx]
    with open(path) as f:
        d = json.load(f)
    CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM = (
        d.get('x_mm', 0.0), d.get('y_mm', 0.0), d.get('z_mm', 0.0))
    CALIB_TIP = tip
    print(f'\n  [calib] Profile "{tip}": '
          f'X={CALIB_X_MM:+.3f}  Y={CALIB_Y_MM:+.3f}  Z={CALIB_Z_MM:+.3f} mm')

    try:
        ans = input('\n  Correct tip mounted? Confirm calibration? [y/N] > ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        raise SystemExit(1)
    if ans != 'y':
        print('[calib] Aborted — re-run to select a different calibration.')
        raise SystemExit(1)

# ── Shared robot state (background thread) ────────────────────────────────────
_state      = {'ft': [0.0] * 6, 'tcp': [0.0] * 6, 'ai0': 0.0}
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
        return {k: (list(v) if isinstance(v, list) else v) for k, v in _state.items()}

# ── Ramp-time model (identical to capacitance_dataset_collector_long_hold.py) ──
def ramp_time_s(depth_mm, vel=VEL_PRESS, accel=ACCEL):
    d = abs(depth_mm) / 1000.0
    if d <= 0.0:
        return 0.0
    d_to_vel = vel * vel / accel
    if d >= d_to_vel:
        return d / vel + vel / accel
    return 2.0 * (d / accel) ** 0.5

# ── Pose builders ───────────────────────────────────────────────────────────────
def _build_pose(pt, extra_z_mm=0.0):
    dx, dy = POINTS[pt]
    pose = list(REFERENCE_POSE)
    pose[0] += (dx + CALIB_X_MM) / 1000.0
    pose[1] += (dy + CALIB_Y_MM) / 1000.0
    pose[2] += (extra_z_mm + CALIB_Z_MM) / 1000.0
    return pose

def _home_pose():
    return _build_pose(ANCHOR_POINT, SAFE_HOME_Z)

# ── Dataset log ────────────────────────────────────────────────────────────────
_log_rows = []
_log_lock = threading.Lock()

FIELDNAMES = (
    ['timestamp', 'datetime',
     'depth_idx', 'depth_mm', 'iteration', 'point', 'phase',
     'point_x_mm', 'point_y_mm', 'raw_sensor_cell',
     'tcp_x', 'tcp_y', 'tcp_z',
     'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
     'ai0', 'futek_force_N']
    + [f'cell_{i + 1}' for i in range(19)]
)

def _log_row(sensor_mod, pt, depth_mm, depth_idx, phase, iteration):
    st  = get_robot_state()
    ft, tcp, ai0 = st['ft'], st['tcp'], st['ai0']
    px, py = POINTS[pt]
    row = {
        'timestamp':      round(time.time(), 4),
        'datetime':       datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'depth_idx':      depth_idx,
        'depth_mm':       depth_mm,
        'iteration':      iteration,
        'point':          pt,
        'phase':          phase,
        'point_x_mm':     round(px, 4),
        'point_y_mm':     round(py, 4),
        'raw_sensor_cell':UR5_TO_SENSOR.get(pt, -1),
        'tcp_x':          round(tcp[0], 6),
        'tcp_y':          round(tcp[1], 6),
        'tcp_z':          round(tcp[2], 6),
        'fx':             round(ft[0], 4),
        'fy':             round(ft[1], 4),
        'fz':             round(ft[2], 4),
        'tx':             round(ft[3], 4),
        'ty':             round(ft[4], 4),
        'tz':             round(ft[5], 4),
        'ai0':            round(ai0, 5),
        'futek_force_N':  round(ai0_to_futek_n(ai0), 4),
    }
    if sensor_mod is not None:
        vals = sensor_mod.get_values()
    else:
        vals = [0.0] * 19
    for i, v in enumerate(vals):
        row[f'cell_{i + 1}'] = round(float(v), 4)

    with _log_lock:
        _log_rows.append(row)

def _log_timed(sensor_mod, pt, depth_mm, depth_idx, phase, iteration, rate_hz, duration_s):
    interval = 1.0 / rate_hz
    t_end    = time.time() + duration_s
    while time.time() < t_end:
        t0 = time.perf_counter()
        _log_row(sensor_mod, pt, depth_mm, depth_idx, phase, iteration)
        rem = interval - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)

def save_dataset(path):
    with _log_lock:
        rows = list(_log_rows)
    if not rows:
        print('[log] Nothing to save.')
        return None
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)
    print(f'[log] Saved {len(rows)} rows -> {path}')
    return path

# ── Sampling plan ────────────────────────────────────────────────────────────
def generate_plan(depths_mm, n_iterations, seed=None):
    """[(depth_idx, depth_mm, iteration, point), ...].

    5 depths (outer) x n_iterations (middle) x 19 points (inner, random order
    per iteration) — every point gets exactly n_iterations samples per depth,
    spread uniformly across time within that depth block."""
    rng  = random.Random(seed)
    pts  = list(POINTS.keys())
    plan = []
    for depth_idx, depth in enumerate(depths_mm):
        for it in range(n_iterations):
            order = pts.copy()
            rng.shuffle(order)
            for pt in order:
                plan.append((depth_idx, depth, it, pt))
    return plan

def print_plan_summary(plan, depths_mm, n_iterations):
    total = len(plan)
    print(f'\n  Sampling plan: {len(depths_mm)} depths x {n_iterations} iterations '
          f'x {len(POINTS)} points = {total} indentations')
    print(f'  First 10 steps:')
    for depth_idx, depth, it, pt in plan[:10]:
        px, py = POINTS[pt]
        print(f'    depth={depth:.1f}mm  iter={it}  P{pt:02d} ({px:+.1f},{py:+.1f}) mm')
    if total > 10:
        print(f'    ... ({total - 10} more)')

# ── Indentation ───────────────────────────────────────────────────────────────
def do_indentation(rtde_c, sensor_mod, pt, depth_mm, depth_idx, iteration,
                    hold_s=1.0, rate_hz=100):
    """One 5-phase step-impulse indentation, logging continuously during the
    3 hold phases (locate/hold/post) and a single snapshot for press/retract
    (moveL blocks, so nothing can be sampled mid-ramp) — identical structure
    to capacitance_dataset_collector_long_hold.py's do_indentation()."""
    surface = _build_pose(pt, 0.0)
    pressed = _build_pose(pt, -depth_mm)

    def snapshot(phase):
        _log_row(sensor_mod, pt, depth_mm, depth_idx, phase, iteration)

    # ── locate (positioning): move to surface, hold ──────────────────────────
    rtde_c.moveL(surface, VEL_TRAVEL, ACCEL)
    snapshot('locate')
    _log_timed(sensor_mod, pt, depth_mm, depth_idx, 'locate', iteration, rate_hz, hold_s)

    # ── press (ramp down): press to depth ─────────────────────────────────────
    rtde_c.moveL(pressed, VEL_PRESS, ACCEL)
    snapshot('press')

    # ── hold (holding): hold at depth (>= ramp) ───────────────────────────────
    _log_timed(sensor_mod, pt, depth_mm, depth_idx, 'hold', iteration, rate_hz, hold_s)

    st = get_robot_state()
    print(f'     [hold]     P{pt:02d} @{depth_mm:.1f}mm   '
          f'Fz={st["ft"][2]:+.2f}N   FUTEK={ai0_to_futek_n(st["ai0"]):+.2f}N')

    # ── retract (ramp up): return to surface ──────────────────────────────────
    rtde_c.moveL(surface, VEL_PRESS, ACCEL)
    snapshot('retract')

    # ── post (releasing): hold at surface after retract ───────────────────────
    _log_timed(sensor_mod, pt, depth_mm, depth_idx, 'post', iteration, rate_hz, hold_s)

    with _log_lock:
        n_rows = len(_log_rows)
    print(f'     rows so far: {n_rows}')

# ── Display helper ──────────────────────────────────────────────────────────
def print_sensor_map(highlight=None):
    print()
    for row in SENSOR_MAP_ROWS:
        indent = ' ' * (2 * (5 - len(row)))
        parts  = []
        for pt in row:
            tag = f'[{pt:02d}]' if pt == highlight else f' {pt:02d} '
            parts.append(tag)
        print('  ' + indent + ' '.join(parts))
    print()

# ── CLI helpers ──────────────────────────────────────────────────────────────
def _ask_int(prompt, default, minimum=1, maximum=1000):
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

def _ask_depths(prompt, default_list):
    default_str = ','.join(f'{d:g}' for d in default_list)
    while True:
        try:
            raw = input(f'{prompt} [{default_str}] > ').strip()
            if raw == '':
                return list(default_list)
            vals = [float(x) for x in raw.replace(',', ' ').split()]
            if len(vals) == 5 and all(0.0 < v <= 20.0 for v in vals):
                return vals
            print('  Enter exactly 5 depths in mm (e.g. 1,2,3,4,5), each between 0 and 20')
        except (ValueError, EOFError, KeyboardInterrupt):
            return list(default_list)

def parse_args():
    p = argparse.ArgumentParser(
        description='Interdome_touch — Star-Nose Sensor + UR5 + FUTEK (long-hold schema)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--iterations', type=int, default=None,
                   help='Iterations per depth (default: ask interactively, fallback 10)')
    p.add_argument('--depths', nargs=5, type=float, default=None,
                   help='Exactly 5 indentation depths in mm (default: ask interactively)')
    p.add_argument('--rate', type=int, default=100,
                   help='Logging rate in Hz during hold phases (default: 100)')
    p.add_argument('--hold', type=float, default=1.0,
                   help='Minimum hold per phase in seconds — a floor; the actual '
                        'hold is raised to at least the ramp time (default: 1.0)')
    p.add_argument('--hold-mult', type=float, default=1.0, dest='hold_mult',
                   help='Hold = ramp_time * this multiplier (>=1.0). default: 1.0')
    p.add_argument('--no-sensor', action='store_true',
                   help='Skip the capacitive sensor (robot + FUTEK only)')
    p.add_argument('--seed', type=int, default=None,
                   help='Random seed for the sampling plan (default: random)')
    return p.parse_args()

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    print('=' * 70)
    print('  Interdome_touch — Star-Nose Sensor + UR5 + FUTEK (long-hold schema)')
    print('=' * 70)

    n_iterations = args.iterations or _ask_int(
        '\n  Iterations per depth [10] > ', default=10, minimum=1, maximum=100)

    depths_mm = args.depths or _ask_depths(
        '  Indentation depths (mm, exactly 5, comma-separated)', DEFAULT_DEPTHS_MM)
    depths_mm = sorted(depths_mm)

    rate_hz   = args.rate
    hold_mult = max(1.0, args.hold_mult)

    print(f'\n  Iterations/depth   : {n_iterations}')
    print(f'  Depths             : {depths_mm} mm  (5 required)')
    print(f'  Log rate           : {rate_hz} Hz')
    print(f'  Points/depth/iter  : 19')

    # ── Calibration ───────────────────────────────────────────────────────────
    select_calibration()

    # ── Points ─────────────────────────────────────────────────────────────────
    global POINTS, POINTS_META
    POINTS, POINTS_META = load_points()
    print(f'\n  [points] Loaded {len(POINTS)} points from '
          f'{os.path.basename(POINTS_META["source"])}')
    print(f'  [points] anchor=P{POINTS_META["anchor_point"]}  '
          f'rotation={POINTS_META["rotation_deg"]:.4f} deg  '
          f'translation={POINTS_META["translation_mm"]}')

    ramp_per_depth = {d: ramp_time_s(d) for d in depths_mm}
    hold_per_depth = {d: max(args.hold, ramp_per_depth[d] * hold_mult) for d in depths_mm}
    print('\n  Depth   ramp_s   hold_s')
    for d in depths_mm:
        print(f'  {d:5.1f}   {ramp_per_depth[d]:6.2f}   {hold_per_depth[d]:6.2f}')

    total_indent = len(depths_mm) * n_iterations * len(POINTS)
    per_indent_s = {d: 3 * hold_per_depth[d] + 2 * ramp_per_depth[d] + 3.0 for d in depths_mm}
    est_s = sum(per_indent_s[d] * n_iterations * len(POINTS) for d in depths_mm)
    print(f'\n  Total indentations : {total_indent}')
    print(f'  Est. duration      : ~{est_s / 60:.0f} min ({est_s / 3600:.1f} h)')

    # ── Start capacitive sensor ───────────────────────────────────────────────
    sensor_mod = None
    if not args.no_sensor:
        print('\n[sensor] Starting capacitive sensor ...')
        import sensor as _s
        _s.start()
        if not _s.wait_until_ready(timeout=60):
            print('[sensor] ERROR: sensor not ready — aborting (use --no-sensor to skip)')
            sys.exit(1)
        sensor_mod = _s
        print('[sensor] Ready!')
    else:
        print('\n[sensor] Skipped (--no-sensor) — capacitive columns will be 0.0')

    # ── Connect robot ─────────────────────────────────────────────────────────
    print(f'\n[robot] Connecting to {ROBOT_IP} ...')
    rtde_r = rtde_c = None
    for attempt in range(3):
        try:
            rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            rtde_c = rtde_control.RTDEControlInterface(
                ROBOT_IP, frequency=500.0,
                flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
            print('[robot] Connected')
            break
        except Exception as e:
            print(f'[robot] Attempt {attempt + 1}/3 failed: {e}')
            rtde_c = rtde_r = None
            time.sleep(2)

    if rtde_c is None:
        print('[robot] Could not connect — aborting')
        sys.exit(1)

    ft_thread = threading.Thread(target=_ft_reader, args=(rtde_r,), daemon=True)
    ft_thread.start()
    time.sleep(0.3)

    print('[robot] Moving to home ...')
    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
    print('[robot] At home\n')

    # ── Plan ───────────────────────────────────────────────────────────────────
    seed = args.seed if args.seed is not None else random.randint(0, 99999)
    plan = generate_plan(depths_mm, n_iterations, seed=seed)
    print_plan_summary(plan, depths_mm, n_iterations)
    print(f'  Seed: {seed}  (save this to reproduce the same order)')

    # ── Output paths + run metadata ──────────────────────────────────────────
    os.makedirs(LOG_DIR, exist_ok=True)
    ts        = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path  = os.path.join(LOG_DIR, f'interdome_{ts}.csv')
    meta_path = os.path.join(LOG_DIR, f'interdome_{ts}_meta.json')

    meta = {
        'timestamp':        ts,
        'csv_file':         os.path.basename(csv_path),
        'depths_mm':        depths_mm,
        'iterations':       n_iterations,
        'points_per_iter':  len(POINTS),
        'total_indentations': total_indent,
        'rate_hz':          rate_hz,
        'hold_floor_s':     args.hold,
        'hold_mult':        hold_mult,
        'hold_s_per_depth': hold_per_depth,
        'ramp_s_per_depth': ramp_per_depth,
        'seed':             seed,
        'sensor_enabled':   sensor_mod is not None,
        'robot_ip':         ROBOT_IP,
        'calibration_tip':  CALIB_TIP,
        'calibration_offset_mm': {'x': CALIB_X_MM, 'y': CALIB_Y_MM, 'z': CALIB_Z_MM},
        'points_source':    POINTS_META['source'],
        'points_meta':      {k: v for k, v in POINTS_META.items() if k != 'source'},
        'ur5_to_sensor':    UR5_TO_SENSOR,
        'futek_calibration': {
            'formula': 'F_futek_N = slope_n_per_v * ai0 + offset_n',
            'slope_n_per_v': FUTEK_SLOPE_N_PER_V,
            'offset_n': FUTEK_OFFSET_N,
            'source': 'force_sensor_calibration/Matlab calibration/step1_loadcell_calibration.json',
            'confirmed_with_user': '2026-07-23',
        },
    }
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)
    print(f'\n[meta] Wrote run metadata -> {meta_path}')

    # ── Main collection loop ───────────────────────────────────────────────────
    completed  = 0
    total      = len(plan)
    start_time = time.time()

    try:
        for step, (depth_idx, depth, it, pt) in enumerate(plan):
            px, py = POINTS[pt]
            hold_s = hold_per_depth[depth]

            print('-' * 70)
            print(f'  Step {step + 1}/{total}  |  Depth {depth:.1f}mm '
                  f'({depth_idx + 1}/{len(depths_mm)})  |  Iter {it + 1}/{n_iterations}  '
                  f'|  P{pt:02d}')
            print(f'  Point P{pt:02d}  ({px:+.1f}, {py:+.1f}) mm  '
                  f'sensor=S{UR5_TO_SENSOR.get(pt, -1)}')
            print_sensor_map(highlight=pt)

            try:
                do_indentation(rtde_c, sensor_mod, pt, depth, depth_idx, it,
                                hold_s=hold_s, rate_hz=rate_hz)
            except KeyboardInterrupt:
                print('\n  Interrupted during indentation — moving to home ...')
                try:
                    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
                except Exception:
                    pass
                print('  Saving partial dataset ...')
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
            print(f'  Progress: {completed}/{total}  |  '
                  f'Elapsed: {elapsed / 60:.1f} min  |  '
                  f'Est. remaining: {remain / 60:.1f} min')

            if completed % 10 == 0:
                print('  [autosave]')
                save_dataset(csv_path)

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

        path = save_dataset(csv_path)
        print(f'\n  Completed {completed}/{total} indentations.')
        if path:
            print(f'  Dataset saved to : {path}')
            print(f'  Metadata saved to: {meta_path}')
            print(f'\n  Analyse with:')
            print(f'    python3 analyze_interdome.py "{path}"')
        print('[done]')

if __name__ == '__main__':
    main()
