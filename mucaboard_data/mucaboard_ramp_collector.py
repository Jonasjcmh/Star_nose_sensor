"""
mucaboard_ramp_collector.py — Star-Nose Sensor | Muca-Board Ramp Collector
==========================================================================
Data-collection PROCESS mirrors Integration_2/main.py (muca sensor board →
data_logger.py schema → analyze_session.py), but the press WORKFLOW is the one
from Capacitance_measurement/capacitance_ramp_collector.py (depth sweep, N
iterations, all 19 points visited once per round in random order, each
indentation locate → press → hold → retract → post with a fixed-time ramp).

Why this shape
--------------
  • There is no LCR here — capacitance comes from the muca board, read exactly
    like main.py does via Integration_2/sensor.py (get_values() → 19 pads).
  • Logging reuses the SAME CSV schema as main.py (Integration_2/data_logger.py):
    a 20 Hz background thread writes timestamp · ur5_point · ur5_pressing ·
    fx/fy/fz · ai0 · cell_1..cell_19. We add a few extra columns
    (depth_mm/phase/round_idx/sample_idx/load_cell_N) that Integration_2's
    analyzer simply ignores, so the depth sweep is preserved for deeper analysis.
  • A press event is flagged by holding ur5_pressing=1 during press+hold so
    analyze_session.get_press_events() segments each indentation just like a
    normal main.py session.
  • Rows stream to disk as they are written (crash/OOM-safe on long runs).
  • Analysis is analyze_session.py — the exact dashboard main.py runs with
    --analyze — so no bespoke analyzer is needed.

Usage
-----
  python mucaboard_ramp_collector.py
  python mucaboard_ramp_collector.py --depths 1,2,3 --iters 5 --ramp 2 \
         --hold 5 --locate 5 --post 5 --prefix ecoflex_domes --analyze
"""

import os
import sys
import csv
import time
import random
import argparse
import threading
import subprocess
from datetime import datetime

import rtde_control
import rtde_receive

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
# Logs stay in THIS folder (mucaboard_data/logs), not Integration_2. analyze.py
# (below) reuses Integration_2/analyze_session.py's functions on these files.
LOG_DIR        = os.path.join(_HERE, 'logs')
ANALYZE_SCRIPT = os.path.join(_HERE, 'analyze.py')

# Pull in the muca-board reader and logger used by Integration_2/main.py.
sys.path.insert(0, _INTEGRATION)
import sensor        # noqa: E402  (Integration_2/sensor.py)
import data_logger   # noqa: E402  (Integration_2/data_logger.py — reuse its filename convention)

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_IP   = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
VEL_TRAVEL = 0.05    # m/s — travel between points
VEL_PRESS  = 0.004   # m/s — fallback press speed (press/retract use a ramp time)
ACCEL      = 0.3     # m/s²
SAFE_HOME_Z = 30.0   # mm above surface at home

LOG_RATE_HZ = 20     # match main.py's 20 Hz logger (analyze_session assumes 0.05 s/frame)

# ── FUTEK load cell ────────────────────────────────────────────────────────────
AI0_ZERO_V       = 5.0
LOADCELL_MAX_N   = 10.0 * 4.44822
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0

def _ai0_to_n(v):
    return -(float(v) - AI0_ZERO_V) * LOADCELL_N_PER_V

# ── Sensor points (mm, relative to reference pose) ───────────────────────────
POINTS = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

REFERENCE_POSE = [
    -0.03664,
    -0.49831,
     0.06071,
    2.346, -2.094, -0.00009
]

SENSOR_MAP_ROWS = [
    [1, 2, 3],
    [4, 5, 6, 7],
    [8, 9, 10, 11, 12],
    [13, 14, 15, 16],
    [17, 18, 19],
]

# ── Calibration globals ────────────────────────────────────────────────────────
CALIB_X_MM    = 0.0
CALIB_Y_MM    = 0.0
CALIB_Z_MM    = 0.0
POINT_OFFSETS = {}     # pt → (dx_mm, dy_mm)

# ── Shared robot state (background FT thread) ─────────────────────────────────
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

# ── UR5 workflow state (what the logger stamps onto every row) ────────────────
_ur5 = {'point': '', 'pressing': False, 'done': False,
        'phase': '', 'depth_mm': 0.0, 'round_idx': -1, 'sample_idx': -1}
_ur5_lock = threading.Event  # placeholder to avoid confusion; real lock below
_ur5_state_lock = threading.Lock()

def set_workflow(**kw):
    with _ur5_state_lock:
        _ur5.update(kw)

def get_workflow():
    with _ur5_state_lock:
        return dict(_ur5)

# ── Fixed-ramp motion (trapezoidal profile) ────────────────────────────────────

# Fraction of the ramp time spent accelerating (and the same decelerating); the
# remaining middle is a constant-velocity cruise. 0.25 → 25/50/25.
RAMP_ACCEL_FRAC = 0.25

def vel_accel_for_ramp(depth_mm, ramp_s, accel_frac=RAMP_ACCEL_FRAC):
    """Speed & accel so a moveL over depth_mm completes in ~ramp_s as a trapezoid.
    v & a scale with depth, so the move takes ramp_s for any depth."""
    d = abs(depth_mm) / 1000.0
    f = min(max(accel_frac, 1e-3), 0.5)
    if d <= 0.0 or ramp_s <= 0.0:
        return VEL_PRESS, ACCEL
    vel   = d / ((1.0 - f) * ramp_s)
    accel = d / (f * (1.0 - f) * ramp_s * ramp_s)
    return vel, accel

# ── Streaming logger (data_logger schema + extras) ────────────────────────────
# Same columns as Integration_2/data_logger.py so analyze_session.py works, plus
# depth_mm/phase/round_idx/sample_idx/load_cell_N (ignored by that analyzer).
BASE_FIELDS = [
    'timestamp', 'datetime', 'ur5_point', 'ur5_pressing', 'ur5_done',
    'tcp_x', 'tcp_y', 'tcp_z', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz', 'ai0',
]
CELL_FIELDS  = [f'cell_{i+1}' for i in range(19)]
EXTRA_FIELDS = ['depth_mm', 'phase', 'round_idx', 'sample_idx', 'load_cell_N']
FIELDNAMES   = BASE_FIELDS + CELL_FIELDS + EXTRA_FIELDS

_log_file   = None
_log_writer = None
_log_fh     = None
_log_count  = 0
_log_lock   = threading.Lock()
_log_stop   = threading.Event()

def _open_log(path):
    global _log_fh, _log_writer, _log_file, _log_count
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _log_fh     = open(path, 'w', newline='')
    _log_writer = csv.DictWriter(_log_fh, fieldnames=FIELDNAMES)
    _log_writer.writeheader()
    _log_fh.flush()
    _log_file   = path
    _log_count  = 0

def _write_row():
    global _log_count
    st  = get_robot_state()
    ft  = st['ft']; tcp = st['tcp']; ai0 = st['ai0']
    wf  = get_workflow()
    vals = sensor.get_values()
    row = {
        'timestamp':    round(time.time(), 4),
        'datetime':     datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'ur5_point':    wf['point'],
        'ur5_pressing': int(wf['pressing']),
        'ur5_done':     int(wf['done']),
        'tcp_x': round(tcp[0], 5), 'tcp_y': round(tcp[1], 5), 'tcp_z': round(tcp[2], 5),
        'fx': round(ft[0], 4), 'fy': round(ft[1], 4), 'fz': round(ft[2], 4),
        'tx': round(ft[3], 4), 'ty': round(ft[4], 4), 'tz': round(ft[5], 4),
        'ai0': round(ai0, 5),
        'depth_mm':     wf['depth_mm'],
        'phase':        wf['phase'],
        'round_idx':    wf['round_idx'],
        'sample_idx':   wf['sample_idx'],
        'load_cell_N':  round(_ai0_to_n(ai0), 4),
    }
    for i, v in enumerate(vals):
        row[f'cell_{i+1}'] = round(float(v), 4)
    with _log_lock:
        if _log_writer is None:
            return
        _log_writer.writerow(row)
        _log_fh.flush()
        _log_count += 1

def _logger_loop():
    interval = 1.0 / LOG_RATE_HZ
    while not _log_stop.is_set():
        t0 = time.perf_counter()
        try:
            _write_row()
        except Exception as e:
            print(f'[logger] {e}')
        rem = interval - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)

def _close_log():
    with _log_lock:
        if _log_fh is not None:
            try:
                _log_fh.flush(); _log_fh.close()
            except Exception:
                pass

def log_count():
    with _log_lock:
        return _log_count

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
    print(f'\n  [calib] Profile "{tip}": '
          f'X={CALIB_X_MM:+.3f}  Y={CALIB_Y_MM:+.3f}  Z={CALIB_Z_MM:+.3f} mm')

    if ppath:
        with open(ppath) as f:
            pd = json.load(f)
        POINT_OFFSETS = {int(k): (v.get('dx_mm', 0.0), v.get('dy_mm', 0.0))
                         for k, v in pd.get('per_point', {}).items()}
        print(f'  [calib] Per-point offsets loaded for {len(POINT_OFFSETS)} points')
    else:
        POINT_OFFSETS = {}
        print('  [calib] No per-point file — global offset only')

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

def generate_plan(depths, n_iters, seed=None):
    """For each depth: n_iters rounds; each round visits all 19 points once in a
    random order. Returns list of (depth_mm, round_idx, sample_idx, point)."""
    rng = random.Random(seed)
    pts = list(POINTS.keys())
    plan = []
    for depth in depths:
        per_point_count = {p: 0 for p in pts}
        for round_idx in range(n_iters):
            order = pts.copy()
            rng.shuffle(order)
            for pt in order:
                plan.append((depth, round_idx, per_point_count[pt], pt))
                per_point_count[pt] += 1
    return plan

def print_plan_summary(plan, depths, n_iters):
    print(f'\n  Plan: {len(depths)} depth(s) × {n_iters} rounds × 19 points '
          f'= {len(plan)} indentations')
    print(f'  Depths (mm): {", ".join(f"{d:g}" for d in depths)}')
    head = '  '.join(f'P{p:02d}' for _, _, _, p in plan[:19])
    print(f'  First round order: {head}')

# ── Indentation (workflow of the capacitance ramp collector) ──────────────────

def do_indentation(rtde_c, pt, depth_mm, round_idx, sample_idx,
                   locate_s, hold_s, post_s, ramp_s):
    """locate → press(ramp_s) → hold → retract(ramp_s) → post.
    The background logger records the muca board + force the whole time; we only
    stamp the workflow state (point/phase/pressing/depth) so analyze_session.py
    can segment the press (ur5_pressing==1 during press+hold)."""
    surface = _build_pose(pt, 0.0)
    pressed = _build_pose(pt, -depth_mm)
    v_press, a_press = vel_accel_for_ramp(depth_mm, ramp_s)

    # ── Locate (not pressing) ─────────────────────────────────────────────────
    set_workflow(point=pt, depth_mm=depth_mm, round_idx=round_idx,
                 sample_idx=sample_idx, phase='locate', pressing=False)
    print(f'     → locate  (moving to surface P{pt:02d}) ...')
    rtde_c.moveL(surface, VEL_TRAVEL, ACCEL)
    time.sleep(locate_s)

    # ── Press ramp → pressing=1 so this counts as one press event ─────────────
    set_workflow(phase='press', pressing=True)
    print(f'     → press   (depth {depth_mm:.2f} mm, target ramp {ramp_s:.1f}s) ...')
    t0 = time.perf_counter()
    rtde_c.moveL(pressed, v_press, a_press)   # blocking
    print(f'     [press]    target ramp = {ramp_s:.1f}s   actual = {time.perf_counter()-t0:.2f}s')

    # ── Hold (still pressing) ─────────────────────────────────────────────────
    set_workflow(phase='hold')
    time.sleep(hold_s)
    v = sensor.get_value_for_ur5_point(pt)
    lc = _ai0_to_n(get_robot_state()['ai0'])
    print(f'     [hold]     LC={lc:.2f} N   sensor(P{pt:02d})={v:.3f}')

    # ── Retract → release (pressing=0) ────────────────────────────────────────
    set_workflow(phase='retract', pressing=False)
    print(f'     → retract  (back to surface, target ramp {ramp_s:.1f}s) ...')
    t0 = time.perf_counter()
    rtde_c.moveL(surface, v_press, a_press)
    print(f'     [retract]  actual = {time.perf_counter()-t0:.2f}s')

    # ── Post ──────────────────────────────────────────────────────────────────
    set_workflow(phase='post')
    time.sleep(post_s)
    print(f'     Done — rows so far: {log_count()}')

# ── Display / input helpers ────────────────────────────────────────────────────

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

def _parse_depths(raw):
    out = []
    for tok in raw.replace(',', ' ').split():
        try:
            d = float(tok)
        except ValueError:
            continue
        if 0.1 <= d <= 10.0:
            out.append(d)
    return out

def _ask_depths(default):
    while True:
        try:
            raw = input(f'  Depths in mm, space/comma separated '
                        f'[{", ".join(f"{d:g}" for d in default)}] > ').strip()
        except (EOFError, KeyboardInterrupt):
            return default
        if raw == '':
            return default
        depths = _parse_depths(raw)
        if depths:
            return depths
        print('  Enter one or more depths between 0.1 and 10.0 mm')

# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Muca-board ramp collector — main.py process, capacitance workflow',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--depths',  type=str,   default=None, help='Depths mm, comma separated [ask, default 2]')
    p.add_argument('--iters',   type=int,   default=None, help='Iterations (rounds) per depth [ask, default 5]')
    p.add_argument('--ramp',    type=float, default=None, help='Ramp time s per press/retract [ask, default 2]')
    p.add_argument('--hold',    type=float, default=None, help='Hold dwell s at full depth [ask, default 5]')
    p.add_argument('--locate',  type=float, default=None, help='Locate dwell s at surface [ask, default 5]')
    p.add_argument('--post',    type=float, default=None, help='Post dwell s after release [ask, default 5]')
    p.add_argument('--seed',    type=int,   default=None, help='Random seed for point order')
    p.add_argument('--prefix',  default=None, help='Log filename prefix [ask, like main.py]')
    p.add_argument('--analyze', action='store_true',
                   help='Run Integration_2/analyze_session.py after the run (like main.py --analyze)')
    return p.parse_args()

def run_analysis(log_file):
    print(f'\n[main] Running analysis on: {os.path.basename(log_file)}')
    try:
        subprocess.run([sys.executable, ANALYZE_SCRIPT,
                        log_file, '--save'])
    except Exception as e:
        print(f'[main] Analysis failed: {e}')

def main():
    args = parse_args()

    print('=' * 70)
    print('  Muca-Board Ramp Collector — Star-Nose Sensor')
    print('  (data process ← main.py · press workflow ← capacitance ramp collector)')
    print('=' * 70)

    depths = _parse_depths(args.depths) if args.depths else _ask_depths([2.0])
    if not depths:
        print('[main] No valid depths — aborting.'); sys.exit(1)
    n_iters = args.iters if args.iters is not None else _ask_int(
        '  Iterations (rounds) per depth [5] > ', 5, 1, 100)
    ramp_s = args.ramp if args.ramp is not None else float(_ask_int(
        '  Ramp time per press & retract (s) [2] > ', 2, 1, 30))
    hold_s = args.hold if args.hold is not None else _ask_float(
        '  Hold dwell (s, at full depth) [5.0] > ', 5.0, 0.1, 120.0)
    locate_s = args.locate if args.locate is not None else _ask_float(
        '  Locate dwell (s, at surface before press) [5.0] > ', 5.0, 0.1, 120.0)
    post_s = args.post if args.post is not None else _ask_float(
        '  Post dwell (s, at surface after release) [5.0] > ', 5.0, 0.1, 120.0)

    # Hold must outlast the ramp so the at-depth reading is settled.
    min_hold = ramp_s * 1.2
    if hold_s < min_hold:
        print(f'  [warn] Hold ({hold_s:.2f}s) does not exceed the ramp '
              f'({ramp_s:.2f}s) — raising hold to {min_hold:.2f}s.')
        hold_s = min_hold

    per_cycle_s = locate_s + ramp_s + hold_s + ramp_s + post_s
    n_indent    = len(depths) * n_iters * 19

    print(f'\n  Depths            : {", ".join(f"{d:g}" for d in depths)} mm')
    print(f'  Iterations/depth  : {n_iters}  (→ {n_indent} indentations total)')
    print(f'  Ramp (press/retr) : {ramp_s:.1f} s each')
    print(f'  Hold dwell        : {hold_s:.2f} s')
    print(f'  Locate dwell      : {locate_s:.2f} s')
    print(f'  Post dwell        : {post_s:.2f} s')
    print(f'  Log rate          : {LOG_RATE_HZ} Hz (data_logger schema)')
    print(f'  ~Cycle time       : {per_cycle_s:.1f} s/point (excl. travel)')

    # ── Calibration ───────────────────────────────────────────────────────────
    select_calibration()

    # ── Log file (data_logger naming convention, in mucaboard_data/logs) ──────
    prefix   = (data_logger.sanitize_name(args.prefix) if args.prefix
                else data_logger.ask_file_prefix())
    log_file = data_logger.build_filename(prefix, LOG_DIR)

    # ── Plan ──────────────────────────────────────────────────────────────────
    seed = args.seed if args.seed is not None else random.randint(0, 99999)
    plan = generate_plan(depths, n_iters, seed=seed)
    print_plan_summary(plan, depths, n_iters)
    print(f'  Seed: {seed}  (save this to reproduce the same order)')

    # ── Muca sensor board (started exactly like main.py) ──────────────────────
    print('\n[sensor] Starting muca board...')
    print('[sensor] Keep the sensor UNTOUCHED — first frame sets the baseline.')
    sensor.start()
    if not sensor.wait_until_ready(timeout=60):
        print('[sensor] ERROR: board not ready — check USB (/dev/ttyACM*).')
        sys.exit(1)
    print('[sensor] Ready!')

    # ── Robot ─────────────────────────────────────────────────────────────────
    print(f'\n[robot] Connecting to {ROBOT_IP} ...')
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        rtde_c = rtde_control.RTDEControlInterface(
            ROBOT_IP, frequency=500.0,
            flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
    except Exception as e:
        print(f'[robot] Connection failed: {e}')
        sys.exit(1)
    print('[robot] Connected')

    ft_thread = threading.Thread(target=_ft_reader, args=(rtde_r,), daemon=True)
    ft_thread.start()
    time.sleep(0.3)

    print('[robot] Moving to home ...')
    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
    print('[robot] At home\n')

    # ── Start streaming logger (20 Hz, like main.py) ──────────────────────────
    _open_log(log_file)
    log_thread = threading.Thread(target=_logger_loop, daemon=True)
    log_thread.start()
    print(f'[main] Logging → {log_file}  ({LOG_RATE_HZ} Hz)')

    try:
        input('  Press ENTER to start the automatic collection '
              '(Ctrl+C to abort) ... ')
    except (EOFError, KeyboardInterrupt):
        pass

    # ── Main collection loop ───────────────────────────────────────────────────
    total      = len(plan)
    completed  = 0
    start_time = time.time()
    prev_key   = None
    try:
        for step, (depth_mm, round_idx, sample_idx, pt) in enumerate(plan):
            key = (depth_mm, round_idx)
            if key != prev_key:
                print('═' * 70)
                print(f'  Depth {depth_mm:g} mm  |  Round {round_idx + 1}/{n_iters}')
                prev_key = key

            px, py = POINTS[pt]
            print('─' * 70)
            print(f'  Step {step + 1}/{total}  |  Depth {depth_mm:g} mm  '
                  f'|  Round {round_idx + 1}/{n_iters}  '
                  f'|  P{pt:02d}  sample {sample_idx + 1}/{n_iters}')
            print(f'  Point P{pt:02d}  ({px:+.0f}, {py:+.0f}) mm')
            print_sensor_map(highlight=pt)

            try:
                do_indentation(rtde_c, pt, depth_mm, round_idx, sample_idx,
                               locate_s=locate_s, hold_s=hold_s, post_s=post_s,
                               ramp_s=ramp_s)
            except KeyboardInterrupt:
                print('\n  Interrupted during indentation — stopping ...')
                break
            except Exception as e:
                print(f'\n  [error] {e}')
                print('  Moving to home for safety ...')
                set_workflow(phase='', pressing=False)
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

            if step == total - 1 or (plan[step + 1][0], plan[step + 1][1]) != key:
                print(f'\n  ✓ Depth {depth_mm:g} mm, round {round_idx + 1} '
                      f'complete (all 19 points).')
    except KeyboardInterrupt:
        print('\n  Interrupted')
    finally:
        set_workflow(phase='', pressing=False, done=True)
        _ft_stop.set()
        _log_stop.set()
        time.sleep(0.2)
        _close_log()
        print('\n[robot] Returning to home ...')
        try:
            rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
            rtde_c.stopScript()
        except Exception:
            pass
        rows = log_count()
        print(f'\n  Completed {completed}/{total} indentations.  Rows: {rows}')
        print(f'  Data: {log_file}')
        print(f'  Analyze:  python3 {os.path.join("mucaboard_data", "analyze.py")} '
              f'"{log_file}" --save')

    if args.analyze and log_count() > 100:
        time.sleep(1)
        run_analysis(log_file)
    print('[done]')


if __name__ == '__main__':
    main()
