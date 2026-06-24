"""
capacitance_dataset_collector.py — Star-Nose Sensor | Capacitance Dataset Builder
==================================================================================
Collects a multi-sample dataset correlating:
  • LCR-6100 capacitance (Cp-Rp, 20 kHz, 1 V, FAST) for each sensor point
  • UR5 TCP position and F/T sensor
  • FUTEK load cell (AI0)

All signals are sampled at 100 Hz (10 ms) into a single CSV per session.

Sampling strategy
-----------------
  N rounds × 19 points in random order per round.
  This ensures every point is measured exactly once per round and samples
  are spread uniformly across time (maximum statistical independence).

Indentation waveform (step impulse)
------------------------------------
  locate  : robot moves to point surface, holds 1 s
  press   : robot presses to depth at VEL_PRESS m/s
  hold    : robot holds at depth, 1 s
  retract : robot retracts to surface at VEL_PRESS m/s
  post    : robot holds at surface, 1 s

Logging window
--------------
  Starts when user confirms wiring (Enter).
  Stops at the end of the post-retract hold.
  All phases within this window are written to CSV.

Usage
-----
  python capacitance_dataset_collector.py
  python capacitance_dataset_collector.py --samples 5 --depth 1.5
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
from datetime import datetime

import rtde_control
import rtde_receive
from lcr6100 import LCR6100, list_ports

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'integration_2'))
LOG_DIR      = os.path.join(_HERE, 'logs')

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_IP   = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
VEL_TRAVEL = 0.05    # m/s — travel between points
VEL_PRESS  = 0.004   # m/s — slow press / retract
ACCEL      = 0.3     # m/s²
SAFE_HOME_Z = 30.0   # mm above surface at home

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
    -0.03746 + 0.0005,
    -0.50066 + 0.0016,
     0.06054,
    -2.35063, 2.08341, -0.00009,
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
POINT_OFFSETS = {}   # pt → (dx_mm, dy_mm)

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

# ── Dataset log ───────────────────────────────────────────────────────────────
_log_rows = []
_log_lock = threading.Lock()

FIELDNAMES = [
    'timestamp', 'datetime',
    'round_idx', 'sample_idx', 'point', 'depth_mm', 'phase',
    'tcp_x', 'tcp_y', 'tcp_z',
    'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
    'ai0', 'load_cell_N',
    'Cp_F', 'Cp_pF', 'Rp_Ohm', 'lcr_ok',
]

def _log_row(pt, depth_mm, phase, round_idx, sample_idx, lcr):
    st  = get_robot_state()
    ft  = st['ft']
    tcp = st['tcp']
    ai0 = st['ai0']
    Cp, Rp, lcr_ok = lcr.get_latest()
    row = {
        'timestamp':   round(time.time(), 4),
        'datetime':    datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'round_idx':   round_idx,
        'sample_idx':  sample_idx,
        'point':       pt,
        'depth_mm':    depth_mm,
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

def _log_timed(pt, depth_mm, phase, round_idx, sample_idx,
               lcr, rate_hz, duration_s):
    interval = 1.0 / rate_hz
    t_end    = time.time() + duration_s
    while time.time() < t_end:
        t0 = time.perf_counter()
        _log_row(pt, depth_mm, phase, round_idx, sample_idx, lcr)
        rem = interval - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)

def save_dataset():
    with _log_lock:
        rows = list(_log_rows)
    if not rows:
        print('[log] Nothing to save.')
        return None
    os.makedirs(LOG_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(LOG_DIR, f'capacitance_dataset_{ts}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)
    print(f'[log] Saved {len(rows)} rows → {path}')
    return path

# ── Calibration ───────────────────────────────────────────────────────────────

def list_calib_files():
    """Return list of (tip_name, global_path, points_path) for each calib file."""
    pattern = os.path.join(_INTEGRATION, 'calib_*.json')
    files   = sorted(glob.glob(pattern))
    results = []
    for path in files:
        base = os.path.basename(path)
        if base.startswith('calib_points_'):
            continue   # skip per-point files in the main list
        tip = base[len('calib_'):-len('.json')]
        pts = os.path.join(_INTEGRATION, f'calib_points_{tip}.json')
        results.append((tip, path, pts if os.path.exists(pts) else None))
    # Also add calib.json (no tip suffix) if it exists
    plain = os.path.join(_INTEGRATION, 'calib.json')
    if os.path.exists(plain):
        pts = os.path.join(_INTEGRATION, 'calib_points.json')
        results.insert(0, ('(default)', plain, pts if os.path.exists(pts) else None))
    return results

def select_calibration():
    """Interactively let user select and load a calibration file."""
    global CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM, POINT_OFFSETS

    files = list_calib_files()
    if not files:
        print(f'[calib] No calibration files found in {_INTEGRATION}')
        print('[calib] Using zero offsets — robot may not hit points correctly!')
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

# ── Sampling plan ─────────────────────────────────────────────────────────────

def generate_plan(n_samples, seed=None):
    """
    Return list of (round_idx, sample_idx, point) tuples.

    Strategy: N rounds, each round visits all 19 points in a random order.
    This gives each point exactly N samples spread uniformly across time.
    'sample_idx' is the per-point counter (0..N-1); 'round_idx' is the round.
    """
    rng  = random.Random(seed)
    pts  = list(POINTS.keys())          # [1..19]
    plan = []
    per_point_count = {p: 0 for p in pts}

    for round_idx in range(n_samples):
        order = pts.copy()
        rng.shuffle(order)
        for pt in order:
            plan.append((round_idx, per_point_count[pt], pt))
            per_point_count[pt] += 1

    return plan

def print_plan_summary(plan, n_samples):
    total = len(plan)
    print(f'\n  Sampling plan: {n_samples} samples × 19 points = {total} indentations')
    print(f'  First 10 steps:')
    for round_idx, sample_idx, pt in plan[:10]:
        px, py = POINTS[pt]
        print(f'    round={round_idx}  sample={sample_idx}  P{pt:02d} '
              f'({px:+.0f},{py:+.0f}) mm')
    if total > 10:
        print(f'    ... ({total - 10} more)')

# ── Indentation ───────────────────────────────────────────────────────────────

def do_indentation(rtde_c, pt, depth_mm, round_idx, sample_idx,
                   lcr, hold_s=10.0, rate_hz=100):
    """
    Execute one step-impulse indentation and log all data at rate_hz.

    Waveform:
      locate:   surface → hold hold_s s
      press:    press motion (logged row-by-row)
      hold:     at depth, hold hold_s s
      retract:  retract motion (logged row-by-row)
      post:     surface → hold hold_s s
    """
    surface = _build_pose(pt, 0.0)
    pressed = _build_pose(pt, -depth_mm)

    def log(phase):
        _log_row(pt, depth_mm, phase, round_idx, sample_idx, lcr)

    # ── Locate: move to surface, hold ────────────────────────────────────────
    print(f'     → locate  (moving to surface P{pt:02d}) ...')
    rtde_c.moveL(surface, VEL_TRAVEL, ACCEL)
    log('locate')
    _log_timed(pt, depth_mm, 'locate', round_idx, sample_idx,
               lcr, rate_hz, hold_s)

    st  = get_robot_state()
    lc  = _ai0_to_n(st['ai0'])
    Cp, _, _ = lcr.get_latest()
    print(f'     [locate]   LC={lc:.2f} N   Cp={Cp*1e12:.2f} pF')

    # ── Press: press to depth ────────────────────────────────────────────────
    print(f'     → press   (depth {depth_mm:.2f} mm) ...')
    rtde_c.moveL(pressed, VEL_PRESS, ACCEL)
    log('press')

    # ── Hold: hold at depth ───────────────────────────────────────────────────
    _log_timed(pt, depth_mm, 'hold', round_idx, sample_idx,
               lcr, rate_hz, hold_s)

    st  = get_robot_state()
    lc  = _ai0_to_n(st['ai0'])
    Cp, _, _ = lcr.get_latest()
    print(f'     [hold]     LC={lc:.2f} N   Cp={Cp*1e12:.2f} pF')

    # ── Retract: return to surface ────────────────────────────────────────────
    print('     → retract  (back to surface) ...')
    rtde_c.moveL(surface, VEL_PRESS, ACCEL)
    log('retract')

    # ── Post: hold at surface after retract ───────────────────────────────────
    _log_timed(pt, depth_mm, 'post', round_idx, sample_idx,
               lcr, rate_hz, hold_s)

    st  = get_robot_state()
    lc  = _ai0_to_n(st['ai0'])
    Cp, _, _ = lcr.get_latest()
    print(f'     [post]     LC={lc:.2f} N   Cp={Cp*1e12:.2f} pF')

    with _log_lock:
        n_rows = len(_log_rows)
    print(f'     Done — dataset rows so far: {n_rows}')

# ── Display helpers ───────────────────────────────────────────────────────────

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

def _select_lcr_port():
    ports = list_ports()
    if not ports:
        print('\n  [LCR] No serial ports found — is the USB cable connected?')
        raw = input('  Enter port manually (e.g. /dev/ttyUSB0): ').strip()
        return raw

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

# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Capacitance dataset collector — 19-point star-nose sensor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--samples', type=int, default=None,
                   help='Samples per point (default: ask interactively, fallback 10)')
    p.add_argument('--depth',   type=float, default=None,
                   help='Indentation depth in mm (default: ask interactively)')
    p.add_argument('--rate',    type=int, default=100,
                   help='Logging rate in Hz (default: 100)')
    p.add_argument('--hold',    type=float, default=1.0,
                   help='Hold duration per phase in seconds (default: 1.0)')
    p.add_argument('--port',    default=None,
                   help='LCR serial port (default: interactive selection)')
    p.add_argument('--seed',    type=int, default=None,
                   help='Random seed for sampling plan (default: random)')
    return p.parse_args()


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


def _ask_float(prompt, default, minimum=0.01, maximum=50.0):
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


def main():
    args = parse_args()

    print('=' * 65)
    print('  Capacitance Dataset Collector — Star-Nose Sensor')
    print('=' * 65)

    # ── Session parameters ────────────────────────────────────────────────────
    n_samples = args.samples or _ask_int(
        '\n  Samples per point [10] > ', default=10, minimum=1, maximum=100)

    depth_mm = args.depth or _ask_float(
        f'  Indentation depth (mm) [1.0] > ', default=1.0, minimum=0.1, maximum=10.0)

    rate_hz  = args.rate
    hold_s   = args.hold

    print(f'\n  Samples per point : {n_samples}')
    print(f'  Indentation depth : {depth_mm:.2f} mm')
    print(f'  Hold per phase    : {hold_s:.1f} s')
    print(f'  Log rate          : {rate_hz} Hz')
    print(f'  Total indentations: {n_samples * 19}')
    print(f'  Est. time         : ~{n_samples * 19 * (3 * hold_s + 20) / 60:.0f} min '
          f'(excl. wiring time)')

    # ── Calibration ───────────────────────────────────────────────────────────
    select_calibration()

    # ── LCR port ──────────────────────────────────────────────────────────────
    lcr_port = args.port or _select_lcr_port()

    # ── Generate plan ─────────────────────────────────────────────────────────
    seed = args.seed if args.seed is not None else random.randint(0, 99999)
    plan = generate_plan(n_samples, seed=seed)
    print_plan_summary(plan, n_samples)
    print(f'  Seed: {seed}  (save this to reproduce the same order)')

    # ── Connect LCR ───────────────────────────────────────────────────────────
    print('\n[LCR] Connecting ...')
    lcr = LCR6100(lcr_port)
    try:
        lcr.connect()
    except Exception as e:
        print(f'[LCR] Connection failed: {e}')
        sys.exit(1)
    lcr.start_polling()
    time.sleep(0.5)
    Cp0, Rp0, ok0 = lcr.get_latest()
    print(f'[LCR] Initial reading: Cp={Cp0*1e12:.2f} pF  Rp={Rp0/1e3:.1f} kΩ  ok={ok0}')

    # ── Connect robot ─────────────────────────────────────────────────────────
    print(f'\n[robot] Connecting to {ROBOT_IP} ...')
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        rtde_c = rtde_control.RTDEControlInterface(
            ROBOT_IP, frequency=500.0,
            flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
    except Exception as e:
        print(f'[robot] Connection failed: {e}')
        lcr.disconnect()
        sys.exit(1)
    print('[robot] Connected')

    ft_thread = threading.Thread(target=_ft_reader, args=(rtde_r,), daemon=True)
    ft_thread.start()
    time.sleep(0.3)

    print('[robot] Moving to home ...')
    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
    print('[robot] At home\n')

    # ── Main collection loop ───────────────────────────────────────────────────
    completed  = 0
    total      = len(plan)
    start_time = time.time()

    try:
        for step, (round_idx, sample_idx, pt) in enumerate(plan):
            px, py = POINTS[pt]

            print('─' * 65)
            print(f'  Step {step + 1}/{total}  |  Round {round_idx + 1}/{n_samples}  '
                  f'|  P{pt:02d}  sample {sample_idx + 1}/{n_samples}')
            print(f'  Point P{pt:02d}  ({px:+.0f}, {py:+.0f}) mm')
            print_sensor_map(highlight=pt)

            # ── Instruct user to wire the LCR ─────────────────────────────
            print(f'  ┌─────────────────────────────────────────────────────┐')
            print(f'  │  Wire the LCR-6100 probes to point  P{pt:02d}            │')
            print(f'  │  Freq: 20 kHz | Mode: Cp-Rp | Volt: 1 V | FAST     │')
            print(f'  └─────────────────────────────────────────────────────┘')
            Cp_now, _, _ = lcr.get_latest()
            print(f'  Current LCR reading: Cp = {Cp_now * 1e12:.2f} pF')

            try:
                input('  Press ENTER when wired and ready ... ')
            except (EOFError, KeyboardInterrupt):
                print('\n  Interrupted — saving partial dataset ...')
                break

            # Confirm LCR is reading something reasonable
            Cp_now, Rp_now, ok_now = lcr.get_latest()
            print(f'  LCR at start: Cp={Cp_now*1e12:.2f} pF  '
                  f'Rp={Rp_now/1e3:.1f} kΩ  ok={ok_now}')

            # ── Indentation (logging starts here) ─────────────────────────
            try:
                do_indentation(rtde_c, pt, depth_mm, round_idx, sample_idx,
                               lcr, hold_s=hold_s, rate_hz=rate_hz)
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
                  f'Elapsed: {elapsed/60:.1f} min  |  '
                  f'Est. remaining: {remain/60:.1f} min')

            # Autosave every 10 completed steps
            if completed % 10 == 0:
                print('  [autosave]')
                save_dataset()

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

        path = save_dataset()
        print(f'\n  Completed {completed}/{total} indentations.')
        if path:
            print(f'  Dataset saved to: {path}')
            print(f'\n  Analyse with:')
            print(f'    python analyze_capacitance_dataset.py "{path}"')
        print('[done]')


if __name__ == '__main__':
    main()
