"""
force_indentation.py  —  Star-Nose Sensor  |  Force Indentation Logger
=======================================================================
Interactive script: navigate the robot to any of the 19 sensor points,
choose indentation depth, and collect force + position data.

Two press modes (chosen at startup)
-------------------------------------
  AUTO   Robot presses to depth, holds for --hold seconds, then retracts
         automatically. Good for repeatable timed indentations.

  MANUAL Robot travels to the point surface and waits. You control when
         it presses (Enter) and when it retracts (Enter again). Logging
         runs continuously while the robot is pressed. Good for manual
         hold experiments of any duration.

What is collected per indentation
----------------------------------
  tcp_x, tcp_y, tcp_z   — actual TCP position (m)
  fx, fy, fz            — UR5 wrist F/T sensor (N)
  tx, ty, tz            — UR5 wrist torques (N·m)
  ai0                   — FUTEK load cell voltage (V)
  load_cell_N           — FUTEK reading converted to Newtons
  point, depth_mm, phase, timestamp

Calibration : Integration_2/calib_short_6mm.json  (x=-2.5, y=+3.0, z=-11.0 mm)
             + per-point offsets from calib_points_short_6mm.json

Usage
-----
  python force_indentation.py
  python force_indentation.py --tip short_6mm   # explicit tip (default)
  python force_indentation.py --hold 3.0        # auto-mode hold time (default 2 s)
  python force_indentation.py --rate 100        # log rate in Hz (default 100)

Commands at the main prompt
----------------------------
  1–19   go to that point (asks for depth, then press/release per mode)
  h      return to safe home
  s      save CSV and continue
  q      save CSV and quit
"""

import os
import sys
import json
import csv
import time
import threading
import argparse
from datetime import datetime

import rtde_control
import rtde_receive

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
LOG_DIR      = os.path.join(_HERE, 'logs')

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_IP    = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
VEL_TRAVEL  = 0.05     # m/s — fast travel between points
VEL_PRESS   = 0.004    # m/s — slow press
ACCEL       = 0.3      # m/s²
SAFE_HOME_Z = 30.0     # mm above surface at home

# ── FUTEK load cell ────────────────────────────────────────────────────────────
AI0_ZERO_V       = 5.0
LOADCELL_MAX_N   = 10.0 * 4.44822
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0

def _ai0_to_n(v):
    return -(float(v) - AI0_ZERO_V) * LOADCELL_N_PER_V

# ── Sensor points ──────────────────────────────────────────────────────────────
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

# ── Calibration ────────────────────────────────────────────────────────────────
CALIB_X_MM    = 0.0
CALIB_Y_MM    = 0.0
CALIB_Z_MM    = 0.0
POINT_OFFSETS = {}

def load_calibration(tip='short_6mm'):
    global CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM, POINT_OFFSETS

    global_file = os.path.join(_INTEGRATION, f'calib_{tip}.json')
    if not os.path.exists(global_file):
        global_file = os.path.join(_INTEGRATION, 'calib.json')
    with open(global_file) as f:
        g = json.load(f)
    CALIB_X_MM = g.get('x_mm', 0.0)
    CALIB_Y_MM = g.get('y_mm', 0.0)
    CALIB_Z_MM = g.get('z_mm', 0.0)
    print(f'[calib] Global: X={CALIB_X_MM:+.3f}  Y={CALIB_Y_MM:+.3f}  '
          f'Z={CALIB_Z_MM:+.3f} mm  ({os.path.basename(global_file)})')

    pts_file = os.path.join(_INTEGRATION, f'calib_points_{tip}.json')
    if os.path.exists(pts_file):
        with open(pts_file) as f:
            d = json.load(f)
        per = d.get('per_point', {})
        POINT_OFFSETS = {int(k): (v['dx_mm'], v['dy_mm']) for k, v in per.items()}
        print(f'[calib] Per-point offsets for {len(POINT_OFFSETS)} points  '
              f'({os.path.basename(pts_file)})')
    else:
        print(f'[calib] No per-point file for tip "{tip}" — global only')

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


# ── Live F/T reader (background thread at ~250 Hz) ─────────────────────────────
_ft_lock  = threading.Lock()
_ft_state = {'ft': [0.0]*6, 'tcp': [0.0]*6, 'ai0': AI0_ZERO_V}
_ft_stop  = threading.Event()

def _ft_reader(rtde_r):
    while not _ft_stop.is_set():
        try:
            ft  = rtde_r.getActualTCPForce()
            tcp = rtde_r.getActualTCPPose()
            ai0 = rtde_r.getStandardAnalogInput0()
            with _ft_lock:
                _ft_state['ft']  = list(ft)
                _ft_state['tcp'] = list(tcp)
                _ft_state['ai0'] = float(ai0)
        except Exception:
            pass
        time.sleep(0.004)

def get_state():
    with _ft_lock:
        return {k: list(v) if isinstance(v, list) else v
                for k, v in _ft_state.items()}


# ── Logger ─────────────────────────────────────────────────────────────────────
_log_rows = []
_log_lock = threading.Lock()

FIELDNAMES = [
    'timestamp', 'datetime', 'point', 'depth_mm', 'phase',
    'tcp_x', 'tcp_y', 'tcp_z',
    'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
    'ai0', 'load_cell_N',
]

def _log_row(pt, depth_mm, phase):
    st  = get_state()
    ft  = st['ft']
    tcp = st['tcp']
    ai0 = st['ai0']
    row = {
        'timestamp':   round(time.time(), 4),
        'datetime':    datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
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
    }
    with _log_lock:
        _log_rows.append(row)

def _log_continuous(pt, depth_mm, phase, rate_hz, stop_event):
    """Log at rate_hz until stop_event is set. Runs in a background thread."""
    interval = 1.0 / rate_hz
    while not stop_event.is_set():
        t0 = time.perf_counter()
        _log_row(pt, depth_mm, phase)
        rem = interval - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)

def _log_timed(pt, depth_mm, phase, rate_hz, duration_s):
    """Log at rate_hz for duration_s seconds (blocking)."""
    interval = 1.0 / rate_hz
    t_end    = time.time() + duration_s
    while time.time() < t_end:
        t0 = time.perf_counter()
        _log_row(pt, depth_mm, phase)
        rem = interval - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)

def save_log():
    with _log_lock:
        rows = list(_log_rows)
    if not rows:
        print('[log] Nothing to save.')
        return None
    os.makedirs(LOG_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(LOG_DIR, f'force_indentation_{ts}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)
    print(f'[log] Saved {len(rows)} rows → {path}')
    return path


# ── Indentation routines ───────────────────────────────────────────────────────

def _print_forces(label=''):
    st = get_state()
    fz = st['ft'][2]
    lc = _ai0_to_n(st['ai0'])
    tag = f'  {label}' if label else ''
    print(f'     Fz={fz:+.2f} N   LC={lc:.2f} N{tag}')

def indent_auto(rtde_c, pt, depth_mm, hold_s, rate_hz):
    """Press to depth, hold hold_s seconds while logging, then retract."""
    px, py   = POINTS[pt]
    pdx, pdy = POINT_OFFSETS.get(pt, (0.0, 0.0))
    print(f'\n  → P{pt:02d}  XY=({px:+.0f},{py:+.0f}) mm  '
          f'offset=({pdx:+.2f},{pdy:+.2f})  depth={depth_mm:.2f} mm  '
          f'hold={hold_s:.1f} s')

    surface = _build_pose(pt, 0.0)
    pressed = _build_pose(pt, -depth_mm)

    print('     Travelling to surface ...')
    rtde_c.moveL(surface, VEL_TRAVEL, ACCEL)
    _log_row(pt, depth_mm, 'approach')

    print(f'     Pressing to {depth_mm:.2f} mm ...')
    rtde_c.moveL(pressed, VEL_PRESS, ACCEL)
    _log_row(pt, depth_mm, 'press')
    _print_forces()

    print(f'     Holding {hold_s:.1f} s  (logging at {rate_hz} Hz) ...')
    _log_timed(pt, depth_mm, 'hold', rate_hz, hold_s)
    _print_forces()

    print('     Retracting ...')
    rtde_c.moveL(surface, VEL_PRESS, ACCEL)
    _log_row(pt, depth_mm, 'retract')

    print(f'     Done — total rows: {len(_log_rows)}')


def indent_manual(rtde_c, pt, depth_mm, rate_hz):
    """Travel to point, then wait for user to press / release."""
    px, py   = POINTS[pt]
    pdx, pdy = POINT_OFFSETS.get(pt, (0.0, 0.0))
    print(f'\n  → P{pt:02d}  XY=({px:+.0f},{py:+.0f}) mm  '
          f'offset=({pdx:+.2f},{pdy:+.2f})  depth={depth_mm:.2f} mm')

    surface = _build_pose(pt, 0.0)
    pressed = _build_pose(pt, -depth_mm)

    print('     Travelling to surface ...')
    rtde_c.moveL(surface, VEL_TRAVEL, ACCEL)
    _log_row(pt, depth_mm, 'approach')
    _print_forces('(at surface)')

    # ── Wait for PRESS command ─────────────────────────────────────────────────
    try:
        input('     [ Press ENTER to press down ] ')
    except (EOFError, KeyboardInterrupt):
        return

    print(f'     Pressing to {depth_mm:.2f} mm ...')
    rtde_c.moveL(pressed, VEL_PRESS, ACCEL)
    _log_row(pt, depth_mm, 'press')
    _print_forces('(engaged)')

    # ── Log continuously until RELEASE command ─────────────────────────────────
    stop_log = threading.Event()
    log_thread = threading.Thread(
        target=_log_continuous,
        args=(pt, depth_mm, 'hold', rate_hz, stop_log),
        daemon=True)
    log_thread.start()

    # Live force display while waiting for release
    print('     Logging... (live forces below)')
    print('     [ Press ENTER to release ]\n')
    try:
        # Show live Fz / LC every 0.5 s until user hits Enter
        _display_stop = threading.Event()

        def _live_display():
            while not _display_stop.is_set():
                st = get_state()
                fz = st['ft'][2]
                lc = _ai0_to_n(st['ai0'])
                rows_now = len(_log_rows)
                print(f'\r     Fz={fz:+.2f} N   LC={lc:.2f} N   rows={rows_now}   ',
                      end='', flush=True)
                time.sleep(0.2)

        disp_thread = threading.Thread(target=_live_display, daemon=True)
        disp_thread.start()

        input('')   # blocks until Enter

        _display_stop.set()
        disp_thread.join(timeout=0.5)
        print()     # newline after the \r display

    except (EOFError, KeyboardInterrupt):
        stop_log.set()
        log_thread.join(timeout=1.0)
        raise

    stop_log.set()
    log_thread.join(timeout=1.0)

    _log_row(pt, depth_mm, 'release')
    _print_forces('(before retract)')

    # ── Retract ───────────────────────────────────────────────────────────────
    print('     Retracting ...')
    rtde_c.moveL(surface, VEL_PRESS, ACCEL)
    _log_row(pt, depth_mm, 'retract')

    print(f'     Done — total rows: {len(_log_rows)}')


# ── Helpers ────────────────────────────────────────────────────────────────────

def print_map():
    print()
    print('  Sensor layout (top view):')
    print('        1   2   3')
    print('       4   5   6   7')
    print('      8   9  10  11  12')
    print('       13  14  15  16')
    print('        17  18  19')
    print()

def ask_mode():
    print()
    print('  ┌─────────────────────────────────────────────────┐')
    print('  │  Press / release mode                           │')
    print('  │                                                 │')
    print('  │  A — AUTO    press → hold N seconds → retract  │')
    print('  │  M — MANUAL  press on Enter, release on Enter  │')
    print('  └─────────────────────────────────────────────────┘')
    while True:
        try:
            ans = input('  Choose mode [A/M] > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            sys.exit(0)
        if ans in ('a', 'auto'):
            return 'auto'
        if ans in ('m', 'manual'):
            return 'manual'
        print('  Please type A or M')


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Force indentation logger — 19-point star-nose sensor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--tip',  default='short_6mm',
                   help='Calibration tip profile (default: short_6mm)')
    p.add_argument('--hold', type=float, default=2.0,
                   help='Hold time in AUTO mode, seconds (default: 2.0)')
    p.add_argument('--rate', type=int,   default=100,
                   help='Logging rate in Hz (default: 100)')
    return p.parse_args()


def main():
    args = parse_args()

    print('=' * 60)
    print('  Force Indentation Logger — Star-Nose Sensor')
    print('=' * 60)
    print(f'  Tip / calibration : {args.tip}')
    print(f'  Log rate          : {args.rate} Hz')
    print(f'  Robot             : {ROBOT_IP}')
    print('=' * 60)

    load_calibration(args.tip)

    # ── Choose mode ────────────────────────────────────────────────────────────
    mode = ask_mode()
    if mode == 'auto':
        print(f'\n  AUTO mode — hold time: {args.hold} s')
    else:
        print('\n  MANUAL mode — you control press and release')

    # ── Connect ────────────────────────────────────────────────────────────────
    print('\n[robot] Connecting ...')
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    rtde_c = rtde_control.RTDEControlInterface(
        ROBOT_IP, frequency=500.0,
        flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
    print('[robot] Connected')

    ft_thread = threading.Thread(target=_ft_reader, args=(rtde_r,), daemon=True)
    ft_thread.start()
    time.sleep(0.2)

    print('[robot] Moving to home ...')
    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
    print('[robot] At home')

    print_map()
    if mode == 'auto':
        print(f'  Commands:  1–19  →  travel + press + hold {args.hold:.0f}s + retract')
    else:
        print('  Commands:  1–19  →  travel to point, then Enter to press / Enter to release')
    print('             h     →  return to home')
    print('             m     →  switch mode (auto ↔ manual)')
    print('             s     →  save CSV and continue')
    print('             q     →  save CSV and quit')
    print()

    current_depth = None

    try:
        while True:
            try:
                tag = 'AUTO' if mode == 'auto' else 'MANUAL'
                cmd = input(f'  [{tag}] Point (1-19 / h / m / s / q) > ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd in ('q', 'quit', 'exit'):
                break

            if cmd in ('h', 'home'):
                print('[robot] Moving to home ...')
                rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
                print('[robot] At home')
                continue

            if cmd in ('s', 'save'):
                save_log()
                continue

            if cmd == 'm':
                mode = 'manual' if mode == 'auto' else 'auto'
                print(f'  Switched to {mode.upper()} mode')
                continue

            try:
                pt = int(cmd)
            except ValueError:
                print(f'  Unknown command "{cmd}" — type 1-19, h, m, s, or q')
                continue
            if pt not in POINTS:
                print(f'  Point must be 1–19 (got {pt})')
                continue

            # ── Depth ─────────────────────────────────────────────────────────
            depth_prompt = (f'  Depth in mm [{current_depth:.1f}] > '
                            if current_depth else '  Depth in mm > ')
            try:
                d_str = input(depth_prompt).strip()
            except (EOFError, KeyboardInterrupt):
                break

            if d_str == '' and current_depth is not None:
                depth_mm = current_depth
            else:
                try:
                    depth_mm = float(d_str)
                    if depth_mm <= 0:
                        raise ValueError
                except ValueError:
                    print('  Enter a positive number (mm)')
                    continue
            current_depth = depth_mm

            # ── Execute ───────────────────────────────────────────────────────
            try:
                if mode == 'auto':
                    indent_auto(rtde_c, pt, depth_mm, args.hold, args.rate)
                else:
                    indent_manual(rtde_c, pt, depth_mm, args.rate)
            except KeyboardInterrupt:
                print('\n  Interrupted — moving to home ...')
                try:
                    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
                except Exception:
                    pass
            except Exception as e:
                print(f'  [error] {e}')
                print('  Moving to home for safety ...')
                try:
                    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
                except Exception:
                    pass

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
        save_log()
        print('[robot] Done')


if __name__ == '__main__':
    main()
