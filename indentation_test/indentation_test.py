"""
indentation_test.py — single-point indentation characterisation
================================================================
A STANDALONE indentation rig (NOT tied to the 19-point star-nose grid). You pick
ONE point by jogging the robot, then press straight down in a continuous ramp and
record every instrument so you can plot indentation-vs-force and
indentation-vs-impedance curves.

Workflow (interactive stages, in order)
---------------------------------------
  1. POSITION (X/Y jog)  — keyboard-jog the TCP in the base X/Y plane, at a safe
     height above the sample, until the tip is over the spot you want to indent.
  2. SURFACE (Z jog)     — keyboard-jog Z down until the tip just touches the
     surface. That contact pose is indentation = 0 mm, and it is coincident with
     the shared REFERENCE_POSE / UR control frame (same convention as the rest of
     the project: a commanded depth is the true indentation depth).
  3. SAVE / LOAD calib   — the chosen point (full TCP pose at surface contact) is
     saved to a SESSION calibration file (calib_session_<name>.json). On later
     runs you load that file to reuse the SAME X/Y point and only re-jog Z (the
     height), which is the normal between-experiments adjustment.
  4. RANGE               — set the indentation range: the maximum depth (mm) the
     ramp presses to (e.g. 1, 2, 3 …).
  5. ITERATIONS          — how many ramp cycles to repeat at the point.
  6. COLLECT             — for each iteration: press 0→max (continuous ramp), hold,
     retract max→0, logging continuously. Memory is flat (streaming CSV).

Data logged (one streaming CSV row per sample, ~LOG_RATE_HZ)
------------------------------------------------------------
  • Robot        : TCP pose (x,y,z,rx,ry,rz) + TCP wrench (fx..tz) via RTDE
  • FUTEK        : AI0 voltage → force (N), direct calibration (2026-07-23)
  • LCR-6100     : the two measured fields (impedance |Z| + phase θ in Z-θ mode,
                   or Cp + Rp in Cp-Rp mode — detected/chosen at startup)
  • XJC F/T      : Fx,Fy,Fz (N) + Mx,My,Mz (N·m), 6-axis sensor over RS485 (poll)
  • Derived      : cmd_depth_mm (commanded) and indentation_mm (actual, from TCP z)

Results
-------
At the end (unless --no-plot) it draws indentation-vs-force,
indentation-vs-|Z| and indentation-vs-phase, saved to plots/. Re-draw any time:
  python3 analyze_indentation.py logs/indentation_<timestamp>.csv

Usage
-----
  python3 indentation_test.py
  python3 indentation_test.py --max-depth 3 --iterations 5
  python3 indentation_test.py --no-xjc            # robot + FUTEK + LCR only
  python3 indentation_test.py --no-lcr --no-xjc   # robot + FUTEK only
  UR_ROBOT_IP=127.0.0.1 python3 indentation_test.py
"""

import os
import sys
import csv
import json
import glob
import time
import threading
import argparse
from datetime import datetime

import rtde_control
import rtde_receive

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE   = os.path.dirname(os.path.abspath(__file__))
_CAPMEAS = os.path.normpath(os.path.join(_HERE, '..', 'Capacitance_measurement'))
LOG_DIR  = os.path.join(_HERE, 'logs')
PLOT_DIR = os.path.join(_HERE, 'plots')
sys.path.insert(0, _HERE)       # local xjc_sensor
sys.path.insert(0, _CAPMEAS)    # shared lcr6100 driver

from xjc_sensor import (                             # noqa: E402  local, reconstructed
    XjcSensor, build_request, parse_poll_frame, POLL_REPLY_LEN)

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_IP   = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
VEL_TRAVEL = 0.05     # m/s — travel / lift
VEL_PRESS  = 0.004    # m/s — slow press / retract ramp
VEL_JOG    = 0.03     # m/s — per-keystroke jog move
ACCEL      = 0.3      # m/s^2
SAFE_CLEAR_MM = 20.0  # mm above the surface for the safe/home height

# Shared start pose — identical to Integration_2/calibrate_ur5.py and
# Interdome_touch/main.py. Keep these in sync. The Z here is the "initial
# position" the jog starts from so it stays coincident with UR control.
REFERENCE_POSE = [
    -0.03664,
    -0.49831,
     0.06071,
    2.346, -2.094, -0.00009,
]

# ── FUTEK load cell calibration (direct AI0 V → force N, confirmed 2026-07-23) ──
# Source: force_sensor_calibration/Matlab calibration/step1_loadcell_calibration.json
#   F_futek_N = FUTEK_SLOPE_N_PER_V * ai0 + FUTEK_OFFSET_N   (R^2 = 0.999964)
FUTEK_SLOPE_N_PER_V = 4.113951054770791
FUTEK_OFFSET_N      = -19.28418747084478

def ai0_to_futek_n(ai0_v):
    return FUTEK_SLOPE_N_PER_V * float(ai0_v) + FUTEK_OFFSET_N

# ── LCR-6100 function codes (SCPI :FUNC:IMP <code>) ─────────────────────────────
# field0, field1 of the FETC? reply depend on the meter's measurement function.
LCR_FUNCTIONS = {
    'ZTD':  ('lcr_Z_ohm',  'lcr_phase_deg', '|Z| (Ohm)', 'phase theta (deg)'),
    'ZTR':  ('lcr_Z_ohm',  'lcr_phase_rad', '|Z| (Ohm)', 'phase theta (rad)'),
    'CPRP': ('lcr_Cp_F',   'lcr_Rp_ohm',    'Cp (F)',    'Rp (Ohm)'),
    'CSRS': ('lcr_Cs_F',   'lcr_Rs_ohm',    'Cs (F)',    'Rs (Ohm)'),
    'RX':   ('lcr_R_ohm',  'lcr_X_ohm',     'R (Ohm)',   'X (Ohm)'),
}
DEFAULT_LCR_LABELS = ('lcr_field0', 'lcr_field1', 'LCR field 0', 'LCR field 1')


# ── Shared robot state (background RTDE thread) ────────────────────────────────
_state      = {'ft': [0.0] * 6, 'tcp': list(REFERENCE_POSE), 'ai0': 0.0}
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


# ── Shared XJC state (background poll thread) ──────────────────────────────────
_xjc        = {'raw': [0.0] * 6, 'values': [0.0] * 6, 'ok': False}
_xjc_lock   = threading.Lock()
_xjc_stop   = threading.Event()

def _xjc_reader(sensor):
    while not _xjc_stop.is_set():
        try:
            raw = sensor.poll_once()
            if raw is None:
                with _xjc_lock:
                    _xjc['ok'] = False
                continue
            off  = sensor.offset if len(sensor.offset) == 6 else [0.0] * 6
            vals = [r - o for r, o in zip(raw, off)]
            with _xjc_lock:
                _xjc['raw'] = raw
                _xjc['values'] = vals
                _xjc['ok'] = True
        except Exception:
            with _xjc_lock:
                _xjc['ok'] = False
            time.sleep(0.01)

def get_xjc_state():
    with _xjc_lock:
        return {'raw': list(_xjc['raw']), 'values': list(_xjc['values']), 'ok': _xjc['ok']}


# ── Collection workflow state (stamped onto every logged row) ──────────────────
_wf      = {'iteration': -1, 'phase': 'idle', 'cmd_depth_mm': 0.0}
_wf_lock = threading.Lock()

def set_wf(**kw):
    with _wf_lock:
        _wf.update(kw)

def get_wf():
    with _wf_lock:
        return dict(_wf)


# ── LCR helpers (function query / switch via the shared driver's serial port) ──
def lcr_query_function(lcr):
    """Best-effort read of the meter's measurement function (:FUNC:IMP?).
    Returns the raw reply (e.g. 'CPRP', 'ZTD') upper-cased, or None."""
    try:
        with lcr._ser_lock:
            lcr._ser.reset_input_buffer()
            lcr._ser.write(b':FUNC:IMP?\r\n')
            line = lcr._ser.readline().decode('ascii', errors='ignore').strip()
        return line.upper() or None
    except Exception:
        return None

def lcr_set_function(lcr, code):
    """Best-effort switch of the meter function. Returns the verified code or None."""
    try:
        with lcr._ser_lock:
            lcr._ser.reset_input_buffer()
            lcr._ser.write(f':FUNC:IMP {code}\r\n'.encode('ascii'))
            time.sleep(0.3)
            lcr._ser.reset_input_buffer()
            lcr._ser.write(b':FUNC:IMP?\r\n')
            line = lcr._ser.readline().decode('ascii', errors='ignore').strip()
        return (line.upper() or code).upper()
    except Exception:
        return None


# ── Streaming CSV logger (flat RAM, crash-safe) ────────────────────────────────
BASE_FIELDS = [
    'timestamp', 'datetime', 'iteration', 'phase', 'cmd_depth_mm', 'indentation_mm',
    'tcp_x', 'tcp_y', 'tcp_z', 'tcp_rx', 'tcp_ry', 'tcp_rz',
    'ur_fx', 'ur_fy', 'ur_fz', 'ur_tx', 'ur_ty', 'ur_tz',
    'ai0', 'futek_force_N',
    'lcr_ok', 'lcr_primary', 'lcr_secondary',
    'xjc_ok', 'xjc_Fx', 'xjc_Fy', 'xjc_Fz', 'xjc_Mx', 'xjc_My', 'xjc_Mz',
]

_log_lock        = threading.Lock()
_log_file        = None
_log_writer      = None
_log_count       = 0
_log_since_flush = 0
_LOG_FLUSH_EVERY = 400

_SURFACE_Z = None   # TCP z (m) at indentation 0 — set before collection

def open_dataset(path):
    global _log_file, _log_writer, _log_count, _log_since_flush
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with _log_lock:
        _log_file   = open(path, 'w', newline='')
        _log_writer = csv.DictWriter(_log_file, fieldnames=BASE_FIELDS)
        _log_writer.writeheader()
        _log_file.flush()
        _log_count = 0
        _log_since_flush = 0
    return path

def _log_row(lcr):
    global _log_count, _log_since_flush
    st  = get_robot_state()
    ft, tcp, ai0 = st['ft'], st['tcp'], st['ai0']
    wf  = get_wf()
    xj  = get_xjc_state()

    if lcr is not None:
        f0, f1, lcr_ok = lcr.get_latest()
    else:
        f0, f1, lcr_ok = float('nan'), float('nan'), False

    indentation = (_SURFACE_Z - tcp[2]) * 1000.0 if _SURFACE_Z is not None else float('nan')

    row = {
        'timestamp':      round(time.time(), 4),
        'datetime':       datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'iteration':      wf['iteration'],
        'phase':          wf['phase'],
        'cmd_depth_mm':   round(wf['cmd_depth_mm'], 4),
        'indentation_mm': round(indentation, 4),
        'tcp_x': round(tcp[0], 6), 'tcp_y': round(tcp[1], 6), 'tcp_z': round(tcp[2], 6),
        'tcp_rx': round(tcp[3], 6), 'tcp_ry': round(tcp[4], 6), 'tcp_rz': round(tcp[5], 6),
        'ur_fx': round(ft[0], 4), 'ur_fy': round(ft[1], 4), 'ur_fz': round(ft[2], 4),
        'ur_tx': round(ft[3], 4), 'ur_ty': round(ft[4], 4), 'ur_tz': round(ft[5], 4),
        'ai0': round(ai0, 5),
        'futek_force_N': round(ai0_to_futek_n(ai0), 4),
        'lcr_ok':   int(bool(lcr_ok)),
        'lcr_primary':   f0,
        'lcr_secondary': f1,
        'xjc_ok':   int(xj['ok']),
        'xjc_Fx': round(xj['values'][0], 5), 'xjc_Fy': round(xj['values'][1], 5),
        'xjc_Fz': round(xj['values'][2], 5), 'xjc_Mx': round(xj['values'][3], 6),
        'xjc_My': round(xj['values'][4], 6), 'xjc_Mz': round(xj['values'][5], 6),
    }
    with _log_lock:
        if _log_writer is None:
            return
        _log_writer.writerow(row)
        _log_count += 1
        _log_since_flush += 1
        if _log_since_flush >= _LOG_FLUSH_EVERY:
            _log_file.flush()
            os.fsync(_log_file.fileno())
            _log_since_flush = 0

def close_dataset():
    global _log_file, _log_writer
    with _log_lock:
        if _log_file is not None:
            try:
                _log_file.flush()
                os.fsync(_log_file.fileno())
            finally:
                _log_file.close()
            _log_file = _log_writer = None
    return _log_count

_log_stop = threading.Event()

def _logger_loop(lcr, rate_hz):
    interval = 1.0 / rate_hz
    while not _log_stop.is_set():
        t0 = time.perf_counter()
        try:
            _log_row(lcr)
        except Exception as e:
            print(f'[logger] {e}')
        rem = interval - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)


# ── Pose builders ───────────────────────────────────────────────────────────────
def _pose_at_depth(surface_pose, depth_mm):
    """TCP pose pressed `depth_mm` below the surface-contact pose (down = -z)."""
    p = list(surface_pose)
    p[2] -= depth_mm / 1000.0
    return p

def _safe_pose(surface_pose):
    return _pose_at_depth(surface_pose, -SAFE_CLEAR_MM)   # lift above the surface


# ── Keyboard jog ────────────────────────────────────────────────────────────────
def _getch():
    """Read a single keypress (returns arrow keys as '\\x1b[A' etc.)."""
    import termios, tty
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def _jog(rtde_c, pose, mode):
    """Interactive jog loop. mode='xy' jogs base X/Y; mode='z' jogs Z.

    Returns the updated pose when the operator confirms (Enter), or None if they
    abort (q). Each keystroke issues a small relative moveL.
    """
    steps = [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0]   # mm
    si = 4 if mode == 'xy' else 2                   # default 1.0 / 0.25 mm
    pose = list(pose)

    if mode == 'xy':
        header = ('  X/Y JOG — position the tip over the target point\n'
                  '    a / d   : -X / +X      (or Left / Right arrow)\n'
                  '    w / s   : +Y / -Y      (or Up / Down arrow)\n'
                  '    - / +   : smaller / bigger step\n'
                  '    ENTER   : confirm X/Y and continue      q : abort')
        keymap = {'a': (0, -1), 'd': (0, +1), 's': (1, -1), 'w': (1, +1),
                  '\x1b[D': (0, -1), '\x1b[C': (0, +1), '\x1b[B': (1, -1), '\x1b[A': (1, +1)}
    else:
        header = ('  Z JOG — lower the tip until it just touches the surface\n'
                  '    s / DOWN : move DOWN (into surface, -Z)\n'
                  '    w / UP   : move UP   (away from surface, +Z)\n'
                  '    - / +    : smaller / bigger step\n'
                  '    ENTER    : set this as surface contact (0 mm)   q : abort')
        keymap = {'s': (2, -1), 'w': (2, +1), '\x1b[B': (2, -1), '\x1b[A': (2, +1)}

    print('\n' + '=' * 70)
    print(header)
    print('=' * 70)

    while True:
        st = get_robot_state()['tcp']
        print(f'\r  step={steps[si]:5.2f} mm   TCP x={st[0]*1000:+8.2f} '
              f'y={st[1]*1000:+8.2f} z={st[2]*1000:+8.2f} mm   ', end='', flush=True)
        ch = _getch()
        if ch in ('\r', '\n'):
            print()
            return pose
        if ch in ('q', 'Q', '\x03'):   # q or Ctrl-C
            print('\n  [jog] aborted')
            return None
        if ch in ('+', '='):
            si = min(si + 1, len(steps) - 1); continue
        if ch in ('-', '_'):
            si = max(si - 1, 0); continue
        if ch not in keymap:
            continue
        axis, sign = keymap[ch]
        pose[axis] += sign * steps[si] / 1000.0
        try:
            rtde_c.moveL(pose, VEL_JOG, ACCEL)
        except Exception as e:
            print(f'\n  [jog] move rejected ({e}) — undoing')
            pose[axis] -= sign * steps[si] / 1000.0


# ── Session calibration file ───────────────────────────────────────────────────
def list_session_calibs():
    out = []
    for p in sorted(glob.glob(os.path.join(_HERE, 'calib_session_*.json'))):
        name = os.path.basename(p)[len('calib_session_'):-len('.json')]
        out.append((name, p))
    return out

def save_session_calib(name, surface_pose, notes=''):
    path = os.path.join(_HERE, f'calib_session_{name}.json')
    data = {
        'name': name,
        'created': datetime.now().isoformat(timespec='seconds'),
        'reference_pose': list(REFERENCE_POSE),
        'surface_pose': [round(v, 6) for v in surface_pose],
        'notes': notes,
    }
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f'  [calib] Saved session calibration → {os.path.basename(path)}')
    return path

def load_session_calib(path):
    with open(path) as f:
        d = json.load(f)
    return list(d['surface_pose']), d

def calibrate(rtde_c, args):
    """Stages 1-3: get the surface-contact pose, by loading a session calib and
    re-jogging Z, or by a full X/Y + Z jog. Returns (surface_pose, meta)."""
    sessions = list_session_calibs()

    use_existing = False
    chosen_path  = None
    if sessions and not args.new_point:
        print('\n  Existing session calibrations:')
        for i, (name, p) in enumerate(sessions):
            try:
                sp = json.load(open(p))['surface_pose']
                z  = sp[2] * 1000.0
            except Exception:
                z = float('nan')
            print(f'    [{i}]  {name:28s}  surface z={z:+.2f} mm')
        print(f'    [{len(sessions)}]  NEW point (full X/Y + Z jog)')
        while True:
            try:
                raw = input(f'\n  Select calibration [0-{len(sessions)}] > ').strip()
                idx = int(raw)
            except (ValueError, EOFError, KeyboardInterrupt):
                continue
            if 0 <= idx <= len(sessions):
                break
        if idx < len(sessions):
            use_existing = True
            chosen_path  = sessions[idx][1]

    if use_existing:
        surface_pose, meta = load_session_calib(chosen_path)
        print(f'  [calib] Loaded "{meta["name"]}"  '
              f'XY=({surface_pose[0]*1000:+.2f},{surface_pose[1]*1000:+.2f}) mm  '
              f'surface z={surface_pose[2]*1000:+.2f} mm')
        # Move to the saved XY at a safe height, then re-jog ONLY Z.
        print('  [calib] Moving to the saved X/Y at safe height ...')
        rtde_c.moveL(_safe_pose(surface_pose), VEL_TRAVEL, ACCEL)
        rtde_c.moveL(_pose_at_depth(surface_pose, 0.0), VEL_TRAVEL, ACCEL)
        ans = input('\n  Re-jog Z (adjust the height) before this experiment? [Y/n] > ').strip().lower()
        if ans in ('', 'y', 'yes'):
            new_pose = _jog(rtde_c, _pose_at_depth(surface_pose, 0.0), 'z')
            if new_pose is None:
                raise SystemExit('[calib] aborted during Z jog')
            surface_pose = new_pose
            if input('  Save the adjusted Z back to this calibration? [y/N] > ').strip().lower() == 'y':
                save_session_calib(meta['name'], surface_pose, meta.get('notes', ''))
        return surface_pose, {'source': os.path.basename(chosen_path), 'name': meta['name'],
                              'reused_point': True}

    # ── New point: full X/Y then Z jog from the reference pose ────────────────
    print('\n  [calib] New point — starting from the reference pose.')
    start = list(REFERENCE_POSE)
    start[2] += SAFE_CLEAR_MM / 1000.0      # start a bit above the reference Z
    rtde_c.moveL(start, VEL_TRAVEL, ACCEL)

    xy_pose = _jog(rtde_c, start, 'xy')
    if xy_pose is None:
        raise SystemExit('[calib] aborted during X/Y jog')
    surface_pose = _jog(rtde_c, xy_pose, 'z')
    if surface_pose is None:
        raise SystemExit('[calib] aborted during Z jog')

    name = (args.name or input('\n  Name this point/session (e.g. ecoflex_dome_a) > ').strip()
            or datetime.now().strftime('point_%Y%m%d_%H%M%S'))
    name = ''.join(c if (c.isalnum() or c in '-_') else '_' for c in name)
    notes = input('  Notes (optional) > ').strip()
    save_session_calib(name, surface_pose, notes)
    return surface_pose, {'source': f'calib_session_{name}.json', 'name': name,
                          'reused_point': False}


# ── Continuous-ramp indentation ─────────────────────────────────────────────────
def run_ramp(rtde_c, surface_pose, max_depth_mm, n_iter, step_mm,
             hold_s, recover_s):
    """Press 0→max in `step_mm` micro-moves (slow), hold, retract max→0, logging
    continuously via the background logger. Repeated `n_iter` times."""
    n_steps = max(1, int(round(max_depth_mm / step_mm)))
    depths  = [max_depth_mm * k / n_steps for k in range(n_steps + 1)]

    # Make sure we start exactly at the surface (indentation 0).
    set_wf(phase='approach', cmd_depth_mm=0.0)
    rtde_c.moveL(_pose_at_depth(surface_pose, 0.0), VEL_TRAVEL, ACCEL)

    for it in range(n_iter):
        print('-' * 70)
        print(f'  Iteration {it + 1}/{n_iter}  —  ramp 0 → {max_depth_mm:.2f} mm '
              f'({n_steps} steps of {step_mm:.3f} mm)')

        # press (ramp down)
        set_wf(iteration=it, phase='press')
        for d in depths:
            set_wf(cmd_depth_mm=d)
            rtde_c.moveL(_pose_at_depth(surface_pose, d), VEL_PRESS, ACCEL)

        # hold at max
        set_wf(phase='hold', cmd_depth_mm=max_depth_mm)
        st = get_robot_state()
        xj = get_xjc_state()
        print(f'     [hold] FUTEK={ai0_to_futek_n(st["ai0"]):+.2f} N   '
              f'UR Fz={st["ft"][2]:+.2f} N   XJC Fz={xj["values"][2]:+.2f} N')
        time.sleep(hold_s)

        # retract (ramp up)
        set_wf(phase='retract')
        for d in reversed(depths):
            set_wf(cmd_depth_mm=d)
            rtde_c.moveL(_pose_at_depth(surface_pose, d), VEL_PRESS, ACCEL)

        # recover at the surface
        set_wf(phase='recover', cmd_depth_mm=0.0)
        time.sleep(recover_s)

        with _log_lock:
            print(f'     rows so far: {_log_count}')

    set_wf(phase='idle')


# ── Simulation doubles (--sim: no robot, no instruments) ───────────────────────
# Synthesize a plausible press so the full collect → CSV → plot pipeline can be
# exercised with zero hardware. Force grows with depth (F = 2d + 0.3d²); |Z|
# falls and phase shifts linearly with depth; XJC Fz ≈ 0.98·F.
import random as _rnd

def _sim_force_n(depth_mm):
    return 2.0 * depth_mm + 0.3 * depth_mm * depth_mm

class _SimController:
    """Stand-in for rtde_control: moveL just updates the shared robot state with
    a synthetic wrench/AI0 derived from the commanded indentation depth."""
    def __init__(self, surface_z):
        self._sz = surface_z
    def moveL(self, pose, vel=0.0, accel=0.0):
        depth = (self._sz - pose[2]) * 1000.0
        F = _sim_force_n(max(0.0, depth)) + _rnd.uniform(-0.03, 0.03)
        ai0 = (F - FUTEK_OFFSET_N) / FUTEK_SLOPE_N_PER_V
        with _state_lock:
            _state['tcp'] = list(pose)
            _state['ft']  = [0.0, 0.0, -F, 0.0, 0.0, 0.0]
            _state['ai0'] = ai0
        time.sleep(0.003)   # mimic a short blocking move so the logger samples it
    def stopScript(self):
        pass

class _SimLCR:
    """Stand-in for LCR6100.get_latest(): |Z|/phase from the current depth."""
    def __init__(self, surface_z):
        self._sz = surface_z
    def get_latest(self):
        with _state_lock:
            z = _state['tcp'][2]
        depth = (self._sz - z) * 1000.0
        return (1000.0 - 120.0 * depth + _rnd.uniform(-2, 2),
                -5.0 - 2.0 * depth + _rnd.uniform(-0.2, 0.2), True)
    def start_polling(self):
        pass
    def disconnect(self):
        pass

class _SimXjc:
    """Stand-in for XjcSensor: poll_once() returns a synthetic 6-axis wrench."""
    def __init__(self, surface_z):
        self._sz = surface_z
        self.offset = [0.0] * 6
    def poll_once(self):
        with _state_lock:
            z = _state['tcp'][2]
        F = _sim_force_n(max(0.0, (self._sz - z) * 1000.0))
        return [_rnd.uniform(-0.05, 0.05), _rnd.uniform(-0.05, 0.05),
                0.98 * F + _rnd.uniform(-0.02, 0.02), 0.0, 0.0, 0.0]
    def tare(self, n=30):
        return self.offset
    def close(self):
        pass


# ── Instrument setup ───────────────────────────────────────────────────────────
def _probe_xjc_port(dev, baud=115200, station=1):
    """True if `dev` answers a Modbus poll request with a valid XJC frame."""
    try:
        import serial
        with serial.Serial(dev, baud, timeout=0.3) as s:
            s.reset_input_buffer()
            s.write(build_request(station))
            buf = bytearray()
            deadline = time.perf_counter() + 0.4
            while len(buf) < POLL_REPLY_LEN and time.perf_counter() < deadline:
                chunk = s.read(POLL_REPLY_LEN - len(buf))
                if not chunk:
                    break
                buf.extend(chunk)
            return parse_poll_frame(bytes(buf), station) is not None
    except Exception:
        return False

def _probe_lcr_port(dev, baud=115200):
    """True if `dev` answers FETC? with a well-formed LCR-6100 reply (4 commas)."""
    try:
        import serial
        with serial.Serial(dev, baud, timeout=0.6) as s:
            s.dtr = False
            s.rts = False
            time.sleep(0.2)
            s.reset_input_buffer()
            s.write(b'FETC?\r\n')
            line = s.readline().decode('ascii', errors='ignore').strip()
            return line.count(',') == 4
    except Exception:
        return False

def detect_instruments(args, robot_connected):
    """Probe every serial port and classify it as the LCR or the XJC; honour
    --lcr-port/--xjc-port overrides and --no-lcr/--no-xjc skips. Prints a summary
    and returns {'lcr': dev|None, 'xjc': dev|None, 'unknown': [...], 'ports': [...]}.
    """
    try:
        from lcr6100 import list_ports
        ports = [dev for dev, _desc in list_ports()]
    except Exception:
        ports = []

    # Both instruments are USB adapters (LCR = USB-serial, XJC = USB-RS485/FTDI),
    # so by default only probe ttyUSB*/ttyACM* and skip the motherboard's legacy
    # ttyS* ports (opening each would add a timeout for no reason). --all-ports
    # forces probing everything.
    if args.all_ports:
        probe_ports = list(ports)
    else:
        probe_ports = [p for p in ports if ('USB' in p or 'ACM' in p)]

    report = {'lcr': None, 'xjc': None, 'unknown': [], 'ports': probe_ports}

    # Overrides take precedence and are not probed (the device may be busy).
    taken = set()
    if not args.no_lcr and args.lcr_port:
        report['lcr'] = args.lcr_port; taken.add(args.lcr_port)
    if not args.no_xjc and args.xjc_port:
        report['xjc'] = args.xjc_port; taken.add(args.xjc_port)

    print('\n' + '=' * 70)
    print('  INSTRUMENT DETECTION')
    print('=' * 70)
    for dev in probe_ports:
        if dev in taken:
            continue
        kind = '?'
        if report['xjc'] is None and not args.no_xjc and \
                _probe_xjc_port(dev, args.xjc_baud, args.xjc_station):
            report['xjc'] = dev; kind = 'XJC F/T'
        elif report['lcr'] is None and not args.no_lcr and _probe_lcr_port(dev):
            report['lcr'] = dev; kind = 'LCR-6100'
        else:
            report['unknown'].append(dev)
        print(f'    probed {dev:16s} → {kind}')
    if not probe_ports:
        print('    (no USB serial ports found to probe'
              + ('' if args.all_ports else '; try --all-ports') + ')')

    def _stat(name, dev, skipped):
        if skipped:
            return f'  {name:20s}: SKIPPED (flag)'
        if dev:
            return f'  {name:20s}: {dev}'
        return f'  {name:20s}: not found'

    print('  ' + '-' * 66)
    rip = ROBOT_IP if robot_connected else f'{ROBOT_IP} (NOT connected)'
    print(f'  {"Robot (UR5 RTDE)":20s}: {"CONNECTED " + rip if robot_connected else rip}')
    print(f'  {"FUTEK load cell":20s}: '
          f'{"via robot AI0" if robot_connected else "unavailable (needs robot)"}')
    print(_stat('LCR-6100', report['lcr'], args.no_lcr))
    print(_stat('XJC F/T sensor', report['xjc'], args.no_xjc))
    if report['unknown']:
        print(f'  {"Unidentified ports":20s}: {", ".join(report["unknown"])}')
    print('=' * 70)
    return report

def _select_serial_port(prompt, ports):
    """Fallback manual picker when auto-detection didn't resolve a port."""
    if not ports:
        raw = input(f'  {prompt} — no ports left, type device path (blank=skip) > ').strip()
        return raw or None
    print(f'\n  {prompt}')
    for i, dev in enumerate(ports):
        print(f'    [{i}]  {dev}')
    while True:
        raw = input(f'  Select port [0-{len(ports)-1}], a path, or blank to skip > ').strip()
        if raw == '':
            return None
        if raw.isdigit() and 0 <= int(raw) < len(ports):
            return ports[int(raw)]
        if os.path.exists(raw):
            return raw
        print('  Invalid selection')

def setup_lcr(args, port):
    """Connect the LCR, detect its function and let the operator keep it or
    switch to Z-θ. Returns (lcr_or_None, labels_tuple, function_code)."""
    if args.no_lcr:
        print('[LCR] Skipped (--no-lcr)')
        return None, DEFAULT_LCR_LABELS, 'disabled'
    from lcr6100 import LCR6100
    if port is None:
        print('[LCR] Not detected — LCR columns will be NaN')
        return None, DEFAULT_LCR_LABELS, 'disabled'
    print(f'[LCR] Using {port}')
    lcr = LCR6100(port)
    lcr.connect()

    func = lcr_query_function(lcr)
    if func and func in LCR_FUNCTIONS:
        print(f'[LCR] Meter is in "{func}" mode ({LCR_FUNCTIONS[func][2]} / {LCR_FUNCTIONS[func][3]})')
    elif func:
        print(f'[LCR] Meter reports function "{func}" (not in my known table)')
    else:
        print('[LCR] Could not read the function from the meter.')
        func = input('  Type the current function code (e.g. ZTD, CPRP) or blank if unknown > ').strip().upper() or None

    ans = input('  Switch the meter to Z-theta (impedance + phase)? [y/N] > ').strip().lower()
    if ans == 'y':
        newf = lcr_set_function(lcr, 'ZTD')
        if newf:
            func = newf
            print(f'[LCR] Function now "{func}"')
        else:
            print('[LCR] Switch command sent but could not verify — check the front panel.')

    labels = LCR_FUNCTIONS.get(func, DEFAULT_LCR_LABELS)
    print(f'[LCR] Logging field0 → {labels[2]}   field1 → {labels[3]}')
    lcr.start_polling()
    time.sleep(0.5)
    return lcr, labels, (func or 'unknown')

def setup_xjc(args, port):
    """Open the XJC F/T sensor in poll mode and optionally tare it."""
    if args.no_xjc:
        print('[XJC] Skipped (--no-xjc)')
        return None
    if port is None:
        print('[XJC] Not detected — XJC columns will be 0.0')
        return None
    print(f'[XJC] Using {port}')
    sensor = XjcSensor(port=port, baud=args.xjc_baud, station=args.xjc_station)
    try:
        sensor.open()
    except Exception as e:
        print(f'[XJC] Could not open {port}: {e} — XJC columns will be 0.0')
        return None
    raw = sensor.poll_once()
    if raw is None:
        print('[XJC] WARNING: no valid frame on first poll — check wiring/baud/station.')
    else:
        print(f'[XJC] First poll OK: Fx,Fy,Fz = {[round(v,3) for v in raw[:3]]} N')
    if input('  Tare the XJC now (unloaded)? [Y/n] > ').strip().lower() in ('', 'y', 'yes'):
        off = sensor.tare(n=30)
        print(f'[XJC] Tared (offset Fz={off[2]:+.3f} N)')
    return sensor


# ── CLI ──────────────────────────────────────────────────────────────────────
def _ask_float(prompt, default, lo, hi):
    while True:
        try:
            raw = input(prompt).strip()
            if raw == '':
                return default
            v = float(raw)
            if lo <= v <= hi:
                return v
            print(f'  Must be between {lo} and {hi}')
        except (ValueError, EOFError, KeyboardInterrupt):
            return default

def _ask_int(prompt, default, lo, hi):
    while True:
        try:
            raw = input(prompt).strip()
            if raw == '':
                return default
            v = int(raw)
            if lo <= v <= hi:
                return v
            print(f'  Must be between {lo} and {hi}')
        except (ValueError, EOFError, KeyboardInterrupt):
            return default

def parse_args():
    p = argparse.ArgumentParser(
        description='Single-point indentation test — robot + FUTEK + LCR + XJC',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('--max-depth', type=float, default=None,
                   help='Indentation range: max depth in mm (default: ask, 3.0)')
    p.add_argument('--iterations', type=int, default=None,
                   help='Ramp cycles at the point (default: ask, 3)')
    p.add_argument('--step', type=float, default=0.05,
                   help='Ramp micro-step in mm (default: 0.05)')
    p.add_argument('--hold', type=float, default=1.0,
                   help='Hold at max depth, seconds (default: 1.0)')
    p.add_argument('--recover', type=float, default=1.0,
                   help='Dwell at the surface between iterations, seconds (default: 1.0)')
    p.add_argument('--rate', type=int, default=60,
                   help='Logging rate in Hz (default: 60)')
    p.add_argument('--name', default=None, help='Session/point name for a NEW calibration')
    p.add_argument('--new-point', action='store_true',
                   help='Force a full X/Y + Z jog even if session calibs exist')
    p.add_argument('--lcr-port', default=None, help='LCR serial device (skip the prompt)')
    p.add_argument('--xjc-port', default=None, help='XJC serial device (skip the prompt)')
    p.add_argument('--xjc-baud', type=int, default=115200, help='XJC baud (default 115200)')
    p.add_argument('--xjc-station', type=int, default=1, help='XJC Modbus station (default 1)')
    p.add_argument('--no-lcr', action='store_true', help='Do not use the LCR meter')
    p.add_argument('--no-xjc', action='store_true', help='Do not use the XJC sensor')
    p.add_argument('--no-plot', action='store_true', help='Do not plot at the end')
    p.add_argument('--sim', action='store_true',
                   help='Hardware-free dry run: no robot/instruments, synthesize a '
                        'press to exercise the collect → CSV → plot pipeline')
    p.add_argument('--detect', action='store_true',
                   help='Only probe which instruments are connected (robot + serial '
                        'LCR/XJC), print the report and exit — no motion, no logging')
    p.add_argument('--all-ports', action='store_true',
                   help='Probe every serial port, including legacy ttyS* (default: '
                        'only ttyUSB*/ttyACM*, where the USB adapters appear)')
    return p.parse_args()


# ── Simulation run (no hardware, no prompts) ───────────────────────────────────
def _run_sim(rtde_c, args):
    """Full collect → CSV → plot pipeline against synthetic signals."""
    surface_pose = list(REFERENCE_POSE)
    max_depth = args.max_depth if args.max_depth is not None else 3.0
    n_iter    = args.iterations if args.iterations is not None else 2
    step_mm   = max(0.01, args.step)

    lcr    = None if args.no_lcr else _SimLCR(_SURFACE_Z)
    labels = LCR_FUNCTIONS['ZTD']
    xjc    = None if args.no_xjc else _SimXjc(_SURFACE_Z)
    if xjc is not None:
        threading.Thread(target=_xjc_reader, args=(xjc,), daemon=True).start()

    print(f'[sim] max_depth={max_depth} mm  iterations={n_iter}  step={step_mm} mm  '
          f'rate={args.rate} Hz')

    os.makedirs(LOG_DIR, exist_ok=True)
    ts        = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path  = os.path.join(LOG_DIR, f'indentation_sim_{ts}.csv')
    meta_path = os.path.join(LOG_DIR, f'indentation_sim_{ts}_meta.json')
    meta = {
        'timestamp': ts, 'csv_file': os.path.basename(csv_path), 'simulated': True,
        'point': {'name': 'sim', 'source': 'sim', 'reused_point': False},
        'surface_pose': [round(v, 6) for v in surface_pose],
        'max_depth_mm': max_depth, 'iterations': n_iter, 'step_mm': step_mm,
        'hold_s': args.hold, 'recover_s': args.recover, 'rate_hz': args.rate,
        'lcr': {'function': 'ZTD(sim)', 'enabled': lcr is not None,
                'primary_col': 'lcr_primary', 'secondary_col': 'lcr_secondary',
                'primary_name': labels[0], 'secondary_name': labels[1],
                'primary_label': labels[2], 'secondary_label': labels[3]},
        'xjc': {'enabled': xjc is not None, 'tare_offset': ([0.0] * 6 if xjc else None)},
    }
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)

    open_dataset(csv_path)
    _log_stop.clear()
    threading.Thread(target=_logger_loop, args=(lcr, args.rate), daemon=True).start()
    print(f'[sim] Streaming → {csv_path}  ({args.rate} Hz)\n')

    start = time.time()
    try:
        run_ramp(rtde_c, surface_pose, max_depth, n_iter, step_mm, args.hold, args.recover)
    except KeyboardInterrupt:
        print('\n  Interrupted.')
    finally:
        _log_stop.set(); _xjc_stop.set()
        time.sleep(0.2)
        n_rows = close_dataset()

    print(f'\n[sim] Completed in {time.time() - start:.1f} s   rows={n_rows}')
    print(f'[sim] Dataset : {csv_path}')
    if not args.no_plot and n_rows:
        try:
            import analyze_indentation
            analyze_indentation.plot(csv_path, meta_path, show=False)
        except Exception as e:
            print(f'[plot] {e}')
    print('[sim] done')


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    global _SURFACE_Z
    args = parse_args()

    print('=' * 70)
    print('  Single-point indentation test — UR5 + FUTEK + LCR-6100 + XJC F/T')
    print('=' * 70)

    # ── Connect robot (needed for the jog) ─────────────────────────────────────
    if args.sim:
        print('\n[sim] Hardware-free dry run — no robot, no instruments.')
        _SURFACE_Z = REFERENCE_POSE[2]
        with _state_lock:
            _state['tcp'] = list(REFERENCE_POSE)
        rtde_c = _SimController(_SURFACE_Z)
        return _run_sim(rtde_c, args)

    # ── Detection-only mode: probe everything, report, exit (no motion) ────────
    if args.detect:
        robot_ok = False
        try:
            r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            r.getActualTCPPose()
            robot_ok = True
            try:
                r.disconnect()
            except Exception:
                pass
        except Exception:
            robot_ok = False
        detect_instruments(args, robot_connected=robot_ok)
        print('\n[detect] Report only — nothing was moved or logged.')
        return

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

    lcr = xjc = None
    labels = DEFAULT_LCR_LABELS
    lcr_func = 'disabled'
    try:
        # ── Stages 1-3: calibration (jog) ──────────────────────────────────────
        surface_pose, calib_meta = calibrate(rtde_c, args)
        _SURFACE_Z = surface_pose[2]

        # ── Stage 4-5: range + iterations ──────────────────────────────────────
        max_depth = args.max_depth if args.max_depth is not None else _ask_float(
            '\n  Indentation range — max depth (mm) [3.0] > ', 3.0, 0.1, 15.0)
        n_iter = args.iterations if args.iterations is not None else _ask_int(
            '  Iterations (ramp cycles) [3] > ', 3, 1, 1000)
        step_mm = max(0.01, args.step)

        # ── Instruments: auto-detect which serial devices are connected ─────────
        report = detect_instruments(args, robot_connected=True)
        lcr_port, xjc_port = report['lcr'], report['xjc']
        leftover = list(report['unknown'])
        # If a sensor wasn't identified but unknown ports remain, offer them.
        if lcr_port is None and leftover and not args.no_lcr:
            lcr_port = _select_serial_port('LCR-6100 — pick from unidentified ports', leftover)
            if lcr_port in leftover:
                leftover.remove(lcr_port)
        if xjc_port is None and leftover and not args.no_xjc:
            xjc_port = _select_serial_port('XJC F/T — pick from unidentified ports', leftover)

        lcr, labels, lcr_func = setup_lcr(args, lcr_port)
        xjc = setup_xjc(args, xjc_port)
        if xjc is not None:
            xjc_thread = threading.Thread(target=_xjc_reader, args=(xjc,), daemon=True)
            xjc_thread.start()

        # ── Summary ──────────────────────────────────────────────────────────────
        print('\n' + '=' * 70)
        print(f'  Point         : {calib_meta["name"]}  '
              f'XY=({surface_pose[0]*1000:+.2f},{surface_pose[1]*1000:+.2f}) mm  '
              f'surface z={surface_pose[2]*1000:+.2f} mm')
        print(f'  Max depth     : {max_depth:.2f} mm   step {step_mm:.3f} mm')
        print(f'  Iterations    : {n_iter}')
        print(f'  Hold / recover: {args.hold:.1f} s / {args.recover:.1f} s')
        print(f'  Log rate      : {args.rate} Hz')
        print(f'  LCR           : {lcr_func}  ({labels[2]} / {labels[3]})')
        print(f'  XJC           : {"poll" if xjc else "disabled"}')
        print('=' * 70)
        try:
            if input('\n  Press ENTER to start (Ctrl-C to abort) ... '):
                pass
        except (EOFError, KeyboardInterrupt):
            raise SystemExit('Aborted before collection.')

        # ── Output files ──────────────────────────────────────────────────────
        os.makedirs(LOG_DIR, exist_ok=True)
        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(LOG_DIR, f'indentation_{ts}.csv')
        meta_path = os.path.join(LOG_DIR, f'indentation_{ts}_meta.json')
        meta = {
            'timestamp': ts, 'csv_file': os.path.basename(csv_path),
            'point': calib_meta, 'surface_pose': [round(v, 6) for v in surface_pose],
            'reference_pose': list(REFERENCE_POSE),
            'max_depth_mm': max_depth, 'iterations': n_iter, 'step_mm': step_mm,
            'hold_s': args.hold, 'recover_s': args.recover, 'rate_hz': args.rate,
            'robot_ip': ROBOT_IP,
            'futek_calibration': {'slope_n_per_v': FUTEK_SLOPE_N_PER_V,
                                  'offset_n': FUTEK_OFFSET_N,
                                  'formula': 'F = slope*ai0 + offset'},
            'lcr': {'function': lcr_func, 'enabled': lcr is not None,
                    'primary_col': 'lcr_primary', 'secondary_col': 'lcr_secondary',
                    'primary_name': labels[0],  'secondary_name': labels[1],
                    'primary_label': labels[2], 'secondary_label': labels[3]},
            'xjc': {'enabled': xjc is not None,
                    'baud': args.xjc_baud, 'station': args.xjc_station,
                    'tare_offset': (xjc.offset if xjc else None)},
        }
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
        print(f'[meta] Wrote run metadata → {meta_path}')

        # ── Start logger + collect ──────────────────────────────────────────────
        open_dataset(csv_path)
        _log_stop.clear()
        log_thread = threading.Thread(target=_logger_loop, args=(lcr, args.rate), daemon=True)
        log_thread.start()
        print(f'[log] Streaming → {csv_path}  ({args.rate} Hz)\n')

        start = time.time()
        try:
            run_ramp(rtde_c, surface_pose, max_depth, n_iter, step_mm,
                     args.hold, args.recover)
        except KeyboardInterrupt:
            print('\n  Interrupted — stopping after the current move ...')

        _log_stop.set()
        time.sleep(0.2)
        n_rows = close_dataset()
        print(f'\n  Completed in {time.time() - start:.1f} s   rows={n_rows}')
        print(f'  Dataset : {csv_path}')
        print(f'  Metadata: {meta_path}')

        # ── Results ──────────────────────────────────────────────────────────────
        if not args.no_plot and n_rows:
            try:
                import analyze_indentation
                analyze_indentation.plot(csv_path, meta_path, show=True)
            except Exception as e:
                print(f'[plot] Could not plot automatically ({e}).')
                print(f'[plot] Run:  python3 analyze_indentation.py "{csv_path}"')

    finally:
        _log_stop.set()
        _xjc_stop.set()
        _ft_stop.set()
        print('\n[robot] Lifting to a safe height ...')
        try:
            cur = get_robot_state()['tcp']
            cur[2] += SAFE_CLEAR_MM / 1000.0      # straight up, off the surface
            rtde_c.moveL(cur, VEL_TRAVEL, ACCEL)
        except Exception:
            pass
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        if lcr is not None:
            try:
                lcr.disconnect()
            except Exception:
                pass
        if xjc is not None:
            try:
                xjc.close()
            except Exception:
                pass
        print('[done]')


if __name__ == '__main__':
    main()
