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
VEL_RELIEF = 0.01     # m/s — retract to relieve load after a force-limit abort
ACCEL      = 0.3      # m/s^2
DEFAULT_MAX_FORCE_N = 45.0   # overload cutoff (N); XJC ±50N, FUTEK 10lb≈44.5N
SAFE_CLEAR_MM = 20.0  # mm above the surface for the safe/home height
SAFE_FIX_LIFT_MM = 50.0  # mm — lift straight up after FIXING the point (safety gap)

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
        time.sleep(0.001)   # ~1 ms — keep the cache fresh for 1 kHz logging
                            # (RTDE itself tops out at 500 Hz on the UR5e)

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


def peak_abs_force():
    """Largest |Fz| seen by the real force sensors right now (N). Uses the XJC
    (when valid) and the robot's own TCP Fz; ignores the FUTEK because it
    saturates (its amplifier rails) well below the sensor-protection range."""
    st = get_robot_state()
    xj = get_xjc_state()
    f_ur  = abs(st['ft'][2])
    f_xjc = abs(xj['values'][2]) if xj['ok'] else 0.0
    return max(f_ur, f_xjc)


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

def _jog(rtde_c, pose):
    """Interactive jog loop over ALL THREE axes (base X, Y and Z) at once.

    Returns the updated pose when the operator fixes the position (Enter), or
    None if they abort (q). Each keystroke issues a small relative moveL.
    """
    steps = [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0]   # mm
    si = 3                                           # default 0.5 mm
    pose = list(pose)

    header = ('  JOG — position the tip (X, Y and Z together).\n'
              '  Watch the FUTEK + XJC Fz read-out: lower Z until the force just\n'
              '  begins to rise above ~0 — that is the surface contact point.\n'
              '    a / d   : -X / +X        (or Left / Right arrow)\n'
              '    w / s   : +Y / -Y        (or Up / Down arrow)\n'
              '    r / f   : +Z / -Z        (up / down into the surface)\n'
              '    - / +   : smaller / bigger step\n'
              '    b       : zero the force baseline here (set current force = Δ0)\n'
              '    ENTER   : fix this position as surface contact (0 mm)   q : abort')
    keymap = {
        'a': (0, -1), 'd': (0, +1), '\x1b[D': (0, -1), '\x1b[C': (0, +1),   # X  ← →
        'w': (1, +1), 's': (1, -1), '\x1b[A': (1, +1), '\x1b[B': (1, -1),   # Y  ↑ ↓
        'r': (2, +1), 'f': (2, -1), 'R': (2, +1), 'F': (2, -1),             # Z  up/down
    }

    print('\n' + '=' * 70)
    print(header)
    print('=' * 70)

    # Force baseline (in air): Δ from here makes the contact point easy to spot.
    def _forces():
        rs = get_robot_state()
        futek = ai0_to_futek_n(rs['ai0'])
        xj = get_xjc_state()
        return futek, (xj['values'][2] if xj['ok'] else float('nan')), xj['ok']
    base_fu, base_xz, _ = _forces()

    while True:
        st = get_robot_state()['tcp']
        fu, xz, xok = _forces()
        xtxt = (f'{xz:+6.2f}N (Δ{xz - base_xz:+5.2f})' if xok else '  --  (no XJC)')
        print(f'\r  step={steps[si]:4.2f}mm  z={st[2]*1000:+7.2f} '
              f'x={st[0]*1000:+7.2f} y={st[1]*1000:+7.2f}mm | '
              f'FUTEK {fu:+6.2f}N (Δ{fu - base_fu:+5.2f}) | '
              f'XJC Fz {xtxt}   ', end='', flush=True)
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
        if ch in ('b', 'B'):
            base_fu, base_xz, _ = _forces()
            print('\n  [jog] force baseline zeroed here'); continue
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

def _fix_position_lift(rtde_c, surface_pose):
    """After the point is fixed, lift straight up SAFE_FIX_LIFT_MM for safety so
    the tip is not resting on the sample during setup. The surface pose (0 mm)
    is unchanged; run_ramp re-approaches it when collection starts."""
    lift = list(surface_pose)
    lift[2] += SAFE_FIX_LIFT_MM / 1000.0
    print(f'  [calib] Position fixed — lifting +{SAFE_FIX_LIFT_MM:.0f} mm in Z for safety ...')
    try:
        rtde_c.moveL(lift, VEL_TRAVEL, ACCEL)
    except Exception as e:
        print(f'  [calib] safety lift rejected ({e})')

def calibrate(rtde_c, args):
    """Stages 1-3: get the surface-contact pose, by loading a session calib (then
    re-jogging) or by a fresh 3-axis jog. After the point is fixed the tool is
    lifted SAFE_FIX_LIFT_MM for safety. Returns (surface_pose, meta)."""
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
        ans = input('\n  Re-jog the position (X/Y/Z) before this experiment? [Y/n] > ').strip().lower()
        if ans in ('', 'y', 'yes'):
            new_pose = _jog(rtde_c, _pose_at_depth(surface_pose, 0.0))
            if new_pose is None:
                raise SystemExit('[calib] aborted during jog')
            surface_pose = new_pose
            if input('  Save the adjusted position back to this calibration? [y/N] > ').strip().lower() == 'y':
                save_session_calib(meta['name'], surface_pose, meta.get('notes', ''))
        _fix_position_lift(rtde_c, surface_pose)
        return surface_pose, {'source': os.path.basename(chosen_path), 'name': meta['name'],
                              'reused_point': True}

    # ── New point: one 3-axis jog from the reference pose ─────────────────────
    print('\n  [calib] New point — starting from the reference pose.')
    start = list(REFERENCE_POSE)
    start[2] += SAFE_CLEAR_MM / 1000.0      # start a bit above the reference Z
    rtde_c.moveL(start, VEL_TRAVEL, ACCEL)

    surface_pose = _jog(rtde_c, start)
    if surface_pose is None:
        raise SystemExit('[calib] aborted during jog')

    name = (args.name or input('\n  Name this point/session (e.g. ecoflex_dome_a) > ').strip()
            or datetime.now().strftime('point_%Y%m%d_%H%M%S'))
    name = ''.join(c if (c.isalnum() or c in '-_') else '_' for c in name)
    notes = input('  Notes (optional) > ').strip()
    save_session_calib(name, surface_pose, notes)
    _fix_position_lift(rtde_c, surface_pose)
    return surface_pose, {'source': f'calib_session_{name}.json', 'name': name,
                          'reused_point': False}


# ── Continuous-ramp indentation ─────────────────────────────────────────────────
def _ramp_cmd_depth(d0, d1, duration_s, stop_evt):
    """Linearly drive cmd_depth_mm from d0 to d1 over duration_s, to match the
    constant-velocity moveL running concurrently. Runs in a daemon thread."""
    t0 = time.perf_counter()
    while not stop_evt.is_set():
        frac = 1.0 if duration_s <= 0 else min(1.0, (time.perf_counter() - t0) / duration_s)
        set_wf(cmd_depth_mm=d0 + (d1 - d0) * frac)
        if frac >= 1.0:
            break
        time.sleep(0.002)

def _constant_velocity_move(rtde_c, target_pose, d0, d1, ramp_speed_mm_s,
                            max_force_n=0.0):
    """Move to target_pose at a constant ramp_speed while advancing cmd_depth_mm
    linearly. If max_force_n > 0, the move runs ASYNCHRONOUSLY and a watchdog
    polls the real force every ~2 ms; when |Fz| (XJC or UR) exceeds the limit it
    calls stopL() and returns True (overload). Otherwise returns False.

    Falls back to a plain blocking moveL (no mid-move cutoff) if the installed
    ur_rtde doesn't accept the asynchronous flag."""
    duration_s = abs(d1 - d0) / max(1e-6, ramp_speed_mm_s)
    speed_mps  = ramp_speed_mm_s / 1000.0
    stop_evt = threading.Event()
    th = threading.Thread(target=_ramp_cmd_depth,
                          args=(d0, d1, duration_s, stop_evt), daemon=True)
    th.start()
    overload = False
    try:
        if max_force_n and max_force_n > 0:
            rtde_c.moveL(target_pose, speed_mps, ACCEL, True)   # asynchronous
            target_z = target_pose[2]
            t_end = time.time() + duration_s * 1.5 + 3.0
            while time.time() < t_end:
                if peak_abs_force() >= max_force_n:
                    try:
                        rtde_c.stopL(2.0)
                    except Exception:
                        pass
                    overload = True
                    break
                if abs(get_robot_state()['tcp'][2] - target_z) <= 8e-5:
                    break                                       # reached target
                time.sleep(0.002)
        else:
            rtde_c.moveL(target_pose, speed_mps, ACCEL)         # blocking, no cutoff
    except TypeError:
        rtde_c.moveL(target_pose, speed_mps, ACCEL)             # old API: no async flag
    finally:
        stop_evt.set()
        th.join(0.2)
    if not overload:
        set_wf(cmd_depth_mm=d1)
    return overload

def run_ramp(rtde_c, surface_pose, max_depth_mm, n_iter, ramp_speed_mm_s,
             hold_s, recover_s, max_force_n=0.0):
    """Press 0→max and retract max→0 as SINGLE constant-velocity moves (true
    `ramp_speed_mm_s`), logged continuously by the background logger. Repeated
    `n_iter` times. If `max_force_n` > 0 the press is force-guarded: exceeding it
    aborts the press, retracts to relieve the load, and stops the run.
    Returns True if an overload abort happened."""
    pressed = _pose_at_depth(surface_pose, max_depth_mm)
    surface = _pose_at_depth(surface_pose, 0.0)
    cycle_s = 2.0 * (max_depth_mm / max(1e-6, ramp_speed_mm_s)) + hold_s + recover_s

    # Make sure we start exactly at the surface (indentation 0).
    set_wf(phase='approach', cmd_depth_mm=0.0)
    rtde_c.moveL(surface, VEL_TRAVEL, ACCEL)

    for it in range(n_iter):
        print('-' * 70)
        guard = f'force-guard {max_force_n:.0f} N' if max_force_n else 'no force guard'
        print(f'  Iteration {it + 1}/{n_iter}  —  ramp 0 → {max_depth_mm:.2f} mm '
              f'@ {ramp_speed_mm_s:.3f} mm/s  (~{cycle_s:.0f} s/cycle)  [{guard}]')

        # press (constant-velocity ramp down, force-guarded)
        set_wf(iteration=it, phase='press')
        if _constant_velocity_move(rtde_c, pressed, 0.0, max_depth_mm,
                                   ramp_speed_mm_s, max_force_n):
            f = peak_abs_force()
            print(f'\n  ⚠ [SAFETY] force limit {max_force_n:.0f} N exceeded '
                  f'(|Fz|≈{f:.1f} N) — retracting to relieve load and stopping the run.')
            set_wf(phase='retract')
            try:
                rtde_c.moveL(surface, VEL_RELIEF, ACCEL)
            except Exception:
                pass
            set_wf(phase='idle')
            return True

        # hold at max
        set_wf(phase='hold', cmd_depth_mm=max_depth_mm)
        st = get_robot_state()
        xj = get_xjc_state()
        print(f'     [hold] FUTEK={ai0_to_futek_n(st["ai0"]):+.2f} N   '
              f'UR Fz={st["ft"][2]:+.2f} N   XJC Fz={xj["values"][2]:+.2f} N')
        # guard the hold too, in case the load creeps up at depth
        if max_force_n and peak_abs_force() >= max_force_n:
            print(f'\n  ⚠ [SAFETY] force limit exceeded at hold — retracting and stopping.')
            set_wf(phase='retract')
            try:
                rtde_c.moveL(surface, VEL_RELIEF, ACCEL)
            except Exception:
                pass
            set_wf(phase='idle')
            return True
        time.sleep(hold_s)

        # retract (constant-velocity ramp up; relieving load, so no cutoff)
        set_wf(phase='retract')
        _constant_velocity_move(rtde_c, surface, max_depth_mm, 0.0, ramp_speed_mm_s)

        # recover at the surface
        set_wf(phase='recover', cmd_depth_mm=0.0)
        time.sleep(recover_s)

        with _log_lock:
            print(f'     rows so far: {_log_count}')

    set_wf(phase='idle')
    return False


# ── Simulation doubles (--sim: no robot, no instruments) ───────────────────────
# Synthesize a plausible press so the full collect → CSV → plot pipeline can be
# exercised with zero hardware. Force grows with depth (F = 2d + 0.3d²); |Z|
# falls and phase shifts linearly with depth; XJC Fz ≈ 0.98·F.
import random as _rnd

def _sim_force_n(depth_mm):
    return 2.0 * depth_mm + 0.3 * depth_mm * depth_mm

class _SimController:
    """Stand-in for rtde_control: moveL interpolates from the current pose to the
    target at `vel` (m/s), updating the shared robot state with a synthetic
    wrench/AI0 along the way — so the background logger captures a real ramp."""
    def __init__(self, surface_z):
        self._sz = surface_z
        self._stop = None
        self._thr  = None
    def _apply(self, pose):
        depth = (self._sz - pose[2]) * 1000.0
        F = _sim_force_n(max(0.0, depth)) + _rnd.uniform(-0.03, 0.03)
        with _state_lock:
            _state['tcp'] = list(pose)
            _state['ft']  = [0.0, 0.0, -F, 0.0, 0.0, 0.0]
            _state['ai0'] = (F - FUTEK_OFFSET_N) / FUTEK_SLOPE_N_PER_V
    def _interp(self, pose, vel, stop_evt=None):
        with _state_lock:
            cur = list(_state['tcp'])
        dist = sum((pose[i] - cur[i]) ** 2 for i in range(3)) ** 0.5
        dur  = dist / max(1e-6, vel)
        n    = max(1, int(dur / 0.01))        # ~10 ms interpolation increments
        for k in range(1, n + 1):
            if stop_evt is not None and stop_evt.is_set():
                break
            f = k / n
            self._apply([cur[i] + (pose[i] - cur[i]) * f for i in range(6)])
            time.sleep(min(0.01, dur / n))
    def moveL(self, pose, vel=0.05, accel=0.0, asynchronous=False):
        if asynchronous:
            self._stop = threading.Event()
            self._thr  = threading.Thread(target=self._interp,
                                          args=(list(pose), vel, self._stop), daemon=True)
            self._thr.start()
            return
        self._interp(list(pose), vel)
    def stopL(self, decel=2.0):
        if self._stop is not None:
            self._stop.set()
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

    # ── Health check: confirm the meter returns numbers AND time its rate ──────
    # The driver already polls FETC? flat-out (no delay), so this rate IS the
    # meter's fastest. The only lever is the front-panel Speed setting.
    import math as _math
    f0 = f1 = float('nan')
    ok = False
    for _ in range(10):
        time.sleep(0.2)
        f0, f1, ok = lcr.get_latest()
        if _math.isfinite(f0):
            break
    if _math.isfinite(f0):
        n0 = lcr.measurement_count
        time.sleep(1.0)
        lcr_hz = lcr.measurement_count - n0
        print(f'[LCR] Live reading OK: {labels[2]}={f0:.4g}  {labels[3]}={f1:.4g}  '
              f'(status_ok={bool(ok)})')
        print(f'[LCR] Update rate ≈ {lcr_hz} Hz (as fast as the meter allows). '
              + ('Set front-panel Speed=FAST to raise it.' if lcr_hz < 30 else 'Looks good.'))
    else:
        print('[LCR] ⚠ No numeric reading from the meter (fields are NaN).')
        print('      Check the front panel: not showing an error (E01), a valid')
        print('      Function selected, FETC? supported, and the probes connected.')
        if input('  Continue anyway with the LCR (y), or disable it (N)? > ').strip().lower() != 'y':
            try:
                lcr.disconnect()
            except Exception:
                pass
            return None, DEFAULT_LCR_LABELS, 'disabled'
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
    p.add_argument('--ramp-speed', type=float, default=None, dest='ramp_speed',
                   help='Constant indentation ramp speed in mm/s (default: ask, 0.1)')
    p.add_argument('--hold', type=float, default=None,
                   help='Holding time in seconds, applied SYMMETRICALLY at max depth '
                        '(pushed) and at the surface (released) each cycle '
                        '(default: ask, 1.0)')
    p.add_argument('--recover', type=float, default=None,
                   help='Override ONLY the surface/release hold, to break symmetry '
                        '(default: equal to --hold)')
    p.add_argument('--max-force', type=float, default=None, dest='max_force',
                   help=f'Overload safety cutoff in N: abort the press + retract if '
                        f'|Fz| (XJC or robot) exceeds it (default: ask, '
                        f'{DEFAULT_MAX_FORCE_N:.0f}; 0 disables the guard)')
    p.add_argument('--rate', type=int, default=1000,
                   help='Logging / sampling rate in Hz (default: 1000 = 1 ms). Note: '
                        'the LCR (~20-100 Hz) and XJC-poll (~200-300 Hz) hardware '
                        'update slower, so their 1 ms rows repeat the latest reading.')
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
    max_depth  = args.max_depth if args.max_depth is not None else 3.0
    n_iter     = args.iterations if args.iterations is not None else 2
    # In sim, default to a brisk ramp so the dry run finishes quickly.
    ramp_speed = args.ramp_speed if args.ramp_speed is not None else 2.0
    hold_s     = args.hold if args.hold is not None else 0.5
    recover_s  = args.recover if args.recover is not None else hold_s   # symmetric
    max_force  = args.max_force if args.max_force is not None else 0.0  # off in sim unless set

    lcr    = None if args.no_lcr else _SimLCR(_SURFACE_Z)
    labels = LCR_FUNCTIONS['ZTD']
    xjc    = None if args.no_xjc else _SimXjc(_SURFACE_Z)
    if xjc is not None:
        threading.Thread(target=_xjc_reader, args=(xjc,), daemon=True).start()

    print(f'[sim] max_depth={max_depth} mm  iterations={n_iter}  '
          f'ramp_speed={ramp_speed} mm/s  rate={args.rate} Hz')

    os.makedirs(LOG_DIR, exist_ok=True)
    ts        = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path  = os.path.join(LOG_DIR, f'indentation_sim_{ts}.csv')
    meta_path = os.path.join(LOG_DIR, f'indentation_sim_{ts}_meta.json')
    meta = {
        'timestamp': ts, 'csv_file': os.path.basename(csv_path), 'simulated': True,
        'point': {'name': 'sim', 'source': 'sim', 'reused_point': False},
        'surface_pose': [round(v, 6) for v in surface_pose],
        'max_depth_mm': max_depth, 'iterations': n_iter, 'ramp_speed_mm_s': ramp_speed,
        'hold_s': hold_s, 'recover_s': recover_s, 'rate_hz': args.rate,
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
        run_ramp(rtde_c, surface_pose, max_depth, n_iter, ramp_speed,
                 hold_s, recover_s, max_force)
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
        # ── Safe in-air pose first (so the XJC tares UNLOADED and the jog's live
        #    force read-out starts from a clean baseline) ──────────────────────
        home = list(REFERENCE_POSE)
        home[2] += SAFE_CLEAR_MM / 1000.0
        print('[robot] Moving to a safe start height ...')
        rtde_c.moveL(home, VEL_TRAVEL, ACCEL)

        # ── Instruments BEFORE the jog: auto-detect + connect, start the XJC
        #    poll thread so its Fz is live while you look for the contact point ─
        report = detect_instruments(args, robot_connected=True)
        lcr_port, xjc_port = report['lcr'], report['xjc']
        leftover = list(report['unknown'])
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
            time.sleep(0.3)   # let the first few polls land before the jog

        # ── Stages 1-3: calibration (jog with live FUTEK + XJC Fz read-out) ────
        surface_pose, calib_meta = calibrate(rtde_c, args)
        _SURFACE_Z = surface_pose[2]

        # ── Stage 4-6: range + iterations + ramp speed ─────────────────────────
        max_depth = args.max_depth if args.max_depth is not None else _ask_float(
            '\n  Indentation range — max depth (mm) [3.0] > ', 3.0, 0.1, 15.0)
        n_iter = args.iterations if args.iterations is not None else _ask_int(
            '  Iterations (ramp cycles) [3] > ', 3, 1, 1000)
        ramp_speed = args.ramp_speed if args.ramp_speed is not None else _ask_float(
            '  Ramp speed (mm/s, constant) [0.1] > ', 0.1, 0.001, 50.0)
        hold_s = args.hold if args.hold is not None else _ask_float(
            '  Holding time at push & release (s, symmetric) [1.0] > ', 1.0, 0.0, 600.0)
        # Symmetric by default: the surface/release hold equals the push hold,
        # unless --recover explicitly overrides it.
        recover_s = args.recover if args.recover is not None else hold_s
        max_force = args.max_force if args.max_force is not None else _ask_float(
            f'  Max-force safety cutoff (N, 0=off) [{DEFAULT_MAX_FORCE_N:.0f}] > ',
            DEFAULT_MAX_FORCE_N, 0.0, 200.0)

        # ── Summary ──────────────────────────────────────────────────────────────
        cycle_s = 2.0 * (max_depth / ramp_speed) + hold_s + recover_s
        print('\n' + '=' * 70)
        print(f'  Point         : {calib_meta["name"]}  '
              f'XY=({surface_pose[0]*1000:+.2f},{surface_pose[1]*1000:+.2f}) mm  '
              f'surface z={surface_pose[2]*1000:+.2f} mm')
        print(f'  Max depth     : {max_depth:.2f} mm')
        print(f'  Ramp speed    : {ramp_speed:.3f} mm/s (constant) '
              f'→ ~{max_depth/ramp_speed:.0f} s per direction')
        print(f'  Iterations    : {n_iter}   (~{cycle_s:.0f} s/cycle, '
              f'~{n_iter*cycle_s/60:.1f} min total)')
        hold_txt = (f'{hold_s:.1f} s at push & release (symmetric)'
                    if recover_s == hold_s else
                    f'push {hold_s:.1f} s / release {recover_s:.1f} s (asymmetric)')
        print(f'  Holding time  : {hold_txt}')
        print(f'  Force cutoff  : '
              + (f'{max_force:.0f} N (abort + retract on overload)' if max_force > 0
                 else 'OFF — no overload guard'))
        print(f'  Sample rate   : {args.rate} Hz ({1000.0/args.rate:.2f} ms) — logger; '
              f'robot≤500Hz, XJC~200-300Hz, LCR~20-100Hz (slower cols repeat)')
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
            'max_depth_mm': max_depth, 'iterations': n_iter,
            'ramp_speed_mm_s': ramp_speed,
            'hold_s': hold_s, 'recover_s': recover_s, 'rate_hz': args.rate,
            'max_force_n': max_force,
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
        aborted = False
        try:
            aborted = run_ramp(rtde_c, surface_pose, max_depth, n_iter, ramp_speed,
                               hold_s, recover_s, max_force)
        except KeyboardInterrupt:
            print('\n  Interrupted — stopping after the current move ...')
        if aborted:
            print('  [run] Stopped early by the force-safety cutoff.')

        _log_stop.set()
        time.sleep(0.2)
        elapsed = time.time() - start
        n_rows = close_dataset()
        print(f'\n  Completed in {elapsed:.1f} s   rows={n_rows}')
        # Achieved rates — so you can see what each instrument REALLY delivered
        # versus the 1 ms target (hardware-limited for the LCR and XJC-poll).
        if elapsed > 0:
            print(f'  Achieved sample rate : {n_rows/elapsed:7.1f} Hz  (logger; target {args.rate} Hz)')
            if xjc is not None:
                print(f'  XJC poll rate        : {xjc.frames/elapsed:7.1f} Hz  '
                      f'({xjc.errors} bad frames)')
            if lcr is not None:
                try:
                    print(f'  LCR measurement rate : {lcr.measurement_count/elapsed:7.1f} Hz')
                except Exception:
                    pass
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
        print('\n[robot] Returning to the initial position ...')
        try:
            # First lift straight up off the surface, THEN travel back to the
            # initial/home pose the run started from (REFERENCE_POSE + clearance).
            cur = get_robot_state()['tcp']
            cur[2] += SAFE_CLEAR_MM / 1000.0
            rtde_c.moveL(cur, VEL_TRAVEL, ACCEL)
            home = list(REFERENCE_POSE)
            home[2] += SAFE_CLEAR_MM / 1000.0
            rtde_c.moveL(home, VEL_TRAVEL, ACCEL)
            print('[robot] At initial position.')
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
