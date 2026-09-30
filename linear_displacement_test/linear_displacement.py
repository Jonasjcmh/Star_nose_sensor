"""
linear_displacement.py — Star-Nose Sensor | Linear Point-to-Point Slide Collector
=================================================================================
A RAW-data collector for LINEAR displacements across the sensor. It is a close
cousin of `mucaboard_data_raw/data_collector_raw.py` (same raw-logging pipeline,
same UR5 calibration-profile selection, same muca-board reader) but instead of a
press → hold → retract at a single pad, it:

    engage to `depth` at the INITIAL point
      → SLIDE in a straight line to the FINAL point at a fixed `speed`
      → retract and go home

You specify these (the rest have sensible defaults):

    • speed          lateral sliding speed          (mm/s)
    • initial point  where every slide starts       (1..19 or a1..e5)
    • final point(s) one OR MORE endpoints          (1..19 or a1..e5)
    • depth          indentation below the surface  (mm)
    • iterations     passes per displacement        (repeat each slide N times)
    • hold start/end optional dwell (s) at the initial and final position of
                     every pass (settle the sensor / capture a rest reading)

With several final points you get one linear trajectory per final point, all
sharing the same initial point: initial→final1 (×iters), initial→final2
(×iters), … Between displacements the tool lifts to a travel clearance and
re-approaches, so it never drags across the sensor off-slide.

Pass --viz to open a LIVE hex map of the 19 sensor cells during the run. It is
strictly read-only (it only reads sensor.get_values()), so it never disturbs the
logging: the robot motion moves to a worker thread and the live view owns the
main thread. Close the window to stop early.

Motion is inspired by `friction_mode/ur5_friction.run_displacement_trajectory`
(engage to a fixed Z depth, then pure position-controlled lateral motion — no
force feedback during the slide). The straight line between the two pads is
interpolated into small `--step` mm segments so the motion stays collinear, the
progress is logged, and a stop request is honoured mid-slide.

The muca board is read with NO processing at all (pure raw ADC counts via
`mucaboard_data_raw/sensor_raw.py`); the baseline (first) frame is stored per
row in `calib_1..calib_19` for reference. FUTEK load cell + TCP pose + wrench
are logged alongside at 20 Hz.

Nothing original is modified — this whole test lives in `linear_displacement_test/`
and only borrows read-only helpers (point map / reference pose / filename
helpers / raw sensor) from the shared modules.

Usage
-----
  python linear_displacement.py                                  # ask for everything
  python linear_displacement.py --from a1 --to e5 --speed 10 --depth 2
  python linear_displacement.py --from c3 --to e5,a3,e3,a1 --speed 12 --depth 2 \
         --iters 3 --viz                                         # 4 spokes, live map
  python linear_displacement.py --from 3 --to 17 --speed 15 --depth 2.5 \
         --iters 3 --round-trip --hold-start 1 --hold-end 2 --prefix ecoflex_line_a3_e3
  python linear_displacement.py --from c3 --to e5,a1 --speed 10 --depth 2 --dry-run
"""

import os
import sys
import csv
import time
import argparse
import threading
from datetime import datetime

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
_RAW_DIR     = os.path.normpath(os.path.join(_HERE, '..', 'mucaboard_data_raw'))
LOG_DIR      = os.path.join(_HERE, 'logs')

# Shared, read-only helpers. sensor_raw (RAW muca reader) lives in
# mucaboard_data_raw; the point map / reference pose and the filename helpers
# come from Integration_2. No original file is modified.
sys.path.insert(0, _RAW_DIR)
sys.path.insert(0, _INTEGRATION)
import sensor_raw as sensor   # noqa: E402  RAW muca board (no normalisation)
import data_logger            # noqa: E402  filename helpers only
import ur5_control            # noqa: E402  POINTS / REFERENCE_POSE / resolve_point

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_IP    = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
VEL_TRAVEL  = 0.05    # m/s — travel between points / above the surface
VEL_ENGAGE  = 0.004   # m/s — slow vertical push to depth and retract
ACCEL       = 0.3     # m/s²
SAFE_HOME_Z = 30.0    # mm above the surface at home
TRAVEL_CLEAR_MM = 15.0  # mm above the surface for lateral moves between points
                        # (so the tip never drags across the sensor off-slide)

LOG_RATE_HZ = 20      # 20 Hz logger (0.05 s / frame)

# ── FUTEK load cell ────────────────────────────────────────────────────────────
AI0_ZERO_V       = 5.0
LOADCELL_MAX_N   = 10.0 * 4.44822
LOADCELL_N_PER_V = LOADCELL_MAX_N / 5.0

def _ai0_to_n(v):
    return -(float(v) - AI0_ZERO_V) * LOADCELL_N_PER_V

# ── Sensor points (mm, relative to reference pose) ─────────────────────────────
# Shared point-sensing mapping — from Integration_2/ur5_control.py (the source
# of truth) so this collector never drifts from it. Points are numbered in
# a1..e5 grid order (P1=a1 … P10=c3 centre … P19=e5).
POINTS         = dict(ur5_control.POINTS)
POINT_LABELS   = dict(ur5_control.POINT_TO_LABEL)
REFERENCE_POSE = list(ur5_control.REFERENCE_POSE)

# Sensor-frame (mm) position of each hex label — the operator's physical view,
# used ONLY to draw the console hex map in the correct orientation (a1/a2/a3 run
# up the left diagonal, not a flat top row). Independent of robot-frame POINTS.
LABEL_XY = {
    'a1': (-16,  0), 'a2': (-12,  7), 'a3': ( -8, 14),
    'b1': (-12, -7), 'b2': ( -8,  0), 'b3': ( -4,  7), 'b4': (  0, 14),
    'c1': ( -8,-14), 'c2': ( -4, -7), 'c3': (  0,  0), 'c4': (  4,  7), 'c5': (  8, 14),
    'd2': (  0,-14), 'd3': (  4, -7), 'd4': (  8,  0), 'd5': ( 12,  7),
    'e3': (  8,-14), 'e4': ( 12, -7), 'e5': ( 16,  0),
}

def _build_label_grid():
    label_to_pt = {lbl: pt for pt, lbl in POINT_LABELS.items()}
    rows = {}
    for lbl, (x, y) in LABEL_XY.items():
        rows.setdefault(y, []).append((x, label_to_pt.get(lbl), lbl))
    return [[(pt, lbl) for _, pt, lbl in sorted(rows[y])]
            for y in sorted(rows, reverse=True)]

SENSOR_MAP_ROWS = _build_label_grid()

# ── Calibration globals (UR5 positional profile, not sensor value) ─────────────
CALIB_X_MM    = 0.0
CALIB_Y_MM    = 0.0
CALIB_Z_MM    = 0.0
POINT_OFFSETS = {}     # pt → (dx_mm, dy_mm)

# ── Shared robot state (background FT thread) ──────────────────────────────────
_state      = {'ft': [0.0]*6, 'tcp': [0.0]*6, 'ai0': AI0_ZERO_V}
_state_lock = threading.Lock()
_ft_stop    = threading.Event()

# Stop flag so a Ctrl+C / error can break a slide cleanly.
_stop_flag  = threading.Event()

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

# ── Slide workflow state (stamped onto every logged row) ───────────────────────
_ur5 = {'from_point': 0, 'to_point': 0, 'sliding': False, 'done': False,
        'phase': '', 'depth_mm': 0.0, 'iter_idx': -1, 'direction': '',
        'progress': 0.0, 'speed_mm_s': 0.0}
_ur5_state_lock = threading.Lock()

def set_workflow(**kw):
    with _ur5_state_lock:
        _ur5.update(kw)

def get_workflow():
    with _ur5_state_lock:
        return dict(_ur5)

# ── Streaming logger (RAW muca counts + baseline + force/pose) ─────────────────
# cell_1..cell_19   = PURE RAW ADC counts (no normalisation / linearisation).
# calib_1..calib_19 = baseline (first) frame captured at startup, for reference.
BASE_FIELDS = [
    'timestamp', 'datetime', 'from_point', 'from_label', 'to_point', 'to_label',
    'sliding', 'done', 'tcp_x', 'tcp_y', 'tcp_z',
    'fx', 'fy', 'fz', 'tx', 'ty', 'tz', 'ai0',
]
CELL_FIELDS  = [f'cell_{i+1}' for i in range(19)]    # raw counts
CALIB_FIELDS = [f'calib_{i+1}' for i in range(19)]   # baseline reference
EXTRA_FIELDS = ['depth_mm', 'phase', 'iter_idx', 'direction', 'progress',
                'speed_mm_s', 'load_cell_N']
FIELDNAMES   = BASE_FIELDS + CELL_FIELDS + CALIB_FIELDS + EXTRA_FIELDS

_log_writer = None
_log_fh     = None
_log_count  = 0
_log_lock   = threading.Lock()
_log_stop   = threading.Event()

def _open_log(path):
    global _log_fh, _log_writer, _log_count
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _log_fh     = open(path, 'w', newline='')
    _log_writer = csv.DictWriter(_log_fh, fieldnames=FIELDNAMES)
    _log_writer.writeheader()
    _log_fh.flush()
    _log_count  = 0

def _write_row():
    global _log_count
    st  = get_robot_state()
    ft  = st['ft']; tcp = st['tcp']; ai0 = st['ai0']
    wf  = get_workflow()
    vals  = sensor.get_values()            # RAW counts, unprocessed
    calib = sensor.get_calibration() or [0.0] * 19
    row = {
        'timestamp':  round(time.time(), 4),
        'datetime':   datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'from_point': wf['from_point'],
        'from_label': POINT_LABELS.get(wf['from_point'], ''),
        'to_point':   wf['to_point'],
        'to_label':   POINT_LABELS.get(wf['to_point'], ''),
        'sliding':    int(wf['sliding']),
        'done':       int(wf['done']),
        'tcp_x': round(tcp[0], 5), 'tcp_y': round(tcp[1], 5), 'tcp_z': round(tcp[2], 5),
        'fx': round(ft[0], 4), 'fy': round(ft[1], 4), 'fz': round(ft[2], 4),
        'tx': round(ft[3], 4), 'ty': round(ft[4], 4), 'tz': round(ft[5], 4),
        'ai0': round(ai0, 5),
        'depth_mm':    wf['depth_mm'],
        'phase':       wf['phase'],
        'iter_idx':    wf['iter_idx'],
        'direction':   wf['direction'],
        'progress':    round(wf['progress'], 4),
        'speed_mm_s':  wf['speed_mm_s'],
        'load_cell_N': round(_ai0_to_n(ai0), 4),
    }
    for i, v in enumerate(vals):
        row[f'cell_{i+1}'] = round(float(v), 4)     # raw (kept as-is)
    for i, v in enumerate(calib):
        row[f'calib_{i+1}'] = round(float(v), 4)    # baseline reference
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

# ── Calibration (UR5 positional profile — same profiles as the raw collector) ──

def list_calib_files():
    import glob
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
        print('[calib] Using zero offsets — robot may not hit the points correctly!')
        return

    print('\n  Available calibration profiles:')
    for i, (tip, gpath, ppath) in enumerate(files):
        with open(gpath) as f:
            d = json.load(f)
        pts_info = f'  + per-point ({os.path.basename(ppath)})' if ppath else ''
        print(f'    [{i}]  {tip:20s}  '
              f'X={d.get("x_mm",0):+.3f}  Y={d.get("y_mm",0):+.3f}  '
              f'Z={d.get("z_mm",0):+.3f} mm{pts_info}')

    while True:
        try:
            idx = int(input(f'\n  Select calibration [0–{len(files)-1}] > ').strip())
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
        POINT_OFFSETS = {ur5_control.resolve_point(k): (v.get('dx_mm', 0.0), v.get('dy_mm', 0.0))
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

# ── Pose builders ──────────────────────────────────────────────────────────────

def _point_xy(pt):
    """Calibrated sensor-frame XY (mm) of a pad: base offset + global + per-point."""
    dx, dy   = POINTS[pt]
    pdx, pdy = POINT_OFFSETS.get(pt, (0.0, 0.0))
    return (dx + CALIB_X_MM + pdx, dy + CALIB_Y_MM + pdy)

def _pose(x_mm, y_mm, z_extra_mm=0.0):
    """Build a 6-DOF TCP pose from calibrated sensor-frame XY + Z below surface."""
    pose = list(REFERENCE_POSE)
    pose[0] += x_mm / 1000.0
    pose[1] += y_mm / 1000.0
    pose[2] += (z_extra_mm + CALIB_Z_MM) / 1000.0
    return pose

def _home_pose():
    cx, cy = _point_xy(10)      # centre pad (c3), lifted clear of the surface
    return _pose(cx, cy, SAFE_HOME_Z)

# ── Linear slide (engage → slide → dwell → retract) ────────────────────────────

def _interp(ax, ay, bx, by, step_mm):
    """Collinear waypoints from A to B, one every ~step_mm (endpoints included)."""
    import math
    dist = math.hypot(bx - ax, by - ay)
    n = max(1, int(math.ceil(dist / max(0.05, step_mm))))
    return [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(n + 1)]

def _slide_segment(rtde_c, ax, ay, bx, by, depth_mm, speed_mps, step_mm,
                   direction, iter_idx):
    """Slide in a straight line A→B at fixed depth, logging progress."""
    pts = _interp(ax, ay, bx, by, step_mm)
    n = len(pts)
    set_workflow(sliding=True, phase='slide', direction=direction,
                 iter_idx=iter_idx, progress=0.0)
    for k, (x, y) in enumerate(pts):
        if _stop_flag.is_set():
            print('     [slide] stop requested — halting slide')
            break
        rtde_c.moveL(_pose(x, y, -depth_mm), speed_mps, ACCEL)
        set_workflow(progress=(k / (n - 1)) if n > 1 else 1.0)
    set_workflow(sliding=False)

def run_linear(rtde_c, from_pt, to_pt, depth_mm, speed_mps, iters,
               round_trip, hold_start_s, hold_end_s, locate_s, step_mm):
    """Full sequence: locate → engage → (hold_start → slide → hold_end) × iters
    → retract. hold_start / hold_end are the dwell times at the initial and
    final positions of every pass."""
    sx, sy = _point_xy(from_pt)
    ex, ey = _point_xy(to_pt)

    set_workflow(from_point=from_pt, to_point=to_pt, depth_mm=depth_mm,
                 speed_mm_s=round(speed_mps * 1000, 1), iter_idx=0,
                 direction='fwd', progress=0.0, sliding=False, done=False)

    # ── Locate: travel above the initial point at clearance ───────────────────
    set_workflow(phase='locate')
    print(f'  → locate  (above P{from_pt:02d}/{POINT_LABELS.get(from_pt,"")}) ...')
    rtde_c.moveL(_pose(sx, sy, TRAVEL_CLEAR_MM), VEL_TRAVEL, ACCEL)
    time.sleep(locate_s)
    if _stop_flag.is_set():
        return

    # ── Engage: down to the surface (fast), then push to depth (slow) ─────────
    set_workflow(phase='engage')
    print(f'  → engage  (down to {depth_mm:.2f} mm) ...')
    rtde_c.moveL(_pose(sx, sy, 0.0), VEL_TRAVEL, ACCEL)
    rtde_c.moveL(_pose(sx, sy, -depth_mm), VEL_ENGAGE, ACCEL)

    cx, cy = sx, sy                       # currently engaged position
    for it in range(iters):
        if _stop_flag.is_set():
            break
        # Hold at the INITIAL position (settled at depth) before sliding.
        set_workflow(iter_idx=it, direction='fwd', phase='hold_start')
        if hold_start_s > 0:
            print(f'  ── Pass {it + 1}/{iters}  hold @ start {hold_start_s:.1f}s ...')
            time.sleep(hold_start_s)
        if _stop_flag.is_set():
            break

        print(f'  ── Pass {it + 1}/{iters}  '
              f'P{from_pt:02d}→P{to_pt:02d} @ {depth_mm:.2f} mm, '
              f'{speed_mps*1000:.0f} mm/s ...')
        _slide_segment(rtde_c, cx, cy, ex, ey, depth_mm, speed_mps, step_mm,
                       'fwd', it)
        cx, cy = ex, ey

        # Hold at the FINAL position.
        set_workflow(phase='hold_end')
        if hold_end_s > 0:
            print(f'  ── Pass {it + 1}/{iters}  hold @ end {hold_end_s:.1f}s ...')
            time.sleep(hold_end_s)

        if round_trip:
            if _stop_flag.is_set():
                break
            print(f'  ── Pass {it + 1}/{iters}  return P{to_pt:02d}→P{from_pt:02d} ...')
            _slide_segment(rtde_c, cx, cy, sx, sy, depth_mm, speed_mps, step_mm,
                           'rev', it)
            cx, cy = sx, sy
            # (the next pass opens with its own hold @ start)
        elif it < iters - 1:
            # Forward-only repeat: lift to clearance, travel back to start,
            # descend, re-engage (never drag across the surface off-slide).
            set_workflow(phase='reposition', sliding=False)
            print('  → reposition (lift, back to start, re-engage) ...')
            rtde_c.moveL(_pose(cx, cy, TRAVEL_CLEAR_MM), VEL_ENGAGE, ACCEL)
            rtde_c.moveL(_pose(sx, sy, TRAVEL_CLEAR_MM), VEL_TRAVEL, ACCEL)
            rtde_c.moveL(_pose(sx, sy, 0.0), VEL_TRAVEL, ACCEL)
            rtde_c.moveL(_pose(sx, sy, -depth_mm), VEL_ENGAGE, ACCEL)
            cx, cy = sx, sy

    # ── Retract: lift off the surface to clearance (ready to travel) ──────────
    set_workflow(phase='retract', sliding=False)
    print('  → retract (lift off the surface) ...')
    try:
        rtde_c.moveL(_pose(cx, cy, 0.0), VEL_ENGAGE, ACCEL)
        rtde_c.moveL(_pose(cx, cy, TRAVEL_CLEAR_MM), VEL_TRAVEL, ACCEL)
    except Exception as e:
        print(f'  [retract] {e}')

# ── Display / input helpers ────────────────────────────────────────────────────

def print_sensor_map(from_pt=None, to_pt=None):
    """Draw the sensor hex (a1..e5). Start pad in (parens), end pad in [brackets]."""
    print()
    width = max(len(r) for r in SENSOR_MAP_ROWS)
    for row in SENSOR_MAP_ROWS:
        indent = ' ' * (2 * (width - len(row)))
        parts  = []
        for pt, lbl in row:
            if pt == from_pt:
                parts.append(f'({lbl})')
            elif pt == to_pt:
                parts.append(f'[{lbl}]')
            else:
                parts.append(f' {lbl} ')
        print('  ' + indent + ' '.join(parts))
    print()

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

def _ask_point(prompt):
    while True:
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print('  Please enter a point: 1–19 or a hex label a1..e5')
            continue
        try:
            return ur5_control.resolve_point(raw)   # accepts 1..19 OR a1..e5
        except ValueError:
            print('  Must be a valid point: 1–19 or a1..e5')

def _parse_points_list(raw):
    """Parse a comma/space separated list of points (numbers 1..19 or labels
    a1..e5) into an ordered, de-duplicated list of point numbers."""
    out = []
    for tok in raw.replace(',', ' ').split():
        pt = ur5_control.resolve_point(tok)   # 1..19 or a1..e5 → number
        if pt not in out:
            out.append(pt)
    return out

def _ask_points_list(prompt):
    while True:
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print('  Please enter one or more points: 1–19 or a1..e5, comma separated')
            continue
        try:
            pts = _parse_points_list(raw)
        except ValueError:
            print('  Must be valid points: 1–19 or a1..e5 (comma separated)')
            continue
        if pts:
            return pts
        print('  Enter at least one point')

# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Linear point-to-point slide collector — RAW counts, no normalisation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--from', dest='from_pt', default=None,
                   help='Initial point — number 1..19 or label a1..e5 [ask]')
    p.add_argument('--to', dest='to_pt', default=None,
                   help='Final point(s) — one or more, comma separated, numbers '
                        '1..19 or labels a1..e5, e.g. e5,c3,a3 [ask]. Each runs '
                        'as a separate initial→final displacement.')
    p.add_argument('--speed',  type=float, default=None,
                   help='Lateral slide speed in mm/s [ask, default 10]')
    p.add_argument('--depth',  type=float, default=None,
                   help='Indentation depth below surface in mm [ask, default 2]')
    p.add_argument('--iters',  type=int,   default=None,
                   help='Number of passes per displacement [ask, default 1]')
    p.add_argument('--round-trip', action='store_true',
                   help='Each pass also slides back to the start (stays engaged)')
    p.add_argument('--hold-start', type=float, default=None,
                   help='Hold (s) at the INITIAL position each pass [ask, default 0]')
    p.add_argument('--hold-end',   type=float, default=None,
                   help='Hold (s) at the FINAL position each pass [ask, default 1]')
    p.add_argument('--locate', type=float, default=2.0,
                   help='Locate dwell (s) above start before engaging [default 2.0]')
    p.add_argument('--step',   type=float, default=0.5,
                   help='Slide interpolation step in mm [default 0.5]')
    p.add_argument('--viz', action='store_true',
                   help='Open a LIVE sensor hex-map window during the run '
                        '(read-only — does not affect the logged data)')
    p.add_argument('--prefix', default=None, help='Log filename prefix [ask]')
    p.add_argument('--dry-run', action='store_true',
                   help='Print the plan + computed poses, no robot / no sensor')
    return p.parse_args()

def _run_viz(from_pt, to_pts, baseline, done_evt):
    """LIVE read-only hex map of the 19 raw sensor cells during the run.

    Reads only sensor.get_values() / get_workflow() — never the serial port or
    the log file — so it CANNOT disturb the data collection. Must run on the
    MAIN thread (matplotlib/GUI requirement); the robot motion runs on a worker
    thread while this is open. Closes itself when `done_evt` is set.
    """
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    base   = list(baseline) if baseline else [0.0] * 19
    labels = [POINT_LABELS.get(i + 1, str(i + 1)) for i in range(19)]   # cell i → label
    xs = [LABEL_XY[l][0] for l in labels]
    ys = [LABEL_XY[l][1] for l in labels]

    plt.rcParams.update({'figure.facecolor': '#111111', 'axes.facecolor': '#111111',
                         'text.color': 'white'})
    fig, ax = plt.subplots(figsize=(6.5, 7.2))
    try:
        fig.canvas.manager.set_window_title('Linear slide — live sensor (read-only)')
    except Exception:
        pass
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_xlim(min(xs) - 5, max(xs) + 5); ax.set_ylim(min(ys) - 6, max(ys) + 7)

    # Endpoint rings: start = green, finals = red (drawn behind the live cells).
    ax.scatter([LABEL_XY[POINT_LABELS[from_pt]][0]], [LABEL_XY[POINT_LABELS[from_pt]][1]],
               s=2900, marker='h', facecolors='none', edgecolors='#2ecc71',
               linewidths=2.6, zorder=1)
    for pt in set(to_pts):
        ax.scatter([LABEL_XY[POINT_LABELS[pt]][0]], [LABEL_XY[POINT_LABELS[pt]][1]],
                   s=2900, marker='h', facecolors='none', edgecolors='#e74c3c',
                   linewidths=2.0, zorder=1)

    sc = ax.scatter(xs, ys, s=1600, marker='h', c=[0.0] * 19, cmap='inferno',
                    vmin=0, vmax=100, edgecolors='#555', linewidths=1.0, zorder=2)
    txts = [ax.text(xs[i], ys[i], '', ha='center', va='center', fontsize=7,
                    color='white', zorder=3) for i in range(19)]
    title = ax.set_title('', fontsize=10, color='white', pad=8)
    info  = fig.text(0.5, 0.02, '', ha='center', fontsize=8.5, color='#33e666',
                     family='monospace')

    def update(_frame):
        vals  = sensor.get_values()
        delta = [abs(vals[i] - base[i]) for i in range(19)]     # display only
        scale = max(50.0, max(delta) if delta else 50.0)
        sc.set_array(np.array(delta)); sc.set_clim(0, scale)
        for i in range(19):
            txts[i].set_text(f'{labels[i]}\n{vals[i]:.0f}')
        wf = get_workflow()
        tolab = POINT_LABELS.get(wf['to_point'], '?')
        title.set_text(f'{POINT_LABELS.get(from_pt, "?")} → {tolab}    '
                       f'{wf["phase"] or "idle"}    pass {max(0, wf["iter_idx"]) + 1}'
                       f'    {wf["progress"] * 100:3.0f}%'
                       f'{"   ● sliding" if wf["sliding"] else ""}')
        info.set_text(f'raw min={min(vals):.0f}  max={max(vals):.0f}     '
                      f'display = |Δ baseline| (logged data is untouched raw)')
        if done_evt.is_set():
            plt.close(fig)

    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()


def main():
    args = parse_args()

    print('=' * 70)
    print('  Linear Point-to-Point Slide Collector — Star-Nose Sensor')
    print('  (RAW muca counts + FUTEK, engage → linear slide → retract)')
    print('=' * 70)

    print_sensor_map()

    # ── Parameters (CLI or interactive) ───────────────────────────────────────
    from_pt = (ur5_control.resolve_point(args.from_pt) if args.from_pt is not None
               else _ask_point('  Initial point [1–19 or a1..e5] > '))
    to_pts  = (_parse_points_list(args.to_pt) if args.to_pt is not None
               else _ask_points_list('  Final point(s) [e.g. e5,c3,a3] > '))
    # The initial point cannot also be a final point.
    to_pts = [p for p in to_pts if p != from_pt]
    if not to_pts:
        print('[main] Need at least one final point different from the initial '
              'point — aborting.')
        sys.exit(1)

    speed_mm_s = args.speed if args.speed is not None else _ask_float(
        '  Slide speed (mm/s) [10] > ', 10.0, 0.5, 100.0)
    depth_mm = args.depth if args.depth is not None else _ask_float(
        '  Indentation depth (mm) [2.0] > ', 2.0, 0.1, 10.0)
    iters = (max(1, args.iters) if args.iters is not None else _ask_int(
        '  Iterations (passes) per displacement [1] > ', 1, 1, 1000))
    hold_start_s = (max(0.0, args.hold_start) if args.hold_start is not None
                    else _ask_float('  Hold at INITIAL position each pass (s) [0.0] > ',
                                    0.0, 0.0, 600.0))
    hold_end_s   = (max(0.0, args.hold_end) if args.hold_end is not None
                    else _ask_float('  Hold at FINAL position each pass (s) [1.0] > ',
                                    1.0, 0.0, 600.0))
    round_trip = args.round_trip
    locate_s   = max(0.0, args.locate)
    step_mm    = max(0.05, args.step)
    speed_mps  = speed_mm_s / 1000.0

    import math
    sx, sy = _point_xy(from_pt)
    slides_per_pass = 2 if round_trip else 1

    print_sensor_map(from_pt=from_pt, to_pt=to_pts[0])
    print(f'  From              : P{from_pt:02d} ({POINT_LABELS.get(from_pt,"")})  '
          f'({sx:+.1f}, {sy:+.1f}) mm')
    to_labels = ', '.join(f'P{p:02d}({POINT_LABELS.get(p,"")})' for p in to_pts)
    print(f'  To ({len(to_pts)})          : {to_labels}')
    print(f'  Speed             : {speed_mm_s:.1f} mm/s')
    print(f'  Depth             : {depth_mm:.2f} mm')
    print(f'  Passes/disp.      : {iters}  '
          f'{"(round-trip)" if round_trip else "(forward only)"}')
    print(f'  Hold start / end  : {hold_start_s:.1f} s / {hold_end_s:.1f} s  '
          f'(per pass)')
    print(f'  Locate dwell      : {locate_s:.1f} s')
    print(f'  Interp step       : {step_mm:.2f} mm')
    print(f'  Live viz          : {"ON (--viz)" if args.viz else "off"}')
    print(f'  Log rate          : {LOG_RATE_HZ} Hz (RAW counts + baseline)')

    total_slide_s = 0.0
    print('\n  Displacements (each = initial → final, run {} pass(es)):'.format(iters))
    for j, to_pt in enumerate(to_pts, 1):
        ex, ey = _point_xy(to_pt)
        dist_mm = math.hypot(ex - sx, ey - sy)
        seg_s   = dist_mm / speed_mm_s if speed_mm_s > 0 else 0.0
        total_slide_s += iters * slides_per_pass * seg_s
        print(f'    {j:>2}. P{from_pt:02d}({POINT_LABELS.get(from_pt,"")}) → '
              f'P{to_pt:02d}({POINT_LABELS.get(to_pt,"")})   '
              f'{dist_mm:5.1f} mm  (~{seg_s:.1f} s/slide, '
              f'{len(_interp(sx, sy, ex, ey, step_mm))} wp)')
    print(f'  ~Slide time total : {total_slide_s:.1f} s '
          f'(excl. engage/dwell/travel/home)')

    if args.dry_run:
        print('\n[dry-run] Plan only — no robot, no sensor, no logging.')
        for to_pt in to_pts:
            ex, ey = _point_xy(to_pt)
            print(f'[dry-run] P{from_pt:02d}→P{to_pt:02d}: '
                  f'start={[round(v,4) for v in _pose(sx, sy, -depth_mm)]}  '
                  f'end={[round(v,4) for v in _pose(ex, ey, -depth_mm)]}')
        print('[dry-run] done.')
        return

    # ── Calibration (UR5 positional profile) ──────────────────────────────────
    select_calibration()

    # ── Log file (data_logger naming, in linear_displacement_test/logs) ───────
    prefix   = (data_logger.sanitize_name(args.prefix) if args.prefix
                else data_logger.ask_file_prefix())
    log_file = data_logger.build_filename(prefix, LOG_DIR)

    # ── Muca sensor board (RAW) ───────────────────────────────────────────────
    print('\n[sensor] Starting muca board (RAW mode)...')
    print('[sensor] Keep the sensor UNTOUCHED — first frame is the baseline reference.')
    sensor.start()
    if not sensor.wait_until_ready(timeout=60):
        print('[sensor] ERROR: board not ready — check USB (/dev/ttyACM*).')
        sys.exit(1)
    print('[sensor] Ready!')
    baseline = sensor.get_calibration()
    if baseline is not None:
        print(f'[sensor] Baseline (calib) frame: min={min(baseline):.0f} '
              f'max={max(baseline):.0f}  →  stored per-row in calib_1..calib_19')

    # ── Robot ─────────────────────────────────────────────────────────────────
    import rtde_control
    import rtde_receive
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

    # ── Start streaming logger (20 Hz) ────────────────────────────────────────
    _open_log(log_file)
    log_thread = threading.Thread(target=_logger_loop, daemon=True)
    log_thread.start()
    print(f'[main] Logging (RAW) → {log_file}  ({LOG_RATE_HZ} Hz)')

    try:
        input('  Press ENTER to start the linear slides (Ctrl+C to abort) ... ')
    except (EOFError, KeyboardInterrupt):
        pass

    # ── Run — one initial→final displacement per final point ──────────────────
    _stop_flag.clear()
    _collection_done = threading.Event()
    start_time = time.time()
    worker = None

    def _do_collection():
        try:
            for j, to_pt in enumerate(to_pts, 1):
                if _stop_flag.is_set():
                    break
                print('═' * 70)
                print(f'  Displacement {j}/{len(to_pts)}:  '
                      f'P{from_pt:02d}({POINT_LABELS.get(from_pt,"")}) → '
                      f'P{to_pt:02d}({POINT_LABELS.get(to_pt,"")})   × {iters} pass(es)')
                run_linear(rtde_c, from_pt, to_pt, depth_mm, speed_mps, iters,
                           round_trip, hold_start_s, hold_end_s, locate_s, step_mm)
        except KeyboardInterrupt:
            print('\n  Interrupted — stopping ...')
            _stop_flag.set()
        except Exception as e:
            print(f'\n  [error] {e}')
        finally:
            _collection_done.set()

    try:
        if args.viz:
            # Motion on a worker thread; the live sensor view owns the main
            # thread (matplotlib). The view only READS shared state, so the
            # collection is unaffected. Close the window to stop early.
            worker = threading.Thread(target=_do_collection, daemon=True)
            worker.start()
            try:
                _run_viz(from_pt, to_pts, baseline, _collection_done)
            except Exception as e:
                print(f'[viz] Live view unavailable ({e}) — running headless.')
                _collection_done.wait()
        else:
            _do_collection()
    except KeyboardInterrupt:
        print('\n  Interrupted — stopping ...')
        _stop_flag.set()
    finally:
        _stop_flag.set()
        if worker is not None:
            worker.join(timeout=15)
        set_workflow(phase='', sliding=False, done=True)
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
        elapsed = time.time() - start_time
        print(f'\n  Completed in {elapsed:.1f} s.  Rows: {log_count()}')
        print(f'  RAW data: {log_file}')
    print('[done]')


if __name__ == '__main__':
    main()
