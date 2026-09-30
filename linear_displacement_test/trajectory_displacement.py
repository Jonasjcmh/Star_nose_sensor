"""
trajectory_displacement.py — Star-Nose Sensor | Multi-Point Trajectory Collector
=================================================================================
A RAW-data collector for MULTI-POINT trajectories across the sensor. It is the
big sibling of `linear_displacement.py` / `bidirectional_displacement.py` — same
raw-logging pipeline, same UR5 calibration-profile selection, same muca-board
reader — but instead of a single start→end slide, you give it a whole SEQUENCE
of pads and it slides through them in order (staying engaged the entire time),
holding at each one, then (optionally) retraces the sequence in REVERSE:

    sequence  b2 → b3 → c4 → d4          (forward)
    reverse   d4 → c4 → b3 → b2          (return, stays engaged)

So for the path b2,b3,c4,d4 one pass is:

    engage at b2
      → HOLD @ b2            (hold_start)
      → slide  b2 → b3  → HOLD @ b3   (hold_mid)
      → slide  b3 → c4  → HOLD @ c4   (hold_mid)
      → slide  c4 → d4  → HOLD @ d4   (hold_end)      ← turnaround
      → slide  d4 → c4  → HOLD @ c4   (hold_mid)
      → slide  c4 → b3  → HOLD @ b3   (hold_mid)
      → slide  b3 → b2  → HOLD @ b2   (hold_start)    ← return
      → retract → home

Three hold times, exactly as requested:
    • hold_start   dwell (s) at the FIRST pad (opening + after the return)
    • hold_end     dwell (s) at the LAST pad (the turnaround)
    • hold_mid     dwell (s) at every INTERMEDIATE pad (both on the way out and
                   on the way back)

You also specify (same as the other collectors; the rest have sensible defaults):
    • points       the ordered sequence of pads (>=2), e.g. b2,b3,c4,d4
    • speed        lateral sliding speed          (mm/s)
    • depth        indentation below the surface  (mm)
    • iterations   repeats of the full trajectory (forward[+reverse])

With --iters N the whole trajectory repeats N times; the opening start-hold
happens once and each pass ends with the return start-hold, so passes chain
without redundant double-holds. Use --no-return for a one-way pass through the
sequence (it lifts and re-approaches the first pad between iterations so it
never drags off-path).

Pass --viz to open a LIVE hex map of the 19 sensor cells during the run. It is
strictly read-only, so it never disturbs the logging. Close the window to stop.

The muca board is read with NO processing at all (pure raw ADC counts via
`mucaboard_data_raw/sensor_raw.py`); the baseline (first) frame is stored per
row in `calib_1..calib_19` for reference. FUTEK load cell + TCP pose + wrench
are logged alongside at 20 Hz. Each logged row carries the CURRENT segment's
from/to pads, `direction` ('fwd' / 'rev'), and `phase` (slide / hold_start /
hold_mid / hold_end / hold_return / locate / engage / retract).

Nothing original is modified — this whole test lives in `linear_displacement_test/`
and only borrows read-only helpers (point map / reference pose / filename
helpers / raw sensor) from the shared modules.

Usage
-----
  python trajectory_displacement.py                                 # ask for everything
  python trajectory_displacement.py --points b2,b3,c4,d4 --speed 10 --depth 2
  python trajectory_displacement.py --points b2,b3,c4,d4 --speed 10 --depth 2 \
         --iters 5 --hold-start 2 --hold-mid 1 --hold-end 2 --viz
  python trajectory_displacement.py --points c3,c4,c5 --no-return --iters 3 \
         --prefix ecoflex_path_c3c5
  python trajectory_displacement.py --points b2,b3,c4,d4 --speed 10 --depth 2 --dry-run
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
                        # (so the tip never drags across the sensor off-path)

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

# ── Trajectory workflow state (stamped onto every logged row) ──────────────────
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

# ── Linear slide primitives ─────────────────────────────────────────────────────

def _interp(ax, ay, bx, by, step_mm):
    """Collinear waypoints from A to B, one every ~step_mm (endpoints included)."""
    import math
    dist = math.hypot(bx - ax, by - ay)
    n = max(1, int(math.ceil(dist / max(0.05, step_mm))))
    return [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(n + 1)]

def _slide_segment(rtde_c, from_pt, to_pt, depth_mm, speed_mps, step_mm,
                   direction, iter_idx):
    """Slide in a straight line from one pad to the next at fixed depth, logging
    progress and the segment's from/to pads."""
    ax, ay = _point_xy(from_pt)
    bx, by = _point_xy(to_pt)
    pts = _interp(ax, ay, bx, by, step_mm)
    n = len(pts)
    set_workflow(from_point=from_pt, to_point=to_pt, sliding=True, phase='slide',
                 direction=direction, iter_idx=iter_idx, progress=0.0)
    for k, (x, y) in enumerate(pts):
        if _stop_flag.is_set():
            print('     [slide] stop requested — halting slide')
            break
        rtde_c.moveL(_pose(x, y, -depth_mm), speed_mps, ACCEL)
        set_workflow(progress=(k / (n - 1)) if n > 1 else 1.0)
    set_workflow(sliding=False)

def _hold(pt, hold_s, phase, direction, iter_idx, tag):
    """Dwell at a pad (already engaged), stamping the hold phase into the log."""
    set_workflow(from_point=pt, to_point=pt, sliding=False, phase=phase,
                 direction=direction, iter_idx=iter_idx)
    if hold_s > 0:
        print(f'  ── {tag}  hold @ {POINT_LABELS.get(pt,"")} {hold_s:.1f}s ...')
        time.sleep(hold_s)

# ── Trajectory (engage → slide/hold through the sequence, [reverse], retract) ───

def run_trajectory(rtde_c, seq, depth_mm, speed_mps, iters, do_return,
                   hold_start_s, hold_mid_s, hold_end_s, locate_s, step_mm):
    """Full sequence for one trajectory (list of pads `seq`, len >= 2).

    locate → engage at seq[0] → hold@start →
      { slide through seq[1..last] (hold_mid at intermediates, hold_end at last)
        [→ slide back through seq[-2..0] (hold_mid at intermediates,
           hold_return at seq[0])] } × iters → retract.

    Stays engaged for the whole (forward[+reverse]) trajectory. The opening
    start-hold happens once; with --no-return + iters>1 the tip lifts and
    re-approaches seq[0] between passes so it never drags off-path."""
    p0 = seq[0]
    sx, sy = _point_xy(p0)

    set_workflow(from_point=p0, to_point=seq[1], depth_mm=depth_mm,
                 speed_mm_s=round(speed_mps * 1000, 1), iter_idx=0,
                 direction='fwd', progress=0.0, sliding=False, done=False)

    # ── Locate: travel above the first pad at clearance ───────────────────────
    set_workflow(phase='locate')
    print(f'  → locate  (above P{p0:02d}/{POINT_LABELS.get(p0,"")}) ...')
    rtde_c.moveL(_pose(sx, sy, TRAVEL_CLEAR_MM), VEL_TRAVEL, ACCEL)
    time.sleep(locate_s)
    if _stop_flag.is_set():
        return

    # ── Engage: down to the surface (fast), then push to depth (slow) ─────────
    set_workflow(phase='engage')
    print(f'  → engage  (down to {depth_mm:.2f} mm) ...')
    rtde_c.moveL(_pose(sx, sy, 0.0), VEL_TRAVEL, ACCEL)
    rtde_c.moveL(_pose(sx, sy, -depth_mm), VEL_ENGAGE, ACCEL)

    # ── Opening hold at the FIRST pad (once) ──────────────────────────────────
    _hold(p0, hold_start_s, 'hold_start', 'fwd', 0, 'start')

    last = len(seq) - 1
    for it in range(iters):
        if _stop_flag.is_set():
            break

        # Forward: slide through seq[1..last], holding at each arrival pad.
        print(f'  ── Trajectory {it + 1}/{iters}  forward  '
              f'{"→".join(POINT_LABELS.get(p,"?") for p in seq)} ...')
        for i in range(last):
            if _stop_flag.is_set():
                break
            _slide_segment(rtde_c, seq[i], seq[i + 1], depth_mm, speed_mps,
                           step_mm, 'fwd', it)
            arrival = seq[i + 1]
            if arrival == seq[last]:
                _hold(arrival, hold_end_s, 'hold_end', 'fwd', it, 'end')
            else:
                _hold(arrival, hold_mid_s, 'hold_mid', 'fwd', it, 'mid')
        if _stop_flag.is_set():
            break

        if do_return:
            # Reverse: slide back through seq[last-1..0], holding at each pad.
            print(f'  ── Trajectory {it + 1}/{iters}  reverse  '
                  f'{"→".join(POINT_LABELS.get(p,"?") for p in reversed(seq))} ...')
            for i in range(last, 0, -1):
                if _stop_flag.is_set():
                    break
                _slide_segment(rtde_c, seq[i], seq[i - 1], depth_mm, speed_mps,
                               step_mm, 'rev', it)
                arrival = seq[i - 1]
                if arrival == p0:
                    _hold(arrival, hold_start_s, 'hold_return', 'rev', it, 'return')
                else:
                    _hold(arrival, hold_mid_s, 'hold_mid', 'rev', it, 'mid')
        elif it < iters - 1:
            # One-way repeat: lift to clearance, travel back to the first pad,
            # descend, re-engage (never drag across the surface off-path).
            ex, ey = _point_xy(seq[last])
            set_workflow(phase='reposition', sliding=False)
            print('  → reposition (lift, back to start, re-engage) ...')
            rtde_c.moveL(_pose(ex, ey, TRAVEL_CLEAR_MM), VEL_ENGAGE, ACCEL)
            rtde_c.moveL(_pose(sx, sy, TRAVEL_CLEAR_MM), VEL_TRAVEL, ACCEL)
            rtde_c.moveL(_pose(sx, sy, 0.0), VEL_TRAVEL, ACCEL)
            rtde_c.moveL(_pose(sx, sy, -depth_mm), VEL_ENGAGE, ACCEL)
            _hold(p0, hold_start_s, 'hold_start', 'fwd', it + 1, 'start')

    # ── Retract: lift off the surface to clearance (ready to travel) ──────────
    # (End pad is seq[0] after a return pass, or seq[last] after a one-way pass.)
    ret_pt = p0 if do_return else seq[last]
    rx, ry = _point_xy(ret_pt)
    set_workflow(from_point=ret_pt, to_point=ret_pt, phase='retract', sliding=False)
    print('  → retract (lift off the surface) ...')
    try:
        rtde_c.moveL(_pose(rx, ry, 0.0), VEL_ENGAGE, ACCEL)
        rtde_c.moveL(_pose(rx, ry, TRAVEL_CLEAR_MM), VEL_TRAVEL, ACCEL)
    except Exception as e:
        print(f'  [retract] {e}')

# ── Display / input helpers ────────────────────────────────────────────────────

def print_sensor_map(seq=None):
    """Draw the sensor hex (a1..e5). First pad in (parens), last pad in
    [brackets], other pads in the path in <angles>."""
    seq = seq or []
    first = seq[0] if seq else None
    last  = seq[-1] if seq else None
    mids  = set(seq[1:-1]) if len(seq) > 2 else set()
    print()
    width = max(len(r) for r in SENSOR_MAP_ROWS)
    for row in SENSOR_MAP_ROWS:
        indent = ' ' * (2 * (width - len(row)))
        parts  = []
        for pt, lbl in row:
            if pt == first:
                parts.append(f'({lbl})')
            elif pt == last and last != first:
                parts.append(f'[{lbl}]')
            elif pt in mids:
                parts.append(f'<{lbl}>')
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

def _parse_sequence(raw):
    """Parse an ORDERED sequence of pads (numbers 1..19 or labels a1..e5),
    comma/space separated. Order is preserved and repeats are allowed (so a
    trajectory may revisit a pad), but two CONSECUTIVE identical pads are
    collapsed (a zero-length slide makes no sense). Returns a list of point
    numbers (>=2)."""
    seq = []
    for tok in raw.replace(',', ' ').split():
        pt = ur5_control.resolve_point(tok)   # 1..19 or a1..e5 → number
        if not seq or seq[-1] != pt:
            seq.append(pt)
    return seq

def _ask_sequence(prompt):
    while True:
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print('  Please enter an ordered sequence: 2+ pads, e.g. b2,b3,c4,d4')
            continue
        try:
            seq = _parse_sequence(raw)
        except ValueError:
            print('  Must be valid pads: 1–19 or a1..e5 (comma separated, in order)')
            continue
        if len(seq) >= 2:
            return seq
        print('  Enter at least TWO distinct pads in order (e.g. b2,b3,c4,d4)')

# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Multi-point trajectory slide collector — RAW counts, no normalisation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--points', dest='points', default=None,
                   help='Ordered sequence of pads (>=2), comma separated, numbers '
                        '1..19 or labels a1..e5, e.g. b2,b3,c4,d4 [ask]')
    p.add_argument('--speed',  type=float, default=None,
                   help='Lateral slide speed in mm/s [ask, default 10]')
    p.add_argument('--depth',  type=float, default=None,
                   help='Indentation depth below surface in mm [ask, default 2]')
    p.add_argument('--iters',  type=int,   default=None,
                   help='Number of full-trajectory repeats [ask, default 1]')
    p.add_argument('--hold-start', type=float, default=None,
                   help='Hold (s) at the FIRST pad — opening & after the return '
                        '[ask, default 1]')
    p.add_argument('--hold-mid',   type=float, default=None,
                   help='Hold (s) at each INTERMEDIATE pad (out and back) '
                        '[ask, default 1]')
    p.add_argument('--hold-end',   type=float, default=None,
                   help='Hold (s) at the LAST pad (turnaround) [ask, default 1]')
    p.add_argument('--no-return', action='store_true',
                   help='One-way pass through the sequence (skip the reverse leg)')
    p.add_argument('--locate', type=float, default=2.0,
                   help='Locate dwell (s) above the first pad before engaging [default 2.0]')
    p.add_argument('--step',   type=float, default=0.5,
                   help='Slide interpolation step in mm [default 0.5]')
    p.add_argument('--viz', action='store_true',
                   help='Open a LIVE sensor hex-map window during the run '
                        '(read-only — does not affect the logged data)')
    p.add_argument('--prefix', default=None, help='Log filename prefix [ask]')
    p.add_argument('--dry-run', action='store_true',
                   help='Print the plan + computed poses, no robot / no sensor')
    return p.parse_args()

def _run_viz(seq, baseline, done_evt):
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
        fig.canvas.manager.set_window_title('Trajectory slide — live sensor (read-only)')
    except Exception:
        pass
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_xlim(min(xs) - 5, max(xs) + 5); ax.set_ylim(min(ys) - 6, max(ys) + 7)

    # Draw the trajectory path (grey line through the ordered pads).
    px = [LABEL_XY[POINT_LABELS[p]][0] for p in seq]
    py = [LABEL_XY[POINT_LABELS[p]][1] for p in seq]
    ax.plot(px, py, '-', color='#888', lw=1.4, zorder=0, alpha=0.7)

    # Rings: first pad = green, last pad = red, intermediates = orange.
    def ring(pt, color, lw):
        ax.scatter([LABEL_XY[POINT_LABELS[pt]][0]], [LABEL_XY[POINT_LABELS[pt]][1]],
                   s=2900, marker='h', facecolors='none', edgecolors=color,
                   linewidths=lw, zorder=1)
    ring(seq[0], '#2ecc71', 2.6)
    ring(seq[-1], '#e74c3c', 2.2)
    for pt in set(seq[1:-1]):
        ring(pt, '#f39c12', 2.0)

    sc = ax.scatter(xs, ys, s=1600, marker='h', c=[0.0] * 19, cmap='inferno',
                    vmin=0, vmax=100, edgecolors='#555', linewidths=1.0, zorder=2)
    txts = [ax.text(xs[i], ys[i], '', ha='center', va='center', fontsize=7,
                    color='white', zorder=3) for i in range(19)]
    title = ax.set_title('', fontsize=10, color='white', pad=8)
    info  = fig.text(0.5, 0.02, '', ha='center', fontsize=8.5, color='#33e666',
                     family='monospace')
    pathstr = '→'.join(POINT_LABELS.get(p, '?') for p in seq)

    def update(_frame):
        vals  = sensor.get_values()
        delta = [abs(vals[i] - base[i]) for i in range(19)]     # display only
        scale = max(50.0, max(delta) if delta else 50.0)
        sc.set_array(np.array(delta)); sc.set_clim(0, scale)
        for i in range(19):
            txts[i].set_text(f'{labels[i]}\n{vals[i]:.0f}')
        wf = get_workflow()
        arrow = '→' if wf['direction'] == 'fwd' else '←'
        seg = (f'{POINT_LABELS.get(wf["from_point"],"?")}{arrow}'
               f'{POINT_LABELS.get(wf["to_point"],"?")}')
        title.set_text(f'{pathstr}    seg {seg}    {wf["phase"] or "idle"}'
                       f'    pass {max(0, wf["iter_idx"]) + 1}'
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
    print('  Multi-Point Trajectory Slide Collector — Star-Nose Sensor')
    print('  (RAW muca counts + FUTEK, engage → path slide/hold → retract)')
    print('=' * 70)

    print_sensor_map()

    # ── Parameters (CLI or interactive) ───────────────────────────────────────
    seq = (_parse_sequence(args.points) if args.points is not None
           else _ask_sequence('  Trajectory sequence [e.g. b2,b3,c4,d4] > '))
    if len(seq) < 2:
        print('[main] Need at least two distinct pads in the sequence — aborting.')
        sys.exit(1)

    speed_mm_s = args.speed if args.speed is not None else _ask_float(
        '  Slide speed (mm/s) [10] > ', 10.0, 0.5, 100.0)
    depth_mm = args.depth if args.depth is not None else _ask_float(
        '  Indentation depth (mm) [2.0] > ', 2.0, 0.1, 10.0)
    iters = (max(1, args.iters) if args.iters is not None else _ask_int(
        '  Trajectory repeats [1] > ', 1, 1, 1000))
    hold_start_s = (max(0.0, args.hold_start) if args.hold_start is not None
                    else _ask_float('  Hold at FIRST pad — start & return (s) [1.0] > ',
                                    1.0, 0.0, 600.0))
    hold_mid_s   = (max(0.0, args.hold_mid) if args.hold_mid is not None
                    else _ask_float('  Hold at each INTERMEDIATE pad (s) [1.0] > ',
                                    1.0, 0.0, 600.0))
    hold_end_s   = (max(0.0, args.hold_end) if args.hold_end is not None
                    else _ask_float('  Hold at LAST pad — turnaround (s) [1.0] > ',
                                    1.0, 0.0, 600.0))
    do_return  = not args.no_return
    locate_s   = max(0.0, args.locate)
    step_mm    = max(0.05, args.step)
    speed_mps  = speed_mm_s / 1000.0

    import math
    slides_per_pass = (2 * (len(seq) - 1)) if do_return else (len(seq) - 1)

    print_sensor_map(seq=seq)
    pathstr = ' → '.join(f'{POINT_LABELS.get(p,"")}' for p in seq)
    revstr  = ' → '.join(f'{POINT_LABELS.get(p,"")}' for p in reversed(seq))
    print(f'  Sequence ({len(seq)})      : {pathstr}')
    if do_return:
        print(f'  Reverse           : {revstr}')
    print(f'  Mode              : {"BIDIRECTIONAL (forward + reverse)" if do_return else "ONE-WAY (forward only)"}, stays engaged')
    print(f'  Speed             : {speed_mm_s:.1f} mm/s')
    print(f'  Depth             : {depth_mm:.2f} mm')
    print(f'  Repeats           : {iters}')
    print(f'  Hold first/return : {hold_start_s:.1f} s  (at {POINT_LABELS.get(seq[0],"")})')
    print(f'  Hold intermediate : {hold_mid_s:.1f} s  (each middle pad)')
    print(f'  Hold last         : {hold_end_s:.1f} s  (at {POINT_LABELS.get(seq[-1],"")})')
    print(f'  Locate dwell      : {locate_s:.1f} s')
    print(f'  Interp step       : {step_mm:.2f} mm')
    print(f'  Live viz          : {"ON (--viz)" if args.viz else "off"}')
    print(f'  Log rate          : {LOG_RATE_HZ} Hz (RAW counts + baseline)')

    total_slide_s = 0.0
    total_dist = 0.0
    print('\n  Segments (each = pad → next pad):')
    for i in range(len(seq) - 1):
        ax, ay = _point_xy(seq[i])
        bx, by = _point_xy(seq[i + 1])
        dist_mm = math.hypot(bx - ax, by - ay)
        total_dist += dist_mm
        seg_s   = dist_mm / speed_mm_s if speed_mm_s > 0 else 0.0
        print(f'    {i+1:>2}. P{seq[i]:02d}({POINT_LABELS.get(seq[i],"")}) → '
              f'P{seq[i+1]:02d}({POINT_LABELS.get(seq[i+1],"")})   '
              f'{dist_mm:5.1f} mm  (~{seg_s:.1f} s)')
    one_pass_slide_s = (total_dist / speed_mm_s if speed_mm_s > 0 else 0.0)
    total_slide_s = iters * (2 if do_return else 1) * one_pass_slide_s
    print(f'  Path length       : {total_dist:.1f} mm one-way  '
          f'({slides_per_pass} slides/pass)')
    print(f'  ~Slide time total : {total_slide_s:.1f} s '
          f'(excl. engage/holds/travel/home)')

    if args.dry_run:
        print('\n[dry-run] Plan only — no robot, no sensor, no logging.')
        for i in range(len(seq) - 1):
            ax, ay = _point_xy(seq[i])
            bx, by = _point_xy(seq[i + 1])
            print(f'[dry-run] P{seq[i]:02d}→P{seq[i+1]:02d}: '
                  f'a={[round(v,4) for v in _pose(ax, ay, -depth_mm)]}  '
                  f'b={[round(v,4) for v in _pose(bx, by, -depth_mm)]}')
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
        input('  Press ENTER to start the trajectory (Ctrl+C to abort) ... ')
    except (EOFError, KeyboardInterrupt):
        pass

    # ── Run the trajectory ────────────────────────────────────────────────────
    _stop_flag.clear()
    _collection_done = threading.Event()
    start_time = time.time()
    worker = None

    def _do_collection():
        try:
            print('═' * 70)
            print(f'  Trajectory:  {pathstr}'
                  f'{"   (+ reverse)" if do_return else ""}   × {iters} pass(es)')
            run_trajectory(rtde_c, seq, depth_mm, speed_mps, iters, do_return,
                           hold_start_s, hold_mid_s, hold_end_s, locate_s, step_mm)
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
                _run_viz(seq, baseline, _collection_done)
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
