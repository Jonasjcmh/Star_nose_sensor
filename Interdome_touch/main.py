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

Calibration & points
---------------------
At startup you choose a per-point calibration file
(Integration_2/calib_points_*.json). BOTH the 19 pressing points AND the
global Z surface-height offset come from that single file, so what you press
is exactly what calibrate_points.py produced. The reference (start) pose is
the same one used by calibrate_ur5.py / calibrate_points.py.

Points
------
The 19 main pressing points come from the calib_points file you select at
startup (points[n].offset_mm, points[n].x_mm/y_mm, or per_point[n].dx/dy on the
nominal grid). In ADDITION you are asked which set of intermediate points to
press — triangle_centroids_*, diagonal_midpoints_* and horizontal_midpoints_*
(rigid / translated / actual_<tag> / plain, or none). All chosen points (main +
extras) go through the full depth x iteration matrix.

Mapping routine
---------------
Before the run, a mapping routine presses every point once, in order
(P1..P19, then triangles, diagonals, horizontals), holding 1 s at the deepest
test depth and printing the sensor response so you can verify the mapping.
Skip with --no-mapping.

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
M indentation depths (mm) x N iterations x P points, where M and N are chosen
interactively (or via --depths / --iterations) and P is the total number of
points (19 main + the chosen intermediate set). Each iteration presses every
point once, in a random order, so every point gets N samples per depth spread
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

# The TCP calibration leaves the tip standing off ABOVE the surface (the global
# Z is calibrated with a gap, not at contact). Without compensation a commanded
# depth d would travel this standoff first and then indent d, i.e. it presses
# standoff+d. We lower the whole Z frame by SURFACE_STANDOFF_MM so extra_z=0 is
# true surface contact and a commanded depth is exactly the indentation depth.
SURFACE_STANDOFF_MM = 5.0   # override at runtime with --standoff

# Shared start pose — identical to Integration_2/calibrate_ur5.py and
# calibrate_points.py. Keep these three in sync.
REFERENCE_POSE = [
    -0.03664,
    -0.49831,
     0.06071,
    2.346, -2.094, -0.00009
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

DEFAULT_DEPTHS_MM = [5.0, 6.0, 7.0, 8.0, 9.0]

# ── Mapping routine (quick 1-press-per-point verification pass at startup) ─────
MAPPING_HOLD_S = 1.0   # hold at depth during the mapping pass

# Extra pressing points beyond the 19-cell grid. Each file is a dict of
# {label: {..., x_mm, y_mm}}. The variant (which file suffix) is chosen
# interactively at startup (or derived from the calibration file / --extra-variant).
EXTRA_SPECS = [
    ('triangle',   'triangle_centroids'),
    ('diagonal',   'diagonal_midpoints'),
    ('horizontal', 'horizontal_midpoints'),
]

# ── Nominal 19-point grid (theoretical) ──────────────────────────────────────
# Used when a chosen calib_points file only stores per-point deviations (dx,dy)
# relative to this grid. Matches ur5_control.py / calibrate_points.py.
NOMINAL_POINTS = {
     1: ( -8.0, +14.0),  2: (  0.0, +14.0),  3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),  5: ( -4.0,  +7.0),  6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),  8: (-16.0,   0.0),  9: ( -8.0,   0.0),
    10: (  0.0,   0.0), 11: ( +8.0,   0.0), 12: (+16.0,   0.0),
    13: (-12.0,  -7.0), 14: ( -4.0,  -7.0), 15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0), 17: ( -8.0, -14.0), 18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

# ── Points + calibration are BOTH taken from one chosen calib_points file ─────
POINTS      = {}   # int pt -> (x_mm, y_mm) full XY offset from REFERENCE_POSE (19 main)
POINTS_META = {}

# Unified registry of EVERY pressing point (main 19 + triangle/diagonal/horizontal
# extras). Keys: int 1..19 for main points, str labels (e.g. 'T1_2_5') for extras.
POINT_REG   = {}   # id -> {'x','y','kind','cell'}
POINT_ORDER = []   # ordered ids: P1..P19, then triangles, diagonals, horizontals

# The chosen file supplies the global Z surface-height offset (CALIB_Z). The XY
# global offset is already baked into POINTS, so CALIB_X/CALIB_Y stay 0.
CALIB_X_MM = CALIB_Y_MM = CALIB_Z_MM = 0.0
CALIB_TIP         = '(none)'
CALIB_POINTS_PATH = None

def list_calib_points_files():
    """[(name, path), ...] for every per-point calibration file
    (calib_points_*.json / calib_points.json) in Integration_2."""
    results = []
    for path in sorted(glob.glob(os.path.join(_INTEGRATION, 'calib_points_*.json'))):
        name = os.path.basename(path)[len('calib_points_'):-len('.json')]
        results.append((name, path))
    default = os.path.join(_INTEGRATION, 'calib_points.json')
    if os.path.exists(default):
        results.insert(0, ('(default)', default))
    return results

def _summarise_points_file(path):
    """(n_points, (gx,gy,gz) or None) for the selection menu."""
    try:
        with open(path) as f:
            d = json.load(f)
    except Exception:
        return 0, None
    g = d.get('global') or {}
    gxyz = ((g.get('x_mm', 0.0), g.get('y_mm', 0.0), g.get('z_mm', 0.0))
            if g else None)
    n = len(d.get('points') or d.get('per_point') or {})
    return n, gxyz

def select_calibration_points():
    """Interactively choose a calib_points_*.json file; sets CALIB_* + path."""
    global CALIB_TIP, CALIB_POINTS_PATH

    files = list_calib_points_files()
    if not files:
        print(f'[calib] No calib_points_*.json files found in {_INTEGRATION}')
        raise SystemExit(1)

    print()
    print('  Available calibration-points files (Integration_2/calib_points_*.json):')
    for i, (name, path) in enumerate(files):
        n, g = _summarise_points_file(path)
        g_txt = (f'X={g[0]:+.2f} Y={g[1]:+.2f} Z={g[2]:+.2f} mm'
                 if g else 'no global offset')
        print(f'    [{i}]  {name:30s}  {n:2d} pts  {g_txt}')

    while True:
        try:
            raw = input(f'\n  Select calibration points file [0-{len(files) - 1}] > ').strip()
            idx = int(raw)
            if 0 <= idx < len(files):
                break
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
        print(f'  Please enter a number between 0 and {len(files) - 1}')

    name, path = files[idx]
    CALIB_TIP         = name
    CALIB_POINTS_PATH = path
    print(f'\n  [calib] Selected "{name}"  ({os.path.basename(path)})')

    try:
        ans = input('\n  Correct tip mounted? Confirm this calibration? [y/N] > ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        raise SystemExit(1)
    if ans != 'y':
        print('[calib] Aborted — re-run to select a different calibration.')
        raise SystemExit(1)

def load_points(path):
    """Load the 19 pressing points + global Z from the chosen calib_points file.

    Everything (theoretical grid + global XY + per-point deviation) is baked
    into the returned points as an XY offset from REFERENCE_POSE, so only
    CALIB_Z = gz needs applying at press time. Handles the schemas used across
    the project:
      * points[n].offset_mm   — calibrate_points.py full format (XY already incl. global)
      * points[n].x_mm/y_mm   — rigid/translated transform fits (+ global if present)
      * per_point[n].dx/dy    — calibrate_points.py old format (deviation on nominal grid)
      * global only           — nominal grid + global XY
    Returns (points, gz, meta).
    """
    with open(path) as f:
        d = json.load(f)

    g  = d.get('global') or {}
    gx, gy, gz = g.get('x_mm', 0.0), g.get('y_mm', 0.0), g.get('z_mm', 0.0)
    raw_points = d.get('points') or {}
    per_point  = d.get('per_point') or {}

    pts = {}
    if raw_points and any('offset_mm' in v for v in raw_points.values()):
        schema = 'points.offset_mm (calibrate_points full)'
        for k, v in raw_points.items():
            ox, oy = v['offset_mm']
            pts[int(k)] = (float(ox), float(oy))
    elif raw_points and any(('x_mm' in v and 'y_mm' in v) for v in raw_points.values()):
        schema = 'points.x_mm/y_mm (transform fit)'
        for k, v in raw_points.items():
            pts[int(k)] = (float(v['x_mm']) + gx, float(v['y_mm']) + gy)
    elif per_point:
        schema = 'per_point.dx/dy on nominal grid (calibrate_points old)'
        for k, v in per_point.items():
            pt = int(k)
            nx, ny = NOMINAL_POINTS[pt]
            pts[pt] = (nx + gx + v.get('dx_mm', 0.0), ny + gy + v.get('dy_mm', 0.0))
    else:
        schema = 'global only (nominal grid + global XY)'
        for pt, (nx, ny) in NOMINAL_POINTS.items():
            pts[pt] = (nx + gx, ny + gy)

    if not pts:
        raise ValueError(f'No points could be parsed from {os.path.basename(path)}')

    meta = {
        'source':           path,
        'schema':           schema,
        'has_global':       bool(g),
        'global_offset_mm': {'x': gx, 'y': gy, 'z': gz},
        'anchor_point':     d.get('anchor_point', ANCHOR_POINT),
        'rotation_deg':     d.get('rotation_deg'),
        'translation_mm':   d.get('translation_mm'),
        'residual_mm':      d.get('residual_mm'),
    }
    return pts, gz, meta

# ── Extra points: discovery + interactive selection ───────────────────────────
def _variant_from_calib(path):
    """Derive the DEFAULT intermediate-point variant from the calibration file.

      * '...rigid...'                   -> 'rigid'
      * '...translated...'              -> 'translated'
      * 'calib_points_short_<tag>.json' -> 'actual_<tag>'  (untransformed ACTUAL points)
      * otherwise                       -> None (plain files)
    """
    name = os.path.basename(path)
    low  = name.lower()
    if 'rigid' in low:
        return 'rigid'
    if 'translated' in low:
        return 'translated'
    stem = name[:-len('.json')] if name.endswith('.json') else name
    for pre in ('calib_points_short_', 'calib_points_'):
        if stem.startswith(pre):
            tag = stem[len(pre):]
            return f'actual_{tag}' if tag else 'actual'
    return None

def discover_extra_variants():
    """Scan Integration_2 for intermediate-point files and group by variant.

    Returns an ordered list of (variant_value, info) where variant_value is None
    for the plain files (triangle_centroids.json) or the suffix string otherwise
    (e.g. 'actual_new_hollow_2'). info = {'files': {kind: filename}, 'count': int}.
    """
    found = {}
    for kind, prefix in EXTRA_SPECS:
        for path in sorted(glob.glob(os.path.join(_INTEGRATION, f'{prefix}*.json'))):
            base   = os.path.basename(path)
            suffix = base[len(prefix):-len('.json')]        # '' or '_rigid' or '_actual_...'
            value  = suffix[1:] if suffix.startswith('_') else suffix
            value  = value or None                          # '' -> plain
            entry  = found.setdefault(value, {'files': {}, 'count': 0})
            entry['files'][kind] = base
    for value, entry in found.items():
        n = 0
        for base in entry['files'].values():
            try:
                n += len(json.load(open(os.path.join(_INTEGRATION, base))))
            except Exception:
                pass
        entry['count'] = n
    return [(v, found[v]) for v in sorted(found, key=lambda v: (v is not None, str(v)))]

def select_extra_variant(default_variant):
    """Ask which intermediate-point set (triangles/diagonals/horizontals) to use.

    Returns (use_extras: bool, variant_value). Pressing Enter picks the variant
    matching the chosen calibration file; a dedicated option skips extras.
    """
    variants = discover_extra_variants()
    print('\n' + '=' * 70)
    print('  INTERMEDIATE POINTS — which set to press? (triangles / diagonals / horizontals)')
    print('=' * 70)
    if not variants:
        print('  No intermediate-point files found — pressing only the 19 main points.')
        return False, None

    default_idx = next((i for i, (v, _) in enumerate(variants) if v == default_variant), None)
    for i, (value, info) in enumerate(variants):
        label = value if value else '(plain)'
        mark  = '   <- matches calibration' if i == default_idx else ''
        print(f'   [{i}]  {label:26s}  {len(info["files"])}/3 kinds, {info["count"]:3d} pts{mark}')
    none_idx = len(variants)
    print(f'   [{none_idx}]  none — press only the 19 main points')

    if default_idx is None:
        default_idx = none_idx   # no matching variant → default to skipping extras
    while True:
        try:
            raw = input(f'\n  Select intermediate set [0-{none_idx}] (Enter={default_idx}) > ').strip()
        except (EOFError, KeyboardInterrupt):
            raw = ''
        if raw == '':
            idx = default_idx
        elif raw.isdigit() and 0 <= int(raw) <= none_idx:
            idx = int(raw)
        else:
            print(f'  Enter a number between 0 and {none_idx}')
            continue
        if idx == none_idx:
            print('  [points] Intermediate points: none (19 main points only)')
            return False, None
        value, _ = variants[idx]
        print(f'  [points] Intermediate set: {value or "(plain)"}')
        return True, value

def load_extra_points(variant, gx=0.0, gy=0.0):
    """Load triangle/diagonal/horizontal extra points for the chosen variant.

    Returns (ordered list of (kind, label, x_mm, y_mm), meta list). The global XY
    offset is applied so extras line up with the 19 main points.
    """
    loaded, meta = [], []
    for kind, prefix in EXTRA_SPECS:
        fname = f'{prefix}_{variant}.json' if variant else f'{prefix}.json'
        path  = os.path.join(_INTEGRATION, fname)
        if not os.path.exists(path):
            meta.append({'kind': kind, 'file': fname, 'found': False, 'count': 0})
            continue
        with open(path) as f:
            d = json.load(f)
        cnt = 0
        for label, v in d.items():
            loaded.append((kind, label, float(v['x_mm']) + gx, float(v['y_mm']) + gy))
            cnt += 1
        meta.append({'kind': kind, 'file': fname, 'found': True, 'count': cnt})
    return loaded, meta

def build_point_registry(main_points, extra_points):
    """Assemble POINT_REG + ordered POINT_ORDER (P1..P19, T*, D*, H*)."""
    reg, order = {}, []
    for pid in sorted(main_points):                 # 1..19 in numeric order
        x, y = main_points[pid]
        reg[pid] = {'x': x, 'y': y, 'kind': 'main', 'cell': UR5_TO_SENSOR.get(pid, -1)}
        order.append(pid)
    for kind, label, x, y in extra_points:          # triangles, diagonals, horizontals
        reg[label] = {'x': x, 'y': y, 'kind': kind, 'cell': -1}
        order.append(label)
    return reg, order

def select_global_z():
    """Pick a global calib_*.json for press height when the points file lacks Z."""
    files = []
    for path in sorted(glob.glob(os.path.join(_INTEGRATION, 'calib_*.json'))):
        b = os.path.basename(path)
        if b.startswith('calib_points_'):
            continue
        files.append((b, path))
    if not files:
        print('[calib] No calib_*.json for Z — press height uses reference Z (risky!)')
        return 0.0
    print('\n  This points file has no surface-height (Z).')
    print('  Select a global calibration (calib_*.json) for press height:')
    for i, (b, path) in enumerate(files):
        try:
            z = json.load(open(path)).get('z_mm', 0.0)
        except Exception:
            z = 0.0
        print(f'    [{i}]  {b:30s}  Z={z:+.3f} mm')
    while True:
        try:
            idx = int(input(f'\n  Select Z calibration [0-{len(files) - 1}] > ').strip())
            if 0 <= idx < len(files):
                break
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
        print(f'  Enter a number between 0 and {len(files) - 1}')
    b, path = files[idx]
    gz = json.load(open(path)).get('z_mm', 0.0)
    print(f'  [calib] Press height Z={gz:+.3f} mm from {b}')
    return gz

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
def _pt_label(pid):
    """Human label: 'P10' for a main point, the raw label (e.g. 'T1_2_5') for extras."""
    return f'P{pid:02d}' if isinstance(pid, int) else str(pid)

def _pt_xy(pid):
    r = POINT_REG[pid]
    return r['x'], r['y']

def _build_pose(pid, extra_z_mm=0.0):
    dx, dy = _pt_xy(pid)
    pose = list(REFERENCE_POSE)
    pose[0] += (dx + CALIB_X_MM) / 1000.0
    pose[1] += (dy + CALIB_Y_MM) / 1000.0
    # Subtract the standoff so extra_z=0 sits at true surface contact, not the
    # calibrated gap above it (see SURFACE_STANDOFF_MM).
    pose[2] += (extra_z_mm + CALIB_Z_MM - SURFACE_STANDOFF_MM) / 1000.0
    return pose

def _home_pose():
    return _build_pose(ANCHOR_POINT, SAFE_HOME_Z)

# ── Dataset log ────────────────────────────────────────────────────────────────
_log_rows = []
_log_lock = threading.Lock()

FIELDNAMES = (
    ['timestamp', 'datetime',
     'depth_idx', 'depth_mm', 'iteration', 'point', 'point_kind', 'phase',
     'point_x_mm', 'point_y_mm', 'raw_sensor_cell',
     'tcp_x', 'tcp_y', 'tcp_z',
     'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
     'ai0', 'futek_force_N']
    + [f'cell_{i + 1}' for i in range(19)]
)

def _log_row(sensor_mod, pt, depth_mm, depth_idx, phase, iteration):
    st  = get_robot_state()
    ft, tcp, ai0 = st['ft'], st['tcp'], st['ai0']
    reg = POINT_REG[pt]
    px, py = reg['x'], reg['y']
    row = {
        'timestamp':      round(time.time(), 4),
        'datetime':       datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'depth_idx':      depth_idx,
        'depth_mm':       depth_mm,
        'iteration':      iteration,
        'point':          _pt_label(pt),
        'point_kind':     reg['kind'],
        'phase':          phase,
        'point_x_mm':     round(px, 4),
        'point_y_mm':     round(py, 4),
        'raw_sensor_cell':reg['cell'],
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

    len(depths) depths (outer) x n_iterations (middle) x P points (inner,
    random order per iteration) — every point gets exactly n_iterations samples
    per depth, spread uniformly across time within that depth block."""
    rng  = random.Random(seed)
    pts  = list(POINT_ORDER)
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
          f'x {len(POINT_ORDER)} points = {total} indentations')
    print(f'  First 10 steps:')
    for depth_idx, depth, it, pt in plan[:10]:
        px, py = _pt_xy(pt)
        print(f'    depth={depth:.1f}mm  iter={it}  {_pt_label(pt):8s} '
              f'({px:+.1f},{py:+.1f}) mm')
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
    print(f'     [hold]     {_pt_label(pt):8s} @{depth_mm:.1f}mm   '
          f'Fz={st["ft"][2]:+.2f}N   FUTEK={ai0_to_futek_n(st["ai0"]):+.2f}N')

    # ── retract (ramp up): return to surface ──────────────────────────────────
    rtde_c.moveL(surface, VEL_PRESS, ACCEL)
    snapshot('retract')

    # ── post (releasing): hold at surface after retract ───────────────────────
    _log_timed(sensor_mod, pt, depth_mm, depth_idx, 'post', iteration, rate_hz, hold_s)

    with _log_lock:
        n_rows = len(_log_rows)
    print(f'     rows so far: {n_rows}')

# ── Mapping routine ───────────────────────────────────────────────────────────
def run_mapping(rtde_c, sensor_mod, depth_mm, hold_s=MAPPING_HOLD_S):
    """Press every point once, in order (P1..P19, T*, D*, H*), holding hold_s at
    depth_mm, and print the sensor response. A quick verification pass over
    exactly the points the full test will use — no data is logged."""
    order = POINT_ORDER
    print('\n' + '=' * 70)
    print(f'  MAPPING ROUTINE — {len(order)} points | depth {depth_mm:.1f}mm | '
          f'hold {hold_s:.0f}s each')
    print('  Order: P1..P19, then triangles (T), diagonals (D), horizontals (H)')
    print('=' * 70)

    def _peak_ur5_point():
        if sensor_mod is None:
            return None, 0.0
        best = max(range(1, 20), key=lambda p: sensor_mod.get_value_for_ur5_point(p))
        return best, sensor_mod.get_value_for_ur5_point(best)

    for i, pid in enumerate(order, 1):
        reg   = POINT_REG[pid]
        label = _pt_label(pid)
        x, y  = reg['x'], reg['y']
        cell  = reg['cell']

        rtde_c.moveL(_build_pose(pid, 0.0), VEL_TRAVEL, ACCEL)
        rtde_c.moveL(_build_pose(pid, -depth_mm), VEL_PRESS, ACCEL)
        time.sleep(hold_s)

        peak_pt, peak_val = _peak_ur5_point()
        if sensor_mod is None:
            resp = '(no sensor)'
        elif reg['kind'] == 'main':
            exp = sensor_mod.get_value_for_ur5_point(pid)
            ok  = '✓' if (peak_pt == pid and exp > 0.05) else ('~' if exp > 0.05 else '✗')
            resp = f'expect S{cell} val={exp:.3f}  peak=P{peak_pt:02d}({peak_val:.3f})  {ok}'
        else:
            resp = f'peak=P{peak_pt:02d}({peak_val:.3f})' if peak_pt else '(no reading)'

        print(f'  [{i:02d}/{len(order)}] {label:8s} {reg["kind"]:10s} '
              f'({x:+6.1f},{y:+6.1f})mm  {resp}')

        rtde_c.moveL(_build_pose(pid, 0.0), VEL_PRESS, ACCEL)

    rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
    print('=' * 70)
    print('  Mapping complete — robot at home. Review the console above.')
    print('=' * 70)

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
        except (EOFError, KeyboardInterrupt):
            return list(default_list)     # Enter-less exit → default
        if raw == '':
            return list(default_list)     # bare Enter → default
        try:
            vals = [float(x) for x in raw.replace(',', ' ').split()]
        except ValueError:
            print('  Could not parse — enter numbers only, e.g. 1,2,3 or 1 2 3')
            continue                      # re-prompt, do NOT silently use default
        if vals and all(0.0 < v <= 20.0 for v in vals):
            return vals
        print('  Enter one or more depths in mm (e.g. 1,2,3), each between 0 and 20')

def parse_args():
    p = argparse.ArgumentParser(
        description='Interdome_touch — Star-Nose Sensor + UR5 + FUTEK (long-hold schema)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--iterations', type=int, default=None,
                   help='Iterations per depth (default: ask interactively, fallback 10)')
    p.add_argument('--depths', nargs='+', type=float, default=None,
                   help='One or more indentation depths in mm (default: ask interactively)')
    p.add_argument('--rate', type=int, default=100,
                   help='Logging rate in Hz during hold phases (default: 100)')
    p.add_argument('--hold', type=float, default=1.0,
                   help='Minimum hold per phase in seconds — a floor; the actual '
                        'hold is raised to at least the ramp time (default: 1.0)')
    p.add_argument('--hold-mult', type=float, default=1.0, dest='hold_mult',
                   help='Hold = ramp_time * this multiplier (>=1.0). default: 1.0')
    p.add_argument('--no-sensor', action='store_true',
                   help='Skip the capacitive sensor (robot + FUTEK only)')
    p.add_argument('--standoff', type=float, default=None,
                   help='Surface standoff in mm baked into the TCP calibration Z '
                        f'(default: {SURFACE_STANDOFF_MM}). Subtracted so a commanded '
                        'depth equals the true indentation depth.')
    p.add_argument('--extra-variant', default=None, dest='extra_variant',
                   help="Force the intermediate-point variant (skips the prompt): e.g. "
                        "'actual_new_hollow_2', 'rigid', 'translated', 'plain', or 'none'")
    p.add_argument('--no-mapping', action='store_true',
                   help='Skip the startup mapping routine (1-press-per-point verification)')
    p.add_argument('--mapping-depth', type=float, default=None, dest='mapping_depth',
                   help='Press depth (mm) for the mapping routine (default: deepest test depth)')
    p.add_argument('--seed', type=int, default=None,
                   help='Random seed for the sampling plan (default: random)')
    return p.parse_args()

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    global SURFACE_STANDOFF_MM
    if args.standoff is not None:
        SURFACE_STANDOFF_MM = args.standoff

    print('=' * 70)
    print('  Interdome_touch — Star-Nose Sensor + UR5 + FUTEK (long-hold schema)')
    print('=' * 70)

    n_iterations = args.iterations or _ask_int(
        '\n  Iterations per depth [10] > ', default=10, minimum=1, maximum=100)

    depths_mm = args.depths or _ask_depths(
        '  Indentation depths (mm, one or more, comma-separated)', DEFAULT_DEPTHS_MM)
    depths_mm = sorted(depths_mm)

    rate_hz   = args.rate
    hold_mult = max(1.0, args.hold_mult)

    print(f'\n  Iterations/depth   : {n_iterations}')
    print(f'  Depths             : {depths_mm} mm  ({len(depths_mm)} depth(s))')
    print(f'  Log rate           : {rate_hz} Hz')

    # ── Calibration points file (points + global Z) ───────────────────────────
    select_calibration_points()

    # ── Points ─────────────────────────────────────────────────────────────────
    global POINTS, POINTS_META, CALIB_Z_MM, POINT_REG, POINT_ORDER
    POINTS, gz, POINTS_META = load_points(CALIB_POINTS_PATH)
    g = POINTS_META['global_offset_mm']
    print(f'\n  [points] Loaded {len(POINTS)} main points from '
          f'{os.path.basename(POINTS_META["source"])}')
    print(f'  [points] schema: {POINTS_META["schema"]}')

    # Press height (Z): use the file's global Z, or fall back to a global calib_*.json.
    if POINTS_META['has_global']:
        CALIB_Z_MM = gz
        print(f'  [points] global offset X={g["x"]:+.3f} Y={g["y"]:+.3f} '
              f'Z={g["z"]:+.3f} mm  (press height uses Z)')
    else:
        CALIB_Z_MM = select_global_z()
    print(f'  [points] surface standoff {SURFACE_STANDOFF_MM:.2f} mm removed '
          f'→ commanded depth = true indentation depth')

    # ── Intermediate points — ASK which set to use ────────────────────────────
    default_variant = _variant_from_calib(CALIB_POINTS_PATH)
    if args.extra_variant is not None:
        ev = args.extra_variant.lower()
        if ev in ('none', 'skip'):
            use_extras, variant = False, None
        elif ev == 'plain':
            use_extras, variant = True, None
        else:
            use_extras, variant = True, args.extra_variant
    else:
        use_extras, variant = select_extra_variant(default_variant)

    if use_extras:
        # Intermediate files come in two coordinate frames:
        #   * plain (triangle_centroids.json, ...) : NOMINAL frame → add the global offset
        #   * rigid / translated / actual_*        : already FULL absolute coords (they
        #     bake in the transform, or global+per_point), so the global offset must NOT
        #     be added again. Doing so double-counts it (~3.5mm) and shifts the
        #     intermediate presses onto the domes.
        def _load(v):
            gx, gy = (g['x'], g['y']) if v is None else (0.0, 0.0)
            return load_extra_points(v, gx=gx, gy=gy)
        extra_points, extra_meta = _load(variant)
        # Fall back to plain if an 'actual_*' variant's files aren't present.
        if variant and str(variant).startswith('actual') and not any(m['found'] for m in extra_meta):
            print(f'  [points] ⚠ No files for variant "{variant}" — falling back to plain')
            variant = None
            extra_points, extra_meta = _load(variant)
        print(f'  [points] extra-point variant: {variant or "(plain)"}')
        for m in extra_meta:
            status = f'{m["count"]:2d} pts' if m['found'] else 'NOT FOUND'
            print(f'  [points]   {m["kind"]:10s} {m["file"]:44s} {status}')
    else:
        extra_points, extra_meta, variant = [], [], None

    # ── Unified registry (order: P1..P19, T*, D*, H*) ─────────────────────────
    POINT_REG, POINT_ORDER = build_point_registry(POINTS, extra_points)
    n_main  = sum(1 for p in POINT_ORDER if POINT_REG[p]['kind'] == 'main')
    n_extra = len(POINT_ORDER) - n_main
    print(f'  [points] TOTAL {len(POINT_ORDER)} points '
          f'({n_main} main + {n_extra} extra) per iteration')

    ramp_per_depth = {d: ramp_time_s(d) for d in depths_mm}
    hold_per_depth = {d: max(args.hold, ramp_per_depth[d] * hold_mult) for d in depths_mm}
    print('\n  Depth   ramp_s   hold_s')
    for d in depths_mm:
        print(f'  {d:5.1f}   {ramp_per_depth[d]:6.2f}   {hold_per_depth[d]:6.2f}')

    n_points     = len(POINT_ORDER)
    total_indent = len(depths_mm) * n_iterations * n_points
    per_indent_s = {d: 3 * hold_per_depth[d] + 2 * ramp_per_depth[d] + 3.0 for d in depths_mm}
    est_s = sum(per_indent_s[d] * n_iterations * n_points for d in depths_mm)
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

    # ── Mapping routine (verify every point once, in order, before the run) ────
    mapping_depth = args.mapping_depth if args.mapping_depth is not None else max(depths_mm)
    if not args.no_mapping:
        try:
            run_mapping(rtde_c, sensor_mod, mapping_depth, hold_s=MAPPING_HOLD_S)
        except KeyboardInterrupt:
            print('\n  Mapping interrupted — returning home ...')
            try:
                rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
            except Exception:
                pass
        try:
            ans = input('\n  Proceed to the full test? [y/N] > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            ans = 'n'
        if ans != 'y':
            print('  Aborted after mapping — no test data collected.')
            _ft_stop.set()
            try:
                rtde_c.stopScript()
            except Exception:
                pass
            return
    else:
        print('[map] Mapping routine skipped (--no-mapping)\n')

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
        'points_per_iter':  len(POINT_ORDER),
        'main_points':      n_main,
        'extra_points':     n_extra,
        'extra_variant':    variant,
        'extra_point_files': extra_meta,
        'point_ids':        [_pt_label(p) for p in POINT_ORDER],
        'surface_standoff_mm': SURFACE_STANDOFF_MM,
        'mapping_routine':  {
            'enabled':  not args.no_mapping,
            'depth_mm': mapping_depth,
            'hold_s':   MAPPING_HOLD_S,
        },
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
        'calibration_points_file': os.path.basename(CALIB_POINTS_PATH),
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
            reg    = POINT_REG[pt]
            px, py = reg['x'], reg['y']
            label  = _pt_label(pt)
            hold_s = hold_per_depth[depth]

            print('-' * 70)
            print(f'  Step {step + 1}/{total}  |  Depth {depth:.1f}mm '
                  f'({depth_idx + 1}/{len(depths_mm)})  |  Iter {it + 1}/{n_iterations}  '
                  f'|  {label}')
            cell_txt = f'sensor=S{reg["cell"]}' if reg['cell'] >= 0 else '(no single cell)'
            print(f'  Point {label:8s} {reg["kind"]:10s} ({px:+.1f}, {py:+.1f}) mm  {cell_txt}')
            if reg['kind'] == 'main':
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
