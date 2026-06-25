"""
single_point_ramp_test.py — Star-Nose Sensor | Fixed-Ramp Single-Point Test
===========================================================================
A variant of single_point_test.py where the press/retract RAMP TIME is set
directly (in whole seconds), instead of being a by-product of a fixed speed.
You pick ONE point, a depth, and a ramp time (e.g. 1, 2, 3 s); the script
derives the robot speed & acceleration so the press AND retract each take that
ramp time — no matter what depth you choose. The hold ("pressing") time at full
depth is fixed at 5 s by default and is NOT asked.

How the fixed ramp works
------------------------
  moveL is commanded with (speed, accel), not a time. To make a move of
  distance d (the depth) take exactly ramp_s, we use a TRAPEZOIDAL profile:
  accelerate for a fraction f of ramp_s, cruise at constant speed for the
  middle, then decelerate. Most of the press is therefore at one well-defined
  indentation rate (the cruise speed) — better for force/Cp characterisation.

        cruise speed v = d / ((1−f)·ramp_s)
        acceleration a = d / (f·(1−f)·ramp_s²)        (f = RAMP_ACCEL_FRAC)

  Because v and a scale with d, changing the depth keeps the ramp at ramp_s.
  Set f = 0.5 for a pure triangular profile (no cruise).

What it does
------------
  1. Press ONE chosen point once (locate → press → hold → retract → post).
  2. Ramp time (press & retract) is a direct INPUT in seconds (e.g. 1/2/3).
  3. Hold (pressing) dwell is FIXED at 5 s (default, not asked).
  4. You still SPECIFY --locate and --post dwell times (seconds).
  5. After the cycle, plots are generated automatically (same as the original).
  6. Press depth is measured downward from the calibrated contact surface.

Signals logged (same schema/CSV as the full collector):
  • LCR-6100 capacitance (Cp-Rp, 20 kHz, 1 V, FAST)
  • UR5 TCP position and F/T sensor
  • FUTEK load cell (AI0)

Usage
-----
  python single_point_ramp_test.py
  python single_point_ramp_test.py --point 10 --depth 9 --ramp 2 --locate 3 --post 8
  python single_point_ramp_test.py --point 5 --depth 3 --ramp 1            # hold=5
  python single_point_ramp_test.py --point 10 --depth 2 --ramp 2 --no-plot
"""

import os
import sys
import csv
import time
import argparse
import threading
from datetime import datetime

import rtde_control
import rtde_receive
from lcr6100 import LCR6100, list_ports

# matplotlib is imported lazily inside plotting so --no-plot / headless still run.

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
_INTEGRATION = os.path.normpath(os.path.join(_HERE, '..', 'Integration_2'))
LOG_DIR      = os.path.join(_HERE, 'logs')
PLOT_DIR     = os.path.join(_HERE, 'plots')

# ── Robot ─────────────────────────────────────────────────────────────────────
ROBOT_IP   = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
VEL_TRAVEL = 0.05    # m/s — travel between points
VEL_PRESS  = 0.004   # m/s — fallback press speed (press/retract now use a ramp time)
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

# ── Ramp-time model & inverse ──────────────────────────────────────────────────

def ramp_time_s(depth_mm, vel=VEL_PRESS, accel=ACCEL):
    """Trapezoidal time to move depth_mm at `vel` with `accel` (one direction)."""
    d = abs(depth_mm) / 1000.0
    if d <= 0.0:
        return 0.0
    d_to_vel = vel * vel / accel
    if d >= d_to_vel:
        return d / vel + vel / accel
    return 2.0 * (d / accel) ** 0.5

# Fraction of the ramp time spent accelerating (and the same decelerating);
# the remaining middle portion is a constant-velocity cruise. 0.25 → 25% accel,
# 50% cruise, 25% decel. Set to 0.5 for a pure triangular profile (no cruise).
RAMP_ACCEL_FRAC = 0.25

def vel_accel_for_ramp(depth_mm, ramp_s, accel_frac=RAMP_ACCEL_FRAC):
    """Speed & accel so a moveL over depth_mm completes in ~ramp_s as a
    TRAPEZOID: accelerate for accel_frac·ramp_s, cruise at constant speed for
    the middle (1−2·accel_frac)·ramp_s, then decelerate. Most of the press is
    therefore at a single, well-defined indentation rate (the cruise speed).

        v = d / ((1−f)·ramp_s)        a = d / (f·(1−f)·ramp_s²)

    where f = accel_frac and d = depth. Depth-independent in time: change the
    depth and the press still takes ramp_s, because v & a scale with d.
    f = 0.5 collapses to the triangular case (v = 2d/T, a = 4d/T²).
    Returns (vel_m_s, accel_m_s2); falls back to fixed press speed if degenerate.
    """
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
    'round_idx', 'sample_idx', 'point', 'depth_mm', 'phase',
    'tcp_x', 'tcp_y', 'tcp_z',
    'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
    'ai0', 'load_cell_N',
    'Cp_F', 'Cp_pF', 'Rp_Ohm', 'lcr_ok',
]

def _log_row(pt, depth_mm, phase, lcr):
    st  = get_robot_state()
    ft  = st['ft']
    tcp = st['tcp']
    ai0 = st['ai0']
    Cp, Rp, lcr_ok = lcr.get_latest()
    row = {
        'timestamp':   round(time.time(), 4),
        'datetime':    datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'round_idx':   0,
        'sample_idx':  0,
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

def _log_timed(pt, depth_mm, phase, lcr, rate_hz, duration_s):
    interval = 1.0 / rate_hz
    t_end    = time.time() + duration_s
    while time.time() < t_end:
        t0 = time.perf_counter()
        _log_row(pt, depth_mm, phase, lcr)
        rem = interval - (time.perf_counter() - t0)
        if rem > 0:
            time.sleep(rem)

# Averaging window (s) for the settled per-phase readings reported at each dwell.
SETTLE_WINDOW_S = 1.0

def _phase_tail_mean(phase, window_s=SETTLE_WINDOW_S, rate_hz=100):
    """Average Cp (pF) and load-cell force (N) over the LAST `window_s` of the
    most recent run of `phase`. Reporting this settled mean — instead of one
    instantaneous sample — cuts sensor/electrical noise and is far more
    repeatable. Returns (cp_pF_mean, load_N_mean, n_samples_used)."""
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

def _move_logged(rtde_c, target, vel, accel, pt, depth_mm, phase, lcr, rate_hz):
    """Run a blocking moveL while logging `phase` continuously from a helper
    thread, so the press/retract ramp spans real time (and shades in plots)
    instead of collapsing to a single zero-width sample. Returns ramp seconds."""
    stop = threading.Event()
    interval = 1.0 / rate_hz

    def _logger():
        while not stop.is_set():
            t0 = time.perf_counter()
            _log_row(pt, depth_mm, phase, lcr)
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

def save_dataset(pt):
    with _log_lock:
        rows = list(_log_rows)
    if not rows:
        print('[log] Nothing to save.')
        return None
    os.makedirs(LOG_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(LOG_DIR, f'single_point_ramp_P{pt:02d}_{ts}.csv')
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
    """
    Build a TCP pose for `pt`. Z is corrected by calibration so the contact
    surface is right; the press depth is applied via `extra_z_mm`
    (negative = deeper into the sensor).
    """
    dx, dy   = POINTS[pt]
    pdx, pdy = POINT_OFFSETS.get(pt, (0.0, 0.0))
    pose     = list(REFERENCE_POSE)
    pose[0] += (dx + CALIB_X_MM + pdx) / 1000.0
    pose[1] += (dy + CALIB_Y_MM + pdy) / 1000.0
    pose[2] += (extra_z_mm + CALIB_Z_MM) / 1000.0
    return pose

def _home_pose():
    return _build_pose(10, SAFE_HOME_Z)

# ── Indentation ───────────────────────────────────────────────────────────────

def do_indentation(rtde_c, pt, depth_mm, lcr, locate_s, hold_s, post_s, ramp_s, rate_hz=100):
    """
    One step-impulse indentation, logging at rate_hz:
      locate (locate_s) → press → hold (hold_s) → retract → post (post_s)
    Press and retract are speed-controlled so each takes ~ramp_s (see
    vel_accel_for_ramp); the dwells are timed waits.
    """
    surface = _build_pose(pt, 0.0)
    pressed = _build_pose(pt, -depth_mm)
    v_press, a_press = vel_accel_for_ramp(depth_mm, ramp_s)

    # ── Locate ───────────────────────────────────────────────────────────────
    print(f'     → locate  (moving to surface P{pt:02d}) ...')
    rtde_c.moveL(surface, VEL_TRAVEL, ACCEL)
    _log_row(pt, depth_mm, 'locate', lcr)
    _log_timed(pt, depth_mm, 'locate', lcr, rate_hz, locate_s)
    cp_m, lc_m, n = _phase_tail_mean('locate', SETTLE_WINDOW_S, rate_hz)
    print(f'     [locate]   LC={lc_m:.2f} N   Cp={cp_m:.2f} pF   (mean of last {n} samples)')

    # ── Press (ramp_s) ─────────────────────────────────────────────────────────
    print(f'     → press   (depth {depth_mm:.2f} mm, target ramp {ramp_s:.1f}s) ...')
    t_press = _move_logged(rtde_c, pressed, v_press, a_press,
                           pt, depth_mm, 'press', lcr, rate_hz)
    print(f'     [press]    target ramp = {ramp_s:.1f}s   actual = {t_press:.2f}s')

    # ── Hold (fixed pressing dwell) ───────────────────────────────────────────
    _log_timed(pt, depth_mm, 'hold', lcr, rate_hz, hold_s)
    cp_m, lc_m, n = _phase_tail_mean('hold', SETTLE_WINDOW_S, rate_hz)
    print(f'     [hold]     LC={lc_m:.2f} N   Cp={cp_m:.2f} pF   (mean of last {n} samples)')

    # ── Retract (ramp_s) ───────────────────────────────────────────────────────
    print(f'     → retract  (back to surface, target ramp {ramp_s:.1f}s) ...')
    t_ret = _move_logged(rtde_c, surface, v_press, a_press,
                         pt, depth_mm, 'retract', lcr, rate_hz)
    print(f'     [retract]  target ramp = {ramp_s:.1f}s   actual = {t_ret:.2f}s')

    # ── Post ─────────────────────────────────────────────────────────────────
    _log_timed(pt, depth_mm, 'post', lcr, rate_hz, post_s)
    cp_m, lc_m, n = _phase_tail_mean('post', SETTLE_WINDOW_S, rate_hz)
    print(f'     [post]     LC={lc_m:.2f} N   Cp={cp_m:.2f} pF   (mean of last {n} samples)')

# ── Plotting ───────────────────────────────────────────────────────────────────

PHASE_COLORS = {
    'locate':  '#cfe8ff',
    'press':   '#ffe2c2',
    'hold':    '#c8f5c8',
    'retract': '#ffd6d6',
    'post':    '#e6e6e6',
}

def _phase_spans(t, phases):
    """Yield (phase, t_start, t_end) for each consecutive run of the same phase."""
    spans = []
    if not phases:
        return spans
    start = 0
    for i in range(1, len(phases) + 1):
        if i == len(phases) or phases[i] != phases[start]:
            # End at the next phase's first sample (t[i]) so spans are
            # contiguous; the final span ends at the last sample.
            t_end = t[i] if i < len(phases) else t[i - 1]
            spans.append((phases[start], t[start], t_end))
            start = i
    return spans

def plot_results(rows, pt, depth_mm, locate_s, hold_s, post_s, show=True):
    """Generate and save a 2×2 summary figure for the single indentation."""
    import matplotlib
    if not show:
        matplotlib.use('Agg')      # headless / save-only
    import matplotlib.pyplot as plt

    if not rows:
        print('[plot] No data to plot.')
        return None

    t0     = rows[0]['timestamp']
    t      = [r['timestamp'] - t0 for r in rows]
    cp     = [r['Cp_pF'] for r in rows]
    load   = [r['load_cell_N'] for r in rows]
    fz     = [r['fz'] for r in rows]
    z0     = rows[0]['tcp_z']
    depth  = [(z0 - r['tcp_z']) * 1000.0 for r in rows]   # mm below start (down = +)
    phases = [r['phase'] for r in rows]
    spans  = _phase_spans(t, phases)

    def shade(ax):
        for phase, ta, tb in spans:
            ax.axvspan(ta, tb, color=PHASE_COLORS.get(phase, '#ffffff'),
                       alpha=0.5, lw=0)

    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle(f'Single-point ramp test — P{pt:02d}   depth={depth_mm:.2f} mm   '
                 f'dwell: locate={locate_s:.1f}s  hold={hold_s:.1f}s  post={post_s:.1f}s',
                 fontsize=13, fontweight='bold')

    # (0,0) Cp vs time
    ax = axs[0, 0]; shade(ax)
    ax.plot(t, cp, color='#1f77b4', lw=1.2)
    ax.set_xlabel('time (s)'); ax.set_ylabel('Cp (pF)')
    ax.set_title('Capacitance vs time'); ax.grid(alpha=0.3)

    # (0,1) force vs time
    ax = axs[0, 1]; shade(ax)
    ax.plot(t, load, color='#d62728', lw=1.2, label='load cell (N)')
    ax.plot(t, fz,   color='#9467bd', lw=1.0, alpha=0.7, label='Fz UR (N)')
    ax.set_xlabel('time (s)'); ax.set_ylabel('force (N)')
    ax.set_title('Force vs time'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (1,0) depth vs time
    ax = axs[1, 0]; shade(ax)
    ax.plot(t, depth, color='#2ca02c', lw=1.2)
    ax.axhline(depth_mm, color='gray', ls='--', lw=0.8,
               label=f'target {depth_mm:.2f} mm')
    ax.set_xlabel('time (s)'); ax.set_ylabel('indentation depth (mm, down +)')
    ax.set_title('Depth vs time'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (1,1) Cp vs load (force–capacitance)
    ax = axs[1, 1]
    sc = ax.scatter(load, cp, c=t, cmap='viridis', s=10)
    ax.set_xlabel('load cell (N)'); ax.set_ylabel('Cp (pF)')
    ax.set_title('Capacitance vs force'); ax.grid(alpha=0.3)
    cb = fig.colorbar(sc, ax=ax); cb.set_label('time (s)')

    # legend for phase shading (kept inside the figure so it's visible on screen)
    from matplotlib.patches import Patch
    handles = [Patch(color=c, alpha=0.5, label=p) for p, c in PHASE_COLORS.items()]
    fig.legend(handles=handles, loc='lower center', ncol=5, fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, 0.01))

    fig.tight_layout(rect=[0, 0.06, 1, 0.96])

    os.makedirs(PLOT_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(PLOT_DIR, f'single_point_ramp_P{pt:02d}_{ts}.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    print(f'[plot] Saved figure → {path}')

    # When showing, leave the figure open so the caller can display it
    # alongside the overlay in a single (blocking) plt.show().
    if not show:
        plt.close(fig)
    return path

def plot_overlay(rows, pt, depth_mm, locate_s, hold_s, post_s, show=True):
    """
    Single time-axis figure overlaying the three signals — depth, force, Cp —
    each on its own y-axis, with the background shaded by phase
    (locate / press / hold / retract / post).
    """
    import matplotlib
    if not show:
        matplotlib.use('Agg')      # headless / save-only
    import matplotlib.pyplot as plt

    if not rows:
        print('[plot] No data to overlay.')
        return None

    t0     = rows[0]['timestamp']
    t      = [r['timestamp'] - t0 for r in rows]
    cp     = [r['Cp_pF'] for r in rows]
    load   = [r['load_cell_N'] for r in rows]
    z0     = rows[0]['tcp_z']
    depth  = [(z0 - r['tcp_z']) * 1000.0 for r in rows]   # mm below start (down = +)
    phases = [r['phase'] for r in rows]
    spans  = _phase_spans(t, phases)

    C_DEPTH, C_FORCE, C_CP = '#2ca02c', '#d62728', '#1f77b4'

    fig, ax_d = plt.subplots(figsize=(13, 6))
    fig.suptitle(f'Single-point ramp overlay — P{pt:02d}   depth={depth_mm:.2f} mm   '
                 f'dwell: locate={locate_s:.1f}s  hold={hold_s:.1f}s  post={post_s:.1f}s',
                 fontsize=13, fontweight='bold')

    # phase-colored background (drawn once, on the base axis)
    for phase, ta, tb in spans:
        ax_d.axvspan(ta, tb, color=PHASE_COLORS.get(phase, '#ffffff'), alpha=0.5, lw=0)

    # axis 1 — depth (left)
    ax_d.plot(t, depth, color=C_DEPTH, lw=1.4, label='depth')
    ax_d.set_xlabel('time (s)')
    ax_d.set_ylabel('indentation depth (mm, down +)', color=C_DEPTH)
    ax_d.tick_params(axis='y', labelcolor=C_DEPTH)
    ax_d.grid(alpha=0.3)

    # axis 2 — force (right)
    ax_f = ax_d.twinx()
    ax_f.plot(t, load, color=C_FORCE, lw=1.4, label='force')
    ax_f.set_ylabel('load cell force (N)', color=C_FORCE)
    ax_f.tick_params(axis='y', labelcolor=C_FORCE)

    # axis 3 — Cp (right, offset outward)
    ax_c = ax_d.twinx()
    ax_c.spines['right'].set_position(('outward', 60))
    ax_c.plot(t, cp, color=C_CP, lw=1.4, label='Cp')
    ax_c.set_ylabel('Cp (pF)', color=C_CP)
    ax_c.tick_params(axis='y', labelcolor=C_CP)

    # combined signal legend (top-left) + phase legend (bottom)
    from matplotlib.lines import Line2D
    sig_handles = [Line2D([0], [0], color=C_DEPTH, lw=1.4, label='depth (mm)'),
                   Line2D([0], [0], color=C_FORCE, lw=1.4, label='force (N)'),
                   Line2D([0], [0], color=C_CP,    lw=1.4, label='Cp (pF)')]
    ax_d.legend(handles=sig_handles, loc='upper left', fontsize=8, framealpha=0.9)

    from matplotlib.patches import Patch
    phase_handles = [Patch(color=c, alpha=0.5, label=p) for p, c in PHASE_COLORS.items()]
    fig.legend(handles=phase_handles, loc='lower center', ncol=5, fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, 0.01))

    fig.tight_layout(rect=[0, 0.06, 1, 0.95])

    os.makedirs(PLOT_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(PLOT_DIR, f'single_point_ramp_overlay_P{pt:02d}_{ts}.png')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    print(f'[plot] Saved overlay figure → {path}')

    # Leave the figure open so the caller can show it together with the
    # 2×2 summary in one blocking plt.show().
    if not show:
        plt.close(fig)
    return path

# ── Helpers ────────────────────────────────────────────────────────────────────

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

# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Fixed-ramp single-point indentation test with auto-plotting',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--point',    type=int,   default=None, help='Point 1–19 (default: ask)')
    p.add_argument('--depth',    type=float, default=None, help='Press depth in mm (default: ask)')
    p.add_argument('--ramp',     type=float, default=None,
                   help='Ramp time (s) for EACH press/retract move, e.g. 1/2/3 (default: ask)')
    p.add_argument('--locate',   type=float, default=None,
                   help='Dwell time (s) at the surface before pressing (default: ask)')
    p.add_argument('--hold',     type=float, default=5.0,
                   help='Pressing/hold time (s) at full depth (default: 5, not asked)')
    p.add_argument('--post',     type=float, default=None,
                   help='Dwell time (s) back at the surface after release (default: ask)')
    p.add_argument('--rate',     type=int,   default=100, help='Logging rate Hz (default: 100)')
    p.add_argument('--port',     default=None, help='LCR serial port (default: interactive)')
    p.add_argument('--no-plot',  action='store_true', help='Skip auto-plotting')
    return p.parse_args()

def main():
    args = parse_args()

    print('=' * 65)
    print('  Fixed-Ramp Single-Point Indentation Test — Star-Nose Sensor')
    print('=' * 65)

    pt = args.point or _ask_int('\n  Point to test [1–19] > ', None, 1, 19)
    if pt not in POINTS:
        print(f'  Invalid point {pt}'); sys.exit(1)

    depth_mm = args.depth if args.depth is not None else _ask_float(
        '  Press depth (mm) [2.0] > ', 2.0, 0.1, 10.0)

    rate_hz = args.rate

    # Ramp time is a DIRECT input now (whole seconds). vel & accel are derived
    # from the depth so the press/retract always take ~ramp_s, any depth.
    ramp_s = args.ramp if args.ramp is not None else float(_ask_int(
        '  Ramp time per press & retract (s) [2] > ', 2, 1, 30))

    # Pressing/hold time at full depth is FIXED (not asked); default 5 s.
    hold_s = args.hold

    locate_s = args.locate if args.locate is not None else _ask_float(
        '  Locate dwell (s, at surface before press) [5.0] > ', 5.0, 0.1, 120.0)

    post_s = args.post if args.post is not None else _ask_float(
        '  Post dwell (s, at surface after release) [5.0] > ', 5.0, 0.1, 120.0)

    # ── Keep hold > ramp so the at-depth reading is taken settled ─────────────
    min_hold = ramp_s * 1.2
    if hold_s < min_hold:
        print(f'  [warn] Hold ({hold_s:.2f}s) does not exceed the ramp '
              f'({ramp_s:.2f}s) — raising hold to {min_hold:.2f}s.')
        hold_s = min_hold

    v_press, a_press = vel_accel_for_ramp(depth_mm, ramp_s)

    print(f'\n  Point             : P{pt:02d}  {POINTS[pt]} mm')
    print(f'  Press depth       : {depth_mm:.2f} mm')
    print(f'  Ramp (press/retr) : {ramp_s:.1f} s each')
    print(f'  Derived motion    : cruise={v_press*1000:.3f} mm/s   '
          f'accel={a_press*1000:.3f} mm/s²')
    print(f'                      trapezoid {RAMP_ACCEL_FRAC*100:.0f}% accel / '
          f'{(1-2*RAMP_ACCEL_FRAC)*100:.0f}% cruise / {RAMP_ACCEL_FRAC*100:.0f}% decel')
    print(f'  Hold (pressing)   : {hold_s:.2f} s')
    print(f'  Locate dwell      : {locate_s:.2f} s')
    print(f'  Post dwell        : {post_s:.2f} s')
    print(f'  Log rate          : {rate_hz} Hz')

    # ── Calibration ───────────────────────────────────────────────────────────
    select_calibration()

    # ── LCR ───────────────────────────────────────────────────────────────────
    lcr_port = args.port or _select_lcr_port()
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
    print('[robot] At home')

    # ── Wire prompt ───────────────────────────────────────────────────────────
    print(f'\n  ┌─────────────────────────────────────────────────────┐')
    print(f'  │  Wire the LCR-6100 probes to point  P{pt:02d}            │')
    print(f'  │  Freq: 20 kHz | Mode: Cp-Rp | Volt: 1 V | FAST     │')
    print(f'  └─────────────────────────────────────────────────────┘')
    Cp_now, _, _ = lcr.get_latest()
    print(f'  Current LCR reading: Cp = {Cp_now*1e12:.2f} pF')

    path = None
    try:
        input('\n  Press ENTER when wired and ready ... ')
        do_indentation(rtde_c, pt, depth_mm, lcr,
                       locate_s=locate_s, hold_s=hold_s, post_s=post_s,
                       ramp_s=ramp_s, rate_hz=rate_hz)
    except (EOFError, KeyboardInterrupt):
        print('\n  Interrupted.')
    except Exception as e:
        print(f'\n  [error] {e}')
    finally:
        _ft_stop.set()
        print('\n[robot] Returning to home ...')
        try:
            rtde_c.moveL(_home_pose(), VEL_TRAVEL, ACCEL)
            rtde_c.stopScript()
        except Exception:
            pass
        lcr.disconnect()
        path = save_dataset(pt)

    # ── Auto-plot ─────────────────────────────────────────────────────────────
    if not args.no_plot:
        with _log_lock:
            rows = list(_log_rows)
        plot_results(rows, pt, depth_mm, locate_s, hold_s, post_s, show=True)
        plot_overlay(rows, pt, depth_mm, locate_s, hold_s, post_s, show=True)

    if path:
        print(f'\n  Data: {path}')
    print('[done]')


if __name__ == '__main__':
    main()
