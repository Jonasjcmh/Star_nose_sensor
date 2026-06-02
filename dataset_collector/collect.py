"""
collect.py  —  Star-Nose Sensor Dataset Collector
==================================================
Presses each of the 19 sensor points at multiple depths (default 7, 8, 9 mm)
and records peak + mean sensor values, robot force, and FUTEK load cell force
for each press. Designed to build a machine-learning dataset.

Usage
-----
  python collect.py                                    # default: short_6mm, 100 samples
  python collect.py --tip short_6mm --samples 100 --depths 7 8 9
  python collect.py --samples 20                      # quick test run
  python collect.py --no-sensor                       # robot motion only
  python collect.py --output my_dataset.csv           # custom output file

Output
------
  data/dataset_<tip>_<timestamp>.csv
  Columns: sample_id, timestamp, point, depth_mm, sample_num,
           cell_1..19_peak, cell_1..19_mean,
           fz_peak_N, fz_mean_N, lc_peak_N, lc_mean_N,
           top_cell, top_cell_value, expected_sensor,
           tcp_x, tcp_y, tcp_z
"""

import argparse
import csv
import json
import os
import sys
import time
import random
from datetime import datetime

import numpy as np

# ── Path: share modules from Integration_2 ───────────────────────────────────
COLLECTOR_DIR = os.path.dirname(os.path.abspath(__file__))
INT2_DIR      = os.path.normpath(os.path.join(COLLECTOR_DIR, '..', 'Integration_2'))
DATA_DIR      = os.path.join(COLLECTOR_DIR, 'data')
sys.path.insert(0, INT2_DIR)

# ── Robot / sensor constants (must mirror ur5_control.py) ────────────────────
ROBOT_IP = '177.22.22.2'

REFERENCE_POSE = [
    -0.03746 + 0.0005,
    -0.50066 + 0.0016,
     0.06054,
    -2.35063, 2.08341, -0.00009,
]

POINTS = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

# UR5 point → sensor array index → expected raw cell activated
UR5_TO_IDX = {
    1:16,  2:12,  3:7,
    4:17,  5:13,  6:8,   7:3,
    8:18,  9:14,  10:9,  11:4,  12:0,
    13:15, 14:10, 15:5,  16:1,
    17:11, 18:6,  19:2,
}
RAW_CELLS = [2,15,28,1,14,27,40,0,13,26,39,52,12,25,38,51,24,37,50]
UR5_TO_RAW = {pt: RAW_CELLS[UR5_TO_IDX[pt]] for pt in range(1, 20)}

ALL_POINTS   = list(range(1, 20))
VELOCITY_T   = 0.05    # travel speed m/s
VELOCITY_P   = 0.01    # press speed m/s
ACCEL        = 0.3
SAFE_Z_MM    = 25.0    # home clearance

# FUTEK load cell
AI0_ZERO_V       = 5.0
LOADCELL_N_PER_V = (10.0 * 4.44822) / 5.0   # 8.896 N/V

def _lc_N(v):
    """AI0 voltage → Newtons (positive = compression)."""
    return -(np.asarray(v, dtype=float) - AI0_ZERO_V) * LOADCELL_N_PER_V


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description='Star-Nose Sensor dataset collector',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--tip',       default='short_6mm',
                   help='Tip profile — loads calib_<tip>.json (default: short_6mm)')
    p.add_argument('--samples',   type=int, default=100,
                   help='Samples per (point × depth) combination (default: 100)')
    p.add_argument('--depths',    nargs='+', type=float, default=[7.0, 8.0, 9.0],
                   help='Press depths in mm (default: 7 8 9)')
    p.add_argument('--dwell',     type=float, default=0.8,
                   help='Dwell time at full depth in seconds (default: 0.8)')
    p.add_argument('--no-sensor', action='store_true',
                   help='Skip capacitive sensor (robot motion + force only)')
    p.add_argument('--randomize', action='store_true',
                   help='Randomize point order each sample round')
    p.add_argument('--output',    default=None,
                   help='Output CSV path (default: data/dataset_<tip>_<ts>.csv)')
    return p.parse_args()


# ── Calibration ───────────────────────────────────────────────────────────────
def load_calib(tip):
    """Return (global_calib, per_point_offsets) for the given tip."""
    gx = gy = gz = 0.0
    per_point = {}

    global_path = os.path.join(INT2_DIR, f'calib_{tip}.json' if tip else 'calib.json')
    if os.path.exists(global_path):
        with open(global_path) as f:
            d = json.load(f)
        gx, gy, gz = d['x_mm'], d['y_mm'], d['z_mm']
        print(f'  Global  : X={gx:+.3f}  Y={gy:+.3f}  Z={gz:+.3f} mm')
    else:
        print(f'  WARNING : {os.path.basename(global_path)} not found — zero global offset')

    pts_path = os.path.join(INT2_DIR, f'calib_points_{tip}.json' if tip else 'calib_points.json')
    if os.path.exists(pts_path):
        with open(pts_path) as f:
            d = json.load(f)
        per_point = {int(k): (v.get('dx_mm', 0.0), v.get('dy_mm', 0.0))
                     for k, v in d.get('per_point', {}).items()}
        print(f'  Per-pt  : {len(per_point)} point offsets loaded')
    else:
        print(f'  WARNING : calib_points_{tip}.json not found — zero per-point offsets')

    return (gx, gy, gz), per_point


def _build_pose(pt, global_calib, per_point, extra_z_mm=0.0):
    gx, gy, gz   = global_calib
    pdx, pdy     = per_point.get(pt, (0.0, 0.0))
    dx, dy       = POINTS[pt]
    pose         = REFERENCE_POSE.copy()
    pose[0]     += (dx + gx + pdx) / 1000.0
    pose[1]     += (dy + gy + pdy) / 1000.0
    pose[2]     += (gz + extra_z_mm) / 1000.0
    return pose


def _home(global_calib, per_point):
    return _build_pose(10, global_calib, per_point, SAFE_Z_MM)


# ── Press + record ────────────────────────────────────────────────────────────
def press_and_record(rtde_c, rtde_r, sensor_mod,
                     pt, depth_mm, dwell_s,
                     global_calib, per_point):
    """
    Press at point pt, sample sensor + force during dwell, retract.
    Returns a measurement dict.
    """
    surface = _build_pose(pt, global_calib, per_point, 0.0)
    pressed = _build_pose(pt, global_calib, per_point, -depth_mm)

    # Travel to surface
    rtde_c.moveL(surface, VELOCITY_T, ACCEL)
    time.sleep(0.05)

    # Press down
    rtde_c.moveL(pressed, VELOCITY_P, ACCEL)
    time.sleep(0.1)   # brief settle before sampling

    # Sample during dwell
    n_ticks   = max(5, int(dwell_s / 0.05))
    cells_buf = []
    fz_buf    = []
    ai0_buf   = []

    for _ in range(n_ticks):
        if sensor_mod is not None:
            cells_buf.append(sensor_mod.get_values()[:])
        if rtde_r is not None:
            try:
                ft = rtde_r.getActualTCPForce()
                fz_buf.append(float(ft[2]))
            except Exception:
                pass
            try:
                ai0_buf.append(float(rtde_r.getStandardAnalogInput0()))
            except Exception:
                pass
        time.sleep(0.05)

    # TCP position at press depth
    tcp = [0.0, 0.0, 0.0]
    try:
        tcp = list(rtde_r.getActualTCPPose()[:3])
    except Exception:
        pass

    # Retract to surface
    rtde_c.moveL(surface, VELOCITY_P, ACCEL)

    # ── Aggregate measurements ────────────────────────────────
    result = {
        'tcp_x': round(tcp[0], 5),
        'tcp_y': round(tcp[1], 5),
        'tcp_z': round(tcp[2], 5),
    }

    if cells_buf:
        arr   = np.array(cells_buf, dtype=float)
        peaks = arr.max(axis=0)
        means = arr.mean(axis=0)
    else:
        peaks = np.zeros(19)
        means = np.zeros(19)

    for i in range(19):
        result[f'cell_{i+1}_peak'] = round(float(peaks[i]), 4)
        result[f'cell_{i+1}_mean'] = round(float(means[i]), 4)

    top_idx = int(np.argmax(peaks))
    result['top_cell']       = top_idx + 1                        # 1-indexed
    result['top_cell_value'] = round(float(peaks[top_idx]), 4)
    result['expected_sensor']= UR5_TO_RAW[pt]                     # e.g. 24 for P1

    if fz_buf:
        fz_arr = np.array(fz_buf)
        result['fz_peak_N'] = round(float(-fz_arr.min()), 4)      # positive compression
        result['fz_mean_N'] = round(float(-fz_arr.mean()), 4)
    else:
        result['fz_peak_N'] = 0.0
        result['fz_mean_N'] = 0.0

    if ai0_buf:
        ai0_arr  = np.array(ai0_buf)
        lc_arr   = _lc_N(ai0_arr)
        result['lc_peak_N'] = round(float(lc_arr.max()), 4)
        result['lc_mean_N'] = round(float(lc_arr.mean()), 4)
    else:
        result['lc_peak_N'] = 0.0
        result['lc_mean_N'] = 0.0

    return result


# ── Save ──────────────────────────────────────────────────────────────────────
FIELDNAMES = (
    ['sample_id', 'timestamp', 'point', 'depth_mm', 'sample_num',
     'expected_sensor', 'top_cell', 'top_cell_value']
    + [f'cell_{i+1}_peak' for i in range(19)]
    + [f'cell_{i+1}_mean' for i in range(19)]
    + ['fz_peak_N', 'fz_mean_N', 'lc_peak_N', 'lc_mean_N']
    + ['tcp_x', 'tcp_y', 'tcp_z']
)

def save_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        w.writeheader()
        w.writerows(rows)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    args   = parse_args()
    tip    = args.tip
    depths = sorted(args.depths)
    n_samp = args.samples
    dwell  = args.dwell

    os.makedirs(DATA_DIR, exist_ok=True)

    total     = len(ALL_POINTS) * len(depths) * n_samp
    est_min   = total * (dwell + 2.0) / 60.0

    print('=' * 62)
    print('  Star-Nose Sensor  —  Dataset Collector')
    print('=' * 62)
    print(f'  Tip profile    : {tip}')
    print(f'  Points         : {len(ALL_POINTS)}  (P1 – P19)')
    print(f'  Depths         : {depths} mm')
    print(f'  Samples/combo  : {n_samp}')
    print(f'  Total presses  : {total:,}')
    print(f'  Est. duration  : ~{est_min:.0f} min  ({est_min/60:.1f} h)')
    print(f'  Dwell time     : {dwell} s per press')
    print(f'  Randomize pts  : {"yes" if args.randomize else "no"}')
    print('=' * 62)

    # ── Load calibration ──────────────────────────────────────
    print(f'\n  Calibration [{tip}]:')
    global_calib, per_point = load_calib(tip)

    print(f'\n  Press depths will be: {depths} mm')
    try:
        ans = input('  Correct tip mounted? Continue? [y/N] > ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        print('\n  Aborted.')
        return
    if ans != 'y':
        print('  Aborted.')
        return

    # ── Start sensor ──────────────────────────────────────────
    sensor_mod = None
    if not args.no_sensor:
        print('\n  Starting sensor ...')
        import sensor as _s
        _s.start()
        if not _s.wait_until_ready(timeout=40):
            print('  WARNING: sensor not ready — collecting without sensor data')
        else:
            sensor_mod = _s
            print('  Sensor ready!')

    # ── Connect to UR5 ────────────────────────────────────────
    print('\n  Connecting to UR5 ...')
    rtde_c = rtde_r = None
    for attempt in range(3):
        try:
            import rtde_control, rtde_receive
            rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
            print('  UR5 connected!')
            break
        except Exception as e:
            print(f'  Attempt {attempt+1}/3 failed: {e}')
            try:
                if rtde_r: rtde_r.disconnect()
            except Exception:
                pass
            rtde_c = rtde_r = None
            time.sleep(2)

    if rtde_c is None:
        print('  Could not connect to UR5 — aborting')
        return

    print('  Moving to home ...')
    rtde_c.moveL(_home(global_calib, per_point), VELOCITY_T, ACCEL)
    print('  Ready.\n')

    # ── Output file ───────────────────────────────────────────
    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_file = args.output or os.path.join(DATA_DIR, f'dataset_{tip}_{ts}.csv')

    collected = []
    sample_id = 0

    try:
        for depth_idx, depth in enumerate(depths):
            depth_total = len(ALL_POINTS) * n_samp
            print(f'\n{"─" * 62}')
            print(f'  Depth {depth:.0f} mm  '
                  f'({depth_idx+1}/{len(depths)})  —  '
                  f'{depth_total} presses')
            print(f'{"─" * 62}')

            for samp in range(n_samp):
                pt_order = ALL_POINTS.copy()
                if args.randomize:
                    random.shuffle(pt_order)

                for pt in pt_order:
                    try:
                        m = press_and_record(
                            rtde_c, rtde_r, sensor_mod,
                            pt, depth, dwell,
                            global_calib, per_point)
                    except Exception as e:
                        print(f'\n  ERROR P{pt} @{depth}mm: {e}')
                        continue

                    row = {
                        'sample_id':  sample_id,
                        'timestamp':  round(time.time(), 3),
                        'point':      pt,
                        'depth_mm':   depth,
                        'sample_num': samp,
                        **m,
                    }
                    collected.append(row)
                    sample_id += 1

                    pct      = sample_id / total * 100
                    peak_val = m['top_cell_value']
                    correct  = '✓' if m['top_cell'] == UR5_TO_IDX[pt] + 1 else '✗'
                    print(
                        f'\r  [{sample_id:5,}/{total:,}] {pct:5.1f}%  '
                        f'P{pt:02d} @{depth:.0f}mm  '
                        f'peak={peak_val:.3f} {correct}  '
                        f'Fz={m["fz_peak_N"]:5.1f}N  '
                        f'LC={m["lc_peak_N"]:5.1f}N      ',
                        end='', flush=True)

            print()  # newline after progress line

            # Incremental save after each depth block
            save_csv(out_file, collected)
            print(f'  Saved {len(collected):,} rows → {os.path.basename(out_file)}')

    except KeyboardInterrupt:
        print('\n\n  Ctrl+C — saving collected data ...')

    finally:
        if collected:
            save_csv(out_file, collected)

        try:
            rtde_c.moveL(_home(global_calib, per_point), VELOCITY_T, ACCEL)
            rtde_c.stopScript()
        except Exception:
            pass

    _print_summary(collected, out_file)


def _print_summary(rows, out_file):
    print(f'\n{"=" * 62}')
    print(f'  COLLECTION COMPLETE')
    print(f'{"=" * 62}')
    print(f'  Total samples : {len(rows):,}')
    print(f'  Output        : {out_file}')

    if not rows:
        return

    by_depth = {}
    for r in rows:
        by_depth.setdefault(r['depth_mm'], []).append(r)

    print(f'\n  {"Depth":>6}  {"Samples":>8}  {"Avg peak":>9}  {"Correct cell%":>14}')
    print(f'  {"─"*6}  {"─"*8}  {"─"*9}  {"─"*14}')
    for depth in sorted(by_depth):
        drows   = by_depth[depth]
        peaks   = [r['top_cell_value'] for r in drows]
        correct = [r for r in drows
                   if r['top_cell'] == UR5_TO_IDX.get(r['point'], -1) + 1]
        pct     = len(correct) / len(drows) * 100 if drows else 0
        print(f'  {depth:>5.0f}mm  {len(drows):>8,}  '
              f'{np.mean(peaks):>8.3f}   {pct:>12.1f}%')

    print(f'\n  Columns saved : {len(FIELDNAMES)}')
    print(f'  {", ".join(FIELDNAMES[:6])}, ...')
    print(f'{"=" * 62}')


if __name__ == '__main__':
    main()
