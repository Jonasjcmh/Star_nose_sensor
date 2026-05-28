"""
calibrate_ur5.py
Interactive UR5 calibration tool — PyCharm compatible.
Type commands to jog the TCP until aligned over S26 (P10 center).

Commands:
  x+    → move right  (increase X)
  x-    → move left   (decrease X)
  y+    → move forward
  y-    → move back
  z+    → move up
  z-    → move down
  step 0.5  → set step size in mm
  press     → press down and read sensor peak
  status    → print current offset + sensor value
  reset     → go back to zero offset
  save      → save calibration and quit
  quit      → quit without saving
"""

import json
import os
import sys
import time
import threading

ROBOT_IP   = "177.22.22.2"
CALIB_DIR  = os.path.dirname(os.path.abspath(__file__))

def calib_file(tip=None):
    name = f'calib_{tip}.json' if tip else 'calib.json'
    return os.path.join(CALIB_DIR, name)

REFERENCE_POSE = [
    -0.03746+0.0005,
    -0.50066+0.0016,
     0.06054,
    -2.35063, 2.08341, -0.00009
]

VELOCITY_JOG   = 0.02
VELOCITY_PRESS = 0.01
ACCEL          = 0.3
INDENT_MM      = 10.0

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sensor

def load_existing(tip=None):
    f_path = calib_file(tip)
    if os.path.exists(f_path):
        with open(f_path) as f:
            d = json.load(f)
        print(f"[calib] Loaded existing offset: "
              f"X={d['x_mm']:+.3f} Y={d['y_mm']:+.3f} Z={d['z_mm']:+.3f} mm")
        return d['x_mm'], d['y_mm'], d['z_mm']
    return 0.0, 0.0, 0.0

def save_calib(x, y, z, tip=None):
    f_path = calib_file(tip)
    with open(f_path, 'w') as f:
        json.dump({'x_mm': round(x,4),
                   'y_mm': round(y,4),
                   'z_mm': round(z,4)}, f, indent=2)
    print(f"\n[calib] Saved to {f_path}")
    print(f"[calib] Paste into ur5_control.py:")
    print(f"         CALIB_X_MM = {round(x,4)}")
    print(f"         CALIB_Y_MM = {round(y,4)}")
    print(f"         CALIB_Z_MM = {round(z,4)}")

def build_pose(dx_mm, dy_mm, dz_mm):
    p = REFERENCE_POSE.copy()
    p[0] += dx_mm / 1000.0
    p[1] += dy_mm / 1000.0
    p[2] += dz_mm / 1000.0
    return p

def sensor_bar(val, width=20):
    n = int(val * width)
    return '[' + '#'*n + '-'*(width-n) + f'] {val:.3f}'

def print_status(ox, oy, oz, step, rtde_r=None):
    vals = sensor.get_values()
    center_val = vals[9] if len(vals) > 9 else 0.0
    tcp = rtde_r.getActualTCPPose() if rtde_r else [0]*6
    print(f"\n  ── Status ──────────────────────────────")
    print(f"  Offset   : X={ox:+7.3f}  Y={oy:+7.3f}  Z={oz:+7.3f} mm")
    print(f"  Step     : {step:.3f} mm")
    print(f"  TCP now  : X={tcp[0]:.5f}  Y={tcp[1]:.5f}  Z={tcp[2]:.5f}")
    print(f"  S26 (P10): {sensor_bar(center_val)}")
    all_vals = [f"S{[2,15,28,1,14,27,40,0,13,26,39,52,12,25,38,51,24,37,50][i]}={v:.2f}"
                for i,v in enumerate(vals) if v > 0.05]
    if all_vals:
        print(f"  Active   : {', '.join(all_vals)}")
    print(f"  ────────────────────────────────────────")

def do_press(rtde_c, rtde_r, ox, oy, oz):
    """Press down at current position and read peak sensor value"""
    surface = build_pose(ox, oy, oz)
    pressed = build_pose(ox, oy, oz - INDENT_MM)

    print(f"\n  Pressing {INDENT_MM}mm down...")
    rtde_c.moveL(pressed, VELOCITY_PRESS, ACCEL)
    time.sleep(0.8)

    # Read sensor multiple times and take peak
    readings = []
    for _ in range(8):
        vals = sensor.get_values()
        readings.append(vals[9] if len(vals) > 9 else 0.0)
        time.sleep(0.1)
    peak = max(readings)

    rtde_c.moveL(surface, VELOCITY_PRESS, ACCEL)

    all_vals = sensor.get_values()
    top3 = sorted(enumerate(all_vals), key=lambda x: -x[1])[:3]
    raw_cells = [2,15,28,1,14,27,40,0,13,26,39,52,12,25,38,51,24,37,50]
    print(f"  Peak S26 value: {sensor_bar(peak)}")
    print(f"  Top 3 sensors: {[(f'S{raw_cells[i]}', round(v,3)) for i,v in top3]}")

    if peak < 0.1:
        print(f"  WARNING: Low reading — pointer may be misaligned")
    elif peak > 0.5:
        print(f"  Good reading — well aligned!")
    else:
        print(f"  Moderate reading — try adjusting position")

def main():
    import argparse
    ap = argparse.ArgumentParser(description='UR5 global calibration')
    ap.add_argument('--tip', default=None,
                    help='Tip name (e.g. short, long_5mm). Saves to calib_<tip>.json')
    args = ap.parse_args()

    tip_label = f' [{args.tip}]' if args.tip else ''
    print("="*55)
    print(f"  UR5 Calibration Tool{tip_label}")
    print("  Target: align TCP over S26 (center of sensor)")
    print("="*55)
    if args.tip:
        print(f"  Tip profile : {args.tip}  →  calib_{args.tip}.json\n")
    print("""
  Commands:
    x+  x-         move right/left
    y+  y-         move forward/back
    z+  z-         move up/down
    step 0.5       set step size (mm)
    press          press and read sensor peak
    status         show current offset + sensor
    reset          clear offset to zero
    save           save calibration file
    quit           exit without saving
""")

    ox, oy, oz = load_existing(args.tip)
    step_mm = 0.5

    print("[calib] Starting sensor...")
    sensor.start()
    if not sensor.wait_until_ready(timeout=40):
        print("[calib] Sensor not ready — check USB connection")
        sys.exit(1)
    print("[calib] Sensor ready!")

    print("[calib] Connecting to UR5...")
    try:
        import rtde_control
        import rtde_receive
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        print("[calib] Connected!")
    except Exception as e:
        print(f"[calib] UR5 connection failed: {e}")
        sys.exit(1)

    print(f"\n[calib] Moving to P10 center (offset X={ox:+.2f} Y={oy:+.2f} Z={oz:+.2f})...")
    rtde_c.moveL(build_pose(ox, oy, oz), 0.05, ACCEL)
    print("[calib] At position. Start jogging.\n")

    print_status(ox, oy, oz, step_mm, rtde_r)

    while True:
        try:
            cmd = input("\n  > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n[calib] Interrupted")
            break

        moved = False

        if cmd == 'x+':
            ox += step_mm;  moved=True
        elif cmd == 'x-':
            ox -= step_mm;  moved=True
        elif cmd == 'y+':
            oy += step_mm;  moved=True
        elif cmd == 'y-':
            oy -= step_mm;  moved=True
        elif cmd == 'z+':
            oz += step_mm;  moved=True
        elif cmd == 'z-':
            oz -= step_mm;  moved=True
        elif cmd.startswith('step'):
            try:
                step_mm = float(cmd.split()[1])
                print(f"  Step set to {step_mm:.3f} mm")
            except:
                print(f"  Usage: step 0.5")
        elif cmd == 'press':
            do_press(rtde_c, rtde_r, ox, oy, oz)
        elif cmd == 'status':
            print_status(ox, oy, oz, step_mm, rtde_r)
        elif cmd == 'reset':
            ox=oy=oz=0.0; moved=True
            print("  Reset to zero offset")
        elif cmd == 'save':
            save_calib(ox, oy, oz, args.tip)
            rtde_c.stopScript()
            print("[calib] Done!")
            return
        elif cmd == 'quit':
            print("[calib] Quit without saving")
            break
        elif cmd == '':
            print_status(ox, oy, oz, step_mm, rtde_r)
        else:
            print(f"  Unknown command: '{cmd}'")
            print(f"  Try: x+ x- y+ y- z+ z- step N press status reset save quit")

        if moved:
            new_pose = build_pose(ox, oy, oz)
            rtde_c.moveL(new_pose, VELOCITY_JOG, ACCEL)
            print(f"  Moved to X={ox:+.3f} Y={oy:+.3f} Z={oz:+.3f} mm")
            vals = sensor.get_values()
            center_val = vals[9] if len(vals) > 9 else 0.0
            print(f"  S26 = {sensor_bar(center_val)}")

    rtde_c.stopScript()

if __name__ == "__main__":
    main()