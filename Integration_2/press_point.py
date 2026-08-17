"""
press_point.py
Interactive single-point press tool for the UR5.

Ask for a point by its hex-grid label (a1..e5) — the SAME names the sensor
and visualizer_2d use — or by hardware number (1..19), and the robot moves to
that point, presses it, dwells, and retracts. Points map identically to the
sensor: P1=a1, P2=a2, P3=a3, P4=b1 ... P16=d5, P17=e3, P18=e4, P19=e5, so
pressing 'a1' hits the exact point the sensor labels a1.

Usage:
    python press_point.py [--tip NAME] [--indent MM] [--dwell S]

If --tip is omitted you'll be shown the available calibration profiles and
asked to pick one (the per-point offsets live in calib_points_<tip>.json).
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ur5_control
import load_calibration

CALIB_DIR = os.path.dirname(os.path.abspath(__file__))


def _available_tips():
    """Tip names for which a calib_points_<tip>.json exists, sorted."""
    tips = []
    for path in glob.glob(os.path.join(CALIB_DIR, 'calib_points_*.json')):
        name = os.path.basename(path)[len('calib_points_'):-len('.json')]
        tips.append(name)
    return sorted(tips)


def _choose_tip():
    """Prompt the user to pick a calibration profile (the 'calib file for the
    points'). Returns the tip name, or None for the default calib_points.json."""
    tips = _available_tips()
    print("\nAvailable calibration profiles (calib_points_<tip>.json):")
    print("   0) (default) calib_points.json")
    for i, t in enumerate(tips, 1):
        print(f"  {i:2d}) {t}")
    while True:
        try:
            sel = input("Choose calibration profile [number] > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            raise SystemExit(1)
        if sel == '' or sel == '0':
            return None
        if sel.isdigit() and 1 <= int(sel) <= len(tips):
            return tips[int(sel) - 1]
        print("  Invalid choice — enter a number from the list.")


def _press(rtde_c, pt, indent_mm, dwell_s):
    """Move to point pt, press by indent_mm, dwell, retract to surface."""
    label = ur5_control.point_label(pt)
    dx, dy = ur5_control.POINTS[pt]
    surface = ur5_control._build_pose(pt, 0.0)
    pressed = ur5_control._build_pose(pt, -indent_mm)
    print(f"\n[press] Point {label} (P{pt}) XY=({dx:+.0f},{dy:+.0f})mm "
          f"indent={indent_mm:.1f}mm dwell={dwell_s:.1f}s")
    rtde_c.moveL(surface, ur5_control.VELOCITY_TRAVEL, ur5_control.ACCELERATION)
    with ur5_control._lock:
        ur5_control.current_point = pt
        ur5_control.is_pressing = True
    rtde_c.moveL(pressed, ur5_control.VELOCITY_PRESS, ur5_control.ACCELERATION)
    import time
    time.sleep(dwell_s)
    rtde_c.moveL(surface, ur5_control.VELOCITY_PRESS, ur5_control.ACCELERATION)
    with ur5_control._lock:
        ur5_control.is_pressing = False
    print(f"[press] Done — {label} released, at surface")


def main():
    ap = argparse.ArgumentParser(description="Press a single UR5 point by label (a1..e5) or number.")
    ap.add_argument('--tip', default=None,
                    help='Calibration profile name (loads calib_<tip>.json / calib_points_<tip>.json)')
    ap.add_argument('--indent', type=float, default=ur5_control.DEFAULT_INDENT_MM,
                    help=f'Press depth in mm (default {ur5_control.DEFAULT_INDENT_MM})')
    ap.add_argument('--dwell', type=float, default=ur5_control.DEFAULT_DWELL_S,
                    help=f'Dwell time in s (default {ur5_control.DEFAULT_DWELL_S})')
    args = ap.parse_args()

    # ── Calibration (ask for the calib file for the points) ──────
    tip = args.tip if args.tip is not None else _choose_tip()
    load_calibration.preview(tip=tip)
    load_calibration.apply(tip=tip)

    # ── Connect ──────────────────────────────────────────────────
    print("\n[press] Connecting to robot...")
    rtde_c, rtde_r = ur5_control.connect()
    if rtde_c is None:
        print("[press] Could not connect to robot — aborting.")
        raise SystemExit(1)

    # ── Move to safe home, then interactive press loop ───────────
    try:
        print("[press] Moving to home position...")
        rtde_c.moveL(ur5_control._home_pose(),
                     ur5_control.VELOCITY_TRAVEL, ur5_control.ACCELERATION)

        print("\n" + "=" * 55)
        print("  Enter a point to press: label a1..e5, or number 1..19")
        print("  'list' to show the mapping, 'q' to quit.")
        print("=" * 55)

        while True:
            try:
                token = input("point > ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if token.lower() in ('q', 'quit', 'exit'):
                break
            if token.lower() in ('list', 'l', '?'):
                for n in range(1, 20):
                    print(f"  {ur5_control.point_label(n):>3} = P{n}")
                continue
            if not token:
                continue
            try:
                pt = ur5_control.resolve_point(token)
            except ValueError as e:
                print(f"  {e}")
                continue
            try:
                _press(rtde_c, pt, args.indent, args.dwell)
            except Exception as e:
                print(f"[press] Error pressing point: {e}")
    finally:
        # ── Always return home + release control ─────────────────
        try:
            print("\n[press] Returning to home position...")
            rtde_c.moveL(ur5_control._home_pose(),
                         ur5_control.VELOCITY_TRAVEL, ur5_control.ACCELERATION)
        except Exception as e:
            print(f"[press] Could not return home: {e}")
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        print("[press] Closed")


if __name__ == "__main__":
    main()
