"""
verify_mapping_robot.py
Robot-only mapping check — NO muca board required.

Drives the UR5 to each calibration point so you can VISUALLY confirm the tip
lands on the correct pad. Poses are built with the SAME maths as the real
experiment (the calib file's reference_pose + global offset + per-point
deviation), so what you see is what the experiment will actually do. Unlike
verify_mapping.py / verify_mapping_v2.py this never reads the sensor, so you can
check the geometry with the board disconnected.

The surface pose (extra_z=0) IS the calibrated sensor-surface contact taken
from the calib file. You are prompted for an indentation depth (mm) to press
BELOW that surface (or pass --indent to skip the prompt). For each point it:
  • moves to the calibrated surface-contact pose,
  • presses the indentation depth below it (so the tip indents the sensor),
  • dwells --dwell s (or waits for ENTER in --step mode) so you can look,
  • lifts back to the surface and moves to the next point.

The pose maths, calib-file loading and point/label tables are reused verbatim
from verify_mapping.py + calibrate_points.py (single source of truth), so this
can never drift from the sensor-based verifier or the collectors.

USAGE
  python3 verify_mapping_robot.py                          # pick calib, tour all 19
  python3 verify_mapping_robot.py --calib calib_points_short_new_hollow_sensor.json
  python3 verify_mapping_robot.py --points 1,10,19         # only these pads (or a1,c3,e5)
  python3 verify_mapping_robot.py --indent 5 --dwell 2     # press 5 mm, hold 2 s each
  python3 verify_mapping_robot.py --step                   # wait for ENTER between points
"""

import os
import sys
import time
import argparse

import calibrate_points as cp
import verify_mapping as vm   # reuse select_calib / load_calib / build_pose

CALIB_DIR = os.path.dirname(os.path.abspath(__file__))

VELOCITY_TRAVEL = cp.VELOCITY_TRAVEL
VELOCITY_PRESS  = cp.VELOCITY_PRESS
ACCELERATION    = cp.ACCELERATION


def _ask_float(prompt, default, minimum, maximum):
    while True:
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            return default
        if raw == '':
            return default
        try:
            val = float(raw)
            if minimum <= val <= maximum:
                return val
        except ValueError:
            pass
        print(f'  Enter a number between {minimum:.1f} and {maximum:.1f}')


def _parse_points(raw):
    """Comma-separated pad numbers (1..19) or hex labels (a1..e5) -> [int]."""
    label_to_pt = {lbl: pt for pt, lbl in cp.UR5_TO_LABEL.items()}
    pts = []
    for tok in raw.split(','):
        tok = tok.strip().lower()
        if tok == '':
            continue
        if tok.isdigit():
            pt = int(tok)
        elif tok in label_to_pt:
            pt = label_to_pt[tok]
        else:
            raise ValueError(f"'{tok}' is not a pad number (1-19) or label (a1-e5)")
        if pt not in cp.POINTS:
            raise ValueError(f"point {pt} out of range (1-19)")
        pts.append(pt)
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=None,
                    help="calib_points_*.json to use (skips the prompt)")
    ap.add_argument("--points", default=None,
                    help="comma-separated pads to visit, e.g. 1,10,19 or a1,c3,e5 "
                         "(default: all 19)")
    ap.add_argument("--indent", type=float, default=None,
                    help="indentation depth in mm below the calibrated sensor surface "
                         "(skips the prompt; interactive default 5)")
    ap.add_argument("--dwell", type=float, default=1.5,
                    help="seconds to hold at each point for inspection (default 1.5)")
    ap.add_argument("--step", action="store_true",
                    help="wait for ENTER between points instead of dwelling")
    args = ap.parse_args()

    print("=" * 60)
    print("  UR5 Mapping Check — ROBOT ONLY (no sensor, a1..e5 scheme)")
    print("=" * 60)

    # ── Calib file ────────────────────────────────────────────────────────────
    calib_path = args.calib
    if calib_path and not os.path.isabs(calib_path):
        calib_path = os.path.join(CALIB_DIR, calib_path)
    if not calib_path:
        calib_path = vm.select_calib()
    if not os.path.exists(calib_path):
        print(f"[verify] No such calib file: {calib_path}")
        sys.exit(1)
    ref, gxyz, per = vm.load_calib(calib_path)

    try:
        points = _parse_points(args.points) if args.points else list(range(1, 20))
    except ValueError as e:
        print(f"[verify] --points: {e}")
        sys.exit(1)

    # extra_z=0 in build_pose IS the calibrated sensor-surface contact (the calib
    # file's reference_pose + global + per-point). The indentation presses that
    # many mm BELOW that surface, so the tip contacts / indents the sensor.
    indent = args.indent if args.indent is not None else _ask_float(
        "  Indentation depth (mm) below the calibrated sensor surface [5] > ",
        5.0, 0.0, 30.0)

    print(f"\n[verify] Calib      : {os.path.basename(calib_path)}")
    print(f"[verify] Global     : X={gxyz[0]:+.2f} Y={gxyz[1]:+.2f} Z={gxyz[2]:+.2f} mm")
    print(f"[verify] Per-point  : {len(per)} deviation(s) loaded")
    print(f"[verify] Indent     : {indent:.1f} mm below calibrated surface   "
          f"({'press' if indent > 0 else 'surface only'})")
    print(f"[verify] Points     : {len(points)}  ->  "
          + ", ".join(f"P{p:02d}/{cp.UR5_TO_LABEL[p]}" for p in points))

    # ── Robot ─────────────────────────────────────────────────────────────────
    import ur5_control
    print("\n[verify] Connecting to UR5 ...")
    rtde_c, _ = ur5_control.connect()
    if rtde_c is None:
        print("[verify] Robot connect FAILED — aborting.")
        sys.exit(1)

    try:
        input("\nPress ENTER to move to P10 (c3 centre, lifted) ...")
        rtde_c.moveL(vm.build_pose(10, ref, gxyz, per, extra_z=cp.SAFE_Z_MM),
                     VELOCITY_TRAVEL, ACCELERATION)
        rtde_c.moveL(vm.build_pose(10, ref, gxyz, per, 0.0),
                     VELOCITY_TRAVEL, ACCELERATION)
        print("At P10 (c3) surface.")
        input("Press ENTER to start the point tour ...")

        for i, pt in enumerate(points):
            label  = cp.UR5_TO_LABEL[pt]
            px, py = cp.POINTS[pt]
            print(f"\n-- {i + 1}/{len(points)}  P{pt:02d} ({label})  "
                  f"grid=({px:+.0f},{py:+.0f})mm --")

            rtde_c.moveL(vm.build_pose(pt, ref, gxyz, per, 0.0),
                         VELOCITY_TRAVEL, ACCELERATION)
            if indent > 0:
                rtde_c.moveL(vm.build_pose(pt, ref, gxyz, per, -indent),
                             VELOCITY_PRESS, ACCELERATION)

            if args.step:
                try:
                    input("   Press ENTER for the next point ...")
                except (EOFError, KeyboardInterrupt):
                    print("\n[verify] Interrupted — returning home.")
                    break
            else:
                time.sleep(args.dwell)

            if indent > 0:
                rtde_c.moveL(vm.build_pose(pt, ref, gxyz, per, 0.0),
                             VELOCITY_PRESS, ACCELERATION)
    except KeyboardInterrupt:
        print("\n[verify] Interrupted — returning home.")
    finally:
        try:
            rtde_c.moveL(vm.build_pose(10, ref, gxyz, per, extra_z=cp.SAFE_Z_MM),
                         VELOCITY_TRAVEL, ACCELERATION)
        except Exception:
            pass
        for meth in ("stopScript", "disconnect"):
            try:
                getattr(rtde_c, meth)()
            except Exception:
                pass

    print("\n[verify] Done — tip visited "
          f"{len(points)} point(s). Confirm each landed on its pad.")


if __name__ == "__main__":
    main()
    # ur_rtde's C++ interfaces can throw from their destructors at interpreter
    # shutdown AFTER all work is done. Flush and hard-exit to skip those.
    sys.stdout.flush()
    os._exit(0)
