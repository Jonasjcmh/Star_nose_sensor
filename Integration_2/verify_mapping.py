"""
verify_mapping.py
Verify that UR5 point numbers map to the correct sensor cells, using a chosen
calibration and the SAME pose maths as the real experiment.

Run this BEFORE a full experiment to confirm the mapping.

WHAT IT DOES
  1. Asks which calib_points_*.json to use (the file carries reference_pose +
     global offset + per-point deviations). --calib skips the prompt.
  2. For every UR5 point 1..19 it builds the pose exactly like
     calibrate_points.build_pose (reference pose + global + per-point deviation),
     presses INDENT mm, reads the sensor, and checks whether the cell that fires
     is the EXPECTED one for that point.
  3. Prints a per-point ✓/✗ and a mapping summary.

Everything (numbering, labels, sensor-cell mapping, reference pose) comes from
calibrate_points.py — the single source of truth shared with calibrate_live.py,
visualizer_2d.py and the interdome collector. Points are numbered in a1..e5
label order: P1=a1 ... P10=c3 (centre) ... P19=e5.

USAGE
  python3 verify_mapping.py                       # pick calib interactively
  python3 verify_mapping.py --calib calib_points_short_new_hollow_sensor.json
  python3 verify_mapping.py --indent 8            # press depth (mm), default 10
  python3 verify_mapping.py --no-robot            # sensor-only dry run (no UR5)
"""

import os
import sys
import glob
import json
import time
import argparse

import calibrate_points as cp

CALIB_DIR = os.path.dirname(os.path.abspath(__file__))
_MUCA_RAW = os.path.normpath(os.path.join(CALIB_DIR, "..", "mucaboard_data_raw"))

VELOCITY_TRAVEL = cp.VELOCITY_TRAVEL
VELOCITY_PRESS  = cp.VELOCITY_PRESS
ACCELERATION    = cp.ACCELERATION


# ── Calibration file selection / loading ──────────────────────────────────────
def list_calib_points_files():
    """[(name, path), ...] for every calib_points_*.json (+ calib_points.json)."""
    files = []
    default = os.path.join(CALIB_DIR, "calib_points.json")
    if os.path.exists(default):
        files.append(("(default)", default))
    for path in sorted(glob.glob(os.path.join(CALIB_DIR, "calib_points_*.json"))):
        files.append((os.path.basename(path)[len("calib_points_"):-len(".json")], path))
    return files


def _summarise(path):
    try:
        with open(path) as f:
            d = json.load(f)
    except Exception:
        return 0, None
    g = d.get("global") or {}
    gxyz = ((g.get("x_mm", 0.0), g.get("y_mm", 0.0), g.get("z_mm", 0.0)) if g else None)
    return len(d.get("points") or d.get("per_point") or {}), gxyz


def select_calib():
    files = list_calib_points_files()
    if not files:
        print(f"[verify] No calib_points_*.json in {CALIB_DIR}")
        sys.exit(1)
    print("\n  Available calibration-points files:")
    for i, (name, path) in enumerate(files):
        n, g = _summarise(path)
        g_txt = (f"X={g[0]:+.2f} Y={g[1]:+.2f} Z={g[2]:+.2f} mm" if g else "no global")
        print(f"    [{i}]  {name:30s}  {n:2d} pts  {g_txt}")
    while True:
        try:
            idx = int(input(f"\n  Select calib file [0-{len(files)-1}] > ").strip())
            if 0 <= idx < len(files):
                return files[idx][1]
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
        print(f"  Enter a number between 0 and {len(files)-1}")


def load_calib(path):
    """Return (reference_pose, (gx,gy,gz), {pt:(dx,dy)}) from a calib file.
    Per-point deviations come from per_point.dx/dy, else points[].deviation_mm."""
    with open(path) as f:
        d = json.load(f)
    ref = d.get("reference_pose") or list(cp.REFERENCE_POSE)
    g   = d.get("global") or {}
    gxyz = (g.get("x_mm", 0.0), g.get("y_mm", 0.0), g.get("z_mm", 0.0))
    per = {}
    if d.get("per_point"):
        for k, v in d["per_point"].items():
            per[int(k)] = (v.get("dx_mm", 0.0), v.get("dy_mm", 0.0))
    elif d.get("points"):
        for k, v in d["points"].items():
            dv = v.get("deviation_mm", [0.0, 0.0])
            per[int(k)] = (dv[0], dv[1])
    if ref != list(cp.REFERENCE_POSE):
        print("[verify] NOTE: using reference_pose from the calib file "
              "(differs from calibrate_points.REFERENCE_POSE).")
    return ref, gxyz, per


def build_pose(pt, ref, gxyz, per, extra_z=0.0):
    """Same maths as calibrate_points.build_pose, but honouring the calib file's
    own reference_pose: ref + grid point + global XY + per-point deviation."""
    gx, gy, gz = gxyz
    px, py     = cp.POINTS[pt]
    dx, dy     = per.get(pt, (0.0, 0.0))
    pose = list(ref)
    pose[0] += (px + gx + dx) / 1000.0
    pose[1] += (py + gy + dy) / 1000.0
    pose[2] += (gz + extra_z) / 1000.0
    return pose


# ── Verification sweep ────────────────────────────────────────────────────────
def capture_response(sen, n=10, dwell=0.08):
    """Peak sensor response per cell over n samples, baseline-subtracted when the
    reader exposes a baseline (raw muca mode). Peak (not average) matches
    calibrate_points.do_press and is robust to a single noisy neighbour.
    Returns a 19-element response vector."""
    rows = []
    for _ in range(n):
        rows.append(sen.get_values())
        time.sleep(dwell)
    peak = [max(r[i] for r in rows) for i in range(19)]
    base_fn = getattr(sen, "get_calibration", None)
    base = (base_fn() if base_fn is not None else None)
    if base is not None:
        return [peak[i] - float(base[i]) for i in range(19)]
    return peak


def idle_report(sen, thr=0.05):
    """Read the sensor with NOTHING pressed and report how many cells are already
    active. If several are, the field is not resting quiet (bad baseline / stuck
    cells / crosstalk) and mapping results will be unreliable no matter how good
    the robot poses are — this is the first thing to check when points map to a
    neighbour."""
    resp   = capture_response(sen, n=8)
    active = sorted(((cp.RAW_CELLS[i], resp[i]) for i in range(19) if resp[i] > thr),
                    key=lambda t: -t[1])
    print(f"[verify] idle: {len(active)}/19 cells active (>{thr}), max={max(resp):.3f}")
    if active:
        hot = ", ".join(f"S{c}({cp.SENSOR_CELL_TO_LABEL.get(c, '?')})={v:.2f}"
                        for c, v in active[:6])
        print(f"[verify]   hot: {hot}")
    if len(active) >= 4:
        print("[verify]   ⚠ MANY cells active with nothing pressed — the field is "
              "not resting quiet.\n"
              "[verify]     A single press should light ~1-3 cells; if the whole "
              "area is hot, the\n"
              "[verify]     winner is just the strongest neighbour. Re-baseline with "
              "the tip OFF the\n"
              "[verify]     sensor, lower SENSITIVITY, and/or press shallower "
              "(--indent 3-4). Mapping\n"
              "[verify]     results are unreliable until idle is quiet.")
    return resp


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=None,
                    help="calib_points_*.json to use (skips the prompt)")
    ap.add_argument("--indent", type=float, default=cp.INDENT_MM,
                    help=f"press depth in mm (default {cp.INDENT_MM})")
    ap.add_argument("--no-robot", action="store_true",
                    help="sensor-only dry run (no UR5 motion)")
    ap.add_argument("--raw", action="store_true",
                    help="read via the RAW muca-board reader "
                         "(mucaboard_data_raw/sensor_raw.py, same as the interdome) "
                         "instead of the normalised Integration_2/sensor.py")
    args = ap.parse_args()

    print("=" * 60)
    print("  UR5 <-> Sensor Mapping Verification  (a1..e5 scheme)")
    print("=" * 60)

    calib_path = args.calib
    if calib_path and not os.path.isabs(calib_path):
        calib_path = os.path.join(CALIB_DIR, calib_path)
    if not calib_path:
        calib_path = select_calib()
    if not os.path.exists(calib_path):
        print(f"[verify] No such calib file: {calib_path}")
        sys.exit(1)
    ref, gxyz, per = load_calib(calib_path)
    print(f"\n[verify] Calib      : {os.path.basename(calib_path)}")
    print(f"[verify] Global     : X={gxyz[0]:+.2f} Y={gxyz[1]:+.2f} Z={gxyz[2]:+.2f} mm")
    print(f"[verify] Per-point  : {len(per)} deviation(s) loaded")
    print(f"[verify] Indent     : {args.indent:.1f} mm")

    # ── Sensor ────────────────────────────────────────────────────────────────
    if args.raw:
        sys.path.insert(0, _MUCA_RAW)
        import sensor_raw as sen
        print("\n[verify] Starting sensor (RAW muca-board ADC counts) ...")
    else:
        import sensor as sen
        print("\n[verify] Starting sensor (normalised) ...")
    sen.start()
    if not sen.wait_until_ready(timeout=30):
        print("[verify] Sensor not ready — aborting.")
        sys.exit(1)
    print("[verify] Sensor ready!")

    # ── Robot ─────────────────────────────────────────────────────────────────
    rtde_c = None
    if not args.no_robot:
        import ur5_control
        print("\n[verify] Connecting to UR5 ...")
        rtde_c, _ = ur5_control.connect()
        if rtde_c is None:
            print("[verify] Robot connect FAILED — aborting (use --no-robot to skip).")
            sys.exit(1)
        input("\nPress ENTER to move to P10 (c3 centre, lifted) ...")
        rtde_c.moveL(build_pose(10, ref, gxyz, per, extra_z=cp.SAFE_Z_MM),
                     VELOCITY_TRAVEL, ACCELERATION)
        print("\n[verify] Idle baseline check (tip lifted, nothing pressed):")
        idle_report(sen)
        rtde_c.moveL(build_pose(10, ref, gxyz, per, 0.0), VELOCITY_TRAVEL, ACCELERATION)
        print("At P10 (c3) surface.")
        input("Press ENTER to start the 19-point verification sweep ...")
    else:
        print("\n[verify] Idle baseline check (nothing pressed):")
        idle_report(sen)

    results = {}
    n_correct = 0
    for pt in range(1, 20):
        label   = cp.UR5_TO_LABEL[pt]
        exp_idx = cp.UR5_TO_IDX[pt]
        exp_raw = cp.UR5_TO_RAW[pt]
        print(f"\n-- P{pt:02d} ({label})  expect S{exp_raw} --")

        if rtde_c is not None:
            rtde_c.moveL(build_pose(pt, ref, gxyz, per, 0.0), VELOCITY_TRAVEL, ACCELERATION)
            rtde_c.moveL(build_pose(pt, ref, gxyz, per, -args.indent), VELOCITY_PRESS, ACCELERATION)
            time.sleep(0.5)

        resp    = capture_response(sen)
        top_idx = max(range(19), key=lambda i: resp[i])
        top_raw = cp.RAW_CELLS[top_idx]
        top_lbl = cp.SENSOR_CELL_TO_LABEL.get(top_raw, "?")
        exp_val = resp[exp_idx]
        correct = (top_idx == exp_idx)
        n_correct += correct
        results[pt] = top_raw

        mark = "OK  ✓" if correct else "WRONG ✗"
        print(f"   expected S{exp_raw:2d}({label}) resp={exp_val:.3f}  |  "
              f"top S{top_raw:2d}({top_lbl}) resp={resp[top_idx]:.3f}   {mark}")

        if rtde_c is not None:
            rtde_c.moveL(build_pose(pt, ref, gxyz, per, 0.0), VELOCITY_PRESS, ACCELERATION)

    if rtde_c is not None:
        rtde_c.moveL(build_pose(10, ref, gxyz, per, extra_z=cp.SAFE_Z_MM),
                     VELOCITY_TRAVEL, ACCELERATION)
        for meth in ("stopScript", "disconnect"):
            try:
                getattr(rtde_c, meth)()
            except Exception:
                pass

    # ── Summary ───────────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print(f"  MAPPING SUMMARY — {n_correct}/19 correct")
    print("=" * 60)
    print(f"  {'UR5':>4} {'label':>6} {'expect':>7} {'observed':>9}  result")
    for pt in range(1, 20):
        label   = cp.UR5_TO_LABEL[pt]
        exp_raw = cp.UR5_TO_RAW[pt]
        got_raw = results.get(pt)
        got_lbl = cp.SENSOR_CELL_TO_LABEL.get(got_raw, "?")
        ok      = (got_raw == exp_raw)
        print(f"  P{pt:02d} {label:>6} {('S'+str(exp_raw)):>7} "
              f"{('S'+str(got_raw)+'('+got_lbl+')'):>9}  {'✓' if ok else '✗ WRONG'}")
    if n_correct == 19:
        print("\n  All points map correctly ✓")
    else:
        print(f"\n  ⚠ {19 - n_correct} mismatch(es) — check calibration / tip alignment.")


if __name__ == "__main__":
    main()
    # ur_rtde's C++ interfaces can throw from their destructors at interpreter
    # shutdown ("terminate called ... / Aborted (core dumped)") AFTER all work is
    # done. Flush and hard-exit to skip those destructors so the run ends clean.
    sys.stdout.flush()
    os._exit(0)
