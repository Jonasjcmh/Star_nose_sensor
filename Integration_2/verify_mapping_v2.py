"""
verify_mapping_v2.py
Verifies UR5 point→sensor mapping AND lets you pick which calibration file to use.

What this version adds over verify_mapping.py:
  1. Interactive menu to choose between the calibration profiles found on disk
     (calib_<tip>.json  +  calib_points_<tip>.json).
  2. Moves the robot to the INITIAL position (P10 centre, with the chosen
     calibration applied) and confirms it before pressing anything.
  3. Presses each of the 19 points using the SAME pose maths as ur5_control.py
     (global offset + per-point offset), so what you verify is what the
     experiment will actually do.
  4. Saves a JSON file with the robot's initial pose (commanded + measured)
     plus every point pose, for future implementations.

HOW TO USE:
  1. Run this script.
  2. Pick a calibration profile from the menu.
  3. Confirm the initial position on the robot.
  4. Optionally run the 19-point press/verify sweep.
  5. Read the mapping summary + saved initial-pose JSON.
"""

import os
import glob
import json
import time
from datetime import datetime

import sensor
import ur5_control
import load_calibration
import rtde_control
import rtde_receive

CALIB_DIR = os.path.dirname(os.path.abspath(__file__))

# Reuse the exact robot config + pose maths from ur5_control so verification
# matches the real experiment.
ROBOT_IP        = ur5_control.ROBOT_IP
VELOCITY_TRAVEL = ur5_control.VELOCITY_TRAVEL
VELOCITY_PRESS  = ur5_control.VELOCITY_PRESS
ACCELERATION    = ur5_control.ACCELERATION
DEFAULT_INDENT  = ur5_control.DEFAULT_INDENT_MM
POINTS          = ur5_control.POINTS
UR5_TO_SENSOR   = ur5_control.UR5_TO_SENSOR

DWELL_S = 1.0   # hold while reading the sensor


# ── Calibration profile discovery ────────────────────────────
def discover_profiles():
    """Find every calibration profile on disk.

    A profile is a `calib_<tip>.json` global-offset file (or the default
    `calib.json`). `calib_points_*.json` files are the per-point companions,
    not standalone profiles, so they are excluded here.

    Returns a list of dicts: {tip, label, global_file, points_file}
    sorted so the default profile comes first.
    """
    profiles = []
    for path in sorted(glob.glob(os.path.join(CALIB_DIR, "calib_*.json"))):
        name = os.path.basename(path)
        # Skip the per-point companion files (calib_points.json / calib_points_*.json)
        if name == "calib_points.json" or name.startswith("calib_points_"):
            continue
        tip = name[len("calib_"):-len(".json")]   # calib_short_6mm.json -> short_6mm
        pts = os.path.join(CALIB_DIR, f"calib_points_{tip}.json")
        profiles.append({
            "tip":         tip,
            "label":       tip,
            "global_file": path,
            "points_file": pts if os.path.exists(pts) else None,
        })

    # Default (calib.json / calib_points.json), tip = None
    default_global = os.path.join(CALIB_DIR, "calib.json")
    if os.path.exists(default_global):
        default_pts = os.path.join(CALIB_DIR, "calib_points.json")
        profiles.insert(0, {
            "tip":         None,
            "label":       "(default)",
            "global_file": default_global,
            "points_file": default_pts if os.path.exists(default_pts) else None,
        })
    return profiles


def _read_global(path):
    try:
        with open(path) as f:
            d = json.load(f)
        return d.get("x_mm", 0.0), d.get("y_mm", 0.0), d.get("z_mm", 0.0)
    except Exception:
        return None


def choose_profile(profiles):
    """Print the menu and return the profile the user selects."""
    print("\n" + "=" * 60)
    print("  AVAILABLE CALIBRATION PROFILES")
    print("=" * 60)
    for i, p in enumerate(profiles, 1):
        g = _read_global(p["global_file"])
        g_txt = (f"X={g[0]:+.2f} Y={g[1]:+.2f} Z={g[2]:+.2f} mm"
                 if g else "unreadable")
        pts_txt = "with per-point" if p["points_file"] else "global only "
        print(f"  {i:2d}) {p['label']:<16s}  {pts_txt}  {g_txt}")
    print("=" * 60)

    while True:
        try:
            raw = input(f"Select profile [1-{len(profiles)}] > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[verify] Aborted.")
            raise SystemExit(1)
        if raw.isdigit() and 1 <= int(raw) <= len(profiles):
            return profiles[int(raw) - 1]
        print("  Invalid choice, try again.")


# ── Pose helpers (delegate to ur5_control so maths stays identical) ──
def build_pose(pt, extra_z_mm=0.0):
    return ur5_control._build_pose(pt, extra_z_mm)


def initial_pose():
    """The initial position = P10 centre at the surface, with calibration."""
    return build_pose(10, 0.0)


def get_active_sensors(values, threshold=0.05):
    active = [(i, v) for i, v in enumerate(values) if v > threshold]
    return sorted(active, key=lambda x: -x[1])


def save_initial_pose(profile, commanded, measured, home, point_poses):
    """Write the robot's initial pose + all point poses to JSON."""
    tip = profile["tip"]
    fname = f"initial_pose_{tip}.json" if tip else "initial_pose_default.json"
    out_path = os.path.join(CALIB_DIR, fname)

    data = {
        "profile":        profile["label"],
        "tip":            tip,
        "timestamp":      datetime.now().isoformat(timespec="seconds"),
        "robot_ip":       ROBOT_IP,
        "calibration": {
            "global_file": os.path.basename(profile["global_file"]),
            "points_file": (os.path.basename(profile["points_file"])
                            if profile["points_file"] else None),
            "x_mm":        ur5_control.CALIB_X_MM,
            "y_mm":        ur5_control.CALIB_Y_MM,
            "z_mm":        ur5_control.CALIB_Z_MM,
        },
        "reference_pose":          ur5_control.REFERENCE_POSE,
        "initial_pose_commanded":  commanded,   # what we told the robot (P10)
        "initial_pose_measured":   measured,    # actual TCP read back
        "home_pose":               home,        # P10 lifted SAFE_HOME_Z_MM
        "point_poses": {str(pt): build_pose(pt, 0.0) for pt in range(1, 20)},
    }
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n[verify] Initial pose saved → {os.path.basename(out_path)}")
    return out_path


def main():
    print("=" * 60)
    print("  UR5 ↔ Sensor Mapping Verification  (v2 — pick calibration)")
    print("=" * 60)

    # ── 1. Pick calibration profile ───────────────────────────
    profiles = discover_profiles()
    if not profiles:
        print("[verify] No calib_*.json files found — nothing to verify.")
        return
    profile = choose_profile(profiles)
    print(f"\n[verify] Using profile: {profile['label']}")

    # Apply it exactly like main.py would (sets global + per-point offsets)
    load_calibration.apply(profile["tip"])

    # ── 2. Start sensor ───────────────────────────────────────
    print("\n[verify] Starting sensor...")
    sensor.start()
    print("[verify] Waiting for sensor calibration...")
    sensor.wait_until_ready(timeout=30)
    print("[verify] Sensor ready!\n")

    # ── 3. Connect UR5 ────────────────────────────────────────
    print("[verify] Connecting to UR5...")
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print("[verify] Connected!\n")

    # ── 4. Move to INITIAL position (P10 + calibration) ───────
    start_pose = initial_pose()
    print("Initial position (P10 centre, calibrated):")
    print(f"  X={start_pose[0]*1000:+.2f} Y={start_pose[1]*1000:+.2f} "
          f"Z={start_pose[2]*1000:+.2f} mm")
    input("Press ENTER to move to the INITIAL position...")
    rtde_c.moveL(start_pose, VELOCITY_TRAVEL, ACCELERATION)
    time.sleep(0.3)
    measured = rtde_r.getActualTCPPose()
    print("At initial position.")
    print(f"  Commanded : X={start_pose[0]:.4f} Y={start_pose[1]:.4f} Z={start_pose[2]:.4f}")
    print(f"  Measured  : X={measured[0]:.4f} Y={measured[1]:.4f} Z={measured[2]:.4f}")

    # Save initial-pose JSON now (independent of running the full sweep)
    home = ur5_control._home_pose()
    save_initial_pose(profile, start_pose, list(measured), home,
                      {pt: build_pose(pt, 0.0) for pt in range(1, 20)})

    # ── 5. Verify the 19 points (optional) ────────────────────
    try:
        ans = input("\nRun the 19-point press/verify sweep? [y/N] > ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = "n"

    results = {}
    if ans == "y":
        input("Press ENTER to start the verification sequence...")
        print()
        for pt in range(1, 20):
            surface = build_pose(pt, 0.0)
            pressed = build_pose(pt, -DEFAULT_INDENT)
            expected = UR5_TO_SENSOR.get(pt)
            print(f"── P{pt:02d} ({POINTS[pt][0]:+.0f},{POINTS[pt][1]:+.0f})mm "
                  f"expect S{expected} ──")

            rtde_c.moveL(surface, VELOCITY_TRAVEL, ACCELERATION)
            rtde_c.moveL(pressed, VELOCITY_PRESS, ACCELERATION)

            time.sleep(0.5)  # settle
            readings = []
            for _ in range(5):
                readings.append(sensor.get_values())
                time.sleep(0.1)
            n = len(readings)
            avg = [sum(r[i] for r in readings) / n for i in range(19)]
            active = get_active_sensors(avg, threshold=0.05)

            if active:
                top_idx, top_val = active[0]
                ok = "✓" if top_idx == expected else "✗"
                print(f"  → Top sensor: index {top_idx} (val={top_val:.3f}) {ok}")
                print(f"  → All active: {[(i, round(v, 3)) for i, v in active[:4]]}")
                results[pt] = top_idx
            else:
                print("  → No sensor activated!")
                results[pt] = None

            rtde_c.moveL(surface, VELOCITY_PRESS, ACCELERATION)
            print()

    # ── 6. Return home + summary ──────────────────────────────
    rtde_c.moveL(home, VELOCITY_TRAVEL, ACCELERATION)
    rtde_c.stopScript()

    if results:
        print("\n" + "=" * 60)
        print("  MAPPING RESULTS")
        print("=" * 60)
        print("\nUR5_TO_SENSOR = {")
        for pt in range(1, 20):
            idx = results.get(pt)
            dx, dy = POINTS[pt]
            exp = UR5_TO_SENSOR.get(pt)
            flag = "" if idx == exp else f"  (expected {exp})"
            print(f"    {pt:2d}: {str(idx):4s},  # P{pt:02d} ({dx:+.0f},{dy:+.0f})mm{flag}")
        print("}")

    print("\n[verify] Done.")


if __name__ == "__main__":
    main()
