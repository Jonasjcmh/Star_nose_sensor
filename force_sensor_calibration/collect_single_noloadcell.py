"""
collect_single_noloadcell.py — collect UR5 fz data for ONE weight, using
the UR robot ONLY (no FUTEK load cell / analog reading).

Same flow as collect_single.py, but it records only the UR5's built-in
force/torque sensor — there is no external load cell and no ai0 channel.

One easy command:

    python collect_single_noloadcell.py

It connects to the UR5, moves wrist 2 to +90 deg (the +z direction),
checks the tool is vertical (and asks to fix it if it is off), then asks
you for the weight, zeroes the force sensor, records a short no-load
baseline, and holds while you place the weight and logs the samples. When
the run is done it rotates wrist 2 back to -90 deg (the -z direction).

One run = one weight = one CSV. Run it again for the next weight.

    # skip the weight prompt by passing it directly:
    python collect_single_noloadcell.py --weight 200
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

# ur5_control lives in Integration_2 (ROBOT_IP).
INTEGRATION_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "Integration_2")
sys.path.insert(0, os.path.abspath(INTEGRATION_DIR))

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None

CALIB_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(CALIB_DIR, "logs")

WRIST2_JOINT_IDX = 4  # [base, shoulder, elbow, wrist1, wrist2, wrist3]

# Wrist 2 targets: +90 deg points the tool +z (collection pose),
# -90 deg points it -z (the rest/start pose we return to afterward).
WRIST2_POS_Z_DEG = 90.0
WRIST2_NEG_Z_DEG = -90.0


# ── Rotation helpers ───────────────────────────────────────────────────
def rotvec_to_R(rotvec):
    """UR axis-angle rotation vector [rx, ry, rz] (rad) -> 3x3 rotation matrix."""
    rv = np.asarray(rotvec, dtype=float)
    theta = np.linalg.norm(rv)
    if theta < 1e-9:
        return np.eye(3)
    k = rv / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def R_to_rotvec(R):
    """3x3 rotation matrix -> UR axis-angle rotation vector [rx, ry, rz] (rad)."""
    R = np.asarray(R, dtype=float)
    cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * np.sin(theta))
    return theta * axis


def tilt_from_vertical_deg(rotvec):
    """Angle (deg) between the tool Z-axis and vertical — 0 = perfectly vertical."""
    R = rotvec_to_R(rotvec)
    tool_z = R @ np.array([0.0, 0.0, 1.0])
    cos_angle = np.clip(abs(tool_z[2]), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def level_pose_rotvec(rotvec):
    """Minimal rotation that makes the tool Z-axis exactly vertical.

    Keeps the tool pointing in the same hemisphere and preserves the spin
    about the tool Z, so the fix is a small tilt correction (not a big swing).
    """
    R = rotvec_to_R(rotvec)
    tool_z = R @ np.array([0.0, 0.0, 1.0])
    target = np.array([0.0, 0.0, 1.0 if tool_z[2] >= 0 else -1.0])
    v = np.cross(tool_z, target)
    s = np.linalg.norm(v)
    c = float(np.dot(tool_z, target))
    if s < 1e-9:
        return np.asarray(rotvec, dtype=float)  # already aligned
    k = v / s
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    R_align = np.eye(3) + s * K + (1 - c) * (K @ K)
    return R_to_rotvec(R_align @ R)


# ── Wrist 2 positioning ────────────────────────────────────────────────
def move_wrist2_to_deg(rtde_c, rtde_r, target_deg, speed=0.3, accel=0.5):
    """Move wrist 2 to an absolute angle (deg), leaving the other joints alone."""
    q = rtde_r.getActualQ()
    q[WRIST2_JOINT_IDX] = math.radians(target_deg)
    print(f"[collect] Moving wrist 2 to {target_deg:+.0f} deg...")
    rtde_c.moveJ(q, speed, accel)


# ── Verticality check (ask & fix if off) ───────────────────────────────
def ensure_vertical(rtde_c, rtde_r, tilt_tol_deg):
    """Check the tool is vertical; if it is off, offer to fix it, else abort.

    Returns the final tilt (deg) once it is within tolerance.
    """
    pose = rtde_r.getActualTCPPose()
    tilt = tilt_from_vertical_deg(pose[3:6])
    print(f"[collect] Tool tilt from vertical: {tilt:.3f} deg "
          f"(want <= {tilt_tol_deg:.2f} deg)")
    if tilt <= tilt_tol_deg:
        return tilt

    print(f"[collect] Tool is {tilt:.2f} deg off vertical — that's not right.")
    ans = input("[collect] Fix it? Robot will rotate the tool to vertical "
                "(holding position). Enter = fix, 'n' = abort: ").strip().lower()
    if ans == "n":
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        sys.exit("[collect] ABORT — tool not vertical.")

    level_pose = list(pose[:3]) + list(level_pose_rotvec(pose[3:6]))
    rtde_c.moveL(level_pose, 0.1, 0.3)
    time.sleep(0.5)
    pose = rtde_r.getActualTCPPose()
    tilt = tilt_from_vertical_deg(pose[3:6])
    print(f"[collect] Tool tilt after fixing: {tilt:.3f} deg")
    return tilt


# ── One CSV row ────────────────────────────────────────────────────────
# ai0 (FUTEK load cell) is still logged for compatibility, but this run
# does not depend on it — it's here just to keep the same CSV schema.
CSV_FIELDS = ["timestamp", "datetime", "weight_g", "loaded",
              "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
              "fx", "fy", "fz", "tx", "ty", "tz", "ai0"]


def _row(rtde_r, loaded, weight_g):
    ft = rtde_r.getActualTCPForce()
    tcp = rtde_r.getActualTCPPose()
    ai0 = rtde_r.getStandardAnalogInput0()
    now = time.time()
    return {
        "timestamp": now,
        "datetime": datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "weight_g": weight_g,
        "loaded": int(loaded),
        "tcp_x": tcp[0], "tcp_y": tcp[1], "tcp_z": tcp[2],
        "tcp_rx": tcp[3], "tcp_ry": tcp[4], "tcp_rz": tcp[5],
        "fx": ft[0], "fy": ft[1], "fz": ft[2],
        "tx": ft[3], "ty": ft[4], "tz": ft[5],
        "ai0": float(ai0),
    }


def ask_weight(cli_weight):
    """Use --weight if given, otherwise prompt the user."""
    if cli_weight is not None:
        return float(cli_weight)
    while True:
        raw = input("\nEnter weight in grams (e.g. 200): ").strip()
        try:
            w = float(raw)
            if w <= 0:
                print("  Weight must be > 0.")
                continue
            return w
        except ValueError:
            print(f"  '{raw}' is not a number — try again.")


def collect_one(args):
    if rtde_control is None or rtde_receive is None:
        sys.exit("ur_rtde is not installed — "
                  "pip install ur-rtde --break-system-packages")

    import ur5_control
    robot_ip = args.robot_ip or ur5_control.ROBOT_IP

    print(f"[collect] Connecting to {robot_ip} ...")
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    print("[collect] Connected.")

    # 1) Move wrist 2 to +90 deg (+z direction), 2) check vertical (ask & fix if off).
    move_wrist2_to_deg(rtde_c, rtde_r, WRIST2_POS_Z_DEG)
    time.sleep(0.5)
    tilt = ensure_vertical(rtde_c, rtde_r, args.tilt_tol_deg)

    weight = ask_weight(args.weight)

    sample_dt = 1.0 / args.rate
    rows = []

    def sample(loaded, weight_g, duration_s):
        t_end = time.time() + duration_s
        while time.time() < t_end:
            rows.append(_row(rtde_r, loaded, weight_g))
            time.sleep(sample_dt)

    try:
        input("\n[collect] Tool mounted, NO weight applied. "
              "Press Enter to zero the FT sensor...")
        rtde_c.zeroFtSensor()
        time.sleep(1.0)

        print(f"[collect] Recording {args.idle_s:.1f}s no-load baseline...")
        sample(loaded=False, weight_g=0.0, duration_s=args.idle_s)

        input(f"\n[collect] Place the {weight:g} g weight on the tool. "
              "Press Enter once settled...")
        print(f"[collect] Holding & recording for {args.dwell:.1f}s...")
        sample(loaded=True, weight_g=weight, duration_s=args.dwell)
        print("[collect] Done holding. You can remove the weight now.")
    except KeyboardInterrupt:
        print("\n[collect] Interrupted")
    finally:
        # Rotate wrist 2 back to -90 deg (-z direction), even on interrupt.
        try:
            print("[collect] Rotating wrist 2 back to -90 deg...")
            move_wrist2_to_deg(rtde_c, rtde_r, WRIST2_NEG_Z_DEG)
        except Exception as e:
            print(f"[collect] WARNING: could not return wrist 2: {e}")
        try:
            rtde_c.stopScript()
        except Exception:
            pass

    if len(rows) < 2:
        sys.exit("[collect] No data recorded")

    span = rows[-1]["timestamp"] - rows[0]["timestamp"]
    achieved_rate = (len(rows) - 1) / span if span > 0 else 0.0
    print(f"\n[collect] {len(rows)} rows over {span:.1f}s "
          f"-> achieved {achieved_rate:.1f} Hz (target {args.rate:.1f} Hz)")

    os.makedirs(LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(
        LOG_DIR, f"fzcal_{args.tip}_posz_{weight:g}g_{ts}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"[collect] Saved -> {csv_path}")

    meta_path = csv_path.replace(".csv", "_meta.json")
    with open(meta_path, "w") as f:
        json.dump({
            "tip": args.tip,
            "weight_g": weight,
            "load_cell": False,
            "tilt_from_vertical_deg": tilt,
            "target_rate_hz": args.rate,
            "achieved_rate_hz": achieved_rate,
            "dwell_s": args.dwell,
            "idle_s": args.idle_s,
            "robot_ip": robot_ip,
        }, f, indent=2)
    print(f"[collect] Metadata -> {meta_path}")
    print("[collect] Run again for the next weight.")


def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weight", type=float, default=None,
                    help="weight in grams (if omitted, you'll be prompted)")
    ap.add_argument("--tip", default="ur_only",
                    help="fixture label used for filenames")
    ap.add_argument("--dwell", type=float, default=10.0,
                    help="hold time (s)")
    ap.add_argument("--idle-s", type=float, default=3.0,
                    help="no-load baseline duration before loading (s)")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="target sample rate (Hz)")
    ap.add_argument("--tilt-tol-deg", type=float, default=0.5,
                    help="max allowed tool-axis tilt from vertical (deg)")
    ap.add_argument("--robot-ip", default=None,
                    help="override UR5 IP (default: ur5_control.ROBOT_IP)")
    return ap


def main():
    collect_one(build_parser().parse_args())


if __name__ == "__main__":
    main()
