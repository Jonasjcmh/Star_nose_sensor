"""
force_calib.py — collect the UR5 fz / FUTEK load-cell data for the force
calibration (data collection only — no fitting or analysis).

Connects to the UR5 directly (rtde_control / rtde_receive), verifies the
tool axis is perpendicular to vertical within tolerance, then holds a
single static pose while known weights are placed on and removed from the
FUTEK load cell, which sits in the same static load path as the UR5 tool
flange. The robot never moves during loading — fz and ai0 both see the
same static gravity load through the load cell. Logs a session CSV
(fz, ai0, TCP pose) at a known sample rate.

Usage:
  python force_calib.py --tip futek_direct --weights 200 100 50 20 10 5
"""

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import numpy as np

# ur5_control lives in Integration_2 (ROBOT_IP / REFERENCE_POSE).
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


# ── Perpendicularity check ─────────────────────────────────────────────
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


def tilt_from_vertical_deg(rotvec):
    """Angle (deg) between the tool Z-axis and vertical — 0 = perfectly
    perpendicular to a horizontal surface, regardless of up/down sign."""
    R = rotvec_to_R(rotvec)
    tool_z = R @ np.array([0.0, 0.0, 1.0])
    cos_angle = np.clip(abs(tool_z[2]), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


# ── Data collection ────────────────────────────────────────────────────
CSV_FIELDS = ["timestamp", "datetime", "weight_g", "loaded",
              "tcp_x", "tcp_y", "tcp_z", "fx", "fy", "fz",
              "tx", "ty", "tz", "ai0"]


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
        "fx": ft[0], "fy": ft[1], "fz": ft[2],
        "tx": ft[3], "ty": ft[4], "tz": ft[5],
        "ai0": float(ai0),
    }


def collect(args):
    if rtde_control is None or rtde_receive is None:
        sys.exit("ur_rtde is not installed — "
                  "pip install ur-rtde --break-system-packages")

    import ur5_control

    robot_ip = args.robot_ip or ur5_control.ROBOT_IP

    print(f"[collect] Connecting to {robot_ip} ...")
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    print("[collect] Connected.")

    # ── Perpendicularity check ──
    # The load cell and weights sit in the tool's static load path (no
    # pressing motion) — if the tool Z-axis isn't vertical, gravity's
    # force vector leaks into fx/fy and fz under-reads the true weight.
    # Check the robot's ACTUAL pose, not a stored constant — the tool
    # must be physically at (or squared to) vertical right now.
    actual_pose = rtde_r.getActualTCPPose()
    tilt = tilt_from_vertical_deg(actual_pose[3:6])
    print(f"[collect] Tool Z-axis tilt from vertical: "
          f"{tilt:.3f} deg (tolerance {args.tilt_tol_deg:.2f} deg)")
    if tilt > args.tilt_tol_deg:
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        sys.exit(
            f"[collect] ABORT — tool axis is {tilt:.2f} deg off vertical, "
            f"exceeds {args.tilt_tol_deg:.2f} deg tolerance. An off-axis "
            "load path means the applied weight's gravity vector isn't "
            "aligned with the tool Z-axis, so fz under-reads the true "
            "force. Re-square the load cell mount or move the tool to a "
            "vertical pose, then re-run."
        )

    sample_dt = 1.0 / args.rate
    rows = []

    def sample(loaded, weight_g, duration_s):
        t_end = time.time() + duration_s
        while time.time() < t_end:
            rows.append(_row(rtde_r, loaded, weight_g))
            time.sleep(sample_dt)

    try:
        input("\n[collect] Load cell mounted, no weight applied. "
              "Press Enter to zero the FT sensor...")
        rtde_c.zeroFtSensor()
        time.sleep(1.0)

        print(f"[collect] Recording {args.idle_s:.1f}s idle baseline "
              f"(fz_zero / ai0_zero)...")
        sample(loaded=False, weight_g=0.0, duration_s=args.idle_s)

        for weight in args.weights:
            for rep in range(args.reps):
                input(f"\n[collect] Place the {weight:g} g weight on the "
                      f"load cell (rep {rep + 1}/{args.reps}). Press Enter "
                      "once settled...")
                sample(loaded=True, weight_g=weight, duration_s=args.dwell)
                input("[collect] Remove the weight. Press Enter once clear...")
                sample(loaded=False, weight_g=0.0, duration_s=0.3)
    except KeyboardInterrupt:
        print("\n[collect] Interrupted")
    finally:
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
    csv_path = os.path.join(LOG_DIR, f"fzcal_{args.tip}_session_{ts}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"[collect] Saved -> {csv_path}")

    meta_path = csv_path.replace(".csv", "_meta.json")
    with open(meta_path, "w") as f:
        json.dump({
            "tip": args.tip,
            "tilt_from_vertical_deg": tilt,
            "target_rate_hz": args.rate,
            "achieved_rate_hz": achieved_rate,
            "weights_g": args.weights, "reps": args.reps,
            "dwell_s": args.dwell, "idle_s": args.idle_s,
            "robot_ip": robot_ip,
        }, f, indent=2)
    print(f"[collect] Metadata -> {meta_path}")


# ── CLI ──────────────────────────────────────────────────────────────
def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tip", required=True,
                    help="config/fixture label used for filenames, e.g. futek_direct")
    ap.add_argument("--weights", type=float, nargs="+", required=True,
                    help="known weights in grams to apply, e.g. 200 100 50 20 10 5")
    ap.add_argument("--reps", type=int, default=1,
                    help="place/remove cycles per weight (1 = single stable hold, no repetition)")
    ap.add_argument("--dwell", type=float, default=10.0,
                    help="hold time per loaded rep (s)")
    ap.add_argument("--idle-s", type=float, default=3.0,
                    help="idle baseline duration before loading (s)")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="target sample rate (Hz)")
    ap.add_argument("--tilt-tol-deg", type=float, default=2.0,
                    help="max allowed tool-axis tilt from vertical before aborting (deg)")
    ap.add_argument("--robot-ip", default=None,
                    help="override UR5 IP (default: ur5_control.ROBOT_IP)")
    return ap


def main():
    args = build_parser().parse_args()
    collect(args)


if __name__ == "__main__":
    main()
