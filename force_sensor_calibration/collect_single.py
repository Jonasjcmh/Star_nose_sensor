"""
collect_single.py — collect UR5 fz / FUTEK load-cell data for ONE weight.

Run it once per weight: it asks you for the weight (e.g. 200), connects to
the UR5, checks the tool is vertical, zeroes the force sensor, records a
short no-load baseline, then holds while you place that single weight and
logs the samples. One run = one weight = one CSV. Run it again for the
next weight (100, 50, ...).

Usage:
  python collect_single.py --tip futek_direct
      -> prompts: "Enter weight in grams:"  (type 200, then run again for 100, ...)

  # or pass the weight directly, skipping the prompt:
  python collect_single.py --tip futek_direct --weight 200
"""

import argparse
import csv
import json
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
    """Angle (deg) between the tool Z-axis and vertical — 0 = perfectly vertical."""
    R = rotvec_to_R(rotvec)
    tool_z = R @ np.array([0.0, 0.0, 1.0])
    cos_angle = np.clip(abs(tool_z[2]), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


# ── One CSV row ────────────────────────────────────────────────────────
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

    weight = ask_weight(args.weight)

    import ur5_control
    robot_ip = args.robot_ip or ur5_control.ROBOT_IP

    print(f"[collect] Connecting to {robot_ip} ...")
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    print("[collect] Connected.")

    # ── Perpendicularity check (actual live pose) ──
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
            f"exceeds {args.tilt_tol_deg:.2f} deg tolerance. Re-square the "
            "load cell mount or move the tool to a vertical pose, then re-run."
        )

    sample_dt = 1.0 / args.rate
    rows = []

    def sample(loaded, weight_g, duration_s):
        t_end = time.time() + duration_s
        while time.time() < t_end:
            rows.append(_row(rtde_r, loaded, weight_g))
            time.sleep(sample_dt)

    try:
        input("\n[collect] Load cell mounted, NO weight applied. "
              "Press Enter to zero the FT sensor...")
        rtde_c.zeroFtSensor()
        time.sleep(1.0)

        print(f"[collect] Recording {args.idle_s:.1f}s no-load baseline...")
        sample(loaded=False, weight_g=0.0, duration_s=args.idle_s)

        input(f"\n[collect] Place the {weight:g} g weight on the load cell. "
              "Press Enter once settled...")
        print(f"[collect] Holding & recording for {args.dwell:.1f}s...")
        sample(loaded=True, weight_g=weight, duration_s=args.dwell)
        print("[collect] Done holding. You can remove the weight now.")
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
    csv_path = os.path.join(
        LOG_DIR, f"fzcal_{args.tip}_{weight:g}g_{ts}.csv")
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
    ap.add_argument("--tip", required=True,
                    help="config/fixture label used for filenames, e.g. futek_direct")
    ap.add_argument("--weight", type=float, default=None,
                    help="weight in grams (if omitted, you'll be prompted)")
    ap.add_argument("--dwell", type=float, default=10.0,
                    help="hold time (s)")
    ap.add_argument("--idle-s", type=float, default=3.0,
                    help="no-load baseline duration before loading (s)")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="target sample rate (Hz)")
    ap.add_argument("--tilt-tol-deg", type=float, default=2.0,
                    help="max allowed tool-axis tilt from vertical before aborting (deg)")
    ap.add_argument("--robot-ip", default=None,
                    help="override UR5 IP (default: ur5_control.ROBOT_IP)")
    return ap


def main():
    collect_one(build_parser().parse_args())


if __name__ == "__main__":
    main()
