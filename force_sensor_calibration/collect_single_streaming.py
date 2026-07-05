"""
collect_single_streaming.py — RTDE-streaming version of collect_single.py

Key differences from collect_single.py
───────────────────────────────────────
1. RTDEReceiveInterface is configured with a fixed `frequency` (default 125 Hz).
   The robot streams a complete data packet at that rate in a background thread;
   each get*() call simply reads the latest buffered value — no network wait.

2. waitForNewData() is called once per loop iteration instead of time.sleep().
   This synchronises the recording loop exactly to the RTDE packet rate, so
   every row is a genuinely fresh measurement with no duplicates and no drift.

3. Only the fields needed for calibration are read (ai0 + fz + tcp_pose).
   Fewer Python calls per iteration → lower per-sample overhead.

4. achieved_rate_hz is now computed per recording phase (not over the whole
   run including user-interaction dead time), so the reported rate is accurate.

Expected rate
─────────────
  --freq 125   →  ~100–125 Hz sustained (Python + RTDE overhead)
  --freq  20   →  ~20 Hz  (same as the original script but without sleep drift)

Usage
─────
  python collect_single_streaming.py --tip futek_direct --weight 200
  python collect_single_streaming.py --tip futek_direct --weight 100 --freq 125
"""

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import numpy as np

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
LOG_DIR   = os.path.join(CALIB_DIR, "logs")

CSV_FIELDS = ["timestamp", "datetime", "weight_g", "loaded",
              "tcp_x", "tcp_y", "tcp_z", "fx", "fy", "fz",
              "tx", "ty", "tz", "ai0"]


# ── Perpendicularity check ─────────────────────────────────────────────────────
def rotvec_to_R(rotvec):
    rv    = np.asarray(rotvec, dtype=float)
    theta = np.linalg.norm(rv)
    if theta < 1e-9:
        return np.eye(3)
    k = rv / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def tilt_from_vertical_deg(rotvec):
    R      = rotvec_to_R(rotvec)
    tool_z = R @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(abs(tool_z[2]), -1.0, 1.0))))


# ── Single sample (reads from already-buffered RTDE data) ─────────────────────
def _row(rtde_r, loaded, weight_g):
    ft  = rtde_r.getActualTCPForce()
    tcp = rtde_r.getActualTCPPose()
    ai0 = rtde_r.getStandardAnalogInput0()
    now = time.time()
    return {
        "timestamp": now,
        "datetime":  datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "weight_g":  weight_g,
        "loaded":    int(loaded),
        "tcp_x": tcp[0], "tcp_y": tcp[1], "tcp_z": tcp[2],
        "fx": ft[0],  "fy": ft[1],  "fz": ft[2],
        "tx": ft[3],  "ty": ft[4],  "tz": ft[5],
        "ai0": float(ai0),
    }


# ── Streaming recording phase ─────────────────────────────────────────────────
def sample_streaming(rtde_r, loaded, weight_g, duration_s):
    """
    Record for `duration_s` seconds, synchronised to RTDE packet delivery.
    waitForNewData() blocks until the next streaming packet arrives, so the
    loop rate matches the frequency set on RTDEReceiveInterface exactly.
    Returns (rows, achieved_hz).
    """
    rows  = []
    t_end = time.time() + duration_s
    while time.time() < t_end:
        rtde_r.waitForNewData()      # sleeps until next RTDE packet — no drift
        rows.append(_row(rtde_r, loaded, weight_g))

    if len(rows) > 1:
        span         = rows[-1]["timestamp"] - rows[0]["timestamp"]
        achieved_hz  = (len(rows) - 1) / span if span > 0 else 0.0
    else:
        achieved_hz  = 0.0

    return rows, achieved_hz


# ── Weight prompt ─────────────────────────────────────────────────────────────
def ask_weight(cli_weight):
    if cli_weight is not None:
        return float(cli_weight)
    while True:
        raw = input("\nEnter weight in grams (e.g. 200): ").strip()
        try:
            w = float(raw)
            if w > 0:
                return w
            print("  Weight must be > 0.")
        except ValueError:
            print(f"  '{raw}' is not a number — try again.")


# ── Main collection routine ───────────────────────────────────────────────────
def collect_one(args):
    if rtde_control is None or rtde_receive is None:
        sys.exit("ur_rtde is not installed.")

    weight = ask_weight(args.weight)

    import ur5_control
    robot_ip = args.robot_ip or ur5_control.ROBOT_IP

    print(f"[collect] Connecting to {robot_ip} at {args.freq} Hz ...")
    # ── KEY CHANGE: configure streaming frequency on construction ──────────────
    # The robot will push one packet every 1/freq seconds from this point on.
    # All subsequent get*() calls read from this continuously updated buffer.
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip, frequency=args.freq)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    print(f"[collect] Connected. Streaming at {args.freq} Hz.")

    # ── Perpendicularity check ─────────────────────────────────────────────────
    actual_pose = rtde_r.getActualTCPPose()
    tilt        = tilt_from_vertical_deg(actual_pose[3:6])
    print(f"[collect] Tool tilt from vertical: {tilt:.3f} deg "
          f"(tolerance {args.tilt_tol_deg:.2f} deg)")
    if tilt > args.tilt_tol_deg:
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        sys.exit(
            f"[collect] ABORT — tool axis {tilt:.2f} deg off vertical "
            f"(limit {args.tilt_tol_deg:.2f} deg). Re-square the mount."
        )

    all_rows = []
    hz_idle  = 0.0
    hz_dwell = 0.0

    try:
        input("\n[collect] NO weight applied. Press Enter to zero FT sensor ...")
        rtde_c.zeroFtSensor()
        time.sleep(0.5)   # let the zero settle before recording

        print(f"[collect] Recording {args.idle_s:.1f}s baseline  "
              f"(target {args.freq:.0f} Hz) ...")
        idle_rows, hz_idle = sample_streaming(
            rtde_r, loaded=False, weight_g=0.0, duration_s=args.idle_s)
        all_rows.extend(idle_rows)
        print(f"[collect]   → {len(idle_rows)} rows  achieved {hz_idle:.1f} Hz")

        input(f"\n[collect] Place the {weight:g} g weight. "
              "Press Enter once settled ...")
        print(f"[collect] Holding & recording for {args.dwell:.1f}s ...")
        dwell_rows, hz_dwell = sample_streaming(
            rtde_r, loaded=True, weight_g=weight, duration_s=args.dwell)
        all_rows.extend(dwell_rows)
        print(f"[collect]   → {len(dwell_rows)} rows  achieved {hz_dwell:.1f} Hz")
        print("[collect] Done. You can remove the weight.")

    except KeyboardInterrupt:
        print("\n[collect] Interrupted.")
    finally:
        try:
            rtde_c.stopScript()
        except Exception:
            pass

    if len(all_rows) < 2:
        sys.exit("[collect] No data recorded.")

    # ── Save CSV ───────────────────────────────────────────────────────────────
    os.makedirs(LOG_DIR, exist_ok=True)
    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(LOG_DIR, f"fzcal_{args.tip}_{weight:g}g_{ts}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        w.writerows(all_rows)
    print(f"[collect] Saved  → {csv_path}")

    # ── Save metadata ──────────────────────────────────────────────────────────
    # achieved_rate_hz is now the mean of idle and dwell phases — no dead time
    meta_path = csv_path.replace(".csv", "_meta.json")
    with open(meta_path, "w") as f:
        json.dump({
            "tip":                    args.tip,
            "weight_g":               weight,
            "tilt_from_vertical_deg": tilt,
            "target_rate_hz":         args.freq,
            "achieved_rate_hz_idle":  round(hz_idle,  2),
            "achieved_rate_hz_dwell": round(hz_dwell, 2),
            "dwell_s":                args.dwell,
            "idle_s":                 args.idle_s,
            "robot_ip":               robot_ip,
        }, f, indent=2)
    print(f"[collect] Metadata → {meta_path}")
    print(f"\n[collect] Summary:")
    print(f"          idle  phase: {len(idle_rows) if 'idle_rows' in dir() else 0:>5} rows "
          f"@ {hz_idle:.1f} Hz")
    print(f"          dwell phase: {len(dwell_rows) if 'dwell_rows' in dir() else 0:>5} rows "
          f"@ {hz_dwell:.1f} Hz")
    print(f"          target rate: {args.freq:.0f} Hz")
    print("[collect] Run again for the next weight.")


# ── CLI ────────────────────────────────────────────────────────────────────────
def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tip",    required=True,
                    help="fixture label, e.g. futek_direct")
    ap.add_argument("--weight", type=float, default=None,
                    help="weight in grams (prompted if omitted)")
    ap.add_argument("--freq",   type=float, default=125.0,
                    help="RTDE streaming frequency in Hz (default 125)")
    ap.add_argument("--dwell",  type=float, default=10.0,
                    help="hold/recording time with weight applied (s)")
    ap.add_argument("--idle-s", type=float, default=3.0,
                    help="no-load baseline recording time (s)")
    ap.add_argument("--tilt-tol-deg", type=float, default=2.0,
                    help="max tool-axis tilt before abort (deg)")
    ap.add_argument("--robot-ip", default=None,
                    help="override UR5 IP (default: ur5_control.ROBOT_IP)")
    return ap


def main():
    collect_one(build_parser().parse_args())


if __name__ == "__main__":
    main()
