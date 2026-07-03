"""
force_calib.py — collect and fit the UR5 fz -> FUTEK force correction.

Implements FORCE_CALIBRATION_SOP.md:

  `collect` (Step 1-2, live) — connects to the UR5 directly (rtde_control /
  rtde_receive), verifies the pressing tool axis is perpendicular to the
  surface within tolerance, sweeps a set of indentation depths at a point,
  and logs a session CSV (fz, ai0, TCP pose) at a known sample rate.

  `fit` (Steps 3-7, offline) — extracts one (fz_robot, lc_futek) sample
  pair per press event from logged session CSVs (dwell-plateau mean, first
  0.3 s of each press dropped, per-session zero subtracted), fits
  slope/offset by OLS, validates against a held-out session (RMSE, Pearson
  r, Bland-Altman), checks the SOP's acceptance criteria, and saves
  calib_fz_<tip>.json.

Usage:
  # 1. Live data collection (run once per fit session, once for holdout):
  python force_calib.py collect --tip short_6mm --point 10 \
      --depths 1 2 3 4 6 --reps 3

  # 2. Offline fit + validation:
  python force_calib.py fit --tip short_6mm \
      --fit logs/fzcal_short_6mm_P10_session_*.csv \
      --holdout logs/fzcal_short_6mm_P10_session_<held_out>.csv
"""

import argparse
import csv
import glob
import json
import os
import sys
import time
from datetime import date, datetime

import numpy as np
import pandas as pd

INTEGRATION_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "Integration_2")
sys.path.insert(0, os.path.abspath(INTEGRATION_DIR))
from analyze_session import ai0_to_newtons, LOADCELL_MAX_LB  # noqa: E402

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None

SETTLE_S = 0.3           # dropped from the start of each press window (Step 3)
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


# ── Step 3: extract per-press (fz_robot, lc_futek) pairs ──────────────
def load_session(path):
    df = pd.read_csv(path)
    df["t"] = df["timestamp"] - df["timestamp"].iloc[0]
    df["ur5_pressing"] = pd.to_numeric(
        df["ur5_pressing"], errors="coerce").fillna(0).astype(int)
    for c in ("fz", "ai0"):
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df["lc_n"] = ai0_to_newtons(df["ai0"])
    return df


def session_sample_rate(df):
    """Achieved sample rate (Hz) from consecutive timestamp deltas."""
    dt = np.diff(df["timestamp"].to_numpy())
    dt = dt[dt > 0]
    return float(1.0 / np.median(dt)) if len(dt) else 0.0


def session_zero(df):
    """Pre-contact idle mean (Step 1); falls back to signal min if the
    recording starts already in contact (no idle window available)."""
    first_press = df.index[df["ur5_pressing"] == 1]
    idle = df.loc[: first_press[0] - 1] if len(first_press) else df.iloc[0:0]
    if len(idle) >= 5:
        return float(idle["fz"].mean()), float(idle["lc_n"].mean())
    print(f"    [warn] no idle window before first press — "
          f"falling back to signal min for zero-reference")
    return float(df["fz"].min()), float(df["lc_n"].min())


def press_windows(df):
    """Contiguous ur5_pressing==1 runs, as (start_idx, end_idx) pairs."""
    pressing = df["ur5_pressing"].to_numpy()
    edges = np.diff(np.concatenate(([0], pressing, [0])))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0]
    return list(zip(starts, ends))


def extract_pairs(path):
    df = load_session(path)
    rate = session_sample_rate(df)
    min_plateau_samples = 3
    if rate > 0 and rate * SETTLE_S < 1.0:
        print(f"    [warn] {os.path.basename(path)}: sample rate {rate:.1f} Hz "
              f"gives < 1 sample in the {SETTLE_S}s settle window")
    fz_zero, lc_zero = session_zero(df)
    pairs = []
    for start, end in press_windows(df):
        window = df.iloc[start:end]
        if len(window) < 2:
            continue
        t0 = window["t"].iloc[0]
        plateau = window[window["t"] - t0 >= SETTLE_S]
        if len(plateau) < min_plateau_samples:
            continue
        fz_robot = float(plateau["fz"].mean()) - fz_zero
        lc_futek = float(plateau["lc_n"].mean()) - lc_zero
        pairs.append((fz_robot, lc_futek))
    return np.array(pairs) if pairs else np.empty((0, 2)), rate


def collect(paths):
    files = sorted({f for p in paths for f in glob.glob(p)} or set(paths))
    all_pairs = []
    rates = []
    for f in files:
        pairs, rate = extract_pairs(f)
        rates.append(rate)
        print(f"  {os.path.basename(f)}: {len(pairs)} press events "
              f"@ {rate:.1f} Hz")
        all_pairs.append(pairs)
    return (np.vstack(all_pairs) if all_pairs else np.empty((0, 2))), files, rates


# ── Step 4: fit the correction ─────────────────────────────────────────
def fit_correction(fz_robot, lc_futek):
    a, b = np.polyfit(fz_robot, lc_futek, 1)
    corrected = a * fz_robot + b
    resid = corrected - lc_futek
    rmse = float(np.sqrt(np.mean(resid ** 2)))
    r2 = float(np.corrcoef(fz_robot, lc_futek)[0, 1] ** 2)
    return float(a), float(b), r2, rmse


# ── Step 5: validate on held-out data ──────────────────────────────────
def bland_altman(a_sig, b_sig):
    diff = a_sig - b_sig
    bias = float(diff.mean())
    std = float(diff.std())
    return bias, bias - 1.96 * std, bias + 1.96 * std


def validate(fz_robot, lc_futek, a, b):
    corrected = a * fz_robot + b
    rmse_raw = float(np.sqrt(np.mean((fz_robot - lc_futek) ** 2)))
    rmse_corr = float(np.sqrt(np.mean((corrected - lc_futek) ** 2)))
    r_corr = float(np.corrcoef(corrected, lc_futek)[0, 1])
    bias_raw, lo_raw, hi_raw = bland_altman(fz_robot, lc_futek)
    bias_corr, lo_corr, hi_corr = bland_altman(corrected, lc_futek)
    return {
        "rmse_raw": rmse_raw, "rmse_corrected": rmse_corr,
        "pearson_r_corrected": r_corr,
        "ba_bias_raw": bias_raw, "ba_loa_raw": (lo_raw, hi_raw),
        "ba_bias_corrected": bias_corr, "ba_loa_corrected": (lo_corr, hi_corr),
    }


# ── Step 6: acceptance criteria ────────────────────────────────────────
def check_acceptance(metrics, lc_futek):
    max_force = float(np.max(np.abs(lc_futek))) if len(lc_futek) else 0.0
    rmse_thresh = max(0.5, 0.05 * max_force)
    checks = {
        "Corrected RMSE <= "
        f"{rmse_thresh:.3f} N": metrics["rmse_corrected"] <= rmse_thresh,
        "Pearson r >= 0.98": metrics["pearson_r_corrected"] >= 0.98,
        "Bland-Altman bias <= 0.2 N": abs(metrics["ba_bias_corrected"]) <= 0.2,
        "+-1.96 sigma LoA within +-1 N": (
            abs(metrics["ba_loa_corrected"][0]) <= 1.0
            and abs(metrics["ba_loa_corrected"][1]) <= 1.0
        ),
    }
    return checks, all(checks.values())


# ── `fit` subcommand ────────────────────────────────────────────────────
def cmd_fit(args):
    print("Fit set:")
    fit_pairs, fit_files, fit_rates = collect(args.fit)
    print("Hold-out set:")
    holdout_pairs, holdout_files, holdout_rates = collect(args.holdout)

    if len(fit_pairs) < 5:
        sys.exit(f"Only {len(fit_pairs)} fit samples — need at least 5. "
                  "Collect more sessions per SOP Step 2.")
    if len(holdout_pairs) < 5:
        sys.exit(f"Only {len(holdout_pairs)} hold-out samples — need at least 5.")

    fz_fit, lc_fit = fit_pairs[:, 0], fit_pairs[:, 1]
    a, b, r2_fit, rmse_fit = fit_correction(fz_fit, lc_fit)
    print(f"\nFit (n={len(fit_pairs)}): "
          f"lc_futek = {a:.4f} * fz_robot + {b:+.4f}")
    print(f"  R^2 = {r2_fit:.4f}   RMSE = {rmse_fit:.4f} N")

    fz_ho, lc_ho = holdout_pairs[:, 0], holdout_pairs[:, 1]
    metrics = validate(fz_ho, lc_ho, a, b)
    print(f"\nHold-out validation (n={len(holdout_pairs)}):")
    print(f"  RMSE raw       = {metrics['rmse_raw']:.4f} N")
    print(f"  RMSE corrected = {metrics['rmse_corrected']:.4f} N")
    print(f"  Pearson r (corrected) = {metrics['pearson_r_corrected']:.4f}")
    print(f"  Bland-Altman bias (corrected) = {metrics['ba_bias_corrected']:+.4f} N")
    print(f"  +-1.96 sigma LoA (corrected)  = "
          f"[{metrics['ba_loa_corrected'][0]:+.4f}, "
          f"{metrics['ba_loa_corrected'][1]:+.4f}] N")

    checks, passed = check_acceptance(metrics, lc_ho)
    print("\nAcceptance (Step 6):")
    for desc, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {desc}")
    print(f"\nOverall: {'PASS' if passed else 'FAIL'}")

    out_path = args.out or os.path.join(CALIB_DIR, f"calib_fz_{args.tip}.json")
    result = {
        "tip": args.tip,
        "date": date.today().isoformat(),
        "slope": a,
        "offset": b,
        "r_squared": r2_fit,
        "rmse_n": rmse_fit,
        "n_samples": int(len(fit_pairs)),
        "futek_rated_lb": LOADCELL_MAX_LB,
        "fit_sample_rate_hz": fit_rates,
        "holdout": {
            "n_samples": int(len(holdout_pairs)),
            "rmse_raw_n": metrics["rmse_raw"],
            "rmse_corrected_n": metrics["rmse_corrected"],
            "pearson_r": metrics["pearson_r_corrected"],
            "ba_bias_n": metrics["ba_bias_corrected"],
            "ba_loa_n": list(metrics["ba_loa_corrected"]),
            "passed": passed,
            "sample_rate_hz": holdout_rates,
        },
        "fit_files": [os.path.basename(f) for f in fit_files],
        "holdout_files": [os.path.basename(f) for f in holdout_files],
    }
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nSaved -> {out_path}")
    if not passed:
        print("Calibration did NOT meet acceptance criteria — see SOP Step 6 "
              "troubleshooting before using this correction.")


# ── `collect` subcommand ────────────────────────────────────────────────
CSV_FIELDS = ["timestamp", "datetime", "ur5_point", "ur5_pressing",
              "tcp_x", "tcp_y", "tcp_z", "fx", "fy", "fz",
              "tx", "ty", "tz", "ai0"]


def _row(rtde_r, pt, pressing):
    ft = rtde_r.getActualTCPForce()
    tcp = rtde_r.getActualTCPPose()
    ai0 = rtde_r.getStandardAnalogInput0()
    now = time.time()
    return {
        "timestamp": now,
        "datetime": datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "ur5_point": pt,
        "ur5_pressing": int(pressing),
        "tcp_x": tcp[0], "tcp_y": tcp[1], "tcp_z": tcp[2],
        "fx": ft[0], "fy": ft[1], "fz": ft[2],
        "tx": ft[3], "ty": ft[4], "tz": ft[5],
        "ai0": float(ai0),
    }


def cmd_collect(args):
    if rtde_control is None or rtde_receive is None:
        sys.exit("ur_rtde is not installed — "
                  "pip install ur-rtde --break-system-packages")

    import ur5_control
    import load_calibration

    robot_ip = args.robot_ip or ur5_control.ROBOT_IP
    pt = args.point
    if pt not in ur5_control.POINTS:
        sys.exit(f"Point P{pt} is not in ur5_control.POINTS")

    # ── Perpendicularity check (before any load_calibration side effects) ──
    tilt = tilt_from_vertical_deg(ur5_control.REFERENCE_POSE[3:6])
    print(f"[collect] Pressing-tool Z-axis tilt from vertical: "
          f"{tilt:.3f} deg (tolerance {args.tilt_tol_deg:.2f} deg)")
    if tilt > args.tilt_tol_deg:
        sys.exit(
            f"[collect] ABORT — tool axis is {tilt:.2f} deg off vertical, "
            f"exceeds {args.tilt_tol_deg:.2f} deg tolerance. Off-axis "
            "presses corrupt the Fz-vs-FUTEK relationship (FUTEK sees the "
            "full contact load, robot Fz only sees the Z-component). "
            "Re-square the tip/mount or re-teach REFERENCE_POSE in "
            "ur5_control.py, then re-run."
        )

    load_calibration.preview(args.tip)
    load_calibration.apply(args.tip)

    print(f"[collect] Connecting to {robot_ip} ...")
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    print("[collect] Connected.")

    home = ur5_control._build_pose(pt, ur5_control.SAFE_HOME_Z_MM)
    surface = ur5_control._build_pose(pt, 0.0)
    sample_dt = 1.0 / args.rate
    rows = []

    def sample(pressing, duration_s):
        t_end = time.time() + duration_s
        while time.time() < t_end:
            rows.append(_row(rtde_r, pt, pressing))
            time.sleep(sample_dt)

    try:
        rtde_c.moveL(home, ur5_control.VELOCITY_TRAVEL, ur5_control.ACCELERATION)
        input("\n[collect] Tip clear of surface, no load. "
              "Press Enter to zero the FT sensor...")
        rtde_c.zeroFtSensor()
        time.sleep(1.0)

        print(f"[collect] Recording {args.idle_s:.1f}s idle baseline "
              f"(fz_zero / ai0_zero)...")
        sample(pressing=False, duration_s=args.idle_s)

        rtde_c.moveL(surface, ur5_control.VELOCITY_TRAVEL, ur5_control.ACCELERATION)

        for depth in args.depths:
            for rep in range(args.reps):
                print(f"[collect] P{pt}  depth={depth:.2f}mm  "
                      f"rep={rep + 1}/{args.reps}")
                pressed = ur5_control._build_pose(pt, -depth)
                rtde_c.moveL(pressed, ur5_control.VELOCITY_PRESS,
                             ur5_control.ACCELERATION)
                sample(pressing=True, duration_s=args.dwell)
                rtde_c.moveL(surface, ur5_control.VELOCITY_PRESS,
                             ur5_control.ACCELERATION)
                sample(pressing=False, duration_s=0.3)

        rtde_c.moveL(home, ur5_control.VELOCITY_TRAVEL, ur5_control.ACCELERATION)
    except KeyboardInterrupt:
        print("\n[collect] Interrupted — returning home")
        rtde_c.moveL(home, ur5_control.VELOCITY_TRAVEL, ur5_control.ACCELERATION)
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
    csv_path = os.path.join(LOG_DIR, f"fzcal_{args.tip}_P{pt}_session_{ts}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"[collect] Saved -> {csv_path}")

    meta_path = csv_path.replace(".csv", "_meta.json")
    with open(meta_path, "w") as f:
        json.dump({
            "tip": args.tip, "point": pt,
            "tilt_from_vertical_deg": tilt,
            "target_rate_hz": args.rate,
            "achieved_rate_hz": achieved_rate,
            "depths_mm": args.depths, "reps": args.reps,
            "dwell_s": args.dwell, "idle_s": args.idle_s,
            "robot_ip": robot_ip,
        }, f, indent=2)
    print(f"[collect] Metadata -> {meta_path}")
    print(f"[collect] Feed this file to `fit --fit/--holdout` next.")


# ── CLI ──────────────────────────────────────────────────────────────
def build_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    c = sub.add_parser("collect", help="live: sweep indentation depths on the robot and log a session CSV")
    c.add_argument("--tip", required=True, help="tip profile name, e.g. short_6mm")
    c.add_argument("--point", type=int, default=10, help="UR5 point to press (default: 10, center)")
    c.add_argument("--depths", type=float, nargs="+", default=[1, 2, 3, 4, 6],
                   help="indentation depths in mm (SOP Step 2 default: 1 2 3 4 6)")
    c.add_argument("--reps", type=int, default=3, help="presses per depth")
    c.add_argument("--dwell", type=float, default=1.5, help="dwell time per press (s)")
    c.add_argument("--idle-s", type=float, default=3.0, help="idle baseline duration before pressing (s)")
    c.add_argument("--rate", type=float, default=20.0, help="target sample rate (Hz), matches main.py's logger")
    c.add_argument("--tilt-tol-deg", type=float, default=2.0,
                   help="max allowed tool-axis tilt from vertical before aborting (deg)")
    c.add_argument("--robot-ip", default=None, help="override UR5 IP (default: ur5_control.ROBOT_IP / UR_ROBOT_IP env var)")
    c.set_defaults(func=cmd_collect)

    f = sub.add_parser("fit", help="offline: fit + validate the correction from logged session CSVs")
    f.add_argument("--tip", required=True, help="tip profile name, e.g. short_6mm")
    f.add_argument("--fit", nargs="+", required=True,
                   help="session CSV(s)/glob(s) to fit the correction on")
    f.add_argument("--holdout", nargs="+", required=True,
                   help="session CSV(s)/glob(s) to validate on (held out from fitting)")
    f.add_argument("--out", default=None,
                   help="output JSON path (default: calib_fz_<tip>.json in this folder)")
    f.set_defaults(func=cmd_fit)

    return ap


def main():
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
