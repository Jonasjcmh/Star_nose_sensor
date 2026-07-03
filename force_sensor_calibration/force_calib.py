"""
force_calib.py — collect and fit the UR5 fz -> FUTEK force correction.

Implements FORCE_CALIBRATION_SOP.md (data collection deviates from the
written SOP — see below; fit/validate/save follow it as documented):

  `collect` (live) — connects to the UR5 directly (rtde_control /
  rtde_receive), verifies the tool axis is perpendicular to vertical within
  tolerance, then holds a single static pose while known weights are
  placed on and removed from the FUTEK load cell, which sits in the same
  static load path as the UR5 tool flange. The robot never moves during
  loading — fz and ai0 both see the same static gravity load through the
  load cell. Logs a session CSV (fz, ai0, TCP pose) at a known sample rate.

  `fit` (offline) — extracts one (fz_robot, lc_futek) sample pair per
  loaded window from logged session CSVs (dwell-plateau mean, first 0.3 s
  after each weight placement dropped, per-session zero subtracted), fits
  slope/offset by OLS, validates against a held-out session (RMSE, Pearson
  r, Bland-Altman), checks the SOP's acceptance criteria, and saves
  calib_fz_<tip>.json.

Usage:
  # 1. Live data collection (run once per fit session, once for holdout):
  python force_calib.py collect --tip futek_direct \
      --weights 50 100 200 500 1000 --reps 3

  # 2. Offline fit + validation:
  python force_calib.py fit --tip futek_direct \
      --fit logs/fzcal_futek_direct_session_*.csv \
      --holdout logs/fzcal_futek_direct_session_<held_out>.csv
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

SETTLE_S = 0.3           # dropped from the start of each loaded window (placement settling)
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


# ── Step 3: extract per-loaded-window (fz_robot, lc_futek) pairs ──────
def load_session(path):
    df = pd.read_csv(path)
    df["t"] = df["timestamp"] - df["timestamp"].iloc[0]
    df["loaded"] = pd.to_numeric(
        df["loaded"], errors="coerce").fillna(0).astype(int)
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
    """Pre-load idle mean (Step 1); falls back to signal min if the
    recording starts already loaded (no idle window available)."""
    first_loaded = df.index[df["loaded"] == 1]
    idle = df.loc[: first_loaded[0] - 1] if len(first_loaded) else df.iloc[0:0]
    if len(idle) >= 5:
        return float(idle["fz"].mean()), float(idle["lc_n"].mean())
    print(f"    [warn] no idle window before first loaded window — "
          f"falling back to signal min for zero-reference")
    return float(df["fz"].min()), float(df["lc_n"].min())


def load_windows(df):
    """Contiguous loaded==1 runs, as (start_idx, end_idx) pairs."""
    loaded = df["loaded"].to_numpy()
    edges = np.diff(np.concatenate(([0], loaded, [0])))
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
    for start, end in load_windows(df):
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
        print(f"  {os.path.basename(f)}: {len(pairs)} loaded events "
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


def cmd_collect(args):
    if rtde_control is None or rtde_receive is None:
        sys.exit("ur_rtde is not installed — "
                  "pip install ur-rtde --break-system-packages")

    import ur5_control

    robot_ip = args.robot_ip or ur5_control.ROBOT_IP

    # ── Perpendicularity check ──
    # The load cell and weights sit in the tool's static load path (no
    # pressing motion) — if the tool Z-axis isn't vertical, gravity's
    # force vector leaks into fx/fy and fz under-reads the true weight.
    tilt = tilt_from_vertical_deg(ur5_control.REFERENCE_POSE[3:6])
    print(f"[collect] Tool Z-axis tilt from vertical: "
          f"{tilt:.3f} deg (tolerance {args.tilt_tol_deg:.2f} deg)")
    if tilt > args.tilt_tol_deg:
        sys.exit(
            f"[collect] ABORT — tool axis is {tilt:.2f} deg off vertical, "
            f"exceeds {args.tilt_tol_deg:.2f} deg tolerance. An off-axis "
            "load path means the applied weight's gravity vector isn't "
            "aligned with the tool Z-axis, so fz under-reads the true "
            "force. Re-square the load cell mount or re-teach "
            "REFERENCE_POSE in ur5_control.py, then re-run."
        )

    print(f"[collect] Connecting to {robot_ip} ...")
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    print("[collect] Connected.")

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
    print(f"[collect] Feed this file to `fit --fit/--holdout` next.")


# ── CLI ──────────────────────────────────────────────────────────────
def build_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    c = sub.add_parser("collect", help="live: hold a static pose while known weights are applied to the load cell, and log a session CSV")
    c.add_argument("--tip", required=True, help="config/fixture label used for filenames, e.g. futek_direct")
    c.add_argument("--weights", type=float, nargs="+", required=True,
                   help="known weights in grams to apply, e.g. 50 100 200 500 1000")
    c.add_argument("--reps", type=int, default=3, help="place/remove cycles per weight")
    c.add_argument("--dwell", type=float, default=1.5, help="dwell time per loaded rep (s)")
    c.add_argument("--idle-s", type=float, default=3.0, help="idle baseline duration before loading (s)")
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
