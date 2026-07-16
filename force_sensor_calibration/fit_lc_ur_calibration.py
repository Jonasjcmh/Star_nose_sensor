#!/usr/bin/env python3
"""
fit_lc_ur_calibration.py — fit across the 5-200 g posz/negz calibration
datasets, for both instruments (FUTEK load cell and UR wrist force
sensor). Produces the three numbers this calibration exists to answer:

  1. Load-cell voltage <-> force change rate (ai0 sensitivity, N/V and V/N),
     fit from fzcal_futek_direct_* against the known applied weight.
  2. The relation between the load-cell measurement (F_lc, from the fit in
     (1)) and the UR wrist's own fz reading — using SAME-SESSION,
     same-timestamp rows (fzcal_futek_direct_* logs fz and ai0 together),
     not cross-session weight matching.
  3. Compensation coefficients for the UR sensor, taking the load cell as
     the reference pattern, fit twice: once on raw fz (mixes push/pull,
     pooled R^2 ~0.1 — only the per-direction fits are usable), and once
     on SIGN-CORRECTED fz — fz_signed = AI0_SIGN[direction] * |fz_raw| —
     which puts the UR reading on the same push/pull convention the load
     cell already has, giving ONE pooled line that actually fits (R^2
     ~0.9): F_lc ~= a * fz_signed + b, so
     fz_corrected = a * (AI0_SIGN[direction] * abs(fz_raw)) + b can be
     applied once the FUTEK is gone. This sign-corrected pooled fit is
     the one saved to calib_fz_lc_pattern.json.

fzcal_ur_only_* (no load cell in the chain) is used as an independent,
second characterization of (3) against the known weight directly — useful
as a same-instrument, different-session repeatability cross-check, not
folded into the primary fit.

Baseline is a known load, not a zero reference (both instruments)
--------------------------------------------------------------------
For BOTH futek_direct and ur_only, the hardware (load cell + holder/hook
for futek_direct; attachment + screws + holder/hook for ur_only) is
already resting on the sensor during the no-load baseline (loaded==0)
too — it isn't removed between the baseline and loaded phases. So the
baseline is a known, non-zero load in its own right, not something to
zero out. Each de-duplicated session therefore contributes TWO absolute
points (baseline, loaded), not one baseline-compensated delta:
  futek_direct:  (ai0_base_mean, F_signed_base = hardware only)
                 (ai0_load_mean, F_signed      = hardware + weight)
  ur_only:       (fz_base_mean,  F_true_base   = hardware only)
                 (fz_load_mean,  F_true        = hardware + weight)
(A pure delta would have cancelled the hardware term exactly, since it's
identical in both phases — so adding hardware mass only matters, and only
makes sense, once the baseline is treated this way instead of subtracted
away.)

Ground truth
------------
F_true = (total_g / 1000) * g * cos(tilt_from_vertical), where total_g is
the nominal placed weight plus the hardware mass between the sensor and
the weight:
  - fzcal_ur_only_* (no load cell): total_g = weight_g + EXTRA_HARDWARE_G_UR_ONLY
    -> 43 g (posz) / 37 g (negz).
  - fzcal_futek_direct_*: TWO different hardware totals in the SAME rig,
    depending on which sensor's ground truth you're computing, because
    the load cell only feels what's mounted ABOVE it while the UR feels
    everything below IT (including the load cell's own body):
      * load cell's own reading (F_true/F_signed, used for the Step 1
        ai0<->force fit): total_g = weight_g + EXTRA_HARDWARE_G_FUTEK_DIRECT
        -> 7 g (posz, holder) / 4 g (negz, hook) -- only what's above the
        load cell.
      * UR sensor's own reading (F_true_ur/F_signed_ur, used whenever fz
        is compared directly against a known weight, bypassing the load
        cell): total_g = weight_g + EXTRA_HARDWARE_G_FUTEK_DIRECT_UR
        -> 50 g (posz) / 47 g (negz) -- holder/hook PLUS everything below
        the UR flange and above the holder/hook: the 3D-printed coupler
        (15 g, the same piece used in every experiment, including
        ur_only) + 4 attachment screws (21 g) + the load cell's own body
        (7 g) = 43 g, common to both directions, plus the holder (7 g,
        posz) or hook (4 g, negz).
Both hardware masses were confirmed by whoever ran the collection.

Which datasets get used (fully automatic -- no code edits needed when
new sessions are collected)
-------------------------------------------------------------------------
Filenames may carry a version tag (fzcal_..._v2_..., v3, ...); an
un-tagged file is implicitly v1. Per instrument (futek_direct, ur_only
tracked separately), only the HIGHEST version number present is trusted
-- older batches are discarded wholesale, not merged weight-by-weight,
because a whole collection batch shares one day's session-to-session
baseline drift that isn't comparable across batches (see
keep_latest_version). Within that latest batch, files are grouped by
(instrument, direction, weight rounded to the nearest gram); if a group
still has more than one file (e.g. a re-run within the same batch), only
the chronologically last one is kept (plus a couple of manual exclusions
for mislabeled one-off attempts, see EXCLUDED_SESSIONS). Run the script
to see exactly which files were selected -- it always prints a
[datasets] manifest before fitting anything.

Usage:
    python fit_lc_ur_calibration.py
"""

import os
import re
import csv
import glob
import json
from datetime import date

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
OUT_DIR = os.path.join(HERE, "plots")
os.makedirs(OUT_DIR, exist_ok=True)

G = 9.80665  # m/s^2

# ai0 sign convention (see plot_force_vs_ai0.py): +z (posz) pushes the
# bridge voltage down, -z (negz) pulls it up. This is a hardware fact, not
# an assumption — checked against raw data (posz dV<0, negz dV>0).
AI0_SIGN = {"posz": -1.0, "negz": 1.0}

# Hardware mass (g) felt by the UR sensor in the ur_only (no load cell)
# rig, on top of the nominal test weight: 3D-printed attachment (15 g) +
# 4 screws (21 g) are common to both directions, plus the holder (7 g,
# posz) or the hook (1 g, negz) used in THIS rig (no load cell).
EXTRA_HARDWARE_G_UR_ONLY = {"posz": 43.0, "negz": 37.0}

# Hardware mass (g) felt by the load cell itself in the futek_direct rig,
# on top of the nominal test weight: only what's mounted ABOVE the load
# cell in the load path -- the holder (7 g, posz, same physical holder as
# ur_only) or the hook (4 g, negz -- a DIFFERENT, heavier hook than the
# 1 g one used in ur_only; confirmed by whoever ran the collection, not
# an inconsistency). Used ONLY for the ai0<->force (Step 1) fit's ground
# truth.
EXTRA_HARDWARE_G_FUTEK_DIRECT = {"posz": 7.0, "negz": 4.0}

# Hardware mass (g) felt by the UR sensor itself in the SAME futek_direct
# rig: the UR holds up everything below it in the load path -- the 3D-
# printed coupler (15 g, common to every experiment) + 4 attachment
# screws (21 g) + the load cell's own body (7 g) = 43 g, common to both
# directions, plus the holder (7 g, posz) or the (4 g, negz) hook above
# the load cell. Used for any ground truth the UR's fz is compared
# against directly (bypassing the load cell) -- NOT for the ai0 fit,
# which uses the value above.
EXTRA_HARDWARE_G_FUTEK_DIRECT_UR = {"posz": 50.0, "negz": 47.0}

WEIGHT_COLORS = {
    5: "#1f77b4", 10: "#ff7f0e", 20: "#2ca02c",
    50: "#d62728", 100: "#9467bd", 200: "#8c564b",
}

# Toggle: show the "{real_g}g->{F_signed}N" text label next to each
# loaded point in Figure 1. Off by default (cleaner plot); flip to True
# to bring the per-point annotations back.
SHOW_POINT_LABELS = False

matplotlib.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
    "font.size": 10,
})

FNAME_RE = re.compile(
    r"fzcal_(?P<instrument>futek_direct|ur_only)_(?P<direction>posz|negz)_"
    r"(?P<weight>\d+(?:\.\d+)?)g_(?:(?P<version>v\d+)_)?(?P<ts>\d{8}_\d{6})\.csv$"
)
# The 20260715 negz v2 re-collection drops the direction token from the
# filename entirely (fzcal_futek_direct_100g_v2_..._meta.json instead of
# fzcal_futek_direct_negz_100g_v2_...) -- direction is only recoverable
# from that session's own meta.json ("axis" field). Tried as a fallback
# when FNAME_RE doesn't match.
FNAME_RE_NO_DIR = re.compile(
    r"fzcal_(?P<instrument>futek_direct|ur_only)_"
    r"(?P<weight>\d+(?:\.\d+)?)g_(?:(?P<version>v\d+)_)?(?P<ts>\d{8}_\d{6})\.csv$"
)


# ── discovery + de-duplication ──────────────────────────────────────────
# Groups sessions targeting "the same" weight without a hand-maintained
# list of expected weights (that list needed a code edit every time a new
# nominal weight was collected -- see git history). Real weights are
# clean grams already, so nearest-gram rounding groups repeat sessions of
# the same target weight while keeping genuinely distinct weights (5g vs
# 10g, or a newly-added 42g) apart automatically.
def nominal_weight_g(weight_g):
    return round(weight_g)


def parse_version(version_str):
    """Un-tagged filenames (no "_v2_" etc.) are implicitly version 1."""
    return int(version_str[1:]) if version_str else 1


# Manual exclusion, confirmed by whoever ran the collection: these two
# ur_only negz files (labeled 201g and 202g) are one-off mislabeled
# attempts that don't represent a real, distinct test point -- ignore them
# entirely. The plain 20g file (fzcal_ur_only_negz_20g_..._180618.csv) is
# used for the 20g point instead.
EXCLUDED_SESSIONS = {
    ("ur_only", "negz", "20260706_181552"),  # labeled 201g
    ("ur_only", "negz", "20260706_181726"),  # labeled 202g
}


def discover(instrument):
    entries = []
    for csv_path in sorted(glob.glob(os.path.join(LOG_DIR, f"fzcal_{instrument}_*.csv"))):
        fname = os.path.basename(csv_path)
        m = FNAME_RE.search(fname)
        if m:
            direction = m.group("direction")
        else:
            m = FNAME_RE_NO_DIR.search(fname)
            if not m:
                continue
            meta_path = csv_path.replace(".csv", "_meta.json")
            with open(meta_path) as f:
                direction = json.load(f)["axis"]
        ts = m.group("ts")
        if (instrument, direction, ts) in EXCLUDED_SESSIONS:
            continue
        weight_g = float(m.group("weight"))
        entries.append({
            "instrument": instrument,
            "direction": direction,
            "weight_g": weight_g,
            "nominal_weight_g": nominal_weight_g(weight_g),
            "ts": ts,
            "version": m.group("version"),  # e.g. "v2", or None for un-tagged files
            "csv_path": csv_path,
            "meta_path": csv_path.replace(".csv", "_meta.json"),
        })
    return entries


def dedupe_latest(entries):
    """Keep only the chronologically-last file per (instrument, direction,
    nominal weight) group."""
    groups = {}
    for e in entries:
        key = (e["instrument"], e["direction"], e["nominal_weight_g"])
        groups.setdefault(key, []).append(e)

    kept = []
    for key in sorted(groups):
        group = sorted(groups[key], key=lambda e: e["ts"])
        if len(group) > 1:
            dropped = ", ".join(f"{e['weight_g']:g}g@{e['ts']}" for e in group[:-1])
            print(f"[dedupe] {key[0]:>12} {key[1]:>4} {key[2]:>5.0f}g: "
                  f"keeping {group[-1]['weight_g']:g}g@{group[-1]['ts']} "
                  f"(dropping {dropped})")
        kept.append(group[-1])
    return kept


# Each full re-collection (v1 implied, then v2, v3, ...) fully supersedes
# the previous one, PER INSTRUMENT: a leftover session from an older
# batch with no counterpart in the newest one (e.g. a v1-only weight, no
# vN match) has its own baseline reading, which drifts ~1 N session-to-
# session -- see the v2 fix that motivated this (git history) -- so
# mixing an old batch's leftover into the newest batch corrupts the fit
# instead of adding data. Keep only the highest version number present
# for each instrument; hardcodes no specific version number, so the next
# re-collection (v3, v4, ...) is picked up with no code change.
def keep_latest_version(entries):
    by_instrument = {}
    for e in entries:
        by_instrument.setdefault(e["instrument"], []).append(e)

    kept = []
    for instrument, group in by_instrument.items():
        max_version = max(parse_version(e["version"]) for e in group)
        kept.extend(e for e in group if parse_version(e["version"]) == max_version)
    return kept


def print_dataset_manifest(entries, label):
    """Easy way to see exactly which files feed a given instrument's fit
    -- always printed (unlike the [dedupe] lines, which only fire when a
    group had more than one candidate), so a re-collection's effect on
    what's in use is visible at a glance."""
    print(f"\n[datasets] {label}: {len(entries)} file(s) in use")
    for e in sorted(entries, key=lambda e: (e["direction"], e["nominal_weight_g"])):
        version_label = e["version"] or "v1 (untagged)"
        print(f"  {e['direction']:>4} {e['nominal_weight_g']:>6.0f}g  {version_label:<14} "
              f"{os.path.basename(e['csv_path'])}")


# ── per-session load ─────────────────────────────────────────────────────
def load_session(entry):
    with open(entry["meta_path"]) as f:
        meta = json.load(f)
    with open(entry["csv_path"]) as f:
        rows = list(csv.DictReader(f))

    loaded = np.array([int(r["loaded"]) for r in rows], dtype=bool)
    fz = np.array([float(r["fz"]) for r in rows])
    fz_base_mean = fz[~loaded].mean()
    fz_load_mean = fz[loaded].mean()
    dfz_load = fz[loaded] - fz_base_mean  # kept for reference/diagnostics only

    result = dict(entry)
    result.update({
        "tilt_deg": meta["tilt_from_vertical_deg"],
        "dfz_mean": float(dfz_load.mean()),
        "dfz_std": float(dfz_load.std()),
        "fz_base_mean": float(fz_base_mean),
        "fz_load_mean": float(fz_load_mean),
    })

    if entry["instrument"] == "futek_direct":
        ai0 = np.array([float(r["ai0"]) for r in rows])
        ai0_base_mean = ai0[~loaded].mean()
        ai0_load_mean = ai0[loaded].mean()
        result.update({
            "dv_mean": float(ai0_load_mean - ai0_base_mean),
            "dv_std": float((ai0[loaded] - ai0_base_mean).std()),
            "ai0_base_mean": float(ai0_base_mean),
            "ai0_load_mean": float(ai0_load_mean),
        })

    tilt_rad = np.deg2rad(meta["tilt_from_vertical_deg"])
    hardware_g = (EXTRA_HARDWARE_G_UR_ONLY[entry["direction"]] if entry["instrument"] == "ur_only"
                  else EXTRA_HARDWARE_G_FUTEK_DIRECT[entry["direction"]])
    total_g = entry["weight_g"] + hardware_g

    F_true_base = hardware_g / 1000.0 * G * np.cos(tilt_rad)
    F_true = total_g / 1000.0 * G * np.cos(tilt_rad)
    result["F_true_base"] = float(F_true_base)
    result["F_true"] = float(F_true)
    # F_signed/F_signed_base orient the ground truth by measurement
    # direction (push=posz vs pull=negz), same AI0_SIGN convention used to
    # sign fz -- for BOTH instruments, since the direction is a property of
    # the experiment (which way the load was applied), not of which sensor
    # is reading it. Needed so a sign-corrected fz can be pooled against a
    # ground truth that's ALSO on the same sign convention (see
    # plot_ur_only_vs_load.py).
    result["F_signed"] = float(AI0_SIGN[entry["direction"]] * F_true)
    result["F_signed_base"] = float(AI0_SIGN[entry["direction"]] * F_true_base)
    if entry["instrument"] == "futek_direct":
        # Separate ground truth for the UR sensor itself in this SAME rig:
        # the UR holds up the load cell's own body too, not just what the
        # load cell feels (see EXTRA_HARDWARE_G_FUTEK_DIRECT_UR above).
        hardware_g_ur = EXTRA_HARDWARE_G_FUTEK_DIRECT_UR[entry["direction"]]
        total_g_ur = entry["weight_g"] + hardware_g_ur
        F_true_ur_base = hardware_g_ur / 1000.0 * G * np.cos(tilt_rad)
        F_true_ur = total_g_ur / 1000.0 * G * np.cos(tilt_rad)
        result["F_true_ur_base"] = float(F_true_ur_base)
        result["F_true_ur"] = float(F_true_ur)
        result["F_signed_ur_base"] = float(AI0_SIGN[entry["direction"]] * F_true_ur_base)
        result["F_signed_ur"] = float(AI0_SIGN[entry["direction"]] * F_true_ur)
    return result


def expand_phases(sessions):
    """Turn futek_direct sessions into a flat list of per-phase point
    dicts, 2 per session (baseline then loaded) — baseline is a known,
    non-zero load here (see module docstring), not a zero reference."""
    points = []
    for s in sessions:
        sign = AI0_SIGN[s["direction"]]
        hardware_g = EXTRA_HARDWARE_G_FUTEK_DIRECT[s["direction"]]
        points.append({"session": s, "phase": "baseline",
                        "ai0": s["ai0_base_mean"], "fz": s["fz_base_mean"],
                        "fz_signed": sign * abs(s["fz_base_mean"]),
                        "F_signed": s["F_signed_base"],
                        "compensated_g": hardware_g})
        points.append({"session": s, "phase": "loaded",
                        "ai0": s["ai0_load_mean"], "fz": s["fz_load_mean"],
                        "fz_signed": sign * abs(s["fz_load_mean"]),
                        "F_signed": s["F_signed"],
                        "compensated_g": s["weight_g"] + hardware_g})
    return points


def linfit(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    m, c = np.polyfit(x, y, 1)
    y_pred = m * x + c
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rmse = float(np.sqrt(np.mean((y - y_pred) ** 2)))
    return float(m), float(c), float(r2), rmse


# ── Step 1: load-cell voltage <-> force change rate ─────────────────────
def fit_loadcell_rate(points):
    """F_signed = m_v * ai0 + c_v (RAW absolute voltage, no baseline
    subtraction), using both the baseline and loaded points from every
    futek_direct session (see expand_phases)."""
    ai0 = [p["ai0"] for p in points]
    f_signed = [p["F_signed"] for p in points]
    return linfit(ai0, f_signed)


# ── Step 2/3: UR fz vs load-cell force, same-session/same-phase pairing ─
def fit_ur_compensation(points, m_v, c_v):
    """F_lc ~= a * fz_robot + b (SOP Step 4 form). F_lc per point comes
    from that SAME phase's own ai0, run through the Step-1 fit — not from
    the known weight directly, so the UR is corrected against what the
    load cell actually reported."""
    fz = [p["fz"] for p in points]
    flc = [m_v * p["ai0"] + c_v for p in points]
    a, b, r2, rmse = linfit(fz, flc)
    return a, b, r2, rmse, flc


def fit_ur_compensation_by_direction(points, m_v, c_v):
    """Same F_lc ~= a*fz_robot + b form as fit_ur_compensation, but fit
    separately for posz and negz. A single pooled (a, b) only makes sense
    if the two directions actually agree — this checks that assumption
    instead of assuming it."""
    out = {}
    for direction in ("posz", "negz"):
        dir_points = [p for p in points if p["session"]["direction"] == direction]
        fz = [p["fz"] for p in dir_points]
        flc = [m_v * p["ai0"] + c_v for p in dir_points]
        a, b, r2, rmse = linfit(fz, flc)
        out[direction] = {"slope": a, "offset": b, "r2": r2, "rmse": rmse, "n": len(dir_points)}
    return out


# ── Step 4: ur_only independent cross-check vs known weight ─────────────
def fit_ur_only_vs_trueweight(sessions):
    """F_true ~= a_ur * fz_robot + b_ur, from the ur_only sessions (no load
    cell in the chain), fit against the known weight directly. 2 points
    per session (baseline, loaded) -> more data, but uses absolute fz (not
    a per-file delta), so it re-exposes session-to-session drift that a
    delta would hide."""
    fz_all, f_true_all = [], []
    for s in sessions:
        fz_all.append(s["fz_base_mean"]); f_true_all.append(s["F_true_base"])
        fz_all.append(s["fz_load_mean"]); f_true_all.append(s["F_true"])
    return linfit(fz_all, f_true_all)


# ── Bland-Altman agreement diagnostic ────────────────────────────────────
def bland_altman(reference, measurement):
    """Classic Bland-Altman: mean of the pair (x) vs difference (y), plus
    the overall bias and +/-1.96 sigma limits of agreement. Used here on
    the POOLED compensation's corrected fz vs F_lc, so it shows exactly
    where a single deployed correction would over/under-shoot — not just
    an R^2 number."""
    reference = np.asarray(reference, dtype=float)
    measurement = np.asarray(measurement, dtype=float)
    diff = measurement - reference
    mean_pair = (measurement + reference) / 2.0
    bias = float(diff.mean())
    sd = float(diff.std())
    return mean_pair, diff, bias, bias - 1.96 * sd, bias + 1.96 * sd


def main():
    futek_entries = dedupe_latest(keep_latest_version(discover("futek_direct")))
    ur_entries = dedupe_latest(keep_latest_version(discover("ur_only")))
    print_dataset_manifest(futek_entries, "futek_direct")
    print_dataset_manifest(ur_entries, "ur_only")

    futek_sessions = sorted((load_session(e) for e in futek_entries),
                             key=lambda s: (s["direction"], s["weight_g"]))
    ur_sessions = sorted((load_session(e) for e in ur_entries),
                          key=lambda s: (s["direction"], s["weight_g"]))

    points = expand_phases(futek_sessions)

    # ── Step 1 ──
    m_v, c_v, r2_v, rmse_v = fit_loadcell_rate(points)
    print("=" * 78)
    print("STEP 1 — FUTEK load cell: voltage <-> force change rate")
    print("baseline (hardware only) and loaded (hardware+weight) each a real, known point")
    print("=" * 78)
    print(f"{'weight_g':>8}{'dir':>6}{'phase':>9}{'real_g':>8}{'ai0(V)':>10}"
          f"{'F_expected(N)':>15}{'F_pred(N)':>11}{'resid(N)':>10}")
    for p in points:
        s = p["session"]
        f_pred = m_v * p["ai0"] + c_v
        print(f"{s['weight_g']:>8.0f}{s['direction']:>6}{p['phase']:>9}{p['compensated_g']:>8.0f}"
              f"{p['ai0']:>10.4f}{p['F_signed']:>+15.4f}{f_pred:>+11.4f}{f_pred - p['F_signed']:>+10.4f}")
    print("  real_g = nominal weight + hardware mass (holder/hook, 7g posz / 4g negz)")
    print("  F_expected(N) = F_signed, the ground truth force from real_g (what SHOULD be measured)")
    print("  F_pred(N) = the fit's own estimate from this point's ai0 voltage")
    print(f"\nn = {len(points)} points ({len(futek_sessions)} sessions x 2 phases each)")
    print(f"F_signed = {m_v:.4f} * ai0 + ({c_v:.5f})   R^2 = {r2_v:.5f}   RMSE = {rmse_v:.4f} N")
    print(f"  -> load-cell sensitivity: {m_v:.4f} N/V  (equivalently {1.0/m_v*1000:.3f} mV/N, "
          f"i.e. {1.0/m_v:.5f} V/N change rate)")

    # ── Step 2/3 ──
    a, b, r2_comp, rmse_comp, flc = fit_ur_compensation(points, m_v, c_v)
    print()
    print("=" * 78)
    print("STEP 2/3 — F_lc vs UR fz (same session/phase) + UR compensation coefficients")
    print("=" * 78)
    print(f"{'weight_g':>8}{'dir':>6}{'phase':>9}{'fz(N)':>10}{'F_lc(N)':>10}")
    for p, f in zip(points, flc):
        s = p["session"]
        print(f"{s['weight_g']:>8.0f}{s['direction']:>6}{p['phase']:>9}{p['fz']:>10.4f}{f:>10.4f}")
    print(f"\nn = {len(points)} points ({len(futek_sessions)} sessions x 2 phases each)")
    print(f"F_lc = {a:.4f} * fz_robot + ({b:.5f})   R^2 = {r2_comp:.5f}   RMSE = {rmse_comp:.4f} N")
    print(f"  -> apply as: fz_corrected = {a:.4f} * fz_raw + ({b:.5f})")

    # ── Step 2/3b: does ONE pooled correction actually make sense? ──
    by_dir = fit_ur_compensation_by_direction(points, m_v, c_v)
    print()
    print("  Per-direction compensation (same F_lc = a*fz + b form, fit separately):")
    for direction in ("posz", "negz"):
        d = by_dir[direction]
        print(f"    {direction}: F_lc = {d['slope']:.4f} * fz_robot + ({d['offset']:.5f})   "
              f"R^2 = {d['r2']:.5f}   RMSE = {d['rmse']:.4f} N   (n={d['n']} points)")
    print(f"    -> pooling hides a {abs(by_dir['posz']['slope'] - by_dir['negz']['slope']):.3f} "
          f"slope gap and {abs(by_dir['posz']['offset'] - by_dir['negz']['offset']):.3f} N "
          f"offset gap between directions.")

    # ── Step 2/3c: sign-corrected fz — same push/pull directionality as
    # the load cell (fz_signed = AI0_SIGN[direction] * |fz_raw|), so a
    # SINGLE pooled correction is meaningful instead of the two disagreeing
    # per-direction lines above. ──
    fz_signed_arr = np.array([p["fz_signed"] for p in points])
    a_s, b_s, r2_s, rmse_s = linfit(fz_signed_arr, flc)
    print()
    print("  Sign-corrected UR compensation (fz_signed = AI0_SIGN[direction] * |fz_raw|, "
          "matching the load cell's own push/pull convention):")
    print(f"    F_lc = {a_s:.4f} * fz_signed + ({b_s:.5f})   R^2 = {r2_s:.5f}   RMSE = {rmse_s:.4f} N")
    print(f"    -> apply as: fz_corrected = {a_s:.4f} * (AI0_SIGN[direction] * |fz_raw|) + ({b_s:.5f}), "
          f"one formula for both directions (R^2 {r2_s:.4f} vs {r2_comp:.4f} pooled-raw)")

    # ── Bland-Altman: where does the DEPLOYABLE (sign-corrected pooled)
    # correction actually agree? Using the raw pooled fit here would just
    # restate how badly pooling-without-signing fails (R^2 ~0.1) — now
    # that a genuine single pooled correction exists, that's the one worth
    # checking agreement bounds for. ──
    fz_corrected_pooled_signed = a_s * fz_signed_arr + b_s
    mean_pair, ba_diff, ba_bias, ba_lo, ba_hi = bland_altman(flc, fz_corrected_pooled_signed)
    print()
    print("  Bland-Altman diagnostic (sign-corrected pooled correction: fz_corrected vs F_lc):")
    print(f"    overall bias = {ba_bias:+.4f} N   limits of agreement = [{ba_lo:+.4f}, {ba_hi:+.4f}] N")
    for direction in ("posz", "negz"):
        dir_diff = [d for p, d in zip(points, ba_diff) if p["session"]["direction"] == direction]
        print(f"    {direction} bias = {np.mean(dir_diff):+.4f} N "
              f"(std {np.std(dir_diff):.4f} N, n={len(dir_diff)})")

    # ── Step 4 (independent cross-check) ──
    a_ur, b_ur, r2_ur, rmse_ur = fit_ur_only_vs_trueweight(ur_sessions)
    print()
    print("=" * 78)
    print("STEP 4 — ur_only cross-check (no load cell, vs known weight directly)")
    print("baseline (hardware only) and loaded (hardware+weight) each a real, known point")
    print("=" * 78)
    print(f"{'weight_g':>8}{'dir':>6}{'phase':>9}{'fz_abs(N)':>11}{'F_true(N)':>11}")
    for s in ur_sessions:
        print(f"{s['weight_g']:>8.0f}{s['direction']:>6}{'baseline':>9}"
              f"{s['fz_base_mean']:>11.4f}{s['F_true_base']:>11.4f}")
        print(f"{s['weight_g']:>8.0f}{s['direction']:>6}{'loaded':>9}"
              f"{s['fz_load_mean']:>11.4f}{s['F_true']:>11.4f}")
    print(f"\nn = {2 * len(ur_sessions)} points ({len(ur_sessions)} sessions x 2 phases each)")
    print(f"F_true = {a_ur:.4f} * fz_robot + ({b_ur:.5f})   R^2 = {r2_ur:.5f}   RMSE = {rmse_ur:.4f} N")
    print(f"  compare vs Step 3 slope/offset: a={a:.4f} vs a_ur={a_ur:.4f} "
          f"({abs(a - a_ur) / abs(a) * 100:.1f}% apart), "
          f"b={b:.4f} vs b_ur={b_ur:.4f}")
    print("=" * 78)

    # ── persist compensation coefficients (SOP Step 7 format) ──
    # Top-level slope/offset is the SIGN-CORRECTED pooled fit -- it's the
    # one deployable formula (fz_signed = AI0_SIGN[direction]*|fz_raw|);
    # the raw pooled fit is kept alongside it for reference/diagnostics
    # only (it mixes push and pull, R^2 ~0.1).
    calib_out = {
        "tip": "futek_direct",
        "date": date.today().isoformat(),
        "slope": a_s,
        "offset": b_s,
        "r_squared": r2_s,
        "rmse_n": rmse_s,
        "fz_input": "fz_signed = AI0_SIGN[direction] * abs(fz_raw); "
                    "AI0_SIGN = {posz: -1, negz: +1}",
        "n_samples": len(points),
        "n_sessions": len(futek_sessions),
        "loadcell_rate_n_per_v": m_v,
        "loadcell_rate_v_per_n": 1.0 / m_v,
        "note": "F_true includes hardware mass for both instruments: ur_only uses "
                "EXTRA_HARDWARE_G_UR_ONLY (43g posz / 37g negz); futek_direct uses "
                "EXTRA_HARDWARE_G_FUTEK_DIRECT (7g posz / 4g negz). Baseline is "
                "treated as a known non-zero load (hardware only), not a zero "
                "reference -- n_samples = sessions x 2 (baseline + loaded).",
        "raw_unsigned_pooled": {
            "slope": a, "offset": b, "r_squared": r2_comp, "rmse_n": rmse_comp,
            "note": "fit on raw fz (no sign correction) -- pools push and pull "
                    "into one line and fits neither well; kept for reference.",
        },
        "cross_check": {
            "tip": "ur_only", "slope": a_ur, "offset": b_ur,
            "r_squared": r2_ur, "rmse_n": rmse_ur,
            "n_samples": 2 * len(ur_sessions),
            "n_sessions": len(ur_sessions),
        },
        "per_direction": by_dir,
        "bland_altman_pooled": {
            "note": "sign-corrected pooled correction vs F_lc",
            "bias_n": ba_bias, "loa_lower_n": ba_lo, "loa_upper_n": ba_hi,
        },
    }
    calib_path = os.path.join(HERE, "calib_fz_lc_pattern.json")
    with open(calib_path, "w") as f:
        json.dump(calib_out, f, indent=2)
    print(f"\nSaved compensation coefficients -> {os.path.relpath(calib_path, HERE)}")

    # ── plot: 4 separate figures, one per step, instead of one 2x2 grid ──
    def phase_style(p):
        s = p["session"]
        color = WEIGHT_COLORS.get(int(s["weight_g"]), "#333")
        marker = "o" if s["direction"] == "posz" else "s"
        loaded = p["phase"] == "loaded"
        return dict(color=color, marker=marker,
                    facecolors=color if loaded else "none",
                    edgecolors=color, s=80 if loaded else 60,
                    linewidths=1 if loaded else 1.5)

    fig1, ax = plt.subplots(figsize=(8, 6.5))
    loaded_points = [p for p in points if p["phase"] == "loaded"]
    for p in loaded_points:
        st = phase_style(p)
        ax.scatter(p["ai0"], p["F_signed"], facecolors=st["facecolors"], edgecolors=st["edgecolors"],
                   marker=st["marker"], s=st["s"], linewidths=st["linewidths"], zorder=4,
                   label=f"{int(p['session']['weight_g'])} g")
        # annotate with the hardware-compensated real weight (nominal +
        # hardware) -- the expected total each point represents, not just
        # the nominal weight_g -- and the resulting F_signed value it
        # should measure. Toggle via SHOW_POINT_LABELS.
        if SHOW_POINT_LABELS:
            ax.annotate(f"{p['compensated_g']:.0f}g\N{RIGHTWARDS ARROW}{p['F_signed']:+.2f}N",
                        xy=(p["ai0"], p["F_signed"]), xytext=(6, 6), textcoords="offset points",
                        fontsize=10, color=st["color"])
    # fit itself still uses ALL points (baseline + loaded) -- baseline is
    # a known, non-zero point, not something to discard from the fit,
    # only from this display.
    ai0_all = np.array([p["ai0"] for p in points])
    margin = 0.05 * (ai0_all.max() - ai0_all.min())
    ai0_range = np.linspace(ai0_all.min() - margin, ai0_all.max() + margin, 200)
    ax.plot(ai0_range, m_v * ai0_range + c_v, "-", color="#1a1a1a", linewidth=2,
            label=f"F = {m_v:.3f}*ai0 + {c_v:.3f} (R²={r2_v:.4f})")
    ax.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax.set_xlabel("ai0, absolute (V)  [loaded points only]")
    ax.set_ylabel("F_signed (N), expected from real weight  [circle=posz, square=negz]")
    ax.set_title("Step 1 — load-cell voltage vs force (loaded points)")
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), fontsize=10, ncol=2, loc="upper left")
    fig1.suptitle("LC <-> UR calibration fit — Step 1: load-cell voltage vs force")
    fig1.tight_layout()
    out1 = os.path.join(OUT_DIR, "lc_ur_calibration_step1_voltage_force.png")
    fig1.savefig(out1, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out1, HERE)}")

    fig2, ax2 = plt.subplots(figsize=(8, 6.5))
    for p, f in zip(points, flc):
        st = phase_style(p)
        ax2.scatter(p["fz"], f, facecolors=st["facecolors"], edgecolors=st["edgecolors"],
                    marker=st["marker"], s=st["s"], linewidths=st["linewidths"], zorder=4,
                    label=f"{int(p['session']['weight_g'])} g")
    fz_all = np.array([p["fz"] for p in points])
    margin = 0.05 * (fz_all.max() - fz_all.min())
    fz_range = np.linspace(fz_all.min() - margin, fz_all.max() + margin, 200)
    ax2.plot(fz_range, a * fz_range + b, "-", color="#1a1a1a", linewidth=2,
             label=f"pooled: F_lc={a:.3f}*fz+{b:.3f} (R²={r2_comp:.4f})")
    ax2.plot(fz_range, by_dir["posz"]["slope"] * fz_range + by_dir["posz"]["offset"], ":",
             color="#1f77b4", linewidth=1.8,
             label=f"posz-only: F_lc={by_dir['posz']['slope']:.3f}*fz+{by_dir['posz']['offset']:.3f} "
                   f"(R²={by_dir['posz']['r2']:.4f})")
    ax2.plot(fz_range, by_dir["negz"]["slope"] * fz_range + by_dir["negz"]["offset"], ":",
             color="#d62728", linewidth=1.8,
             label=f"negz-only: F_lc={by_dir['negz']['slope']:.3f}*fz+{by_dir['negz']['offset']:.3f} "
                   f"(R²={by_dir['negz']['r2']:.4f})")
    ax2.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax2.axvline(0, color="gray", lw=0.8, alpha=0.5)
    ax2.set_xlabel("fz_ur, absolute (N)  [filled=loaded, open=baseline]")
    ax2.set_ylabel("F_lc (N)  [futek_direct, same-session/phase paired]")
    ax2.set_title("Step 2/3 — UR compensation vs load cell, raw fz (absolute)")
    handles, labels = ax2.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax2.legend(by_label.values(), by_label.keys(), fontsize=10, ncol=1, loc="upper left")
    fig2.suptitle("LC <-> UR calibration fit — Step 2/3: UR compensation vs load cell, raw fz")
    fig2.tight_layout()
    out2 = os.path.join(OUT_DIR, "lc_ur_calibration_step2_raw_fz.png")
    fig2.savefig(out2, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out2, HERE)}")

    # ── Figure 3: sign-corrected fz — same push/pull directionality as the
    # load cell, so ONE pooled line actually fits (vs the X-crossing raw
    # figure above) ──
    fig3, ax2s = plt.subplots(figsize=(8, 6.5))
    for p, f in zip(points, flc):
        st = phase_style(p)
        ax2s.scatter(p["fz_signed"], f, facecolors=st["facecolors"], edgecolors=st["edgecolors"],
                     marker=st["marker"], s=st["s"], linewidths=st["linewidths"], zorder=4,
                     label=f"{int(p['session']['weight_g'])} g")
    fz_signed_all = np.array([p["fz_signed"] for p in points])
    margin = 0.05 * (fz_signed_all.max() - fz_signed_all.min())
    fz_signed_range = np.linspace(fz_signed_all.min() - margin, fz_signed_all.max() + margin, 200)
    ax2s.plot(fz_signed_range, a_s * fz_signed_range + b_s, "-", color="#1a1a1a", linewidth=2,
              label=f"pooled: F_lc={a_s:.3f}*fz_signed+{b_s:.3f} (R²={r2_s:.4f})")
    ax2s.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax2s.axvline(0, color="gray", lw=0.8, alpha=0.5)
    ax2s.set_xlabel("fz_signed = AI0_SIGN[direction]*|fz_ur| (N)  [filled=loaded, open=baseline]")
    ax2s.set_ylabel("F_lc (N)  [futek_direct, same-session/phase paired]")
    ax2s.set_title("Step 2/3 — UR compensation vs load cell, sign-corrected fz\n"
                    "(same push/pull directionality as load cell)")
    handles, labels = ax2s.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax2s.legend(by_label.values(), by_label.keys(), fontsize=10, ncol=2, loc="upper left")
    fig3.suptitle("LC <-> UR calibration fit — Step 2/3: UR compensation vs load cell, sign-corrected fz")
    fig3.tight_layout()
    out3 = os.path.join(OUT_DIR, "lc_ur_calibration_step3_signed_fz.png")
    fig3.savefig(out3, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out3, HERE)}")

    # ── Figure 4: Bland-Altman diagnostic for the sign-corrected pooled
    # correction (the deployable one) ──
    fig4, ax3 = plt.subplots(figsize=(8, 6.5))
    for p, mp, d in zip(points, mean_pair, ba_diff):
        st = phase_style(p)
        ax3.scatter(mp, d, facecolors=st["facecolors"], edgecolors=st["edgecolors"],
                    marker=st["marker"], s=st["s"], linewidths=st["linewidths"], zorder=4,
                    label=f"{int(p['session']['weight_g'])} g")
    ax3.axhline(ba_bias, color="#1a1a1a", linewidth=2,
                label=f"bias = {ba_bias:+.3f} N")
    ax3.axhline(ba_lo, color="#888888", linestyle="--", linewidth=1.5,
                label=f"limits of agreement = [{ba_lo:+.3f}, {ba_hi:+.3f}] N")
    ax3.axhline(ba_hi, color="#888888", linestyle="--", linewidth=1.5)
    ax3.axhline(0, color="gray", lw=0.8, alpha=0.4)
    ax3.set_xlabel("mean(fz_corrected, F_lc) (N)")
    ax3.set_ylabel("fz_corrected - F_lc (N)  [circle=posz, square=negz]")
    ax3.set_title("Bland-Altman — sign-corrected pooled correction\nagreement with load cell")
    handles, labels = ax3.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax3.legend(by_label.values(), by_label.keys(), fontsize=10, ncol=2, loc="upper left")
    fig4.suptitle("LC <-> UR calibration fit — Bland-Altman (sign-corrected pooled correction)")
    fig4.tight_layout()
    out4 = os.path.join(OUT_DIR, "lc_ur_calibration_step4_bland_altman.png")
    fig4.savefig(out4, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out4, HERE)}")


if __name__ == "__main__":
    main()
