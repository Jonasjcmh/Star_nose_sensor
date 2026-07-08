#!/usr/bin/env python3
"""
plot_force_vs_ai0.py — load-cell calibration: signed force vs RAW ai0
(volts, no baseline subtraction), same measurement style as
futek_linearization.py's voltage_vs_force.png (one mean+/-std point per
weight/direction), fit F_signed = m*ai0 + c.

Data: fzcal_futek_direct*.csv, BOTH directions, for weights
5/10/20/50/100/200 g.
  - fzcal_futek_direct_posz_*   -> +z direction (holder)
  - fzcal_futek_direct_*        -> -z direction (hook-hung)
(fzcal_ur_only_* files are the UR wrist's own FT estimate — no load cell
mounted for those runs (meta "load_cell": false) — excluded here.)

Is "F" here really a resultant? Can ai0 go negative?
---------------------------------------------------------
This load cell is mounted on ONE axis only, so what it measures is a
single signed force component (like Fz), not a 3-axis magnitude — it CAN
be negative (+ for -z/hook-hung pulling one way, - for +z/holder pushing
the other), the same way Fz is signed for the UR wrist. Calling it
"resultant" in earlier versions of this script was a misnomer; a true
resultant (sqrt(fx^2+fy^2+fz^2)) can never be negative, but that is not
what's being measured here.

Raw ai0 is a different story: it's an absolute ADC voltage that NEVER
goes negative. Checked directly against the literal, unprocessed ai0
column in every fzcal_futek_direct*.csv file (no baseline subtraction,
no math): min = 4.1744 V, max = 5.1782 V. The bridge's zero-force point
sits at ~4.6-4.7 V (baked into the excitation/conditioning), and loading
it in either direction only ever nudges ai0 within that ~4.17-5.18 V
band — it's a hardware fact, not a processing artifact.

So: x-axis (raw ai0) is positive-only (~4.17-5.18 V) and y-axis (signed
F) spans negative to positive (~-1.96 N to +1.96 N). That asymmetry is
real and expected, not a bug.

F_true per weight = ((weight_g + extra_hardware_g) / 1000) * g * cos(tilt_from_vertical)

The load cell doesn't only feel the nominal test weight — it also feels
whatever hardware sits between it and the weight. That hardware differs
by direction:
  - posz (+z, holder-mounted)  -> holder, 7 g
  - negz (-z, hook-hung)       -> hook, 4 g
So e.g. a "200 g" posz run actually loads the cell with 207 g, and a
"200 g" negz run loads it with 204 g. Both offsets are added into F_true
below (EXTRA_HARDWARE_G).

Usage:
    python plot_force_vs_ai0.py
"""

import os
import glob
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
OUT_DIR = os.path.join(HERE, "plots")
os.makedirs(OUT_DIR, exist_ok=True)

G = 9.80665  # m/s^2

WEIGHT_COLORS = {
    5: "#1f77b4", 10: "#ff7f0e", 20: "#2ca02c",
    50: "#d62728", 100: "#9467bd", 200: "#8c564b",
}

# Extra hardware mass (g) between the load cell and the nominal test
# weight, felt by the load cell in addition to weight_g.
EXTRA_HARDWARE_G = {"+z": 7.0, "-z": 4.0}  # posz: holder, negz: hook

matplotlib.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.spines.top": False,
    "axes.spines.right": False,
})


def load_dataset(meta_path):
    with open(meta_path) as f:
        meta = json.load(f)
    csv_path = meta_path.replace("_meta.json", ".csv")
    ai0_base, ai0_load = [], []
    with open(csv_path) as f:
        header = f.readline().strip().split(",")
        i_loaded = header.index("loaded")
        i_ai0 = header.index("ai0")
        for line in f:
            parts = line.strip().split(",")
            loaded = int(parts[i_loaded])
            ai0 = float(parts[i_ai0])
            (ai0_load if loaded else ai0_base).append(ai0)

    direction = "+z" if "_posz_" in os.path.basename(csv_path) else "-z"
    tilt_rad = np.deg2rad(meta["tilt_from_vertical_deg"])
    total_weight_g = meta["weight_g"] + EXTRA_HARDWARE_G[direction]
    F_true = total_weight_g * G / 1000.0 * np.cos(tilt_rad)  # magnitude, always >= 0
    F_true_uncomp = meta["weight_g"] * G / 1000.0 * np.cos(tilt_rad)  # ignores hardware mass
    sign = -1.0 if direction == "+z" else +1.0  # +z pushes ai0 down, -z pushes ai0 up

    return {
        "weight_g": meta["weight_g"],
        "total_weight_g": total_weight_g,
        "direction": direction,
        "sign": sign,
        "ai0_base": np.array(ai0_base),   # raw volts, F_signed = 0
        "ai0_load": np.array(ai0_load),   # raw volts, F_signed = sign*F_true
        "F_true": F_true,
        "F_signed": sign * F_true,
        "F_signed_uncomp": sign * F_true_uncomp,
    }


def linfit(x, y):
    """F_signed = m*ai0 + c via least squares; returns m, c, R^2."""
    m, c = np.polyfit(x, y, 1)
    y_pred = m * x + c
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return m, c, r2


def pooled_xy(datasets, f_signed_key="F_signed"):
    x, y = [], []
    for d in datasets:
        x.append(d["ai0_base"]); y.append(np.zeros_like(d["ai0_base"]))
        x.append(d["ai0_load"]); y.append(np.full_like(d["ai0_load"], d[f_signed_key]))
    return np.concatenate(x), np.concatenate(y)


def main():
    meta_files = sorted(glob.glob(os.path.join(LOG_DIR, "fzcal_futek_direct*_meta.json")))
    if not meta_files:
        raise FileNotFoundError(f"No fzcal_futek_direct*_meta.json in {LOG_DIR}")

    datasets = [load_dataset(mf) for mf in meta_files]
    datasets.sort(key=lambda d: (d["direction"], d["weight_g"]))

    x_all, y_all = pooled_xy(datasets, "F_signed")
    m, c, r2 = linfit(x_all, y_all)

    x_all_u, y_all_u = pooled_xy(datasets, "F_signed_uncomp")
    m_u, c_u, r2_u = linfit(x_all_u, y_all_u)

    print("=" * 90)
    print("LOAD CELL CALIBRATION — F_signed = m*ai0 + c  (raw volts, no baseline subtraction)")
    print("compensated = weight_g + hardware (holder/hook); uncompensated = weight_g only")
    print("=" * 90)
    print(f"{'weight_g':>8}{'total_g':>8}{'dir':>6}{'ai0_load':>10}"
          f"{'F_comp(N)':>11}{'F_uncomp(N)':>12}{'shift(N)':>10}")
    for d in datasets:
        ai0_load_mean = d["ai0_load"].mean()
        shift = d["F_signed"] - d["F_signed_uncomp"]
        print(f"{d['weight_g']:>8.0f}{d['total_weight_g']:>8.0f}{d['direction']:>6}"
              f"{ai0_load_mean:>10.4f}{d['F_signed']:>+11.5f}{d['F_signed_uncomp']:>+12.5f}{shift:>+10.5f}")
    print()
    print(f"n = {len(x_all)} samples (baseline + loaded, both directions)")
    print(f"raw ai0 range: [{x_all.min():.4f}, {x_all.max():.4f}] V  (positive only — see docstring)")
    print(f"compensated:   F_signed = {m:.4f} * ai0 + ({c:.4f})   R^2 = {r2:.5f}")
    print(f"uncompensated: F_signed = {m_u:.4f} * ai0 + ({c_u:.4f})   R^2 = {r2_u:.5f}")
    print("=" * 90)

    # ── plot ──
    # Same measurement style as futek_linearization.py's voltage_vs_force.png:
    # one point per (weight, direction) at the mean raw ai0, with an x error
    # bar = std(ai0) during that phase. Faint background scatter = every
    # individual raw sample, so the real per-sample dispersion is visible too.
    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    marker = "o"  # same shape for both directions — direction is just which
                  # sign of F this weight lands on, not a separate category
    for d in datasets:
        w = int(d["weight_g"])
        color = WEIGHT_COLORS.get(w, "#333")
        label = f"{w:g} g"

        ax.scatter(d["ai0_load"], np.full_like(d["ai0_load"], d["F_signed"]),
                   color=color, marker=marker, s=10, alpha=0.15, zorder=2)
        ax.scatter(d["ai0_base"], np.zeros_like(d["ai0_base"]),
                   color=color, marker=marker, s=10, alpha=0.1, zorder=2)

        ax.errorbar(d["ai0_load"].mean(), d["F_signed"], xerr=d["ai0_load"].std(),
                     fmt=marker, color=color, markersize=9, capsize=4,
                     label=label, zorder=4)
        ax.errorbar(d["ai0_base"].mean(), 0.0, xerr=d["ai0_base"].std(),
                     fmt=marker, color=color, markersize=6, alpha=0.4,
                     capsize=3, zorder=3)

        # uncompensated point (raw weight_g, no hardware mass) + thin guide
        # line down to the compensated point, so the shift is visible.
        ax.plot(ai0_load_mean := d["ai0_load"].mean(), d["F_signed_uncomp"],
                 marker=marker, mfc="none", mec=color, mew=1.3, markersize=8, zorder=4)
        ax.plot([ai0_load_mean, ai0_load_mean], [d["F_signed_uncomp"], d["F_signed"]],
                 ":", color=color, lw=1, alpha=0.6, zorder=3)

    x_range = np.linspace(x_all.min() * 0.999, x_all.max() * 1.001, 200)
    ax.plot(x_range, m * x_range + c, "-", color="#1a1a1a", linewidth=2,
            label=f"compensated fit: F = {m:.3f}*ai0 + ({c:.3f})  (R²={r2:.4f})", zorder=5)
    ax.plot(x_range, m_u * x_range + c_u, "--", color="#888888", linewidth=2,
            label=f"uncompensated fit: F = {m_u:.3f}*ai0 + ({c_u:.3f})  (R²={r2_u:.4f})", zorder=5)
    ax.axhline(0, color="gray", lw=0.8, alpha=0.5)

    ax.set_xlabel("Raw ai0 (V)  —  no baseline subtracted")
    ax.set_ylabel("F, signed by direction (N)  [+ = -z/hook-hung, - = +z/holder]")
    ax.set_title("FUTEK load cell — signed force vs raw ai0\n"
                 "filled = hardware-compensated, open = uncompensated (weight_g only), "
                 "dotted line = shift")
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), fontsize=7, ncol=2, loc="upper left", framealpha=0.9)

    out = os.path.join(OUT_DIR, "force_vs_ai0.png")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out, HERE)}")


if __name__ == "__main__":
    main()
