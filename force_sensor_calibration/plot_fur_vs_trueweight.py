#!/usr/bin/env python3
"""
plot_fur_vs_trueweight.py — UR wrist FT sensor only (fzcal_ur_only_*, no
load cell mounted): does the UR's own force estimate (|Fz| and the
tare-corrected resultant |F|) actually track the true applied force?

"True force" isn't just the nominal test weight — the UR wrist also feels
whatever attachment hardware sits between it and the weight. That hardware
differs by direction (no load cell in this chain — see
plot_force_vs_ai0.py for the load-cell-mounted case, which uses a
different, lighter hardware stack: holder/hook only):
  - negz (hook-hung): UR attachment (15 g) + 4 screws (21 g) + 1 screw (1 g) = 37 g
  - posz (holder):    UR attachment (15 g) + 4 screws (21 g) + holder (7 g) = 43 g

F_true = (weight_g + extra_hardware_g) / 1000 * g * cos(tilt_from_vertical)

Usage:
    python plot_fur_vs_trueweight.py
"""

import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
OUT_DIR = os.path.join(HERE, "plots")
os.makedirs(OUT_DIR, exist_ok=True)

G = 9.80665

WEIGHT_COLORS = {
    5: "#1f77b4", 10: "#ff7f0e", 20: "#2ca02c",
    50: "#d62728", 100: "#9467bd", 200: "#8c564b",
}

# UR-attachment + screws + (holder | extra screw), no load cell in the chain.
EXTRA_HARDWARE_G = {"negz": 15 + 21 + 1, "posz": 15 + 21 + 7}  # 37 g, 43 g

matplotlib.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.spines.top": False,
    "axes.spines.right": False,
})


def load_dataset(csv_path, meta_path):
    with open(meta_path) as f:
        meta = json.load(f)
    df = pd.read_csv(csv_path)
    base = df[df["loaded"] == 0][["fx", "fy", "fz"]].mean()
    load = df[df["loaded"] == 1]
    fx_c = load["fx"] - base["fx"]
    fy_c = load["fy"] - base["fy"]
    fz_c = load["fz"] - base["fz"]
    resultant = np.sqrt(fx_c ** 2 + fy_c ** 2 + fz_c ** 2)

    direction = "posz" if "_posz_" in os.path.basename(csv_path) else "negz"
    tilt_rad = np.deg2rad(meta["tilt_from_vertical_deg"])
    total_weight_g = meta["weight_g"] + EXTRA_HARDWARE_G[direction]
    F_true = total_weight_g * G / 1000.0 * np.cos(tilt_rad)

    return {
        "weight_g": meta["weight_g"],
        "direction": direction,
        "F_true": F_true,
        "fz_abs_mean": fz_c.abs().mean(),
        "resultant_mean": resultant.mean(),
    }


def linfit(x, y):
    m, c = np.polyfit(x, y, 1)
    y_pred = m * x + c
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return m, c, r2


def main():
    csv_paths = sorted(glob.glob(os.path.join(LOG_DIR, "fzcal_ur_only_*.csv")))
    datasets = []
    for csv_path in csv_paths:
        meta_path = csv_path.replace(".csv", "_meta.json")
        if not os.path.exists(meta_path):
            print(f"[skip] no meta for {csv_path}")
            continue
        with open(meta_path) as f:
            weight_g = json.load(f)["weight_g"]
        if int(weight_g) not in WEIGHT_COLORS:
            print(f"[skip] {os.path.basename(csv_path)}: {weight_g:g} g is not one of "
                  f"the standard calibration weights {sorted(WEIGHT_COLORS)}")
            continue
        datasets.append(load_dataset(csv_path, meta_path))

    if not datasets:
        raise FileNotFoundError(f"No fzcal_ur_only_*.csv in {LOG_DIR}")

    x = np.array([d["F_true"] for d in datasets])
    y_fz = np.array([d["fz_abs_mean"] for d in datasets])
    y_res = np.array([d["resultant_mean"] for d in datasets])

    m_fz, c_fz, r2_fz = linfit(x, y_fz)
    m_res, c_res, r2_res = linfit(x, y_res)

    print("=" * 78)
    print("UR WRIST (no load cell) — measured force vs true applied force")
    print("(true force = nominal weight + attachment hardware, direction-dependent)")
    print("=" * 78)
    print(f"{'weight_g':>8}{'dir':>6}{'F_true(N)':>11}{'|Fz|(N)':>10}{'|F|(N)':>10}")
    for d in sorted(datasets, key=lambda d: (d["direction"], d["weight_g"])):
        print(f"{d['weight_g']:>8.0f}{d['direction']:>6}{d['F_true']:>11.4f}"
              f"{d['fz_abs_mean']:>10.4f}{d['resultant_mean']:>10.4f}")
    print()
    print(f"|Fz|      = {m_fz:.4f} * F_true + ({c_fz:.4f})   R^2 = {r2_fz:.5f}")
    print(f"resultant = {m_res:.4f} * F_true + ({c_res:.4f})   R^2 = {r2_res:.5f}")
    print("=" * 78)

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    seen_weights = set()
    for d in datasets:
        w = int(d["weight_g"])
        color = WEIGHT_COLORS.get(w, "#333")
        label = f"{w:g} g" if w not in seen_weights else None
        seen_weights.add(w)
        ax.scatter(d["F_true"], d["fz_abs_mean"], color=color, marker="o",
                   s=70, zorder=4, label=label)
        ax.scatter(d["F_true"], d["resultant_mean"], facecolors="none",
                   edgecolors=color, marker="o", s=70, linewidths=1.5, zorder=4)

    x_range = np.linspace(0, x.max() * 1.1, 100)
    ax.plot(x_range, x_range, "k--", linewidth=1, alpha=0.4, label="ideal: measured = F_true")
    ax.plot(x_range, m_fz * x_range + c_fz, "-", color="#1a6eb5", linewidth=2,
            label=f"|Fz| fit: {m_fz:.3f}*F_true + {c_fz:.3f} (R²={r2_fz:.4f})", zorder=3)
    ax.plot(x_range, m_res * x_range + c_res, "-", color="#c0392b", linewidth=2,
            label=f"|F| fit: {m_res:.3f}*F_true + {c_res:.3f} (R²={r2_res:.4f})", zorder=3)

    ax.set_xlabel("F_true — nominal weight + attachment hardware (N)")
    ax.set_ylabel("UR wrist measured force (N)  [filled = |Fz|, open = resultant |F|]")
    ax.set_title("UR wrist force estimate vs true applied force (no load cell)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), fontsize=7, ncol=2, loc="upper left")

    out = os.path.join(OUT_DIR, "fur_vs_trueweight.png")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out, HERE)}")


if __name__ == "__main__":
    main()
