#!/usr/bin/env python3
"""
plot_fresultant_vs_fz.py — UR wrist FT sensor only (fzcal_ur_only_*, no
load cell mounted): resultant |F| vs signed Fz, all weights, both
directions, ONE plot.

negz and posz are not two separate categories — running the same
weights in both directions just sweeps one continuous range of signed
Fz (large negative -> zero -> large positive). Same marker, same color
per weight, both directions on it.

Weights used: 5/10/20/50/100/200 g. 201 g / 202 g excluded (off-nominal
one-off weights).

Compensated weight: the UR wrist also feels the attachment hardware
between it and the nominal test weight, and that hardware differs by
direction (no load cell in this chain):
  - negz (hook-hung): UR attachment (15 g) + 4 screws (21 g) + 1 screw (1 g) = 37 g
  - posz (holder):    UR attachment (15 g) + 4 screws (21 g) + holder (7 g) = 43 g

Two panels, one figure: left = true expected force including
attachment hardware, right = nominal weight only. Same measured scatter
in both; only the true-force diamond markers differ.

Usage:
    python plot_fresultant_vs_fz.py
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

EXTRA_HARDWARE_G = {"negz": 15 + 21 + 1, "posz": 15 + 21 + 7}  # 37 g, 43 g

matplotlib.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.spines.top": False,
    "axes.spines.right": False,
})


def fz_and_resultant(csv_path):
    df = pd.read_csv(csv_path)
    base = df[df["loaded"] == 0][["fx", "fy", "fz"]].mean()
    load = df[df["loaded"] == 1]
    fx_c = load["fx"] - base["fx"]
    fy_c = load["fy"] - base["fy"]
    fz_c = load["fz"] - base["fz"]
    resultant = np.sqrt(fx_c ** 2 + fy_c ** 2 + fz_c ** 2)
    return fz_c.to_numpy(), resultant.to_numpy()


def expected_force_magnitude(weight_g, direction, tilt_deg, compensate):
    """Actual physical force from the known mass — just weight*g*cos(tilt),
    no direction sign attached. It's a magnitude; the sign it lands on in
    the plot comes from the real measured fz, not an assumption."""
    total_g = weight_g + (EXTRA_HARDWARE_G[direction] if compensate else 0)
    return total_g * G / 1000.0 * np.cos(np.deg2rad(tilt_deg))


def main():
    csv_paths = sorted(glob.glob(os.path.join(LOG_DIR, "fzcal_ur_only_*.csv")))

    entries = []
    for csv_path in csv_paths:
        name = os.path.basename(csv_path)
        meta_path = csv_path.replace(".csv", "_meta.json")
        with open(meta_path) as f:
            meta = json.load(f)
        weight_g = int(meta["weight_g"])
        if weight_g not in WEIGHT_COLORS:
            print(f"[skip] {name}: {weight_g:g} g not in standard set "
                  f"{sorted(WEIGHT_COLORS)} (excluding 201/202 g)")
            continue
        direction = "posz" if "_posz_" in name else "negz"
        fz_c, resultant = fz_and_resultant(csv_path)
        entries.append({
            "weight_g": weight_g, "direction": direction, "tilt_deg": meta["tilt_from_vertical_deg"],
            "fz_c": fz_c, "resultant": resultant,
        })

    fz_all = np.concatenate([e["fz_c"] for e in entries])

    fig, axes = plt.subplots(1, 2, figsize=(15, 6.5), sharex=True, sharey=True)
    panels = [(axes[0], True, "Including attachment hardware (compensated)"),
              (axes[1], False, "Nominal weight only (uncompensated)")]

    for ax, compensate, title in panels:
        for e in entries:
            color = WEIGHT_COLORS[e["weight_g"]]
            ax.scatter(e["fz_c"], e["resultant"], color=color, marker="o",
                       s=16, alpha=0.5, zorder=3)
            f_true_mag = expected_force_magnitude(e["weight_g"], e["direction"],
                                                   e["tilt_deg"], compensate)
            fz_mean = e["fz_c"].mean()  # real measured value carries its own sign
            ax.plot(fz_mean, f_true_mag, marker="D", color=color, mec="black",
                    mew=0.6, markersize=9, zorder=6)

        ref = np.linspace(fz_all.min(), fz_all.max(), 200)
        ax.plot(ref, np.abs(ref), "k--", lw=1.5, alpha=0.5, label="ideal: |F| = |Fz|", zorder=4)
        ax.axvline(0, color="gray", lw=0.8, alpha=0.5)
        ax.set_xlabel("Tare-corrected Fz (N)  [- = negz, + = posz]")
        ax.set_title(title)

    axes[0].set_ylabel("Tare-corrected resultant |F| (N)")

    handles = [plt.Line2D([0], [0], marker="o", color="w", label=f"{w:g} g",
                           markerfacecolor=c, markersize=8)
               for w, c in WEIGHT_COLORS.items()]
    handles.append(plt.Line2D([0], [0], linestyle="--", color="k", alpha=0.5,
                               label="ideal: |F| = |Fz|"))
    handles.append(plt.Line2D([0], [0], marker="D", color="w", label="true expected force",
                               markerfacecolor="#333", markeredgecolor="black", markersize=8))
    fig.legend(handles=handles, fontsize=8, ncol=4, loc="upper center", bbox_to_anchor=(0.5, 1.03))

    fig.suptitle("UR wrist FT — resultant |F| vs Fz, full range (all weights, both directions)", y=1.1)

    out = os.path.join(OUT_DIR, "fresultant_vs_fz.png")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor="white", bbox_inches="tight")
    print(f"Saved -> {os.path.relpath(out, HERE)}")


if __name__ == "__main__":
    main()
