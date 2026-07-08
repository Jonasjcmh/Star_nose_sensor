#!/usr/bin/env python3
"""
plot_fur_vs_flc.py — is the UR wrist's own force estimate (F_ur) just a
scaled version of the true, load-cell-measured force (F_lc)? Fit
F_ur = alpha * F_lc + beta and report alpha, beta.

Why an intercept and not through-origin
-----------------------------------------
F_ur is a magnitude (sqrt(fx_c^2+fy_c^2+fz_c^2)), so even at F_lc=0 it
carries a positive noise floor (fx/fy/fz noise doesn't cancel after the
square root). Forcing the fit through the origin makes alpha absorb that
floor instead, which biases alpha and hurts R^2 — fitting a free
intercept beta captures the floor directly (it lines up with the ~0.19 N
intercept seen in plot_resultant_vs_fz.py) and gives a much better fit.

F_lc: from the FUTEK load cell (fzcal_futek_direct*.csv), using the same
per-file-baseline calibration as plot_force_vs_ai0.py:
    F_lc = m_lc * |ai0 - V_baseline| + c_lc
(recomputed here, self-contained, from the same raw files.)

F_ur: from the UR wrist's own FT estimate (fzcal_ur_only_*.csv), the
tare-corrected resultant |F| = sqrt(fx_c^2+fy_c^2+fz_c^2). F_ur is
measured directly off the wrist sensor — it never goes through a
weight-based "true force" calculation, so hardware-mass compensation
never touches it.

Both are used as MEANS per (weight, direction) — the two sensors were
recorded in separate sessions (different files, no shared timestamps), so
there's no row-by-row pairing; matching happens at the (weight, direction)
level; 6 weights x 2 directions = 12 pairs.

Compensated vs uncompensated (two panels, one figure)
-------------------------------------------------------
The ground truth used to fit ai0 -> F_lc in the first place is built from
the known weight: F_true = nominal test weight + attachment hardware
between the load cell and the weight (holder=7g / hook=4g — a fixed constant per
direction, not per weight). Compensated includes that hardware mass;
uncompensated drops it and uses the nominal weight alone. That constant
only shifts m_lc/c_lc (the ai0->F_lc calibration); since it's the same
shift at every weight, F_lc changes by roughly a constant amount at each
point, which
mostly moves alpha/beta of the F_ur-vs-F_lc fit rather than the
correlation strength (R^2) — this pair of panels is here to check that,
not assume it.

Usage:
    python plot_fur_vs_flc.py
"""

import os
import re
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

matplotlib.rcParams.update({
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.spines.top": False,
    "axes.spines.right": False,
})


# Extra hardware mass (g) between the load cell and the nominal test
# weight — same values as plot_force_vs_ai0.py (holder/hook only; the
# load cell setup does NOT carry the UR attachment + screws stack that
# fzcal_ur_only_* runs do).
EXTRA_HARDWARE_G = {"posz": 7.0, "negz": 4.0}  # posz: holder, negz: hook


# ── load cell (F_lc) ────────────────────────────────────────────────────
def load_lc_dataset(meta_path):
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
            (ai0_load if int(parts[i_loaded]) else ai0_base).append(float(parts[i_ai0]))
    direction = "posz" if "_posz_" in os.path.basename(csv_path) else "negz"
    ai0_base = np.array(ai0_base)
    ai0_load = np.array(ai0_load)
    v_baseline = ai0_base.mean()
    return {
        "weight_g": meta["weight_g"],
        "direction": direction,
        "tilt_deg": meta["tilt_from_vertical_deg"],
        "dv_base": ai0_base - v_baseline,
        "dv_load": ai0_load - v_baseline,
    }


def fit_loadcell_calibration(compensate):
    """F = m*|dV| + c, pooled over all fzcal_futek_direct* files (both
    directions). compensate=True: F_true = (L3 + L1+L2)*g*cos(tilt), with
    L1+L2 = holder/hook hardware mass. compensate=False: F_true = L3*g*cos(tilt),
    nominal weight only."""
    meta_files = sorted(glob.glob(os.path.join(LOG_DIR, "fzcal_futek_direct*_meta.json")))
    datasets = [load_lc_dataset(mf) for mf in meta_files]

    x, y = [], []
    for d in datasets:
        total_g = d["weight_g"] + (EXTRA_HARDWARE_G[d["direction"]] if compensate else 0)
        F_true = total_g * G / 1000.0 * np.cos(np.deg2rad(d["tilt_deg"]))
        x.append(np.abs(d["dv_base"])); y.append(np.zeros_like(d["dv_base"]))
        x.append(np.abs(d["dv_load"])); y.append(np.full_like(d["dv_load"], F_true))
    x = np.concatenate(x); y = np.concatenate(y)
    m_lc, c_lc = np.polyfit(x, y, 1)
    return m_lc, c_lc, datasets


def flc_per_weight_direction(m_lc, c_lc, datasets):
    """Mean F_lc estimate per (weight, direction), from the loaded-phase samples."""
    out = {}
    for d in datasets:
        dv = d["dv_load"].mean()
        out[(d["weight_g"], d["direction"])] = m_lc * abs(dv) + c_lc
    return out


# ── UR wrist (F_ur) ─────────────────────────────────────────────────────
def fur_per_weight_direction():
    pattern = re.compile(r"fzcal_ur_only_(posz|negz)_(\d+(?:\.\d+)?)g_")
    out = {}
    for csv_path in sorted(glob.glob(os.path.join(LOG_DIR, "fzcal_ur_only_*.csv"))):
        match = pattern.search(os.path.basename(csv_path))
        if not match:
            continue
        direction, weight_str = match.groups()
        weight_g = float(weight_str)
        df = pd.read_csv(csv_path)
        base = df[df["loaded"] == 0][["fx", "fy", "fz"]].mean()
        load = df[df["loaded"] == 1]
        fx_c = load["fx"] - base["fx"]
        fy_c = load["fy"] - base["fy"]
        fz_c = load["fz"] - base["fz"]
        resultant = np.sqrt(fx_c ** 2 + fy_c ** 2 + fz_c ** 2)
        out[(weight_g, direction)] = resultant.mean()
    return out


def build_rows(compensate, fur):
    m_lc, c_lc, lc_datasets = fit_loadcell_calibration(compensate)
    flc = flc_per_weight_direction(m_lc, c_lc, lc_datasets)

    keys = sorted(set(flc) & set(fur))
    if not keys:
        raise RuntimeError("No matching (weight, direction) pairs between F_lc and F_ur datasets")
    rows = [(w, d, flc[(w, d)], fur[(w, d)]) for w, d in keys]

    x = np.array([r[2] for r in rows])  # F_lc
    y = np.array([r[3] for r in rows])  # F_ur
    alpha, beta = np.polyfit(x, y, 1)   # free intercept
    y_pred = alpha * x + beta
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return m_lc, c_lc, rows, alpha, beta, r2


def main():
    fur = fur_per_weight_direction()

    fig, axes = plt.subplots(1, 2, figsize=(15, 6.5), sharex=True, sharey=True)
    panels = [(axes[0], True, "Load-cell F_lc including hardware (compensated)"),
              (axes[1], False, "Load-cell F_lc from nominal weight only (uncompensated)")]

    for ax, compensate, title in panels:
        tag = "compensated" if compensate else "uncompensated"
        m_lc, c_lc, rows, alpha, beta, r2 = build_rows(compensate, fur)

        print("=" * 60)
        print(f"F_ur (UR wrist resultant) vs F_lc (load-cell estimate)  [{tag}]")
        print(f"[load cell fit used] F_lc = {m_lc:.4f}*|dV| + {c_lc:.5f}")
        print("=" * 60)
        print(f"{'weight_g':>8}{'dir':>7}{'F_lc(N)':>10}{'F_ur(N)':>10}{'F_ur/F_lc':>11}")
        for w, d, flc_v, fur_v in rows:
            ratio = fur_v / flc_v if flc_v else float("nan")
            print(f"{w:>8.0f}{d:>7}{flc_v:>10.4f}{fur_v:>10.4f}{ratio:>11.3f}")
        print()
        print(f"n = {len(rows)} matched (weight, direction) pairs")
        print(f"F_ur = alpha * F_lc + beta   ->   alpha = {alpha:.4f}   beta = {beta:.4f}   R^2 = {r2:.5f}")
        print("=" * 60)

        x = np.array([r[2] for r in rows])
        seen_weights = set()
        for w, d, flc_v, fur_v in rows:
            w_int = int(w)
            label = f"{w:g} g" if w_int not in seen_weights else None
            seen_weights.add(w_int)
            ax.scatter(flc_v, fur_v, color=WEIGHT_COLORS.get(w_int, "#333"),
                       marker="o", s=80, label=label, zorder=4)

        x_range = np.linspace(0, x.max() * 1.1, 100)
        ax.plot(x_range, alpha * x_range + beta, "-", color="#1a1a1a", linewidth=2,
                label=f"fit: F_ur={alpha:.3f}*F_lc+{beta:.3f} (R²={r2:.4f})", zorder=3)
        ax.plot(x_range, x_range, "k--", linewidth=1, alpha=0.4, label="F_ur = F_lc (identity)")

        ax.set_xlabel("F_lc — load-cell estimate (N)")
        ax.set_title(title)
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        ax.legend(fontsize=7, ncol=2, loc="upper left")

    axes[0].set_ylabel("F_ur — UR wrist resultant (N)")
    fig.suptitle("UR wrist force estimate vs load-cell force")

    out = os.path.join(OUT_DIR, "fur_vs_flc.png")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out, HERE)}")


if __name__ == "__main__":
    main()
