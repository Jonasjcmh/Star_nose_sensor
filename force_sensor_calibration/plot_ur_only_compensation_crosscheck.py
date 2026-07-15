#!/usr/bin/env python3
"""
plot_ur_only_compensation_crosscheck.py — cross-validates the two UR
compensation formulas fit in plot_lc_ur_force_vs_time.py (from the
futek_direct rig: load cell + UR together, sign-corrected fz, pooled
-200..+200 g) against the INDEPENDENT fzcal_ur_only_* dataset (UR sensor
alone, different session, different hardware stack). This is genuine
held-out validation: the coefficients below were fit on futek_direct data
and are here applied, unmodified, to data they have never seen.

Coefficients under test (hardcoded from plot_lc_ur_force_vs_time.py's
printed "COEFFICIENTS SUMMARY", items #3 and #5 — re-run that script if
the underlying logs ever change):
  #3  F_lc   = 0.7284 * fz_signed + (-0.20451)   [UR vs load cell]
  #5  F_true = 1.0010 * fz_signed + (-0.27483)   [UR vs known weight]
where fz_signed = AI0_SIGN[direction] * |fz_raw|, same sign-correction
convention used throughout (negative for posz, positive for negz).

For every raw sample (baseline + loaded) of every ur_only session, plots
F_true (ground truth: known weight + ur_only's own hardware mass) against
fz_signed, with:
  - the two transferred correction lines (#3, #5) evaluated as-is (no
    refitting) — this is what "compensation coefficients that generalize"
    would need to look like,
  - a fresh fit on the ur_only data itself, as a best-case reference
    (upper bound on what's achievable for this specific rig/session).
Reports each transferred correction's real prediction RMSE on this
held-out dataset, which is the meaningful number here — R² from the
original fit does NOT tell you this.

Usage:
    python plot_ur_only_compensation_crosscheck.py
"""

import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import fit_lc_ur_calibration as core

HERE = core.HERE
OUT_DIR = core.OUT_DIR

# Coefficients under test, from plot_lc_ur_force_vs_time.py (futek_direct,
# sign-corrected fz, pooled -200..+200 g). See module docstring.
COEFF_VS_FLC = (0.7284, -0.20451)     # item #3: F_lc   = a*fz_signed + b
COEFF_VS_FTRUE = (1.0010, -0.27483)   # item #5: F_true = a*fz_signed + b


def load_raw_series(csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    loaded = np.array([int(r["loaded"]) for r in rows], dtype=bool)
    fz = np.array([float(r["fz"]) for r in rows])
    return loaded, fz


def main():
    entries = core.dedupe_latest(core.discover("ur_only"))
    sessions = sorted((core.load_session(e) for e in entries),
                       key=lambda s: (s["direction"], s["weight_g"]))
    entry_by_key = {(e["direction"], e["nominal_weight_g"]): e for e in entries}

    fz_signed_all, f_true_all, meta = [], [], []
    for s in sessions:
        entry = entry_by_key[(s["direction"], s["weight_g"])]
        loaded, fz = load_raw_series(entry["csv_path"])
        sign = core.AI0_SIGN[s["direction"]]
        fz_signed = sign * np.abs(fz)
        f_true_base_signed = sign * s["F_true_base"]

        fz_signed_all.extend(fz_signed[~loaded].tolist())
        fz_signed_all.extend(fz_signed[loaded].tolist())
        f_true_all.extend([f_true_base_signed] * int((~loaded).sum()))
        f_true_all.extend([s["F_signed"]] * int(loaded.sum()))
        meta.extend([(s["direction"], s["weight_g"])] * len(fz))

    fz_signed_all = np.array(fz_signed_all)
    f_true_all = np.array(f_true_all)
    n = len(fz_signed_all)
    print(f"ur_only dataset: {n} raw samples ({len(sessions)} sessions x ~{n // len(sessions)} samples each)")

    # ── Apply the two TRANSFERRED corrections, unmodified, and score them
    #    against this held-out data (real prediction error, not a fit R²) ──
    pred_flc = COEFF_VS_FLC[0] * fz_signed_all + COEFF_VS_FLC[1]
    pred_ftrue = COEFF_VS_FTRUE[0] * fz_signed_all + COEFF_VS_FTRUE[1]
    rmse_flc = float(np.sqrt(np.mean((pred_flc - f_true_all) ** 2)))
    rmse_ftrue = float(np.sqrt(np.mean((pred_ftrue - f_true_all) ** 2)))
    bias_flc = float(np.mean(pred_flc - f_true_all))
    bias_ftrue = float(np.mean(pred_ftrue - f_true_all))
    print(f"\nTransferred correction #3 (fit on F_lc, futek_direct) applied to ur_only:")
    print(f"  F_pred = {COEFF_VS_FLC[0]:.4f}*fz_signed + ({COEFF_VS_FLC[1]:.5f})")
    print(f"  prediction RMSE = {rmse_flc:.4f} N   bias = {bias_flc:+.4f} N   (n={n})")
    print(f"\nTransferred correction #5 (fit on F_true, futek_direct) applied to ur_only:")
    print(f"  F_pred = {COEFF_VS_FTRUE[0]:.4f}*fz_signed + ({COEFF_VS_FTRUE[1]:.5f})")
    print(f"  prediction RMSE = {rmse_ftrue:.4f} N   bias = {bias_ftrue:+.4f} N   (n={n})")

    # ── Fresh fit on ur_only itself, as a best-case reference ──
    own_a, own_b, own_r2, own_rmse = core.linfit(fz_signed_all, f_true_all)
    print(f"\nur_only's own fit (best case for this rig/session):")
    print(f"  F_true = {own_a:.4f}*fz_signed + ({own_b:.5f})   R^2 = {own_r2:.5f}   RMSE = {own_rmse:.4f} N")

    # ── Plot ──
    fig, ax = plt.subplots(figsize=(10, 8))
    for direction, weight in [(d, w) for d in ("posz", "negz") for w in [5, 10, 20, 50, 100, 200]]:
        idx = [i for i, m in enumerate(meta) if m == (direction, weight)]
        if not idx:
            continue
        color = core.WEIGHT_COLORS.get(int(weight), "#333")
        marker = "o" if direction == "posz" else "s"
        ax.scatter(fz_signed_all[idx], f_true_all[idx], color=color, marker=marker,
                   s=12, alpha=0.25, linewidths=0, zorder=2)

    margin = 0.05 * (fz_signed_all.max() - fz_signed_all.min())
    x_range = np.linspace(fz_signed_all.min() - margin, fz_signed_all.max() + margin, 200)
    ax.plot(x_range, own_a * x_range + own_b, "-", color="#1a1a1a", linewidth=2.5,
             label=f"ur_only's own fit: F={own_a:.3f}*fz+{own_b:.3f} (R²={own_r2:.4f})")
    ax.plot(x_range, COEFF_VS_FLC[0] * x_range + COEFF_VS_FLC[1], "--", color="#1a6eb5", linewidth=2,
             label=f"transferred #3 (vs F_lc): F={COEFF_VS_FLC[0]:.3f}*fz+{COEFF_VS_FLC[1]:.3f} "
                   f"(RMSE on ur_only={rmse_flc:.3f} N)")
    ax.plot(x_range, COEFF_VS_FTRUE[0] * x_range + COEFF_VS_FTRUE[1], ":", color="#d62728", linewidth=2,
             label=f"transferred #5 (vs F_true): F={COEFF_VS_FTRUE[0]:.3f}*fz+{COEFF_VS_FTRUE[1]:.3f} "
                   f"(RMSE on ur_only={rmse_ftrue:.3f} N)")
    ax.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax.axvline(0, color="gray", lw=0.8, alpha=0.5)
    ax.set_xlabel("fz_signed = AI0_SIGN[direction]*|fz_raw| (N) — UR sensor, ur_only sessions")
    ax.set_ylabel("F_true (N) — known weight + hardware, signed")
    ax.set_title("UR compensation cross-check — transferred coefficients (from futek_direct)\n"
                 "evaluated on the independent ur_only dataset (held-out, no load cell)")

    weight_handles = [plt.Line2D([0], [0], marker="o", color="w", label=f"{w:g} g",
                                   markerfacecolor=core.WEIGHT_COLORS[w], markersize=8)
                      for w in sorted(core.WEIGHT_COLORS)]
    handles, _ = ax.get_legend_handles_labels()
    ax.legend(handles=weight_handles + handles, fontsize=8, ncol=1, loc="upper left")

    fig.tight_layout()
    out = os.path.join(OUT_DIR, "ur_only_compensation_crosscheck.png")
    fig.savefig(out, dpi=150, facecolor="white")
    print(f"\nSaved -> {os.path.relpath(out, HERE)}")


if __name__ == "__main__":
    main()
