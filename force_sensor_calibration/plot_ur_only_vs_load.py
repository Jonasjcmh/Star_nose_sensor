#!/usr/bin/env python3
"""
plot_ur_only_vs_load.py — UR wrist sensor (fz) vs the known load used for
testing, using ONLY the fzcal_ur_only_* sessions (no load cell installed
in this chain at all). This isolates the question "does the robot's own
force sensor track a known weight", independent of any load-cell fit.

Reuses fit_lc_ur_calibration.py's discovery/de-dupe/baseline-compensation
so both scripts agree on the numbers.

The baseline (loaded==0) is NOT a zero reference here: the attachment
(15 g) + 4 screws (21 g) + holder (7 g, posz) or hook (1 g, negz) are
already resting on the sensor during that phase too, so it's a known,
non-zero load in its own right. Each de-duplicated (direction, weight)
session therefore contributes TWO absolute (fz, F_true) points, not one
baseline-compensated delta:
  (fz_base_mean, F_true_base = hardware only)
  (fz_load_mean, F_true      = hardware + weight_g)

Fits F_true = a*fz_ur + b over all these points, pooled AND separately per
direction (a pooled fit across both directions is only meaningful if they
actually agree). Also fits a SIGN-CORRECTED pooled version, using
fz_signed = AI0_SIGN[direction] * |fz_raw| AND F_signed =
AI0_SIGN[direction] * F_true (same convention as fit_lc_ur_calibration.py)
-- BOTH sides need to be oriented by direction, not just fz, or pooling a
signed fz against an unsigned F_true makes the fit worse instead of
better (verified: raw pooled R^2=0.895 vs signed-fz/unsigned-F_true
R^2=0.021 vs signed-fz/signed-F_true R^2=0.889).

Usage:
    python plot_ur_only_vs_load.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import fit_lc_ur_calibration as core

HERE = core.HERE
OUT_DIR = core.OUT_DIR


def absolute_points(sessions):
    """Expand each session into its two absolute (fz, F_true) points."""
    fz, f_true, weight_g, phase = [], [], [], []
    for s in sessions:
        fz.append(s["fz_base_mean"]); f_true.append(s["F_true_base"])
        weight_g.append(s["weight_g"]); phase.append("baseline")
        fz.append(s["fz_load_mean"]); f_true.append(s["F_true"])
        weight_g.append(s["weight_g"]); phase.append("loaded")
    return fz, f_true, weight_g, phase


def signed_points(sessions):
    """Expand each session into its two absolute (fz_signed, F_signed, direction)
    points, fz_signed = AI0_SIGN[direction] * |fz_raw|. Ground truth is
    ALSO oriented by direction (F_signed = AI0_SIGN[direction] * F_true,
    not the unsigned F_true) -- pooling a signed fz against an unsigned
    ground truth is exactly what made the pooled sign-corrected fit worse
    than the raw one; both sides need the same sign convention."""
    fz_signed, f_signed, weight_g, phase, direction = [], [], [], [], []
    for s in sessions:
        sign = core.AI0_SIGN[s["direction"]]
        fz_signed.append(sign * abs(s["fz_base_mean"])); f_signed.append(s["F_signed_base"])
        weight_g.append(s["weight_g"]); phase.append("baseline"); direction.append(s["direction"])
        fz_signed.append(sign * abs(s["fz_load_mean"])); f_signed.append(s["F_signed"])
        weight_g.append(s["weight_g"]); phase.append("loaded"); direction.append(s["direction"])
    return fz_signed, f_signed, weight_g, phase, direction


def main():
    entries = core.dedupe_latest(core.discover("ur_only"))
    sessions = sorted((core.load_session(e) for e in entries),
                       key=lambda s: (s["direction"], s["weight_g"]))

    print(f"{'weight_g':>8}{'dir':>6}{'phase':>9}{'fz_abs(N)':>11}{'F_true(N)':>11}")
    for s in sessions:
        print(f"{s['weight_g']:>8.0f}{s['direction']:>6}{'baseline':>9}"
              f"{s['fz_base_mean']:>11.4f}{s['F_true_base']:>11.4f}")
        print(f"{s['weight_g']:>8.0f}{s['direction']:>6}{'loaded':>9}"
              f"{s['fz_load_mean']:>11.4f}{s['F_true']:>11.4f}")

    # ── pooled fit (both directions together) ──
    fz_all, f_true_all, _, _ = absolute_points(sessions)
    a, b, r2, rmse = core.linfit(fz_all, f_true_all)
    print(f"\nn = {len(fz_all)} points ({len(sessions)} sessions x 2 phases each)")
    print(f"pooled:  F_true = {a:.4f} * fz_robot + ({b:.5f})   R^2 = {r2:.5f}   RMSE = {rmse:.4f} N")

    # ── per-direction fit ──
    by_dir = {}
    for direction in ("posz", "negz"):
        dir_sessions = [s for s in sessions if s["direction"] == direction]
        fz, f_true, _, _ = absolute_points(dir_sessions)
        by_dir[direction] = core.linfit(fz, f_true)
        a_d, b_d, r2_d, rmse_d = by_dir[direction]
        print(f"{direction:>7}: F_true = {a_d:.4f} * fz_robot + ({b_d:.5f})   "
              f"R^2 = {r2_d:.5f}   RMSE = {rmse_d:.4f} N   (n={len(fz)} points)")

    # ── sign-corrected pooled fit: fz AND ground truth both oriented by
    # direction (F_signed = AI0_SIGN[direction]*F_true), so the two sides
    # actually share a sign convention -- pooling fz_signed against the
    # unsigned F_true was the bug that made the earlier version worse. ──
    fz_signed_all, f_signed_all, weight_g_all, phase_all, direction_all = signed_points(sessions)
    a_s, b_s, r2_s, rmse_s = core.linfit(fz_signed_all, f_signed_all)
    print(f"\nsign-corrected pooled (fz_signed = AI0_SIGN[direction]*|fz_raw|, "
          f"F_signed = AI0_SIGN[direction]*F_true):")
    print(f"  F_signed = {a_s:.4f} * fz_signed + ({b_s:.5f})   R^2 = {r2_s:.5f}   RMSE = {rmse_s:.4f} N"
          f"  (n={len(fz_signed_all)} points, vs pooled-raw R^2 = {r2:.5f})")

    # ── plot: fz_abs vs F_true scatter, ONE FIGURE PER DIRECTION, plus a
    # 3rd figure with both directions pooled on the sign-corrected fz.
    # Fits are unchanged (still fit on baseline+loaded points); only the
    # baseline ("hardware only") points are hidden from the scatter, per
    # request -- loaded points only, no gridlines, Helvetica titles. ──
    TITLE_FONT = "Helvetica"

    for direction in ("posz", "negz"):
        d_sessions = [s for s in sessions if s["direction"] == direction]
        fz, f_true, weight_g, phase = absolute_points(d_sessions)

        fig, ax = plt.subplots(figsize=(7, 5.5))
        seen_weights = set()
        for x, y, w, p in zip(fz, f_true, weight_g, phase):
            if p != "loaded":
                continue
            color = core.WEIGHT_COLORS.get(int(w), "#333")
            label = f"{int(w)} g" if int(w) not in seen_weights else None
            seen_weights.add(int(w))
            ax.scatter(x, y, color=color, marker="o", s=80, zorder=4, label=label)

        a_d, b_d, r2_d, _ = by_dir[direction]
        x_range = np.linspace(min(fz) * 1.1, max(fz) * 1.1, 200)
        ax.plot(x_range, a_d * x_range + b_d, "-", color="#1a1a1a", linewidth=2,
                label=f"fit: F={a_d:.3f}*fz+{b_d:.3f} (R²={r2_d:.4f})")
        ax.axhline(0, color="gray", lw=0.8, alpha=0.5)
        ax.axvline(0, color="gray", lw=0.8, alpha=0.5)
        ax.set_title(direction, fontname=TITLE_FONT)
        ax.set_xlabel("fz_ur, absolute (N)  [loaded points only]")
        ax.set_ylabel("F_true (N)")
        ax.legend(fontsize=8, ncol=1, loc="upper left")

        out = os.path.join(OUT_DIR, f"ur_only_vs_load_{direction}.png")
        fig.tight_layout()
        fig.savefig(out, dpi=150, facecolor="white")
        plt.close(fig)
        print(f"\nSaved -> {os.path.relpath(out, HERE)}")

    # ── Figure 3: both directions pooled, on sign-corrected fz — marker
    # SHAPE carries the directionality (circle=posz, square=negz), since
    # fz_signed already puts both directions on the same sign axis ──
    fig3, ax3 = plt.subplots(figsize=(7, 5.5))
    seen_weights = set()
    for x, y, w, p, d in zip(fz_signed_all, f_signed_all, weight_g_all, phase_all, direction_all):
        if p != "loaded":
            continue
        color = core.WEIGHT_COLORS.get(int(w), "#333")
        marker = "o" if d == "posz" else "s"
        label = f"{int(w)} g" if int(w) not in seen_weights else None
        seen_weights.add(int(w))
        ax3.scatter(x, y, color=color, marker=marker, s=80, zorder=4, label=label)

    x_range = np.linspace(min(fz_signed_all) * 1.1, max(fz_signed_all) * 1.1, 200)
    ax3.plot(x_range, a_s * x_range + b_s, "-", color="#1a1a1a", linewidth=2,
             label=f"pooled fit: F={a_s:.3f}*fz_signed+{b_s:.3f} (R²={r2_s:.4f})")
    ax3.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax3.axvline(0, color="gray", lw=0.8, alpha=0.5)
    ax3.set_title("both directions, sign-corrected", fontname=TITLE_FONT)
    ax3.set_xlabel("fz_signed = AI0_SIGN[direction]*|fz_ur| (N)  [circle=posz, square=negz, loaded points only]")
    ax3.set_ylabel("F_signed (N) = AI0_SIGN[direction]*F_true")
    ax3.legend(fontsize=8, ncol=1, loc="upper left")

    out3 = os.path.join(OUT_DIR, "ur_only_vs_load_signed.png")
    fig3.tight_layout()
    fig3.savefig(out3, dpi=150, facecolor="white")
    plt.close(fig3)
    print(f"\nSaved -> {os.path.relpath(out3, HERE)}")


if __name__ == "__main__":
    main()
