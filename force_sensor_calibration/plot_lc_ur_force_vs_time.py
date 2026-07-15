#!/usr/bin/env python3
"""
plot_lc_ur_force_vs_time.py — for the fzcal_futek_direct_* sessions (load
cell installed alongside the UR robot), converts the load cell's raw ai0
voltage to force using the fitted linear transformation and plots it
together with the UR robot's own force (fz) on the SAME force axis, vs
time. Also produces the Force-vs-Voltage linearization plot the
transformation comes from.

Reuses fit_lc_ur_calibration.py's discovery/de-dupe and session loading
(direction, weight_g, F_signed, hardware mass) so all scripts agree on
those numbers, but fits its OWN voltage<->force transformation here (see
below) rather than reusing core.fit_loadcell_rate's mean-based fit.

Voltage -> force linear transformation
---------------------------------------
Uses EVERY individual raw sample from the loaded window of every session
(200 samples/session here), not just the session mean, plus one baseline
point per session (the mean -- baseline is a known, non-zero load, the
hardware mass alone, not a zero reference; see fit_lc_ur_calibration.py's
module docstring). Fit: F_signed = m_v * ai0 + c_v.

UR force display convention (this plot only)
-----------------------------------------------
The UR's raw fz doesn't follow the load cell's signed convention (posz
negative, negz positive) -- both directions mostly read negative on fz
directly. For THIS plot only, fz is re-signed to match the load cell's
convention for direct visual comparison: F_ur_display = AI0_SIGN[direction]
* |fz|, i.e. negative for posz and positive for negz, same as F_lc. This
is a display transform, not a change to fz itself or to the Step 2/3
compensation fit elsewhere, which still needs the real signed fz.

Panel ordering (Force vs time)
--------------------------------
Panels run continuously from -200 g to +200 g using the load cell's own
sign convention as the ordering key (posz -> negative, negz -> positive,
same as F_signed) -- NOT grouped by direction. So the sequence is
posz 200/100/50/20/10/5 g, then negz 5/10/20/50/100/200 g, laid out
left-to-right, top-to-bottom.

UR compensation coefficients (new)
------------------------------------
A separate fit, pooling EVERY raw sample (both baseline and loaded, both
directions) with F_lc (from the voltage<->force fit) as the reference
"real" value and the UR's raw, actually-signed fz as the value to
compensate: F_lc = comp_a * fz_raw + comp_b. This is the SOP Step-4-style
compensation, refit here from the full per-sample dataset rather than
per-session means. Reported alongside its per-direction breakdown, since
posz and negz do not agree (see plot for the split).

Outputs
-------
  plots/lc_ur_force_vs_time.png   -- 2 rows x 6 columns, ordered -200 g to
    +200 g, F_lc(t) and F_ur_display(t) overlaid on the same force axis,
    loaded window only, same y-scale across all panels.
  plots/lc_linearization.png      -- the Force vs Voltage fit, showing
    EVERY raw loaded sample (not just the mean), the baseline anchor per
    session, the fit line, and per-weight/direction annotations with the
    point count and the hardware-compensated weight (nominal + hardware).
  plots/ur_compensation_linearization.png -- F_lc (from the load cell) vs
    fz, every sample, both directions pooled -200..+200 g: raw fz (left)
    and sign-corrected fz (right), with fit coefficients.
  plots/ur_vs_trueweight_linearization.png -- same idea, but against
    F_true (the KNOWN weight + hardware mass, bypassing the load cell
    entirely) instead of F_lc -- the fundamental ground-truth check.

All fit coefficients found are printed in one consolidated summary at the
end of the run.

Usage:
    python plot_lc_ur_force_vs_time.py
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

WEIGHT_ORDER = [5, 10, 20, 50, 100, 200]


def load_raw_series(csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["timestamp"]) for r in rows])
    t = t - t[0]
    loaded = np.array([int(r["loaded"]) for r in rows], dtype=bool)
    fz = np.array([float(r["fz"]) for r in rows])
    ai0 = np.array([float(r["ai0"]) for r in rows])
    return t, loaded, fz, ai0


def main():
    entries = core.dedupe_latest(core.discover("futek_direct"))
    sessions = sorted((core.load_session(e) for e in entries),
                       key=lambda s: (s["direction"], s["weight_g"]))
    entry_by_key = {(e["direction"], e["nominal_weight_g"]): e for e in entries}
    session_by_key = {(s["direction"], s["nominal_weight_g"]): s for s in sessions}

    # ── Build the fit dataset: baseline mean anchor + EVERY raw loaded
    #    sample, per session — and keep the per-(direction, weight) raw
    #    loaded arrays around for the plot/annotations.
    fit_ai0, fit_f = [], []
    cluster = {}
    for s in sessions:
        key = (s["direction"], s["weight_g"])
        entry = entry_by_key[key]
        t, loaded, fz, ai0 = load_raw_series(entry["csv_path"])
        ai0_loaded_raw = ai0[loaded]

        fit_ai0.append(s["ai0_base_mean"]); fit_f.append(s["F_signed_base"])
        fit_ai0.extend(ai0_loaded_raw.tolist())
        fit_f.extend([s["F_signed"]] * len(ai0_loaded_raw))

        hardware_g = core.EXTRA_HARDWARE_G_FUTEK_DIRECT[s["direction"]]
        cluster[key] = {
            "ai0_loaded_raw": ai0_loaded_raw,
            "ai0_base_mean": s["ai0_base_mean"],
            "F_signed": s["F_signed"],
            "F_signed_base": s["F_signed_base"],
            "n_loaded": len(ai0_loaded_raw),
            "compensated_g": s["weight_g"] + hardware_g,
        }

    m_v, c_v, r2_v, rmse_v = core.linfit(fit_ai0, fit_f)
    print(f"Linearization uses {len(fit_ai0)} points "
          f"({len(sessions)} baseline means + {len(fit_ai0) - len(sessions)} raw loaded samples)")
    print(f"F = {m_v:.4f}*ai0 + ({c_v:.5f})  (R^2={r2_v:.5f}, RMSE={rmse_v:.4f} N)")

    # ── Signed ordering: -200 g ... +200 g, posz negative / negz positive,
    #    same key as F_signed. NOT grouped by direction. ──
    ORDERED_KEYS = ([("posz", w) for w in sorted(WEIGHT_ORDER, reverse=True)]
                    + [("negz", w) for w in sorted(WEIGHT_ORDER)])

    # ── Pass 1: load every panel, convert ai0->F_lc, re-sign fz to match
    #    the load cell's convention, track global y-range. Also pull each
    #    panel's EXPECTED (ground-truth) levels: F_signed for the load
    #    cell (hardware = holder/hook only, 7g/4g) and F_signed_ur for the
    #    UR sensor (hardware = coupler+screws+LC body+holder/hook, 50g/47g)
    #    -- these are genuinely different ground truths for the two
    #    instruments in this SAME rig (see fit_lc_ur_calibration.py). ──
    panel_data = {}
    y_lo, y_hi = np.inf, -np.inf
    for direction, weight in ORDERED_KEYS:
        sign = core.AI0_SIGN[direction]
        entry = entry_by_key.get((direction, float(weight)))
        if entry is None:
            continue
        t, loaded, fz, ai0 = load_raw_series(entry["csv_path"])
        fz_display = sign * np.abs(fz)  # match LC's sign convention (display only)
        fz_base_mean = fz_display[~loaded].mean()
        ai0_base_mean = ai0[~loaded].mean()

        load_start = t[loaded][0]
        t_load = t[loaded] - load_start
        fz_load = fz_display[loaded]
        f_lc_load = m_v * ai0[loaded] + c_v

        s_ground = session_by_key[(direction, float(weight))]
        f_lc_expected = s_ground["F_signed"]
        f_ur_expected = s_ground["F_signed_ur"]

        panel_data[(direction, weight)] = {
            "t": t_load, "f_ur": fz_load, "f_lc": f_lc_load,
            "dFz": fz_load.mean() - fz_base_mean,
            "dFlc": f_lc_load.mean() - (m_v * ai0_base_mean + c_v),
            "signed_weight": sign * weight,
            "f_lc_expected": f_lc_expected, "f_ur_expected": f_ur_expected,
        }
        y_lo = min(y_lo, fz_load.min(), f_lc_load.min(), f_lc_expected, f_ur_expected)
        y_hi = max(y_hi, fz_load.max(), f_lc_load.max(), f_lc_expected, f_ur_expected)

    margin = 0.05 * (y_hi - y_lo)
    y_range = (y_lo - margin, y_hi + margin)

    # ── Pass 2: plot the grid, 2 rows x 6 columns filled in signed order ──
    ncols = len(WEIGHT_ORDER)
    fig, axes = plt.subplots(2, ncols, figsize=(4 * ncols, 7))
    for i, (direction, weight) in enumerate(ORDERED_KEYS):
        row, col = divmod(i, ncols)
        ax = axes[row, col]
        d = panel_data.get((direction, weight))
        if d is None:
            ax.set_visible(False)
            continue

        ax.plot(d["t"], d["f_lc"], color="#1a6eb5", linewidth=1.2, label="F_lc (load cell)")
        ax.plot(d["t"], d["f_ur"], color="#d62728", linewidth=1.2, label="F_ur (UR robot, LC sign convention)")
        ax.axhline(d["f_lc_expected"], color="#7fb8e0", linestyle=":", linewidth=1.3,
                   label=f"expected F_lc={d['f_lc_expected']:+.3f} N")
        ax.axhline(d["f_ur_expected"], color="#e89a9a", linestyle=":", linewidth=1.3,
                   label=f"expected F_ur={d['f_ur_expected']:+.3f} N")
        ax.set_ylim(*y_range)
        ax.set_title(f"{d['signed_weight']:+.0f} g  ({direction})\ndFlc={d['dFlc']:+.3f} N, dFz={d['dFz']:+.3f} N",
                     fontsize=9)
        if row == 1:
            ax.set_xlabel("time since load start (s)", fontsize=8)
        if col == 0:
            ax.set_ylabel("Force (N)", fontsize=8)
        if row == 0 and col == 0:
            ax.legend(fontsize=6, loc="lower right")

    fig.suptitle("Load-cell force vs UR robot force, vs time, loaded window only\n"
                 "futek_direct sessions — ordered -200 g to +200 g (posz negative, negz positive), same y-scale\n"
                 "F_ur re-signed to match LC convention: negative=posz, positive=negz (display only)   |   "
                 "dotted lines: expected F_lc/F_ur from known weight + hardware mass",
                 fontsize=11, fontweight="bold")
    fig.tight_layout()
    out1 = os.path.join(OUT_DIR, "lc_ur_force_vs_time.png")
    fig.savefig(out1, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out1, HERE)}")

    # ── Linearization plot: every raw loaded sample + baseline anchors ──
    fig2, ax2 = plt.subplots(figsize=(11, 8))
    for (direction, weight), c in cluster.items():
        color = core.WEIGHT_COLORS.get(int(weight), "#333")
        marker = "o" if direction == "posz" else "s"

        # every raw loaded sample -- a faint cloud, not a single mean point
        ax2.scatter(c["ai0_loaded_raw"], np.full_like(c["ai0_loaded_raw"], c["F_signed"]),
                    color=color, marker=marker, s=14, alpha=0.25, linewidths=0, zorder=2)

        # baseline anchor (session mean) -- open marker
        ax2.scatter(c["ai0_base_mean"], c["F_signed_base"], facecolors="none", edgecolors=color,
                    marker=marker, s=70, linewidths=1.5, zorder=4)

        # cluster mean (loaded) vs its estimated point, with a residual connector
        ai0_mean = c["ai0_loaded_raw"].mean()
        f_est = m_v * ai0_mean + c_v
        ax2.plot([ai0_mean, ai0_mean], [c["F_signed"], f_est], ":", color=color, linewidth=1, alpha=0.7, zorder=3)
        ax2.scatter(ai0_mean, c["F_signed"], color=color, marker=marker, s=90, linewidths=1.2,
                    edgecolors="black", zorder=5)
        ax2.scatter(ai0_mean, f_est, color=color, marker="x", s=70, linewidths=2, zorder=5)

        # annotation: point count + hardware-compensated weight. Offset
        # scales with weight index so the small-weight cluster (which sits
        # close together near F=0) doesn't overlap; direction sets which
        # side (posz above-left, negz below-right).
        idx = WEIGHT_ORDER.index(int(weight))
        y_off = (14 + idx * 11) * (1 if direction == "negz" else -1)
        x_off = 6 if direction == "negz" else -6
        ha = "left" if direction == "negz" else "right"
        ax2.annotate(f"{int(weight)}g\N{RIGHTWARDS ARROW}{c['compensated_g']:.0f}g (n={c['n_loaded']})",
                     xy=(ai0_mean, c["F_signed"]), xytext=(x_off, y_off), textcoords="offset points",
                     fontsize=6.5, color=color, ha=ha,
                     arrowprops=dict(arrowstyle="-", color=color, lw=0.6, alpha=0.6))

    ai0_all = np.array(fit_ai0)
    margin2 = 0.05 * (ai0_all.max() - ai0_all.min())
    x_range = np.linspace(ai0_all.min() - margin2, ai0_all.max() + margin2, 200)
    ax2.plot(x_range, m_v * x_range + c_v, "-", color="#1a1a1a", linewidth=2, zorder=6)
    ax2.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax2.set_xlabel("ai0, absolute (V)")
    ax2.set_ylabel("F_signed (N)")
    ax2.set_title("FUTEK load cell linearization — Force vs Voltage\n"
                   f"F = {m_v:.4f}*ai0 + ({c_v:.4f})  (R²={r2_v:.4f}, n={len(fit_ai0)} points)\n"
                   "annotations: n loaded samples, nominal weight (hardware-compensated weight)")

    weight_handles = [plt.Line2D([0], [0], marker="o", color="w", label=f"{w:g} g",
                                  markerfacecolor=core.WEIGHT_COLORS[w], markersize=8)
                      for w in sorted(core.WEIGHT_COLORS)]
    style_handles = [
        plt.Line2D([0], [0], marker="o", color="w", label="raw loaded samples (cloud)",
                   markerfacecolor="#333", alpha=0.4, markersize=6),
        plt.Line2D([0], [0], marker="o", color="w", label="loaded mean (black edge)",
                   markerfacecolor="#333", markeredgecolor="black", markersize=8),
        plt.Line2D([0], [0], marker="o", color="w", label="baseline mean (open)",
                   markerfacecolor="none", markeredgecolor="#333", markersize=8),
        plt.Line2D([0], [0], marker="x", color="#333", label="estimated (fit prediction)",
                   linestyle="none", markersize=8),
        plt.Line2D([0], [0], color="#1a1a1a", linewidth=2, label="fit line"),
    ]
    ax2.legend(handles=weight_handles + style_handles, fontsize=7.5, ncol=2, loc="upper left")

    fig2.tight_layout()
    out2 = os.path.join(OUT_DIR, "lc_linearization.png")
    fig2.savefig(out2, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out2, HERE)}")

    # ── UR compensation linearization: F_lc (real value) vs fz (UR sensor),
    #    EVERY sample (baseline + loaded), both directions pooled, ordered
    #    -200..+200 g. Two versions:
    #      raw:  fz as actually recorded (real signed reading) — the honest
    #            diagnostic; posz and negz disagree in sign behavior, so
    #            one pooled fit is a poor compromise (see left panel).
    #      sign-corrected: fz_display = AI0_SIGN[direction] * |fz|, the same
    #            display convention used in the force-vs-time plot, so both
    #            F_lc and fz share one reference and trace a single line
    #            (right panel) — but note this requires knowing which
    #            direction (sign) the applied load is in beforehand, since
    #            that's what picks the sign to apply to |fz|.
    comp_fz, comp_fz_signed, comp_flc, comp_meta = [], [], [], []
    for direction, weight in ORDERED_KEYS:
        entry = entry_by_key.get((direction, float(weight)))
        if entry is None:
            continue
        t, loaded, fz, ai0 = load_raw_series(entry["csv_path"])
        f_lc_all = m_v * ai0 + c_v
        fz_signed = core.AI0_SIGN[direction] * np.abs(fz)
        comp_fz.extend(fz.tolist())
        comp_fz_signed.extend(fz_signed.tolist())
        comp_flc.extend(f_lc_all.tolist())
        comp_meta.extend([(direction, weight)] * len(fz))

    comp_a, comp_b, comp_r2, comp_rmse = core.linfit(comp_fz, comp_flc)
    print(f"\nUR compensation, raw fz (pooled, {len(comp_fz)} samples, -200..+200 g): "
          f"F_lc = {comp_a:.4f}*fz_raw + ({comp_b:.5f})   R^2 = {comp_r2:.5f}   RMSE = {comp_rmse:.4f} N")
    print(f"  -> apply as: fz_corrected = {comp_a:.4f} * fz_raw + ({comp_b:.5f})")

    by_dir = {}
    for direction in ("posz", "negz"):
        idx = [i for i, m in enumerate(comp_meta) if m[0] == direction]
        fz_d = [comp_fz[i] for i in idx]
        flc_d = [comp_flc[i] for i in idx]
        by_dir[direction] = core.linfit(fz_d, flc_d)
        a_d, b_d, r2_d, rmse_d = by_dir[direction]
        print(f"  {direction}: F_lc = {a_d:.4f}*fz_raw + ({b_d:.5f})   R^2 = {r2_d:.5f}   "
              f"RMSE = {rmse_d:.4f} N   (n={len(fz_d)})")

    comp_a2, comp_b2, comp_r2_2, comp_rmse2 = core.linfit(comp_fz_signed, comp_flc)
    print(f"\nUR compensation, sign-corrected fz (pooled, {len(comp_fz_signed)} samples): "
          f"F_lc = {comp_a2:.4f}*fz_signed + ({comp_b2:.5f})   R^2 = {comp_r2_2:.5f}   RMSE = {comp_rmse2:.4f} N")
    print(f"  -> apply as: fz_corrected = {comp_a2:.4f} * (AI0_SIGN[direction]*|fz_raw|) + ({comp_b2:.5f})")

    fig3, (ax3a, ax3b) = plt.subplots(1, 2, figsize=(18, 7.5))

    for direction, weight in ORDERED_KEYS:
        idx = [i for i, m in enumerate(comp_meta) if m == (direction, weight)]
        color = core.WEIGHT_COLORS.get(int(weight), "#333")
        marker = "o" if direction == "posz" else "s"
        flc_arr = np.array([comp_flc[i] for i in idx])
        ax3a.scatter(np.array([comp_fz[i] for i in idx]), flc_arr,
                     color=color, marker=marker, s=10, alpha=0.2, linewidths=0, zorder=2)
        ax3b.scatter(np.array([comp_fz_signed[i] for i in idx]), flc_arr,
                     color=color, marker=marker, s=10, alpha=0.2, linewidths=0, zorder=2)

    fz_all_arr = np.array(comp_fz)
    margin3 = 0.05 * (fz_all_arr.max() - fz_all_arr.min())
    x_range3 = np.linspace(fz_all_arr.min() - margin3, fz_all_arr.max() + margin3, 200)
    ax3a.plot(x_range3, comp_a * x_range3 + comp_b, "-", color="#1a1a1a", linewidth=2,
              label=f"pooled: F_lc={comp_a:.3f}*fz+{comp_b:.3f} (R²={comp_r2:.4f})")
    ax3a.plot(x_range3, by_dir["posz"][0] * x_range3 + by_dir["posz"][1], ":", color="#1f77b4", linewidth=1.8,
              label=f"posz-only: F_lc={by_dir['posz'][0]:.3f}*fz+{by_dir['posz'][1]:.3f} (R²={by_dir['posz'][2]:.4f})")
    ax3a.plot(x_range3, by_dir["negz"][0] * x_range3 + by_dir["negz"][1], ":", color="#d62728", linewidth=1.8,
              label=f"negz-only: F_lc={by_dir['negz'][0]:.3f}*fz+{by_dir['negz'][1]:.3f} (R²={by_dir['negz'][2]:.4f})")
    ax3a.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax3a.axvline(0, color="gray", lw=0.8, alpha=0.5)
    ax3a.set_xlabel("fz_raw (N) — UR sensor, real signed reading")
    ax3a.set_ylabel("F_lc (N) — load cell, real value")
    ax3a.set_title("Raw fz (honest diagnostic)\nposz/negz disagree — one pooled fit is a poor compromise")
    weight_handles3 = [plt.Line2D([0], [0], marker="o", color="w", label=f"{w:g} g",
                                   markerfacecolor=core.WEIGHT_COLORS[w], markersize=8)
                       for w in sorted(core.WEIGHT_COLORS)]
    handles3a, _ = ax3a.get_legend_handles_labels()
    ax3a.legend(handles=weight_handles3 + handles3a, fontsize=7.5, ncol=2, loc="upper left")

    fz_signed_arr = np.array(comp_fz_signed)
    margin3b = 0.05 * (fz_signed_arr.max() - fz_signed_arr.min())
    x_range3b = np.linspace(fz_signed_arr.min() - margin3b, fz_signed_arr.max() + margin3b, 200)
    ax3b.plot(x_range3b, comp_a2 * x_range3b + comp_b2, "-", color="#1a1a1a", linewidth=2,
              label=f"F_lc={comp_a2:.3f}*fz_signed+{comp_b2:.3f} (R²={comp_r2_2:.4f})")
    ax3b.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax3b.axvline(0, color="gray", lw=0.8, alpha=0.5)
    ax3b.set_xlabel("fz_signed = AI0_SIGN[direction]*|fz_raw| (N)")
    ax3b.set_ylabel("F_lc (N) — load cell, real value")
    ax3b.set_title("Sign-corrected fz (same LC reference)\nsingle linear relationship, but needs direction known beforehand")
    handles3b, _ = ax3b.get_legend_handles_labels()
    ax3b.legend(handles=weight_handles3 + handles3b, fontsize=7.5, ncol=2, loc="upper left")

    fig3.suptitle(f"UR compensation linearization — F_lc (real) vs fz (UR sensor), every sample, "
                  f"-200..+200 g pooled (n={len(comp_fz)})", fontsize=12, fontweight="bold")
    fig3.tight_layout()
    out3 = os.path.join(OUT_DIR, "ur_compensation_linearization.png")
    fig3.savefig(out3, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out3, HERE)}")

    # ── UR vs KNOWN weight (ground truth) linearization: same idea as the
    #    LC compensation above, but skips the load cell entirely. Uses
    #    F_signed_ur / F_signed_ur_base — NOT F_signed/F_signed_base, which
    #    are the LOAD CELL's own ground truth (hardware = holder/hook only,
    #    7g/4g). The UR sensor holds up the load cell's own body too, so
    #    its ground truth needs the larger hardware total (50g/47g) — see
    #    EXTRA_HARDWARE_G_FUTEK_DIRECT_UR in fit_lc_ur_calibration.py. ──
    truth_fz, truth_fz_signed, truth_ftrue, truth_meta = [], [], [], []
    for s in sessions:
        entry = entry_by_key[(s["direction"], s["weight_g"])]
        t, loaded, fz, ai0 = load_raw_series(entry["csv_path"])
        fz_signed = core.AI0_SIGN[s["direction"]] * np.abs(fz)

        truth_fz.extend(fz[~loaded].tolist()); truth_fz.extend(fz[loaded].tolist())
        truth_fz_signed.extend(fz_signed[~loaded].tolist()); truth_fz_signed.extend(fz_signed[loaded].tolist())
        truth_ftrue.extend([s["F_signed_ur_base"]] * int((~loaded).sum()))
        truth_ftrue.extend([s["F_signed_ur"]] * int(loaded.sum()))
        truth_meta.extend([(s["direction"], s["weight_g"])] * len(fz))

    truth_a, truth_b, truth_r2, truth_rmse = core.linfit(truth_fz, truth_ftrue)
    print(f"\nUR vs known weight, raw fz (pooled, {len(truth_fz)} samples): "
          f"F_true = {truth_a:.4f}*fz_raw + ({truth_b:.5f})   R^2 = {truth_r2:.5f}   RMSE = {truth_rmse:.4f} N")

    truth_by_dir = {}
    for direction in ("posz", "negz"):
        idx = [i for i, m in enumerate(truth_meta) if m[0] == direction]
        fz_d = [truth_fz[i] for i in idx]
        ft_d = [truth_ftrue[i] for i in idx]
        truth_by_dir[direction] = core.linfit(fz_d, ft_d)
        a_d, b_d, r2_d, rmse_d = truth_by_dir[direction]
        print(f"  {direction}: F_true = {a_d:.4f}*fz_raw + ({b_d:.5f})   R^2 = {r2_d:.5f}   "
              f"RMSE = {rmse_d:.4f} N   (n={len(fz_d)})")

    truth_a2, truth_b2, truth_r2_2, truth_rmse2 = core.linfit(truth_fz_signed, truth_ftrue)
    print(f"\nUR vs known weight, sign-corrected fz (pooled, {len(truth_fz_signed)} samples): "
          f"F_true = {truth_a2:.4f}*fz_signed + ({truth_b2:.5f})   R^2 = {truth_r2_2:.5f}   RMSE = {truth_rmse2:.4f} N")
    print(f"  -> apply as: fz_corrected = {truth_a2:.4f} * (AI0_SIGN[direction]*|fz_raw|) + ({truth_b2:.5f})")

    fig4, (ax4a, ax4b) = plt.subplots(1, 2, figsize=(18, 7.5))
    for direction, weight in ORDERED_KEYS:
        idx = [i for i, m in enumerate(truth_meta) if m == (direction, weight)]
        color = core.WEIGHT_COLORS.get(int(weight), "#333")
        marker = "o" if direction == "posz" else "s"
        ft_arr = np.array([truth_ftrue[i] for i in idx])
        ax4a.scatter(np.array([truth_fz[i] for i in idx]), ft_arr,
                     color=color, marker=marker, s=10, alpha=0.2, linewidths=0, zorder=2)
        ax4b.scatter(np.array([truth_fz_signed[i] for i in idx]), ft_arr,
                     color=color, marker=marker, s=10, alpha=0.2, linewidths=0, zorder=2)

    fz_arr_all = np.array(truth_fz)
    margin4 = 0.05 * (fz_arr_all.max() - fz_arr_all.min())
    x_range4 = np.linspace(fz_arr_all.min() - margin4, fz_arr_all.max() + margin4, 200)
    ax4a.plot(x_range4, truth_a * x_range4 + truth_b, "-", color="#1a1a1a", linewidth=2,
              label=f"pooled: F_true={truth_a:.3f}*fz+{truth_b:.3f} (R²={truth_r2:.4f})")
    ax4a.plot(x_range4, truth_by_dir["posz"][0] * x_range4 + truth_by_dir["posz"][1], ":",
              color="#1f77b4", linewidth=1.8,
              label=f"posz-only: F_true={truth_by_dir['posz'][0]:.3f}*fz+{truth_by_dir['posz'][1]:.3f} "
                    f"(R²={truth_by_dir['posz'][2]:.4f})")
    ax4a.plot(x_range4, truth_by_dir["negz"][0] * x_range4 + truth_by_dir["negz"][1], ":",
              color="#d62728", linewidth=1.8,
              label=f"negz-only: F_true={truth_by_dir['negz'][0]:.3f}*fz+{truth_by_dir['negz'][1]:.3f} "
                    f"(R²={truth_by_dir['negz'][2]:.4f})")
    ax4a.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax4a.axvline(0, color="gray", lw=0.8, alpha=0.5)
    ax4a.set_xlabel("fz_raw (N) — UR sensor, real signed reading")
    ax4a.set_ylabel("F_true (N) — known weight + hardware, signed")
    ax4a.set_title("Raw fz vs KNOWN weight (ground truth)\nno load cell involved — posz/negz still disagree")
    weight_handles4 = [plt.Line2D([0], [0], marker="o", color="w", label=f"{w:g} g",
                                   markerfacecolor=core.WEIGHT_COLORS[w], markersize=8)
                       for w in sorted(core.WEIGHT_COLORS)]
    handles4a, _ = ax4a.get_legend_handles_labels()
    ax4a.legend(handles=weight_handles4 + handles4a, fontsize=7.5, ncol=2, loc="upper left")

    fz_signed_arr_all = np.array(truth_fz_signed)
    margin4b = 0.05 * (fz_signed_arr_all.max() - fz_signed_arr_all.min())
    x_range4b = np.linspace(fz_signed_arr_all.min() - margin4b, fz_signed_arr_all.max() + margin4b, 200)
    ax4b.plot(x_range4b, truth_a2 * x_range4b + truth_b2, "-", color="#1a1a1a", linewidth=2,
              label=f"F_true={truth_a2:.3f}*fz_signed+{truth_b2:.3f} (R²={truth_r2_2:.4f})")
    ax4b.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax4b.axvline(0, color="gray", lw=0.8, alpha=0.5)
    ax4b.set_xlabel("fz_signed = AI0_SIGN[direction]*|fz_raw| (N)")
    ax4b.set_ylabel("F_true (N) — known weight + hardware, signed")
    ax4b.set_title("Sign-corrected fz vs KNOWN weight (ground truth)\nfinal compensation curve, no load cell needed")
    handles4b, _ = ax4b.get_legend_handles_labels()
    ax4b.legend(handles=weight_handles4 + handles4b, fontsize=7.5, ncol=2, loc="upper left")

    fig4.suptitle(f"UR sensor vs KNOWN weight (load + hardware), every sample, "
                  f"-200..+200 g pooled (n={len(truth_fz)})", fontsize=12, fontweight="bold")
    fig4.tight_layout()
    out4 = os.path.join(OUT_DIR, "ur_vs_trueweight_linearization.png")
    fig4.savefig(out4, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out4, HERE)}")

    # ── Consolidated coefficients summary, every curve fit in this run ──
    print("\n" + "=" * 78)
    print("COEFFICIENTS SUMMARY — all curves fit in this run")
    print("=" * 78)
    print(f"1. LC linearization (ai0 -> F_signed), n={len(fit_ai0)}:")
    print(f"     F_signed = {m_v:.4f} * ai0 + ({c_v:.5f})   R^2={r2_v:.5f}  RMSE={rmse_v:.4f} N")
    print(f"\n2. UR compensation vs F_lc (load cell), raw fz, n={len(comp_fz)}:")
    print(f"     pooled : F_lc = {comp_a:.4f}*fz + ({comp_b:.5f})   R^2={comp_r2:.5f}")
    print(f"     posz   : F_lc = {by_dir['posz'][0]:.4f}*fz + ({by_dir['posz'][1]:.5f})   R^2={by_dir['posz'][2]:.5f}")
    print(f"     negz   : F_lc = {by_dir['negz'][0]:.4f}*fz + ({by_dir['negz'][1]:.5f})   R^2={by_dir['negz'][2]:.5f}")
    print(f"\n3. UR compensation vs F_lc (load cell), sign-corrected fz, n={len(comp_fz_signed)}:")
    print(f"     F_lc = {comp_a2:.4f}*fz_signed + ({comp_b2:.5f})   R^2={comp_r2_2:.5f}")
    print(f"\n4. UR vs known weight (F_true), raw fz, n={len(truth_fz)}:")
    print(f"     pooled : F_true = {truth_a:.4f}*fz + ({truth_b:.5f})   R^2={truth_r2:.5f}")
    print(f"     posz   : F_true = {truth_by_dir['posz'][0]:.4f}*fz + ({truth_by_dir['posz'][1]:.5f})   "
          f"R^2={truth_by_dir['posz'][2]:.5f}")
    print(f"     negz   : F_true = {truth_by_dir['negz'][0]:.4f}*fz + ({truth_by_dir['negz'][1]:.5f})   "
          f"R^2={truth_by_dir['negz'][2]:.5f}")
    print(f"\n5. UR vs known weight (F_true), sign-corrected fz, n={len(truth_fz_signed)}:")
    print(f"     F_true = {truth_a2:.4f}*fz_signed + ({truth_b2:.5f})   R^2={truth_r2_2:.5f}")
    print("=" * 78)


if __name__ == "__main__":
    main()
