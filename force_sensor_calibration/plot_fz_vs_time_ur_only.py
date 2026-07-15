#!/usr/bin/env python3
"""
plot_fz_vs_time_ur_only.py — raw UR fz vs time, one panel per (direction,
weight), for the fzcal_ur_only_* sessions (no load cell installed).
2 rows (posz, negz) x 6 columns (5/10/20/50/100/200 g), same weight in
the same column on both rows so the two directions line up for
comparison.

Reuses fit_lc_ur_calibration.py's discovery/de-duplication so the same
"use just the last one" rule applies (e.g. the ur_only negz ~200 g point,
attempted 3 times, shows only the kept 202 g run).

Each panel shows ONLY the loaded window (loaded==1), time re-zeroed to
the start of that window, plus the loaded-phase mean as a dashed line.
Every panel shares the SAME y-axis range -- sized to the largest range
seen across all sessions -- so magnitudes are directly comparable panel
to panel. No gridlines.

Each panel also shows the EXPECTED force level, i.e. F_true = (weight_g +
EXTRA_HARDWARE_G_UR_ONLY[direction]) worth of force, NOT just the nominal
weight_g -- the attachment/screws/holder-or-hook are already on the
sensor too, so e.g. a nominal "5 g" posz point is really a ~48 g
equivalent load. Drawn as two symmetric dashed lines at +F_true and
-F_true: the raw fz sign convention isn't assumed here (posz and negz
raw fz were both found to trend negative with load in this rig, unlike
the load cell's own AI0_SIGN convention), so both possible signs are
shown rather than guessing one.

Usage:
    python plot_fz_vs_time_ur_only.py
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
    return t, loaded, fz


def main():
    entries = core.dedupe_latest(core.discover("ur_only"))
    sessions = {(e["direction"], e["nominal_weight_g"]): e for e in entries}

    # ── Pass 1: load every panel's loaded-window data, track global y-range ──
    panel_data = {}
    global_lo, global_hi = np.inf, -np.inf
    for direction in ("posz", "negz"):
        for weight in WEIGHT_ORDER:
            entry = sessions.get((direction, float(weight)))
            if entry is None:
                continue

            t, loaded, fz = load_raw_series(entry["csv_path"])
            fz_base_mean = fz[~loaded].mean()
            fz_load_mean = fz[loaded].mean()

            load_start = t[loaded][0]
            t_load = t[loaded] - load_start  # re-zero to the start of the loaded window
            fz_load = fz[loaded]

            session = core.load_session(entry)
            f_true = session["F_true"]  # expected force: (weight_g + hardware_g) worth
            total_g = weight + core.EXTRA_HARDWARE_G_UR_ONLY[direction]

            panel_data[(direction, weight)] = (t_load, fz_load, fz_base_mean, fz_load_mean,
                                                f_true, total_g)
            global_lo = min(global_lo, fz_load.min(), -f_true)
            global_hi = max(global_hi, fz_load.max(), f_true)

    margin = 0.05 * (global_hi - global_lo)
    y_range = (global_lo - margin, global_hi + margin)

    print(f"{'dir':>6}{'weight_g':>10}{'hardware_g':>12}{'real_g':>9}{'F_true(N)':>12}")
    for direction in ("posz", "negz"):
        for weight in WEIGHT_ORDER:
            data = panel_data.get((direction, weight))
            if data is None:
                continue
            _, _, _, _, f_true, total_g = data
            hardware_g = core.EXTRA_HARDWARE_G_UR_ONLY[direction]
            print(f"{direction:>6}{weight:>10.0f}{hardware_g:>12.0f}{total_g:>9.0f}{f_true:>12.4f}")

    # ── Pass 2: plot every panel with the shared y-range ──
    fig, axes = plt.subplots(2, len(WEIGHT_ORDER), figsize=(4 * len(WEIGHT_ORDER), 7))

    for row, direction in enumerate(("posz", "negz")):
        for col, weight in enumerate(WEIGHT_ORDER):
            ax = axes[row, col]
            data = panel_data.get((direction, weight))
            if data is None:
                ax.set_visible(False)
                continue
            t_load, fz_load, fz_base_mean, fz_load_mean, f_true, total_g = data
            color = core.WEIGHT_COLORS.get(weight, "#333")

            ax.plot(t_load, fz_load, color=color, linewidth=1.0)
            ax.axhline(fz_load_mean, color="#333333", linestyle="--", linewidth=1.0,
                       label=f"measured mean={fz_load_mean:.3f} N")
            ax.axhline(f_true, color="#2ca02c", linestyle=":", linewidth=1.3,
                       label=f"expected ±{f_true:.3f} N ({int(total_g)} g real)")
            ax.axhline(-f_true, color="#2ca02c", linestyle=":", linewidth=1.3)

            ax.set_ylim(y_range)
            ax.set_title(f"{direction} — {weight} g nominal ({int(total_g)} g real w/ hardware)\n"
                         f"dFz={fz_load_mean - fz_base_mean:+.3f} N   F_true=±{f_true:.3f} N", fontsize=8.5)
            ax.legend(fontsize=5.5, loc="lower right", framealpha=0.85)
            ax.tick_params(labelsize=7)
            if row == 1:
                ax.set_xlabel("time since load start (s)", fontsize=8)
            if col == 0:
                ax.set_ylabel("fz (N)", fontsize=8)
            ax.grid(False)

    fig.suptitle("UR wrist fz vs time, loaded window only — ur_only sessions (no load cell)\n"
                 "rows: posz (top) / negz (bottom)   |   same y-scale across all panels",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "ur_only_fz_vs_time.png")
    fig.savefig(out, dpi=150, facecolor="white")
    print(f"Saved -> {os.path.relpath(out, HERE)}")


if __name__ == "__main__":
    main()
