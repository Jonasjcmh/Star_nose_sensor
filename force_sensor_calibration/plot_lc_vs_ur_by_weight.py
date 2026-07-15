#!/usr/bin/env python3
"""
plot_lc_vs_ur_by_weight.py — LC (load cell) vs UR robot force, per weight,
using only the fzcal_futek_direct_* sessions (load cell installed;
fzcal_ur_only_* has no load cell and is excluded here).

Reuses fit_lc_ur_calibration.py's discovery/de-dupe and load-cell
voltage<->force fit, so both scripts agree on the numbers.

For each deduped (direction, weight) session, using the LOADED phase's
absolute readings (the fit itself is calibrated on absolute ai0/fz,
baseline included as a known non-zero point — see fit_lc_ur_calibration.py):
  F_lc        = m_v * ai0_load_mean + c_v                     (load-cell force)
  F_ur_raw    = fz_load_mean                                   (UR fz, raw, uncorrected)
  F_ur_signed = AI0_SIGN[direction] * abs(fz_load_mean)         (UR fz, sign-corrected
                to the load cell's own push/pull convention — see fit_lc_ur_calibration.py)

Two panels (posz / negz), grouped bars per weight, THREE bars each: LC,
UR raw, and UR sign-corrected. The raw bar keeps the original sign
mismatch visible (negz in particular can come out the wrong way vs LC);
the sign-corrected bar puts F_ur on the same sign convention as F_lc so
the two magnitudes can actually be compared directly, bar height to bar
height.

Usage:
    python plot_lc_vs_ur_by_weight.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import fit_lc_ur_calibration as core

HERE = core.HERE
OUT_DIR = core.OUT_DIR


def main():
    entries = core.dedupe_latest(core.discover("futek_direct"))
    sessions = sorted((core.load_session(e) for e in entries),
                       key=lambda s: (s["direction"], s["weight_g"]))

    points = core.expand_phases(sessions)
    m_v, c_v, r2_v, rmse_v = core.fit_loadcell_rate(points)
    for s in sessions:
        s["F_lc"] = m_v * s["ai0_load_mean"] + c_v
        s["F_ur_raw"] = s["fz_load_mean"]
        s["F_ur_signed"] = core.AI0_SIGN[s["direction"]] * abs(s["fz_load_mean"])

    print(f"LC voltage<->force fit used: F_lc = {m_v:.4f}*ai0 + ({c_v:.5f})  (R^2={r2_v:.5f})")
    print(f"{'weight_g':>8}{'dir':>6}{'F_lc(N)':>10}{'F_ur_raw(N)':>12}{'F_ur_sign(N)':>13}"
          f"{'diff_raw(N)':>12}{'diff_sign(N)':>13}")
    for s in sessions:
        print(f"{s['weight_g']:>8.0f}{s['direction']:>6}{s['F_lc']:>10.4f}"
              f"{s['F_ur_raw']:>12.4f}{s['F_ur_signed']:>13.4f}"
              f"{s['F_ur_raw'] - s['F_lc']:>12.4f}{s['F_ur_signed'] - s['F_lc']:>13.4f}")

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5), sharey=True)

    for ax, direction in zip(axes, ["posz", "negz"]):
        d_sessions = sorted([s for s in sessions if s["direction"] == direction],
                             key=lambda s: s["weight_g"])
        x = np.arange(len(d_sessions))
        bw = 0.26

        ax.bar(x - bw, [s["F_lc"] for s in d_sessions], bw,
               label="LC (load cell)", color="#1a6eb5", zorder=3)
        ax.bar(x, [s["F_ur_raw"] for s in d_sessions], bw,
               label="UR robot (fz, raw)", color="#d62728", alpha=0.45, zorder=3)
        ax.bar(x + bw, [s["F_ur_signed"] for s in d_sessions], bw,
               label="UR robot (fz, sign-corrected)", color="#d62728", zorder=3)

        ax.axhline(0, color="black", lw=0.8, alpha=0.6)
        ax.set_xticks(x)
        ax.set_xticklabels([f"{int(s['weight_g'])} g" for s in d_sessions])
        ax.set_title(direction)
        ax.set_xlabel("Applied weight")
        ax.grid(axis="y", alpha=0.25)

    axes[0].set_ylabel("Force (N), absolute (loaded phase)")
    axes[0].legend(fontsize=8.5, loc="upper left")
    fig.suptitle("LC vs UR robot force per weight — futek_direct sessions (load cell installed)\n"
                 "UR sign-corrected: fz_signed = AI0_SIGN[direction]*|fz_raw|, same convention as the load cell")

    out = os.path.join(OUT_DIR, "lc_vs_ur_by_weight.png")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor="white")
    print(f"\nSaved -> {os.path.relpath(out, HERE)}")


if __name__ == "__main__":
    main()
