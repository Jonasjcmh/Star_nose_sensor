"""
plot_muca_combo_grid.py — Star-Nose Sensor | Capacitance Combination Calibration
==================================================================================
For every (combination, point) pair recorded by combination_calibration_collector.py,
bar-plots the muca board's 19-cell raw reading (mean +/- std across muca-phase
samples) and annotates the LCR meter's Cp reading (mean +/- std across lcr-phase
samples, same combination) for that combination.

Grid layout: one row per combination (2,4,6,8,10,12 units), one column per
calibration point (a,b,c,d,e) — 6 x 5 = 30 subplots.

Usage
-----
  python plot_muca_combo_grid.py
  python plot_muca_combo_grid.py logs/flat_calibration_muca_lcr_session_20260826_225636.csv
  python plot_muca_combo_grid.py --out combo_grid.png
"""

import argparse
import csv
import glob
import os
from collections import defaultdict

import numpy as np
import matplotlib.pyplot as plt

_HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(_HERE, "logs")

N_CELLS = 19
CELL_COLS = [f"cell_{i}" for i in range(1, N_CELLS + 1)]

# dataviz reference palette — categorical slots 1 (blue) and 2 (orange)
COLOR_BASELINE = "#2a78d6"
COLOR_ACTIVE = "#eb6834"

POINT_ORDER = ["a", "b", "c", "d", "e"]
POINT_TITLE = {"a": "a1", "b": "b2", "c": "c3", "d": "d4", "e": "e5"}


def latest_session_csv():
    candidates = sorted(glob.glob(os.path.join(LOG_DIR, "*.csv")), key=os.path.getmtime)
    if not candidates:
        raise FileNotFoundError(f"no session CSVs found in {LOG_DIR}")
    return candidates[-1]


def load(csv_path):
    """Return (combos, muca, lcr):
    combos: [(index, label)] sorted by index
    muca:   {(index, point_letter): [[cell_1..cell_19] per sample]}
    lcr:    {index: [Cp_pF per sample]}
    """
    combo_labels = {}
    muca = defaultdict(list)
    lcr = defaultdict(list)

    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            idx = int(row["combination_index"])
            combo_labels.setdefault(idx, row["combination_label"])

            if row["phase"] == "muca" and row["point_label"]:
                letter = row["point_label"][0]
                values = [float(row[c]) for c in CELL_COLS]
                muca[(idx, letter)].append(values)
            elif row["phase"] == "lcr":
                lcr[idx].append(float(row["Cp_pF"]))

    combos = sorted(combo_labels.items())  # [(idx, label), ...]
    return combos, muca, lcr


def combo_title(label):
    return label if label.endswith("units") else f"{label}units"


def plot_grid(combos, muca, lcr, out_path):
    nrows = len(combos)
    ncols = len(POINT_ORDER)

    fig, axes = plt.subplots(nrows, ncols, figsize=(3.6 * ncols, 3.0 * nrows), squeeze=False)
    x = np.arange(1, N_CELLS + 1)

    for r, (idx, label) in enumerate(combos):
        cp_samples = np.array(lcr.get(idx, []))
        cp_mean = cp_samples.mean() if cp_samples.size else float("nan")
        cp_std = cp_samples.std() if cp_samples.size else float("nan")

        for c, letter in enumerate(POINT_ORDER):
            ax = axes[r][c]
            samples = muca.get((idx, letter))

            if not samples:
                ax.axis("off")
                continue

            arr = np.array(samples)
            means = arr.mean(axis=0)
            stds = arr.std(axis=0)

            active_cell = int(np.argmax(means))
            colors = [COLOR_ACTIVE if i == active_cell else COLOR_BASELINE for i in range(N_CELLS)]

            ax.bar(x, means, yerr=stds, color=colors, width=0.7,
                   capsize=1.5, ecolor="#8a8a86", linewidth=0)

            ax.annotate(f"{means[active_cell]:.0f}",
                        xy=(x[active_cell], means[active_cell]),
                        xytext=(0, 3), textcoords="offset points",
                        ha="center", fontsize=7, color="#0b0b0b")

            ax.set_title(f"pt {POINT_TITLE[letter]} (cell_{active_cell + 1})", fontsize=8.5)
            ax.set_yscale("log")
            ax.set_xticks(x[::4])
            ax.set_xticklabels(x[::4], fontsize=6)
            ax.tick_params(axis="y", labelsize=6)
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)
            ax.grid(axis="y", which="major", color="#e6e5e0", linewidth=0.6, zorder=0)
            ax.set_axisbelow(True)

            if c == 0:
                ax.set_ylabel(
                    f"{combo_title(label)}\nLCR Cp={cp_mean:.3f}±{cp_std:.3f} pF",
                    fontsize=8.5,
                )
            if r == nrows - 1:
                ax.set_xlabel("cell", fontsize=7)

    handles = [
        plt.Rectangle((0, 0), 1, 1, color=COLOR_BASELINE, label="baseline cell"),
        plt.Rectangle((0, 0), 1, 1, color=COLOR_ACTIVE, label="active cell (max mean)"),
    ]
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.suptitle(
        "muca 19-cell raw reading per combination × calibration point (with LCR Cp)",
        fontsize=13, y=0.995,
    )
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.965),
               ncol=2, frameon=False, fontsize=10)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"saved {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", nargs="?", help="session CSV (default: latest in logs/)")
    parser.add_argument("--out", default=None, help="output PNG path")
    args = parser.parse_args()

    csv_path = args.csv_path or latest_session_csv()
    combos, muca, lcr = load(csv_path)
    if not muca:
        raise SystemExit(f"no muca-phase rows with a point_label found in {csv_path}")

    out_path = args.out or os.path.splitext(csv_path)[0] + "_combo_grid.png"
    plot_grid(combos, muca, lcr, out_path)


if __name__ == "__main__":
    main()
