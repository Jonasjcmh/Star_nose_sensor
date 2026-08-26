"""
plot_muca_bars.py — Star-Nose Sensor | Capacitance Combination Calibration
===========================================================================
Bar-plots the muca board's 19-cell snapshot for each calibration point
(a1, b2, c3, d4, e5, ...) recorded by combination_calibration_collector.py.

For each point, every muca-phase sample in the session CSV is averaged
per cell (cell_1..cell_19), giving one bar chart per point that shows
which cell actually lit up when that capacitor was moved to the board.

Usage
-----
  python plot_muca_bars.py
  python plot_muca_bars.py logs/testing_cap_v1_session_20260826_220641.csv
  python plot_muca_bars.py --out muca_bars.png
"""

import argparse
import csv
import glob
import os

import numpy as np
import matplotlib.pyplot as plt

_HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(_HERE, "logs")

N_CELLS = 19
CELL_COLS = [f"cell_{i}" for i in range(1, N_CELLS + 1)]

# dataviz reference palette — categorical slots 1 (blue) and 2 (orange)
COLOR_BASELINE = "#2a78d6"
COLOR_ACTIVE = "#eb6834"


def latest_session_csv():
    candidates = sorted(glob.glob(os.path.join(LOG_DIR, "*.csv")), key=os.path.getmtime)
    if not candidates:
        raise FileNotFoundError(f"no session CSVs found in {LOG_DIR}")
    return candidates[-1]


def load_muca_points(csv_path):
    """Return {point_label: [[cell_1..cell_19] per sample]} in first-seen order."""
    points = {}
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if row["phase"] != "muca" or not row["point_label"]:
                continue
            label = row["point_label"]
            values = [float(row[c]) for c in CELL_COLS]
            points.setdefault(label, []).append(values)
    return points


def plot_points(points, out_path):
    labels = list(points.keys())
    n = len(labels)
    ncols = 3
    nrows = -(-n // ncols)  # ceil

    fig, axes = plt.subplots(nrows, ncols, figsize=(5.2 * ncols, 4 * nrows), squeeze=False)
    x = np.arange(1, N_CELLS + 1)

    for idx, label in enumerate(labels):
        ax = axes[idx // ncols][idx % ncols]
        samples = np.array(points[label])  # (n_samples, 19)
        means = samples.mean(axis=0)
        stds = samples.std(axis=0)

        active_cell = int(np.argmax(means))
        colors = [COLOR_ACTIVE if i == active_cell else COLOR_BASELINE for i in range(N_CELLS)]

        ax.bar(x, means, yerr=stds, color=colors, width=0.7,
               capsize=2, ecolor="#8a8a86", linewidth=0)

        # direct label on the one bar that matters — not every bar
        ax.annotate(f"{means[active_cell]:.0f}",
                    xy=(x[active_cell], means[active_cell]),
                    xytext=(0, 4), textcoords="offset points",
                    ha="center", fontsize=9, color="#0b0b0b")

        ax.set_title(f"point {label}  (active: cell_{active_cell + 1})", fontsize=11)
        ax.set_xlabel("cell")
        ax.set_ylabel("muca raw reading")
        ax.set_yscale("log")
        ax.set_xticks(x)
        ax.set_xticklabels(x, fontsize=7)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.grid(axis="y", which="major", color="#e6e5e0", linewidth=0.8, zorder=0)
        ax.set_axisbelow(True)

    # hide unused subplot slots
    for idx in range(n, nrows * ncols):
        axes[idx // ncols][idx % ncols].axis("off")

    handles = [
        plt.Rectangle((0, 0), 1, 1, color=COLOR_BASELINE, label="baseline cell"),
        plt.Rectangle((0, 0), 1, 1, color=COLOR_ACTIVE, label="active cell (max mean)"),
    ]
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.suptitle("muca 19-cell response per calibration point", fontsize=13, y=0.99)
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.955),
               ncol=2, frameon=False, fontsize=10)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"saved {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", nargs="?", help="session CSV (default: latest in logs/)")
    parser.add_argument("--out", default=None, help="output PNG path")
    args = parser.parse_args()

    csv_path = args.csv_path or latest_session_csv()
    points = load_muca_points(csv_path)
    if not points:
        raise SystemExit(f"no muca-phase rows with a point_label found in {csv_path}")

    out_path = args.out or os.path.splitext(csv_path)[0] + "_bars.png"
    plot_points(points, out_path)


if __name__ == "__main__":
    main()
