#!/usr/bin/env python3
"""
plot_interdome_points.py
Reference visualization of the interdome pressing grid in the NEW a1..e5 label
scheme: the 19 main lattice points plus the three intermediate sets
(triangle centroids, diagonal midpoints, horizontal midpoints).

Coordinates are drawn in the SENSOR / DISPLAY frame (calibrate_points.DISPLAY_XY
for the mains, disp_x_mm/disp_y_mm for the intermediates) — the natural hex
layout where rows are constant-y lines. This is the frame you want for a figure;
the robot-frame x_mm/y_mm is a fixed rotation of it.

Usage
    python3 plot_interdome_points.py                       # default variant
    python3 plot_interdome_points.py --variant new_flat_sensor
    python3 plot_interdome_points.py --int-labels          # also label intermediates
    python3 plot_interdome_points.py --show                # pop up a window

Outputs plots/interdome_points_<variant>.png and .svg
"""
import os
import sys
import json
import argparse
import itertools

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate_points as cp

import matplotlib
import matplotlib.pyplot as plt

HERE       = os.path.dirname(os.path.abspath(__file__))
PLOTS_DIR  = os.path.join(HERE, "plots")

DISP  = {int(k): tuple(v) for k, v in cp.DISPLAY_XY.items()}   # pt -> (x,y) sensor frame
LABEL = {int(k): v for k, v in cp.UR5_TO_LABEL.items()}        # pt -> 'a1'..'e5'

# Same three sets / order as generate_intermediate_points.EXTRA_SPECS.
SETS = [
    ("triangle",   "triangle_centroids",  "#d62728", "D", "Triangle centroids"),
    ("diagonal",   "diagonal_midpoints",  "#2ca02c", "^", "Diagonal midpoints"),
    ("horizontal", "horizontal_midpoints", "#1f77b4", "s", "Horizontal midpoints"),
]


def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def lattice_edges(coords, gap_factor=1.3):
    """Hex-neighbour edges on the theoretical display grid (for context lines)."""
    ids   = list(coords)
    dists = sorted(set(round(_dist(coords[a], coords[b]), 6)
                       for a, b in itertools.combinations(ids, 2)))
    thr = dists[-1]
    for i in range(len(dists) - 1):
        if dists[i + 1] / dists[i] > gap_factor:
            thr = (dists[i] + dists[i + 1]) / 2.0
            break
    return [(i, j) for i, j in itertools.combinations(ids, 2)
            if _dist(coords[i], coords[j]) <= thr]


def load_set(prefix, variant):
    name = f"{prefix}_{variant}.json" if variant else f"{prefix}.json"
    path = os.path.join(HERE, name)
    if not os.path.exists(path):
        return None, path
    with open(path) as f:
        return json.load(f), path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", default="new_solid_sensor",
                    help="variant tag, e.g. new_solid_sensor / new_flat_sensor "
                         "/ new_hollow_sensor (default: new_solid_sensor)")
    ap.add_argument("--int-labels", action="store_true",
                    help="also draw the intermediate-point labels (busy)")
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    args = ap.parse_args()

    if not args.show:
        matplotlib.use("Agg")

    fig, ax = plt.subplots(figsize=(12, 11))

    # ── hex lattice context lines ────────────────────────────────────────────
    for i, j in lattice_edges(DISP):
        ax.plot([DISP[i][0], DISP[j][0]], [DISP[i][1], DISP[j][1]],
                "-", color="0.82", lw=1.2, zorder=1)

    # ── intermediate points ──────────────────────────────────────────────────
    n_by_kind = {}
    for kind, prefix, color, marker, label in SETS:
        data, path = load_set(prefix, args.variant)
        if data is None:
            print(f"[warn] missing {os.path.basename(path)} — skipping {kind}")
            n_by_kind[kind] = 0
            continue
        xs = [r["disp_x_mm"] for r in data.values()]
        ys = [r["disp_y_mm"] for r in data.values()]
        ax.scatter(xs, ys, s=70, marker=marker, facecolor=color, edgecolor="k",
                   linewidths=0.5, alpha=0.9, zorder=3,
                   label=f"{label} (n={len(data)})")
        if args.int_labels:
            for r in data.values():
                ax.annotate(r["vertices"] if isinstance(r["vertices"], str)
                            else "".join(r["vertices"]),
                            (r["disp_x_mm"], r["disp_y_mm"]),
                            fontsize=5, color=color, ha="center", va="center",
                            xytext=(0, 7), textcoords="offset points", zorder=4)
        n_by_kind[kind] = len(data)

    # ── 19 main points (drawn last, on top) ──────────────────────────────────
    mx = [DISP[p][0] for p in sorted(DISP)]
    my = [DISP[p][1] for p in sorted(DISP)]
    ax.scatter(mx, my, s=430, marker="o", facecolor="#fff2cc",
               edgecolor="k", linewidths=1.6, zorder=5,
               label="Main points (n=19)")
    for p in sorted(DISP):
        ax.annotate(f"{LABEL[p]}", DISP[p], fontsize=10, fontweight="bold",
                    ha="center", va="center", zorder=6)

    ax.set_aspect("equal")
    ax.set_xlabel("sensor-frame x  [mm]")
    ax.set_ylabel("sensor-frame y  [mm]")
    ax.set_title(f"Interdome pressing grid — variant '{args.variant}'\n"
                 f"19 main points + {n_by_kind.get('triangle',0)} centroids + "
                 f"{n_by_kind.get('diagonal',0)} diagonal + "
                 f"{n_by_kind.get('horizontal',0)} horizontal midpoints "
                 f"({19 + sum(n_by_kind.values())} total)")
    ax.grid(True, ls=":", color="0.9", zorder=0)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), framealpha=0.95)
    fig.tight_layout()

    os.makedirs(PLOTS_DIR, exist_ok=True)
    stem = os.path.join(PLOTS_DIR, f"interdome_points_{args.variant}")
    fig.savefig(stem + ".png", dpi=200, bbox_inches="tight")
    fig.savefig(stem + ".svg", bbox_inches="tight")
    print(f"[plot] wrote {stem}.png")
    print(f"[plot] wrote {stem}.svg")
    print(f"[plot] totals: 19 main + " +
          " + ".join(f"{n} {k}" for k, n in n_by_kind.items()) +
          f" = {19 + sum(n_by_kind.values())} points")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
