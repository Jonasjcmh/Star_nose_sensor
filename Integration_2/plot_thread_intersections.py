#!/usr/bin/env python3
"""
plot_thread_intersections.py
Shows HOW the 19 interdome main points are NAMED from crossing threads.

The sensor is woven from two diagonal families of conductive threads:
    • LETTER threads  a,b,c,d,e   run at +60°   (display direction (4, 7))
    • NUMBER threads  1,2,3,4,5   run at −60°   (display direction (4,−7))

Every pressing point is the crossing of one letter thread and one number
thread, and it takes their names: e.g. point «b3» is where thread b meets
thread 3. This figure draws the two families, labels each thread at its end,
and marks the 19 crossings with their names.

Usage
    python3 plot_thread_intersections.py [--variant new_solid_sensor] [--show]
Outputs plots/thread_intersections_<variant>.png / .svg
"""
import os, sys, argparse
from collections import defaultdict
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate_points as cp
import matplotlib
import matplotlib.pyplot as plt

HERE      = os.path.dirname(os.path.abspath(__file__))
PLOTS_DIR = os.path.join(HERE, "plots")

DISP  = {int(k): tuple(v) for k, v in cp.DISPLAY_XY.items()}
LABEL = {int(k): v for k, v in cp.UR5_TO_LABEL.items()}

LETTER_DIR = np.array([4.0,  7.0]); LETTER_DIR /= np.hypot(*LETTER_DIR)   # +60°
NUMBER_DIR = np.array([4.0, -7.0]); NUMBER_DIR /= np.hypot(*NUMBER_DIR)   # −60°

LETTER_COLOR = "#2ca02c"   # green  – a..e
NUMBER_COLOR = "#d62728"   # red    – 1..5


def _thread_families():
    """key -> list of point coords, one dict for letters, one for numbers."""
    letters, numbers = defaultdict(list), defaultdict(list)
    for p in DISP:
        letters[LABEL[p][0]].append(np.array(DISP[p], float))
        numbers[LABEL[p][1]].append(np.array(DISP[p], float))
    return letters, numbers


def _draw_family(ax, fam, direction, color, pad=4.0, lbl_gap=3.0):
    """Draw each thread as a line through its collinear points and label it at
    the LOW (−direction) end, so a family fans out along one clean edge."""
    for key, pts in sorted(fam.items()):
        t   = [float(p @ direction) for p in pts]
        lo, hi = pts[int(np.argmin(t))], pts[int(np.argmax(t))]
        a = lo - direction * pad
        b = hi + direction * pad
        ax.plot([a[0], b[0]], [a[1], b[1]], "-", color=color, lw=2.0,
                alpha=0.55, zorder=1)
        tip = a - direction * lbl_gap
        ax.text(tip[0], tip[1], key, color=color, fontsize=15, fontweight="bold",
                ha="center", va="center", zorder=6,
                bbox=dict(boxstyle="circle,pad=0.18", fc="white", ec=color, lw=1.5))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", default="new_solid_sensor")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    if not args.show:
        matplotlib.use("Agg")

    letters, numbers = _thread_families()

    fig, ax = plt.subplots(figsize=(11.5, 10.5))

    _draw_family(ax, letters, LETTER_DIR, LETTER_COLOR)                  # a..e (+60°)
    _draw_family(ax, numbers, NUMBER_DIR, NUMBER_COLOR)                  # 1..5 (−60°)

    # 19 crossings = main points, drawn on top with their names
    mx = [DISP[p][0] for p in sorted(DISP)]
    my = [DISP[p][1] for p in sorted(DISP)]
    ax.scatter(mx, my, s=470, marker="o", facecolor="#fff2cc",
               edgecolor="k", lw=1.7, zorder=5)
    for p in sorted(DISP):
        ax.annotate(LABEL[p], DISP[p], fontsize=11, fontweight="bold",
                    ha="center", va="center", zorder=6)

    ax.set_aspect("equal")
    ax.grid(False)
    ax.axis("off")
    fig.tight_layout()

    os.makedirs(PLOTS_DIR, exist_ok=True)
    stem = os.path.join(PLOTS_DIR, f"thread_intersections_{args.variant}")
    fig.savefig(stem + ".png", dpi=200, bbox_inches="tight")
    fig.savefig(stem + ".svg", bbox_inches="tight")
    print(f"[plot] wrote {stem}.png / .svg")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
