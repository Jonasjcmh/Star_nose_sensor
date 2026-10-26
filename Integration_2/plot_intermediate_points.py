import json
import os

import matplotlib
matplotlib.use("Agg")   
import matplotlib.pyplot as plt

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"

HORIZONTAL_JSON = os.path.join(BASE_DIR, "horizontal_midpoints.json")
DIAGONAL_JSON   = os.path.join(BASE_DIR, "diagonal_midpoints.json")
TRIANGLE_JSON   = os.path.join(BASE_DIR, "triangle_centroids.json")
OUT_PNG         = os.path.join(BASE_DIR, "calibration_points_plot.png")

POINTS = {
     1: (  -8.0,  -14.0),   2: ( -12.0,   -7.0),   3: ( -16.0,   +0.0),
     4: (  +0.0,  -14.0),   5: (  -4.0,   -7.0),   6: (  -8.0,   +0.0),
     7: ( -12.0,   +7.0),   8: (  +8.0,  -14.0),   9: (  +4.0,   -7.0),
    10: (  +0.0,   +0.0),  11: (  -4.0,   +7.0),  12: (  -8.0,  +14.0),
    13: ( +12.0,   -7.0),  14: (  +8.0,   +0.0),  15: (  +4.0,   +7.0),
    16: (  +0.0,  +14.0),  17: ( +16.0,   +0.0),  18: ( +12.0,   +7.0),
    19: (  +8.0,  +14.0),
}

def load_json(path):
    if not os.path.exists(path):
        print(f"[warn] Missing file, skipping: {path}")
        return {}
    with open(path) as f:
        return json.load(f)
 
 
def main():
    horiz = load_json(HORIZONTAL_JSON)
    diag  = load_json(DIAGONAL_JSON)
    tri   = load_json(TRIANGLE_JSON)
 
    fig, ax = plt.subplots(figsize=(9, 9))

    # ── 19 original points ────────────────────────────────────────────────────
    xs = [x for x, y in POINTS.values()]
    ys = [y for x, y in POINTS.values()]
    ax.scatter(xs, ys, c="black", marker="s", s=90, zorder=5, label="Original (19)")
    for pid, (x, y) in POINTS.items():
        ax.annotate(str(pid), (x, y), textcoords="offset points",
                    xytext=(0, 8), ha="center", fontsize=8, fontweight="bold")
 
    # ── Horizontal midpoints ──────────────────────────────────────────────────
    if horiz:
        xs = [d["x_mm"] for d in horiz.values()]
        ys = [d["y_mm"] for d in horiz.values()]
        ax.scatter(xs, ys, c="#1f77b4", marker="o", s=55, zorder=4, label=f"Horizontal ({len(horiz)})")
 
    # ── Diagonal midpoints ────────────────────────────────────────────────────
    if diag:
        xs = [d["x_mm"] for d in diag.values()]
        ys = [d["y_mm"] for d in diag.values()]
        ax.scatter(xs, ys, c="#2ca02c", marker="o", s=55, zorder=4, label=f"Diagonal ({len(diag)})")
 
    # ── Triangle centroids ────────────────────────────────────────────────────
    if tri:
        xs = [d["x_mm"] for d in tri.values()]
        ys = [d["y_mm"] for d in tri.values()]
        ax.scatter(xs, ys, c="#ff7f0e", marker="^", s=55, zorder=4, label=f"Triangle centroid ({len(tri)})")
 
    total = 19 + len(horiz) + len(diag) + len(tri)
    ax.set_title(f"All calibration points  ({total} total)", fontsize=13, fontweight="bold")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
 
    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=150)
    print(f"Saved -> {OUT_PNG}")
    print(f"Total points plotted: {total}")
 
 
if __name__ == "__main__":
    main()
 