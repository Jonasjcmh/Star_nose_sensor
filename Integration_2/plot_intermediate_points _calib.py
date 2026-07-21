"""
caliberate_plot_all_points.py
Plots all four tiers of calibration points for the 19-point hex sensor:
    - 19 original (nominal) vertex points     (black squares)
    - 14 horizontal edge-midpoints             (blue circles,   H labels)
    - 28 diagonal edge-midpoints                (green circles,  D labels)
    - 24 triangle centroids                     (orange triangles, T labels)
    - actual robot-calibrated positions for the 19 points
      (red squares, from calib_short_mocap.json — deviation magnitude
      from nominal is printed to console, not drawn as arrows)

Reads the JSON files produced by:
    caliberate_intermediate_points.py   -> horizontal_midpoints.json
    caliberate_diagonal_midpoints.py    -> diagonal_midpoints.json
    caliberate_triangle_centroids.py    -> triangle_centroids.json
    (your UR5 calibration run)          -> calib_short_mocap.json

MOCAP FILE FORMAT ASSUMED (same as calib_points.json from
calibrate_points.py):
    {
      "global": {"x_mm": ..., "y_mm": ..., "z_mm": ...},
      "per_point": {"1": {"dx_mm": ..., "dy_mm": ...}, ...},
      "scan_results": {...}
    }
Actual position for point N = nominal(N) + global offset + per_point(N)
offset. If your file uses a different structure, see load_mocap_offsets()
below — it's isolated in one function so it's easy to adapt.

────────────────────────────────────────────────────────────────────
Usage:
  python3 caliberate_plot_all_points.py
Output: calibration_points_plot.png (saved next to the JSON files)
────────────────────────────────────────────────────────────────────
"""

import json
import os

import matplotlib
matplotlib.use("Agg")   # headless-safe; swap to TkAgg if you want an interactive window
import matplotlib.pyplot as plt

# ── Paths (edit if your files live elsewhere) ─────────────────────────────────
BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"

HORIZONTAL_JSON = os.path.join(BASE_DIR, "horizontal_midpoints.json")
DIAGONAL_JSON   = os.path.join(BASE_DIR, "diagonal_midpoints.json")
TRIANGLE_JSON   = os.path.join(BASE_DIR, "triangle_centroids.json")
MOCAP_JSON      = os.path.join(BASE_DIR, "calib_points_short_20july.json")
OUT_SVG         = os.path.join(BASE_DIR, "calibration_points_plot_calib1.svg")

# Original 19 points (physical sensor numbering, verified against chip photo)
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


def load_mocap_offsets(path):
    """
    Returns dict {point_id (int): (actual_x_mm, actual_y_mm)} for however
    many points the mocap file covers. Handles the calib_points.json-style
    structure (global + per_point offsets). Returns {} if file is missing
    or doesn't match the expected shape (prints a warning either way).
    """
    data = load_json(path)
    if not data:
        return {}

    if "per_point" not in data:
        print(f"[warn] {path} has no 'per_point' key — skipping deviation overlay")
        return {}

    g = data.get("global", {})
    gx, gy = g.get("x_mm", 0.0), g.get("y_mm", 0.0)

    actual = {}
    for key, off in data["per_point"].items():
        try:
            pid = int(key)
        except ValueError:
            continue
        if pid not in POINTS:
            continue
        nom_x, nom_y = POINTS[pid]
        dx, dy = off.get("dx_mm", 0.0), off.get("dy_mm", 0.0)
        actual[pid] = (nom_x + gx + dx, nom_y + gy + dy)
    return actual


def main():
    horiz = load_json(HORIZONTAL_JSON)
    diag  = load_json(DIAGONAL_JSON)
    tri   = load_json(TRIANGLE_JSON)
    actual = load_mocap_offsets(MOCAP_JSON)

    fig, ax = plt.subplots(figsize=(9, 9))

    # ── 19 original (nominal) points ──────────────────────────────────────────
    xs = [x for x, y in POINTS.values()]
    ys = [y for x, y in POINTS.values()]
    ax.scatter(xs, ys, c="black", marker="s", s=90, zorder=5, label="Nominal (19)")
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

    # ── Actual (robot-calibrated) positions ───────────────────────────────────
    if actual:
        ax_pts = [actual[pid] for pid in sorted(actual)]
        ax.scatter([p[0] for p in ax_pts], [p[1] for p in ax_pts],
                   c="red", marker="s", s=70, zorder=6, linewidths=2,
                   label=f"Actual (robot-calib, {len(actual)})")
        for pid in sorted(actual):
            x, y = actual[pid]
            ax.annotate(str(pid), (x, y), textcoords="offset points",
                        xytext=(0, -12), ha="center", fontsize=8,
                        fontweight="bold", color="red")

        mags = []
        for pid, (ax_x, ax_y) in actual.items():
            nom_x, nom_y = POINTS[pid]
            dx, dy = ax_x - nom_x, ax_y - nom_y
            mag = (dx**2 + dy**2) ** 0.5
            mags.append(mag)

        if mags:
            print(f"[deviation] mean = {sum(mags)/len(mags):.3f} mm, "
                  f"max = {max(mags):.3f} mm")

    total = 19 + len(horiz) + len(diag) + len(tri)
    ax.set_title(f"All calibration points  ({total} total)", fontsize=13, fontweight="bold")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0),
              fontsize=7, borderaxespad=0.)

    plt.tight_layout()
    plt.savefig(OUT_SVG, dpi=150, bbox_inches="tight")
    print(f"Saved -> {OUT_SVG}")
    print(f"Total points plotted: {total}")


if __name__ == "__main__":
    main()