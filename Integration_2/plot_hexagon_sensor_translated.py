import json
import os
import math

import matplotlib
matplotlib.use("Agg")   # headless-safe; swap to TkAgg if you want an interactive window
import matplotlib.pyplot as plt
from matplotlib.patches import RegularPolygon

# ── Paths (edit if your files live elsewhere) ─────────────────────────────────
BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"

TRANSLATED_JSON = os.path.join(BASE_DIR, "calib_points_supposed_translated.json")
HORIZONTAL_JSON = os.path.join(BASE_DIR, "horizontal_midpoints_translated.json")
DIAGONAL_JSON   = os.path.join(BASE_DIR, "diagonal_midpoints_translated.json")
TRIANGLE_JSON   = os.path.join(BASE_DIR, "triangle_centroids_translated.json")
MOCAP_JSON      = os.path.join(BASE_DIR, "calib_points_short_6mm.json")
OUT_PNG         = os.path.join(BASE_DIR, "calibration_points_plotting.svg")

# Hexagon circumradius for drawing the 19 unit-cell outlines.
# Derived from same-row point spacing (8mm) so adjacent hexagons tile
# edge-to-edge with no gaps or overlaps: spacing = sqrt(3) * radius
# for pointy-top hexagons (vertex pointing up/down, matching the
# physical sensor's actual hex cell orientation).
HEX_RADIUS = 8.0 / math.sqrt(3)   # ~4.6188 mm

# calib_points_short_6mm.json's per_point offsets are relative to
# ORIGINAL_POINTS (the numbering the robot actually used) — and
# calib_points_supposed_translated.json ALSO uses that same original
# numbering (since translate_supposed_points.py was built on
# ORIGINAL_POINTS). So no ID conversion is needed for PAIRING/matching
# points to each other — both files already agree on the same numbering.
ORIGINAL_POINTS = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

# DIAGONAL/physical-chip numbering (matches the numbers actually printed
# on the sensor, verified against the chip photo). Used ONLY for the
# text labels drawn on the plot — NOT for pairing/matching points to
# each other (that's done in original-numbering space above, since
# that's what the data files use internally).
DIAGONAL_POINTS = {
     1: (  -8.0,  -14.0),   2: ( -12.0,   -7.0),   3: ( -16.0,   +0.0),
     4: (  +0.0,  -14.0),   5: (  -4.0,   -7.0),   6: (  -8.0,   +0.0),
     7: ( -12.0,   +7.0),   8: (  +8.0,  -14.0),   9: (  +4.0,   -7.0),
    10: (  +0.0,   +0.0),  11: (  -4.0,   +7.0),  12: (  -8.0,  +14.0),
    13: ( +12.0,   -7.0),  14: (  +8.0,   +0.0),  15: (  +4.0,   +7.0),
    16: (  +0.0,  +14.0),  17: ( +16.0,   +0.0),  18: ( +12.0,   +7.0),
    19: (  +8.0,  +14.0),
}
_COORD_TO_DIAG_ID = {v: k for k, v in DIAGONAL_POINTS.items()}
ORIGINAL_ID_TO_DIAGONAL_LABEL = {
    orig_id: _COORD_TO_DIAG_ID[coord] for orig_id, coord in ORIGINAL_POINTS.items()
}


def load_json(path):
    if not os.path.exists(path):
        print(f"\n[!!! WARNING !!!] File not found, this data will NOT appear in the plot: {path}\n")
        return {}
    with open(path) as f:
        return json.load(f)


def load_translated_points(path):
    """
    {diagonal_point_id (int): (x_mm, y_mm)} from
    calib_points_supposed_translated.json. This file's own point IDs are
    already in the diagonal/physical-chip numbering (same as POINTS
    above, just translated), so no ID conversion is needed here —
    only the mocap file needs conversion (see load_mocap_offsets).
    """
    data = load_json(path)
    if not data:
        return {}
    return {int(pid): (d["x_mm"], d["y_mm"]) for pid, d in data["points"].items()}


def load_mocap_offsets(path):
    """
    Returns dict {point_id (int): (actual_x_mm, actual_y_mm)}, keyed
    with the SAME original numbering used by
    calib_points_supposed_translated.json — no ID conversion needed,
    since both files already agree on that numbering.
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
        if pid not in ORIGINAL_POINTS:
            continue
        nom_x, nom_y = ORIGINAL_POINTS[pid]
        dx, dy = off.get("dx_mm", 0.0), off.get("dy_mm", 0.0)
        actual[pid] = (nom_x + gx + dx, nom_y + gy + dy)
    return actual


def main():
    translated = load_translated_points(TRANSLATED_JSON)
    horiz = load_json(HORIZONTAL_JSON)
    diag  = load_json(DIAGONAL_JSON)
    tri   = load_json(TRIANGLE_JSON)
    actual = load_mocap_offsets(MOCAP_JSON)

    fig, ax = plt.subplots(figsize=(9, 9))

    # ── 19 unit hexagons, centered on the TRANSLATED nominal points ────────────
    # (drawn first, so they sit behind the point markers)
    for pid, (cx, cy) in translated.items():
        hexagon = RegularPolygon(
            (cx, cy), numVertices=6, radius=HEX_RADIUS,
            orientation=0,   # 0 = pointy-top, matches sensor cell shape
            facecolor="none", edgecolor="#888888", linewidth=1.0, zorder=1)
        ax.add_patch(hexagon)

    # ── 19 translated nominal points ───────────────────────────────────────────
    if translated:
        xs = [x for x, y in translated.values()]
        ys = [y for x, y in translated.values()]
        ax.scatter(xs, ys, c="black", marker="s", s=90, zorder=5, label="Nominal, translated (19)")
        for pid, (x, y) in translated.items():
            diag_label = ORIGINAL_ID_TO_DIAGONAL_LABEL[pid]
            ax.annotate(str(diag_label), (x, y), textcoords="offset points",
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
            diag_label = ORIGINAL_ID_TO_DIAGONAL_LABEL[pid]
            ax.annotate(str(diag_label), (x, y), textcoords="offset points",
                        xytext=(0, -12), ha="center", fontsize=8,
                        fontweight="bold", color="red")

        mags = []
        for pid, (ax_x, ax_y) in actual.items():
            nom_x, nom_y = translated[pid]
            dx, dy = ax_x - nom_x, ax_y - nom_y
            mag = (dx**2 + dy**2) ** 0.5
            mags.append(mag)

        if mags:
            print(f"[deviation] mean = {sum(mags)/len(mags):.3f} mm, "
                  f"max = {max(mags):.3f} mm")

    total = len(translated) + len(horiz) + len(diag) + len(tri)
    ax.set_title(f"All calibration points (translated)  ({total} total)", fontsize=13, fontweight="bold")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9)

    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=150)
    print(f"Saved -> {OUT_PNG}")
    print(f"Total points plotted: {total}")


if __name__ == "__main__":
    main()