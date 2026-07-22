"""
caliberate_plot_rigid.py
Plots all calibration point tiers using the RIGID-TRANSFORMED dataset
(rotation + translation fit via fit_rigid_transform_supposed_points.py):
    - 19 rigid-transformed "supposed to be" points  (black squares + hexagon cells)
    - 19 actual robot-calibrated points               (red squares, labeled)
    - 14 horizontal edge-midpoints                     (blue circles,   H labels)
    - 28 diagonal edge-midpoints                        (green circles,  D labels)
    - 24 triangle centroids                             (orange triangles, T labels)

Reads:
    calib_points_supposed_rigid_transformed.json  -> rigid-transformed nominal (black) points
    calib_points_short_6mm.json                   -> actual (red) points
    horizontal_midpoints_rigid.json               -> H edge midpoints
    diagonal_midpoints_rigid.json                 -> D edge midpoints
    triangle_centroids_rigid.json                 -> T centroids

Labels displayed use the DIAGONAL/physical-chip numbering (matching
the actual sensor chip photo), even though all underlying pairing/
math is done in the original arbitrary numbering (matching the mocap
file) — same label-conversion approach as
calibration_points_plotting_fixed.py.

Hexagons are rotated along with the points (same rotation angle as
the fitted transform), so they stay aligned with the point grid
instead of just being redrawn axis-aligned at the wrong orientation.

Output: calibration_points_plot_rigid.png
────────────────────────────────────────────────────────────────────
"""

import json
import os
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import RegularPolygon

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"

RIGID_JSON      = os.path.join(BASE_DIR, "calib_points_supposed_rigid_transformed.json")
MOCAP_JSON      = os.path.join(BASE_DIR, "calib_points_short_6mm.json")
HORIZONTAL_JSON = os.path.join(BASE_DIR, "horizontal_midpoints_rigid.json")
DIAGONAL_JSON   = os.path.join(BASE_DIR, "diagonal_midpoints_rigid.json")
TRIANGLE_JSON   = os.path.join(BASE_DIR, "triangle_centroids_rigid.json")
OUT_PNG         = os.path.join(BASE_DIR, "calibration_points_plot_rigid.svg")

HEX_RADIUS = 8.0 / math.sqrt(3)   # ~4.6188 mm
ANCHOR = 10   # pivot point for the view-rotation (matches the anchor used in the fit)

# Original nominal points (original arbitrary numbering) — needed to
# interpret calib_points_short_6mm.json, since its per_point offsets
# are relative to THIS grid.
ORIGINAL_POINTS = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

# DIAGONAL/physical-chip numbering — used ONLY for display labels
# (matches the numbers actually printed on the sensor chip).
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


def load_rigid_points(path):
    """{point_id: (x_mm, y_mm)} from calib_points_supposed_rigid_transformed.json"""
    data = load_json(path)
    if not data:
        return {}, 0.0
    points = {int(pid): (d["x_mm"], d["y_mm"]) for pid, d in data["points"].items()}
    rotation_deg = data.get("rotation_deg", 0.0)
    return points, rotation_deg


def load_actual_points(path):
    """{point_id: (actual_x_mm, actual_y_mm)}, keyed with ORIGINAL numbering."""
    data = load_json(path)
    if not data or "per_point" not in data:
        return {}
    g = data.get("global", {})
    gx, gy = g.get("x_mm", 0.0), g.get("y_mm", 0.0)
    actual = {}
    for key, off in data["per_point"].items():
        pid = int(key)
        if pid not in ORIGINAL_POINTS:
            continue
        nx, ny = ORIGINAL_POINTS[pid]
        dx, dy = off.get("dx_mm", 0.0), off.get("dy_mm", 0.0)
        actual[pid] = (nx + gx + dx, ny + gy + dy)
    return actual


def rotate_point(px, py, pivot_x, pivot_y, angle_deg):
    """Rotate a single (px, py) around (pivot_x, pivot_y) by angle_deg."""
    theta = math.radians(angle_deg)
    dx, dy = px - pivot_x, py - pivot_y
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    new_dx = dx * cos_t - dy * sin_t
    new_dy = dx * sin_t + dy * cos_t
    return pivot_x + new_dx, pivot_y + new_dy


def rotate_dict(points_dict, pivot_x, pivot_y, angle_deg):
    """Rotate every (x,y) pair in a {id: (x,y)} dict around a pivot."""
    return {k: rotate_point(x, y, pivot_x, pivot_y, angle_deg)
            for k, (x, y) in points_dict.items()}


def rotate_labeled_dict(labeled_dict, pivot_x, pivot_y, angle_deg, key="x_mm", key_y="y_mm"):
    """Rotate the x_mm/y_mm fields inside a {label: {...}} dict (H/D/T format)."""
    out = {}
    for label, d in labeled_dict.items():
        new_x, new_y = rotate_point(d[key], d[key_y], pivot_x, pivot_y, angle_deg)
        out[label] = {**d, key: round(new_x, 4), key_y: round(new_y, 4)}
    return out


def main():
    rigid, rotation_deg = load_rigid_points(RIGID_JSON)
    actual = load_actual_points(MOCAP_JSON)
    horiz  = load_json(HORIZONTAL_JSON)
    diag   = load_json(DIAGONAL_JSON)
    tri    = load_json(TRIANGLE_JSON)

    # ── VIEW ROTATION: spin the ENTIRE scene back to upright, purely for ──────
    # display. This does NOT change any actual alignment/residual numbers --
    # it's the same idea as rotating a photo instead of rotating the object
    # in it. We rotate every point (nominal, actual, H, D, T) by the SAME
    # -rotation_deg around the SAME pivot (point 10's position, since that's
    # the one guaranteed to sit exactly on its actual counterpart), so all
    # relative distances and alignments stay frozen -- only the picture's
    # overall tilt on screen changes.
    if rigid:
        pivot_x, pivot_y = rigid[ANCHOR]
        view_angle = -rotation_deg
        rigid  = rotate_dict(rigid, pivot_x, pivot_y, view_angle)
        actual = rotate_dict(actual, pivot_x, pivot_y, view_angle)
        horiz  = rotate_labeled_dict(horiz, pivot_x, pivot_y, view_angle)
        diag   = rotate_labeled_dict(diag, pivot_x, pivot_y, view_angle)
        tri    = rotate_labeled_dict(tri, pivot_x, pivot_y, view_angle)
        # After undoing the rotation for the view, hexagons drawn straight
        # (orientation=0) will tile correctly again -- the tilt that used to
        # require rotated hexagons has been rotated away along with everything else.
        rotation_deg = 0.0

    fig, ax = plt.subplots(figsize=(9, 9))

    # ── 19 unit hexagons, rotated + centered along with the points, so ─────────
    # edges between neighboring hexagons touch perfectly (see explanation:
    # rotating centers without rotating the hexagons themselves breaks tiling).
    hex_orientation_rad = math.radians(rotation_deg)
    for pid, (cx, cy) in rigid.items():
        hexagon = RegularPolygon(
            (cx, cy), numVertices=6, radius=HEX_RADIUS,
            orientation=hex_orientation_rad,
            facecolor="none", edgecolor="#888888", linewidth=1.0, zorder=1)
        ax.add_patch(hexagon)

    # ── Rigid-transformed nominal (black) points ────────────────────────────────
    if rigid:
        xs = [x for x, y in rigid.values()]
        ys = [y for x, y in rigid.values()]
        ax.scatter(xs, ys, c="black", marker="s", s=90, zorder=5,
                   label="Nominal, rigid-fit (19)")
        for pid, (x, y) in rigid.items():
            diag_label = ORIGINAL_ID_TO_DIAGONAL_LABEL[pid]
            ax.annotate(str(diag_label), (x, y), textcoords="offset points",
                        xytext=(0, 8), ha="center", fontsize=8, fontweight="bold")

    # ── Horizontal midpoints ──────────────────────────────────────────────────
    if horiz:
        xs = [d["x_mm"] for d in horiz.values()]
        ys = [d["y_mm"] for d in horiz.values()]
        ax.scatter(xs, ys, c="#1f77b4", marker="o", s=55, zorder=4,
                   label=f"Horizontal ({len(horiz)})")

    # ── Diagonal midpoints ────────────────────────────────────────────────────
    if diag:
        xs = [d["x_mm"] for d in diag.values()]
        ys = [d["y_mm"] for d in diag.values()]
        ax.scatter(xs, ys, c="#2ca02c", marker="o", s=55, zorder=4,
                   label=f"Diagonal ({len(diag)})")

    # ── Triangle centroids ────────────────────────────────────────────────────
    if tri:
        xs = [d["x_mm"] for d in tri.values()]
        ys = [d["y_mm"] for d in tri.values()]
        ax.scatter(xs, ys, c="#ff7f0e", marker="^", s=55, zorder=4,
                   label=f"Triangle centroid ({len(tri)})")

    # ── Actual (robot-calibrated) points, labeled ──────────────────────────────
    if actual:
        xs = [actual[pid][0] for pid in sorted(actual)]
        ys = [actual[pid][1] for pid in sorted(actual)]
        ax.scatter(xs, ys, c="red", marker="s", s=70, zorder=6, linewidths=2,
                   label=f"Actual (robot-calib, {len(actual)})")
        for pid in sorted(actual):
            x, y = actual[pid]
            diag_label = ORIGINAL_ID_TO_DIAGONAL_LABEL[pid]
            ax.annotate(str(diag_label), (x, y), textcoords="offset points",
                        xytext=(0, -12), ha="center", fontsize=8,
                        fontweight="bold", color="red")

        mags = []
        for pid, (ax_x, ax_y) in actual.items():
            nom_x, nom_y = rigid[pid]
            mags.append(math.hypot(ax_x - nom_x, ax_y - nom_y))
        if mags:
            print(f"[residual] mean = {sum(mags)/len(mags):.4f} mm, "
                  f"max = {max(mags):.4f} mm")

    total = len(rigid) + len(horiz) + len(diag) + len(tri)
    ax.set_title("Calibration", fontsize=13, fontweight="bold")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9)

    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=150)
    print(f"Saved -> {OUT_PNG}")
    print(f"Total points plotted (excluding actual overlay): {total}")


if __name__ == "__main__":
    main()