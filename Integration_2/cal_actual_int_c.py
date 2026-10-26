"""
caliberate_triangle_centroids_actual.py
Computes triangle centroids using PURE GEOMETRY on the ACTUAL
(measured, robot-calibrated) point positions -- no reference to the
theoretical grid at all.

This one didn't actually need the angle trick: triangle detection
only needs to know WHICH points are mutually adjacent (any neighbor
direction), not to separate horizontal from diagonal. Auto-detecting
the neighbor threshold (same technique as the other two scripts) and
finding 3-cliques on that adjacency graph works directly on the real,
noisy point cloud with no theoretical grid involved.

CALCULATIONS use the calibration file's own (ORIGINAL) point
numbering. LABELS are converted to HARDWARE numbering for human
readability -- see caliberate_horizontal_midpoints_actual.py's
docstring for details. This conversion never touches coordinates or
the point-finding math.

Output: triangle_centroids_actual_<TAG>.json
────────────────────────────────────────────────────────────────────
"""

import json
import os
import itertools

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"

CALIB_SOURCE_FILE = os.path.join(BASE_DIR, "calib_points_short_new_hollow_2.json")
TAG = "new_hollow_2"

OUT_PATH = os.path.join(BASE_DIR, f"triangle_centroids_actual_{TAG}.json")

# ── Label conversion ONLY (never used for coordinates/math) ──────────────────
ORIGINAL_POINTS = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}
HARDWARE_POINTS = {
     1: (  -8.0,  -14.0),   2: ( -12.0,   -7.0),   3: ( -16.0,   +0.0),
     4: (  +0.0,  -14.0),   5: (  -4.0,   -7.0),   6: (  -8.0,   +0.0),
     7: ( -12.0,   +7.0),   8: (  +8.0,  -14.0),   9: (  +4.0,   -7.0),
    10: (  +0.0,   +0.0),  11: (  -4.0,   +7.0),  12: (  -8.0,  +14.0),
    13: ( +12.0,   -7.0),  14: (  +8.0,   +0.0),  15: (  +4.0,   +7.0),
    16: (  +0.0,  +14.0),  17: ( +16.0,   +0.0),  18: ( +12.0,   +7.0),
    19: (  +8.0,  +14.0),
}
_COORD_TO_HW_ID = {v: k for k, v in HARDWARE_POINTS.items()}
ORIGINAL_TO_HARDWARE_ID = {
    orig_id: _COORD_TO_HW_ID[coord] for orig_id, coord in ORIGINAL_POINTS.items()
}


def load_actual_points(path):
    with open(path) as f:
        data = json.load(f)
    actual = {}
    for key, d in data["points"].items():
        pid = int(key)
        ax, ay = d["offset_mm"]
        actual[pid] = (ax, ay)
    return actual


def _dist(a, b):
    (x1, y1), (x2, y2) = a, b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def find_neighbor_threshold(points, gap_factor=1.3):
    ids = list(points.keys())
    dists = sorted(set(round(_dist(points[a], points[b]), 6)
                        for a, b in itertools.combinations(ids, 2)))
    for i in range(len(dists) - 1):
        if dists[i + 1] / dists[i] > gap_factor:
            return (dists[i] + dists[i + 1]) / 2.0
    return dists[-1]


def build_adjacency(points, threshold):
    ids = list(points.keys())
    adj = {i: set() for i in ids}
    for i, j in itertools.combinations(ids, 2):
        if _dist(points[i], points[j]) <= threshold:
            adj[i].add(j)
            adj[j].add(i)
    return adj


def find_triangles(points):
    threshold = find_neighbor_threshold(points)
    adj = build_adjacency(points, threshold)
    ids = list(points.keys())
    triangles = []
    for i, j, k in itertools.combinations(ids, 3):
        if j in adj[i] and k in adj[i] and k in adj[j]:
            triangles.append(tuple(sorted((i, j, k))))
    return sorted(set(triangles))


def main():
    actual = load_actual_points(CALIB_SOURCE_FILE)
    threshold = find_neighbor_threshold(actual)
    print(f"[auto] Detected neighbor threshold: {threshold:.4f} mm\n")

    triangles = find_triangles(actual)
    centroids = {}
    for (a, b, c) in triangles:
        xa, ya = actual[a]
        xb, yb = actual[b]
        xc, yc = actual[c]
        hw_a = ORIGINAL_TO_HARDWARE_ID[a]
        hw_b = ORIGINAL_TO_HARDWARE_ID[b]
        hw_c = ORIGINAL_TO_HARDWARE_ID[c]
        label = f"T{hw_a}_{hw_b}_{hw_c}"
        centroids[label] = {
            "vertices": [hw_a, hw_b, hw_c],
            "x_mm": round((xa + xb + xc) / 3.0, 4),
            "y_mm": round((ya + yb + yc) / 3.0, 4),
        }

    print(f"{'Label':<10} {'Vertices':<12} {'x_mm':>8} {'y_mm':>8}")
    print("-" * 42)
    for label, d in centroids.items():
        v = d["vertices"]
        vstr = f"{v[0]}-{v[1]}-{v[2]}"
        print(f"{label:<10} {vstr:<12} {d['x_mm']:>8.2f} {d['y_mm']:>8.2f}")

    print(f"\nTotal triangle-centroid points: {len(centroids)}  (expected 24)")

    with open(OUT_PATH, "w") as f:
        json.dump(centroids, f, indent=2)
    print(f"Saved -> {OUT_PATH}")


if __name__ == "__main__":
    main()