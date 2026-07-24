"""
caliberate_diagonal_midpoints_actual.py
Computes diagonal edge-midpoints using PURE GEOMETRY on the ACTUAL
(measured, robot-calibrated) point positions -- no reference to the
theoretical grid's topology at all.

Same neighbor-detection + angle-clustering approach as
caliberate_horizontal_midpoints_actual.py (see that file's docstring
for the full explanation). The 42 real lattice edges split into 3
angular clusters, 60 degrees apart; the one with exactly 14 edges is
horizontal (handled by the other script), and the remaining TWO
clusters (14 + 14 = 28) are the diagonal edges, handled here.

CALCULATIONS use the calibration file's own (ORIGINAL) point
numbering. LABELS are converted to HARDWARE numbering for human
readability -- see caliberate_horizontal_midpoints_actual.py's
docstring for details. This conversion never touches coordinates or
the point-finding math.

Output: diagonal_midpoints_actual_<TAG>.json
────────────────────────────────────────────────────────────────────
"""

import json
import os
import itertools
import math

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"

CALIB_SOURCE_FILE = os.path.join(BASE_DIR, "calib_points_short_new_hollow_2.json")
TAG = "new_hollow_2"

OUT_PATH = os.path.join(BASE_DIR, f"diagonal_midpoints_actual_{TAG}.json")

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


def find_all_neighbor_edges(points):
    threshold = find_neighbor_threshold(points)
    ids = list(points.keys())
    edges = []
    for i, j in itertools.combinations(ids, 2):
        if _dist(points[i], points[j]) <= threshold:
            edges.append(tuple(sorted((i, j))))
    return sorted(set(edges))


def edge_angle_deg(points, edge):
    i, j = edge
    (x1, y1), (x2, y2) = points[i], points[j]
    return math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180


def cluster_edges_by_angle(points, edges, n_clusters=3):
    angled = sorted(((edge_angle_deg(points, e), e) for e in edges), key=lambda t: t[0])
    angles_only = [a for a, e in angled]
    gaps = [(angles_only[i + 1] - angles_only[i], i) for i in range(len(angles_only) - 1)]
    gaps.sort(key=lambda t: -t[0])
    split_indices = sorted(i for _, i in gaps[:n_clusters - 1])
    clusters = []
    start = 0
    for idx in split_indices:
        clusters.append([e for a, e in angled[start:idx + 1]])
        start = idx + 1
    clusters.append([e for a, e in angled[start:]])
    return clusters


def find_diagonal_edges(points):
    """The two non-14-edge... actually: the two clusters that AREN'T
    the horizontal (14-edge) one. Their union is the 28 diagonal edges."""
    all_edges = find_all_neighbor_edges(points)
    clusters = cluster_edges_by_angle(points, all_edges, n_clusters=3)
    diagonal = []
    horizontal_found = False
    for c in clusters:
        if len(c) == 14 and not horizontal_found:
            horizontal_found = True   # skip exactly one 14-edge cluster (horizontal)
            continue
        diagonal.extend(c)
    if not horizontal_found:
        raise ValueError(
            f"Expected one 14-edge (horizontal) cluster, got sizes "
            f"{[len(c) for c in clusters]}"
        )
    return diagonal


def main():
    actual = load_actual_points(CALIB_SOURCE_FILE)
    diag_edges = find_diagonal_edges(actual)

    mids = {}
    for (a, b) in diag_edges:
        xa, ya = actual[a]
        xb, yb = actual[b]
        hw_a = ORIGINAL_TO_HARDWARE_ID[a]
        hw_b = ORIGINAL_TO_HARDWARE_ID[b]
        label = f"D{hw_a}_{hw_b}"
        mids[label] = {
            "between": [hw_a, hw_b],
            "x_mm": round((xa + xb) / 2.0, 4),
            "y_mm": round((ya + yb) / 2.0, 4),
        }

    print(f"{'Label':<8} {'Between':<10} {'x_mm':>8} {'y_mm':>8}")
    print("-" * 38)
    for label, d in mids.items():
        between_str = f"{d['between'][0]}-{d['between'][1]}"
        print(f"{label:<8} {between_str:<10} {d['x_mm']:>8.2f} {d['y_mm']:>8.2f}")

    print(f"\nTotal diagonal intermediate points: {len(mids)}  (expected 28)")

    with open(OUT_PATH, "w") as f:
        json.dump(mids, f, indent=2)
    print(f"Saved -> {OUT_PATH}")


if __name__ == "__main__":
    main()