"""
caliberate_horizontal_midpoints_actual.py
Computes horizontal edge-midpoints using PURE GEOMETRY on the ACTUAL
(measured, robot-calibrated) point positions -- no reference to the
theoretical grid's topology at all.

────────────────────────────────────────────────────────────────────
HOW THIS WORKS (fully derived from the actual point cloud alone)

1. Auto-detect the neighbor distance threshold, same technique as
   before: sort all pairwise distances, find the first big relative
   jump, use the midpoint of that jump as the cutoff. This correctly
   finds all 42 "real lattice edges" even though individual distances
   now range ~7.7-8.2mm (real deviations blur the old clean 8.0 vs
   8.062mm split) -- the GAP to the next-nearest tier (~13mm+) is
   still huge, so neighbor detection itself stays robust.

2. Classify each of the 42 neighbor edges into "horizontal" vs
   "diagonal" by EDGE ANGLE, not distance. In a hex grid there are
   3 natural edge directions, 60 degrees apart. Even with mm-level
   position noise and an overall few-degree rotation, edges cluster
   TIGHTLY into 3 angular groups with ~60 degree gaps between them --
   far more robust than the <0.1mm distance difference between
   horizontal/diagonal edges, which real noise washes out completely.
   The cluster with exactly 14 edges is "horizontal"; the other two
   clusters (14 each) are "diagonal", handled by
   caliberate_diagonal_midpoints_actual.py.

3. Take the midpoint of each identified horizontal edge.

CALCULATIONS use the calibration file's own point numbering (matches
calibrate_points.py's ORIGINAL_POINTS -- required for the ACTUAL
positions to be interpreted correctly). LABELS in the output JSON are
converted to HARDWARE/physical-chip numbering (matches what's printed
on the sensor), via ORIGINAL_TO_HARDWARE_ID below, purely for human
readability -- this conversion happens ONLY on the label text, never
on the coordinates or the point-finding math.

OUTPUT NAMING: matches main.py's auto-discovery convention -- see
_variant_from_calib(): calib_points_short_<TAG>.json -> looks for
files named *_actual_<TAG>.json.

Output: horizontal_midpoints_actual_<TAG>.json
────────────────────────────────────────────────────────────────────
"""

import json
import os
import itertools
import math

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"

# EDIT THIS if your calibration file isn't named calib_points_new.json:
CALIB_SOURCE_FILE = os.path.join(BASE_DIR, "calib_points_short_new_hollow_2.json")
TAG = "new_hollow_2"   # EDIT to match your calibration filename's tag

OUT_PATH = os.path.join(BASE_DIR, f"horizontal_midpoints_actual_{TAG}.json")

# ── Label conversion ONLY (never used for coordinates/math) ──────────────────
# ORIGINAL numbering (matches calibrate_points.py -- what the calib file's
# point keys 1-19 actually mean) vs HARDWARE numbering (matches the chip photo).
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
    """{point_id (int): (actual_x_mm, actual_y_mm)}"""
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
    """Auto-detect the max distance that counts as 'lattice neighbors'."""
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
    """Edge angle folded into [0, 180) -- direction, not sense."""
    i, j = edge
    (x1, y1), (x2, y2) = points[i], points[j]
    return math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180


def cluster_edges_by_angle(points, edges, n_clusters=3):
    """
    Cluster edges into n_clusters groups by angle, using the largest
    gaps in the sorted angle list (handles up to n_clusters-1 gaps).
    Returns list of edge-lists, one per cluster.
    """
    angled = sorted(((edge_angle_deg(points, e), e) for e in edges), key=lambda t: t[0])
    angles_only = [a for a, e in angled]

    # find the (n_clusters - 1) largest gaps between consecutive angles
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


def find_horizontal_edges(points):
    """
    The 'horizontal' cluster is the one with exactly 14 edges (matches
    the 19-point hex grid's known row structure: 2+3+4+3+2 = 14).
    """
    all_edges = find_all_neighbor_edges(points)
    clusters = cluster_edges_by_angle(points, all_edges, n_clusters=3)
    horizontal = next((c for c in clusters if len(c) == 14), None)
    if horizontal is None:
        raise ValueError(
            f"Expected a 14-edge cluster, got cluster sizes {[len(c) for c in clusters]} "
            "-- check the data or n_clusters"
        )
    return horizontal


def main():
    actual = load_actual_points(CALIB_SOURCE_FILE)
    horiz_edges = find_horizontal_edges(actual)

    mids = {}
    for (a, b) in horiz_edges:
        xa, ya = actual[a]
        xb, yb = actual[b]
        hw_a = ORIGINAL_TO_HARDWARE_ID[a]
        hw_b = ORIGINAL_TO_HARDWARE_ID[b]
        label = f"H{hw_a}_{hw_b}"
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

    print(f"\nTotal horizontal intermediate points: {len(mids)}  (expected 14)")

    with open(OUT_PATH, "w") as f:
        json.dump(mids, f, indent=2)
    print(f"Saved -> {OUT_PATH}")


if __name__ == "__main__":
    main()