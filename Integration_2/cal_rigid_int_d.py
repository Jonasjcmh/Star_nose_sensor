"""
caliberate_diagonal_midpoints_rigid.py
Computes diagonal edge-midpoints using the RIGID-TRANSFORMED points.

Same topology-vs-coordinates split as
caliberate_intermediate_points_rigid.py: diagonal-neighbor ID pairs
are determined from the ORIGINAL undistorted grid (where the
auto-detected neighbor threshold and same-y/different-y distinction
work correctly), then midpoints are computed using the rigid-
transformed coordinates for those same ID pairs.
────────────────────────────────────────────────────────────────────
"""

import json
import os
import itertools

BASE_DIR = "/home/cao/Documents/Star_muse_sensor/Star_nose_sensor/Integration_2"
INPUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_rigid_transformed_new_hollow_2.json")
OUT_PATH   = os.path.join(BASE_DIR, "diagonal_midpoints_rigid_new_hollow_2.json")

# Original (untransformed) nominal points — used ONLY to determine
# topology (which IDs are diagonal neighbors), never for coordinates.
POINTS = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}


def load_transformed_points(path):
    with open(path) as f:
        data = json.load(f)
    return {int(pid): (d["x_mm"], d["y_mm"]) for pid, d in data["points"].items()}


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


def find_diagonal_pairs(points):
    """Diagonal-neighbor ID pairs, determined from the ORIGINAL grid."""
    threshold = find_neighbor_threshold(points)
    ids = list(points.keys())
    pairs = []
    for i, j in itertools.combinations(ids, 2):
        xi, yi = points[i]
        xj, yj = points[j]
        if yi == yj:
            continue   # same row -> horizontal, not this script's job
        if _dist(points[i], points[j]) <= threshold:
            pairs.append(tuple(sorted((i, j))))
    return sorted(set(pairs))


def main():
    transformed = load_transformed_points(INPUT_JSON)
    pairs = find_diagonal_pairs(POINTS)

    mids = {}
    for id_a, id_b in pairs:
        xa, ya = transformed[id_a]
        xb, yb = transformed[id_b]
        label = f"D{id_a}_{id_b}"
        mids[label] = {
            "between": [id_a, id_b],
            "x_mm": round((xa + xb) / 2.0, 4),
            "y_mm": round((ya + yb) / 2.0, 4),
        }

    print(f"{'Label':<8} {'Between':<10} {'x_mm':>8} {'y_mm':>8}")
    print("-" * 38)
    for label, d in mids.items():
        between_str = f"P{d['between'][0]}-P{d['between'][1]}"
        print(f"{label:<8} {between_str:<10} {d['x_mm']:>8.2f} {d['y_mm']:>8.2f}")

    print(f"\nTotal diagonal intermediate points: {len(mids)}  (expected 28)")

    with open(OUT_PATH, "w") as f:
        json.dump(mids, f, indent=2)
    print(f"Saved -> {OUT_PATH}")


if __name__ == "__main__":
    main()