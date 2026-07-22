"""
caliberate_triangle_centroids_rigid.py
Computes triangle centroids using the RIGID-TRANSFORMED points.

Same topology-vs-coordinates split as the other two "_rigid" scripts:
triangle triples are found from the ORIGINAL undistorted grid (where
adjacency detection is reliable), then centroids are computed using
the rigid-transformed coordinates for those same triples.
────────────────────────────────────────────────────────────────────
"""

import json
import os
import itertools

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"
INPUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_rigid_transformed.json")
OUT_PATH   = os.path.join(BASE_DIR, "triangle_centroids_rigid.json")

# Original (untransformed) nominal points — used ONLY to determine
# topology (which triples form triangles), never for coordinates.
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


def build_adjacency(points, threshold):
    ids = list(points.keys())
    adj = {i: set() for i in ids}
    for i, j in itertools.combinations(ids, 2):
        if _dist(points[i], points[j]) <= threshold:
            adj[i].add(j)
            adj[j].add(i)
    return adj


def find_triangles(points):
    """Triangle triples, determined from the ORIGINAL grid."""
    threshold = find_neighbor_threshold(points)
    adj = build_adjacency(points, threshold)
    ids = list(points.keys())
    triangles = []
    for i, j, k in itertools.combinations(ids, 3):
        if j in adj[i] and k in adj[i] and k in adj[j]:
            triangles.append(tuple(sorted((i, j, k))))
    return sorted(set(triangles))


def main():
    transformed = load_transformed_points(INPUT_JSON)
    triangles = find_triangles(POINTS)

    centroids = {}
    for (a, b, c) in triangles:
        (xa, ya) = transformed[a]
        (xb, yb) = transformed[b]
        (xc, yc) = transformed[c]
        label = f"T{a}_{b}_{c}"
        centroids[label] = {
            "vertices": [a, b, c],
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