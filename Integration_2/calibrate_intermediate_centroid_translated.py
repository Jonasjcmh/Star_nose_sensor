"""
caliberate_triangle_centroids_translated.py
Computes triangle centroids for the 19-point sensor grid, using the
TRANSLATED "supposed to be" points (from translate_supposed_points.py
output) instead of the raw nominal ones.

Same logic as caliberate_triangle_centroids.py — neighbor threshold
auto-detected from geometry, triangles found via 3-clique detection
on the adjacency graph — but reads points from
calib_points_supposed_translated.json instead of a hardcoded dict.

Uses the ORIGINAL arbitrary P1-P19 numbering (matches
calib_points_short_6mm.json), same as the other two "_translated"
scripts in this set.

Translation is a rigid shift, so it doesn't change relative spacing
between points — the triangle count (24) is identical to the
untranslated version.
────────────────────────────────────────────────────────────────────
"""

import json
import os
import itertools

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"
INPUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_translated.json")
OUT_PATH   = os.path.join(BASE_DIR, "triangle_centroids_translated.json")


def load_points(path):
    with open(path) as f:
        data = json.load(f)
    return {int(pid): (d["x_mm"], d["y_mm"]) for pid, d in data["points"].items()}


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


def triangle_centroids(points):
    triangles = find_triangles(points)
    result = {}
    for (a, b, c) in triangles:
        (xa, ya), (xb, yb), (xc, yc) = points[a], points[b], points[c]
        x_c = (xa + xb + xc) / 3.0
        y_c = (ya + yb + yc) / 3.0
        label = f"T{a}_{b}_{c}"
        result[label] = {
            "vertices": [a, b, c],
            "x_mm": round(x_c, 4),
            "y_mm": round(y_c, 4),
        }
    return result


def main():
    points = load_points(INPUT_JSON)
    threshold = find_neighbor_threshold(points)
    print(f"[auto] Detected neighbor threshold: {threshold:.4f} mm\n")

    centroids = triangle_centroids(points)

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