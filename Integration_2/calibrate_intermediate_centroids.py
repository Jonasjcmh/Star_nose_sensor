

import json
import itertools

OUT_PATH = "/home/divuthejo/Star_nose_sensor/Integration_2/triangle_centroids.json"

# Physical sensor numbering (verified against the physical chip photo —
# NOT simple top-to-bottom row order; numbering runs diagonally).
POINTS = {
     1: (  -8.0,  -14.0),   2: ( -12.0,   -7.0),   3: ( -16.0,   +0.0),
     4: (  +0.0,  -14.0),   5: (  -4.0,   -7.0),   6: (  -8.0,   +0.0),
     7: ( -12.0,   +7.0),   8: (  +8.0,  -14.0),   9: (  +4.0,   -7.0),
    10: (  +0.0,   +0.0),  11: (  -4.0,   +7.0),  12: (  -8.0,  +14.0),
    13: ( +12.0,   -7.0),  14: (  +8.0,   +0.0),  15: (  +4.0,   +7.0),
    16: (  +0.0,  +14.0),  17: ( +16.0,   +0.0),  18: ( +12.0,   +7.0),
    19: (  +8.0,  +14.0),
}


def _dist(a, b):
    (x1, y1), (x2, y2) = a, b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def find_neighbor_threshold(points, gap_factor=1.3):
    """
    Auto-detect the max distance that counts as 'lattice neighbors'.

    Sorts all unique pairwise distances and walks up the list looking
    for the first place where the NEXT distance is at least gap_factor
    times bigger than the current one (a "jump" out of the neighbor
    tier into the next-nearest tier). The threshold returned is the
    midpoint of that jump, so it works regardless of the grid's actual
    spacing in mm.

    gap_factor=1.3 means: any relative jump of 30%+ is treated as
    "this is where real neighbors end." For a healthy triangular
    lattice, the real neighbor distances cluster tightly together
    (within a couple %), then the next-nearest tier is 60-70%+
    farther away, so 1.3 leaves comfortable margin either side.
    """
    ids = list(points.keys())
    dists = sorted(set(round(_dist(points[a], points[b]), 6)
                        for a, b in itertools.combinations(ids, 2)))
    for i in range(len(dists) - 1):
        if dists[i + 1] / dists[i] > gap_factor:
            return (dists[i] + dists[i + 1]) / 2.0
    return dists[-1]   # fallback: no clear gap, treat everything as neighbors


def build_adjacency(points, threshold):
    """Two points are neighbors if within threshold of each other."""
    ids = list(points.keys())
    adj = {i: set() for i in ids}
    for i, j in itertools.combinations(ids, 2):
        if _dist(points[i], points[j]) <= threshold:
            adj[i].add(j)
            adj[j].add(i)
    return adj


def find_triangles(points):
    """Return all 3-cliques (mutually-adjacent triples) as sorted tuples."""
    threshold = find_neighbor_threshold(points)
    adj = build_adjacency(points, threshold)
    ids = list(points.keys())
    triangles = []
    for i, j, k in itertools.combinations(ids, 3):
        if j in adj[i] and k in adj[i] and k in adj[j]:
            triangles.append(tuple(sorted((i, j, k))))
    return sorted(set(triangles))


def triangle_centroids(points):
    """Returns dict keyed by label 'T<a>_<b>_<c>' -> centroid + vertex info."""
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
    threshold = find_neighbor_threshold(POINTS)
    print(f"[auto] Detected neighbor threshold: {threshold:.4f} mm\n")

    centroids = triangle_centroids(POINTS)

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