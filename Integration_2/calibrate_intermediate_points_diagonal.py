

import json
import itertools

OUT_PATH = "/home/divuthejo/Star_nose_sensor/Integration_2/diagonal_midpoints.json"

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
    Same logic as in caliberate_triangle_centroids.py — sorts all
    unique pairwise distances and finds the first big relative jump,
    returning the midpoint of that jump as the threshold. Works
    regardless of the grid's actual mm spacing.
    """
    ids = list(points.keys())
    dists = sorted(set(round(_dist(points[a], points[b]), 6)
                        for a, b in itertools.combinations(ids, 2)))
    for i in range(len(dists) - 1):
        if dists[i + 1] / dists[i] > gap_factor:
            return (dists[i] + dists[i + 1]) / 2.0
    return dists[-1]   # fallback: no clear gap, treat everything as neighbors


def find_diagonal_edges(points):
    """
    Return sorted (a, b) pairs that are diagonal (row-to-row) neighbors:
    within the auto-detected neighbor threshold, AND on different rows
    (different y). Same-row pairs are horizontal edges, handled by
    caliberate_intermediate_points.py instead.
    """
    threshold = find_neighbor_threshold(points)
    ids = list(points.keys())
    edges = []
    for i, j in itertools.combinations(ids, 2):
        xi, yi = points[i]
        xj, yj = points[j]
        if yi == yj:
            continue   # same row -> horizontal edge, not this script's job
        if _dist(points[i], points[j]) <= threshold:
            edges.append(tuple(sorted((i, j))))
    return sorted(set(edges))


def diagonal_midpoints(points):
    """Returns dict keyed by label 'D<a>_<b>' -> midpoint + vertex info."""
    edges = find_diagonal_edges(points)
    result = {}
    for (a, b) in edges:
        (xa, ya), (xb, yb) = points[a], points[b]
        x_mid = (xa + xb) / 2.0
        y_mid = (ya + yb) / 2.0
        label = f"D{a}_{b}"
        result[label] = {
            "between": [a, b],
            "x_mm": round(x_mid, 4),
            "y_mm": round(y_mid, 4),
        }
    return result


def main():
    threshold = find_neighbor_threshold(POINTS)
    print(f"[auto] Detected neighbor threshold: {threshold:.4f} mm\n")

    mids = diagonal_midpoints(POINTS)

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