"""
caliberate_diagonal_midpoints_translated.py
Computes diagonal edge-midpoints for the 19-point sensor grid,
using the TRANSLATED "supposed to be" points (from
translate_supposed_points.py output) instead of the raw nominal ones.

Same logic as caliberate_diagonal_midpoints.py — neighbor threshold
is auto-detected from geometry (not hardcoded), and horizontal vs.
diagonal edges are told apart by same-y vs. different-y (not by
comparing specific mm distances) — but reads points from
calib_points_supposed_translated.json instead of a hardcoded dict.

Uses the ORIGINAL arbitrary P1-P19 numbering (matches
calib_points_short_6mm.json), same as
caliberate_intermediate_points_translated.py.

Translation is a rigid shift, so it doesn't change relative spacing
between points — the diagonal-edge count (28) is identical to the
untranslated version.
────────────────────────────────────────────────────────────────────
"""

import json
import os
import itertools

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"
INPUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_translated.json")
OUT_PATH   = os.path.join(BASE_DIR, "diagonal_midpoints_translated.json")


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


def find_diagonal_edges(points):
    """
    Diagonal (row-to-row) neighbor pairs: within the auto-detected
    threshold, AND on different rows (different y). Same-row pairs
    are horizontal edges, handled by
    caliberate_intermediate_points_translated.py instead.
    """
    threshold = find_neighbor_threshold(points)
    ids = list(points.keys())
    edges = []
    for i, j in itertools.combinations(ids, 2):
        xi, yi = points[i]
        xj, yj = points[j]
        if yi == yj:
            continue
        if _dist(points[i], points[j]) <= threshold:
            edges.append(tuple(sorted((i, j))))
    return sorted(set(edges))


def diagonal_midpoints(points):
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
    points = load_points(INPUT_JSON)
    threshold = find_neighbor_threshold(points)
    print(f"[auto] Detected neighbor threshold: {threshold:.4f} mm\n")

    mids = diagonal_midpoints(points)

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