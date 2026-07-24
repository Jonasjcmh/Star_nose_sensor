"""
cal_actual_int_c.py
Computes triangle centroids using the ACTUAL (robot-calibrated) points —
NO rigid transformation applied.

Untransformed counterpart of cal_rigid_int_c.py. Triangle triples are found from
the ORIGINAL undistorted grid (where adjacency detection is reliable), then
centroids are computed from the ACTUAL calibrated coordinates:

    actual[pid] = nominal[pid] + global offset + per-point (dx, dy)

Usage:
  python3 cal_actual_int_c.py
  python3 cal_actual_int_c.py --input calib_points_short_<tip>.json --output triangle_centroids_actual_<tip>.json
"""

import argparse
import json
import os
import itertools

BASE_DIR = "/home/cao/Documents/Star_muse_sensor/Star_nose_sensor/Integration_2"
DEFAULT_INPUT  = os.path.join(BASE_DIR, "calib_points_short_new_hollow_2.json")
DEFAULT_OUTPUT = os.path.join(BASE_DIR, "triangle_centroids_actual_new_hollow_2.json")

# Original (untransformed) nominal points — used to determine topology (which
# triples form triangles) and as the base for actual coordinates.
POINTS = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}


def load_actual_points(path):
    """actual[pid] = nominal[pid] + global offset + per-point offset (with
    fallbacks to points[pid].offset_mm / x_mm,y_mm)."""
    with open(path) as f:
        data = json.load(f)
    g = data.get("global", {})
    gx, gy = g.get("x_mm", 0.0), g.get("y_mm", 0.0)
    coords = {}
    per_point = data.get("per_point")
    points    = data.get("points")
    if per_point:
        for key, off in per_point.items():
            pid = int(key)
            if pid not in POINTS:
                continue
            nx, ny = POINTS[pid]
            coords[pid] = (nx + gx + off.get("dx_mm", 0.0),
                           ny + gy + off.get("dy_mm", 0.0))
    elif points:
        for key, v in points.items():
            pid = int(key)
            if "offset_mm" in v:
                coords[pid] = tuple(v["offset_mm"])
            elif "x_mm" in v:
                coords[pid] = (v["x_mm"], v["y_mm"])
    return coords


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
    ap = argparse.ArgumentParser(description="Triangle centroids for ACTUAL (untransformed) points")
    ap.add_argument("--input",  default=DEFAULT_INPUT,  help="actual calib_points_*.json")
    ap.add_argument("--output", default=DEFAULT_OUTPUT, help="output json path")
    args = ap.parse_args()

    actual    = load_actual_points(args.input)
    triangles = find_triangles(POINTS)

    centroids = {}
    for (a, b, c) in triangles:
        (xa, ya) = actual[a]
        (xb, yb) = actual[b]
        (xc, yc) = actual[c]
        label = f"T{a}_{b}_{c}"
        centroids[label] = {
            "vertices": [a, b, c],
            "x_mm": round((xa + xb + xc) / 3.0, 4),
            "y_mm": round((ya + yb + yc) / 3.0, 4),
        }

    print(f"Source (actual points): {os.path.basename(args.input)}")
    print(f"{'Label':<10} {'Vertices':<12} {'x_mm':>8} {'y_mm':>8}")
    print("-" * 42)
    for label, d in centroids.items():
        v = d["vertices"]
        vstr = f"{v[0]}-{v[1]}-{v[2]}"
        print(f"{label:<10} {vstr:<12} {d['x_mm']:>8.2f} {d['y_mm']:>8.2f}")

    print(f"\nTotal triangle-centroid points: {len(centroids)}  (expected 24)")

    with open(args.output, "w") as f:
        json.dump(centroids, f, indent=2)
    print(f"Saved -> {args.output}")


if __name__ == "__main__":
    main()
