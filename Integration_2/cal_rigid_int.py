"""
caliberate_intermediate_points_rigid.py
Computes horizontal edge-midpoints using the RIGID-TRANSFORMED points
(rotation + translation fit via fit_rigid_transform_supposed_points.py).

────────────────────────────────────────────────────────────────────
KEY IDEA: topology vs. coordinates are handled separately.

Rotation tilts the whole grid, so transformed points no longer share
exact (or even close) y-values within what used to be a "row" — you
can't re-detect rows by grouping y-coordinates in the transformed
data. But a RIGID transform (rotation + translation) never changes
WHICH points are neighbors of which — it only moves their
coordinates. So:

    1. Determine "which point IDs are horizontal neighbors" using the
       ORIGINAL, undistorted nominal grid (POINTS below) — same row
       grouping as the very first caliberate_intermediate_points.py.
    2. Once we know the ID pairs (e.g. "1 and 2 are horizontal
       neighbors"), look up THEIR coordinates in the rigid-transformed
       data and take the midpoint there.

This gives the same 14 H<a>_<b> labels as always, just positioned
using the rotated+translated coordinates.
────────────────────────────────────────────────────────────────────
"""

import json
import os

BASE_DIR = "/home/cao/Documents/Star_muse_sensor/Star_nose_sensor/Integration_2"
INPUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_rigid_transformed_new_hollow_2.json")
OUT_PATH   = os.path.join(BASE_DIR, "horizontal_midpoints_rigid_new_hollow_2.json")

# Original (untransformed) nominal points — used ONLY to determine
# topology (which IDs are horizontal neighbors), never for coordinates.
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


def find_horizontal_pairs(points):
    """Determine horizontal-neighbor ID pairs from the ORIGINAL grid."""
    rows = {}
    for pt_id, (x, y) in points.items():
        rows.setdefault(y, []).append((pt_id, x))
    for y in rows:
        rows[y].sort(key=lambda t: t[1])
    pairs = []
    for y in rows:
        row = rows[y]
        for (id_a, _), (id_b, _) in zip(row, row[1:]):
            pairs.append((id_a, id_b))
    return pairs


def main():
    transformed = load_transformed_points(INPUT_JSON)
    pairs = find_horizontal_pairs(POINTS)

    mids = {}
    for id_a, id_b in pairs:
        xa, ya = transformed[id_a]
        xb, yb = transformed[id_b]
        label = f"H{id_a}_{id_b}"
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

    print(f"\nTotal horizontal intermediate points: {len(mids)}  (expected 14)")

    with open(OUT_PATH, "w") as f:
        json.dump(mids, f, indent=2)
    print(f"Saved -> {OUT_PATH}")


if __name__ == "__main__":
    main()