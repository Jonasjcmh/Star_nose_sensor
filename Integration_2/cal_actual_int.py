"""
cal_actual_int.py
Computes horizontal edge-midpoints using the ACTUAL (robot-calibrated) points —
NO rigid transformation applied.

This is the untransformed counterpart of cal_rigid_int.py. The rigid version
reads coordinates from the rotation+translation fit; this one reads the actual
calibrated positions directly, computed the same way fit_rigid.py does:

    actual[pid] = nominal[pid] + global offset + per-point (dx, dy)

Topology (which point IDs are horizontal neighbors) is still determined from the
ORIGINAL undistorted nominal grid — that never changes — so the labels match
the rigid version exactly (H<a>_<b>), only the coordinates differ.

Usage:
  python3 cal_actual_int.py
  python3 cal_actual_int.py --input calib_points_short_<tip>.json --output horizontal_midpoints_actual_<tip>.json
"""

import argparse
import json
import os

BASE_DIR = "/home/cao/Documents/Star_muse_sensor/Star_nose_sensor/Integration_2"
DEFAULT_INPUT  = os.path.join(BASE_DIR, "calib_points_short_new_hollow_2.json")
DEFAULT_OUTPUT = os.path.join(BASE_DIR, "horizontal_midpoints_actual_new_hollow_2.json")

# Original (untransformed) nominal points — used ONLY to determine topology
# (which IDs are horizontal neighbors) and as the base for actual coordinates.
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
    """actual[pid] = nominal[pid] + global offset + per-point offset.

    Falls back to points[pid].offset_mm or points[pid].x_mm/y_mm if a file has
    no per_point block, so it also works with newer calibrate_points.py output.
    """
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
    ap = argparse.ArgumentParser(description="Horizontal midpoints for ACTUAL (untransformed) points")
    ap.add_argument("--input",  default=DEFAULT_INPUT,  help="actual calib_points_*.json")
    ap.add_argument("--output", default=DEFAULT_OUTPUT, help="output json path")
    args = ap.parse_args()

    actual = load_actual_points(args.input)
    pairs  = find_horizontal_pairs(POINTS)

    mids = {}
    for id_a, id_b in pairs:
        xa, ya = actual[id_a]
        xb, yb = actual[id_b]
        label = f"H{id_a}_{id_b}"
        mids[label] = {
            "between": [id_a, id_b],
            "x_mm": round((xa + xb) / 2.0, 4),
            "y_mm": round((ya + yb) / 2.0, 4),
        }

    print(f"Source (actual points): {os.path.basename(args.input)}")
    print(f"{'Label':<8} {'Between':<10} {'x_mm':>8} {'y_mm':>8}")
    print("-" * 38)
    for label, d in mids.items():
        between_str = f"P{d['between'][0]}-P{d['between'][1]}"
        print(f"{label:<8} {between_str:<10} {d['x_mm']:>8.2f} {d['y_mm']:>8.2f}")

    print(f"\nTotal horizontal intermediate points: {len(mids)}  (expected 14)")

    with open(args.output, "w") as f:
        json.dump(mids, f, indent=2)
    print(f"Saved -> {args.output}")


if __name__ == "__main__":
    main()
