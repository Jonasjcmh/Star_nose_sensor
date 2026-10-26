"""
caliberate_intermediate_points_translated.py
Computes horizontal edge-midpoints for the 19-point sensor grid,
using the TRANSLATED "supposed to be" points instead of the raw
nominal ones.

────────────────────────────────────────────────────────────────────
WHY THIS VERSION
translate_supposed_points.py aligned all 19 nominal points to the
actual robot-calibrated position of point 10 (rigid translation,
same offset applied to every point). This script picks up that
translated result and re-derives the horizontal edge-midpoints from
it — same geometry/logic as caliberate_intermediate_points.py, just
reading a different input source instead of a hardcoded POINTS dict.

Uses the ORIGINAL arbitrary P1-P19 numbering (matches
calib_points_short_6mm.json and translate_supposed_points.py) —
NOT the diagonal/physical-chip numbering used elsewhere. This
numbering is required here because it's the one the robot actually
used when the mocap file was recorded.

WHY MIDPOINT (unchanged from the original script)
    x_mid = (x1 + x2) / 2
    y_mid = (y1 + y2) / 2
(y1 == y2 within a row, so this reduces to averaging x.)

GROUPING LOGIC (unchanged)
1. Group points by y-coordinate -> rows.
2. Sort each row left-to-right by x.
3. Take the midpoint of each consecutive pair within a row.

Translation doesn't change relative spacing between points (it's a
rigid shift), so the row structure and midpoint count (14) are
identical to the untranslated version — only the absolute
coordinates shift by the offset.
────────────────────────────────────────────────────────────────────
"""

import json
import os

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"
INPUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_translated.json")
OUT_PATH   = os.path.join(BASE_DIR, "horizontal_midpoints_translated.json")


def load_points(path):
    """Load the translated points from calib_points_supposed_translated.json."""
    with open(path) as f:
        data = json.load(f)
    return {int(pid): (d["x_mm"], d["y_mm"]) for pid, d in data["points"].items()}


def group_rows(points):
    """Group points by y-coordinate (row), sorted left-to-right by x."""
    rows = {}
    for pt_id, (x, y) in points.items():
        rows.setdefault(y, []).append((pt_id, x))
    for y in rows:
        rows[y].sort(key=lambda t: t[1])
    return {y: rows[y] for y in sorted(rows.keys(), reverse=True)}


def horizontal_midpoints(points):
    """Returns dict keyed by 'H<a>_<b>' -> midpoint + vertex info."""
    rows = group_rows(points)
    midpoints = {}
    for y, row in rows.items():
        for (id_a, x_a), (id_b, x_b) in zip(row, row[1:]):
            label = f"H{id_a}_{id_b}"
            x_mid = (x_a + x_b) / 2.0
            y_mid = y
            midpoints[label] = {
                "between": [id_a, id_b],
                "x_mm": round(x_mid, 4),
                "y_mm": round(y_mid, 4),
            }
    return midpoints


def main():
    points = load_points(INPUT_JSON)
    mids = horizontal_midpoints(points)

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