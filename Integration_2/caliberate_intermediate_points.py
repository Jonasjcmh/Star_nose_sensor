"""
Given the 19 points of the star-nose sensor, compute the 14 horizontal intermediate points that lie between each pair of adjacent points in the same row.
The 19 points are arranged in 5 rows (y-coordinates):
     y=+14 (top row, 3 pts)   : 12, 16, 19
    y=+7  (4 pts)            : 7, 11, 15, 18
    y=0   (middle row, 5 pts): 3, 6, 10, 14, 17
    y=-7  (4 pts)            : 2, 5, 9, 13
    y=-14 (bottom row, 3 pts): 1, 4, 8
 
The horizontal intermediate points are the midpoints of the line segments connecting each pair of adjacent points in

    x_mid = (x1 + x2) / 2
    y_mid = y                      (since y1 == y2 in a row)

This is just the standard 2D midpoint formula
    M = ((x1+x2)/2, (y1+y2)/2)
specialized to a horizontal segment.

GROUPING LOGIC
1. Group the 19 points by their y-coordinate -> gives you the 5 rows.
2. Sort each row left-to-right by x.
3. Walk consecutive pairs in each row and take the midpoint of each.
   (NOT every pair — only immediate neighbors, since that's what
   "the line joining the two [adjacent] points" means physically.)

Row-by-row point counts -> midpoint counts (14 total, unchanged by
the renumbering — grouping is still by row/y, only the ID labels
attached to each point changed):
    y=+14: 3 points      -> 2 midpoints
    y=+7 : 4 points      -> 3 midpoints
    y= 0 : 5 points      -> 4 midpoints
    y=-7 : 4 points      -> 3 midpoints
    y=-14: 3 points      -> 2 midpoints
                             -----
                             14
────────────────────────────────────────────────────────────────────
"""

import json
 
OUT_PATH = "/home/divuthejo/Star_nose_sensor/Integration_2/horizontal_midpoints.json"
 
POINTS = {
     1: (  -8.0,  -14.0),   2: ( -12.0,   -7.0),   3: ( -16.0,   +0.0),
     4: (  +0.0,  -14.0),   5: (  -4.0,   -7.0),   6: (  -8.0,   +0.0),
     7: ( -12.0,   +7.0),   8: (  +8.0,  -14.0),   9: (  +4.0,   -7.0),
    10: (  +0.0,   +0.0),  11: (  -4.0,   +7.0),  12: (  -8.0,  +14.0),
    13: ( +12.0,   -7.0),  14: (  +8.0,   +0.0),  15: (  +4.0,   +7.0),
    16: (  +0.0,  +14.0),  17: ( +16.0,   +0.0),  18: ( +12.0,   +7.0),
    19: (  +8.0,  +14.0),
}
 
 
def group_rows(points):
    """Group points by y-coordinate (row), sorted left-to-right by x."""
    rows = {}
    for pt_id, (x, y) in points.items():
        rows.setdefault(y, []).append((pt_id, x))
    for y in rows:
        rows[y].sort(key=lambda t: t[1])   # sort by x within row
    # return rows top->bottom (largest y first)
    return {y: rows[y] for y in sorted(rows.keys(), reverse=True)}
 
 
def horizontal_midpoints(points):
    """
    Returns a dict keyed by a new label (e.g. 'H1_2') -> (x_mid, y_mid),
    plus metadata about which two original points it sits between.
    """
    rows = group_rows(points)
    midpoints = {}
    for y, row in rows.items():
        for (id_a, x_a), (id_b, x_b) in zip(row, row[1:]):
            label = f"H{id_a}_{id_b}"          # e.g. H1_2 = midpoint between P1 and P2
            x_mid = (x_a + x_b) / 2.0
            y_mid = y                           # same row -> y unchanged
            midpoints[label] = {
                "between": [id_a, id_b],
                "x_mm": round(x_mid, 4),
                "y_mm": round(y_mid, 4),
            }
    return midpoints
 
 
def main():
    mids = horizontal_midpoints(POINTS)
 
    print(f"{'Label':<8} {'Between':<10} {'x_mm':>8} {'y_mm':>8}")
    print("-" * 38)
    for label, d in mids.items():
        between_str = f"P{d['between'][0]}-P{d['between'][1]}"
        print(f"{label:<8} {between_str:<10} {d['x_mm']:>8.2f} {d['y_mm']:>8.2f}")
 
    print(f"\nTotal horizontal intermediate points: {len(mids)}")
 
    with open(OUT_PATH, "w") as f:
        json.dump(mids, f, indent=2)
    print(f"Saved -> {OUT_PATH}")
 
 
if __name__ == "__main__":
    main()
 