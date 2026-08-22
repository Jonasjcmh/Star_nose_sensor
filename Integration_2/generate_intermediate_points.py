#!/usr/bin/env python3
"""
generate_intermediate_points.py
Single source of truth for the interdome "extra" pressing points — triangle
centroids, diagonal midpoints and horizontal midpoints — in the NEW a1..e5
label-order scheme (matches calibrate_points.py / calibrate_live.py /
visualizer_2d.py).

────────────────────────────────────────────────────────────────────────────
WHAT IT DOES
Given a calib_points_*.json produced by calibrate_points.py (the new full
format with points[N].offset_mm), it:

  1. Reads each main point's ACTUAL calibrated position — points[N].offset_mm,
     the full robot-frame XY offset from REFERENCE_POSE (already includes the
     global offset + the per-point calibration deviation).

  2. Builds the hex-lattice adjacency in the SENSOR / DISPLAY frame
     (calibrate_points.DISPLAY_XY), where the natural hex rows are the
     constant-y lines:

         disp y=+14 :  a3  b4  c5
         disp y=+7  :  a2  b3  c4  d5
         disp y= 0  :  a1  b2  c3  d4  e5     <- middle row
         disp y=-7  :  b1  c2  d3  e4
         disp y=-14 :  c1  d2  e3

     Classification MUST be done in this frame: the robot frame is a fixed
     rotation of it, so a sensor "row" is NOT a constant-y line in robot XY
     (that is exactly the bug in the old horizontal script, which grouped by
     robot-frame y).

  3. Emits three point sets, coordinates taken from the ACTUAL positions:
       • triangle_centroids : centroid of every lattice 3-clique (up/down tris)
       • diagonal_midpoints : midpoint of every cross-row neighbour edge
       • horizontal_midpoints: midpoint of every same-row neighbour edge

MATHS NOTE
The centroid / midpoint operators commute with any affine map. The sensor
frame and the robot frame are related by a fixed affine map, so building the
intermediate points from the ACTUAL robot-frame offsets is identical to
building them in the sensor frame and mapping across — there is no frame bias.
We classify neighbours on the clean THEORETICAL display grid (robust to
per-point calibration noise) but place the points from the ACTUAL offsets.

OUTPUT (matches Interdome_touch/main.py's variant convention)
    triangle_centroids_<tag>.json
    diagonal_midpoints_<tag>.json
    horizontal_midpoints_<tag>.json
where <tag> is derived from the calib file name the same way interdome's
_variant_from_calib does (e.g. calib_points_short_new_hollow_sensor.json ->
tag "actual_new_hollow_sensor"). Each file is a dict:

    { "<label>": { "vertices":[...], "vertex_pts":[...],
                   "x_mm":.., "y_mm":..,               # ROBOT frame (drive these)
                   "disp_x_mm":.., "disp_y_mm":.. } }  # sensor frame (for plots)

interdome reads x_mm / y_mm and the dict key (the label); the rest is metadata.
"""
import os
import sys
import json
import argparse
import itertools

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate_points as cp

CALIB_DIR     = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CALIB = "calib_points_short_new_hollow_sensor.json"

# (kind, output-file prefix) — same list/order as Interdome_touch/main.py EXTRA_SPECS
EXTRA_SPECS = [
    ("triangle",   "triangle_centroids"),
    ("diagonal",   "diagonal_midpoints"),
    ("horizontal", "horizontal_midpoints"),
]

LABEL  = cp.UR5_TO_LABEL      # pt -> 'a1'.. 'e5'
DISP   = cp.DISPLAY_XY        # pt -> (x,y) THEORETICAL sensor-frame coords
Y_TOL  = 1e-6                 # display-y equality tolerance (grid is exact)


def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


# ── Load the ACTUAL calibrated positions ──────────────────────────────────────
def load_actual(path):
    """pt -> (x_mm, y_mm) full robot-frame offset, from points[N].offset_mm."""
    with open(path) as f:
        d = json.load(f)
    pts = d.get("points") or {}
    if not pts or not all("offset_mm" in v for v in pts.values()):
        raise SystemExit(
            f"[error] {os.path.basename(path)} is not the new full format "
            f"(needs points[N].offset_mm from calibrate_points.py).")
    actual = {int(k): (float(v["offset_mm"][0]), float(v["offset_mm"][1]))
              for k, v in pts.items()}
    missing = set(range(1, 20)) - set(actual)
    if missing:
        raise SystemExit(f"[error] calib file missing points: {sorted(missing)}")
    return actual


# ── Lattice adjacency (built on the clean THEORETICAL display grid) ───────────
def neighbor_threshold(coords, gap_factor=1.3):
    """Largest distance that still counts as a lattice neighbour — the midpoint
    of the first >gap_factor jump in the sorted unique pairwise distances."""
    ids   = list(coords)
    dists = sorted(set(round(_dist(coords[a], coords[b]), 6)
                       for a, b in itertools.combinations(ids, 2)))
    for i in range(len(dists) - 1):
        if dists[i + 1] / dists[i] > gap_factor:
            return (dists[i] + dists[i + 1]) / 2.0
    return dists[-1]


def build_adjacency(coords, thr):
    adj = {i: set() for i in coords}
    for i, j in itertools.combinations(coords, 2):
        if _dist(coords[i], coords[j]) <= thr:
            adj[i].add(j)
            adj[j].add(i)
    return adj


def _same_row(a, b):
    return abs(DISP[a][1] - DISP[b][1]) <= Y_TOL


def _lbl_key(pt):
    return LABEL[pt]


def _tri_label(tri):
    return "T_" + "_".join(LABEL[p] for p in sorted(tri, key=_lbl_key))


def _edge_label(prefix, a, b):
    x, y = sorted((a, b), key=_lbl_key)
    return f"{prefix}_{LABEL[x]}_{LABEL[y]}"


def _centroid(actual, pts):
    xs = [actual[p][0] for p in pts]
    ys = [actual[p][1] for p in pts]
    return sum(xs) / len(xs), sum(ys) / len(ys)


# ── Build the three point sets ────────────────────────────────────────────────
def build_sets(actual):
    thr = neighbor_threshold(DISP)
    adj = build_adjacency(DISP, thr)

    # Triangles = mutually-adjacent triples (3-cliques).
    triangles = sorted({tuple(sorted((i, j, k)))
                        for i, j, k in itertools.combinations(adj, 3)
                        if j in adj[i] and k in adj[i] and k in adj[j]})

    # Edges, split by same-row (horizontal) vs cross-row (diagonal).
    horiz, diag = [], []
    for i, j in itertools.combinations(sorted(adj), 2):
        if j not in adj[i]:
            continue
        (horiz if _same_row(i, j) else diag).append((i, j))

    def _record(pts):
        rx, ry = _centroid(actual, pts)
        dx, dy = _centroid(DISP, pts)
        return {
            "vertices":   [LABEL[p] for p in sorted(pts, key=_lbl_key)],
            "vertex_pts": sorted(pts, key=_lbl_key),
            "x_mm":       round(rx, 4),
            "y_mm":       round(ry, 4),
            "disp_x_mm":  round(dx, 4),
            "disp_y_mm":  round(dy, 4),
        }

    tri_d = {_tri_label(t):          _record(t)      for t in triangles}
    # horizontals ordered top row -> bottom, left -> right (display frame)
    horiz_sorted = sorted(horiz, key=lambda e: (-DISP[e[0]][1],
                                                min(DISP[e[0]][0], DISP[e[1]][0])))
    dia_d  = {_edge_label("D", a, b): _record((a, b)) for a, b in diag}
    hor_d  = {_edge_label("H", a, b): _record((a, b)) for a, b in horiz_sorted}
    return {"triangle": tri_d, "diagonal": dia_d, "horizontal": hor_d}, thr


# ── Output naming — mirrors Interdome_touch/main.py _variant_from_calib ────────
def variant_tag(path):
    name = os.path.basename(path)
    low  = name.lower()
    if "rigid" in low:
        return "rigid"
    if "translated" in low:
        return "translated"
    stem = name[:-len(".json")] if name.endswith(".json") else name
    for pre in ("calib_points_short_", "calib_points_"):
        if stem.startswith(pre):
            tag = stem[len(pre):]
            return f"actual_{tag}" if tag else "actual"
    return None


def out_name(prefix, tag):
    return f"{prefix}_{tag}.json" if tag else f"{prefix}.json"


# ── Maths / sanity self-check ─────────────────────────────────────────────────
def self_check(sets, actual):
    ok = True
    # 1. counts
    n_t, n_d, n_h = len(sets["triangle"]), len(sets["diagonal"]), len(sets["horizontal"])
    print(f"  counts: {n_t} triangles, {n_d} diagonals, {n_h} horizontals")
    if n_h != 14:
        print(f"  ⚠ expected 14 horizontal midpoints, got {n_h}"); ok = False
    # 2. every intermediate point must be the exact mean of its ACTUAL vertices
    for kind, d in sets.items():
        for lbl, r in d.items():
            mx, my = _centroid(actual, r["vertex_pts"])
            if abs(mx - r["x_mm"]) > 5e-4 or abs(my - r["y_mm"]) > 5e-4:
                print(f"  ⚠ {lbl}: centroid mismatch"); ok = False
    # 3. horizontal midpoints must join same-row vertices (equal display y)
    for lbl, r in sets["horizontal"].items():
        a, b = r["vertex_pts"]
        if not _same_row(a, b):
            print(f"  ⚠ {lbl}: horizontal edge is not same-row"); ok = False
    # 4. diagonal midpoints must join different rows
    for lbl, r in sets["diagonal"].items():
        a, b = r["vertex_pts"]
        if _same_row(a, b):
            print(f"  ⚠ {lbl}: diagonal edge is same-row"); ok = False
    print("  maths self-check:", "PASS ✓" if ok else "FAIL ✗")
    return ok


def output_paths(calib_path):
    """The three intermediate-file paths that generate() would write for this
    calib file (whether or not they exist yet). Lets callers check first."""
    tag = variant_tag(calib_path)
    return {kind: os.path.join(CALIB_DIR, out_name(prefix, tag))
            for kind, prefix in EXTRA_SPECS}


def generate(calib_path, write=True, quiet=False):
    """Build (and optionally write) the triangle/diagonal/horizontal sets for a
    calib_points file. Reusable API for Interdome_touch/main.py. Returns
    (sets, tag, paths_written)."""
    if not os.path.isabs(calib_path):
        calib_path = os.path.join(CALIB_DIR, calib_path)
    if not os.path.exists(calib_path):
        raise SystemExit(f"[error] no such calib file: {calib_path}")

    actual    = load_actual(calib_path)
    sets, thr = build_sets(actual)
    tag       = variant_tag(calib_path)

    if not quiet:
        print(f"[gen] source   : {os.path.basename(calib_path)}")
        print(f"[gen] variant  : {tag or '(plain)'}")
        print(f"[gen] nbr thr  : {thr:.3f} mm (display frame)")
        self_check(sets, actual)

    written = {}
    if write:
        for kind, prefix in EXTRA_SPECS:
            path = os.path.join(CALIB_DIR, out_name(prefix, tag))
            with open(path, "w") as f:
                json.dump(sets[kind], f, indent=2)
            written[kind] = path
            if not quiet:
                print(f"[gen] wrote {len(sets[kind]):2d} {kind:10s} -> "
                      f"{os.path.basename(path)}")
    return sets, tag, written


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("calib", nargs="?", default=DEFAULT_CALIB,
                    help=f"calib_points_*.json (default: {DEFAULT_CALIB})")
    ap.add_argument("--dry-run", action="store_true",
                    help="compute + self-check but do not write files")
    args = ap.parse_args()

    generate(args.calib, write=not args.dry_run)
    if args.dry_run:
        print("[gen] --dry-run: no files written.")


if __name__ == "__main__":
    main()
