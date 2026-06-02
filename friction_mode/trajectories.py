"""
trajectories.py
Named XY trajectory definitions for the friction mode experiment.

All coordinates are in mm, relative to the sensor center (P10 = 0, 0).
Each function returns a list of (x_mm, y_mm) waypoints that the robot
will visit in order at the specified depth / force level.
"""
import math


def _interp(p0, p1, n):
    """Linear interpolation between two (x, y) points, n samples inclusive."""
    return [
        (p0[0] + (p1[0] - p0[0]) * i / (n - 1),
         p0[1] + (p1[1] - p0[1]) * i / (n - 1))
        for i in range(n)
    ]


def line_h(n_steps=40):
    """Horizontal sweep — left to right across the center row (y = 0)."""
    return _interp((-16.0, 0.0), (16.0, 0.0), n_steps)


def line_v(n_steps=40):
    """Vertical sweep — bottom to top through the center column (x = 0)."""
    return _interp((0.0, -14.0), (0.0, 14.0), n_steps)


def diagonal_lr(n_steps=40):
    """Diagonal sweep — lower-left to upper-right."""
    return _interp((-12.0, -7.0), (12.0, 7.0), n_steps)


def diagonal_rl(n_steps=40):
    """Diagonal sweep — lower-right to upper-left."""
    return _interp((12.0, -7.0), (-12.0, 7.0), n_steps)


def circle(n_steps=72, radius_mm=12.0):
    """
    Circular path around the sensor center.
    Starts at the rightmost point (angle = 0) and goes counterclockwise.
    A closed loop: last point = first point.
    """
    pts = [
        (radius_mm * math.cos(2 * math.pi * i / n_steps),
         radius_mm * math.sin(2 * math.pi * i / n_steps))
        for i in range(n_steps + 1)
    ]
    return pts


def raster(rows=5, cols=9):
    """
    Boustrophedon (snake) raster scan covering the full sensor area.
    Alternates left→right and right→left for each row to minimize
    travel time between rows.
    """
    ys = [-14.0 + 28.0 * r / (rows - 1) for r in range(rows)]
    xs = [-16.0 + 32.0 * c / (cols - 1) for c in range(cols)]
    pts = []
    for i, y in enumerate(ys):
        row_xs = xs if i % 2 == 0 else xs[::-1]
        pts.extend((x, y) for x in row_xs)
    return pts


def cross(n_steps=30):
    """
    Cross pattern: horizontal sweep then vertical sweep.
    Useful for comparing orthogonal friction directions.
    """
    h = _interp((-16.0, 0.0), (16.0, 0.0), n_steps)
    # lift and return are handled by the robot; just concatenate
    v = _interp((0.0, -14.0), (0.0, 14.0), n_steps)
    return h + v


def spiral(n_turns=2, n_steps=120, r_max_mm=14.0):
    """
    Archimedean spiral from center outward.
    n_turns: number of full rotations
    """
    pts = []
    for i in range(n_steps + 1):
        t = i / n_steps
        r = r_max_mm * t
        angle = 2 * math.pi * n_turns * t
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    return pts


def star_path(n_steps_per_seg=10):
    """
    Continuous path visiting all 19 sensor points in the standard sequence,
    with interpolated waypoints between each pair of adjacent stops.
    Matches the discrete-press trajectory of the main experiment.
    """
    POINTS = {
         1: ( -8.0, +14.0),  2: (  0.0, +14.0),  3: ( +8.0, +14.0),
         4: (-12.0,  +7.0),  5: ( -4.0,  +7.0),  6: ( +4.0,  +7.0),
         7: (+12.0,  +7.0),  8: (-16.0,   0.0),  9: ( -8.0,   0.0),
        10: (  0.0,   0.0), 11: ( +8.0,   0.0), 12: (+16.0,   0.0),
        13: (-12.0,  -7.0), 14: ( -4.0,  -7.0), 15: ( +4.0,  -7.0),
        16: (+12.0,  -7.0), 17: ( -8.0, -14.0), 18: (  0.0, -14.0),
        19: ( +8.0, -14.0),
    }
    SEQUENCE = [10, 1, 2, 3, 7, 6, 5, 4, 8, 9, 10, 11, 12,
                16, 15, 14, 13, 17, 18, 19, 10]
    pts = []
    for i in range(len(SEQUENCE) - 1):
        p0 = POINTS[SEQUENCE[i]]
        p1 = POINTS[SEQUENCE[i + 1]]
        seg = _interp(p0, p1, n_steps_per_seg)
        pts.extend(seg[:-1])   # exclude last point to avoid duplicate at junction
    pts.append(POINTS[SEQUENCE[-1]])
    return pts


# Registry — name → function
TRAJECTORIES = {
    'line_h':      line_h,
    'line_v':      line_v,
    'diagonal_lr': diagonal_lr,
    'diagonal_rl': diagonal_rl,
    'circle':      circle,
    'raster':      raster,
    'cross':       cross,
    'spiral':      spiral,
    'star':        star_path,
}


def preview(name, pts):
    """Print a compact ASCII preview of a trajectory."""
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    print(f"  Trajectory '{name}': {len(pts)} waypoints")
    print(f"    X range : {min(xs):+.1f} .. {max(xs):+.1f} mm")
    print(f"    Y range : {min(ys):+.1f} .. {max(ys):+.1f} mm")
    print(f"    First   : ({xs[0]:+.1f}, {ys[0]:+.1f}) mm")
    print(f"    Last    : ({xs[-1]:+.1f}, {ys[-1]:+.1f}) mm")
