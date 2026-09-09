"""
imu_trajectories.py
Parameterised XY trajectory definitions for the inertial (IMU) test.

Inspired by friction_mode/trajectories.py, but every shape is sized by a
single knob so you can "define the diameter or the covered area" at run time:

    • circle / spiral            → diameter (mm)
    • raster / square / cross    → covered span width × height (mm)
    • line_h / line_v            → length (mm)

All coordinates are in mm, centred on the working-area origin (0, 0), which
maps to the robot reference pose in ur5_imu.py. The whole working area is a
square of at most WORKAREA_MAX_MM (230 mm = 23 cm) per side; every generated
trajectory is clamped to fit inside it.

The same module also provides the geometry helpers used to import and rescale
external trajectories from JSON (see import_trajectory.py):

    scale_to_area(pts, span_mm, fill=True)   → recentre + scale a point list
    clamp_to_area(pts, span_mm)              → shrink only if it overflows
    bbox(pts)                                → (xmin, xmax, ymin, ymax)
"""
import math

# ── Working area ───────────────────────────────────────────────────────────────
WORKAREA_MAX_MM = 230.0          # hard cap: 23 cm × 23 cm square (per side)
DEFAULT_SPAN_MM = 120.0          # default covered span when none is given


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers (shared with the JSON importer)
# ─────────────────────────────────────────────────────────────────────────────

def bbox(pts):
    """Return (xmin, xmax, ymin, ymax) of a list of (x, y) points."""
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def _recenter(pts):
    """
    Translate points so their bounding-box centre sits at the origin.
    Any extra per-waypoint columns (e.g. a yaw angle) are preserved untouched.
    """
    xmin, xmax, ymin, ymax = bbox(pts)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    return [(p[0] - cx, p[1] - cy, *p[2:]) for p in pts]


def scale_to_area(pts, span_mm=WORKAREA_MAX_MM, fill=True):
    """
    Recentre a trajectory on the origin and scale its XY into the working area.

    Parameters
    ----------
    pts     : list of (x, y) or (x, y, yaw, ...) in arbitrary units. Only X and
              Y are scaled; any trailing columns (yaw angle, etc.) pass through
              unchanged — angles must not be rescaled.
    span_mm : target span of the LARGER bounding-box dimension (mm).
              Capped at WORKAREA_MAX_MM so the robot never exceeds 23 cm.
    fill    : True  → scale so the larger dimension equals span_mm (fill area).
              False → only shrink if the shape is bigger than span_mm (clamp).

    The aspect ratio is always preserved.
    """
    span_mm = min(float(span_mm), WORKAREA_MAX_MM)
    centred = _recenter(pts)
    xmin, xmax, ymin, ymax = bbox(centred)
    max_dim = max(xmax - xmin, ymax - ymin)
    if max_dim <= 1e-9:
        return centred                      # degenerate (single point / empty)
    factor = span_mm / max_dim
    if not fill:
        factor = min(1.0, factor)           # clamp mode: never enlarge
    return [(p[0] * factor, p[1] * factor, *p[2:]) for p in centred]


def clamp_to_area(pts, span_mm=WORKAREA_MAX_MM):
    """Safety shrink: scale down (never up) so nothing exceeds the area."""
    return scale_to_area(pts, span_mm=span_mm, fill=False)


# ─────────────────────────────────────────────────────────────────────────────
# Trajectory generators — each centred at (0, 0), sized in mm
# ─────────────────────────────────────────────────────────────────────────────

def _interp(p0, p1, n):
    """Linear interpolation between two (x, y) points, n samples inclusive."""
    n = max(2, int(n))
    return [
        (p0[0] + (p1[0] - p0[0]) * i / (n - 1),
         p0[1] + (p1[1] - p0[1]) * i / (n - 1))
        for i in range(n)
    ]


def circle(diameter_mm=DEFAULT_SPAN_MM, n_steps=120, **_):
    """Closed circle of the given diameter, counter-clockwise from +X."""
    r = diameter_mm / 2.0
    return [
        (r * math.cos(2 * math.pi * i / n_steps),
         r * math.sin(2 * math.pi * i / n_steps))
        for i in range(n_steps + 1)
    ]


def spiral(diameter_mm=DEFAULT_SPAN_MM, n_turns=4, n_steps=400, **_):
    """Archimedean spiral from the centre outward to diameter_mm."""
    r_max = diameter_mm / 2.0
    pts = []
    for i in range(n_steps + 1):
        t = i / n_steps
        r = r_max * t
        angle = 2 * math.pi * n_turns * t
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    return pts


def raster(width_mm=DEFAULT_SPAN_MM, height_mm=DEFAULT_SPAN_MM,
           n_lines=8, n_pts_per_line=40, **_):
    """
    Boustrophedon (snake) raster covering width_mm × height_mm.
    Alternates left→right / right→left each row to minimise travel.
    """
    n_lines = max(2, int(n_lines))
    ys = [-height_mm / 2 + height_mm * r / (n_lines - 1) for r in range(n_lines)]
    x0, x1 = -width_mm / 2, width_mm / 2
    pts = []
    for i, y in enumerate(ys):
        a, b = (x0, x1) if i % 2 == 0 else (x1, x0)
        pts.extend((x, y) for x, y in _interp((a, y), (b, y), n_pts_per_line))
    return pts


def square(side_mm=DEFAULT_SPAN_MM, n_per_side=40, **_):
    """Closed square outline of the given side length."""
    h = side_mm / 2.0
    corners = [(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)]
    pts = []
    for i in range(len(corners) - 1):
        seg = _interp(corners[i], corners[i + 1], n_per_side)
        pts.extend(seg[:-1])
    pts.append(corners[-1])
    return pts


def cross(size_mm=DEFAULT_SPAN_MM, n_steps=40, **_):
    """Plus-shaped cross: horizontal sweep then vertical sweep."""
    h = _interp((-size_mm / 2, 0.0), (size_mm / 2, 0.0), n_steps)
    v = _interp((0.0, -size_mm / 2), (0.0, size_mm / 2), n_steps)
    return h + v


def line_h(length_mm=DEFAULT_SPAN_MM, n_steps=60, **_):
    """Horizontal line sweep, left to right through the origin."""
    return _interp((-length_mm / 2, 0.0), (length_mm / 2, 0.0), n_steps)


def line_v(length_mm=DEFAULT_SPAN_MM, n_steps=60, **_):
    """Vertical line sweep, bottom to top through the origin."""
    return _interp((0.0, -length_mm / 2), (0.0, length_mm / 2), n_steps)


def figure8(size_mm=DEFAULT_SPAN_MM, n_steps=240, **_):
    """Lemniscate (figure-of-eight) inscribed in a size_mm square."""
    a = size_mm / 2.0
    pts = []
    for i in range(n_steps + 1):
        t = 2 * math.pi * i / n_steps
        pts.append((a * math.sin(t), a * math.sin(t) * math.cos(t)))
    return pts


# ── Registry — name → (builder, size-parameter name) ────────────────────────────
# The size-parameter name is what a single generic --size value maps onto, so the
# same knob defines "diameter" for round shapes and "covered area" for the rest.
TRAJECTORIES = {
    'circle':  (circle,   'diameter_mm'),
    'spiral':  (spiral,   'diameter_mm'),
    'raster':  (raster,   'width_mm'),      # size also drives height_mm below
    'square':  (square,   'side_mm'),
    'cross':   (cross,    'size_mm'),
    'line_h':  (line_h,   'length_mm'),
    'line_v':  (line_v,   'length_mm'),
    'figure8': (figure8,  'size_mm'),
}

LABELS = {
    'circle':  'Circle',
    'spiral':  'Archimedean Spiral',
    'raster':  'Raster Scan',
    'square':  'Square Outline',
    'cross':   'Cross (+)',
    'line_h':  'Horizontal Line',
    'line_v':  'Vertical Line',
    'figure8': 'Figure-of-Eight',
}


def build(name, size_mm=None, **params):
    """
    Build a named trajectory, sizing it with a single `size_mm` knob.

    `size_mm` is applied to whichever parameter the shape is sized by
    (diameter for circle/spiral, side for square, width+height for raster,
    length for lines, overall size for cross/figure8). Any explicit keyword
    in `params` (e.g. n_turns, n_steps, height_mm) overrides the default.

    The result is always clamped to the 23 cm × 23 cm working area.
    """
    if name not in TRAJECTORIES:
        raise ValueError(f"unknown trajectory '{name}' "
                         f"(choose from {', '.join(TRAJECTORIES)})")
    builder, size_key = TRAJECTORIES[name]
    if size_mm is not None:
        params.setdefault(size_key, size_mm)
        if name == 'raster':                       # raster is 2-D: fill both dims
            params.setdefault('height_mm', size_mm)
    pts = builder(**params)
    return clamp_to_area(pts, WORKAREA_MAX_MM)


def describe(name, pts):
    """Print a compact summary of a trajectory."""
    xmin, xmax, ymin, ymax = bbox(pts)
    print(f"  Trajectory '{name}': {len(pts)} waypoints")
    print(f"    X span : {xmin:+.1f} .. {xmax:+.1f} mm  ({xmax - xmin:.1f} mm wide)")
    print(f"    Y span : {ymin:+.1f} .. {ymax:+.1f} mm  ({ymax - ymin:.1f} mm tall)")
    print(f"    Start  : ({pts[0][0]:+.1f}, {pts[0][1]:+.1f}) mm")
    print(f"    End    : ({pts[-1][0]:+.1f}, {pts[-1][1]:+.1f}) mm")


if __name__ == '__main__':
    # Quick self-test / preview of every registered trajectory.
    print(f"Working area: {WORKAREA_MAX_MM:.0f} x {WORKAREA_MAX_MM:.0f} mm\n")
    for key in TRAJECTORIES:
        describe(key, build(key, size_mm=DEFAULT_SPAN_MM))
        print()
