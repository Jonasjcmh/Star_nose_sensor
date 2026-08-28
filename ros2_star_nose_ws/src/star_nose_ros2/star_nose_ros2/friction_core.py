"""
friction_core.py
Self-contained geometry + trajectory generation for the ROS2 friction nodes.

Ported (not cross-imported) from Star_nose_sensor/friction_mode so the package
stays deployable on its own, following the same "port, don't reach across the
repo" convention as the other nodes here.

Two things this module owns:

1. Robot <-> sensor frame conversion. The sensor/display frame is a ~120°
   rotation of the UR5 base frame (calibrate_points._ROBOT_TO_SENSOR). The
   trajectories below are authored in the SENSOR frame (a "horizontal" line is
   horizontal ON THE SENSOR), so traj_points() converts them to the ROBOT frame
   before returning — otherwise a "horizontal" sweep runs almost entirely along
   the display Y axis. Everything downstream (pose building, clamping) is
   robot-frame.

2. Sensor-area boundary. clamp_to_sensor() keeps a robot-frame XY inside the
   real sensor grid (the DISPLAY-frame bounding box of the 19 cells + margin),
   so jogs and trajectories can never wander off the sensor.
"""
import math

# ── Robot <-> sensor (display) frame ──────────────────────────────────────────
# Same matrix as Integration_2/calibrate_points._ROBOT_TO_SENSOR. Maps a
# robot-frame XY (mm) to the sensor/display XY (mm). det == 1 (area preserving)
# but it is NOT orthonormal — a rotation plus a small shear.
_ROBOT_TO_SENSOR = ((-0.5, -6.0 / 7.0),
                    (7.0 / 8.0, -0.5))


def to_display(x_mm, y_mm):
    """Robot-frame XY (mm) -> sensor/display XY (mm)."""
    (a, b), (c, d) = _ROBOT_TO_SENSOR
    return (a * x_mm + b * y_mm, c * x_mm + d * y_mm)


def sensor_to_robot(sdx, sdy):
    """Sensor/display-frame vector (mm) -> robot-frame vector (mm). Inverse of
    to_display. Used both to orient the trajectories and to convert a jog along
    the on-screen axes into a robot move."""
    (a, b), (c, d) = _ROBOT_TO_SENSOR
    det = a * d - b * c
    return ((d * sdx - b * sdy) / det, (-c * sdx + a * sdy) / det)


# ── Sensor-area boundary (display frame) ──────────────────────────────────────
# The 19 sensor cells span x in [-16, 16], y in [-14, 14] in the display frame
# (the hex grid drawn by visualizer_2d / calibrate_live). We allow a small
# margin past the outermost cells. clamp_to_sensor works on robot-frame input.
SENSOR_MARGIN_MM = 2.0
_DX_MIN, _DX_MAX = -16.0 - SENSOR_MARGIN_MM, 16.0 + SENSOR_MARGIN_MM
_DY_MIN, _DY_MAX = -14.0 - SENSOR_MARGIN_MM, 14.0 + SENSOR_MARGIN_MM


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clamp_to_sensor(rx, ry):
    """Clamp a robot-frame XY (mm) so the tip stays inside the real sensor area
    (display-frame bounding box of the cells + margin). Returns robot-frame XY."""
    dx, dy = to_display(rx, ry)
    return sensor_to_robot(_clamp(dx, _DX_MIN, _DX_MAX),
                           _clamp(dy, _DY_MIN, _DY_MAX))


# ── Trajectory shapes (SENSOR/display frame, mm, centred on P10) ───────────────
# Ported from friction_mode/trajectories.py — keep in sync if that file changes.
def _interp(p0, p1, n):
    return [(p0[0] + (p1[0] - p0[0]) * i / (n - 1),
             p0[1] + (p1[1] - p0[1]) * i / (n - 1)) for i in range(n)]


def _line_h(n=40):
    return _interp((-16.0, 0.0), (16.0, 0.0), n)


def _line_v(n=40):
    return _interp((0.0, -14.0), (0.0, 14.0), n)


def _diagonal_lr(n=40):
    return _interp((-12.0, -7.0), (12.0, 7.0), n)


def _diagonal_rl(n=40):
    return _interp((12.0, -7.0), (-12.0, 7.0), n)


def _cross(n=30):
    return _interp((-16.0, 0.0), (16.0, 0.0), n) + _interp((0.0, -14.0), (0.0, 14.0), n)


def _circle(n=72, r=12.0):
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n))
            for i in range(n + 1)]


def _spiral(n=120, r_max=14.0, turns=2):
    pts = []
    for i in range(n + 1):
        t = i / n
        ang = 2 * math.pi * turns * t
        pts.append((r_max * t * math.cos(ang), r_max * t * math.sin(ang)))
    return pts


def _raster(rows=5, cols=9):
    ys = [-14.0 + 28.0 * r / (rows - 1) for r in range(rows)]
    xs = [-16.0 + 32.0 * c / (cols - 1) for c in range(cols)]
    pts = []
    for i, y in enumerate(ys):
        pts.extend((x, y) for x in (xs if i % 2 == 0 else xs[::-1]))
    return pts


PATTERNS = {
    'line_h': _line_h, 'line_v': _line_v,
    'diagonal_lr': _diagonal_lr, 'diagonal_rl': _diagonal_rl,
    'cross': _cross, 'circle': _circle, 'spiral': _spiral, 'raster': _raster,
}

_BASE_N = {'circle': 72, 'spiral': 120}
_BASE_R = {'circle': 12.0, 'spiral': 14.0}


def traj_points(name, scale=1.0):
    """Return trajectory waypoints in the ROBOT frame at the given scale.

    circle/spiral regenerate geometry (radius AND point count) so they stay
    smooth as they grow; other patterns just scale their fixed points. The
    sensor-frame result is converted to robot frame via sensor_to_robot so the
    shape traces as drawn on the sensor.
    """
    if name not in PATTERNS:
        raise KeyError(f'unknown trajectory {name!r}')
    if name == 'circle':
        n = int(_clamp(_BASE_N['circle'] * scale, 24, 300))
        raw = _circle(n=n, r=_BASE_R['circle'] * scale)
    elif name == 'spiral':
        n = int(_clamp(_BASE_N['spiral'] * scale, 60, 400))
        raw = _spiral(n=n, r_max=_BASE_R['spiral'] * scale)
    else:
        raw = [(x * scale, y * scale) for x, y in PATTERNS[name]()]
    return [sensor_to_robot(x, y) for x, y in raw]
