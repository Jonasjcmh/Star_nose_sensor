"""
UR5 Hexagonal Grid Trajectory — RTDE
Robot IP : 177.22.22.2
19 points distributed in a hexagon (edge ~21.4 mm)
Row layout : 3-4-5-4-3  |  H-spacing: 8 mm  |  V-spacing: 7 mm

Each point supports:
  - indentation depth (mm) : how deep the robot presses below the surface Z
  - dwell time (s)         : how long it holds the pressed position

To change the trajectory order, edit only SEQUENCE.
To change depth/dwell globally, edit DEFAULT_INDENT_MM and DEFAULT_DWELL_S.
To override a specific point, add it to POINT_OVERRIDES.

Dependencies:
    pip install ur-rtde
"""

import time
import rtde_control
import rtde_receive

# ──────────────────────────────────────────────
# CONNECTION
# ──────────────────────────────────────────────
ROBOT_IP = "177.22.22.2"

# ──────────────────────────────────────────────
# MOTION PARAMETERS
# ──────────────────────────────────────────────
VELOCITY_TRAVEL  = 0.05   # m/s  — speed moving between points
VELOCITY_PRESS   = 0.01   # m/s  — slower speed when pressing down
ACCELERATION     = 0.3    # m/s²
BLEND_RADIUS     = 0.00  # m    — 0.0 = full stop between points

# ──────────────────────────────────────────────
# INDENTATION DEFAULTS  (apply to all points)
# ──────────────────────────────────────────────
DEFAULT_INDENT_MM = 6.00   # mm — how deep to press below surface Z
DEFAULT_DWELL_S   = 1.5   # s  — how long to hold at pressed depth

# ──────────────────────────────────────────────
# PER-POINT OVERRIDES  (optional)
# Add a point number here to give it different depth/dwell than the defaults.
# Any point NOT listed here uses DEFAULT_INDENT_MM and DEFAULT_DWELL_S.
#
# Format: { point_number: (indent_mm, dwell_s), ... }
# Example: point 10 (center) presses 2 mm and holds for 1 second,
#          point 1 presses only 0.5 mm and holds for 0.2 seconds.
# ──────────────────────────────────────────────
POINT_OVERRIDES = {
    # 10: (2.0, 1.0),
    #  1: (0.5, 0.2),
}

# ──────────────────────────────────────────────
# REFERENCE POSE — hexagon center (point 10) at surface Z
# Format: [x, y, z, rx, ry, rz]  (meters, radians)
# X, Y offsets and Z indentation are applied on top of this pose.
# ──────────────────────────────────────────────
REFERENCE_POSE = [-0.03746+0.0005, -0.50066+0.0016, 0.06054, -2.35063, 2.08341, -0.00009]

# ──────────────────────────────────────────────
# POINT DICTIONARY
# Keys   : point number (1-19)
# Values : (x_offset_mm, y_offset_mm) relative to REFERENCE_POSE (point 10)
# ──────────────────────────────────────────────
POINTS = {
    #        x_mm    y_mm
     1: (  -8.0,  +14.0),   # row 1
     2: (   0.0,  +14.0),
     3: (  +8.0,  +14.0),
     4: ( -12.0,   +7.0),   # row 2
     5: (  -4.0,   +7.0),
     6: (  +4.0,   +7.0),
     7: ( +12.0,   +7.0),
     8: ( -16.0,    0.0),   # row 3
     9: (  -8.0,    0.0),
    10: (   0.0,    0.0),   # center — same XY as REFERENCE_POSE
    11: (  +8.0,    0.0),
    12: ( +16.0,    0.0),
    13: ( -12.0,   -7.0),   # row 4
    14: (  -4.0,   -7.0),
    15: (  +4.0,   -7.0),
    16: ( +12.0,   -7.0),
    17: (  -8.0,  -14.0),   # row 5
    18: (   0.0,  -14.0),
    19: (  +8.0,  -14.0),
}

# ──────────────────────────────────────────────
# SEQUENCE  ← edit this to change the trajectory
# List of point numbers in visit order.
# ──────────────────────────────────────────────
SEQUENCE = [10, 1, 2, 3, 7, 6, 5, 4, 8, 9, 10, 11, 12, 16, 15, 14, 13, 17, 18, 19, 10]


# ──────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────
def get_indent_dwell(point_number: int) -> tuple:
    """Return (indent_mm, dwell_s) for a point — override or default."""
    return POINT_OVERRIDES.get(point_number, (DEFAULT_INDENT_MM, DEFAULT_DWELL_S))


def build_pose(point_number: int, extra_z_mm: float = 0.0) -> list:
    """
    Return the absolute TCP pose for a given point number.
    extra_z_mm < 0  → press into surface (indentation)
    extra_z_mm = 0  → surface level (travel height)
    Only X, Y, and Z are modified — orientation never changes.
    """
    dx_mm, dy_mm = POINTS[point_number]
    pose = REFERENCE_POSE.copy()
    pose[0] += dx_mm / 1000.0
    pose[1] += dy_mm / 1000.0
    pose[2] += extra_z_mm / 1000.0
    return pose


def visit_point(rtde_c, step: int, total: int, point_number: int) -> None:
    """
    Full press sequence for one point:
      1. Travel to XY position at surface Z
      2. Press down by indent_mm
      3. Hold for dwell_s
      4. Retract back to surface Z
    """
    indent_mm, dwell_s = get_indent_dwell(point_number)
    dx_mm, dy_mm = POINTS[point_number]

    surface_pose  = build_pose(point_number, extra_z_mm=0.0)
    pressed_pose  = build_pose(point_number, extra_z_mm=-indent_mm)  # negative = deeper

    print(f"  [{step:02d}/{total}] P{point_number:02d}  "
          f"XY=({dx_mm:+.1f}, {dy_mm:+.1f}) mm  "
          f"indent={indent_mm:.2f} mm  dwell={dwell_s:.2f} s")

    # 1. Arrive at surface XY position
    rtde_c.moveL(surface_pose, VELOCITY_TRAVEL, ACCELERATION)

    # 2. Press down slowly
    rtde_c.moveL(pressed_pose, VELOCITY_PRESS, ACCELERATION)

    # 3. Hold at depth
    time.sleep(dwell_s)

    # 4. Retract back to surface Z
    rtde_c.moveL(surface_pose, VELOCITY_PRESS, ACCELERATION)


def main() -> None:
    # ── Print config summary ──────────────────────────────────
    print("=" * 60)
    print("  UR5 Hexagonal Trajectory — RTDE")
    print(f"  Robot IP         : {ROBOT_IP}")
    print(f"  Sequence         : {SEQUENCE}")
    print(f"  Steps            : {len(SEQUENCE)}")
    print(f"  Surface Z        : {REFERENCE_POSE[2]:.5f} m")
    print(f"  Default indent   : {DEFAULT_INDENT_MM:.2f} mm")
    print(f"  Default dwell    : {DEFAULT_DWELL_S:.2f} s")
    if POINT_OVERRIDES:
        print(f"  Overrides        : {POINT_OVERRIDES}")
    print("=" * 60)

    # ── Connect ───────────────────────────────────────────────
    print("\nConnecting to robot…")
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print("Connected.\n")

    # ── Print current TCP pose ────────────────────────────────
    current = rtde_r.getActualTCPPose()
    print("Current TCP pose:")
    print(f"  X={current[0]:.5f}  Y={current[1]:.5f}  Z={current[2]:.5f}")
    print(f"  Rx={current[3]:.5f}  Ry={current[4]:.5f}  Rz={current[5]:.5f}")

    print(f"\nReference pose (point 10 / surface):")
    print(f"  X={REFERENCE_POSE[0]:.5f}  Y={REFERENCE_POSE[1]:.5f}  Z={REFERENCE_POSE[2]:.5f}")
    print(f"  Rx={REFERENCE_POSE[3]:.5f}  Ry={REFERENCE_POSE[4]:.5f}  Rz={REFERENCE_POSE[5]:.5f}")

    input("\nPress ENTER to move to start position (point 10), or Ctrl+C to abort: ")

    # ── Move to start ─────────────────────────────────────────
    print("\nMoving to point 10 (hexagon center)…")
    rtde_c.moveL(REFERENCE_POSE, VELOCITY_TRAVEL, ACCELERATION)
    print("At start position.\n")

    input("Press ENTER to begin trajectory, or Ctrl+C to abort: ")

    # ── Execute sequence ──────────────────────────────────────
    print(f"\nStarting trajectory…\n")
    total = len(SEQUENCE)

    for step, point_number in enumerate(SEQUENCE, start=1):
        visit_point(rtde_c, step, total, point_number)

    # ── Disconnect ────────────────────────────────────────────
    rtde_c.stopScript()
    print("\nDone. RTDE script stopped.")


if __name__ == "__main__":
    main()