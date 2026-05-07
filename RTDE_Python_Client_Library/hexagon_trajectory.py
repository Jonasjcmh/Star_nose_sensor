"""
UR5 Hexagonal Grid Trajectory — RTDE
Robot IP : 177.22.22.2
19 points distributed in a hexagon (edge ~21.4 mm)
Row layout : 3-4-5-4-3  |  H-spacing: 8 mm  |  V-spacing: 7 mm

Z is kept CONSTANT throughout the entire trajectory — only X and Y change.
Orientation (rx, ry, rz) is also kept constant from REFERENCE_POSE.

To change the trajectory, edit only SEQUENCE — use point numbers (1-19).
Point coordinates are defined once in POINTS and never need to be touched.

Dependencies:
    pip install ur-rtde
"""

import rtde_control
import rtde_receive

# ──────────────────────────────────────────────
# CONNECTION
# ──────────────────────────────────────────────
ROBOT_IP = "177.22.22.2"

# ──────────────────────────────────────────────
# MOTION PARAMETERS
# ──────────────────────────────────────────────
VELOCITY     = 0.05   # m/s
ACCELERATION = 0.3    # m/s²
BLEND_RADIUS = 0.000  # m — 0.0 = full stop at every point

# ──────────────────────────────────────────────
# REFERENCE POSE — hexagon center (point 10)
# Format: [x, y, z, rx, ry, rz]  (meters, radians)
# Z and orientation are FIXED for the entire trajectory.
# ──────────────────────────────────────────────
REFERENCE_POSE = [-0.03746, -0.50066, 0.06054+0.003

    , -2.35063, 2.08341, -0.00009]

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
# SEQUENCE  ← only edit this to change the trajectory
# List of point numbers in the order the robot will visit them.
# You can repeat points, skip points, or reorder freely.
# ──────────────────────────────────────────────
SEQUENCE = [10, 1, 2, 3, 7, 6, 5, 4, 8, 9, 10, 11, 12, 16, 15, 14, 13, 17, 18, 19, 10]


# ──────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────
def build_pose(point_number: int) -> list:
    """
    Return the absolute TCP pose for a given point number.
    Only X and Y are offset from REFERENCE_POSE — Z and orientation never change.
    """
    dx_mm, dy_mm = POINTS[point_number]
    pose = REFERENCE_POSE.copy()
    pose[0] += dx_mm / 1000.0
    pose[1] += dy_mm / 1000.0
    return pose


def main() -> None:
    print("=" * 55)
    print("  UR5 Hexagonal Trajectory — RTDE")
    print(f"  Robot IP  : {ROBOT_IP}")
    print(f"  Sequence  : {SEQUENCE}")
    print(f"  Steps     : {len(SEQUENCE)}")
    print(f"  Velocity  : {VELOCITY} m/s")
    print(f"  Z (fixed) : {REFERENCE_POSE[2]:.5f} m")
    print("=" * 55)

    # ── Connect ───────────────────────────────────────────────
    print("\nConnecting to robot…")
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print("Connected.\n")

    # ── Print current TCP pose for verification ───────────────
    current = rtde_r.getActualTCPPose()
    print("Current TCP pose:")
    print(f"  X={current[0]:.5f}  Y={current[1]:.5f}  Z={current[2]:.5f}")
    print(f"  Rx={current[3]:.5f}  Ry={current[4]:.5f}  Rz={current[5]:.5f}")

    print(f"\nReference pose (point 10 / hexagon center):")
    print(f"  X={REFERENCE_POSE[0]:.5f}  Y={REFERENCE_POSE[1]:.5f}  Z={REFERENCE_POSE[2]:.5f}")
    print(f"  Rx={REFERENCE_POSE[3]:.5f}  Ry={REFERENCE_POSE[4]:.5f}  Rz={REFERENCE_POSE[5]:.5f}")

    input("\nPress ENTER to move to start position (point 10), or Ctrl+C to abort: ")

    # ── Move to start position (point 10 = REFERENCE_POSE) ────
    print("\nMoving to point 10 (hexagon center)…")
    rtde_c.moveL(REFERENCE_POSE, VELOCITY, ACCELERATION)
    print("At start position.\n")

    input("Press ENTER to begin trajectory, or Ctrl+C to abort: ")

    # ── Execute sequence ──────────────────────────────────────
    print(f"\nStarting trajectory — Z locked at {REFERENCE_POSE[2]:.5f} m\n")

    for i, point_number in enumerate(SEQUENCE):
        pose = build_pose(point_number)
        dx_mm, dy_mm = POINTS[point_number]

        # No blend on the last step — full stop
        blend = BLEND_RADIUS if i < len(SEQUENCE) - 1 else 0.0

        print(f"  [{i+1:02d}/{len(SEQUENCE)}] P{point_number:02d}  "
              f"X={pose[0]:+.5f}  Y={pose[1]:+.5f}  Z={pose[2]:+.5f}  "
              f"(dx={dx_mm:+.1f} mm  dy={dy_mm:+.1f} mm)")

        rtde_c.moveL(pose, VELOCITY, ACCELERATION, blend)

    # ── Disconnect ────────────────────────────────────────────
    rtde_c.stopScript()
    print("\nDone. RTDE script stopped.")


if __name__ == "__main__":
    main()

