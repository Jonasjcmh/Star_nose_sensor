"""
ur_calibration.py  —  Inertial (IMU) Test
=========================================
Maps trajectory-frame coordinates (the 0..230 "drawing" units used in
Trajectories.txt) into the UR robot BASE frame (mm), from two measured
point correspondences:

    trajectory (0,   0)   →  UR base (26.71,  -394.66) mm     # origin corner
    trajectory (230, 230) →  UR base (138.42, -689.91) mm     # opposite corner

Because the two points are opposite corners of a square area, the mapping is
solved as an orientation-preserving SIMILARITY transform (uniform scale +
rotation + translation, no shear, no mirror)::

    X = a*x - b*y + tx
    Y = b*x + a*y + ty          with  a = s·cosθ,  b = s·sinθ

This reproduces both points exactly and turns the 230-unit square into a
223.2 mm square in the robot frame (scale ≈ 0.9705, rotation ≈ -114.28°),
so circles stay circles. A yaw/theta angle in the trajectory frame is rotated
by the same θ to stay physically aligned.

If the shapes come out MIRRORED, the intended mapping is the reflection variant
instead — flip MIRROR to True below (it swaps the handedness of the transform).
"""
import math

# ── Measured correspondences: trajectory-frame → UR base frame (mm) ────────────
P0_TRAJ = (0.0,   0.0)
P0_UR   = (26.71,  -394.66)
P1_TRAJ = (230.0, 230.0)
P1_UR   = (138.42, -689.91)

# Set True only if the transformed shapes are mirrored w.r.t. what you expect.
MIRROR = False


def _solve():
    """Solve a, b, tx, ty for the similarity mapping P0,P1 (traj → UR)."""
    (x0, y0), (X0, Y0) = P0_TRAJ, P0_UR
    (x1, y1), (X1, Y1) = P1_TRAJ, P1_UR
    dx, dy = (x1 - x0), (y1 - y0)          # trajectory-frame diagonal
    dX, dY = (X1 - X0), (Y1 - Y0)          # UR-frame diagonal
    den = dx * dx + dy * dy
    if MIRROR:
        # Reflection variant: X = a*x + b*y + tx ; Y = b*x - a*y + ty
        a = (dX * dx - dY * dy) / den
        b = (dX * dy + dY * dx) / den
    else:
        # Rotation variant:   X = a*x - b*y + tx ; Y = b*x + a*y + ty
        a = (dX * dx + dY * dy) / den
        b = (dY * dx - dX * dy) / den
    tx = X0 - (a * x0 - (b if not MIRROR else -b) * y0)
    ty = Y0 - (b * x0 + (a if not MIRROR else -a) * y0)
    return a, b, tx, ty


A, B, TX, TY = _solve()
SCALE      = math.hypot(A, B)
ROT_DEG    = math.degrees(math.atan2(B, A))


def to_ur(x, y):
    """Trajectory-frame (x, y) → UR base-frame (X, Y) in mm."""
    if MIRROR:
        return (A * x + B * y + TX, B * x - A * y + TY)
    return (A * x - B * y + TX, B * x + A * y + TY)


def to_ur_yaw(theta_deg):
    """
    Trajectory-frame yaw (deg) → UR base-frame yaw (deg).
    A heading rotates with the frame; a mirror also negates it.
    """
    if MIRROR:
        return (-theta_deg + ROT_DEG) % 360.0
    return (theta_deg + ROT_DEG) % 360.0


def corners():
    """The four UR-frame corners of the 0..230 area (for previews / bounds)."""
    return [to_ur(0, 0), to_ur(230, 0), to_ur(230, 230), to_ur(0, 230)]


def ur_bounds(margin_mm=0.0):
    """Axis-aligned (Xmin, Xmax, Ymin, Ymax) bounding box of the area, mm."""
    cs = corners()
    xs = [c[0] for c in cs]; ys = [c[1] for c in cs]
    return (min(xs) - margin_mm, max(xs) + margin_mm,
            min(ys) - margin_mm, max(ys) + margin_mm)


def summary():
    print('UR calibration (trajectory-frame → UR base frame):')
    print(f'  {P0_TRAJ} → {P0_UR} mm')
    print(f'  {P1_TRAJ} → {P1_UR} mm')
    print(f'  mode      : {"MIRROR (reflected)" if MIRROR else "rotation (no mirror)"}')
    print(f'  a, b      : {A:+.6f}, {B:+.6f}')
    print(f'  tx, ty    : {TX:+.3f}, {TY:+.3f} mm')
    print(f'  scale     : {SCALE:.5f}   (230 u → {SCALE*230:.2f} mm)')
    print(f'  rotation  : {ROT_DEG:+.3f} deg')
    print(f'  yaw shift : {"-θ" if MIRROR else "θ"} {ROT_DEG:+.3f} deg')
    print('  corners (UR mm):')
    for name, pt in zip(['(0,0)', '(230,0)', '(230,230)', '(0,230)'], corners()):
        print(f'    {name:>10} → ({pt[0]:8.2f}, {pt[1]:8.2f})')


if __name__ == '__main__':
    summary()
