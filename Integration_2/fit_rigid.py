"""
fit_rigid_transform_supposed_points.py

Finds the best-fit RIGID transformation (rotation + translation) that
maps the 19 "supposed to be" (theoretical/nominal) points onto the 19
actual robot-calibrated points — ANCHORED so that point 10 maps EXACTLY
onto its actual position (zero residual at point 10), with rotation
solved to best-fit all 19 points around that same anchor.

────────────────────────────────────────────────────────────────────
WHY ANCHORED ON POINT 10 (not a free centroid-based fit)

An unconstrained best-fit (Kabsch algorithm centered on the centroid
of all 19 points) minimizes the TOTAL error across all points, but
that means NO single point matches exactly -- including point 10,
which is what translate_supposed_points.py originally anchored on.
That version left every point, including 10, with some residual.

This version instead: (1) forces point 10 to line up exactly with
its actual position, matching the original intent of
translate_supposed_points.py, and (2) ALSO fits a rotation on top of
that anchor, unlike the plain-translation-only version, since the
data clearly shows the sensor is mounted at a slight angle in
addition to being shifted.

THE MATH

Standard rigid transform:
    P_actual = R * P_theory + t

To force point 10 to map exactly:
    actual[10] = R * theory[10] + t
    =>  t = actual[10] - R * theory[10]

Substituting into the general form:
    P_actual[i] = R * theory[i] + actual[10] - R * theory[10]
                = R * (theory[i] - theory[10]) + actual[10]

So if we CENTER both point sets on point 10 (instead of on the
centroid, like a standard Kabsch fit would), the anchor constraint is
automatically satisfied for ANY rotation R -- because point 10 minus
itself is (0,0), and R @ (0,0) = (0,0), so P_new[10] always equals
actual[10] exactly, no matter what R turns out to be.

That leaves rotation R as the only free parameter, solved via the
same Kabsch/SVD approach as before, just applied to point-10-centered
coordinates instead of centroid-centered ones:

    1. theory_c[i] = theory[i]  - theory[10]     (center on anchor)
       actual_c[i] = actual[i]  - actual[10]
    2. H = sum_i( theory_c[i] outer_product actual_c[i] )
    3. SVD: H = U * S * V^T
    4. R = V * U^T   (flip sign of V's last column if det(R) < 0,
                       to force a proper rotation, not a reflection)
    5. t = actual[10] - R @ theory[10]   (equivalent to the anchor
                                            constraint above)

This will generally have a slightly HIGHER total residual across all
19 points than the unconstrained centroid-based fit (since it's a
more constrained problem), but point 10 will match exactly, which is
the actual requirement here.

Uses the ORIGINAL arbitrary P1-P19 numbering throughout (matches
calib_points_short_new_hollow_2.json).

Output:
    calib_points_supposed_rigid_transformed.json
────────────────────────────────────────────────────────────────────
"""

import json
import os
import math

import numpy as np

BASE_DIR = "/home/cao/Documents/Star_muse_sensor/Star_nose_sensor/Integration_2"
MOCAP_JSON = os.path.join(BASE_DIR, "calib_points_short_new_hollow_2.json")
OUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_rigid_transformed_new_hollow_2.json")

ANCHOR = 10   # point forced to match exactly

# "Supposed to be" (nominal/theoretical) points — from calibrate_points.py
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
    """actual[pid] = nominal[pid] + global offset + per_point offset"""
    with open(path) as f:
        data = json.load(f)
    g = data.get("global", {})
    gx, gy = g.get("x_mm", 0.0), g.get("y_mm", 0.0)
    actual = {}
    for key, off in data["per_point"].items():
        pid = int(key)
        if pid not in POINTS:
            continue
        nx, ny = POINTS[pid]
        dx, dy = off.get("dx_mm", 0.0), off.get("dy_mm", 0.0)
        actual[pid] = (nx + gx + dx, ny + gy + dy)
    return actual


def fit_anchored_rotation(P_theory, P_actual, anchor_theory, anchor_actual):
    """
    Best-fit rotation R such that, once translation is forced to make
    the anchor point match exactly, the OTHER points fit as well as
    possible in a least-squares sense. Returns (R (2x2), theta_deg).
    """
    theory_c = P_theory - anchor_theory
    actual_c = P_actual - anchor_actual

    H = theory_c.T @ actual_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:   # reflection guard -> force proper rotation
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    theta_deg = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return R, theta_deg


def main():
    actual = load_actual_points(MOCAP_JSON)

    ids = sorted(POINTS.keys())
    P_theory = np.array([POINTS[i] for i in ids])
    P_actual = np.array([actual[i] for i in ids])

    anchor_theory = np.array(POINTS[ANCHOR])
    anchor_actual = np.array(actual[ANCHOR])

    R, theta_deg = fit_anchored_rotation(P_theory, P_actual, anchor_theory, anchor_actual)
    t = anchor_actual - R @ anchor_theory   # forces exact match at ANCHOR

    print(f"Anchor point:         {ANCHOR}")
    print(f"Best-fit rotation:    {theta_deg:+.4f} degrees (around point {ANCHOR})")
    print(f"Resulting translation:({t[0]:+.4f}, {t[1]:+.4f}) mm")
    print()

    # Apply the fitted transform to every nominal point
    P_transformed = (R @ P_theory.T).T + t
    transformed = {pid: (round(float(x), 4), round(float(y), 4))
                   for pid, (x, y) in zip(ids, P_transformed)}

    # Residuals: how far each transformed point still is from actual
    residuals = {}
    for pid in ids:
        tx, ty = transformed[pid]
        ax_, ay_ = actual[pid]
        residuals[pid] = math.hypot(ax_ - tx, ay_ - ty)

    print(f"{'pid':>4} {'nominal':>18} {'transformed':>20} {'actual':>20} {'residual':>10}")
    for pid in ids:
        nx, ny = POINTS[pid]
        tx, ty = transformed[pid]
        ax_, ay_ = actual[pid]
        marker = "  <- ANCHOR" if pid == ANCHOR else ""
        print(f"{pid:>4} ({nx:+7.2f},{ny:+7.2f}) "
              f"({tx:+8.3f},{ty:+8.3f}) "
              f"({ax_:+8.3f},{ay_:+8.3f}) "
              f"{residuals[pid]:>9.4f}{marker}")

    mags = list(residuals.values())
    print(f"\nResidual after anchored rigid fit: "
          f"mean = {sum(mags)/len(mags):.4f} mm, max = {max(mags):.4f} mm "
          f"(point {max(residuals, key=residuals.get)})")
    print(f"Anchor point {ANCHOR} residual: {residuals[ANCHOR]:.8f} mm (should be ~0)")

    out = {
        "anchor_point": ANCHOR,
        "rotation_deg": round(theta_deg, 4),
        "rotation_matrix": R.tolist(),
        "translation_mm": {"x_mm": round(float(t[0]), 4), "y_mm": round(float(t[1]), 4)},
        "residual_mm": {"mean": round(sum(mags) / len(mags), 4), "max": round(max(mags), 4)},
        "points": {str(pid): {"x_mm": x, "y_mm": y} for pid, (x, y) in transformed.items()},
    }
    with open(OUT_JSON, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved -> {OUT_JSON}")


if __name__ == "__main__":
    main()