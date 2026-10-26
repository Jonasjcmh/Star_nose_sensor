"""
fit_rigid_transform_supposed_points.py

Finds the best-fit RIGID transformation (rotation + translation) that
maps the 19 "supposed to be" (theoretical/nominal) points onto the 19
actual robot-calibrated points, using ALL 19 point correspondences —
not just anchoring on a single point like translate_supposed_points.py
did.

────────────────────────────────────────────────────────────────────
WHY THIS IS DIFFERENT FROM translate_supposed_points.py
That script computed ONE offset from point 10 alone (actual[10] -
nominal[10]) and applied it to every point. That's correct only if
the sensor is purely SHIFTED relative to its theoretical mount, with
no rotation. In reality, mounting also introduces a small rotation
(the sensor sits ~8 degrees rotated versus its theoretical CAD
orientation) — a translation-only fit can't capture that, so all
non-anchor points carry residual error from the un-modeled rotation.

This script fits BOTH rotation and translation simultaneously, using
every point (not just one), which is the proper way to align two
sets of corresponding points related by a rigid-body motion.

THE MATH — 2D version of P^A = T_B^A * P^B (same idea as the 4x4
homogeneous transform example, just simplified to 2D since Z=0 for
every point here):

    P_actual = R * P_theory + t

    R = [[cos(theta), -sin(theta)],
         [sin(theta),  cos(theta)]]   (2x2 rotation matrix)
    t = [tx, ty]                       (2D translation vector)

HOW theta AND t ARE FOUND — the Kabsch / orthogonal Procrustes
algorithm (standard method for best-fit rigid registration between
two corresponding point sets, minimizing total squared error across
ALL points simultaneously):

    1. Compute centroids of both point sets:
         centroid_theory = mean(P_theory)
         centroid_actual = mean(P_actual)

    2. Center both point sets on their centroids:
         theory_c[i] = P_theory[i] - centroid_theory
         actual_c[i] = P_actual[i] - centroid_actual

    3. Build the 2x2 cross-covariance matrix:
         H = sum_i( theory_c[i] outer_product actual_c[i] )

    4. Singular Value Decomposition:  H = U * S * V^T

    5. Optimal rotation:  R = V * U^T
       (if det(R) < 0, this would be a reflection, not a pure
       rotation — flip the sign of the last column of V and
       recompute R to force a proper rotation)

    6. Optimal translation:  t = centroid_actual - R @ centroid_theory

This is the least-squares-optimal rigid transform: no other rotation
+ translation combination gives a lower total squared error over all
19 points.

Uses the ORIGINAL arbitrary P1-P19 numbering throughout (matches
calib_points_short_6mm.json and translate_supposed_points.py).

Output:
    calib_points_supposed_rigid_transformed.json
────────────────────────────────────────────────────────────────────
"""

import json
import os
import math

import numpy as np

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"
MOCAP_JSON = os.path.join(BASE_DIR, "calib_points_short_6mm.json")
OUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_rigid_transformed.json")

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


def fit_rigid_transform(P_theory, P_actual):
    """
    Kabsch algorithm: best-fit 2D rotation + translation mapping
    P_theory onto P_actual, minimizing sum of squared errors.
    Returns (R (2x2 array), t (2-vector), theta_deg (float)).
    """
    centroid_theory = P_theory.mean(axis=0)
    centroid_actual = P_actual.mean(axis=0)

    theory_c = P_theory - centroid_theory
    actual_c = P_actual - centroid_actual

    H = theory_c.T @ actual_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:   # reflection guard -> force proper rotation
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_actual - R @ centroid_theory
    theta_deg = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return R, t, theta_deg


def main():
    actual = load_actual_points(MOCAP_JSON)

    ids = sorted(POINTS.keys())
    P_theory = np.array([POINTS[i] for i in ids])
    P_actual = np.array([actual[i] for i in ids])

    R, t, theta_deg = fit_rigid_transform(P_theory, P_actual)

    print(f"Best-fit rotation:    {theta_deg:+.4f} degrees")
    print(f"Best-fit translation: ({t[0]:+.4f}, {t[1]:+.4f}) mm")
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
        print(f"{pid:>4} ({nx:+7.2f},{ny:+7.2f}) "
              f"({tx:+8.3f},{ty:+8.3f}) "
              f"({ax_:+8.3f},{ay_:+8.3f}) "
              f"{residuals[pid]:>9.4f}")

    mags = list(residuals.values())
    print(f"\nResidual after rigid (rotation+translation) fit: "
          f"mean = {sum(mags)/len(mags):.4f} mm, max = {max(mags):.4f} mm "
          f"(point {max(residuals, key=residuals.get)})")

    out = {
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