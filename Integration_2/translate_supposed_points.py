"""
translate_supposed_points.py

Step 1: We have two sets of calibration points for the 19-point sensor:
    - "supposed to be" (nominal) points -> POINTS dict in calibrate_points.py
    - "actual" (robot-calibrated) points -> calib_points_short_6mm.json
      (actual[pid] = nominal[pid] + global offset + per_point offset)

Step 2: Compute the single translation vector that moves the supposed-to-be
point 10 onto the actual point 10:
    offset = actual[10] - nominal[10]

Step 3: Since it's a rigid translation, add that SAME offset (x and y) to
every other supposed-to-be point too:
    new_supposed[pid] = nominal[pid] + offset   for all pid in 1..19

Step 4: Save these new positions to a new file — the translated version of
the "supposed to be" points.

Output:
    calib_points_supposed_translated.json
"""

import json
import os

BASE_DIR = "/home/divuthejo/Star_nose_sensor/Integration_2"
MOCAP_JSON = os.path.join(BASE_DIR, "calib_points_short_6mm.json")
OUT_JSON = os.path.join(BASE_DIR, "calib_points_supposed_translated.json")

ALIGN_ON = 10

# "Supposed to be" (nominal) points — from calibrate_points.py
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
        nom_x, nom_y = POINTS[pid]
        dx, dy = off.get("dx_mm", 0.0), off.get("dy_mm", 0.0)
        actual[pid] = (nom_x + gx + dx, nom_y + gy + dy)
    return actual


def main():
    actual = load_actual_points(MOCAP_JSON)

    # Step 2: translation vector from supposed point 10 -> actual point 10
    nom_x10, nom_y10 = POINTS[ALIGN_ON]
    act_x10, act_y10 = actual[ALIGN_ON]
    offset_x = act_x10 - nom_x10
    offset_y = act_y10 - nom_y10
    print(f"offset = actual[{ALIGN_ON}] - nominal[{ALIGN_ON}] = "
          f"({offset_x:+.4f}, {offset_y:+.4f}) mm")

    # Step 3: apply the SAME offset to every supposed-to-be point
    new_supposed = {
        pid: (round(x + offset_x, 4), round(y + offset_y, 4))
        for pid, (x, y) in POINTS.items()
    }

    print(f"\n{'pid':>4} {'nominal':>18} {'new (translated)':>20}")
    for pid in sorted(POINTS):
        nx, ny = POINTS[pid]
        tx, ty = new_supposed[pid]
        print(f"{pid:>4} ({nx:+7.2f},{ny:+7.2f}) -> ({tx:+7.2f},{ty:+7.2f})")

    # Step 4: save to a new file
    out = {
        "offset_mm": {"x_mm": offset_x, "y_mm": offset_y},
        "aligned_on_point": ALIGN_ON,
        "points": {str(pid): {"x_mm": x, "y_mm": y} for pid, (x, y) in new_supposed.items()},
    }
    with open(OUT_JSON, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved -> {OUT_JSON}")


if __name__ == "__main__":
    main()
