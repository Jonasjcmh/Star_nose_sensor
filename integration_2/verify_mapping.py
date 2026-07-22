"""
verify_mapping.py
Verifies that UR5 point numbers correctly map to sensor indices.
Run this BEFORE the full experiment to confirm the mapping.

HOW TO USE:
1. Run this script
2. It will command UR5 to press each point one by one
3. Watch the terminal — it shows which sensor index activates most
4. Compare against expected mapping
"""

import time
import threading
import sensor
import rtde_control
import rtde_receive

# ── UR5 config (copied exactly from ur5_control.py) ──────────
ROBOT_IP        = "177.22.22.2"
VELOCITY_TRAVEL = 0.05
VELOCITY_PRESS  = 0.01
ACCELERATION    = 0.3
DEFAULT_INDENT  = 6.0   # mm
DWELL_S         = 2.0   # hold longer so we can read sensor

REFERENCE_POSE = [
    -0.03695, -0.49906,
     0.06054, 2.352, -2.08341, -0.00009
]

POINTS = {
     1: ( -8.0, +14.0),
     2: (  0.0, +14.0),
     3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),
     5: ( -4.0,  +7.0),
     6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),
     8: (-16.0,   0.0),
     9: ( -8.0,   0.0),
    10: (  0.0,   0.0),
    11: ( +8.0,   0.0),
    12: (+16.0,   0.0),
    13: (-12.0,  -7.0),
    14: ( -4.0,  -7.0),
    15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),
    17: ( -8.0, -14.0),
    18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

def build_pose(pt, extra_z_mm=0.0):
    dx, dy = POINTS[pt]
    pose = REFERENCE_POSE.copy()
    pose[0] += dx / 1000.0
    pose[1] += dy / 1000.0
    pose[2] += extra_z_mm / 1000.0
    return pose

def get_active_sensors(values, threshold=0.1):
    """Return sorted list of (sensor_idx, value) above threshold."""
    active = [(i, v) for i, v in enumerate(values) if v > threshold]
    return sorted(active, key=lambda x: -x[1])

def main():
    print("=" * 55)
    print("  UR5 ↔ Sensor Mapping Verification")
    print("=" * 55)

    # Start sensor
    print("\n[verify] Starting sensor...")
    sensor.start()
    print("[verify] Waiting for sensor calibration...")
    sensor.wait_until_ready(timeout=30)
    print("[verify] Sensor ready!\n")

    # Connect UR5
    print("[verify] Connecting to UR5...")
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    print("[verify] Connected!\n")

    # Move to centre first
    input("Press ENTER to move to P10 (centre)...")
    rtde_c.moveL(REFERENCE_POSE, VELOCITY_TRAVEL, ACCELERATION)
    print("At P10.\n")

    # Results table
    results = {}

    input("Press ENTER to start verification sequence...")
    print()

    # Test each point
    for pt in range(1, 20):
        surface = build_pose(pt, 0.0)
        pressed = build_pose(pt, -DEFAULT_INDENT)

        print(f"── P{pt:02d} ({POINTS[pt][0]:+.0f},{POINTS[pt][1]:+.0f})mm ──")

        # Travel to point
        rtde_c.moveL(surface, VELOCITY_TRAVEL, ACCELERATION)

        # Press
        rtde_c.moveL(pressed, VELOCITY_PRESS, ACCELERATION)

        # Read sensor while pressing
        time.sleep(0.5)  # settle
        readings = []
        for _ in range(5):
            readings.append(sensor.get_values())
            time.sleep(0.1)

        # Average readings
        avg = [sum(r[i] for r in readings)/len(readings) for i in range(19)]
        active = get_active_sensors(avg, threshold=0.05)

        if active:
            top_idx, top_val = active[0]
            print(f"  → Top sensor: index {top_idx} (val={top_val:.3f})")
            print(f"  → All active: {[(i,round(v,3)) for i,v in active[:4]]}")
            results[pt] = top_idx
        else:
            print(f"  → No sensor activated!")
            results[pt] = None

        # Retract
        rtde_c.moveL(surface, VELOCITY_PRESS, ACCELERATION)
        print()

    # Return home
    rtde_c.moveL(REFERENCE_POSE, VELOCITY_TRAVEL, ACCELERATION)
    rtde_c.stopScript()

    # Print mapping summary
    print("\n" + "=" * 55)
    print("  MAPPING RESULTS")
    print("  Copy this into ur5_control.py and sofa_scene.py")
    print("=" * 55)
    print("\nUR5_TO_SENSOR = {")
    for pt in range(1, 20):
        idx = results.get(pt)
        dx, dy = POINTS[pt]
        print(f"    {pt:2d}: {str(idx):4s},  # P{pt:02d} ({dx:+.0f},{dy:+.0f})mm")
    print("}")

if __name__ == "__main__":
    main()