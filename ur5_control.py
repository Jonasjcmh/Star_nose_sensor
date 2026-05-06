"""
ur5_control.py
UR5 RTDE trajectory — hexagonal grid.
Includes X/Y calibration offset to correct TCP misalignment.
"""
import time
import threading
import rtde_control
import rtde_receive

ROBOT_IP        = "177.22.22.2"
VELOCITY_TRAVEL = 0.05
VELOCITY_PRESS  = 0.01
ACCELERATION    = 0.3

DEFAULT_INDENT_MM = 6.00
DEFAULT_DWELL_S   = 1.5
POINT_OVERRIDES   = {}

# ── Calibration offset ───────────────────────────────────────
# Adjust these to correct TCP misalignment.
# Run calibrate_ur5.py to find the correct values.
# Positive X = move right, Positive Y = move forward
CALIB_X_MM = 0.0   # ← set after calibration
CALIB_Y_MM = 0.0   # ← set after calibration
CALIB_Z_MM = 0.0   # ← set after calibration (surface height)

REFERENCE_POSE = [
    -0.03746+0.0005,
    -0.50066+0.0016,
     0.06054,
    -2.35063, 2.08341, -0.00009
]

# Correct physical positions matching sensor layout
# P10 = center (0,0), all others relative in mm
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
    10: (  0.0,   0.0),   # center
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

# Verified mapping: UR5 point → raw sensor cell index
UR5_TO_SENSOR = {
    1:2,   2:15,  3:28,
    4:1,   5:14,  6:27,  7:40,
    8:0,   9:13,  10:26, 11:39, 12:52,
    13:12, 14:25, 15:38, 16:51,
    17:24, 18:37, 19:50,
}

SEQUENCE = [10,1,2,3,7,6,5,4,8,9,10,11,12,16,15,14,13,17,18,19,10]

# ── Shared state ─────────────────────────────────────────────
current_point = None
is_pressing   = False
is_done       = False
current_ft    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Fx Fy Fz Tx Ty Tz
current_tcp   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x y z rx ry rz
_lock         = threading.Lock()
_rtde_r_ref   = [None]  # shared reference to rtde_receive

def get_state():
    with _lock:
        return {
            'point':    current_point,
            'pressing': is_pressing,
            'done':     is_done,
            'ft':       list(current_ft),
            'tcp':      list(current_tcp),
        }

def get_force():
    """Get current TCP force [Fx, Fy, Fz, Tx, Ty, Tz] in N/Nm"""
    with _lock:
        return list(current_ft)

def get_tcp():
    """Get current TCP pose [x, y, z, rx, ry, rz]"""
    with _lock:
        return list(current_tcp)

def _force_reader_loop():
    """Read force/pose at 125Hz in background"""
    global current_ft, current_tcp
    while True:
        rtde_r = _rtde_r_ref[0]
        if rtde_r is not None:
            try:
                ft  = rtde_r.getActualTCPForce()
                tcp = rtde_r.getActualTCPPose()
                with _lock:
                    current_ft  = list(ft)
                    current_tcp = list(tcp)
            except Exception:
                pass
        time.sleep(0.008)  # 125Hz

def set_calibration(x_mm=0.0, y_mm=0.0, z_mm=0.0):
    """Update calibration offsets at runtime"""
    global CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM
    CALIB_X_MM = x_mm
    CALIB_Y_MM = y_mm
    CALIB_Z_MM = z_mm
    print(f"[ur5] Calibration set: X={x_mm:+.3f} Y={y_mm:+.3f} Z={z_mm:+.3f} mm")

def _get_indent_dwell(pt):
    return POINT_OVERRIDES.get(pt, (DEFAULT_INDENT_MM, DEFAULT_DWELL_S))

def _build_pose(pt, extra_z_mm=0.0):
    dx, dy = POINTS[pt]
    pose = REFERENCE_POSE.copy()
    # Apply point offset + calibration correction
    pose[0] += (dx + CALIB_X_MM) / 1000.0
    pose[1] += (dy + CALIB_Y_MM) / 1000.0
    pose[2] += (extra_z_mm + CALIB_Z_MM) / 1000.0
    return pose

def _visit_point(rtde_c, step, total, pt, on_press=None, on_release=None):
    global current_point, is_pressing
    indent_mm, dwell_s = _get_indent_dwell(pt)
    surface = _build_pose(pt, 0.0)
    pressed = _build_pose(pt, -indent_mm)
    print(f"  [{step:02d}/{total}] P{pt:02d} sensor=S{UR5_TO_SENSOR[pt]} "
          f"XY=({POINTS[pt][0]:+.0f},{POINTS[pt][1]:+.0f})mm")

    with _lock: current_point=pt; is_pressing=False
    rtde_c.moveL(surface, VELOCITY_TRAVEL, ACCELERATION)
    with _lock: is_pressing=True
    rtde_c.moveL(pressed, VELOCITY_PRESS, ACCELERATION)
    if on_press: on_press(pt)
    time.sleep(dwell_s)
    rtde_c.moveL(surface, VELOCITY_PRESS, ACCELERATION)
    with _lock: is_pressing=False
    if on_release: on_release(pt)

def run_trajectory(on_press=None, on_release=None, interactive=True):
    global is_done

    print("="*55)
    print(f"  UR5 Hex Trajectory | offset X={CALIB_X_MM:+.2f} Y={CALIB_Y_MM:+.2f} Z={CALIB_Z_MM:+.2f} mm")
    print("="*55)

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    _rtde_r_ref[0] = rtde_r

    # Start background force reader
    force_thread = threading.Thread(
        target=_force_reader_loop, daemon=True)
    force_thread.start()
    print("[ur5] Force reader started at 125Hz")

    # ── Connect with timeout ──────────────────────────────────
    rtde_c = None
    rtde_r = None

    for attempt in range(3):
        try:
            print(f"[ur5] Connecting attempt {attempt+1}/3...")
            rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            print(f"[ur5] Receive OK")
            rtde_c = rtde_control.RTDEControlInterface(
                ROBOT_IP,
                frequency=500.0,
                flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT
            )
            print(f"[ur5] Control OK!")
            break
        except Exception as e:
            print(f"[ur5] Attempt {attempt+1} failed: {e}")
            try:
                if rtde_r: rtde_r.disconnect()
            except:
                pass
            rtde_r = None
            rtde_c = None
            time.sleep(2)

    if rtde_c is None:
        print("[ur5] Could not connect — skipping trajectory")
        with _lock:
            is_done = True
        return

    # ── Print current position ───────────────────────────────
    try:
        tcp = rtde_r.getActualTCPPose()
        print(f"[ur5] TCP: X={tcp[0]:.4f} Y={tcp[1]:.4f} Z={tcp[2]:.4f}")
        print(f"[ur5] Robot mode: {rtde_r.getRobotMode()}")
    except Exception as e:
        print(f"[ur5] Could not read TCP: {e}")

    # ── Move to start ─────────────────────────────────────────
    print("[ur5] Moving to P10 (center)...")
    try:
        rtde_c.moveL(_build_pose(10, 0.0), VELOCITY_TRAVEL, ACCELERATION)
        print("[ur5] At P10 — starting trajectory")
    except Exception as e:
        print(f"[ur5] Failed to move to P10: {e}")
        rtde_c.stopScript()
        with _lock:
            is_done = True
        return

    # ── Execute sequence ──────────────────────────────────────
    total = len(SEQUENCE)
    for step, pt in enumerate(SEQUENCE, 1):
        try:
            _visit_point(rtde_c, step, total, pt, on_press, on_release)
        except Exception as e:
            print(f"[ur5] Error at P{pt}: {e}")
            break

    try:
        rtde_c.stopScript()
    except:
        pass

    with _lock:
        is_done = True
    print("\n[ur5] Trajectory complete!")

if __name__ == "__main__":
    run_trajectory(interactive=True)