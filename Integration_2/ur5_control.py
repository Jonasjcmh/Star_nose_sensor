"""
ur5_control.py
UR5 RTDE trajectory — hexagonal grid.
Includes X/Y calibration offset to correct TCP misalignment.
"""
import os
import math
import socket
import time
import threading
import rtde_control
import rtde_receive

# Override with UR_ROBOT_IP env var to connect to URSim:
#   UR_ROBOT_IP=127.0.0.1 python main.py --sim
ROBOT_IP        = os.environ.get("UR_ROBOT_IP", "177.22.22.2")
VELOCITY_TRAVEL = 0.05
VELOCITY_PRESS  = 0.01
ACCELERATION    = 0.3

DEFAULT_INDENT_MM = 6.00
DEFAULT_DWELL_S   = 1.5
POINT_OVERRIDES   = {}

SAFE_HOME_Z_MM    = 30.0   # clearance above surface at home position

# ── Calibration offset ───────────────────────────────────────
# Adjust these to correct TCP misalignment.
# Run calibrate_ur5.py to find the correct values.
# Positive X = move right, Positive Y = move forward
CALIB_X_MM    = 0.0   # ← set after calibration
CALIB_Y_MM    = 0.0   # ← set after calibration
CALIB_Z_MM    = 0.0   # ← set after calibration (surface height)
POINT_OFFSETS = {}    # pt → (dx_mm, dy_mm) from calib_points_<tip>.json

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

# Corrected mapping: UR5 point → raw sensor cell index
# Sensor is physically mounted 120° CCW relative to robot frame.
UR5_TO_SENSOR = {
    1:24,  2:12,  3:0,
    4:37,  5:25,  6:13,  7:1,
    8:50,  9:38,  10:26, 11:14, 12:2,
    13:51, 14:39, 15:27, 16:15,
    17:52, 18:40, 19:28,
}

SEQUENCE = [10,1,2,3,7,6,5,4,8,9,10,11,12,16,15,14,13,17,18,19,10]

# ── Shared state ─────────────────────────────────────────────
current_point = None
is_pressing   = False
is_done       = False
current_ft    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Fx Fy Fz Tx Ty Tz
current_tcp   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x y z rx ry rz
current_ai0   = 0.0  # standard analog input 0 (AI0), volts
_lock         = threading.Lock()
_rtde_r_ref   = [None]  # shared reference to rtde_receive
_stop_flag    = threading.Event()  # set to request early stop + home return

def request_stop():
    """Signal the trajectory to stop after the current point and return home."""
    _stop_flag.set()

def get_state():
    with _lock:
        return {
            'point':    current_point,
            'pressing': is_pressing,
            'done':     is_done,
            'ft':       list(current_ft),
            'tcp':      list(current_tcp),
            'ai0':      current_ai0,
        }

def get_force():
    """Get current TCP force [Fx, Fy, Fz, Tx, Ty, Tz] in N/Nm"""
    with _lock:
        return list(current_ft)

def get_tcp():
    """Get current TCP pose [x, y, z, rx, ry, rz]"""
    with _lock:
        return list(current_tcp)

def get_ai0():
    """Get current standard analog input 0 (AI0) in volts"""
    with _lock:
        return current_ai0

def _force_reader_loop():
    """Read force/pose/AI0 at 125Hz in background"""
    global current_ft, current_tcp, current_ai0
    while True:
        rtde_r = _rtde_r_ref[0]
        if rtde_r is not None:
            try:
                ft  = rtde_r.getActualTCPForce()
                tcp = rtde_r.getActualTCPPose()
                ai0 = rtde_r.getStandardAnalogInput0()
                with _lock:
                    current_ft  = list(ft)
                    current_tcp = list(tcp)
                    current_ai0 = float(ai0)
            except Exception:
                pass
        time.sleep(0.008)  # 125Hz

def set_calibration(x_mm=0.0, y_mm=0.0, z_mm=0.0):
    """Update global calibration offsets at runtime."""
    global CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM
    CALIB_X_MM = x_mm
    CALIB_Y_MM = y_mm
    CALIB_Z_MM = z_mm
    print(f"[ur5] Calibration set: X={x_mm:+.3f} Y={y_mm:+.3f} Z={z_mm:+.3f} mm")

def set_point_offsets(offsets):
    """Load per-point (dx, dy) offsets from calib_points_<tip>.json."""
    global POINT_OFFSETS
    POINT_OFFSETS = offsets
    print(f"[ur5] Per-point offsets loaded for {len(offsets)} point(s)")

def _home_pose():
    """Safe park position: P10 center lifted SAFE_HOME_Z_MM above the mat."""
    return _build_pose(10, SAFE_HOME_Z_MM)

def _return_home(rtde_c):
    """Move to safe home position. Called on every exit path."""
    try:
        print("[ur5] Returning to home position...")
        rtde_c.moveL(_home_pose(), VELOCITY_TRAVEL, ACCELERATION)
        print("[ur5] At home position")
    except Exception as e:
        print(f"[ur5] Could not return to home: {e}")

def _get_indent_dwell(pt):
    return POINT_OVERRIDES.get(pt, (DEFAULT_INDENT_MM, DEFAULT_DWELL_S))

def _build_pose(pt, extra_z_mm=0.0):
    dx, dy = POINTS[pt]
    pdx, pdy = POINT_OFFSETS.get(pt, (0.0, 0.0))
    pose = REFERENCE_POSE.copy()
    pose[0] += (dx + CALIB_X_MM + pdx) / 1000.0
    pose[1] += (dy + CALIB_Y_MM + pdy) / 1000.0
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

class _ScriptController:
    """Fallback motion controller via URScript on port 30002.

    Provides the same moveL / stopScript interface as RTDEControlInterface
    but works when RTDE Control cannot connect (e.g. URSim on Apple Silicon).
    The first move uses movej(get_inverse_kin()) to position the arm in the
    workspace; all subsequent moves use movel for Cartesian linear paths.
    """

    def __init__(self, ip, rtde_r):
        self._ip     = ip
        self._rtde_r = rtde_r
        self._first  = True   # first call: use IK movej to enter workspace

    def moveL(self, pose, vel, acc):
        ps = ", ".join(f"{v:.6f}" for v in pose)
        if self._first:
            script = f"movej(get_inverse_kin(p[{ps}]), a={acc:.4f}, v={vel:.4f})"
            self._first = False
        else:
            script = f"movel(p[{ps}], a={acc:.4f}, v={vel:.4f})"
        self._send(script)
        self._wait_stop()

    def stopScript(self):
        pass   # URScript commands are stateless; nothing to stop

    def _send(self, script):
        s = socket.socket()
        s.settimeout(5)
        s.connect((self._ip, 30002))
        s.recv(4096)          # discard welcome banner
        s.send((script + "\n").encode())
        s.close()

    def _wait_stop(self, timeout=30):
        """Poll TCP speed via RTDEReceive until robot stops moving."""
        time.sleep(0.3)       # let motion start
        stable = 0
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                spd = self._rtde_r.getActualTCPSpeed()
                mag = math.sqrt(sum(v**2 for v in spd[:3]))
                if mag < 0.003:
                    stable += 1
                    if stable >= 3:
                        return
                else:
                    stable = 0
            except Exception:
                pass
            time.sleep(0.05)


def run_trajectory(on_press=None, on_release=None, interactive=True):
    global is_done

    print("="*55)
    print(f"  UR5 Hex Trajectory | offset X={CALIB_X_MM:+.2f} Y={CALIB_Y_MM:+.2f} Z={CALIB_Z_MM:+.2f} mm")
    print("="*55)

    # ── RTDEReceive (always try first — works even on URSim) ──
    rtde_r = None
    for attempt in range(3):
        try:
            rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            print("[ur5] Receive connected")
            break
        except Exception as e:
            print(f"[ur5] Receive attempt {attempt+1}/3 failed: {e}")
            rtde_r = None
            time.sleep(2)

    if rtde_r is None:
        print("[ur5] Could not open RTDEReceive — skipping trajectory")
        with _lock:
            is_done = True
        return

    _rtde_r_ref[0] = rtde_r

    # Start background force reader
    force_thread = threading.Thread(
        target=_force_reader_loop, daemon=True)
    force_thread.start()
    print("[ur5] Force reader started at 125Hz")

    # ── RTDEControl (may fail on some sims → URScript fallback) ──
    rtde_c = None
    for attempt in range(3):
        try:
            print(f"[ur5] Control attempt {attempt+1}/3...")
            rtde_c = rtde_control.RTDEControlInterface(
                ROBOT_IP,
                frequency=500.0,
                flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT
            )
            print("[ur5] RTDE Control connected!")
            break
        except Exception as e:
            print(f"[ur5] Control attempt {attempt+1} failed: {e}")
            rtde_c = None
            time.sleep(2)

    if rtde_c is None:
        print("[ur5] RTDE Control unavailable — using URScript fallback")
        rtde_c = _ScriptController(ROBOT_IP, rtde_r)

    # ── Print current position ───────────────────────────────
    try:
        tcp = rtde_r.getActualTCPPose()
        print(f"[ur5] TCP: X={tcp[0]:.4f} Y={tcp[1]:.4f} Z={tcp[2]:.4f}")
        print(f"[ur5] Robot mode: {rtde_r.getRobotMode()}")
    except Exception as e:
        print(f"[ur5] Could not read TCP: {e}")

    # ── Move to home, then descend to start ──────────────────
    print("[ur5] Moving to home position...")
    try:
        rtde_c.moveL(_home_pose(), VELOCITY_TRAVEL, ACCELERATION)
        rtde_c.moveL(_build_pose(10, 0.0), VELOCITY_TRAVEL, ACCELERATION)
        print("[ur5] At P10 — starting trajectory")
    except Exception as e:
        print(f"[ur5] Failed to move to P10: {e}")
        try:
            rtde_c.stopScript()
        except:
            pass
        with _lock:
            is_done = True
        return

    # ── Execute sequence ──────────────────────────────────────
    total = len(SEQUENCE)
    _stop_flag.clear()
    try:
        for step, pt in enumerate(SEQUENCE, 1):
            if _stop_flag.is_set():
                print("[ur5] Stop requested — aborting trajectory")
                break
            try:
                _visit_point(rtde_c, step, total, pt, on_press, on_release)
            except Exception as e:
                print(f"[ur5] Error at P{pt}: {e}")
                break
        else:
            print("\n[ur5] Trajectory complete!")
    except KeyboardInterrupt:
        print("\n[ur5] Interrupted by user")
    finally:
        _return_home(rtde_c)
        try:
            rtde_c.stopScript()
        except:
            pass
        with _lock:
            is_done = True

if __name__ == "__main__":
    run_trajectory(interactive=True)