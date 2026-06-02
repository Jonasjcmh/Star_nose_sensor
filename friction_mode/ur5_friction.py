"""
ur5_friction.py
UR5 control for the friction mode experiment.

Two execution modes
───────────────────
  displacement  Robot moves along an XY trajectory at a fixed Z depth
                (pure position control, no force feedback during motion).

  force         Robot engages until the FUTEK load cell reads the target
                contact force, then follows the XY trajectory while a
                per-waypoint P-controller adjusts Z to maintain that force.

Public API
──────────
  run_displacement_trajectory(pts, depth_mm, speed_mps, on_waypoint)
  run_force_trajectory(pts, target_N, speed_mps, on_waypoint)
  get_state()  →  dict compatible with data_logger.log()
  get_force()  →  [Fx, Fy, Fz, Tx, Ty, Tz]
  request_stop()
  set_calibration(x_mm, y_mm, z_mm)
"""

import os
import math
import time
import threading

import rtde_control
import rtde_receive

# ── Connection ────────────────────────────────────────────────────────────────
ROBOT_IP = os.environ.get("UR_ROBOT_IP", "177.22.22.2")

# ── Motion parameters ─────────────────────────────────────────────────────────
VELOCITY_HOME    = 0.08    # m/s  — fast travel to/from home
VELOCITY_ENGAGE  = 0.004   # m/s  — slow downward push until contact
VELOCITY_SLIDE   = 0.008   # m/s  — default lateral sliding speed
ACCELERATION     = 0.3     # m/s²

# ── Force control parameters ──────────────────────────────────────────────────
FUTEK_ZERO_V   = 5.0
FUTEK_N_PER_V  = 44.482 / 5.0       # 8.896 N/V  (10 lb / 5 V rated)

FORCE_KP_MM_PER_N = 0.4             # P-gain: mm of Z correction per N of error
FORCE_MAX_DZ_MM   = 1.5             # maximum Z correction per waypoint
MAX_ENGAGE_MM     = 15.0            # hard safety limit — never go deeper than this

# ── Reference pose (matches ur5_control.py) ───────────────────────────────────
REFERENCE_POSE = [
    -0.03746 + 0.0005,
    -0.50066 + 0.0016,
     0.06054,
    -2.35063, 2.08341, -0.00009,
]
SAFE_HOME_Z_MM = 30.0   # clearance above surface at home

# ── Calibration offsets (set via set_calibration) ─────────────────────────────
CALIB_X_MM = 0.0
CALIB_Y_MM = 0.0
CALIB_Z_MM = 0.0

# ── Shared state (read by logger at 20 Hz) ────────────────────────────────────
current_point = None
is_sliding    = False
is_done       = False
current_ft    = [0.0] * 6
current_tcp   = [0.0] * 6
current_ai0   = 0.0
current_mode  = 'idle'    # 'displacement' | 'force' | 'idle'

_lock       = threading.Lock()
_rtde_r_ref = [None]
_stop_flag  = threading.Event()


# ── Public helpers ────────────────────────────────────────────────────────────

def request_stop():
    """Ask the trajectory to stop after the current waypoint and return home."""
    _stop_flag.set()


def set_calibration(x_mm=0.0, y_mm=0.0, z_mm=0.0):
    """Apply XYZ calibration offset (same values as ur5_control.py)."""
    global CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM
    CALIB_X_MM, CALIB_Y_MM, CALIB_Z_MM = x_mm, y_mm, z_mm
    print(f"[ur5] Calibration: X={x_mm:+.3f}  Y={y_mm:+.3f}  Z={z_mm:+.3f} mm")


def get_state():
    """Return dict compatible with data_logger.log() ur5_state argument."""
    with _lock:
        return {
            'point':    current_point,
            'pressing': is_sliding,
            'done':     is_done,
            'ft':       list(current_ft),
            'tcp':      list(current_tcp),
            'ai0':      current_ai0,
        }


def get_force():
    """Return [Fx, Fy, Fz, Tx, Ty, Tz] in N / N·m."""
    with _lock:
        return list(current_ft)


def ai0_to_N(v):
    """Convert AI0 voltage reading to FUTEK load cell force in Newtons."""
    return -(v - FUTEK_ZERO_V) * FUTEK_N_PER_V


# ── Trajectory entry points ───────────────────────────────────────────────────

def run_displacement_trajectory(trajectory_pts, depth_mm,
                                 speed_mps=None, on_waypoint=None):
    """
    Move along trajectory_pts at a fixed Z indentation depth.

    Parameters
    ----------
    trajectory_pts : list of (x_mm, y_mm)
    depth_mm       : positive depth below the surface (mm)
    speed_mps      : lateral sliding speed (m/s); default VELOCITY_SLIDE
    on_waypoint    : optional callback(waypoint_index, x_mm, y_mm, z_mm)
    """
    _execute(trajectory_pts,
             mode='displacement',
             depth_mm=depth_mm,
             target_N=None,
             speed_mps=speed_mps or VELOCITY_SLIDE,
             on_waypoint=on_waypoint)


def run_force_trajectory(trajectory_pts, target_N,
                          speed_mps=None, on_waypoint=None):
    """
    Move along trajectory_pts while maintaining a constant FUTEK contact force.

    The robot first descends slowly until the load cell reads target_N,
    then executes lateral motion. A P-controller adjusts Z before each
    waypoint move to compensate for surface height variation.

    Parameters
    ----------
    trajectory_pts : list of (x_mm, y_mm)
    target_N       : desired contact force from FUTEK load cell (N)
    speed_mps      : lateral sliding speed (m/s); default VELOCITY_SLIDE
    on_waypoint    : optional callback(waypoint_index, x_mm, y_mm, z_mm)
    """
    _execute(trajectory_pts,
             mode='force',
             depth_mm=None,
             target_N=target_N,
             speed_mps=speed_mps or VELOCITY_SLIDE,
             on_waypoint=on_waypoint)


# ── Internal implementation ───────────────────────────────────────────────────

def _build_pose(x_mm, y_mm, z_extra_mm=0.0):
    """Build a 6-DOF TCP pose from sensor-relative XY coordinates."""
    pose = list(REFERENCE_POSE)
    pose[0] += (x_mm + CALIB_X_MM) / 1000.0
    pose[1] += (y_mm + CALIB_Y_MM) / 1000.0
    pose[2] += (z_extra_mm + CALIB_Z_MM) / 1000.0
    return pose


def _home_pose():
    return _build_pose(0.0, 0.0, SAFE_HOME_Z_MM)


def _return_home(rtde_c):
    try:
        print("[ur5] Returning to home...")
        rtde_c.moveL(_home_pose(), VELOCITY_HOME, ACCELERATION)
        print("[ur5] At home")
    except Exception as e:
        print(f"[ur5] Home failed: {e}")


def _force_reader_loop():
    """Background thread — update shared state at ~125 Hz."""
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
        time.sleep(0.008)


def _measure_lc_baseline(rtde_r, n_samples=60):
    """
    Average FUTEK AI0 voltage over n_samples to get a rest-state baseline.
    Called before the robot contacts the surface.
    """
    buf = []
    for _ in range(n_samples):
        try:
            buf.append(rtde_r.getStandardAnalogInput0())
        except Exception:
            pass
        time.sleep(0.02)
    mean_v  = sum(buf) / len(buf) if buf else FUTEK_ZERO_V
    baseline = ai0_to_N(mean_v)
    print(f"[ur5] FUTEK baseline: {mean_v:.4f} V  →  {baseline:.3f} N (raw, ~0 after zero-ref)")
    return baseline


def _engage_displacement(rtde_c, x_mm, y_mm, depth_mm):
    """Approach surface then push down to depth_mm. Returns depth_mm."""
    print(f"[ur5] Approaching surface at ({x_mm:+.1f}, {y_mm:+.1f}) mm ...")
    rtde_c.moveL(_build_pose(x_mm, y_mm, 0.0), VELOCITY_HOME, ACCELERATION)
    print(f"[ur5] Engaging to {depth_mm:.1f} mm depth ...")
    rtde_c.moveL(_build_pose(x_mm, y_mm, -depth_mm), VELOCITY_ENGAGE, ACCELERATION)
    return depth_mm


def _engage_force(rtde_c, rtde_r, x_mm, y_mm, target_N, lc_baseline_N):
    """
    Descend slowly until FUTEK reads (lc_baseline_N + target_N).
    Returns the depth (mm) at which engagement was achieved.
    """
    print(f"[ur5] Approaching ({x_mm:+.1f}, {y_mm:+.1f}) mm ...")
    rtde_c.moveL(_build_pose(x_mm, y_mm, 0.0), VELOCITY_HOME, ACCELERATION)
    print(f"[ur5] Engaging until FUTEK = {target_N:.1f} N ...")

    z_mm = 0.0
    step = 0.15   # mm per descent step (fine enough to not overshoot)
    while z_mm < MAX_ENGAGE_MM:
        if _stop_flag.is_set():
            break
        z_mm += step
        rtde_c.moveL(_build_pose(x_mm, y_mm, -z_mm), VELOCITY_ENGAGE, ACCELERATION)

        ai0 = rtde_r.getStandardAnalogInput0()
        f   = ai0_to_N(ai0) - lc_baseline_N
        if f >= target_N:
            print(f"[ur5] Contact at z = {z_mm:.2f} mm,  FUTEK = {f:.2f} N")
            return z_mm

        time.sleep(0.02)

    print(f"[ur5] WARNING: target {target_N:.1f} N not reached within {MAX_ENGAGE_MM} mm — "
          "check calibration or increase MAX_ENGAGE_MM")
    return z_mm


def _execute(trajectory_pts, mode, depth_mm, target_N, speed_mps, on_waypoint):
    global is_done, is_sliding, current_point, current_mode

    with _lock:
        is_done  = False
        current_mode = mode

    # ── Connect RTDEReceive ────────────────────────────────────
    rtde_r = None
    for attempt in range(3):
        try:
            rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            print("[ur5] Receive connected")
            break
        except Exception as e:
            print(f"[ur5] Receive {attempt + 1}/3 failed: {e}")
            time.sleep(2)
    if rtde_r is None:
        print("[ur5] Cannot connect to robot — aborting")
        with _lock:
            is_done = True
        return

    _rtde_r_ref[0] = rtde_r
    threading.Thread(target=_force_reader_loop, daemon=True).start()
    print("[ur5] Force/AI0 reader started at ~125 Hz")

    # ── Connect RTDEControl ────────────────────────────────────
    rtde_c = None
    for attempt in range(3):
        try:
            rtde_c = rtde_control.RTDEControlInterface(
                ROBOT_IP, frequency=500.0,
                flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
            print("[ur5] Control connected")
            break
        except Exception as e:
            print(f"[ur5] Control {attempt + 1}/3 failed: {e}")
            time.sleep(2)
    if rtde_c is None:
        print("[ur5] RTDE Control unavailable — aborting")
        with _lock:
            is_done = True
        return

    _stop_flag.clear()

    try:
        # ── Home ──────────────────────────────────────────────
        print("[ur5] Moving to home position ...")
        rtde_c.moveL(_home_pose(), VELOCITY_HOME, ACCELERATION)

        x0, y0 = trajectory_pts[0]

        # ── Initial engagement ─────────────────────────────────
        if mode == 'displacement':
            z_current = _engage_displacement(rtde_c, x0, y0, depth_mm)
            lc_baseline_N = None
        else:
            # Measure FUTEK baseline before contact
            print("[ur5] Measuring FUTEK baseline (robot in air) ...")
            lc_baseline_N = _measure_lc_baseline(rtde_r)
            z_current = _engage_force(rtde_c, rtde_r, x0, y0,
                                       target_N, lc_baseline_N)

        if _stop_flag.is_set():
            return

        # ── Execute trajectory ─────────────────────────────────
        n = len(trajectory_pts)
        print(f"\n[ur5] Starting {mode} trajectory — {n} waypoints  "
              f"speed = {speed_mps*1000:.1f} mm/s")

        with _lock:
            is_sliding    = True
            current_point = 'friction'

        for i, (x_mm, y_mm) in enumerate(trajectory_pts):
            if _stop_flag.is_set():
                print("[ur5] Stop requested — ending trajectory")
                break

            # Force mode: adjust Z before each lateral move
            if mode == 'force':
                ai0 = rtde_r.getStandardAnalogInput0()
                f   = ai0_to_N(ai0) - lc_baseline_N
                error = target_N - f                     # positive → need more depth
                dz = FORCE_KP_MM_PER_N * error
                dz = max(-FORCE_MAX_DZ_MM, min(FORCE_MAX_DZ_MM, dz))
                z_current = max(0.0, min(MAX_ENGAGE_MM, z_current + dz))

            pose = _build_pose(x_mm, y_mm, -z_current)
            rtde_c.moveL(pose, speed_mps, ACCELERATION)

            with _lock:
                current_point = f'{i + 1}/{n}'

            if on_waypoint:
                on_waypoint(i, x_mm, y_mm, z_current)

        print(f"[ur5] Trajectory complete ({n} waypoints)")

    except KeyboardInterrupt:
        print("[ur5] Interrupted by user")
    finally:
        with _lock:
            is_sliding = False
        _return_home(rtde_c)
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        with _lock:
            is_done = True


if __name__ == "__main__":
    import sys
    print("ur5_friction.py — run via main_friction.py")
    sys.exit(0)
