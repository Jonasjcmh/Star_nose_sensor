"""
ur5_imu.py
UR5 control for the inertial (IMU) test — position control ONLY.

Stripped-down twin of friction_mode/ur5_friction.py:
    • NO capacitive sensor
    • NO FUTEK load cell / force feedback
    • NO calibration profile / XY-Z offsets

The robot simply carries the end-effector (IMU) through an XY trajectory in a
horizontal plane at a fixed, safe height above the reference pose. Motion is
pure position control (moveL per waypoint) — nothing is pressed into anything.

Public API
──────────
  run_trajectory(pts, height_mm, speed_mps, on_waypoint)
  get_state()      → dict {tcp, moving, done, wp, wp_total}
  request_stop()
"""
import os
import time
import threading

import numpy as np
import rtde_control
import rtde_receive

from imu_trajectories import clamp_to_area, WORKAREA_MAX_MM

# ── Connection ────────────────────────────────────────────────────────────────
ROBOT_IP = os.environ.get("UR_ROBOT_IP", "177.22.22.2")

# ── Motion parameters ─────────────────────────────────────────────────────────
VELOCITY_TRAVEL = 0.08     # m/s — fast travel to/from home and between heights
VELOCITY_MOVE   = 0.03     # m/s — default trajectory speed (30 mm/s)
ACCELERATION    = 0.3      # m/s²

# ── Working plane (free-air motion above the reference pose) ───────────────────
# The IMU test moves in air, so the "work height" is a clearance ABOVE the
# reference pose rather than a press depth below a surface.
DEFAULT_HEIGHT_MM = 30.0   # plane in which the trajectory is executed
TRAVEL_HEIGHT_MM  = 60.0   # extra-clear height used to travel to the start point

# ── Reference pose (centre of the working area; matches ur5_friction.py) ───────
REFERENCE_POSE = [
    0.06674,
    -0.52282,
     0.05258,
    0.201, 3.133, 0.004,
]

# ── Yaw control ───────────────────────────────────────────────────────────────
# Imported trajectories may carry a per-waypoint yaw (theta, deg). When enabled,
# the tool is rotated about the base VERTICAL (Z) axis by (yaw - YAW_REF_DEG),
# composed with the reference orientation. YAW_REF_DEG shifts the zero so you can
# align the trajectory frame with the robot if needed.
APPLY_YAW   = True
YAW_REF_DEG = 0.0

# ── Shared state (read by the live viewer / logger) ────────────────────────────
_lock  = threading.Lock()
_state = {
    'tcp':      [0.0] * 6,   # actual TCP pose [x,y,z, rx,ry,rz] (m / rad)
    'moving':   False,       # True while executing the trajectory
    'done':     False,       # True once the run has finished / returned home
    'wp':       0,           # current waypoint index (1-based)
    'wp_total': 0,           # total waypoints in the active trajectory
    'yaw':      None,        # target yaw (deg) of the current waypoint, if any
}

_rtde_r_ref = [None]
_stop_flag  = threading.Event()

# Live central-point offset (mm), applied to EVERY waypoint. Adjustable at
# runtime from the visualizer (arrow keys / mouse) exactly like friction_live.
_center = [0.0, 0.0]

# Live yaw offset (deg) — a MANUAL initial orientation set from the visualizer.
# Every waypoint's yaw is applied as (waypoint_yaw + _yaw_offset), so you fix
# the starting orientation by hand and the tool then follows the trajectory's
# yaw relative to it.
_yaw_offset = [0.0]

# Half-width (mm) of the per-waypoint safety clamp. Defaults to the 23 cm area
# but is widened by run_trajectory() for absolute/calibrated (UR-frame) runs.
_clamp_half = [WORKAREA_MAX_MM / 2.0]

# Live trajectory speed (m/s) and work-plane height (mm) — read fresh each
# waypoint so they can be regulated from the visualizer console while running.
_speed  = [VELOCITY_MOVE]
_height = [DEFAULT_HEIGHT_MM]


def set_speed(mps):
    """Set the live trajectory speed (m/s)."""
    with _lock:
        _speed[0] = max(0.001, float(mps))


def get_speed():
    with _lock:
        return _speed[0]


def set_height_mm(mm):
    """Set the live work-plane height above the reference pose (mm)."""
    with _lock:
        _height[0] = float(mm)


def get_height_mm():
    with _lock:
        return _height[0]


# ── Public helpers ────────────────────────────────────────────────────────────

def request_stop():
    """Ask the trajectory to stop after the current waypoint and return home."""
    _stop_flag.set()


def get_state():
    """Snapshot of the shared robot state (thread-safe copy)."""
    with _lock:
        return {
            'tcp':      list(_state['tcp']),
            'moving':   _state['moving'],
            'done':     _state['done'],
            'wp':       _state['wp'],
            'wp_total': _state['wp_total'],
            'yaw':      _state['yaw'],
        }


# ── Live central-point offset (set from the visualizer at run time) ────────────

def set_center_offset(x_mm, y_mm):
    """Set the absolute XY offset (mm) applied to the whole trajectory."""
    with _lock:
        _center[0] = float(x_mm)
        _center[1] = float(y_mm)


def nudge_center(dx_mm, dy_mm):
    """Shift the central point by (dx, dy) mm (relative)."""
    with _lock:
        _center[0] += float(dx_mm)
        _center[1] += float(dy_mm)


def get_center_offset():
    """Return the current [x_mm, y_mm] central-point offset."""
    with _lock:
        return list(_center)


def set_yaw_offset(deg):
    """Set the manual initial-yaw offset (deg) added to every waypoint yaw."""
    with _lock:
        _yaw_offset[0] = float(deg)


def get_yaw_offset():
    """Return the current manual yaw offset (deg)."""
    with _lock:
        return _yaw_offset[0]


def _effective_yaw(base_yaw_deg):
    """Combine a waypoint's yaw with the manual offset.
    Returns None when there is nothing to apply (keeps reference orientation)."""
    off = get_yaw_offset()
    if base_yaw_deg is None:
        return off if (APPLY_YAW and abs(off) > 1e-9) else None
    return base_yaw_deg + off


# ── Orientation helpers (rotation-vector ↔ matrix, base-Z yaw) ─────────────────

def _rotvec_to_matrix(rv):
    """Axis-angle rotation vector → 3×3 rotation matrix (Rodrigues)."""
    rv = np.asarray(rv, dtype=float)
    ang = np.linalg.norm(rv)
    if ang < 1e-9:
        return np.eye(3)
    ax = rv / ang
    x, y, z = ax
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def _matrix_to_rotvec(R):
    """3×3 rotation matrix → axis-angle rotation vector."""
    ang = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if ang < 1e-9:
        return np.zeros(3)
    if abs(ang - np.pi) < 1e-6:                       # near 180°: use diagonal
        ax = np.sqrt(np.clip((np.diag(R) + 1.0) / 2.0, 0.0, None))
        return ang * ax / (np.linalg.norm(ax) or 1.0)
    ax = np.array([R[2, 1] - R[1, 2],
                   R[0, 2] - R[2, 0],
                   R[1, 0] - R[0, 1]]) / (2.0 * np.sin(ang))
    return ang * ax


# Reference orientation matrix, computed once from REFERENCE_POSE.
_R_REF = _rotvec_to_matrix(REFERENCE_POSE[3:6])


def _yaw_rotvec(yaw_deg):
    """Rotation vector for the tool yawed about base Z by (yaw - YAW_REF_DEG)."""
    a = np.radians(yaw_deg - YAW_REF_DEG)
    Rz = np.array([[np.cos(a), -np.sin(a), 0.0],
                   [np.sin(a),  np.cos(a), 0.0],
                   [0.0,        0.0,       1.0]])
    return _matrix_to_rotvec(Rz @ _R_REF)


# ── Pose construction ─────────────────────────────────────────────────────────

def _build_pose(x_mm, y_mm, z_mm, yaw_deg=None):
    """
    Build a 6-DOF TCP pose from working-area XY plus a height above reference.
    When a yaw is supplied and APPLY_YAW is set, the tool is rotated about the
    base vertical axis to that heading; otherwise the reference orientation is
    kept.
    """
    pose = list(REFERENCE_POSE)
    pose[0] += x_mm / 1000.0
    pose[1] += y_mm / 1000.0
    pose[2] += z_mm / 1000.0
    if yaw_deg is not None and APPLY_YAW:
        pose[3], pose[4], pose[5] = _yaw_rotvec(yaw_deg)
    return pose


def _home_pose():
    return _build_pose(0.0, 0.0, TRAVEL_HEIGHT_MM)


def _offset_clamped(x_mm, y_mm):
    """Add the live central-point offset and hard-clamp to the safety window."""
    cx, cy = get_center_offset()
    half = _clamp_half[0]
    x = max(-half, min(half, x_mm + cx))
    y = max(-half, min(half, y_mm + cy))
    return x, y


def ur_to_offset(X_mm, Y_mm):
    """UR base-frame (X, Y) mm → offset (mm) from the reference pose XY."""
    return (X_mm - REFERENCE_POSE[0] * 1000.0,
            Y_mm - REFERENCE_POSE[1] * 1000.0)


# ── Background TCP reader ──────────────────────────────────────────────────────

def _tcp_reader_loop():
    """Update the shared TCP pose at ~125 Hz for the live viewer."""
    while not _stop_flag.is_set():
        rtde_r = _rtde_r_ref[0]
        if rtde_r is not None:
            try:
                tcp = rtde_r.getActualTCPPose()
                with _lock:
                    _state['tcp'] = list(tcp)
            except Exception:
                pass
        time.sleep(0.008)


# ── Connection helpers ─────────────────────────────────────────────────────────

def _connect_receive():
    for attempt in range(3):
        try:
            r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            print("[ur5] Receive connected")
            return r
        except Exception as e:
            print(f"[ur5] Receive {attempt + 1}/3 failed: {e}")
            time.sleep(2)
    return None


def _connect_control():
    for attempt in range(3):
        try:
            c = rtde_control.RTDEControlInterface(
                ROBOT_IP, frequency=500.0,
                flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
            print("[ur5] Control connected")
            return c
        except Exception as e:
            print(f"[ur5] Control {attempt + 1}/3 failed: {e}")
            time.sleep(2)
    return None


def _return_home(rtde_c):
    try:
        print("[ur5] Returning to home ...")
        rtde_c.moveL(_home_pose(), VELOCITY_TRAVEL, ACCELERATION)
        print("[ur5] At home")
    except Exception as e:
        print(f"[ur5] Home failed: {e}")


# ── Trajectory entry point ─────────────────────────────────────────────────────

def run_trajectory(pts, height_mm=DEFAULT_HEIGHT_MM,
                   speed_mps=None, on_waypoint=None,
                   fit=True, clamp_mm=WORKAREA_MAX_MM):
    """
    Move the end-effector along `pts` in a horizontal plane, position control.

    Parameters
    ----------
    pts        : list of (x_mm, y_mm[, yaw]) as offsets from the reference pose
    height_mm  : plane height above the reference pose (mm, free air)
    speed_mps  : trajectory speed (m/s); default VELOCITY_MOVE
    on_waypoint: optional callback(index, x_mm, y_mm, z_mm)
    fit        : True  → recentre + scale the path into the 23 cm area
                 False → absolute mode: keep coordinates as given (used for
                         calibrated UR-frame trajectories), no recentre.
    clamp_mm   : per-waypoint safety window (mm). Widen for absolute runs so
                 the true calibrated positions are not clipped.
    """
    _clamp_half[0] = clamp_mm / 2.0
    # Seed the live speed/height from the call (the console can override live).
    _speed[0]  = speed_mps or _speed[0]
    if height_mm is not None:
        _height[0] = height_mm

    if fit:
        # Safety: never let a generated/rescaled path exceed the 23 cm area.
        pts = clamp_to_area(list(pts), clamp_mm)
    else:
        pts = list(pts)                       # absolute: positions are exact
    n = len(pts)
    if n == 0:
        print("[ur5] Empty trajectory — nothing to do")
        return

    with _lock:
        _state.update(moving=False, done=False, wp=0, wp_total=n)
    _stop_flag.clear()

    rtde_r = _connect_receive()
    if rtde_r is None:
        print("[ur5] Cannot connect to robot — aborting")
        with _lock:
            _state['done'] = True
        return
    _rtde_r_ref[0] = rtde_r
    threading.Thread(target=_tcp_reader_loop, daemon=True).start()

    rtde_c = _connect_control()
    if rtde_c is None:
        print("[ur5] RTDE Control unavailable — aborting")
        with _lock:
            _state['done'] = True
        return

    try:
        # Home, then travel above the first waypoint at extra clearance.
        print("[ur5] Moving to home position ...")
        rtde_c.moveL(_home_pose(), VELOCITY_TRAVEL, ACCELERATION)

        x0, y0 = _offset_clamped(pts[0][0], pts[0][1])
        yaw0 = _effective_yaw(pts[0][2] if len(pts[0]) >= 3 else None)
        print(f"[ur5] Travelling above start ({x0:+.1f}, {y0:+.1f}) mm"
              + (f", yaw {yaw0:+.0f}°" if yaw0 is not None and APPLY_YAW else "")
              + " ...")
        # Reach the start yaw during the approach so sliding begins settled.
        rtde_c.moveL(_build_pose(x0, y0, TRAVEL_HEIGHT_MM, yaw0),
                     VELOCITY_TRAVEL, ACCELERATION)
        print(f"[ur5] Descending to work plane (+{_height[0]:.0f} mm) ...")
        rtde_c.moveL(_build_pose(x0, y0, _height[0], yaw0),
                     VELOCITY_TRAVEL, ACCELERATION)

        if _stop_flag.is_set():
            return

        print(f"[ur5] Executing trajectory — {n} waypoints  "
              f"speed = {_speed[0] * 1000:.1f} mm/s")
        with _lock:
            _state['moving'] = True

        for i, wp in enumerate(pts):
            if _stop_flag.is_set():
                print("[ur5] Stop requested — ending trajectory")
                break
            x_mm, y_mm = wp[0], wp[1]
            # Waypoint yaw plus the live manual yaw offset (both applied fresh
            # each waypoint so orientation and centre can be tuned live).
            yaw_eff = _effective_yaw(wp[2] if len(wp) >= 3 else None)
            fx, fy = _offset_clamped(x_mm, y_mm)
            # Live speed / height, read fresh each waypoint.
            rtde_c.moveL(_build_pose(fx, fy, _height[0], yaw_eff),
                         _speed[0], ACCELERATION)
            with _lock:
                _state['wp'] = i + 1
                _state['yaw'] = yaw_eff
            if on_waypoint:
                on_waypoint(i, x_mm, y_mm, height_mm)

        print(f"[ur5] Trajectory complete ({_state['wp']}/{n} waypoints)")

    except KeyboardInterrupt:
        print("[ur5] Interrupted by user")
    finally:
        with _lock:
            _state['moving'] = False
        _return_home(rtde_c)
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        with _lock:
            _state['done'] = True


if __name__ == "__main__":
    import sys
    print("ur5_imu.py — run via run_imu.py or imu_live.py")
    sys.exit(0)
