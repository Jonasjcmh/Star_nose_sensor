"""
robot_viz_meshcat.py
Browser-based UR5 digital twin using Meshcat + Three.js.

Opens a browser tab with a 3D interactive robot viewer.
Reads live joint angles from RTDE; falls back to demo animation
and retries connection every 5 seconds.

Usage:
  python robot_viz_meshcat.py          # real robot
  python robot_viz_meshcat.py --sim    # URSim at localhost
"""

import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import meshcat
    import meshcat.geometry as g
    import meshcat.transformations as tf
except ImportError:
    print("meshcat not found — install: pip install meshcat")
    sys.exit(1)

# ── Constants ───────────────────────────────────────────────────────────────────
PI = math.pi
N  = 19   # sensor cells

ROBOT_IP = os.environ.get("UR_ROBOT_IP", "177.22.22.2")

SENSOR_SHARED_FILE = "/tmp/star_nose_sensor.json"

# ── Physical sensor geometry (fixed on table, robot has an indenter) ────────────
# Sensor centre = UR5 P10 reference TCP position (indenter tip at surface)
SENSOR_REF = np.array([-0.03746 + 0.0005, -0.50066 + 0.0016, 0.06054])

# UR5 scan-point offsets (mm) relative to P10 centre
_POINTS_MM = {
     1: ( -8.0, +14.0),  2: (  0.0, +14.0),  3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),  5: ( -4.0,  +7.0),  6: ( +4.0,  +7.0),  7: (+12.0,  +7.0),
     8: (-16.0,   0.0),  9: ( -8.0,   0.0), 10: (  0.0,   0.0), 11: ( +8.0,   0.0),
    12: (+16.0,   0.0),
    13: (-12.0,  -7.0), 14: ( -4.0,  -7.0), 15: ( +4.0,  -7.0), 16: (+12.0,  -7.0),
    17: ( -8.0, -14.0), 18: (  0.0, -14.0), 19: ( +8.0, -14.0),
}

# USED_CELLS order (from sensor.py) maps CSV cell_i+1 → raw sensor index
_USED_CELLS    = [2, 15, 28, 1, 14, 27, 40, 0, 13, 26, 39, 52, 12, 25, 38, 51, 24, 37, 50]
_UR5_TO_SENSOR = {1:24, 2:12, 3:0, 4:37, 5:25, 6:13, 7:1, 8:50, 9:38,
                  10:26, 11:14, 12:2, 13:51, 14:39, 15:27, 16:15, 17:52, 18:40, 19:28}
_SENSOR_TO_UR5 = {v: k for k, v in _UR5_TO_SENSOR.items()}

# World positions of the 19 sensor cells (CSV cell_1..19 order)
CELL_WORLD_POS = np.array([
    SENSOR_REF + np.array([_POINTS_MM[_SENSOR_TO_UR5[_USED_CELLS[i]]][0] / 1000.0,
                            _POINTS_MM[_SENSOR_TO_UR5[_USED_CELLS[i]]][1] / 1000.0,
                            0.0])
    for i in range(N)
])  # shape (19, 3)

# Indenter geometry
INDENTER_LENGTH = 0.040   # 40 mm
INDENTER_RADIUS = 0.006   # 6 mm
INDENTER_COLOR  = 0xC8CBD6  # brushed metal

# ── UR5 CB3 DH forward kinematics ──────────────────────────────────────────────
UR5_DH = [
    (0.0,     0.089159,  PI / 2),
    (-0.425,  0.0,       0.0   ),
    (-0.39225,0.0,       0.0   ),
    (0.0,     0.10915,   PI / 2),
    (0.0,     0.09465,  -PI / 2),
    (0.0,     0.0823,    0.0   ),
]

Q_HOME = [0.0, -PI / 2, 0.0, -PI / 2, 0.0, 0.0]


def _dh(theta, a, d, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,  -st*ca,  st*sa,  a*ct],
        [st,   ct*ca, -ct*sa,  a*st],
        [0.0,  sa,     ca,     d   ],
        [0.0,  0.0,    0.0,    1.0 ],
    ])


def ur5_fk(q):
    """Return (7,3) joint-origin positions in base frame."""
    T   = np.eye(4)
    pts = [T[:3, 3].copy()]
    for (a, d, alpha), qi in zip(UR5_DH, q):
        T = T @ _dh(float(qi), a, d, alpha)
        pts.append(T[:3, 3].copy())
    return np.array(pts)   # (7, 3)


def ur5_fk_full(q):
    """Return list of 7 full 4×4 transforms (base + 6 joints)."""
    T  = np.eye(4)
    Ts = [T.copy()]
    for (a, d, alpha), qi in zip(UR5_DH, q):
        T = T @ _dh(float(qi), a, d, alpha)
        Ts.append(T.copy())
    return Ts   # len=7


# ── Shared state ────────────────────────────────────────────────────────────────
_state = {
    "q":        Q_HOME[:],
    "cells":    [0.0] * N,
    "ft":       [0.0, 0.0, 0.0],
    "fz":       0.0,
    "pressing": False,
    "connected": False,
    "trail":    [],
}
_state_lock = threading.Lock()
_running    = [True]

TRAIL_MAX = 80


# ── Sensor reader ───────────────────────────────────────────────────────────────
def _read_sensor():
    try:
        import json
        with open(SENSOR_SHARED_FILE) as f:
            d = json.load(f)
        if d.get("ready"):
            return d["values"]
    except Exception:
        pass
    return None


# ── RTDE + demo live thread ─────────────────────────────────────────────────────
def _live_thread(ip):
    RETRY = 5.0
    t0    = time.time()

    def _connect():
        try:
            import rtde_receive
            r = rtde_receive.RTDEReceiveInterface(ip)
            print(f"[meshcat] RTDE connected to {ip}")
            with _state_lock:
                _state["connected"] = True
            return r
        except Exception as e:
            print(f"[meshcat] Could not connect to {ip}: {e}")
            with _state_lock:
                _state["connected"] = False
            return None

    print(f"[meshcat] Connecting to {ip} …")
    r          = _connect()
    last_retry = time.time()
    if r is None:
        print(f"[meshcat] No robot — demo mode (retry every {RETRY:.0f}s)")

    while _running[0]:
        if r is not None:
            try:
                q     = list(r.getActualQ())
                ft    = r.getActualTCPForce()
                cells = _read_sensor() or [0.0] * N
                tcp   = ur5_fk(q)[-1]
                with _state_lock:
                    _state["q"]         = q
                    _state["cells"]     = cells
                    _state["ft"]        = list(ft[:3]) if ft else [0.0, 0.0, 0.0]
                    _state["fz"]        = abs(ft[2]) if ft else 0.0
                    _state["pressing"]  = any(v > 0.1 for v in cells)
                    _state["connected"] = True
                    trail = _state["trail"]
                    trail.append(tcp.copy())
                    if len(trail) > TRAIL_MAX:
                        trail.pop(0)
            except Exception as e:
                print(f"[meshcat] RTDE error: {e} — switching to demo")
                r = None
                with _state_lock:
                    _state["connected"] = False
                last_retry = time.time()
        else:
            t = time.time() - t0
            q = [
                0.30 * math.sin(0.40 * t),
                -PI/2 + 0.30 * math.sin(0.30 * t + 1.0),
                0.40 * math.sin(0.50 * t + 0.5),
                -PI/2 + 0.20 * math.sin(0.70 * t),
                0.30 * math.sin(0.60 * t + 2.0),
                0.10 * math.sin(t),
            ]
            cells = [max(0.0, min(1.0,
                         0.4 * math.sin(t + i * 0.4) *
                         math.sin(t * 0.7 + i * 0.2) + 0.2))
                     for i in range(N)]
            tcp = ur5_fk(q)[-1]
            with _state_lock:
                _state["q"]         = q[:]
                _state["cells"]     = cells
                _state["connected"] = False
                trail = _state["trail"]
                trail.append(tcp.copy())
                if len(trail) > TRAIL_MAX:
                    trail.pop(0)

            if time.time() - last_retry >= RETRY:
                print(f"[meshcat] Retrying {ip} …")
                r          = _connect()
                last_retry = time.time()
                if r is None:
                    print(f"[meshcat] Still unavailable — next retry in {RETRY:.0f}s")

        time.sleep(0.05)


# ── Transform helpers ───────────────────────────────────────────────────────────
def _seg_tf(p1, p2):
    """4×4 transform that places a unit Y-axis cylinder between p1 and p2."""
    direction = p2 - p1
    length    = float(np.linalg.norm(direction))
    if length < 1e-6:
        length = 1e-6
    mid    = (p1 + p2) * 0.5
    d_norm = direction / length

    # Rotation: Y → d_norm
    y     = np.array([0.0, 1.0, 0.0])
    cross = np.cross(y, d_norm)
    dot   = float(np.dot(y, d_norm))
    cn    = float(np.linalg.norm(cross))

    if cn < 1e-6:
        R = np.eye(3) if dot > 0 else np.diag([-1.0, -1.0, 1.0])
    else:
        axis  = cross / cn
        angle = math.atan2(cn, dot)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = mid
    return T, length


def _trans(xyz):
    T = np.eye(4)
    T[:3, 3] = xyz
    return T


# ── Colour helpers ──────────────────────────────────────────────────────────────
def _rgb_to_hex(r, g, b):
    return (int(r * 255) << 16) | (int(g * 255) << 8) | int(b * 255)


def _cell_color(v):
    # blue (cold) → cyan → green → yellow → red (hot)
    v = max(0.0, min(1.0, v))
    if v < 0.25:
        t = v / 0.25
        return _rgb_to_hex(0, t * 0.8, 1.0)
    elif v < 0.5:
        t = (v - 0.25) / 0.25
        return _rgb_to_hex(0, 0.8, 1.0 - t)
    elif v < 0.75:
        t = (v - 0.5) / 0.25
        return _rgb_to_hex(t, 1.0 - t * 0.5, 0)
    else:
        t = (v - 0.75) / 0.25
        return _rgb_to_hex(1.0, 0.5 - t * 0.5, 0)


# ── Geometry constants ──────────────────────────────────────────────────────────
UR_BLUE    = 0x2E6EC8
JOINT_DARK = 0x1A2A3A
TCP_RED    = 0xFF3030
TRAIL_COL  = 0x4488FF
FLOOR_COL  = 0x0D1B2A

# Link segment lengths (from DH d and a parameters)
SEG_LENGTHS = [
    0.089159,   # base → shoulder
    0.425,      # shoulder → elbow
    0.39225,    # elbow → wrist1
    0.10915,    # wrist1 → wrist2
    0.09465,    # wrist2 → wrist3
    0.0823,     # wrist3 → TCP
]

SEG_RADII = [0.068, 0.054, 0.043, 0.040, 0.038, 0.036]
JNT_RADII = [0.075, 0.068, 0.062, 0.048, 0.044, 0.044, 0.020]

SENSOR_PLT_COL = 0x1A2A3A  # sensor platform colour

# Force arrow colours and scale
ARROW_COLS  = {"fx": 0xFF3333, "fy": 0x33FF55, "fz": 0x3388FF, "fr": 0xFFDD11}
FORCE_SCALE = 0.015   # m per Newton (15 mm = 1 N)


def _arrow_tf(origin, direction, magnitude):
    """Return (shaft_T, shaft_len, head_T, head_len) for a force arrow."""
    if magnitude < 0.3:
        return None, 0, None, 0
    tip       = origin + direction * magnitude * FORCE_SCALE
    shaft_end = origin + direction * magnitude * FORCE_SCALE * 0.75
    T_shaft, shaft_len = _seg_tf(origin, shaft_end)
    T_head,  head_len  = _seg_tf(shaft_end, tip)
    return T_shaft, shaft_len, T_head, head_len


def _draw_arrow(vis, name, origin, direction, magnitude):
    """Update a pre-created force arrow (shaft + head cylinders)."""
    T_shaft, sl, T_head, hl = _arrow_tf(origin, direction, magnitude)
    if T_shaft is None:
        vis[f"force/{name}/shaft"].set_transform(_trans([0, 0, -10]))
        vis[f"force/{name}/head"].set_transform(_trans([0, 0, -10]))
        return
    vis[f"force/{name}/shaft"].set_object(
        g.Cylinder(max(1e-4, sl), 0.003),
        g.MeshLambertMaterial(color=ARROW_COLS[name])
    )
    vis[f"force/{name}/shaft"].set_transform(T_shaft)
    vis[f"force/{name}/head"].set_object(
        g.Cylinder(max(1e-4, hl), 0.009),
        g.MeshLambertMaterial(color=ARROW_COLS[name])
    )
    vis[f"force/{name}/head"].set_transform(T_head)


# ── Scene setup ─────────────────────────────────────────────────────────────────
def _setup(vis):
    # Floor grid
    vis["scene/floor"].set_object(
        g.Box([2.0, 2.0, 0.002]),
        g.MeshLambertMaterial(color=FLOOR_COL, wireframe=True)
    )
    vis["scene/floor"].set_transform(_trans([0, 0, -0.002]))

    # Robot links (Y-axis cylinders, fixed radii, will be repositioned each frame)
    for i in range(6):
        vis[f"robot/link{i}"].set_object(
            g.Cylinder(SEG_LENGTHS[i], SEG_RADII[i]),
            g.MeshLambertMaterial(color=UR_BLUE)
        )
    # Joint spheres
    for i in range(6):
        vis[f"robot/joint{i}"].set_object(
            g.Sphere(JNT_RADII[i]),
            g.MeshLambertMaterial(color=JOINT_DARK)
        )
    # TCP flange marker
    vis["robot/tcp"].set_object(
        g.Sphere(JNT_RADII[6]),
        g.MeshLambertMaterial(color=TCP_RED)
    )
    # Indenter cylinder (transform updated each frame)
    vis["robot/indenter"].set_object(
        g.Cylinder(INDENTER_LENGTH, INDENTER_RADIUS),
        g.MeshLambertMaterial(color=INDENTER_COLOR)
    )
    vis["robot/indenter"].set_transform(_trans([0, 0, -10]))

    # Trail spheres
    for i in range(TRAIL_MAX):
        vis[f"trail/p{i}"].set_object(
            g.Sphere(0.003),
            g.MeshLambertMaterial(color=TRAIL_COL, opacity=0.4,
                                  transparent=True)
        )
        vis[f"trail/p{i}"].set_transform(_trans([0, 0, -10]))  # hide off-screen

    # Sensor platform (table-mounted, fixed)
    vis["sensor/platform"].set_object(
        g.Box([0.040, 0.036, 0.004]),
        g.MeshLambertMaterial(color=SENSOR_PLT_COL)
    )
    vis["sensor/platform"].set_transform(_trans(SENSOR_REF + np.array([0.0, 0.0, -0.003])))

    # Fixed sensor cells at table positions (colour updated each frame)
    for i in range(N):
        vis[f"sensor/c{i}"].set_object(
            g.Sphere(0.005),
            g.MeshLambertMaterial(color=0x112233)
        )
        vis[f"sensor/c{i}"].set_transform(_trans(CELL_WORLD_POS[i]))

    # Force arrows — shaft + head for each axis + resultant (hidden until force present)
    for name in ("fx", "fy", "fz", "fr"):
        vis[f"force/{name}/shaft"].set_object(
            g.Cylinder(0.001, 0.003),
            g.MeshLambertMaterial(color=ARROW_COLS[name])
        )
        vis[f"force/{name}/shaft"].set_transform(_trans([0, 0, -10]))
        vis[f"force/{name}/head"].set_object(
            g.Cylinder(0.001, 0.009),
            g.MeshLambertMaterial(color=ARROW_COLS[name])
        )
        vis[f"force/{name}/head"].set_transform(_trans([0, 0, -10]))


# ── Per-frame update ────────────────────────────────────────────────────────────
def _update(vis, q, cells, ft, trail, connected):
    pts = ur5_fk(q)     # (7, 3)
    Ts  = ur5_fk_full(q)  # 7 transforms

    # Links
    for i in range(6):
        T_seg, _seg_len = _seg_tf(pts[i], pts[i + 1])
        vis[f"robot/link{i}"].set_transform(T_seg)

    # Joint spheres
    for i in range(6):
        vis[f"robot/joint{i}"].set_transform(_trans(pts[i]))

    # TCP
    vis["robot/tcp"].set_transform(_trans(pts[6]))

    # Trail
    for i in range(TRAIL_MAX):
        if i < len(trail):
            vis[f"trail/p{i}"].set_transform(_trans(trail[i]))
        else:
            vis[f"trail/p{i}"].set_transform(_trans([0, 0, -10]))

    # Indenter: extends from flange (pts[6]) along tool-Z direction
    tz           = Ts[6][:3, 2]
    indenter_tip = pts[6] + tz * INDENTER_LENGTH
    T_ind, ind_len = _seg_tf(pts[6], indenter_tip)
    vis["robot/indenter"].set_object(
        g.Cylinder(max(1e-4, ind_len), INDENTER_RADIUS),
        g.MeshLambertMaterial(color=INDENTER_COLOR)
    )
    vis["robot/indenter"].set_transform(T_ind)

    # Fixed sensor cells — update colour/size by activation (position stays from _setup)
    for i in range(N):
        v      = cells[i] if i < len(cells) else 0.0
        color  = _cell_color(v)
        radius = 0.004 + v * 0.003
        vis[f"sensor/c{i}"].set_object(
            g.Sphere(radius),
            g.MeshLambertMaterial(color=color)
        )

    # Force arrows at indenter tip (contact point)
    tcp_pos = indenter_tip
    fx_v = ft[0] if len(ft) > 0 else 0.0
    fy_v = ft[1] if len(ft) > 1 else 0.0
    fz_v = ft[2] if len(ft) > 2 else 0.0
    fr_v = np.array([fx_v, fy_v, fz_v])
    fr_mag = float(np.linalg.norm(fr_v))
    _draw_arrow(vis, "fx", tcp_pos, np.array([1, 0, 0]), abs(fx_v) * (1 if fx_v >= 0 else -1))
    _draw_arrow(vis, "fy", tcp_pos, np.array([0, 1, 0]), abs(fy_v) * (1 if fy_v >= 0 else -1))
    _draw_arrow(vis, "fz", tcp_pos, np.array([0, 0, 1]), abs(fz_v) * (1 if fz_v >= 0 else -1))
    if fr_mag > 0.3:
        _draw_arrow(vis, "fr", tcp_pos, fr_v / fr_mag, fr_mag)
    else:
        vis["force/fr/shaft"].set_transform(_trans([0, 0, -10]))
        vis["force/fr/head"].set_transform(_trans([0, 0, -10]))


# ── Main ────────────────────────────────────────────────────────────────────────
def main():
    import argparse
    p = argparse.ArgumentParser(description="UR5 Meshcat digital twin")
    p.add_argument("--sim", action="store_true", help="Connect to URSim at localhost")
    p.add_argument("--ip",  default=None,        help="Override robot IP")
    args = p.parse_args()

    if args.sim:
        os.environ["UR_ROBOT_IP"] = "127.0.0.1"
    if args.ip:
        os.environ["UR_ROBOT_IP"] = args.ip

    ip = os.environ.get("UR_ROBOT_IP", ROBOT_IP)

    vis = meshcat.Visualizer()
    vis.open()
    print(f"[meshcat] Viewer → {vis.url()}")
    print(f"[meshcat] Open the URL above in a browser if it doesn't open automatically.")

    _setup(vis)

    t = threading.Thread(target=_live_thread, args=(ip,), daemon=True)
    t.start()

    print("[meshcat] Running — press Ctrl+C to stop\n")
    try:
        while True:
            with _state_lock:
                q     = list(_state["q"])
                cells = list(_state["cells"])
                ft    = list(_state["ft"])
                fz    = _state["fz"]
                trail = list(_state["trail"])
                conn  = _state["connected"]

            _update(vis, q, cells, ft, trail, conn)

            status = "● ROBOT LIVE" if conn else "○ SIMULATION"
            fr_str = f"|F|={float(np.linalg.norm(ft)):.1f}N"
            print(f"\r[meshcat] {status}  Fz={fz:+.2f}N  {fr_str}  "
                  f"cells={sum(1 for v in cells if v > 0.05):2d}/19  "
                  f"trail={len(trail):3d}pts",
                  end="", flush=True)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[meshcat] Stopped")
    finally:
        _running[0] = False


if __name__ == "__main__":
    main()
