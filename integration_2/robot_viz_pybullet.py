"""
robot_viz_pybullet.py
UR5 digital twin using PyBullet — OpenGL 3D window with lighting.

Loads a minimal UR5 URDF, animates joints from live RTDE or demo,
overlays sensor hex-grid and TCP trail as debug visuals.

Usage:
  python robot_viz_pybullet.py          # real robot
  python robot_viz_pybullet.py --sim    # URSim at localhost
"""

import math
import os
import sys
import tempfile
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("pybullet not found — install: conda install -c conda-forge pybullet")
    sys.exit(1)

# ── Constants ───────────────────────────────────────────────────────────────────
PI = math.pi
N  = 19

ROBOT_IP = os.environ.get("UR_ROBOT_IP", "177.22.22.2")
SENSOR_SHARED_FILE = "/tmp/star_nose_sensor.json"

Q_HOME = [0.0, -PI / 2, 0.0, -PI / 2, 0.0, 0.0]

# Physical sensor geometry (fixed on table, robot has indenter)
SENSOR_REF = np.array([-0.03746 + 0.0005, -0.50066 + 0.0016, 0.06054])

_POINTS_MM = {
     1: ( -8.0, +14.0),  2: (  0.0, +14.0),  3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),  5: ( -4.0,  +7.0),  6: ( +4.0,  +7.0),  7: (+12.0,  +7.0),
     8: (-16.0,   0.0),  9: ( -8.0,   0.0), 10: (  0.0,   0.0), 11: ( +8.0,   0.0),
    12: (+16.0,   0.0),
    13: (-12.0,  -7.0), 14: ( -4.0,  -7.0), 15: ( +4.0,  -7.0), 16: (+12.0,  -7.0),
    17: ( -8.0, -14.0), 18: (  0.0, -14.0), 19: ( +8.0, -14.0),
}
_USED_CELLS    = [2, 15, 28, 1, 14, 27, 40, 0, 13, 26, 39, 52, 12, 25, 38, 51, 24, 37, 50]
_UR5_TO_SENSOR = {1:24, 2:12, 3:0, 4:37, 5:25, 6:13, 7:1, 8:50, 9:38,
                  10:26, 11:14, 12:2, 13:51, 14:39, 15:27, 16:15, 17:52, 18:40, 19:28}
_SENSOR_TO_UR5 = {v: k for k, v in _UR5_TO_SENSOR.items()}

CELL_WORLD_POS = np.array([
    SENSOR_REF + np.array([_POINTS_MM[_SENSOR_TO_UR5[_USED_CELLS[i]]][0] / 1000.0,
                            _POINTS_MM[_SENSOR_TO_UR5[_USED_CELLS[i]]][1] / 1000.0,
                            0.0])
    for i in range(N)
])  # shape (19, 3)

INDENTER_LENGTH = 0.040   # 40 mm
INDENTER_RADIUS = 0.006   # 6 mm


# ── UR5 CB3 URDF (minimal — cylinders + spheres, no external mesh files) ────────
# Joint origins from the official ur_description package (ROS-industrial/ur_description).
# Visual geometry is simplified (cylinders for links, spheres for joints).
UR5_URDF = """\
<?xml version="1.0"?>
<robot name="ur5_twin">

  <material name="ur_blue"><color rgba="0.18 0.42 0.78 1"/></material>
  <material name="dark"><color rgba="0.10 0.11 0.13 1"/></material>
  <material name="tcp_red"><color rgba="0.90 0.18 0.18 1"/></material>
  <material name="joint_gray"><color rgba="0.22 0.26 0.30 1"/></material>
  <material name="indenter_mat"><color rgba="0.78 0.80 0.84 1"/></material>

  <!-- WORLD -->
  <link name="world"/>

  <!-- BASE -->
  <link name="base_link">
    <inertial><mass value="2.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.044"/>
      <geometry><cylinder length="0.088" radius="0.075"/></geometry>
      <material name="dark"/>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.088"/>
      <geometry><sphere radius="0.075"/></geometry>
      <material name="joint_gray"/>
    </visual>
  </link>
  <joint name="world_joint" type="fixed">
    <parent link="world"/><child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>

  <!-- SHOULDER (pan around Z) -->
  <link name="shoulder_link">
    <inertial><mass value="1.0"/><inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/></inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry><sphere radius="0.068"/></geometry>
      <material name="joint_gray"/>
    </visual>
  </link>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/><child link="shoulder_link"/>
    <origin rpy="0 0 0" xyz="0 0 0.089159"/>
    <axis xyz="0 0 1"/>
    <limit effort="150" lower="-6.28" upper="6.28" velocity="3.15"/>
    <dynamics damping="0" friction="0"/>
  </joint>

  <!-- UPPER ARM (lift around local Y; arm tube along local -X = 425mm) -->
  <link name="upper_arm_link">
    <inertial><mass value="1.5"/><inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.005"/></inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry><sphere radius="0.062"/></geometry>
      <material name="joint_gray"/>
    </visual>
    <visual>
      <origin rpy="0 1.5708 0" xyz="-0.2125 0 0"/>
      <geometry><cylinder length="0.425" radius="0.054"/></geometry>
      <material name="ur_blue"/>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="-0.425 0 0"/>
      <geometry><sphere radius="0.058"/></geometry>
      <material name="joint_gray"/>
    </visual>
  </link>
  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/><child link="upper_arm_link"/>
    <origin rpy="0 1.5707963 0" xyz="0 0.13585 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="150" lower="-6.28" upper="6.28" velocity="3.15"/>
    <dynamics damping="0" friction="0"/>
  </joint>

  <!-- FOREARM (elbow around local Y; tube along local -X = 392mm) -->
  <link name="forearm_link">
    <inertial><mass value="1.2"/><inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.004"/></inertial>
    <visual>
      <origin rpy="0 1.5708 0" xyz="-0.196125 0 0"/>
      <geometry><cylinder length="0.39225" radius="0.043"/></geometry>
      <material name="ur_blue"/>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="-0.39225 0 0"/>
      <geometry><sphere radius="0.048"/></geometry>
      <material name="joint_gray"/>
    </visual>
  </link>
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/><child link="forearm_link"/>
    <origin rpy="0 0 0" xyz="-0.425 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="150" lower="-6.28" upper="6.28" velocity="3.15"/>
    <dynamics damping="0" friction="0"/>
  </joint>

  <!-- WRIST 1 (after rpy=[0,pi/2,0] rotation; extends 94.65mm along local Y) -->
  <link name="wrist_1_link">
    <inertial><mass value="0.3"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
    <visual>
      <origin rpy="1.5708 0 0" xyz="0 0.04733 0"/>
      <geometry><cylinder length="0.09465" radius="0.038"/></geometry>
      <material name="ur_blue"/>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0.09465 0"/>
      <geometry><sphere radius="0.044"/></geometry>
      <material name="joint_gray"/>
    </visual>
  </link>
  <joint name="wrist_1_joint" type="revolute">
    <parent link="forearm_link"/><child link="wrist_1_link"/>
    <origin rpy="0 1.5707963 0" xyz="-0.39225 0.1197 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="28" lower="-6.28" upper="6.28" velocity="6.28"/>
    <dynamics damping="0" friction="0"/>
  </joint>

  <!-- WRIST 2 (after rpy=[pi/2,0,0] rotation; extends 94.65mm along local Z) -->
  <link name="wrist_2_link">
    <inertial><mass value="0.3"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.04733"/>
      <geometry><cylinder length="0.09465" radius="0.038"/></geometry>
      <material name="ur_blue"/>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.09465"/>
      <geometry><sphere radius="0.042"/></geometry>
      <material name="joint_gray"/>
    </visual>
  </link>
  <joint name="wrist_2_joint" type="revolute">
    <parent link="wrist_1_link"/><child link="wrist_2_link"/>
    <origin rpy="1.5707963 0 0" xyz="0 0.09465 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="28" lower="-6.28" upper="6.28" velocity="6.28"/>
    <dynamics damping="0" friction="0"/>
  </joint>

  <!-- WRIST 3 (after rpy=[pi/2,0,0] rotation; extends 82.3mm along local Y) -->
  <link name="wrist_3_link">
    <inertial><mass value="0.2"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
    <visual>
      <origin rpy="1.5708 0 0" xyz="0 0.04115 0"/>
      <geometry><cylinder length="0.0823" radius="0.036"/></geometry>
      <material name="dark"/>
    </visual>
  </link>
  <joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/><child link="wrist_3_link"/>
    <origin rpy="1.5707963 0 0" xyz="0 0 0.09465"/>
    <axis xyz="0 1 0"/>
    <limit effort="28" lower="-6.28" upper="6.28" velocity="6.28"/>
    <dynamics damping="0" friction="0"/>
  </joint>

  <!-- TCP / END-EFFECTOR + INDENTER (extends +Y beyond wrist_3_link) -->
  <link name="tcp_link">
    <inertial><mass value="0.1"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial>
    <visual>
      <origin rpy="1.5708 0 0" xyz="0 0 0"/>
      <geometry><cylinder length="0.008" radius="0.032"/></geometry>
      <material name="joint_gray"/>
    </visual>
    <visual>
      <origin rpy="1.5708 0 0" xyz="0 0.020 0"/>
      <geometry><cylinder length="0.040" radius="0.006"/></geometry>
      <material name="indenter_mat"/>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0.041 0"/>
      <geometry><sphere radius="0.007"/></geometry>
      <material name="indenter_mat"/>
    </visual>
  </link>
  <joint name="tcp_joint" type="fixed">
    <parent link="wrist_3_link"/><child link="tcp_link"/>
    <origin rpy="0 0 0" xyz="0 0.0823 0"/>
  </joint>

</robot>
"""

# ── UR5 DH forward kinematics (for trail / sensor display) ─────────────────────
UR5_DH = [
    (0.0,     0.089159,  PI / 2),
    (-0.425,  0.0,       0.0   ),
    (-0.39225,0.0,       0.0   ),
    (0.0,     0.10915,   PI / 2),
    (0.0,     0.09465,  -PI / 2),
    (0.0,     0.0823,    0.0   ),
]


def _dh(theta, a, d, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,  -st*ca,  st*sa,  a*ct],
        [st,   ct*ca, -ct*sa,  a*st],
        [0.0,  sa,     ca,     d   ],
        [0.0,  0.0,    0.0,    1.0 ],
    ])


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
    "q":         Q_HOME[:],
    "cells":     [0.0] * N,
    "ft":        [0.0, 0.0, 0.0],
    "fz":        0.0,
    "pressing":  False,
    "connected": False,
}
_state_lock = threading.Lock()
_running    = [True]


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
            print(f"[pybullet] RTDE connected to {ip}")
            with _state_lock:
                _state["connected"] = True
            return r
        except Exception as e:
            print(f"[pybullet] Could not connect to {ip}: {e}")
            with _state_lock:
                _state["connected"] = False
            return None

    print(f"[pybullet] Connecting to {ip} …")
    r          = _connect()
    last_retry = time.time()
    if r is None:
        print(f"[pybullet] No robot — demo mode (retry every {RETRY:.0f}s)")

    while _running[0]:
        if r is not None:
            try:
                q     = list(r.getActualQ())
                ft    = r.getActualTCPForce()
                cells = _read_sensor() or [0.0] * N
                with _state_lock:
                    _state["q"]         = q
                    _state["cells"]     = cells
                    _state["ft"]        = list(ft[:3]) if ft else [0.0, 0.0, 0.0]
                    _state["fz"]        = abs(ft[2]) if ft else 0.0
                    _state["pressing"]  = any(v > 0.1 for v in cells)
                    _state["connected"] = True
            except Exception as e:
                print(f"[pybullet] RTDE error: {e} — switching to demo")
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
            with _state_lock:
                _state["q"]         = q[:]
                _state["cells"]     = cells
                _state["connected"] = False
            if time.time() - last_retry >= RETRY:
                print(f"[pybullet] Retrying {ip} …")
                r          = _connect()
                last_retry = time.time()
                if r is None:
                    print(f"[pybullet] Still unavailable — next retry in {RETRY:.0f}s")

        time.sleep(0.05)


# ── Main ────────────────────────────────────────────────────────────────────────
def main():
    import argparse
    parser = argparse.ArgumentParser(description="UR5 PyBullet digital twin")
    parser.add_argument("--sim",      action="store_true", help="URSim at localhost")
    parser.add_argument("--ip",       default=None,        help="Override robot IP")
    parser.add_argument("--headless", action="store_true", help="No GUI (DIRECT mode)")
    args = parser.parse_args()

    if args.sim:
        os.environ["UR_ROBOT_IP"] = "127.0.0.1"
    if args.ip:
        os.environ["UR_ROBOT_IP"] = args.ip
    ip = os.environ.get("UR_ROBOT_IP", ROBOT_IP)

    # Write embedded URDF to temp file
    urdf_path = os.path.join(tempfile.gettempdir(), "ur5_twin.urdf")
    with open(urdf_path, "w") as f:
        f.write(UR5_URDF)

    # Connect PyBullet
    mode = p.DIRECT if args.headless else p.GUI
    client = p.connect(mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Camera
    p.resetDebugVisualizerCamera(
        cameraDistance=1.4,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[0.0, 0.0, 0.45]
    )
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

    # Load plane + robot
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
    print(f"[pybullet] Robot loaded (id={robot_id})")

    # Find the 6 revolute joints in URDF order
    n_joints = p.getNumJoints(robot_id)
    rev_joints = [
        j for j in range(n_joints)
        if p.getJointInfo(robot_id, j)[2] == p.JOINT_REVOLUTE
    ]
    print(f"[pybullet] Revolute joints: {rev_joints}")

    # HUD text
    _STATUS = p.addUserDebugText(
        "○ SIMULATION", [0.0, -0.6, 0.85],
        textColorRGB=[1.0, 0.8, 0.2], textSize=1.4
    )
    _FZ = p.addUserDebugText(
        "Fz =  0.00 N", [0.0, -0.6, 0.78],
        textColorRGB=[0.7, 0.9, 1.0], textSize=1.2
    )

    # Fixed sensor cell visual bodies (table-mounted, colour updated via changeVisualShape)
    _cell_mb = []
    for i in range(N):
        vs = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.005,
            rgbaColor=[0.07, 0.13, 0.20, 1.0]
        )
        mb = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=vs,
            basePosition=CELL_WORLD_POS[i].tolist()
        )
        _cell_mb.append(mb)

    # Sensor platform box (static)
    _plt_vs = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.020, 0.018, 0.002],
        rgbaColor=[0.10, 0.16, 0.23, 1.0]
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=_plt_vs,
        basePosition=(SENSOR_REF + np.array([0.0, 0.0, -0.003])).tolist()
    )

    # Force arrows — 4 pre-allocated lines (Fx=red, Fy=green, Fz=blue, Fr=yellow)
    _ORIGIN = [0.0, 0.0, 0.0]
    _FX_LINE = p.addUserDebugLine(_ORIGIN, _ORIGIN, [1.0, 0.2, 0.2], lineWidth=3)
    _FY_LINE = p.addUserDebugLine(_ORIGIN, _ORIGIN, [0.2, 1.0, 0.2], lineWidth=3)
    _FZ_LINE = p.addUserDebugLine(_ORIGIN, _ORIGIN, [0.3, 0.6, 1.0], lineWidth=3)
    _FR_LINE = p.addUserDebugLine(_ORIGIN, _ORIGIN, [1.0, 0.9, 0.1], lineWidth=5)
    FORCE_SCALE = 0.015   # m per Newton

    # Trail management
    TRAIL_MAX   = 200
    trail_pts   = []
    trail_lines = []   # list of debug line IDs

    # Start live thread
    t = threading.Thread(target=_live_thread, args=(ip,), daemon=True)
    t.start()

    print("[pybullet] Running — close the window or press Ctrl+C to stop\n")

    try:
        while _running[0]:
            # Detect window close (GUI mode)
            if mode == p.GUI:
                try:
                    p.getConnectionInfo(client)
                except Exception:
                    break

            with _state_lock:
                q     = list(_state["q"])
                cells = list(_state["cells"])
                fz    = _state["fz"]
                ft    = list(_state.get("ft", [0.0]*3))
                conn  = _state["connected"]

            # ── Update joint angles ────────────────────────────────────────────
            for ji, qv in enumerate(q[:len(rev_joints)]):
                p.resetJointState(robot_id, rev_joints[ji], qv)

            # ── FK for TCP + trail ────────────────────────────────────────────
            Ts  = ur5_fk_full(q)
            tcp = Ts[6][:3, 3]

            # ── Trail ─────────────────────────────────────────────────────────
            trail_pts.append(tcp.copy())
            if len(trail_pts) > TRAIL_MAX:
                trail_pts.pop(0)

            for lid in trail_lines:
                p.removeUserDebugItem(lid)
            trail_lines.clear()

            n = len(trail_pts)
            if n > 1:
                for k in range(n - 1):
                    alpha = k / n
                    color = [0.15 + 0.85 * alpha, 0.4 * alpha, 1.0 - 0.7 * alpha]
                    trail_lines.append(
                        p.addUserDebugLine(
                            trail_pts[k].tolist(), trail_pts[k + 1].tolist(),
                            lineColorRGB=color, lineWidth=2
                        )
                    )

            # ── Fixed sensor cells — update colour by activation ───────────────
            for i in range(N):
                v   = cells[i] if i < len(cells) else 0.0
                r_c = min(1.0, v * 1.5)
                g_c = max(0.0, 1.0 - v * 1.5)
                p.changeVisualShape(_cell_mb[i], -1, rgbaColor=[r_c, g_c, 0.1, 1.0])

            # ── Force arrows ───────────────────────────────────────────────────
            fx_v, fy_v, fz_v = (ft + [0.0, 0.0, 0.0])[:3]
            fr_v = np.array([fx_v, fy_v, fz_v])
            tcp_l = tcp.tolist()
            # Fx — red (robot X = world X)
            p.addUserDebugLine(tcp_l,
                (tcp + np.array([1,0,0]) * fx_v * FORCE_SCALE).tolist(),
                [1.0, 0.2, 0.2], lineWidth=3, replaceItemUniqueId=_FX_LINE)
            # Fy — green
            p.addUserDebugLine(tcp_l,
                (tcp + np.array([0,1,0]) * fy_v * FORCE_SCALE).tolist(),
                [0.2, 1.0, 0.2], lineWidth=3, replaceItemUniqueId=_FY_LINE)
            # Fz — blue
            p.addUserDebugLine(tcp_l,
                (tcp + np.array([0,0,1]) * fz_v * FORCE_SCALE).tolist(),
                [0.3, 0.6, 1.0], lineWidth=3, replaceItemUniqueId=_FZ_LINE)
            # Resultant — yellow (only when force present)
            fr_mag = float(np.linalg.norm(fr_v))
            if fr_mag > 0.3:
                p.addUserDebugLine(tcp_l,
                    (tcp + fr_v * FORCE_SCALE).tolist(),
                    [1.0, 0.9, 0.1], lineWidth=5, replaceItemUniqueId=_FR_LINE)
            else:
                p.addUserDebugLine(tcp_l, tcp_l,
                    [1.0, 0.9, 0.1], lineWidth=1, replaceItemUniqueId=_FR_LINE)

            # ── HUD ────────────────────────────────────────────────────────────
            if conn:
                p.addUserDebugText("● ROBOT LIVE", [0.0, -0.6, 0.85],
                                   textColorRGB=[0.2, 1.0, 0.5], textSize=1.4,
                                   replaceItemUniqueId=_STATUS)
            else:
                p.addUserDebugText("○ SIMULATION", [0.0, -0.6, 0.85],
                                   textColorRGB=[1.0, 0.8, 0.2], textSize=1.4,
                                   replaceItemUniqueId=_STATUS)

            p.addUserDebugText(f"Fz = {fz:+.2f} N", [0.0, -0.6, 0.78],
                               textColorRGB=[0.7, 0.9, 1.0], textSize=1.2,
                               replaceItemUniqueId=_FZ)

            p.stepSimulation()
            time.sleep(0.033)   # ~30 fps

    except KeyboardInterrupt:
        print("\n[pybullet] Stopped")
    finally:
        _running[0] = False
        try:
            p.disconnect()
        except Exception:
            pass
        print("[pybullet] Disconnected")


if __name__ == "__main__":
    main()
