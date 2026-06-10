"""
robot_viz_gazebo.py
UR5 digital twin using Gazebo Classic 11 + ROS2 Humble.

Each robot segment is an independent Gazebo model positioned via FK
using ROS2 set_entity_state service calls.  Sensor cell activations
are shown as colour-coded spheres (5 levels, respawned on level change).
Force arrows update at 10 Hz.  Falls back to demo animation when no
RTDE connection is available and retries every 5 s.

Usage:
  python robot_viz_gazebo.py          # real robot
  python robot_viz_gazebo.py --sim    # URSim at localhost
  python robot_viz_gazebo.py --demo   # demo animation only
"""

# ── Ensure system Python 3.10 + ROS2 Humble env ──────────────────────────────
import os, sys

_ROS2_SETUP = "/opt/ros/humble/setup.bash"

# If ROS2 is not sourced, re-exec with the SAME Python (preserves conda env).
if os.environ.get("ROS_DISTRO") != "humble":
    cmd = f"source {_ROS2_SETUP} && exec {sys.executable} {' '.join(sys.argv)}"
    os.execv("/bin/bash", ["/bin/bash", "-c", cmd])

# ROS2 Humble requires Python 3.10. Fall back to system Python only if needed.
if sys.version_info[:2] != (3, 10):
    _PY310 = "/usr/bin/python3.10"
    cmd = f"source {_ROS2_SETUP} && exec {_PY310} {' '.join(sys.argv)}"
    os.execv("/bin/bash", ["/bin/bash", "-c", cmd])

# ── Standard imports ──────────────────────────────────────────────────────────
import math, threading, time, subprocess, json, textwrap
import numpy as np

# ── ROS2 imports ──────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
    from gazebo_msgs.srv import SetEntityState, SpawnEntity, DeleteEntity
    from gazebo_msgs.msg import EntityState
    from geometry_msgs.msg import Pose, Twist
except ImportError as e:
    print(f"[gazebo] ROS2 import failed: {e}")
    sys.exit(1)

# ── Constants ─────────────────────────────────────────────────────────────────
PI = math.pi
N  = 19

# Official UR5 mesh directory (from ros-humble-ur-description)
_MESH_BASE = "/opt/ros/humble/share/ur_description/meshes/ur5/visual"

# (model_name, Ts_index, mesh_file, visual_xyz, visual_rpy)
# Ts_index: which FK frame (ur5_fk_full result) drives this link's world pose.
# visual_xyz/rpy: visual origin offsets from the UR5 URDF — these go into the
# SDF <visual><pose> so Gazebo composes them with the model's world pose.
_UR5_LINKS = [
    ("ur5_base",     0, "base.dae",     (0.0, 0.0,  0.0),    (0.0,   0.0,   PI)),
    ("ur5_shoulder", 1, "shoulder.dae", (0.0, 0.0,  0.0),    (0.0,   0.0,   PI)),
    ("ur5_upperarm", 2, "upperarm.dae", (0.0, 0.0,  0.13585),(PI/2,  0.0,  -PI/2)),
    ("ur5_forearm",  3, "forearm.dae",  (0.0, 0.0,  0.0165), (PI/2,  0.0,  -PI/2)),
    ("ur5_wrist1",   4, "wrist1.dae",   (0.0, 0.0, -0.093),  (PI/2,  0.0,   0.0)),
    ("ur5_wrist2",   5, "wrist2.dae",   (0.0, 0.0, -0.095),  (0.0,   0.0,   0.0)),
    ("ur5_wrist3",   6, "wrist3.dae",   (0.0, 0.0, -0.0818), (PI/2,  0.0,   0.0)),
]

ROBOT_IP           = os.environ.get("UR_ROBOT_IP", "177.22.22.2")
SENSOR_SHARED_FILE = "/tmp/star_nose_sensor.json"
WORLD_SDF_PATH     = "/tmp/ur5_twin_world.sdf"
# Written once the scene is loaded + confirmed; main.py waits on it before
# moving the robot.  Removed at startup and on exit.
GAZEBO_READY_FILE  = "/tmp/star_nose_gazebo_ready"

SENSOR_REF = np.array([-0.03746 + 0.0005, -0.50066 + 0.0016, 0.06054])

_POINTS_MM = {
     1: ( -8.0, +14.0),  2: (  0.0, +14.0),  3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),  5: ( -4.0,  +7.0),  6: ( +4.0,  +7.0),  7: (+12.0,  +7.0),
     8: (-16.0,   0.0),  9: ( -8.0,   0.0), 10: (  0.0,   0.0), 11: ( +8.0,   0.0),
    12: (+16.0,   0.0),
    13: (-12.0,  -7.0), 14: ( -4.0,  -7.0), 15: ( +4.0,  -7.0), 16: (+12.0,  -7.0),
    17: ( -8.0, -14.0), 18: (  0.0, -14.0), 19: ( +8.0, -14.0),
}
_USED_CELLS    = [2,15,28, 1,14,27,40, 0,13,26,39,52, 12,25,38,51, 24,37,50]
_UR5_TO_SENSOR = {1:24,2:12,3:0,4:37,5:25,6:13,7:1,8:50,9:38,
                  10:26,11:14,12:2,13:51,14:39,15:27,16:15,17:52,18:40,19:28}
_SENSOR_TO_UR5 = {v: k for k, v in _UR5_TO_SENSOR.items()}

CELL_WORLD_POS = np.array([
    SENSOR_REF + np.array([_POINTS_MM[_SENSOR_TO_UR5[_USED_CELLS[i]]][0] / 1000.0,
                            _POINTS_MM[_SENSOR_TO_UR5[_USED_CELLS[i]]][1] / 1000.0,
                            0.0])
    for i in range(N)
])

INDENTER_LENGTH = 0.040
INDENTER_RADIUS = 0.006

# ── UR5 FK derived from URDF joint chain ──────────────────────────────────────
# Each revolute joint rotates around its local Z axis; the <origin rpy> sets
# the frame orientation before the rotation.  This exactly matches the URDF
# so the visual-origin offsets in _UR5_LINKS are applied in the correct frame.

Q_HOME = [0.0, -PI/2, 0.0, -PI/2, 0.0, 0.0]

def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr, cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def _T_origin(xyz, rpy):
    T = np.eye(4)
    T[:3,:3] = _rpy_to_R(*rpy)
    T[:3,3]  = xyz
    return T

def _T_rz(q):
    T = np.eye(4)
    c, s = math.cos(q), math.sin(q)
    T[:3,:3] = [[c,-s,0],[s,c,0],[0,0,1]]
    return T

# Fixed joint: base_link → base_link_inertia (rpy = 0 0 π)
_T_BASE = _T_origin((0., 0., 0.), (0., 0., PI))

# (xyz, rpy) for each revolute joint's <origin> in the URDF
_URDF_JOINTS = [
    ((0.0, 0.0, 0.089159),   (0.0,   0.0,  0.0)),          # shoulder_pan
    ((0.0, 0.0, 0.0),        (PI/2,  0.0,  0.0)),           # shoulder_lift
    ((-0.425, 0.0, 0.0),     (0.0,   0.0,  0.0)),           # elbow
    ((-0.39225, 0.0, 0.10915),(0.0,   0.0,  0.0)),           # wrist_1
    ((0.0, -0.09465, 0.0),   (PI/2,  0.0,  0.0)),           # wrist_2
    ((0.0,  0.0823, 0.0),    (PI/2,  PI,   PI)),             # wrist_3
]

def ur5_fk_full(q):
    """Returns 7 world-frame 4×4 transforms matching the URDF link frames:
    Ts[0]=base_link_inertia, Ts[1]=shoulder_link, …, Ts[6]=wrist_3_link."""
    T = _T_BASE.copy()
    Ts = [T.copy()]
    for (xyz, rpy), qi in zip(_URDF_JOINTS, q):
        T = T @ _T_origin(xyz, rpy) @ _T_rz(float(qi))
        Ts.append(T.copy())
    return Ts


def _rot_to_quat(R):
    tr = R[0,0]+R[1,1]+R[2,2]
    if tr > 0:
        s=0.5/math.sqrt(tr+1); w=0.25/s
        x=(R[2,1]-R[1,2])*s; y=(R[0,2]-R[2,0])*s; z=(R[1,0]-R[0,1])*s
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s=2.*math.sqrt(1+R[0,0]-R[1,1]-R[2,2]); w=(R[2,1]-R[1,2])/s
        x=0.25*s; y=(R[0,1]+R[1,0])/s; z=(R[0,2]+R[2,0])/s
    elif R[1,1]>R[2,2]:
        s=2.*math.sqrt(1+R[1,1]-R[0,0]-R[2,2]); w=(R[0,2]-R[2,0])/s
        x=(R[0,1]+R[1,0])/s; y=0.25*s; z=(R[1,2]+R[2,1])/s
    else:
        s=2.*math.sqrt(1+R[2,2]-R[0,0]-R[1,1]); w=(R[1,0]-R[0,1])/s
        x=(R[0,2]+R[2,0])/s; y=(R[1,2]+R[2,1])/s; z=0.25*s
    return (x,y,z,w)


def _seg_tf(p1, p2):
    """Midpoint + Z-aligned quaternion for a cylinder from p1 to p2."""
    d = p2-p1; L = float(np.linalg.norm(d))
    if L < 1e-6: L = 1e-6
    mid = (p1+p2)*0.5; dn = d/L
    z = np.array([0.,0.,1.])
    cross = np.cross(z,dn); dot = float(np.dot(z,dn)); cn = float(np.linalg.norm(cross))
    if cn < 1e-6:
        R = np.eye(3) if dot > 0 else np.diag([-1.,1.,-1.])
    else:
        ax=cross/cn; ang=math.atan2(cn,dot)
        K=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
        R=np.eye(3)+math.sin(ang)*K+(1-math.cos(ang))*(K@K)
    return mid, _rot_to_quat(R), L


# ── SDF builders ──────────────────────────────────────────────────────────────
def _c(rgba): return f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"

def _sdf_cylinder(name, length, radius, rgba=(0.18,0.42,0.78,1.)):
    c = _c(rgba)
    return (f'<?xml version="1.0"?><sdf version="1.6">'
            f'<model name="{name}"><static>true</static><link name="link">'
            f'<visual name="v"><geometry><cylinder>'
            f'<radius>{radius}</radius><length>{length}</length>'
            f'</cylinder></geometry>'
            f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse>'
            f'<specular>0.15 0.15 0.15 1</specular></material>'
            f'</visual></link></model></sdf>')

def _sdf_sphere(name, radius, rgba=(0.07,0.13,0.20,1.)):
    c = _c(rgba)
    return (f'<?xml version="1.0"?><sdf version="1.6">'
            f'<model name="{name}"><static>true</static><link name="link">'
            f'<visual name="v"><geometry><sphere><radius>{radius}</radius></sphere></geometry>'
            f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>'
            f'</visual></link></model></sdf>')

def _sdf_box(name, size, rgba=(0.10,0.16,0.23,1.)):
    c = _c(rgba); sx,sy,sz = size
    return (f'<?xml version="1.0"?><sdf version="1.6">'
            f'<model name="{name}"><static>true</static><link name="link">'
            f'<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
            f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>'
            f'</visual></link></model></sdf>')

def _sdf_mesh_link(name, mesh_file, vis_xyz, vis_rpy):
    """SDF model wrapping one official UR5 .dae mesh.

    The visual <pose> encodes the URDF visual-origin offset so that when
    Gazebo sets this model's world pose to the FK joint frame, the mesh
    lines up with the real robot geometry.
    """
    x, y, z = vis_xyz
    r, p, yaw = vis_rpy
    uri = f"file://{_MESH_BASE}/{mesh_file}"
    return (f'<?xml version="1.0"?><sdf version="1.6">'
            f'<model name="{name}"><static>true</static>'
            f'<link name="link">'
            f'<visual name="v">'
            f'<pose>{x} {y} {z} {r} {p} {yaw}</pose>'
            f'<geometry><mesh><uri>{uri}</uri></mesh></geometry>'
            f'</visual></link></model></sdf>')


# ── Gazebo world SDF ──────────────────────────────────────────────────────────
WORLD_SDF = """\
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="ur5_twin">
    <!-- gazebo_ros_state is a WorldPlugin — must live in the SDF -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>

    <gravity>0 0 0</gravity>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <light name="fill" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.35 0.38 0.45 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.5 -0.1 -0.9</direction>
    </light>

    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.06 0.09 0.14 1</ambient>
            <diffuse>0.06 0.09 0.14 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <scene>
      <ambient>0.18 0.20 0.26 1</ambient>
      <background>0.04 0.06 0.10 1</background>
      <shadows>false</shadows>
    </scene>

    <gui>
      <camera name="user_camera">
        <pose>1.6 -1.4 1.1 0 0.33 2.38</pose>
      </camera>
    </gui>
  </world>
</sdf>
"""

# ── Sensor cell activation levels ─────────────────────────────────────────────
_CELL_LEVELS = [
    ((0.00, 0.07, 0.55, 1.0), 0.004),  # 0 deep blue  — inactive
    ((0.00, 0.50, 1.00, 1.0), 0.005),  # 1 blue
    ((0.05, 0.85, 0.10, 1.0), 0.006),  # 2 green
    ((1.00, 0.88, 0.00, 1.0), 0.007),  # 3 yellow
    ((1.00, 0.05, 0.05, 1.0), 0.008),  # 4 red — fully active
]
N_LEVELS = len(_CELL_LEVELS)

def _cell_level(v):
    return min(int(max(0., min(1., v)) * N_LEVELS), N_LEVELS - 1)

_ARROW_RGBA = {
    "fx": (1.0, 0.20, 0.20, 1.0),
    "fy": (0.2, 1.00, 0.20, 1.0),
    "fz": (0.3, 0.60, 1.00, 1.0),
    "fr": (1.0, 0.90, 0.10, 1.0),
}
FORCE_SCALE = 0.015   # m per Newton

# ── Shared state ──────────────────────────────────────────────────────────────
_state = {"q": Q_HOME[:], "cells": [0.]*N, "ft": [0.,0.,0.],
          "fz": 0., "connected": False}
_state_lock = threading.Lock()
_running    = [True]


def _read_sensor():
    try:
        with open(SENSOR_SHARED_FILE) as f:
            d = json.load(f)
        if d.get("ready"):
            return d["values"]
    except Exception:
        pass
    return None


def _live_thread(ip):
    RETRY = 5.0; t0 = time.time()

    def _connect():
        try:
            import rtde_receive
            r = rtde_receive.RTDEReceiveInterface(ip)
            print(f"\n[gazebo] RTDE connected to {ip}")
            with _state_lock: _state["connected"] = True
            return r
        except Exception as e:
            print(f"\n[gazebo] Cannot connect to {ip}: {e}")
            with _state_lock: _state["connected"] = False
            return None

    r = _connect(); last_retry = time.time()
    if r is None:
        print(f"[gazebo] Demo mode — retry every {RETRY:.0f}s")

    while _running[0]:
        if r is not None:
            try:
                q = list(r.getActualQ())
                ft = r.getActualTCPForce()
                cells = _read_sensor() or [0.]*N
                with _state_lock:
                    _state.update(q=q, cells=cells,
                                  ft=list(ft[:3]) if ft else [0.,0.,0.],
                                  fz=abs(ft[2]) if ft else 0.,
                                  connected=True)
            except Exception as e:
                print(f"\n[gazebo] RTDE error: {e} — demo mode")
                r = None
                with _state_lock: _state["connected"] = False
                last_retry = time.time()
        else:
            t = time.time() - t0
            q = [0.30*math.sin(0.40*t),
                 -PI/2+0.30*math.sin(0.30*t+1.0),
                  0.40*math.sin(0.50*t+0.5),
                 -PI/2+0.20*math.sin(0.70*t),
                  0.30*math.sin(0.60*t+2.0),
                  0.10*math.sin(t)]
            cells = [max(0.,min(1., 0.4*math.sin(t+i*0.4)*math.sin(t*0.7+i*0.2)+0.2))
                     for i in range(N)]
            with _state_lock:
                _state.update(q=q[:], cells=cells, connected=False)
            if time.time()-last_retry >= RETRY:
                r = _connect(); last_retry = time.time()
        time.sleep(0.05)


# ── Gazebo ROS2 node ──────────────────────────────────────────────────────────
class GazeboVizNode(Node):
    def __init__(self):
        super().__init__('ur5_gazebo_viz')
        self._spawn_cli     = self.create_client(SpawnEntity,    '/spawn_entity')
        self._delete_cli    = self.create_client(DeleteEntity,   '/delete_entity')
        self._set_state_cli = self.create_client(SetEntityState, '/set_entity_state')
        self._cell_lv = [-1] * N   # current rendered level per cell

    # ── Helpers ───────────────────────────────────────────────────────────────
    # ── Service call helpers ──────────────────────────────────────────────────
    def _wait(self, future, timeout=10.):
        """Wait for a future that the background executor is processing."""
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.005)
        return future.result() if future.done() else None

    def wait_services(self, timeout=40.):
        print("[gazebo] Waiting for Gazebo services", end="", flush=True)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if (self._spawn_cli.service_is_ready() and
                    self._set_state_cli.service_is_ready()):
                print(" — ready")
                return True
            time.sleep(0.5)
            print(".", end="", flush=True)
        print("\n[gazebo] ERROR: services timed out")
        return False

    def _spawn(self, name, xml, x=0., y=0., z=0., qx=0., qy=0., qz=0., qw=1.):
        req = SpawnEntity.Request()
        req.name = name; req.xml = xml; req.reference_frame = "world"
        req.initial_pose.position.x    = float(x)
        req.initial_pose.position.y    = float(y)
        req.initial_pose.position.z    = float(z)
        req.initial_pose.orientation.x = float(qx)
        req.initial_pose.orientation.y = float(qy)
        req.initial_pose.orientation.z = float(qz)
        req.initial_pose.orientation.w = float(qw)
        r = self._wait(self._spawn_cli.call_async(req))
        if r and not r.success:
            self.get_logger().warn(f"spawn {name}: {r.status_message}")

    def _delete(self, name):
        req = DeleteEntity.Request(); req.name = name
        self._wait(self._delete_cli.call_async(req), timeout=5.)

    def _set_pose(self, name, pos, quat=(0.,0.,0.,1.)):
        """Synchronous pose update — waits for confirmation from Gazebo."""
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.reference_frame = "world"
        p = Pose()
        p.position.x=float(pos[0]); p.position.y=float(pos[1]); p.position.z=float(pos[2])
        p.orientation.x=float(quat[0]); p.orientation.y=float(quat[1])
        p.orientation.z=float(quat[2]); p.orientation.w=float(quat[3])
        req.state.pose = p; req.state.twist = Twist()
        self._wait(self._set_state_cli.call_async(req), timeout=2.)

    # ── Scene setup ───────────────────────────────────────────────────────────
    def setup_scene(self):
        METAL = (0.78, 0.80, 0.84, 1.)

        # Delete any models left from previous runs (both old names and new names)
        old = ([info[0] for info in _UR5_LINKS] +
               ["ur5_tcp", "ur5_indenter", "sensor_platform"] +
               [f"sensor_cell_{i}" for i in range(N)] +
               [f"arrow_{ax}_{p}" for ax in _ARROW_RGBA for p in ("shaft", "head")] +
               # legacy names from old cylinder/sphere implementation
               [f"ur5_link{i}" for i in range(6)] +
               [f"ur5_joint{i}" for i in range(7)])
        for name in old:
            self._delete(name)

        print("[gazebo] Spawning official UR5 mesh models …")
        # Spawn each link directly at its home FK pose so the robot is
        # visible immediately (no reliance on a first set_entity_state call).
        Ts_home = ur5_fk_full(Q_HOME)
        for name, ts_idx, mesh_file, vis_xyz, vis_rpy in _UR5_LINKS:
            T = Ts_home[ts_idx]
            pos = T[:3, 3]; quat = _rot_to_quat(T[:3, :3])
            self._spawn(name,
                        _sdf_mesh_link(name, mesh_file, vis_xyz, vis_rpy),
                        x=pos[0], y=pos[1], z=pos[2],
                        qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3])

        T6  = Ts_home[6]; tcp = T6[:3, 3]; tzv = T6[:3, 2]
        tip = tcp + tzv * INDENTER_LENGTH
        mid, iquat, _ = _seg_tf(tcp, tip)
        self._spawn("ur5_indenter",
                    _sdf_cylinder("ur5_indenter", INDENTER_LENGTH, INDENTER_RADIUS, METAL),
                    x=mid[0], y=mid[1], z=mid[2],
                    qx=iquat[0], qy=iquat[1], qz=iquat[2], qw=iquat[3])

        self._spawn("sensor_platform",
                    _sdf_box("sensor_platform", (0.040, 0.036, 0.004), (0.10, 0.16, 0.23, 1.)),
                    x=SENSOR_REF[0], y=SENSOR_REF[1], z=SENSOR_REF[2] - 0.003)

        print("[gazebo] Spawning sensor cells …")
        rgba0, r0 = _CELL_LEVELS[0]
        for i in range(N):
            self._spawn(f"sensor_cell_{i}",
                        _sdf_sphere(f"sensor_cell_{i}", r0, rgba0),
                        x=CELL_WORLD_POS[i][0], y=CELL_WORLD_POS[i][1],
                        z=CELL_WORLD_POS[i][2])
            self._cell_lv[i] = 0

        print("[gazebo] Spawning force arrows …")
        for ax, rgba in _ARROW_RGBA.items():
            self._spawn(f"arrow_{ax}_shaft",
                        _sdf_cylinder(f"arrow_{ax}_shaft", 0.001, 0.003, rgba), z=-100.)
            self._spawn(f"arrow_{ax}_head",
                        _sdf_cylinder(f"arrow_{ax}_head",  0.001, 0.009, rgba), z=-100.)

        print("[gazebo] Scene ready!")

    # ── Per-frame updates ─────────────────────────────────────────────────────
    def update_robot(self, q):
        Ts = ur5_fk_full(q)
        for name, ts_idx, _, _, _ in _UR5_LINKS:
            T = Ts[ts_idx]
            pos  = T[:3, 3]
            quat = _rot_to_quat(T[:3, :3])
            self._set_pose(name, pos, quat)
        # Indenter extends from TCP along tool Z axis
        T6  = Ts[6]
        tcp = T6[:3, 3]
        tz  = T6[:3, 2]
        tip = tcp + tz * INDENTER_LENGTH
        mid, quat, _ = _seg_tf(tcp, tip)
        self._set_pose("ur5_indenter", mid, quat)
        return tip

    def update_sensor_cells(self, cells):
        for i in range(N):
            lv = _cell_level(cells[i] if i < len(cells) else 0.)
            if lv == self._cell_lv[i]:
                continue
            rgba, radius = _CELL_LEVELS[lv]
            self._delete(f"sensor_cell_{i}")
            self._spawn(f"sensor_cell_{i}",
                        _sdf_sphere(f"sensor_cell_{i}", radius, rgba),
                        x=CELL_WORLD_POS[i][0], y=CELL_WORLD_POS[i][1],
                        z=CELL_WORLD_POS[i][2])
            self._cell_lv[i] = lv

    def update_force_arrows(self, ft, tcp_tip):
        axes = [("fx", np.array([1.,0.,0.]), ft[0] if len(ft)>0 else 0.),
                ("fy", np.array([0.,1.,0.]), ft[1] if len(ft)>1 else 0.),
                ("fz", np.array([0.,0.,1.]), ft[2] if len(ft)>2 else 0.)]
        fr_v = np.array([ft[i] if i < len(ft) else 0. for i in range(3)])
        fr_m = float(np.linalg.norm(fr_v))

        for ax, dv, mag in axes:
            if abs(mag) < 0.3:
                self._set_pose(f"arrow_{ax}_shaft", [0,0,-100.])
                self._set_pose(f"arrow_{ax}_head",  [0,0,-100.])
                continue
            mid1 = tcp_tip + dv * abs(mag) * FORCE_SCALE * 0.75
            tip  = tcp_tip + dv * abs(mag) * FORCE_SCALE
            ms, qs, _ = _seg_tf(tcp_tip, mid1)
            mh, qh, _ = _seg_tf(mid1, tip)
            self._set_pose(f"arrow_{ax}_shaft", ms, qs)
            self._set_pose(f"arrow_{ax}_head",  mh, qh)

        if fr_m > 0.3:
            fd   = fr_v / fr_m
            mid1 = tcp_tip + fd * fr_m * FORCE_SCALE * 0.75
            tip  = tcp_tip + fd * fr_m * FORCE_SCALE
            ms, qs, _ = _seg_tf(tcp_tip, mid1)
            mh, qh, _ = _seg_tf(mid1, tip)
            self._set_pose("arrow_fr_shaft", ms, qs)
            self._set_pose("arrow_fr_head",  mh, qh)
        else:
            self._set_pose("arrow_fr_shaft", [0,0,-100.])
            self._set_pose("arrow_fr_head",  [0,0,-100.])


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    import argparse
    ap = argparse.ArgumentParser(description="UR5 Gazebo digital twin")
    ap.add_argument("--sim",  action="store_true", help="URSim at localhost")
    ap.add_argument("--ip",   default=None,        help="Override robot IP")
    ap.add_argument("--demo", action="store_true", help="Demo animation only")
    args = ap.parse_args()

    if args.sim:  os.environ["UR_ROBOT_IP"] = "127.0.0.1"
    if args.ip:   os.environ["UR_ROBOT_IP"] = args.ip
    ip = os.environ.get("UR_ROBOT_IP", ROBOT_IP)

    # Clear any stale ready marker from a previous run
    if os.path.exists(GAZEBO_READY_FILE):
        try: os.remove(GAZEBO_READY_FILE)
        except OSError: pass

    with open(WORLD_SDF_PATH, "w") as f:
        f.write(WORLD_SDF)
    print(f"[gazebo] World → {WORLD_SDF_PATH}")

    # Kill any stale Gazebo process
    subprocess.call(["sudo", "pkill", "-9", "-f", "gzserver"], stderr=subprocess.DEVNULL)
    subprocess.call(["sudo", "pkill", "-9", "-f", "gzclient"],  stderr=subprocess.DEVNULL)
    subprocess.call(["sudo", "fuser", "-k", "11345/tcp"],        stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    # Build Gazebo environment — GAZEBO_PLUGIN_PATH must include the ROS2 lib
    # dir so gzserver can locate libgazebo_ros_init.so and libgazebo_ros_factory.so.
    gz_env = os.environ.copy()
    ros2_lib = "/opt/ros/humble/lib"
    existing = gz_env.get("GAZEBO_PLUGIN_PATH", "")
    gz_env["GAZEBO_PLUGIN_PATH"] = f"{ros2_lib}:{existing}" if existing else ros2_lib

    # gzserver: physics + ROS2 bridge (system plugins via -s)
    # libgazebo_ros_state.so (WorldPlugin) is embedded in the SDF.
    srv_cmd = (f"source {_ROS2_SETUP} && "
               f"gzserver --verbose "
               f"-s libgazebo_ros_init.so "
               f"-s libgazebo_ros_factory.so "
               f"-s libgazebo_ros_force_system.so "
               f"{WORLD_SDF_PATH}")
    gz_proc = subprocess.Popen(["bash", "-c", srv_cmd], env=gz_env)

    # gzclient: 3D GUI (launched separately so closing it doesn't kill gzserver)
    cli_cmd = f"source {_ROS2_SETUP} && gzclient --verbose"
    subprocess.Popen(["bash", "-c", cli_cmd], env=gz_env)

    rclpy.init()
    node = GazeboVizNode()

    # Single executor in a background thread — all service calls (_wait,
    # _set_pose fire-and-forget) are processed here throughout the lifetime.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()

    print("[gazebo] Waiting for Gazebo to initialise …")
    time.sleep(8.)

    if not node.wait_services(timeout=40.):
        gz_proc.terminate(); executor.shutdown(); rclpy.shutdown(); return

    # Load the robot + sensor cells at the home pose so they are visible
    # in the Gazebo window immediately.
    node.setup_scene()

    # Signal that the scene is loaded and the robot is visible.  main.py
    # waits on this marker, then asks the user to confirm before moving
    # the robot — the confirmation prompt lives in main.py so it owns the
    # terminal cleanly (this subprocess never reads stdin).
    try:
        with open(GAZEBO_READY_FILE, "w") as f:
            f.write(str(time.time()))
        print(f"[gazebo] Ready marker → {GAZEBO_READY_FILE}")
    except OSError as e:
        print(f"[gazebo] Could not write ready marker: {e}")

    print("[gazebo] Scene loaded — UR5 visible at home pose")

    # Start live/demo thread
    threading.Thread(target=_live_thread, args=(ip,), daemon=True).start()

    print("[gazebo] Running — press Ctrl+C to stop\n")

    SENSOR_HZ = 5.0;  last_sensor = 0.
    FORCE_HZ  = 10.0; last_force  = 0.

    try:
        while _running[0]:
            if gz_proc.poll() is not None:
                print("\n[gazebo] Gazebo window closed"); break

            with _state_lock:
                q     = list(_state["q"])
                cells = list(_state["cells"])
                ft    = list(_state["ft"])
                fz    = _state["fz"]
                conn  = _state["connected"]

            tcp_tip = node.update_robot(q)

            now = time.time()
            if now - last_sensor >= 1./SENSOR_HZ:
                node.update_sensor_cells(cells)
                last_sensor = now
            if now - last_force >= 1./FORCE_HZ:
                node.update_force_arrows(ft, tcp_tip)
                last_force = now

            status = "● ROBOT LIVE" if conn else "○ SIMULATION"
            fr_m   = float(np.linalg.norm(ft))
            n_act  = sum(1 for v in cells if v > 0.05)
            print(f"\r[gazebo] {status}  Fz={fz:+.2f}N  |F|={fr_m:.1f}N  "
                  f"active={n_act:2d}/19", end="", flush=True)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[gazebo] Stopped")
    finally:
        _running[0] = False
        if os.path.exists(GAZEBO_READY_FILE):
            try: os.remove(GAZEBO_READY_FILE)
            except OSError: pass
        executor.shutdown()
        rclpy.shutdown()
        try: gz_proc.terminate(); gz_proc.wait(timeout=5)
        except Exception: pass
        print("[gazebo] Done")


if __name__ == "__main__":
    main()
