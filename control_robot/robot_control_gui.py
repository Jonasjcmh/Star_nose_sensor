"""
robot_control_gui.py — XYZ robot jog + XJC force read-out + Robotiq gripper GUI
===============================================================================
A single Tkinter window to:

  • JOG the UR5e TCP in base X / Y / Z, fully from the KEYBOARD (WASD + Q/E, or
    the arrow keys + R/F), with a selectable step size and travel speed — the
    same control frame / reference pose convention as the rest of the project
    (indentation_test, Integration_2).
  • WATCH the live force/torque from the XJC X-6A 6-axis sensor (RS485/Modbus,
    poll mode) AND the robot's own TCP pose + wrench, refreshed continuously.
  • SEE the sensor wrench as a RIGID-BODY 3D view: a box (the sensor) with a live
    red force arrow and a blue torque arrow, so you read direction at a glance.
  • DRIVE a Robotiq 2-finger gripper (2F-85/140) through the UR URCap socket on
    the robot (port 63352) — open/close from the keyboard (O / C) or buttons,
    plus a position slider with speed/force. Shares the robot's IP, no cabling.

Keyboard map (when the window has focus and you're not typing in a field)
-------------------------------------------------------------------------
  A / D : -X / +X        W / S : +Y / -Y        Q / E : +Z / -Z (up / down)
  ← / → : -X / +X        ↑ / ↓ : +Y / -Y        R / F : +Z / -Z
  O : gripper OPEN       C : gripper CLOSE       + / - : bigger / smaller step
  SPACE : STOP the robot

Everything is optional and degrades gracefully: if the robot, the XJC port, the
gripper URCap or matplotlib is missing, the other panels still work.

Usage
-----
  python3 robot_control_gui.py
  python3 robot_control_gui.py --ip 177.22.22.2 --xjc-port /dev/ttyUSB0
  python3 robot_control_gui.py --no-xjc            # robot + gripper only
  python3 robot_control_gui.py --sim               # no hardware, fake signals
  UR_ROBOT_IP=127.0.0.1 python3 robot_control_gui.py

Safety
------
Jog moves are small relative moveL steps at a travel speed; keep the step size
modest near the sample and keep a hand on the robot's e-stop. The XJC Fz
read-out turns red past a soft warning threshold so you can see contact.
"""

from __future__ import annotations

import os
import sys
import json
import time
import argparse
import threading
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

# Saved home position: HOME_FILE is the "latest" (auto-loaded next session);
# every save also drops a timestamped copy in HOMES_DIR as a per-session history.
HOME_FILE = os.path.join(_HERE, 'home_pose.json')
HOMES_DIR = os.path.join(_HERE, 'homes')

# Routine builder: named routines live in routines/<name>.json. The legacy single
# routine.json is migrated in on first run so older work is never lost.
ROUTINE_FILE = os.path.join(_HERE, 'routine.json')
ROUTINES_DIR = os.path.join(_HERE, 'routines')
ROUTINE_SPEED_DEFAULT = 40.0   # mm/s — moveL speed between positions

from xjc_sensor import (  # noqa: E402  local, reconstructed driver
    XjcSensor, build_request, parse_poll_frame, POLL_REPLY_LEN)
from robotiq_gripper import RobotiqGripper  # noqa: E402
from theme import apply_theme, badge  # noqa: E402  product-style ttk theme (light/dark)
from joystick import JoystickReader, HAVE_PYGAME  # noqa: E402  Xbox/gamepad analog jog

try:
    from scipy.spatial.transform import Rotation as _Rot   # robust rotvec math
    _HAVE_SCIPY = True
except Exception as _e:   # pragma: no cover
    print(f'[rot] scipy unavailable ({_e}) — trigger Z-rotation disabled')
    _HAVE_SCIPY = False

# ── Robot constants (shared with indentation_test / Integration_2) ─────────────
ROBOT_IP = os.environ.get('UR_ROBOT_IP', '177.22.22.2')
GRIPPER_PORT = 63352
ACCEL = 0.3          # m/s^2 — used by moveL (go-to-reference) and stopL
XJC_FZ_WARN_N = 10.0  # |Fz| above this turns the read-out red (soft contact flag)

# ── servoL streaming jog (hold-to-move, smooth) ────────────────────────────────
SERVO_HZ = 125.0            # control-loop rate for the servoL stream
SERVO_DT = 1.0 / SERVO_HZ   # s — segment time handed to servoL each tick
SERVO_LOOKAHEAD = 0.10      # s — servoL smoothing horizon (0.03–0.2; higher = smoother/softer)
SERVO_GAIN = 300            # servoL proportional gain (100–2000; higher = stiffer/snappier)
JOG_SPEED_DEFAULT = 20.0    # mm/s — initial jog velocity
JOG_SPEED_MIN = 1.0
JOG_SPEED_MAX = 150.0
JOG_SPEED_STEP = 5.0        # mm/s per +/- keypress
RELEASE_DEBOUNCE_MS = 40    # ignore the phantom KeyRelease X11 emits during auto-repeat

# Software accel/decel ramp (mm/s²): the commanded Cartesian speed is slew-limited
# toward the target so starts/stops are smooth — matters most at high speeds. The
# decel is deliberately GENTLER than the accel so releasing a key/stick eases the
# TCP to a stop (coasts) instead of braking abruptly.
JOG_ACCEL_MMSS = 350.0      # ramp-up rate
JOG_DECEL_MMSS = 160.0      # ramp-down rate on release (soft, coasting stop)

# Per-axis jog direction sign (base frame). X and Y are inverted to match the real
# installation (the stick/keys were moving opposite to the physical axes).
JOG_AXIS_SIGN = (-1.0, -1.0, 1.0)

# Trigger-driven rotation about base Z (RT − LT). Speed in rad/s at full trigger,
# with its own accel/decel ramp (rad/s²); sign flips the direction.
JOG_ROT_SPEED = 0.5
JOG_ROT_ACCEL = 2.5
JOG_ROT_DECEL = 1.5
JOG_RZ_SIGN = 1.0

# Shared reference start pose (identical to indentation_test.REFERENCE_POSE).
REFERENCE_POSE = [-0.03664, -0.49831, 0.06071, 2.346, -2.094, -0.00009]

# 3D rigid-body view: arrow reaches the "full" length at these reference loads.
FORCE_REF_N = 20.0        # N  — force arrow full-scale
TORQUE_REF_NM = 1.0       # N·m — torque arrow full-scale

# Is matplotlib (with the TkAgg + 3D backends) importable? Checked once here so
# the rigid-body view degrades gracefully when it is not installed.
try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  registers 3d projection
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
    import numpy as _np
    _HAVE_MPL = True
except Exception as _e:   # pragma: no cover - only when matplotlib absent
    print(f'[view] matplotlib unavailable ({_e}) — 3D rigid-body view disabled')
    _HAVE_MPL = False


# ── XJC serial auto-detect (same approach as indentation_test) ─────────────────
def _probe_xjc_port(dev, baud=115200, station=1):
    """True if `dev` answers a Modbus poll request with a valid XJC frame."""
    try:
        import serial
        with serial.Serial(dev, baud, timeout=0.3) as s:
            s.reset_input_buffer()
            s.write(build_request(station))
            buf = bytearray()
            deadline = time.perf_counter() + 0.4
            while len(buf) < POLL_REPLY_LEN and time.perf_counter() < deadline:
                chunk = s.read(POLL_REPLY_LEN - len(buf))
                if not chunk:
                    break
                buf.extend(chunk)
            return parse_poll_frame(bytes(buf), station) is not None
    except Exception:
        return False


def autodetect_xjc_port(baud=115200, station=1, all_ports=False):
    """Probe serial ports and return the first that answers as an XJC, or None.

    By default only ttyUSB*/ttyACM* (where the USB-RS485 adapter appears) are
    probed; `all_ports` also tries legacy ttyS*. Mirrors indentation_test."""
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = []
    if not all_ports:
        ports = [p for p in ports if ('USB' in p or 'ACM' in p)]
    for dev in ports:
        if _probe_xjc_port(dev, baud, station):
            print(f'[XJC] auto-detected on {dev}')
            return dev
        print(f'[XJC] probed {dev} → no XJC')
    return None


# ── XJC poll thread ────────────────────────────────────────────────────────────
class XjcReader:
    """Background poll loop; keeps the latest tared 6-axis wrench thread-safely."""

    def __init__(self, sensor):
        self.sensor = sensor
        self._lock = threading.Lock()
        self._state = {'raw': [0.0] * 6, 'values': [0.0] * 6, 'ok': False}
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                raw = self.sensor.poll_once()
                if raw is None:
                    with self._lock:
                        self._state['ok'] = False
                    continue
                off = self.sensor.offset if len(self.sensor.offset) == 6 else [0.0] * 6
                vals = [r - o for r, o in zip(raw, off)]
                with self._lock:
                    self._state = {'raw': raw, 'values': vals, 'ok': True}
            except Exception:
                with self._lock:
                    self._state['ok'] = False
                time.sleep(0.01)

    def get(self):
        with self._lock:
            return {'raw': list(self._state['raw']),
                    'values': list(self._state['values']),
                    'ok': self._state['ok']}

    def tare(self, n=30):
        return self.sensor.tare(n=n)

    def stop(self):
        self._stop.set()


# ── Simulation doubles ─────────────────────────────────────────────────────────
class _SimRobot:
    """Minimal rtde stand-in: holds a pose, reports a tiny synthetic wrench."""

    def __init__(self):
        self._pose = list(REFERENCE_POSE)
        self._lock = threading.Lock()

    # rtde_control side
    def moveL(self, pose, vel=0.05, accel=0.0, *a, **k):
        with self._lock:
            self._pose = list(pose)
        return True

    def servoL(self, pose, vel=0.5, acc=0.5, t=SERVO_DT, lookahead=0.1, gain=300, *a, **k):
        with self._lock:
            self._pose = list(pose)
        return True

    def servoStop(self, *a, **k):
        pass

    def stopL(self, *a, **k):
        pass

    def stopScript(self):
        pass

    def isConnected(self):
        return True

    def isProgramRunning(self):
        return True

    def reuploadScript(self):
        return True

    def reconnect(self):
        return True

    # rtde_receive side
    def getActualTCPPose(self):
        with self._lock:
            return list(self._pose)

    def getActualTCPForce(self):
        with self._lock:
            depth = max(0.0, (REFERENCE_POSE[2] - self._pose[2]) * 1000.0)
        return [0.0, 0.0, -(2.0 * depth), 0.0, 0.0, 0.0]


class _SimXjc:
    """XjcSensor stand-in: Fz grows as the (sim) TCP presses below the reference."""

    def __init__(self, robot):
        self.robot = robot
        self.offset = [0.0] * 6

    def poll_once(self):
        z = self.robot.getActualTCPPose()[2]
        depth = max(0.0, (REFERENCE_POSE[2] - z) * 1000.0)
        import random
        return [random.uniform(-0.03, 0.03), random.uniform(-0.03, 0.03),
                1.96 * depth + random.uniform(-0.02, 0.02), 0.0, 0.0, 0.0]

    def tare(self, n=30):
        return self.offset


class _SimGripper:
    """RobotiqGripper stand-in: remembers a position, reports ACTIVE."""

    def __init__(self):
        self._pos = 0
        self._active = False

    def connect(self, *a, **k):
        pass

    def activate(self, auto_calibrate=False):
        self._active = True

    def is_active(self):
        return self._active

    def get_current_position(self):
        return self._pos

    def get_open_position(self):
        return 0

    def get_closed_position(self):
        return 255

    def move(self, position, speed, force):
        self._pos = max(0, min(255, int(position)))
        return True, self._pos

    def disconnect(self):
        pass


# ── Rigid-body 3D view ──────────────────────────────────────────────────────────
class RigidBodyView:
    """Embedded matplotlib 3D view of the sensor as a rigid body, showing all 6
    degrees of freedom in the sensor frame:

      • forces  Fx,Fy,Fz → three STRAIGHT arrows along X/Y/Z, AND the box itself
        shifts slightly in the net-force direction
      • torques Mx,My,Mz → three CURVED arrows (arcs) about X/Y/Z, AND the box
        tilts about the net-torque axis

    So you don't just read arrows — you watch the little body push and tilt under
    load. Arrows are colour-coded X=red, Y=green, Z=blue (straight = force, curved
    = torque). There is no grid/axes clutter; magnitudes go in the title.
    """

    # sensor body half-extents (view units; X,Y wide, Z short — a puck)
    _BX, _BY, _BZ = 0.42, 0.42, 0.22
    _AXIS_C = ('#e5484d', '#2aa745', '#3b82f6')   # X, Y, Z
    _TORQUE_R = 0.80          # radius of the torque arcs (view units)
    _TORQUE_SWEEP = 5.2       # rad at full-scale torque (~300°)
    _MAX_DISP = 0.33          # max box translation at full-scale force (view units)
    _MAX_TILT = 0.52          # max box rotation at full-scale torque (rad, ~30°)
    # 8 cuboid corners (index = xi*4 + yi*2 + zi) and the 6 faces as corner loops
    _FACES = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4],
              [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    _FACE_AXIS = (0, 0, 1, 1, 2, 2)               # which axis colours each face

    def __init__(self, parent, force_ref_n=FORCE_REF_N, torque_ref_nm=TORQUE_REF_NM,
                 colors=None):
        self.force_ref = force_ref_n
        self.torque_ref = torque_ref_nm
        c = colors or {}
        self._bg = c.get('surface', 'white')
        self._fg = c.get('text', 'black')
        self._muted = c.get('muted', '0.5')
        self._corners = _np.array(
            [(sx * self._BX, sy * self._BY, sz * self._BZ)
             for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
        self.fig = Figure(figsize=(3.3, 3.3), dpi=100, facecolor=self._bg)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.widget = self.canvas.get_tk_widget()
        self._dyn = []            # dynamic artists cleared each update
        self._draw_static()

    def _draw_static(self):
        ax = self.ax
        ax.clear()
        ax.set_facecolor(self._bg)
        lim = 1.35
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        try:
            ax.set_box_aspect((1, 1, 1))
        except Exception:
            pass
        ax.set_axis_off()         # no grid, panes, ticks or frame — clean floating body
        # faint axis tip labels for orientation (X red, Y green, Z blue)
        for i, lab in enumerate(('X', 'Y', 'Z')):
            p = [0.0, 0.0, 0.0]; p[i] = 1.25
            ax.text(p[0], p[1], p[2], lab, color=self._AXIS_C[i], fontsize=8,
                    ha='center', va='center')
        ax.set_title('|F|= —   |M|= —', fontsize=8, color=self._fg)
        self.canvas.draw()

    @staticmethod
    def _clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    @staticmethod
    def _rot(axis_vec, angle):
        """Rodrigues rotation matrix about `axis_vec` by `angle` (rad)."""
        n = float(_np.linalg.norm(axis_vec))
        if n < 1e-9 or abs(angle) < 1e-9:
            return _np.eye(3)
        k = _np.asarray(axis_vec, float) / n
        K = _np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        return _np.eye(3) + _np.sin(angle) * K + (1 - _np.cos(angle)) * (K @ K)

    def _draw_box(self, R, t):
        """Box faces + edges after rigid transform (rotation R, translation t)."""
        verts = (self._corners @ R.T) + t
        faces = [[verts[i] for i in f] for f in self._FACES]
        fcolors = [self._AXIS_C[self._FACE_AXIS[k]] for k in range(6)]
        poly = Poly3DCollection(faces, facecolors=fcolors, alpha=0.16,
                                edgecolors=self._muted, linewidths=1.2)
        self.ax.add_collection3d(poly)
        return [poly], verts

    def _force_arrow(self, origin, axis, frac, color):
        if abs(frac) < 0.02:
            return []
        L = self._clamp(frac * 1.0, -1.25, 1.25)
        comp = [0.0, 0.0, 0.0]; comp[axis] = L
        q = self.ax.quiver(origin[0], origin[1], origin[2], comp[0], comp[1], comp[2],
                           color=color, linewidth=2.6, arrow_length_ratio=0.16)
        return [q]

    def _torque_arc(self, origin, axis, frac, color):
        if abs(frac) < 0.02:
            return []
        theta = self._clamp(frac, -1.0, 1.0) * self._TORQUE_SWEEP
        u = [0.0, 0.0, 0.0]; v = [0.0, 0.0, 0.0]
        u[(axis + 1) % 3] = 1.0
        v[(axis + 2) % 3] = 1.0
        u = _np.array(u); v = _np.array(v)
        ts = _np.linspace(0.0, theta, 48)
        pts = _np.array(origin) + self._TORQUE_R * (
            _np.outer(_np.cos(ts), u) + _np.outer(_np.sin(ts), v))
        line, = self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=color, linewidth=2.3)
        head = self.ax.quiver(pts[-2, 0], pts[-2, 1], pts[-2, 2],
                              pts[-1, 0] - pts[-2, 0], pts[-1, 1] - pts[-2, 1],
                              pts[-1, 2] - pts[-2, 2], color=color, linewidth=2.3,
                              arrow_length_ratio=6.0)
        return [line, head]

    def update(self, values):
        """Redraw all 6 DOF: box tilt/shift + force arrows + torque arcs."""
        F = _np.asarray(values[:3], dtype=float)
        M = _np.asarray(values[3:6], dtype=float)
        for a in self._dyn:
            try:
                a.remove()
            except Exception:
                pass
        self._dyn = []

        # box response: translate along F, rotate about M (both clamped small)
        f_frac = F / max(self.force_ref, 1e-9)
        m_frac = M / max(self.torque_ref, 1e-9)
        t = _np.array([self._clamp(f_frac[i], -1.0, 1.0) * self._MAX_DISP
                       for i in range(3)])
        ang = self._clamp(float(_np.linalg.norm(m_frac)), 0.0, 1.0) * self._MAX_TILT
        R = self._rot(M, ang)
        box_artists, _ = self._draw_box(R, t)
        self._dyn += box_artists

        # arrows/arcs emanate from the (moved) box centre, so they act on the body
        for i in range(3):
            self._dyn += self._force_arrow(t, i, f_frac[i], self._AXIS_C[i])
            self._dyn += self._torque_arc(t, i, m_frac[i], self._AXIS_C[i])

        self.ax.set_title(
            f'|F|={float(_np.linalg.norm(F)):.2f} N   '
            f'|M|={float(_np.linalg.norm(M)):.3f} N·m', fontsize=8, color=self._fg)
        self.canvas.draw_idle()


# ── GUI ────────────────────────────────────────────────────────────────────────
class ControlGUI:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.rtde_c = None
        self.rtde_r = None
        self.xjc = None
        self.gripper = None
        self._grip_lock = threading.Lock()    # serialise gripper moves (non-blocking)
        self.view = None                      # RigidBodyView (if enabled)
        self._view_tick = 0

        # ── continuous servoL jog state ──────────────────────────────────────────
        self._jog_lock = threading.Lock()
        self._jog_active = {}                 # key-id -> (axis, sign) currently held
        self._release_pending = {}            # key-id -> after() id for debounce
        self._jog_stop = threading.Event()
        self._jog_suspend = threading.Event()  # set during go-to-reference moveL
        self._jog_thread = None
        self._jog_speed_mps = JOG_SPEED_DEFAULT / 1000.0
        self._control_lost = False             # True after the UR control script stops
        self._cur_vel = [0.0, 0.0, 0.0]        # ramped (slew-limited) jog velocity, m/s
        self._cur_wz = 0.0                     # ramped Z-rotation jog velocity, rad/s

        # Xbox / gamepad analog jog (left stick XY, right stick vertical Z)
        self.joy = None
        if HAVE_PYGAME and not getattr(self.args, 'no_joystick', False):
            self.joy = JoystickReader(deadzone=self.args.joy_deadzone).start()

        # Routine builder — two steps: (1) a library of taught POINTS (name+pose),
        # (2) an ordered SEQUENCE of steps that reference points (repeats allowed),
        # each with its own hold duration, plus an iteration count.
        self._points = []
        self._steps = []
        self._routine_stop = threading.Event()
        self._routine_thread = None
        self._routine_running = False

        # saved home pose (loaded from last session if present)
        self._home_pose = None
        self._home_saved_at = self._load_home()

        # force/torque visualisation source (XJC sensor or the UR robot's own wrench)
        self._force_source = tk.StringVar(value='XJC')
        self._ur_offset = [0.0] * 6            # software tare for the UR TCP wrench

        root.title('UR5e Control Console')
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.C, self.FONTS = apply_theme(root, getattr(self.args, 'theme', 'light'))

        self._connect_hardware()
        self._build_ui()
        self._bind_keys()
        if self._robot_ok:
            self._jog_thread = threading.Thread(target=self._jog_streamer, daemon=True)
            self._jog_thread.start()
        self._refresh()   # start the live read-out loop

    # ── hardware ────────────────────────────────────────────────────────────────
    def _connect_hardware(self):
        if self.args.sim:
            sim = _SimRobot()
            self.rtde_c = sim
            self.rtde_r = sim
            self.xjc = XjcReader(_SimXjc(sim))
            self.xjc.start()
            self.gripper = _SimGripper()
            self._robot_ok = True
            self._xjc_ok = True
            self._gripper_connected = False
            return

        # Robot (RTDE)
        self._robot_ok = False
        try:
            import rtde_control
            import rtde_receive
            self.rtde_c = rtde_control.RTDEControlInterface(self.args.ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.args.ip)
            self._robot_ok = True
        except Exception as e:
            print(f'[robot] NOT connected ({e}) — jog disabled')

        # XJC force sensor — use the given port, else auto-detect it.
        self._xjc_ok = False
        if not self.args.no_xjc:
            port = self.args.xjc_port
            if port is None:
                print('[XJC] No --xjc-port given — auto-detecting …')
                port = autodetect_xjc_port(self.args.xjc_baud, self.args.xjc_station,
                                           self.args.all_ports)
            if port is None:
                print('[XJC] Not detected — force read-out disabled '
                      '(try --xjc-port, or --all-ports)')
            else:
                try:
                    sensor = XjcSensor(port=port, baud=self.args.xjc_baud,
                                       station=self.args.xjc_station)
                    sensor.open()
                    self.xjc = XjcReader(sensor)
                    self.xjc.start()
                    self._xjc_ok = True
                    self._xjc_port = port
                except Exception as e:
                    print(f'[XJC] NOT opened on {port} ({e}) — force read-out disabled')

        # Gripper is connected on demand from the UI (needs the robot IP).
        self._gripper_connected = False

    # ── UI layout ───────────────────────────────────────────────────────────────
    def _card(self, parent, title):
        """A flat surface 'card' with a hairline border and a small caps title.
        Returns the inner content frame to grid children into."""
        border = ttk.Frame(parent, style='Border.TFrame')
        inner = ttk.Frame(border, style='Card.TFrame', padding=(16, 14))
        inner.pack(fill='both', expand=True, padx=1, pady=1)   # 1-px border shows through
        ttk.Label(inner, text=title.upper(), style='CardTitle.TLabel'
                  ).grid(row=0, column=0, columnspan=6, sticky='w', pady=(0, 12))
        return border, inner

    def _build_ui(self):
        C = self.C
        self.root.configure(bg=C['bg'])

        # ── Top app bar ───────────────────────────────────────────────────────────
        bar = ttk.Frame(self.root, style='Bar.TFrame', padding=(16, 10))
        bar.grid(row=0, column=0, sticky='ew')
        tk.Frame(self.root, bg=C['border'], height=1).grid(row=1, column=0, sticky='ew')
        ttk.Label(bar, text='●', style='AppTitle.TLabel',
                  foreground=C['accent']).grid(row=0, column=0, padx=(0, 8))
        ttk.Label(bar, text='UR5e Control Console', style='AppTitle.TLabel'
                  ).grid(row=0, column=1, sticky='w')
        bar.columnconfigure(2, weight=1)
        badges = ttk.Frame(bar, style='Bar.TFrame')
        badges.grid(row=0, column=3, sticky='e')
        self._badges = {}
        for i, key in enumerate(('robot', 'control', 'xjc', 'gripper', 'joystick')):
            b = badge(badges, '…', C['muted'], C['surface2'])
            b.grid(row=0, column=i, padx=4)
            self._badges[key] = b

        # ── Tabbed content: Manual control + Routines ───────────────────────────────
        nb = ttk.Notebook(self.root)
        nb.grid(row=2, column=0, sticky='nsew')
        self.root.rowconfigure(2, weight=1)
        self.root.columnconfigure(0, weight=1)
        manual = ttk.Frame(nb, style='TFrame', padding=14)
        routines = ttk.Frame(nb, style='TFrame', padding=14)
        nb.add(manual, text='  Manual control  ')
        nb.add(routines, text='  Routines  ')

        main = manual          # the manual-control cards grid into this tab
        pad = dict(padx=7, pady=4)

        # ── Motion card ─────────────────────────────────────────────────────────────
        jogb, jog = self._card(main, 'Motion · base frame')
        jogb.grid(row=0, column=0, sticky='nsew', padx=(0, 7), pady=(0, 7))

        pad_ = ttk.Frame(jog, style='Card.TFrame')
        pad_.grid(row=1, column=0, columnspan=6, sticky='w')
        self._hold_button(pad_, '+Y', 1, +1).grid(row=0, column=1, **pad)
        self._hold_button(pad_, '–X', 0, -1).grid(row=1, column=0, **pad)
        self._hold_button(pad_, '+X', 0, +1).grid(row=1, column=2, **pad)
        self._hold_button(pad_, '–Y', 1, -1).grid(row=2, column=1, **pad)
        tk.Frame(pad_, bg=C['border'], width=1).grid(row=0, column=3, rowspan=3, sticky='ns', padx=14)
        self._hold_button(pad_, '+Z', 2, +1, width=6).grid(row=0, column=4, **pad)
        ttk.Label(pad_, text='up / down', style='Muted.TLabel').grid(row=1, column=4)
        self._hold_button(pad_, '–Z', 2, -1, width=6).grid(row=2, column=4, **pad)

        spd = ttk.Frame(jog, style='Card.TFrame')
        spd.grid(row=2, column=0, columnspan=6, sticky='ew', pady=(14, 0))
        ttk.Label(spd, text='Jog speed', style='Muted.TLabel').grid(row=0, column=0, padx=(0, 10))
        self._speed_var = tk.DoubleVar(value=JOG_SPEED_DEFAULT)
        ttk.Scale(spd, from_=JOG_SPEED_MIN, to=JOG_SPEED_MAX, orient='horizontal',
                  variable=self._speed_var, length=160,
                  command=self._on_speed_scale).grid(row=0, column=1, sticky='ew')
        self._speed_lbl = ttk.Label(spd, text=f'{JOG_SPEED_DEFAULT:.0f} mm/s', style='Value.TLabel')
        self._speed_lbl.grid(row=0, column=2, padx=(12, 0))

        home = ttk.Frame(jog, style='Card.TFrame')
        home.grid(row=3, column=0, columnspan=6, sticky='ew', pady=(14, 0))
        ttk.Button(home, text='Set home', style='Ghost.TButton',
                   command=self.save_home).grid(row=0, column=0, sticky='ew', **pad)
        self._home_btn_go = ttk.Button(home, text='Go home', style='Ghost.TButton',
                                       command=self.go_home,
                                       state=('normal' if self._home_pose else 'disabled'))
        self._home_btn_go.grid(row=0, column=1, sticky='ew', **pad)
        ttk.Button(home, text='Reference', style='Ghost.TButton',
                   command=self.go_reference).grid(row=0, column=2, sticky='ew', **pad)
        ttk.Button(home, text='Reconnect', style='Primary.TButton',
                   command=self.reconnect_robot).grid(row=1, column=0, columnspan=2, sticky='ew', **pad)
        ttk.Button(home, text='■  STOP', style='Danger.TButton',
                   command=self.stop_robot).grid(row=1, column=2, sticky='ew', **pad)

        self._home_lbl = ttk.Label(jog, text=self._home_text(), style='Muted.TLabel')
        self._home_lbl.grid(row=4, column=0, columnspan=6, sticky='w', pady=(14, 0))
        self._pose_lbl = ttk.Label(jog, text='TCP  —', style='Value.TLabel')
        self._pose_lbl.grid(row=5, column=0, columnspan=6, sticky='w', pady=(4, 0))
        self._ctrl_lbl = ttk.Label(jog, text='', style='Card.TLabel')  # legacy ref (badge used now)

        # ── Force / torque card ─────────────────────────────────────────────────────
        fb, force = self._card(main, 'Force · torque')
        fb.grid(row=0, column=1, sticky='nsew', padx=7, pady=(0, 7))
        # sensor source selector — drives both this read-out and the 3D view
        src = ttk.Frame(force, style='Card.TFrame')
        src.grid(row=1, column=0, columnspan=2, sticky='w', pady=(0, 8))
        ttk.Label(src, text='Source', style='Muted.TLabel').grid(row=0, column=0, padx=(0, 8))
        ttk.Radiobutton(src, text='XJC', value='XJC', variable=self._force_source,
                        command=self._on_source_change).grid(row=0, column=1, padx=(0, 6))
        ttk.Radiobutton(src, text='UR robot', value='UR', variable=self._force_source,
                        command=self._on_source_change).grid(row=0, column=2)
        self._force_hdr = ttk.Label(force, text='', style='Muted.TLabel')
        self._force_hdr.grid(row=2, column=0, columnspan=2, sticky='w', pady=(0, 6))
        self._wrench_lbls = {}
        for i, name in enumerate(['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']):
            ttk.Label(force, text=name, style='Muted.TLabel'
                      ).grid(row=3 + i, column=0, sticky='w', padx=(0, 18), pady=1)
            lbl = ttk.Label(force, text='—', style='Value.TLabel', width=10, anchor='e')
            lbl.grid(row=3 + i, column=1, sticky='e', pady=1)
            self._wrench_lbls[name] = lbl
        self._tare_btn = ttk.Button(force, text='Tare', style='Ghost.TButton',
                                    command=self.tare_active)
        self._tare_btn.grid(row=9, column=0, columnspan=2, sticky='ew', pady=(12, 0))
        self._on_source_change()

        # ── Gripper card ────────────────────────────────────────────────────────────
        gb, grip = self._card(main, 'Robotiq gripper')
        gb.grid(row=0, column=2, sticky='nsew', padx=7, pady=(0, 7))
        self._grip_status = ttk.Label(grip, text='Not connected', style='Muted.TLabel')
        self._grip_status.grid(row=1, column=0, columnspan=2, sticky='w', pady=(0, 6))
        ttk.Button(grip, text='Connect + activate', style='Primary.TButton',
                   command=self.connect_gripper).grid(row=2, column=0, columnspan=2, sticky='ew', pady=(0, 10))
        ttk.Button(grip, text='Open', style='Ghost.TButton',
                   command=lambda: self.gripper_move(0)).grid(row=3, column=0, sticky='ew', **pad)
        ttk.Button(grip, text='Close', style='Ghost.TButton',
                   command=lambda: self.gripper_move(255)).grid(row=3, column=1, sticky='ew', **pad)
        ttk.Label(grip, text='Position   0 open → 255 closed', style='Muted.TLabel'
                  ).grid(row=4, column=0, columnspan=2, sticky='w', pady=(10, 2))
        self._grip_pos = tk.IntVar(value=0)
        self._grip_scale = ttk.Scale(grip, from_=0, to=255, orient='horizontal',
                                     variable=self._grip_pos, length=200)
        self._grip_scale.grid(row=5, column=0, columnspan=2, sticky='ew')
        ttk.Button(grip, text='Go to position', style='Ghost.TButton',
                   command=lambda: self.gripper_move(int(self._grip_pos.get()))
                   ).grid(row=6, column=0, columnspan=2, sticky='ew', pady=(6, 10))
        ttk.Label(grip, text='Speed', style='Muted.TLabel').grid(row=7, column=0, sticky='w')
        self._grip_speed = tk.IntVar(value=255)
        ttk.Spinbox(grip, from_=0, to=255, width=6,
                    textvariable=self._grip_speed).grid(row=7, column=1, sticky='e', pady=2)
        ttk.Label(grip, text='Force', style='Muted.TLabel').grid(row=8, column=0, sticky='w')
        self._grip_force = tk.IntVar(value=150)
        ttk.Spinbox(grip, from_=0, to=255, width=6,
                    textvariable=self._grip_force).grid(row=8, column=1, sticky='e', pady=2)
        self._grip_pos_lbl = ttk.Label(grip, text='pos  —', style='Value.TLabel')
        self._grip_pos_lbl.grid(row=9, column=0, columnspan=2, sticky='w', pady=(10, 0))

        # ── Rigid-body 3D view card ─────────────────────────────────────────────────
        want_view = _HAVE_MPL and not self.args.no_view
        if want_view:
            vb, view = self._card(main, 'Sensor 6-DOF · arrows=force  arcs=torque  (X·Y·Z)')
            vb.grid(row=0, column=3, sticky='nsew', padx=(7, 0), pady=(0, 7))
            self.view = RigidBodyView(view, colors=C)
            self.view.widget.grid(row=1, column=0, sticky='nsew')

        # ── Keyboard hint strip ─────────────────────────────────────────────────────
        kb, legend = self._card(main, 'Keyboard')
        kb.grid(row=1, column=0, columnspan=4, sticky='ew', pady=(0, 0))
        legend_txt = ('HOLD to move   A/D –X/+X   W/S +Y/–Y   Q/E +Z/–Z   arrows jog X/Y · R/F jog Z\n'
                      'Gamepad: left stick XY · right stick Z · LT/RT rotate about Z      '
                      'O/C gripper   +/– speed   SPACE stop')
        ttk.Label(legend, text=legend_txt, style='Muted.TLabel', font=self.FONTS['mono_sm'],
                  justify='left').grid(row=1, column=0, sticky='w')

        # ── Routines tab ────────────────────────────────────────────────────────────
        self._build_routines_tab(routines)

        # ── Status bar ──────────────────────────────────────────────────────────────
        tk.Frame(self.root, bg=C['border'], height=1).grid(row=3, column=0, sticky='ew')
        statusbar = ttk.Frame(self.root, style='Bar.TFrame', padding=(14, 6))
        statusbar.grid(row=4, column=0, sticky='ew')
        self._status = ttk.Label(statusbar, text='Ready', style='Bar.TLabel')
        self._status.pack(side='left')

    # ── Routines tab — Step 1 Points · Step 2 Sequence ───────────────────────────
    def _scroll_area(self, host, height):
        """A vertically scrollable frame inside `host`; returns the inner frame."""
        C = self.C
        canvas = tk.Canvas(host, bg=C['surface'], highlightthickness=0, height=height)
        vsb = ttk.Scrollbar(host, orient='vertical', command=canvas.yview)
        inner = ttk.Frame(canvas, style='Card.TFrame')
        inner.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=inner, anchor='nw')
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side='left', fill='both', expand=True)
        vsb.pack(side='right', fill='y')
        canvas.bind('<Enter>', lambda e: canvas.bind_all(
            '<MouseWheel>', lambda ev: canvas.yview_scroll(int(-ev.delta / 120), 'units')))
        canvas.bind('<Leave>', lambda e: canvas.unbind_all('<MouseWheel>'))
        return inner

    def _build_routines_tab(self, parent):
        """Step 1: teach a library of named POINTS. Step 2: build an ordered SEQUENCE
        of steps that reference those points (repeats OK), each with a hold time, plus
        an iteration count."""
        pad = dict(padx=6, pady=4)
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(0, weight=1)

        # ── Step 1 · Points ───────────────────────────────────────────────────────
        pb, pc = self._card(parent, 'Step 1 · Points   (teach named positions)')
        pb.grid(row=0, column=0, sticky='nsew', padx=(0, 7), pady=(0, 10))
        pc.columnconfigure(0, weight=1); pc.rowconfigure(2, weight=1)
        ph = ttk.Frame(pc, style='Card.TFrame'); ph.grid(row=1, column=0, sticky='ew')
        for t, w in (('Name', 12), ('Captured pose  x, y, z mm', 24), ('', 12)):
            ttk.Label(ph, text=t, style='Muted.TLabel', width=w, anchor='w').pack(side='left', padx=3)
        phost = ttk.Frame(pc, style='Card.TFrame'); phost.grid(row=2, column=0, sticky='nsew', pady=(4, 0))
        self._points_host = self._scroll_area(phost, 230)
        ttk.Button(pc, text='+  Add point', style='Ghost.TButton',
                   command=lambda: self._point_add()).grid(row=3, column=0, sticky='w', pady=(8, 0))

        # ── Step 2 · Sequence ───────────────────────────────────────────────────────
        sb, sc = self._card(parent, 'Step 2 · Sequence   (order · repeat · hold)')
        sb.grid(row=0, column=1, sticky='nsew', padx=(7, 0), pady=(0, 10))
        sc.columnconfigure(0, weight=1); sc.rowconfigure(2, weight=1)
        sh = ttk.Frame(sc, style='Card.TFrame'); sh.grid(row=1, column=0, sticky='ew')
        for t, w in (('#', 3), ('Point', 15), ('Hold s', 7), ('', 10)):
            ttk.Label(sh, text=t, style='Muted.TLabel', width=w, anchor='w').pack(side='left', padx=3)
        shost = ttk.Frame(sc, style='Card.TFrame'); shost.grid(row=2, column=0, sticky='nsew', pady=(4, 0))
        self._steps_host = self._scroll_area(shost, 230)
        ttk.Button(sc, text='+  Add step', style='Ghost.TButton',
                   command=lambda: self._step_add()).grid(row=3, column=0, sticky='w', pady=(8, 0))

        # ── Run bar (spans both columns) ────────────────────────────────────────────
        rb, run = self._card(parent, 'Run')
        rb.grid(row=1, column=0, columnspan=2, sticky='ew')
        # line 1 — iterations / speed / run / stop
        ttk.Label(run, text='Iterations', style='Muted.TLabel').grid(row=1, column=0, sticky='w', **pad)
        self._routine_iters = tk.IntVar(value=1)
        ttk.Spinbox(run, from_=1, to=99999, width=6,
                    textvariable=self._routine_iters).grid(row=1, column=1, **pad)
        ttk.Label(run, text='Move speed mm/s', style='Muted.TLabel').grid(row=1, column=2, sticky='w', **pad)
        self._routine_speed = tk.DoubleVar(value=ROUTINE_SPEED_DEFAULT)
        ttk.Spinbox(run, from_=1, to=250, width=6,
                    textvariable=self._routine_speed).grid(row=1, column=3, **pad)
        ttk.Button(run, text='▶  Run', style='Primary.TButton',
                   command=self.run_routine).grid(row=1, column=4, sticky='ew', **pad)
        ttk.Button(run, text='■  Stop', style='Danger.TButton',
                   command=self.stop_routine).grid(row=1, column=5, sticky='ew', **pad)
        # line 2 — named routines: pick / type a name, then Save / Load / Delete
        ttk.Label(run, text='Routine', style='Muted.TLabel').grid(row=2, column=0, sticky='w', **pad)
        self._routine_name = tk.StringVar(value='my_routine')
        self._routine_picker = ttk.Combobox(run, textvariable=self._routine_name, width=18,
                                            values=self._routine_list())
        self._routine_picker.grid(row=2, column=1, columnspan=2, sticky='w', **pad)
        self._routine_picker.bind('<<ComboboxSelected>>', lambda e: self.load_routine())
        ttk.Button(run, text='Save', style='Ghost.TButton',
                   command=self.save_routine).grid(row=2, column=3, sticky='ew', **pad)
        ttk.Button(run, text='Load', style='Ghost.TButton',
                   command=self.load_routine).grid(row=2, column=4, sticky='ew', **pad)
        ttk.Button(run, text='Delete', style='Ghost.TButton',
                   command=self.delete_routine).grid(row=2, column=5, sticky='ew', **pad)
        self._routine_lbl = ttk.Label(run, text='idle', style='Value.TLabel')
        self._routine_lbl.grid(row=3, column=0, columnspan=6, sticky='w', **pad)

        self._startup_load_routine()

    # ── Step 1 · points library ──────────────────────────────────────────────────
    @staticmethod
    def _pose_txt(p):
        return f'{p[0]*1000:+.1f}, {p[1]*1000:+.1f}, {p[2]*1000:+.1f}'

    def _point_names(self):
        return [p['name'].get() for p in self._points]

    def _point_add(self, name=None, pose=None):
        i = len(self._points)
        row = ttk.Frame(self._points_host, style='Card.TFrame'); row.pack(fill='x', pady=1)
        name_v = tk.StringVar(value=name if name is not None else f'P{i + 1}')
        pt = {'name': name_v, 'pose': pose, 'lbl': None, 'frame': row}
        ttk.Entry(row, textvariable=name_v, width=12).pack(side='left', padx=3)
        name_v.trace_add('write', lambda *a: self._refresh_step_points())
        lbl = ttk.Label(row, width=24, anchor='w', style='Value.TLabel', text='— not set —')
        lbl.pack(side='left', padx=3); pt['lbl'] = lbl
        ttk.Button(row, text='Set', style='Ghost.TButton', width=4,
                   command=lambda p=pt: self._point_capture(p)).pack(side='left', padx=2)
        ttk.Button(row, text='Go', style='Ghost.TButton', width=4,
                   command=lambda p=pt: self._point_goto(p)).pack(side='left', padx=2)
        ttk.Button(row, text='✕', style='Ghost.TButton', width=3,
                   command=lambda p=pt: self._point_remove(p)).pack(side='left', padx=2)
        if pose:
            lbl.config(text=self._pose_txt(pose))
        self._points.append(pt)
        self._refresh_step_points()
        return pt

    def _point_remove(self, pt):
        if self._routine_running:
            return
        try:
            pt['frame'].destroy()
        except Exception:
            pass
        if pt in self._points:
            self._points.remove(pt)
        for st in self._steps:          # unlink steps that referenced it
            if st['point'] is pt:
                st['point'] = None
        self._refresh_step_points()

    def _point_capture(self, pt):
        if not self._robot_ok:
            self._set_status('Robot not connected — cannot capture pose')
            return
        try:
            pose = [float(v) for v in self.rtde_r.getActualTCPPose()]
        except Exception as e:
            self._set_status(f'Capture failed: {e}')
            return
        pt['pose'] = pose
        pt['lbl'].config(text=self._pose_txt(pose))
        self._set_status(f'Captured {pt["name"].get()}')

    def _point_goto(self, pt):
        if pt['pose'] is None:
            self._set_status(f'{pt["name"].get()} has no pose yet — press Set')
            return
        if self._routine_running:
            return
        self._move_to_pose(pt['pose'], pt['name'].get())

    # ── Step 2 · sequence ─────────────────────────────────────────────────────────
    def _step_add(self, point=None, hold=1.0):
        row = ttk.Frame(self._steps_host, style='Card.TFrame'); row.pack(fill='x', pady=1)
        hold_v = tk.DoubleVar(value=hold)
        st = {'point': point, 'hold': hold_v, 'num': None, 'combo': None, 'frame': row}
        num = ttk.Label(row, width=3, anchor='w', style='Muted.TLabel'); num.pack(side='left', padx=3)
        st['num'] = num
        combo = ttk.Combobox(row, width=14, state='readonly', values=self._point_names())
        combo.pack(side='left', padx=3); st['combo'] = combo
        if point is not None:
            combo.set(point['name'].get())
        combo.bind('<<ComboboxSelected>>', lambda e, s=st: self._step_pick(s))
        ttk.Spinbox(row, from_=0, to=600, increment=0.5, width=6,
                    textvariable=hold_v).pack(side='left', padx=3)
        ttk.Button(row, text='↑', style='Ghost.TButton', width=3,
                   command=lambda s=st: self._step_move(s, -1)).pack(side='left', padx=1)
        ttk.Button(row, text='↓', style='Ghost.TButton', width=3,
                   command=lambda s=st: self._step_move(s, +1)).pack(side='left', padx=1)
        ttk.Button(row, text='✕', style='Ghost.TButton', width=3,
                   command=lambda s=st: self._step_remove(s)).pack(side='left', padx=1)
        self._steps.append(st)
        self._renumber_steps()
        return st

    def _step_pick(self, st):
        name = st['combo'].get()
        st['point'] = next((p for p in self._points if p['name'].get() == name), None)

    def _step_remove(self, st):
        if self._routine_running:
            return
        try:
            st['frame'].destroy()
        except Exception:
            pass
        if st in self._steps:
            self._steps.remove(st)
        self._renumber_steps()

    def _step_move(self, st, delta):
        if self._routine_running:
            return
        i = self._steps.index(st)
        j = i + delta
        if 0 <= j < len(self._steps):
            self._steps[i], self._steps[j] = self._steps[j], self._steps[i]
            for s in self._steps:       # re-pack in new order
                s['frame'].pack_forget()
            for s in self._steps:
                s['frame'].pack(fill='x', pady=1)
            self._renumber_steps()

    def _renumber_steps(self):
        for i, s in enumerate(self._steps):
            s['num'].config(text=str(i + 1))

    def _refresh_step_points(self):
        """Keep every step's point dropdown in sync with the current point names."""
        names = self._point_names()
        for s in self._steps:
            pt = s['point']
            cur = pt['name'].get() if (pt in self._points) else ''
            s['combo'].config(values=names)
            s['combo'].set(cur if cur in names else '')

    def _routine_sequence(self):
        """Ordered (name, pose, hold) for steps whose point exists and has a pose."""
        out = []
        for s in self._steps:
            p = s['point']
            if p in self._points and p['pose'] is not None:
                out.append((p['name'].get(), list(p['pose']), max(0.0, float(s['hold'].get()))))
        return out

    def run_routine(self):
        if not self._robot_ok:
            self._set_status('Robot not connected — cannot run routine')
            return
        if self._routine_running:
            return
        steps = self._routine_sequence()
        if not steps:
            self._set_status('Sequence empty — add steps that point to a Set position')
            return
        iters = max(1, int(self._routine_iters.get()))
        speed = max(0.005, float(self._routine_speed.get()) / 1000.0)
        names = ' → '.join(n for n, _, _ in steps)
        if not messagebox.askyesno('Run routine',
                                    f'Run {len(steps)} steps for {iters} iteration(s)?\n\n'
                                    f'{names}\n\nMake sure the path is clear.'):
            return
        self._routine_stop.clear()
        self._routine_running = True     # set on the main thread so it's seen at once
        self._routine_thread = threading.Thread(
            target=self._routine_worker, args=(steps, iters, speed), daemon=True)
        self._routine_thread.start()

    def _routine_worker(self, steps, iters, speed):
        self._jog_suspend.set()          # hand the robot to the routine (no servo jog)
        self._jog_clear_all()
        try:
            self.rtde_c.servoStop()
        except Exception:
            pass
        try:
            for it in range(iters):
                if self._routine_stop.is_set():
                    break
                for name, pose, hold in steps:
                    if self._routine_stop.is_set():
                        break
                    self._set_routine(f'iter {it+1}/{iters}  →  moving to {name}')
                    try:
                        self.rtde_c.moveL(list(pose), speed, ACCEL)
                    except Exception as e:
                        self._set_routine(f'move to {name} failed: {e}')
                        self._routine_stop.set()
                        break
                    self._set_routine(f'iter {it+1}/{iters}  →  holding {name}  ({hold:.1f}s)')
                    # interruptible hold
                    t_end = time.time() + hold
                    while time.time() < t_end and not self._routine_stop.is_set():
                        time.sleep(0.05)
            done = 'stopped' if self._routine_stop.is_set() else 'complete'
            self._set_routine(f'Routine {done}')
            self._set_status(f'Routine {done}')
        finally:
            self._routine_running = False
            self._jog_suspend.clear()

    def stop_routine(self):
        if self._routine_running:
            self._routine_stop.set()
            self._set_status('Stopping routine…')

    def _set_routine(self, msg):
        try:
            self.root.after(0, lambda: self._routine_lbl.config(text=msg))
        except Exception:
            pass

    # ── named routines: list / save / load / delete (routines/<name>.json) ───────
    @staticmethod
    def _routine_sanitize(name):
        name = (name or '').strip() or 'routine'
        return ''.join(c if (c.isalnum() or c in '-_ ') else '_' for c in name)

    def _routine_list(self):
        try:
            names = [os.path.splitext(f)[0] for f in os.listdir(ROUTINES_DIR)
                     if f.endswith('.json')]
        except Exception:
            names = []
        return sorted(names)

    def _refresh_routine_picker(self):
        try:
            self._routine_picker.config(values=self._routine_list())
        except Exception:
            pass

    def save_routine(self):
        name = self._routine_sanitize(self._routine_name.get())
        self._routine_name.set(name)
        data = {
            'iterations': int(self._routine_iters.get()),
            'speed_mm_s': float(self._routine_speed.get()),
            'points': [{'name': p['name'].get(),
                        'pose': ([round(v, 6) for v in p['pose']] if p['pose'] else None)}
                       for p in self._points],
            'sequence': [{'point': (s['point']['name'].get() if s['point'] in self._points else None),
                          'hold': float(s['hold'].get())} for s in self._steps],
        }
        try:
            os.makedirs(ROUTINES_DIR, exist_ok=True)
            with open(os.path.join(ROUTINES_DIR, f'{name}.json'), 'w') as f:
                json.dump(data, f, indent=2)
            self._refresh_routine_picker()
            self._set_status(f'Routine "{name}" saved')
        except Exception as e:
            self._set_status(f'Save failed: {e}')

    def load_routine(self):
        name = self._routine_sanitize(self._routine_name.get())
        path = os.path.join(ROUTINES_DIR, f'{name}.json')
        if not os.path.exists(path):
            self._set_status(f'Routine "{name}" not found')
            return
        self._load_routine_file(path)
        self._routine_name.set(name)
        self._set_status(f'Routine "{name}" loaded')

    def delete_routine(self):
        name = self._routine_sanitize(self._routine_name.get())
        path = os.path.join(ROUTINES_DIR, f'{name}.json')
        if not os.path.exists(path):
            self._set_status(f'Routine "{name}" not found')
            return
        if not messagebox.askyesno('Delete routine', f'Delete routine "{name}"?'):
            return
        try:
            os.remove(path)
        except Exception as e:
            self._set_status(f'Delete failed: {e}')
            return
        self._refresh_routine_picker()
        self._set_status(f'Routine "{name}" deleted')

    def _clear_points(self):
        for p in list(self._points):
            try:
                p['frame'].destroy()
            except Exception:
                pass
        self._points = []

    def _clear_steps(self):
        for s in list(self._steps):
            try:
                s['frame'].destroy()
            except Exception:
                pass
        self._steps = []

    def _startup_load_routine(self):
        """Migrate the legacy routine.json (once), then auto-load the newest routine
        in routines/ — or seed empty rows if there are none."""
        os.makedirs(ROUTINES_DIR, exist_ok=True)
        if os.path.exists(ROUTINE_FILE) and not self._routine_list():
            try:
                import shutil
                shutil.copy(ROUTINE_FILE, os.path.join(ROUTINES_DIR, 'my_routine.json'))
            except Exception:
                pass
        names = self._routine_list()
        self._refresh_routine_picker()
        if names:
            newest = max(names, key=lambda n: os.path.getmtime(
                os.path.join(ROUTINES_DIR, f'{n}.json')))
            self._routine_name.set(newest)
            self._load_routine_file(os.path.join(ROUTINES_DIR, f'{newest}.json'))
        else:
            for _ in range(3):
                self._point_add()

    def _load_routine_file(self, path):
        try:
            with open(path) as f:
                d = json.load(f)
        except Exception as e:
            self._set_status(f'Load failed: {e}')
            return
        self._routine_iters.set(int(d.get('iterations', 1)))
        self._routine_speed.set(float(d.get('speed_mm_s', ROUTINE_SPEED_DEFAULT)))
        self._clear_points(); self._clear_steps()
        for pd in d.get('points', []):
            self._point_add(name=pd.get('name'), pose=(pd.get('pose') or None))
        if not self._points:
            self._point_add()
        for sd in d.get('sequence', []):
            pt = next((p for p in self._points if p['name'].get() == sd.get('point')), None)
            self._step_add(point=pt, hold=float(sd.get('hold', 1.0)))
        self._refresh_step_points()

    # ── keyboard ─────────────────────────────────────────────────────────────────
    def _bind_keys(self):
        """Full keyboard control. Jog keys are HOLD-to-move (KeyPress starts a
        continuous servoL jog on that axis, KeyRelease stops it, de-bounced against
        X11 auto-repeat). Action keys are one-shot. All ignored while typing."""
        # base-frame jog: axis 0=X, 1=Y, 2=Z
        jog_keys = {
            'a': (0, -1), 'd': (0, +1), 'Left': (0, -1), 'Right': (0, +1),
            'w': (1, +1), 's': (1, -1), 'Up': (1, +1), 'Down': (1, -1),
            'q': (2, +1), 'e': (2, -1), 'r': (2, +1), 'f': (2, -1),
        }
        for key, (axis, sign) in jog_keys.items():
            for k in self._key_variants(key):
                self.root.bind(f'<KeyPress-{k}>',
                               lambda ev, kk=key, a=axis, s=sign: self._key_press(ev, kk, a, s))
                self.root.bind(f'<KeyRelease-{k}>',
                               lambda ev, kk=key: self._key_release(ev, kk))

        # one-shot action keys
        for k in ('o', 'O'):
            self.root.bind(f'<KeyPress-{k}>', lambda ev: self._guarded(ev, lambda: self.gripper_move(0)))
        for k in ('c', 'C'):
            self.root.bind(f'<KeyPress-{k}>', lambda ev: self._guarded(ev, lambda: self.gripper_move(255)))
        self.root.bind('<space>', lambda ev: self._guarded(ev, self.stop_robot))
        for k in ('plus', 'equal', 'KP_Add'):
            self.root.bind(f'<KeyPress-{k}>', lambda ev: self._guarded(ev, lambda: self.change_speed(+1)))
        for k in ('minus', 'KP_Subtract'):
            self.root.bind(f'<KeyPress-{k}>', lambda ev: self._guarded(ev, lambda: self.change_speed(-1)))

    @staticmethod
    def _key_variants(key):
        return [key, key.upper()] if (len(key) == 1 and key.isalpha()) else [key]

    def _is_typing(self):
        return isinstance(self.root.focus_get(),
                          (ttk.Spinbox, ttk.Combobox, tk.Entry, tk.Spinbox))

    def _guarded(self, event, fn):
        if not self._is_typing():
            fn()

    def _key_press(self, event, key, axis, sign):
        if self._is_typing():
            return
        # a KeyPress arriving for a key with a pending release == X11 auto-repeat:
        # cancel the scheduled stop so the motion stays continuous.
        aid = self._release_pending.pop(key, None)
        if aid is not None:
            self.root.after_cancel(aid)
        self._jog_set(key, axis, sign)

    def _key_release(self, event, key):
        # debounce: don't stop immediately; a real release has no following KeyPress.
        aid = self._release_pending.pop(key, None)
        if aid is not None:
            self.root.after_cancel(aid)
        self._release_pending[key] = self.root.after(
            RELEASE_DEBOUNCE_MS, lambda: self._debounced_clear(key))

    def _debounced_clear(self, key):
        self._release_pending.pop(key, None)
        self._jog_clear(key)

    def change_speed(self, direction):
        """Nudge the jog velocity up/down with the +/- keys."""
        v = self._speed_var.get() + direction * JOG_SPEED_STEP
        v = max(JOG_SPEED_MIN, min(JOG_SPEED_MAX, v))
        self._speed_var.set(v)
        self._jog_speed_mps = v / 1000.0
        self._speed_lbl.config(text=f'{v:5.0f} mm/s')
        self._set_status(f'Jog speed = {v:.0f} mm/s')

    def _on_speed_scale(self, _value):
        v = self._speed_var.get()
        self._jog_speed_mps = v / 1000.0
        self._speed_lbl.config(text=f'{v:5.0f} mm/s')

    # ── continuous servoL jog ────────────────────────────────────────────────────
    def _hold_button(self, parent, text, axis, sign, width=6):
        """A jog button that moves while pressed and stops on release."""
        b = ttk.Button(parent, text=text, width=width, style='Jog.TButton', takefocus=False)
        key = f'btn:{text}'
        b.bind('<ButtonPress-1>', lambda ev: self._jog_set(key, axis, sign))
        b.bind('<ButtonRelease-1>', lambda ev: self._jog_clear(key))
        b.bind('<Leave>', lambda ev: self._jog_clear(key))   # pointer slips off → stop
        return b

    def _jog_set(self, key, axis, sign):
        if not self._robot_ok:
            self._set_status('Robot not connected — cannot jog')
            return
        with self._jog_lock:
            self._jog_active[key] = (axis, sign)

    def _jog_clear(self, key):
        with self._jog_lock:
            self._jog_active.pop(key, None)

    def _jog_clear_all(self):
        with self._jog_lock:
            self._jog_active.clear()

    def _current_jog_velocity(self):
        """Target base-frame velocity (m/s) from keys/buttons AND the analog stick.
        Keys contribute ±1 per axis; the joystick adds its continuous fraction, so
        stick deflection scales speed proportionally. Opposing inputs cancel."""
        axes = [0.0, 0.0, 0.0]
        with self._jog_lock:
            for (axis, sign) in self._jog_active.values():
                axes[axis] += sign
        if self.joy is not None and self.joy.connected():
            jv = self.joy.get()                     # [fx, fy, fz] in [-1, 1]
            for i in range(3):
                axes[i] += jv[i]
        s = self._jog_speed_mps
        return [JOG_AXIS_SIGN[i] * max(-1.0, min(1.0, axes[i])) * s for i in range(3)]

    def _current_jog_wz(self):
        """Target Z-rotation velocity (rad/s) from the gamepad triggers (RT − LT)."""
        if not _HAVE_SCIPY or self.joy is None or not self.joy.connected():
            return 0.0
        v = self.joy.get()
        rz = v[3] if len(v) > 3 else 0.0
        return JOG_RZ_SIGN * max(-1.0, min(1.0, rz)) * JOG_ROT_SPEED

    def _jog_streamer(self):
        """~125 Hz loop: while any jog input is held, stream servoL targets along the
        base-frame direction; when none are held (or suspended), servoStop + idle.

        The target is advanced by the MEASURED elapsed time each tick (not a fixed
        period) so the achieved Cartesian speed equals the commanded mm/s even if
        the loop jitters; servoL is told the same dt so it paces the segment."""
        target = None
        servoing = False
        last_t = None
        health = 0
        while not self._jog_stop.is_set():
            # If the UR control script has stopped (protective stop / mode change),
            # don't hammer servoL — idle until the user reconnects.
            if self._control_lost:
                target, servoing, last_t = None, False, None
                self._cur_vel = [0.0, 0.0, 0.0]; self._cur_wz = 0.0
                time.sleep(0.3)
                continue

            t0 = time.perf_counter()
            suspended = self._jog_suspend.is_set()
            tgt = [0.0, 0.0, 0.0] if suspended else self._current_jog_velocity()
            tgt_wz = 0.0 if suspended else self._current_jog_wz()
            dt = SERVO_DT if last_t is None else t0 - last_t
            dt = min(max(dt, 0.002), 0.05)   # clamp out scheduling spikes

            # Slew-limit the applied velocity toward the target (accel/decel ramp).
            accel = JOG_ACCEL_MMSS / 1000.0
            decel = JOG_DECEL_MMSS / 1000.0
            cur = self._cur_vel
            for i in range(3):
                dv = tgt[i] - cur[i]
                # decelerating if the target pulls the speed toward zero
                ramping_down = abs(tgt[i]) < abs(cur[i]) or (tgt[i] * cur[i] < 0)
                lim = (decel if ramping_down else accel) * dt
                if dv > lim:
                    cur[i] += lim
                elif dv < -lim:
                    cur[i] -= lim
                else:
                    cur[i] = tgt[i]
            # same ramp for the Z-rotation velocity (triggers)
            dwz = tgt_wz - self._cur_wz
            rlim = (JOG_ROT_DECEL if abs(tgt_wz) < abs(self._cur_wz) or tgt_wz * self._cur_wz < 0
                    else JOG_ROT_ACCEL) * dt
            if dwz > rlim:
                self._cur_wz += rlim
            elif dwz < -rlim:
                self._cur_wz -= rlim
            else:
                self._cur_wz = tgt_wz

            moving = (any(abs(v) > 1e-6 for v in cur) or any(abs(v) > 1e-12 for v in tgt)
                      or abs(self._cur_wz) > 1e-6 or abs(tgt_wz) > 1e-9)
            try:
                if moving:
                    # throttled health check (~15 Hz): servoL here does NOT raise when
                    # the control script dies, so we must poll isProgramRunning.
                    health += 1
                    if health >= 8:
                        health = 0
                        if not self.rtde_c.isProgramRunning():
                            self._on_control_lost()
                            target, servoing, last_t = None, False, None
                            self._cur_vel = [0.0, 0.0, 0.0]
                            self._cur_wz = 0.0
                            continue
                    if target is None:
                        target = list(self.rtde_r.getActualTCPPose())
                    for i in range(3):
                        target[i] += cur[i] * dt
                    if _HAVE_SCIPY and abs(self._cur_wz) > 1e-9:
                        # rotate the TCP about BASE Z: compose Rz(dθ) · R_current
                        dth = self._cur_wz * dt
                        R_new = _Rot.from_rotvec([0.0, 0.0, dth]) * _Rot.from_rotvec(target[3:6])
                        target[3:6] = list(R_new.as_rotvec())
                    self.rtde_c.servoL(target, 0.5, 0.5, dt, SERVO_LOOKAHEAD, SERVO_GAIN)
                    servoing = True
                    last_t = t0
                else:
                    if servoing:
                        try:
                            self.rtde_c.servoStop()
                        except Exception:
                            pass
                        servoing = False
                    target = None
                    last_t = None
            except Exception as e:
                self._set_status(f'servo error: {e}')
                target = None
                servoing = False
                last_t = None
                self._cur_vel = [0.0, 0.0, 0.0]; self._cur_wz = 0.0
            if moving:
                rem = SERVO_DT - (time.perf_counter() - t0)
                time.sleep(rem if rem > 0 else 0.0005)
            else:
                time.sleep(0.01)   # idle: no need to spin at 125 Hz

    def _move_to_pose(self, pose, label):
        """Suspend the servo stream, servoStop, then moveL to `pose` (named `label`)."""
        def _run():
            self._jog_suspend.set()
            self._jog_clear_all()
            time.sleep(3 * SERVO_DT)
            try:
                self.rtde_c.servoStop()
            except Exception:
                pass
            speed = max(0.005, self._jog_speed_mps)
            try:
                self.rtde_c.moveL(list(pose), speed, ACCEL)
                self._set_status(f'Moved to {label}')
            except Exception as e:
                self._set_status(f'Move to {label} rejected: {e}')
            finally:
                self._jog_suspend.clear()
        threading.Thread(target=_run, daemon=True).start()

    def go_reference(self):
        if not self._robot_ok:
            return
        if messagebox.askyesno('Go to reference pose',
                               'Move the TCP to the shared reference pose?\n'
                               'Make sure the path is clear.'):
            self._move_to_pose(REFERENCE_POSE, 'reference pose')

    # ── home position (per session) ──────────────────────────────────────────────
    def _home_text(self):
        if self._home_pose is None:
            return 'home: not set'
        p = self._home_pose
        when = f'  @ {self._home_saved_at}' if self._home_saved_at else ''
        return (f'home  x={p[0]*1000:+.1f} y={p[1]*1000:+.1f} z={p[2]*1000:+.1f} mm{when}')

    def _load_home(self):
        """Load the last saved home pose from HOME_FILE; returns its save-time or None."""
        try:
            with open(HOME_FILE) as f:
                d = json.load(f)
            self._home_pose = [float(v) for v in d['pose']]
            return d.get('saved_at')
        except Exception:
            self._home_pose = None
            return None

    def save_home(self):
        """Capture the current TCP pose as this session's home: write HOME_FILE plus
        a timestamped copy in homes/, and enable Go home."""
        if not self._robot_ok:
            self._set_status('Robot not connected — cannot save home')
            return
        try:
            pose = [float(v) for v in self.rtde_r.getActualTCPPose()]
        except Exception as e:
            self._set_status(f'Could not read pose: {e}')
            return
        ts = datetime.now()
        self._home_pose = pose
        self._home_saved_at = ts.isoformat(timespec='seconds')
        data = {'pose': [round(v, 6) for v in pose], 'saved_at': self._home_saved_at}
        try:
            os.makedirs(HOMES_DIR, exist_ok=True)
            with open(os.path.join(HOMES_DIR, f'home_{ts.strftime("%Y%m%d_%H%M%S")}.json'), 'w') as f:
                json.dump(data, f, indent=2)
            with open(HOME_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            where = 'saved'
        except Exception as e:
            where = f'kept for this session (file write failed: {e})'
        self._home_btn_go.config(state='normal')
        self._home_lbl.config(text=self._home_text())
        self._set_status(f'Home {where}: '
                         f'x={pose[0]*1000:+.1f} y={pose[1]*1000:+.1f} z={pose[2]*1000:+.1f} mm')

    def go_home(self):
        if not self._robot_ok or self._home_pose is None:
            self._set_status('No home saved yet — use "Set home" first')
            return
        if messagebox.askyesno('Go home',
                               'Move the TCP to the saved home position?\n'
                               'Make sure the path is clear.'):
            self._move_to_pose(self._home_pose, 'home')

    def stop_robot(self):
        self._routine_stop.set()          # also halts a running routine
        self._jog_clear_all()
        try:
            self.rtde_c.servoStop()
        except Exception:
            pass
        try:
            self.rtde_c.stopL(2.0)
        except Exception:
            pass
        self._set_status('STOP sent')

    # ── control-script loss / recovery ───────────────────────────────────────────
    def _on_control_lost(self):
        """The UR control script stopped (protective stop, e-stop, or Local mode).
        Stop jogging and tell the user; recovery is via the Reconnect button."""
        if self._control_lost:
            return
        self._control_lost = True
        self._jog_clear_all()
        self._set_status('⚠ Robot control stopped (protective/e-stop or Local mode) — '
                         'clear it on the pendant, then press Reconnect')

    def reconnect_robot(self):
        """Restart the RTDE control script after a stop (user cleared it on pendant)."""
        if not self._robot_ok:
            return

        def _run():
            self._set_status('Reconnecting robot control…')
            try:
                if not self.rtde_c.isConnected():
                    self.rtde_c.reconnect()
                self.rtde_c.reuploadScript()
                time.sleep(0.5)
                if self.rtde_c.isConnected() and self.rtde_c.isProgramRunning():
                    self._control_lost = False
                    self._set_status('Robot control restored — jog ready')
                else:
                    self._set_status('Reconnect failed — clear any protective/e-stop on '
                                     'the pendant (and set Remote), then retry')
            except Exception as e:
                self._set_status(f'Reconnect error: {e}')
        threading.Thread(target=_run, daemon=True).start()

    # ── force/torque source (XJC sensor vs UR robot wrench) ──────────────────────
    def _on_source_change(self):
        src = self._force_source.get()
        if src == 'UR':
            self._force_hdr.config(text='UR TCP wrench   (N, N·m)')
            self._tare_btn.config(text='Tare UR')
        else:
            self._force_hdr.config(text='XJC 6-axis   (N, N·m)')
            self._tare_btn.config(text='Tare XJC')

    def _active_wrench(self):
        """Return (values6, ok) for the selected source, tared."""
        if self._force_source.get() == 'UR':
            if not self._robot_ok:
                return [0.0] * 6, False
            try:
                raw = self.rtde_r.getActualTCPForce()
            except Exception:
                return [0.0] * 6, False
            return [raw[i] - self._ur_offset[i] for i in range(6)], True
        if self._xjc_ok and self.xjc is not None:
            st = self.xjc.get()
            return st['values'], st['ok']
        return [0.0] * 6, False

    def tare_active(self):
        """Tare whichever source is selected (XJC sensor, or UR wrench offset)."""
        if self._force_source.get() == 'UR':
            if not self._robot_ok:
                self._set_status('Robot not connected — cannot tare UR')
                return
            try:
                self._ur_offset = [float(v) for v in self.rtde_r.getActualTCPForce()]
                self._set_status(f'UR wrench tared (Fz offset {self._ur_offset[2]:+.2f} N)')
            except Exception as e:
                self._set_status(f'UR tare failed: {e}')
            return
        if not self._xjc_ok:
            self._set_status('XJC not connected — cannot tare')
            return

        def _run():
            try:
                off = self.xjc.tare(n=30)
                self._set_status(f'XJC tared (Fz offset {off[2]:+.3f} N)')
            except Exception as e:
                self._set_status(f'Tare failed: {e}')
        threading.Thread(target=_run, daemon=True).start()

    def connect_gripper(self):
        if self.args.sim:
            self.gripper.connect()
            self.gripper.activate()
            self._gripper_connected = True
            self._grip_status.config(text='ACTIVE (sim)')
            return

        def _run():
            try:
                g = RobotiqGripper()
                g.connect(self.args.ip, GRIPPER_PORT)
                self._set_grip_status('Activating…')
                g.activate()
                self.gripper = g
                self._gripper_connected = True
                self._set_grip_status('ACTIVE')
                self._set_status('Gripper connected and active')
            except Exception as e:
                self._set_grip_status('Connect failed')
                self._set_status(f'Gripper connect failed: {e}')
        threading.Thread(target=_run, daemon=True).start()

    def gripper_move(self, position):
        if not self._gripper_connected:
            self._set_status('Gripper not connected')
            return
        spe = int(self._grip_speed.get())
        forc = int(self._grip_force.get())

        def _run():
            # Non-blocking lock: a held O/C key auto-repeats; don't flood the socket.
            if not self._grip_lock.acquire(blocking=False):
                return
            try:
                self.gripper.move(position, spe, forc)
            except Exception as e:
                self._set_status(f'Gripper move failed: {e}')
            finally:
                self._grip_lock.release()
        threading.Thread(target=_run, daemon=True).start()

    # ── live read-out loop ───────────────────────────────────────────────────────
    def _refresh(self):
        # Robot pose (receive interface works even if control is down)
        if self._robot_ok:
            try:
                p = self.rtde_r.getActualTCPPose()
                self._pose_lbl.config(
                    text=f'TCP  x={p[0]*1000:+8.2f}  y={p[1]*1000:+8.2f}  '
                         f'z={p[2]*1000:+8.2f} mm')
            except Exception:
                pass
            # control-script health indicator (also catches a stop while idle)
            self._ctrl_tick = getattr(self, '_ctrl_tick', 0) + 1
            if not self._control_lost and self._ctrl_tick % 12 == 0:
                try:
                    if not self.rtde_c.isProgramRunning():
                        self._on_control_lost()
                except Exception:
                    pass

        # Active force source (XJC or UR) → numeric block + 3D view
        v, ok = self._active_wrench()
        if ok:
            for i, name in enumerate(['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']):
                self._wrench_lbls[name].config(text=f'{v[i]:+8.3f}', foreground=self.C['text'])
            over = abs(v[2]) > XJC_FZ_WARN_N
            self._wrench_lbls['Fz'].config(foreground=self.C['warn'] if over else self.C['text'])
            self._view_tick += 1
            if self.view is not None and self._view_tick % 3 == 0:
                try:
                    self.view.update(v)
                except Exception:
                    pass
        else:
            for name in self._wrench_lbls:
                self._wrench_lbls[name].config(text='—', foreground=self.C['muted'])

        # Gripper position
        if self._gripper_connected and self.gripper is not None:
            try:
                self._grip_pos_lbl.config(text=f'pos  {self.gripper.get_current_position()}')
            except Exception:
                pass

        self._update_badges()
        self._refresh_after = self.root.after(80, self._refresh)   # ~12 Hz UI refresh

    def _update_badges(self):
        """Refresh the top-bar status pills (robot / control / XJC / gripper)."""
        C = self.C
        def set_badge(key, text, on):
            b = self._badges.get(key)
            if b is not None:
                b.config(text=text, fg=(C['success'] if on else C['muted']))
        set_badge('robot', '● ROBOT' if self._robot_ok else '○ ROBOT', self._robot_ok)
        if not self._robot_ok:
            self._badges['control'].config(text='○ CONTROL', fg=C['muted'])
        elif self._control_lost:
            self._badges['control'].config(text='▲ CONTROL LOST', fg=C['danger'])
        else:
            self._badges['control'].config(text='● CONTROL', fg=C['success'])
        set_badge('xjc', '● XJC' if self._xjc_ok else '○ XJC', self._xjc_ok)
        set_badge('gripper', '● GRIPPER' if self._gripper_connected else '○ GRIPPER',
                  self._gripper_connected)
        joy_on = self.joy is not None and self.joy.connected()
        set_badge('joystick', '● PAD' if joy_on else '○ PAD', joy_on)

    # ── helpers ──────────────────────────────────────────────────────────────────
    def _set_status(self, msg):
        # called from worker threads; never let a Tk hiccup (shutdown etc.) kill them
        try:
            self.root.after(0, lambda: self._status.config(text=f'  {msg}'))
        except Exception:
            print(f'[status] {msg}')

    def _set_grip_status(self, msg):
        try:
            self.root.after(0, lambda: self._grip_status.config(text=msg))
        except Exception:
            pass

    def on_close(self):
        # cancel the pending UI refresh so it can't fire after widgets are gone
        try:
            if getattr(self, '_refresh_after', None) is not None:
                self.root.after_cancel(self._refresh_after)
        except Exception:
            pass
        # stop the servo stream + any running routine first (no motion outlives us)
        self._routine_stop.set()
        self._jog_stop.set()
        self._jog_clear_all()
        try:
            if self.joy is not None:
                self.joy.stop()
        except Exception:
            pass
        if self._jog_thread is not None:
            self._jog_thread.join(0.5)
        try:
            if self._robot_ok:
                self.rtde_c.servoStop()
        except Exception:
            pass
        try:
            if self.xjc is not None:
                self.xjc.stop()
        except Exception:
            pass
        try:
            if self._gripper_connected and self.gripper is not None:
                self.gripper.disconnect()
        except Exception:
            pass
        try:
            if self._robot_ok and hasattr(self.rtde_c, 'stopScript'):
                self.rtde_c.stopScript()
        except Exception:
            pass
        self.root.destroy()


# ── CLI ─────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description='GUI: UR5e XYZ jog + XJC force read-out + Robotiq gripper',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('--ip', default=ROBOT_IP,
                   help=f'Robot IP (also the gripper URCap host; default {ROBOT_IP})')
    p.add_argument('--xjc-port', default=None,
                   help='XJC serial device, e.g. /dev/ttyUSB0 (default: auto-detect)')
    p.add_argument('--all-ports', action='store_true',
                   help='When auto-detecting the XJC, probe every serial port '
                        '(including legacy ttyS*), not just ttyUSB*/ttyACM*')
    p.add_argument('--xjc-baud', type=int, default=115200, help='XJC baud (default 115200)')
    p.add_argument('--xjc-station', type=int, default=1, help='XJC Modbus station (default 1)')
    p.add_argument('--no-xjc', action='store_true', help='Do not use the XJC sensor')
    p.add_argument('--no-view', action='store_true',
                   help='Do not show the 3D rigid-body force/torque view')
    p.add_argument('--theme', choices=['light', 'dark'], default='light',
                   help='UI theme (default: light / white)')
    p.add_argument('--no-joystick', action='store_true',
                   help='Do not read an Xbox/gamepad for analog jog')
    p.add_argument('--joy-deadzone', type=float, default=0.15,
                   help='Analog stick dead-zone fraction (default 0.15)')
    p.add_argument('--sim', action='store_true',
                   help='Hardware-free dry run: fake robot/XJC/gripper to try the GUI')
    return p.parse_args()


def main():
    args = parse_args()
    root = tk.Tk()
    ControlGUI(root, args)
    root.mainloop()


if __name__ == '__main__':
    main()
