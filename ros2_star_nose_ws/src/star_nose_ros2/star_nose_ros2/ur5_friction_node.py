"""
ur5_friction_node.py
Owns the RTDE connection and runs the friction trajectory / force-control loop.
Ports the control logic from Star_nose_sensor/friction_mode/ur5_friction.py +
friction_live.py's robot worker, driven by ROS topics instead of a pygame queue.

Design: ASYNCHRONOUS, latest-value-wins. The friction control loop runs in its
own thread and reads the most recent desired state each cycle; command messages
never block it and never run the move themselves. A deadman timeout pauses/lifts
the arm if commands stop arriving (e.g. the UI died).

Topics
  sub  /friction/command   (std_msgs/String, JSON)  partial desired-state update
  pub  /ur5/tcp_pose       (geometry_msgs/PoseStamped)   live TCP pose
  pub  /star_nose/status   (std_msgs/String)             human-readable state
  pub  /friction/waypoint  (std_msgs/Int32)              current waypoint index

Desired-state fields (all optional in each message; merged into the held state):
  running   bool     start/keep looping the selected pattern
  pattern   str      line_h | line_v | diagonal_lr | diagonal_rl | cross |
                     circle | spiral | raster
  paused    bool
  contact   str      "depth" | "force"
  depth     float    indentation depth (mm)      -- depth mode
  force     float    target contact force (N)     -- force mode
  speed     float    lateral slide speed (mm/s)
  scale     float    pattern size multiplier
  off_x     float    robot-frame X offset (mm)
  off_y     float    robot-frame Y offset (mm)
  direction int      +1 / -1

Runs without rtde installed or without a robot: it publishes status and accepts
commands so the whole pipeline (UI <-> this node <-> logger) can be tested dry.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped

from star_nose_ros2 import friction_core as core

try:
    import rtde_control
    import rtde_receive
    _HAS_RTDE = True
except ImportError:
    _HAS_RTDE = False

# ── Motion / force constants (from ur5_friction.py) ───────────────────────────
REFERENCE_POSE = [-0.03664, -0.49831, 0.06071, 2.346, -2.094, -0.00009]
VELOCITY_HOME = 0.08
VELOCITY_ENGAGE = 0.004
ACCELERATION = 0.3
SAFE_HOME_Z_MM = 30.0
HOVER_MM = 8.0

FUTEK_ZERO_V = 5.0
FUTEK_N_PER_V = 44.482 / 5.0
FORCE_KP_MM_PER_N = 0.4
FORCE_MAX_DZ_MM = 1.5
MAX_ENGAGE_MM = 15.0            # hard safety: never indent deeper than this


def _ai0_to_N(v):
    return -(v - FUTEK_ZERO_V) * FUTEK_N_PER_V


def _rotvec_to_quat(rx, ry, rz):
    ang = math.sqrt(rx * rx + ry * ry + rz * rz)
    if ang < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(ang / 2.0)
    return (rx / ang * s, ry / ang * s, rz / ang * s, math.cos(ang / 2.0))


class Ur5FrictionNode(Node):
    def __init__(self):
        super().__init__('ur5_friction_node')

        self.declare_parameter('robot_ip', '177.22.22.2')
        self.declare_parameter('pose_publish_hz', 20.0)
        self.declare_parameter('command_timeout_s', 1.0)   # deadman
        self.declare_parameter('calib_x_mm', 0.0)
        self.declare_parameter('calib_y_mm', 0.0)
        self.declare_parameter('calib_z_mm', 0.0)

        self.calib = (
            float(self.get_parameter('calib_x_mm').value),
            float(self.get_parameter('calib_y_mm').value),
            float(self.get_parameter('calib_z_mm').value),
        )
        self.cmd_timeout = float(self.get_parameter('command_timeout_s').value)

        # Held desired state (latest-value-wins), guarded by _lock.
        self._lock = threading.Lock()
        self._state = dict(
            running=False, pattern='line_h', paused=False, contact='depth',
            depth=4.0, force=5.0, speed=8.0, scale=1.0,
            off_x=0.0, off_y=0.0, direction=1)
        self._last_cmd_t = 0.0        # 0 => no command yet (deadman inactive
        #                               until the first command arrives)

        # Live robot cache (updated by the reader thread).
        self._cache_lock = threading.Lock()
        self._tcp = None
        self._ai0 = FUTEK_ZERO_V

        self.status_pub = self.create_publisher(String, '/star_nose/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ur5/tcp_pose', 10)
        self.wp_pub = self.create_publisher(Int32, '/friction/waypoint', 10)
        self.create_subscription(String, '/friction/command', self._on_command, 10)

        hz = max(1.0, float(self.get_parameter('pose_publish_hz').value))
        self.create_timer(1.0 / hz, self._publish_pose)

        self.rtde_c = None
        self.rtde_r = None
        self._stop = threading.Event()

        if _HAS_RTDE:
            self._set_status('connecting')
            threading.Thread(target=self._connect,
                             args=(self.get_parameter('robot_ip').value,),
                             daemon=True).start()
        else:
            self.get_logger().warn('rtde not installed -- running dry (no robot).')
            self._set_status('idle:no_robot')

        threading.Thread(target=self._control_loop, daemon=True).start()

    # ── Command intake (async, merge into held state) ──────────────
    def _on_command(self, msg):
        try:
            upd = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn(f'bad command JSON: {msg.data[:80]!r}')
            return
        with self._lock:
            for k, v in upd.items():
                if k in self._state:
                    self._state[k] = v
            self._last_cmd_t = time.time()

    def _snap(self):
        with self._lock:
            return dict(self._state), self._last_cmd_t

    def _set_status(self, text):
        self.status_pub.publish(String(data=text))

    # ── RTDE lifecycle ─────────────────────────────────────────────
    def _connect(self, robot_ip):
        try:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
            self.rtde_c = rtde_control.RTDEControlInterface(
                robot_ip, frequency=500.0,
                flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
            self.get_logger().info(f'Connected to UR5 at {robot_ip}')
            threading.Thread(target=self._reader_loop, daemon=True).start()
            self.rtde_c.moveL(self._pose(0.0, 0.0, SAFE_HOME_Z_MM), VELOCITY_HOME, ACCELERATION)
            self._set_status('idle')
        except Exception as e:
            self.get_logger().error(f'UR5 connection failed: {e}')
            self.rtde_c = self.rtde_r = None
            self._set_status('error:no_robot')

    def _reader_loop(self):
        while rclpy.ok() and not self._stop.is_set():
            r = self.rtde_r
            if r is not None:
                try:
                    tcp = r.getActualTCPPose()
                    ai0 = r.getStandardAnalogInput0()
                    with self._cache_lock:
                        self._tcp = list(tcp)
                        self._ai0 = float(ai0)
                except Exception:
                    pass
            time.sleep(0.008)

    def _pose(self, x_mm, y_mm, z_mm):
        cx, cy, cz = self.calib
        p = list(REFERENCE_POSE)
        p[0] += (x_mm + cx) / 1000.0
        p[1] += (y_mm + cy) / 1000.0
        p[2] += (z_mm + cz) / 1000.0
        return p

    def _ai0_now(self):
        with self._cache_lock:
            return self._ai0

    # ── Control loop (own thread) ──────────────────────────────────
    def _control_loop(self):
        """Outer loop: when the desired state says running (and not paused,
        deadman ok, robot present) start a pattern run; otherwise idle/hover."""
        while rclpy.ok() and not self._stop.is_set():
            st, last_t = self._snap()
            if self.rtde_c is None:
                time.sleep(0.1)
                continue
            if self._deadman(last_t):
                self._set_status('deadman:lift')
                self._lift()
                time.sleep(0.1)
                continue
            if st['running'] and not st['paused']:
                self._run_pattern()          # blocks until it should stop
            else:
                time.sleep(0.05)

    def _deadman(self, last_t):
        return last_t > 0.0 and (time.time() - last_t) > self.cmd_timeout

    def _lift(self):
        """Raise to hover above the surface at the current XY offset."""
        st, _ = self._snap()
        try:
            ax, ay = core.clamp_to_sensor(st['off_x'], st['off_y'])
            self.rtde_c.moveL(self._pose(ax, ay, HOVER_MM), VELOCITY_HOME, ACCELERATION)
        except Exception:
            pass

    def _run_pattern(self):
        st, _ = self._snap()
        pattern = st['pattern']
        contact = st['contact']
        try:
            base = core.traj_points(pattern, st['scale'])
        except KeyError:
            self._set_status(f'error:unknown_pattern:{pattern}')
            time.sleep(0.2)
            return
        n = len(base)
        if n < 2:
            time.sleep(0.1)
            return

        cur_scale = st['scale']
        # Engage at the first waypoint (+offset), clamped to the sensor.
        i = 0 if st['direction'] >= 0 else n - 1
        x0, y0 = base[i]
        ax, ay = core.clamp_to_sensor(x0 + st['off_x'], y0 + st['off_y'])
        z, baseline = self._engage(ax, ay, contact, st)
        if z is None:
            return
        self._set_status(f'running:{pattern}:{contact}')

        while rclpy.ok() and not self._stop.is_set():
            st, last_t = self._snap()
            if not st['running'] or st['paused'] or self._deadman(last_t):
                break
            # Live pattern / scale change -> regenerate geometry, keep phase.
            if st['pattern'] != pattern or st['scale'] != cur_scale:
                pattern = st['pattern']
                cur_scale = st['scale']
                try:
                    new = core.traj_points(pattern, cur_scale)
                except KeyError:
                    break
                frac = i / n if n else 0.0
                base, n = new, len(new)
                i = min(int(round(frac * n)), n - 1)
                self._set_status(f'running:{pattern}:{contact}')
                continue

            x, y = base[i]
            ax, ay = core.clamp_to_sensor(x + st['off_x'], y + st['off_y'])
            z = self._hold_z(z, baseline, contact, st)
            try:
                self.rtde_c.moveL(self._pose(ax, ay, -z), st['speed'] / 1000.0, ACCELERATION)
            except Exception as e:
                self.get_logger().error(f'moveL failed: {e}')
                break
            self.wp_pub.publish(Int32(data=i + 1))

            i += 1 if st['direction'] >= 0 else -1
            if i >= n:
                i = 0
            elif i < 0:
                i = n - 1

        # Leaving the pattern (stop / pause / deadman): lift off the surface.
        self._lift()
        self._set_status('idle' if not self._stop.is_set() else 'stopping')

    def _engage(self, x_mm, y_mm, contact, st):
        """Descend onto the surface. Returns (z_mm, baseline_N|None) or
        (None, None) if aborted/failed."""
        try:
            self.rtde_c.moveL(self._pose(x_mm, y_mm, HOVER_MM), VELOCITY_HOME, ACCELERATION)
            if contact == 'force':
                self._set_status('engaging:force')
                buf = []
                for _ in range(30):
                    buf.append(self._ai0_now())
                    time.sleep(0.01)
                baseline = _ai0_to_N(sum(buf) / len(buf)) if buf else _ai0_to_N(FUTEK_ZERO_V)
                target = float(st['force'])
                self.rtde_c.moveL(self._pose(x_mm, y_mm, 0.0), VELOCITY_ENGAGE, ACCELERATION)
                z = 0.0
                while z < MAX_ENGAGE_MM:
                    s2, last_t = self._snap()
                    if not s2['running'] or self._stop.is_set() or self._deadman(last_t):
                        return None, None
                    z += 0.15
                    self.rtde_c.moveL(self._pose(x_mm, y_mm, -z), VELOCITY_ENGAGE, ACCELERATION)
                    if (_ai0_to_N(self._ai0_now()) - baseline) >= target:
                        break
                return z, baseline
            # depth mode
            z = min(float(st['depth']), MAX_ENGAGE_MM)
            self.rtde_c.moveL(self._pose(x_mm, y_mm, 0.0), VELOCITY_ENGAGE, ACCELERATION)
            self.rtde_c.moveL(self._pose(x_mm, y_mm, -z), VELOCITY_ENGAGE, ACCELERATION)
            return z, None
        except Exception as e:
            self.get_logger().error(f'engage failed: {e}')
            return None, None

    def _hold_z(self, z, baseline, contact, st):
        if contact != 'force':
            return min(float(st['depth']), MAX_ENGAGE_MM)
        f = _ai0_to_N(self._ai0_now()) - baseline
        err = float(st['force']) - f
        dz = max(-FORCE_MAX_DZ_MM, min(FORCE_MAX_DZ_MM, FORCE_KP_MM_PER_N * err))
        return max(0.0, min(MAX_ENGAGE_MM, z + dz))

    # ── Pose publishing ────────────────────────────────────────────
    def _publish_pose(self):
        with self._cache_lock:
            tcp = list(self._tcp) if self._tcp else None
        if tcp is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = tcp[0], tcp[1], tcp[2]
        qx, qy, qz, qw = _rotvec_to_quat(tcp[3], tcp[4], tcp[5])
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pose_pub.publish(msg)

    def shutdown(self):
        self._stop.set()
        if self.rtde_c is not None:
            try:
                self.rtde_c.moveL(self._pose(0.0, 0.0, SAFE_HOME_Z_MM), VELOCITY_HOME, ACCELERATION)
                self.rtde_c.stopScript()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = Ur5FrictionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
