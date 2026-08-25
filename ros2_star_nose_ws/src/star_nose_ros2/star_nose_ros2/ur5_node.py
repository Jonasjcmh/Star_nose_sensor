"""
ur5_node.py
Owns the RTDE connection to the UR5. Ports the Cartesian move logic from
Star_nose_sensor/Integration_2/ur5_control.py + calibrate_points.py
(REFERENCE_POSE + per-point mm offset -> absolute TCP pose), but loads the
19-point table from config/points.yaml instead of a hardcoded dict.

Topics:
  sub  /star_nose/goto_point   (std_msgs/Int32)   point id 1-19 -> move there
  pub  /ur5/tcp_pose           (geometry_msgs/PoseStamped)  live TCP pose
  pub  /star_nose/status       (std_msgs/String)  "idle" | "moving" | "at_point:<id>" | "error:<msg>"

Runs without a real robot (or without rtde installed) so you can test the
node wiring on this machine before deploying to the UR5's Humble box --
in that case it just logs and publishes status without a live pose.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped

try:
    import rtde_control
    import rtde_receive
    _HAS_RTDE = True
except ImportError:
    _HAS_RTDE = False

VELOCITY_TRAVEL = 0.05
ACCELERATION = 0.3


def rotvec_to_quaternion(rx, ry, rz):
    """UR TCP orientation is an axis-angle rotation vector; convert to a quaternion."""
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = rx / angle, ry / angle, rz / angle
    s = math.sin(angle / 2.0)
    return (ax * s, ay * s, az * s, math.cos(angle / 2.0))


class Ur5Node(Node):
    def __init__(self):
        super().__init__('ur5_node')

        self.declare_parameter('robot_ip', '177.22.22.2')
        self.declare_parameter('points_file', '')
        self.declare_parameter('pose_publish_hz', 20.0)

        self.points_mm, self.reference_pose = self._load_points(
            self.get_parameter('points_file').value)

        self.status_pub = self.create_publisher(String, '/star_nose/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/ur5/tcp_pose', 10)
        self.create_subscription(Int32, '/star_nose/goto_point', self._on_goto_point, 10)

        self.rtde_c = None
        self.rtde_r = None
        hz = max(1.0, float(self.get_parameter('pose_publish_hz').value))
        self.create_timer(1.0 / hz, self._publish_pose)

        if _HAS_RTDE:
            # Connecting can block for a long time (default TCP connect
            # timeout) if the robot is off/unreachable -- do it off the
            # executor thread so the node still comes up, spins, and
            # responds to shutdown signals immediately.
            self._publish_status('connecting')
            threading.Thread(
                target=self._connect,
                args=(self.get_parameter('robot_ip').value,),
                daemon=True,
            ).start()
        else:
            self.get_logger().warn(
                'rtde_control/rtde_receive not installed -- running without a robot '
                '(node wiring only, no live pose).')
            self._publish_status('idle')

    def _load_points(self, points_file):
        import yaml
        import os
        if not points_file:
            from ament_index_python.packages import get_package_share_directory
            points_file = os.path.join(
                get_package_share_directory('star_nose_ros2'), 'config', 'points.yaml')
            if not os.path.exists(points_file):
                # not installed yet -- fall back to the source tree copy
                points_file = os.path.join(
                    os.path.dirname(__file__), '..', 'config', 'points.yaml')
        with open(points_file) as f:
            data = yaml.safe_load(f)
        points_mm = {int(k): v for k, v in data['points_mm'].items()}
        rp = data['reference_pose']
        reference_pose = [rp['x'], rp['y'], rp['z'], rp['rx'], rp['ry'], rp['rz']]
        self.get_logger().info(f'Loaded {len(points_mm)} points from {points_file}')
        return points_mm, reference_pose

    def _connect(self, robot_ip):
        """Runs on a background thread -- may block for a while if robot_ip
        is unreachable, so must never run on the executor thread."""
        try:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
            self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
            self.get_logger().info(f'Connected to UR5 at {robot_ip}')
            self._publish_status('idle')
        except Exception as e:
            self.get_logger().error(f'UR5 connection failed: {e}')
            self.rtde_c = None
            self.rtde_r = None
            self._publish_status('error:no_robot')

    def _build_pose(self, pt):
        dx, dy = self.points_mm[pt]
        pose = list(self.reference_pose)
        pose[0] += dx / 1000.0
        pose[1] += dy / 1000.0
        return pose

    def _on_goto_point(self, msg):
        pt = msg.data
        if pt not in self.points_mm:
            self._publish_status(f'error:invalid_point_{pt}')
            return
        if self.rtde_c is None:
            self.get_logger().warn(f'goto_point {pt} received but no robot connection')
            self._publish_status('error:no_robot')
            return
        self._publish_status('moving')
        target = self._build_pose(pt)
        self.rtde_c.moveL(target, VELOCITY_TRAVEL, ACCELERATION)
        self._publish_status(f'at_point:{pt}')

    def _publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def _publish_pose(self):
        if self.rtde_r is None:
            return
        try:
            tcp = self.rtde_r.getActualTCPPose()
        except Exception as e:
            self.get_logger().error(f'getActualTCPPose failed: {e}')
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = tcp[0], tcp[1], tcp[2]
        qx, qy, qz, qw = rotvec_to_quaternion(tcp[3], tcp[4], tcp[5])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Ur5Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.rtde_c is not None:
            try:
                node.rtde_c.stopScript()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
