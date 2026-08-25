"""
data_logger_node.py
Combines the three sensor sources + live UR5 state into one CSV row per tick.
Ports Star_nose_sensor/Integration_2/sensor_bridge.py (MUCA capacitive mat,
serial) directly -- it's a known-good read loop. FUTEK load cell and LCR
meter reads are left as TODO stubs (Integration_2/analyze_session.py only
shows offline voltage->N conversion, not the live acquisition call, and the
LCR driver isn't in this repo yet -- port from whatever reads them today
rather than guessing the API here).

Topics:
  sub  /ur5/tcp_pose      (geometry_msgs/PoseStamped)
  sub  /star_nose/status  (std_msgs/String)
  sub  /star_nose/log_enable (std_msgs/Bool)  start/stop writing rows

Output: CSV in ~/ros2_star_nose_ws/logs/<prefix>_<timestamp>.csv
"""
import csv
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

try:
    import serial
    _HAS_SERIAL = True
except ImportError:
    _HAS_SERIAL = False

LOG_DIR = os.path.expanduser('~/ros2_star_nose_ws/logs')

# MUCA raw-cell order for UR5 points 1-19 -- ported from sensor_bridge.py.
# NOTE: see config/points.yaml's warning -- this table has disagreed with
# the one in ur5_control.py/calibrate_points.py historically. Verify before
# trusting it.
MUCA_USED_CELLS = [
    2, 15, 28, 1, 14, 27, 40, 0, 13, 26, 39, 52, 12, 25, 38, 51, 24, 37, 50,
]
MUCA_SKIN_CELLS = 12 * 21


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        self.declare_parameter('log_prefix', 'session')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('muca_serial_port', '/dev/ttyACM0')
        self.declare_parameter('muca_baud', 115200)

        self._latest_pose = None
        self._latest_status = ''
        self._muca_raw = [0] * 19
        self._muca_lock = threading.Lock()
        self._logging_enabled = False
        self._csv_writer = None
        self._csv_file = None

        self.create_subscription(PoseStamped, '/ur5/tcp_pose', self._on_pose, 10)
        self.create_subscription(String, '/star_nose/status', self._on_status, 10)
        self.create_subscription(Bool, '/star_nose/log_enable', self._on_log_enable, 10)

        if _HAS_SERIAL:
            port = self.get_parameter('muca_serial_port').value
            baud = self.get_parameter('muca_baud').value
            threading.Thread(target=self._muca_read_loop, args=(port, baud), daemon=True).start()
        else:
            self.get_logger().warn('pyserial not installed -- MUCA column will stay zero.')

        hz = max(0.1, float(self.get_parameter('rate_hz').value))
        self.create_timer(1.0 / hz, self._tick)

    # ── Subscriptions ──────────────────────────────────────────────
    def _on_pose(self, msg):
        self._latest_pose = msg

    def _on_status(self, msg):
        self._latest_status = msg.data

    def _on_log_enable(self, msg):
        if msg.data and not self._logging_enabled:
            self._start_csv()
        elif not msg.data and self._logging_enabled:
            self._stop_csv()

    # ── MUCA (capacitive mat) read loop -- ported from sensor_bridge.py ──
    def _muca_read_loop(self, port, baud):
        try:
            ser = serial.Serial(port, baud, timeout=5)
        except Exception as e:
            self.get_logger().error(f'MUCA serial open failed on {port}: {e}')
            return
        self.get_logger().info(f'MUCA connected on {port}')
        while rclpy.ok():
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line or not line[0].isdigit():
                    continue
                vals = list(map(int, line.split(',')))
                if len(vals) != MUCA_SKIN_CELLS:
                    continue
                raw = [vals[c] for c in MUCA_USED_CELLS]
                with self._muca_lock:
                    self._muca_raw = raw
            except Exception as e:
                self.get_logger().error(f'MUCA read error: {e}')
                time.sleep(0.1)

    # ── FUTEK load cell -- TODO: port the live acquisition call ──────
    def _read_futek_force_n(self):
        """Return latest FUTEK force reading in Newtons. TODO: port the live
        DAQ read (AI0 voltage -> N conversion already exists in
        Integration_2/analyze_session.py:ai0_to_newtons, but that file only
        processes logged CSVs -- find/port whatever does the live read)."""
        return None

    # ── LCR meter -- TODO: port the live acquisition call ────────────
    def _read_lcr(self):
        """Return latest LCR reading. TODO: port from whatever currently
        talks to the LCR meter (see Capacitance_measurement/Emstat_connection.py
        for the connection pattern used for the related Emstat instrument)."""
        return None

    # ── CSV lifecycle ─────────────────────────────────────────────
    def _start_csv(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        prefix = self.get_parameter('log_prefix').value
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(LOG_DIR, f'{prefix}_{ts}.csv')
        self._csv_file = open(path, 'w', newline='')
        header = (
            ['t', 'status', 'tcp_x', 'tcp_y', 'tcp_z', 'futek_n', 'lcr']
            + [f'muca_{i+1}' for i in range(19)]
        )
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(header)
        self._logging_enabled = True
        self.get_logger().info(f'Logging -> {path}')

    def _stop_csv(self):
        self._logging_enabled = False
        if self._csv_file:
            self._csv_file.close()
        self._csv_file = None
        self._csv_writer = None
        self.get_logger().info('Logging stopped')

    def _tick(self):
        if not self._logging_enabled:
            return
        pose = self._latest_pose
        tcp_x = pose.pose.position.x if pose else ''
        tcp_y = pose.pose.position.y if pose else ''
        tcp_z = pose.pose.position.z if pose else ''
        with self._muca_lock:
            muca = list(self._muca_raw)
        row = (
            [time.time(), self._latest_status, tcp_x, tcp_y, tcp_z,
             self._read_futek_force_n(), self._read_lcr()]
            + muca
        )
        self._csv_writer.writerow(row)
        self._csv_file.flush()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node._stop_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
