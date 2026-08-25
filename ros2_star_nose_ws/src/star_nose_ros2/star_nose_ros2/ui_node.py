"""
ui_node.py
Minimal terminal front-end: send the UR5 to a point, start/stop logging,
watch live status. Swap this for a GUI/joystick node later without
touching ur5_node or data_logger_node -- they only know about the topics
below, not about what's driving them.

Commands:
  goto <1-19>   move to that point
  log on|off    start/stop CSV logging
  quit
"""
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool


class UiNode(Node):
    def __init__(self):
        super().__init__('ui_node')
        self.goto_pub = self.create_publisher(Int32, '/star_nose/goto_point', 10)
        self.log_pub = self.create_publisher(Bool, '/star_nose/log_enable', 10)
        self.create_subscription(String, '/star_nose/status', self._on_status, 10)

    def _on_status(self, msg):
        self.get_logger().info(f'status: {msg.data}')

    def goto(self, pt):
        self.goto_pub.publish(Int32(data=pt))

    def set_logging(self, enabled):
        self.log_pub.publish(Bool(data=enabled))


def _input_loop(node):
    print("Commands: goto <1-19> | log on|off | quit")
    while rclpy.ok():
        try:
            cmd = input('ui > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd.startswith('goto'):
            try:
                pt = int(cmd.split()[1])
                node.goto(pt)
            except (IndexError, ValueError):
                print('Usage: goto <1-19>')
        elif cmd == 'log on':
            node.set_logging(True)
        elif cmd == 'log off':
            node.set_logging(False)
        elif cmd == 'quit':
            break
        else:
            print("Commands: goto <1-19> | log on|off | quit")
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UiNode()
    threading.Thread(target=_input_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
