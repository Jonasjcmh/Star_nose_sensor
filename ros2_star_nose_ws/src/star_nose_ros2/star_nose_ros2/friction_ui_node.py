"""
friction_ui_node.py
Terminal front-end for the friction rig. Holds the desired state, publishes it
as JSON on /friction/command, and republishes on a heartbeat so ur5_friction_node's
deadman stays satisfied while this UI is alive. Subscribes to status + waypoint
for display.

This is the thin, testable UI. The pygame friction_live.py view can publish the
SAME JSON via the same command dict — swap this node's input loop for the pygame
render loop later without touching ur5_friction_node or the logger.

Commands (type, then Enter):
  run <pattern>   start looping (line_h line_v diagonal_lr diagonal_rl cross
                  circle spiral raster)
  stop            stop + lift/home
  pause | resume
  speed <mm/s> | scale <x> | depth <mm> | force <N>
  contact depth|force
  x+ x- y+ y-     jog the pattern offset along the SENSOR axes by `step`
  step <mm>       set jog step (default 1.0)
  dir +|-         trajectory direction
  center          reset offset to 0,0
  status          print the current desired state
  quit
"""
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from star_nose_ros2 import friction_core as core

PATTERNS = set(core.PATTERNS.keys())


class FrictionUiNode(Node):
    def __init__(self):
        super().__init__('friction_ui_node')
        self.declare_parameter('heartbeat_hz', 5.0)
        self.cmd_pub = self.create_publisher(String, '/friction/command', 10)
        self.create_subscription(String, '/star_nose/status', self._on_status, 10)
        self.create_subscription(Int32, '/friction/waypoint', self._on_wp, 10)

        self.step = 1.0
        self.state = dict(
            running=False, pattern='line_h', paused=False, contact='depth',
            depth=4.0, force=5.0, speed=8.0, scale=1.0,
            off_x=0.0, off_y=0.0, direction=1)

        hz = max(0.5, float(self.get_parameter('heartbeat_hz').value))
        self.create_timer(1.0 / hz, self._heartbeat)

    # ── ROS callbacks ──────────────────────────────────────────────
    def _on_status(self, msg):
        self.get_logger().info(f'status: {msg.data}')

    def _on_wp(self, msg):
        pass  # available for a live counter; quiet by default

    def _heartbeat(self):
        """Republish the full desired state so the robot node's deadman never
        trips while this UI is running."""
        self.cmd_pub.publish(String(data=json.dumps(self.state)))

    def publish(self):
        self.cmd_pub.publish(String(data=json.dumps(self.state)))

    # ── Command handling ───────────────────────────────────────────
    def handle(self, cmd):
        parts = cmd.split()
        if not parts:
            return True
        c = parts[0]
        if c == 'run':
            if len(parts) < 2 or parts[1] not in PATTERNS:
                print(f'  patterns: {sorted(PATTERNS)}')
                return True
            self.state.update(pattern=parts[1], running=True, paused=False)
        elif c == 'stop':
            self.state.update(running=False, paused=False)
        elif c == 'pause':
            self.state['paused'] = True
        elif c == 'resume':
            self.state['paused'] = False
        elif c in ('speed', 'scale', 'depth', 'force', 'step') and len(parts) >= 2:
            try:
                v = float(parts[1])
            except ValueError:
                print('  need a number'); return True
            if c == 'step':
                self.step = v
            else:
                self.state[c] = v
        elif c == 'contact' and len(parts) >= 2 and parts[1] in ('depth', 'force'):
            self.state['contact'] = parts[1]
        elif c in ('x+', 'x-', 'y+', 'y-'):
            sdx = self.step if c == 'x+' else -self.step if c == 'x-' else 0.0
            sdy = self.step if c == 'y+' else -self.step if c == 'y-' else 0.0
            ndx, ndy = core.sensor_to_robot(sdx, sdy)
            ax, ay = core.clamp_to_sensor(self.state['off_x'] + ndx,
                                          self.state['off_y'] + ndy)
            self.state.update(off_x=round(ax, 3), off_y=round(ay, 3))
        elif c == 'dir' and len(parts) >= 2:
            self.state['direction'] = 1 if parts[1].startswith('+') else -1
        elif c == 'center':
            self.state.update(off_x=0.0, off_y=0.0)
        elif c == 'status':
            print('  ' + json.dumps(self.state))
            return True
        elif c == 'quit':
            return False
        else:
            print('  ?  (run/stop/pause/resume/speed/scale/depth/force/'
                  'contact/x+/x-/y+/y-/step/dir/center/status/quit)')
            return True
        self.publish()
        return True


def _input_loop(node):
    print(__doc__)
    while rclpy.ok():
        try:
            cmd = input('friction > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if not node.handle(cmd):
            break
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FrictionUiNode()
    threading.Thread(target=_input_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
