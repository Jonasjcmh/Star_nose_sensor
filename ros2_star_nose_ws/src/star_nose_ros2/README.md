# star_nose_ros2

ROS2 front-end for the 19-point KYWO tactile sensor + UR5 rig
(`Star_nose_sensor/Integration_2/` has the original, non-ROS scripts this
is being ported from).

## Nodes
- `ur5_node` -- owns the RTDE connection, moves on `/star_nose/goto_point`,
  publishes `/ur5/tcp_pose` and `/star_nose/status`.
- `data_logger_node` -- reads MUCA (capacitive mat, ported from
  `sensor_bridge.py`), FUTEK, and LCR, and writes a CSV row per tick while
  `/star_nose/log_enable` is true. FUTEK/LCR live reads are still TODO stubs.
- `ui_node` -- terminal front-end (`goto <1-19>`, `log on|off`). Swap for a
  joystick or GUI node later without touching the other two.

## Friction mode (continuous trajectories)
Discrete `goto_point` is position-only; friction needs continuous streaming +
force control, so it has its own nodes (ported from `friction_mode/`):

- `ur5_friction_node` -- owns RTDE, runs the trajectory / force-control loop in
  its OWN thread (asynchronous, latest-value-wins). Subscribes
  `/friction/command` (`std_msgs/String`, JSON), publishes `/ur5/tcp_pose`,
  `/star_nose/status`, `/friction/waypoint`. A **deadman** lifts the arm if
  commands stop arriving (`command_timeout_s`, default 1 s).
- `friction_ui_node` -- terminal UI; holds the desired state and publishes it as
  JSON, **heartbeating** at `heartbeat_hz` (default 5 Hz) so the deadman stays
  satisfied. `run <pattern>`, `stop`, `speed/scale/depth/force`, `x+/y+` jog, etc.
- `friction_core` -- shared geometry (robot<->sensor frame) + trajectory shapes.
  Trajectories are authored in the SENSOR frame and converted to robot frame, and
  every waypoint is clamped to the sensor area, so patterns trace as drawn and
  never run off the sensor.

Run:
```
ros2 launch star_nose_ros2 friction.launch.py            # real robot
ros2 launch star_nose_ros2 friction.launch.py robot_ip:=127.0.0.1   # URSim
```
The pygame `friction_live.py` can later publish the same `/friction/command`
JSON via the same `state` dict — the UI node is deliberately a thin swap point.

Command JSON fields: `running, pattern, paused, contact(depth|force), depth,
force, speed, scale, off_x, off_y, direction`. Sync model: control loop is
async/decoupled; use `message_filters` in the logger if you need per-sample
pose+sensor time alignment.

## Dev machine vs. robot machine
Developed here on Jazzy (Ubuntu 24.04) -- rclpy's API is the same as
Humble for what these nodes use, so this builds and the wiring can be
tested without a robot attached. The UR5's own machine runs Humble and is
where this actually drives hardware: `git pull` there, then
`rosdep install` + `colcon build`.

## Build & run
```
cd ~/ros2_star_nose_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch star_nose_ros2 bringup.launch.py
```

## Known open issue
`config/points.yaml`'s `point_to_raw_cell` table is unverified -- see the
warning at the top of that file. Multiple different point->cell tables
exist across the old `Integration_2` scripts and disagree with each other;
confirm against hardware with `check_point_coordinates.py`'s live hover
tour before trusting calibration results built on this.
