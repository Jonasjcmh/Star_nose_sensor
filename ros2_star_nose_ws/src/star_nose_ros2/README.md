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
