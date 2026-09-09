# Inertial Test (IMU)

Robot-motion sandbox for inertial / IMU measurements, inspired by
`friction_mode/` but stripped down to **position control only**:

- **No** capacitive sensor
- **No** FUTEK load cell / force feedback
- **No** calibration profile

The UR5 just carries the end-effector (IMU) through XY trajectories in a
horizontal plane in free air. The working area is a square of **at most
23 cm × 23 cm** (`WORKAREA_MAX_MM = 230` mm); everything is clamped to fit.

## Files

| File | Purpose |
|------|---------|
| `imu_trajectories.py` | Parameterised shapes (circle, spiral, raster, square, cross, lines, figure-8) sized by a single **diameter / covered-span** knob. Also the geometry helpers used to scale imported paths. |
| `ur5_imu.py` | UR5 position control (moveL per waypoint). No sensors, no calibration. |
| `run_imu.py` | Simple headless runner — pick a trajectory + size and go. |
| `imu_live.py` | Live matplotlib dashboard showing the planned path and the end-effector position/trail as it moves (the "demo-like" view). |
| `import_trajectory.py` | Import an XY(+yaw) trajectory from a JSON file and rescale XY to the working area. |
| `convert_txt_trajectories.py` | Convert the MATLAB-style `Trajectories.txt` (x/y/theta arrays) into one importable JSON per trajectory. |
| `imported_trajectories/*.json` | The 15 converted paths (circles, lines, X, star) with per-waypoint yaw, in the native 0..230 drawing frame. |
| `ur_calibration.py` | Similarity transform: trajectory drawing frame (0..230 units) → UR robot base frame (mm), from two measured corner correspondences. |
| `transform_trajectories.py` | Apply that calibration to every trajectory, writing `trajectories_ur/*.json` in absolute UR coordinates. |
| `trajectories_ur/*.json` | The 15 trajectories in the **UR base frame** (`"frame": "ur_base_mm"`), run at their true calibrated table position. |
| `sample_trajectories/heart.json` | Example JSON path in arbitrary units. |

## Quick start

```bash
cd inertial_test_IMU

# List available shapes
python run_imu.py --list

# Preview a shape without moving the robot
python run_imu.py --traj spiral --size 120 --dry-run

# Run a 12 cm spiral on the robot (30 mm/s)
python run_imu.py --traj spiral --size 120 --speed 30

# Live viewer: watch the end-effector follow a 10 cm circle
python imu_live.py --traj circle --size 100

# Live viewer with NO robot (simulated motion — safe to try anywhere)
python imu_live.py --traj raster --size 140 --lines 10 --no-robot
```

## Live controller (keyboard, in the viewer)

`imu_live.py` is a friction-demo-style live controller. It loads a **library**
of trajectories — the calibrated JSONs in `trajectories_ur/` (override with
`--lib-dir`) **plus** the generated shapes — and you drive everything from the
keyboard with the window focused:

| Input | Action |
|-------|--------|
| `,` / `.` | previous / next trajectory in the library |
| `[` / `]` | manual **yaw offset** − / + (`{` / `}` = big steps) |
| `y` | reset the yaw offset to 0 |
| `← → ↑ ↓` | move the **whole path** (central point), live |
| `shift`+arrow | move the central point in big steps (×5) |
| `-` / `=` | smaller / larger central-point step |
| `n` | reset central point to the reference origin (0,0) |
| `space` | **run** the selected trajectory / **stop** if running |
| `backspace` | stop and return home |
| `r` | re-scan the JSON library folder (pick up new/edited files) |
| `enter` | open the **console** (pseudo-terminal); `esc` closes |
| mouse click / drag | place the central point on the XY panel |

**Console (speed / height and more).** Press `enter` to open a typed command
line at the bottom of the window, then:

| Command | Effect |
|---------|--------|
| `speed 40` (`s 40`) | set trajectory speed to 40 mm/s |
| `height 25` (`h 25`) | set work-plane height to 25 mm |
| `step 2` | central-point key step (mm) |
| `yaw 90` | set the manual yaw offset (deg) |
| `traj star` / `sel 14` | select a trajectory by name or number |
| `run` / `stop` | start / stop |

Speed and height apply **live** — on the robot they take effect mid-run
(speed and plane height are re-read every waypoint); in `--no-robot` the
simulation follows them too. Both are shown in the readout panel.
Height ranges **−50 … +50 mm** about the reference pose (negative = below it);
speed is bounded 1 … 200 mm/s.

**Fixed start point.** The robot always begins at the trajectory's own first
waypoint (the yellow square) — the start is not adjustable.

**Manual initial yaw.** Set the initial orientation by hand with `[` / `]`
before running; it's previewed as the red heading arrow at the start point
while idle. During motion each waypoint's yaw is applied as
`waypoint_yaw + offset`, so the tool follows the trajectory's yaw relative to
the orientation you fixed. For shapes with no per-waypoint yaw, the offset acts
as a constant tool heading.

The current heading is shown two ways: a **degree label** riding the arrow tip,
and a **compass rose** (lower-left of the XY panel) with a live needle and
0/90/180/270° ticks. 0° points along +X, 90° along +Y (base frame).

Switching trajectory **while running** restarts the motion (robot returns home,
then runs it). Central-point and yaw changes apply live without restarting; the
yellow `+` is the central point. `--no-robot` simulates the same behaviour.

```bash
python imu_live.py --list                                   # print the library
python imu_live.py --no-robot                               # simulate, pick with , .
python imu_live.py --json trajectories_ur/star_5_arm_hand_drawn.json
python imu_live.py --lib-dir imported_trajectories --traj spiral
```

## Sizing

One knob, `--size`, sets the **diameter** for round shapes (circle, spiral)
and the **covered span** for the rest (raster, square, cross, lines, figure-8).
Anything above 230 mm is clamped to the working-area limit. Shape-specific
extras: `--turns` (spiral), `--lines` (raster), `--steps` (waypoint density).

## Importing JSON trajectories

Accepted JSON forms (units are arbitrary and get rescaled). A third value per
waypoint (or a `theta`/`yaw` array) is treated as a **yaw angle in degrees**
and carried through unchanged — only X/Y are rescaled, never the angle:

```json
[[x, y], [x, y], ...]
[[x, y, yaw], ...]
{"waypoints": [[x, y, yaw], ...], "yaw_units": "deg"}
{"points": [{"x": .., "y": .., "yaw": ..}, ...]}
{"x": [..], "y": [..], "theta": [..]}
```

### Converting `Trajectories.txt`

The MATLAB-style `Trajectories.txt` (blocks of `x = [...]`, `y = [...]`,
`theta = [...]` in `[x(mm), y(mm), theta(deg)]`) is converted to one JSON per
trajectory with `convert_txt_trajectories.py`:

```bash
python convert_txt_trajectories.py            # → imported_trajectories/*.json
python convert_txt_trajectories.py --list     # just list the blocks
```

This produces the 15 named paths (4 circles, 9 lines, an X, a star) in
`imported_trajectories/`, each carrying its yaw column. View or run any of
them like a normal JSON import:

```bash
python imu_live.py --json imported_trajectories/star_5_arm_hand_drawn.json --no-robot
python import_trajectory.py imported_trajectories/line_45deg.json --preview
```

### Aligning to the robot (UR base frame)

The drawing-frame coordinates (0..230 units) are mapped to real UR base-frame
positions by `ur_calibration.py`, from two measured corner correspondences:

```
trajectory (0,   0)   → UR base (26.71,  -394.66) mm      # origin corner
trajectory (230, 230) → UR base (138.42, -689.91) mm      # opposite corner
```

Solved as an orientation-preserving similarity transform: scale ≈ 0.9705
(230 u → 223.2 mm square), rotation ≈ −114.28°. Yaw rotates with the frame.

```bash
python ur_calibration.py                         # print the transform + corners
python transform_trajectories.py --preview       # → trajectories_ur/*.json + overlay PNG
```

`trajectories_ur/*.json` are tagged `"frame": "ur_base_mm"` and run in
**absolute mode** — no recentring, no rescale — at their true table
position:

```bash
python imu_live.py --json trajectories_ur/star_5_arm_hand_drawn.json --no-robot
python import_trajectory.py trajectories_ur/circle_r_45_start_0deg_near_imu3.json --run --speed 30
```

> The working area is 23 cm; the calibrated square measured from the two
> corner points is ~223 mm (22.3 cm). The viewer draws that actual rotated
> square as the working-area outline, so trajectories and their start points
> safety clamp is widened automatically to the trajectory's true span. If a
> transformed shape ever looks mirrored, set `MIRROR = True` in
> `ur_calibration.py` and re-run `transform_trajectories.py`.

The live viewer shows the yaw as a red heading arrow at the end-effector and
in the readout.

**Yaw drives the tool orientation.** During motion the end-effector is rotated
about the base **vertical (Z)** axis to each waypoint's yaw, composed with the
reference orientation (`APPLY_YAW = True` in `ur5_imu.py`). The start yaw is
reached during the approach so sliding begins settled. `YAW_REF_DEG` shifts the
yaw zero if the trajectory frame needs aligning to the robot; set
`APPLY_YAW = False` to hold the tool fixed and only record/display yaw. Only X
and Y are ever rescaled to the working area — angles pass through unchanged.

```bash
# Preview the sample heart scaled to the working area
python import_trajectory.py sample_trajectories/heart.json --preview

# Scale to a 14 cm span and save the normalised path
python import_trajectory.py sample_trajectories/heart.json --size 140 --save heart_scaled.json

# Import and run it on the robot
python import_trajectory.py sample_trajectories/heart.json --run --speed 40

# View an imported path in the live dashboard
python imu_live.py --json sample_trajectories/heart.json --size 140
```

By default the importer **fills** the target span (scales up or down, aspect
preserved). Use `--no-fill` to only shrink oversized paths and otherwise keep
their original scale.

## Robot / safety notes

- Robot IP defaults to `177.22.22.2`; override with the `UR_ROBOT_IP` env var.
- Motion happens in a plane at `--height` mm **above** the reference pose
  (default 30 mm), with travel moves at 60 mm clearance. Nothing is pressed
  into a surface.
- The reference pose is the centre of the working area
  (`REFERENCE_POSE` in `ur5_imu.py`, shared with `ur5_friction.py`).
- Every trajectory — generated or imported — is clamped to 23 cm × 23 cm
  before it is sent to the robot. Still, **keep the e-stop within reach**,
  and verify the reference pose leaves room for the chosen span.
