# control_robot — XYZ jog + XJC force + Robotiq gripper GUI

A single Tkinter window to drive the UR5e by hand while watching the XJC force
sensor and operating a Robotiq 2-finger gripper.

## What it does
- **Jog X / Y / Z** of the TCP in the base frame — **hold-to-move**: press and
  hold a key or button and the robot moves continuously until you release. Motion
  is a smooth 125 Hz `servoL` stream (velocity control) with **software accel/decel
  ramps** (`JOG_ACCEL_MMSS` / `JOG_DECEL_MMSS`) so starts/stops are jerk-free even
  at high speed. A jog-speed slider (`+`/`-` keys adjust it) sets the top speed.
- **Xbox / gamepad analog jog** — left stick → X/Y, right stick (vertical) → Z,
  **LT / RT triggers → rotate the TCP about base Z** (RT one way, LT the other).
  Dead-zone + squared response; stick, triggers, keys and buttons all feed the same
  accel/decel ramp. (Z-rotation needs `scipy`; tuned by `JOG_ROT_SPEED`/`JOG_RZ_SIGN`.)
- **White or dark UI** — `--theme light` (default) or `--theme dark`.
- **Force source selector** — the Force card and 3D view can show either the **XJC
  sensor** or the **UR robot's own TCP wrench**; the **Tare** button zeroes whichever
  is selected (the UR tare is a software offset to null out tool weight).
- **Routines tab (two steps)** — **Step 1 · Points**: teach a library of named
  positions (＋ Add point, capture live pose, Go to preview). **Step 2 · Sequence**:
  add steps that each pick a point from a dropdown (**repeats allowed**), with a
  per-step hold time and ↑/↓ reordering. Set the iteration count + move speed, then
  **▶ Run**. **Named routines**: type/pick a name and Save / Load / Delete multiple
  routines (stored as `routines/<name>.json`); the newest auto-loads at startup and
  the legacy `routine.json` is migrated in on first run. STOP / Space aborts a run.

X and Y jog are inverted (`JOG_AXIS_SIGN`) to match the physical install; the
release deceleration is deliberately gentle (`JOG_DECEL_MMSS`) so the TCP coasts
to a stop instead of braking hard.
- **Live force read-out** of the XJC X-6A 6-axis sensor (Fx…Mz, poll mode over
  RS485/Modbus) plus the robot's own TCP pose and Fz. A **Tare XJC** button
  zeroes the sensor; Fz turns red past a soft contact threshold.
- **Set / Go home** — *Set home* saves the current TCP pose as this session's
  home (written to `home_pose.json`, with a timestamped copy under `homes/`);
  *Go home* returns to it. The latest home auto-loads on the next launch.
- **Robotiq 2F gripper** (2F-85/140) via the UR URCap socket on the robot
  (port 63352, shares the robot IP): connect+activate, open, close, and a
  position slider with speed/force.

Each subsystem degrades gracefully — a missing robot, XJC port, or gripper
URCap disables only its own panel.

## Run
```bash
python3 robot_control_gui.py --ip 177.22.22.2    # XJC port auto-detected
python3 robot_control_gui.py --xjc-port /dev/ttyUSB0   # or pin it explicitly
python3 robot_control_gui.py --all-ports         # probe ttyS* too when detecting
python3 robot_control_gui.py --no-xjc            # robot + gripper only
python3 robot_control_gui.py --sim               # no hardware, fake signals
UR_ROBOT_IP=127.0.0.1 python3 robot_control_gui.py
```

The XJC port is **auto-detected** by probing each USB serial port with a Modbus
poll (ttyUSB*/ttyACM*; add `--all-ports` for legacy ttyS*) — same approach as
`indentation_test`. Pass `--xjc-port` to skip detection.

## Requirements
- `ur_rtde` (robot), `pyserial` (XJC), `matplotlib`/`numpy` (3D view),
  `pygame` (gamepad) — all already in the project env.
- The gripper needs the **Robotiq_Grippers URCap** installed and running on the
  UR controller (it serves the socket on port 63352). No extra cabling: connect
  to the robot's IP.

## Look
A flat, dark "control console" theme (GitHub-dark palette) — card panels instead
of grooved frames, flat accent/danger buttons, a top bar with live status pills
(robot / control / XJC / gripper), and a dark 3D view. Styling lives in
`theme.py` (`apply_theme(root)` → `(COLORS, FONTS)`); edit the `COLORS` dict there
to recolour the whole app.

## 3D view
The sensor is drawn as a **solid rigid body** showing all **6 DOF**: forces
Fx/Fy/Fz as straight arrows, torques Mx/My/Mz as curved arrows (arcs), **and the
box itself shifts along the net force and tilts about the net torque** so you can
read the load intuitively. No grid/axes clutter. Colour-coded **X=red, Y=green,
Z=blue**; `|F|`/`|M|` in the title. The **Source** selector (XJC / UR robot) and
**Tare** sit in the Force card.

## Files
- `robot_control_gui.py` — the Tkinter app.
- `theme.py` — product-style ttk theme, `light` (white) + `dark` palettes.
- `joystick.py` — Xbox/gamepad analog reader (pygame, headless).
- `robotiq_gripper.py` — Robotiq URCap socket client (SET/GET ACT, GTO, POS…).
- `xjc_sensor.py` — copy of the reconstructed XJC F/T driver.

## Jog tuning
The continuous jog uses `servoL` (velocity control). Constants at the top of
`robot_control_gui.py`:
- `SERVO_HZ` (125) — stream rate.
- `SERVO_LOOKAHEAD` (0.10 s) — higher = smoother/softer, lower = snappier.
- `SERVO_GAIN` (300) — higher = stiffer tracking.
- `JOG_SPEED_DEFAULT` / `_MIN` / `_MAX` — the mm/s slider range.
If `servoL` ever trips controller error **C271A1** ("runtime too much behind"),
lower `SERVO_HZ`/speed or raise `SERVO_LOOKAHEAD`.

## Safety
Hold-to-move is continuous velocity control — releasing the key/button, pressing
**STOP** (`Space`), or closing the window all `servoStop`. Keep the jog speed
modest near the sample and a hand on the e-stop. **Go to reference pose** stops
the jog stream, then does a normal `moveL`.
