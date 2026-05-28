# Star-Nose Sensor System

A UR5 robot-integrated capacitive tactile sensor platform for texture and material characterisation, inspired by the star-nosed mole's somatosensory layout.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture Diagram](#architecture-diagram)
3. [Hardware Setup](#hardware-setup)
4. [Sensor Layout](#sensor-layout)
5. [Environment Setup](#environment-setup)
6. [File-by-File Reference](#file-by-file-reference)
7. [Usage Sheet — All Modes & Commands](#usage-sheet--all-modes--commands)
8. [Tip Profile System](#tip-profile-system)
9. [Calibration Workflow](#calibration-workflow)
10. [Data Format](#data-format)
11. [Output & Analysis](#output--analysis)
12. [Dependencies](#dependencies)

---

## System Overview

Star-Nose Sensor couples a 19-point capacitive tactile sensor with a UR5 collaborative robot arm. The robot positions the sensor over a hexagonal grid of contact points, presses down a configurable depth, and records multi-cell pressure together with TCP force/torque and an external FUTEK load cell. Sessions are logged to CSV, and a post-processing pipeline produces standardised plots for every session.

The system supports **multiple tip profiles**: each tip has its own global X/Y/Z calibration (`calib_<tip>.json`) and per-point fine calibration (`calib_points_<tip>.json`), both loaded automatically at session start.

```
┌────────────────────────────────────────────────────────────────────┐
│                     STAR-NOSE SENSOR SYSTEM                        │
│                                                                    │
│  ┌────────────┐   serial      ┌─────────────┐                     │
│  │ Capacitive │──────────────▶│  sensor.py  │                     │
│  │   Sensor   │ /dev/ttyACM0  │  (19 cells) │                     │
│  │ 252 cells  │  115200 baud  └──────┬──────┘                     │
│  └────────────┘                      │  /tmp/star_nose_sensor.json │
│                                      │                             │
│  ┌────────────┐   RTDE/TCP    ┌──────▼──────┐                     │
│  │    UR5     │◀─────────────▶│ur5_control  │                     │
│  │   Robot    │  177.22.22.2  │  (21 wpts)  │                     │
│  └────────────┘               └──────┬──────┘                     │
│                                      │                             │
│  ┌────────────┐   AI0 (0–10V) │                                   │
│  │   FUTEK    │───────────────┘  (logged as ai0 column)           │
│  │ Load Cell  │  5V = 0 N                                         │
│  │  10 lb     │  Compression → voltage < 5V                       │
│  └────────────┘                                                    │
│                                                                    │
│         ┌──────────────────────┬───────────────────┐              │
│         │                      │                   │              │
│  ┌──────▼──────┐  ┌───────────▼──────┐  ┌────────▼───┐          │
│  │  main.py    │  │ visualizer_2d.py │  │ sofa_scene │          │
│  │  (logger)   │  │  (pygame HUD)    │  │  (3D SOFA) │          │
│  └──────┬──────┘  └──────────────────┘  └────────────┘          │
│         │                                                          │
│  ┌──────▼──────┐                                                   │
│  │  logs/*.csv │──▶  analyze_session.py  ──▶  plots/              │
│  └─────────────┘                                                   │
└────────────────────────────────────────────────────────────────────┘
```

---

## Architecture Diagram

### Data-flow timeline during a session

```
t=0                                                              t=end
│                                                                   │
│  sensor.py ──────────────────────────────────────────────────▶  │  20 Hz serial read
│      │                                                            │
│      └── /tmp/star_nose_sensor.json ─────────────────────────▶  │  30 Hz update
│                ▲                    ▲                             │
│                │                   │                             │
│      visualizer_2d.py          sofa_scene.py                    │  read shared JSON
│      (pygame display)          (SOFA 3D)                         │
│                                                                   │
│  ur5_control.py ─────────────────────────────────────────────▶  │  RTDE @ 125 Hz
│      │  loads calib_<tip>.json  +  calib_points_<tip>.json       │
│      ├── travel → surface → press <depth> mm → dwell → lift     │
│      └── repeat for 21 waypoints                                  │
│                                                                   │
│  FUTEK load cell ────────────────────────────────────────────▶  │  via UR5 AI0
│                                                                   │
│  data_logger.py ─────────────────────────────────────────────▶  │  20 Hz CSV write
│      └── logs/{prefix}_session_{timestamp}.csv                   │
│                                                                   │
│  [session ends] ──▶ analyze_session.py ──▶ plots/{session}/     │
```

### Process-launch sequence (`main.py`)

```
main.py
  │
  ├─ [0] load_calibration.py   startup     (global + per-point offsets)
  ├─ [1] sensor.py             subprocess  (serial reader + shared JSON)
  ├─ [2] ur5_control.py        thread      (RTDE trajectory + force reader)
  ├─ [3] data_logger.py        thread      (20 Hz CSV logger)
  ├─ [4] visualizer_2d.py      subprocess  (pygame, reads shared JSON)
  ├─ [5] sofa_scene.py         subprocess  (SOFA, reads shared JSON)
  └─ [6] robot_viz_*.py        subprocess  (optional 3D robot twin)
```

---

## Hardware Setup

| Component | Specification |
|-----------|---------------|
| Robot | UR5 collaborative arm |
| Robot IP | `177.22.22.2` |
| Sensor | Capacitive grid — 12 cols × 21 rows = 252 cells |
| Active cells | 19 (mapped from full grid) |
| Serial port | `/dev/ttyACM0` @ 115 200 baud |
| Default press depth | 6 mm (configurable per session) |
| Dwell time | 1.5 s per contact point |
| Travel speed | 0.05 m/s |
| Press speed | 0.01 m/s |
| Force sensor rate | 125 Hz (6-axis TCP) |
| Logging rate | 20 Hz |
| Load cell | FUTEK 10 lb compression, connected to UR5 AI0 |
| Load cell zero | 5.0 V = 0 N (amplifier offset) |
| Load cell sensitivity | 8.896 N/V (44.482 N / 5 V range) |

---

## Sensor Layout

The 19 active cells are arranged in a 5-row hexagonal grid. Each cell has a point label (P1–P19), a raw sensor index (S0–S52), and physical coordinates relative to the centre point P10.

```
         P1(S24)  P2(S12)  P3(S0)          y = +14 mm
    P4(S37)  P5(S25)  P6(S13)  P7(S1)      y =  +7 mm
  P8(S50) P9(S38) P10(S26) P11(S14) P12(S2)  y = 0 mm  ← centre
    P13(S51) P14(S39) P15(S27) P16(S15)    y =  -7 mm
         P17(S52) P18(S40)  P19(S28)        y = -14 mm

         ← x = -16 mm              x = +16 mm →
```

> **Note:** The sensor is mounted ~120° rotated relative to the robot frame.
> `UR5_TO_IDX` in each script corrects for this — do not change unless you physically re-mount the sensor.

### UR5 Trajectory (21 waypoints)

```
10 → 1 → 2 → 3 → 7 → 6 → 5 → 4 → 8 → 9 → 10 →
11 → 12 → 16 → 15 → 14 → 13 → 17 → 18 → 19 → 10
```

---

## Environment Setup

The project uses a dedicated **conda environment** (`star_nose`) with Python 3.10. This avoids conflicts with the system ROS2 environment.

```bash
# Create and activate (first time only)
conda create -n star_nose python=3.10 -y
conda activate star_nose
conda install -c conda-forge ffmpeg -y
python -m pip install ur-rtde numpy matplotlib pandas pyserial pygame meshcat pybullet

# Every subsequent session
conda activate star_nose
cd ~/Documents/Star_muse_sensor/Star_nose_sensor/Integration_2
```

> **Why conda?** ROS2 sourcing sets `VIRTUAL_ENV=/usr`, which hides `~/.local`
> packages from Python. The conda environment is fully isolated from this.

SOFA (v25.12.00) is pre-installed at `~/sofa/SOFA_v25.12.00_Linux` and is
added to the Python path automatically when the conda environment activates.

---

## File-by-File Reference

### `main.py` — System Orchestrator

Entry point that wires every subsystem together.

**Startup sequence (interactive):**
1. Parse CLI flags
2. Show calibration confirmation — prints tip name, file, X/Y/Z offsets → press `y` to continue
3. Ask for press depth in mm (Enter = keep default 6 mm)
4. Start sensor, robot, logger, visualisers

**Flags:**

| Flag | Effect |
|------|--------|
| `--tip <name>` | Load `calib_<name>.json` + `calib_points_<name>.json` |
| `--log-prefix <str>` | Prefix for the CSV filename |
| `--duration <s>` | Auto-stop after N seconds |
| `--no-sofa` | Skip SOFA 3D visualiser |
| `--no-viz` | Skip pygame 2D HUD |
| `--no-robot` | Sensor + logging only (no robot motion) |
| `--demo` | Simulated sensor data, no hardware |
| `--sim` | Connect to URSim at localhost |
| `--robot-viz` | Launch matplotlib 3D robot twin |
| `--robot-viz-meshcat` | Launch browser-based Meshcat robot twin |
| `--robot-viz-pybullet` | Launch PyBullet OpenGL robot twin |
| `--analyze` | Run full analysis after session ends |
| `--viz-only` | 2D visualiser only |
| `--sofa-only` | SOFA only |
| `--log-only` | Logging only |

---

### `sensor.py` — Capacitive Sensor Driver

Handles all serial communication and makes readings available to other processes via a shared JSON file.

- Opens `/dev/ttyACM0` at 115 200 baud
- Reads 252-cell grid, extracts 19 mapped cells (`USED_CELLS`)
- Normalises: `value = clip((raw/baseline − 1) × SENSITIVITY, 0, 1) ^ GAMMA`
- Writes to `/tmp/star_nose_sensor.json` at ~30 Hz
- Auto-reconnects on serial error

| Constant | Default | Effect |
|----------|---------|--------|
| `SENSITIVITY` | 30.0 | Higher = more sensitive |
| `GAMMA` | 0.5 | < 1 boosts weak signals |

---

### `ur5_control.py` — Robot Controller

Implements the UR5 RTDE control interface and trajectory execution.

- Applies **global** calibration offset (X/Y/Z) via `set_calibration()`
- Applies **per-point** (dx, dy) offsets via `set_point_offsets()` — loaded from `calib_points_<tip>.json`
- Press depth and dwell configurable at runtime

| Parameter | Default | Where to change |
|-----------|---------|-----------------|
| `DEFAULT_INDENT_MM` | 6.0 mm | Prompted at session start, or edit directly |
| `DEFAULT_DWELL_S` | 1.5 s | Edit directly in file |
| `POINT_OVERRIDES` | `{}` | Dict of `{pt: (depth_mm, dwell_s)}` overrides |

---

### `load_calibration.py` — Calibration Loader

Loads both calibration files for a given tip and applies them to `ur5_control`.

```python
# Called automatically by main.py
load_calibration.preview(tip='short_6mm')   # shows summary, asks y/N
load_calibration.apply(tip='short_6mm')     # loads global + per-point offsets
```

Files loaded:
- `calib_<tip>.json` → global X/Y/Z offset
- `calib_points_<tip>.json` → per-point dx/dy corrections (if present)

---

### `calibrate_ur5.py` — Global TCP Calibration

Interactively aligns the TCP over the sensor centre (P10/S26) to set the global X/Y/Z offset.

```bash
python calibrate_ur5.py              # default calib.json
python calibrate_ur5.py --tip short_6mm   # saves calib_short_6mm.json
```

| Command | Action |
|---------|--------|
| `x+` / `x-` | Jog X axis |
| `y+` / `y-` | Jog Y axis |
| `z+` / `z-` | Jog Z axis (surface height) |
| `step <mm>` | Change jog step size |
| `press` | Test press, read sensor peak at P10 |
| `status` | Show TCP pose and current offsets |
| `reset` | Zero offsets |
| `save` | Write `calib_<tip>.json` |
| `quit` | Exit without saving |

---

### `calibrate_points.py` — Per-Point Fine Calibration

Interactive tool that fine-tunes each of the 19 points individually after the global calibration.

```bash
python calibrate_points.py                        # all 19 points, default tip
python calibrate_points.py --tip short_6mm        # loads/saves short_6mm profile
python calibrate_points.py --tip short_6mm --scan # auto-scan then show map
python calibrate_points.py --tip short_6mm --point 5      # fix one point
python calibrate_points.py --tip short_6mm --points 4 5 6 # fix several
python calibrate_points.py --tip short_6mm --map           # show deviation map
```

**Interactive commands per point:**

| Command | Action |
|---------|--------|
| `x+` / `x-` | Nudge right / left |
| `y+` / `y-` | Nudge forward / back |
| `step <mm>` | Change nudge step size |
| `press` | Press and read sensor response |
| `teach` | Record current TCP position as offset (use with freedrive) |
| `status` | Show offset + sensor snapshot |
| `map` | Show deviation map for all points so far |
| `ok` | Accept offset → move to next point |
| `skip` | Skip this point |
| `save` | Save all collected offsets and quit |
| `quit` | Quit without saving |

**Live hexmap window:**
- Left panel: all 19 sensor cells coloured by pressure; target cell highlighted in red
- Right panel: bar chart of all 19 cells; **red vertical line marks the current target point** (P-number, not sensor index)

Saves to `calib_points_<tip>.json`.

---

### `data_logger.py` — CSV Session Logger

Runs as a thread inside `main.py`. Appends one row per 20 Hz tick.

**CSV columns (30 total):**

| Column | Description |
|--------|-------------|
| `timestamp` | Unix epoch (float) |
| `datetime` | ISO 8601 string |
| `ur5_point` | Current waypoint (1–19) |
| `ur5_pressing` | 1 during press phase |
| `ur5_done` | 1 after trajectory completes |
| `tcp_x/y/z` | TCP position (m) |
| `fx/fy/fz` | TCP force (N) |
| `tx/ty/tz` | TCP torque (N·m) |
| `ai0` | FUTEK load cell voltage (V); 5 V = 0 N |
| `cell_1…cell_19` | Normalised pressure 0.0–1.0 (sensor electrical order) |

---

### `visualizer_2d.py` — Pygame 2D HUD

Real-time hexagonal pressure map with statistics sidebar. Sensor values are correctly mapped to physical positions using `POS_TO_SENSOR` (accounts for the ~120° sensor rotation).

| Key | Action |
|-----|--------|
| `L` | Toggle cell labels (P1–P19 + Sxx) |
| `V` | Toggle numeric pressure values |
| `C` | Recalibrate baseline |
| `ESC` | Quit |

---

### `sofa_scene.py` — 3D SOFA Visualiser

Live 3D view using SOFA v25.12.00. 19 coloured spheres scale in height and radius with pressure.

| Key | View |
|-----|------|
| `1` | Top-down |
| `2` | Isometric |
| `3` | Side |

---

### `robot_viz_3d.py` — Matplotlib 3D Robot Twin

Real-time matplotlib window showing robot arm pose + sensor hexmap side by side.
Backend auto-selected: `MacOSX` on macOS, `TkAgg` on Linux.

```bash
python robot_viz_3d.py              # live, real robot
python robot_viz_3d.py --sim        # URSim at localhost
python robot_viz_3d.py --replay session.csv   # replay from CSV
python robot_viz_3d.py --demo       # animated demo, no hardware
```

---

### `robot_viz_meshcat.py` — Browser-Based Robot Twin

Opens a browser tab with an interactive Three.js 3D robot viewer. Reads live joint angles from RTDE.

```bash
python robot_viz_meshcat.py         # real robot → open URL printed in terminal
python robot_viz_meshcat.py --sim   # URSim
```

> Recommended for the best interactive 3D view.

---

### `robot_viz_pybullet.py` — PyBullet OpenGL Robot Twin

Native OpenGL window with lighting. Loads a UR5 URDF, overlays sensor hex-grid and TCP trail.

```bash
python robot_viz_pybullet.py
python robot_viz_pybullet.py --sim
```

---

### `animate_session.py` — Hexmap Session Animation

Replays a logged session as an animated hexmap. Backend auto-selected for macOS / Linux.

```bash
python animate_session.py                        # latest session
python animate_session.py dome_empty_burcu       # partial name match
python animate_session.py --speed 2.0            # 2× playback
python animate_session.py --step 3               # every 3rd frame
python animate_session.py --save                 # export MP4 (requires ffmpeg)
python animate_session.py --save --gif           # export GIF
```

**Window layout:**
- **Hexmap** — cells coloured by pressure; red border on the point being pressed
- **Bar chart** — all 19 cells labelled P1–P19 (physical order)
- **History strip** — rolling 200-frame time history
- **AI0 trace** — FUTEK load cell voltage; y-axis scaled to actual data range
- **Progress bar** — session timeline

---

### `analyze_session.py` — Post-Processing Dashboard

Full analysis pipeline. Loads a session CSV and produces standardised figures.

```bash
python analyze_session.py                       # latest session
python analyze_session.py my_label             # partial name match
python analyze_session.py --save               # save all figures to plots/
python analyze_session.py --force              # force analysis only
python analyze_session.py --loadcell           # FUTEK vs robot force only
python analyze_session.py --all                # compare all sessions
```

**Figures generated:**

| Figure | File | Contents |
|--------|------|---------|
| Overview | `overview.png` | Timeline heatmap, peak-per-event bars, hex maps, correlation matrix |
| Per-point | `perpoint.png` | Bar chart of all 19 cells per contact point |
| Hex maps | `hexmaps.png` | Spatial pressure per contact point |
| Force | `force.png` | TCP force timeline, force–sensor scatter, per-point box plots |
| Analog | `analog.png` | AI0 voltage timeline, per-event bars, AI0 vs Fz scatter |
| **Load cell** | `loadcell_vs_robot.png` | FUTEK vs robot force (see below) |
| Comparison | `comparison.png` | Multi-session overlay |

**Load cell vs robot force plot (`--loadcell`):**

Converts AI0 voltage to Newtons using: `F = (ai0 − 5.0) × 8.896 N/V`

| Panel | Contents |
|-------|---------|
| Time series | Both signals in N on the same axis, full session |
| Scatter | Load cell vs Robot Fz (pressing frames), Pearson r |
| Residuals | `Robot Fz − Load cell` over time with mean bias |
| Bland–Altman | Agreement plot: bias ± 1.96σ limits of agreement |
| Per-press bars | Peak force per point — load cell vs robot side by side |

To adjust the load cell calibration constants edit the top of `analyze_session.py`:

```python
AI0_ZERO_V       = 5.0    # V at zero force
LOADCELL_MAX_LB  = 10.0   # rated capacity (lb)
LOADCELL_V_RANGE = 5.0    # V from zero to full scale
```

---

### `verify_mapping.py` — UR5 ↔ Sensor Mapping Verifier

Validates or regenerates the `UR5_TO_IDX` mapping table.

```bash
python verify_mapping.py
# Follow prompts — robot presses each point, script records peak cell
```

Use this after physically re-mounting the sensor.

---

## Usage Sheet — All Modes & Commands

### Running the Full System

```bash
conda activate star_nose
cd ~/Documents/Star_muse_sensor/Star_nose_sensor/Integration_2

# Full session — all subsystems, default tip
python main.py

# With a specific tip profile
python main.py --tip short_6mm

# Custom label + auto-stop
python main.py --tip short_6mm --log-prefix foam_test --duration 120

# No SOFA (faster startup)
python main.py --tip short_6mm --no-sofa

# Headless (no visualisers)
python main.py --tip short_6mm --no-sofa --no-viz

# With browser-based robot visualiser
python main.py --tip short_6mm --robot-viz-meshcat --no-sofa
```

### Session Startup Prompts

Every run with the robot enabled shows two interactive prompts before the robot moves:

```
==================================================
  CALIBRATION CHECK — please confirm before moving
==================================================
  Tip profile : short_6mm
  File        : calib_short_6mm.json
  X offset    : +0.850 mm
  Y offset    : -0.320 mm
  Z offset    : +6.000 mm
==================================================
  Correct tip mounted? Continue? [y/N] > y

  Current indentation : 6.0 mm
  Press depth in mm [Enter = keep current] > 4.5
  Indentation set to  : 4.5 mm
```

### Subsystem Flags

```bash
python main.py --no-sofa               # skip SOFA
python main.py --no-viz                # skip pygame HUD
python main.py --no-robot              # sensor + logging only
python main.py --no-sofa --no-viz      # headless logging
python main.py --viz-only              # 2D HUD only
python main.py --sofa-only             # SOFA only
python main.py --log-only              # logging only
python main.py --demo                  # simulated sensor, no hardware
python main.py --sim                   # connect to URSim (Docker)
```

### Post-Session Analysis

```bash
python analyze_session.py                           # latest session
python analyze_session.py foam_test                 # partial name
python analyze_session.py --save                    # save PNGs
python analyze_session.py --force                   # force plots only
python analyze_session.py --loadcell                # load cell comparison only
python analyze_session.py --loadcell --save         # save load cell plot
python analyze_session.py --all                     # compare all sessions
```

### Session Animation

```bash
python animate_session.py                   # latest session
python animate_session.py foam_test         # partial name
python animate_session.py --speed 2.0       # 2× faster
python animate_session.py --save            # export MP4
python animate_session.py --save --gif      # export GIF
```

### Quick Reference Table

| Goal | Command |
|------|---------|
| Full session (default tip) | `python main.py` |
| Full session (named tip) | `python main.py --tip short_6mm` |
| No SOFA | `python main.py --no-sofa` |
| Headless logging | `python main.py --no-sofa --no-viz` |
| No robot | `python main.py --no-robot` |
| Demo / no hardware | `python main.py --demo` |
| Meshcat robot twin | `python main.py --robot-viz-meshcat` |
| Analyse latest | `python analyze_session.py` |
| Load cell comparison | `python analyze_session.py --loadcell` |
| Animate latest | `python animate_session.py` |
| Animate fast | `python animate_session.py --speed 2` |
| Export animation | `python animate_session.py --save` |
| Calibrate (global) | `python calibrate_ur5.py --tip short_6mm` |
| Calibrate (per-point) | `python calibrate_points.py --tip short_6mm` |
| Scan then show map | `python calibrate_points.py --tip short_6mm --scan` |
| Show deviation map | `python calibrate_points.py --tip short_6mm --map` |
| Verify cell mapping | `python verify_mapping.py` |
| Raw sensor readout | `python sensor_bridge.py` |

---

## Tip Profile System

Each physical tip (different length, material, or geometry) has its own calibration stored in two files:

| File | Contents | Created by |
|------|---------|-----------|
| `calib_<tip>.json` | Global X/Y/Z TCP offset | `calibrate_ur5.py --tip <name>` |
| `calib_points_<tip>.json` | Per-point dx/dy corrections + scan results | `calibrate_points.py --tip <name>` |

### Creating a new tip profile

```bash
# Step 1 — global alignment (set centre + surface height)
python calibrate_ur5.py --tip metal_7mm

# Step 2 — per-point fine calibration
python calibrate_points.py --tip metal_7mm

# Step 3 — run sessions with this tip
python main.py --tip metal_7mm --log-prefix metal_7mm_ecoflex
```

### Tip naming examples

```
short_4mm   long_5mm   flat_rubber   metal_7mm   cone_3mm
```

Any name works — it becomes part of the filename. Default (no `--tip`) uses `calib.json`.

### How offsets are applied

```
TCP position = REFERENCE_POSE
            + POINTS[pt]          (nominal grid position)
            + (CALIB_X, CALIB_Y)  (global tip offset from calib_<tip>.json)
            + (pdx, pdy)          (per-point offset from calib_points_<tip>.json)
```

---

## Calibration Workflow

### Step 1 — Global calibration (`calibrate_ur5.py`)

Aligns the TCP pointer precisely over P10 (sensor centre, S26):

```
1.  python calibrate_ur5.py --tip <name>
2.  Use x+/x-/y+/y-  to centre over S26
3.  Use z-/z+         to reach correct surface height
4.  Type 'press'      to verify sensor peaks at P10
5.  Type 'save'       → writes calib_<name>.json
```

### Step 2 — Per-point calibration (`calibrate_points.py`)

Fine-tunes each of the 19 points:

```
1.  python calibrate_points.py --tip <name> --scan
    (robot presses every point, shows deviation map)

2.  python calibrate_points.py --tip <name> --points 3 7 12
    (interactive fix for specific bad points)

3.  For each point: nudge with x+/y+, press to verify, ok to accept
```

The live window during interactive calibration shows:
- **Hexmap**: physical sensor layout; target point has red border
- **Bar chart**: all 19 values; red line marks the **target point's bar** (e.g. P1, not sensor index)

### Step 3 — Session start confirmation

`main.py` always shows the loaded calibration and asks for confirmation **before the robot moves**, along with the press depth for the session.

---

## Data Format

### CSV Session Log

Each session produces one CSV in `logs/` (30 columns, 20 Hz):

```
timestamp, datetime, ur5_point, ur5_pressing, ur5_done,
tcp_x, tcp_y, tcp_z,
fx, fy, fz, tx, ty, tz,
ai0,
cell_1, cell_2, ..., cell_19
```

**`cell_N` ordering:** Sensor electrical order (index 0–18, matching `USED_CELLS`). Visualisation scripts apply `POS_TO_SENSOR` to map each cell to its correct physical hex position.

**`ai0`:** Raw voltage from UR5 analog input 0. Conversion to Newtons (positive = compression): `F = −(ai0 − 5.0) × 8.896 N/V`. Both load cell and robot Fz are plotted as positive compression force for direct comparison.

### Shared Sensor State (`/tmp/star_nose_sensor.json`)

```json
{
  "cells": [0.0, 0.12, 0.0, 0.87, ...],
  "timestamp": 1746548400.123,
  "connected": true
}
```

### Calibration Files

**`calib_<tip>.json`** (global):
```json
{ "x_mm": -3.0, "y_mm": 1.0, "z_mm": 6.0 }
```

**`calib_points_<tip>.json`** (per-point):
```json
{
  "global":    { "x_mm": -3.0, "y_mm": 1.0, "z_mm": 6.0 },
  "per_point": {
    "1":  { "dx_mm": 0.25, "dy_mm": -0.10 },
    "2":  { "dx_mm": 0.00, "dy_mm":  0.05 },
    ...
  },
  "scan_results": {
    "1": { "expected_raw": 24, "actual_raw": 24, "correct": true, ... },
    ...
  }
}
```

---

## Output & Analysis

After each session, `analyze_session.py` saves figures to `plots/{session_name}/`:

```
plots/
└── foam_test_session_20260528_150039/
    ├── overview.png          ← timeline heatmap + statistics
    ├── perpoint.png          ← bar charts per contact point
    ├── hexmaps.png           ← spatial hex maps per point
    ├── force.png             ← TCP force analysis
    ├── analog.png            ← AI0 voltage timeline
    └── loadcell_vs_robot.png ← FUTEK vs robot force comparison
```

**`loadcell_vs_robot.png` layout:**
```
┌──────────────────────────────────────────────────────────┐
│  Time series: FUTEK (N) + Robot Fz (N) — full session    │
├─────────────┬──────────────────┬───────────────────────── ┤
│  Scatter    │  Residuals       │  Bland–Altman             │
│  (r=0.97)  │  Robot − LC (N)  │  bias ± 1.96σ             │
├─────────────┴──────────────────┴───────────────────────── ┤
│  Per-press peak bars: FUTEK vs Robot for every P1…P19     │
└──────────────────────────────────────────────────────────┘
```

---

## Dependencies

### Python packages (conda `star_nose` environment)

| Package | Purpose |
|---------|---------|
| `ur-rtde` | UR5 RTDE interface (`rtde_control`, `rtde_receive`) |
| `numpy` | Numerical processing |
| `matplotlib` | All plots and animations |
| `pandas` | CSV logging and analysis |
| `pyserial` | Serial communication with sensor |
| `pygame` | 2D real-time visualiser |
| `meshcat` | Browser-based 3D robot twin |
| `pybullet` | OpenGL 3D robot twin |
| `ffmpeg` | MP4 animation export (`conda install -c conda-forge ffmpeg`) |

### System requirements

| Requirement | Notes |
|-------------|-------|
| Python 3.10 | Required (ur-rtde binaries are compiled for 3.10) |
| SOFA v25.12.00 | Pre-installed at `~/sofa/SOFA_v25.12.00_Linux` |
| conda | For isolated environment management |
| UR5 RTDE | Enabled on the robot controller (port 30004) |

### Installation (one-time)

```bash
conda create -n star_nose python=3.10 -y
conda activate star_nose
conda install -c conda-forge ffmpeg -y
python -m pip install ur-rtde numpy matplotlib pandas pyserial pygame meshcat pybullet
```

---

*Star-Nose Sensor System — UR5 + capacitive tactile sensor + FUTEK load cell integration.*
