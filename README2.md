# Star-Nose Sensor System

A UR5 robot-integrated capacitive tactile sensor platform for texture and material characterization, inspired by the star-nosed mole's somatosensory layout.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture Diagram](#architecture-diagram)
3. [Hardware Setup](#hardware-setup)
4. [Sensor Layout](#sensor-layout)
5. [File-by-File Reference](#file-by-file-reference)
6. [Usage Sheet — All Modes & Commands](#usage-sheet--all-modes--commands)
7. [Data Format](#data-format)
8. [Output & Analysis](#output--analysis)
9. [Dependencies](#dependencies)

---

## System Overview

Star-Nose Sensor couples a 19-point capacitive tactile sensor with a UR5 collaborative robot arm. The robot positions the sensor over a hexagonal grid of contact points, presses down a fixed depth, and records multi-cell pressure together with TCP force/torque. Sessions are logged to CSV, and a post-processing pipeline produces standardised plots for every session.

```
┌─────────────────────────────────────────────────────────────────┐
│                   STAR-NOSE SENSOR SYSTEM                       │
│                                                                 │
│  ┌────────────┐    serial     ┌─────────────┐                  │
│  │ Capacitive │──────────────▶│  sensor.py  │                  │
│  │   Sensor   │  /dev/ttyACM0 │  (19 cells) │                  │
│  │ 252 cells  │   115200 baud └──────┬──────┘                  │
│  └────────────┘                      │  shared JSON             │
│                                      │  /tmp/kywo_sensor.json  │
│  ┌────────────┐   RTDE/TCP    ┌──────▼──────┐                  │
│  │    UR5     │◀─────────────▶│ur5_control  │                  │
│  │   Robot    │  177.22.22.2  │  (21 wpts)  │                  │
│  └────────────┘               └──────┬──────┘                  │
│                                      │                          │
│              ┌───────────────────────┼───────────────────┐     │
│              │                       │                   │     │
│       ┌──────▼──────┐    ┌──────────▼──────┐  ┌────────▼───┐ │
│       │  main.py    │    │ visualizer_2d   │  │ sofa_scene │ │
│       │  (logger)   │    │  (pygame HUD)   │  │  (3D SOFA) │ │
│       └──────┬──────┘    └─────────────────┘  └────────────┘ │
│              │                                                  │
│       ┌──────▼──────┐                                          │
│       │  logs/*.csv │──▶  analyze_session.py  ──▶  plots/     │
│       └─────────────┘                                          │
└─────────────────────────────────────────────────────────────────┘
```

---

## Architecture Diagram

### Data-flow timeline during a session

```
t=0                                                            t=end
│                                                                 │
│  sensor.py ────────────────────────────────────────────────▶  │  20 Hz serial read
│      │                                                          │
│      └── /tmp/kywo_sensor.json (shared memory) ──────────▶    │  30 Hz update
│                ▲                    ▲                           │
│                │                   │                           │
│      visualizer_2d.py          sofa_scene.py                  │  read shared JSON
│      (pygame display)          (SOFA 3D)                       │
│                                                                 │
│  ur5_control.py ───────────────────────────────────────────▶  │  RTDE @ 125 Hz force
│      │                                                          │
│      ├── travel to point ──▶ dwell ──▶ press 6mm ──▶ lift    │
│      └── repeat for 21 waypoints                               │
│                                                                 │
│  data_logger.py ───────────────────────────────────────────▶  │  20 Hz CSV write
│      └── logs/{prefix}_session_{timestamp}.csv                 │
│                                                                 │
│  [session ends] ──▶ analyze_session.py ──▶ plots/{session}/   │
```

### Process-launch sequence (main.py)

```
main.py
  │
  ├─ [1] sensor.py          subprocess  (serial reader + shared JSON)
  ├─ [2] ur5_control.py     thread      (RTDE trajectory + force reader)
  ├─ [3] data_logger.py     thread      (20 Hz CSV logger)
  ├─ [4] visualizer_2d.py   subprocess  (pygame, reads shared JSON)
  └─ [5] sofa_scene.py      subprocess  (SOFA, reads shared JSON)
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
| Indentation depth | 6 mm (default) |
| Dwell time | 1.5 s per contact point |
| Travel speed | 0.05 m/s |
| Press speed | 0.01 m/s |
| Force sensor rate | 125 Hz (6-axis TCP) |
| Logging rate | 20 Hz |

---

## Sensor Layout

The 19 active cells are arranged in a 5-row hexagonal grid. Each cell has a label (P1–P19), a raw sensor index (0–52), and physical coordinates relative to the centre point P10.

```
         P1(2)    P2(15)   P3(28)          y = +14 mm
    P4(1)   P5(14)   P6(27)   P7(40)       y =  +7 mm
  P8(0)  P9(13) P10(26) P11(39) P12(52)   y =   0 mm  ← centre
   P13(12) P14(25) P15(38) P16(51)         y =  -7 mm
        P17(24)  P18(37)  P19(50)          y = -14 mm

         ← x = -16 mm          x = +16 mm →
```

Numbers in parentheses are raw hardware cell indices. P10 (raw index 26, labelled S26) is the mechanical reference used for robot calibration.

### UR5 Trajectory (21 waypoints)

The robot visits the 19 sensor points plus two additional passes through the centre:

```
10 → 1 → 2 → 3 → 7 → 6 → 5 → 4 → 8 → 9 → 10 →
11 → 12 → 16 → 15 → 14 → 13 → 17 → 18 → 19 → 10
```

This spiral-like path minimises travel distance while ensuring every cell is contacted once.

---

## File-by-File Reference

### `main.py` — System Orchestrator

The entry point that wires every subsystem together. Responsibilities:

- Parses command-line flags to select which subsystems to launch
- Spawns `sensor.py`, `visualizer_2d.py`, and `sofa_scene.py` as **subprocesses** (so each has its own Python interpreter)
- Runs `ur5_control` and `data_logger` as **threads** (shared memory with main)
- Waits for the UR5 trajectory to complete, then runs `analyze_session.py`
- Implements a clean shutdown: kills subprocesses, joins threads, flushes CSV

Key functions:

| Function | Purpose |
|----------|---------|
| `run_sensor_subprocess()` | Launches `sensor.py` in its own process |
| `run_visualizer_subprocess()` | Launches `visualizer_2d.py` |
| `run_sofa_subprocess()` | Launches `sofa_scene.py` |
| `monitor_ur5()` | Thread: polls UR5 state, writes to shared dict |
| `main()` | Argument parsing + orchestration |

---

### `sensor.py` — Capacitive Sensor Driver

Handles all serial communication with the sensor hardware and makes readings available to other processes via a shared JSON file.

- Opens `/dev/ttyACM0` at 115 200 baud with low-latency ioctl settings
- Reads the full 252-cell grid, extracts 19 mapped cells
- Normalises: `value = clip((raw / baseline - 1) × SENSITIVITY, 0, 1) ^ GAMMA`
  - `SENSITIVITY = 30.0` — amplification factor
  - `GAMMA = 0.5` — power-law tone-mapping (boosts low signals)
- Writes normalised values to `/tmp/kywo_sensor.json` at ~30 Hz
- Auto-reconnects on serial error (fast retry 0.3 s, slow retry 3.0 s)

Key constants:

| Constant | Value | Effect |
|----------|-------|--------|
| `SENSITIVITY` | 30.0 | Higher = more sensitive, risk of saturation |
| `GAMMA` | 0.5 | < 1 boosts dim signals; = 1 linear |
| `N_CELLS` | 19 | Active sensor points |
| `GRID_COLS` | 12 | Hardware grid width |

---

### `sensor_bridge.py` — Legacy Sensor Interface

A simpler, standalone serial reader without shared-memory output. Used for quick diagnostics or as a drop-in sensor reader in alternative pipelines.

- `SENSITIVITY = 8.0`, no gamma correction
- Prints raw normalised values to stdout
- Does **not** write a shared JSON file
- Suitable for one-off debugging sessions

---

### `ur5_control.py` — Robot Controller

Implements the full UR5 RTDE (Real-Time Data Exchange) control interface.

- Connects to the robot at `177.22.22.2`
- Loads calibration offsets from `calib.json` (`CALIB_X_MM`, `CALIB_Y_MM`, `CALIB_Z_MM`)
- Executes the 21-waypoint hexagonal trajectory:
  - Travel at 0.05 m/s to XY position above point
  - Descend 6 mm at 0.01 m/s (press phase)
  - Dwell 1.5 s while data is logged
  - Retract, move to next point
- Background thread reads 6-axis TCP force/torque at 125 Hz
- Exposes `get_state()` dict: `{point, pressing, done, tcp_pose, force}`

Key configuration:

| Parameter | Value |
|-----------|-------|
| Robot IP | `177.22.22.2` |
| Travel speed | 0.05 m/s |
| Press speed | 0.01 m/s |
| Acceleration | 0.3 m/s² |
| Press depth | 6 mm |
| Dwell | 1.5 s |

---

### `calibrate_ur5.py` — Interactive TCP Calibration Tool

Command-line tool to align the robot's TCP pointer over the sensor centre point (P10 / S26) before a session.

Interactive commands:

| Command | Action |
|---------|--------|
| `x+` / `x-` | Jog X axis by step size |
| `y+` / `y-` | Jog Y axis by step size |
| `z+` / `z-` | Jog Z axis by step size |
| `step <mm>` | Change jog step size |
| `press` | Perform a test press and read sensor peak |
| `status` | Print current TCP pose and calibration offsets |
| `reset` | Zero the calibration offsets |
| `save` | Write offsets to `calib.json` |
| `quit` | Exit without saving |

Workflow:

```
1. Run: python3.10 calibrate_ur5.py
2. Use x+/x-/y+/y- to centre pointer over S26
3. Use z- to set correct contact height
4. Type 'press' to verify sensor activates at P10
5. Type 'save' → writes calib.json
```

---

### `data_logger.py` — CSV Session Logger

Runs as a thread inside `main.py`. At 20 Hz it samples the shared sensor state and the UR5 state dict and appends one row to the session CSV.

CSV columns (29 total):

| Column | Type | Description |
|--------|------|-------------|
| `timestamp` | float | Unix epoch seconds |
| `datetime` | str | ISO 8601 human-readable |
| `ur5_point` | int | Current waypoint index (1–19) |
| `ur5_pressing` | int | 1 while robot is in press phase |
| `ur5_done` | int | 1 after trajectory completes |
| `tcp_x/y/z` | float | TCP position in metres |
| `fx/fy/fz` | float | TCP force in Newtons |
| `tx/ty/tz` | float | TCP torque in N·m |
| `cell_1…cell_19` | float | Normalised pressure 0.0–1.0 |

Files are saved to `logs/{prefix}_session_{YYYYMMDD_HHMMSS}.csv`.

---

### `load_calibration.py` — Calibration Loader

Minimal utility (22 lines). Reads `calib.json` and passes the offsets to `ur5_control.set_calibration()`. Called at startup by `main.py` before the trajectory begins.

```python
# Usage (programmatic)
from load_calibration import load_calibration
load_calibration(ur5)   # ur5 is a UR5Control instance
```

---

### `sofa_scene.py` — 3D SOFA Visualizer

Renders a live 3D view of the sensor using the SOFA Framework (v25.12.00).

- 19 coloured spheres positioned at the physical sensor layout coordinates
- Sphere **height** and **radius** scale with normalised pressure
- **Colour ramp**: teal (0) → green → yellow → orange → red (1.0)
- Reads from `/tmp/kywo_sensor.json` (no serial access, subprocess-safe)
- Keyboard shortcuts inside the SOFA window:

| Key | Camera view |
|-----|-------------|
| `1` | Top-down |
| `2` | Isometric |
| `3` | Side |

---

### `visualizer_2d.py` — Pygame 2D HUD

A real-time 2D dashboard split into two panels:

**Left panel — Hexagonal pressure map**
- Draws the 19 sensor points at their physical hex layout
- Cell fill colour and radius scale with pressure
- Colour ramp matches the SOFA scene (teal → red)
- Optional overlays toggled by keyboard

**Right panel — Statistics sidebar**
- Active cell count
- Peak cell value and index
- UR5 current point and pressing state
- Intensity bar for each of the 19 cells

Keyboard controls:

| Key | Action |
|-----|--------|
| `L` | Toggle cell labels (P1–P19) |
| `V` | Toggle numeric pressure values |
| `C` | Recalibrate baseline (re-zero sensor) |
| `D` | Toggle demo mode (simulated data) |
| `ESC` | Quit |

Connection indicator (top-right corner):

| Colour | Meaning |
|--------|---------|
| GREEN | Live data from sensor.py |
| AMBER | Buffered / demo mode |
| RED | Reconnecting |

---

### `analyze_session.py` — Post-Processing & Plotting

Full analysis pipeline (921 lines). Loads one or more session CSVs and produces a set of standardised figures saved to `plots/{session_name}/`.

**Plots generated:**

| Figure | Filename | Contents |
|--------|----------|---------|
| Overview | `overview.png` | Timeline heatmap, peak-per-event bars, per-point hex maps, cross-cell correlation matrix, event-duration histogram |
| Per-point responses | `perpoint.png` | Bar chart of all 19 cell values for each UR5 contact point |
| Hex maps | `hexmaps.png` | Spatial pressure distribution rendered on hex layout for every contact point |
| Force analysis | `force.png` | TCP force timeline, force-vs-sensor scatter, per-point force box plots |
| Comparison | `comparison.png` | Side-by-side overlay of multiple sessions |

**Metrics computed per press event:**

- Peak and mean per cell
- Target cell accuracy (was the highest-activated cell the expected one?)
- Peak normal force (Fz) and mean force during press

Usage:

```bash
# Analyse the most recent session (auto-called by main.py)
python3.10 analyze_session.py

# Analyse a specific file
python3.10 analyze_session.py logs/my_session.csv

# Compare multiple sessions
python3.10 analyze_session.py logs/session_A.csv logs/session_B.csv
```

---

### `verify_mapping.py` — UR5 ↔ Sensor Mapping Verifier

Interactive tool to validate or regenerate the `UR5_TO_SENSOR` mapping table in `ur5_control.py`.

Workflow:

```
1. Run: python3.10 verify_mapping.py
2. Press ENTER to begin
3. Robot presses each point in order
4. Script records which sensor cell peaks highest
5. Prints updated UR5_TO_SENSOR dict for copy-paste into ur5_control.py
```

Use this after physically repositioning the sensor or after a major mechanical change.

---

## Usage Sheet — All Modes & Commands

### Prerequisites

```bash
# Required Python version
python3.10

# Install Python dependencies
pip install pyserial pandas numpy matplotlib pygame

# SOFA Framework must be installed separately (v25.12.00)
# UR5 RTDE libraries: ur5_control, ur5_receive
```

### Running the Full System

```bash
# Full integration (sensor + robot + 2D viz + 3D SOFA + logging)
python3.10 main.py

# Custom session label (changes CSV filename prefix)
python3.10 main.py --log-prefix my_experiment

# Auto-stop after N seconds
python3.10 main.py --duration 120

# Custom log prefix AND duration
python3.10 main.py --log-prefix texture_test --duration 90
```

### Subsystem Flags (skip components)

```bash
# Skip 3D SOFA visualizer (faster startup, no SOFA required)
python3.10 main.py --no-sofa

# Skip 2D pygame visualizer
python3.10 main.py --no-viz

# Skip UR5 robot (sensor + logging only, manual contact)
python3.10 main.py --no-robot

# Skip both visualizers (headless logging)
python3.10 main.py --no-sofa --no-viz
```

### Isolated Launch Modes

```bash
# 2D visualizer only (reads /tmp/kywo_sensor.json)
python3.10 main.py --viz-only

# 3D SOFA scene only
python3.10 main.py --sofa-only

# Sensor + logger only (no robot, no visualizers)
python3.10 main.py --log-only

# Simulated sensor data — no hardware required
python3.10 main.py --demo
```

### Post-Session Analysis

```bash
# Analyse the most recent session in logs/
python3.10 main.py --analyze

# Analyse directly
python3.10 analyze_session.py

# Analyse a specific CSV
python3.10 analyze_session.py logs/my_experiment_session_20260506_170404.csv

# Compare two or more sessions
python3.10 analyze_session.py logs/session_A.csv logs/session_B.csv
```

### Calibration Workflow

```bash
# Step 1 — physically position the robot and run the calibration tool
python3.10 calibrate_ur5.py

# Step 2 — inside the tool
#   x+  x-  y+  y-  z+  z-   → jog by current step
#   step 0.5                   → set jog step to 0.5 mm
#   press                      → test press, prints sensor peak
#   status                     → show TCP pose and offsets
#   reset                      → zero offsets
#   save                       → write calib.json
#   quit                       → exit

# Step 3 — apply saved calibration when starting main.py
#   (load_calibration.py is called automatically by main.py)
```

### Mapping Verification

```bash
# Verify or regenerate UR5 ↔ sensor cell mapping
python3.10 verify_mapping.py
# Follow on-screen prompts (ENTER to advance through points)
```

### Standalone Sensor Test

```bash
# Quick sensor readout without robot or visualizer
python3.10 sensor_bridge.py
```

### Quick Reference Table

| Goal | Command |
|------|---------|
| Full experiment | `python3.10 main.py --log-prefix my_label` |
| No 3D viewer | `python3.10 main.py --no-sofa` |
| No visualizers (headless) | `python3.10 main.py --no-sofa --no-viz` |
| No robot (manual) | `python3.10 main.py --no-robot` |
| Demo / no hardware | `python3.10 main.py --demo` |
| 2D viewer only | `python3.10 main.py --viz-only` |
| 3D viewer only | `python3.10 main.py --sofa-only` |
| Logging only | `python3.10 main.py --log-only` |
| Analyse latest session | `python3.10 main.py --analyze` |
| Analyse specific file | `python3.10 analyze_session.py logs/file.csv` |
| Compare sessions | `python3.10 analyze_session.py logs/a.csv logs/b.csv` |
| Calibrate TCP | `python3.10 calibrate_ur5.py` |
| Verify cell mapping | `python3.10 verify_mapping.py` |
| Raw sensor readout | `python3.10 sensor_bridge.py` |

---

## Data Format

### CSV Session Log

Each session produces one CSV file in `logs/` with 29 columns at 20 Hz:

```
timestamp, datetime, ur5_point, ur5_pressing, ur5_done,
tcp_x, tcp_y, tcp_z,
fx, fy, fz, tx, ty, tz,
cell_1, cell_2, ..., cell_19
```

### Shared Sensor State (`/tmp/kywo_sensor.json`)

Written by `sensor.py`, read by visualisers and logger:

```json
{
  "cells": [0.0, 0.12, 0.0, 0.87, ...],   // 19 normalised values
  "timestamp": 1746548400.123,
  "connected": true
}
```

### Calibration (`calib.json`)

```json
{
  "x_mm": -3.0,
  "y_mm":  1.0,
  "z_mm":  0.0
}
```

---

## Output & Analysis

After each session `analyze_session.py` saves four figures to `plots/{session_name}/`:

```
plots/
└── my_experiment_session_20260506_170404/
    ├── overview.png     ← timeline heatmap + statistics
    ├── perpoint.png     ← bar charts per contact point
    ├── hexmaps.png      ← spatial hex maps per point
    └── force.png        ← TCP force analysis
```

The **overview** figure layout:

```
┌──────────────────────────────────────────┐
│  Timeline heatmap (cells × time)         │
├──────────┬────────────┬──────────────────┤
│ Peak per │  Hex maps  │  Correlation     │
│  event   │ (sample)   │  matrix          │
├──────────┴────────────┴──────────────────┤
│  Event duration histogram                │
└──────────────────────────────────────────┘
```

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `pyserial` | Serial communication with sensor hardware |
| `pandas` | CSV logging and analysis |
| `numpy` | Numerical processing and normalisation |
| `matplotlib` | All analysis plots |
| `pygame` | 2D real-time visualiser |
| `ur5_control` / `ur5_receive` | UR5 RTDE interface (robot vendor library) |
| SOFA v25.12.00 | 3D scene rendering |
| Python 3.10 | Required interpreter version |

---

*Generated for the Star-Nose Sensor project — UR5 + capacitive tactile sensor integration.*
