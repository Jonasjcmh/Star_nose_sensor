# Integration_2 — KYWO Sensor System

UR5 robot + 19-cell capacitive tactile sensor with real-time digital twin visualisation.

**Physical setup:** the sensor is fixed on the table; the robot end-effector carries a 40 mm × 6 mm cylindrical indenter that presses onto the sensor cells one by one.

---

## Table of Contents

1. [Environment Setup](#environment-setup)
2. [Quick Start](#quick-start)
3. [Running `main.py`](#running-mainpy)
4. [Demo System (no hardware needed)](#demo-system-no-hardware-needed)
5. [Standalone Visualizers](#standalone-visualizers)
6. [Post-Session Analysis](#post-session-analysis)
7. [File Reference](#file-reference)

---

## Environment Setup

All scripts run inside the `sofa-env` conda environment.

```bash
# Create the environment (first time only)
conda create -n sofa-env python=3.12 -y

# Install core packages
conda install -n sofa-env -c conda-forge pybullet numpy scipy -y
pip install --extra-index-url https://pypi.org/simple \
    meshcat pyserial pandas matplotlib ur-rtde
```

> **Every command below assumes you are in the `Integration_2/` directory:**
> ```bash
> cd /path/to/Star_nose_sensor/Integration_2
> ```

---

## Quick Start

```bash
# Hardware-free demo — opens a browser 3D view, replays a real session CSV
conda run -n sofa-env python demo_system.py

# Same but in a PyBullet OpenGL window
conda run -n sofa-env python demo_system.py --backend pybullet

# Full live session (robot + sensor + logging)
conda run -n sofa-env python main.py --no-sofa
```

---

## Running `main.py`

`main.py` is the system orchestrator. It starts the sensor, robot, logger and visualisers together.

### Common invocations

```bash
# Full system — sensor + robot + 2D HUD + logging
conda run -n sofa-env python main.py

# Skip the SOFA 3D window (faster startup, no SOFA required)
conda run -n sofa-env python main.py --no-sofa

# Add the browser-based digital twin alongside the live session
conda run -n sofa-env python main.py --no-sofa --robot-viz-meshcat

# Add the PyBullet digital twin
conda run -n sofa-env python main.py --no-sofa --robot-viz-pybullet

# Connect to URSim simulator instead of real robot
conda run -n sofa-env python main.py --no-sofa --sim --robot-viz-meshcat

# Custom CSV label + auto-stop after 120 s
conda run -n sofa-env python main.py --no-sofa --log-prefix texture_test --duration 120

# Auto-run analysis when the session ends
conda run -n sofa-env python main.py --no-sofa --analyze
```

### All flags

| Flag | Effect |
|------|--------|
| `--no-sofa` | Skip the SOFA 3D window |
| `--no-viz` | Skip the 2D pygame HUD |
| `--no-robot` | No robot — sensor + logging only |
| `--demo` | Simulated sensor data, no robot, no hardware |
| `--sim` | Connect to URSim at `localhost` (Docker) |
| `--sim-sensor` | Simulated sensor, real robot still moves |
| `--log-only` | Sensor + logging, no visualisers |
| `--viz-only` | 2D HUD only |
| `--sofa-only` | SOFA only |
| `--robot-viz` | Launch matplotlib 3D arm viewer |
| `--robot-viz-meshcat` | Launch browser-based Meshcat digital twin |
| `--robot-viz-pybullet` | Launch PyBullet OpenGL digital twin |
| `--analyze` | Run `analyze_session.py` when session ends |
| `--log-prefix NAME` | Set CSV filename prefix |
| `--duration SEC` | Auto-stop after N seconds |

---

## Demo System (no hardware needed)

`demo_system.py` replays a recorded session CSV with full IK-based arm animation — no robot or sensor connected.

```bash
# Default: Meshcat browser tab (auto-opens)
conda run -n sofa-env python demo_system.py

# PyBullet OpenGL window
conda run -n sofa-env python demo_system.py --backend pybullet

# Specific CSV file
conda run -n sofa-env python demo_system.py \
    --csv logs/dome_empty_tuesday_10_session_20260512_163300.csv

# 2× speed, loop forever
conda run -n sofa-env python demo_system.py --speed 2.0 --loop

# PyBullet, fast, looping
conda run -n sofa-env python demo_system.py --backend pybullet --speed 3.0 --loop
```

**What you see:**

- UR5 arm moving through the 19-point hexagonal scan pattern
- Indenter cylinder at the end-effector (40 mm × 6 mm)
- 19 sensor cells fixed on the table, colour-coded blue → red by activation
- Force arrows (Fx red, Fy green, Fz blue, resultant yellow) at the contact point
- TCP trail tracing the scan path
- Terminal progress bar with scan point, cell count, and force

> The first run pre-computes IK for all frames (~5–10 s). Subsequent replays are instant.

---

## Standalone Visualizers

These scripts run independently and connect to a live robot via RTDE, falling back to a sine-wave demo if the robot is unreachable (retry every 5 s).

### Meshcat — browser-based Three.js

```bash
# Real robot
conda run -n sofa-env python robot_viz_meshcat.py

# URSim simulator
conda run -n sofa-env python robot_viz_meshcat.py --sim

# Custom IP
conda run -n sofa-env python robot_viz_meshcat.py --ip 192.168.1.100
```

Opens `http://127.0.0.1:7000` in your browser. The scene contains:

- Full UR5 arm (links as cylinders, joints as spheres)
- Indenter cylinder updating in real time from FK
- 19 fixed sensor cells with heat-map colouring
- Force arrows at the indenter tip
- Fading TCP trail (last 80 points)

### PyBullet — OpenGL window

```bash
# Real robot
conda run -n sofa-env python robot_viz_pybullet.py

# URSim simulator
conda run -n sofa-env python robot_viz_pybullet.py --sim

# Headless (no window, for testing)
conda run -n sofa-env python robot_viz_pybullet.py --headless
```

Shows the same scene in an OpenGL window with:

- URDF-rendered arm with indenter geometry
- Coloured sensor cell spheres on the table
- Force arrows as debug lines
- Fading TCP trail

---

## Post-Session Analysis

```bash
# Analyse the most recent CSV in logs/
conda run -n sofa-env python analyze_session.py

# Analyse a specific file
conda run -n sofa-env python analyze_session.py \
    logs/texture_test_session_20260521_125139.csv

# Compare two sessions side-by-side
conda run -n sofa-env python analyze_session.py \
    logs/session_A.csv logs/session_B.csv

# Save all plots to plots/
conda run -n sofa-env python analyze_session.py logs/session.csv --save
```

Plots are saved to `plots/{session_name}/`:

| File | Contents |
|------|----------|
| `overview.png` | Timeline heatmap, peak bars, hex maps, correlation matrix |
| `perpoint.png` | Bar chart of all 19 cells per contact point |
| `hexmaps.png` | Spatial hex maps for every contact point |
| `force.png` | TCP force timeline and force-vs-sensor scatter |

---

## File Reference

| File | Purpose |
|------|---------|
| `main.py` | System orchestrator — wires all subsystems |
| `sensor.py` | Serial driver for the 252-cell capacitive sensor |
| `ur5_control.py` | RTDE robot controller + 21-point trajectory |
| `data_logger.py` | 20 Hz CSV session logger |
| `demo_system.py` | Hardware-free CSV replay with IK animation |
| `robot_viz_meshcat.py` | Browser digital twin (Meshcat / Three.js) |
| `robot_viz_pybullet.py` | OpenGL digital twin (PyBullet URDF) |
| `visualizer_2d.py` | Real-time pygame hex-grid HUD |
| `sofa_scene.py` | 3D SOFA Framework scene |
| `calibrate_ur5.py` | Interactive TCP calibration tool |
| `analyze_session.py` | Post-processing plots and statistics |
| `verify_mapping.py` | UR5 ↔ sensor cell mapping verifier |
| `sensor_bridge.py` | Lightweight serial sensor reader (diagnostics) |
| `calib.json` *(root)* | TCP calibration offsets (x/y/z mm) |
| `logs/` | Session CSVs (29 columns, 20 Hz) |
| `plots/` | Analysis figures (auto-created) |

### CSV columns

```
timestamp, datetime,
ur5_point, ur5_pressing, ur5_done,
tcp_x, tcp_y, tcp_z, fx, fy, fz, tx, ty, tz,
cell_1 … cell_19
```

### Sensor cell layout

```
        P1      P2      P3          y = +14 mm
    P4      P5      P6      P7      y =  +7 mm
  P8    P9    P10   P11   P12       y =   0 mm  ← centre (reference)
    P13     P14     P15     P16     y =  -7 mm
        P17     P18     P19         y = -14 mm

  x = -16 mm ──────────────── x = +16 mm
```
