"""
robot_viz_3d.py
Real-time 3D robot arm + sensor visualization.

Three modes:
  live    — reads joint angles from RTDE, sensor from /tmp/star_nose_sensor.json
  replay  — replays a session CSV log (TCP path + sensor, no joint data needed)
  demo    — animated test with no robot or sensor required

Usage:
  python robot_viz_3d.py               # live, real robot (UR_ROBOT_IP or default)
  python robot_viz_3d.py --sim         # live, URSim at localhost:30004
  python robot_viz_3d.py --ip 10.0.0.2
  python robot_viz_3d.py --replay test_dome_session_20260521_130512.csv
  python robot_viz_3d.py --replay session.csv --speed 4   # 4× playback
  python robot_viz_3d.py --demo
"""

import os
import sys
import time
import math
import glob
import json
import argparse
import threading
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import RegularPolygon, FancyArrowPatch
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable

matplotlib.use("MacOSX")   # native macOS interactive window

# ── Paths ──────────────────────────────────────────────────────────────────────
HERE     = Path(__file__).parent
LOGS_DIR = HERE / "logs"
SENSOR_SHARED_FILE = Path("/tmp/star_nose_sensor.json")

# ── UR5 CB3 forward kinematics ─────────────────────────────────────────────────
#   Standard DH parameters from UR5 technical specification
#   Each row: [a (m), d (m), alpha (rad)]
UR5_DH = np.array([
    [ 0.0,       0.089159,  np.pi / 2 ],   # joint 1 — shoulder
    [-0.42500,   0.0,       0.0       ],   # joint 2 — upper arm
    [-0.39225,   0.0,       0.0       ],   # joint 3 — forearm
    [ 0.0,       0.10915,   np.pi / 2 ],   # joint 4 — wrist 1
    [ 0.0,       0.09465,  -np.pi / 2 ],   # joint 5 — wrist 2
    [ 0.0,       0.0823,    0.0       ],   # joint 6 — wrist 3 / TCP
])

# Default "home" joint angles when no RTDE data is available
Q_HOME = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])


def _dh(theta, a, d, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,  -st * ca,  st * sa,  a * ct],
        [st,   ct * ca, -ct * sa,  a * st],
        [0.0,  sa,       ca,       d     ],
        [0.0,  0.0,      0.0,      1.0   ],
    ])


def ur5_fk(q):
    """Return (7,3) array of joint-frame origins in robot-base coordinates.

    Index 0 = base origin, indices 1-6 = joint 1..6 / TCP.
    """
    T   = np.eye(4)
    pts = [T[:3, 3].copy()]
    for (a, d, alpha), qi in zip(UR5_DH, q):
        T = T @ _dh(float(qi), a, d, alpha)
        pts.append(T[:3, 3].copy())
    return np.array(pts)    # (7, 3)


# ── Sensor / robot geometry ────────────────────────────────────────────────────
N = 19
CELL_COLS = [f"cell_{i+1}" for i in range(N)]

# Physical (x_mm, y_mm) of each sensor cell index 0..18
POINTS_MM = np.array([
    (-8, +14), ( 0, +14), (+8, +14),
    (-12, +7), (-4, +7),  (+4, +7),  (+12, +7),
    (-16,  0), (-8,  0),  ( 0,  0),  (+8,  0),  (+16, 0),
    (-12, -7), (-4, -7),  (+4, -7),  (+12, -7),
    (-8, -14), ( 0, -14), (+8, -14),
], dtype=float)

# UR5 scan grid: target positions relative to P10-center (m)
SCAN_GRID_XY = {
     1: (-0.008,  +0.014),  2: ( 0.000, +0.014),  3: (+0.008, +0.014),
     4: (-0.012,  +0.007),  5: (-0.004, +0.007),  6: (+0.004, +0.007),  7: (+0.012, +0.007),
     8: (-0.016,   0.000),  9: (-0.008,  0.000), 10: ( 0.000,  0.000), 11: (+0.008,  0.000), 12: (+0.016,  0.000),
    13: (-0.012,  -0.007), 14: (-0.004, -0.007), 15: (+0.004, -0.007), 16: (+0.012, -0.007),
    17: (-0.008,  -0.014), 18: ( 0.000, -0.014), 19: (+0.008, -0.014),
}

# Reference TCP at P10 (center of scan grid), from ur5_control.py
REF_X, REF_Y, REF_Z = -0.03741, -0.49886, 0.06054

CMAP = LinearSegmentedColormap.from_list("star_nose", [
    "#2ab5a0", "#33e666", "#ffe619", "#ff7300", "#dc0000"
])

# ── Shared mutable state (data thread → render thread) ────────────────────────
_state = {
    "q":        Q_HOME.copy(),   # joint angles (rad)
    "tcp":      np.array([REF_X, REF_Y, REF_Z]),
    "cells":    [0.0] * N,
    "pressing": False,
    "point":    None,
    "fz":       0.0,
    "trail":    [],              # list of (x,y,z) TCP positions
    "t":        0.0,
    "connected": False,   # True when RTDE is live
}
_state_lock = threading.Lock()
_running    = [True]


# ── Data sources ───────────────────────────────────────────────────────────────
def _read_sensor_shared():
    """Read sensor values from the shared file written by sensor.py."""
    try:
        if SENSOR_SHARED_FILE.exists():
            age = time.time() - SENSOR_SHARED_FILE.stat().st_mtime
            if age < 1.5:
                with open(SENSOR_SHARED_FILE) as f:
                    d = json.load(f)
                if d.get("ready"):
                    return d["values"]
    except Exception:
        pass
    return None


def _live_thread(ip):
    """RTDE live mode with automatic fallback to demo animation.

    Tries to connect to the robot. If unavailable, runs synthetic motion
    and retries the connection every RETRY_INTERVAL seconds automatically.
    """
    trail_max      = 300
    RETRY_INTERVAL = 5.0
    t0             = time.time()

    def _try_connect():
        try:
            import rtde_receive
            r = rtde_receive.RTDEReceiveInterface(ip)
            print("[viz3d] RTDE connected!")
            with _state_lock:
                _state["connected"] = True
            return r
        except Exception as e:
            print(f"[viz3d] Could not connect to {ip}: {e}")
            with _state_lock:
                _state["connected"] = False
            return None

    print(f"[viz3d] Connecting to robot at {ip} ...")
    r          = _try_connect()
    last_retry = time.time()

    if r is None:
        print(f"[viz3d] No robot — running demo animation "
              f"(retrying every {RETRY_INTERVAL:.0f}s)")

    while _running[0]:
        if r is not None:
            # ── Live RTDE path ──────────────────────────────────────────────
            try:
                q        = list(r.getActualQ())
                tcp      = r.getActualTCPPose()[:3]
                ft       = r.getActualTCPForce()
                cells    = _read_sensor_shared() or [0.0] * N
                pressing = any(v > 0.1 for v in cells)
                with _state_lock:
                    _state["q"]         = q
                    _state["tcp"]       = np.array(tcp)
                    _state["cells"]     = cells
                    _state["pressing"]  = pressing
                    _state["fz"]        = abs(ft[2]) if ft else 0.0
                    _state["connected"] = True
                    trail = _state["trail"]
                    trail.append(np.array(tcp))
                    if len(trail) > trail_max:
                        trail.pop(0)
            except Exception as e:
                print(f"[viz3d] RTDE read error: {e} — falling back to demo")
                r = None
                with _state_lock:
                    _state["connected"] = False
                last_retry = time.time()
        else:
            # ── Demo fallback path ──────────────────────────────────────────
            t = time.time() - t0
            q = np.array([
                0.3 * math.sin(0.4 * t),
                -np.pi / 2 + 0.3 * math.sin(0.3 * t + 1.0),
                0.4 * math.sin(0.5 * t + 0.5),
                -np.pi / 2 + 0.2 * math.sin(0.7 * t),
                0.3 * math.sin(0.6 * t + 2.0),
                0.1 * math.sin(t),
            ])
            pts   = ur5_fk(q)
            tcp   = pts[-1]
            cells = [max(0.0, min(1.0, 0.4 * math.sin(t + i * 0.4) *
                                   math.sin(t * 0.7 + i * 0.2) + 0.2))
                     for i in range(N)]
            with _state_lock:
                _state["q"]         = q
                _state["tcp"]       = tcp
                _state["cells"]     = cells
                _state["connected"] = False
                _state["t"]         = t
                trail = _state["trail"]
                trail.append(tcp.copy())
                if len(trail) > trail_max:
                    trail.pop(0)

            # Periodic reconnect attempt
            if time.time() - last_retry >= RETRY_INTERVAL:
                print(f"[viz3d] Retrying connection to {ip} ...")
                r = _try_connect()
                last_retry = time.time()
                if r is None:
                    print(f"[viz3d] Still unavailable — "
                          f"next retry in {RETRY_INTERVAL:.0f}s")

        time.sleep(0.05)


def _replay_thread(csv_path, speed):
    df = pd.read_csv(csv_path)
    for c in CELL_COLS:
        df[c] = pd.to_numeric(df[c], errors="coerce").fillna(0.0)
    df["ur5_pressing"] = pd.to_numeric(
        df.get("ur5_pressing", 0), errors="coerce").fillna(0).astype(int)
    df["ur5_point"] = pd.to_numeric(
        df.get("ur5_point", float("nan")), errors="coerce")

    dt_csv = df["timestamp"].diff().median()
    dt_play = max(0.005, dt_csv / speed) if not math.isnan(dt_csv) else 0.05

    trail_max = 300
    t0 = time.time()

    for _, row in df.iterrows():
        if not _running[0]:
            break
        cells    = [float(row[c]) for c in CELL_COLS]
        pressing = int(row["ur5_pressing"]) == 1
        pt       = row.get("ur5_point")
        pt       = int(pt) if pd.notna(pt) else None

        has_tcp = all(c in row for c in ("tcp_x", "tcp_y", "tcp_z"))
        tcp = (np.array([row["tcp_x"], row["tcp_y"], row["tcp_z"]])
               if has_tcp else _state["tcp"])

        with _state_lock:
            _state["tcp"]      = tcp
            _state["cells"]    = cells
            _state["pressing"] = pressing
            _state["point"]    = pt
            _state["fz"]       = abs(float(row["fz"])) if "fz" in row else 0.0
            _state["t"]        = float(row["timestamp"]) - float(df["timestamp"].iloc[0])
            trail = _state["trail"]
            trail.append(tcp.copy())
            if len(trail) > trail_max:
                trail.pop(0)

        time.sleep(dt_play)

    print("[viz3d] Replay finished")


def _demo_thread():
    """Synthetic sinusoidal motion for testing with no robot / sensor."""
    t0 = time.time()
    trail_max = 300
    while _running[0]:
        t = time.time() - t0
        q = np.array([
            0.3 * math.sin(0.4 * t),
            -np.pi / 2 + 0.3 * math.sin(0.3 * t + 1.0),
            0.4 * math.sin(0.5 * t + 0.5),
            -np.pi / 2 + 0.2 * math.sin(0.7 * t),
            0.3 * math.sin(0.6 * t + 2.0),
            0.1 * math.sin(t),
        ])
        pts  = ur5_fk(q)
        tcp  = pts[-1]
        cells = [max(0.0, min(1.0, 0.4 * math.sin(t + i * 0.4) *
                               math.sin(t * 0.7 + i * 0.2) + 0.2))
                 for i in range(N)]
        with _state_lock:
            _state["q"]     = q
            _state["tcp"]   = tcp
            _state["cells"] = cells
            _state["t"]     = t
            trail = _state["trail"]
            trail.append(tcp.copy())
            if len(trail) > trail_max:
                trail.pop(0)
        time.sleep(0.05)


# ── Figure setup ───────────────────────────────────────────────────────────────
def _build_scan_grid():
    """Compute 3D positions of the 19 scan target points on the surface plane."""
    pts = {}
    for pt, (dx, dy) in SCAN_GRID_XY.items():
        pts[pt] = np.array([REF_X + dx, REF_Y + dy, REF_Z])
    return pts


# Link colours: base→shoulder, shoulder→elbow, forearm, wrist1, wrist2, tool
LINK_COLORS = ["#888888", "#4488cc", "#44aacc", "#44ccaa", "#ffaa33", "#ff5533"]
JOINT_COLOR = "#ffffff"


def main():
    args    = _parse_args()
    mode    = args.mode
    scan_grid = _build_scan_grid()

    # ── Launch data thread ───────────────────────────────────────────────────
    if mode == "live":
        ip = args.ip or os.environ.get("UR_ROBOT_IP", "127.0.0.1" if args.sim else "177.22.22.2")
        t  = threading.Thread(target=_live_thread, args=(ip,), daemon=True)
        t.start()
        title_suffix = f"live @ {ip}"

    elif mode == "replay":
        csvs = sorted(glob.glob(str(LOGS_DIR / "*.csv")))
        if args.file:
            matches = [f for f in csvs if args.file in os.path.basename(f)]
            csv_path = matches[-1] if matches else csvs[-1]
        else:
            csv_path = csvs[-1] if csvs else None
        if not csv_path:
            print(f"[viz3d] No CSV found in {LOGS_DIR}")
            sys.exit(1)
        print(f"[viz3d] Replaying: {os.path.basename(csv_path)}  speed={args.speed}×")
        t = threading.Thread(target=_replay_thread,
                             args=(csv_path, args.speed), daemon=True)
        t.start()
        title_suffix = f"replay: {os.path.basename(csv_path)}"

    else:  # demo
        t = threading.Thread(target=_demo_thread, daemon=True)
        t.start()
        title_suffix = "demo (synthetic)"

    # ── Build figure ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 9), facecolor="#0d0d14")
    fig.canvas.manager.set_window_title("KYWO — Robot 3D Visualizer")
    gs  = gridspec.GridSpec(
        2, 2, figure=fig,
        width_ratios=[1.6, 1],
        height_ratios=[1.3, 1],
        hspace=0.08, wspace=0.08,
        left=0.03, right=0.97, top=0.94, bottom=0.04,
    )

    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_hex  = fig.add_subplot(gs[0, 1])
    ax_info = fig.add_subplot(gs[1, 1])

    for ax in [ax_hex, ax_info]:
        ax.set_facecolor("#111120")
        for sp in ax.spines.values():
            sp.set_edgecolor("#333344")

    ax3d.set_facecolor("#0d0d14")
    ax3d.xaxis.pane.fill = False
    ax3d.yaxis.pane.fill = False
    ax3d.zaxis.pane.fill = False
    ax3d.xaxis.pane.set_edgecolor("#222233")
    ax3d.yaxis.pane.set_edgecolor("#222233")
    ax3d.zaxis.pane.set_edgecolor("#222233")
    ax3d.grid(True, color="#1a1a2a", linewidth=0.4)
    ax3d.set_xlabel("X (m)", fontsize=7, color="#666688")
    ax3d.set_ylabel("Y (m)", fontsize=7, color="#666688")
    ax3d.set_zlabel("Z (m)", fontsize=7, color="#666688")
    ax3d.tick_params(colors="#444466", labelsize=6)
    ax3d.set_xlim(-0.40, 0.40)
    ax3d.set_ylim(-0.75, 0.30)
    ax3d.set_zlim(0.00, 1.10)
    ax3d.set_title(f"UR5 — {title_suffix}",
                   color="#ccccdd", fontsize=10, pad=8)

    # ── Static elements: scan grid ───────────────────────────────────────────
    gx = [p[0] for p in scan_grid.values()]
    gy = [p[1] for p in scan_grid.values()]
    gz = [p[2] for p in scan_grid.values()]
    ax3d.scatter(gx, gy, gz, s=20, c="#334455", alpha=0.6, zorder=1)
    for pt, pos in scan_grid.items():
        ax3d.text(pos[0], pos[1], pos[2] + 0.002,
                  f"P{pt}", fontsize=4.5, color="#445566",
                  ha="center", va="bottom")

    # Surface plane (the tactile scanning table)
    sx = np.linspace(-0.10, 0.10, 2)
    sy = np.linspace(-0.55, -0.45, 2)
    SX, SY = np.meshgrid(sx, sy)
    ax3d.plot_surface(SX, SY, np.full_like(SX, REF_Z - 0.001),
                      alpha=0.06, color="#2266aa", zorder=0)

    # ── Hex map setup ────────────────────────────────────────────────────────
    ax_hex.set_aspect("equal")
    ax_hex.axis("off")
    ax_hex.set_title("Sensor pressure map", fontsize=9,
                     color="#aaaacc", pad=4)
    hex_patches = []
    hex_texts   = []
    for xmm, ymm in POINTS_MM:
        h = RegularPolygon((xmm, ymm), numVertices=6, radius=4.3,
                           facecolor=CMAP(0.0), edgecolor="#333344",
                           linewidth=0.6)
        ax_hex.add_patch(h)
        hex_patches.append(h)
        t = ax_hex.text(xmm, ymm, "", ha="center", va="center",
                        fontsize=5, color="white")
        hex_texts.append(t)
    ax_hex.set_xlim(-22, 22)
    ax_hex.set_ylim(-20, 20)

    sm = ScalarMappable(cmap=CMAP, norm=Normalize(0, 1))
    sm.set_array([])
    cb = fig.colorbar(sm, ax=ax_hex, shrink=0.5, pad=0.02)
    cb.ax.yaxis.label.set_color("#aaaacc")
    cb.ax.tick_params(colors="#aaaacc", labelsize=6)

    ax_info.axis("off")
    status_text = ax_info.text(0.05, 0.98, "○ SIMULATION",
                               transform=ax_info.transAxes,
                               fontsize=9, color="#ffcc33", va="top",
                               fontfamily="monospace", fontweight="bold")
    info_text = ax_info.text(0.05, 0.86, "", transform=ax_info.transAxes,
                             fontsize=9, color="#ccccdd", va="top",
                             fontfamily="monospace")
    fig.suptitle("KYWO — UR5 + Tactile Sensor",
                 color="#ddddee", fontsize=11, y=0.98)

    # ── Dynamic artists (robot arm) ──────────────────────────────────────────
    arm_lines = [ax3d.plot([], [], [], lw=3.5, color=c, solid_capstyle="round")[0]
                 for c in LINK_COLORS]
    joint_scat = ax3d.scatter([], [], [], s=55, c=JOINT_COLOR,
                              edgecolors="#000000", linewidth=0.8, zorder=5)
    tcp_scat   = ax3d.scatter([], [], [], s=120, c="#ff4444",
                              marker="*", zorder=6)
    trail_line = ax3d.plot([], [], [], lw=0.8, color="#33aaff",
                           alpha=0.5, linestyle="--")[0]

    # Base frame indicator
    ax3d.scatter([0], [0], [0], s=60, c="#ffff00", marker="^", zorder=7)
    ax3d.text(0, 0, 0.02, "base", fontsize=5.5, color="#888866")

    # ── Render loop ──────────────────────────────────────────────────────────
    plt.ion()
    plt.show(block=False)

    print("[viz3d] Rendering — close window to quit")

    while _running[0]:
        # Snapshot state
        with _state_lock:
            q         = np.array(_state["q"])
            tcp       = _state["tcp"].copy()
            cells     = list(_state["cells"])
            pressing  = _state["pressing"]
            point     = _state["point"]
            fz        = _state["fz"]
            elapsed   = _state["t"]
            trail     = list(_state["trail"])
            connected = _state.get("connected", False)

        # ── 3D arm ──────────────────────────────────────────────────────────
        pts = ur5_fk(q)    # (7, 3) — base + 6 joints

        for i, line in enumerate(arm_lines):
            p0, p1 = pts[i], pts[i + 1]
            line.set_data_3d([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]])

        jx, jy, jz = pts[1:, 0], pts[1:, 1], pts[1:, 2]
        joint_scat._offsets3d = (jx, jy, jz)
        tcp_scat._offsets3d   = ([tcp[0]], [tcp[1]], [tcp[2]])

        if len(trail) > 1:
            tr  = np.array(trail)
            trail_line.set_data_3d(tr[:, 0], tr[:, 1], tr[:, 2])
        else:
            trail_line.set_data_3d([], [], [])

        # ── Hex map ──────────────────────────────────────────────────────────
        vmax = max(max(cells), 1e-6)
        for i, (patch, txt) in enumerate(zip(hex_patches, hex_texts)):
            v   = float(np.clip(cells[i], 0.0, 1.0))
            patch.set_facecolor(CMAP(v))
            lw  = 2.0 if i == (point - 1 if point else -1) else 0.6
            ec  = "#ff4444" if (pressing and i == (point - 1 if point else -1)) else "#333344"
            patch.set_linewidth(lw)
            patch.set_edgecolor(ec)
            txt.set_text(f"{v:.2f}" if v > 0.02 else "")
            txt.set_color("white" if v > 0.45 else "#888899")

        # ── Info panel ───────────────────────────────────────────────────────
        n_act   = sum(1 for v in cells if v > 0.05)
        max_v   = max(cells) if cells else 0
        q_deg   = np.degrees(q)
        pressing_str = "▼ PRESSING" if pressing else "moving  "
        point_str    = f"P{point}" if point else "—"

        info = (
            f"t = {elapsed:7.1f} s\n\n"
            f"UR5 target  : {point_str}\n"
            f"Status      : {pressing_str}\n"
            f"|Fz|         : {fz:6.2f} N\n\n"
            f"Active cells: {n_act:2d} / 19\n"
            f"Peak pressure: {max_v:.3f}\n\n"
            f"TCP  X  {tcp[0]:+.4f} m\n"
            f"TCP  Y  {tcp[1]:+.4f} m\n"
            f"TCP  Z  {tcp[2]:+.4f} m\n\n"
            f"Joints (°)\n"
            + "\n".join(f"  J{i+1}  {a:+7.1f}" for i, a in enumerate(q_deg))
        )
        if connected:
            status_text.set_text("● ROBOT LIVE")
            status_text.set_color("#33ff88")
        else:
            status_text.set_text("○ SIMULATION")
            status_text.set_color("#ffcc33")
        info_text.set_text(info)

        try:
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.07)    # ~14 fps
        except Exception:
            break

        if not plt.get_fignums():
            break

    _running[0] = False
    print("[viz3d] Closed")


# ── CLI ────────────────────────────────────────────────────────────────────────
def _parse_args():
    p = argparse.ArgumentParser()
    g = p.add_mutually_exclusive_group()
    g.add_argument("--sim",    action="store_true",
                   help="Live mode — connect to URSim at localhost")
    g.add_argument("--replay", metavar="FILE",
                   help="Replay mode — session CSV (partial name ok)")
    g.add_argument("--demo",   action="store_true",
                   help="Demo mode — synthetic animation, no robot/sensor needed")

    p.add_argument("--ip",    default=None,
                   help="Custom robot IP (overrides UR_ROBOT_IP env var)")
    p.add_argument("--speed", type=float, default=1.0,
                   help="Replay speed multiplier (default 1.0)")

    args = p.parse_args()
    args.file = args.replay     # store the CSV filename for later
    if args.demo:
        args.mode = "demo"
    elif args.replay:
        args.mode = "replay"
    else:
        args.mode = "live"
    return args


if __name__ == "__main__":
    try:
        import pandas, numpy, matplotlib
    except ImportError:
        os.system(f"{sys.executable} -m pip install pandas numpy matplotlib")
    main()
