"""
demo_system.py
Self-contained demonstration of the complete KYWO sensing system.

Replays a real session CSV — no robot, no sensor hardware needed.

What it shows
─────────────
  • UR5 arm moving through the 19-point hexagonal scan pattern
  • 19-cell tactile sensor heat-map animating in real time at the TCP
  • Force arrows (Fx, Fy, Fz, resultant) at the TCP
  • TCP trail tracing the scan path
  • Rich terminal status showing scan progress

The arm pose is reconstructed from the logged TCP positions using
scipy IK (pre-computed once at load time), so the 3D animation is
faithful to the actual robot motion.

Usage
─────
  python demo_system.py                   # Meshcat browser (default)
  python demo_system.py --backend pybullet
  python demo_system.py --csv logs/dome_empty_tuesday_10_session_20260512_163300.csv
  python demo_system.py --speed 2.0       # 2× playback
  python demo_system.py --loop            # repeat forever
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PI = math.pi
N  = 19

# ── UR5 DH ─────────────────────────────────────────────────────────────────────
UR5_DH = [
    (0.0,     0.089159,  PI / 2),
    (-0.425,  0.0,       0.0   ),
    (-0.39225,0.0,       0.0   ),
    (0.0,     0.10915,   PI / 2),
    (0.0,     0.09465,  -PI / 2),
    (0.0,     0.0823,    0.0   ),
]
Q_HOME = np.array([0.0, -PI/2, 0.0, -PI/2, 0.0, 0.0])


def _dh(theta, a, d, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,  -st*ca,  st*sa,  a*ct],
        [st,   ct*ca, -ct*sa,  a*st],
        [0.0,  sa,     ca,     d   ],
        [0.0,  0.0,    0.0,    1.0 ],
    ])


def ur5_fk(q):
    T   = np.eye(4)
    pts = [T[:3, 3].copy()]
    for (a, d, alpha), qi in zip(UR5_DH, q):
        T = T @ _dh(float(qi), a, d, alpha)
        pts.append(T[:3, 3].copy())
    return np.array(pts)


def ur5_ik(target_xyz, q_init):
    """Numerical IK using scipy — minimises FK TCP error."""
    from scipy.optimize import minimize

    def cost(q):
        return float(np.sum((ur5_fk(q)[-1] - target_xyz) ** 2))

    bounds = [(-2 * PI, 2 * PI)] * 6
    res = minimize(cost, q_init, method="L-BFGS-B", bounds=bounds,
                   options={"maxiter": 300, "ftol": 1e-9})
    return res.x


# ── Dataset ─────────────────────────────────────────────────────────────────────
LOGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
CELL_COLS = [f"cell_{i+1}" for i in range(N)]

# Priority order for auto-selection
_PREFERRED = [
    "dome_empty_tuesday_10_session_20260512_163300.csv",
    "dome_empty_monday_2_session_20260511_134351.csv",
    "ursim_19pts_session_20260525_225720.csv",
]


def _auto_pick_csv():
    for name in _PREFERRED:
        path = os.path.join(LOGS_DIR, name)
        if os.path.isfile(path):
            return path
    # Fall back to any session CSV
    import glob
    files = sorted(glob.glob(os.path.join(LOGS_DIR, "*_session_*.csv")))
    if files:
        return files[-1]   # most recent
    raise FileNotFoundError(f"No session CSV found in {LOGS_DIR}")


def load_session(csv_path):
    """Load CSV and precompute joint angles via IK."""
    import pandas as pd

    print(f"[demo] Loading {os.path.basename(csv_path)} …")
    df = pd.read_csv(csv_path)
    print(f"[demo] {len(df)} rows, columns: {list(df.columns[:6])} …")

    # Ensure cell columns exist
    for col in CELL_COLS:
        if col not in df.columns:
            df[col] = 0.0

    # Extract TCP positions
    tcp_xyz = df[["tcp_x", "tcp_y", "tcp_z"]].to_numpy()
    has_tcp = (np.abs(tcp_xyz).sum(axis=1) > 0.001)  # rows with real TCP data

    # Force / torque
    ft_cols = []
    for c in ["fx", "fy", "fz"]:
        ft_cols.append(df[c].to_numpy() if c in df.columns else np.zeros(len(df)))
    ft_xyz = np.column_stack(ft_cols)

    # Sensor cells
    cells_all = df[CELL_COLS].to_numpy()

    # Timestamps (relative seconds from start)
    ts = df["timestamp"].to_numpy(dtype=float)
    t_rel = ts - ts[0]

    # ── Pre-compute joint angles using IK ────────────────────────────────────
    print("[demo] Computing IK for all frames (this may take ~10s) …")
    q_traj = np.zeros((len(df), 6))
    q_prev = Q_HOME.copy()

    for i in range(len(df)):
        if has_tcp[i] and np.linalg.norm(tcp_xyz[i]) > 0.01:
            q = ur5_ik(tcp_xyz[i], q_prev)
            q_traj[i] = q
            q_prev    = q
        else:
            # No TCP data — hold previous pose
            q_traj[i] = q_prev

    print(f"[demo] IK done. Duration: {t_rel[-1]:.1f}s, frames: {len(df)}")

    # ur5_point column (which scan point, if available)
    pt_col = df["ur5_point"].fillna("").astype(str).tolist()
    pr_col = df["ur5_pressing"].fillna(0).astype(int).tolist()

    return {
        "t_rel":   t_rel,
        "q_traj":  q_traj,
        "tcp_xyz": tcp_xyz,
        "ft_xyz":  ft_xyz,
        "cells":   cells_all,
        "point":   pt_col,
        "pressing":pr_col,
        "n":       len(df),
    }


# ── Shared replay state ──────────────────────────────────────────────────────────
_rstate = {
    "q":      Q_HOME.tolist(),
    "cells":  [0.0] * N,
    "ft":     [0.0, 0.0, 0.0],
    "fz":     0.0,
    "trail":  [],
    "point":  "",
    "pressing": False,
    "progress": 0.0,
    "done":   False,
}
_rlock   = threading.Lock()
_running = [True]
TRAIL_MAX = 150


def replay_thread(data, speed=1.0, loop=False):
    """Feed replay data into _rstate at the original recorded rate."""
    while _running[0]:
        trail = []
        n     = data["n"]
        t0    = time.time()
        dur   = float(data["t_rel"][-1])

        for i in range(n):
            if not _running[0]:
                return

            t_target = data["t_rel"][i] / speed
            elapsed  = time.time() - t0
            wait     = t_target - elapsed
            if wait > 0:
                time.sleep(wait)

            tcp = data["tcp_xyz"][i]
            if np.linalg.norm(tcp) > 0.01:
                trail.append(tcp.copy())
                if len(trail) > TRAIL_MAX:
                    trail.pop(0)

            cells = [max(0.0, min(1.0, float(v)))
                     for v in data["cells"][i]]

            with _rlock:
                _rstate["q"]        = data["q_traj"][i].tolist()
                _rstate["cells"]    = cells
                _rstate["ft"]       = data["ft_xyz"][i].tolist()
                _rstate["fz"]       = abs(float(data["ft_xyz"][i][2]))
                _rstate["trail"]    = list(trail)
                _rstate["point"]    = data["point"][i]
                _rstate["pressing"] = bool(data["pressing"][i])
                _rstate["progress"] = i / max(n - 1, 1)

        with _rlock:
            _rstate["done"] = not loop

        if not loop:
            break

        print("\n[demo] Looping …")

    with _rlock:
        _rstate["done"] = True
    print("\n[demo] Replay finished")


# ── Meshcat backend ─────────────────────────────────────────────────────────────
def run_meshcat(speed=1.0, loop=False, csv_path=None):
    try:
        import meshcat
        import meshcat.geometry as g
        import meshcat.transformations as tf_mod
    except ImportError:
        print("meshcat not found — install: pip install meshcat")
        sys.exit(1)

    from robot_viz_meshcat import (
        ur5_fk, ur5_fk_full, _seg_tf, _trans, _cell_color, _setup,
        TRAIL_MAX as _TM,
        ARROW_COLS, FORCE_SCALE, _draw_arrow, _update,
    )

    csv_path = csv_path or _auto_pick_csv()
    data     = load_session(csv_path)

    vis = meshcat.Visualizer()
    vis.open()
    print(f"\n[demo] Browser → {vis.url()}")
    _setup(vis)

    # Start replay
    t = threading.Thread(target=replay_thread,
                         args=(data, speed, loop), daemon=True)
    t.start()

    print("[demo] Replay running — press Ctrl+C to stop\n")

    bar_width = 40
    try:
        while _running[0]:
            with _rlock:
                q       = list(_rstate["q"])
                cells   = list(_rstate["cells"])
                ft      = list(_rstate["ft"])
                fz      = _rstate["fz"]
                trail   = list(_rstate["trail"])
                pt      = _rstate["point"]
                pressing= _rstate["pressing"]
                prog    = _rstate["progress"]
                done    = _rstate["done"]

            _update(vis, q, cells, ft, trail, connected=False)

            # Terminal progress bar
            filled  = int(prog * bar_width)
            bar     = "█" * filled + "░" * (bar_width - filled)
            press_s = "▼ PRESSING" if pressing else "          "
            active  = sum(1 for v in cells if v > 0.05)
            fr_mag  = float(np.linalg.norm(ft))
            print(f"\r[demo] [{bar}] {prog*100:5.1f}%  "
                  f"P{pt or '-':>3}  {press_s}  "
                  f"Fz={fz:+5.2f}N |F|={fr_mag:5.2f}N  "
                  f"cells={active:2d}/19",
                  end="", flush=True)

            if done:
                break
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[demo] Stopped")
    finally:
        _running[0] = False

    print("\n\n" + "="*60)
    print("  KYWO System Demo — Summary")
    print("="*60)
    print(f"  Dataset   : {os.path.basename(csv_path)}")
    print(f"  Frames    : {data['n']}")
    print(f"  Duration  : {data['t_rel'][-1]:.1f}s")
    print(f"  Max Fz    : {data['ft_xyz'][:,2].min():.2f} N (negative = pressing)")
    print(f"  Max cell  : {data['cells'].max():.3f}")
    print("="*60)
    print("  Features demonstrated:")
    print("    ✓ UR5 arm kinematic animation (IK from logged TCP)")
    print("    ✓ 19-cell tactile sensor heat-map at TCP")
    print("    ✓ Force arrows: Fx (red), Fy (green), Fz (blue), |F| (yellow)")
    print("    ✓ TCP trail tracing the hexagonal scan pattern")
    print("    ✓ Meshcat browser viewer — interactive 3D (orbit/zoom/pan)")
    print("="*60)


# ── PyBullet backend ────────────────────────────────────────────────────────────
def run_pybullet(speed=1.0, loop=False, csv_path=None):
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print("pybullet not found — install: conda install -c conda-forge pybullet")
        sys.exit(1)

    import tempfile
    from robot_viz_pybullet import (
        UR5_URDF, ur5_fk_full,
        CELL_WORLD_POS, N as _N,
    )

    csv_path = csv_path or _auto_pick_csv()
    data     = load_session(csv_path)

    # Write URDF
    urdf_path = os.path.join(tempfile.gettempdir(), "ur5_twin_demo.urdf")
    with open(urdf_path, "w") as f:
        f.write(UR5_URDF)

    c = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(1.4, 45, -25, [0, 0, 0.45])
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

    p.loadURDF("plane.urdf")
    rid = p.loadURDF(urdf_path, useFixedBase=True)
    n_j = p.getNumJoints(rid)
    rev = [j for j in range(n_j)
           if p.getJointInfo(rid, j)[2] == p.JOINT_REVOLUTE]

    # HUD
    _ST  = p.addUserDebugText("REPLAY", [0, -0.6, 0.88],
                               textColorRGB=[0.3, 0.8, 1.0], textSize=1.4)
    _FZT = p.addUserDebugText("Fz = 0.00 N", [0, -0.6, 0.80],
                               textColorRGB=[0.7, 0.9, 1.0], textSize=1.2)

    # Fixed sensor cell visual bodies
    _cell_mb = []
    for _i in range(_N):
        _vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.005,
                                  rgbaColor=[0.07, 0.13, 0.20, 1.0])
        _cell_mb.append(p.createMultiBody(baseMass=0, baseVisualShapeIndex=_vs,
                                          basePosition=CELL_WORLD_POS[_i].tolist()))
    # Sensor platform
    _pvs = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.020, 0.018, 0.002],
                               rgbaColor=[0.10, 0.16, 0.23, 1.0])
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=_pvs,
                      basePosition=(CELL_WORLD_POS.mean(axis=0) + [0,0,-0.003]).tolist())

    _FX = p.addUserDebugLine([0,0,0], [0,0,0], [1,0.2,0.2], lineWidth=3)
    _FY = p.addUserDebugLine([0,0,0], [0,0,0], [0.2,1,0.2], lineWidth=3)
    _FZ = p.addUserDebugLine([0,0,0], [0,0,0], [0.3,0.6,1], lineWidth=3)
    _FR = p.addUserDebugLine([0,0,0], [0,0,0], [1,0.9,0.1], lineWidth=5)
    FORCE_SCALE = 0.015

    trail_pts   = []
    trail_lines = []

    t = threading.Thread(target=replay_thread,
                         args=(data, speed, loop), daemon=True)
    t.start()

    print("[demo] PyBullet running — close window or Ctrl+C to stop\n")
    bar_width = 40

    try:
        while _running[0]:
            try:
                p.getConnectionInfo(c)
            except Exception:
                break

            with _rlock:
                q       = list(_rstate["q"])
                cells   = list(_rstate["cells"])
                ft      = list(_rstate["ft"])
                fz      = _rstate["fz"]
                pt      = _rstate["point"]
                pressing= _rstate["pressing"]
                prog    = _rstate["progress"]
                done    = _rstate["done"]

            for ji, qv in enumerate(q[:len(rev)]):
                p.resetJointState(rid, rev[ji], qv)

            Ts  = ur5_fk_full(q)
            tcp = Ts[6][:3, 3]

            # Trail
            trail_pts.append(tcp.copy())
            if len(trail_pts) > 200:
                trail_pts.pop(0)
            for lid in trail_lines:
                p.removeUserDebugItem(lid)
            trail_lines.clear()
            nn = len(trail_pts)
            for k in range(nn - 1):
                a_v = k / nn
                col = [0.15 + 0.85 * a_v, 0.4 * a_v, 1 - 0.7 * a_v]
                trail_lines.append(p.addUserDebugLine(
                    trail_pts[k].tolist(), trail_pts[k+1].tolist(),
                    col, lineWidth=2))

            # Fixed sensor cells — update colour by activation
            for i in range(_N):
                v   = cells[i] if i < len(cells) else 0.0
                r_c = min(1.0, v * 1.5)
                g_c = max(0.0, 1.0 - v * 1.5)
                p.changeVisualShape(_cell_mb[i], -1, rgbaColor=[r_c, g_c, 0.1, 1.0])

            # Force arrows
            fx_v, fy_v, fz_v = (ft + [0,0,0])[:3]
            tcp_l = tcp.tolist()
            p.addUserDebugLine(tcp_l,
                (tcp + np.array([1,0,0])*fx_v*FORCE_SCALE).tolist(),
                [1,0.2,0.2], lineWidth=3, replaceItemUniqueId=_FX)
            p.addUserDebugLine(tcp_l,
                (tcp + np.array([0,1,0])*fy_v*FORCE_SCALE).tolist(),
                [0.2,1,0.2], lineWidth=3, replaceItemUniqueId=_FY)
            p.addUserDebugLine(tcp_l,
                (tcp + np.array([0,0,1])*fz_v*FORCE_SCALE).tolist(),
                [0.3,0.6,1], lineWidth=3, replaceItemUniqueId=_FZ)
            fr_v  = np.array([fx_v, fy_v, fz_v])
            fr_mag= float(np.linalg.norm(fr_v))
            if fr_mag > 0.3:
                p.addUserDebugLine(tcp_l,
                    (tcp + fr_v*FORCE_SCALE).tolist(),
                    [1,0.9,0.1], lineWidth=5, replaceItemUniqueId=_FR)
            else:
                p.addUserDebugLine(tcp_l, tcp_l,
                    [1,0.9,0.1], lineWidth=1, replaceItemUniqueId=_FR)

            # HUD
            press_s = "▼ PRESSING" if pressing else f"P{pt or '-':>3}     "
            p.addUserDebugText(f"REPLAY  {press_s}  {prog*100:.0f}%",
                               [0, -0.6, 0.88], textColorRGB=[0.3, 0.8, 1.0],
                               textSize=1.4, replaceItemUniqueId=_ST)
            p.addUserDebugText(f"Fz = {fz_v:+.2f} N",
                               [0, -0.6, 0.80], textColorRGB=[0.7, 0.9, 1.0],
                               textSize=1.2, replaceItemUniqueId=_FZT)

            filled = int(prog * bar_width)
            bar    = "█"*filled + "░"*(bar_width-filled)
            print(f"\r[demo] [{bar}] {prog*100:5.1f}%  "
                  f"Fz={fz_v:+5.2f}N |F|={fr_mag:5.2f}N  "
                  f"cells={sum(1 for v in cells if v>0.05):2d}/19",
                  end="", flush=True)

            p.stepSimulation()
            time.sleep(0.033)

            if done:
                break

    except KeyboardInterrupt:
        print("\n[demo] Stopped")
    finally:
        _running[0] = False
        try:
            p.disconnect()
        except Exception:
            pass


# ── Entry point ─────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="KYWO system demo — replay a session CSV",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--backend", choices=["meshcat", "pybullet"],
                        default="meshcat",
                        help="3D backend (default: meshcat → browser tab)")
    parser.add_argument("--csv",   default=None,
                        help="Path to session CSV (auto-picks if omitted)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier (default 1.0)")
    parser.add_argument("--loop",  action="store_true",
                        help="Repeat replay indefinitely")
    args = parser.parse_args()

    print("="*60)
    print("  KYWO Tactile Sensing System — Demo")
    print("="*60)
    print(f"  Backend : {args.backend}")
    print(f"  Speed   : {args.speed}×")
    print(f"  Loop    : {args.loop}")
    print("="*60 + "\n")

    if args.backend == "meshcat":
        run_meshcat(speed=args.speed, loop=args.loop, csv_path=args.csv)
    else:
        run_pybullet(speed=args.speed, loop=args.loop, csv_path=args.csv)


if __name__ == "__main__":
    main()
