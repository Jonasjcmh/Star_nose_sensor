"""
main.py
KYWO Integration — sensor + UR5 + SOFA + 2D viz + logging + analysis.

Usage:
  python3.10 main.py                     # everything
  python3.10 main.py --no-sofa           # no SOFA 3D
  python3.10 main.py --no-viz            # no 2D visualizer
  python3.10 main.py --no-sofa --no-viz  # sensor + robot + logging only
  python3.10 main.py --no-robot          # no robot
  python3.10 main.py --demo              # simulated sensor, no robot
  python3.10 main.py --sofa-only         # SOFA only
  python3.10 main.py --viz-only          # 2D visualizer only
  python3.10 main.py --log-only          # logging only
  python3.10 main.py --analyze           # analyze after session ends
"""

import os
import sys
import time
import argparse
import threading
import subprocess
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SOFA_BIN    = os.path.expanduser(
    "~/sofa/SOFA_v25.12.00_Linux/bin/runSofa-25.12.00")
SOFA_PLUGIN = os.path.expanduser(
    "~/sofa/SOFA_v25.12.00_Linux/plugins/SofaPython3/lib/libSofaPython3.so")
SOFA_SCENE  = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "sofa_scene.py")
VIZ_SCRIPT      = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "visualizer_2d.py")
VIZ3D_SCRIPT     = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "robot_viz_3d.py")
VIZ_MESHCAT_SCRIPT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "robot_viz_meshcat.py")
VIZ_PYBULLET_SCRIPT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "robot_viz_pybullet.py")
VIZ_GAZEBO_SCRIPT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "robot_viz_gazebo.py")
ANALYZE_SCRIPT  = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "analyze_session.py")
INTEGRATION_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR         = os.path.join(INTEGRATION_DIR, "logs")

# Marker file the Gazebo twin writes once its scene is loaded.
# main waits for it before moving the robot so motion and visualization start together.
GAZEBO_READY_FILE = "/tmp/star_nose_gazebo_ready"
# Gazebo twin's stdout/stderr go here so its verbose logs don't clutter the
# main terminal (which needs a clean prompt for the start-robot confirmation).
GAZEBO_LOG_FILE   = "/tmp/star_nose_gazebo_viz.log"

class NoRobotState:
    UR5_TO_SENSOR = {}
    _lock = threading.Lock()
    is_done = True

    @staticmethod
    def get_state():
        return {
            'point': '',
            'pressing': False,
            'done': True,
            'ft': [0.0] * 6,
            'tcp': [0.0] * 6,
            'ai0': 0.0,
        }

    @staticmethod
    def get_force():
        return [0.0] * 6

def load_runtime_modules(robot_enabled=True):
    """Import hardware modules after CLI parsing."""
    global sensor, ur5_control, data_logger
    import sensor
    import data_logger
    if robot_enabled:
        import ur5_control
    else:
        ur5_control = NoRobotState()

def parse_args():
    p = argparse.ArgumentParser(description='KYWO sensor integration')
    p.add_argument('--no-sofa',   action='store_true',
                   help='Skip SOFA 3D visualization')
    p.add_argument('--no-viz',    action='store_true',
                   help='Skip 2D pygame visualizer')
    p.add_argument('--no-robot',  action='store_true',
                   help='Skip UR5 robot movement')
    p.add_argument('--demo',      action='store_true',
                   help='Demo mode — simulated sensor, no robot')
    p.add_argument('--sim-sensor', action='store_true',
                   help='Simulated sensor data — robot still runs (use with --sim)')
    p.add_argument('--log-only',  action='store_true',
                   help='Sensor + logging only')
    p.add_argument('--sofa-only', action='store_true',
                   help='SOFA only')
    p.add_argument('--viz-only',  action='store_true',
                   help='2D visualizer only')
    p.add_argument('--sim',       action='store_true',
                   help='Connect to URSim at localhost:30004 (Docker)')
    p.add_argument('--robot-viz', action='store_true',
                   help='Launch matplotlib 3D robot visualizer alongside the session')
    p.add_argument('--robot-viz-meshcat', action='store_true',
                   help='Launch browser-based Meshcat digital twin (Three.js)')
    p.add_argument('--robot-viz-pybullet', action='store_true',
                   help='Launch PyBullet OpenGL digital twin')
    p.add_argument('--robot-viz-gazebo', action='store_true',
                   help='Launch Gazebo + ROS2 digital twin')
    p.add_argument('--analyze',   action='store_true',
                   help='Run analysis after session ends')
    p.add_argument('--log-prefix',
                   help='Beginning of the log filename')
    p.add_argument('--duration', type=float,
                   help='Stop automatically after this many seconds')
    return p.parse_args()

def print_banner(args):
    print("="*60)
    print("  KYWO — Sensor + UR5 + Visualization + Logging")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)
    print(f"  SOFA 3D    : {'OFF' if args.no_sofa  else 'ON'}")
    print(f"  2D viz     : {'OFF' if args.no_viz   else 'ON'}")
    print(f"  UR5 robot  : {'OFF' if args.no_robot or args.demo else 'ON'}")
    print(f"  Demo mode  : {'ON'  if args.demo     else 'OFF'}")
    print(f"  Logging    : ON")
    print(f"  Auto-analyze: {'ON' if args.analyze  else 'OFF'}")
    print("="*60)

def launch_sofa():
    print("[main] Launching SOFA 3D...")
    proc = subprocess.Popen(
        [SOFA_BIN, "--load", SOFA_PLUGIN, SOFA_SCENE])
    print("[main] Waiting 8s for SOFA to initialize...")
    time.sleep(8)
    print("[main] SOFA ready!")
    return proc

def launch_viz2d():
    print("[main] Launching 2D visualizer...")
    proc = subprocess.Popen([sys.executable, VIZ_SCRIPT])
    time.sleep(2)
    print("[main] 2D visualizer ready!")
    return proc

def wait_for_gazebo_ready(proc, timeout=600.0):
    """Block until the Gazebo twin signals its scene is loaded.

    The twin writes GAZEBO_READY_FILE once the robot is spawned and visible.
    Returns True when the marker appears. Returns False (with a warning) if
    the twin process exits or the timeout elapses, so a Gazebo failure never
    blocks the experiment indefinitely.
    """
    print("[main] Waiting for Gazebo simulation to load the robot...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(GAZEBO_READY_FILE):
            print("[main] Gazebo scene loaded")
            return True
        if proc.poll() is not None:
            print("[main] WARNING: Gazebo exited before loading — "
                  "starting robot anyway")
            return False
        time.sleep(0.5)
    print("[main] WARNING: Gazebo load-wait timed out — starting robot anyway")
    return False

def run_ur5_safe(on_press=None, on_release=None):
    """Run UR5 trajectory with retry logic"""
    for attempt in range(3):
        try:
            print(f"[ur5] Connection attempt {attempt+1}/3...")
            ur5_control.run_trajectory(
                on_press=on_press,
                on_release=on_release,
                interactive=False,
            )
            return
        except Exception as e:
            print(f"[ur5] Attempt {attempt+1} failed: {e}")
            if attempt < 2:
                print("[ur5] Retrying in 3s...")
                time.sleep(3)

    print("[ur5] All attempts failed — skipping trajectory")
    with ur5_control._lock:
        ur5_control.is_done = True

def start_demo_sensor():
    """Simulated sensor for demo/testing"""
    import math

    def demo_loop():
        frame = 0
        sensor._is_ready = True
        print("[sensor] Demo mode — simulating pressure data")
        while True:
            t = frame * 0.05
            vals = [
                max(0.0, min(1.0,
                    0.5 * math.sin(t + i * 0.4) *
                    math.sin(t * 0.7 + i * 0.2) + 0.3))
                for i in range(19)
            ]
            with sensor._lock:
                sensor._values = vals
            frame += 1
            time.sleep(0.02)

    threading.Thread(target=demo_loop, daemon=True).start()
    sensor._start_shared_writer()
    print("[sensor] Demo sensor running")

def run_analysis(log_file):
    """Run post-processing analysis after session"""
    print(f"\n[main] Running analysis on: {os.path.basename(log_file)}")
    try:
        subprocess.run([
            sys.executable, ANALYZE_SCRIPT,
            os.path.basename(log_file),
            '--save'
        ])
    except Exception as e:
        print(f"[main] Analysis failed: {e}")

def main():
    args = parse_args()

    # Handle shortcuts
    if args.sofa_only:
        args.no_viz   = True
    if args.viz_only:
        args.no_sofa  = True
    if args.log_only:
        args.no_sofa  = True
        args.no_viz   = True
    if args.demo:
        args.no_robot = True
    if args.sim:
        os.environ["UR_ROBOT_IP"] = "127.0.0.1"
        print("[main] Simulator mode — robot IP set to 127.0.0.1")

    load_runtime_modules(robot_enabled=not args.no_robot)

    print_banner(args)

    # ── Load calibration ──────────────────────────────────────
    try:
        import load_calibration
        load_calibration.apply()
    except Exception as e:
        print(f"[main] No calibration file: {e}")

    # ── Start sensor ──────────────────────────────────────────
    _sim_sensor = args.demo or getattr(args, 'sim_sensor', False)
    if _sim_sensor:
        print("\n[main] Using simulated sensor data")
        start_demo_sensor()
    else:
        print("\n[main] Starting sensor...")
        sensor.start()
        print("[main] Waiting for sensor calibration...")
        if not sensor.wait_until_ready(timeout=60):
            print("[main] ERROR: Sensor not ready!")
            print("[main] Check USB connection to /dev/ttyACM0")
            print("[main] Or use --demo for simulated data")
            sys.exit(1)
        print("[main] Sensor ready!\n")

    # ── Start data logger ─────────────────────────────────────
    log_prefix = (data_logger.sanitize_name(args.log_prefix)
                  if args.log_prefix
                  else data_logger.ask_file_prefix())
    log_file = data_logger.build_filename(log_prefix, LOG_DIR)
    data_logger.start(log_file)
    print(f"[main] Logging → {log_file}")

    # ── Logging thread at 20Hz ────────────────────────────────
    _stop_log = threading.Event()

    def log_loop():
        while not _stop_log.is_set():
            try:
                data_logger.log(
                    sensor.get_values(),
                    ur5_control.get_state()
                )
            except Exception as e:
                print(f"[logger] Error: {e}")
            time.sleep(0.05)

    log_thread = threading.Thread(target=log_loop, daemon=True)
    log_thread.start()
    print("[main] Logger running at 20Hz")

    # ── Launch visualizers ────────────────────────────────────
    procs = []

    if not args.no_sofa:
        try:
            procs.append(('SOFA', launch_sofa()))
        except Exception as e:
            print(f"[main] SOFA failed to launch: {e}")

    if not args.no_viz:
        try:
            # Small delay so shared file exists before viz reads it
            time.sleep(1)
            procs.append(('2D viz', launch_viz2d()))
        except Exception as e:
            print(f"[main] 2D visualizer failed: {e}")

    if getattr(args, 'robot_viz', False):
        try:
            sim_flag = ["--sim"] if args.sim else []
            proc = subprocess.Popen(
                [sys.executable, VIZ3D_SCRIPT] + sim_flag)
            procs.append(('3D viz', proc))
            print("[main] 3D robot visualizer launched")
        except Exception as e:
            print(f"[main] 3D visualizer failed: {e}")

    if getattr(args, 'robot_viz_meshcat', False):
        try:
            sim_flag = ["--sim"] if args.sim else []
            proc = subprocess.Popen(
                [sys.executable, VIZ_MESHCAT_SCRIPT] + sim_flag)
            procs.append(('meshcat', proc))
            print("[main] Meshcat browser digital twin launched")
        except Exception as e:
            print(f"[main] Meshcat visualizer failed: {e}")

    if getattr(args, 'robot_viz_pybullet', False):
        try:
            sim_flag = ["--sim"] if args.sim else []
            proc = subprocess.Popen(
                [sys.executable, VIZ_PYBULLET_SCRIPT] + sim_flag)
            procs.append(('pybullet', proc))
            print("[main] PyBullet digital twin launched")
        except Exception as e:
            print(f"[main] PyBullet visualizer failed: {e}")

    gazebo_proc = None
    if getattr(args, 'robot_viz_gazebo', False):
        try:
            # Clear any stale marker so we only react to this run's signal
            if os.path.exists(GAZEBO_READY_FILE):
                os.remove(GAZEBO_READY_FILE)
            sim_flag = ["--sim"] if args.sim else []
            # Redirect the twin's verbose output to a log file so the main
            # terminal stays clean for the start-robot confirmation prompt.
            gz_log = open(GAZEBO_LOG_FILE, "w")
            gazebo_proc = subprocess.Popen(
                [sys.executable, VIZ_GAZEBO_SCRIPT] + sim_flag,
                stdout=gz_log, stderr=subprocess.STDOUT)
            procs.append(('gazebo', gazebo_proc))
            print(f"[main] Gazebo digital twin launched (log → {GAZEBO_LOG_FILE})")
        except Exception as e:
            print(f"[main] Gazebo visualizer failed: {e}")

    # ── Wait for Gazebo to be up, then confirm before moving robot ──
    if gazebo_proc is not None and not args.no_robot and not args.demo:
        if wait_for_gazebo_ready(gazebo_proc):
            try:
                input("\n[main] Gazebo is up — check the window, then press "
                      "Enter to start the robot... ")
            except EOFError:
                pass  # non-interactive: start immediately

    # ── Start UR5 trajectory ──────────────────────────────────
    ur5_thread = None
    if not args.no_robot and not args.demo:  # --demo always implies --no-robot

        USED_CELLS = sensor.USED_CELLS

        def on_press(pt):
            raw = ur5_control.UR5_TO_SENSOR.get(pt, -1)
            idx = USED_CELLS.index(raw) \
                  if raw in USED_CELLS else -1
            vals = sensor.get_values()
            v    = vals[idx] if 0 <= idx < len(vals) else 0.0
            ft   = ur5_control.get_force()
            print(f"[main] ▼ P{pt:02d} → S{raw} | "
                  f"sensor={v:.3f} | "
                  f"Fz={ft[2]:.2f}N")

        def on_release(pt):
            raw = ur5_control.UR5_TO_SENSOR.get(pt, -1)
            idx = USED_CELLS.index(raw) \
                  if raw in USED_CELLS else -1
            vals = sensor.get_values()
            v    = vals[idx] if 0 <= idx < len(vals) else 0.0
            ft   = ur5_control.get_force()
            print(f"[main] ▲ P{pt:02d} released | "
                  f"peak={v:.3f} | "
                  f"Fz={ft[2]:.2f}N")

        print("\n[main] Starting UR5 trajectory...")
        ur5_thread = threading.Thread(
            target=run_ur5_safe,
            kwargs=dict(
                on_press=on_press,
                on_release=on_release
            ),
            daemon=False
        )
        ur5_thread.start()
        print("[main] UR5 trajectory started!\n")

    # ── Monitor loop ──────────────────────────────────────────
    print("[main] Running — press Ctrl+C to stop early\n")
    start_time = time.time()

    try:
        while True:
            state  = ur5_control.get_state()
            vals   = sensor.get_values()
            ft     = ur5_control.get_force()
            active = sum(1 for v in vals if v > 0.05)
            maxv   = max(vals) if vals else 0
            rows   = data_logger.get_row_count()
            pt     = state.get('point', '-')
            press  = 'PRESSING' if state.get('pressing') else 'moving  '
            fz     = ft[2] if ft else 0.0
            ai0    = state.get('ai0', 0.0)
            elapsed = time.time() - start_time

            # Check visualizers still alive
            dead = [n for n, p in procs if p.poll() is not None]
            for n in dead:
                procs[:] = [(name, p) for name, p in procs
                            if name != n]
                print(f"\n[main] {n} closed")

            print(f"[main] "
                  f"t={elapsed:6.1f}s | "
                  f"active={active:2d}/19 | "
                  f"max={maxv:.3f} | "
                  f"Fz={fz:+6.2f}N | "
                  f"AI0={ai0:5.3f}V | "
                  f"UR5=P{pt} {press} | "
                  f"rows={rows:5d} | "
                  f"viz={'|'.join(n for n, _ in procs) or 'none'}")

            # ── Exit conditions ───────────────────────────────
            if args.duration and elapsed >= args.duration:
                print(f"\n[main] Duration reached ({args.duration:.1f}s)")
                break

            # Robot finished trajectory
            _robot_active = not args.no_robot and not args.demo
            if _robot_active:
                if state.get('done'):
                    print("\n[main] ✓ UR5 trajectory complete!")
                    time.sleep(2)  # collect last frames
                    break

            # All visualizers closed
            if procs and len(procs) == 0:
                print("\n[main] All visualizers closed")
                break

            # No exit condition — keep running until Ctrl+C
            if not _robot_active:
                if procs and len(procs) == 0:
                    break

            time.sleep(2)

    except KeyboardInterrupt:
        print("\n[main] Stopped by user")

    # ── Cleanup ───────────────────────────────────────────────
    print("\n[main] Shutting down...")

    # Signal UR5 to stop after current point and return home
    if ur5_thread is not None and ur5_thread.is_alive():
        print("[main] Waiting for UR5 to return to home position...")
        ur5_control.request_stop()
        ur5_thread.join(timeout=20)
        if ur5_thread.is_alive():
            print("[main] WARNING: UR5 thread did not finish in time")

    # Stop logger
    _stop_log.set()
    time.sleep(0.3)
    data_logger.stop()

    total_time = time.time() - start_time
    rows       = data_logger.get_row_count()
    rate       = rows / total_time if total_time > 0 else 0

    print(f"\n[main] Session complete!")
    print(f"  Duration : {total_time:.1f}s")
    print(f"  Rows     : {rows:,}")
    print(f"  Rate     : {rate:.1f} Hz")
    print(f"  File     : {log_file}")

    # Close visualizers
    for name, proc in procs:
        try:
            proc.terminate()
            print(f"[main] {name} closed")
        except Exception:
            pass

    print("="*60)

    # ── Auto-analyze ──────────────────────────────────────────
    if args.analyze and rows > 100:
        print("\n[main] Starting post-processing analysis...")
        time.sleep(1)
        run_analysis(log_file)

    print("\n[main] Done!")

if __name__ == "__main__":
    main()
