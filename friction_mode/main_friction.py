"""
main_friction.py
Friction mode experiment — lateral sliding with the star-nose sensor.

Two motion modes
────────────────
  --displacement   Fixed Z depth, robot slides along the chosen trajectory.
  --force          FUTEK-controlled Z, robot maintains constant contact force
                   while sliding.

Available trajectories (--trajectory)
──────────────────────────────────────
  line_h       Horizontal sweep  left → right  (center row, y = 0)
  line_v       Vertical sweep    bottom → top  (center col, x = 0)
  diagonal_lr  Diagonal          lower-left → upper-right
  diagonal_rl  Diagonal          lower-right → upper-left
  circle       Circular path around center  (default r = 12 mm)
  raster       Boustrophedon grid scan
  cross        Horizontal + vertical sweep combined
  spiral       Archimedean spiral from center outward
  star         Continuous path through all 19 sensor points

Examples
────────
  # Slide horizontally at 6 mm depth on the real robot
  python main_friction.py --displacement --depth 6 --trajectory line_h

  # Circular trace at 5 N contact force on the real robot
  python main_friction.py --force --target-force 5 --trajectory circle

  # Raster scan at 9 mm depth with URSim, auto-save analysis
  python main_friction.py --displacement --depth 9 --trajectory raster --sim --analyze

  # Sensor-only test (no robot), log 30 s of raw sensor data
  python main_friction.py --displacement --depth 0 --no-robot --duration 30
"""

import os
import sys
import json
import time
import argparse
import threading
from datetime import datetime

# ── Path setup — share sensor/logger from Integration_2 ──────────────────────
FRICTION_DIR    = os.path.dirname(os.path.abspath(__file__))
INTEGRATION_DIR = os.path.normpath(os.path.join(FRICTION_DIR, '..', 'Integration_2'))
sys.path.insert(0, INTEGRATION_DIR)
sys.path.insert(0, FRICTION_DIR)

LOG_DIR = os.path.join(FRICTION_DIR, 'logs')


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Friction mode — lateral sliding experiment',
        formatter_class=argparse.RawDescriptionHelpFormatter)

    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--displacement', action='store_true',
                      help='Fixed Z depth lateral motion')
    mode.add_argument('--force', action='store_true',
                      help='FUTEK-controlled Z, constant contact force')

    p.add_argument('--trajectory', default='line_h',
                   help='Trajectory name (see module docstring)')
    p.add_argument('--depth', type=float, default=6.0,
                   help='Indentation depth in mm [displacement mode]')
    p.add_argument('--target-force', type=float, default=5.0,
                   help='Target contact force in N [force mode]')
    p.add_argument('--speed', type=float, default=None,
                   help='Lateral sliding speed in mm/s (default: 8 mm/s)')
    p.add_argument('--steps', type=int, default=None,
                   help='Override number of trajectory waypoints')

    p.add_argument('--no-robot',  action='store_true', help='No UR5 movement')
    p.add_argument('--no-sensor', action='store_true', help='No tactile sensor')
    p.add_argument('--sim',       action='store_true',
                   help='Connect to URSim at localhost (Docker)')
    p.add_argument('--analyze',   action='store_true',
                   help='Run analyze_session.py after the session')
    p.add_argument('--log-prefix', default=None,
                   help='Custom log filename prefix')
    p.add_argument('--tip', default=None,
                   help='Tip profile — loads calib_<tip>.json')
    p.add_argument('--duration', type=float, default=None,
                   help='Stop after this many seconds (for sensor-only runs)')
    return p.parse_args()


# ── Calibration (inline — avoids ur5_control import in load_calibration.py) ──

def _apply_calibration(ur5, tip=None):
    """Load calib[_tip].json from Integration_2 and apply to ur5_friction."""
    name = f'calib_{tip}.json' if tip else 'calib.json'
    path = os.path.join(INTEGRATION_DIR, name)
    if not os.path.exists(path):
        print(f"[calib] {name} not found — using zero offset")
        return

    with open(path) as f:
        d = json.load(f)
    ur5.set_calibration(d.get('x_mm', 0.0),
                        d.get('y_mm', 0.0),
                        d.get('z_mm', 0.0))
    print(f"[calib] Loaded from {name}")


def _confirm_calibration(tip=None):
    """Ask user to confirm the correct tip is mounted."""
    label = tip if tip else '(default)'
    print()
    print("=" * 50)
    print("  CALIBRATION CHECK")
    print("=" * 50)
    print(f"  Tip profile : {label}")
    try:
        ans = input("  Correct tip mounted? Continue? [y/N] > ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        raise SystemExit(1)
    if ans != 'y':
        print("[calib] Aborted.")
        raise SystemExit(1)
    print()


# ── Banner ────────────────────────────────────────────────────────────────────

def print_banner(args):
    mode  = 'DISPLACEMENT' if args.displacement else 'FORCE CONTROL'
    speed = (args.speed if args.speed else 8.0)
    print("=" * 60)
    print("  KYWO — Friction Mode Experiment")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)
    print(f"  Mode       : {mode}")
    print(f"  Trajectory : {args.trajectory}")
    if args.displacement:
        print(f"  Depth      : {args.depth:.1f} mm")
    else:
        print(f"  Target F   : {args.target_force:.1f} N")
    print(f"  Speed      : {speed:.1f} mm/s")
    print(f"  UR5 robot  : {'OFF' if args.no_robot  else 'ON'}")
    print(f"  Sensor     : {'OFF' if args.no_sensor else 'ON'}")
    print(f"  Sim mode   : {'ON'  if args.sim       else 'OFF'}")
    print("=" * 60)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    if args.sim:
        os.environ["UR_ROBOT_IP"] = "127.0.0.1"
        print("[main] Simulator mode — UR_ROBOT_IP = 127.0.0.1")

    # Speed: CLI is mm/s; ur5_friction uses m/s internally
    speed_mps = (args.speed / 1000.0) if args.speed else None

    # ── Deferred imports ──────────────────────────────────────
    import trajectories as traj_lib
    import data_logger
    import ur5_friction as ur5

    if not args.no_sensor:
        import sensor

    print_banner(args)

    # ── Calibration ───────────────────────────────────────────
    if not args.no_robot:
        _confirm_calibration(args.tip)
        _apply_calibration(ur5, args.tip)

    # ── Build trajectory ──────────────────────────────────────
    name = args.trajectory
    if name not in traj_lib.TRAJECTORIES:
        print(f"[main] Unknown trajectory '{name}'. Available: "
              f"{', '.join(traj_lib.TRAJECTORIES)}")
        raise SystemExit(1)

    fn = traj_lib.TRAJECTORIES[name]
    if args.steps:
        try:
            pts = fn(n_steps=args.steps)
        except TypeError:
            pts = fn()
    else:
        pts = fn()

    traj_lib.preview(name, pts)

    # ── Start tactile sensor ──────────────────────────────────
    if not args.no_sensor:
        print("\n[main] Starting tactile sensor...")
        sensor.start()
        if not sensor.wait_until_ready(timeout=60):
            print("[main] ERROR: sensor not ready!")
            print("[main] Check USB → /dev/ttyACM0 or use --no-sensor")
            raise SystemExit(1)
        print("[main] Sensor ready!\n")

    # ── Start data logger ─────────────────────────────────────
    if args.log_prefix:
        prefix = data_logger.sanitize_name(args.log_prefix)
    else:
        prefix = data_logger.ask_file_prefix()

    log_file = data_logger.build_filename(prefix, LOG_DIR)
    data_logger.start(log_file)
    print(f"[main] Logging → {log_file}")

    _stop_log = threading.Event()

    def log_loop():
        while not _stop_log.is_set():
            try:
                sv = sensor.get_values() if not args.no_sensor else [0.0] * 19
                data_logger.log(sv, ur5.get_state())
            except Exception as e:
                print(f"[logger] {e}")
            time.sleep(0.05)    # 20 Hz

    log_thread = threading.Thread(target=log_loop, daemon=True)
    log_thread.start()
    print("[main] Logger running at 20 Hz")

    # ── Indentation / force confirmation ──────────────────────
    if not args.no_robot:
        if args.displacement:
            print(f"\n  Indentation depth : {args.depth:.1f} mm")
            try:
                ans = input("  Change depth? [Enter = keep] > ").strip()
                if ans:
                    args.depth = float(ans)
                    print(f"  Depth set to {args.depth:.1f} mm\n")
            except (EOFError, ValueError):
                pass
        else:
            print(f"\n  Target contact force : {args.target_force:.1f} N")
            try:
                ans = input("  Change target force? [Enter = keep] > ").strip()
                if ans:
                    args.target_force = float(ans)
                    print(f"  Target force set to {args.target_force:.1f} N\n")
            except (EOFError, ValueError):
                pass

    # ── Launch robot ──────────────────────────────────────────
    ur5_thread = None
    if not args.no_robot:
        def waypoint_cb(i, x, y, z):
            if i % 20 == 0 or i == len(pts) - 1:
                ai0 = ur5.current_ai0
                f   = ur5.ai0_to_N(ai0)
                print(f"[main] wp {i + 1:4d}/{len(pts)} "
                      f"XY=({x:+5.1f},{y:+5.1f}) mm  "
                      f"Z={z:4.2f} mm  FUTEK={f:.2f} N")

        def robot_fn():
            if args.displacement:
                ur5.run_displacement_trajectory(
                    pts, args.depth, speed_mps, on_waypoint=waypoint_cb)
            else:
                ur5.run_force_trajectory(
                    pts, args.target_force, speed_mps, on_waypoint=waypoint_cb)

        ur5_thread = threading.Thread(target=robot_fn, daemon=False)
        ur5_thread.start()
        print("\n[main] Robot trajectory started — press Ctrl+C to stop early\n")

    # ── Monitor loop ──────────────────────────────────────────
    print("[main] Running...\n")
    start_time = time.time()

    try:
        while True:
            state   = ur5.get_state()
            sv      = sensor.get_values() if not args.no_sensor else [0.0] * 19
            ft      = ur5.get_force()
            ai0     = state.get('ai0', 0.0)
            f_lc    = ur5.ai0_to_N(ai0)
            fz      = ft[2] if ft else 0.0
            rows    = data_logger.get_row_count()
            wp      = state.get('point', '-')
            slide   = 'SLIDING' if state.get('pressing') else 'moving '
            elapsed = time.time() - start_time
            active  = sum(1 for v in sv if v > 0.05)
            maxv    = max(sv) if sv else 0.0

            print(f"[main] t={elapsed:6.1f}s | "
                  f"cells={active:2d}/19  max={maxv:.3f} | "
                  f"Fz={fz:+6.2f} N  FUTEK={f_lc:5.2f} N | "
                  f"wp={wp} {slide} | rows={rows}")

            # Duration limit (sensor-only runs)
            if args.duration and elapsed >= args.duration:
                print(f"\n[main] Duration {args.duration:.0f}s reached")
                break

            # Robot finished
            if not args.no_robot and state.get('done'):
                print("\n[main] Robot trajectory complete!")
                time.sleep(1)
                break

            time.sleep(2)

    except KeyboardInterrupt:
        print("\n[main] Stopped by user")

    # ── Cleanup ───────────────────────────────────────────────
    if ur5_thread and ur5_thread.is_alive():
        print("[main] Waiting for robot to return home...")
        ur5.request_stop()
        ur5_thread.join(timeout=25)

    _stop_log.set()
    time.sleep(0.2)
    data_logger.stop()

    elapsed = time.time() - start_time
    rows    = data_logger.get_row_count()
    rate    = rows / elapsed if elapsed > 0 else 0
    print(f"\n[main] Session done!")
    print(f"  Duration : {elapsed:.1f} s")
    print(f"  Rows     : {rows:,}  ({rate:.1f} Hz)")
    print(f"  File     : {log_file}")
    print("=" * 60)

    # ── Optional post-analysis ────────────────────────────────
    if args.analyze and rows > 50:
        import subprocess
        analyze = os.path.join(INTEGRATION_DIR, 'analyze_session.py')
        print(f"\n[main] Running analysis...")
        subprocess.run(
            [sys.executable, analyze, os.path.basename(log_file), '--save'],
            cwd=INTEGRATION_DIR)

    print("\n[main] Done!")


if __name__ == '__main__':
    main()
