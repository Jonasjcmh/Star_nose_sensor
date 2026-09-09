"""
run_imu.py  —  Inertial (IMU) Test  |  Simple Trajectory Runner
===============================================================
Runs a single named trajectory on the real UR5, position control only.
No capacitive sensor, no FUTEK load cell, no calibration — just motion.

Define the size with ONE knob:
    --size    diameter (circle/spiral) or covered span (raster/square/...) in mm
Anything larger than the 23 cm × 23 cm working area is automatically clamped.

Usage
-----
  python run_imu.py --traj spiral --size 120         # 12 cm spiral
  python run_imu.py --traj circle --size 100         # 10 cm circle
  python run_imu.py --traj raster --size 140 --lines 10
  python run_imu.py --traj square --size 120 --speed 40
  python run_imu.py --list                           # list trajectory names
  python run_imu.py --traj circle --dry-run          # print path, don't move

Options
-------
  --traj    NAME    trajectory (circle, spiral, raster, square, cross,
                    line_h, line_v, figure8)
  --size    MM      diameter / covered span (default 120, max 230)
  --speed   MM/S    trajectory speed (default 30 mm/s)
  --height  MM      work-plane height above the reference pose (default 30)
  --turns   N       spiral turns (spiral only, default 4)
  --lines   N       raster lines (raster only, default 8)
  --steps   N       waypoints per line/arc (shape dependent)
  --dry-run         build + preview the trajectory without moving the robot
"""
import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_trajectories as traj


def parse_args():
    p = argparse.ArgumentParser(
        description='Inertial (IMU) test — simple trajectory runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--traj',   default='spiral', metavar='NAME',
                   help='trajectory name (default: spiral)')
    p.add_argument('--size',   type=float, default=traj.DEFAULT_SPAN_MM,
                   help=f'diameter / covered span in mm (default '
                        f'{traj.DEFAULT_SPAN_MM:.0f}, max {traj.WORKAREA_MAX_MM:.0f})')
    p.add_argument('--speed',  type=float, default=30.0,
                   help='trajectory speed in mm/s (default 30)')
    p.add_argument('--height', type=float, default=None,
                   help='work-plane height above reference in mm (default 30)')
    p.add_argument('--turns',  type=int, default=None, help='spiral turns')
    p.add_argument('--lines',  type=int, default=None, help='raster lines')
    p.add_argument('--steps',  type=int, default=None,
                   help='waypoints per segment/arc')
    p.add_argument('--dry-run', action='store_true',
                   help='build + preview only, no robot motion')
    p.add_argument('--list', action='store_true',
                   help='list available trajectory names and exit')
    return p.parse_args()


def build_from_args(args):
    """Assemble shape-specific keyword params from the CLI and build points."""
    params = {}
    if args.turns is not None:
        params['n_turns'] = args.turns
    if args.lines is not None:
        params['n_lines'] = args.lines
    if args.steps is not None:
        # Map the generic --steps onto whichever step arg the shape uses.
        for k in ('n_steps', 'n_pts_per_line', 'n_per_side'):
            params[k] = args.steps
    return traj.build(args.traj, size_mm=args.size, **params)


def main():
    args = parse_args()

    if args.list:
        print('Available trajectories:')
        for k, lbl in traj.LABELS.items():
            print(f'  {k:<10} {lbl}')
        return

    if args.traj not in traj.TRAJECTORIES:
        print(f"[run] Unknown trajectory '{args.traj}'. "
              f"Use --list to see options.")
        sys.exit(1)

    pts = build_from_args(args)

    print('=' * 60)
    print('  Inertial (IMU) Test — Trajectory Runner')
    print('=' * 60)
    print(f'  Trajectory : {traj.LABELS[args.traj]}  ({args.traj})')
    print(f'  Requested  : {args.size:.0f} mm  (working-area cap '
          f'{traj.WORKAREA_MAX_MM:.0f} mm)')
    print(f'  Speed      : {args.speed:.0f} mm/s')
    print(f'  Height     : {args.height if args.height is not None else 30:.0f} '
          f'mm above reference')
    print('-' * 60)
    traj.describe(args.traj, pts)
    print('=' * 60)

    if args.dry_run:
        print('[run] Dry-run — robot NOT moved.')
        return

    # Import robot layer lazily so --dry-run / --list work without a robot.
    import ur5_imu as ur5

    def _progress(i, x, y, _z):
        print(f'\r  waypoint {i + 1}/{len(pts)}  '
              f'({x:+6.1f}, {y:+6.1f}) mm', end='', flush=True)

    height = args.height if args.height is not None else ur5.DEFAULT_HEIGHT_MM
    print(f'\n[run] Robot ON — {os.environ.get("UR_ROBOT_IP", ur5.ROBOT_IP)}. '
          f'Keep the e-stop within reach.\n')
    try:
        ur5.run_trajectory(pts, height_mm=height,
                           speed_mps=args.speed / 1000.0,
                           on_waypoint=_progress)
    except KeyboardInterrupt:
        ur5.request_stop()
        print('\n[run] Stop requested — returning home.')
    print('\n[run] Done.')


if __name__ == '__main__':
    main()
