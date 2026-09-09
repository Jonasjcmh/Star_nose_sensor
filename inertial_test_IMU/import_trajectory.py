"""
import_trajectory.py  —  Inertial (IMU) Test  |  JSON Trajectory Importer
=========================================================================
Load an arbitrary XY trajectory from a JSON file, recentre it, and scale it
to the robot working area (a square of at most 23 cm × 23 cm). The imported
path can then be previewed, saved back out normalised, or run on the robot.

Accepted JSON shapes (units are arbitrary — they get rescaled):
    [[x, y], [x, y], ...]
    {"waypoints": [[x, y], ...]}
    {"points":    [{"x": .., "y": ..}, ...]}
    {"x": [..], "y": [..]}

Scaling
-------
  --fill (default) : scale so the longer side of the path fills --size mm,
                     capped at 23 cm. Aspect ratio is preserved.
  --no-fill        : only shrink if the path is larger than --size mm
                     (keeps the original scale otherwise).

Usage
-----
  python import_trajectory.py path.json --preview
  python import_trajectory.py path.json --size 140 --save scaled.json
  python import_trajectory.py path.json --run --speed 40
  python import_trajectory.py path.json --preview --no-fill
"""
import os
import sys
import json
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_trajectories as traj


# ─────────────────────────────────────────────────────────────────────────────
# Loading / parsing
# ─────────────────────────────────────────────────────────────────────────────

def parse_points(data):
    """
    Coerce one of the accepted JSON structures into a list of waypoints.

    Each waypoint is (x, y) or (x, y, yaw) when a yaw/theta channel is present.
    Yaw is carried through unchanged (angles are never rescaled on import).
    """
    if isinstance(data, dict):
        if 'waypoints' in data:
            seq = data['waypoints']
        elif 'points' in data:
            seq = data['points']
        elif 'x' in data and 'y' in data:
            th = data.get('theta') or data.get('yaw')
            if th is not None:
                return [(float(x), float(y), float(t))
                        for x, y, t in zip(data['x'], data['y'], th)]
            return [(float(x), float(y)) for x, y in zip(data['x'], data['y'])]
        else:
            raise ValueError("JSON object must contain 'waypoints', 'points', "
                             "or both 'x' and 'y' arrays")
    elif isinstance(data, list):
        seq = data
    else:
        raise ValueError('unsupported JSON top-level type')

    pts = []
    for item in seq:
        if isinstance(item, dict):
            th = item.get('theta', item.get('yaw'))
            if th is not None:
                pts.append((float(item['x']), float(item['y']), float(th)))
            else:
                pts.append((float(item['x']), float(item['y'])))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            if len(item) >= 3:
                pts.append((float(item[0]), float(item[1]), float(item[2])))
            else:
                pts.append((float(item[0]), float(item[1])))
        else:
            raise ValueError(f'cannot interpret waypoint: {item!r}')
    if len(pts) < 2:
        raise ValueError('trajectory needs at least 2 waypoints')
    return pts


def load_trajectory(path, span_mm=traj.WORKAREA_MAX_MM, fill=True):
    """
    Load a JSON trajectory.

    Returns (pts, absolute):
      • absolute=False → points recentred + scaled into the 23 cm working area
        (arbitrary/native imports).
      • absolute=True  → the file is tagged "frame": "ur_base_mm" (a calibrated
        UR-frame trajectory); points are returned as absolute UR base mm,
        untouched. Yaw (if present) is preserved in both cases.
    """
    with open(path) as f:
        data = json.load(f)
    if isinstance(data, dict) and data.get('frame') == 'ur_base_mm':
        return parse_points(data), True
    raw = parse_points(data)
    return traj.scale_to_area(raw, span_mm=span_mm, fill=fill), False


def load_and_scale(path, span_mm=traj.WORKAREA_MAX_MM, fill=True):
    """Back-compat helper: scaled points only (drops the absolute flag)."""
    return load_trajectory(path, span_mm=span_mm, fill=fill)[0]


# ─────────────────────────────────────────────────────────────────────────────
# Output helpers
# ─────────────────────────────────────────────────────────────────────────────

def save_points(pts, path):
    """Write scaled points back out in the canonical {"waypoints": ...} form."""
    has_yaw = any(len(p) >= 3 for p in pts)
    payload = {
        'units': 'mm',
        'yaw_units': 'deg' if has_yaw else None,
        'frame': 'reference-relative working area',
        'workarea_max_mm': traj.WORKAREA_MAX_MM,
        'waypoints': [[round(p[0], 3), round(p[1], 3)]
                      + ([round(p[2], 3)] if len(p) >= 3 else [])
                      for p in pts],
    }
    with open(path, 'w') as f:
        json.dump(payload, f, indent=2)
    print(f'[import] Saved {len(pts)} scaled waypoints '
          f'({"x,y,yaw" if has_yaw else "x,y"}) → {path}')


def preview(pts, title):
    """Static matplotlib preview of the scaled path inside the working area."""
    import matplotlib.pyplot as plt
    half = traj.WORKAREA_MAX_MM / 2.0
    fig, ax = plt.subplots(figsize=(7, 7), facecolor='#111111')
    ax.set_facecolor('#111111')
    ax.add_patch(plt.Rectangle((-half, -half), 2 * half, 2 * half, fill=False,
                               edgecolor='#663333', linestyle='--'))
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, color='#33e666', linewidth=1.5)
    ax.plot(xs[0], ys[0], 's', color='#3498db', ms=8, label='start')
    ax.plot(xs[-1], ys[-1], 'o', color='#dc0000', ms=8, label='end')
    ax.set_xlim(-half * 1.1, half * 1.1);  ax.set_ylim(-half * 1.1, half * 1.1)
    ax.set_aspect('equal')
    ax.set_title(title, color='white')
    ax.set_xlabel('X (mm)', color='#aaaaaa');  ax.set_ylabel('Y (mm)', color='#aaaaaa')
    ax.tick_params(colors='#aaaaaa')
    for sp in ax.spines.values():
        sp.set_edgecolor('#444444')
    ax.grid(color='#444444', alpha=0.35)
    ax.legend(facecolor='#111111', labelcolor='white', edgecolor='#444444')
    plt.tight_layout()
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# CLI / Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Import + scale a JSON trajectory into the IMU working area',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('json', help='path to the JSON trajectory file')
    p.add_argument('--size',   type=float, default=traj.WORKAREA_MAX_MM,
                   help=f'target span in mm (default/cap {traj.WORKAREA_MAX_MM:.0f})')
    p.add_argument('--no-fill', action='store_true',
                   help='only shrink oversized paths; do not enlarge')
    p.add_argument('--preview', action='store_true',
                   help='show a static plot of the scaled path')
    p.add_argument('--save',   default=None, metavar='OUT',
                   help='write the scaled path to a new JSON file')
    p.add_argument('--run',    action='store_true',
                   help='execute the scaled path on the robot')
    p.add_argument('--speed',  type=float, default=30.0,
                   help='trajectory speed in mm/s when --run (default 30)')
    p.add_argument('--height', type=float, default=None,
                   help='work-plane height above reference in mm (default 30)')
    return p.parse_args()


def main():
    args = parse_args()

    if not os.path.isfile(args.json):
        print(f"[import] File not found: {args.json}")
        sys.exit(1)

    try:
        pts, absolute = load_trajectory(args.json, span_mm=args.size,
                                        fill=not args.no_fill)
    except (ValueError, KeyError, json.JSONDecodeError) as e:
        print(f"[import] Could not parse '{args.json}': {e}")
        sys.exit(1)

    name = os.path.basename(args.json)
    print('=' * 60)
    print('  Inertial (IMU) Test — JSON Trajectory Import')
    print('=' * 60)
    print(f'  File     : {name}')
    if absolute:
        print('  Frame    : ur_base_mm — ABSOLUTE calibrated UR positions '
              '(no rescale)')
    else:
        print(f'  Mode     : {"fill" if not args.no_fill else "clamp-only"}  '
              f'(target {args.size:.0f} mm, cap {traj.WORKAREA_MAX_MM:.0f} mm)')
    traj.describe(name, pts)
    print('=' * 60)

    if args.save:
        save_points(pts, args.save)

    if args.preview:
        preview(pts, f'Imported: {name}')

    if args.run:
        import ur5_imu as ur5
        height = args.height if args.height is not None else ur5.DEFAULT_HEIGHT_MM

        # Absolute (calibrated) trajectories: convert UR base mm → reference
        # offsets and run without recentring/rescaling; widen the safety window
        # to the actual span so true positions are not clipped.
        if absolute:
            run_pts = [(ur5.ur_to_offset(p[0], p[1]) + tuple(p[2:]))
                       for p in pts]
            span = max(max(abs(o[0]), abs(o[1])) for o in run_pts)
            clamp_mm = 2.0 * span + 40.0
            fit = False
        else:
            run_pts, clamp_mm, fit = pts, traj.WORKAREA_MAX_MM, True

        def _progress(i, x, y, _z):
            print(f'\r  waypoint {i + 1}/{len(run_pts)}  '
                  f'({x:+7.1f}, {y:+7.1f}) mm-off', end='', flush=True)

        print(f'\n[import] Robot ON — {os.environ.get("UR_ROBOT_IP", ur5.ROBOT_IP)}. '
              f'Keep the e-stop within reach.\n')
        try:
            ur5.run_trajectory(run_pts, height_mm=height,
                               speed_mps=args.speed / 1000.0,
                               on_waypoint=_progress, fit=fit, clamp_mm=clamp_mm)
        except KeyboardInterrupt:
            ur5.request_stop()
            print('\n[import] Stop requested — returning home.')
        print('\n[import] Done.')

    if not (args.save or args.preview or args.run):
        print('[import] Nothing to do — add --preview, --save OUT, or --run.')


if __name__ == '__main__':
    main()
