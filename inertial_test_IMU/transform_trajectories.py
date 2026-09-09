"""
transform_trajectories.py  —  Inertial (IMU) Test
=================================================
Transform every imported trajectory from the drawing frame (0..230 units) into
the UR robot BASE frame (mm) using ur_calibration.py, and write new JSON files
tagged `"frame": "ur_base_mm"`. XY becomes absolute UR base coordinates; a
yaw/theta column is rotated by the same frame rotation.

Files tagged `ur_base_mm` are executed by import_trajectory.py / imu_live.py in
ABSOLUTE mode (no recentre, no 23 cm rescale) — they run at their true
calibrated position on the table.

Usage
-----
  python transform_trajectories.py                     # imported_trajectories/ → trajectories_ur/
  python transform_trajectories.py -i in_dir -o out_dir
  python transform_trajectories.py --preview           # also write an overlay PNG
"""
import os
import sys
import glob
import json
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ur_calibration as cal

_HERE = os.path.dirname(os.path.abspath(__file__))


def transform_payload(data):
    """Return a new payload with waypoints mapped into the UR base frame."""
    wps = data.get('waypoints', [])
    has_yaw = any(len(w) >= 3 for w in wps)
    out = []
    for w in wps:
        X, Y = cal.to_ur(float(w[0]), float(w[1]))
        row = [round(X, 4), round(Y, 4)]
        if len(w) >= 3:
            row.append(round(cal.to_ur_yaw(float(w[2])), 4))
        out.append(row)
    return {
        'name': data.get('name'),
        'units': 'mm',
        'frame': 'ur_base_mm',
        'yaw_units': 'deg' if has_yaw else None,
        'source': data.get('source'),
        'calibration': {
            'P0_traj': cal.P0_TRAJ, 'P0_ur': cal.P0_UR,
            'P1_traj': cal.P1_TRAJ, 'P1_ur': cal.P1_UR,
            'scale': round(cal.SCALE, 6), 'rotation_deg': round(cal.ROT_DEG, 4),
            'mirror': cal.MIRROR,
        },
        'n_waypoints': len(out),
        'waypoints': out,
    }


def make_preview(pairs, path):
    """Overlay native (0..230) vs UR-frame paths so orientation can be checked."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    n = len(pairs)
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), facecolor='#111111')
    for ax, title in zip(axes, ['Trajectory frame (0..230 units)',
                                 'UR base frame (mm)']):
        ax.set_facecolor('#111111'); ax.set_aspect('equal')
        ax.set_title(title, color='white')
        ax.tick_params(colors='#aaaaaa')
        for sp in ax.spines.values():
            sp.set_edgecolor('#444444')
        ax.grid(color='#444444', alpha=0.3)
    cmap = plt.get_cmap('turbo')
    for i, (name, native, urf) in enumerate(pairs):
        c = cmap(i / max(n - 1, 1))
        axes[0].plot([p[0] for p in native], [p[1] for p in native],
                     color=c, lw=1.0)
        axes[1].plot([p[0] for p in urf], [p[1] for p in urf],
                     color=c, lw=1.0, label=name)
    # UR-frame calibration square + measured corners
    cs = cal.corners() + [cal.corners()[0]]
    axes[1].plot([c[0] for c in cs], [c[1] for c in cs],
                 '--', color='#663333', lw=1.2)
    for (lbl, pt) in [('(0,0)', cal.P0_UR), ('(230,230)', cal.P1_UR)]:
        axes[1].plot(pt[0], pt[1], 'x', color='white', ms=9)
        axes[1].annotate(lbl, pt, color='white', fontsize=8)
    axes[1].legend(fontsize=5, facecolor='#111111', labelcolor='white',
                   edgecolor='#444444', loc='center left', bbox_to_anchor=(1.0, 0.5))
    fig.tight_layout()
    fig.savefig(path, dpi=70, facecolor='#111111')
    print(f'[transform] Preview → {path}')


def parse_args():
    p = argparse.ArgumentParser(
        description='Transform imported trajectories into the UR base frame',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('-i', '--in', dest='indir',
                   default=os.path.join(_HERE, 'imported_trajectories'),
                   help='input dir of native JSONs (default: imported_trajectories/)')
    p.add_argument('-o', '--out', default=os.path.join(_HERE, 'trajectories_ur'),
                   help='output dir (default: trajectories_ur/)')
    p.add_argument('--preview', action='store_true',
                   help='also write a native-vs-UR overlay PNG')
    return p.parse_args()


def main():
    args = parse_args()
    files = sorted(glob.glob(os.path.join(args.indir, '*.json')))
    if not files:
        print(f'[transform] No JSON files in {args.indir}')
        sys.exit(1)

    cal.summary()
    print()
    os.makedirs(args.out, exist_ok=True)
    pairs = []
    for f in files:
        with open(f) as fp:
            data = json.load(fp)
        payload = transform_payload(data)
        out_path = os.path.join(args.out, os.path.basename(f))
        with open(out_path, 'w') as fp:
            json.dump(payload, fp, indent=2)
        native = [(w[0], w[1]) for w in data.get('waypoints', [])]
        urf = [(w[0], w[1]) for w in payload['waypoints']]
        pairs.append((payload['name'] or os.path.basename(f), native, urf))
        print(f'  {os.path.basename(f):<40} {payload["n_waypoints"]:>4d} pts → UR frame')

    print(f'\n[transform] Wrote {len(pairs)} files → {args.out}/')
    if args.preview:
        make_preview(pairs, os.path.join(args.out, '_preview_native_vs_ur.png'))


if __name__ == '__main__':
    main()
