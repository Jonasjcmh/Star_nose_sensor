"""
convert_txt_trajectories.py  —  Inertial (IMU) Test
===================================================
Convert the MATLAB-style `Trajectories.txt` waypoint arrays into one importable
JSON file per trajectory, so they can be loaded with import_trajectory.py /
imu_live.py --json.

Input format (repeated blocks)::

    % ---- Circle r=45 start=0deg (near IMU3) ----
    x     = [161.90, 161.89, ...]
    y     = [ 45.00,  46.00, ...]
    theta = [  0.00,   1.00, ...]      % yaw angle, degrees

Each block becomes::

    {
      "name": "Circle r=45 start=0deg (near IMU3)",
      "units": "mm",
      "yaw_units": "deg",
      "source": "Trajectories.txt",
      "waypoints": [[x, y, theta], ...]
    }

The absolute (0..230 mm) coordinates are kept as-is here; recentring and
scaling into the 23 cm working area happens on import (import_trajectory.py),
which preserves the yaw column untouched.

Usage
-----
  python convert_txt_trajectories.py                       # Trajectories.txt → imported_trajectories/
  python convert_txt_trajectories.py Trajectories.txt -o out_dir
  python convert_txt_trajectories.py --list                # just list the blocks
"""
import os
import re
import sys
import json
import argparse

_HERE = os.path.dirname(os.path.abspath(__file__))

_HEADER_RE = re.compile(r'^%\s*-+\s*(.*?)\s*-+\s*$')
_ARRAY_RE  = re.compile(r'^(x|y|theta)\s*=\s*\[([^\]]*)\]', re.IGNORECASE)


def _sanitize(name):
    """Turn a trajectory title into a safe file-name stem."""
    stem = re.sub(r'[^0-9a-zA-Z]+', '_', name).strip('_').lower()
    return stem or 'trajectory'


def parse_blocks(text):
    """
    Parse the .txt into a list of dicts: {name, x[], y[], theta[]}.

    Arrays are matched by name so x/y/theta order within a block does not
    matter; a block is finalised when the next header appears.
    """
    blocks = []
    current = None

    def _finalise(b):
        if b and b.get('x') and b.get('y'):
            n = min(len(b['x']), len(b['y']),
                    len(b['theta']) if b.get('theta') else len(b['x']))
            b['n'] = n
            blocks.append(b)

    for line in text.splitlines():
        h = _HEADER_RE.match(line)
        if h:
            _finalise(current)
            current = {'name': h.group(1), 'x': [], 'y': [], 'theta': []}
            continue
        a = _ARRAY_RE.match(line)
        if a and current is not None:
            key = a.group(1).lower()
            nums = [float(v) for v in a.group(2).replace(',', ' ').split()]
            current[key] = nums
    _finalise(current)
    return blocks


def block_to_payload(b, source):
    """Build the JSON payload for one parsed block."""
    n = b['n']
    have_theta = len(b['theta']) >= n
    waypoints = []
    for i in range(n):
        wp = [round(b['x'][i], 4), round(b['y'][i], 4)]
        if have_theta:
            wp.append(round(b['theta'][i], 4))
        waypoints.append(wp)
    return {
        'name': b['name'],
        'units': 'mm',
        'yaw_units': 'deg' if have_theta else None,
        'source': source,
        'n_waypoints': n,
        'waypoints': waypoints,
    }


def parse_args():
    p = argparse.ArgumentParser(
        description='Convert Trajectories.txt waypoint arrays into JSON files',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('input', nargs='?', default=os.path.join(_HERE, 'Trajectories.txt'),
                   help='input .txt (default: ./Trajectories.txt)')
    p.add_argument('-o', '--out', default=os.path.join(_HERE, 'imported_trajectories'),
                   help='output directory (default: ./imported_trajectories)')
    p.add_argument('--list', action='store_true',
                   help='list the trajectory blocks and exit (no files written)')
    return p.parse_args()


def main():
    args = parse_args()
    if not os.path.isfile(args.input):
        print(f'[convert] File not found: {args.input}')
        sys.exit(1)

    with open(args.input, encoding='utf-8', errors='replace') as f:
        blocks = parse_blocks(f.read())

    if not blocks:
        print('[convert] No trajectory blocks found — check the file format.')
        sys.exit(1)

    source = os.path.basename(args.input)
    print(f'[convert] Parsed {len(blocks)} trajectories from {source}:')
    for b in blocks:
        yaw = 'x,y,yaw' if len(b['theta']) >= b['n'] else 'x,y'
        print(f'    {b["name"]:<40}  {b["n"]:>4d} pts  ({yaw})')

    if args.list:
        return

    os.makedirs(args.out, exist_ok=True)
    index = []
    for b in blocks:
        stem = _sanitize(b['name'])
        path = os.path.join(args.out, f'{stem}.json')
        # De-duplicate stems if two titles sanitise to the same name.
        k = 2
        while os.path.exists(path) and stem not in {os.path.splitext(os.path.basename(p))[0] for p in index}:
            path = os.path.join(args.out, f'{stem}_{k}.json')
            k += 1
        with open(path, 'w') as fp:
            json.dump(block_to_payload(b, source), fp, indent=2)
        index.append(path)

    print(f'\n[convert] Wrote {len(index)} JSON files → {args.out}/')
    print('[convert] Try:  python imu_live.py --json '
          f'{os.path.relpath(index[0], _HERE)} --no-robot')


if __name__ == '__main__':
    main()
