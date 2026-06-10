"""
load_calibration.py
Call this at the start of main.py to apply saved calibration.
Pass tip name to load a tip-specific profile (calib_<tip>.json).
"""
import json
import os
import ur5_control

CALIB_DIR = os.path.dirname(__file__)

def _calib_path(tip=None):
    name = f'calib_{tip}.json' if tip else 'calib.json'
    return os.path.join(CALIB_DIR, name)

def _calib_pts_path(tip=None):
    name = f'calib_points_{tip}.json' if tip else 'calib_points.json'
    return os.path.join(CALIB_DIR, name)

def preview(tip=None):
    """Print calibration details and ask the user to confirm before continuing.
    Returns True if confirmed, exits the process if rejected."""
    f_path = _calib_path(tip)
    tip_label = tip if tip else '(default)'
    print()
    print("=" * 50)
    print("  CALIBRATION CHECK — please confirm before moving")
    print("=" * 50)
    print(f"  Tip profile : {tip_label}")
    print(f"  File        : {os.path.basename(f_path)}")
    if os.path.exists(f_path):
        with open(f_path) as f:
            d = json.load(f)
        print(f"  X offset    : {d['x_mm']:+.3f} mm")
        print(f"  Y offset    : {d['y_mm']:+.3f} mm")
        print(f"  Z offset    : {d['z_mm']:+.3f} mm")
    else:
        print("  ⚠  File not found — zero offset will be used")
    print("=" * 50)
    try:
        answer = input("  Correct tip mounted? Continue? [y/N] > ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\n[calib] Aborted.")
        raise SystemExit(1)
    if answer != 'y':
        print("[calib] Aborted by user — check your tip and re-run.")
        raise SystemExit(1)
    print()

def apply(tip=None):
    label = f' [{tip}]' if tip else ''

    # ── Global X/Y/Z offset ───────────────────────────────────
    f_path = _calib_path(tip)
    if not os.path.exists(f_path):
        print(f"[calib] No calibration file found ({os.path.basename(f_path)}) — using zero offset")
        return
    with open(f_path) as f:
        d = json.load(f)
    ur5_control.set_calibration(
        x_mm=d['x_mm'],
        y_mm=d['y_mm'],
        z_mm=d['z_mm']
    )
    print(f"[calib] Global{label}: X={d['x_mm']:+.3f} Y={d['y_mm']:+.3f} Z={d['z_mm']:+.3f} mm")

    # ── Per-point offsets ─────────────────────────────────────
    pts_path = _calib_pts_path(tip)
    if os.path.exists(pts_path):
        with open(pts_path) as f:
            pd = json.load(f)
        offsets = {int(k): (v.get('dx_mm', 0.0), v.get('dy_mm', 0.0))
                   for k, v in pd.get('per_point', {}).items()}
        ur5_control.set_point_offsets(offsets)
    else:
        print(f"[calib] No per-point file ({os.path.basename(pts_path)}) — using global only")
