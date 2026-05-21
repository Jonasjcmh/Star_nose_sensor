"""
load_calibration.py
Call this at the start of main.py to apply saved calibration.
"""
import json
import os
import ur5_control

CALIB_FILE = os.path.join(os.path.dirname(__file__), 'calib.json')

def apply():
    if not os.path.exists(CALIB_FILE):
        print("[calib] No calibration file found — using zero offset")
        return
    with open(CALIB_FILE) as f:
        d = json.load(f)
    ur5_control.set_calibration(
        x_mm=d['x_mm'],
        y_mm=d['y_mm'],
        z_mm=d['z_mm']
    )
    print(f"[calib] Applied: X={d['x_mm']:+.3f} Y={d['y_mm']:+.3f} Z={d['z_mm']:+.3f} mm")