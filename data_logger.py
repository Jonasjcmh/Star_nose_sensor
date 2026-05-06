"""
data_logger.py
Logs sensor + UR5 force/pose/state to CSV.
Filename includes user-defined label for dataset identification.
"""
import csv
import os
import time
import threading
from datetime import datetime

_rows     = []
_lock     = threading.Lock()
_running  = False
_filename = None

def ask_dataset_label():
    """Ask user for a dataset label before starting the session."""
    print("\n" + "="*55)
    print("  Dataset label")
    print("="*55)
    print("  Describe this recording session.")
    print("  Examples:")
    print("    ecoflex_flat_layer")
    print("    dragonskin_solid_domes")
    print("    ecoflex_empty_domes_v2")
    print("    calibration_test")
    print("-"*55)
    label = input("  Label: ").strip()
    if not label:
        label = "unlabelled"
    # Sanitize — replace spaces and special chars with underscore
    label = ''.join(c if c.isalnum() or c in '-_' else '_'
                    for c in label)
    print(f"  Label set: '{label}'")
    print("="*55 + "\n")
    return label

def build_filename(label, base_dir):
    """Build session filename with timestamp + label."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename  = f"session_{timestamp}_{label}.csv"
    os.makedirs(base_dir, exist_ok=True)
    return os.path.join(base_dir, filename)

def start(filename):
    global _running, _filename, _rows
    _filename = filename
    _rows     = []
    _running  = True
    print(f"[logger] Started → {os.path.basename(filename)}")

def stop():
    global _running
    _running = False
    _save()

def log(sensor_values, ur5_state):
    if not _running:
        return

    ft  = ur5_state.get('ft',  [0.0]*6)
    tcp = ur5_state.get('tcp', [0.0]*6)

    row = {
        'timestamp':    round(time.time(), 4),
        'datetime':     datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'ur5_point':    ur5_state.get('point',    ''),
        'ur5_pressing': int(ur5_state.get('pressing', False)),
        'ur5_done':     int(ur5_state.get('done',     False)),
        'tcp_x':  round(tcp[0], 5) if len(tcp) > 0 else 0,
        'tcp_y':  round(tcp[1], 5) if len(tcp) > 1 else 0,
        'tcp_z':  round(tcp[2], 5) if len(tcp) > 2 else 0,
        'fx':  round(ft[0], 4) if len(ft) > 0 else 0,
        'fy':  round(ft[1], 4) if len(ft) > 1 else 0,
        'fz':  round(ft[2], 4) if len(ft) > 2 else 0,
        'tx':  round(ft[3], 4) if len(ft) > 3 else 0,
        'ty':  round(ft[4], 4) if len(ft) > 4 else 0,
        'tz':  round(ft[5], 4) if len(ft) > 5 else 0,
    }

    for i, v in enumerate(sensor_values):
        row[f'cell_{i+1}'] = round(float(v), 4)

    with _lock:
        _rows.append(row)

def _save():
    if not _rows:
        print("[logger] No data to save")
        return
    os.makedirs(os.path.dirname(
        os.path.abspath(_filename)), exist_ok=True)
    fieldnames = list(_rows[0].keys())
    with open(_filename, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(_rows)
    print(f"[logger] Saved {len(_rows)} rows → {_filename}")

def get_row_count():
    with _lock:
        return len(_rows)