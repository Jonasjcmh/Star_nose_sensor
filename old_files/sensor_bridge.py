"""
sensor_bridge.py
Reads capacitive sensor mat via serial.
Correct raw cell mapping: P1→S2, P2→S15 ... P19→S50
"""
import serial
import threading
import time

SERIAL_PORT  = '/dev/ttyACM0'
SERIAL_RATE  = 115200
SKIN_COLS    = 12
SKIN_ROWS    = 21
SKIN_CELLS   = SKIN_COLS * SKIN_ROWS  # 252

# Raw cell indices matching UR5 physical positions
# Order: P1→P19 (index 0→18 in this list)
USED_CELLS = [
     2, 15, 28,      # P1  P2  P3   (row +14mm)
     1, 14, 27, 40,  # P4  P5  P6  P7  (row +7mm)
     0, 13, 26, 39, 52,  # P8..P12 (row 0mm)
    12, 25, 38, 51,  # P13..P16 (row -7mm)
    24, 37, 50,      # P17 P18 P19 (row -14mm)
]

# UR5 point number (1-19) → index in USED_CELLS (0-18)
UR5_TO_IDX = {i+1: i for i in range(19)}

SENSITIVITY = 8.0

_values      = [0.0] * 19
_raw_values  = [0]   * 19   # raw uncalibrated readings
_calibration = None
_is_ready    = False
_lock        = threading.Lock()
_thread      = None

def start():
    global _thread
    if _thread is not None and _thread.is_alive():
        return
    _thread = threading.Thread(target=_read_loop, daemon=True)
    _thread.start()
    print(f"[sensor] Started on {SERIAL_PORT}")

def get_values():
    """Normalised 0.0-1.0 values, index = UR5 point index (0-based)"""
    with _lock:
        return list(_values)

def get_raw():
    """Raw sensor readings before calibration subtraction"""
    with _lock:
        return list(_raw_values)

def get_value_for_ur5_point(point_number):
    """Get normalised value for a specific UR5 point (1-19)"""
    idx = UR5_TO_IDX.get(point_number, 0)
    with _lock:
        return _values[idx]

def is_ready():
    return _is_ready

def wait_until_ready(timeout=30):
    t0 = time.time()
    while not _is_ready:
        if time.time() - t0 > timeout:
            print("[sensor] Timeout!")
            return False
        time.sleep(0.1)
    return True

def recalibrate():
    """Force a new baseline calibration on next frame"""
    global _calibration, _is_ready
    with _lock:
        _calibration = None
        _is_ready    = False
    print("[sensor] Recalibrating...")

def _read_loop():
    global _calibration, _is_ready, _values, _raw_values

    print(f"[sensor] Connecting to {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_RATE, timeout=5)
    except Exception as e:
        print(f"[sensor] Failed: {e}")
        return

    print("[sensor] Connected — waiting for device calibration...")
    frame_count = 0

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            if not line[0].isdigit():
                print(f"[sensor] {line}")
                continue

            vals = list(map(int, line.split(',')))
            if len(vals) != SKIN_CELLS:
                continue

            raw = [vals[USED_CELLS[i]] for i in range(19)]

            if _calibration is None:
                _calibration = raw[:]
                _is_ready    = True
                print("[sensor] Calibration done — live!")
                continue

            normalised = [
                max(0.0, min((raw[i] - _calibration[i]) / SENSITIVITY, 1.0))
                for i in range(19)
            ]

            with _lock:
                _values     = normalised
                _raw_values = raw

            frame_count += 1
            if frame_count % 200 == 0:
                active = sum(1 for v in normalised if v > 0.05)
                print(f"[sensor] frame={frame_count} active={active}/19 max={max(normalised):.3f}")

        except ValueError:
            continue
        except Exception as e:
            print(f"[sensor] Error: {e}")
            time.sleep(0.1)