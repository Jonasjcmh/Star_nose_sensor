"""
sensor_raw.py — Star-Nose Sensor | RAW muca-board reader
========================================================
A raw-data twin of Integration_2/sensor.py. It reads the muca board exactly
the same way (same serial port, same USED_CELLS mapping), but it does NO
processing whatsoever on the sensor values:

    • NO normalisation      (no  (raw - baseline) / SENSITIVITY)
    • NO clamping to [0, 1]
    • NO gamma / linearisation  (no  ** GAMMA)

`get_values()` therefore returns the *pure raw ADC counts* of the 19 used
cells, straight off the board. The first frame is still captured as the
calibration baseline, but it is only stored for reference (via
`get_calibration()`) — it is never subtracted from the live values.

Drop-in API-compatible with Integration_2/sensor.py so the raw collector can
`import sensor_raw as sensor`:
    start(), wait_until_ready(), get_values(), get_value_for_ur5_point(),
    is_ready(), is_connected(), get_connection_age()
plus the extra `get_calibration()` for the baseline frame.

The original Integration_2/sensor.py is left untouched.
"""
import serial
import threading
import time
import os

SERIAL_PORT     = '/dev/ttyACM0'
SERIAL_RATE     = 115200
SKIN_CELLS      = 252

# Same 19 used cells as Integration_2/sensor.py (kept in sync intentionally).
USED_CELLS      = [
     0, 1, 2,
    12, 13, 14, 15,
    24, 25, 26, 27, 28,
    37, 38, 39, 40,
    50, 51, 52,
]

READ_TIMEOUT    = 2.0
RECONNECT_FAST  = 0.3
RECONNECT_SLOW  = 3.0

_values         = [0]   * 19   # RAW ADC counts of the 19 used cells (no processing)
_calibration    = None         # baseline (first) frame — stored for reference only
_last_good      = [0]   * 19
_is_ready       = False
_connected      = False
_last_frame_t   = 0.0
_frame_count    = 0
_lock           = threading.Lock()
_thread         = None

# UR5 point number → index into the 19-value (USED_CELLS-ordered) array.
# NEW point-sensing mapping (source of truth: Integration_2/ur5_control.py +
# calibrate_points.py): points are numbered in a1..e5 grid order, so UR5 point
# N presses raw cell USED_CELLS[N-1] → its index in the 19-value array is simply
# N-1 (identity). This intentionally REPLACES the old non-sequential map that
# Integration_2/sensor.py still carries.
_UR5_TO_IDX = {n: n - 1 for n in range(1, 20)}


def start():
    global _thread
    if _thread is not None and _thread.is_alive():
        return
    _thread = threading.Thread(target=_read_loop, daemon=True)
    _thread.start()
    print(f"[sensor_raw] Started on {SERIAL_PORT} (RAW mode — no normalisation)")


def get_values():
    """Return the 19 pure raw ADC counts (as floats, for a uniform CSV)."""
    with _lock:
        return [float(v) for v in _values]


def get_raw():
    """Alias of get_values() — in raw mode the values are already raw counts."""
    return get_values()


def get_calibration():
    """Return the baseline (first) frame captured at startup, or None."""
    with _lock:
        return list(_calibration) if _calibration is not None else None


def get_value_for_ur5_point(point_number):
    idx = _UR5_TO_IDX.get(point_number, -1)
    with _lock:
        return float(_values[idx]) if 0 <= idx < 19 else 0.0


def is_ready():
    return _is_ready


def is_connected():
    return _connected and (time.time() - _last_frame_t) < READ_TIMEOUT


def get_connection_age():
    return time.time() - _last_frame_t if _last_frame_t > 0 else 999


def wait_until_ready(timeout=30):
    t0 = time.time()
    while not _is_ready:
        if time.time() - t0 > timeout:
            print("[sensor_raw] Timeout!")
            return False
        time.sleep(0.1)
    return True


def _find_port():
    if os.path.exists(SERIAL_PORT):
        return SERIAL_PORT
    for prefix in ['/dev/ttyACM', '/dev/ttyUSB']:
        for i in range(5):
            p = f"{prefix}{i}"
            if os.path.exists(p):
                print(f"[sensor_raw] Found: {p}")
                return p
    return SERIAL_PORT


def _set_low_latency(ser):
    try:
        import fcntl, ctypes
        buf = ctypes.create_string_buffer(72)
        fcntl.ioctl(ser.fd, 0x541E, buf)
        flags = ctypes.c_uint32.from_buffer(buf, 28).value
        flags |= 0x2000
        ctypes.c_uint32.from_buffer(buf, 28).value = flags
        fcntl.ioctl(ser.fd, 0x541F, buf)
        print("[sensor_raw] Low latency enabled")
    except Exception:
        pass


def _read_loop():
    global _calibration, _is_ready
    global _values, _last_good
    global _connected, _last_frame_t, _frame_count

    consecutive_errors = 0

    while True:
        port = _find_port()
        print(f"[sensor_raw] Connecting to {port}...")
        ser = None

        try:
            ser = serial.Serial(
                port, SERIAL_RATE,
                timeout=READ_TIMEOUT,
                write_timeout=1.0
            )
            ser.reset_input_buffer()
            _set_low_latency(ser)
            _connected         = True
            consecutive_errors = 0
            print("[sensor_raw] Connected!")

            while True:
                try:
                    raw_bytes = ser.readline()

                    if not raw_bytes:
                        age = get_connection_age()
                        if age > READ_TIMEOUT and _last_frame_t > 0:
                            consecutive_errors += 1
                            if consecutive_errors > 3:
                                print(f"[sensor_raw] Timeout {age:.1f}s")
                                break
                        continue

                    _last_frame_t      = time.time()
                    consecutive_errors = 0

                    try:
                        line = raw_bytes.decode('utf-8',
                                               errors='ignore').strip()
                    except Exception:
                        continue

                    if not line:
                        continue

                    if not line[0].isdigit():
                        print(f"[sensor_raw] {line}")
                        continue

                    try:
                        parts = line.split(',')
                        if len(parts) != SKIN_CELLS:
                            continue
                        vals = [int(x) for x in parts]
                    except ValueError:
                        continue

                    raw = [vals[USED_CELLS[i]] for i in range(19)]

                    # First frame → store baseline for reference ONLY.
                    # Live values are always the pure raw counts (never adjusted).
                    if _calibration is None:
                        with _lock:
                            _calibration = raw[:]
                            _values      = raw[:]
                            _last_good   = raw[:]
                            _is_ready    = True
                        print("[sensor_raw] Baseline captured — live (RAW)!")
                        continue

                    with _lock:
                        _values    = raw
                        _last_good = raw[:]

                    _frame_count += 1
                    if _frame_count % 300 == 0:
                        print(f"[sensor_raw] f={_frame_count} "
                              f"min={min(raw)} max={max(raw)}")

                except serial.SerialException as e:
                    print(f"[sensor_raw] Serial error: {e}")
                    consecutive_errors += 1
                    if consecutive_errors > 2:
                        break
                    time.sleep(0.1)

                except Exception as e:
                    print(f"[sensor_raw] Error: {e}")
                    time.sleep(0.05)

        except serial.SerialException as e:
            print(f"[sensor_raw] Cannot open {port}: {e}")
        except Exception as e:
            print(f"[sensor_raw] Unexpected: {e}")
        finally:
            _connected = False
            if ser:
                try: ser.close()
                except: pass
            if _is_ready:
                with _lock:
                    _values = list(_last_good)

        age = get_connection_age()
        delay = RECONNECT_FAST if (age < 5.0 and _is_ready) \
                else RECONNECT_SLOW
        print(f"[sensor_raw] Reconnecting in {delay:.1f}s...")
        time.sleep(delay)
