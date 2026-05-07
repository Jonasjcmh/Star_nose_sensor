"""
sensor.py
Robust serial reader with shared file for cross-process communication.
"""
import serial
import threading
import time
import json
import os

SERIAL_PORT     = '/dev/ttyACM0'
SERIAL_RATE     = 115200
SKIN_CELLS      = 252
USED_CELLS      = [
     2, 15, 28,
     1, 14, 27, 40,
     0, 13, 26, 39, 52,
    12, 25, 38, 51,
    24, 37, 50,
]
SENSITIVITY     = 30.0
GAMMA           = 0.5
READ_TIMEOUT    = 2.0
RECONNECT_FAST  = 0.3
RECONNECT_SLOW  = 3.0
SHARED_FILE     = '/tmp/kywo_sensor.json'

_values         = [0.0] * 19
_raw_values     = [0]   * 19
_last_good      = [0.0] * 19
_calibration    = None
_is_ready       = False
_connected      = False
_last_frame_t   = 0.0
_frame_count    = 0
_lock           = threading.Lock()
_thread         = None

def start():
    global _thread
    if _thread is not None and _thread.is_alive():
        return
    _thread = threading.Thread(target=_read_loop, daemon=True)
    _thread.start()
    _start_shared_writer()
    print(f"[sensor] Started on {SERIAL_PORT}")

def get_values():
    with _lock:
        return list(_values)

def get_raw():
    with _lock:
        return list(_raw_values)

def get_value_for_ur5_point(point_number):
    idx = point_number - 1
    with _lock:
        return _values[idx] if 0 <= idx < 19 else 0.0

def get_value_for_raw_cell(raw_cell):
    """Get value by raw sensor cell index"""
    if raw_cell in USED_CELLS:
        idx = USED_CELLS.index(raw_cell)
        with _lock:
            return _values[idx]
    return 0.0

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
            print("[sensor] Timeout!")
            return False
        time.sleep(0.1)
    return True

def recalibrate():
    global _calibration, _is_ready
    with _lock:
        _calibration = None
        _is_ready    = False
    print("[sensor] Recalibrating...")

def read_shared():
    """Read sensor data from shared file — for subprocess access"""
    try:
        if not os.path.exists(SHARED_FILE):
            return None
        age = time.time() - os.path.getmtime(SHARED_FILE)
        if age > 1.5:
            return None
        with open(SHARED_FILE) as f:
            return json.load(f)
    except Exception:
        return None

def _write_shared():
    """Write sensor state to shared file at 30Hz"""
    while True:
        try:
            with _lock:
                data = {
                    'values':    list(_values),
                    'ready':     _is_ready,
                    'connected': _connected,
                    'timestamp': time.time(),
                }
            tmp = SHARED_FILE + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f)
            os.replace(tmp, SHARED_FILE)
        except Exception:
            pass
        time.sleep(0.033)

def _start_shared_writer():
    t = threading.Thread(target=_write_shared, daemon=True)
    t.start()

def _find_port():
    if os.path.exists(SERIAL_PORT):
        return SERIAL_PORT
    for prefix in ['/dev/ttyACM', '/dev/ttyUSB']:
        for i in range(5):
            p = f"{prefix}{i}"
            if os.path.exists(p):
                print(f"[sensor] Found: {p}")
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
        print("[sensor] Low latency enabled")
    except Exception:
        pass

def _normalise(raw):
    return [
        max(0.0, min(
            ((raw[i] - _calibration[i]) / SENSITIVITY),
            1.0
        )) ** GAMMA
        for i in range(19)
    ]

def _read_loop():
    global _calibration, _is_ready
    global _values, _raw_values, _last_good
    global _connected, _last_frame_t, _frame_count

    consecutive_errors = 0

    while True:
        port = _find_port()
        print(f"[sensor] Connecting to {port}...")
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
            print("[sensor] Connected!")

            while True:
                try:
                    raw_bytes = ser.readline()

                    if not raw_bytes:
                        age = get_connection_age()
                        if age > READ_TIMEOUT and _last_frame_t > 0:
                            consecutive_errors += 1
                            if consecutive_errors > 3:
                                print(f"[sensor] Timeout {age:.1f}s")
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
                        print(f"[sensor] {line}")
                        continue

                    try:
                        parts = line.split(',')
                        if len(parts) != SKIN_CELLS:
                            continue
                        vals = [int(x) for x in parts]
                    except ValueError:
                        continue

                    raw = [vals[USED_CELLS[i]] for i in range(19)]

                    if _calibration is None:
                        _calibration = raw[:]
                        _is_ready    = True
                        print("[sensor] Calibration done — live!")
                        continue

                    normalised = _normalise(raw)

                    with _lock:
                        _values     = normalised
                        _raw_values = raw
                        _last_good  = normalised[:]

                    _frame_count += 1
                    if _frame_count % 300 == 0:
                        active = sum(1 for v in normalised if v > 0.05)
                        print(f"[sensor] f={_frame_count} "
                              f"active={active}/19 "
                              f"max={max(normalised):.3f}")

                except serial.SerialException as e:
                    print(f"[sensor] Serial error: {e}")
                    consecutive_errors += 1
                    if consecutive_errors > 2:
                        break
                    time.sleep(0.1)

                except Exception as e:
                    print(f"[sensor] Error: {e}")
                    time.sleep(0.05)

        except serial.SerialException as e:
            print(f"[sensor] Cannot open {port}: {e}")
        except Exception as e:
            print(f"[sensor] Unexpected: {e}")
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
        print(f"[sensor] Reconnecting in {delay:.1f}s...")
        time.sleep(delay)