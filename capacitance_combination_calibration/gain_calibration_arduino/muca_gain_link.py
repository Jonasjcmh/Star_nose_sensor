"""
muca_gain_link.py — Star-Nose Sensor | muca board link WITH gain control
========================================================================
A serial owner for the muca board that does two things the existing readers
cannot:

  1. streams the raw frames (like ../../mucaboard_data_raw/sensor_raw.py), and
  2. SENDS commands back to the board — specifically `G<n>` to change the
     FT5316 analog gain (factory-mode register 0x07) between measurements.

Why a new module instead of extending sensor_raw.py
---------------------------------------------------
sensor_raw.py and Integration_2/sensor.py are read-only consumers of the port
and are used by the whole existing pipeline. Only ONE process can own a serial
port, and adding a write path there would change behaviour for every script
that imports them. This module is therefore self-contained and leaves both of
them untouched — it just keeps USED_CELLS in sync with sensor_raw.py, which
remains the source of truth for that mapping.

Requires firmware/Muca_Raw_gain/Muca_Raw_gain.ino on the board. Against the
ORIGINAL firmware everything still works except set_gain(), which will time
out waiting for the acknowledgement — that is the expected way to tell the two
sketches apart, and `probe_firmware()` uses exactly that.

Line protocol
-------------
  data      : 252 comma-separated integers, one line per frame
  replies   : lines starting with '#'  (from Muca_Raw_gain.ino)
  library   : lines starting with '['  (from the Muca library itself)
No reply or library line ever starts with a digit, so data and control never
collide.

NO PROCESSING is applied to the values — same contract as sensor_raw.py. The
first frame is kept as a reference baseline via get_baseline(), never
subtracted.
"""

import os
import sys
import glob
import time
import threading

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise ImportError("pyserial is required:  pip install pyserial")


SERIAL_RATE = 115200
SKIN_CELLS  = 252          # 21 TX x 12 RX, matches NUM_TX*NUM_RX on the board
N_CELLS     = 19           # the used subset, below

# Kept intentionally in sync with mucaboard_data_raw/sensor_raw.py.
# If that file's USED_CELLS ever changes, change it here too.
USED_CELLS = [
     0, 1, 2,
    12, 13, 14, 15,
    24, 25, 26, 27, 28,
    37, 38, 39, 40,
    50, 51, 52,
]

RAW_FULL_SCALE = 65535     # the board reports 16-bit words per cell

READ_TIMEOUT   = 2.0
ACK_TIMEOUT    = 5.0       # how long to wait for a '# GAIN ...' reply


def list_serial_ports():
    """(device, description) for every serial port, likely candidates first."""
    ports = [(p.device, p.description) for p in serial.tools.list_ports.comports()]

    # macOS sometimes hides the Arduino behind a /dev/cu.* that comports()
    # reports late; add anything obvious that is missing.
    for pattern in ('/dev/cu.usbmodem*', '/dev/cu.usbserial*',
                    '/dev/ttyACM*', '/dev/ttyUSB*'):
        for dev in glob.glob(pattern):
            if dev not in [d for d, _ in ports]:
                ports.append((dev, 'detected by glob'))

    def rank(item):
        dev, desc = item
        text = (dev + ' ' + desc).lower()
        for i, key in enumerate(('usbmodem', 'ttyacm', 'arduino', 'usbserial', 'ttyusb')):
            if key in text:
                return (i, dev)
        return (99, dev)

    return sorted(set(ports), key=rank)


def choose_port(preferred=None, interactive=True):
    """Resolve a port: explicit > single obvious candidate > ask the operator."""
    if preferred:
        return preferred

    ports = list_serial_ports()
    if not ports:
        raise RuntimeError("No serial ports found — is the muca board plugged in?")

    if not interactive:
        return ports[0][0]

    print("\nAvailable serial ports:")
    for i, (dev, desc) in enumerate(ports):
        print(f"  {i}: {dev}  -  {desc}")
    while True:
        raw = input("Select muca board port index [0]: ").strip()
        if not raw:
            return ports[0][0]
        try:
            return ports[int(raw)][0]
        except (ValueError, IndexError):
            print("  Please enter one of the indices listed above.")


class MucaGainLink:
    """Owns the muca serial port: streams frames, sends gain commands."""

    def __init__(self, port, baud=SERIAL_RATE):
        self.port = port
        self.baud = baud

        self._ser        = None
        self._thread     = None
        self._stop       = threading.Event()
        self._lock       = threading.Lock()

        self._values     = [0.0] * N_CELLS   # raw counts, 19 used cells
        self._full_frame = [0.0] * SKIN_CELLS
        self._baseline   = None
        self._ready      = False
        self._last_frame = 0.0
        self._frames     = 0

        self._replies    = []                # '#' lines, newest last
        self._reply_lock = threading.Lock()

        self.current_gain = None             # last gain we successfully set

    # -- lifecycle ---------------------------------------------------------

    def connect(self, wait_ready=True, timeout=30.0):
        self._ser = serial.Serial(self.port, self.baud,
                                  timeout=READ_TIMEOUT, write_timeout=2.0)
        time.sleep(2.0)          # the Arduino resets when the port opens
        self._ser.reset_input_buffer()

        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"[muca_link] Connected on {self.port}")

        if wait_ready and not self.wait_until_ready(timeout):
            raise RuntimeError(f"No frames from the muca board within {timeout:.0f}s")
        return self

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        print("[muca_link] Disconnected")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- reading -----------------------------------------------------------

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except Exception as e:
                print(f"[muca_link] Serial error: {e}")
                time.sleep(0.1)
                continue

            if not raw:
                continue

            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if not line[0].isdigit():
                # Firmware reply ('#') or library chatter ('[') — never data.
                with self._reply_lock:
                    self._replies.append(line)
                    if len(self._replies) > 200:
                        del self._replies[:100]
                print(f"[muca_link] {line}")
                continue

            parts = line.split(',')
            if len(parts) != SKIN_CELLS:
                continue                     # partial line, e.g. right after reset
            try:
                vals = [int(x) for x in parts]
            except ValueError:
                continue

            used = [float(vals[c]) for c in USED_CELLS]

            with self._lock:
                self._full_frame = [float(v) for v in vals]
                self._values     = used
                if self._baseline is None:
                    self._baseline = used[:]
                self._ready      = True
                self._last_frame = time.time()
                self._frames    += 1

    def wait_until_ready(self, timeout=30.0):
        t0 = time.time()
        while not self._ready:
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.05)
        return True

    def get_values(self):
        """The 19 used cells, pure raw counts, no processing."""
        with self._lock:
            return list(self._values)

    def get_full_frame(self):
        """All 252 cells, pure raw counts."""
        with self._lock:
            return list(self._full_frame)

    def get_baseline(self):
        with self._lock:
            return list(self._baseline) if self._baseline is not None else None

    def is_connected(self):
        return (time.time() - self._last_frame) < READ_TIMEOUT if self._last_frame else False

    def frame_count(self):
        with self._lock:
            return self._frames

    def flush_frames(self, settle_s):
        """Discard everything for settle_s — call after changing the gain."""
        t0 = time.time()
        while time.time() - t0 < settle_s:
            time.sleep(0.02)

    # -- commanding --------------------------------------------------------

    def _send(self, text):
        self._ser.write((text + "\n").encode('ascii'))
        self._ser.flush()

    def _wait_reply(self, prefix, timeout=ACK_TIMEOUT):
        """Wait for a '#' line starting with prefix. Returns it, or None."""
        deadline = time.time() + timeout
        seen = 0
        with self._reply_lock:
            seen = len(self._replies)
        while time.time() < deadline:
            with self._reply_lock:
                for line in self._replies[seen:]:
                    if line.startswith(prefix):
                        return line
                seen = len(self._replies)
            time.sleep(0.02)
        return None

    def set_gain(self, gain, settle_s=0.5):
        """Set the FT5316 analog gain (register 0x07). Returns the readback.

        Raises RuntimeError if the board does not acknowledge — which is what
        happens on the ORIGINAL firmware, since it has no command channel.
        """
        self._send(f"G{int(gain)}")
        reply = self._wait_reply("# GAIN")
        if reply is None:
            raise RuntimeError(
                f"No '# GAIN' acknowledgement for gain={gain}. "
                "Is firmware/Muca_Raw_gain/Muca_Raw_gain.ino flashed to the board?")

        readback = None
        tokens = reply.split()
        if "READBACK" in tokens:
            try:
                readback = int(tokens[tokens.index("READBACK") + 1])
            except (ValueError, IndexError):
                readback = None

        self.current_gain = int(gain)
        self.flush_frames(settle_s)
        return readback

    def query_gain(self):
        self._send("G?")
        return self._wait_reply("# GAIN")

    def set_report_rate(self, rate, settle_s=0.5):
        self._send(f"R{int(rate)}")
        reply = self._wait_reply("# RATE")
        self.flush_frames(settle_s)
        return reply

    def info(self):
        self._send("I")
        return self._wait_reply("# INFO")

    def probe_firmware(self, timeout=3.0):
        """'gain' if Muca_Raw_gain.ino is running, 'original' otherwise."""
        self._send("I")
        return "gain" if self._wait_reply("# INFO", timeout) else "original"


# ---------------------------------------------------------------------------
# Manual smoke test:  python muca_gain_link.py [--port /dev/cu.usbmodem1101]
# ---------------------------------------------------------------------------

def _main():
    import argparse
    ap = argparse.ArgumentParser(description="muca board link smoke test")
    ap.add_argument('--port', default=None)
    ap.add_argument('--gain', type=int, default=None,
                    help="optionally set this gain and show the effect")
    args = ap.parse_args()

    port = choose_port(args.port)
    link = MucaGainLink(port).connect()
    try:
        kind = link.probe_firmware()
        print(f"\nFirmware detected: {kind}")

        vals = link.get_values()
        print(f"19 used cells: min={min(vals):.0f} max={max(vals):.0f} "
              f"(full scale {RAW_FULL_SCALE})")

        if args.gain is not None:
            if kind != "gain":
                print("Cannot set gain — the original firmware is flashed.")
            else:
                rb = link.set_gain(args.gain, settle_s=1.0)
                vals = link.get_values()
                print(f"After gain={args.gain} (readback {rb}): "
                      f"min={min(vals):.0f} max={max(vals):.0f}")
    finally:
        link.close()


if __name__ == '__main__':
    _main()
