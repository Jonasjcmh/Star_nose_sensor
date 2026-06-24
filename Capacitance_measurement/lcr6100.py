"""
lcr6100.py - RS Pro LCR-6100 serial driver
===========================================
Connects via USB-serial and provides a thread-safe interface for
reading Cp/Rp measurements.

IMPORTANT: Configure the meter via its front panel before connecting:
  Function : Cp-Rp
  Frequency: 20 kHz
  Speed    : FAST
  Voltage  : 1.0 V

This driver does NOT send configuration commands on connect — they
cause E01 errors on the device display.  It only sends FETC? to
request measurements.

Serial protocol (LCR-6100 REV D8.06, confirmed by diagnostic)
--------------------------------------------------------------
  Baud  : 115200, 8-N-1, no flow control
  FETC? response: "+1.938e-12,+1.000e+20,OUT ,AUX-OK,NG<CR><LF>"
    field 0 : Cp  [F]
    field 1 : Rp  [Ohm]
    field 2 : status  "OK  " = in range | "OUT " = open / out of range
    field 3 : aux     "AUX-OK"
    field 4 : pass/fail  "OK" = good | "NG" = no good
"""

import math
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise ImportError("pyserial is required:  pip install pyserial")


def list_ports():
    """Return list of (device, description) for all detected serial ports."""
    ports = serial.tools.list_ports.comports()
    return [(p.device, p.description) for p in sorted(ports, key=lambda p: p.device)]


class LCR6100:

    BAUD    = 115200
    TIMEOUT = 1.0       # seconds — readline timeout

    def __init__(self, port, baud=None):
        self._port = port
        self._baud = baud if baud is not None else self.BAUD
        self._ser  = None

        self._ser_lock   = threading.Lock()
        self._cache_lock = threading.Lock()

        self._Cp = float('nan')
        self._Rp = float('nan')
        self._ok = False
        self._n  = 0

        self._stop   = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self):
        """Open the serial port and flush any leftover bytes."""
        self._ser = serial.Serial(
            port          = self._port,
            baudrate      = self._baud,
            bytesize      = serial.EIGHTBITS,
            parity        = serial.PARITY_NONE,
            stopbits      = serial.STOPBITS_ONE,
            timeout       = self.TIMEOUT,
            write_timeout = self.TIMEOUT,
            dsrdtr        = False,   # do not use DSR/DTR flow control
            rtscts        = False,   # do not use RTS/CTS flow control
            xonxoff       = False,   # do not use XON/XOFF flow control
        )
        # De-assert control lines so they don't disturb the device on a fresh
        # terminal open (OS toggles DTR/RTS which appears as framing noise)
        self._ser.dtr = False
        self._ser.rts = False
        time.sleep(0.5)              # let the line settle before any I/O
        self._ser.reset_input_buffer()
        print("[LCR] Connected to " + self._port + " at " + str(self._baud) + " baud")

    def disconnect(self):
        """Stop polling and close the port."""
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(b'LOCAL\r\n')   # return front-panel control to user
                time.sleep(0.05)
            except Exception:
                pass
            self._ser.close()
        print("[LCR] Disconnected")

    # ------------------------------------------------------------------
    # Measurement
    # ------------------------------------------------------------------

    def _fetch_raw(self):
        """
        Flush buffer, send FETC?, return the raw response string.
        Retries up to 3 times — the first response after a fresh port open
        can be a partial/garbage line due to line-settle timing.
        """
        with self._ser_lock:
            for _ in range(3):
                if self._ser.in_waiting:
                    self._ser.read(self._ser.in_waiting)
                self._ser.reset_input_buffer()

                self._ser.write(b'FETC?\r\n')

                raw  = self._ser.readline()
                line = raw.decode('ascii', errors='ignore').strip()

                # A valid FETC? response always has exactly 4 commas (5 fields)
                if line.count(',') == 4:
                    return line

            return ''

    def _parse(self, resp):
        """
        Parse the 5-field FETC? response.
        Returns (Cp_F, Rp_Ohm, ok).  Returns (nan, nan, False) on error.
        """
        try:
            parts = [p.strip() for p in resp.split(',')]
            if len(parts) < 5:
                return float('nan'), float('nan'), False
            Cp     = float(parts[0])
            Rp     = float(parts[1])
            status = parts[2]
            pf     = parts[4]
            ok = (status == 'OK') and (pf == 'OK')
            if not (math.isfinite(Cp) and math.isfinite(Rp)):
                return float('nan'), float('nan'), False
            return Cp, Rp, ok
        except (IndexError, ValueError):
            return float('nan'), float('nan'), False

    def measure(self):
        """
        Request and return one measurement.  Blocking (~10-50 ms).
        Returns (Cp_F, Rp_Ohm, ok).
        """
        return self._parse(self._fetch_raw())

    # ------------------------------------------------------------------
    # Background polling (for live visualiser / dataset collector)
    # ------------------------------------------------------------------

    def start_polling(self):
        """Start a daemon thread that continuously polls and caches readings."""
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._poll_loop, name='lcr6100-poll', daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while not self._stop.is_set():
            try:
                Cp, Rp, ok = self.measure()
                with self._cache_lock:
                    self._Cp = Cp
                    self._Rp = Rp
                    self._ok = ok
                    self._n += 1
            except Exception:
                time.sleep(0.05)

    def get_latest(self):
        """Return (Cp_F, Rp_Ohm, ok) from the most recent background reading."""
        with self._cache_lock:
            return self._Cp, self._Rp, self._ok

    @property
    def measurement_count(self):
        with self._cache_lock:
            return self._n

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    def get_Cp_pF(self):
        Cp, _, _ = self.get_latest()
        return Cp * 1e12

    def __repr__(self):
        Cp, Rp, ok = self.get_latest()
        return "LCR6100(" + self._port + ") Cp=" + str(round(Cp * 1e12, 3)) + " pF  ok=" + str(ok)


# ----------------------------------------------------------------------
# Standalone test
# ----------------------------------------------------------------------

if __name__ == '__main__':
    import sys

    ports = list_ports()
    if not ports:
        print("No serial ports found.")
        sys.exit(1)

    print("Available ports:")
    for i, (dev, desc) in enumerate(ports):
        print("  " + str(i) + ": " + dev + "  -  " + desc)

    try:
        idx = int(input("Select port index: ").strip())
        port = ports[idx][0]
    except (ValueError, IndexError, KeyboardInterrupt):
        sys.exit(1)

    lcr = LCR6100(port)
    lcr.connect()
    lcr.start_polling()

    print("\nPolling - Ctrl-C to stop\n")
    try:
        while True:
            Cp, Rp, ok = lcr.get_latest()
            print("\r  Cp = " + str(round(Cp * 1e12, 3)).rjust(10) +
                  " pF    ok=" + str(ok) +
                  "  n=" + str(lcr.measurement_count) + "   ", end='')
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        lcr.disconnect()
