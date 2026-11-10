"""
lcr6100.py - RS Pro / GW Instek LCR-6100 serial driver
=======================================================
Connects via USB-serial (CDC virtual COM port) and provides a thread-safe
interface for configuring the meter and reading measurements.

Command set: GW Instek LCR-6000 Series User Manual, chapter 11
(the RS Pro LCR-6100 is the same instrument, firmware REV D8.06 confirmed).

Measurement-mode configuration (FIXED)
--------------------------------------
Earlier versions did not configure the meter at all because "config commands
cause E01 errors on the device display".  E01 = "Bad command".  The causes were:

  * the commands tried were not in the LCR-6000 command set
    (':FUNC:IMP CPRP', ':FREQ', ':APER', ':VOLT:LEV' with a leading colon ...);
  * on disconnect the driver sent 'LOCAL', which does not exist on this meter
    (the correct command is 'UNLOCK' / 'SYST:KEYLOCK OFF');
  * commands were never read back, so a rejected setting went unnoticed.

The valid commands (manual section 11.7-11.14) are now used, each one is sent
on its own line (no ';' chaining), and every setting is read back and
verified.  A wrong mode raises LCRConfigError instead of silently logging the
wrong quantity.

    FUNC Cp-Rp            measurement function  (FUNC?)
    FREQ 20000            test frequency in Hz  (FREQ?)    no 'Hz' suffix
    VOLT 1.0              test signal level, V  (VOLT?)    no 'V'  suffix
    APER FAST             speed SLOW|MED|FAST   (APER?  -> 'fast,0')
    APER <n>              averaging 0..256
    FUNC:RANG:AUTO ON     auto range            (FUNC:RANG:AUTO?)
    TRIG:SOUR INT         continuous internal trigger
    DISP:PAGE MEAS        FREQ/VOLT are refused on LIST / CORRECTION pages

Serial protocol
---------------
  Baud  : 115200, 8-N-1, no flow control  (SYSTEM page -> BAUD)
  EOL   : must match SYSTEM page -> TERMINATOR (LF | CR | CR+LF).
          Our meter answers with CR+LF, so the default is 'crlf'.
  FETC? : "+1.938e-12,+1.000e+20,OUT ,AUX-OK,NG"
            field 0 : primary   (Cp [F] in Cp-Rp)
            field 1 : secondary (Rp [Ohm] in Cp-Rp)   - absent in DCR mode
            field 2 : status  "OK" | "OUT" (open / out of range) | BINn
            field 3 : aux     "AUX-OK"
            field 4 : pass/fail "OK" | "NG"

Backwards compatibility
-----------------------
LCR6100(port).connect() + start_polling() + get_latest() works exactly as
before.  get_latest() returns (primary, secondary, ok), i.e. (Cp, Rp, ok)
in the default Cp-Rp mode.  Pass config=... (or use add_lcr_args /
lcr_from_args in a collector) to have the mode set on connect.
"""

import math
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise ImportError("pyserial is required:  pip install pyserial")


# ----------------------------------------------------------------------
# Measurement functions:  canonical name -> (primary, secondary, units)
# ----------------------------------------------------------------------
FUNCTIONS = {
    'Cs-Rs': ('Cs', 'Rs', ('F', 'Ohm')),
    'Cs-D':  ('Cs', 'D',  ('F', '')),
    'Cp-Rp': ('Cp', 'Rp', ('F', 'Ohm')),
    'Cp-D':  ('Cp', 'D',  ('F', '')),
    'Lp-Rp': ('Lp', 'Rp', ('H', 'Ohm')),
    'Lp-Q':  ('Lp', 'Q',  ('H', '')),
    'Ls-Rs': ('Ls', 'Rs', ('H', 'Ohm')),
    'Ls-Q':  ('Ls', 'Q',  ('H', '')),
    'Rs-Q':  ('Rs', 'Q',  ('Ohm', '')),
    'Rp-Q':  ('Rp', 'Q',  ('Ohm', '')),
    'R-X':   ('R',  'X',  ('Ohm', 'Ohm')),
    'Z-thr': ('Z',  'theta', ('Ohm', 'rad')),
    'Z-thd': ('Z',  'theta', ('Ohm', 'deg')),
    'Z-D':   ('Z',  'D',  ('Ohm', '')),
    'Z-Q':   ('Z',  'Q',  ('Ohm', '')),
    'DCR':   ('DCR', None, ('Ohm', None)),
}
_FUNC_LOOKUP = {k.lower().replace('-', ''): k for k in FUNCTIONS}

SPEEDS = ('SLOW', 'MED', 'FAST')
EOLS = {'lf': b'\n', 'cr': b'\r', 'crlf': b'\r\n'}

# Settings used for all Star-nose capacitance work so far
DEFAULT_CONFIG = {
    'function': 'Cp-Rp',
    'freq_hz':  20000.0,
    'volt_v':   1.0,
    'speed':    'FAST',
    'avg':      None,      # None = leave as is, 0/1 = off, up to 256
    'auto_range': True,
}

OVERFLOW = 9.0e19          # meter reports +/-1e20 when it cannot measure


class LCRConfigError(RuntimeError):
    """The meter refused or did not apply a requested setting."""


def normalize_function(name):
    """'cprp', 'CP-RP', 'Cp-Rp' -> 'Cp-Rp'.  Raises ValueError if unknown."""
    key = str(name).strip().lower().replace('-', '').replace('_', '')
    key = key.replace('θ', 'th')            # Z-θr typed literally
    if key not in _FUNC_LOOKUP:
        raise ValueError("unknown LCR function '" + str(name) + "'. Valid: " +
                         ', '.join(FUNCTIONS))
    return _FUNC_LOOKUP[key]


def list_ports():
    """Return list of (device, description) for all detected serial ports."""
    ports = serial.tools.list_ports.comports()
    return [(p.device, p.description) for p in sorted(ports, key=lambda p: p.device)]


class LCR6100:

    BAUD    = 115200
    TIMEOUT = 1.0       # seconds — read timeout

    def __init__(self, port, baud=None, eol='crlf', config=None, verbose=True):
        self._port = port
        self._baud = baud if baud is not None else self.BAUD
        if eol not in EOLS:
            raise ValueError("eol must be one of " + ', '.join(EOLS))
        self._eol  = EOLS[eol]
        self._ser  = None
        self._verbose = verbose

        self.config = None if config is None else dict(config)
        self.applied_config = {}         # read back from the meter
        self.function = 'Cp-Rp'          # assumed until configured / queried

        self._ser_lock   = threading.Lock()
        self._cache_lock = threading.Lock()

        self._P  = float('nan')
        self._S  = float('nan')
        self._ok = False
        self._t  = float('nan')
        self._n  = 0

        self._stop   = threading.Event()
        self._thread = None

    def _log(self, msg):
        if self._verbose:
            print("[LCR] " + msg)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self):
        """Open the serial port, flush, and apply `config` if one was given."""
        self._ser = serial.Serial(
            port          = self._port,
            baudrate      = self._baud,
            bytesize      = serial.EIGHTBITS,
            parity        = serial.PARITY_NONE,
            stopbits      = serial.STOPBITS_ONE,
            timeout       = self.TIMEOUT,
            write_timeout = self.TIMEOUT,
            dsrdtr        = False,
            rtscts        = False,
            xonxoff       = False,
        )
        # De-assert control lines so they don't disturb the device on a fresh
        # open (OS toggles DTR/RTS which appears as framing noise)
        self._ser.dtr = False
        self._ser.rts = False
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._log("Connected to " + self._port + " at " + str(self._baud) + " baud")

        if self.config is not None:
            self.configure(**self.config)
        else:
            # No config requested: just learn the current mode for labelling.
            try:
                self.function = normalize_function(self.query('FUNC?'))
            except Exception:
                pass

    def disconnect(self):
        """Stop polling, give the keypad back to the user and close the port."""
        self.stop_polling()
        if self._ser and self._ser.is_open:
            try:
                with self._ser_lock:
                    self._write_raw('UNLOCK')   # NOT 'LOCAL' (=> E01)
                time.sleep(0.05)
            except Exception:
                pass
            self._ser.close()
        self._log("Disconnected")

    # ------------------------------------------------------------------
    # Low-level I/O  (caller must hold _ser_lock for _*_raw)
    # ------------------------------------------------------------------

    def _write_raw(self, cmd):
        self._ser.write(cmd.encode('ascii') + self._eol)
        self._ser.flush()

    def _readline_raw(self):
        """Read one response line.  Accepts LF, CR or CR+LF endings."""
        end = b'\r' if self._eol == b'\r' else b'\n'
        raw = self._ser.read_until(end)
        return raw.decode('ascii', errors='ignore').strip()

    def _flush_input(self):
        if self._ser.in_waiting:
            self._ser.read(self._ser.in_waiting)
        self._ser.reset_input_buffer()

    def write(self, cmd, settle_s=0.15):
        """Send a set-command (no response expected)."""
        with self._ser_lock:
            self._flush_input()
            self._write_raw(cmd)
            time.sleep(settle_s)

    def query(self, cmd, retries=2):
        """Send a query and return the response line ('' on timeout)."""
        with self._ser_lock:
            for _ in range(retries + 1):
                self._flush_input()
                self._write_raw(cmd)
                line = self._readline_raw()
                # Hand-shake mode echoes the command first - skip the echo
                if line.lower().startswith(cmd.lower()):
                    line = line[len(cmd):].strip() or self._readline_raw()
                if line:
                    return line
            return ''

    _query = query          # visualize_lcr.py calls _query()

    def idn(self):
        return self.query('*IDN?')

    def error(self):
        """Last error string from the meter ('no error.' when fine)."""
        return self.query('ERR?')

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def configure(self, function=None, freq_hz=None, volt_v=None, speed=None,
                  avg=None, auto_range=None, trigger_internal=True, verify=True):
        """
        Set the measurement mode and read every setting back.

        Any argument left as None is not touched.  Raises LCRConfigError if
        the meter does not report the requested value afterwards.
        Safe to call while background polling is running.
        """
        failures = []

        def check(name, cmd, query, expected, compare):
            self.write(cmd, settle_s=0.3 if name == 'function' else 0.15)
            if not verify:
                return
            got = self.query(query)
            try:
                good = compare(got)
            except Exception:
                good = False
            self.applied_config[name] = got
            if not good:
                failures.append(name + ": sent '" + cmd + "', meter reports '" +
                                got + "' (expected " + str(expected) + ")")

        # FREQ/VOLT are rejected on LIST/CORRECTION pages; DISP:PAGE must be
        # sent alone because it takes long to execute (manual 11.6.2).
        self.write('DISP:PAGE MEAS', settle_s=0.6)

        if trigger_internal:
            check('trigger', 'TRIG:SOUR INT', 'TRIG:SOUR?', 'INT',
                  lambda g: g.strip().upper().startswith('INT'))

        if function is not None:
            fn = normalize_function(function)
            # The meter accepts either 'Z-thr' or 'Z-\xe9r'; send ASCII form
            check('function', 'FUNC ' + fn, 'FUNC?', fn,
                  lambda g: normalize_function(g) == fn)
            self.function = fn

        if freq_hz is not None:
            f = float(freq_hz)
            check('freq_hz', 'FREQ ' + _fmt_num(f), 'FREQ?', f,
                  lambda g: abs(_num(g) - f) <= max(1e-3 * f, 0.5))

        if volt_v is not None:
            v = float(volt_v)
            check('volt_v', 'VOLT ' + _fmt_num(v), 'VOLT?', v,
                  lambda g: abs(_num(g) - v) <= max(0.01 * v, 0.005))

        if speed is not None:
            sp = str(speed).strip().upper()
            sp = 'MED' if sp.startswith('MED') else sp
            if sp not in SPEEDS:
                raise ValueError("speed must be SLOW, MED or FAST")
            check('speed', 'APER ' + sp, 'APER?', sp,
                  lambda g: g.split(',')[0].strip().upper().startswith(sp[:3]))

        if avg is not None:
            n = int(avg)
            if not 0 <= n <= 256:
                raise ValueError("avg must be 0..256")
            check('avg', 'APER ' + str(n), 'APER:AVG?', n,
                  lambda g: int(_num(g)) in ((0, 1) if n <= 1 else (n,)))

        if auto_range is not None:
            want = 'AUTO' if auto_range else 'HOLD'
            check('auto_range', 'FUNC:RANG:AUTO ' + ('ON' if auto_range else 'OFF'),
                  'FUNC:RANG:AUTO?', want,
                  lambda g: g.strip().upper() in ((want, 'ON') if auto_range
                                                  else (want, 'OFF')))

        # Discard measurements taken under the old settings
        with self._cache_lock:
            self._P = self._S = float('nan')
            self._ok = False

        if failures:
            err = ''
            try:
                err = self.error()
            except Exception:
                pass
            raise LCRConfigError("LCR configuration not applied:\n  " +
                                 "\n  ".join(failures) +
                                 ("\n  meter ERR?: " + err if err else ''))

        self._log("Configured: " + self.describe())
        return dict(self.applied_config)

    def describe(self):
        """Compact read-back summary, e.g. 'Cp-Rp | 20 kHz | 1 V | FAST'."""
        a = self.applied_config
        parts = ["Mode: " + self.function]
        try:
            f = _num(a['freq_hz'])
            parts.append(('%g kHz' % (f / 1e3)) if f >= 1e3 else ('%g Hz' % f))
        except (KeyError, ValueError):
            pass
        try:
            parts.append('%g V' % _num(a['volt_v']))
        except (KeyError, ValueError):
            pass
        if 'speed' in a:
            parts.append(a['speed'].split(',')[0].strip().upper())
        if len(parts) == 1:
            parts.append("front-panel settings")
        return " | ".join(parts)

    @property
    def labels(self):
        """(primary_name, secondary_name) for the current function."""
        p, s, _ = FUNCTIONS.get(self.function, ('primary', 'secondary', None))
        return p, s

    # ------------------------------------------------------------------
    # Measurement
    # ------------------------------------------------------------------

    def _fetch_raw(self):
        """
        Send FETC? and return a response line that starts with a number.
        Retries — the first line after a fresh port open can be garbage.
        """
        with self._ser_lock:
            for _ in range(3):
                self._flush_input()
                self._write_raw('FETC?')
                line = self._readline_raw()
                if line and _is_number(line.split(',')[0]):
                    return line
            return ''

    @staticmethod
    def _parse(resp):
        """
        Parse a FETC? response.  Returns (primary, secondary, ok).
        secondary is nan in DCR mode.  (nan, nan, False) on error.
        """
        try:
            parts = [p.strip() for p in resp.split(',')]
            nums = []
            for p in parts:
                if _is_number(p) and len(nums) < 2:
                    nums.append(float(p))
                else:
                    break
            if not nums:
                return float('nan'), float('nan'), False
            flags = [p.upper() for p in parts[len(nums):]]
            P = nums[0]
            S = nums[1] if len(nums) > 1 else float('nan')
            ok = (math.isfinite(P) and abs(P) < OVERFLOW
                  and 'OUT' not in flags
                  and (not flags or flags[-1] != 'NG'))
            if len(nums) > 1 and not (math.isfinite(S) and abs(S) < OVERFLOW):
                ok = False
            return P, S, ok
        except (IndexError, ValueError):
            return float('nan'), float('nan'), False

    def measure(self):
        """
        Request and return one measurement.  Blocking (~10-50 ms).
        Returns (primary, secondary, ok)  ->  (Cp_F, Rp_Ohm, ok) in Cp-Rp.
        """
        return self._parse(self._fetch_raw())

    # ------------------------------------------------------------------
    # Background polling (for live visualiser / dataset collectors)
    # ------------------------------------------------------------------

    def start_polling(self):
        """Start a daemon thread that continuously polls and caches readings."""
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._poll_loop, name='lcr6100-poll', daemon=True)
        self._thread.start()

    def stop_polling(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _poll_loop(self):
        while not self._stop.is_set():
            try:
                P, S, ok = self.measure()
                t = time.time()
                with self._cache_lock:
                    self._P, self._S, self._ok, self._t = P, S, ok, t
                    self._n += 1
            except Exception:
                time.sleep(0.05)

    def get_latest(self):
        """Return (primary, secondary, ok) from the latest background reading."""
        with self._cache_lock:
            return self._P, self._S, self._ok

    def get_sample(self):
        """Latest reading as dict: t (unix s), primary, secondary, ok, n."""
        with self._cache_lock:
            return {'t': self._t, 'primary': self._P, 'secondary': self._S,
                    'ok': self._ok, 'n': self._n}

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
        P, S, ok = self.get_latest()
        return ("LCR6100(" + self._port + ", " + self.function + ") primary=" +
                str(P) + " secondary=" + str(S) + " ok=" + str(ok))


def _is_number(s):
    try:
        float(s)
        return True
    except (TypeError, ValueError):
        return False


def _num(s):
    """Parse a read-back number; the meter appends units ('1.000e+00V')."""
    s = str(s).strip()
    while s and not (s[-1].isdigit() or s[-1] == '.'):
        s = s[:-1]
    return float(s)


def _fmt_num(x):
    """20000.0 -> '20000', 0.3 -> '0.3'  (meter rejects unit suffixes)."""
    return ('%.6g' % x)


# ----------------------------------------------------------------------
# argparse helpers shared by all collectors
# ----------------------------------------------------------------------

def add_lcr_args(parser):
    """Add --lcr-func/--lcr-freq/--lcr-volt/--lcr-speed/... to a parser."""
    g = parser.add_argument_group('LCR-6100 measurement mode')
    g.add_argument('--lcr-func', default=DEFAULT_CONFIG['function'],
                   help="Measurement function, e.g. Cp-Rp, Cp-D, Cs-Rs, Z-thd "
                        "[default Cp-Rp]")
    g.add_argument('--lcr-freq', type=float, default=DEFAULT_CONFIG['freq_hz'],
                   help='Test frequency Hz [20000]')
    g.add_argument('--lcr-volt', type=float, default=DEFAULT_CONFIG['volt_v'],
                   help='Test signal level V [1.0]')
    g.add_argument('--lcr-speed', default=DEFAULT_CONFIG['speed'],
                   choices=['SLOW', 'MED', 'FAST', 'slow', 'med', 'fast'],
                   help='Measurement speed [FAST]')
    g.add_argument('--lcr-avg', type=int, default=None,
                   help='Averaging 0..256 [leave unchanged]')
    g.add_argument('--lcr-eol', default='crlf', choices=list(EOLS),
                   help='Must match SYSTEM > TERMINATOR on the meter [crlf]')
    g.add_argument('--lcr-no-config', action='store_true',
                   help='Do not send settings; use whatever the front panel has')
    return parser


def lcr_config_from_args(args):
    if getattr(args, 'lcr_no_config', False):
        return None
    return {
        'function':   getattr(args, 'lcr_func', DEFAULT_CONFIG['function']),
        'freq_hz':    getattr(args, 'lcr_freq', DEFAULT_CONFIG['freq_hz']),
        'volt_v':     getattr(args, 'lcr_volt', DEFAULT_CONFIG['volt_v']),
        'speed':      getattr(args, 'lcr_speed', DEFAULT_CONFIG['speed']),
        'avg':        getattr(args, 'lcr_avg', None),
        'auto_range': True,
    }


def lcr_from_args(port, args, baud=None, warn_columns=True):
    """Build an LCR6100 from parsed add_lcr_args() options (not yet connected)."""
    cfg = lcr_config_from_args(args)
    if (warn_columns and cfg is not None
            and normalize_function(cfg['function']) != 'Cp-Rp'):
        print("[LCR] NOTE: function is " + normalize_function(cfg['function']) +
              " - CSV columns 'Cp_F'/'Rp_Ohm' will hold the primary/secondary "
              "values of that function.")
    return LCR6100(port, baud=baud, eol=getattr(args, 'lcr_eol', 'crlf'),
                   config=cfg)


# ----------------------------------------------------------------------
# Standalone test:  python lcr6100.py [--lcr-func Cp-D ...]
# ----------------------------------------------------------------------

if __name__ == '__main__':
    import argparse
    import sys

    ap = argparse.ArgumentParser(description='LCR-6100 quick test')
    ap.add_argument('--port', default=None)
    add_lcr_args(ap)
    args = ap.parse_args()

    port = args.port
    if port is None:
        ports = list_ports()
        if not ports:
            print("No serial ports found.")
            sys.exit(1)
        print("Available ports:")
        for i, (dev, desc) in enumerate(ports):
            print("  " + str(i) + ": " + dev + "  -  " + desc)
        try:
            port = ports[int(input("Select port index: ").strip())][0]
        except (ValueError, IndexError, KeyboardInterrupt, EOFError):
            sys.exit(1)

    lcr = lcr_from_args(port, args)
    try:
        lcr.connect()
    except LCRConfigError as e:
        print(e)
        lcr.disconnect()
        sys.exit(1)
    print("[LCR] IDN: " + lcr.idn())
    lcr.start_polling()
    pn, sn = lcr.labels

    print("\nPolling - Ctrl-C to stop\n")
    try:
        while True:
            P, S, ok = lcr.get_latest()
            print("\r  " + pn + " = " + ('%.5g' % P).rjust(12) + "   " +
                  str(sn) + " = " + ('%.5g' % S).rjust(12) +
                  "   ok=" + str(ok) + "  n=" + str(lcr.measurement_count) + "   ",
                  end='')
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        lcr.disconnect()
