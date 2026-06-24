"""
lcr6100_diagnostic.py — RS Pro LCR-6100 connection diagnostics
===============================================================
Run this BEFORE using lcr6100.py to identify the correct:
  • serial port
  • baud rate
  • command syntax & terminator
  • response format

Steps
-----
  1. Auto-scan: tries every port × baud rate combination, sends '*IDN?'
     and shows raw bytes received.
  2. Interactive REPL: once you know port + baud, open a raw terminal
     to type commands and see the literal response (hex + ASCII).
  3. Patch helper: prints the exact lines to update in lcr6100.py.

Usage
-----
  python lcr6100_diagnostic.py           # full scan then REPL
  python lcr6100_diagnostic.py --repl    # skip scan, go straight to REPL
  python lcr6100_diagnostic.py --port /dev/ttyUSB0 --baud 9600 --repl
"""

import sys
import time
import argparse

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print('pyserial is required:  pip install pyserial')
    sys.exit(1)

# ── Candidate settings to probe ────────────────────────────────────────────────
BAUD_CANDIDATES = [9600, 19200, 38400, 57600, 115200]

# Commands to try during auto-scan (sent with each terminator variant)
PROBE_CMDS = [
    '*IDN?',   # SCPI identity
    'ID?',     # some GW Instek / RS Pro
    'VER?',    # firmware version
    'MEAS?',   # trigger + return measurement
    'FETC?',   # return last measurement (abbreviated)
]

TERMINATORS = [b'\r\n', b'\n', b'\r']
TERM_NAMES  = {b'\r\n': 'CRLF', b'\n': 'LF', b'\r': 'CR'}

# ── Helpers ────────────────────────────────────────────────────────────────────

def _hex_ascii(raw: bytes) -> str:
    """Format bytes as both hex and printable ASCII."""
    hexpart = ' '.join(f'{b:02X}' for b in raw)
    ascpart = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw)
    return f'{hexpart}   |   {ascpart!r}'


def _read_all(ser, timeout=0.4) -> bytes:
    """Read everything available within timeout seconds."""
    ser.timeout = timeout
    buf = b''
    t0  = time.time()
    while time.time() - t0 < timeout:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            t0 = time.time()   # reset idle timer on each chunk
    return buf


def _try_port_baud(port, baud, verbose=True):
    """
    Open port at baud, send each probe command with each terminator,
    and return (baud, terminator, cmd, response) for the first non-empty reply.
    Returns None if nothing responds.
    """
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.5,
                            bytesize=8, parity='N', stopbits=1,
                            rtscts=False, dsrdtr=False)
    except serial.SerialException as e:
        if verbose:
            print(f'      [open error] {e}')
        return None

    time.sleep(0.2)
    ser.reset_input_buffer()

    found = None
    for term in TERMINATORS:
        for cmd in PROBE_CMDS:
            payload = cmd.encode('ascii') + term
            try:
                ser.reset_input_buffer()
                ser.write(payload)
                time.sleep(0.3)
                resp = _read_all(ser, timeout=0.5)
                if resp:
                    found = (baud, term, cmd, resp)
                    if verbose:
                        print(f'      ✓  baud={baud}  term={TERM_NAMES[term]}  '
                              f'cmd={cmd!r}  →  {_hex_ascii(resp)}')
                    ser.close()
                    return found
            except Exception:
                pass

    ser.close()
    return None


# ── Step 1: Auto-scan ──────────────────────────────────────────────────────────

def auto_scan(target_port=None):
    ports = serial.tools.list_ports.comports()
    if not ports:
        print('  No serial ports detected. Check USB cable and driver.')
        return None

    if target_port:
        to_scan = [p for p in ports if p.device == target_port]
        if not to_scan:
            print(f'  Port {target_port} not found. Available:')
            for p in ports:
                print(f'    {p.device}  —  {p.description}')
            return None
    else:
        to_scan = list(ports)

    print(f'\n  Scanning {len(to_scan)} port(s) × {len(BAUD_CANDIDATES)} baud rates ...\n')

    for port_info in sorted(to_scan, key=lambda p: p.device):
        print(f'  ── {port_info.device}  ({port_info.description}) ──')
        for baud in BAUD_CANDIDATES:
            print(f'    baud={baud} ... ', end='', flush=True)
            result = _try_port_baud(port_info.device, baud, verbose=False)
            if result:
                baud_r, term, cmd, resp = result
                print(f'GOT RESPONSE')
                print(f'      cmd={cmd!r}  term={TERM_NAMES[term]}')
                print(f'      raw: {_hex_ascii(resp)}')
                return (port_info.device, baud_r, term, resp)
            else:
                print('no response')

    print('\n  No response from any port/baud combination.')
    print('  Check: USB cable seated? Device powered on? Correct USB driver?')
    return None


# ── Step 2: Interactive REPL ───────────────────────────────────────────────────

def interactive_repl(port, baud, default_term=b'\r\n'):
    print(f'\n{"="*60}')
    print(f'  Interactive REPL  |  {port}  @  {baud}')
    print(f'{"="*60}')
    print(f'  Type a command and press Enter to send.')
    print(f'  Prefix with  \\n  to use LF-only, \\r  for CR-only.')
    print(f'  Special commands:')
    print(f'    !scan          probe all baud rates on this port')
    print(f'    !baud <N>      switch to baud rate N')
    print(f'    !raw <hex>     send raw hex bytes (e.g. !raw 46 45 54 43 48 3F 0D 0A)')
    print(f'    !config        send all LCR config commands and show responses')
    print(f'    !meas          attempt FETCH? / MEAS? and show raw response')
    print(f'    q / quit       exit REPL')
    print()

    term = default_term

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.8,
                            bytesize=8, parity='N', stopbits=1)
    except serial.SerialException as e:
        print(f'  [error] Cannot open {port}: {e}')
        return

    time.sleep(0.3)
    ser.reset_input_buffer()
    print(f'  Port open. Default terminator: {TERM_NAMES[term]}\n')

    def send_read(payload_bytes, read_timeout=0.8):
        ser.reset_input_buffer()
        ser.write(payload_bytes)
        resp = _read_all(ser, timeout=read_timeout)
        return resp

    def show(sent, resp):
        print(f'  sent : {_hex_ascii(sent)}')
        if resp:
            print(f'  recv : {_hex_ascii(resp)}')
            # Try to decode as ASCII
            try:
                txt = resp.decode('ascii').strip()
                print(f'  text : {txt!r}')
            except Exception:
                pass
        else:
            print('  recv : (no response within timeout)')
        print()

    config_cmds = [
        'FUNC Cp-Rp',
        ':FUNC:IMP CPRP',
        'FREQ 20000',
        ':FREQ 20000',
        'SPEED FAST',
        ':APER FAST',
        'VOLT 1.000',
        ':VOLT:LEV 1.0',
    ]

    meas_cmds = ['FETC?', 'FETCH?', 'MEAS?', ':FETC?', ':FETCH?', 'READ?']

    while True:
        try:
            raw_input_str = input('  > ').strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not raw_input_str:
            continue

        if raw_input_str.lower() in ('q', 'quit', 'exit'):
            break

        # ── Special commands ──────────────────────────────────────────────────
        if raw_input_str == '!scan':
            print(f'  Scanning all baud rates on {port} ...')
            for b in BAUD_CANDIDATES:
                print(f'    {b} ... ', end='', flush=True)
                ser.close()
                r = _try_port_baud(port, b, verbose=False)
                if r:
                    print(f'RESPONSE  →  {_hex_ascii(r[3])}')
                else:
                    print('none')
                ser = serial.Serial(port, baudrate=baud, timeout=0.8,
                                    bytesize=8, parity='N', stopbits=1)
            continue

        if raw_input_str.startswith('!baud '):
            try:
                new_baud = int(raw_input_str.split()[1])
                ser.close()
                baud = new_baud
                ser = serial.Serial(port, baudrate=baud, timeout=0.8,
                                    bytesize=8, parity='N', stopbits=1)
                time.sleep(0.2)
                print(f'  Switched to {baud} baud.')
            except Exception as e:
                print(f'  Error: {e}')
            continue

        if raw_input_str.startswith('!raw '):
            try:
                hex_str = raw_input_str[5:].strip()
                payload = bytes(int(h, 16) for h in hex_str.split())
                resp    = send_read(payload)
                show(payload, resp)
            except Exception as e:
                print(f'  Error parsing hex: {e}')
            continue

        if raw_input_str == '!config':
            print('  Sending config commands (each with CRLF terminator):')
            for cmd in config_cmds:
                payload = cmd.encode('ascii') + b'\r\n'
                resp    = send_read(payload, read_timeout=0.5)
                print(f'    {cmd!r:30s} → {_hex_ascii(resp) if resp else "(no response)"}')
            continue

        if raw_input_str == '!meas':
            print('  Trying measurement commands:')
            for cmd in meas_cmds:
                for t in TERMINATORS:
                    payload = cmd.encode('ascii') + t
                    resp    = send_read(payload, read_timeout=1.0)
                    tag = f'{cmd}+{TERM_NAMES[t]}'
                    print(f'    {tag:20s} → {_hex_ascii(resp) if resp else "(no response)"}')
            continue

        # ── Normal command ────────────────────────────────────────────────────
        # Allow explicit terminator override: \n or \r at start
        if raw_input_str.startswith('\\n '):
            t       = b'\n'
            cmd_str = raw_input_str[3:]
        elif raw_input_str.startswith('\\r '):
            t       = b'\r'
            cmd_str = raw_input_str[3:]
        else:
            t       = term
            cmd_str = raw_input_str

        payload = cmd_str.encode('ascii') + t
        resp    = send_read(payload, read_timeout=1.0)
        show(payload, resp)

    ser.close()
    print('\n  REPL closed.')


# ── Step 3: Patch helper ───────────────────────────────────────────────────────

def print_patch(port, baud, term, fetch_cmd, func_cmd=None,
                freq_cmd=None, speed_cmd=None, volt_cmd=None):
    print()
    print('=' * 60)
    print('  UPDATE lcr6100.py with these values:')
    print('=' * 60)
    term_str = repr(term.decode('ascii'))
    print(f"""
In the LCR6100 class:

    BAUD    = {baud}
    _TERM   = {term_str}  # add this attribute and use in _write/_query

    # Replace command class attributes with:
    _CMD_FUNC  = {func_cmd!r  or "'FUNC Cp-Rp'  # VERIFY this"}
    _CMD_FREQ  = {freq_cmd!r  or "'FREQ 20000'  # VERIFY this"}
    _CMD_SPEED = {speed_cmd!r or "'SPEED FAST'  # VERIFY this"}
    _CMD_VOLT  = {volt_cmd!r  or "'VOLT 1.000'  # VERIFY this"}
    _CMD_FETCH = {fetch_cmd!r}

In _write() and _query(), change '\\\\r\\\\n' to {term_str}.
""")


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='LCR-6100 serial connection diagnostics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--port',  default=None,
                   help='Target port (default: scan all)')
    p.add_argument('--baud',  type=int, default=None,
                   help='Baud rate (default: auto-scan)')
    p.add_argument('--repl',  action='store_true',
                   help='Skip scan and go straight to interactive REPL')
    return p.parse_args()


def main():
    args = parse_args()

    print('=' * 60)
    print('  LCR-6100 Connection Diagnostic')
    print('=' * 60)

    # ── List ports ────────────────────────────────────────────────────────────
    all_ports = list(serial.tools.list_ports.comports())
    print(f'\n  Serial ports detected: {len(all_ports)}')
    for p in sorted(all_ports, key=lambda x: x.device):
        print(f'    {p.device:20s}  {p.description}')

    if not all_ports:
        print('\n  No ports found. Possible causes:')
        print('  • USB cable not connected')
        print('  • Missing USB driver (check lsusb)')
        print('  • Device not powered on')
        sys.exit(1)

    # ── Determine port + baud ─────────────────────────────────────────────────
    port = args.port
    baud = args.baud
    term = b'\r\n'

    if not args.repl:
        print()
        result = auto_scan(target_port=port)
        if result:
            port, baud, term, _ = result
            print(f'\n  ✓ Device found on {port} @ {baud} baud  '
                  f'(terminator: {TERM_NAMES[term]})')
        else:
            print('\n  Could not auto-detect device.')
            print('  You can still try the interactive REPL manually.')

    # ── Ask for port + baud if still unknown ─────────────────────────────────
    if port is None:
        port_choices = [p.device for p in sorted(all_ports, key=lambda x: x.device)]
        print('\n  Available ports:')
        for i, pn in enumerate(port_choices):
            print(f'    [{i}]  {pn}')
        try:
            idx  = int(input('  Select port index: ').strip())
            port = port_choices[idx]
        except (ValueError, IndexError, EOFError, KeyboardInterrupt):
            print('  Invalid — exiting.')
            sys.exit(1)

    if baud is None:
        raw = input(f'  Baud rate [9600]: ').strip()
        baud = int(raw) if raw else 9600

    # ── Interactive REPL ──────────────────────────────────────────────────────
    print()
    try:
        go = input('  Open interactive REPL? [Y/n] > ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        go = 'n'

    if go != 'n':
        interactive_repl(port, baud, default_term=term)

    # ── Patch helper ─────────────────────────────────────────────────────────
    print()
    try:
        show_patch = input('  Show lcr6100.py patch instructions? [y/N] > ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        show_patch = 'n'

    if show_patch == 'y':
        fetch_cmd = input('  Confirmed FETCH command (e.g. FETCH? or MEAS?): ').strip()
        print_patch(port, baud, term, fetch_cmd)

    print('\n[done]')


if __name__ == '__main__':
    main()
