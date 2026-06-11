#!/usr/bin/env python3
"""
measure_resistance_live.py
──────────────────────────
Live resistance measurement using the PalmSens MethodSCRIPT SDK.

Shows two live plots side by side as data arrives point-by-point:
  LEFT  →  I vs V   (builds up as the sweep runs)
  RIGHT →  R vs V   (R = V/I per point, flat line = good resistor)

At the end, prints EmStat R vs multimeter R and % difference.

REQUIREMENTS:
  pip install pyserial numpy matplotlib

FOLDER STRUCTURE:
  palmsens_resistance/
  ├── palmsens/
  │   ├── __init__.py        ← empty file
  │   ├── instrument.py
  │   ├── mscript.py
  │   └── serialport.py
  └── measure_resistance_live.py   ← THIS FILE
"""

import sys
import logging
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import palmsens.instrument
import palmsens.mscript
import palmsens.serialport
from palmsens.instrument import CommunicationTimeout

# ══════════════════════════════════════════════════════
#  CONFIG — only edit this block
# ══════════════════════════════════════════════════════
# Set to None to auto-detect the EmStat USB port (works on Mac/Linux/Windows).
# Or set explicitly, e.g. '/dev/tty.usbmodem1101' on Mac, 'COM5' on Windows.
DEVICE_PORT  = None      # None = auto-detect
BAUD_RATE    = None      # None = auto-detect (921600 EmStat4 | 230400 Pico)

MULTIMETER_R = 99.8      # Ω — your multimeter reading for comparison

# LSV sweep settings
V_BEGIN    = -0.5        # V
V_END      =  0.5        # V
V_STEP     =  0.005      # V  (5 mV)
SCAN_RATE  =  0.1        # V/s  (100 mV/s)

# Current range for EmStat4:
#   Available: 1nA, 10nA, 100nA, 1uA, 10uA, 100uA, 1mA, 10mA, 100mA
#   Your resistor: 0.5V / 99.8Ω ≈ 5mA max → use 10mA range (next one up)
AUTORANGE_MAX = '10m'    # 10 mA  — matches PSTrace's "10mA" option
AUTORANGE_MIN = '1u'     # 1 µA

# Seconds to wait per line before retrying (increase for very slow scan rates)
LINE_TIMEOUT_S = 10.0

# ══════════════════════════════════════════════════════
#  METHODSCRIPT — LSV
# ══════════════════════════════════════════════════════
# TIP: Instead of this inline script, you can export directly from PSTrace:
#   Method menu → Export to MethodSCRIPT → save as lsv_resistance.mscr
# Then swap device.write(LSV_SCRIPT) for device.send_script('lsv_resistance.mscr')
#
# Column mapping (from mscript.py VarType IDs):
#   pck_add p  →  column 0  →  type 'da'  (Applied potential, V)
#   pck_add c  →  column 1  →  type 'ba'  (WE current, A)
LSV_SCRIPT = (
    "var c\n"
    "var p\n"
    "set_pgstat_mode 0\n"
    "set_max_bandwidth 200\n"
    f"set_range_minmax da {int(V_BEGIN*1000)}m {int(V_END*1000)}m\n"
    f"set_range_minmax ba -{AUTORANGE_MAX} {AUTORANGE_MAX}\n"
    f"set_autoranging ba {AUTORANGE_MIN} {AUTORANGE_MAX}\n"
    f"set_e {int(V_BEGIN*1000)}m\n"
    "wait 1000m\n"
    "cell_on\n"
    f"meas_loop_lsv p c {int(V_BEGIN*1000)}m {int(V_END*1000)}m "
    f"{int(V_STEP*1000)}m {int(SCAN_RATE*1000)}m\n"
    "pck_start\n"
    "pck_add p\n"
    "pck_add c\n"
    "pck_end\n"
    "endloop\n"
    "on_finished:\n"
    "cell_off\n"
    "\n"
)

# ══════════════════════════════════════════════════════
#  LIVE PLOT SETUP
# ══════════════════════════════════════════════════════
plt.ion()
fig = plt.figure(figsize=(13, 5))
gs  = gridspec.GridSpec(1, 2, figure=fig, wspace=0.35)

# ── Left: I vs V ──────────────────────────────────────
ax_iv = fig.add_subplot(gs[0])
ax_iv.set_xlabel('Potential (V)')
ax_iv.set_ylabel('Current (A)')
ax_iv.set_title('I vs V  (live)')
ax_iv.axhline(0, color='lightgray', lw=0.8)
ax_iv.axvline(0, color='lightgray', lw=0.8)
line_iv,  = ax_iv.plot([], [], 'bo', ms=3, label='measured')
line_fit, = ax_iv.plot([], [], 'r--', lw=1.5, label='linear fit')
ax_iv.legend(fontsize=8)
ax_iv.ticklabel_format(axis='y', style='sci', scilimits=(-3, 3), useOffset=False)

# ── Right: R vs V ─────────────────────────────────────
ax_rv = fig.add_subplot(gs[1])
ax_rv.set_xlabel('Potential (V)')
ax_rv.set_ylabel('Resistance (Ω)')
ax_rv.set_title('R vs V  (live)')
line_rv,  = ax_rv.plot([], [], 'g.', ms=4, label='R = V/I (per point)')
ax_rv.axhline(
    MULTIMETER_R, color='darkorange', lw=1.5, ls='--',
    label=f'Multimeter: {MULTIMETER_R:.1f} Ω'
)
ax_rv.legend(fontsize=8)

fig.tight_layout()
plt.pause(0.05)

# ══════════════════════════════════════════════════════
#  DATA STORAGE
# ══════════════════════════════════════════════════════
voltages     = []
currents     = []
result_lines = []


def update_plots():
    """Called after every new data point — redraws both plots live."""
    V = np.array(voltages)
    I = np.array(currents)
    n = len(V)

    # ── I vs V ────────────────────────────────────────
    line_iv.set_data(V, I)
    ax_iv.relim()
    ax_iv.autoscale_view()

    # Linear fit once we have enough points (slope = 1/R)
    R_fit_str = "—"
    if n >= 5:
        coeffs = np.polyfit(V, I, 1)
        if abs(coeffs[0]) > 1e-12:
            R_fit     = 1.0 / coeffs[0]
            R_fit_str = f"{R_fit:.2f} Ω"
            line_fit.set_data(V, np.polyval(coeffs, V))

    # ── R vs V ────────────────────────────────────────
    # Skip |V| < 1mV to avoid division by ~zero at the sweep midpoint
    mask = np.abs(V) > 1e-3
    if mask.sum() > 1:
        line_rv.set_data(V[mask], V[mask] / I[mask])
        ax_rv.relim()
        ax_rv.autoscale_view()

    fig.suptitle(
        f'n = {n}  │  R_fit = {R_fit_str}  │  R_multimeter = {MULTIMETER_R:.1f} Ω',
        fontsize=11
    )
    plt.pause(0.001)


# ══════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════
logging.basicConfig(
    level=logging.INFO,
    format='[%(module)s] %(message)s',
    stream=sys.stdout
)
LOG = logging.getLogger(__name__)

# ══════════════════════════════════════════════════════
#  MEASUREMENT
# ══════════════════════════════════════════════════════
input('\n>>> Close PSTrace if open. Clip in your RESISTOR, then press Enter...\n')

# Auto-detect port and baud rate if not set manually
_port, _baud = (
    palmsens.serialport.auto_detect_port()
    if DEVICE_PORT is None
    else (DEVICE_PORT, BAUD_RATE)
)
LOG.info('Using port: %s  baud: %d', _port, _baud)

with palmsens.serialport.Serial(_port, _baud, timeout=1) as comm:
    device = palmsens.instrument.Instrument(comm)

    # Clear any hanging script from a previous run
    device.abort_and_sync()

    device_type = device.get_device_type()
    LOG.info('Connected to: %s', device_type)

    # Send MethodSCRIPT  (Option A — inline)
    LOG.info('Sending LSV MethodSCRIPT...')
    device.write(LSV_SCRIPT)
    # Option B — PSTrace export (comment out A and uncomment this):
    # device.send_script('lsv_resistance.mscr')

    LOG.info('Sweep running. Plotting live...')

    # Read lines one at a time and parse each data package immediately
    # This is what makes the plot live — not waiting for all data first
    while True:
        try:
            line = device.readline(line_timeout=LINE_TIMEOUT_S)
        except CommunicationTimeout:
            plt.pause(0.05)   # keep the plot responsive while waiting
            continue

        if line == '\n':      # bare newline = end of measurement
            LOG.info('Measurement complete.')
            break

        result_lines.append(line)

        if line.startswith('P'):
            # parse_mscript_data_package() is from YOUR mscript.py
            # It decodes the hex-encoded variable string into SI-unit floats
            try:
                pkg = palmsens.mscript.parse_mscript_data_package(line.rstrip('\n'))
                v   = pkg[0].value   # Applied potential (V)
                i   = pkg[1].value   # WE current (A)

                if abs(i) > 1e-15:   # skip uninitialised zero-current first point
                    voltages.append(v)
                    currents.append(i)
                    update_plots()
                    print(f'  V = {v:+.4f} V    I = {i:+.3e} A    R = {v/i:.1f} Ω')

            except Exception as e:
                LOG.warning('Parse error on line %r: %s', line, e)

        elif line.startswith('!'):
            LOG.error('Device error: %s', line.strip())

# ══════════════════════════════════════════════════════
#  FINAL RESULTS
# ══════════════════════════════════════════════════════
V = np.array(voltages)
I = np.array(currents)

print('\n' + '═' * 55)

if len(V) >= 5:
    coeffs   = np.polyfit(V, I, 1)
    R_emstat = 1.0 / coeffs[0]
    pct_diff = abs(R_emstat - MULTIMETER_R) / MULTIMETER_R * 100

    print(f'  Data points         :  {len(V)}')
    print(f'  R (EmStat LSV fit)  :  {R_emstat:.4f} Ω')
    print(f'  R (Multimeter)      :  {MULTIMETER_R:.4f} Ω')
    print(f'  Difference          :  {pct_diff:.2f}%')

    if pct_diff < 2:
        status = '✓ GOOD  — EmStat agrees with multimeter'
    elif pct_diff < 5:
        status = '~ OK    — small deviation, check connections'
    else:
        status = '✗ CHECK — large deviation, check wiring or current range'
    print(f'  Status              :  {status}')

    fig.suptitle(
        f'DONE  │  R_emstat = {R_emstat:.2f} Ω  │  '
        f'R_multimeter = {MULTIMETER_R:.2f} Ω  │  Δ = {pct_diff:.1f}%',
        fontsize=12,
        color='green' if pct_diff < 5 else 'red'
    )
else:
    print(f'  WARNING: Only {len(V)} points — check wiring and COM port.')
    print('  Common causes: wrong COM port | RE/CE not shorted | wrong range')

print('═' * 55 + '\n')

plt.ioff()
plt.show()