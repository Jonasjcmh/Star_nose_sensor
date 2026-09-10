"""
gain_sweep_collector.py — Star-Nose Sensor | FT5316 analog-gain calibration
===========================================================================
Same experiment as ../combination_calibration_collector.py, with ONE axis
added: the FT5316's analog gain (factory-mode register 0x07).

WHY
---
../combination_calibration_collector.py established the link between the true
capacitance (LCR-6100, Cp in pF) and the muca board's raw counts. Re-reading
its 2026-08-26 session shows the link is real but the operating point is bad:

    unloaded cell        ~20 counts
    cell with 1-2.5 pF   64,200 - 65,200 counts   (16-bit ceiling = 65,535)
    slope                ~ -580 counts/pF   (R^2 ~ 0.99 over 6 combinations)
    extrapolated Cp=0    ~65,770 counts     -> ABOVE full scale
    range actually used  ~900 counts = 1.4% of 16 bits

Two things are wrong there. The zero-capacitance intercept sits above the
ceiling, so the top of the range is clipped; and the slope is NEGATIVE — more
capacitance gives a LOWER reading, which is backwards for mutual-capacitance
coupling. Both are signatures of an AFE driven past its linear range. The
capacitors are inside the datasheet's "optimal 1-4 pF" window, so the gain,
not the capacitor, is the thing to change.

Gain is global to the chip, so this sweep cannot fix cell-to-cell spread —
that stays a software calibration. What it fixes is the scale and the
clipping, which no amount of software can undo.

WHAT THIS SCRIPT DOES
---------------------
Phase 0  BASELINE   nothing attached. For each gain, log the board. This is
                    what pins down the Cp=0 intercept per gain, which is the
                    number that tells you whether you are clipping.

Phase 1  per capacitor combination:
         LCR        once per combination — the ground truth, gain-independent,
                    identical to combination_calibration_collector.py.
         MUCA       per point, and then FOR EACH GAIN at that point: set the
                    gain over serial, let it settle, log for the dwell time.

The full 19-cell snapshot is logged every frame, exactly as before — there is
still no fixed table pinning a name like "a1" to a board channel, so nothing
is lost regardless of which cell actually moved.

OUTPUT
------
One CSV in logs/. Columns are those of combination_calibration_collector.py
plus `gain` and `gain_readback`, so analyze_gain_sweep.py can read the old
files as a single `native`-gain sweep and compare them directly against a new
one.

REQUIREMENTS
------------
  * firmware/Muca_Raw_gain/Muca_Raw_gain.ino flashed to the muca board
    (the script refuses to run against the original firmware — it would
    silently record the same gain N times)
  * LCR-6100 on its own USB port, configured Cp-Rp / 20 kHz / FAST / 1.0 V
  * both instruments plugged in at once; only the capacitor moves by hand

Usage
-----
  python gain_sweep_collector.py
  python gain_sweep_collector.py --prefix gain_sweep_solid --gains 1,4,8,12,16,20,24,28,31
  python gain_sweep_collector.py --skip-baseline --dwell 3
"""

import os
import sys
import csv
import time
import argparse
from datetime import datetime

_HERE     = os.path.dirname(os.path.abspath(__file__))
_PARENT   = os.path.normpath(os.path.join(_HERE, '..'))
_ROOT     = os.path.normpath(os.path.join(_HERE, '..', '..'))
_CAP_MEAS = os.path.join(_ROOT, 'Capacitance_measurement')
LOG_DIR   = os.path.join(_HERE, 'logs')

# Reuse the existing LCR driver rather than re-implementing it. The muca board
# is handled by the local link module, which — unlike sensor_raw.py — can also
# write to the port.
sys.path.insert(0, _CAP_MEAS)
sys.path.insert(0, _HERE)
from lcr6100 import LCR6100, list_ports              # noqa: E402
from muca_gain_link import (MucaGainLink, choose_port,  # noqa: E402
                            N_CELLS, RAW_FULL_SCALE)

LCR_POLL_HZ  = 20
MUCA_POLL_HZ = 20

# A spread across the byte, coarse at the top. Nothing authoritative documents
# the useful range of register 0x07 — that is what the sweep is for. Adjust
# with --gains once the first run shows where the interesting region is.
DEFAULT_GAINS = [1, 2, 4, 6, 8, 10, 12, 16, 20, 24, 28, 31]

# IDENTICAL to combination_calibration_collector.py's column layout, in the same
# order, with the two new columns APPENDED at the end. A file from this script is
# therefore a drop-in for every reader of the older logs — plot_muca_bars.py,
# plot_muca_combo_grid.py and the ../logs/ analysis all keep working, and the two
# generations of dataset can be concatenated.
BASE_FIELDNAMES = (['phase', 'combination_index', 'combination_label',
                    'point_seq', 'point_label',
                    'timestamp', 'datetime', 'elapsed_s',
                    'Cp_pF', 'Rp_ohm', 'lcr_ok']
                   + [f'cell_{k + 1}' for k in range(N_CELLS)])

# The gain axis is what this script adds; appended so column positions 1..30 are
# unchanged from the older files.
FIELDNAMES = BASE_FIELDNAMES + ['gain', 'gain_readback']


# ---------------------------------------------------------------------------
# Small interactive helpers (same style as combination_calibration_collector.py)
# ---------------------------------------------------------------------------

def sanitize_name(value, default="combo"):
    value = value.strip()
    if not value:
        value = default
    return ''.join(c if c.isalnum() or c in '-_' else '_' for c in value)


def ask_float(prompt, default):
    raw = input(f"{prompt} [{default}]: ").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        print(f"  Not a number, using default {default}")
        return default


def ask_int(prompt, default=None):
    while True:
        suffix = f" [{default}]: " if default is not None else ": "
        raw = input(f"{prompt}{suffix}").strip()
        if not raw and default is not None:
            return default
        try:
            return int(raw)
        except ValueError:
            print("  Please enter a whole number.")


def parse_gains(text):
    gains = []
    for token in text.replace(' ', '').split(','):
        if not token:
            continue
        try:
            gains.append(int(token))
        except ValueError:
            print(f"  Ignoring non-numeric gain '{token}'")
    return gains


# ---------------------------------------------------------------------------
# Connections
# ---------------------------------------------------------------------------

def connect_lcr():
    ports = list_ports()
    if not ports:
        print("No serial ports found for the LCR-6100.")
        sys.exit(1)
    print("\nAvailable serial ports:")
    for i, (dev, desc) in enumerate(ports):
        print(f"  {i}: {dev}  -  {desc}")
    idx = ask_int("Select LCR-6100 port index", default=0)
    lcr = LCR6100(ports[idx][0])
    lcr.connect()
    lcr.start_polling()
    print("Waiting for first LCR reading...")
    t0 = time.time()
    while lcr.measurement_count == 0 and time.time() - t0 < 5.0:
        time.sleep(0.05)
    return lcr


def connect_muca(port=None):
    print("\nSelect the MUCA BOARD port (a different port from the LCR):")
    chosen = choose_port(port)
    link = MucaGainLink(chosen).connect()

    kind = link.probe_firmware()
    if kind != "gain":
        link.close()
        print("\n" + "!" * 70)
        print("  The board is NOT running Muca_Raw_gain.ino.")
        print("  It did not answer the 'I' command, which means the original")
        print("  firmware is flashed and the gain cannot be changed from here.")
        print("  Flash firmware/Muca_Raw_gain/Muca_Raw_gain.ino and try again.")
        print("  (To reproduce the historical scale instead, use the original")
        print("   firmware with ../combination_calibration_collector.py.)")
        print("!" * 70)
        sys.exit(1)

    print("Muca board ready (gain firmware detected).")
    return link


# ---------------------------------------------------------------------------
# Logging passes
# ---------------------------------------------------------------------------

def log_lcr_phase(lcr, writer, combo_idx, combo_label, duration_s):
    """One LCR pass per combination — the ground truth. Gain plays no part."""
    print(f"\n[LCR] Logging combination '{combo_label}' for {duration_s:.1f} s ...")
    period  = 1.0 / LCR_POLL_HZ
    t_start = time.time()
    n = 0
    while time.time() - t_start < duration_s:
        t = time.time()
        Cp, Rp, ok = lcr.get_latest()
        writer.writerow({
            'phase':             'lcr',
            'gain':              '',
            'gain_readback':     '',
            'combination_index': combo_idx,
            'combination_label': combo_label,
            'point_seq':         '',
            'point_label':       '',
            'timestamp':         round(t, 4),
            'datetime':          datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'elapsed_s':         round(t - t_start, 3),
            'Cp_pF':             round(Cp * 1e12, 6),
            'Rp_ohm':            round(Rp, 6),
            'lcr_ok':            int(ok),
        })
        n += 1
        print(f"\r  Cp = {Cp * 1e12:10.4f} pF   n={n}   ", end='')
        sys.stdout.flush()
        time.sleep(max(0.0, period - (time.time() - t)))
    print(f"\n[LCR] Done — {n} samples logged.")


def log_muca_at_gain(link, writer, phase, gain, readback,
                     combo_idx, combo_label, point_seq, point_label, duration_s):
    """One muca pass at ONE gain. Logs the full 19-cell snapshot per frame."""
    period  = 1.0 / MUCA_POLL_HZ
    t_start = time.time()
    n = 0
    saturated = 0
    while time.time() - t_start < duration_s:
        t = time.time()
        values = link.get_values()
        row = {
            'phase':             phase,
            'gain':              gain,
            'gain_readback':     readback if readback is not None else '',
            'combination_index': combo_idx,
            'combination_label': combo_label,
            'point_seq':         point_seq,
            'point_label':       point_label,
            'timestamp':         round(t, 4),
            'datetime':          datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'elapsed_s':         round(t - t_start, 3),
        }
        for k in range(N_CELLS):
            row[f'cell_{k + 1}'] = values[k]
        writer.writerow(row)
        n += 1
        if max(values) >= RAW_FULL_SCALE - 1 or min(values) <= 0:
            saturated += 1
        print(f"\r    gain={gain:>3}  min={min(values):6.0f}  max={max(values):6.0f}  n={n}   ",
              end='')
        sys.stdout.flush()
        time.sleep(max(0.0, period - (time.time() - t)))

    flag = ""
    if saturated:
        flag = f"   *** {saturated}/{n} frames touched a rail ***"
    print(f"\r    gain={gain:>3}  done, {n} samples{flag}" + " " * 12)


def sweep_gains(link, writer, phase, gains, settle_s,
                combo_idx, combo_label, point_seq, point_label, dwell_s):
    """Walk every gain at the current physical configuration."""
    for gain in gains:
        try:
            readback = link.set_gain(gain, settle_s=settle_s)
        except RuntimeError as e:
            print(f"\n    [!] {e}")
            continue
        log_muca_at_gain(link, writer, phase, gain, readback,
                         combo_idx, combo_label, point_seq, point_label, dwell_s)


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="FT5316 analog-gain sweep collector (LCR + muca board).")
    ap.add_argument('--prefix', default=None,
                    help="Log filename prefix (asked interactively if omitted).")
    ap.add_argument('--gains', default=None,
                    help=f"Comma-separated gains. Default: "
                         f"{','.join(str(g) for g in DEFAULT_GAINS)}")
    ap.add_argument('--dwell', type=float, default=None,
                    help="Seconds to log at each gain (default asked, 5.0).")
    ap.add_argument('--settle', type=float, default=0.75,
                    help="Seconds to discard after each gain change (default 0.75).")
    ap.add_argument('--skip-baseline', action='store_true',
                    help="Skip the no-load baseline pass (NOT recommended — it is "
                         "what determines the Cp=0 intercept per gain).")
    ap.add_argument('--muca-port', default=None)
    args = ap.parse_args()

    print("=" * 70)
    print("  FT5316 Analog-Gain Sweep Collector  (register 0x07)")
    print("=" * 70)

    prefix = args.prefix or input("Log file prefix [gain_sweep]: ").strip() or "gain_sweep"
    prefix = sanitize_name(prefix, default="gain_sweep")

    gains = parse_gains(args.gains) if args.gains else list(DEFAULT_GAINS)
    if not gains:
        print("No valid gains given.")
        sys.exit(1)
    print(f"\nGains to sweep ({len(gains)}): {gains}")

    dwell = args.dwell if args.dwell is not None else \
        ask_float("Seconds to log at each gain", default=5.0)

    n_points = ask_int("How many points per combination?", default=5)
    n_combos = ask_int("How many capacitor combinations?", default=1)

    per_point = len(gains) * (dwell + args.settle)
    total     = per_point * n_points * n_combos
    if not args.skip_baseline:
        total += len(gains) * (dwell + args.settle)
    print(f"\nRough hands-off time: {per_point / 60:.1f} min per point, "
          f"{total / 60:.1f} min total (plus your handling time).")

    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename  = os.path.join(LOG_DIR, f"{prefix}_session_{timestamp}.csv")

    lcr  = connect_lcr()
    link = connect_muca(args.muca_port)

    lcr_duration = 10.0

    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=FIELDNAMES, restval='')
            writer.writeheader()

            # -- Phase 0: baseline, nothing attached --------------------------
            if not args.skip_baseline:
                print(f"\n{'=' * 70}\nPHASE 0 — BASELINE (no capacitor attached)\n{'=' * 70}")
                print("This is what fixes the Cp=0 intercept for every gain, and")
                print("therefore tells you which gains clip. Leave ALL muca inputs")
                print("open — nothing wired to them.")
                input("  Detach everything from the muca board, then press Enter...")
                sweep_gains(link, writer, 'baseline', gains, args.settle,
                            0, 'baseline', '', 'none', dwell)
                f.flush()

            # -- Phase 1: combinations ---------------------------------------
            for i in range(1, n_combos + 1):
                print(f"\n{'=' * 70}\nCombination {i}/{n_combos}\n{'=' * 70}")
                label = input("  Label for this combination (e.g. '10pF_only'): ")
                label = sanitize_name(label, default=f"combo_{i}")

                # LCR — once per combination, gain-independent ground truth.
                input("  Wire the capacitor under test to the LCR-6100 probes, then press Enter...")
                lcr_duration = ask_float("  LCR logging duration (s)", default=lcr_duration)
                log_lcr_phase(lcr, writer, i, label, lcr_duration)
                f.flush()

                # Muca — per point, and the full gain sweep at each point.
                for j in range(1, n_points + 1):
                    print(f"\n>>> Point {j}/{n_points}")
                    input("  Move the SAME capacitor to the muca board, then press Enter...")
                    point_label = input("  Name this point (e.g. 'a1'): ").strip() or f"point_{j}"
                    print(f"  Sweeping {len(gains)} gains, {dwell:.1f} s each — "
                          f"do not touch the setup...")
                    sweep_gains(link, writer, 'muca', gains, args.settle,
                                i, label, j, point_label, dwell)
                    f.flush()

                if i < n_combos:
                    input("\n  Swap in the next capacitor combination, then press Enter...")

    finally:
        try:
            lcr.disconnect()
        except Exception:
            pass
        link.close()
        print(f"\nSaved log -> {filename}")
        print(f"\nNext:  python analyze_gain_sweep.py "
              f"{os.path.relpath(filename, _HERE)}")


if __name__ == '__main__':
    main()
