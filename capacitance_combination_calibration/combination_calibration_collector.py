"""
combination_calibration_collector.py — Star-Nose Sensor | Capacitance Combination Calibration
================================================================================================
Ground-truth calibration collector linking real capacitance (LCR-6100) to
the muca board's raw reading (see
mucaboard_data/README_capacitance_estimation.md for why this fit is needed
at all — Cp_pF from the LCR and V from the muca board are two different
electrical quantities that only get linked by matching them up).

IMPORTANT: no flexible tactile sensor (dome) is attached for this
calibration, and there's no fixed lookup table pinning a name like "a1" to
one specific muca board channel index. Instead, point NAMES are decided by
the operator, per reading, at the time they take it: whichever muca reading
you take first in a combination, you name it (e.g. "a1"); the next one you
name "a2" or whatever you actually wired it to, and so on.

Two different things change during a session, on two different axes:

  • COMBINATION — the actual capacitor (or capacitor combination) under
    test. It is measured on the LCR-6100 ONCE per combination — that's its
    true, ground-truth value, independent of which point it's later logged
    as on the muca board.
  • POINT        — after the LCR measurement, that SAME capacitor is moved
    by hand to a muca-board input, you tell the script what to call that
    point (e.g. "a1"), and it's logged. Repeat for as many points as you
    asked for at the start (e.g. 5 → a1, a3, b2, ... whatever you name them).

So for each combination:
  1. LCR PHASE  (once)          — wire the capacitor to the LCR-6100 probes,
     poll Cp/Rp for a user-chosen duration, log it.
  2. MUCA PHASE (once per point) — move the SAME capacitor to a muca input,
     name that point, read the board for a user-chosen duration, and log
     it. Because there's no fixed channel table, the FULL 19-cell board
     snapshot is logged each time (cell_1..cell_19), tagged with the name
     you gave it, so nothing is lost regardless of which cell actually
     moved.
  3. Swap in the next capacitor combination by hand and repeat, for as many
     combinations as requested at the start.

Both instruments are wired into the laptop over their own USB ports at the
same time (same board/USB setup as visualizer_2d.py), so their serial
connections are opened once, up front, and stay open all session — only the
capacitor itself moves by hand. Logging is never simultaneous.

All samples (both phases, all combinations, all points) go to ONE combined
CSV in logs/, distinguished by the `phase` column.

Usage
-----
  python combination_calibration_collector.py
  python combination_calibration_collector.py --prefix cap_calib_test
"""

import os
import sys
import csv
import time
import argparse
from datetime import datetime

_HERE     = os.path.dirname(os.path.abspath(__file__))
_MUCA_RAW = os.path.normpath(os.path.join(_HERE, '..', 'mucaboard_data_raw'))
_CAP_MEAS = os.path.normpath(os.path.join(_HERE, '..', 'Capacitance_measurement'))
LOG_DIR   = os.path.join(_HERE, 'logs')

# Reuse the existing hardware drivers instead of re-implementing them —
# mucaboard_data_raw/sensor_raw.py (muca board, no on-the-fly processing) and
# Capacitance_measurement/lcr6100.py (LCR-6100 serial driver).
sys.path.insert(0, _MUCA_RAW)
sys.path.insert(0, _CAP_MEAS)
import sensor_raw as muca                  # noqa: E402
from lcr6100 import LCR6100, list_ports    # noqa: E402

N_CELLS      = 19   # muca.get_values() always returns this many raw cells
LCR_POLL_HZ  = 20
MUCA_POLL_HZ = 20


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


def connect_muca():
    muca.start()
    print("Waiting for muca board...")
    if not muca.wait_until_ready(timeout=30):
        print("Muca board did not become ready — check the connection.")
        sys.exit(1)
    print("Muca board ready.")


def log_lcr_phase(lcr, writer, combo_idx, combo_label, duration_s):
    """One LCR logging pass for this combination — NOT per point."""
    print(f"\n[LCR] Logging combination '{combo_label}' for {duration_s:.1f} s ...")
    period  = 1.0 / LCR_POLL_HZ
    t_start = time.time()
    n = 0
    while time.time() - t_start < duration_s:
        t = time.time()
        Cp, Rp, ok = lcr.get_latest()
        writer.writerow({
            'phase':               'lcr',
            'combination_index':   combo_idx,
            'combination_label':   combo_label,
            'point_seq':           '',
            'point_label':         '',
            'timestamp':           round(t, 4),
            'datetime':            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'elapsed_s':           round(t - t_start, 3),
            'Cp_pF':               round(Cp * 1e12, 6),
            'Rp_ohm':              round(Rp, 6),
            'lcr_ok':              int(ok),
        })
        n += 1
        print(f"\r  Cp = {Cp * 1e12:10.4f} pF   n={n}   ", end='')
        sys.stdout.flush()
        time.sleep(max(0.0, period - (time.time() - t)))
    print(f"\n[LCR] Done — {n} samples logged.")


def log_muca_phase(writer, combo_idx, combo_label, point_seq, point_label, duration_s):
    """One muca logging pass for this combination, on the point just named.
    No fixed channel table — logs the FULL 19-cell board snapshot each frame.
    """
    print(f"\n[MUCA] Logging combination '{combo_label}' point '{point_label}' for {duration_s:.1f} s ...")
    period  = 1.0 / MUCA_POLL_HZ
    t_start = time.time()
    n = 0
    while time.time() - t_start < duration_s:
        t = time.time()
        values = muca.get_values()
        row = {
            'phase':               'muca',
            'combination_index':   combo_idx,
            'combination_label':   combo_label,
            'point_seq':           point_seq,
            'point_label':         point_label,
            'timestamp':           round(t, 4),
            'datetime':            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'elapsed_s':           round(t - t_start, 3),
        }
        for k in range(N_CELLS):
            row[f'cell_{k + 1}'] = values[k]
        writer.writerow(row)
        n += 1
        print(f"\r  min={min(values):.0f}  max={max(values):.0f}   n={n}   ", end='')
        sys.stdout.flush()
        time.sleep(max(0.0, period - (time.time() - t)))
    print(f"\n[MUCA] Done — {n} samples logged.")


def main():
    ap = argparse.ArgumentParser(
        description="Capacitance combination calibration collector (LCR + muca board).")
    ap.add_argument('--prefix', default=None,
                     help="Log filename prefix (asked interactively if omitted).")
    args = ap.parse_args()

    print("=" * 60)
    print("  Capacitance Combination Calibration Collector")
    print("=" * 60)

    prefix = args.prefix or input("Log file prefix [cap_combo_calib]: ").strip() or "cap_combo_calib"
    prefix = sanitize_name(prefix, default="cap_combo_calib")
    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename  = os.path.join(LOG_DIR, f"{prefix}_session_{timestamp}.csv")

    n_points = ask_int("\nHow many points do you want to log per combination?", default=5)
    n_combos = ask_int("How many capacitor combinations will you run?", default=1)

    # Both instruments are wired into the laptop over their own USB ports at
    # the same time, so both connections are opened up front and stay open
    # for the whole session. Only the capacitor under test moves by hand.
    lcr = connect_lcr()
    connect_muca()

    fieldnames = (['phase', 'combination_index', 'combination_label', 'point_seq', 'point_label',
                   'timestamp', 'datetime', 'elapsed_s', 'Cp_pF', 'Rp_ohm', 'lcr_ok']
                  + [f'cell_{k + 1}' for k in range(N_CELLS)])

    lcr_duration  = 10.0
    muca_duration = 10.0

    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, restval='')
            writer.writeheader()

            for i in range(1, n_combos + 1):
                print(f"\n{'=' * 60}\nCombination {i}/{n_combos}\n{'=' * 60}")
                label = input("  Label for this combination (e.g. '10pF_only'): ")
                label = sanitize_name(label, default=f"combo_{i}")

                # LCR — once per combination.
                input("  Wire the capacitor under test to the LCR-6100 probes, then press Enter...")
                lcr_duration = ask_float("  LCR logging duration (s)", default=lcr_duration)
                log_lcr_phase(lcr, writer, i, label, lcr_duration)
                f.flush()

                # Muca — once per point. The operator names each point as they take it —
                # there's no fixed table saying which physical input is "a1" etc.
                for j in range(1, n_points + 1):
                    print(f"\n>>> Point {j}/{n_points}")
                    input("  Move the SAME capacitor to the muca board, then press Enter...")
                    point_label = input("  Name this point (e.g. 'a1'): ").strip() or f"point_{j}"
                    muca_duration = ask_float("  Muca logging duration (s)", default=muca_duration)
                    log_muca_phase(writer, i, label, j, point_label, muca_duration)
                    f.flush()

                if i < n_combos:
                    input("\n  Swap in the next capacitor combination, then press Enter to continue...")
    finally:
        lcr.disconnect()
        print(f"\nSaved log → {filename}")


if __name__ == '__main__':
    main()
