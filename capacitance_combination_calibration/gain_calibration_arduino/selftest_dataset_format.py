"""
selftest_dataset_format.py — prove the new dataset is a drop-in for the old one
==============================================================================
gain_sweep_collector.py writes a CSV that MUST stay readable by everything that
already reads ../combination_calibration_collector.py's logs. This script checks
that claim without any hardware attached:

  1. the first 30 columns are byte-identical, in order, to the original
     collector's, with `gain` and `gain_readback` appended at the end;
  2. a synthetic sweep with a known ground truth is generated;
  3. analyze_gain_sweep.py recovers that ground truth and correctly flags the
     gain that rails;
  4. ../plot_muca_bars.py and ../plot_muca_combo_grid.py — written for the OLD
     format — parse the new file unmodified;
  5. every Muca library call in firmware/Muca_Raw_gain/Muca_Raw_gain.ino is a
     function the library actually DEFINES — Muca.h declares some methods that
     Muca.cpp never implements (`getFWVersion()` is one), and those only fail
     at LINK time, long after the sketch looks fine.

Run it after touching either collector, or after changing sensor_raw.py's
USED_CELLS.

  python selftest_dataset_format.py            # keeps nothing behind
  python selftest_dataset_format.py --keep     # leaves the artefacts for inspection

NOTE on the legacy plotters: they average every `phase == 'muca'` row for a
point, which in a gain-sweep file means averaging ACROSS gains. They parse the
file correctly — that is what this test asserts — but for a sweep file their
per-point bars mix gains and are not meaningful. Use analyze_gain_sweep.py for
sweep data, and the legacy plotters for single-gain sessions.
"""

import os
import re
import sys
import csv
import random
import shutil
import argparse
import subprocess
import tempfile

HERE   = os.path.dirname(os.path.abspath(__file__))
PARENT = os.path.normpath(os.path.join(HERE, '..'))
sys.path.insert(0, HERE)

from gain_sweep_collector import FIELDNAMES, BASE_FIELDNAMES, N_CELLS  # noqa: E402

GAINS  = [4, 8, 16, 31]
COMBOS = [('2units', 2.526), ('4units', 1.699), ('6units', 1.349),
          ('8units', 1.164), ('10units', 1.043), ('12units', 0.975)]
POINTS = {'a1': 0, 'b2': 4, 'c3': 9, 'd4': 14, 'e5': 18}

TRUE_COUNTS_PER_PF = 900.0     # per unit of gain, in the synthetic model
BASELINE_COUNTS    = 20.0
FULL_SCALE         = 65535

_passed, _failed = 0, 0


def check(ok, message):
    global _passed, _failed
    if ok:
        _passed += 1
        print(f"  PASS  {message}")
    else:
        _failed += 1
        print(f"  FAIL  {message}")


# ---------------------------------------------------------------------------

def test_columns():
    print("\n[1] Column layout")
    original = (['phase', 'combination_index', 'combination_label',
                 'point_seq', 'point_label',
                 'timestamp', 'datetime', 'elapsed_s',
                 'Cp_pF', 'Rp_ohm', 'lcr_ok']
                + [f'cell_{k + 1}' for k in range(19)])
    check(BASE_FIELDNAMES == original,
          "base columns match combination_calibration_collector.py, in order")
    check(FIELDNAMES[:len(original)] == original,
          "new columns are appended, not interleaved")
    check(FIELDNAMES[len(original):] == ['gain', 'gain_readback'],
          "appended columns are exactly gain, gain_readback")


def synth_value(gain, cp, active):
    """Linear in gain and in Cp, saturating at full scale — enough to test the
    analyzer's slope, headroom and rail detection."""
    if not active:
        return BASELINE_COUNTS + random.gauss(0, 1)
    v = TRUE_COUNTS_PER_PF * gain * cp + 300.0 * gain
    return min(FULL_SCALE, max(0.0, v + random.gauss(0, 1.5)))


def write_synthetic(path, frames=12, lcr_frames=20):
    random.seed(7)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES, restval='')
        w.writeheader()

        def muca(phase, gain, ci, cl, ps, pl, cp, active):
            for n in range(frames):
                row = {'phase': phase, 'gain': gain, 'gain_readback': gain,
                       'combination_index': ci, 'combination_label': cl,
                       'point_seq': ps, 'point_label': pl,
                       'timestamp': 1e9 + n, 'elapsed_s': round(n * 0.05, 3),
                       'datetime': '2026-09-11 10:00:00.000'}
                for k in range(N_CELLS):
                    row[f'cell_{k + 1}'] = round(synth_value(gain, cp, k == active), 1)
                w.writerow(row)

        for g in GAINS:
            muca('baseline', g, 0, 'baseline', '', 'none', 0.0, -1)

        for i, (label, cp) in enumerate(COMBOS, 1):
            for n in range(lcr_frames):
                w.writerow({'phase': 'lcr', 'gain': '', 'gain_readback': '',
                            'combination_index': i, 'combination_label': label,
                            'point_seq': '', 'point_label': '',
                            'timestamp': 1e9 + n, 'elapsed_s': round(n * 0.05, 3),
                            'datetime': '2026-09-11 10:00:00.000',
                            'Cp_pF': round(cp + random.gauss(0, 0.002), 5),
                            'Rp_ohm': 1e20, 'lcr_ok': 1})
            for j, (pl, cell) in enumerate(POINTS.items(), 1):
                for g in GAINS:
                    muca('muca', g, i, label, j, pl, cp, cell)


def run(cmd, cwd):
    p = subprocess.run([sys.executable] + cmd, cwd=cwd,
                       capture_output=True, text=True)
    return p.returncode, p.stdout + p.stderr


def test_analyzer(csv_path, outdir):
    print("\n[3] analyze_gain_sweep.py recovers the synthetic truth")
    rc, out = run(['analyze_gain_sweep.py', csv_path, '--outdir', outdir], HERE)
    check(rc == 0, "analyzer runs without error")
    if rc != 0:
        print(out[-1500:])
        return

    rows = {}
    for line in out.splitlines():
        m = re.match(r'\s*(\d+)\s+\d+\s+([\d.]+)\s+([\d.]+)\s+', line)
        if m:
            rows[int(m.group(1))] = (float(m.group(2)), float(m.group(3)))

    check(set(rows) == set(GAINS),
          f"all {len(GAINS)} gains appear in the summary")

    for g in (4, 8, 16):               # gain 31 rails, so its slope is bent
        expected = TRUE_COUNTS_PER_PF * g
        got = rows.get(g, (0, 0))[0]
        check(abs(got - expected) / expected < 0.02,
              f"gain {g}: recovered {got:.0f} counts/pF vs {expected:.0f} expected")

    check('RECOMMENDED GAIN: 16' in out,
          "recommends gain 16 (best unrailed resolution)")
    check('RAILED' in out,
          "flags gain 31 as railed (it saturates by construction)")
    check(os.path.isfile(os.path.join(outdir, 'gain_summary.csv')),
          "writes the table view (gain_summary.csv)")


def test_legacy_plotters(csv_path):
    print("\n[4] The OLD plotters parse the NEW file unmodified")
    for script in ('plot_muca_bars.py', 'plot_muca_combo_grid.py'):
        if not os.path.isfile(os.path.join(PARENT, script)):
            print(f"  SKIP  {script} not found")
            continue
        rc, out = run([script, csv_path], PARENT)
        check(rc == 0 and 'saved' in out.lower(),
              f"../{script} reads the sweep CSV")
        # Their output lands beside the CSV, i.e. in the temp dir — nothing to
        # clean up inside the repo.


# Functions Muca.cpp actually defines (upstream master, verified against source).
# Muca.h additionally DECLARES getFWVersion, printInfo, setNumTouchPoints and
# setResolution; of those, only printInfo is implemented. Anything declared but
# not defined links with "undefined reference to `Muca::<name>()'".
MUCA_DEFINED = {
    'readRegister', 'setRegister', 'getRegister', 'getRegisters',
    'setConfig', 'setGain', 'printAllRegisters', 'printInfo', 'autocal',
    'selectLines', 'init', 'update', 'updated', 'getTouch', 'getTouchData',
    'setTouchPoints', 'getNumberOfTouches', 'setReportRate', 'useRawData',
    'getRawData',
}
MUCA_DECLARED_NOT_DEFINED = {'getFWVersion', 'setNumTouchPoints', 'setResolution'}

MUCA_LIB_CANDIDATES = [
    '~/Documents/Arduino/libraries/Muca/Muca.cpp',
    '~/Arduino/libraries/Muca/Muca.cpp',
    '~/Library/Arduino15/libraries/Muca/Muca.cpp',
]


def defined_in_installed_library():
    """Parse the installed Muca.cpp if we can find it; else fall back to the
    verified upstream list. Returns (set_of_names, source_description)."""
    for cand in MUCA_LIB_CANDIDATES:
        path = os.path.expanduser(cand)
        if os.path.isfile(path):
            try:
                src = open(path, errors='replace').read()
            except OSError:
                continue
            names = set(re.findall(r'\bMuca::(\w+)\s*\(', src))
            names.add('updated')          # #define updated() update()
            if names:
                return names, path
    return set(MUCA_DEFINED), 'upstream source (no local library found)'


def test_firmware_symbols():
    print("\n[5] Firmware calls only functions the library defines")
    sketch = os.path.join(HERE, 'firmware', 'Muca_Raw_gain', 'Muca_Raw_gain.ino')
    if not os.path.isfile(sketch):
        print("  SKIP  Muca_Raw_gain.ino not found")
        return

    src = open(sketch, errors='replace').read()
    # Strip comments so the explanatory notes don't count as calls.
    src = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
    src = re.sub(r'//[^\n]*', '', src)

    called = set(re.findall(r'\bmuca\.(\w+)\s*\(', src))
    defined, source = defined_in_installed_library()
    print(f"        checked against: {source}")

    missing = sorted(called - defined)
    check(not missing,
          f"all {len(called)} library calls resolve"
          + (f" -- MISSING: {', '.join(missing)}" if missing else ""))

    used_bad = sorted(called & MUCA_DECLARED_NOT_DEFINED)
    check(not used_bad,
          "no declared-but-undefined method is called"
          + (f" -- would fail at link: {', '.join(used_bad)}" if used_bad else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--keep', action='store_true',
                    help="leave the synthetic CSV and results behind")
    args = ap.parse_args()

    print("=" * 70)
    print("  Dataset-format self-test  (no hardware required)")
    print("=" * 70)

    test_columns()

    # Everything transient goes to the system temp dir, never into the repo —
    # the legacy plotters write their PNG next to the CSV they are given.
    tmpdir   = tempfile.mkdtemp(prefix='gain_selftest_')
    csv_path = os.path.join(tmpdir, 'synthetic_sweep.csv')

    print("\n[2] Synthetic sweep")
    write_synthetic(csv_path)
    check(os.path.getsize(csv_path) > 0, f"generated {csv_path}")

    test_analyzer(csv_path, os.path.join(tmpdir, 'results'))
    test_legacy_plotters(csv_path)
    test_firmware_symbols()

    if args.keep:
        print(f"\n  Artefacts kept in {tmpdir}")
    else:
        shutil.rmtree(tmpdir, ignore_errors=True)

    print("\n" + "=" * 70)
    print(f"  {_passed} passed, {_failed} failed")
    print("=" * 70)
    sys.exit(1 if _failed else 0)


if __name__ == '__main__':
    main()
