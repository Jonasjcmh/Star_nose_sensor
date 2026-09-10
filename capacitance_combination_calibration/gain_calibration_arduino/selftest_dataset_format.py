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
     format — parse the new file unmodified.

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
