#!/usr/bin/env python3
"""
generate_all_curves.py — runs every calibration/compensation analysis
script built up in this project, in a sensible order, so every curve and
coefficient can be regenerated with a single command.

Each script stays self-contained (recomputes what it needs from the raw
logs, per this project's convention) rather than importing results from
another, so this is a thin orchestrator: it just calls each script's
main() in turn and reports what got produced.

Order (later scripts don't depend on earlier ones' output, but this
groups "load cell + UR together" first, then "UR alone", then the
cross-check that ties them together):
  1. fit_lc_ur_calibration.py
       Step 1: load-cell voltage <-> force (ai0 -> F_signed)
       Step 2/3: F_lc vs UR fz, same-session pairing + per-direction
                 compensation + Bland-Altman
       Step 4: ur_only cross-check vs known weight
  2. plot_lc_vs_ur_by_weight.py
       LC vs UR force per weight, grouped bars (futek_direct)
  3. plot_ur_only_vs_load.py
       UR sensor (fz, absolute) vs known load (ur_only)
  4. plot_fz_vs_time_ur_only.py
       UR fz vs time, loaded window only, per weight/direction (ur_only)
  5. plot_lc_ur_force_vs_time.py
       Force vs time (-200..+200 g ordered), LC linearization (all raw
       samples), UR compensation vs F_lc (raw + sign-corrected), UR vs
       known weight F_true (raw + sign-corrected, UR-side hardware),
       consolidated coefficients summary
  6. plot_ur_only_compensation_crosscheck.py
       Held-out validation: coefficients fit on futek_direct, applied to
       the independent ur_only dataset

Usage:
    python generate_all_curves.py
"""

import importlib
import time

SCRIPTS = [
    "fit_lc_ur_calibration",
    "plot_lc_vs_ur_by_weight",
    "plot_ur_only_vs_load",
    "plot_fz_vs_time_ur_only",
    "plot_lc_ur_force_vs_time",
    "plot_ur_only_compensation_crosscheck",
]


def main():
    t_start = time.time()
    for name in SCRIPTS:
        print("\n" + "#" * 78)
        print(f"# Running {name}.py")
        print("#" * 78)
        module = importlib.import_module(name)
        module.main()

    print("\n" + "#" * 78)
    print(f"# Done — {len(SCRIPTS)} scripts, {time.time() - t_start:.1f}s")
    print("#" * 78)


if __name__ == "__main__":
    main()
