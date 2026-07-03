# Force Sensor Calibration

This folder documents how the UR5's internal Z-force reading (`fz`) is calibrated against the FUTEK load cell, which serves as the trusted force reference for this project.

## Why this exists

The star-nose sensor rig records two independent measurements of contact force during every press:

- **`fz`** — the UR5's internal TCP force/torque sensor, read via RTDE (`getActualTCPForce()`).
- **`ai0`** — the FUTEK 10 lb compression load cell, wired to the UR5's analog input and converted to Newtons via its factory calibration (`F = -(ai0 - 5.0) * 8.896 N/V`).

The UR5's internal sensor is convenient (no extra wiring, sampled at 125 Hz) but is not metrology-grade: it drifts, isn't re-zeroed every session, and includes tool/mounting load. The FUTEK is the more trustworthy reference, so we use it to derive a correction for `fz` rather than relying on the robot's raw reading.

**Goal:** the FUTEK is a temporary reference, not a permanent part of the rig. Once the correction is fitted and shown to repeat across multiple independent sessions (see "Step 8" in the SOP), the FUTEK can be unmounted and future sessions can rely on the corrected `fz` alone — freeing up the AI0 wiring and one point of hardware failure. After removal, the FUTEK should still be reinstalled periodically for spot-checks, since there's no longer a live reference to catch new drift otherwise.

## Contents

| File | Purpose |
|---|---|
| [`FORCE_CALIBRATION_SOP.md`](./FORCE_CALIBRATION_SOP.md) | Step-by-step procedure: zeroing, data collection, fitting the correction (slope + offset), validation, acceptance criteria, and how/where to apply the result. |

## Relationship to the rest of the repo

This calibration is separate from the **geometric** X/Y/Z calibration done by `Integration_2/calibrate_ur5.py` and `calibrate_points.py` (which align the tip over the sensor grid). This SOP instead corrects the **force magnitude** reading (`fz`) itself.

The scripts referenced in the SOP (`main.py`, `ur5_control.py`, `data_logger.py`, `analyze_session.py`) live in [`Integration_2/`](../Integration_2/), which is the active version of the control/analysis pipeline. `analyze_session.py --loadcell` already produces the FUTEK-vs-robot diagnostic plots (scatter, Pearson r, residuals, Bland–Altman) used as inputs to this calibration.

Once a calibration is fitted, its constants should be saved as `calib_fz_<tip>.json` (see the SOP for the format) and applied wherever robot Z-force is consumed downstream, without altering the raw logged `fz` values.
