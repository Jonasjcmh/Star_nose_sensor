# Standard Operating Procedure — UR5 Z-Force Calibration Against FUTEK Load Cell

**System:** Star-Nose Sensor (UR5 + capacitive tactile sensor)
**Applies to:** `Integration_2/`
**Reference standard:** FUTEK 10 lb compression load cell (AI0)
**Sensor under calibration:** UR5 internal TCP force sensor (`fz`, from `getActualTCPForce()`)

---

## 1. Purpose

The UR5's internal force/torque sensor (`fz` in the session CSV) is used throughout this project as the "robot force" signal, but it is not independently metrology-grade — it drifts, is not re-zeroed every session, and includes tool/mounting load. The FUTEK load cell is the trusted reference (it has its own factory voltage-to-force calibration, already implemented as `F = -(ai0 - 5.0) * 8.896 N/V` in `analyze_session.py`).

This SOP defines how to derive a correction (scale + offset) that maps the UR5's raw `fz` reading onto true Z-force as measured by the FUTEK, and how to validate and apply that correction.

---

## 2. Equipment

| Item | Notes |
|---|---|
| FUTEK 10 lb compression load cell | Wired to UR5 analog input AI0, 0–10 V range, 5 V = 0 N |
| UR5 robot | Internal TCP force/torque sensor, RTDE `getActualTCPForce()` |
| Star-nose sensor tip (any profile) | Only used as the pressing tool; capacitive readings are not part of this calibration |
| `Integration_2/main.py`, `ur5_control.py`, `data_logger.py`, `analyze_session.py` | Existing session, logging, and analysis pipeline |

---

## 3. Prerequisites

1. FUTEK is correctly mounted in the load path and wired to AI0 — confirm polarity with a hand-press test (`ai0` should drop below 5 V under compression).
2. Global tip calibration (`calib_<tip>.json`) is current — run `calibrate_ur5.py` if the tip changed.
3. Repo is on a clean working state (`git status`) before starting, so calibration output files are easy to review in the diff.

---

## 4. Procedure

### Step 1 — Zero both sensors

Before each calibration session:

1. Lift the tip clear of the surface (no contact, no load).
2. Read `fz` and `ai0` for ~5 s idle (e.g. via `main.py` live display or a short logged session).
3. Record:
   - `fz_zero` — mean idle `fz` (N)
   - `ai0_zero` — mean idle `ai0` (V), should be close to 5.0 V

These are session-specific baselines; `analyze_session.py` already computes them internally (`fz_zero`, `lc_zero` in `plot_loadcell_vs_robot`) by taking the min of the full signal — for a dedicated calibration run, prefer the pre-contact idle mean instead of the signal min, since min conflates zero-drift with an actual data point.

### Step 2 — Collect a calibration dataset spanning the working force range

Run multiple pressing sessions that vary indentation depth, so the FUTEK sees a spread of forces rather than one repeated value:

```bash
python main.py --tip <tip_name>
```

- Vary `DEFAULT_INDENT_MM` (default 6.00 mm) across at least 4–5 depths, e.g. 1, 2, 3, 4, 6 mm, either by editing the value at the `main.py` indentation prompt or via `POINT_OVERRIDES` in `ur5_control.py`.
- Use dwell time ≥ `DEFAULT_DWELL_S` (1.5 s) so each press has a steady-state plateau, not just the transient rise.
- Cover a range of points (P1–P19), not just the centre, so the correction isn't biased to one contact location.
- Aim for ≥ 100 independent press events total across all depths/points — each press contributes one plateau sample.

Each session logs `fz` and `ai0` at 20 Hz to `logs/{prefix}_session_{timestamp}.csv` automatically via `data_logger.py`.

### Step 3 — Extract paired samples

For each press event, use only the **dwell plateau**, not the rise/fall transient:

- Filter rows where `ur5_pressing == 1`.
- Drop the first ~0.3 s of each press window (motion settling).
- For each remaining window, take the mean `fz` and mean `ai0` → convert `ai0` to `lc_N` using the existing `ai0_to_newtons()` conversion in `analyze_session.py`.
- Subtract each session's own `fz_zero` / `lc_zero` (Step 1) from every sample.

This produces one `(fz_robot, lc_futek)` pair per press event. `analyze_session.py --loadcell` already visualizes the raw relationship (scatter, Pearson r, residuals, Bland–Altman) — run it first on each session as a sanity check before pooling data:

```bash
python analyze_session.py --loadcell logs/<session>.csv
```

### Step 4 — Fit the correction

Treat FUTEK as ground truth (`y`) and robot `fz` as the signal to correct (`x`). Fit an ordinary least-squares line:

```
lc_futek ≈ a · fz_robot + b
```

- `a` (slope) corrects gain/scale error in the UR5 force sensor.
- `b` (offset) corrects residual zero-bias not caught by Step 1.

Report: slope `a`, intercept `b`, R², RMSE (N), and sample count `n`. Use `numpy.polyfit(fz_robot, lc_futek, 1)` or equivalent.

### Step 5 — Validate on held-out data

Split the calibration dataset before fitting (e.g. 80% fit / 20% hold-out, or better: fit on sessions 1–N and validate on a separate session collected on a different day).

For the hold-out set, compute:
- RMSE of raw `fz_robot` vs `lc_futek` (baseline, uncorrected)
- RMSE of `(a · fz_robot + b)` vs `lc_futek` (corrected)
- Bland–Altman bias and ±1.96σ limits of agreement, before and after correction (reuse the logic already in `plot_loadcell_vs_robot`)

### Step 6 — Acceptance criteria

Calibration passes if, on the hold-out set:

| Metric | Threshold |
|---|---|
| Corrected RMSE | ≤ 0.5 N (or ≤ 5% of max applied force, whichever is larger) |
| Pearson r | ≥ 0.98 |
| Bland–Altman bias | ≤ 0.2 N |
| ±1.96σ limits of agreement | within ±1 N |

Adjust thresholds if the application's required force resolution differs — these defaults assume the 10 lb (44.5 N) FUTEK range and sub-Newton press forces typical of this rig.

If the fit fails acceptance, check for: FUTEK wiring/clipping (`ai0` pinned at 0 V or 10 V), tip flexing/slipping, insufficient force range in the calibration data, or a UR5 F/T sensor that needs re-zeroing at the controller (Polyscope: Installation → General → Zero on the F/T sensor).

### Step 7 — Store and apply the correction

Save the fitted constants alongside the existing calibration files, e.g. `calib_fz_<tip>.json`:

```json
{
  "tip": "short_6mm",
  "date": "2026-07-03",
  "slope": 1.023,
  "offset": -0.041,
  "r_squared": 0.991,
  "rmse_n": 0.31,
  "n_samples": 148,
  "futek_rated_lb": 10.0
}
```

Apply as `fz_corrected = slope * fz_robot + offset` wherever robot Z-force is used downstream (e.g. `analyze_session.py` force plots, any real-time force-based control in `ur5_control.py`). Keep the raw `fz` column in the CSV unchanged — apply the correction only at analysis/consumption time, so raw logs remain reproducible.

---

## 5. Re-calibration frequency

Repeat this procedure:

- Whenever the FUTEK is remounted or rewired.
- Whenever the UR5 F/T sensor is re-zeroed at the controller or after a robot restart.
- When switching to a different tip profile with significantly different mass/geometry.
- At minimum, once per data-collection campaign (e.g. monthly), and always before publication-quality data collection.

---

## 6. Records

Keep, per calibration run:

1. Raw session CSV(s) used (`logs/*.csv`).
2. The fitted `calib_fz_<tip>.json`.
3. The `--loadcell` diagnostic plot(s) (`plots/<session>/loadcell_vs_robot.png`) for both the fit and validation sessions.
4. A one-line log entry: date, operator, tip, slope, offset, RMSE, pass/fail against Section 4.6.
