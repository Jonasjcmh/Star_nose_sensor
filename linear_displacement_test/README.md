# Linear Displacement Test

Point-to-point **linear slide** collector for the Star-Nose sensor. The UR5
engages the tip to a fixed `depth` at an initial pad, slides in a straight line
to a final pad at a fixed `speed`, then retracts and homes — logging the **raw**
muca-board counts, FUTEK load cell, and TCP pose the whole way.

It is a close cousin of `mucaboard_data_raw/data_collector_raw.py` (same raw
logging + UR5 calibration-profile selection + raw sensor reader) and borrows its
motion model from `friction_mode/ur5_friction.run_displacement_trajectory`
(engage to a fixed Z depth, then pure position-controlled lateral motion, no
force feedback during the slide).

Nothing outside this folder is modified — it only *imports* read-only helpers:

| Borrowed from | What |
|---------------|------|
| `Integration_2/ur5_control.py` | `POINTS` (P1..P19 / a1..e5 map), `REFERENCE_POSE`, `resolve_point` |
| `Integration_2/data_logger.py` | log filename helpers (`sanitize_name` / `ask_file_prefix` / `build_filename`) |
| `mucaboard_data_raw/sensor_raw.py` | RAW muca-board reader (no normalisation) |
| `Integration_2/calib_*.json` | UR5 positional calibration profiles (selected at run time) |

## What you specify

The core parameters (CLI flags or interactive prompts):

| Parameter | Flag | Meaning |
|-----------|------|---------|
| **speed** | `--speed` | lateral slide speed, mm/s |
| **initial point** | `--from` | where every slide starts — `1..19` or `a1..e5` |
| **final point(s)** | `--to` | one **or more** endpoints, comma separated — `e5,c3,a3` |
| **depth** | `--depth` | indentation below the surface, mm |
| **iterations** | `--iters` | passes per displacement (repeat each slide N times) |

**Multiple final points.** With one initial point and several finals you get one
linear trajectory per final, all sharing the same start:
`initial→final1 (×iters)`, `initial→final2 (×iters)`, … Each is an independent
`initial → finalN` displacement. Between them the tool lifts to a travel
clearance (15 mm) and re-approaches, so it never drags across the sensor
off-slide. If the initial point appears in the `--to` list it is dropped.

Optional extras:

| Flag | Default | Meaning |
|------|---------|---------|
| `--round-trip` | off | each pass also slides back to the start (stays engaged) |
| `--hold-start S` | ask, 0 | hold (s) at the **initial** position each pass |
| `--hold-end S` | ask, 1 | hold (s) at the **final** position each pass |
| `--locate S` | 2.0 | dwell (s) above the start before engaging |
| `--step MM` | 0.5 | straight-line interpolation step (smaller = smoother / faster stop) |
| `--viz` | off | live sensor hex-map window during the run (read-only) |
| `--prefix NAME` | ask | log filename prefix |
| `--dry-run` | off | print the plan + computed poses only (no robot, no sensor) |

**Holds.** Each pass can pause at the two ends: `--hold-start` at the initial
position (e.g. capture a settled rest reading before sliding) and `--hold-end`
at the final position (let the sensor settle at the destination). Set either to
`0` to skip it. They are asked interactively when not passed.

**Live sensor view (`--viz`).** Opens a matplotlib window showing all 19 sensor
cells as a hex map, colour-coded live by their deviation from the baseline, with
the current phase / pass / progress and the start (green) and final (red) pads
ringed. It is **strictly read-only** — it only calls `sensor.get_values()`, never
the serial port or the log — so it cannot affect the collected data. While it is
open the robot motion runs on a worker thread and the window owns the main
thread; **closing the window stops the run early**. The colours are a display
convenience (`|value − baseline|`); the CSV still stores the untouched raw counts.

## Quick start

```bash
cd linear_displacement_test

# Preview the plan without touching the robot or the sensor
python linear_displacement.py --from a1 --to e5 --speed 12 --depth 2.5 --dry-run

# One slide a1 → e5 at 10 mm/s, 2 mm deep (asks for calibration + prefix)
python linear_displacement.py --from a1 --to e5 --speed 10 --depth 2

# One initial point (c3) → four finals, each slid 3 times
python linear_displacement.py --from c3 --to e5,a3,e3,a1 --speed 12 --depth 2 --iters 3

# Three back-and-forth passes between a3 and e3, 15 mm/s, 2.5 mm deep
python linear_displacement.py --from a3 --to e3 --speed 15 --depth 2.5 \
       --iters 3 --round-trip --prefix ecoflex_line_a3_e3

# Fully interactive
python linear_displacement.py
```

## Motion sequence

```
home
 → for each FINAL point:
      locate above START at clearance (dwell --locate)
      engage: descend to surface, then push to -depth
      for each pass (--iters):
          hold at START (--hold-start)
          slide START → FINAL at speed (fixed depth)   [logged, sliding=1]
          hold at FINAL (--hold-end)
          if --round-trip:  slide FINAL → START
          else (more passes): lift to clearance, travel to START, re-engage
      retract: lift off to clearance
 → home
```

Every final point reuses the same initial point, so a run with `--to e5,a3,e3`
is three independent `initial→finalN` displacements back to back, all in one CSV
(tell them apart with `to_point` / `to_label`).

The straight line is interpolated into `--step` mm collinear waypoints
(`moveL` each at `speed`) so the path stays linear, `progress` (0→1) is logged,
and a `Ctrl+C` breaks out cleanly after the current segment.

## Output

One CSV per run in `linear_displacement_test/logs/`
(`<prefix>_session_<timestamp>.csv`), streamed at 20 Hz. Columns:

- `cell_1..cell_19` — **pure raw ADC counts** (no normalisation / linearisation).
- `calib_1..calib_19` — the baseline (first) frame, stored per row for reference.
- `tcp_x/y/z`, `fx..tz`, `ai0`, `load_cell_N` — TCP pose, wrench, FUTEK voltage/force.
- `from_point/from_label`, `to_point/to_label` — the slide endpoints.
- `sliding` (1 while moving laterally), `phase`
  (`locate`/`engage`/`hold_start`/`slide`/`hold_end`/`reposition`/`retract`),
  `direction` (`fwd`/`rev`), `iter_idx`, `progress` (0→1 along the current
  slide), `depth_mm`, `speed_mm_s`, `done`.

Segment a slide downstream with `sliding == 1` (optionally split fwd/rev by
`direction`, and passes by `iter_idx`).

## Robot / safety notes

- Robot IP defaults to `177.22.22.2`; override with `UR_ROBOT_IP`.
- Points, `REFERENCE_POSE`, and per-point calibration offsets are the **same**
  as the raw collector / `ur5_control.py`, so a slide lands on the same pads.
- Engage and retract are slow (`VEL_ENGAGE = 4 mm/s`); the lateral slide runs at
  your `--speed`. Depth is bounded 0.1–10 mm, speed 0.5–100 mm/s.
- Position control only — there is **no** force limit during the slide; pick a
  `depth` the tip and sensor tolerate, and **keep the e-stop within reach**.
- Use `--dry-run` first to confirm the endpoints and poses before moving.
