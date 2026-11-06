# indentation_test — single-point indentation characterisation

A **standalone** rig (not tied to the 19-point star-nose grid) that presses one
point straight down in a continuous ramp while recording four instruments, then
plots indentation-vs-force and indentation-vs-impedance/phase.

## Instruments logged
| Source | Columns | How |
|---|---|---|
| UR5 robot | `tcp_x..tcp_rz`, `ur_fx..ur_tz` | RTDE (`rtde_receive`) |
| FUTEK load cell | `ai0`, `futek_force_N` | robot AI0 → N (direct calib, 2026-07-23) |
| LCR-6100 | `lcr_primary`, `lcr_secondary`, `lcr_ok` | serial `FETC?`; the two fields are **\|Z\| + phase θ** in Z-θ mode or **Cp + Rp** in Cp-Rp mode |
| XJC X-6A-XD80-H28 | `xjc_Fx..xjc_Mz`, `xjc_ok` | RS485 / Modbus-RTU **poll** mode (`xjc_sensor.py`) |
| Derived | `cmd_depth_mm`, `indentation_mm` | commanded vs actual (from TCP z − surface z) |

## Workflow (interactive stages)
1. **Instruments up first** — the tool moves to a safe in-air height, the LCR/XJC
   are detected and connected, and the **XJC is tared unloaded**, so its Fz is
   live and near zero before you touch anything.
2. **Position (3-axis jog)** — keyboard-jog the TCP in base **X, Y and Z together**.
   The jog shows a **live FUTEK + XJC Fz read-out**: lower Z until the force just
   begins to rise above ~0 — that's the surface contact point (indentation 0 mm,
   coincident with the shared `REFERENCE_POSE` / UR-control frame). When you
   **fix** the position the tool lifts **+5 cm in Z for safety** (the 0 mm pose is
   kept; the run re-approaches it automatically).
3. **Save/Load calib** — the point (surface pose) is saved to
   `calib_session_<name>.json`. Later runs **load** it to reuse the same point and
   re-jog (usually just **Z**, the normal between-experiments height change).
4. **Material** — asked each run (the calib point can stay the same while the
   material changes). It goes into the dataset name and its own results folder.
5. **Range** — max indentation depth (mm).
6. **Iterations** — number of ramp cycles.
7. **Ramp speed** — constant indentation speed in mm/s (default **0.1 mm/s**).
8. **Holding time** — dwell held **symmetrically** at max depth (pushed) and at
   the surface (released, before the next cycle); default **1.0 s**. Use
   `--recover` only if you want to break the symmetry.
9. **Force cutoff** — overload safety limit in N (default **45 N**; `0` disables).
   During the press, a watchdog polls the real force (XJC + robot Fz) every ~2 ms;
   if |Fz| exceeds the limit it **stops the move, retracts to relieve the load,
   and ends the run**. Protects the sensors (XJC ±50 N, FUTEK 10 lb ≈ 44.5 N).
10. **Collect** — per iteration: press 0→max, hold, retract max→0, hold, as
    **single constant-velocity moves** at the chosen mm/s, logged continuously to
    a streaming CSV (flat RAM).

### Sampling rate (1 ms target) — what each instrument really delivers
The logger samples all instruments into one row at `--rate` Hz (**default 1000 =
1 ms**). Each instrument's reader runs as fast as its hardware allows and the row
repeats the latest cached value for the slower ones:

| Source | Realistic update rate |
|---|---|
| Logger (CSV rows) | ~1 kHz target (Python-bound; end-of-run prints the achieved Hz) |
| Robot RTDE (pose/wrench/AI0 → FUTEK) | ≤ 500 Hz (UR5e RTDE ceiling) |
| XJC (poll mode) | ~200–300 Hz (serial round-trip at 115200 baud) |
| LCR-6100 (`FETC?`) | ~20–100 Hz (meter-limited) |

True 1 ms *new* samples from the XJC would need stream mode; the LCR simply can't
go that fast. The run prints the **actual achieved rate** per instrument at the end.

### Jog keys (single 3-axis loop, with live force)
`a`/`d` = −X/+X (or ←/→) · `w`/`s` = +Y/−Y (or ↑/↓) · `r`/`f` = +Z/−Z (up / down
into the surface) · `-`/`+` = step size · `b` = zero the force baseline here (so
Δ reads 0) · **ENTER** fixes the point (then lifts +5 cm) · `q` aborts.
The status line shows `FUTEK ±N (Δ±N)` and `XJC Fz ±N (Δ±N)` — hover just above
the surface, press `b` to zero, then ease Z down until Δ lifts off zero.

## Run
```bash
python3 indentation_test.py                    # ask everything
python3 indentation_test.py --max-depth 3 --iterations 5
python3 indentation_test.py --no-xjc           # robot + FUTEK + LCR only
python3 indentation_test.py --new-point        # force a full X/Y + Z jog
UR_ROBOT_IP=127.0.0.1 python3 indentation_test.py
```
Useful flags: `--lcr-port /dev/ttyUSB0 --xjc-port /dev/ttyUSB1 --xjc-station 1 --xjc-baud 115200`,
`--ramp-speed 0.1 --hold 1 --recover 1 --rate 1000`, `--no-lcr`, `--no-plot`.

## Check what's connected (no motion)
Every run auto-detects which instruments are present: it pings the robot (RTDE)
and **probes each USB serial port**, identifying the LCR (answers `FETC?`) and the
XJC (answers a Modbus poll with a valid CRC frame), then logs each with its
detected port. To just see the report and exit — nothing moves, nothing is
logged:
```bash
python3 indentation_test.py --detect
```
```
  INSTRUMENT DETECTION
    probed /dev/ttyACM0     → ?
    probed /dev/ttyUSB0     → XJC F/T
    probed /dev/ttyUSB1     → LCR-6100
  ------------------------------------------------------------------
  Robot (UR5 RTDE)    : CONNECTED 177.22.22.2
  FUTEK load cell     : via robot AI0
  LCR-6100            : /dev/ttyUSB1
  XJC F/T sensor      : /dev/ttyUSB0
  Unidentified ports  : /dev/ttyACM0
```
Only `ttyUSB*`/`ttyACM*` are probed by default (the USB adapters); add
`--all-ports` to include legacy `ttyS*`. `--lcr-port`/`--xjc-port` override
detection; `--no-lcr`/`--no-xjc` skip an instrument entirely. If a sensor isn't
auto-identified but unknown ports remain, you're offered a manual pick.

## Hardware-free dry run
```bash
python3 indentation_test.py --sim              # synthesize a press; writes CSV + plots
```

## LCR mode
At startup the script reads the meter's function (`:FUNC:IMP?`), prints it, and
offers to switch to **Z-θ** (impedance + phase). The two logged fields and the
plot axis labels follow whatever mode you end up in (stored in the run
`*_meta.json`). Set frequency / level / speed on the front panel as usual.

After polling starts it runs a **live health check** — it reads the meter a few
times and prints the actual numbers (`[LCR] Live reading OK: ...`). If the fields
come back NaN it warns and lets you disable the LCR or continue. The plots show
**every finite reading**; they do *not* gate on the meter's pass/fail (NG) bit,
which is often 0 for perfectly valid impedance measurements (that bit is a
comparator/binning flag, not a validity flag). If a past run's LCR curves look
empty, re-run `analyze_indentation.py` on its CSV — the fixed analyzer will draw
them.

## End of run
When the experiment finishes (or on Ctrl-C / error) the tool lifts off the
surface and **returns to the initial position** (the `REFERENCE_POSE` + clearance
height the run started from), then releases the robot.

## XJC sensor
`xjc_sensor.py` is a self-contained reconstruction of the driver documented in
`../XJC force sensor/README_FT_CALIBRATION.md` (§3). Run its built-in self-test
against the manual's test vectors — no hardware needed:
```bash
python3 xjc_sensor.py      # prints OK for CRC, poll parse, stream parse
```
Default is **poll** mode (float precision), polled in a background thread. Wire
the sensor through a USB-RS485 adapter (24 V supply shared GND); 115200 8-N-1,
station 1 by default.

## Results — one folder per material, dataset + plots together
Each run writes everything into **`results/<material>/`**, with the material in
every filename, so a test is fully self-contained:
```
results/<material>/
  indentation_<material>_<timestamp>.csv          # the dataset
  indentation_<material>_<timestamp>_meta.json    # run metadata (incl. material)
  indentation_<material>_<timestamp>_force.png
  indentation_<material>_<timestamp>_impedance.png
  indentation_<material>_<timestamp>_phase.png
  indentation_<material>_<timestamp>_timeseries.png
```
Repeat tests on the same material collect in its folder (distinguished by
timestamp). Plots are auto-made at the end (unless `--no-plot`) and the analyzer
always writes them **next to the CSV** it's given:
```bash
python3 analyze_indentation.py results/<material>/indentation_<material>_<timestamp>.csv
```
- `*_force.png` — indentation vs force (FUTEK, with XJC Fz overlaid)
- `*_impedance.png` — indentation vs \|Z\| (or Cp)
- `*_phase.png` — indentation vs phase θ (or Rp)
- `*_timeseries.png` — **cycles over time**: a multi-panel time series
  (indentation / force / \|Z\| / phase vs elapsed time) showing each cycle one
  after another, with iteration boundaries marked (`c1`, `c2`, …)

In the indentation-vs-* figures each ramp iteration is a separate trace.
