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
1. **Position (X/Y jog)** — keyboard-jog the TCP in base X/Y over the target.
2. **Surface (Z jog)** — jog Z down to first contact; that pose is indentation 0 mm
   (coincident with the shared `REFERENCE_POSE` / UR-control frame).
3. **Save/Load calib** — the point (surface pose) is saved to
   `calib_session_<name>.json`. Later runs **load** it to reuse the same X/Y and
   only re-jog **Z** (the normal between-experiments height change).
4. **Range** — max indentation depth (mm).
5. **Iterations** — number of ramp cycles.
6. **Collect** — per iteration: press 0→max (continuous), hold, retract max→0,
   logging continuously to a streaming CSV (flat RAM).

### Jog keys
- **X/Y:** `a`/`d` = −X/+X, `w`/`s` = +Y/−Y (or arrow keys) · `-`/`+` step · `ENTER` confirm · `q` abort
- **Z:** `s`/`↓` down (into surface), `w`/`↑` up · `-`/`+` step · `ENTER` sets contact · `q` abort

## Run
```bash
python3 indentation_test.py                    # ask everything
python3 indentation_test.py --max-depth 3 --iterations 5
python3 indentation_test.py --no-xjc           # robot + FUTEK + LCR only
python3 indentation_test.py --new-point        # force a full X/Y + Z jog
UR_ROBOT_IP=127.0.0.1 python3 indentation_test.py
```
Useful flags: `--lcr-port /dev/ttyUSB0 --xjc-port /dev/ttyUSB1 --xjc-station 1 --xjc-baud 115200`,
`--step 0.05 --hold 1 --recover 1 --rate 60`, `--no-lcr`, `--no-plot`.

## LCR mode
At startup the script reads the meter's function (`:FUNC:IMP?`), prints it, and
offers to switch to **Z-θ** (impedance + phase). The two logged fields and the
plot axis labels follow whatever mode you end up in (stored in the run
`*_meta.json`). Set frequency / level / speed on the front panel as usual.

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

## Results
Auto-plotted at the end (unless `--no-plot`) and re-runnable:
```bash
python3 analyze_indentation.py logs/indentation_<timestamp>.csv
```
Produces, in `plots/`:
- `*_force.png` — indentation vs force (FUTEK, with UR Fz / XJC Fz overlaid)
- `*_impedance.png` — indentation vs \|Z\| (or Cp)
- `*_phase.png` — indentation vs phase θ (or Rp)

Each ramp iteration is a separate trace.
```
