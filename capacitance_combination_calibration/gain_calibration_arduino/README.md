# gain_calibration_arduino — calibrating the FT5316's sensitivity

**The question this folder answers:** can the muca board's sensitivity be changed
on the chip itself, or are we stuck with one fixed scale forever?

**Answer:** it can be changed. The scale has never moved because the upstream
firmware leaves the gain register alone, not because the chip cannot do it.

---

## 1. The chip and the knob

The muca board is built on the **FocalTech FT5316DME**, a mutual-capacitance
touch-panel controller. `Technical_Datasheet/FT5316DME_info-uift5316dme.pdf`
(one level up, in `../Technical_Datasheet/`) is FocalTech's *preliminary*
datasheet. It gives the analog envelope and nothing else:

| From the datasheet | Value | Why it matters here |
|---|---|---|
| ADC | 12-bit | the board reports 16-bit words, so a frame value is an accumulation, not one conversion |
| Lines | 21 TX × 12 RX | = **252**, exactly the `SKIN_CELLS` the board streams |
| Max channel capacitance | 60 pF | our combinations sit far below this |
| **Optimal mutual capacitor** | **1 pF – 4 pF** | our 0.98–2.53 pF caps are *inside* the sweet spot |
| Auto-calibration | "insensitive to capacitance and environmental variations" | a feature for a phone, a hazard for us — see §6 |

It contains **no register map**. The gain knob lives in the FT5x06/FT5x16
*factory-mode* register set that the Muca Arduino library drives:

| Register | Library call | Effect |
|---|---|---|
| `0x00` = `0x40` (MODE_TEST) | — | enter factory mode (**required first**) |
| **`0x07`** | **`muca.setGain(n)`** | **AFE analog gain — counts per pF** |
| `0x88` | `muca.setReportRate(3..14)` | scan rate ↔ integration time ↔ noise |
| `0xA0`, `0xA7` | `setConfig()`, `init()` | auto-calibration behaviour |
| `0x00` = `0xC0` | `muca.useRawData(true)` | raw-data streaming |

Register `0x07` is **volatile** — it must be re-applied after every reset.

Source: [Muca.cpp](https://github.com/muca-board/Muca/blob/master/Muca.cpp),
[Muca.h](https://github.com/muca-board/Muca/blob/master/Muca.h).

---

## 2. Why we need to touch it

Re-analysing `../logs/flat_calibration_muca_lcr_session_20260826_225636.csv`
(the 2026-08-26 LCR + muca session, 6 combinations × 5 points) with
`analyze_gain_sweep.py`:

```
   gain  pts   counts/pF     R^2   span%   Cp=0 fit  rail gap    noise  res [pF]  verdict
 native    5       553.8  0.9959    1.31      65726       353      1.4    0.0026  CLIPPING
```

Read that row carefully:

- **`counts/pF` = 554** — the link between Cp and raw counts is real and strong
  (R² = 0.996 across six capacitors, on all five points independently).
- **`span%` = 1.31** — the entire 0.98 → 2.53 pF sweep moves the reading by
  ~900 counts, **1.3 % of the 16-bit range**. Almost the whole scale is unused.
- **`Cp = 0` fit = 65,726** — extrapolated to no capacitance, the fit lands
  **above the 65,535 ceiling**. The top of the range is clipped, and no
  software calibration recovers a clipped range.
- The slope is **negative**: more capacitance gives a *lower* reading, which is
  backwards for mutual-capacitance coupling.

The capacitors are inside the datasheet's optimal 1–4 pF window, so the
capacitor is not the problem. Both symptoms point the same way: at the default
gain the AFE is being driven past its linear range, and we are reading the
compressed top of the curve.

`results/` holds this characterisation as generated output — the "before"
picture.

---

## 3. What is in this folder

```
gain_calibration_arduino/
├── firmware/
│   ├── Muca_Raw_original/Muca_Raw_original.ino   verbatim upstream — the control
│   └── Muca_Raw_gain/Muca_Raw_gain.ino           adds live gain control
├── muca_gain_link.py        serial owner: streams frames AND sends commands
├── gain_sweep_collector.py  the experiment (LCR ground truth × gain sweep)
├── analyze_gain_sweep.py    compares gains, recommends one, plots
├── selftest_dataset_format.py  proves the CSV is a drop-in (no hardware needed)
├── logs/                    sweep CSVs land here
└── results/                 tables + figures from the analyzer
```

### The two firmwares

**`Muca_Raw_original.ino`** is the upstream example, unchanged. It is what the
board has been running, and therefore what produced every dataset in `../logs/`,
`../../mucaboard_data/` and `../../mucaboard_data_raw/`. Note its
`// muca.setGain(100);` — commented out upstream, which is exactly why the scale
has never changed. **Keep this to reproduce the historical scale.**

**`Muca_Raw_gain.ino`** streams the *same* 252 comma-separated integers at the
*same* 115200 baud, byte-for-byte — so `sensor_raw.py`, `sensor.py` and the
visualizers all keep working unmodified. It only *adds* a serial command
channel:

| Command | Effect |
|---|---|
| `G<n>` | set analog gain (register `0x07`), replies `# GAIN n READBACK m` |
| `G?` | report the gain currently set |
| `R<n>` | set report rate, 3–14 |
| `P` / `S` | pause / resume streaming |
| `I` | firmware + configuration info |
| `A` | run auto-calibration (**see §6 — not during a sweep**) |
| `H` | help |

Every reply starts with `#`, and the library's own chatter starts with `[`.
Neither ever starts with a digit, so a reader that only accepts digit-leading
lines as data — which is what `sensor_raw.py` already does — is unaffected.

`STARTUP_GAIN` is `0` by default, meaning *do not touch register `0x07` at
boot*. Flashing this sketch alone therefore changes nothing measurable; the
sweep sets the gain explicitly at every step.

### Why a new Python serial module

Only one process can own a serial port, and `sensor_raw.py` /
`Integration_2/sensor.py` are read-only consumers used by the whole existing
pipeline. `muca_gain_link.py` is self-contained so **both of those stay
untouched** — it just keeps `USED_CELLS` in sync with `sensor_raw.py`, which
remains the source of truth for that mapping.

---

## 4. Running the sweep

**Before you start:** LCR-6100 set to Cp-Rp / 20 kHz / FAST / 1.0 V on the front
panel, both instruments on their own USB ports, and
`firmware/Muca_Raw_gain/Muca_Raw_gain.ino` flashed to the board. The collector
refuses to run against the original firmware rather than silently logging the
same gain N times.

```bash
python gain_sweep_collector.py
python gain_sweep_collector.py --prefix gain_sweep_solid --gains 1,4,8,12,16,20,24,28,31 --dwell 5
```

The workflow mirrors `../combination_calibration_collector.py` exactly, with one
axis added:

- **Phase 0 — baseline.** Nothing attached. Sweep every gain. This measures the
  Cp = 0 point per gain, which is what decides whether a gain clips. Do not skip it.
- **Phase 1 — per combination.** LCR once (ground truth, gain-independent), then
  per point: the operator names the point, and the script walks every gain,
  settling and logging at each.

As before, there is **no fixed name→channel table** — the operator names points
at the bench, and the full 19-cell snapshot is logged every frame so nothing is
lost regardless of which cell actually moved.

### The output CSV

Columns are **identical to `../combination_calibration_collector.py`, in the
same order**, with two new columns appended at the end:

```
phase, combination_index, combination_label, point_seq, point_label,
timestamp, datetime, elapsed_s, Cp_pF, Rp_ohm, lcr_ok, cell_1 … cell_19,
gain, gain_readback
```

So a sweep file is a drop-in for every reader of the older logs
(`../plot_muca_bars.py`, `../plot_muca_combo_grid.py`), the two generations can
be concatenated, and old files can be replayed as a single `native`-gain sweep.
`gain_readback` is the value read back **off the chip**, not merely what we
asked for. The only new `phase` value is `baseline`; anything filtering on
`phase == 'muca'` ignores it automatically.

`selftest_dataset_format.py` verifies all of this without any hardware — it
checks the column order against the original collector, generates a synthetic
sweep with a known ground truth, confirms the analyzer recovers it and flags the
railing gain, and runs both legacy plotters over the new file:

```bash
python selftest_dataset_format.py      # 14 checks, nothing left behind
```

**One caveat on the legacy plotters.** They average *every* `phase == 'muca'`
row for a point, which in a sweep file means averaging across gains. They parse
the file correctly — that is what the self-test asserts — but their per-point
bars mix gains and are not meaningful for sweep data. Use
`analyze_gain_sweep.py` for sweeps and the legacy plotters for single-gain
sessions.

---

## 5. Reading the results

```bash
python analyze_gain_sweep.py logs/gain_sweep_session_*.csv \
    --legacy ../logs/flat_calibration_muca_lcr_session_20260826_225636.csv
```

The analyzer finds the responding cell **from the data** (the cell whose median
moves most across combinations), fits raw counts against the LCR's Cp per gain,
and scores four things. A gain has to survive all four:

1. **Sensitivity** — |slope| in counts/pF. Higher is better, *only if* 2–4 hold.
2. **Linearity** — R² of the fit. High slope with poor R² means the AFE is
   bending, not measuring.
3. **Headroom** — distance from both rails, including the extrapolated Cp = 0
   intercept. Outside `[0, 65535]` means clipping, which is unrecoverable.
4. **Resolution** — frame noise ÷ |slope| = the smallest capacitance change the
   board can actually see, in pF. **This is the tie-breaker**: raising the gain
   raises the slope *and* the noise, so more gain is not automatically better.

It prints a ranked table, writes `results/gain_summary.csv` and
`results/gain_per_point.csv`, and renders a four-panel figure. Passing
`--legacy` puts the old default-gain data in the same table as the new sweep,
which is the comparison that matters.

---

## 6. Two things gain will *not* fix

**Gain is global.** One AFE setting for the whole chip, so it cannot equalise
cell-to-cell spread. That stays a software calibration — the
`SENSITIVITY = 30.0` / `GAMMA = 0.5` layer in `Integration_2/sensor.py`, which
is independent of this register and unaffected by anything here.

**Auto-calibration fights absolute measurement.** The datasheet sells
"auto-calibration: insensitive to capacitance and environmental variations".
For a phone screen that is the point; for measuring absolute capacitance it
means the chip silently re-baselines, so the same physical capacitor can read
differently minutes apart. The `A` command exists for deliberate experiments
only — **never run it mid-sweep**, it invalidates the comparison between gains.
Whether `0xA0`/`0xA7` leave it active in our configuration is still worth
verifying directly.

---

## 7. If no gain comes out clean

If every gain in the sweep clips or fits poorly, the fixed capacitance the cell
sees is too large before the capacitor under test is even attached. That is a
wiring problem, not a register problem: shorten the leads, guard them, or reduce
the pad area, then re-run the sweep.
