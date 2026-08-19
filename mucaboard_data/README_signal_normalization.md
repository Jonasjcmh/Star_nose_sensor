# Mucaboard raw-signal normalization — scientific background

This document explains the physical and mathematical basis of the normalization
applied to every mucaboard reading before it ever reaches a CSV file, why each
step exists, a worked numeric example, and where it is implemented in this
repository. It is a companion to `README_capacitance_estimation.md` (which
covers estimating capacitance *from* this normalized signal) — this document
covers the earlier stage: how the raw sensor gets turned into the `cell_1`…
`cell_19` values in the first place.

---

## 1. What the sensor actually measures

The mucaboard uses an FT5x16-family FocalTech touch controller: a **mutual-
capacitance** sensing IC. Mutual-capacitance sensing works by driving one set
of electrodes (TX) and measuring induced charge on a crossing set (RX); a
finger or, here, a rigid indentation probe, locally changes the fringing
field between a TX/RX pair, and the chip reports that as a change in a raw
per-node count. This is the same sensing principle used in essentially all
modern capacitive touchscreens and touch tablets, going back to the original
multi-touch capacitive tablet work of Lee, Buxton & Smith (1985) and
described in detail in Barrett & Omote's review of projected-capacitive touch
technology (2010) — see References.

Two things follow directly from this sensing principle, and both are exactly
what the normalization pipeline exists to correct for:

- **Every node/pad has its own idle baseline.** Trace length, local
  dielectric environment (the silicone/elastomer dome geometry, in this
  board's case), and per-channel manufacturing variance all shift each
  electrode's *untouched* raw count differently. This is a standard,
  well-documented property of capacitive sensor arrays generally, not
  specific to touchscreens — see Baxter's *Capacitive Sensors: Design and
  Applications* (1996), the standard reference text for the field.
- **The baseline itself drifts** with temperature, humidity, and time. Any
  capacitive sensing system that wants a stable "zero" has to re-establish
  it, typically by capturing a fresh baseline at startup — the same problem
  addressed in the capacitive-displacement-sensing literature, e.g. Kang,
  Lee & Moon's drift-compensation technique for capacitive displacement
  sensors (2010).

## 2. The raw signal is not directly usable

Because of §1, a *raw* FT5x16 count by itself tells you nothing on its own —
`raw = 8412` means nothing without knowing that particular channel's current
baseline. Sixteen-bit raw counts also have no natural upper bound tied to
"how hard is this being pressed," so two different channels' raw counts
aren't comparable to each other, and neither are two sessions' worth of raw
counts from the same channel if the baseline has drifted between them. A hex
map that colors 19 cells by raw count would mostly be showing you
manufacturing variance, not touch.

## 3. The normalization pipeline

This project's firmware/host code (`Integration_2/sensor.py`) converts every
raw count to a normalized reading with:

```
V_i = clip( (raw_i − baseline_i) / SENSITIVITY , 0, 1 ) ^ GAMMA
```

with `SENSITIVITY = 30.0`, `GAMMA = 0.5`. Each term solves a distinct,
well-established problem:

### 3.1 Baseline subtraction — `raw_i − baseline_i`

`baseline_i` is captured from the very first raw frame read after the sensor
connects (`Integration_2/sensor.py`, `_read_loop()`): the board must be
untouched at that moment. Subtracting it converts an arbitrary per-channel
absolute count into a **relative** signal where 0 uniformly means "no
touch," on every channel, regardless of that channel's individual idle
count. This is the standard offset-calibration step in capacitive sensing
(Baxter, 1996; Kang et al., 2010) — without it, the 19 cells could not be
compared to one another or plotted on a shared color scale.

### 3.2 Sensitivity scaling — `÷ SENSITIVITY`

Dividing by a fixed gain constant (empirically tuned to this
board/tip/elastomer combination) rescales the arbitrary-magnitude raw delta
into a working range where "a full press" lands near 1 and "no press" lands
near 0. This is a simple linear gain normalization — the capacitive-sensing
equivalent of choosing a sensible full-scale range for an ADC.

### 3.3 Saturating clip — `clip(..., 0, 1)`

Clamping bounds every output to `[0, 1]` regardless of noise spikes or an
unusually hard press, so nothing downstream (plots, thresholds, control
logic) needs to handle an unbounded or negative signal. This is a standard
saturating-nonlinearity guard in sensor front-ends generally.

**This step is the one with a real, unavoidable cost: it is not invertible.**
Any two raw deltas that both exceed `30` (in this dataset's scaling) become
indistinguishable — both clip to exactly `1.0` — and any raw delta at or
below `baseline_i` clips to exactly `0.0`. Once clipped, the original
magnitude is gone from the record; no downstream calculation can recover it.

### 3.4 Gamma correction — `^ GAMMA (0.5)`

Raising a linear `[0,1]` fraction to a power less than 1 (here, a square
root) stretches small values upward and compresses values near the top —
e.g. `0.01 → 0.10` (a 10× boost) while `0.81 → 0.90`. This exists for the
same underlying reason gamma encoding exists in imaging and video: the
perceptually/functionally important part of the signal is often
disproportionately concentrated near the low end, so a linear encoding wastes
resolution there. Poynton's *Rehabilitation of gamma* (1998) is the standard
technical treatment of why and how this kind of power-law re-encoding is
applied to sensor/display signals.

The deeper justification for *why* a power-law transform matches perceived/
functional sensitivity comes from psychophysics: Stevens' power law (Stevens,
1957) established that perceived intensity of a physical stimulus generally
follows `ψ = k·φ^n` — a power-law relationship between physical stimulus
magnitude `φ` and perceived magnitude `ψ`, with the exponent `n` varying by
modality but very often `< 1` for stimuli where small increments near zero
matter more than equal-sized increments near saturation. A `GAMMA = 0.5`
transform is exactly this kind of compressive-near-the-top,
expansive-near-the-bottom re-encoding — applied here so a barely-touching
press is clearly distinguishable from noise, at the deliberate cost of finer
discrimination between "hard press" and "very hard press."

## 4. Worked example

Suppose one cell has `baseline_i = 8000` (its untouched raw count, captured
at session start) and a press produces `raw_i = 8090`.

```
step 1  raw_i − baseline_i         = 8090 − 8000       = 90
step 2  ÷ SENSITIVITY (30.0)       = 90 / 30            = 3.0
step 3  clip(3.0, 0, 1)            = 1.0        <- clipped! magnitude 90 vs 30 is now lost
step 4  1.0 ^ 0.5                  = 1.0
```

Now a lighter press, `raw_i = 8006`:

```
step 1  8006 − 8000                = 6
step 2  6 / 30                     = 0.20
step 3  clip(0.20, 0, 1)           = 0.20         <- not clipped
step 4  0.20 ^ 0.5                 = 0.4472
```

Notice step 4: a raw delta that is only **20%** of the way to saturation
produces a normalized value that is **44.7%** of the way to 1.0 — the gamma
step's expansion of small signals, directly visible.

**Python** (exactly matching `Integration_2/sensor.py`):
```python
SENSITIVITY = 30.0
GAMMA = 0.5

def normalise(raw_i, baseline_i):
    ratio = max(0.0, min((raw_i - baseline_i) / SENSITIVITY, 1.0))
    return ratio ** GAMMA
```

**Partial inverse** (recovers the pre-gamma, pre-clip ratio — only exact for
non-clipped values; see §3.3 and the note below):
```python
def approx_inverse(V_i):
    return V_i ** 2  # = (raw_i - baseline_i) / SENSITIVITY, only if V_i was not clipped
```
```matlab
% MATLAB/Octave equivalent, as used ad hoc in this project's analysis:
delta_from_baseline = V_i .^ 2 * SENSITIVITY;   % recovers (raw_i - baseline_i), non-clipped values only
```

## 5. What this means for downstream analysis

- **Every value in every mucaboard CSV in this repo (`cell_1`…`cell_19`) is
  already the fully-processed `V_i`**, not a raw count. All of the MATLAB
  analysis in `mucaboard_data/matlab/` (V0, Press, ΔV, the hex maps, the
  capacitance fit in `estimate_capacitance.m`) operates on this normalized
  signal.
- **The transform is one-way.** §3.3's clipping is lossy by construction —
  the exact pre-clip magnitude is unrecoverable for any value that hit 0 or
  1 exactly (common in this dataset: e.g. many release-phase readings sit at
  exactly `0.0000`, and some heavily-pressed points hit exactly `1.0000`).
  Even for non-clipped values, recovering `raw_i` itself (not just
  `raw_i − baseline_i`) is impossible from these logs, because
  `baseline_i` is only ever held in memory for the duration of one session
  (`Integration_2/sensor.py`'s `_calibration` variable) and is never written
  to any file this project produces.
- **This is why `estimate_capacitance.m` uses an independent LCR-meter
  measurement** rather than trying to invert this formula back to a physical
  capacitance — the gamma/clip transform is specifically not designed to be
  inverted, so an independent ground-truth measurement is the correct way to
  relate `V` back to a real physical quantity (picofarads), and that
  empirical fit is per-surface/per-point rather than a fixed formula for
  exactly this reason.

## 6. Where this is implemented

| Step | File |
|---|---|
| Raw count → `_calibration` (baseline capture) | `Integration_2/sensor.py`, `_read_loop()` |
| The normalization formula itself | `Integration_2/sensor.py`, `_normalise()` |
| Reading `V_i` into a CSV row | `mucaboard_data/mucaboard_ramp_collector.py`, `vals = sensor.get_values()` |
| Consuming `V_i` in MATLAB (no further transform) | `mucaboard_data/matlab/read_muca_csv.m` |
| Relating `V_i` back to real capacitance (independent fit, not an inverse) | `mucaboard_data/matlab/estimate_capacitance.m`, `README_capacitance_estimation.md` |

---

## References

1. Barrett, J., & Omote, R. (2010). Projected-Capacitive Touch Technology.
   *Information Display*, 26(3). https://doi.org/10.1002/j.2637-496x.2010.tb00229.x
   — mutual-capacitance touch sensing principle.

2. Lee, S. K., Buxton, W., & Smith, K. C. (1985). A multi-touch three
   dimensional touch-sensitive tablet. *Proceedings of the SIGCHI Conference
   on Human Factors in Computing Systems (CHI '85)*.
   https://doi.org/10.1145/317456.317461
   — foundational capacitive multi-touch sensing paper.

3. Baxter, L. K. (1996). *Capacitive Sensors: Design and Applications*. IEEE
   Press. https://doi.org/10.1109/9780470544228
   — standard reference text on capacitive sensor design, baseline/offset
   behavior, and per-channel variance.

4. Kang, D., Lee, W., & Moon, W. (2010). A technique for drift compensation
   of an area-varying capacitive displacement sensor for nano-metrology.
   *Procedia Engineering*, 5. https://doi.org/10.1016/j.proeng.2010.09.134
   — baseline drift compensation in capacitive sensors.

5. Stevens, S. S. (1957). On the psychophysical law. *Psychological
   Review*, 64(3), 153–181. https://doi.org/10.1037/h0046162
   — the power-law relationship between physical stimulus magnitude and
   perceived/functional magnitude that motivates a gamma < 1 transform.

6. Poynton, C. (1998). Rehabilitation of gamma. In *Human Vision and
   Electronic Imaging III* (SPIE Proceedings, Vol. 3299).
   https://doi.org/10.1117/12.320126
   — technical treatment of gamma/power-law re-encoding in sensor and
   display signal chains.
