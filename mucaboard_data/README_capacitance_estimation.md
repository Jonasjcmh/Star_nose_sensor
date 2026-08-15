# Mucaboard capacitance estimation — equations & hex-map cell definitions

This document explains, end to end, how a raw mucaboard reading becomes the
normalized `[0,1]` value used throughout the analysis, how that value is
converted into an **estimated capacitance in picoFarads (pF)**, and exactly
how every cell in the hexagonal plots (`mucaboard_data/matlab/analyze_mucaboard.m`
and `mucaboard_data/matlab/estimate_capacitance.m`) is computed.

Scripts: `mucaboard_data/matlab/estimate_capacitance.m` (+ helper functions
`read_lcr_csv.m`, `load_lcr_files.m`, `lcr_stats_by_point_depth.m`,
`muca_stats_by_point_depth.m`, `match_lcr_muca_by_depth.m`, `fit_by_point.m`,
`lcr_delta_c_over_c0.m`, `muca_delta_v_over_v0.m`). Figures are written to
`mucaboard_data/capacitive_relationship/`.

---

## 1. Raw mucaboard reading → normalized `[0,1]` value

The mucaboard uses an FT5x16 (FocalTech) touch-controller chip that reports
**16-bit raw mutual-capacitance counts**, not Farads and not an 8-bit 0–255
value. The firmware/host code (`Integration_2/sensor.py`) converts each
cell's raw count to a normalized reading with:

```
V_i = clip( (raw_i − baseline_i) / SENSITIVITY , 0, 1 ) ^ GAMMA
```

with

```
SENSITIVITY = 30.0
GAMMA       = 0.5
```

- `raw_i` — the cell's current raw 16-bit count.
- `baseline_i` — that cell's idle/untouched raw count (per-cell calibration
  offset, captured at startup).
- `clip(x, 0, 1)` — clamps the sensitivity-scaled delta into `[0, 1]`.
- `^ GAMMA` (square root, since `GAMMA=0.5`) — a gamma correction that
  boosts sensitivity to small presses.

This `V_i ∈ [0,1]` is the "muca normalized reading" / "own-cell value" used
everywhere downstream (force-vs-time plots, hex maps, and as the `x`
variable in the capacitance fit below). **It has no direct unit — it is a
chip-internal, gamma-corrected, per-cell-normalized touch-delta signal, not
a capacitance.**

---

## 2. Estimating real capacitance (pF) from `V`

### 2.1 Why a fit is needed at all

`V` (mucaboard) and `Cp_pF` (a real capacitance, in picoFarads, measured
directly with an LCR meter in `Capacitance_measurement/logs/`) are two
**different electrical quantities measured with different topologies**:

- `Cp_pF`: direct 2-wire self-capacitance of the electrode, measured with
  the electrode wired straight to an LCR meter (no touch chip involved).
- `V`: the FT5x16 chip's own internal mutual-capacitance-derived touch
  signal, gamma-corrected and normalized as in §1.

They were also **never measured simultaneously** — `Cp_pF` comes from a
separate session where the same electrodes/surfaces were wired to the LCR
meter instead of the mucaboard. So there's no first-principles unit
conversion; instead, the two datasets are linked **indirectly**, by
matching rows that share the same `(point, depth_mm)` and the same FUTEK
load-cell force sensor / calibration:

- **Flat & solid**: LCR sweep files use the same `depth_mm` labels
  (5,6,7,8,9 mm) as the mucaboard sessions → matched directly, offset = 0.
- **Hollow**: the LCR file's `depth_mm` (0–4 mm) corresponds to the
  mucaboard session's `depth_mm − 5` (5–9 mm) — confirmed by `load_cell_N`
  matching closely at that offset → matched with **+5 mm offset**.

For each matched `(point, depth)` pair, both sides are first reduced to the
**hold-phase mean** (`phase == 'hold'`) of their respective signal:

```
mean_Cp_pF(point, depth)   = mean( Cp_pF   over hold-phase rows for that point & depth )   [lcr_stats_by_point_depth.m]
mean_V(point, depth)       = mean( V_own_cell over hold-phase rows, pooled over iterations )   [muca_stats_by_point_depth.m]
```

### 2.2 The fit

A linear regression is fit **per surface**, pooling all matched
`(point, depth)` pairs for that surface:

```
Cp_pF ≈ a · V + b
```

fit by ordinary least squares (`polyfit(V, Cp_pF, 1)`). Coefficients from
the current dataset (`n` = number of matched point/depth rows):

| Surface | a (slope, pF per unit V) | b (intercept, pF) | R²    | RMSE (pF) | n  |
|---------|---------------------------|--------------------|-------|-----------|----|
| Flat    | −0.9748                   | 1.8112             | 0.216 | 0.3658    | 95 |
| Solid   | −0.8214                   | 2.3364             | 0.440 | 0.1096    | 95 |
| Hollow  | −2.0666                   | 2.8529             | 0.481 | 0.2821    | 15 |

So, e.g. for the **solid** surface: `Cp_pF ≈ −0.8214 · V + 2.3364`.

The slope is **negative** for every surface: `Cp_pF` decreases as `V`
increases (press gets deeper) — consistent with the two signals coming
from different topologies (§2.1), not a sign error.

### 2.3 Why pooled R² is low, and the per-point alternative

Pooling all 19 points into one fit mixes pads with very different absolute
capacitance baselines (intercepts) and different depth-sensitivities
(slopes, including sign flips), which drags R² down. A **separate fit per
point** (`fit_by_point.m`, `Cp_pF ≈ a_p · V + b_p` for each point `p`, using
that point's 5 depths as samples) does much better on average:

| Surface | mean per-point R² | pooled R² |
|---------|--------------------|-----------|
| Flat    | 0.564              | 0.216     |
| Solid   | 0.708              | 0.440     |
| Hollow  | 0.808              | 0.481     |

Per-point coefficients are printed to the console by
`estimate_capacitance.m` (`FIT (per point)` table, columns `point, slope,
intercept, R2, n`) and used for the small-multiples plots
`capacitance_perpoint_<surface>.png`. Use the **per-point** fit for a
specific pad if higher accuracy is needed; use the **pooled** fit only for
a rough, surface-wide estimate.

### 2.4 Converting a live mucaboard reading to pF

Given a live normalized reading `V` for a cell on a known surface (and,
ideally, its point ID), the estimated capacitance is:

```
Cp_pF_estimated = a_point · V + b_point        (preferred — use fit_by_point.m coefficients for that point)
Cp_pF_estimated = a_surface · V + b_surface     (fallback — pooled coefficients from §2.2)
```

---

## 3. Hexagonal-plot cell values

All hex maps use the mucaboard's fixed 19-point layout (`muca_layout.m`,
hex radius `8.0/sqrt(3)`), one figure per plot type with 3 panels
(flat/solid/hollow), a shared color scale (`get_cmap()`) across all 3
panels, and grey/blank hexagons for points with no data (`NaN`).

### 3.1 `capacitance_deltaC_pF_hexmap.png` — absolute capacitance swing (pF)

Per point `p`, over that point's full set of LCR-measured depths on that
surface:

```
C0(p)      = min( Cp_pF over all measured depths for point p )
Cmax(p)    = max( Cp_pF over all measured depths for point p )
ΔC(p) [pF] = Cmax(p) − C0(p)                      ≥ 0
```

`C0` here is the **minimum capacitance value actually observed for that
point**, not tied to a specific depth label — more robust than "value at
the shallowest depth" since `Cp_pF` isn't perfectly monotonic with depth
for every pad. Implemented in `lcr_delta_c_over_c0.m` (1st output,
`delta_pf`). Color scale label: `delta-C (pF)`.

### 3.2 `capacitance_deltaC_hexmap.png` — relative capacitance change ΔC/C0

Same `C0(p)` and `Cmax(p)` as above, normalized by the baseline:

```
ΔC/C0 (p) = ( Cmax(p) − C0(p) ) / C0(p)           ≥ 0 (NaN if C0(p) == 0)
```

This is the standard capacitive-sensing "delta-over-baseline" normalization
— it corrects for each pad having a very different absolute self-
capacitance baseline (the main driver of the low pooled-fit R² in §2.3).
Implemented in `lcr_delta_c_over_c0.m` (2nd output, `delta_over_c0`). Color
scale label: `delta-C / C0 (fraction)`.

Both §3.1 and §3.2 are LCR-measured (real pF), so **hollow only has data
for 3 points** (3/10/14) — that's all the direct-LCR-connected hollow
dataset covers; the other 16 hexagons are grey.

### 3.3 `muca_deltaV_hexmap.png` — relative reading change ΔV/V0 (muca-only, no LCR)

Same idea as §3.2 but computed **entirely from the mucaboard's own
normalized signal** `V` (§1), not `Cp_pF` — this gives full 19-point
coverage on all 3 surfaces, since it doesn't depend on the LCR dataset's
point coverage. Per point `p`:

```
V0(p) = V(p, depth = 5 mm)     (shallowest depth with real hold-phase per-point data;
                                 depth_mm=0 rows are idle rows, not usable)
V1(p) = V(p, depth = 10 mm)    (deepest depth)

ΔV/V0(p) = (V1(p) − V0(p)) / V0(p),   if |V0(p)| ≥ min_v0 (default 0.05)
         = NaN (grey),               otherwise
```

The `min_v0` guard exists because a `V0` near the sensor noise floor (e.g.
`V0 = 0.003`) makes the ratio numerically unstable (swings into the
hundreds/thousands), which would swamp the color scale for every other
point. Implemented in `muca_delta_v_over_v0.m`. Color scale label:
`delta-V / V0 (fraction)`.

### 3.4 Raw cross-talk hex maps (`analyze_mucaboard.m`, not `estimate_capacitance.m`)

These are unrelated to the capacitance estimation above — they show, per
`POINT_ID` pressed, the (mean- or median-of-iterations) **raw normalized
reading `V`** of every one of the 19 cells at a fixed depth, i.e. how much
each cell "feels" a press centered on `POINT_ID` (cross-talk). No delta,
no baseline, no fit — just `V_i` (or `median_i(V_i)` over iterations)
directly from §1, plotted with the same hex layout and color scale
machinery as §3.

---

## 4. Quick reference — all equations

```
1) Raw → normalized reading (per cell i, sensor.py):
   V_i = clip((raw_i − baseline_i) / 30.0, 0, 1) ^ 0.5

2) Muca reading → estimated capacitance (per surface, pooled fit):
   flat:   Cp_pF ≈ −0.9748 · V + 1.8112   (R²=0.216, n=95)
   solid:  Cp_pF ≈ −0.8214 · V + 2.3364   (R²=0.440, n=95)
   hollow: Cp_pF ≈ −2.0666 · V + 2.8529   (R²=0.481, n=15)
   (or per-point a_p, b_p from fit_by_point.m — see console output / capacitance_perpoint_*.png)

3) Hex map — absolute ΔC (pF), per point:
   C0(p)   = min(Cp_pF over all depths for point p)
   ΔC(p)   = max(Cp_pF over all depths for point p) − C0(p)

4) Hex map — relative ΔC/C0, per point:
   ΔC/C0(p) = ΔC(p) / C0(p)

5) Hex map — relative ΔV/V0, per point (muca-only, no LCR):
   ΔV/V0(p) = (V(p,10mm) − V(p,5mm)) / V(p,5mm),  only if |V(p,5mm)| ≥ 0.05
```
