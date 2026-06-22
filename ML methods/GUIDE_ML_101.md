# Machine Learning 101 for the Star-Nose Capacitive Sensor

**From theory to implementation — static localisation & dynamic drawing reconstruction**

This guide is written for *your* setup: a 19-cell hexagonal capacitive tactile
sensor mounted on a UR5, with a FUTEK load cell, logging to CSV. It is split
into two problems you asked about:

- **Part A — Static:** where is a press, and how much area does it cover?
- **Part B — Dynamic:** how is the pointer moving, in what direction, what
  shape, and can we redraw the path it traced?

Every concept is paired with a runnable script in this folder. Read a section,
run the matching script, look at the output, then move on.

---

## 0. Background you need first

### 0.1 What the sensor gives you

The firmware exposes **19 cells** arranged on a hex lattice. Each raw count is
normalised in `sensor.py` as:

```
value_i = clip( (raw_i - baseline_i) / SENSITIVITY , 0..1 ) ** GAMMA
```

with `SENSITIVITY = 30`, `GAMMA = 0.5`. The `GAMMA = 0.5` is a square-root
compression: it boosts weak responses so far-away cells still contribute. Keep
this in mind — your features live in a *perceptually warped* activation space,
not raw capacitance. That is fine for ML, but if you ever want true physical
linearity (e.g. force ∝ capacitance), invert the gamma first: `value**2`.

The 19 cell coordinates (mm, sensor-local) are the single source of truth and
are encoded in `snm_common.POINTS_MM`:

```
        (-8,14) (0,14) (8,14)
    (-12,7)(-4,7)(4,7)(12,7)
 (-16,0)(-8,0)(0,0)(8,0)(16,0)
    (-12,-7)(-4,-7)(4,-7)(12,-7)
        (-8,-14)(0,-14)(8,-14)
```

Nearest-neighbour spacing is ~8 mm. **This 8 mm is your native pixel size — and
the whole point of ML/interpolation is to localise *below* 8 mm.**

### 0.2 Why this is a learning problem (and where labels come from)

You do not have to hand-label anything. The **UR5 is the ground-truth oracle**:
it drives the pointer to a commanded `(pos_x_mm, pos_y_mm)` at a known depth,
and your logger records the sensor frame at the same instant. So every row of
`frames.csv` is a `(input = 19 cells) → (target = known x,y,depth)` example.
This is classic **supervised regression**.

### 0.3 Your data, already prepared

```
ML_model/datasets/frames.csv   ~28k rows — one row per sensor sample
ML_model/datasets/events.csv   ~500 rows — one aggregated row per press
```

`frames.csv` columns: `session, point_id, pos_x_mm, pos_y_mm, tcp_z_mm, fz_N,
cell_1..cell_19, n_active, centroid_x_mm, centroid_y_mm, spread_mm,
diameter_est_mm`.

`events.csv` adds peak/mean aggregates per press: `depth_mm, fz_peak_N,
diameter_est_mm, peak_centroid_*`, plus `peak_cell_i` and `mean_cell_i`.

You already trained Random Forests (`saved_models/*_rf.pkl`) — this guide
explains what they're doing, how to evaluate them honestly, and what else to
reach for.

### 0.4 The golden rule of evaluation: split by *session*, not by row

Frames from the same press are nearly identical. If you shuffle rows randomly
into train/test, the model "memorises" and you get fantasy accuracy. Always do
**leave-one-session-out** (group) cross-validation so the test data is a press
the model never saw. All scripts here use `GroupKFold` on the `session` column.

---

# Part A — Static: position & covering area

**Question:** from one 19-cell frame, where is the contact and how big is it?

### A.1 The physics baseline — weighted centroid (no ML)

The center of pressure is the activation-weighted average of cell positions:

```
c = ( Σ_i  w_i · p_i ) / ( Σ_i w_i )       w_i = activation of cell i,  p_i = (x_i,y_i)
```

This is parameter-free, runs in microseconds, and is genuinely hard to beat for
a single clean contact. **Always compute it first as your baseline.** It is in
`snm_common.weighted_centroid()`. Spread (RMS radius) and contact area follow
from the same weights.

Run it:

```bash
python "ML methods/snm_common.py"        # prints features for the first frames
```

### A.2 Where the baseline fails — and why you want ML

The centroid is biased because:

- **Edge bias:** near the rim, missing cells on one side pull the centroid
  inward. A model can learn this distortion.
- **Non-linear cell response** (the gamma, plus dome curvature and per-cell
  gain differences) means equal force ≠ equal activation across cells.
- **Multi-touch / odd footprints:** a single centroid can't represent two
  contacts or an elongated tip.

ML *learns the inverse map* from the raw 19-vector to true position, correcting
all of the above from data — using the UR5 labels.

### A.3 The model ladder (pick by data size & needs)

| Model | When to use | Pros | Cons |
|---|---|---|---|
| Weighted centroid | always (baseline) | zero training, instant, interpretable | biased at edges, single contact only |
| k-NN regressor | small data, smooth field | trivial, interpolates well | needs to store all data, no extrapolation |
| **Random Forest** (you have this) | tabular 19+features, moderate data | robust, handles non-linearity, feature importance | not smooth, larger model files |
| Gaussian Process | small data, need **uncertainty** | gives ± confidence per prediction | O(n²), slow >~4k points |
| MLP / small CNN | lots of data, want sub-mm | best accuracy, learns spatial patterns | needs more data + tuning |

For 19 inputs and a few thousand presses, **Random Forest is the right default**
(which is why your existing pipeline uses it). Add a **Gaussian Process** when
you care about *how confident* a position estimate is (useful for rejecting
ambiguous contacts).

### A.4 Feature engineering

Don't feed only the 19 raw cells. Give the model the raw field **plus** the
physically-meaningful summaries so trees can split on them directly:

```
[cell_1..cell_19] + [centroid_x, centroid_y, spread, n_active, area, diameter]
```

This is exactly what `snm_common.design_matrix()` builds. Engineered features
act as a strong prior and usually cut error noticeably with tiny cost.

### A.5 "Covering area" for a static push

You asked specifically about the **area covered** by a static pushing point.
Two complementary estimates, both in `snm_common`:

1. **Soft hex-coverage** (`contact_area_mm2`): each active cell contributes its
   hex footprint (~`8² · √3/2 ≈ 55 mm²`) scaled by activation. Good, simple,
   monotone with contact size.
2. **Equivalent-disc diameter** (`diameter_est_mm = 2·√(A/π)`): converts area
   to a single intuitive number to compare tip sizes / press depths.

If you want a *supervised* area/diameter model (you have `diameter_est_mm` and
press `depth_mm` labels in `events.csv`), train a regressor exactly like
position — same script, swap the target column. Your existing
`diameter_estimator_rf.pkl` already does this.

### A.6 Implementation — run it

```bash
pip install -r "ML methods/requirements.txt"

# Cross-validated comparison of baseline vs kNN vs RF (vs GP if data is small)
python "ML methods/train_static.py"

# Train on the aggregated presses instead of every frame
python "ML methods/train_static.py" --use-events

# Save the best model to ML_model/saved_models/position_regressor_static.pkl
python "ML methods/train_static.py" --save

# Use the saved model (or just the analytic features) on one frame
python "ML methods/infer_static.py"
```

`train_static.py` prints, for each model, the **session-held-out RMSE in mm** —
the number to put in your paper. Expect the learned models to beat the centroid
baseline mostly near the edges; report both.

### A.7 Hooking it into the live system

In your real-time loop you already have `sensor.get_values()` returning 19
values. To get a live position estimate:

```python
import snm_common as snm
from infer_static import load_model, predict_position
bundle = load_model()                      # once, at startup
xy = predict_position(sensor.get_values(), bundle)   # each frame -> (x, y) mm
```

---

# Part B — Dynamic: motion, direction, shape & drawing reconstruction

**Question:** the pointer *moves*. Estimate its trajectory, direction, the shape
of the motion, and redraw what was traced.

The key shift from Part A: you now work with a **time series of frames**, and
position is just step one. Reconstruction = localise each frame, then stitch.

### B.1 Step 1 — per-frame localisation (sub-cell)

Reuse Part A: each frame → a centroid (or your trained model). Because
activation bleeds across neighbours, the centroid moves *continuously* between
cells, giving you resolution finer than the 8 mm pitch. This continuous
interpolation is what makes drawing reconstruction possible at all.

### B.2 Step 2 — smoothing & tracking (Kalman filter)

Raw per-frame centroids are jittery and snap between cells. A **constant-
velocity Kalman filter** fuses each new measurement with a motion model and
returns a smooth `[x, y, vx, vy]`. From the velocity you get, for free:

- **speed** = ‖(vx, vy)‖
- **direction / heading** = `atan2(vy, vx)`

No training required. Implemented in `dynamic_tracking.py` (`CVKalman`). This is
the standard, lightweight choice; only reach for heavier sequence models (B.5)
if you need to *classify* gestures, not just track them.

```bash
python "ML methods/dynamic_tracking.py"     # speed, heading, path descriptors
```

### B.3 Step 3 — directionality & global shape descriptors

`dynamic_tracking.path_descriptors()` summarises a whole stroke:

- **principal direction** (PCA / SVD of the points): the dominant axis of the
  motion in degrees — your "directionality of the displacement".
- **straightness** = net displacement / path length (1.0 = straight line,
  →0 = scribble/loop).
- **total turning** = accumulated absolute heading change (how curvy).

These three scalars already let you distinguish "a straight horizontal swipe"
from "a circle" from "a zig-zag" without any deep learning.

### B.4 Step 4 — reconstruct the drawing

`reconstruct_drawing.py` runs the full chain:

1. **gate** frames by total activation → drop pen-up segments,
2. **localise** every pen-down frame (centroid / model),
3. **Kalman smooth** the path,
4. **fit a smoothing spline** (arc-length) for a clean continuous curve,
5. **render** the stroke over the sensor layout to a PNG.

```bash
# Demo on one session of frames.csv (treated as a pseudo-stroke)
python "ML methods/reconstruct_drawing.py" --out reconstruction.png

# Your own recorded swipe (CSV with cell_1..cell_19 in time order)
python "ML methods/reconstruct_drawing.py" --csv my_swipe.csv --out my_draw.png
```

> Note: `frames.csv` is made of *discrete presses*, not a continuous swipe, so
> the demo path will look like jumps between points. To see real reconstruction,
> record a session while moving the pointer continuously across the dome (see
> B.7) and pass it with `--csv`.

### B.5 When you need more — sequence models (optional)

If you want to **classify** what shape was drawn (circle vs square vs letter),
or handle fast/ambiguous motion, move up to a sequence model. Options, easiest
first:

- **DTW + 1-NN (Dynamic Time Warping):** compare a new trajectory against a few
  labelled templates. No training, works with a handful of examples. Best
  starting point for gesture/shape classification.
- **1-D CNN / Temporal Conv Net (TCN):** input = the (T × 19) activation movie
  or (T × 2) trajectory; output = class or denoised path. Needs ~hundreds of
  labelled strokes.
- **LSTM / GRU seq2seq:** map the frame sequence directly to a smoothed
  trajectory or to pen-up/pen-down + coordinates (the "handwriting" formulation).
  Most powerful, most data-hungry. Only worth it once the Kalman + spline
  pipeline is clearly the bottleneck.
- **Rasterize + image CNN:** accumulate the reconstructed stroke into a small
  image and classify with a tiny CNN — reuses standard vision tooling.

Rule of thumb: **Kalman + spline for *reconstruction*, DTW for *recognition*,
deep nets only when you have the labelled data to justify them.** `torch` is
listed (commented) in `requirements.txt` for when you get there.

### B.6 Estimating shape of the *contact* vs shape of the *motion*

Two different "shapes" — don't conflate them:

- **Contact footprint shape** (static, per frame): fit a 2-D Gaussian /
  covariance ellipse to the activation field → major/minor axis + orientation.
  This tells you the tip/contact geometry. (Eigen-decompose the weighted
  covariance of `POINTS_MM`; a few lines on top of `snm_common`.)
- **Motion/path shape** (dynamic, over time): the descriptors in B.3 and the
  reconstructed curve in B.4.

### B.7 Data-collection protocol for the dynamic case

Your current logger captures discrete presses. For motion/drawing you need
**continuous-contact recordings with ground truth**:

1. Program the UR5 to trace **known geometric paths** at the dome surface —
   lines at several angles, circles of known radius, polygons — keeping the
   pointer in contact and logging frames at a fixed rate (the dt you pass to the
   Kalman filter).
2. Log the TCP `(x, y)` each frame: that *is* the ground-truth trajectory, so
   you can measure reconstruction error (RMSE between reconstructed and true
   path, or Fréchet/DTW distance for shape).
3. Vary **speed** — Kalman tuning (`q`, `r`) and any learned model must hold
   across the speeds you care about.
4. Repeat each path on several sessions for honest held-out evaluation.

This turns "reconstruct a drawing" into a measurable, paper-ready experiment:
trace known shapes, reconstruct, report path RMSE and shape distance.

---

## 3. Suggested order of work

1. Run `snm_common.py` and `train_static.py` — confirm the baseline and get your
   first cross-validated position RMSE. (Part A done.)
2. `train_static.py --save`, wire `infer_static` into the live HUD.
3. Record one continuous circle + one line with the UR5 (B.7).
4. Run `reconstruct_drawing.py --csv ...` on them; compare to TCP ground truth.
5. Tune the Kalman `q`/`r`; add spline smoothing strength as needed.
6. Only if recognition is required: add DTW template matching, then consider a
   TCN/LSTM.

## 4. Pitfalls checklist

- Random row splits → inflated accuracy. **Use GroupKFold on `session`.**
- Forgetting the `GAMMA=0.5` warp when relating activation to force.
- Treating discrete-press data as if it were a continuous swipe.
- Reporting only the learned model — **always show the centroid baseline too**.
- Over-smoothing the spline so real corners vanish; under-smoothing so jitter
  survives. Tune on known shapes.
- Edge contacts: centroid bias is worst there; that's where ML earns its keep.

## 5. Files in this folder

| File | Role |
|---|---|
| `snm_common.py` | geometry + features (centroid, spread, area, diameter, design matrix) |
| `train_static.py` | Part A: train/compare position models, session CV, save best |
| `infer_static.py` | Part A: predict position + area features for one/live frame |
| `dynamic_tracking.py` | Part B: Kalman tracking, speed, heading, path descriptors |
| `reconstruct_drawing.py` | Part B: gate→localise→smooth→spline→render a drawing PNG |
| `requirements.txt` | dependencies |

## 6. References / where to read more

- Center of pressure & tactile localisation: any robotics tactile-sensing
  review (e.g. surveys on capacitive/​resistive taxels).
- Kalman filtering: Welch & Bishop, *An Introduction to the Kalman Filter*.
- Gaussian Processes: Rasmussen & Williams, *GPML* (ch. 2, regression).
- Random Forests: Breiman (2001).
- Sequence modelling for handwriting/trajectories: Graves (2013),
  *Generating Sequences with RNNs*; for TCNs, Bai et al. (2018).
- Dynamic Time Warping for gesture recognition: Sakoe & Chiba (1978).
```
