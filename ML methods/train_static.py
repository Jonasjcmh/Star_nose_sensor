"""
train_static.py  —  STATIC pipeline: position, contact area & footprint
================================================================================
Goal
----
Given a single 19-cell capacitive frame (a static press), estimate:
  * (x, y) contact POSITION in mm,
  * contact AREA / covering footprint (mm^2) and equivalent diameter,
  * (optionally) press DEPTH from force, if columns are present.

Ground truth comes from the UR5: the robot commands each point to a known
(pos_x_mm, pos_y_mm) and depth, which are logged alongside the sensor frame.
That is what makes this a *supervised* learning problem.

Models compared
---------------
  1. Analytic weighted centroid           (no training, physics baseline)
  2. k-Nearest-Neighbours regressor       (simple, interpolating)
  3. Random Forest regressor              (your existing approach; robust)
  4. Gaussian Process regressor           (gives calibrated UNCERTAINTY)   [optional]

We report leave-one-session-out style cross-validation RMSE so the numbers
reflect generalisation to a new pressing session, not memorisation.

Run
---
    python "ML methods/train_static.py"                 # uses ML_model/datasets/frames.csv
    python "ML methods/train_static.py" --use-events    # train on aggregated events.csv
    python "ML methods/train_static.py" --save          # save best model to saved_models/
"""

from __future__ import annotations
import argparse
import os
import numpy as np

import snm_common as snm

try:
    from sklearn.ensemble import RandomForestRegressor
    from sklearn.neighbors import KNeighborsRegressor
    from sklearn.model_selection import GroupKFold
    from sklearn.metrics import mean_squared_error
    import joblib
    HAVE_SK = True
except Exception:  # pragma: no cover
    HAVE_SK = False


def rmse_xy(true_xy: np.ndarray, pred_xy: np.ndarray) -> float:
    """Euclidean RMSE in mm: sqrt(mean(||pred-true||^2))."""
    err = np.linalg.norm(pred_xy - true_xy, axis=1)
    return float(np.sqrt(np.mean(err ** 2)))


def load_xy(use_events: bool):
    df = snm.load_events() if use_events else snm.load_frames()
    F = snm.frame_matrix(df)
    y = df[["pos_x_mm", "pos_y_mm"]].to_numpy(dtype=float)
    groups = df["session"].to_numpy()           # one group per pressing session
    return df, F, y, groups


def evaluate(use_events: bool = False, save: bool = False):
    df, F, y, groups = load_xy(use_events)
    X = snm.design_matrix(F, include_raw=True)
    print(f"Dataset: {len(df)} rows | {len(np.unique(groups))} sessions | "
          f"X={X.shape}  ({'events' if use_events else 'frames'})")

    # --- 0. Analytic baseline (no training) -------------------------------
    base_pred = snm.weighted_centroid(F)
    print(f"\n[baseline] weighted-centroid  RMSE = {rmse_xy(y, base_pred):6.2f} mm")

    if not HAVE_SK:
        print("\n(scikit-learn not installed -> skipping learned models. "
              "pip install scikit-learn joblib)")
        return

    n_groups = len(np.unique(groups))
    cv = GroupKFold(n_splits=min(5, max(2, n_groups)))

    models = {
        "kNN(k=5)": KNeighborsRegressor(n_neighbors=5, weights="distance"),
        "RandomForest": RandomForestRegressor(
            n_estimators=200, max_depth=None, n_jobs=-1, random_state=0),
    }

    # Gaussian Process is great for uncertainty but O(n^2); only on small sets.
    if len(df) <= 4000:
        try:
            from sklearn.gaussian_process import GaussianProcessRegressor
            from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel
            from sklearn.multioutput import MultiOutputRegressor
            kern = ConstantKernel() * RBF(length_scale=5.0) + WhiteKernel(1e-2)
            models["GaussianProcess"] = MultiOutputRegressor(
                GaussianProcessRegressor(kernel=kern, normalize_y=True, alpha=1e-6))
        except Exception:
            pass

    results = {}
    for name, model in models.items():
        fold_rmse = []
        for tr, te in cv.split(X, y, groups):
            model.fit(X[tr], y[tr])
            fold_rmse.append(rmse_xy(y[te], model.predict(X[te])))
        results[name] = (np.mean(fold_rmse), np.std(fold_rmse))
        print(f"[cv] {name:16s} RMSE = {np.mean(fold_rmse):6.2f} "
              f"+/- {np.std(fold_rmse):4.2f} mm")

    # --- contact area / diameter quick sanity vs logged estimate ----------
    if "diameter_est_mm" in df.columns:
        d_pred = snm.diameter_est_mm(F)
        d_true = df["diameter_est_mm"].to_numpy(dtype=float)
        mae = np.mean(np.abs(d_pred - d_true))
        print(f"\n[area] diameter estimate vs logged: MAE = {mae:5.2f} mm "
              f"(both are estimates; checks geometry consistency)")

    # --- save the best learned position model -----------------------------
    if save and results:
        best = min(results, key=lambda k: results[k][0])
        out_dir = os.path.join(os.path.dirname(snm.default_dataset_dir()),
                               "saved_models")
        os.makedirs(out_dir, exist_ok=True)
        final = models[best].fit(X, y)                 # refit on all data
        path = os.path.join(out_dir, "position_regressor_static.pkl")
        joblib.dump({"model": final,
                     "columns": snm.design_matrix_columns(True),
                     "include_raw": True}, path)
        print(f"\nSaved best model ({best}) -> {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--use-events", action="store_true",
                    help="train on aggregated events.csv instead of frames.csv")
    ap.add_argument("--save", action="store_true", help="save best model")
    args = ap.parse_args()
    evaluate(args.use_events, args.save)
