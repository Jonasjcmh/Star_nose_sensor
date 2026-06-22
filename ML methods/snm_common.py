"""
snm_common.py  —  Shared geometry + feature utilities for the Star-Nose sensor
================================================================================
This module is the single source of truth for:
  * the physical (x, y) layout of the 19 capacitive cells (mm),
  * loading the project datasets (frames.csv / events.csv),
  * turning a 19-vector of cell activations into interpretable spatial features
    (centroid, spread, contact area, active-cell count, ...).

It is imported by every script in the "ML methods" folder so the geometry and
feature definitions never drift between the static and dynamic pipelines.

Cell ordering convention
-------------------------
Throughout this folder a "frame" is the 19-vector ordered exactly as the
`cell_1 ... cell_19` columns in the CSV logs, which is the same order as
`POINTS_MM` below. Index i (0..18) therefore has physical coordinate POINTS_MM[i].
"""

from __future__ import annotations
import os
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# 1. Sensor geometry  (taken from Integration_2/visualizer_2d.py -> POINTS_MM)
# ---------------------------------------------------------------------------
# Hexagonal "star-nose" layout. Units: millimetres, sensor-local frame.
POINTS_MM = np.array([
    (-8, +14), (0, +14), (+8, +14),
    (-12, +7), (-4, +7), (+4, +7), (+12, +7),
    (-16,  0), (-8,  0), (0,  0), (+8,  0), (+16, 0),
    (-12, -7), (-4, -7), (+4, -7), (+12, -7),
    (-8, -14), (0, -14), (+8, -14),
], dtype=float)

N_CELLS = len(POINTS_MM)                 # 19
CELL_COLS = [f"cell_{i+1}" for i in range(N_CELLS)]

# Nearest-neighbour spacing of the lattice (mm). Rows are 7 mm apart in y and
# cells are 8 mm apart in x within a row -> nn distance ~ 8 mm. Used to set the
# default "active" threshold area scale.
NN_SPACING_MM = 8.0


# ---------------------------------------------------------------------------
# 2. Dataset loading
# ---------------------------------------------------------------------------
def default_dataset_dir() -> str:
    """ML_model/datasets relative to the repo root (this file lives in 'ML methods')."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    return os.path.join(repo, "ML_model", "datasets")


def load_frames(path: str | None = None) -> pd.DataFrame:
    """Per-frame data: one row per sensor sample during a press."""
    path = path or os.path.join(default_dataset_dir(), "frames.csv")
    return pd.read_csv(path)


def load_events(path: str | None = None) -> pd.DataFrame:
    """Per-event data: one aggregated row per (session, point, visit) press."""
    path = path or os.path.join(default_dataset_dir(), "events.csv")
    return pd.read_csv(path)


def frame_matrix(df: pd.DataFrame) -> np.ndarray:
    """Extract the (N, 19) activation matrix from a frames/events dataframe.

    Works for frames.csv (`cell_i`) and falls back to the `mean_cell_i`
    columns used in events.csv if the plain cell columns are absent.
    """
    if all(c in df.columns for c in CELL_COLS):
        return df[CELL_COLS].to_numpy(dtype=float)
    mean_cols = [f"mean_cell_{i+1}" for i in range(N_CELLS)]
    if all(c in df.columns for c in mean_cols):
        return df[mean_cols].to_numpy(dtype=float)
    raise KeyError("No cell_i or mean_cell_i columns found in dataframe.")


# ---------------------------------------------------------------------------
# 3. Spatial features from a single 19-vector
# ---------------------------------------------------------------------------
def _as_2d(frame: np.ndarray) -> np.ndarray:
    frame = np.asarray(frame, dtype=float)
    if frame.ndim == 1:
        frame = frame[None, :]
    if frame.shape[1] != N_CELLS:
        raise ValueError(f"Expected {N_CELLS} cells, got shape {frame.shape}")
    return frame


def weighted_centroid(frame: np.ndarray) -> np.ndarray:
    """Activation-weighted centre of pressure, (N, 2) in mm.

    This is the classic analytic position estimate: c = sum(w_i * p_i)/sum(w_i).
    Robust, parameter-free, and a strong baseline for the static problem.
    """
    F = _as_2d(frame)
    w = np.clip(F, 0, None)
    tot = w.sum(axis=1, keepdims=True)
    tot[tot == 0] = 1.0
    return (w @ POINTS_MM) / tot


def spread_mm(frame: np.ndarray) -> np.ndarray:
    """Activation-weighted RMS radius about the centroid (N,), mm.

    A simple scalar proxy for how 'wide' the contact footprint is.
    """
    F = _as_2d(frame)
    w = np.clip(F, 0, None)
    c = weighted_centroid(F)
    d2 = ((POINTS_MM[None, :, :] - c[:, None, :]) ** 2).sum(axis=2)  # (N,19)
    tot = w.sum(axis=1)
    tot[tot == 0] = 1.0
    return np.sqrt((w * d2).sum(axis=1) / tot)


def n_active(frame: np.ndarray, thresh: float = 0.05) -> np.ndarray:
    """Number of cells above `thresh` (N,). Matches the firmware 'active' rule."""
    F = _as_2d(frame)
    return (F > thresh).sum(axis=1)


def contact_area_mm2(frame: np.ndarray, thresh: float = 0.05) -> np.ndarray:
    """Estimated covered area (N,), mm^2.

    Each active cell is assigned its Voronoi-ish footprint cell of area
    (NN_SPACING_MM^2 * sqrt(3)/2) for a hex lattice ~ 55 mm^2, scaled by how
    fully the cell is activated (soft coverage). This is the 'covering area'
    estimate for static pushing points.
    """
    F = _as_2d(frame)
    hex_cell_area = (NN_SPACING_MM ** 2) * (np.sqrt(3) / 2.0)
    soft = np.clip(F, 0, 1)
    soft[F <= thresh] = 0.0
    return soft.sum(axis=1) * hex_cell_area


def diameter_est_mm(frame: np.ndarray, thresh: float = 0.05) -> np.ndarray:
    """Equivalent-disc diameter from the contact area (N,), mm. d = 2*sqrt(A/pi)."""
    A = contact_area_mm2(frame, thresh)
    return 2.0 * np.sqrt(A / np.pi)


def feature_table(frame: np.ndarray, thresh: float = 0.05) -> dict:
    """Convenience: all engineered features for a batch of frames as a dict of arrays."""
    c = weighted_centroid(frame)
    return {
        "centroid_x_mm": c[:, 0],
        "centroid_y_mm": c[:, 1],
        "spread_mm": spread_mm(frame),
        "n_active": n_active(frame, thresh),
        "contact_area_mm2": contact_area_mm2(frame, thresh),
        "diameter_est_mm": diameter_est_mm(frame, thresh),
    }


def design_matrix(frame: np.ndarray, include_raw: bool = True) -> np.ndarray:
    """Build a feature matrix for ML models.

    Columns = [19 raw cells (optional)] + [centroid_x, centroid_y, spread,
    n_active, area, diameter]. Returning raw + engineered features lets tree
    models exploit both the full field and the physically-meaningful summaries.
    """
    F = _as_2d(frame)
    feats = feature_table(F)
    eng = np.column_stack([feats[k] for k in
                           ("centroid_x_mm", "centroid_y_mm", "spread_mm",
                            "n_active", "contact_area_mm2", "diameter_est_mm")])
    return np.hstack([F, eng]) if include_raw else eng


def design_matrix_columns(include_raw: bool = True) -> list[str]:
    eng = ["centroid_x_mm", "centroid_y_mm", "spread_mm",
           "n_active", "contact_area_mm2", "diameter_est_mm"]
    return (CELL_COLS + eng) if include_raw else eng


if __name__ == "__main__":
    # Tiny self-test / demo using the project dataset.
    df = load_frames()
    X = frame_matrix(df)
    print(f"Loaded {len(df)} frames, matrix shape {X.shape}")
    feats = feature_table(X[:5])
    for k, v in feats.items():
        print(f"  {k:18s}: {np.round(v, 2)}")
