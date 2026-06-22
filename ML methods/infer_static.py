"""
infer_static.py  —  Use a trained static model on a live / single frame
================================================================================
Loads a model saved by train_static.py (--save) and predicts (x, y) position +
reports the analytic contact-area features for a single 19-cell frame.

This is the function you would call from your real-time loop (e.g. inside
visualizer_2d.py or main.py) after reading `sensor.get_values()`.

Run a demo on the first dataset frame:
    python "ML methods/infer_static.py"

Use your own frame:
    python "ML methods/infer_static.py" --frame 0.18,0.25,0.0,...  (19 comma values)
"""

from __future__ import annotations
import argparse
import os
import numpy as np
import snm_common as snm


def load_model(path: str | None = None):
    import joblib
    if path is None:
        path = os.path.join(os.path.dirname(snm.default_dataset_dir()),
                            "saved_models", "position_regressor_static.pkl")
    return joblib.load(path)


def predict_position(frame19, bundle) -> np.ndarray:
    """frame19: length-19 list/array of cell activations -> (x, y) mm."""
    F = np.asarray(frame19, dtype=float)[None, :]
    X = snm.design_matrix(F, include_raw=bundle.get("include_raw", True))
    return bundle["model"].predict(X)[0]


def describe(frame19) -> None:
    F = np.asarray(frame19, dtype=float)[None, :]
    f = snm.feature_table(F)
    print("Analytic (physics) estimates from this frame:")
    print(f"  centroid (x,y)   : ({f['centroid_x_mm'][0]:+6.2f}, "
          f"{f['centroid_y_mm'][0]:+6.2f}) mm")
    print(f"  spread (rms r)   : {f['spread_mm'][0]:6.2f} mm")
    print(f"  active cells     : {int(f['n_active'][0])}")
    print(f"  contact area     : {f['contact_area_mm2'][0]:6.1f} mm^2")
    print(f"  equiv. diameter  : {f['diameter_est_mm'][0]:6.2f} mm")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", type=str, default=None,
                    help="19 comma-separated activation values")
    ap.add_argument("--model", type=str, default=None)
    args = ap.parse_args()

    if args.frame:
        frame = [float(x) for x in args.frame.split(",")]
        assert len(frame) == snm.N_CELLS, f"need {snm.N_CELLS} values"
    else:
        df = snm.load_frames()
        frame = snm.frame_matrix(df)[0]
        print("(no --frame given; using first dataset frame)\n")

    describe(frame)
    try:
        bundle = load_model(args.model)
        xy = predict_position(frame, bundle)
        print(f"\nLearned model position: ({xy[0]:+6.2f}, {xy[1]:+6.2f}) mm")
    except FileNotFoundError:
        print("\n(no trained model found - run train_static.py --save first)")
