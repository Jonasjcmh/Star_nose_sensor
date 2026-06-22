"""
dynamic_tracking.py  —  DYNAMIC pipeline: tracking, direction, speed
================================================================================
Goal
----
Turn a *time series* of 19-cell frames into a smooth pointer TRAJECTORY and
derive motion descriptors:
  * filtered position over time            (constant-velocity Kalman filter)
  * instantaneous velocity & SPEED         (mm/s)
  * DIRECTIONALITY of the displacement     (heading angle; dominant axis via PCA)
  * a coarse SHAPE descriptor of the path  (straightness, total turning)

Why a Kalman filter?
--------------------
The per-frame weighted centroid is noisy and jumps between discrete cells.
A constant-velocity Kalman filter fuses successive centroids with a motion
model, giving a smooth position+velocity estimate and naturally yielding the
direction of travel. It is the standard, lightweight tool for this and needs
no training.

Inputs
------
A CSV of consecutive frames with `cell_1..cell_19` columns (e.g. a slice of
frames.csv, or your own swipe log). Optionally a `timestamp` column; otherwise
a fixed dt is assumed.

Run a demo on one session from frames.csv:
    python "ML methods/dynamic_tracking.py"
"""

from __future__ import annotations
import argparse
import numpy as np
import snm_common as snm


# ---------------------------------------------------------------------------
# Constant-velocity Kalman filter (state = [x, y, vx, vy])
# ---------------------------------------------------------------------------
class CVKalman:
    def __init__(self, dt=0.05, q=5.0, r=4.0):
        self.dt = dt
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]], float)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], float)
        self.Q = q * np.array([[dt**3/3, 0, dt**2/2, 0],
                               [0, dt**3/3, 0, dt**2/2],
                               [dt**2/2, 0, dt, 0],
                               [0, dt**2/2, 0, dt]], float)
        self.R = r * np.eye(2)
        self.x = None
        self.P = np.eye(4) * 100.0

    def step(self, z, dt=None):
        if dt is not None:
            self.F[0, 2] = self.F[1, 3] = dt
        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            return self.x.copy()
        # predict
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # update
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return self.x.copy()


def track(frames: np.ndarray, dt: float = 0.05):
    """frames: (T, 19) -> dict with position, velocity, speed, heading."""
    cen = snm.weighted_centroid(frames)            # (T, 2) raw measurements
    kf = CVKalman(dt=dt)
    states = np.array([kf.step(z, dt) for z in cen])
    pos = states[:, :2]
    vel = states[:, 2:]
    speed = np.linalg.norm(vel, axis=1)
    heading = np.arctan2(vel[:, 1], vel[:, 0])     # radians
    return {"raw_centroid": cen, "pos": pos, "vel": vel,
            "speed": speed, "heading_rad": heading}


def path_descriptors(pos: np.ndarray) -> dict:
    """Global shape/direction summary of a trajectory."""
    d = np.diff(pos, axis=0)
    seg = np.linalg.norm(d, axis=1)
    path_len = float(seg.sum())
    net = float(np.linalg.norm(pos[-1] - pos[0]))
    straightness = net / path_len if path_len > 0 else 0.0
    # dominant direction via PCA of the points
    c = pos - pos.mean(0)
    if len(c) > 1:
        _, _, vt = np.linalg.svd(c, full_matrices=False)
        principal = vt[0]
        principal_angle = float(np.degrees(np.arctan2(principal[1], principal[0])))
    else:
        principal_angle = 0.0
    # total absolute turning (how curvy)
    ang = np.arctan2(d[:, 1], d[:, 0])
    turning = float(np.sum(np.abs(np.diff(np.unwrap(ang))))) if len(ang) > 1 else 0.0
    return {"path_length_mm": path_len, "net_displacement_mm": net,
            "straightness": straightness, "principal_dir_deg": principal_angle,
            "total_turning_rad": turning}


def demo():
    df = snm.load_frames()
    sess = df["session"].iloc[0]
    sub = df[df["session"] == sess]
    frames = snm.frame_matrix(sub)
    # treat consecutive rows as a pseudo-trajectory for demonstration
    out = track(frames, dt=0.05)
    desc = path_descriptors(out["pos"])
    print(f"Session: {sess}  ({len(frames)} frames)")
    print(f"  mean speed     : {out['speed'].mean():6.2f} mm/s")
    print(f"  mean heading   : {np.degrees(out['heading_rad']).mean():6.1f} deg")
    print("  path descriptors:")
    for k, v in desc.items():
        print(f"     {k:20s}: {v:8.3f}")
    print("\n(For a real swipe, feed a CSV recorded while the pointer moves "
          "continuously across the dome.)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=str, default=None,
                    help="CSV with cell_1..cell_19 columns (consecutive frames)")
    ap.add_argument("--dt", type=float, default=0.05)
    args = ap.parse_args()
    if args.csv:
        import pandas as pd
        df = pd.read_csv(args.csv)
        frames = snm.frame_matrix(df)
        out = track(frames, dt=args.dt)
        for k, v in path_descriptors(out["pos"]).items():
            print(f"{k:20s}: {v:8.3f}")
    else:
        demo()
