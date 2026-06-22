"""
reconstruct_drawing.py  —  DYNAMIC pipeline: reconstruct the drawn path
================================================================================
Goal
----
Given a recording of the pointer moving across the dome (a sequence of 19-cell
frames), reconstruct the 2-D DRAWING the pointer traced and save it as a PNG.

Pipeline (theory -> code)
-------------------------
  1. Per-frame sub-cell localisation
       weighted centroid gives (x,y) for each frame. Because activation spreads
       over neighbouring cells, the centroid interpolates *between* cells,
       giving sub-cell (sub-8mm) resolution.
  2. Gating
       drop frames with too little total activation (pointer lifted / no contact)
       so pen-up segments don't create spurious strokes.
  3. Smoothing
       a constant-velocity Kalman filter (see dynamic_tracking.py) removes
       per-frame jitter while preserving real curvature.
  4. Resampling / spline
       fit a smoothing spline (arc-length parameterised) to render a clean,
       continuous curve instead of a dotted scatter.
  5. Render
       overlay the reconstructed stroke on the sensor cell layout.

Run a demo (uses one session of frames.csv as a pseudo-drawing):
    python "ML methods/reconstruct_drawing.py" --out reconstruction.png

Reconstruct your own swipe:
    python "ML methods/reconstruct_drawing.py" --csv my_swipe.csv --out my_draw.png
"""

from __future__ import annotations
import argparse
import numpy as np
import snm_common as snm
from dynamic_tracking import track


def gate_frames(frames: np.ndarray, min_total: float = 0.3):
    """Keep only frames where the pointer is actually in contact (pen-down)."""
    totals = np.clip(frames, 0, None).sum(axis=1)
    mask = totals >= min_total
    return mask


def smooth_spline(pos: np.ndarray, n_out: int = 400, smooth: float | None = None):
    """Arc-length smoothing spline through the trajectory points."""
    try:
        from scipy.interpolate import splprep, splev
    except Exception:
        return pos  # scipy not available -> return raw path
    # Drop consecutive duplicate points: zero-length segments break splprep.
    keep = np.concatenate(([True], np.any(np.diff(pos, axis=0) != 0, axis=1)))
    p = pos[keep]
    if len(p) < 4:
        return pos
    s = smooth if smooth is not None else len(p) * 0.5
    try:
        tck, _ = splprep([p[:, 0], p[:, 1]], s=s)
        u = np.linspace(0, 1, n_out)
        x, y = splev(u, tck)
        return np.column_stack([x, y])
    except Exception:
        return p  # fall back to the (deduped) raw path


def reconstruct(frames: np.ndarray, dt: float = 0.05, min_total: float = 0.3):
    mask = gate_frames(frames, min_total)
    used = frames[mask] if mask.any() else frames
    out = track(used, dt=dt)
    curve = smooth_spline(out["pos"])
    return {"raw": out["raw_centroid"], "smoothed": out["pos"], "curve": curve}


def render(rec: dict, out_path: str, title: str = "Reconstructed drawing"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 7))
    # sensor cells
    ax.scatter(snm.POINTS_MM[:, 0], snm.POINTS_MM[:, 1],
               s=420, facecolors="none", edgecolors="#bbbbbb", linewidths=1)
    for i, (x, y) in enumerate(snm.POINTS_MM):
        ax.text(x, y, str(i + 1), ha="center", va="center",
                fontsize=7, color="#999999")
    # reconstruction
    ax.plot(rec["raw"][:, 0], rec["raw"][:, 1], ".", ms=3,
            color="#ccccff", label="raw centroids", alpha=0.5)
    ax.plot(rec["curve"][:, 0], rec["curve"][:, 1], "-", lw=2.5,
            color="#2aa878", label="reconstructed stroke")
    if len(rec["curve"]):
        ax.plot(*rec["curve"][0], "go", ms=8, label="start")
        ax.plot(*rec["curve"][-1], "rs", ms=8, label="end")
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.set_title(title); ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.2)
    fig.tight_layout(); fig.savefig(out_path, dpi=130)
    print(f"Saved -> {out_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=str, default=None)
    ap.add_argument("--out", type=str, default="reconstruction.png")
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--min-total", type=float, default=0.3)
    args = ap.parse_args()

    if args.csv:
        import pandas as pd
        df = pd.read_csv(args.csv)
        title = f"Reconstructed: {args.csv}"
    else:
        df = snm.load_frames()
        sess = df["session"].iloc[0]
        df = df[df["session"] == sess]
        title = f"Demo reconstruction ({sess})"
        print("(no --csv; using one session of frames.csv as a pseudo-drawing)")

    frames = snm.frame_matrix(df)
    rec = reconstruct(frames, dt=args.dt, min_total=args.min_total)
    render(rec, args.out, title)
