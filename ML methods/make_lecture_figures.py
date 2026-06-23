"""
make_lecture_figures.py  —  Generate all illustrations for LECTURE_ML.md
================================================================================
Produces a mix of:
  * REAL-DATA figures from ML_model/datasets (sensor layout, a real activation
    frame, cross-validated model comparison), and
  * SCHEMATIC diagrams (pipelines, the weighted-centroid idea, the Kalman loop,
    path descriptors), and
  * SIMULATED-but-labelled demonstrations of the dynamic pipeline (tracking and
    drawing reconstruction over a known ground-truth path), because the current
    dataset is discrete presses, not continuous swipes.

All outputs go to:  ML methods/figures/figNN_*.png

Run:
    python "ML methods/make_lecture_figures.py"
"""

from __future__ import annotations
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Circle

import snm_common as snm

HERE = os.path.dirname(os.path.abspath(__file__))
FIGDIR = os.path.join(HERE, "figures")
os.makedirs(FIGDIR, exist_ok=True)

# ---- shared style ----------------------------------------------------------
GREEN = "#2aa878"
BLUE = "#3b6ea5"
AMBER = "#e0a020"
RED = "#cc5050"
GREY = "#888888"
LGREY = "#cfcfcf"
INK = "#222222"
plt.rcParams.update({"font.size": 11, "axes.titlesize": 13,
                     "figure.facecolor": "white", "savefig.facecolor": "white"})


def save(fig, name):
    p = os.path.join(FIGDIR, name)
    fig.tight_layout()
    fig.savefig(p, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print("saved", os.path.relpath(p, HERE))


# ---------------------------------------------------------------------------
# Forward sensor model (used for synthetic dynamic demos & the heatmap)
# ---------------------------------------------------------------------------
def forward_activation(contact_xy, sigma=7.0, gamma=0.5, noise=0.0, rng=None):
    """Map a contact point (or array of points) to a 19-cell activation vector.

    Each cell responds as a Gaussian of distance to the contact, then the same
    gamma compression the firmware applies. This is the generative counterpart
    of the inverse problem the ML solves.
    """
    contact_xy = np.atleast_2d(contact_xy)
    out = []
    for c in contact_xy:
        d2 = ((snm.POINTS_MM - c) ** 2).sum(axis=1)
        a = np.exp(-d2 / (2 * sigma ** 2))
        a = np.clip(a, 0, 1) ** gamma
        if noise and rng is not None:
            a = np.clip(a + rng.normal(0, noise, a.shape), 0, 1)
        out.append(a)
    return np.array(out)


def hex_circles(ax, color_edge=LGREY):
    for i, (x, y) in enumerate(snm.POINTS_MM):
        ax.add_patch(Circle((x, y), 3.2, fill=False, ec=color_edge, lw=1.2, zorder=2))


# ===========================================================================
# FIG 1 — sensor layout
# ===========================================================================
def fig_layout():
    fig, ax = plt.subplots(figsize=(5.6, 6.4))
    hex_circles(ax)
    ax.scatter(snm.POINTS_MM[:, 0], snm.POINTS_MM[:, 1], s=10, color=GREY, zorder=3)
    for i, (x, y) in enumerate(snm.POINTS_MM):
        ax.text(x, y, str(i + 1), ha="center", va="center", fontsize=9, color=INK)
    ax.set_title("Fig. 1 — The 19-cell star-nose layout (mm)")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.set_aspect("equal"); ax.grid(alpha=0.2)
    ax.set_xlim(-20, 20); ax.set_ylim(-20, 20)
    ax.text(0, -19, "nearest-neighbour spacing ~ 8 mm  =  native 'pixel' size",
            ha="center", fontsize=9, color=GREY)
    save(fig, "fig01_sensor_layout.png")


# ===========================================================================
# FIG 2 — forward model: a contact -> activation field (interpolated)
# ===========================================================================
def fig_forward():
    from scipy.interpolate import griddata
    contact = np.array([4.0, 5.0])
    act = forward_activation(contact, sigma=7.0)[0]
    gx, gy = np.meshgrid(np.linspace(-18, 18, 200), np.linspace(-18, 18, 200))
    gi = griddata(snm.POINTS_MM, act, (gx, gy), method="cubic", fill_value=0)

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.2))
    for ax in axes:
        ax.set_aspect("equal"); ax.set_xlim(-18, 18); ax.set_ylim(-18, 18)
        ax.set_xlabel("x (mm)")
    axes[0].set_ylabel("y (mm)")

    cf = axes[0].contourf(gx, gy, gi, levels=20, cmap="viridis")
    hex_circles(axes[0], color_edge="white")
    axes[0].plot(*contact, "*", color="white", ms=18, mec="k", label="true contact")
    axes[0].set_title("Fig. 2a — Contact → smooth activation field")
    axes[0].legend(loc="upper left", fontsize=9)
    fig.colorbar(cf, ax=axes[0], shrink=0.85, label="activation")

    sc = axes[1].scatter(snm.POINTS_MM[:, 0], snm.POINTS_MM[:, 1],
                         s=200 + 900 * act, c=act, cmap="viridis", ec="k", zorder=3)
    axes[1].plot(*contact, "*", color="white", ms=18, mec="k")
    axes[1].set_title("Fig. 2b — What the 19 cells actually report")
    fig.colorbar(sc, ax=axes[1], shrink=0.85, label="cell activation")
    save(fig, "fig02_forward_model.png")


# ===========================================================================
# FIG 3 — weighted-centroid concept (schematic over real geometry)
# ===========================================================================
def fig_centroid():
    contact = np.array([6.0, 4.0])
    act = forward_activation(contact, sigma=7.0)[0]
    c = snm.weighted_centroid(act)
    fig, ax = plt.subplots(figsize=(6.2, 6.4))
    hex_circles(ax)
    for (x, y), a in zip(snm.POINTS_MM, act):
        ax.add_patch(Circle((x, y), 3.2 * (0.3 + 0.7 * a), color=GREEN, alpha=0.55, zorder=2))
        # weight "pull" arrows toward centroid scaled by activation
    ax.scatter(snm.POINTS_MM[:, 0], snm.POINTS_MM[:, 1], s=6, color=GREY, zorder=3)
    ax.plot(*c, "o", color=BLUE, ms=12, label="weighted centroid  ĉ", zorder=5)
    ax.plot(*contact, "*", color=RED, ms=16, mec="k", label="true contact", zorder=5)
    ax.set_title("Fig. 3 — Weighted centroid\nĉ = Σ wᵢ·pᵢ / Σ wᵢ")
    ax.set_aspect("equal"); ax.grid(alpha=0.2)
    ax.set_xlim(-20, 20); ax.set_ylim(-20, 20)
    ax.legend(loc="upper left", fontsize=9)
    save(fig, "fig03_centroid.png")


# ===========================================================================
# FIG 4 — static pipeline (block schematic)
# ===========================================================================
def _box(ax, xy, w, h, text, color):
    b = FancyBboxPatch((xy[0] - w / 2, xy[1] - h / 2), w, h,
                       boxstyle="round,pad=0.02,rounding_size=0.08",
                       fc=color, ec=INK, lw=1.3, zorder=2)
    ax.add_patch(b)
    ax.text(xy[0], xy[1], text, ha="center", va="center", fontsize=10, zorder=3)


def _arrow(ax, p0, p1):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=16,
                                 lw=1.6, color=INK, zorder=1))


def fig_static_pipeline():
    fig, ax = plt.subplots(figsize=(12, 3.3))
    ax.set_xlim(0, 12); ax.set_ylim(0, 3); ax.axis("off")
    ys = 1.5
    _box(ax, (1.3, ys), 2.2, 1.4, "19 cell\nactivations\n(one frame)", "#e8f0fb")
    _box(ax, (4.0, ys), 2.2, 1.4, "Features\nraw 19 +\ncentroid, area…", "#eaf6ef")
    _box(ax, (6.8, ys), 2.2, 1.4, "Model\ncentroid / RF /\nGP / MLP", "#fbf0e0")
    _box(ax, (9.6, ys), 2.4, 1.4, "Outputs\nposition (x,y)\narea, diameter", "#fbeaea")
    for a, b in [(2.4, 2.9), (5.1, 5.7), (7.9, 8.4)]:
        _arrow(ax, (a, ys), (b, ys))
    ax.text(6.0, 2.8, "Fig. 4 — Static pipeline", ha="center", fontsize=13)
    ax.text(6.0, 0.25, "Training labels (x,y,depth) come from the UR5 commanded pose",
            ha="center", fontsize=9, color=GREY)
    save(fig, "fig04_static_pipeline.png")


# ===========================================================================
# FIG 5 — real cross-validated model comparison
# ===========================================================================
def fig_model_comparison():
    from sklearn.ensemble import RandomForestRegressor
    from sklearn.neighbors import KNeighborsRegressor
    from sklearn.model_selection import GroupKFold
    df = snm.load_events()
    F = snm.frame_matrix(df)
    y = df[["pos_x_mm", "pos_y_mm"]].to_numpy(float)
    groups = df["session"].to_numpy()
    X = snm.design_matrix(F, include_raw=True)

    def rmse(a, b):
        return float(np.sqrt(np.mean(np.linalg.norm(a - b, axis=1) ** 2)))

    res = {"centroid\n(baseline)": rmse(y, snm.weighted_centroid(F))}
    cv = GroupKFold(n_splits=5)
    for name, m in [("kNN", KNeighborsRegressor(5, weights="distance")),
                    ("Random\nForest", RandomForestRegressor(200, n_jobs=-1, random_state=0))]:
        errs = []
        for tr, te in cv.split(X, y, groups):
            m.fit(X[tr], y[tr]); errs.append(rmse(y[te], m.predict(X[te])))
        res[name] = float(np.mean(errs))

    fig, ax = plt.subplots(figsize=(7, 4.6))
    names = list(res); vals = [res[k] for k in names]
    colors = [GREY, AMBER, GREEN]
    bars = ax.bar(names, vals, color=colors, ec=INK)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.2, f"{v:.1f} mm",
                ha="center", fontsize=10)
    ax.set_ylabel("position RMSE (mm)  ↓ better")
    ax.set_title("Fig. 5 — Held-out (session CV) position error\non your events.csv")
    ax.grid(axis="y", alpha=0.25)
    save(fig, "fig05_model_comparison.png")


# ===========================================================================
# FIG 6 — contact area / diameter concept
# ===========================================================================
def fig_area():
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.0))
    for sz, ax, ttl in [(4.5, axes[0], "small tip"), (9.0, axes[1], "large tip")]:
        act = forward_activation(np.array([0, 0]), sigma=sz)[0]
        A = snm.contact_area_mm2(act)[0]; d = snm.diameter_est_mm(act)[0]
        hex_circles(ax)
        for (x, y), a in zip(snm.POINTS_MM, act):
            ax.add_patch(Circle((x, y), 3.2, color=AMBER, alpha=float(a), zorder=2))
        ax.add_patch(Circle((0, 0), d / 2, fill=False, ec=RED, lw=2, ls="--", zorder=4))
        ax.set_aspect("equal"); ax.set_xlim(-20, 20); ax.set_ylim(-20, 20)
        ax.set_title(f"{ttl}\nArea ≈ {A:.0f} mm²,  d ≈ {d:.1f} mm")
        ax.set_xlabel("x (mm)")
    axes[0].set_ylabel("y (mm)")
    fig.suptitle("Fig. 6 — 'Covering area' from a static press "
                 "(soft hex-coverage → equivalent disc)", y=1.02)
    save(fig, "fig06_contact_area.png")


# ===========================================================================
# FIG 7 — Kalman loop schematic
# ===========================================================================
def fig_kalman_loop():
    fig, ax = plt.subplots(figsize=(9, 3.6))
    ax.set_xlim(0, 9); ax.set_ylim(0, 3); ax.axis("off")
    _box(ax, (1.4, 1.5), 2.2, 1.3, "PREDICT\nx ← F·x\nP ← F P Fᵀ + Q", "#e8f0fb")
    _box(ax, (4.5, 1.5), 2.2, 1.3, "MEASURE\nz = centroid\nof new frame", "#eaf6ef")
    _box(ax, (7.5, 1.5), 2.4, 1.3, "UPDATE\nx ← x + K(z−Hx)\nsmooth pos+vel", "#fbf0e0")
    _arrow(ax, (2.5, 1.5), (3.4, 1.5))
    _arrow(ax, (5.6, 1.5), (6.3, 1.5))
    ax.add_patch(FancyArrowPatch((7.5, 0.8), (1.4, 0.8), connectionstyle="arc3,rad=0.25",
                 arrowstyle="-|>", mutation_scale=16, lw=1.5, color=GREY))
    ax.text(4.5, 0.25, "repeat every frame", ha="center", color=GREY, fontsize=9)
    ax.text(4.5, 2.8, "Fig. 7 — Constant-velocity Kalman loop", ha="center", fontsize=13)
    save(fig, "fig07_kalman_loop.png")


# ===========================================================================
# Simulated continuous stroke (shared by figs 8 & 10)
# ===========================================================================
def simulate_stroke(kind="circle", n=120, seed=0):
    rng = np.random.default_rng(seed)
    t = np.linspace(0, 1, n)
    if kind == "circle":
        path = np.column_stack([9 * np.cos(2 * np.pi * t), 9 * np.sin(2 * np.pi * t)])
    else:  # an "S"-like letter
        path = np.column_stack([8 * np.sin(2 * np.pi * t),
                                12 * (t - 0.5)])
    frames = forward_activation(path, sigma=6.0, noise=0.04, rng=rng)
    return path, frames


def fig_tracking():
    from dynamic_tracking import track
    path, frames = simulate_stroke("circle", seed=1)
    out = track(frames, dt=0.05)
    fig, ax = plt.subplots(figsize=(6.4, 6.6))
    hex_circles(ax)
    ax.plot(path[:, 0], path[:, 1], "-", color=LGREY, lw=4, label="ground truth")
    ax.plot(out["raw_centroid"][:, 0], out["raw_centroid"][:, 1], ".",
            color=AMBER, ms=5, label="raw centroids (noisy)")
    ax.plot(out["pos"][:, 0], out["pos"][:, 1], "-", color=GREEN, lw=2.2,
            label="Kalman-smoothed")
    ax.set_aspect("equal"); ax.set_xlim(-18, 18); ax.set_ylim(-18, 18)
    ax.set_title("Fig. 8 — Tracking a moving pointer\n(simulated known circle)")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.2)
    save(fig, "fig08_tracking.png")


# ===========================================================================
# FIG 9 — dynamic pipeline schematic
# ===========================================================================
def fig_dynamic_pipeline():
    fig, ax = plt.subplots(figsize=(13, 3.3))
    ax.set_xlim(0, 13); ax.set_ylim(0, 3); ax.axis("off")
    ys = 1.5
    boxes = [("frames\nover time\n(T×19)", "#e8f0fb"),
             ("localise\neach frame\n(centroid)", "#eaf6ef"),
             ("gate\npen-down\nframes", "#f0eafb"),
             ("Kalman\nsmooth\n+velocity", "#fbf0e0"),
             ("spline\nfit", "#eafbf3"),
             ("drawing +\ndirection,\nshape", "#fbeaea")]
    xs = np.linspace(1.4, 11.6, len(boxes))
    for x, (t, c) in zip(xs, boxes):
        _box(ax, (x, ys), 1.9, 1.4, t, c)
    for a, b in zip(xs[:-1], xs[1:]):
        _arrow(ax, (a + 0.95, ys), (b - 0.95, ys))
    ax.text(6.5, 2.85, "Fig. 9 — Dynamic pipeline", ha="center", fontsize=13)
    save(fig, "fig09_dynamic_pipeline.png")


# ===========================================================================
# FIG 10 — reconstruction vs ground truth (simulated letter)
# ===========================================================================
def fig_reconstruction():
    from reconstruct_drawing import reconstruct
    path, frames = simulate_stroke("S", n=140, seed=3)
    rec = reconstruct(frames, dt=0.05, min_total=0.0)
    fig, ax = plt.subplots(figsize=(6.0, 6.8))
    hex_circles(ax)
    ax.plot(path[:, 0], path[:, 1], "-", color=LGREY, lw=5, label="ground truth")
    ax.plot(rec["curve"][:, 0], rec["curve"][:, 1], "-", color=GREEN, lw=2.4,
            label="reconstructed")
    if len(rec["curve"]):
        ax.plot(*rec["curve"][0], "go", ms=9); ax.plot(*rec["curve"][-1], "rs", ms=9)
    ax.set_aspect("equal"); ax.set_xlim(-16, 16); ax.set_ylim(-16, 16)
    ax.set_title("Fig. 10 — Reconstructed drawing\n(simulated 'S' stroke)")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.2)
    save(fig, "fig10_reconstruction.png")


# ===========================================================================
# FIG 11 — path descriptors schematic
# ===========================================================================
def fig_descriptors():
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.4))
    t = np.linspace(0, 1, 60)
    # straight
    p1 = np.column_stack([14 * t - 7, 2 * np.ones_like(t)])
    # curved
    p2 = np.column_stack([12 * np.sin(np.pi * t) - 0, 10 * t - 5])
    # scribble
    p3 = np.column_stack([8 * np.sin(6 * np.pi * t), 8 * np.cos(4 * np.pi * t)])
    from dynamic_tracking import path_descriptors
    for ax, p, ttl in [(axes[0], p1, "straight swipe"),
                       (axes[1], p2, "gentle curve"),
                       (axes[2], p3, "scribble")]:
        d = path_descriptors(p)
        ax.plot(p[:, 0], p[:, 1], "-", color=GREEN, lw=2.4)
        ax.plot(*p[0], "go"); ax.plot(*p[-1], "rs")
        # principal axis
        c = p.mean(0); ang = np.radians(d["principal_dir_deg"])
        v = np.array([np.cos(ang), np.sin(ang)]) * 8
        ax.plot([c[0] - v[0], c[0] + v[0]], [c[1] - v[1], c[1] + v[1]],
                "--", color=BLUE, lw=1.5)
        ax.set_aspect("equal"); ax.set_xlim(-12, 12); ax.set_ylim(-12, 12)
        ax.set_title(f"{ttl}\nstraightness={d['straightness']:.2f}, "
                     f"turning={d['total_turning_rad']:.1f}")
        ax.grid(alpha=0.2)
    fig.suptitle("Fig. 11 — Global path descriptors (blue dashed = PCA principal direction)",
                 y=1.02)
    save(fig, "fig11_descriptors.png")


if __name__ == "__main__":
    fig_layout()
    fig_forward()
    fig_centroid()
    fig_static_pipeline()
    fig_model_comparison()
    fig_area()
    fig_kalman_loop()
    fig_tracking()
    fig_dynamic_pipeline()
    fig_reconstruction()
    fig_descriptors()
    print("\nAll figures written to", os.path.relpath(FIGDIR, HERE))
