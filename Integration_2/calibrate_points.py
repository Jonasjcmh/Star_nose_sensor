"""
calibrate_points.py
Per-point calibration tool for the 19-point KYWO tactile sensor + UR5.

Compares the robot's actual TCP position against the expected grid position,
presses at each point, and checks whether the correct sensor cell activates.
A live hexmap window shows all 19 capacitor values in real time while you
nudge the robot. A deviation map summarises offsets and detection quality.

────────────────────────────────────────────────────────────────────
Usage:
  python3 calibrate_points.py                # full scan + interactive fix
  python3 calibrate_points.py --scan         # scan only, then show map
  python3 calibrate_points.py --point 5      # interactive fix for P5 only
  python3 calibrate_points.py --points 4 5 6 7  # fix multiple points
  python3 calibrate_points.py --map          # show deviation map from saved data
  python3 calibrate_points.py --no-sensor    # skip sensor (robot motion only)

Commands during interactive per-point adjustment:
  x+  x-         nudge right / left  (step mm)
  y+  y-         nudge forward / back
  step 0.5       set nudge step size (mm)
  press          press down and read sensor response
  teach          record current TCP as calibration offset (use with freedrive)
  status         show offset + sensor snapshot in terminal
  map            show deviation map for all points so far
  ok             accept offset for this point → move to next
  skip           skip this point (no change saved)
  save           save all offsets collected so far and quit
  quit           quit without saving anything
────────────────────────────────────────────────────────────────────
Output: calib_points.json (same folder)
  {
    "global":    { "x_mm": ..., "y_mm": ..., "z_mm": ... },
    "per_point": { "1": {"dx_mm": 0.0, "dy_mm": 0.0}, ... },
    "scan_results": {
      "1": { "expected_raw": 24, "actual_raw": 24,
             "expected_val": 0.82, "top_val": 0.82,
             "correct": true, "tcp": [...] }, ...
    }
  }
────────────────────────────────────────────────────────────────────
NOTE: To apply per-point offsets in live sessions, ur5_control.py
must be updated to load calib_points.json.  The current version uses
only the global X/Y/Z offset from calib.json.
"""

import argparse
import glob
import json
import os
import sys
import time
import threading

# ── Optional matplotlib (graceful fallback if headless) ───────────────────────
try:
    import platform, matplotlib
    matplotlib.use("MacOSX" if platform.system() == "Darwin" else "TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import RegularPolygon
    from matplotlib.colors import LinearSegmentedColormap, Normalize
    from matplotlib.cm import ScalarMappable
    import numpy as np
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False

# ── Constants ─────────────────────────────────────────────────────────────────
ROBOT_IP  = "177.22.22.2"
CALIB_DIR = os.path.dirname(os.path.abspath(__file__))

def calib_file(tip=None):
    name = f"calib_{tip}.json" if tip else "calib.json"
    return os.path.join(CALIB_DIR, name)

def calib_pts_file(tip=None):
    name = f"calib_points_{tip}.json" if tip else "calib_points.json"
    return os.path.join(CALIB_DIR, name)

VELOCITY_TRAVEL = 0.05
VELOCITY_PRESS  = 0.01
ACCELERATION    = 0.3
INDENT_MM       = 10.0
SAFE_Z_MM       = 20.0

REFERENCE_POSE = [
    -0.03664,
    -0.49831,
     0.06071,
    2.346, -2.094, -0.00009
]

POINTS = {
     1: ( -8.0, +14.0),   2: (  0.0, +14.0),   3: ( +8.0, +14.0),
     4: (-12.0,  +7.0),   5: ( -4.0,  +7.0),   6: ( +4.0,  +7.0),
     7: (+12.0,  +7.0),   8: (-16.0,   0.0),   9: ( -8.0,   0.0),
    10: (  0.0,   0.0),  11: ( +8.0,   0.0),  12: (+16.0,   0.0),
    13: (-12.0,  -7.0),  14: ( -4.0,  -7.0),  15: ( +4.0,  -7.0),
    16: (+12.0,  -7.0),  17: ( -8.0, -14.0),  18: (  0.0, -14.0),
    19: ( +8.0, -14.0),
}

# UR5 point → index in sensor.get_values() 19-element array
UR5_TO_IDX = {
    1: 16,  2: 12,  3:  7,
    4: 17,  5: 13,  6:  8,  7:  3,
    8: 18,  9: 14, 10:  9, 11:  4, 12:  0,
   13: 15, 14: 10, 15:  5, 16:  1,
   17: 11, 18:  6, 19:  2,
}
RAW_CELLS = [2, 15, 28, 1, 14, 27, 40, 0, 13, 26, 39, 52, 12, 25, 38, 51, 24, 37, 50]
UR5_TO_RAW = {pt: RAW_CELLS[UR5_TO_IDX[pt]] for pt in range(1, 20)}

SCAN_ORDER = list(range(1, 20))

# Colourmap matching animate_session.py
if _HAS_MPL:
    SENSOR_CMAP = LinearSegmentedColormap.from_list(
        "star_nose", ["#2ab5a0", "#33e666", "#ffe619", "#ff7300", "#dc0000"])
    BG   = "#111111"
    EDGE = "#444444"

# ── File I/O ──────────────────────────────────────────────────────────────────
def load_global_calib(tip=None):
    f_path = calib_file(tip)
    if os.path.exists(f_path):
        with open(f_path) as f:
            d = json.load(f)
        gx, gy, gz = d.get("x_mm", 0.0), d.get("y_mm", 0.0), d.get("z_mm", 0.0)
        print(f"[calib] Global offset: X={gx:+.3f} Y={gy:+.3f} Z={gz:+.3f} mm")
        return gx, gy, gz
    print(f"[calib] No {os.path.basename(f_path)} — using zero global offset")
    return 0.0, 0.0, 0.0


def load_point_offsets(tip=None):
    f_path = calib_pts_file(tip)
    if os.path.exists(f_path):
        with open(f_path) as f:
            d = json.load(f)
        offsets = {int(k): (v.get("dx_mm", 0.0), v.get("dy_mm", 0.0))
                   for k, v in d.get("per_point", {}).items()}
        print(f"[calib] Loaded per-point offsets for {len(offsets)} point(s)")
        return offsets, d.get("scan_results", {})
    return {}, {}


def save_results(global_calib, per_point_offsets, scan_results, out_path):
    gx, gy, gz = global_calib

    # Full record for every point = theoretical grid position + global offset
    # + per-point deviation, expressed both as mm and as an absolute robot pose.
    points_full = {}
    for pt in range(1, 20):
        dx, dy = per_point_offsets.get(pt, (0.0, 0.0))
        tx, ty = POINTS[pt]
        pose = build_pose(pt, global_calib, dx, dy, 0.0)   # surface, calibrated
        points_full[str(pt)] = {
            "theoretical_mm": [tx, ty],                       # nominal grid XY
            "deviation_mm":   [round(dx, 4), round(dy, 4)],   # calibration nudge
            "offset_mm":      [round(tx + gx + dx, 4),        # XY offset from ref pose
                               round(ty + gy + dy, 4)],
            "pose":           [round(v, 6) for v in pose],    # absolute UR5 pose
        }

    data = {
        "global": {"x_mm": gx, "y_mm": gy, "z_mm": gz},
        "reference_pose": REFERENCE_POSE,
        "per_point": {                                        # kept for ur5_control
            str(pt): {"dx_mm": round(dx, 4), "dy_mm": round(dy, 4)}
            for pt, (dx, dy) in sorted(per_point_offsets.items())
        },
        "points": points_full,                                # full per-point record
        "scan_results": scan_results,
    }
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n[calib] Saved full {len(points_full)}-point file → {out_path}")


# ── Interactive calibration selection / naming ────────────────────────────────
def discover_global_calibs():
    """Find every global-offset file (calib_<name>.json / calib.json).

    The per-point companions (calib_points*.json) are NOT starting points,
    so they are excluded. Returns a list of dicts sorted with the default first.
    """
    def _read(path):
        try:
            with open(path) as f:
                d = json.load(f)
            return (d.get("x_mm", 0.0), d.get("y_mm", 0.0), d.get("z_mm", 0.0))
        except Exception:
            return None

    profiles = []
    for path in sorted(glob.glob(os.path.join(CALIB_DIR, "calib_*.json"))):
        name = os.path.basename(path)
        if name == "calib_points.json" or name.startswith("calib_points_"):
            continue
        base = name[len("calib_"):-len(".json")]      # calib_short_6mm.json -> short_6mm
        pts = os.path.join(CALIB_DIR, f"calib_points_{base}.json")
        profiles.append({
            "base": base, "path": path, "offset": _read(path),
            "points_file": pts if os.path.exists(pts) else None,
        })

    default = os.path.join(CALIB_DIR, "calib.json")
    if os.path.exists(default):
        dp = os.path.join(CALIB_DIR, "calib_points.json")
        profiles.insert(0, {
            "base": None, "path": default, "offset": _read(default),
            "points_file": dp if os.path.exists(dp) else None,
        })
    return profiles


def choose_starting_calib():
    """Ask which calibration file to use as the global X/Y/Z starting point.

    Returns (global_calib, base_label, points_file_or_None). Per-point
    deviations are then built on top of this global offset.
    """
    profiles = discover_global_calibs()
    print("\n" + "=" * 62)
    print("  CHOOSE STARTING CALIBRATION  (global X/Y/Z base)")
    print("=" * 62)
    if not profiles:
        print("  No calib_*.json files found — using zero global offset.")
        return (0.0, 0.0, 0.0), None, None
    for i, p in enumerate(profiles, 1):
        o = p["offset"]
        o_txt = (f"X={o[0]:+.2f} Y={o[1]:+.2f} Z={o[2]:+.2f} mm"
                 if o else "unreadable")
        pts = "  [+ per-point]" if p["points_file"] else ""
        print(f"  {i:2d}) {os.path.basename(p['path']):<30s} {o_txt}{pts}")
    print("   0) zero offset  (no global calibration)")
    print("=" * 62)
    while True:
        try:
            raw = input(f"Select starting calibration [0-{len(profiles)}] > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[calib] Aborted.")
            sys.exit(1)
        if raw == "0":
            print("[calib] Using zero global offset.")
            return (0.0, 0.0, 0.0), None, None
        if raw.isdigit() and 1 <= int(raw) <= len(profiles):
            p = profiles[int(raw) - 1]
            g = p["offset"] or (0.0, 0.0, 0.0)
            print(f"[calib] Base global: X={g[0]:+.3f} Y={g[1]:+.3f} Z={g[2]:+.3f} mm "
                  f"(from {os.path.basename(p['path'])})")
            return g, p["base"], p["points_file"]
        print("  Invalid choice, try again.")


def load_existing_deviations(points_file):
    """Load the per-point deviations from the chosen calibration file.

    These are the base deviations we build on during this session — always
    loaded (no prompt) so calibration starts from the chosen file's values.
    """
    if not points_file or not os.path.exists(points_file):
        return {}, {}
    try:
        with open(points_file) as f:
            d = json.load(f)
        offsets = {int(k): (v.get("dx_mm", 0.0), v.get("dy_mm", 0.0))
                   for k, v in d.get("per_point", {}).items()}
        print(f"[calib] Loaded {len(offsets)} per-point deviation(s) from "
              f"{os.path.basename(points_file)}")
        return offsets, d.get("scan_results", {})
    except Exception as e:
        print(f"[calib] Could not read {os.path.basename(points_file)}: {e}")
        return {}, {}


def prompt_save(global_calib, per_point_offsets, scan_results, default_name):
    """Ask for the output calibration-points filename, then save to it.

    Returns the base name (no extension) so the caller can reuse it, e.g.
    for a matching deviation-map PNG.
    """
    print("\n" + "=" * 62)
    print("  SAVE CALIBRATION POINTS")
    print("=" * 62)
    while True:
        try:
            name = input(f"  Output file name [{default_name}] > ").strip()
        except (EOFError, KeyboardInterrupt):
            name = ""
        if not name:
            name = default_name
        name = os.path.basename(name)              # ignore any directory part
        if not name.endswith(".json"):
            name += ".json"
        out_path = os.path.join(CALIB_DIR, name)
        if os.path.exists(out_path):
            try:
                ow = input(f"  {name} exists — overwrite? [y/N] > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                ow = "n"
            if ow != "y":
                continue
        save_results(global_calib, per_point_offsets, scan_results, out_path)
        return name[:-len(".json")]

# ── Pose builder ──────────────────────────────────────────────────────────────
def build_pose(pt, global_calib, extra_dx=0.0, extra_dy=0.0, extra_z=0.0):
    gx, gy, gz = global_calib
    dx, dy = POINTS[pt]
    pose = REFERENCE_POSE.copy()
    pose[0] += (dx + gx + extra_dx) / 1000.0
    pose[1] += (dy + gy + extra_dy) / 1000.0
    pose[2] += (gz + extra_z) / 1000.0
    return pose


def home_pose(global_calib):
    return build_pose(10, global_calib, extra_z=SAFE_Z_MM)

# ── Sensor helpers ────────────────────────────────────────────────────────────
def _sensor_bar(val, width=20):
    n = int(val * width)
    return "[" + "#" * n + "-" * (width - n) + f"] {val:.3f}"

# ── Live sensor display thread ────────────────────────────────────────────────
_live_stop   = threading.Event()
_live_thread = None


def _live_worker(sensor_mod, pt, interval=0.35):
    """Prints a single updating line to terminal with current sensor values."""
    exp_idx = UR5_TO_IDX[pt]
    exp_raw = RAW_CELLS[exp_idx]
    while not _live_stop.wait(interval):
        vals = sensor_mod.get_values()
        exp_val = vals[exp_idx]
        bar = _sensor_bar(exp_val)
        others = [(RAW_CELLS[i], v) for i, v in enumerate(vals)
                  if v > 0.05 and i != exp_idx]
        others_str = ("  | " + "  ".join(f"S{r}={v:.2f}" for r, v in others[:4])
                      if others else "")
        print(f"\r  [S{exp_raw:2d}] {bar}{others_str}        ", end="", flush=True)


def start_live_display(sensor_mod, pt):
    global _live_thread, _live_stop
    if sensor_mod is None:
        return
    _live_stop.clear()
    _live_thread = threading.Thread(target=_live_worker,
                                    args=(sensor_mod, pt), daemon=True)
    _live_thread.start()


def stop_live_display():
    global _live_thread
    _live_stop.set()
    if _live_thread:
        _live_thread.join(timeout=1.0)
        _live_thread = None
    print()   # newline after the in-place line

# ── Live hexmap window ────────────────────────────────────────────────────────
_live_fig     = None
_live_patches = []
_live_texts   = []
_live_title   = None
_live_bar_rects = []


def open_live_hexmap(pt):
    """
    Open an interactive matplotlib hexmap window.
    All 19 sensor cells shown; current target is highlighted in red border.
    Call update_live_hexmap() after every command to refresh it.
    """
    global _live_fig, _live_patches, _live_texts, _live_title, _live_bar_rects
    if not _HAS_MPL:
        return

    plt.ion()
    _live_fig, (ax_hex, ax_bar) = plt.subplots(
        1, 2, figsize=(12, 6),
        gridspec_kw={"width_ratios": [5, 4]})
    _live_fig.set_facecolor(BG)
    _live_fig.suptitle("", fontsize=11, fontweight="bold",
                        color="white", y=0.97)
    _live_title = _live_fig.texts[0]

    for ax in (ax_hex, ax_bar):
        ax.set_facecolor(BG)
        for sp in ax.spines.values():
            sp.set_edgecolor(EDGE)

    # ── Hexmap ────────────────────────────────────────────────────────────────
    _live_patches.clear()
    _live_texts.clear()
    for pt_n in SCAN_ORDER:
        xmm, ymm = POINTS[pt_n]
        patch = RegularPolygon(
            (xmm, ymm), numVertices=6, radius=4.5,
            facecolor=SENSOR_CMAP(0.0), edgecolor=EDGE, linewidth=0.8)
        ax_hex.add_patch(patch)
        _live_patches.append(patch)
        txt = ax_hex.text(xmm, ymm - 0.2, f"P{pt_n}", ha="center", va="center",
                          fontsize=5.5, color="white")
        _live_texts.append(txt)

    ax_hex.set_xlim(-23, 23)
    ax_hex.set_ylim(-21, 21)
    ax_hex.set_aspect("equal")
    ax_hex.axis("off")
    ax_hex.set_title("Live sensor values", fontsize=9, color="white", pad=4)

    sm = ScalarMappable(cmap=SENSOR_CMAP, norm=Normalize(0, 1))
    sm.set_array([])
    cb = _live_fig.colorbar(sm, ax=ax_hex, shrink=0.5, pad=0.02,
                             label="Pressure")
    cb.ax.yaxis.label.set_color("white")
    cb.ax.tick_params(colors="white")

    # ── Bar chart ─────────────────────────────────────────────────────────────
    _live_bar_rects.clear()
    x_pos = np.arange(19)
    rects = ax_bar.bar(x_pos, np.zeros(19),
                       color=[SENSOR_CMAP(0.0)] * 19,
                       edgecolor=EDGE, linewidth=0.4)
    _live_bar_rects.extend(rects)

    ax_bar.axvline(pt - 1, color="red", linewidth=2, alpha=0.6,
                   label=f"Target P{pt} → S{UR5_TO_RAW[pt]}")
    ax_bar.legend(fontsize=7, facecolor=BG, labelcolor="white",
                  edgecolor=EDGE)
    ax_bar.set_xlim(-0.5, 18.5)
    ax_bar.set_ylim(0, 1.05)
    ax_bar.set_xticks(x_pos)
    ax_bar.set_xticklabels([f"P{n}" for n in SCAN_ORDER],
                            rotation=90, fontsize=5, color="#aaaaaa")
    ax_bar.tick_params(axis="y", colors="#aaaaaa", labelsize=7)
    ax_bar.set_ylabel("Pressure", fontsize=8, color="#aaaaaa")
    ax_bar.set_title("All cells", fontsize=9, color="white", pad=4)
    ax_bar.grid(axis="y", color=EDGE, alpha=0.5, linewidth=0.5)

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.pause(0.05)
    print("[hexmap] Live window opened — it will update after each command.")


def update_live_hexmap(sensor_mod, pt, dx, dy):
    """Refresh the live hexmap with current sensor values."""
    if not _HAS_MPL or _live_fig is None or sensor_mod is None:
        return
    if not plt.fignum_exists(_live_fig.number):
        return   # window was closed by user

    vals = sensor_mod.get_values()
    exp_idx = UR5_TO_IDX[pt]
    exp_raw = RAW_CELLS[exp_idx]
    exp_val = vals[exp_idx]

    for i, (patch, txt) in enumerate(zip(_live_patches, _live_texts)):
        pt_n = SCAN_ORDER[i]
        v = float(np.clip(vals[UR5_TO_IDX[pt_n]], 0.0, 1.0))
        patch.set_facecolor(SENSOR_CMAP(v))
        if pt_n == pt:                         # target point
            patch.set_edgecolor("red")
            patch.set_linewidth(2.8)
        else:
            patch.set_edgecolor(EDGE)
            patch.set_linewidth(0.8)
        val_n = vals[UR5_TO_IDX[pt_n]]
        txt.set_text(f"P{pt_n}\n{val_n:.2f}" if val_n > 0.02 else f"P{pt_n}")
        txt.set_color("white" if v > 0.45 else "#cccccc")

    for i, rect in enumerate(_live_bar_rects):
        pt_n = SCAN_ORDER[i]
        v = float(np.clip(vals[UR5_TO_IDX[pt_n]], 0.0, 1.0))
        rect.set_height(v)
        rect.set_facecolor(SENSOR_CMAP(v))

    _live_title.set_text(
        f"P{pt:02d}  |  target S{exp_raw}={exp_val:.3f}  "
        f"|  offset dX={dx:+.2f} dY={dy:+.2f} mm")

    _live_fig.canvas.draw_idle()
    _live_fig.canvas.flush_events()


def close_live_hexmap():
    global _live_fig
    if _HAS_MPL and _live_fig is not None:
        try:
            plt.close(_live_fig)
        except Exception:
            pass
        _live_fig = None

# ── Deviation map ─────────────────────────────────────────────────────────────
def show_deviation_map(per_point_offsets, scan_results, save_path=None):
    """
    Static plot showing:
      Left panel  — hexmap coloured by detection quality; arrows show per-point
                    offset direction and magnitude.
      Right panel — bar chart of expected-cell peak value for each point;
                    hatched bars = wrong cell detected.
    """
    if not _HAS_MPL:
        print("[map] matplotlib not available — cannot show deviation map")
        return

    fig, (ax_map, ax_bar) = plt.subplots(
        1, 2, figsize=(15, 7.5),
        gridspec_kw={"width_ratios": [5, 4]})
    fig.set_facecolor(BG)
    fig.suptitle("Per-Point Calibration Deviation Map", fontsize=13,
                 fontweight="bold", color="white", y=0.97)

    for ax in (ax_map, ax_bar):
        ax.set_facecolor(BG)
        for sp in ax.spines.values():
            sp.set_edgecolor(EDGE)

    # ── Colour key for map ────────────────────────────────────────────────────
    STATUS_COLOR = {
        "correct_high":   "#22cc55",   # green  : correct, val > 0.40
        "correct_mid":    "#aacc22",   # yellow : correct, val 0.20–0.40
        "correct_low":    "#ff9900",   # orange : correct, val < 0.20
        "wrong":          "#cc2222",   # red    : wrong cell activated
        "offset_only":    "#4499cc",   # blue   : has offset, no scan
        "untouched":      "#333333",   # dark   : no data at all
    }

    def _status(pt):
        key = str(pt)
        dx, dy = per_point_offsets.get(pt, (0.0, 0.0))
        has_offset = (abs(dx) + abs(dy)) > 0.001
        if key not in scan_results or "correct" not in scan_results[key]:
            return "offset_only" if has_offset else "untouched", 0.0
        r = scan_results[key]
        ev = r.get("expected_val", 0.0)
        if not r.get("correct"):
            return "wrong", ev
        if ev > 0.40:
            return "correct_high", ev
        if ev > 0.20:
            return "correct_mid", ev
        return "correct_low", ev

    # ── Left: hexmap with offset arrows ───────────────────────────────────────
    arrow_scale = 3.0   # magnification so small offsets are visible

    for pt in SCAN_ORDER:
        xmm, ymm = POINTS[pt]
        status, ev = _status(pt)
        colour = STATUS_COLOR[status]
        dx, dy = per_point_offsets.get(pt, (0.0, 0.0))

        # Hexagon at nominal position
        patch = RegularPolygon(
            (xmm, ymm), numVertices=6, radius=4.3,
            facecolor=colour, edgecolor="#666666", linewidth=0.8, alpha=0.85)
        ax_map.add_patch(patch)

        # Label
        ax_map.text(xmm, ymm + 0.5, f"P{pt}", ha="center", va="center",
                    fontsize=6, color="white", fontweight="bold")
        if ev > 0:
            ax_map.text(xmm, ymm - 1.5, f"{ev:.2f}", ha="center", va="center",
                        fontsize=5, color="white")

        # Offset arrow (drawn from nominal centre, scaled)
        mag = (dx**2 + dy**2) ** 0.5
        if mag > 0.05:
            ax_map.annotate(
                "",
                xy=(xmm + dx * arrow_scale, ymm + dy * arrow_scale),
                xytext=(xmm, ymm),
                arrowprops=dict(
                    arrowstyle="->",
                    color="white",
                    lw=1.5,
                    mutation_scale=10,
                ),
            )
            # Magnitude text
            ax_map.text(
                xmm + dx * arrow_scale * 0.55,
                ymm + dy * arrow_scale * 0.55 + 1.0,
                f"{mag:.1f}mm",
                ha="center", va="bottom", fontsize=4.5,
                color="#dddddd",
            )

    ax_map.set_xlim(-24, 24)
    ax_map.set_ylim(-22, 22)
    ax_map.set_aspect("equal")
    ax_map.axis("off")
    ax_map.set_title(
        f"Offset arrows scaled ×{arrow_scale}   "
        "| green=good  yellow=moderate  orange=low  red=wrong  blue=no scan",
        fontsize=7.5, color="#cccccc", pad=6)

    # Legend patches
    import matplotlib.patches as mpatches
    legend_items = [
        mpatches.Patch(color=STATUS_COLOR["correct_high"],  label="Correct, val>0.40"),
        mpatches.Patch(color=STATUS_COLOR["correct_mid"],   label="Correct, val 0.20–0.40"),
        mpatches.Patch(color=STATUS_COLOR["correct_low"],   label="Correct, val<0.20"),
        mpatches.Patch(color=STATUS_COLOR["wrong"],         label="Wrong cell detected"),
        mpatches.Patch(color=STATUS_COLOR["offset_only"],   label="Offset set, not scanned"),
        mpatches.Patch(color=STATUS_COLOR["untouched"],     label="No data"),
    ]
    ax_map.legend(handles=legend_items, loc="lower right", fontsize=6.5,
                  facecolor=BG, labelcolor="white", edgecolor=EDGE,
                  framealpha=0.85)

    # Scale bar (1 mm real)
    ref_x, ref_y = -21, -20
    ax_map.annotate(
        "",
        xy=(ref_x + arrow_scale, ref_y),
        xytext=(ref_x, ref_y),
        arrowprops=dict(arrowstyle="<->", color="white", lw=1.2))
    ax_map.text(ref_x + arrow_scale / 2, ref_y + 1.0, "1 mm",
                ha="center", va="bottom", fontsize=6, color="white")

    # ── Right: bar chart of detection quality ─────────────────────────────────
    pts_sorted = SCAN_ORDER
    bar_colors, bar_heights, bar_hatch = [], [], []

    for pt in pts_sorted:
        status, ev = _status(pt)
        bar_heights.append(ev)
        bar_colors.append(STATUS_COLOR[status])
        bar_hatch.append("//" if status == "wrong" else "")

    x_pos = np.arange(19)
    for i, (h, c, hatch) in enumerate(zip(bar_heights, bar_colors, bar_hatch)):
        ax_bar.bar(x_pos[i], h, color=c, edgecolor="#888888",
                   linewidth=0.6, hatch=hatch, width=0.75)

    # Per-point offset magnitude as a step line (right y-axis)
    ax_bar2 = ax_bar.twinx()
    mags = [(per_point_offsets.get(pt, (0.0, 0.0))[0]**2 +
             per_point_offsets.get(pt, (0.0, 0.0))[1]**2)**0.5
            for pt in pts_sorted]
    ax_bar2.step(x_pos, mags, where="mid", color="#aaaaff",
                 linewidth=1.2, linestyle="--", label="Offset mag (mm)")
    ax_bar2.set_ylabel("Offset magnitude (mm)", fontsize=7, color="#aaaaff")
    ax_bar2.tick_params(axis="y", colors="#aaaaff", labelsize=6)
    ax_bar2.set_ylim(0, max(mags) * 2.5 + 0.5 if max(mags) > 0 else 1.0)
    ax_bar2.legend(loc="upper right", fontsize=6, facecolor=BG,
                   labelcolor="white", edgecolor=EDGE)

    ax_bar.axhline(0.40, color="#22cc55", linewidth=0.8,
                   linestyle=":", label="Good (0.40)")
    ax_bar.axhline(0.20, color="#ff9900", linewidth=0.8,
                   linestyle=":", label="Low (0.20)")
    ax_bar.set_xlim(-0.5, 18.5)
    ax_bar.set_ylim(0, 1.05)
    ax_bar.set_xticks(x_pos)
    ax_bar.set_xticklabels([f"P{p}" for p in pts_sorted],
                            rotation=90, fontsize=6, color="#aaaaaa")
    ax_bar.tick_params(axis="y", colors="#aaaaaa", labelsize=7)
    ax_bar.set_ylabel("Expected-cell peak value", fontsize=8, color="#aaaaaa")
    ax_bar.set_title("Detection quality per point\n(hatched = wrong cell)",
                     fontsize=8.5, color="white", pad=4)
    ax_bar.grid(axis="y", color=EDGE, alpha=0.4, linewidth=0.5)
    ax_bar.legend(loc="upper left", fontsize=6, facecolor=BG,
                  labelcolor="white", edgecolor=EDGE)

    plt.tight_layout(rect=[0, 0, 1, 0.94])

    if save_path:
        plt.savefig(save_path, dpi=150, facecolor=BG)
        print(f"[map] Deviation map saved → {save_path}")
    else:
        plt.show(block=False)
        plt.pause(0.1)
        print("[map] Deviation map displayed (close the window when done).")

# ── Press + read ──────────────────────────────────────────────────────────────
def do_press(rtde_c, pt, global_calib, per_point_offsets, sensor_mod):
    """Press at point pt, sample sensor, return result dict."""
    dx, dy = per_point_offsets.get(pt, (0.0, 0.0))
    pressed = build_pose(pt, global_calib, dx, dy, -INDENT_MM)
    surface = build_pose(pt, global_calib, dx, dy, 0.0)

    stop_live_display()   # pause terminal live line during press
    print(f"\n  Pressing {INDENT_MM:.0f}mm at P{pt} ...")

    rtde_c.moveL(pressed, VELOCITY_PRESS, ACCELERATION)
    time.sleep(0.5)

    readings = []
    for _ in range(10):
        if sensor_mod:
            readings.append(sensor_mod.get_values()[:])
        time.sleep(0.08)

    rtde_c.moveL(surface, VELOCITY_PRESS, ACCELERATION)
    time.sleep(0.3)

    if not sensor_mod or not readings:
        print("  [no sensor] press complete")
        return None

    peak_vals = [max(r[i] for r in readings) for i in range(19)]
    exp_idx   = UR5_TO_IDX[pt]
    exp_raw   = RAW_CELLS[exp_idx]
    exp_val   = peak_vals[exp_idx]
    top_idx   = max(range(19), key=lambda i: peak_vals[i])
    top_raw   = RAW_CELLS[top_idx]
    top_val   = peak_vals[top_idx]
    correct   = (top_idx == exp_idx)

    sep = "✓" if correct else "✗"
    print(f"\n  {sep} Expected : S{exp_raw:2d} (idx {exp_idx:2d}) → {_sensor_bar(exp_val)}")
    if not correct:
        print(f"    Actual   : S{top_raw:2d} (idx {top_idx:2d}) → {_sensor_bar(top_val)}")

    ranked = sorted(enumerate(peak_vals), key=lambda x: -x[1])[:3]
    top3_str = "  | ".join(
        f"S{RAW_CELLS[i]}{'→' if i==exp_idx else ''}={v:.3f}"
        for i, v in ranked if v > 0.01)
    print(f"  Top cells: {top3_str}")

    if exp_val < 0.10:
        print("  ⚠ Very low reading — robot may be badly misaligned")
    elif exp_val < 0.30:
        print("  ⚠ Low reading — consider nudging")
    elif exp_val > 0.55:
        print("  ✓ Good reading")
    else:
        print("  ~ Moderate reading")

    return {
        "expected_raw": exp_raw,
        "actual_raw":   top_raw,
        "expected_val": round(exp_val, 4),
        "top_val":      round(top_val, 4),
        "correct":      correct,
        "peak_vals":    [round(v, 4) for v in peak_vals],
    }

# ── Status display ────────────────────────────────────────────────────────────
def print_status(pt, dx, dy, step_mm, rtde_r, sensor_mod, global_calib):
    print(f"\n  ── P{pt} Status ───────────────────────────────────────────")
    print(f"  Per-point offset : dX={dx:+7.3f}  dY={dy:+7.3f} mm")
    print(f"  Nudge step       : {step_mm:.3f} mm")
    if rtde_r:
        tcp = rtde_r.getActualTCPPose()
        nom_x = REFERENCE_POSE[0] + (POINTS[pt][0] + global_calib[0]) / 1000
        nom_y = REFERENCE_POSE[1] + (POINTS[pt][1] + global_calib[1]) / 1000
        err_x = (tcp[0] - nom_x) * 1000
        err_y = (tcp[1] - nom_y) * 1000
        print(f"  TCP now          : X={tcp[0]:.5f}  Y={tcp[1]:.5f}  Z={tcp[2]:.5f}")
        print(f"  Nominal (+ glob) : X={nom_x:.5f}  Y={nom_y:.5f}")
        print(f"  Δ from nominal   : dX={err_x:+.3f}  dY={err_y:+.3f} mm "
              f"(= global + per-point)")
    if sensor_mod:
        vals = sensor_mod.get_values()
        exp_idx = UR5_TO_IDX[pt]
        exp_raw = RAW_CELLS[exp_idx]
        exp_val = vals[exp_idx]
        print(f"  S{exp_raw:2d} (expected) : {_sensor_bar(exp_val)}")
        active = [(RAW_CELLS[i], v) for i, v in enumerate(vals) if v > 0.05]
        if active:
            print("  Active cells     : " +
                  "  ".join(f"S{r}={v:.2f}" for r, v in active))
    print(f"  ──────────────────────────────────────────────────────────")

# ── Interactive per-point ─────────────────────────────────────────────────────
def interactive_point(pt, rtde_c, rtde_r, global_calib,
                      per_point_offsets, scan_results, sensor_mod):
    """
    Interactive adjustment loop for one point.
    Returns True → next point | None → save+quit or quit.
    """
    dx, dy = per_point_offsets.get(pt, (0.0, 0.0))
    step_mm = 0.5
    exp_raw = UR5_TO_RAW[pt]

    print(f"\n{'='*62}")
    print(f"  P{pt:02d}  nominal XY=({POINTS[pt][0]:+.0f},{POINTS[pt][1]:+.0f}) mm  "
          f"expected S{exp_raw}")
    print(f"{'='*62}")
    print("  x+/x-/y+/y-  step N  press  teach  status  map  ok  skip  save  quit")

    # Move to surface
    rtde_c.moveL(build_pose(pt, global_calib, dx, dy, 0.0),
                 VELOCITY_TRAVEL, ACCELERATION)

    # Open live window and start terminal live display
    open_live_hexmap(pt)
    update_live_hexmap(sensor_mod, pt, dx, dy)
    print_status(pt, dx, dy, step_mm, rtde_r, sensor_mod, global_calib)
    start_live_display(sensor_mod, pt)

    while True:
        try:
            stop_live_display()   # pause while waiting for command
            cmd = input(f"\n  P{pt} > ").strip().lower()
            # restart live display after input received
        except (EOFError, KeyboardInterrupt):
            print("\n  Interrupted")
            close_live_hexmap()
            return None

        moved = False
        save_and_quit = False

        if cmd == "x+":
            dx += step_mm;  moved = True
        elif cmd == "x-":
            dx -= step_mm;  moved = True
        elif cmd == "y+":
            dy += step_mm;  moved = True
        elif cmd == "y-":
            dy -= step_mm;  moved = True
        elif cmd.startswith("step"):
            try:
                step_mm = float(cmd.split()[1])
                print(f"  Step set to {step_mm:.3f} mm")
            except (IndexError, ValueError):
                print("  Usage: step 0.5")

        elif cmd == "press":
            per_point_offsets[pt] = (dx, dy)
            result = do_press(rtde_c, pt, global_calib, per_point_offsets, sensor_mod)
            if result:
                result["tcp"] = ([round(v, 6) for v in rtde_r.getActualTCPPose()]
                                 if rtde_r else [])
                scan_results[str(pt)] = result
            rtde_c.moveL(build_pose(pt, global_calib, dx, dy, 0.0),
                         VELOCITY_PRESS, ACCELERATION)

        elif cmd == "teach":
            # Record current TCP position → compute per-point offset
            if rtde_r:
                tcp = rtde_r.getActualTCPPose()
                nom_x = REFERENCE_POSE[0] + (POINTS[pt][0] + global_calib[0]) / 1000
                nom_y = REFERENCE_POSE[1] + (POINTS[pt][1] + global_calib[1]) / 1000
                dx = round((tcp[0] - nom_x) * 1000, 4)
                dy = round((tcp[1] - nom_y) * 1000, 4)
                per_point_offsets[pt] = (dx, dy)
                print(f"  ✓ Taught from TCP → dX={dx:+.3f} dY={dy:+.3f} mm")
            else:
                print("  ✗ No robot connection")

        elif cmd == "status":
            print_status(pt, dx, dy, step_mm, rtde_r, sensor_mod, global_calib)

        elif cmd == "map":
            print("  Generating deviation map ...")
            per_point_offsets[pt] = (dx, dy)
            show_deviation_map(per_point_offsets, scan_results)

        elif cmd in ("ok", ""):
            per_point_offsets[pt] = (dx, dy)
            print(f"  ✓ P{pt} accepted: dX={dx:+.3f} dY={dy:+.3f} mm")
            close_live_hexmap()
            return True

        elif cmd == "skip":
            print(f"  Skipped P{pt} — offset unchanged")
            close_live_hexmap()
            return True

        elif cmd == "save":
            per_point_offsets[pt] = (dx, dy)
            save_and_quit = True

        elif cmd == "quit":
            close_live_hexmap()
            return None

        else:
            print(f"  Unknown: '{cmd}' — try x+ x- y+ y- step N press teach "
                  f"status map ok skip save quit")

        if moved:
            per_point_offsets[pt] = (dx, dy)
            rtde_c.moveL(build_pose(pt, global_calib, dx, dy, 0.0),
                         VELOCITY_TRAVEL, ACCELERATION)
            print(f"  Moved → dX={dx:+.3f} dY={dy:+.3f} mm")

        # Refresh live hexmap and restart terminal display after every command
        update_live_hexmap(sensor_mod, pt, dx, dy)
        start_live_display(sensor_mod, pt)

        if save_and_quit:
            close_live_hexmap()
            return None

# ── Scan all points ───────────────────────────────────────────────────────────
def scan_all(points_to_scan, rtde_c, rtde_r, global_calib,
             per_point_offsets, scan_results, sensor_mod):
    total = len(points_to_scan)
    print(f"\n  Scanning {total} point(s) ...")
    for i, pt in enumerate(points_to_scan, 1):
        dx, dy = per_point_offsets.get(pt, (0.0, 0.0))
        print(f"\n  [{i:02d}/{total}] P{pt:02d}  "
              f"XY=({POINTS[pt][0]:+.0f},{POINTS[pt][1]:+.0f}) mm  "
              f"S{UR5_TO_RAW[pt]}  offset dX={dx:+.2f} dY={dy:+.2f}")
        try:
            rtde_c.moveL(build_pose(pt, global_calib, dx, dy, 0.0),
                         VELOCITY_TRAVEL, ACCELERATION)
        except Exception as e:
            print(f"  ✗ Move failed: {e}")
            continue
        tcp = rtde_r.getActualTCPPose() if rtde_r else []
        result = do_press(rtde_c, pt, global_calib, per_point_offsets, sensor_mod)
        if result:
            result["tcp"] = [round(v, 6) for v in tcp]
            scan_results[str(pt)] = result
        elif tcp:
            scan_results[str(pt)] = {"tcp": [round(v, 6) for v in tcp]}
    try:
        rtde_c.moveL(home_pose(global_calib), VELOCITY_TRAVEL, ACCELERATION)
    except Exception:
        pass
    return scan_results

# ── Summary report ────────────────────────────────────────────────────────────
def print_summary(scan_results, per_point_offsets):
    print(f"\n{'='*72}")
    print(f"  CALIBRATION SUMMARY — {len(scan_results)} point(s) scanned")
    print(f"{'='*72}")
    print(f"  {'P':>3}  {'Exp.cell':>8}  {'Act.cell':>8}  {'Exp.val':>8}  "
          f"{'Top val':>8}  {'OK':>4}  {'dX':>7}  {'dY':>7}")
    print(f"  {'-'*3}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*4}  "
          f"{'-'*7}  {'-'*7}")
    ok_count = 0
    for pt in SCAN_ORDER:
        if str(pt) not in scan_results:
            continue
        r  = scan_results[str(pt)]
        dx, dy = per_point_offsets.get(pt, (0.0, 0.0))
        c  = r.get("correct")
        if c is True:
            ok_count += 1
        ok = "✓" if c is True else ("✗" if c is False else "—")
        print(f"  P{pt:2d}  "
              f"    S{r.get('expected_raw','?'):>4}  "
              f"    S{r.get('actual_raw','?'):>4}  "
              f"{r.get('expected_val', 0.0):>8.3f}  "
              f"{r.get('top_val', 0.0):>8.3f}  "
              f"{ok:>4}  {dx:>+7.3f}  {dy:>+7.3f}")
    n_sensor = sum(1 for r in scan_results.values() if "correct" in r)
    if n_sensor:
        print(f"\n  Correct cell: {ok_count}/{n_sensor} "
              f"({ok_count/n_sensor*100:.0f}%)")
    bad = [p for p in SCAN_ORDER
           if scan_results.get(str(p), {}).get("correct") is False]
    low = [p for p in SCAN_ORDER
           if scan_results.get(str(p), {}).get("correct") is not False
           and scan_results.get(str(p), {}).get("expected_val", 1.0) < 0.25]
    if bad:
        print("  ✗ Wrong cell: " + ", ".join(f"P{p}" for p in bad))
    if low:
        print("  ⚠ Low val  : " + ", ".join(f"P{p}" for p in low))
    if not bad and not low:
        print("  All scanned points look good!")
    print(f"{'='*72}")

# ── Argument parsing ──────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Per-point UR5 + sensor calibration")
    p.add_argument("--scan",      action="store_true",
                   help="Auto-scan all points, then show deviation map")
    p.add_argument("--point",     type=int,
                   help="Single point for interactive adjustment")
    p.add_argument("--points",    nargs="+", type=int,
                   help="List of points for interactive adjustment")
    p.add_argument("--map",       action="store_true",
                   help="Show deviation map from saved calib_points.json and exit")
    p.add_argument("--save-map",  type=str, metavar="FILE",
                   help="Save deviation map to PNG file and exit")
    p.add_argument("--no-sensor", action="store_true",
                   help="Skip sensor checks (robot motion only)")
    p.add_argument("--tip", default=None,
                   help="Tip name (e.g. short, long_5mm). Loads calib_<tip>.json, saves calib_points_<tip>.json")
    return p.parse_args()

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()
    tip = args.tip

    # ── Map-only mode (loads from --tip or default) ───────────────────────────
    if args.map or args.save_map:
        global_calib = load_global_calib(tip)
        per_point_offsets, scan_results = load_point_offsets(tip)
        save_path = args.save_map or None
        show_deviation_map(per_point_offsets, scan_results, save_path=save_path)
        if args.map and not args.save_map:
            input("[map] Press Enter to close and exit ...")
        return

    # ── Choose the starting calibration (global X/Y/Z base) ───────────────────
    if tip:
        # Explicit --tip keeps the old non-interactive behaviour.
        global_calib = load_global_calib(tip)
        per_point_offsets, scan_results = load_point_offsets(tip)
        base_label = tip
        print(f"[calib] Tip profile  : {tip}")
        print(f"[calib] Global file  : calib_{tip}.json")
    else:
        global_calib, base_label, base_points_file = choose_starting_calib()
        per_point_offsets, scan_results = load_existing_deviations(base_points_file)

    default_out = (f"calib_points_{base_label}.json"
                   if base_label else "calib_points.json")

    # ── Determine target points ───────────────────────────────────────────────
    if args.point:
        target_points = [args.point]
    elif args.points:
        target_points = sorted(set(args.points))
    else:
        target_points = SCAN_ORDER

    invalid = [p for p in target_points if p not in POINTS]
    if invalid:
        print(f"[calib] Invalid point(s): {invalid}  (valid: 1–19)")
        sys.exit(1)

    print("=" * 62)
    print("  Per-Point Calibration — Star-Nose Sensor")
    print("=" * 62)

    # ── Start sensor ──────────────────────────────────────────────────────────
    sensor_mod = None
    if not args.no_sensor:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import sensor as sensor_mod
        print("[calib] Starting sensor ...")
        sensor_mod.start()
        if not sensor_mod.wait_until_ready(timeout=40):
            print("[calib] ⚠ Sensor not ready — continuing without it")
            sensor_mod = None
        else:
            print("[calib] Sensor ready!")

    # ── Connect to UR5 ────────────────────────────────────────────────────────
    rtde_c = rtde_r = None
    print("[calib] Connecting to UR5 ...")
    for attempt in range(3):
        try:
            import rtde_control, rtde_receive
            rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
            print("[calib] UR5 connected!")
            break
        except Exception as e:
            print(f"[calib] Attempt {attempt+1}/3 failed: {e}")
            try:
                if rtde_r: rtde_r.disconnect()
            except Exception:
                pass
            rtde_r = rtde_c = None
            time.sleep(2)

    if rtde_c is None:
        print("[calib] Could not connect to UR5 — aborting")
        sys.exit(1)

    tcp = rtde_r.getActualTCPPose()
    print(f"[calib] TCP: X={tcp[0]:.4f} Y={tcp[1]:.4f} Z={tcp[2]:.4f}")
    print("[calib] Moving to home ...")
    rtde_c.moveL(home_pose(global_calib), VELOCITY_TRAVEL, ACCELERATION)
    print("[calib] At home. Starting.\n")

    # ── Scan mode ─────────────────────────────────────────────────────────────
    if args.scan:
        scan_results = scan_all(
            target_points, rtde_c, rtde_r,
            global_calib, per_point_offsets, scan_results, sensor_mod)
        print_summary(scan_results, per_point_offsets)
        saved_base = prompt_save(global_calib, per_point_offsets, scan_results, default_out)
        map_path = os.path.join(CALIB_DIR, f"deviation_map_{saved_base}.png")
        show_deviation_map(per_point_offsets, scan_results, save_path=map_path)
        print("[calib] Scan complete.")

    # ── Interactive mode ──────────────────────────────────────────────────────
    else:
        print("  TIP: run --scan first to find which points need fixing, "
              "then --points N N ... to fix them.\n")

        for pt in target_points:
            print(f"\n  → Baseline press at P{pt} ...")
            dx, dy = per_point_offsets.get(pt, (0.0, 0.0))
            rtde_c.moveL(build_pose(pt, global_calib, dx, dy, 0.0),
                         VELOCITY_TRAVEL, ACCELERATION)
            result = do_press(rtde_c, pt, global_calib, per_point_offsets, sensor_mod)
            if result:
                result["tcp"] = ([round(v, 6) for v in rtde_r.getActualTCPPose()]
                                 if rtde_r else [])
                scan_results[str(pt)] = result
            rtde_c.moveL(build_pose(pt, global_calib, dx, dy, 0.0),
                         VELOCITY_PRESS, ACCELERATION)

            cont = interactive_point(
                pt, rtde_c, rtde_r,
                global_calib, per_point_offsets, scan_results, sensor_mod)

            if cont is None:
                prompt_save(global_calib, per_point_offsets, scan_results, default_out)
                show_deviation_map(per_point_offsets, scan_results)
                break
        else:
            print_summary(scan_results, per_point_offsets)
            prompt_save(global_calib, per_point_offsets, scan_results, default_out)
            show_deviation_map(per_point_offsets, scan_results)
            print("\n[calib] All done!")

    # ── Clean up ──────────────────────────────────────────────────────────────
    stop_live_display()
    close_live_hexmap()
    try:
        rtde_c.moveL(home_pose(global_calib), VELOCITY_TRAVEL, ACCELERATION)
        rtde_c.stopScript()
    except Exception:
        pass

    if _HAS_MPL:
        plt.ioff()
        try:
            plt.show(block=True)   # keep any remaining windows open
        except Exception:
            pass

    print("[calib] Done. Goodbye.")


if __name__ == "__main__":
    main()
