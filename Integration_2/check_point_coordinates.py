"""
check_point_coordinates.py
LIVE coordinate + mapping check for the 19 UR5 calibration points.

The robot drives to each point (P1..P19). A window shows, in real time:
  • LEFT  (ROBOT frame): the TCP offset from REFERENCE_POSE the arm executes,
    with the live TCP as a red ✕ and a trail. Written coordinates + the
    UR5→sensor mapping (Sraw + a1..e5 label) are printed next to every point.
  • RIGHT (SENSOR frame): the same points as the hex grid (matches
    visualizer_2d.POINTS_MM), with the current target highlighted.

WHICH COORDINATE SET IS "CORRECT"?
──────────────────────────────────
There is only ONE set of 19 points. `ur5_control.POINTS` and
`calibrate_points.POINTS` are identical (robot frame). `visualizer_2d.POINTS_MM`
is that SAME set rotated ~120° into the sensor frame (the sensor is mounted
~120° from the robot base). So the robot-frame plot looking "flipped" vs the
sensor grid is expected — it is the sensor grid rotated 120°, not an error.

The robot frame is correct for DRIVING the arm; the sensor frame is correct for
MATCHING the display.

The tour NEVER presses or touches the sensor — it hovers over each point. On
start it ASKS how far below the surface to fly (the "closeness"), showing the
maximum, which is taken from the calibration point (|global z| + 5 mm; e.g.
hollow-dome v2 z=-10.6 → 15.6). The sensor is read live, so cells that respond
as the tip nears each point light up on the hex grid — a contact-free look at
the UR5→sensor mapping.

⚠ THIS MOVES THE REAL ROBOT (but does not touch the sensor).

Usage:
  python3 check_point_coordinates.py                  # asks for closeness, then tours
  python3 check_point_coordinates.py --tip short      # max derived from calib_short.json
  python3 check_point_coordinates.py --closeness 12   # skip the prompt (clamped to max)
  python3 check_point_coordinates.py --points 5 10 17 --dwell 1.5
"""

import argparse
import math
import sys
import time

import calibrate_points as cp   # single source of truth for the constants

ROBOT_IP = cp.ROBOT_IP

# Hard safety cap: how far BELOW the reference surface the tip may ever go (mm).
# Derived from the deepest calib in use (hollow-dome v2: global z = -10.6 mm plus
# ~5 mm ≈ 15.6 mm). The tour Z (--z-mm) is clamped so it can get this close to
# the sensor but no closer — it never bottoms out / presses.
MAX_CLOSE_MM = 15.6


# ── Helpers ───────────────────────────────────────────────────────────────────
def idx_label(idx):
    """Hex label ('a1'..'e5') for a get_values() array index."""
    return cp.SENSOR_CELL_TO_LABEL[cp.RAW_CELLS[idx]]


def _dist3d_mm(a, b):
    return math.hypot((a[0]-b[0])*1000, (a[1]-b[1])*1000, (a[2]-b[2])*1000)


def print_table(points, global_calib, closeness, max_close):
    gx, gy, gz = global_calib
    rp = cp.REFERENCE_POSE
    print("=" * 92)
    print("  POINT COORDINATES  —  robot offset from REFERENCE_POSE  +  UR5→sensor mapping")
    print("=" * 92)
    print(f"  REFERENCE_POSE : X={rp[0]:+.5f}  Y={rp[1]:+.5f}  Z={rp[2]:+.5f} m  (P10/c3 centre)")
    print(f"  Global offset  : X={gx:+.3f}  Y={gy:+.3f}  Z={gz:+.3f} mm   |   "
          f"closeness {closeness:.1f} mm below surface (max {max_close:.1f}, no touch)")
    print("-" * 92)
    print(f"  {'P':>3} {'lbl':>4} {'Sraw':>5} | {'robot ΔX':>9} {'robot ΔY':>9} | "
          f"{'sensor X':>9} {'sensor Y':>9} (mm)")
    print("-" * 92)
    for pt in points:
        dx, dy = cp.POINTS[pt]
        sx, sy = cp.DISPLAY_XY[pt]
        print(f"  P{pt:>2} {cp.UR5_TO_LABEL[pt]:>4} S{cp.UR5_TO_RAW[pt]:>4} | "
              f"{dx+gx:>+9.2f} {dy+gy:>+9.2f} | {sx:>+9.1f} {sy:>+9.1f}")
    print("=" * 92)


# ── Live figure ───────────────────────────────────────────────────────────────
class LiveView:
    def __init__(self, points, global_calib, tour_z):
        import matplotlib.pyplot as plt
        from matplotlib.patches import RegularPolygon
        self.plt = plt
        self.points = points
        self.global_calib = global_calib
        self.tour_z = tour_z
        self.rp = cp.REFERENCE_POSE
        gx, gy, _ = global_calib

        BG, EDGE = cp.BG, cp.EDGE
        plt.ion()
        self.fig, (self.ax_r, self.ax_s) = plt.subplots(1, 2, figsize=(15.5, 7.8))
        self.fig.set_facecolor(BG)
        self.fig.suptitle("", fontsize=12, fontweight="bold", color="white", y=0.98)
        self.title = self.fig.texts[0]

        for ax in (self.ax_r, self.ax_s):
            ax.set_facecolor(BG)
            for sp in ax.spines.values():
                sp.set_edgecolor(EDGE)
            ax.tick_params(colors="#aaaaaa", labelsize=7)

        # LEFT — robot frame (offset mm from reference) with written coords
        self.ax_r.axhline(0, color=EDGE, lw=0.8)
        self.ax_r.axvline(0, color=EDGE, lw=0.8)
        self.ax_r.plot(0, 0, marker="*", ms=16, color="#ffcc00", zorder=5)
        self.ax_r.annotate("REFERENCE_POSE\n(P10/c3)", (0, 0),
                           textcoords="offset points", xytext=(6, 6),
                           fontsize=6, color="#ffcc00")
        self._robot_dot = {}
        for pt in points:
            ox, oy = cp.POINTS[pt][0] + gx, cp.POINTS[pt][1] + gy
            (dot,) = self.ax_r.plot(ox, oy, "o", ms=6, color="#3a6a8a",
                                    markeredgecolor="#88bbdd", markeredgewidth=0.5,
                                    zorder=2)
            self._robot_dot[pt] = dot
            self.ax_r.annotate(
                f"P{pt} {cp.UR5_TO_LABEL[pt]}·S{cp.UR5_TO_RAW[pt]}\n({ox:+.0f},{oy:+.0f})",
                (ox, oy), textcoords="offset points", xytext=(4, 4),
                fontsize=5.0, color="#a9b8c6")
        (self._trail,) = self.ax_r.plot([], [], "-", color="#ff8844",
                                        lw=1.2, alpha=0.6, zorder=3)
        (self._tcp,) = self.ax_r.plot([], [], "X", ms=15, color="#ff3333",
                                      markeredgecolor="white", markeredgewidth=1.0,
                                      zorder=6)
        (self._tgt_ring,) = self.ax_r.plot([], [], "o", ms=17, mfc="none",
                                           mec="#33ff88", mew=2.0, zorder=4)
        self.ax_r.set_aspect("equal")
        self.ax_r.set_xlim(-26, 26)
        self.ax_r.set_ylim(-23, 23)
        self.ax_r.set_xlabel("Robot +X offset from reference (mm)", fontsize=8, color="#ccc")
        self.ax_r.set_ylabel("Robot +Y offset from reference (mm)", fontsize=8, color="#ccc")
        self.ax_r.set_title("ROBOT frame — live TCP (✕) vs targets  [drives the arm]",
                            fontsize=9, color="white", pad=6)
        self.ax_r.grid(True, color=EDGE, alpha=0.35, lw=0.5)

        # RIGHT — sensor hex grid with written coords + mapping
        self._hex = {}
        for pt in cp.SCAN_ORDER:
            sx, sy = cp.DISPLAY_XY[pt]
            patch = RegularPolygon((sx, sy), numVertices=6, radius=4.3,
                                   facecolor="#22333a", edgecolor="#666666",
                                   linewidth=0.8)
            self.ax_s.add_patch(patch)
            self._hex[pt] = patch
            self.ax_s.text(sx, sy + 1.2, f"P{pt} {cp.UR5_TO_LABEL[pt]}", ha="center",
                           va="center", fontsize=5.5, color="white", fontweight="bold")
            self.ax_s.text(sx, sy - 1.9, f"S{cp.UR5_TO_RAW[pt]} ({sx:+.0f},{sy:+.0f})",
                           ha="center", va="center", fontsize=4.5, color="#aaccff")
        (self._det_ring,) = self.ax_s.plot([], [], "o", ms=20, mfc="none",
                                           mec="#ff8800", mew=2.2, zorder=5)
        self.ax_s.set_xlim(-26, 26)
        self.ax_s.set_ylim(-23, 23)
        self.ax_s.set_aspect("equal")
        self.ax_s.axis("off")
        self.ax_s.set_title("SENSOR frame — green=target  orange=cell that fired",
                            fontsize=9, color="white", pad=6)

        self.fig.tight_layout(rect=[0, 0, 1, 0.94])
        self._trail_x, self._trail_y = [], []
        plt.pause(0.05)

    def alive(self):
        return self.plt.fignum_exists(self.fig.number)

    def set_target(self, pt):
        gx, gy, _ = self.global_calib
        if pt is None:
            self._tgt_ring.set_data([], [])
        else:
            self._tgt_ring.set_data([cp.POINTS[pt][0] + gx], [cp.POINTS[pt][1] + gy])
        for p, patch in self._hex.items():
            hot = (p == pt)
            patch.set_edgecolor("#33ff88" if hot else "#666666")
            patch.set_linewidth(2.6 if hot else 0.8)

    def set_sensor(self, peak):
        """Colour hex cells by peak reading and ring the cell that fired most."""
        for pt, patch in self._hex.items():
            v = max(0.0, min(1.0, peak[pt - 1]))
            patch.set_facecolor(cp.SENSOR_CMAP(v))
        top_idx = max(range(19), key=lambda i: peak[i])
        if peak[top_idx] > 0.05:
            top_pt = top_idx + 1
            self._det_ring.set_data([cp.DISPLAY_XY[top_pt][0]], [cp.DISPLAY_XY[top_pt][1]])
        else:
            self._det_ring.set_data([], [])

    def mark_result(self, pt, correct):
        self._robot_dot[pt].set_color("#33cc55" if correct else "#cc3333")
        self._robot_dot[pt].set_markersize(9)

    def update(self, tcp, pt, moving, extra=""):
        if not self.alive():
            return False
        rp = self.rp
        lx, ly, lz = (tcp[0]-rp[0])*1000, (tcp[1]-rp[1])*1000, (tcp[2]-rp[2])*1000
        self._tcp.set_data([lx], [ly])
        self._trail_x.append(lx); self._trail_y.append(ly)
        if len(self._trail_x) > 140:
            self._trail_x = self._trail_x[-140:]; self._trail_y = self._trail_y[-140:]
        self._trail.set_data(self._trail_x, self._trail_y)
        if pt is not None:
            tgt = cp.build_pose(pt, self.global_calib, extra_z=self.tour_z)
            ex, ey = (tcp[0]-tgt[0])*1000, (tcp[1]-tgt[1])*1000
            state = "moving →" if moving else "AT POINT"
            self.title.set_text(
                f"P{pt:02d} ({cp.UR5_TO_LABEL[pt]}·S{cp.UR5_TO_RAW[pt]}) {state}   |   "
                f"TCP off ref X={lx:+.2f} Y={ly:+.2f} Z={lz:+.2f} mm   |   "
                f"err vs target dX={ex:+.2f} dY={ey:+.2f} mm{extra}")
        self.fig.canvas.draw_idle(); self.fig.canvas.flush_events()
        return True


# ── Robot connection ──────────────────────────────────────────────────────────
def connect():
    print("[live] Connecting to UR5 ...")
    for attempt in range(3):
        try:
            import rtde_control, rtde_receive
            rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
            print("[live] UR5 connected!")
            return rtde_c, rtde_r
        except Exception as e:
            print(f"[live] Attempt {attempt+1}/3 failed: {e}")
            time.sleep(2)
    return None, None


def drive(rtde_c, rtde_r, view, target, pt, speed, accel,
          sensor_mod=None, peak=None):
    """Async-move to `target`, animating until arrival. If `peak` (a 19-list)
    and `sensor_mod` are given, accumulate per-cell peaks live. Returns False if
    the window was closed."""
    POS_TOL_MM, SPEED_TOL, TIMEOUT_S = 0.4, 3.0, 25.0
    rtde_c.moveL(target, speed, accel, True)
    t0 = time.time()
    while True:
        if not view.alive():
            try: rtde_c.stopL(2.0)
            except Exception: pass
            return False
        tcp = rtde_r.getActualTCPPose()
        if sensor_mod is not None and peak is not None:
            vals = sensor_mod.get_values()
            for i in range(19):
                if vals[i] > peak[i]:
                    peak[i] = vals[i]
            view.set_sensor(peak)
        spd = rtde_r.getActualTCPSpeed()
        spd_mm = math.hypot(spd[0], spd[1], spd[2]) * 1000
        reached = _dist3d_mm(tcp, target) < POS_TOL_MM and spd_mm < SPEED_TOL
        view.update(tcp, pt, moving=not reached)
        view.plt.pause(0.03)
        if reached or (time.time() - t0) > TIMEOUT_S:
            return True


def dwell(rtde_r, view, pt, seconds, sensor_mod=None, peak=None):
    end = time.time() + seconds
    while time.time() < end:
        if not view.alive():
            return False
        if sensor_mod is not None and peak is not None:
            vals = sensor_mod.get_values()
            for i in range(19):
                if vals[i] > peak[i]:
                    peak[i] = vals[i]
            view.set_sensor(peak)
        view.update(rtde_r.getActualTCPPose(), pt, moving=False)
        view.plt.pause(0.03)
    return True


def ask_closeness(max_close):
    """Prompt for how far below the surface to fly. Max comes from the calib."""
    print("\n" + "=" * 62)
    print("  CLOSENESS TO SENSOR")
    print("=" * 62)
    print("  How far BELOW the surface should the tip fly, in mm?")
    print(f"    0     = at the surface")
    print(f"    {max_close:.1f}  = closest allowed  (from the calibration point)")
    print("  The robot only HOVERS at this depth — it never presses/touches.")
    while True:
        try:
            raw = input(f"  Closeness mm [0 – {max_close:.1f}] > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[live] Aborted."); sys.exit(0)
        if raw == "":
            print("  Enter a value (e.g. 12.5)"); continue
        try:
            v = float(raw)
        except ValueError:
            print("  Not a number — try again"); continue
        if v < 0:
            print("  Must be >= 0"); continue
        if v > max_close:
            print(f"  Above the {max_close:.1f} mm limit — clamped to {max_close:.1f}")
            v = max_close
        print(f"  → Flying {v:.1f} mm below the surface.")
        return v


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="LIVE UR5 point-coordinate + UR5→sensor mapping check")
    ap.add_argument("--tip", default=None,
                    help="Tip name → load calib_<tip>.json for the global offset")
    ap.add_argument("--no-global", action="store_true",
                    help="Ignore calib.json (zero global offset)")
    ap.add_argument("--points", nargs="+", type=int, default=None,
                    help="Subset of points (default: all 1..19)")
    ap.add_argument("--closeness", type=float, default=None,
                    help="mm below the surface to fly (skips the prompt). "
                         "Clamped to the calibration-derived max.")
    ap.add_argument("--no-sensor", action="store_true",
                    help="Do not start the sensor / colour the hex cells")
    ap.add_argument("--dwell", type=float, default=1.0,
                    help="Seconds to pause at each point (default 1.0)")
    ap.add_argument("--speed", type=float, default=cp.VELOCITY_TRAVEL, help="TCP speed m/s")
    ap.add_argument("--accel", type=float, default=cp.ACCELERATION, help="TCP accel m/s^2")
    args = ap.parse_args()

    global_calib = (0.0, 0.0, 0.0) if args.no_global else cp.load_global_calib(args.tip)
    gx, gy, gz = global_calib
    global_xy = (gx, gy, 0.0)      # closeness is measured from the reference surface,
                                   # so gz is NOT re-applied to Z (only to the max).
    points = args.points if args.points else list(cp.SCAN_ORDER)
    bad = [p for p in points if p not in cp.POINTS]
    if bad:
        print(f"[live] Invalid point(s): {bad}  (valid 1..19)"); sys.exit(1)

    # Max closeness comes from the calibration point: |global z| + 5 mm margin
    # (e.g. hollow-dome v2 z=-10.6 → 15.6). No calib → the default cap.
    max_close = MAX_CLOSE_MM if gz == 0.0 else round(abs(gz) + 5.0, 1)

    # Resolve the closeness value: CLI overrides, otherwise ask the user.
    if args.closeness is not None:
        closeness = max(0.0, args.closeness)
        if closeness > max_close:
            print(f"[live] ⚠ closeness {closeness:.1f} > max {max_close:.1f} — clamping")
            closeness = max_close
    else:
        closeness = ask_closeness(max_close)

    print_table(points, global_calib, closeness, max_close)

    if not cp._HAS_MPL:
        print("[live] matplotlib not available — cannot show the live view."); sys.exit(1)

    # Sensor — on by default so you can watch cells respond as the tip nears.
    sensor_mod = None
    if not args.no_sensor:
        sys.path.insert(0, cp.CALIB_DIR)
        import sensor as sensor_mod
        print("[live] Starting sensor ...")
        sensor_mod.start()
        if not sensor_mod.wait_until_ready(timeout=40):
            print("[live] ⚠ Sensor not ready — continuing without it")
            sensor_mod = None
        else:
            print("[live] Sensor ready!")

    print(f"\n⚠  The robot WILL hover over {len(points)} point(s) at "
          f"{closeness:.1f} mm below the surface (max {max_close:.1f}), "
          f"speed {args.speed} m/s. It does NOT touch/press.")
    try:
        input("   Press Enter to start (Ctrl-C to abort) ...")
    except (EOFError, KeyboardInterrupt):
        print("\n[live] Aborted."); sys.exit(0)

    rtde_c, rtde_r = connect()
    if rtde_c is None:
        print("[live] Could not connect to UR5 — aborting"); sys.exit(1)

    view = LiveView(points, global_xy, -closeness)
    try:
        print("[live] Moving to home (safe height) ...")
        view.set_target(None)
        if not drive(rtde_c, rtde_r, view, cp.home_pose(global_xy), None,
                     args.speed, args.accel):
            raise KeyboardInterrupt

        for pt in points:
            if not view.alive():
                break
            print(f"[live] → P{pt} ({cp.UR5_TO_LABEL[pt]}·S{cp.UR5_TO_RAW[pt]})")
            view.set_target(pt)
            peak = [0.0] * 19            # live proximity peak for this point
            hover_pose = cp.build_pose(pt, global_xy, extra_z=-closeness)
            if not drive(rtde_c, rtde_r, view, hover_pose, pt, args.speed, args.accel,
                         sensor_mod, peak):
                break
            if not dwell(rtde_r, view, pt, args.dwell, sensor_mod, peak):
                break

        print("[live] Returning home ...")
        view.set_target(None)
        drive(rtde_c, rtde_r, view, cp.home_pose(global_xy), None,
              args.speed, args.accel)
        print("[live] Tour complete.")
    except KeyboardInterrupt:
        print("\n[live] Interrupted — stopping robot.")
        try: rtde_c.stopL(2.0)
        except Exception: pass
    finally:
        try: rtde_c.stopScript()
        except Exception: pass
        if view.alive():
            print("[live] Close the window to exit.")
            view.plt.ioff()
            try: view.plt.show(block=True)
            except Exception: pass


if __name__ == "__main__":
    main()
