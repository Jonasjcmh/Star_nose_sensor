"""
calibrate_live.py
LIVE calibration window (pygame) — drives the UR5 through the press points and
shows, in real time, the same hex grid as visualizer_2d.py with calibration
overlays.

WHAT IT DOES
────────────
One self-contained process:
  • DRIVES the robot: home → for each point press down by --indent, dwell,
    retract → home.  (⚠ THIS MOVES THE REAL ROBOT AND TOUCHES THE SENSOR.)
  • DRAWS a live sensor-frame hex grid (matches visualizer_2d.POINTS_MM /
    calibrate_points.DISPLAY_XY), cells coloured live from the sensor.

OVERLAYS (all live)
───────────────────
  • GREEN ring   — the point we are pressing (the TARGET).
  • ORANGE ring  — the hex cell that actually fired (max live reading); if it
                   sits on the green ring, the UR5→sensor mapping is spot on.
  • WHITE dot    — where the tip ACTUALLY pressed, recorded at the bottom of the
                   press and mapped robot-frame → sensor-frame (so target vs
                   actual deviation is visible right on the grid).
  • CYAN star    — the live robot tip, moving continuously, with a fading trail.
  • AMBER cross  — the current live X/Y trim (0,0 = none), on the map.

Coordinates: the live TCP (robot frame) is mapped into the sensor/display frame
with calibrate_points._to_display after subtracting REFERENCE_POSE and the
global calibration offset, so it lines up with the DISPLAY_XY hexes exactly.

LIVE TRIM (X/Y/Z) — a small manual nudge (±10 mm) on top of the loaded
calibration, applied to every press and updated live even mid-dwell:
  • Drag the MAP     to set the X/Y trim (click sets it to that spot).
  • Drag the TIP HEIGHT gauge (right of the map) to set the Z trim.
  • Or use ← → ↑ ↓ for X/Y and -/= for Z (hold to repeat); N resets to 0.
This is NOT saved to calib_points.json — it's a live fine-tune for this run.

Usage
─────
  python3 calibrate_live.py                       # default calib, all 19 points
  python3 calibrate_live.py --tip new_hollow_dome_v2
  python3 calibrate_live.py --points 5 10 17 --indent 6 --dwell 1.5
  python3 calibrate_live.py --no-robot            # sensor-only view (no driving)
  python3 calibrate_live.py --no-robot --no-sensor  # pure demo, no hardware

Keys:  ESC/Q quit (robot returns home)   C recalibrate sensor
       T toggle trail   L toggle labels   P pause between points
       arrows/-/=  nudge X/Y/Z trim   N reset trim   (or drag map/gauge)
"""
import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import pygame
except ImportError:
    os.system(f"{sys.executable} -m pip install pygame")
    import pygame

import calibrate_points as cp   # single source of truth for geometry/constants

# ── Colours / layout (mirrors visualizer_2d.py) ───────────────────────────────
W, H   = 1000, 720
FPS    = 60
BG     = (18,  18,  24)
PANEL  = (26,  26,  36)
CARD   = (35,  35,  50)
TEXT   = (200, 200, 210)
MUTED  = (80,  80, 100)
WHITE  = (255, 255, 255)
GREEN  = (42,  200, 120)
RED    = (220, 60,  60)
AMBER  = (240, 160, 30)
CYAN   = (60,  220, 230)
ORANGE = (255, 136, 0)

HEX_R          = 34
HEX_CX, HEX_CY = 350, 360
SX, SY         = 13.0, 13.0     # mm → px (sensor-frame display)

# Live manual-trim range (mm) — the map/gauge sliders and arrow/-= keys nudge
# on top of the loaded calibration; kept small since this is a fine-tune, not
# a full re-jog like friction_live's.
OFFSET_LIMIT_XY = 10.0
OFFSET_LIMIT_Z  = 10.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def lerp_color(v):
    """Same 5-stop ramp as visualizer_2d / calibrate_points.SENSOR_CMAP."""
    v = max(0.0, min(1.0, v))
    ramp = [(42, 181, 160), (51, 230, 102), (255, 230, 25),
            (255, 115, 0), (220, 0, 0)]
    idx = v * (len(ramp) - 1)
    lo  = int(idx)
    hi  = min(lo + 1, len(ramp) - 1)
    t   = idx - lo
    return tuple(int(ramp[lo][j] + t * (ramp[hi][j] - ramp[lo][j]))
                 for j in range(3))


def hex_pts(cx, cy, r):
    return [(cx + r * math.cos(math.radians(60 * i + 30)),
             cy + r * math.sin(math.radians(60 * i + 30))) for i in range(6)]


def star_pts(cx, cy, r, spikes=5):
    """Points for a filled star marker (outer r, inner 0.45 r)."""
    pts = []
    for i in range(spikes * 2):
        rad = r if i % 2 == 0 else r * 0.45
        ang = math.radians(-90 + i * 180.0 / spikes)
        pts.append((cx + rad * math.cos(ang), cy + rad * math.sin(ang)))
    return pts


def blit(surf, text, font, col, x, y, align='left'):
    s = font.render(str(text), True, col)
    if align == 'center':
        x -= s.get_width() // 2
    elif align == 'right':
        x -= s.get_width()
    surf.blit(s, (int(x), int(y)))
    return s.get_width()


def disp_to_screen(xmm, ymm):
    """Sensor-frame mm → screen px (y flipped, like visualizer_2d)."""
    return HEX_CX + xmm * SX, HEX_CY - ymm * SY


def robot_to_screen(xmm, ymm):
    """Robot-frame mm (e.g. a live X/Y trim) → screen px, via the display frame."""
    return disp_to_screen(*cp._to_display(xmm, ymm))


def screen_to_robot(sx, sy):
    """Inverse of robot_to_screen: screen px → robot-frame mm. Used to turn a
    mouse click/drag on the hex map into a live X/Y trim."""
    xd = (sx - HEX_CX) / SX
    yd = (HEX_CY - sy) / SY
    (a, b), (c, d) = cp._ROBOT_TO_SENSOR
    det = a * d - b * c
    return ((d * xd - b * yd) / det, (-c * xd + a * yd) / det)


def tcp_to_display(tcp, global_calib):
    """Live TCP pose (robot frame, m) → sensor-frame mm, aligned with DISPLAY_XY.

    Subtract REFERENCE_POSE and the global offset so a tip sitting exactly on a
    calibrated point lands on that hex's centre; per-point deviation then shows
    as a small offset of the star/white-dot from the hex."""
    gx, gy, _ = global_calib
    rdx = (tcp[0] - cp.REFERENCE_POSE[0]) * 1000.0 - gx
    rdy = (tcp[1] - cp.REFERENCE_POSE[1]) * 1000.0 - gy
    return cp._to_display(rdx, rdy)


# ── Shared state between robot worker and render loop ─────────────────────────
class State:
    def __init__(self):
        self.lock       = threading.Lock()
        self.target     = None          # current target point (1..19) or None
        self.pressing   = False
        self.status     = "starting"
        self.done       = False
        self.actual     = {}            # pt → (xmm, ymm) recorded press location
        self.connected  = False
        # Live manual trim (mm), on top of the loaded calibration — dragged via
        # the map (X/Y) or the TIP HEIGHT gauge (Z), or nudged with arrows/-=.
        self.extra_x    = 0.0
        self.extra_y    = 0.0
        self.extra_z    = 0.0

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def snapshot(self):
        with self.lock:
            return (self.target, self.pressing, self.status, self.done,
                    dict(self.actual), self.connected)

    def get_offset(self):
        with self.lock:
            return self.extra_x, self.extra_y, self.extra_z


# ── Robot worker thread ───────────────────────────────────────────────────────
def robot_worker(state, stop_evt, args, global_calib, offsets, points):
    """Drive the arm through `points`, pressing each. Reads live TCP via
    ur5_control.get_tcp() (fed by the 125 Hz force reader) — never touches the
    RTDE receive interface directly, so there is no cross-thread contention."""
    import ur5_control

    state.set(status="connecting to robot")
    rtde_c, _ = ur5_control.connect()
    if rtde_c is None:
        state.set(status="ROBOT CONNECT FAILED", done=True)
        return
    state.set(connected=True)

    def moveL(pose, vel):
        rtde_c.moveL(pose, vel, cp.ACCELERATION)

    try:
        state.set(status="moving to home")
        moveL(cp.home_pose(global_calib), cp.VELOCITY_TRAVEL)

        for pt in points:
            if stop_evt.is_set():
                break
            dx, dy = offsets.get(pt, (0.0, 0.0))
            lx, ly, lz = state.get_offset()   # live manual trim (drag/keys)

            def surface_pose(lx, ly, lz):
                return cp.build_pose(pt, global_calib, dx + lx, dy + ly, lz)

            def pressed_pose(lx, ly, lz):
                return cp.build_pose(pt, global_calib, dx + lx, dy + ly,
                                      lz - args.indent)

            state.set(target=pt, pressing=False,
                      status=f"→ P{pt} ({cp.UR5_TO_LABEL[pt]})")
            moveL(surface_pose(lx, ly, lz), cp.VELOCITY_TRAVEL)
            if stop_evt.is_set():
                break

            state.set(pressing=True, status=f"pressing P{pt} ({cp.UR5_TO_LABEL[pt]})")
            moveL(pressed_pose(lx, ly, lz), cp.VELOCITY_PRESS)

            # Record where the tip actually ended up (sensor-frame mm).
            time.sleep(0.05)
            tcp = ur5_control.get_tcp()
            with state.lock:
                state.actual[pt] = tcp_to_display(tcp, global_calib)

            # Dwell, honouring pause/stop — and re-issuing the press if the
            # live trim is dragged/nudged while sitting on this point, so the
            # slider/keys feel live even mid-press (matches friction_live).
            t_end = time.time() + args.dwell
            while time.time() < t_end and not stop_evt.is_set():
                while args.paused[0] and not stop_evt.is_set():
                    state.set(status=f"PAUSED at P{pt}")
                    time.sleep(0.1)
                    t_end = time.time() + args.dwell
                nx, ny, nz = state.get_offset()
                if (nx, ny, nz) != (lx, ly, lz):
                    lx, ly, lz = nx, ny, nz
                    moveL(pressed_pose(lx, ly, lz), cp.VELOCITY_PRESS)
                time.sleep(0.03)

            moveL(surface_pose(lx, ly, lz), cp.VELOCITY_PRESS)
            state.set(pressing=False)

        state.set(status="returning home", target=None)
        moveL(cp.home_pose(global_calib), cp.VELOCITY_TRAVEL)
    except Exception as e:
        state.set(status=f"robot error: {e}")
    finally:
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        state.set(done=True, status="done — close window to exit")


# ── Sensor ────────────────────────────────────────────────────────────────────
def demo_values(frame):
    t = frame * 0.04
    return [max(0.0, min(1.0, 0.5 * math.sin(t + i * 0.4)
                         * math.sin(t * 0.7 + i * 0.3) + 0.25))
            for i in range(19)]


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Live UR5 calibration window (pygame)")
    ap.add_argument("--tip", default=None,
                    help="Calibration profile: loads calib_<tip>.json (global) "
                         "and calib_points_<tip>.json (per-point offsets)")
    ap.add_argument("--no-global", action="store_true",
                    help="Ignore calib.json (zero global offset)")
    ap.add_argument("--points", nargs="+", type=int, default=None,
                    help="Subset of points to press (default: all 1..19)")
    ap.add_argument("--indent", type=float, default=cp.INDENT_MM,
                    help=f"Press depth in mm (default {cp.INDENT_MM})")
    ap.add_argument("--dwell", type=float, default=1.0,
                    help="Seconds held at the bottom of each press (default 1.0)")
    ap.add_argument("--no-robot", action="store_true",
                    help="Do not drive the robot (sensor-only view)")
    ap.add_argument("--no-sensor", action="store_true",
                    help="Do not read the sensor (demo colours)")
    args = ap.parse_args()
    args.paused = [False]   # mutable flag shared with the worker

    global_calib = (0.0, 0.0, 0.0) if args.no_global else cp.load_global_calib(args.tip)
    offsets, _   = cp.load_point_offsets(args.tip)
    points       = args.points if args.points else list(cp.SCAN_ORDER)
    bad = [p for p in points if p not in cp.POINTS]
    if bad:
        print(f"[live] Invalid point(s): {bad} (valid 1..19)")
        sys.exit(1)

    # ── Sensor ────────────────────────────────────────────────
    sensor_mod = None
    if not args.no_sensor:
        import sensor as sensor_mod
        print("[live] Starting sensor ...")
        sensor_mod.start()
        if not sensor_mod.wait_until_ready(timeout=40):
            print("[live] ⚠ Sensor not ready — falling back to demo colours")
            sensor_mod = None
        else:
            print("[live] Sensor ready!")

    # ── Confirm before moving the robot ───────────────────────
    state    = State()
    stop_evt = threading.Event()
    worker   = None
    if not args.no_robot:
        print(f"\n⚠  The robot WILL press {len(points)} point(s) by "
              f"{args.indent:.1f} mm (dwell {args.dwell:.1f}s). It TOUCHES the sensor.")
        try:
            input("   Press Enter to start (Ctrl-C to abort) ...")
        except (EOFError, KeyboardInterrupt):
            print("\n[live] Aborted.")
            sys.exit(0)
        worker = threading.Thread(
            target=robot_worker,
            args=(state, stop_evt, args, global_calib, offsets, points),
            daemon=True)
        worker.start()
    else:
        state.set(status="sensor-only (no robot)", target=None)

    # ── pygame ────────────────────────────────────────────────
    pygame.init()
    pygame.display.set_caption("KYWO — Live Calibration")
    screen = pygame.display.set_mode((W, H))
    clock  = pygame.time.Clock()
    try:
        font_lg = pygame.font.SysFont('DejaVuSans', 16, bold=True)
        font_md = pygame.font.SysFont('DejaVuSans', 13)
        font_sm = pygame.font.SysFont('DejaVuSans', 10)
    except Exception:
        font_lg = pygame.font.Font(None, 22)
        font_md = pygame.font.Font(None, 18)
        font_sm = pygame.font.Font(None, 14)

    # Live-TCP trail (screen px)
    trail       = []
    show_trail  = True
    show_labels = True
    frame_n     = 0
    held_since  = {}   # key → frame it went down, for hold-to-repeat nudging

    # ── TIP HEIGHT gauge (Z) — draggable slider, mirrors friction_live.
    gx, gy0, gy1 = 655, 60, 560
    Ztop, Zbot   = OFFSET_LIMIT_Z, -OFFSET_LIMIT_Z
    GAUGE_HIT    = pygame.Rect(gx - 20, gy0 - 12, 40, gy1 - gy0 + 24)

    def z_to_y(zmm):
        return gy0 + (Ztop - clamp(zmm, Zbot, Ztop)) / (Ztop - Zbot) * (gy1 - gy0)

    def y_to_z(ypix):
        frac = clamp((ypix - gy0) / (gy1 - gy0), 0.0, 1.0)
        return Ztop - frac * (Ztop - Zbot)

    def drag_z(ypix):
        state.set(extra_z=round(clamp(y_to_z(ypix), -OFFSET_LIMIT_Z, OFFSET_LIMIT_Z), 2))

    dragging_z = False

    # ── Hex map as an X/Y trim pad — drag anywhere on the map to nudge.
    # Kept clear of the gauge column (x > 630) and the title / colour bar.
    HEXMAP_HIT = pygame.Rect(40, 45, 585, 600)

    def drag_xy(pos):
        ox, oy = screen_to_robot(*pos)
        state.set(extra_x=round(clamp(ox, -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2),
                  extra_y=round(clamp(oy, -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))

    dragging_xy = False

    running = True
    while running:
        frame_n += 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 \
                    and GAUGE_HIT.collidepoint(event.pos):
                dragging_z = True
                drag_z(event.pos[1])
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 \
                    and HEXMAP_HIT.collidepoint(event.pos):
                dragging_xy = True
                drag_xy(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging_z = False
                dragging_xy = False
            elif event.type == pygame.MOUSEMOTION and dragging_z:
                drag_z(event.pos[1])
            elif event.type == pygame.MOUSEMOTION and dragging_xy:
                drag_xy(event.pos)
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_t:
                    show_trail = not show_trail
                elif event.key == pygame.K_l:
                    show_labels = not show_labels
                elif event.key == pygame.K_p:
                    args.paused[0] = not args.paused[0]
                elif event.key == pygame.K_c and sensor_mod is not None:
                    try:
                        sensor_mod.recalibrate()
                    except Exception:
                        pass
                elif event.key == pygame.K_n:
                    state.set(extra_x=0.0, extra_y=0.0, extra_z=0.0)
                elif event.key == pygame.K_LEFT:
                    state.set(extra_x=round(clamp(state.get_offset()[0] - 0.5,
                              -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))
                elif event.key == pygame.K_RIGHT:
                    state.set(extra_x=round(clamp(state.get_offset()[0] + 0.5,
                              -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))
                elif event.key == pygame.K_UP:
                    state.set(extra_y=round(clamp(state.get_offset()[1] + 0.5,
                              -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))
                elif event.key == pygame.K_DOWN:
                    state.set(extra_y=round(clamp(state.get_offset()[1] - 0.5,
                              -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))
                elif event.key in (pygame.K_EQUALS, pygame.K_KP_PLUS):
                    state.set(extra_z=round(clamp(state.get_offset()[2] + 0.2,
                              -OFFSET_LIMIT_Z, OFFSET_LIMIT_Z), 2))
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    state.set(extra_z=round(clamp(state.get_offset()[2] - 0.2,
                              -OFFSET_LIMIT_Z, OFFSET_LIMIT_Z), 2))

        # ── Hold-to-repeat for arrow/-= nudges ─────────────────
        pressed_keys = pygame.key.get_pressed()

        def held(key, delay=10, rate=3):
            if not pressed_keys[key]:
                held_since.pop(key, None)
                return False
            d = frame_n - held_since.setdefault(key, frame_n)
            return d >= delay and (d - delay) % rate == 0

        if held(pygame.K_LEFT):
            state.set(extra_x=round(clamp(state.get_offset()[0] - 0.3,
                      -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))
        if held(pygame.K_RIGHT):
            state.set(extra_x=round(clamp(state.get_offset()[0] + 0.3,
                      -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))
        if held(pygame.K_UP):
            state.set(extra_y=round(clamp(state.get_offset()[1] + 0.3,
                      -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))
        if held(pygame.K_DOWN):
            state.set(extra_y=round(clamp(state.get_offset()[1] - 0.3,
                      -OFFSET_LIMIT_XY, OFFSET_LIMIT_XY), 2))
        if held(pygame.K_EQUALS):
            state.set(extra_z=round(clamp(state.get_offset()[2] + 0.1,
                      -OFFSET_LIMIT_Z, OFFSET_LIMIT_Z), 2))
        if held(pygame.K_KP_PLUS):
            state.set(extra_z=round(clamp(state.get_offset()[2] + 0.1,
                      -OFFSET_LIMIT_Z, OFFSET_LIMIT_Z), 2))
        if held(pygame.K_MINUS):
            state.set(extra_z=round(clamp(state.get_offset()[2] - 0.1,
                      -OFFSET_LIMIT_Z, OFFSET_LIMIT_Z), 2))
        if held(pygame.K_KP_MINUS):
            state.set(extra_z=round(clamp(state.get_offset()[2] - 0.1,
                      -OFFSET_LIMIT_Z, OFFSET_LIMIT_Z), 2))

        # ── Gather live data ──────────────────────────────────
        if sensor_mod is not None:
            values = sensor_mod.get_values()
        else:
            values = demo_values(frame_n)

        target, pressing, status, done, actual, connected = state.snapshot()
        extra_x, extra_y, extra_z = state.get_offset()

        tcp_disp = None
        if not args.no_robot and connected:
            try:
                import ur5_control
                tcp = ur5_control.get_tcp()
                tcp_disp = tcp_to_display(tcp, global_calib)
            except Exception:
                tcp_disp = None

        # Cell that fired most (for the orange ring)
        fired_idx = max(range(19), key=lambda i: values[i]) if values else 0
        fired_val = values[fired_idx] if values else 0.0

        # ── Draw ──────────────────────────────────────────────
        screen.fill(BG)

        # Left panel: hex map
        pygame.draw.rect(screen, PANEL, pygame.Rect(10, 10, 700, 700),
                         border_radius=10)
        if dragging_xy:
            pygame.draw.rect(screen, CYAN, pygame.Rect(10, 10, 700, 700),
                             border_radius=10, width=2)
        blit(screen, "Live calibration map", font_lg, TEXT, 30, 22)
        blit(screen, "drag ↔ trim X/Y", font_sm,
             CYAN if dragging_xy else MUTED, 700, 26, 'right')

        # Hexagons (DISPLAY_XY order == visualizer_2d.POINTS_MM)
        for pt in cp.SCAN_ORDER:
            xmm, ymm = cp.DISPLAY_XY[pt]
            cx, cy   = disp_to_screen(xmm, ymm)
            idx      = cp.UR5_TO_IDX[pt]
            v        = float(values[idx]) if idx < len(values) else 0.0
            col      = lerp_color(v)
            pts      = hex_pts(cx, cy, HEX_R)
            pygame.draw.polygon(screen, col, pts)
            pygame.draw.polygon(
                screen,
                (60, 60, 80) if v < 0.1 else tuple(min(255, c + 40) for c in col),
                pts, 1)

            tc = WHITE if v > 0.4 else TEXT
            if show_labels:
                blit(screen, cp.UR5_TO_LABEL[pt], font_sm, tc, cx, cy - 14, 'center')
                blit(screen, f"P{pt}", font_sm,
                     (180, 180, 180) if v > 0.4 else MUTED, cx, cy, 'center')
            if v > 0.02:
                blit(screen, f"{v:.2f}", font_sm, tc, cx, cy + 12, 'center')

        # GREEN ring — target point
        if target is not None:
            cx, cy = disp_to_screen(*cp.DISPLAY_XY[target])
            pygame.draw.circle(screen, GREEN, (int(cx), int(cy)), HEX_R + 6, 3)

        # ORANGE ring — cell that fired most
        if fired_val > 0.05:
            fpt = cp.SCAN_ORDER[fired_idx]
            cx, cy = disp_to_screen(*cp.DISPLAY_XY[fpt])
            pygame.draw.circle(screen, ORANGE, (int(cx), int(cy)), HEX_R + 12, 2)

        # WHITE dots — recorded actual press locations
        for pt, (xmm, ymm) in actual.items():
            cx, cy = disp_to_screen(xmm, ymm)
            pygame.draw.circle(screen, WHITE, (int(cx), int(cy)), 5)
            pygame.draw.circle(screen, (40, 40, 40), (int(cx), int(cy)), 5, 1)

        # CYAN star — live robot tip, with trail
        if tcp_disp is not None:
            sx, sy = disp_to_screen(*tcp_disp)
            trail.append((sx, sy))
            if len(trail) > 120:
                trail = trail[-120:]
            if show_trail and len(trail) > 1:
                pygame.draw.lines(screen, (40, 120, 130), False,
                                  [(int(x), int(y)) for x, y in trail], 2)
            spts = star_pts(sx, sy, 13)
            pygame.draw.polygon(screen, CYAN, spts)
            pygame.draw.polygon(screen, WHITE, spts, 1)

        # AMBER marker — current live X/Y trim (drag the map / arrow keys)
        if extra_x != 0.0 or extra_y != 0.0:
            mx, my = robot_to_screen(extra_x, extra_y)
            pygame.draw.line(screen, AMBER, (mx - 8, my), (mx + 8, my), 2)
            pygame.draw.line(screen, AMBER, (mx, my - 8), (mx, my + 8), 2)
            pygame.draw.circle(screen, AMBER, (int(mx), int(my)), 10, 1)

        # ── TIP HEIGHT gauge (Z) — draggable slider ────────────
        pygame.draw.rect(screen, CARD, (gx - 9, gy0 - 6, 18, gy1 - gy0 + 12),
                         border_radius=6)
        if dragging_z:
            pygame.draw.rect(screen, CYAN, (gx - 9, gy0 - 6, 18, gy1 - gy0 + 12),
                             border_radius=6, width=2)
        blit(screen, "TIP", font_sm, MUTED, gx, gy0 - 20, 'center')
        blit(screen, "TRIM", font_sm, MUTED, gx, gy0 - 9, 'center')
        for zt in (-5, 0, 5):
            yt = z_to_y(zt)
            pygame.draw.line(screen, (60, 60, 80) if zt else GREEN,
                             (gx - 6, yt), (gx + 6, yt), 1)
            blit(screen, f"{zt:+d}" if zt else "0", font_sm, MUTED if zt else GREEN,
                 gx + 14, yt - 6)
        yh = z_to_y(extra_z)
        hcol = CYAN if extra_z >= 0 else RED
        pygame.draw.polygon(screen, hcol, [(gx - 13, yh), (gx - 23, yh - 6), (gx - 23, yh + 6)])
        pygame.draw.circle(screen, hcol, (gx, int(yh)), 5)
        blit(screen, f"{extra_z:+.1f}mm", font_sm, hcol, gx - 66, yh - 6)
        blit(screen, "drag ↕", font_sm, CYAN if dragging_z else MUTED, gx, gy1 + 8, 'center')

        # Colour scale
        bx, by, bw, bh = 40, 676, 640, 10
        for px in range(bw):
            pygame.draw.rect(screen, lerp_color(px / bw), (bx + px, by, 1, bh))
        pygame.draw.rect(screen, MUTED, (bx, by, bw, bh), 1)
        blit(screen, "0.0", font_sm, MUTED, bx, by + 12)
        blit(screen, "1.0", font_sm, MUTED, bx + bw - 18, by + 12)

        # ── Right panel ───────────────────────────────────────
        pygame.draw.rect(screen, PANEL, pygame.Rect(718, 10, 272, 700),
                         border_radius=10)

        if args.no_robot:
            dot_col, rob = MUTED, "no robot"
        elif done:
            dot_col, rob = AMBER, "done"
        elif connected:
            dot_col, rob = GREEN, "live"
        else:
            dot_col, rob = AMBER, "connecting"
        pygame.draw.circle(screen, dot_col, (740, 32), 7)
        blit(screen, rob, font_sm, dot_col, 752, 25)
        blit(screen, "Status", font_lg, TEXT, 730, 46)

        tgt_lbl = f"P{target} ({cp.UR5_TO_LABEL[target]})" if target else "—"
        fired_lbl = (f"{cp.UR5_TO_LABEL[cp.SCAN_ORDER[fired_idx]]} "
                     f"({fired_val:.2f})") if fired_val > 0.05 else "—"
        match = (target is not None and fired_val > 0.05
                 and cp.SCAN_ORDER[fired_idx] == target)
        stats = [
            ("Target (green)", tgt_lbl, GREEN if target else MUTED),
            ("Pressing", "YES" if pressing else "no", RED if pressing else MUTED),
            ("Fired cell (orange)", fired_lbl,
             ORANGE if fired_val > 0.05 else MUTED),
            ("Mapping", "MATCH ✓" if match else ("mismatch" if target else "—"),
             GREEN if match else (RED if target and fired_val > 0.05 else MUTED)),
            ("Presses recorded", f"{len(actual)} / {len(points)}", TEXT),
            ("Live trim  (drag map/gauge, arrows, n=reset)",
             f"{extra_x:+.1f}, {extra_y:+.1f}, {extra_z:+.1f} mm",
             AMBER if (extra_x, extra_y, extra_z) != (0.0, 0.0, 0.0) else MUTED),
        ]
        sy = 72
        for label, val, col in stats:
            pygame.draw.rect(screen, CARD, pygame.Rect(730, sy, 248, 46),
                             border_radius=6)
            blit(screen, label, font_sm, MUTED, 742, sy + 6)
            blit(screen, val, font_md, col, 742, sy + 22)
            sy += 54

        pygame.draw.line(screen, (50, 50, 70), (730, sy), (978, sy), 1)
        sy += 8
        blit(screen, status, font_sm, AMBER, 730, sy)
        sy += 18

        # Legend
        blit(screen, "Legend", font_sm, MUTED, 730, sy); sy += 16
        legend = [
            (GREEN,  "○ target press point"),
            (ORANGE, "○ cell that fired"),
            (WHITE,  "● actual press location"),
            (CYAN,   "★ live robot tip"),
        ]
        for col, txt in legend:
            pygame.draw.circle(screen, col, (740, sy + 6), 5)
            blit(screen, txt, font_sm, TEXT, 754, sy); sy += 16

        sy = H - 112
        pygame.draw.line(screen, (50, 50, 70), (730, sy - 4), (978, sy - 4), 1)
        for hint in ["ESC/Q quit   C recal   T trail   L labels",
                     "P pause   N reset trim",
                     "drag map = X/Y trim   drag gauge = Z trim",
                     "arrows = X/Y trim   -/= = Z trim  (hold to repeat)"]:
            blit(screen, hint, font_sm, MUTED, 732, sy); sy += 16
        if args.paused[0]:
            blit(screen, "‖ PAUSED", font_md, AMBER, 732, sy)

        pygame.display.flip()
        clock.tick(FPS)

    # ── Shutdown: ask worker to stop, let it return home ──────
    stop_evt.set()
    if worker is not None and worker.is_alive():
        print("[live] Stopping — robot returning home ...")
        worker.join(timeout=30)
    pygame.quit()
    print("[live] Closed")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[live] Fatal: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            pygame.quit()
        except Exception:
            pass
