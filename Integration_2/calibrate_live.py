"""
calibrate_live.py
LIVE per-point calibration — the interactive terminal loop of
calibrate_points.py married to the pygame pressure map of visualizer_2d.py.

WHAT IT DOES
────────────
EVERYTHING happens in ONE pygame window — there is NO terminal juggling. You
type the SAME commands as calibrate_points.py directly INTO the window (a
command line is drawn at the bottom), so the live pressure map never disappears
behind the terminal. pygame owns the MAIN thread (SDL requirement); a BACKGROUND
worker drives the robot, consuming the commands you type.

  Commands (type into the window, then Enter):
        x+  x-  y+  y-   nudge by `step` mm      step 0.5   set step size
        press            press down + read sensor (peak captured)
        teach            record current TCP as the offset (freedrive)
        status           print offset + TCP + sensor snapshot (terminal)
        map              save a deviation-map PNG so far
        ok  (or Enter)   accept this point's offset → next point
        skip             leave this point unchanged → next point
        back             re-open the previous point
        save             save everything and finish
        quit             finish without saving this session
        trail / labels   toggle the movement trail / hex labels
        recal            recalibrate the sensor baseline
    (⚠ press / nudge MOVE THE REAL ROBOT and TOUCH THE SENSOR.)

  The hex map (same look as visualizer_2d.py) colours every cell by its live
  pressure (green→yellow→orange→red), so a press lights the touched cell live.

  Mouse (as well as the typed commands):
        • DRAG on the hex map      → set the current point's X/Y offset (drag to
                                     preview the AMBER target, release to move).
        • DRAG the DEPTH slider    → live press depth (right edge of the map).
    The Z / surface height is NOT adjusted here — it comes straight from the
    chosen calibration file (calib_<tip>.json), exactly like calibrate_points.py.

OVERLAYS (all live, on the pygame map)
──────────────────────────────────────
  • GREEN ring    — the point being calibrated (the TARGET), at the hex centre.
  • ORANGE ring   — the hex cell that fired most (max live reading).
  • CYAN circle   — the live robot INDENTOR position (sensor frame). A line from
                    the target hex centre to this circle shows how far off-centre
                    the tip is — watch it shrink as you nudge x±/y±. Turns into a
                    bold RED contact ring while pressing.
  • WHITE dot     — where the indentor ACTUALLY pressed at each point (recorded
                    at the bottom of the press), kept as history.
  • faint trail   — the indentor's path while the robot is moving.

Output: calib_points_<name>.json — identical format to calibrate_points.py
(per_point / points / scan_results), so it stays compatible with the rest of
the tooling (deviation maps, ur5_control loading, etc.).

Usage
─────
  python3 calibrate_live.py                       # choose base calib, all 19 pts
  python3 calibrate_live.py --tip new_hollow_dome_v2
  python3 calibrate_live.py --points 5 10 17 --indent 8
  python3 calibrate_live.py --no-robot            # sensor-only view (no driving)
  python3 calibrate_live.py --no-robot --no-sensor  # pure demo, no hardware

All commands are typed INTO the window. ESC clears the command line; the window
close button finishes (robot returns home).
              T toggle trail           L toggle labels
"""
import argparse
import collections
import math
import os
import queue
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import pygame
except ImportError:
    os.system(f"{sys.executable} -m pip install pygame")
    import pygame

import calibrate_points as cp   # single source of truth for geometry/press/save

# ── Colours / layout (mirrors visualizer_2d.py) ───────────────────────────────
W, H   = 1000, 908       # extra height for the in-window command console
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

# Draggable depth slider range (mm) and map-drag X/Y offset clamp (mm).
DEPTH_MIN, DEPTH_MAX = 0.0, 12.0
OFFSET_LIMIT         = 12.0


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


def screen_to_robot(sx, sy):
    """Inverse of disp_to_screen + the robot→sensor rotation: screen px →
    robot-frame mm (measured from REFERENCE_POSE + global calib). Used to turn a
    click/drag on the hex map into a per-point X/Y offset."""
    xd = (sx - HEX_CX) / SX
    yd = (HEX_CY - sy) / SY
    (a, b), (c, d) = cp._ROBOT_TO_SENSOR
    det = a * d - b * c
    return ((d * xd - b * yd) / det, (-c * xd + a * yd) / det)


def screen_to_offset(pos, pt):
    """Screen px → this point's robot-frame (dx, dy) offset from its nominal
    position (same convention as `teach` and offsets[pt])."""
    rx, ry = screen_to_robot(*pos)
    return rx - cp.POINTS[pt][0], ry - cp.POINTS[pt][1]


def tcp_to_display(tcp, global_calib):
    """Live TCP pose (robot frame, m) → sensor-frame mm, aligned with DISPLAY_XY.

    Subtract REFERENCE_POSE and the global offset so a tip sitting exactly on a
    calibrated point lands on that hex's centre; per-point deviation then shows
    as a small offset of the star/white-dot from the hex."""
    gx, gy, _ = global_calib
    rdx = (tcp[0] - cp.REFERENCE_POSE[0]) * 1000.0 - gx
    rdy = (tcp[1] - cp.REFERENCE_POSE[1]) * 1000.0 - gy
    return cp._to_display(rdx, rdy)


# ── Shared state between the terminal loop and the render thread ──────────────
class State:
    def __init__(self):
        self.lock       = threading.Lock()
        self.target     = None          # current target point (1..19) or None
        self.offset     = (0.0, 0.0)    # current per-point (dx, dy) mm
        self.step       = 0.5           # nudge step (mm)
        self.pressing   = False
        self.status     = "starting"
        self.done       = False
        self.connected  = False
        self.actual     = {}            # pt → (xmm, ymm) recorded press location
        self.peak       = None          # frozen peak vals (19) from last press
        self.peak_pt    = None          # which point the frozen peak belongs to
        self.depth      = cp.INDENT_MM  # live press depth (mm) — depth slider
        self.want_map   = False         # show deviation map in main thread on exit
        self.log        = collections.deque(maxlen=200)  # console scrollback

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def push_log(self, msg):
        with self.lock:
            self.log.append(msg)

    def record_actual(self, pt, xy):
        with self.lock:
            self.actual[pt] = xy

    def snapshot(self):
        with self.lock:
            return dict(
                target=self.target, offset=self.offset, step=self.step,
                pressing=self.pressing, status=self.status, done=self.done,
                connected=self.connected, actual=dict(self.actual),
                peak=(list(self.peak) if self.peak else None),
                peak_pt=self.peak_pt, depth=self.depth, want_map=self.want_map,
                log=list(self.log))


# ── Thread-safe TCP reader shim (so we never poke rtde_r from two threads) ────
class _TcpReader:
    """Stands in for rtde_receive in cp.print_status — reads the 125 Hz cache."""
    def getActualTCPPose(self):
        import ur5_control
        return ur5_control.get_tcp()


# ── Sensor demo colours (when --no-sensor) ────────────────────────────────────
def demo_values(frame):
    t = frame * 0.04
    return [max(0.0, min(1.0, 0.5 * math.sin(t + i * 0.4)
                         * math.sin(t * 0.7 + i * 0.3) + 0.25))
            for i in range(19)]


# ── Render loop: live pygame pressure map + in-window command line (MAIN thread)
def render_loop(state, stop_evt, args, sensor_mod, global_calib, points, cmd_q):
    pygame.init()
    pygame.display.set_caption("KYWO — Live Calibration")
    screen = pygame.display.set_mode((W, H))
    clock  = pygame.time.Clock()
    try:
        pygame.key.start_text_input()
    except Exception:
        pass
    try:
        font_lg = pygame.font.SysFont('DejaVuSans', 16, bold=True)
        font_md = pygame.font.SysFont('DejaVuSans', 13)
        font_sm = pygame.font.SysFont('DejaVuSans', 10)
    except Exception:
        font_lg = pygame.font.Font(None, 22)
        font_md = pygame.font.Font(None, 18)
        font_sm = pygame.font.Font(None, 14)

    trail       = []
    show_trail  = True
    show_labels = True
    frame_n     = 0
    cmd_buf     = ""          # what the user is currently typing

    # Command console geometry (full-width panel across the bottom)
    CON_X, CON_Y = 10, 720
    CON_W, CON_H = 980, H - 720 - 10
    CON_LINES    = 6          # output lines visible in the console
    INP_H        = 32         # input row height

    def submit(text):
        """Handle a submitted command line: render-only ones locally, the rest
        go to the worker thread via cmd_q."""
        nonlocal show_trail, show_labels
        c = text.strip().lower()
        low = c
        if low in ("trail",):
            show_trail = not show_trail
            return
        if low in ("labels", "label"):
            show_labels = not show_labels
            return
        if low in ("recal", "recalibrate", "c"):
            if sensor_mod is not None:
                try:
                    sensor_mod.recalibrate()
                    state.push_log("sensor recalibrated")
                except Exception:
                    pass
            return
        # Everything else drives the robot — hand to the worker.
        state.push_log(f"> {c}" if c else "> (ok)")
        cmd_q.put(c)

    # ── Draggable DEPTH slider (right edge of the map panel) ──────────────────
    # Sets cp.INDENT_MM live, so the next press goes to the new depth. Does NOT
    # move the robot by itself. Z stays whatever the loaded calib file gives.
    gx, gy0, gy1 = 655, 70, 560
    GAUGE_HIT    = pygame.Rect(gx - 22, gy0 - 14, 44, gy1 - gy0 + 28)

    def depth_to_y(d):
        f = (DEPTH_MAX - clamp(d, DEPTH_MIN, DEPTH_MAX)) / (DEPTH_MAX - DEPTH_MIN)
        return gy0 + f * (gy1 - gy0)

    def y_to_depth(ypix):
        f = clamp((ypix - gy0) / (gy1 - gy0), 0.0, 1.0)
        return DEPTH_MAX - f * (DEPTH_MAX - DEPTH_MIN)

    def drag_depth(ypix):
        d = round(y_to_depth(ypix), 1)
        cp.INDENT_MM = d            # do_press reads this module global
        state.set(depth=d)

    dragging_depth = False

    # ── Drag on the hex MAP to set the current point's X/Y offset ─────────────
    # Drag to preview (AMBER marker); on release, one 'setxy' goes to the worker
    # which moves the arm there. Only active while a point is being calibrated.
    HEXMAP_HIT   = pygame.Rect(20, 40, 620, 620)
    dragging_xy  = False
    pending_xy   = None            # (dx, dy) preview while dragging

    while not stop_evt.is_set():
        frame_n += 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                cmd_q.put("quit")     # let the worker home the arm, then exit
                stop_evt.set()
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if GAUGE_HIT.collidepoint(event.pos):
                    dragging_depth = True
                    drag_depth(event.pos[1])
                elif HEXMAP_HIT.collidepoint(event.pos) \
                        and state.snapshot()['target'] is not None:
                    dragging_xy = True
                    pending_xy  = screen_to_offset(
                        event.pos, state.snapshot()['target'])
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging_depth = False
                if dragging_xy and pending_xy is not None:
                    dx, dy = pending_xy
                    dx = clamp(dx, -OFFSET_LIMIT, OFFSET_LIMIT)
                    dy = clamp(dy, -OFFSET_LIMIT, OFFSET_LIMIT)
                    cmd_q.put(f"setxy {dx:.3f} {dy:.3f}")   # worker moves the arm
                    state.push_log(f"> map → dX={dx:+.2f} dY={dy:+.2f}")
                dragging_xy = False
                pending_xy  = None
            elif event.type == pygame.MOUSEMOTION:
                if dragging_depth:
                    drag_depth(event.pos[1])
                elif dragging_xy and state.snapshot()['target'] is not None:
                    pending_xy = screen_to_offset(
                        event.pos, state.snapshot()['target'])
            elif event.type == pygame.TEXTINPUT:
                cmd_buf += event.text
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    submit(cmd_buf)
                    cmd_buf = ""
                elif event.key == pygame.K_BACKSPACE:
                    cmd_buf = cmd_buf[:-1]
                elif event.key == pygame.K_ESCAPE:
                    cmd_buf = ""      # clear the line (does NOT quit)

        # ── Gather live data ──────────────────────────────────
        if sensor_mod is not None:
            values = sensor_mod.get_values()
        else:
            values = demo_values(frame_n)

        s = state.snapshot()
        target   = s['target']
        offset   = s['offset']
        pressing = s['pressing']

        tcp_disp = None
        if not args.no_robot and s['connected']:
            try:
                import ur5_control
                tcp_disp = tcp_to_display(ur5_control.get_tcp(), global_calib)
            except Exception:
                tcp_disp = None

        fired_idx = max(range(19), key=lambda i: values[i]) if values else 0
        fired_val = values[fired_idx] if values else 0.0

        # ── Draw ──────────────────────────────────────────────
        screen.fill(BG)

        pygame.draw.rect(screen, PANEL, pygame.Rect(10, 10, 700, 700),
                         border_radius=10)
        blit(screen, "Live calibration map", font_lg, TEXT, 30, 22)

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
        tgt_screen = None
        if target is not None:
            tgt_screen = disp_to_screen(*cp.DISPLAY_XY[target])
            pygame.draw.circle(screen, GREEN,
                               (int(tgt_screen[0]), int(tgt_screen[1])), HEX_R + 6, 3)
            # A cross-hair marks the exact hex CENTRE we want the indentor to hit.
            cx, cy = tgt_screen
            pygame.draw.line(screen, GREEN, (cx - 7, cy), (cx + 7, cy), 1)
            pygame.draw.line(screen, GREEN, (cx, cy - 7), (cx, cy + 7), 1)

        # AMBER preview marker while dragging the map to set X/Y (not sent yet)
        if dragging_xy and pending_xy is not None and target is not None:
            pdx, pdy = pending_xy
            mx, my = disp_to_screen(
                *cp._to_display(cp.POINTS[target][0] + pdx,
                                cp.POINTS[target][1] + pdy))
            pygame.draw.line(screen, AMBER,
                             (int(tgt_screen[0]), int(tgt_screen[1])),
                             (int(mx), int(my)), 1)
            pygame.draw.circle(screen, AMBER, (int(mx), int(my)), 7, 2)
            pygame.draw.line(screen, AMBER, (mx - 9, my), (mx + 9, my), 1)
            pygame.draw.line(screen, AMBER, (mx, my - 9), (mx, my + 9), 1)
            blit(screen, f"dX={pdx:+.1f} dY={pdy:+.1f}", font_sm, AMBER,
                 mx + 10, my - 6)

        # ORANGE ring — cell that fired most
        if fired_val > 0.05:
            fpt = cp.SCAN_ORDER[fired_idx]
            cx, cy = disp_to_screen(*cp.DISPLAY_XY[fpt])
            pygame.draw.circle(screen, ORANGE, (int(cx), int(cy)), HEX_R + 12, 2)

        # WHITE dots — recorded actual press locations (history)
        for pt, (xmm, ymm) in s['actual'].items():
            cx, cy = disp_to_screen(xmm, ymm)
            pygame.draw.circle(screen, WHITE, (int(cx), int(cy)), 4)
            pygame.draw.circle(screen, (40, 40, 40), (int(cx), int(cy)), 4, 1)

        # Live INDENTOR — small circle at the tip; trail while moving; a line to
        # the target centre shows how far off-centre it is (watch it shrink on
        # x+/x-/y+/y- nudges). Bold red contact ring while pressing.
        if tcp_disp is not None:
            tx, ty = disp_to_screen(*tcp_disp)
            trail.append((tx, ty))
            if len(trail) > 140:
                trail = trail[-140:]
            if show_trail and len(trail) > 1:
                pygame.draw.lines(screen, (40, 120, 130), False,
                                  [(int(x), int(y)) for x, y in trail], 2)

            if tgt_screen is not None:
                # deviation line + magnitude in mm
                pygame.draw.line(screen, (120, 200, 210),
                                 (int(tgt_screen[0]), int(tgt_screen[1])),
                                 (int(tx), int(ty)), 1)
                dxm = tcp_disp[0] - cp.DISPLAY_XY[target][0]
                dym = tcp_disp[1] - cp.DISPLAY_XY[target][1]
                mag = math.hypot(dxm, dym)
                blit(screen, f"{mag:.1f}mm", font_sm, (150, 210, 220),
                     (tx + tgt_screen[0]) / 2, (ty + tgt_screen[1]) / 2 - 12,
                     'center')

            if pressing:
                pygame.draw.circle(screen, RED, (int(tx), int(ty)), 11, 3)
                pygame.draw.circle(screen, CYAN, (int(tx), int(ty)), 6)
            else:
                pygame.draw.circle(screen, CYAN, (int(tx), int(ty)), 6)
                pygame.draw.circle(screen, WHITE, (int(tx), int(ty)), 6, 1)

        # Colour scale
        bx, by, bw, bh = 40, 676, 640, 10
        for px in range(bw):
            pygame.draw.rect(screen, lerp_color(px / bw), (bx + px, by, 1, bh))
        pygame.draw.rect(screen, MUTED, (bx, by, bw, bh), 1)
        blit(screen, "0.0", font_sm, MUTED, bx, by + 12)
        blit(screen, "1.0", font_sm, MUTED, bx + bw - 18, by + 12)

        # ── DEPTH slider (draggable) ──────────────────────────
        cur_depth = s['depth']
        blit(screen, "DEPTH", font_sm, TEXT, gx, gy0 - 20, 'center')
        pygame.draw.line(screen, MUTED, (gx, gy0), (gx, gy1), 2)
        for dd in range(int(DEPTH_MIN), int(DEPTH_MAX) + 1, 2):
            ty = depth_to_y(dd)
            pygame.draw.line(screen, (60, 60, 80), (gx - 5, ty), (gx + 5, ty), 1)
            blit(screen, f"{dd}", font_sm, MUTED, gx + 10, ty - 6)
        ky = depth_to_y(cur_depth)
        knob_col = CYAN if dragging_depth else WHITE
        pygame.draw.circle(screen, knob_col, (gx, int(ky)), 8)
        pygame.draw.circle(screen, (30, 30, 40), (gx, int(ky)), 8, 1)
        blit(screen, f"{cur_depth:.1f} mm", font_md, knob_col,
             gx, gy1 + 8, 'center')

        # ── Right panel ───────────────────────────────────────
        pygame.draw.rect(screen, PANEL, pygame.Rect(718, 10, 272, 700),
                         border_radius=10)

        if args.no_robot:
            dot_col, rob = MUTED, "no robot"
        elif s['done']:
            dot_col, rob = AMBER, "done"
        elif s['connected']:
            dot_col, rob = GREEN, "live"
        else:
            dot_col, rob = AMBER, "connecting"
        pygame.draw.circle(screen, dot_col, (740, 32), 7)
        blit(screen, rob, font_sm, dot_col, 752, 25)
        blit(screen, "Calibration", font_lg, TEXT, 730, 46)

        dx, dy   = offset
        tgt_lbl  = f"P{target} ({cp.UR5_TO_LABEL[target]})" if target else "—"
        fired_lbl = (f"{cp.UR5_TO_LABEL[cp.SCAN_ORDER[fired_idx]]} "
                     f"({fired_val:.2f})") if fired_val > 0.05 else "—"
        match = (target is not None and fired_val > 0.05
                 and cp.SCAN_ORDER[fired_idx] == target)

        peak_lbl = "—"
        if s['peak'] is not None and s['peak_pt'] is not None:
            pv = s['peak'][cp.UR5_TO_IDX[s['peak_pt']]]
            peak_lbl = f"P{s['peak_pt']}: {pv:.2f}"

        stats = [
            ("Target (green)", tgt_lbl, GREEN if target else MUTED),
            ("Offset dX / dY", f"{dx:+.2f} / {dy:+.2f} mm", TEXT),
            ("Step size", f"{s['step']:.2f} mm", TEXT),
            ("Press depth (slider)", f"{s['depth']:.1f} mm", CYAN),
            ("Pressing", "YES" if pressing else "no", RED if pressing else MUTED),
            ("Fired cell (orange)", fired_lbl,
             ORANGE if fired_val > 0.05 else MUTED),
            ("Mapping", "MATCH ✓" if match else ("mismatch" if target else "—"),
             GREEN if match else (RED if target and fired_val > 0.05 else MUTED)),
            ("Last press peak", peak_lbl, TEXT),
            ("Presses recorded", f"{len(s['actual'])} / {len(points)}", TEXT),
        ]
        sy = 72
        for label, val, col in stats:
            pygame.draw.rect(screen, CARD, pygame.Rect(730, sy, 248, 38),
                             border_radius=6)
            blit(screen, label, font_sm, MUTED, 742, sy + 4)
            blit(screen, val, font_md, col, 742, sy + 19)
            sy += 43

        pygame.draw.line(screen, (50, 50, 70), (730, sy), (978, sy), 1)
        sy += 6
        blit(screen, s['status'], font_sm, AMBER, 730, sy)
        sy += 20

        # Legend (inside the right panel)
        blit(screen, "Legend", font_sm, MUTED, 730, sy); sy += 16
        for col, txt in [(GREEN,  "+ target centre"),
                         (ORANGE, "○ cell fired"),
                         (CYAN,   "● indentor"),
                         (WHITE,  "● where it pressed"),
                         (RED,    "○ press contact")]:
            pygame.draw.circle(screen, col, (740, sy + 6), 5)
            blit(screen, txt, font_sm, TEXT, 754, sy); sy += 16

        # ── Command CONSOLE (full-width panel across the bottom) ──────
        pygame.draw.rect(screen, PANEL, pygame.Rect(CON_X, CON_Y, CON_W, CON_H),
                         border_radius=10)
        blit(screen, "Command console", font_lg, TEXT, CON_X + 14, CON_Y + 8)
        blit(screen, "x+ x- y+ y-  ·  press  ·  step N  ·  teach  ·  ok/Enter  ·  "
                     "skip  ·  back  ·  save  ·  quit  ·  drag map = X/Y  ·  "
                     "drag right slider = depth  ·  ESC clears",
             font_sm, MUTED, CON_X + 150, CON_Y + 13)

        # Output scrollback — last CON_LINES messages (commands + responses).
        out_y = CON_Y + 32
        log_lines = s['log'][-CON_LINES:]
        for msg in log_lines:
            col = GREEN if msg.startswith('>') else TEXT
            blit(screen, msg[:118], font_sm, col, CON_X + 16, out_y)
            out_y += 15

        # Input row — the field you type into.
        inp_y = CON_Y + CON_H - INP_H - 6
        pygame.draw.rect(screen, CARD,
                         pygame.Rect(CON_X + 8, inp_y, CON_W - 16, INP_H),
                         border_radius=6)
        prompt = f"P{target} > " if target else "> "
        cursor = "_" if (frame_n // 20) % 2 == 0 else " "
        blit(screen, prompt, font_md, GREEN, CON_X + 20, inp_y + 8)
        pw = font_md.size(prompt)[0]
        blit(screen, cmd_buf + cursor, font_md, WHITE, CON_X + 20 + pw, inp_y + 8)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


# ── Interactive per-point loop (commands come from the in-window prompt) ──────
def interactive_point(pt, rtde_c, tcp_reader, global_calib, offsets,
                      scan_results, sensor_mod, state, cmd_q):
    """One point's adjustment loop. Commands arrive via cmd_q (typed into the
    pygame window). Returns True → next | 'back' → previous | None → save+finish
    | 'quit' → finish without saving."""
    dx, dy   = offsets.get(pt, (0.0, 0.0))
    step_mm  = state.snapshot()['step']
    exp_raw  = cp.UR5_TO_RAW[pt]

    def log(msg):
        print("  " + msg)
        state.push_log(msg)

    print(f"\n{'='*62}")
    print(f"  P{pt:02d} ({cp.UR5_TO_LABEL[pt]})  "
          f"nominal XY=({cp.POINTS[pt][0]:+.0f},{cp.POINTS[pt][1]:+.0f}) mm  "
          f"expected S{exp_raw}")
    print(f"{'='*62}")

    state.set(target=pt, offset=(dx, dy), step=step_mm, pressing=False,
              peak=None, peak_pt=None, status=f"→ P{pt} ({cp.UR5_TO_LABEL[pt]})")
    log(f"P{pt} ({cp.UR5_TO_LABEL[pt]}) — nudge x/y, press, ok")
    rtde_c.moveL(cp.build_pose(pt, global_calib, dx, dy, 0.0),
                 cp.VELOCITY_TRAVEL, cp.ACCELERATION)

    while True:
        cmd = cmd_q.get()            # blocks until the user submits a command
        cmd = (cmd or "").strip().lower()
        moved = False

        if cmd == "x+":
            dx += step_mm; moved = True
        elif cmd == "x-":
            dx -= step_mm; moved = True
        elif cmd == "y+":
            dy += step_mm; moved = True
        elif cmd == "y-":
            dy -= step_mm; moved = True
        elif cmd.startswith("step"):
            try:
                step_mm = float(cmd.split()[1])
                state.set(step=step_mm)
                log(f"step = {step_mm:.3f} mm")
            except (IndexError, ValueError):
                log("usage: step 0.5")

        elif cmd.startswith("setxy"):
            # Absolute (dx, dy) from a map drag in the window; moves the arm there.
            try:
                _, sxv, syv = cmd.split()
                dx = clamp(float(sxv), -OFFSET_LIMIT, OFFSET_LIMIT)
                dy = clamp(float(syv), -OFFSET_LIMIT, OFFSET_LIMIT)
                moved = True
            except (ValueError, IndexError):
                log("bad setxy")

        elif cmd == "press":
            offsets[pt] = (dx, dy)
            state.set(pressing=True, status=f"pressing P{pt}")
            result = cp.do_press(rtde_c, pt, global_calib, offsets, sensor_mod)
            state.set(pressing=False)
            if result:
                result["tcp"] = [round(v, 6) for v in tcp_reader.getActualTCPPose()]
                scan_results[str(pt)] = result
                state.set(peak=result["peak_vals"], peak_pt=pt,
                          status=f"P{pt} peak captured")
                state.record_actual(pt, tcp_to_display(
                    tcp_reader.getActualTCPPose(), global_calib))
                ok = "✓" if result.get("correct") else "✗ WRONG CELL"
                log(f"P{pt} press: S{result['expected_raw']}="
                    f"{result['expected_val']:.2f} {ok}")
            else:
                log(f"P{pt} pressed (no sensor)")
            rtde_c.moveL(cp.build_pose(pt, global_calib, dx, dy, 0.0),
                         cp.VELOCITY_PRESS, cp.ACCELERATION)

        elif cmd == "teach":
            tcp = tcp_reader.getActualTCPPose()
            nom_x = cp.REFERENCE_POSE[0] + (cp.POINTS[pt][0] + global_calib[0]) / 1000
            nom_y = cp.REFERENCE_POSE[1] + (cp.POINTS[pt][1] + global_calib[1]) / 1000
            dx = round((tcp[0] - nom_x) * 1000, 4)
            dy = round((tcp[1] - nom_y) * 1000, 4)
            offsets[pt] = (dx, dy)
            state.set(offset=(dx, dy))
            log(f"taught dX={dx:+.2f} dY={dy:+.2f} mm")

        elif cmd == "status":
            cp.print_status(pt, dx, dy, step_mm, tcp_reader, sensor_mod, global_calib)
            log(f"P{pt} dX={dx:+.2f} dY={dy:+.2f} (see terminal)")

        elif cmd == "map":
            # matplotlib GUI isn't thread-safe here — save a PNG snapshot.
            offsets[pt] = (dx, dy)
            path = os.path.join(cp.CALIB_DIR, "deviation_map_live_preview.png")
            try:
                cp.show_deviation_map(offsets, scan_results, save_path=path)
                log("saved deviation_map_live_preview.png")
            except Exception as e:
                log(f"map failed: {e}")

        elif cmd in ("ok", ""):
            offsets[pt] = (dx, dy)
            log(f"P{pt} accepted dX={dx:+.2f} dY={dy:+.2f}")
            return True

        elif cmd == "skip":
            log(f"P{pt} skipped")
            return True

        elif cmd == "back":
            offsets[pt] = (dx, dy)
            log("← back one point")
            return "back"

        elif cmd == "save":
            offsets[pt] = (dx, dy)
            return None

        elif cmd == "quit":
            log("quitting (no save)")
            return "quit"

        else:
            log(f"unknown: '{cmd}'")

        if moved:
            offsets[pt] = (dx, dy)
            state.set(offset=(dx, dy))
            rtde_c.moveL(cp.build_pose(pt, global_calib, dx, dy, 0.0),
                         cp.VELOCITY_TRAVEL, cp.ACCELERATION)
            log(f"dX={dx:+.2f} dY={dy:+.2f} mm")


# ── Robot-driving worker (runs in a BACKGROUND thread) ────────────────────────
def calibration_worker(state, stop_evt, rtde_c, tcp_reader, global_calib,
                       offsets, scan_results, sensor_mod, points, default_out, cmd_q):
    """Drive the arm, run the in-window command loop, save, return home.

    Runs off the main thread so pygame can own the main thread (SDL requires
    its window to be created/pumped there — otherwise it vanishes the moment
    the process focus leaves the window). Commands arrive via cmd_q (typed into
    the window); TCP is read from the 125 Hz cache, so it never contends with
    the render thread for the RTDE receive interface."""
    def save_to(name):
        out = os.path.join(cp.CALIB_DIR, name)
        cp.save_results(global_calib, offsets, scan_results, out)
        state.push_log(f"saved {name}")

    saved = False
    try:
        state.set(status="moving to home")
        rtde_c.moveL(cp.home_pose(global_calib), cp.VELOCITY_TRAVEL, cp.ACCELERATION)

        # Baseline press for each point up front (like calibrate_points.py), so
        # the map is populated before you start nudging.
        i = 0
        while 0 <= i < len(points):
            pt = points[i]
            print(f"\n  → Baseline press at P{pt} ...")
            dx, dy = offsets.get(pt, (0.0, 0.0))
            state.set(target=pt, offset=(dx, dy),
                      status=f"baseline P{pt}", pressing=True)
            rtde_c.moveL(cp.build_pose(pt, global_calib, dx, dy, 0.0),
                         cp.VELOCITY_TRAVEL, cp.ACCELERATION)
            result = cp.do_press(rtde_c, pt, global_calib, offsets, sensor_mod)
            state.set(pressing=False)
            if result:
                result["tcp"] = [round(v, 6) for v in tcp_reader.getActualTCPPose()]
                scan_results[str(pt)] = result
                state.set(peak=result["peak_vals"], peak_pt=pt)
                state.record_actual(pt, tcp_to_display(
                    tcp_reader.getActualTCPPose(), global_calib))
            rtde_c.moveL(cp.build_pose(pt, global_calib, dx, dy, 0.0),
                         cp.VELOCITY_PRESS, cp.ACCELERATION)

            res = interactive_point(pt, rtde_c, tcp_reader, global_calib,
                                    offsets, scan_results, sensor_mod, state, cmd_q)
            if res == "back":
                i = max(0, i - 1)
            elif res is True:
                i += 1
            elif res == "quit":
                break
            else:                       # 'save' → auto-save + finish
                save_to(default_out)
                saved = True
                break

        if i >= len(points) and not saved:      # finished all points normally
            cp.print_summary(scan_results, offsets)
            save_to(default_out)
            state.set(want_map=True)             # main thread shows it on exit
            saved = True

        state.set(status="returning home", target=None)
        rtde_c.moveL(cp.home_pose(global_calib), cp.VELOCITY_TRAVEL, cp.ACCELERATION)
    except Exception as e:
        print(f"[live] Robot error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        state.set(done=True, status="done — close the window (ESC/Q) to exit")
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        stop_evt.set()   # tell the main-thread render loop to exit


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Live per-point UR5 calibration (pygame)")
    ap.add_argument("--tip", default=None,
                    help="Calibration profile: loads calib_<tip>.json (global) "
                         "and calib_points_<tip>.json (per-point offsets)")
    ap.add_argument("--no-global", action="store_true",
                    help="Ignore calib.json (zero global offset)")
    ap.add_argument("--points", nargs="+", type=int, default=None,
                    help="Subset of points to calibrate (default: all 1..19)")
    ap.add_argument("--indent", type=float, default=cp.INDENT_MM,
                    help=f"Press depth in mm (default {cp.INDENT_MM})")
    ap.add_argument("--no-robot", action="store_true",
                    help="Do not drive the robot (sensor-only pygame view)")
    ap.add_argument("--no-sensor", action="store_true",
                    help="Do not read the sensor (demo colours)")
    args = ap.parse_args()

    cp.INDENT_MM = args.indent   # cp.do_press presses by this depth

    # ── Choose starting calibration (global X/Y/Z base + per-point deviations) ─
    if args.no_global:
        global_calib, offsets, scan_results = (0.0, 0.0, 0.0), {}, {}
        default_out = "calib_points.json"
    elif args.tip:
        global_calib = cp.load_global_calib(args.tip)
        offsets, scan_results = cp.load_point_offsets(args.tip)
        default_out = f"calib_points_{args.tip}.json"
    else:
        global_calib, base_label, base_points_file = cp.choose_starting_calib()
        offsets, scan_results = cp.load_existing_deviations(base_points_file)
        default_out = (f"calib_points_{base_label}.json"
                       if base_label else "calib_points.json")

    points = args.points if args.points else list(cp.SCAN_ORDER)
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

    state    = State()
    stop_evt = threading.Event()
    cmd_q    = queue.Queue()        # in-window command line → worker thread
    state.push_log("console ready — type commands here, then Enter")

    # ── Robot ─────────────────────────────────────────────────
    rtde_c = None
    tcp_reader = _TcpReader()
    if not args.no_robot:
        print(f"\n⚠  The robot WILL move and press by {args.indent:.1f} mm — it "
              f"TOUCHES the sensor. Calibrating {len(points)} point(s).")
        try:
            input("   Press Enter to start (Ctrl-C to abort) ...")
        except (EOFError, KeyboardInterrupt):
            print("\n[live] Aborted.")
            sys.exit(0)
        import ur5_control
        print("[live] Connecting to robot ...")
        rtde_c, _ = ur5_control.connect()
        if rtde_c is None:
            print("[live] Robot connect FAILED — aborting.")
            sys.exit(1)
        state.set(connected=True)

    # ── Start the robot-driving worker (background thread) ────────
    # pygame stays on the MAIN thread; the command loop runs in the worker and
    # reads what you type into the window, so window + prompt live together.
    worker = None
    if not args.no_robot:
        worker = threading.Thread(
            target=calibration_worker,
            args=(state, stop_evt, rtde_c, tcp_reader, global_calib,
                  offsets, scan_results, sensor_mod, points, default_out, cmd_q),
            daemon=True)
        worker.start()
    else:
        state.set(status="sensor-only (no robot)")

    # ── Render loop (MAIN thread — keeps the SDL window alive) ──────
    try:
        render_loop(state, stop_evt, args, sensor_mod, global_calib, points, cmd_q)
    except KeyboardInterrupt:
        stop_evt.set()
        cmd_q.put("quit")   # unblock the worker so it can home + exit cleanly

    # Window closed (or worker finished). Let the worker return the arm home and
    # finish any save prompt still in flight before we exit.
    if worker is not None:
        worker.join()

    # Final deviation map — matplotlib is only safe on the main thread.
    if state.snapshot()['want_map'] and cp._HAS_MPL:
        try:
            cp.show_deviation_map(offsets, scan_results)
            cp.plt.ioff()
            cp.plt.show(block=True)
        except Exception:
            pass
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
