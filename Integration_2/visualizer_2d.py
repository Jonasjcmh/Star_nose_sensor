"""
visualizer_2d.py
2D sensor visualizer — works both standalone and from main.py subprocess.
Reads sensor data from shared file when running as subprocess.

────────────────────────────────────────────────────────────────────
NUMBERING: everything (POINTS_MM, RAW_CELLS, POINT_LABELS, and the
`values` array from sensor.py) shares ONE order: a1, a2, a3, b1, b2,
b3, b4, c1..c5, d2..d5, e3..e5. Index i in any of those lists means
the same physical point in all of them -- values[i] IS the reading at
POINT_LABELS[i]/POINTS_MM[i]/RAW_CELLS[i]. No permutation needed to
go between them.

GEOMETRY: points sit on a rhombic/hex lattice formed by two families
of diagonal lines on the physical sensor grid -- letters a..e running
down-and-to-the-right as they increase, numbers 1..5 running up-and-
-to-the-right as they increase (per hand-drawn reference grid). Valid
label combos are a1-3, b1-4, c1-5, d2-5, e3-5, which traces out a
hexagon.

Hardware point numbers (1-19, from ur5_control) happen to be the same
sequence as this label order (P1=a1, P2=a2, ... P19=e5), so
UR5_TO_IDX is a simple identity mapping. Must stay in sync with the
identical _UR5_TO_IDX in sensor.py.
────────────────────────────────────────────────────────────────────
"""
import sys
import os
import math
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import pygame
except ImportError:
    os.system(f"{sys.executable} -m pip install pygame")
    import pygame

try:
    import sensor as _sensor_mod
    HAS_SENSOR = True
except ImportError:
    HAS_SENSOR = False

try:
    import ur5_control
    HAS_UR5 = True
except ImportError:
    HAS_UR5 = False

N = 19
# POSITION order: index i corresponds to hex label POINT_LABELS[i]
# (a1,a2,a3,b1,...,e5) directly.
#
# Derived from letter axis u=(4,-7) [down-right, a->e] and number
# axis v=(4,7) [up-right, 1->5], i.e. pos = i_letter*u + n_number*v,
# recentered so c3 sits at (0,0). This reproduces the same hexagon
# footprint/spacing as before, just correctly oriented per the
# hand-drawn reference grid.
POINTS_MM = [
    (-16,   0), (-12,   7), ( -8,  14),           # a1, a2, a3
    (-12,  -7), ( -8,   0), ( -4,   7), (  0,  14), # b1, b2, b3, b4
    ( -8, -14), ( -4,  -7), (  0,   0), (  4,   7), (  8,  14), # c1..c5
    (  0, -14), (  4,  -7), (  8,   0), ( 12,   7), # d2, d3, d4, d5
    (  8, -14), ( 12,  -7), ( 16,   0),            # e3, e4, e5
]
# Raw MUCA board channel for each point, in the same order as
# POINTS_MM above. Kept for reference to the real hardware wiring;
# not used for display (the on-screen number is just the position
# index, i.e. S0 at a1, S1 at a2, ...). Must stay in sync with
# USED_CELLS in sensor.py.
RAW_CELLS = [0,1,2, 12,13,14,15, 24,25,26,27,28, 37,38,39,40, 50,51,52]

# Hardware point number (1-19) -> index in the 19-element sensor array.
# Identity mapping, since hardware point order is the same sequence as
# POINT_LABELS below (P1=a1, P2=a2, ... P19=e5). Must stay in sync
# with _UR5_TO_IDX in sensor.py.
UR5_TO_IDX = {n: n - 1 for n in range(1, 20)}

# Hex-grid labels (column letter + row number), in the same order as
# POINTS_MM/RAW_CELLS above -- index i -> label POINT_LABELS[i] ->
# value at values[i]. All three share one single order.
POINT_LABELS = [
    'a1', 'a2', 'a3',
    'b1', 'b2', 'b3', 'b4',
    'c1', 'c2', 'c3', 'c4', 'c5',
    'd2', 'd3', 'd4', 'd5',
    'e3', 'e4', 'e5',
]

# `values` (from sensor.py) is already in this same POINTS_MM/
# POINT_LABELS order (see sensor.py's USED_CELLS), so values[i] IS the
# reading for POINT_LABELS[i] directly -- no permutation needed to go
# from array index to label. SENSOR_TO_LABEL is just an alias kept for
# readability at the two call sites below.
SENSOR_TO_LABEL = POINT_LABELS

def _point_label(pt):
    """Convert a 1-based HARDWARE point number (from ur5_control) to its
    hex-grid label. Defers to ur5_control.point_label, which derives the label
    from the sensor cell the point physically presses (a1=P3, c3 center=P10),
    so the readout matches the press tool. Falls back to str(pt) otherwise."""
    if HAS_UR5:
        try:
            return ur5_control.point_label(int(pt))
        except (TypeError, ValueError):
            pass
    return str(pt)

W, H    = 920, 680
FPS     = 25
BG      = (18,  18,  24)
PANEL   = (26,  26,  36)
CARD    = (35,  35,  50)
TEXT    = (200, 200, 210)
MUTED   = (80,  80, 100)
WHITE   = (255, 255, 255)
GREEN   = ( 42, 200, 120)
RED     = (220,  60,  60)
AMBER   = (240, 160,  30)

def lerp_color(v):
    v = max(0.0, min(1.0, v))
    ramp = [
        ( 42, 181, 160),
        ( 51, 230, 102),
        (255, 230,  25),
        (255, 115,   0),
        (220,   0,   0),
    ]
    idx = v * (len(ramp)-1)
    lo  = int(idx)
    hi  = min(lo+1, len(ramp)-1)
    t   = idx - lo
    return tuple(int(ramp[lo][j] + t*(ramp[hi][j]-ramp[lo][j]))
                 for j in range(3))

def hex_pts(cx, cy, r):
    return [
        (cx + r*math.cos(math.radians(60*i+30)),
         cy + r*math.sin(math.radians(60*i+30)))
        for i in range(6)
    ]

def blit(surf, text, font, col, x, y, align='left'):
    s = font.render(str(text), True, col)
    if align == 'center': x -= s.get_width()//2
    elif align == 'right': x -= s.get_width()
    surf.blit(s, (int(x), int(y)))
    return s.get_width()

def demo_values(frame):
    t = frame * 0.04
    return [
        max(0.0, min(1.0,
            0.5*math.sin(t + i*0.4) *
            math.sin(t*0.7 + i*0.3) + 0.25))
        for i in range(N)
    ]

def get_sensor_values_safe():
    """Get sensor values — try direct memory first, then shared file"""
    if HAS_SENSOR:
        try:
            if _sensor_mod.is_ready():
                return _sensor_mod.get_values(), True
        except Exception:
            pass
        # Fallback to shared file
        try:
            data = _sensor_mod.read_shared()
            if data and data.get('ready'):
                return data['values'], True
        except Exception:
            pass
    return [0.0]*N, False

def get_ur5_state_safe():
    if HAS_UR5:
        try:
            return ur5_control.get_state()
        except Exception:
            pass
    return {}

def is_sensor_connected():
    if HAS_SENSOR:
        try:
            if _sensor_mod.is_connected():
                return True
            data = _sensor_mod.read_shared()
            if data:
                age = time.time() - data.get('timestamp', 0)
                return age < 1.5 and data.get('connected', False)
        except Exception:
            pass
    return False

def is_sensor_ready():
    if HAS_SENSOR:
        try:
            if _sensor_mod.is_ready():
                return True
            data = _sensor_mod.read_shared()
            if data and data.get('ready'):
                return True
        except Exception:
            pass
    return False

def main():
    try:
        pygame.init()
        pygame.display.set_caption("KYWO — Sensor 2D")
        screen = pygame.display.set_mode((W, H))
        clock  = pygame.time.Clock()
    except Exception as e:
        print(f"[viz] pygame init failed: {e}")
        return

    try:
        font_lg = pygame.font.SysFont('DejaVuSans', 16, bold=True)
        font_md = pygame.font.SysFont('DejaVuSans', 13)
        font_sm = pygame.font.SysFont('DejaVuSans', 10)
    except Exception:
        font_lg = pygame.font.Font(None, 22)
        font_md = pygame.font.Font(None, 18)
        font_sm = pygame.font.Font(None, 14)

    # ── Show loading screen ───────────────────────────────────
    screen.fill(BG)
    blit(screen, "KYWO Sensor Visualizer",
         font_lg, TEXT, W//2, H//2-50, 'center')
    blit(screen, "Connecting to sensor...",
         font_md, AMBER, W//2, H//2, 'center')
    blit(screen, "D=demo  ESC=quit",
         font_sm, MUTED, W//2, H//2+30, 'center')
    pygame.display.flip()

    # ── Determine mode ────────────────────────────────────────
    demo_mode   = False
    sensor_ready = False
    dots = 0
    t0   = time.time()

    # If sensor is already ready (shared file from main.py) skip wait
    if is_sensor_ready():
        sensor_ready = True
        print("[viz] Sensor ready via shared file!")
    elif HAS_SENSOR and not _sensor_mod.is_ready():
        # Start sensor ourselves (standalone mode)
        try:
            _sensor_mod.start()
        except Exception as e:
            print(f"[viz] Cannot start sensor: {e}")
            demo_mode = True

    # Wait for sensor with responsive loading screen
    if not demo_mode and not sensor_ready:
        while not is_sensor_ready():
            elapsed = time.time() - t0
            if elapsed > 20:
                print("[viz] Timeout — demo mode")
                demo_mode = True
                break

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit(); return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        pygame.quit(); return
                    if event.key == pygame.K_d:
                        demo_mode = True; break

            if demo_mode:
                break

            dots = (dots+1) % 4
            screen.fill(BG)
            blit(screen, "KYWO Sensor Visualizer",
                 font_lg, TEXT, W//2, H//2-50, 'center')
            blit(screen,
                 f"Calibrating{'.'*(dots+1)}  ({elapsed:.0f}s)",
                 font_md, AMBER, W//2, H//2, 'center')
            blit(screen, "D=demo  ESC=quit",
                 font_sm, MUTED, W//2, H//2+30, 'center')
            pygame.display.flip()
            clock.tick(4)

        if not demo_mode:
            sensor_ready = True
            print("[viz] Sensor ready!")

    if demo_mode:
        print("[viz] Demo mode")

    # ── Snapshot thread ───────────────────────────────────────
    snap      = {'values': [0.0]*N, 'ur5': {}}
    snap_lock = threading.Lock()
    frame_n   = [0]

    def update_snap():
        while True:
            try:
                if demo_mode:
                    vals = demo_values(frame_n[0])
                    ur5  = {}
                else:
                    vals, _ = get_sensor_values_safe()
                    ur5     = get_ur5_state_safe()

                with snap_lock:
                    snap['values'] = vals
                    snap['ur5']    = ur5
            except Exception as e:
                print(f"[viz] snap error: {e}")
            time.sleep(1/30)

    threading.Thread(target=update_snap, daemon=True).start()

    # ── Main loop ─────────────────────────────────────────────
    show_labels = True
    show_values = True
    HEX_R = 38
    HEX_CX, HEX_CY = 340, 350
    SX, SY = 14.5, 14.5

    running = True
    while running:
        frame_n[0] += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_l:
                    show_labels = not show_labels
                elif event.key == pygame.K_v:
                    show_values = not show_values
                elif event.key == pygame.K_c:
                    if HAS_SENSOR and not demo_mode:
                        try: _sensor_mod.recalibrate()
                        except: pass
                elif event.key == pygame.K_d:
                    demo_mode = not demo_mode

        try:
            with snap_lock:
                values    = list(snap['values'])
                ur5_state = dict(snap['ur5'])
        except Exception:
            values    = [0.0]*N
            ur5_state = {}

        screen.fill(BG)

        # ── Left panel: hex map ──────────────────────────────
        pygame.draw.rect(screen, PANEL,
                        pygame.Rect(10, 10, 648, 660),
                        border_radius=10)
        blit(screen, "Pressure map", font_lg, TEXT, 30, 22)

        # Connection indicator
        if demo_mode:
            dot_col, status = AMBER, "demo"
        elif is_sensor_connected():
            dot_col, status = GREEN, "live"
        elif is_sensor_ready():
            dot_col, status = AMBER, "buffered"
        else:
            dot_col, status = RED, "reconnecting"

        pygame.draw.circle(screen, dot_col, (618, 30), 7)
        blit(screen, status, font_sm, dot_col, 600, 38, 'right')

        # Hexagons
        for i, (xmm, ymm) in enumerate(POINTS_MM):
            try:
                cx  = HEX_CX + xmm * SX
                cy  = HEX_CY - ymm * SY
                # values is already in POINTS_MM order -- see the
                # POINTS_MM comment above -- so no permutation here.
                v   = float(values[i]) if i < len(values) else 0.0
                col = lerp_color(v)
                pts = hex_pts(cx, cy, HEX_R)

                pygame.draw.polygon(screen, col, pts)
                pygame.draw.polygon(screen,
                    (60,60,80) if v < 0.1 else
                    tuple(min(255, c+40) for c in col),
                    pts, 1)

                tc = WHITE if v > 0.4 else TEXT
                mc = (180,180,180) if v > 0.4 else MUTED

                if show_labels:
                    blit(screen, POINT_LABELS[i],
                         font_sm, tc, cx, cy-14, 'center')
                    blit(screen, f"S{RAW_CELLS[i]}",
                         font_sm, mc, cx, cy, 'center')

                if show_values and v > 0.02:
                    blit(screen, f"{v:.2f}",
                         font_sm, tc, cx, cy+12, 'center')

            except Exception:
                continue

        # Color scale
        bx, by, bw, bh = 50, 638, 548, 10
        for px in range(bw):
            pygame.draw.rect(screen, lerp_color(px/bw),
                            (bx+px, by, 1, bh))
        pygame.draw.rect(screen, MUTED, (bx, by, bw, bh), 1)
        blit(screen, "0.0", font_sm, MUTED, bx, by+13)
        blit(screen, "1.0", font_sm, MUTED, bx+bw-18, by+13)

        # ── Right panel ──────────────────────────────────────
        pygame.draw.rect(screen, PANEL,
                        pygame.Rect(666, 10, 244, 660),
                        border_radius=10)

        active = sum(1 for v in values if v > 0.05)
        maxv   = max(values) if values else 0.0
        pt     = ur5_state.get('point', '—')
        press  = ur5_state.get('pressing', False)

        stats = [
            ("Active", f"{active} / 19",
             GREEN if active > 0 else MUTED),
            ("Peak",   f"{maxv:.3f}",
             lerp_color(maxv) if maxv > 0.01 else MUTED),
            ("UR5",    _point_label(pt), TEXT),
            ("Robot",
             "PRESSING" if press else "idle",
             RED if press else MUTED),
        ]
        sy = 45
        for label, val, col in stats:
            pygame.draw.rect(screen, CARD,
                            pygame.Rect(678, sy, 224, 48),
                            border_radius=6)
            blit(screen, label, font_sm, MUTED, 690, sy+6)
            blit(screen, val,   font_md, col,   690, sy+22)
            sy += 56

        pygame.draw.line(screen, (50,50,70),
                        (678, sy), (900, sy), 1)
        sy += 8
        blit(screen, "Cell intensity", font_sm, MUTED, 682, sy)
        sy += 16

        # `values` is in POINTS_MM/POINT_LABELS order (see the comment
        # up top), so si here IS the same index used for the hex map.
        for si in range(N):
            try:
                v    = float(values[si]) if si < len(values) else 0.0
                col  = lerp_color(v)
                fill = int(v * 138)

                label = SENSOR_TO_LABEL[si]
                blit(screen, label, font_sm, MUTED, 678, sy+1)
                pygame.draw.rect(screen, (38,38,52),
                                (710, sy, 138, 9), border_radius=3)
                if fill > 1:
                    pygame.draw.rect(screen, col,
                                    (710, sy, fill, 9),
                                    border_radius=3)
                if v > 0.02:
                    blit(screen, f"{v:.2f}",
                         font_sm, TEXT, 852, sy+1)
                sy += 13
            except Exception:
                sy += 13

        sy = H - 72
        pygame.draw.line(screen, (50,50,70),
                        (678, sy-4), (900, sy-4), 1)
        for hint in ["L=labels","V=values",
                     "C=recalibrate","D=demo","ESC=quit"]:
            blit(screen, hint, font_sm, MUTED, 682, sy)
            sy += 13

        try:
            pygame.display.flip()
        except Exception as e:
            print(f"[viz] Display error: {e}")
            break

        clock.tick(FPS)

    try:
        pygame.quit()
    except Exception:
        pass
    print("[viz] Closed")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[viz] Fatal: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try: pygame.quit()
        except: pass
        #For the trajectory/friction dashboard later → yes, the web stack makes sense. If you move the robot onto ROS2, a Python backend publishing over websocket to an html/canvas or three.js 3D view is the right call for a richer, remote-viewable dashboard. We'll revisit that separately.
        