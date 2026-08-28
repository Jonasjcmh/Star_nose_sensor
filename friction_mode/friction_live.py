"""
friction_live.py
GAME-STYLE live driver for the friction-mode trajectories — one pygame window
with the live pressure map (like visualizer_2d / calibrate_live) plus an
interactive keyboard AND a typed command console.

WHAT IT DOES
────────────
One pygame window (MAIN thread) shows the live sensor hex map coloured by
pressure (green→yellow→orange→red) with the robot indentor and the selected
trajectory drawn on top. A BACKGROUND worker drives the UR5, reusing
ur5_friction._build_pose + trajectories.py.

You fly the robot like a game — LETTER keys + arrows only (no symbol keys):

  MOVE (live XY offset — shifts the whole path, in air AND mid-trajectory):
    ← → ↑ ↓    X / Y offset   (by `step`, default 1 mm)     n  centre offset
               or DRAG THE MOUSE anywhere on the pressure map — click sets
               the offset to that spot, drag to move it live

  DEPTH / FORCE / SPEED / SURFACE-Z (all live, hold to repeat):
    q / e      depth − / +   (in FORCE mode: target force − / +)
    - / =      surface-Z down / up  (nudge the calib touch height live) — or
               DRAG THE MOUSE on the TIP HEIGHT gauge (right side of the map)
    j / k      slide speed − / +
    m          toggle contact mode: DEPTH (fixed Z) <-> FORCE (FUTEK constant N)

  TRAJECTORY — press a letter to SELECT (previewed on the map); pressing a new
               letter while running switches the pattern LIVE:
    c circle   s spiral   h line_h   v line_v
    d diag ↗   f diag ↖   x cross    r raster   t star
    o flip direction LIVE      p pause / resume

  SCALE (circle/pattern radius, hold to repeat):  9 / 0
    (circle r = 12*scale mm, spiral r = 14*scale mm — point count scales too,
    so bigger patterns stay smooth curves), or console 'scale 1.5'

  ACTIONS:
    SPACE      run the selected trajectory — LOOPS until you stop
    g          go to hover home         BACKSPACE  STOP / abort (lift + home)
    Tab        toggle the CONSOLE — type values, e.g.
                   x+  x-  y+  y-   jog XY offset by step
                   z+  z-           move the indentor UP / DOWN
                   step 0.5   depth 3   speed 8   scale 1.5   (set)
                   depth+  speed-  z-  x+2  (relative nudges)
                   mode force   target 5   hover 8   run circle

  ⚠ SAFETY: indentation is hard-capped so the tip never presses deeper than
     the calibration Z + MAX_PRESS_MM (5 mm), no matter the depth/trim.

  In FORCE mode the robot measures a FUTEK baseline in the air, descends until
  the target force is reached, then a P-controller trims Z each waypoint to hold
  it while sliding. The surface Z is loaded from calib_<tip>.json (z_mm) — you
  choose the file at startup — and -/= (or the gauge drag, or console z+/z-)
  nudge it live.

Usage
─────
  python3 friction_live.py                 # choose calib, real robot + sensor
  python3 friction_live.py --tip short_new_solid
  python3 friction_live.py --no-robot      # sensor-only, drive the preview only
  python3 friction_live.py --no-robot --no-sensor   # pure demo

⚠ SPACE / arrows MOVE THE REAL ROBOT and TOUCH THE SENSOR.
"""
import argparse
import collections
import glob
import json
import math
import os
import queue
import sys
import threading
import time

FRICTION_DIR    = os.path.dirname(os.path.abspath(__file__))
INTEGRATION_DIR = os.path.normpath(os.path.join(FRICTION_DIR, "..", "Integration_2"))
sys.path.insert(0, INTEGRATION_DIR)
sys.path.insert(0, FRICTION_DIR)

try:
    import pygame
except ImportError:
    os.system(f"{sys.executable} -m pip install pygame")
    import pygame

import calibrate_points as cp     # canonical display frame (DISPLAY_XY / _to_display)
import trajectories as traj_lib
import ur5_friction as ur5

# ── Ranges / defaults ─────────────────────────────────────────────────────────
STEP_DEFAULT,  STEP_MIN,  STEP_MAX  = 1.0, 0.1, 5.0     # jog step (mm)
MAX_PRESS_MM                        = 7            # SAFETY: never indent
                                                        # deeper than calib Z + 10 mm
DEPTH_DEFAULT, DEPTH_MIN, DEPTH_MAX = 4.0, 0.0, MAX_PRESS_MM   # press depth (mm)
SPEED_DEFAULT, SPEED_MIN, SPEED_MAX = 8.0, 1.0, 30.0    # slide speed (mm/s)
HOVER_DEFAULT, HOVER_MIN, HOVER_MAX = 8.0, 2.0, 30.0    # idle height above surface
FORCE_DEFAULT, FORCE_MIN, FORCE_MAX = 5.0, 0.5, 20.0    # target contact force (N)
# Surface-Z trim: no tight range. Lower bound tied to the press cap so the
# surface approach still can't exceed calib + MAX_PRESS_MM; upper is open.
ZTRIM_MIN,     ZTRIM_MAX            = -MAX_PRESS_MM, 50.0
SCALE_MIN,     SCALE_MAX            = 0.2, 4.0          # pattern size (circle r = 12*scale)
OFFSET_LIMIT                        = 25.0              # |jog offset| clamp (mm)

# Surface Z (mm) loaded from the calibration file; the live z_trim adds to this.
BASE_CALIB_Z = 0.0

# Letter → trajectory name (game hotkeys)
TRAJ_KEYS = {
    'c': 'circle', 's': 'spiral', 'h': 'line_h', 'v': 'line_v',
    'd': 'diagonal_lr', 'f': 'diagonal_rl', 'x': 'cross',
    'r': 'raster', 't': 'star',
}

# ── Colours / layout ──────────────────────────────────────────────────────────
W, H   = 1180, 946
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
BLUE   = (90,  150, 230)

HEX_R          = 32
HEX_CX, HEX_CY = 380, 370
SX, SY         = 13.5, 13.5


def lerp_color(v):
    """5-stop ramp, same as visualizer_2d / calibrate_live."""
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
    """Sensor-frame mm → screen px (y flipped)."""
    return HEX_CX + xmm * SX, HEX_CY - ymm * SY


def robot_to_screen(xmm, ymm):
    """Robot-frame mm (trajectory coords) → screen px, via the display frame."""
    return disp_to_screen(*cp._to_display(xmm, ymm))


def tcp_to_screen(tcp):
    """Live TCP pose (robot frame, m) → screen px."""
    rdx = (tcp[0] - ur5.REFERENCE_POSE[0]) * 1000.0 - ur5.CALIB_X_MM
    rdy = (tcp[1] - ur5.REFERENCE_POSE[1]) * 1000.0 - ur5.CALIB_Y_MM
    return robot_to_screen(rdx, rdy)


def screen_to_robot(sx, sy):
    """Inverse of robot_to_screen: screen px → robot-frame mm.
    Used to turn a mouse click/drag on the hex map into an XY offset."""
    xd = (sx - HEX_CX) / SX
    yd = (HEX_CY - sy) / SY
    (a, b), (c, d) = cp._ROBOT_TO_SENSOR
    det = a * d - b * c
    return ((d * xd - b * yd) / det, (-c * xd + a * yd) / det)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ── Sensor-area boundary (from the visualizer) ────────────────────────────────
# The valid touch region is the sensor grid as drawn in the hex map, i.e. the
# bounding box of the sensor cells in the DISPLAY frame (+ a small margin). The
# old ±OFFSET_LIMIT clamp was a blind robot-frame square, which — because the
# robot frame is a ~120° rotation of the sensor — let the tip travel well past
# the real sensor edge. Deriving the box from cp.DISPLAY_XY keeps it aligned
# with what the operator sees.
_GDX = [d[0] for d in cp.DISPLAY_XY.values()]
_GDY = [d[1] for d in cp.DISPLAY_XY.values()]
SENSOR_MARGIN_MM = 2.0
DXD_MIN, DXD_MAX = min(_GDX) - SENSOR_MARGIN_MM, max(_GDX) + SENSOR_MARGIN_MM
DYD_MIN, DYD_MAX = min(_GDY) - SENSOR_MARGIN_MM, max(_GDY) + SENSOR_MARGIN_MM


def _sensor_to_robot(sdx, sdy):
    """Invert cp._ROBOT_TO_SENSOR: a vector in the sensor/display frame → the
    robot frame (same math as screen_to_robot, without the pixel scaling)."""
    (a, b), (c, d) = cp._ROBOT_TO_SENSOR
    det = a * d - b * c
    return ((d * sdx - b * sdy) / det, (-c * sdx + a * sdy) / det)


def sensor_delta_to_robot(sdx, sdy):
    """A jog along the SENSOR-display axes (x+/y+/arrows — what the operator is
    watching on the hex map) → a robot-frame (dx,dy) increment. Without this the
    raw robot-frame jog sends the tip off diagonally across the sensor."""
    return _sensor_to_robot(sdx, sdy)


def clamp_to_sensor(ox, oy):
    """Clamp a robot-frame offset so the indentor stays inside the real sensor
    area (the DISPLAY-frame bounding box of the cells), replacing the blind
    ±OFFSET_LIMIT robot-frame square."""
    dx, dy = cp._to_display(ox, oy)
    return _sensor_to_robot(clamp(dx, DXD_MIN, DXD_MAX),
                            clamp(dy, DYD_MIN, DYD_MAX))


# circle/spiral base geometry — point count scales WITH the pattern's scale
# factor so a bigger circle/spiral stays a smooth curve instead of a coarser
# polygon (fixed point count + bigger radius = longer straight segments =
# the robot visibly cuts corners instead of tracing the curve).
_BASE_N = {'circle': 72, 'spiral': 120}
_BASE_R = {'circle': 12.0, 'spiral': 14.0}


def traj_points(name, scale):
    """Trajectory points (ROBOT frame) at the given scale. circle/spiral
    regenerate their geometry (radius AND point count); other patterns just
    scale their fixed points (straight lines don't lose fidelity when stretched).

    trajectories.py authors its coordinates in the SENSOR/display frame — a
    "horizontal" line is horizontal ON THE SENSOR (corner a1→e5), a circle is a
    circle on the sensor, etc. The robot base frame is a ~120° rotation of that,
    so we convert sensor→robot here; otherwise line_h etc. trace the rotated
    version (e.g. a "horizontal" sweep runs almost entirely along display Y).
    Everything downstream (_build_pose, preview, clamp_to_sensor) is robot-frame.
    """
    if name == 'circle':
        n = int(clamp(_BASE_N['circle'] * scale, 24, 300))
        raw = traj_lib.circle(n_steps=n, radius_mm=_BASE_R['circle'] * scale)
    elif name == 'spiral':
        n = int(clamp(_BASE_N['spiral'] * scale, 60, 400))
        raw = traj_lib.spiral(n_steps=n, r_max_mm=_BASE_R['spiral'] * scale)
    else:
        raw = [(x * scale, y * scale) for x, y in traj_lib.TRAJECTORIES[name]()]
    return [_sensor_to_robot(x, y) for x, y in raw]


def parse_numeric(cmd, kw, inc):
    """Parse a console setting command for keyword `kw`.

    Accepts:  'kw 0.5' → set 0.5 (absolute)  |  'kw+' / 'kw-' → ±inc
              'kw+2' / 'kw-2' / 'kw -2' → relative by that amount
    Returns ('abs', value) | ('rel', delta) | None (not this keyword)."""
    if not cmd.startswith(kw):
        return None
    rest = cmd[len(kw):].strip().replace(" ", "")
    if rest == "":
        return None
    if rest == "+":
        return ("rel", inc)
    if rest == "-":
        return ("rel", -inc)
    try:
        if rest[0] in "+-":
            return ("rel", float(rest))
        return ("abs", float(rest))
    except ValueError:
        return None


def set_ztrim(state, new_trim):
    """Set the live surface-Z trim: effective surface = calib Z + trim.
    ur5._build_pose reads ur5.CALIB_Z_MM live, so this shifts every pose."""
    new_trim = round(clamp(new_trim, ZTRIM_MIN, ZTRIM_MAX), 2)
    ur5.CALIB_Z_MM = BASE_CALIB_Z + new_trim
    state.set(z_trim=new_trim)
    return new_trim


def adjust_depth(state, delta, contact, log=True):
    """Depth key in depth mode, target force in force mode."""
    if contact == 'force':
        v = state.bump('target_N', delta, FORCE_MIN, FORCE_MAX)
        if log: state.push_log(f"target {v:.1f} N")
    else:
        v = state.bump('depth', delta, DEPTH_MIN, DEPTH_MAX)
        if log: state.push_log(f"depth {v:.1f} mm")


def adjust_surfz(state, delta, log=True):
    v = set_ztrim(state, state.snap()['z_trim'] + delta)
    if log:
        state.push_log(f"surface Z {BASE_CALIB_Z + v:.1f} (trim {v:+.1f})")


# ── Shared state ──────────────────────────────────────────────────────────────
class State:
    def __init__(self):
        self.lock      = threading.Lock()
        self.status    = "starting"
        self.connected = False
        self.done      = False
        self.mode      = "game"          # "game" | "console"
        self.selected  = "circle"
        self.step      = STEP_DEFAULT
        self.depth     = DEPTH_DEFAULT
        self.contact   = "depth"         # "depth" (fixed Z) | "force" (FUTEK)
        self.target_N  = FORCE_DEFAULT   # target contact force (N) in force mode
        self.z_now     = 0.0             # live engaged depth (mm) during a run
        self.force_N   = 0.0             # live FUTEK force (N, baselined) during a run
        self.z_trim    = 0.0             # live surface-Z trim on top of calib Z (mm)
        self.speed     = SPEED_DEFAULT
        self.hover     = HOVER_DEFAULT
        self.direction = 1               # +1 forward/CCW, -1 reverse/CW
        self.off_x     = 0.0             # live XY offset (mm), applied everywhere
        self.off_y     = 0.0
        self.running   = False
        self.paused    = False           # pause/resume a running trajectory
        self.scale     = 1.0             # pattern size scale factor
        self.lap       = 0               # completed loops of the current run
        self.wp        = 0               # current waypoint index during a run
        self.n_wp      = 0
        self.log       = collections.deque(maxlen=200)

    def snap(self):
        with self.lock:
            return dict(self.__dict__, log=list(self.log))

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def push_log(self, msg):
        with self.lock:
            self.log.append(msg)

    # game-key mutations (clamped) ------------------------------------------------
    def jog(self, dx, dy):
        with self.lock:
            # dx, dy are ±1 along the SENSOR-display axes; convert to a robot-
            # frame increment and clamp to the real sensor area.
            ndx, ndy = sensor_delta_to_robot(dx * self.step, dy * self.step)
            self.off_x, self.off_y = clamp_to_sensor(self.off_x + ndx,
                                                     self.off_y + ndy)
            return self.off_x, self.off_y

    def bump(self, field, delta, lo, hi):
        with self.lock:
            setattr(self, field, round(clamp(getattr(self, field) + delta, lo, hi), 3))
            return getattr(self, field)


# ── Robot connection (persistent — ur5_friction has no connect()) ─────────────
def connect_robot():
    import rtde_control, rtde_receive
    rtde_r = None
    for attempt in range(3):
        try:
            rtde_r = rtde_receive.RTDEReceiveInterface(ur5.ROBOT_IP)
            print("[live] Receive connected")
            break
        except Exception as e:
            print(f"[live] Receive {attempt+1}/3 failed: {e}")
            time.sleep(2)
    if rtde_r is None:
        return None, None
    ur5._rtde_r_ref[0] = rtde_r
    threading.Thread(target=ur5._force_reader_loop, daemon=True).start()

    rtde_c = None
    for attempt in range(3):
        try:
            rtde_c = rtde_control.RTDEControlInterface(
                ur5.ROBOT_IP, frequency=500.0,
                flags=rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
            print("[live] Control connected")
            break
        except Exception as e:
            print(f"[live] Control {attempt+1}/3 failed: {e}")
            time.sleep(2)
    return rtde_c, rtde_r


def _read_calib(path):
    try:
        with open(path) as f:
            d = json.load(f)
        return d.get("x_mm", 0.0), d.get("y_mm", 0.0), d.get("z_mm", 0.0)
    except Exception:
        return None


def select_calibration(tip=None):
    """Choose the calibration file whose Z sets the initial touch height.

    With --tip: loads calib_<tip>.json non-interactively. Otherwise prints a
    menu of calib_*.json files in Integration_2 (Z shown) and asks which to use.
    Returns the chosen path, or None for zero offset."""
    if tip:
        cand = os.path.join(INTEGRATION_DIR, f"calib_{tip}.json")
        if os.path.exists(cand):
            return cand
        print(f"[calib] calib_{tip}.json not found — using zero offset")
        return None

    files = sorted(f for f in glob.glob(os.path.join(INTEGRATION_DIR, "calib_*.json"))
                   if not os.path.basename(f).startswith("calib_points"))
    plain = os.path.join(INTEGRATION_DIR, "calib.json")
    if os.path.exists(plain):
        files.insert(0, plain)
    if not files:
        print("[calib] No calib_*.json files found — using zero offset")
        return None

    print("\n" + "=" * 58)
    print("  SELECT CALIBRATION  (its Z = initial surface / touch height)")
    print("=" * 58)
    for i, p in enumerate(files, 1):
        o = _read_calib(p)
        z = f"Z={o[2]:+.2f} mm" if o else "unreadable"
        print(f"  {i:2d}) {os.path.basename(p):<34s} {z}")
    print("   0) none (zero offset — tip will NOT touch correctly)")
    print("=" * 58)
    while True:
        try:
            ans = input(f"  Choose calibration [0-{len(files)}] > ").strip()
        except (EOFError, KeyboardInterrupt):
            raise SystemExit(1)
        if ans == "0":
            return None
        if ans.isdigit() and 1 <= int(ans) <= len(files):
            return files[int(ans) - 1]
        print("  Invalid choice, try again.")


def apply_calibration(path):
    """Apply the chosen calibration file; returns (label, z_mm)."""
    if not path:
        print("[live] Using zero offset")
        return "(zero offset)", 0.0
    o = _read_calib(path) or (0.0, 0.0, 0.0)
    ur5.set_calibration(*o)
    return os.path.basename(path), o[2]


# ── Worker: drives the robot, consumes commands from cmd_q ────────────────────
def worker(state, stop_evt, abort_evt, rtde_c, cmd_q):
    def log(m):
        print("  " + m)
        state.push_log(m)

    def moveL(pose, vel):
        rtde_c.moveL(pose, vel, ur5.ACCELERATION)

    def hover_pose():
        s = state.snap()
        return ur5._build_pose(s['off_x'], s['off_y'], s['hover'])

    def go_hover(msg=None):
        state.set(status="hover")
        moveL(hover_pose(), ur5.VELOCITY_HOME)
        if msg:
            log(msg)

    def read_force(baseline):
        try:
            return ur5.ai0_to_N(ur5.get_state()['ai0']) - baseline
        except Exception:
            return 0.0

    def press_cap():
        """SAFETY: deepest indentation allowed = calib surface + MAX_PRESS_MM.
        Accounts for the live surface-Z trim so the tip never goes below
        (calib Z + 5 mm) no matter how depth/trim are set."""
        return max(0.0, MAX_PRESS_MM + state.snap()['z_trim'])

    def engage(ox, oy, x0, y0, contact):
        """Descend onto the surface at (x0,y0)+offset. Returns (z, baseline)."""
        # Keep the descent point inside the real sensor area (trajectory geometry
        # + offset can otherwise land off the sensor).
        x0, y0 = clamp_to_sensor(x0 + ox, y0 + oy)
        ox = oy = 0.0
        moveL(ur5._build_pose(x0 + ox, y0 + oy, state.snap()['hover']), ur5.VELOCITY_HOME)
        cap = press_cap()
        if contact == 'force':
            state.set(status="measuring FUTEK baseline (in air)")
            buf = []
            for _ in range(50):
                buf.append(ur5.get_state()['ai0']); time.sleep(0.02)
            baseline = (ur5.ai0_to_N(sum(buf) / len(buf)) if buf
                        else ur5.ai0_to_N(ur5.FUTEK_ZERO_V))
            target = state.snap()['target_N']
            state.set(status=f"engaging to {target:.1f} N")
            z = 0.0
            reached = False
            while z < cap:                              # capped at calib + 5 mm
                if abort_evt.is_set() or stop_evt.is_set():
                    break
                z += 0.15
                moveL(ur5._build_pose(x0 + ox, y0 + oy, -z), ur5.VELOCITY_ENGAGE)
                f = read_force(baseline)
                state.set(z_now=z, force_N=f)
                if f >= target:
                    log(f"contact at z={z:.2f} mm, {f:.2f} N"); reached = True; break
            if not reached:
                log(f"⚠ target {target:.1f} N not reached within Z cap ({cap:.1f} mm)")
            return z, baseline
        z = min(state.snap()['depth'], cap)
        moveL(ur5._build_pose(x0 + ox, y0 + oy, 0.0), ur5.VELOCITY_ENGAGE)
        moveL(ur5._build_pose(x0 + ox, y0 + oy, -z), ur5.VELOCITY_ENGAGE)
        state.set(z_now=z)
        return z, None

    def hold_z(z, baseline, contact):
        """Per-step engaged Z, always capped at the safety limit (calib + 5)."""
        cap = press_cap()
        if contact != 'force':
            return min(state.snap()['depth'], cap)
        f   = read_force(baseline)
        err = state.snap()['target_N'] - f
        dz  = clamp(ur5.FORCE_KP_MM_PER_N * err, -ur5.FORCE_MAX_DZ_MM, ur5.FORCE_MAX_DZ_MM)
        state.set(force_N=f)
        return clamp(z + dz, 0.0, cap)

    def lift_home():
        cs = state.snap()
        try:
            moveL(ur5._build_pose(cs['off_x'], cs['off_y'], 0.0), ur5.VELOCITY_ENGAGE)
        except Exception:
            pass
        state.set(running=False, z_now=0.0, paused=False)
        go_hover("done — hover")

    def run_trajectory(name):
        """Continuously loop until stopped. Trajectory (selected), direction,
        speed, scale, offset, depth/force and pause are all read LIVE — so
        pressing a new letter switches the running pattern on the fly."""
        if name not in traj_lib.TRAJECTORIES:
            log(f"unknown trajectory '{name}'"); return
        cur   = name
        sc    = state.snap()['scale']
        base  = traj_points(cur, sc)          # scale baked into the geometry
        n     = len(base)
        if n < 2:
            log("trajectory too short"); return
        contact = state.snap()['contact']
        abort_evt.clear()
        # keep 'selected' == the running pattern so the preview always matches
        state.set(selected=cur, running=True, wp=0, n_wp=n, lap=0, paused=False,
                  status=f"looping {cur} [{contact}]")
        log(f"run {cur}: {contact}, looping until STOP (Backspace)")
        try:
            s0 = state.snap()
            i = 0 if s0['direction'] >= 0 else n - 1
            x0, y0 = base[i]
            z, baseline = engage(s0['off_x'], s0['off_y'], x0, y0, contact)

            while not (abort_evt.is_set() or stop_evt.is_set()):
                cs = state.snap()

                # LIVE trajectory switch — a new letter changed 'selected'
                if cs['selected'] != cur and cs['selected'] in traj_lib.TRAJECTORIES:
                    cur = cs['selected']
                    sc = cs['scale']
                    base = traj_points(cur, sc)
                    n = len(base)
                    i = 0 if cs['direction'] >= 0 else n - 1
                    state.set(n_wp=n, lap=0)
                    log(f"switched → {cur}")
                    continue

                # LIVE scale change — regenerate geometry so circle/spiral
                # stay smooth curves instead of a coarser polygon as they grow
                if cs['scale'] != sc:
                    sc = cs['scale']
                    frac = i / n if n else 0.0
                    base = traj_points(cur, sc)
                    n = len(base)
                    i = min(int(round(frac * n)), n - 1)
                    state.set(n_wp=n)
                    log(f"scale {sc:.2f}x")
                    continue

                if cs['paused']:
                    state.set(status=f"PAUSED {cur}")
                    z = hold_z(z, baseline, contact)
                    time.sleep(0.08)
                    continue

                ox, oy = cs['off_x'], cs['off_y']
                speed = cs['speed'] / 1000.0
                x, y = base[i]
                # Clamp the waypoint (pattern + offset) to the sensor area so the
                # trajectory never runs off the sensor, whatever the scale/offset.
                ax, ay = clamp_to_sensor(x + ox, y + oy)
                z = hold_z(z, baseline, contact)
                moveL(ur5._build_pose(ax, ay, -z), speed)
                state.set(wp=i + 1, z_now=z,
                          status=f"looping {cur} lap {cs['lap']} "
                                 f"({'CW/rev' if cs['direction'] < 0 else 'CCW/fwd'})")
                i += (1 if cs['direction'] >= 0 else -1)   # LIVE direction
                if i >= n:
                    i = 0; state.set(lap=cs['lap'] + 1)
                elif i < 0:
                    i = n - 1; state.set(lap=cs['lap'] + 1)
        except Exception as e:
            log(f"trajectory error: {e}")
        finally:
            lift_home()

    # ── init: home, then hover ───────────────────────────────
    try:
        state.set(status="homing")
        moveL(ur5._build_pose(0, 0, ur5.SAFE_HOME_Z_MM), ur5.VELOCITY_HOME)
        go_hover("ready — select a trajectory, SPACE to run")
    except Exception as e:
        log(f"init error: {e}")

    last_jog = None
    while not stop_evt.is_set():
        # 1) apply idle jog: if the offset/hover changed while idle, physically
        #    move there (during a run this is handled per-waypoint instead)
        s = state.snap()
        cur = (round(s['off_x'], 3), round(s['off_y'], 3), round(s['hover'], 3),
               round(ur5.CALIB_Z_MM, 3))
        if not s['running'] and cur != last_jog:
            try:
                moveL(hover_pose(), ur5.VELOCITY_HOME)
            except Exception as e:
                log(f"jog error: {e}")
            last_jog = cur

        # 2) consume one command
        try:
            cmd = cmd_q.get(timeout=0.12)
        except queue.Empty:
            continue
        cmd = (cmd or "").strip().lower()

        if cmd.startswith("run"):
            name = cmd.split()[1] if len(cmd.split()) > 1 else state.snap()['selected']
            run_trajectory(name)
            last_jog = None
        elif cmd == "home":
            go_hover("home"); last_jog = None
        elif cmd == "stop":
            abort_evt.set()
        elif cmd == "quit":
            break
        elif cmd == "flip":
            state.set(direction=-state.snap()['direction']); log("direction flipped")
        elif cmd == "center":
            state.set(off_x=0.0, off_y=0.0); last_jog = None; log("offset reset")
        elif cmd.startswith("select"):
            parts = cmd.split()
            if len(parts) > 1 and parts[1] in traj_lib.TRAJECTORIES:
                state.set(selected=parts[1]); log(f"selected {parts[1]}")
        elif cmd.startswith("mode"):
            parts = cmd.split()
            if len(parts) > 1 and parts[1] in ("depth", "force"):
                state.set(contact=parts[1]); log(f"contact mode {parts[1]}")
            else:
                log("usage: mode depth | mode force")
        else:
            # numeric settings — each accepts 'kw N' (set) and 'kw+'/'kw-'/'kw-2'
            # (relative), e.g.  step 0.5   depth+   z-   speed 8   scale 1.2
            handled = False
            for kw, inc, lo, hi, field, unit in [
                ("depth",  0.5, DEPTH_MIN,  DEPTH_MAX,  "depth",    " mm"),
                ("speed",  1.0, SPEED_MIN,  SPEED_MAX,  "speed",    " mm/s"),
                ("step",   0.1, STEP_MIN,   STEP_MAX,   "step",     " mm"),
                ("scale",  0.1, SCALE_MIN,  SCALE_MAX,  "scale",    "x"),
                ("hover",  1.0, HOVER_MIN,  HOVER_MAX,  "hover",    " mm"),
                ("target", 0.5, FORCE_MIN,  FORCE_MAX,  "target_N", " N"),
                ("force",  0.5, FORCE_MIN,  FORCE_MAX,  "target_N", " N"),
            ]:
                r = parse_numeric(cmd, kw, inc)
                if r:
                    cur = state.snap()[field]
                    newv = clamp(r[1] if r[0] == "abs" else cur + r[1], lo, hi)
                    state.set(**{field: newv})
                    log(f"{kw} {newv:g}{unit}")
                    handled = True
                    break
            if not handled:
                # X / Y jog offset:  x+ x- y+ y- (relative, along the SENSOR
                # axes) | x 5 (absolute robot-frame). Clamped to the sensor area.
                cur = state.snap()
                ox, oy, sstep = cur['off_x'], cur['off_y'], cur['step']
                rx = parse_numeric(cmd, "x", sstep)
                ry = parse_numeric(cmd, "y", sstep)
                if rx or ry:
                    if rx:
                        if rx[0] == "abs":
                            ox = rx[1]
                        else:
                            ndx, ndy = sensor_delta_to_robot(rx[1], 0.0)
                            ox += ndx; oy += ndy
                    if ry:
                        if ry[0] == "abs":
                            oy = ry[1]
                        else:
                            ndx, ndy = sensor_delta_to_robot(0.0, ry[1])
                            ox += ndx; oy += ndy
                    ox, oy = clamp_to_sensor(ox, oy)
                    state.set(off_x=round(ox, 2), off_y=round(oy, 2))
                    last_jog = None
                    log(f"offset {ox:+.1f}, {oy:+.1f} mm")
                    handled = True
            if not handled:
                # Z jog:  z-  moves the indentor DOWN,  z+  moves it UP
                r = parse_numeric(cmd, "z", 0.5)
                if r:
                    cur = state.snap()['z_trim']
                    v = set_ztrim(state, r[1] if r[0] == "abs" else cur + r[1])
                    log(f"surface Z {BASE_CALIB_Z + v:.1f} mm (trim {v:+.1f})")
                    last_jog = None
                    handled = True
            if not handled:
                log(f"unknown: '{cmd}'  (try: x+ · y- · z- · step 0.5 · depth+)")

    # shutdown → home
    try:
        state.set(status="returning home")
        moveL(ur5._build_pose(0, 0, ur5.SAFE_HOME_Z_MM), ur5.VELOCITY_HOME)
        rtde_c.stopScript()
    except Exception:
        pass
    state.set(done=True, status="done")
    stop_evt.set()


# ── Sensor demo colours ───────────────────────────────────────────────────────
def demo_values(frame):
    t = frame * 0.04
    return [max(0.0, min(1.0, 0.5 * math.sin(t + i * 0.4)
                         * math.sin(t * 0.7 + i * 0.3) + 0.25))
            for i in range(19)]


# ── Render loop (MAIN thread) ─────────────────────────────────────────────────
def render_loop(state, stop_evt, abort_evt, args, sensor_mod, cmd_q):
    pygame.init()
    pygame.display.set_caption("KYWO — Friction Live")
    screen = pygame.display.set_mode((W, H))
    clock  = pygame.time.Clock()
    try:
        pygame.key.start_text_input()
    except Exception:
        pass
    try:
        font_lg = pygame.font.SysFont('DejaVuSans', 16, bold=True)
        font_md = pygame.font.SysFont('DejaVuSans', 13)
        font_sm = pygame.font.SysFont('DejaVuSans', 11)
    except Exception:
        font_lg = pygame.font.Font(None, 22)
        font_md = pygame.font.Font(None, 18)
        font_sm = pygame.font.Font(None, 15)

    trail = []
    show_trail = True
    show_labels = True
    cmd_buf = ""
    frame_n = 0
    held_since = {}   # key → frame it went down, for hold-to-repeat jogging

    CON_X, CON_Y = 10, 792
    CON_W, CON_H = W - 20, H - 792 - 8
    CON_LINES    = 4
    INP_H        = 30

    def send(text):
        nonlocal show_trail, show_labels
        c = text.strip().lower()
        if c in ("trail",):
            show_trail = not show_trail; return
        if c in ("labels", "label"):
            show_labels = not show_labels; return
        if c in ("recal", "recalibrate"):
            if sensor_mod is not None:
                try: sensor_mod.recalibrate(); state.push_log("sensor recalibrated")
                except Exception: pass
            return
        state.push_log(f"> {c}")
        cmd_q.put(c)

    def game_key(ev):
        """Hotkeys active in GAME mode — LETTERS + arrows only, no symbols."""
        k = ev.key
        s = state.snap()
        cN = s['contact']
        if k == pygame.K_LEFT:    state.jog(-1, 0)      # X / Y offset (live)
        elif k == pygame.K_RIGHT: state.jog(+1, 0)
        elif k == pygame.K_UP:    state.jog(0, +1)
        elif k == pygame.K_DOWN:  state.jog(0, -1)
        elif k == pygame.K_e:     adjust_depth(state, +0.5, cN)   # deeper / +force
        elif k == pygame.K_q:     adjust_depth(state, -0.5, cN)   # shallower / -force
        elif k in (pygame.K_EQUALS, pygame.K_KP_PLUS):
            adjust_surfz(state, +0.2)     # z+  → indentor UP
        elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
            adjust_surfz(state, -0.2)     # z-  → indentor DOWN
        elif k == pygame.K_k:
            v = state.bump('speed', +1, SPEED_MIN, SPEED_MAX); state.push_log(f"speed {v:.0f}")
        elif k == pygame.K_j:
            v = state.bump('speed', -1, SPEED_MIN, SPEED_MAX); state.push_log(f"speed {v:.0f}")
        elif k == pygame.K_n:
            state.set(off_x=0.0, off_y=0.0); state.push_log("offset centered")
        elif k == pygame.K_o:
            state.set(direction=-s['direction']); state.push_log("flip direction")
        elif k == pygame.K_p:
            state.set(paused=not s['paused'])
            state.push_log("resumed" if s['paused'] else "paused")
        elif k == pygame.K_m:
            nm = 'force' if cN == 'depth' else 'depth'
            state.set(contact=nm); state.push_log(f"contact mode: {nm}")
        elif k == pygame.K_9:
            v = state.bump('scale', -0.1, SCALE_MIN, SCALE_MAX)
            state.push_log(f"scale {v:.2f}x  (circle r≈{12*v:.0f} mm)")
        elif k == pygame.K_0:
            v = state.bump('scale', +0.1, SCALE_MIN, SCALE_MAX)
            state.push_log(f"scale {v:.2f}x  (circle r≈{12*v:.0f} mm)")
        elif k == pygame.K_SPACE:
            cmd_q.put(f"run {s['selected']}")
        elif k == pygame.K_g:
            cmd_q.put("home")
        elif k == pygame.K_BACKSPACE:
            abort_evt.set(); cmd_q.put("stop"); state.push_log("STOP")
        else:
            ch = pygame.key.name(k)
            if ch in TRAJ_KEYS:
                state.set(selected=TRAJ_KEYS[ch]); state.push_log(f"select {TRAJ_KEYS[ch]}")

    preview_cache = {}   # name → robot-frame pts

    # ── Z HEIGHT GAUGE geometry — also a draggable slider (mouse), not just
    # keyboard. Fixed layout, so computed once outside the frame loop.
    gx, gy0, gy1 = 712, 70, 560
    Ztop, Zbot = 16.0, -7.0                 # mm above / below the touch surface
    GAUGE_HIT = pygame.Rect(gx - 20, gy0 - 12, 40, gy1 - gy0 + 24)

    def z_to_y(zmm):
        return gy0 + (Ztop - clamp(zmm, Zbot, Ztop)) / (Ztop - Zbot) * (gy1 - gy0)

    def y_to_z(ypix):
        frac = clamp((ypix - gy0) / (gy1 - gy0), 0.0, 1.0)
        return Ztop - frac * (Ztop - Zbot)

    def drag_z(ypix, log=False):
        v = set_ztrim(state, y_to_z(ypix) - state.snap()['hover'])
        if log:
            state.push_log(f"surface Z {BASE_CALIB_Z + v:.1f} mm (trim {v:+.1f})")

    dragging_z = False

    # ── HEX MAP as an XY-offset pad — drag anywhere on the map to shift the
    # whole pattern (same off_x/off_y the arrow keys drive). Kept clear of the
    # Z gauge column (x > 680) and the title / colour-scale bar.
    HEXMAP_HIT = pygame.Rect(40, 45, 640, 675)

    def drag_xy(pos, log=False):
        ox, oy = clamp_to_sensor(*screen_to_robot(*pos))
        ox, oy = round(ox, 2), round(oy, 2)
        state.set(off_x=ox, off_y=oy)
        if log:
            state.push_log(f"offset {ox:+.1f}, {oy:+.1f} mm (drag)")

    dragging_xy = False

    while not stop_evt.is_set():
        frame_n += 1
        mode = state.snap()['mode']
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                abort_evt.set(); cmd_q.put("quit"); stop_evt.set()
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 \
                    and GAUGE_HIT.collidepoint(event.pos):
                dragging_z = True
                drag_z(event.pos[1])
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 \
                    and HEXMAP_HIT.collidepoint(event.pos):
                dragging_xy = True
                drag_xy(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if dragging_z:
                    drag_z(event.pos[1], log=True)
                if dragging_xy:
                    drag_xy(event.pos, log=True)
                dragging_z = False
                dragging_xy = False
            elif event.type == pygame.MOUSEMOTION and dragging_z:
                drag_z(event.pos[1])
            elif event.type == pygame.MOUSEMOTION and dragging_xy:
                drag_xy(event.pos)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_TAB:
                state.set(mode=("console" if mode == "game" else "game")); cmd_buf = ""
            elif mode == "console":
                if event.type == pygame.TEXTINPUT:
                    if event.text != "\t":
                        cmd_buf += event.text
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        send(cmd_buf); cmd_buf = ""
                    elif event.key == pygame.K_BACKSPACE:
                        cmd_buf = cmd_buf[:-1]
                    elif event.key == pygame.K_ESCAPE:
                        state.set(mode="game"); cmd_buf = ""
            elif event.type == pygame.KEYDOWN:
                game_key(event)

        # ── Hold-to-repeat: keep changing X/Y/Z/etc while the key is HELD ──
        # (works during a run — the worker reads these live each waypoint).
        if state.snap()['mode'] == 'game':
            pressed = pygame.key.get_pressed()

            def held(key, delay=10, rate=3):
                """True on a steady cadence while held (after the first tap,
                which game_key already handled)."""
                if not pressed[key]:
                    held_since.pop(key, None)
                    return False
                d = frame_n - held_since.setdefault(key, frame_n)
                return d >= delay and (d - delay) % rate == 0

            cN = state.snap()['contact']
            if held(pygame.K_LEFT):  state.jog(-1, 0)
            if held(pygame.K_RIGHT): state.jog(+1, 0)
            if held(pygame.K_UP):    state.jog(0, +1)
            if held(pygame.K_DOWN):  state.jog(0, -1)
            if held(pygame.K_e):     adjust_depth(state, +0.3, cN, log=False)
            if held(pygame.K_q):     adjust_depth(state, -0.3, cN, log=False)
            if held(pygame.K_EQUALS):   adjust_surfz(state, +0.1, log=False)  # z+
            if held(pygame.K_KP_PLUS):  adjust_surfz(state, +0.1, log=False)
            if held(pygame.K_MINUS):    adjust_surfz(state, -0.1, log=False)  # z-
            if held(pygame.K_KP_MINUS): adjust_surfz(state, -0.1, log=False)
            if held(pygame.K_k):     state.bump('speed', +0.5, SPEED_MIN, SPEED_MAX)
            if held(pygame.K_j):     state.bump('speed', -0.5, SPEED_MIN, SPEED_MAX)
            if held(pygame.K_0):     state.bump('scale', +0.05, SCALE_MIN, SCALE_MAX)
            if held(pygame.K_9):     state.bump('scale', -0.05, SCALE_MIN, SCALE_MAX)
        else:
            held_since.clear()

        # ── Live data ─────────────────────────────────────────
        values = sensor_mod.get_values() if sensor_mod is not None else demo_values(frame_n)
        s = state.snap()

        tcp = None
        if not args.no_robot and s['connected']:
            try:
                tcp = ur5.get_state()['tcp']
            except Exception:
                tcp = None
        ft  = ur5.get_force() if (not args.no_robot and s['connected']) else [0.0] * 6
        try:
            f_lc = ur5.ai0_to_N(ur5.get_state()['ai0']) if (not args.no_robot and s['connected']) else 0.0
        except Exception:
            f_lc = 0.0

        fired_idx = max(range(19), key=lambda i: values[i]) if values else 0
        fired_val = values[fired_idx] if values else 0.0

        # ── Draw hex map ──────────────────────────────────────
        screen.fill(BG)
        pygame.draw.rect(screen, PANEL, pygame.Rect(10, 10, 760, 760), border_radius=10)
        if dragging_xy:
            pygame.draw.rect(screen, CYAN, pygame.Rect(10, 10, 760, 760),
                             border_radius=10, width=2)
        blit(screen, "Friction live — pressure map", font_lg, TEXT, 28, 20)
        blit(screen, "drag ↔ offset X/Y", font_sm,
             CYAN if dragging_xy else MUTED, 760, 24, 'right')

        for pt in cp.SCAN_ORDER:
            cx, cy = disp_to_screen(*cp.DISPLAY_XY[pt])
            idx = cp.UR5_TO_IDX[pt]
            v = float(values[idx]) if idx < len(values) else 0.0
            col = lerp_color(v)
            pts = hex_pts(cx, cy, HEX_R)
            pygame.draw.polygon(screen, col, pts)
            pygame.draw.polygon(
                screen, (60, 60, 80) if v < 0.1 else tuple(min(255, c + 40) for c in col),
                pts, 1)
            if show_labels:
                blit(screen, cp.UR5_TO_LABEL[pt], font_sm,
                     WHITE if v > 0.4 else TEXT, cx, cy - 8, 'center')
            if v > 0.02:
                blit(screen, f"{v:.2f}", font_sm, WHITE if v > 0.4 else TEXT,
                     cx, cy + 6, 'center')

        # ORANGE ring — fired cell
        if fired_val > 0.05:
            cx, cy = disp_to_screen(*cp.DISPLAY_XY[cp.SCAN_ORDER[fired_idx]])
            pygame.draw.circle(screen, ORANGE, (int(cx), int(cy)), HEX_R + 10, 2)

        # ── Trajectory preview (selected), shifted by the live XY offset ──
        name = s['selected']
        ox, oy, sc = s['off_x'], s['off_y'], s['scale']
        cache_key = (name, round(sc, 2))
        if cache_key not in preview_cache:
            try:
                preview_cache[cache_key] = traj_points(name, sc)
            except Exception:
                preview_cache[cache_key] = []
            if len(preview_cache) > 40:      # scale changes constantly; keep it bounded
                preview_cache.clear()
        pts_r = preview_cache[cache_key]
        if s['direction'] < 0:
            pts_r = list(reversed(pts_r))
        # Clamp to the sensor area so the preview matches the (clamped) motion.
        scr = [robot_to_screen(*clamp_to_sensor(x + ox, y + oy)) for (x, y) in pts_r]
        if len(scr) > 1:
            # the path is only a GUIDE line; the single moving point is the robot
            pygame.draw.lines(screen, BLUE, False,
                              [(int(x), int(y)) for x, y in scr], 2)
            sx, sy = scr[0]                                  # start = small cross
            pygame.draw.line(screen, GREEN, (sx - 5, sy), (sx + 5, sy), 2)
            pygame.draw.line(screen, GREEN, (sx, sy - 5), (sx, sy + 5), 2)

        # ── Live indentor ─────────────────────────────────────
        if tcp is not None:
            tx, ty = tcp_to_screen(tcp)
            trail.append((tx, ty))
            if len(trail) > 160:
                trail = trail[-160:]
            if show_trail and len(trail) > 1:
                pygame.draw.lines(screen, (40, 120, 130), False,
                                  [(int(x), int(y)) for x, y in trail], 2)
            if s['running']:
                pygame.draw.circle(screen, RED, (int(tx), int(ty)), 11, 3)
                pygame.draw.circle(screen, CYAN, (int(tx), int(ty)), 6)
            else:
                pygame.draw.circle(screen, CYAN, (int(tx), int(ty)), 6)
                pygame.draw.circle(screen, WHITE, (int(tx), int(ty)), 6, 1)

        # ── Z HEIGHT GAUGE (side view — the top-down map can't show up/down) ──
        # Shows the tip height vs the calib touch surface. Drag it with the
        # mouse, or z-/z+ (keyboard), to move the indentor's surface Z live.
        pygame.draw.rect(screen, CARD, (gx - 9, gy0 - 6, 18, gy1 - gy0 + 12), border_radius=6)
        if dragging_z:
            pygame.draw.rect(screen, CYAN, (gx - 9, gy0 - 6, 18, gy1 - gy0 + 12),
                             border_radius=6, width=2)
        blit(screen, "TIP", font_sm, MUTED, gx, gy0 - 20, 'center')
        blit(screen, "HEIGHT", font_sm, MUTED, gx, gy0 - 9, 'center')
        blit(screen, "drag ↕", font_sm, MUTED if not dragging_z else CYAN, gx, gy1 + 8, 'center')
        for zt in (5, 10, 15):                  # air ticks (above surface)
            yt = z_to_y(zt)
            pygame.draw.line(screen, (60, 60, 80), (gx - 6, yt), (gx + 6, yt), 1)
            blit(screen, f"+{zt}", font_sm, MUTED, gx + 14, yt - 6)
        ys = z_to_y(0.0)                         # calib touch surface
        pygame.draw.line(screen, GREEN, (gx - 14, ys), (gx + 14, ys), 2)
        blit(screen, "touch", font_sm, GREEN, gx + 14, ys - 6)
        yc = z_to_y(-MAX_PRESS_MM)               # safety cap
        pygame.draw.line(screen, RED, (gx - 14, yc), (gx + 14, yc), 2)
        blit(screen, f"max -{MAX_PRESS_MM:.0f}", font_sm, RED, gx + 14, yc - 6)
        # current tip height above the calib surface
        if tcp is not None:
            tip_h = (tcp[2] - ur5.REFERENCE_POSE[2]) * 1000.0 - BASE_CALIB_Z
        elif s['running']:
            tip_h = s['z_trim'] - s['z_now']
        else:
            tip_h = s['z_trim'] + s['hover']
        yh = z_to_y(tip_h)
        hcol = CYAN if tip_h >= 0 else RED
        pygame.draw.polygon(screen, hcol, [(gx - 13, yh), (gx - 23, yh - 6), (gx - 23, yh + 6)])
        pygame.draw.circle(screen, hcol, (gx, int(yh)), 5)
        blit(screen, f"{tip_h:+.1f}mm", font_sm, hcol, gx - 66, yh - 6)

        # colour scale
        bx, by, bw, bh = 40, 736, 640, 10
        for px in range(bw):
            pygame.draw.rect(screen, lerp_color(px / bw), (bx + px, by, 1, bh))
        pygame.draw.rect(screen, MUTED, (bx, by, bw, bh), 1)
        blit(screen, "0.0", font_sm, MUTED, bx, by + 12)
        blit(screen, "1.0", font_sm, MUTED, bx + bw - 18, by + 12)

        # ── Right panel: stats + key guide ────────────────────
        pygame.draw.rect(screen, PANEL, pygame.Rect(786, 10, 384, 760), border_radius=10)
        if args.no_robot:
            dot, rob = MUTED, "no robot"
        elif s['done']:
            dot, rob = AMBER, "done"
        elif s['connected']:
            dot, rob = GREEN, "live"
        else:
            dot, rob = AMBER, "connecting"
        pygame.draw.circle(screen, dot, (806, 30), 7)
        blit(screen, rob, font_sm, dot, 820, 23)
        blit(screen, f"MODE: {s['mode'].upper()}", font_lg,
             CYAN if s['mode'] == 'console' else TEXT, 900, 20)

        if s['contact'] == 'force':
            setpoint = ("Target F   q / e", f"{s['target_N']:.1f} N", CYAN)
        else:
            setpoint = ("Depth   q / e", f"{s['depth']:.1f} / {MAX_PRESS_MM:.0f}max mm", TEXT)
        rows = [
            ("Selected  (c s h v d f x r t)",
             f"{name}  ({'CW/rev' if s['direction']<0 else 'CCW/fwd'})", GREEN),
            ("Contact mode   m", s['contact'].upper(),
             CYAN if s['contact'] == 'force' else TEXT),
            setpoint,
            ("Speed   j / k", f"{s['speed']:.0f} mm/s", TEXT),
            ("XY offset   arrows  (n=center)",
             f"{s['off_x']:+.1f}, {s['off_y']:+.1f} mm  (step {s['step']:.1f})", AMBER),
            ("Z (indentor)   z- / z+ : - / =",
             f"{BASE_CALIB_Z + s['z_trim']:.1f} mm  (calib {BASE_CALIB_Z:.1f}, "
             f"trim {s['z_trim']:+.1f})  cap +{MAX_PRESS_MM:.0f}", TEXT),
            ("Scale 9/0  (circle r≈%d mm)" % int(12 * s['scale']),
             f"{s['scale']:.2f}x   hover {s['hover']:.1f} mm", TEXT),
            ("Run  (lap · WP · z)",
             (f"{'PAUSED' if s['paused'] else 'RUN'}  lap {s['lap']}  "
              f"WP {s['wp']}/{s['n_wp']}  z={s['z_now']:.1f}"
              if s['running'] else "idle"),
             AMBER if s['paused'] else (RED if s['running'] else MUTED)),
            ("Contact Fz / FUTEK", f"{ft[2]:+.1f} N / {f_lc:.1f} N"
             + (f"  (Δ{s['force_N']:+.1f})" if s['running'] and s['contact'] == 'force' else ""),
             CYAN if s['running'] and s['contact'] == 'force' else TEXT),
            ("Cells / peak", f"{sum(1 for v in values if v>0.05)}/19  {fired_val:.2f}",
             lerp_color(fired_val) if fired_val > 0.05 else MUTED),
        ]
        yy = 46
        for label, val, col in rows:
            pygame.draw.rect(screen, CARD, pygame.Rect(798, yy, 360, 30), border_radius=6)
            blit(screen, label, font_sm, MUTED, 808, yy + 2)
            blit(screen, val, font_md, col, 808, yy + 15)
            yy += 33

        yy += 4
        pygame.draw.line(screen, (50, 50, 70), (798, yy), (1158, yy), 1); yy += 8
        blit(screen, "TRAJECTORIES  (press to select · SPACE run)", font_sm, MUTED, 798, yy)
        yy += 16
        guide = [
            "c circle  s spiral  h horiz  v vert  x cross",
            "d diag /  f diag \\  r raster  t star",
            "arrows = X/Y    q/e = depth    -/= = surface Z",
            "  (or drag the map for X/Y, the TIP HEIGHT gauge for Z)",
            "j/k = speed    9/0 = scale(radius)    o = flip",
            "p = pause   n = center   m = depth/force",
            "SPACE run (loops)   g home   BACKSPACE stop",
            "Tab console: step 0.5 · depth+ · z- · scale 1.5",
        ]
        for line in guide:
            blit(screen, line, font_sm, TEXT, 800, yy); yy += 16

        yy += 6
        pygame.draw.line(screen, (50, 50, 70), (798, yy), (1158, yy), 1); yy += 6
        blit(screen, s['status'], font_sm, AMBER, 800, yy)

        # ── Console (bottom) ──────────────────────────────────
        pygame.draw.rect(screen, PANEL, pygame.Rect(CON_X, CON_Y, CON_W, CON_H),
                         border_radius=10)
        blit(screen, "Console", font_lg, TEXT, CON_X + 14, CON_Y + 6)
        blit(screen, "Tab type · jog: x+ x- y+ y- z+ z-  · set: step 0.5 depth 3 "
                     "scale 1.5 speed 8 · mode force · run · home · stop · quit",
             font_sm, MUTED, CON_X + 96, CON_Y + 10)
        oy2 = CON_Y + 30
        for msg in s['log'][-CON_LINES:]:
            blit(screen, msg[:150], font_sm,
                 GREEN if msg.startswith('>') else TEXT, CON_X + 16, oy2)
            oy2 += 15
        inp_y = CON_Y + CON_H - INP_H - 6
        active = s['mode'] == 'console'
        pygame.draw.rect(screen, CARD if active else (28, 28, 40),
                         pygame.Rect(CON_X + 8, inp_y, CON_W - 16, INP_H), border_radius=6)
        prompt = "console > " if active else "game mode — Tab to type > "
        cursor = "_" if (active and (frame_n // 20) % 2 == 0) else ""
        blit(screen, prompt, font_md, GREEN if active else MUTED, CON_X + 18, inp_y + 7)
        pw = font_md.size(prompt)[0]
        blit(screen, cmd_buf + cursor, font_md, WHITE, CON_X + 18 + pw, inp_y + 7)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Game-style friction trajectory driver")
    ap.add_argument("--tip", default=None, help="calib_<tip>.json in Integration_2")
    ap.add_argument("--no-robot", action="store_true", help="No UR5 (preview only)")
    ap.add_argument("--no-sensor", action="store_true", help="No sensor (demo colours)")
    args = ap.parse_args()

    state    = State()
    stop_evt = threading.Event()
    abort_evt = threading.Event()
    cmd_q    = queue.Queue()
    state.push_log("ready — game mode. Arrows jog, letter selects, SPACE runs.")

    # ── Sensor ────────────────────────────────────────────────
    sensor_mod = None
    if not args.no_sensor:
        import sensor as sensor_mod
        print("[live] Starting sensor ...")
        sensor_mod.start()
        if not sensor_mod.wait_until_ready(timeout=40):
            print("[live] ⚠ Sensor not ready — demo colours")
            sensor_mod = None
        else:
            print("[live] Sensor ready!")

    # ── Robot ─────────────────────────────────────────────────
    rtde_c = None
    if not args.no_robot:
        global BASE_CALIB_Z
        calib_path = select_calibration(args.tip)
        label, _ = apply_calibration(calib_path)
        BASE_CALIB_Z = ur5.CALIB_Z_MM     # surface Z from calib; z_trim adds to it
        print(f"[live] Calibration: {label}  (surface Z = {BASE_CALIB_Z:.2f} mm)")
        print("\n⚠  This drives the REAL robot (arrows jog, SPACE runs a "
              "trajectory that presses the sensor).")
        try:
            input("   Press Enter to connect + home (Ctrl-C to abort) ...")
        except (EOFError, KeyboardInterrupt):
            print("\n[live] Aborted."); sys.exit(0)
        print("[live] Connecting ...")
        rtde_c, _ = connect_robot()
        if rtde_c is None:
            print("[live] Robot connect FAILED — aborting."); sys.exit(1)
        state.set(connected=True)
        threading.Thread(
            target=worker,
            args=(state, stop_evt, abort_evt, rtde_c, cmd_q),
            daemon=True).start()
    else:
        state.set(status="sensor-only (no robot)")

    # ── Render (main thread) ──────────────────────────────────
    try:
        render_loop(state, stop_evt, abort_evt, args, sensor_mod, cmd_q)
    except KeyboardInterrupt:
        abort_evt.set(); stop_evt.set(); cmd_q.put("quit")

    # give the worker a moment to home
    t0 = time.time()
    while not args.no_robot and not state.snap()['done'] and time.time() - t0 < 30:
        time.sleep(0.1)
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
