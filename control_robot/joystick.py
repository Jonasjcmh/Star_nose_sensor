"""
joystick.py — Xbox / gamepad analog reader for the control console
==================================================================
A small background reader (pygame, headless via the SDL "dummy" video driver)
that exposes the two analog sticks as a normalised velocity-fraction vector:

    left stick  → X (axis 0) and Y (axis 1)
    right stick → Z (axis 4, vertical)

Values are in [-1, 1] after a dead-zone and an optional squared response curve
(finer control near centre, full speed at full deflection). Hot-plug is handled:
unplug → zeros + ``connected() == False``; replug is picked back up.

Verified against an "Xbox One S Controller": 6 axes, sticks on 0/1/3/4, triggers
on 2/5 (which rest at -1, so they are never mistaken for a centred stick).
"""

from __future__ import annotations

import os
import math
import time
import threading

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')   # no window; joystick only

try:
    import pygame
    HAVE_PYGAME = True
except Exception as _e:   # pragma: no cover
    print(f'[joy] pygame unavailable ({_e}) — joystick disabled')
    HAVE_PYGAME = False


def _deadzone(v, d):
    """Zero out |v|<d and rescale the remainder to span the full 0..1 range."""
    if abs(v) < d:
        return 0.0
    return (v - math.copysign(d, v)) / (1.0 - d)


class JoystickReader:
    """Polls gamepad 0 in a daemon thread; :meth:`get` returns [fx, fy, fz]."""

    def __init__(self, deadzone=0.15, expo=True,
                 ax_x=0, ax_y=1, ax_z=4,
                 inv_x=False, inv_y=True, inv_z=True, poll_hz=100,
                 ax_lt=2, ax_rt=5, trig_deadzone=0.05, inv_rz=False):
        self.deadzone = deadzone
        self.expo = expo
        self.ax = (ax_x, ax_y, ax_z)
        self.inv = (-1.0 if inv_x else 1.0, -1.0 if inv_y else 1.0,
                    -1.0 if inv_z else 1.0)
        self.ax_lt = ax_lt
        self.ax_rt = ax_rt
        self.trig_deadzone = trig_deadzone
        self.inv_rz = -1.0 if inv_rz else 1.0
        self.poll_dt = 1.0 / poll_hz
        # [fx, fy, fz, rz]  — rz from the triggers (RT − LT)
        self._vec = [0.0, 0.0, 0.0, 0.0]
        self._lock = threading.Lock()
        self._connected = False
        self._name = None
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if not HAVE_PYGAME:
            return self
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def _response(self, v):
        return v * abs(v) if self.expo else v   # squared magnitude, keep sign

    def _trigger(self, raw):
        """Map a trigger axis (rests at -1, pressed +1) to 0..1 with a dead-zone."""
        t = (raw + 1.0) / 2.0
        return 0.0 if t < self.trig_deadzone else (t - self.trig_deadzone) / (1.0 - self.trig_deadzone)

    def _loop(self):
        try:
            pygame.init()
            pygame.joystick.init()
        except Exception as e:
            print(f'[joy] init failed ({e}) — joystick disabled')
            return
        js = None
        while not self._stop.is_set():
            # (re)open device 0 on hot-plug
            if js is None:
                try:
                    if pygame.joystick.get_count() > 0:
                        js = pygame.joystick.Joystick(0)
                        js.init()
                        self._name = js.get_name()
                        self._connected = True
                        print(f'[joy] connected: {self._name} ({js.get_numaxes()} axes)')
                    else:
                        self._connected = False
                        with self._lock:
                            self._vec = [0.0, 0.0, 0.0]
                        # re-scan for hot-plug
                        pygame.joystick.quit()
                        pygame.joystick.init()
                        time.sleep(0.5)
                        continue
                except Exception:
                    time.sleep(0.5)
                    continue
            try:
                pygame.event.pump()
                n = js.get_numaxes()
                raw = [js.get_axis(a) for a in self.ax]
                # triggers → Z rotation (RT − LT); absent on pads with < 6 axes
                lt = self._trigger(js.get_axis(self.ax_lt)) if self.ax_lt < n else 0.0
                rt = self._trigger(js.get_axis(self.ax_rt)) if self.ax_rt < n else 0.0
            except Exception:
                js = None
                self._connected = False
                with self._lock:
                    self._vec = [0.0, 0.0, 0.0, 0.0]
                time.sleep(0.3)
                continue
            vec = [self._response(_deadzone(r, self.deadzone)) * s
                   for r, s in zip(raw, self.inv)]
            vec.append((rt - lt) * self.inv_rz)
            with self._lock:
                self._vec = vec
            time.sleep(self.poll_dt)

    def get(self):
        with self._lock:
            return list(self._vec)

    def connected(self):
        return self._connected

    def name(self):
        return self._name

    def stop(self):
        self._stop.set()
