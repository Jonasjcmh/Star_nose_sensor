"""
Robotiq 2F-85 driver for a gripper connected to a UR robot (Tool I/O),
controlled through the Robotiq URCap socket (port 63352 on the robot IP).

Requirements on the robot side:
  - Gripper connected to the Tool I/O and Robotiq URCap installed/activated
  - Robot powered on and reachable from this PC
  - No PolyScope program driving the gripper at the same time

Positions are 0..255 (0 = fully open, 255 = fully closed).
Object status (OBJ register):
  0 = moving, 1 = stopped by contact while opening,
  2 = stopped by contact while closing (object gripped),
  3 = reached requested position (no object).
"""
import socket
import threading
import time

STROKE_MM = 85.0  # Robotiq 2F-85


def pos_to_mm(pos):
    """Approximate finger opening in mm (linear approximation)."""
    return STROKE_MM * (1.0 - pos / 255.0)


def mm_to_pos(mm):
    mm = max(0.0, min(STROKE_MM, mm))
    return int(round(255 * (1.0 - mm / STROKE_MM)))


class RobotiqGripper:
    def __init__(self, ip, port=63352, timeout=2.0):
        self.ip, self.port, self.timeout = ip, port, timeout
        self._sock = None
        self._lock = threading.Lock()

    # ---- connection ----
    def connect(self):
        self._sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)

    def disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            finally:
                self._sock = None

    # ---- low level ----
    def _send(self, text):
        with self._lock:
            self._sock.sendall(text.encode() + b"\n")
            return self._sock.recv(1024).decode().strip()

    def _get(self, var):
        return int(self._send(f"GET {var}").split()[1])

    def _set(self, var, value):
        return self._send(f"SET {var} {int(value)}")

    # ---- high level ----
    def activate(self, timeout=15.0):
        """Reset and activate the gripper (it opens and closes fully)."""
        self._set("ACT", 0)
        time.sleep(0.5)
        self._set("ACT", 1)
        self._set("GTO", 1)
        t0 = time.time()
        while self._get("STA") != 3:  # 3 = activation completed
            if time.time() - t0 > timeout:
                raise TimeoutError("Gripper activation timed out")
            time.sleep(0.2)

    def move(self, pos, speed=150, force=100):
        """Send a move command (non-blocking). pos/speed/force in 0..255."""
        pos, speed, force = (max(0, min(255, int(v))) for v in (pos, speed, force))
        self._set("SPE", speed)
        self._set("FOR", force)
        self._set("POS", pos)
        self._set("GTO", 1)

    def open(self, speed=150, force=100):
        self.move(0, speed, force)

    def close(self, speed=150, force=100):
        self.move(255, speed, force)

    def get_state(self):
        obj = self._get("OBJ")
        return {
            "position": self._get("POS"),
            "requested": self._get("PRE"),
            "obj_status": obj,
            "moving": obj == 0,
            "object_detected": obj in (1, 2),
            "activated": self._get("STA") == 3,
            "fault": self._get("FLT"),
        }

    def move_and_wait(self, pos, speed=150, force=100, timeout=10.0):
        """Blocking move. Returns the final state dict."""
        self.move(pos, speed, force)
        time.sleep(0.15)
        t0 = time.time()
        while True:
            st = self.get_state()
            if not st["moving"]:
                return st
            if time.time() - t0 > timeout:
                raise TimeoutError("Gripper move timed out")
            time.sleep(0.05)


class SimulatedGripper:
    """Same interface as RobotiqGripper, no hardware needed (for GUI testing)."""

    def __init__(self, object_pos=None):
        self.position = 0.0
        self.requested = 0
        self.speed = 150
        self.object_pos = object_pos  # closing stops here if set (simulated object)
        self._t = time.time()
        self._obj = 3
        self._active = False

    def connect(self):
        pass

    def disconnect(self):
        pass

    def activate(self, timeout=15.0):
        self._active = True
        self.position = 0.0
        self.requested = 0

    def move(self, pos, speed=150, force=100):
        self.requested = max(0, min(255, int(pos)))
        self.speed = int(speed)
        self._obj = 0

    def open(self, speed=150, force=100):
        self.move(0, speed, force)

    def close(self, speed=150, force=100):
        self.move(255, speed, force)

    def _step(self):
        now = time.time()
        dt, self._t = now - self._t, now
        if self._obj != 0:
            return
        rate = 40 + self.speed * 2  # counts per second
        delta = self.requested - self.position
        step = max(-rate * dt, min(rate * dt, delta))
        new = self.position + step
        if self.object_pos is not None and self.position <= self.object_pos < new:
            self.position = float(self.object_pos)
            self._obj = 2  # stopped by object while closing
            return
        self.position = new
        if abs(self.requested - self.position) < 0.5:
            self.position = float(self.requested)
            self._obj = 3

    def get_state(self):
        self._step()
        return {
            "position": int(round(self.position)),
            "requested": self.requested,
            "obj_status": self._obj,
            "moving": self._obj == 0,
            "object_detected": self._obj in (1, 2),
            "activated": self._active,
            "fault": 0,
        }

    def move_and_wait(self, pos, speed=150, force=100, timeout=10.0):
        self.move(pos, speed, force)
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self.get_state()
            if not st["moving"]:
                return st
            time.sleep(0.02)
        raise TimeoutError("Simulated move timed out")
