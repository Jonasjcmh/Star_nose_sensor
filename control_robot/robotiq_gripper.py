"""
robotiq_gripper.py — Robotiq 2-finger gripper client over the UR URCap socket
=============================================================================
Drives a Robotiq 2F-85 / 2F-140 that is plugged into a Universal Robots arm
and running the Robotiq_Grippers URCap. The URCap exposes a tiny ASCII socket
server on the robot controller at TCP port 63352; we talk to it with the usual
``SET <var> <val>`` / ``GET <var>`` commands (ACT, GTO, POS, SPE, FOR, …).

No pyserial, no Modbus: the gripper shares the robot's IP. Connect to the SAME
IP you give rtde_control. This is the standard interface used by the official
Universal_Robots ur_rtde example of the same name.

Positions/speeds/forces are the raw 0-255 gripper units (0 = open, 255 = closed
for a 2F-85). :meth:`move_and_wait_for_pos` blocks until motion finishes and
returns the final position + an :class:`ObjectStatus`.
"""

from __future__ import annotations

import socket
import threading
import time
from enum import Enum
from typing import Optional, Tuple, Union, OrderedDict


class RobotiqGripper:
    """Communicates with a Robotiq gripper via the UR URCap socket (port 63352)."""

    # WRITE variables (device set-able)
    ACT = 'ACT'   # activation (1 = activate, 0 = reset)
    GTO = 'GTO'   # go-to (1 = follow POS/SPE/FOR, 0 = stop)
    ATR = 'ATR'   # automatic release (emergency slow open)
    ADR = 'ADR'   # automatic release direction (0 = open, 1 = close)
    FOR = 'FOR'   # force  (0-255)
    SPE = 'SPE'   # speed  (0-255)
    POS = 'POS'   # target position (0-255)
    # READ variables (status)
    STA = 'STA'   # gripper status (see GripperStatus)
    PRE = 'PRE'   # position request echo
    OBJ = 'OBJ'   # object detection status (see ObjectStatus)
    FLT = 'FLT'   # fault status

    ENCODING = 'UTF-8'

    class GripperStatus(Enum):
        RESET = 0        # gripper is in reset / automatic release
        ACTIVATING = 1   # calibration / activation in progress
        # 2 is not used
        ACTIVE = 3       # activation completed, ready

    class ObjectStatus(Enum):
        MOVING = 0                 # fingers are moving toward the target
        STOPPED_OUTER_OBJECT = 1   # stopped early — object while opening
        STOPPED_INNER_OBJECT = 2   # stopped early — object while closing
        AT_DEST = 3                # reached the requested position

    def __init__(self):
        self.socket: Optional[socket.socket] = None
        self.command_lock = threading.Lock()
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255

    # ── connection ─────────────────────────────────────────────────────────────
    def connect(self, hostname: str, port: int = 63352, socket_timeout: float = 2.0) -> None:
        """Open the TCP socket to the gripper server on the robot controller."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.socket.settimeout(socket_timeout)

    def disconnect(self) -> None:
        if self.socket is not None:
            try:
                self.socket.close()
            finally:
                self.socket = None

    # ── low-level variable access ──────────────────────────────────────────────
    def _set_vars(self, var_dict: "OrderedDict[str, Union[int, float]]") -> bool:
        """Send one ``SET`` with several variables; returns True on ``ack``."""
        cmd = 'SET'
        for variable, value in var_dict.items():
            cmd += f' {variable} {int(value)}'
        cmd += '\n'
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return self._is_ack(data)

    def _set_var(self, variable: str, value: Union[int, float]) -> bool:
        from collections import OrderedDict as _OD
        return self._set_vars(_OD([(variable, value)]))

    def _get_var(self, variable: str) -> int:
        """Send ``GET <var>`` and return the integer value in the reply."""
        with self.command_lock:
            cmd = f'GET {variable}\n'
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError(f'Unexpected response {data!r}: expected {variable}')
        return int(value_str)

    @staticmethod
    def _is_ack(data: bytes) -> bool:
        return data == b'ack'

    # ── activation / reset ─────────────────────────────────────────────────────
    def _reset(self) -> None:
        self._set_var(self.ACT, 0)
        self._set_var(self.ATR, 0)
        while self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0:
            self._set_var(self.ACT, 0)
            self._set_var(self.ATR, 0)
            time.sleep(0.01)
        time.sleep(0.5)

    def activate(self, auto_calibrate: bool = False) -> None:
        """Reset then activate the gripper, blocking until it is ACTIVE.

        With ``auto_calibrate`` it also drives the fingers fully closed/open to
        learn the true 0-255 travel limits for this model and payload.
        """
        if not self.is_active():
            self._reset()
            while self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0:
                time.sleep(0.01)
            self._set_var(self.ACT, 1)
            time.sleep(1.0)
            while self._get_var(self.ACT) != 1 or self._get_var(self.STA) != 3:
                time.sleep(0.01)
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self) -> bool:
        return RobotiqGripper.GripperStatus(self._get_var(self.STA)) == \
            RobotiqGripper.GripperStatus.ACTIVE

    # ── limits ─────────────────────────────────────────────────────────────────
    def get_min_position(self) -> int:
        return self._min_position

    def get_max_position(self) -> int:
        return self._max_position

    def get_open_position(self) -> int:
        return self._min_position

    def get_closed_position(self) -> int:
        return self._max_position

    def is_open(self) -> bool:
        return self.get_current_position() <= self._min_position

    def is_closed(self) -> bool:
        return self.get_current_position() >= self._max_position

    def get_current_position(self) -> int:
        return self._get_var(self.POS)

    # ── motion ─────────────────────────────────────────────────────────────────
    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """Send a non-blocking move to clipped ``position`` at ``speed``/``force``.

        Returns (command_accepted, clipped_target_position).
        """
        def clip(v, lo, hi):
            return max(lo, min(hi, int(v)))

        clip_pos = clip(position, self._min_position, self._max_position)
        clip_spe = clip(speed, self._min_speed, self._max_speed)
        clip_for = clip(force, self._min_force, self._max_force)

        from collections import OrderedDict as _OD
        var_dict = _OD([(self.POS, clip_pos), (self.SPE, clip_spe),
                        (self.FOR, clip_for), (self.GTO, 1)])
        return self._set_vars(var_dict), clip_pos

    def move_and_wait_for_pos(self, position: int, speed: int, force: int
                              ) -> Tuple[int, "RobotiqGripper.ObjectStatus"]:
        """Move, then block until the fingers stop; return (position, ObjectStatus)."""
        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError('Gripper did not ack the move command')

        # wait until the request register echoes our target (command registered)
        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)

        # wait while the object status reports MOVING
        obj = RobotiqGripper.ObjectStatus(self._get_var(self.OBJ))
        while obj == RobotiqGripper.ObjectStatus.MOVING:
            obj = RobotiqGripper.ObjectStatus(self._get_var(self.OBJ))
            time.sleep(0.001)
        return self.get_current_position(), obj

    def auto_calibrate(self, log: bool = True) -> None:
        """Fully close then open to discover this gripper's travel limits."""
        pos, obj = self.move_and_wait_for_pos(self.get_closed_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(obj) != RobotiqGripper.ObjectStatus.AT_DEST:
            self._max_position = pos
        pos, obj = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(obj) != RobotiqGripper.ObjectStatus.AT_DEST:
            self._min_position = pos
        if log:
            print(f'[gripper] calibrated: open={self._min_position} '
                  f'closed={self._max_position}')
