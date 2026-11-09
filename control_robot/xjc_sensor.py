"""
xjc_sensor.py — XJCSENSOR X-6A-XD80-H28 6-axis F/T sensor over RS485 / Modbus RTU
=================================================================================
Reconstructed, self-contained driver for the XJC force/torque sensor used in the
`XJC force sensor/` calibration work. It reproduces the protocol, CRC, stream
resync, timestamping and CSV export of the reference web monitor
(`xjc_ft_monitor_1.html`); see ../"XJC force sensor"/README_FT_CALIBRATION.md,
sections 3.1–3.3, for the specification and section 3.2 for the test vectors used
by the `__main__` self-test below.

The pure protocol helpers (CRC, frame builders/parsers) have no I/O. The
:class:`XjcSensor` class adds the pyserial transport but accepts any duck-typed
serial object, so it can be driven by a fake port in tests without hardware.

For the indentation collector we use POLL mode: one request → one 29-byte reply
of 6 float32 values (Fx,Fy,Fz in N; Mx,My,Mz in N·m), polled continuously in a
background thread. Stream mode is implemented for completeness.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Iterable, Iterator, List, Optional, Protocol, Sequence

# ── Protocol constants (README §3.1) ────────────────────────────────────────────
POLL_REQUEST_LEN = 8
POLL_REPLY_LEN   = 29
STREAM_FRAME_LEN = 16
STREAM_HEADER    = bytes((0x20, 0x4E))
POLL_FUNCTION    = 0x04
POLL_BYTE_COUNT  = 0x18          # 24 bytes = 6 × float32
STOP_CMD         = bytes([0xFF]) * 50

# rate_code → nominal sample rate (Hz). 1000 Hz needs a higher baud than 115200.
STREAM_RATES = {0: 100, 1: 250, 2: 500, 3: 1000}

# CSV layout — identical to the web monitor so its exports load unchanged.
CSV_COLUMNS: List[str] = (
    ['t_s', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
    + ['raw_Fx', 'raw_Fy', 'raw_Fz', 'raw_Mx', 'raw_My', 'raw_Mz']
)


# ── CRC-16/MODBUS ────────────────────────────────────────────────────────────────
def crc16_modbus(data: bytes) -> bytes:
    """CRC-16/MODBUS of `data`, returned as 2 bytes LOW-byte-first (wire order)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


# ── Poll mode ────────────────────────────────────────────────────────────────────
def build_request(station: int = 1) -> bytes:
    """8-byte poll request: read 12 holding registers (6 float32) from reg 0.

    For station 1 this is the canonical `01 04 00 00 00 0C F0 0F`.
    """
    body = bytes((station, POLL_FUNCTION, 0x00, 0x00, 0x00, 0x0C))
    return body + crc16_modbus(body)


def parse_poll_frame(frame: bytes, station: int = 1) -> Optional[List[float]]:
    """Validate a 29-byte poll reply and return [Fx,Fy,Fz,Mx,My,Mz] or None.

    Checks length, station/function/byte-count header and the CRC (over the first
    27 bytes). Values are 6 big-endian float32 — forces in N, moments in N·m.
    """
    if len(frame) != POLL_REPLY_LEN:
        return None
    if frame[0] != station or frame[1] != POLL_FUNCTION or frame[2] != POLL_BYTE_COUNT:
        return None
    if crc16_modbus(frame[:27]) != frame[27:29]:
        return None
    return list(struct.unpack('>6f', frame[3:27]))


# ── Stream mode (README §3.1) ─────────────────────────────────────────────────────
def build_stream_start(station: int = 1, rate_code: int = 2) -> bytes:
    """11-byte stream-start command for the given rate code (default 500 Hz).

    Writes `cc = rate_code` to register 0x019A: body is
    `<st> 10 01 9A 00 01 02 00 <cc>` followed by the MODBUS CRC.
    """
    if rate_code not in STREAM_RATES:
        raise ValueError(f'rate_code must be one of {sorted(STREAM_RATES)}')
    body = bytes((station, 0x10, 0x01, 0x9A, 0x00, 0x01, 0x02, 0x00, rate_code))
    return body + crc16_modbus(body)


def extract_stream_frames(buffer: bytearray) -> List[List[float]]:
    """Pop every complete, CRC-valid 16-byte stream frame from `buffer`.

    Scans for the `20 4E` header, verifies the CRC over the first 14 bytes, and
    on a bad frame drops one byte and resyncs. Consumed/garbage bytes are removed
    from `buffer` in place; a trailing partial frame is left for the next call.
    Returns a list of [Fx,Fy,Fz,Mx,My,Mz] (forces ÷100 N, moments ÷1000 N·m).
    """
    out: List[List[float]] = []
    i = 0
    n = len(buffer)
    while i + STREAM_FRAME_LEN <= n:
        if buffer[i] != STREAM_HEADER[0] or buffer[i + 1] != STREAM_HEADER[1]:
            i += 1
            continue
        frame = bytes(buffer[i:i + STREAM_FRAME_LEN])
        if crc16_modbus(frame[:14]) != frame[14:16]:
            i += 1
            continue
        ints = struct.unpack('<6h', frame[2:14])
        out.append([ints[0] / 100.0, ints[1] / 100.0, ints[2] / 100.0,
                    ints[3] / 1000.0, ints[4] / 1000.0, ints[5] / 1000.0])
        i += STREAM_FRAME_LEN
    del buffer[:i]
    return out


def _is_finite(value: float) -> bool:
    return value == value and value not in (float('inf'), float('-inf'))


# ── Sample + timing ──────────────────────────────────────────────────────────────
@dataclass
class Sample:
    """One sensor sample. ``values`` are tared (raw - offset); ``raw`` are not."""
    idx:    int
    t_s:    float
    t_pc:   float
    raw:    List[float]
    values: List[float]


class StreamClock:
    """Index-based timestamps for stream mode: frames that arrive in one USB
    burst are spread backwards at the nominal period and anchored to the PC
    clock with light drift correction (README §3.3)."""

    def __init__(self, rate_hz: float, pc_anchor: float, drift_gain: float = 0.01):
        self.dt         = 1.0 / float(rate_hz)
        self.t0_pc      = pc_anchor
        self.drift_gain = drift_gain
        self.idx        = 0

    def stamp_chunk(self, n_frames: int, pc_now: float) -> List[float]:
        stamps = []
        for k in range(n_frames):
            stamps.append(self.idx * self.dt)
            self.idx += 1
        # gentle anchor so the index clock tracks the PC clock over time
        if stamps:
            predicted = self.t0_pc + stamps[-1]
            self.t0_pc += self.drift_gain * (pc_now - predicted)
        return stamps


class SerialLike(Protocol):
    """Minimal duck-typed serial interface (so tests can inject a fake port)."""
    def read(self, size: int) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def reset_input_buffer(self) -> None: ...
    def close(self) -> None: ...


# ── Sensor reader ──────────────────────────────────────────────────────────────
@dataclass
class XjcSensor:
    """Reader for the XJC F/T sensor in poll or stream mode.

    Pass an already-open ``transport`` (e.g. a fake serial port in tests), or
    leave it ``None`` and call :meth:`open` to create a real pyserial port.
    """
    port:           Optional[str] = None
    baud:           int = 115200
    station:        int = 1
    transport:      Optional[SerialLike] = None
    poll_timeout_s: float = 0.05
    offset:         List[float] = field(default_factory=lambda: [0.0] * 6)
    frames:         int = 0
    errors:         int = 0
    rx_bytes:       int = 0

    # ── transport lifecycle ──────────────────────────────────────────────────
    def open(self, timeout_s: float = None) -> 'XjcSensor':
        """Create a real pyserial port on ``self.port`` (115200 8-N-1 default)."""
        import serial  # local import so tests with a fake transport need no pyserial
        if self.port is None:
            raise ValueError('XjcSensor.open() needs a port (set self.port first)')
        self.transport = serial.Serial(
            port     = self.port,
            baudrate = self.baud,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = timeout_s if timeout_s is not None else self.poll_timeout_s,
        )
        return self

    def close(self) -> None:
        if self.transport is not None:
            try:
                self.transport.close()
            except Exception:
                pass
            self.transport = None

    def _require_transport(self) -> SerialLike:
        if self.transport is None:
            raise RuntimeError('XjcSensor has no transport — call open() first')
        return self.transport

    # ── poll ───────────────────────────────────────────────────────────────────
    def poll_once(self) -> Optional[List[float]]:
        """Send one request and return the 6 RAW values, or None on a bad reply."""
        tr = self._require_transport()
        try:
            tr.reset_input_buffer()
        except Exception:
            pass
        tr.write(build_request(self.station))
        deadline = time.perf_counter() + self.poll_timeout_s
        buf = bytearray()
        while len(buf) < POLL_REPLY_LEN and time.perf_counter() < deadline:
            chunk = tr.read(POLL_REPLY_LEN - len(buf))
            if chunk:
                buf.extend(chunk)
            elif not chunk:
                break
        self.rx_bytes += len(buf)
        vals = parse_poll_frame(bytes(buf), self.station)
        if vals is None:
            self.errors += 1
            return None
        self.frames += 1
        return vals

    def poll_samples(self, n: int) -> Iterator[Sample]:
        """Yield up to `n` tared Samples, polling as fast as the link allows."""
        got = 0
        while got < n:
            t_pc = time.perf_counter()
            raw  = self.poll_once()
            if raw is None:
                continue
            yield self._make_sample(got, t_pc, t_pc, raw)
            got += 1

    # ── stream ─────────────────────────────────────────────────────────────────
    def start_stream(self, rate_code: int = 2) -> None:
        tr = self._require_transport()
        tr.write(STOP_CMD)
        time.sleep(0.12)
        try:
            tr.reset_input_buffer()
        except Exception:
            pass
        tr.write(build_stream_start(self.station, rate_code))

    def stop_stream(self) -> None:
        if self.transport is not None:
            self.transport.write(STOP_CMD)

    def stream_samples(self, rate_code: int = 2, duration_s: float = 1.0) -> Iterator[Sample]:
        tr = self._require_transport()
        rate = STREAM_RATES[rate_code]
        self.start_stream(rate_code)
        clock = StreamClock(rate, time.perf_counter())
        buffer = bytearray()
        idx = 0
        t_end = time.perf_counter() + duration_s
        try:
            while time.perf_counter() < t_end:
                chunk = tr.read(STREAM_FRAME_LEN * 8)
                if chunk:
                    buffer.extend(chunk)
                    self.rx_bytes += len(chunk)
                frames = extract_stream_frames(buffer)
                if not frames:
                    continue
                stamps = clock.stamp_chunk(len(frames), time.perf_counter())
                for raw, t_s in zip(frames, stamps):
                    self.frames += 1
                    yield self._make_sample(idx, t_s, time.perf_counter(), raw)
                    idx += 1
        finally:
            self.stop_stream()

    # ── tare ─────────────────────────────────────────────────────────────────
    def tare(self, samples: Iterable[Sample] = None, n: int = 30) -> List[float]:
        """Average `n` raw samples per channel and store them as the tare offset.

        With no `samples` iterable, polls `n` fresh samples from the transport.
        """
        if samples is None:
            samples = list(self.poll_samples(n))
        else:
            samples = list(samples)[:n]
        if not samples:
            return list(self.offset)
        cols = list(zip(*[s.raw for s in samples]))
        self.offset = [sum(c) / len(c) for c in cols]
        return list(self.offset)

    def clear_tare(self) -> None:
        self.offset = [0.0] * 6

    def _make_sample(self, idx: int, t_s: float, t_pc: float, raw: List[float]) -> Sample:
        off = self.offset if len(self.offset) == len(raw) else [0.0] * len(raw)
        values = [r - o for r, o in zip(raw, off)]
        return Sample(idx=idx, t_s=t_s, t_pc=t_pc, raw=list(raw), values=values)


# ── CSV export (web-monitor compatible) ───────────────────────────────────────
class FtCsvWriter:
    """Write samples in the web monitor's CSV layout (t_s zeroed to first row)."""

    def __init__(self, path: str):
        import csv
        self._fh     = open(path, 'w', newline='')
        self._writer = csv.writer(self._fh)
        self._writer.writerow(CSV_COLUMNS)
        self._t0     = None

    def add(self, t_s: float, values: Sequence[float], raw: Sequence[float]) -> None:
        if self._t0 is None:
            self._t0 = t_s
        row = ([round(t_s - self._t0, 4)]
               + [round(v, 6) for v in values]
               + [round(r, 6) for r in raw])
        self._writer.writerow(row)

    def close(self) -> None:
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ── Self-test against the README §3.2 test vectors (no hardware) ────────────────
if __name__ == '__main__':
    def _hx(s):
        return bytes.fromhex(s.replace(' ', ''))

    ok = True

    # Poll request for station 1
    req = build_request(1)
    want = _hx('01 04 00 00 00 0C F0 0F')
    print(f'build_request(1) = {req.hex(" ")}  {"OK" if req == want else "FAIL"}')
    ok &= req == want

    # Poll reply from the manual → 2.1, 2.1, 2.09, 2.1, 2.1, 2.09
    man = _hx('01 04 18 40 06 66 66 40 06 66 66 40 05 C2 8F '
              '40 06 66 66 40 06 66 66 40 05 C2 8F F2 0D')
    vals = parse_poll_frame(man, 1)
    approx = vals is not None and all(abs(a - b) < 1e-2 for a, b in
                                      zip(vals, [2.1, 2.1, 2.09, 2.1, 2.1, 2.09]))
    print(f'parse manual reply = {[round(v,3) for v in vals] if vals else None}  '
          f'{"OK" if approx else "FAIL"}')
    ok &= approx

    # Real captured reply
    real = _hx('01 04 18 3e 0c 50 1b bd 1b 28 d9 3e 87 d1 62 '
               'bb 74 50 bf 3c 2d 09 dc bb 2f b8 f6 e2 54')
    vals = parse_poll_frame(real, 1)
    expect = [0.137, -0.0379, 0.2653, -0.0037, 0.0106, -0.0027]
    approx = vals is not None and all(abs(a - b) < 2e-3 for a, b in zip(vals, expect))
    print(f'parse real reply   = {[round(v,4) for v in vals] if vals else None}  '
          f'{"OK" if approx else "FAIL"}')
    ok &= approx

    # Corrupted frame (flip one bit) → rejected
    bad = bytearray(real)
    bad[5] ^= 0x01
    print(f'corrupted frame rejected = {"OK" if parse_poll_frame(bytes(bad), 1) is None else "FAIL"}')
    ok &= parse_poll_frame(bytes(bad), 1) is None

    # Stream frame from the manual → -0.01, 2.90, 0.01 N and -0.512, 0.002, 0.000 N·m
    sf = _hx('20 4E FF FF 22 01 01 00 00 FE 02 00 00 00 42 3D')
    frames = extract_stream_frames(bytearray(sf))
    expect = [-0.01, 2.90, 0.01, -0.512, 0.002, 0.000]
    approx = len(frames) == 1 and all(abs(a - b) < 1e-3 for a, b in zip(frames[0], expect))
    print(f'extract stream frame = {[round(v,3) for v in frames[0]] if frames else None}  '
          f'{"OK" if approx else "FAIL"}')
    ok &= approx

    print('\nALL TESTS PASSED' if ok else '\nSOME TESTS FAILED')
    raise SystemExit(0 if ok else 1)
