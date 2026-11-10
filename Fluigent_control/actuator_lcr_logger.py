#!/usr/bin/env python3
"""
actuator_lcr_logger.py — inflatable-actuator protocols with LCR data logging
===========================================================================

Drives a Fluigent Flow EZ (pressure) + P-Switch (8 valve outputs) through a
protocol of pressure set-points, linear inflation/deflation ramps, cycles and
staircases, choosing which P-Switch outputs (= which actuators) are open in
every step.  At the same time it logs, at a fixed rate, to one CSV:

    commanded set-point, measured Flow EZ pressure, open valves,
    LCR-6100 reading (mode chosen on the command line, verified on connect)

Easiest: just run it
--------------------
    python -W ignore actuator_lcr_logger.py

With no arguments the script asks every option one question at a time
(simulate?, ceiling, sweep or JSON file, valves, set-points, timings, name,
LCR mode / frequency / level / speed / port, log rate, vent time, analysis).
Enter keeps the value shown in [brackets] — your previous answers, kept in
wizard_last.json — and invalid answers are asked again.  At the end it
prints the equivalent command line, shows the schedule and asks to start.

Two ways to describe what to do on the command line
---------------------------------------------------
1. A JSON protocol file (full control, see protocols/example_actuators.json):

       python actuator_lcr_logger.py --protocol protocols/example_actuators.json

2. Quick sweep from the command line (no file needed):

       python actuator_lcr_logger.py --valves "A1;A2;A1+A2" \
           --setpoints 100,200,300 --ramp 2 --hold 5 --rest 5 --cycles 3

   = for every valve combination, for every set-point: `cycles` x
     [ramp 0->sp, hold, ramp sp->0, rest].  Actuator names come from
     --actuators "A1=out1,A2=out2" (default names out1..out8).

Useful flags
------------
    --dry-run      print the resolved schedule and total time, touch nothing
    --simulate     run everything against simulated Fluigent + LCR (tests the
                   logging pipeline at your desk)
    --measure capacitance | impedance
                   LCR mode: Cp-D (default) or Z-theta in degrees
    --no-lcr       pressure only
    --lcr-port /dev/tty.usbmodemXXXX   (otherwise asked interactively)
    --lcr-func Cp-Rp --lcr-freq 20000 --lcr-volt 1 --lcr-speed FAST
    --max-pressure 500   safety ceiling in mbar (also capped by the device)
    --log-rate 50        CSV rows per second
    --repeat 2           run the whole protocol twice

Output (logs/<name>_<YYYYmmdd_HHMMSS>/)
--------------------------------------
    data.csv        one row per log tick (see CSV_COLUMNS)
    protocol.json   the protocol exactly as run (re-run it with --protocol)
    meta.json       LCR mode read-back + IDN, Fluigent channel info, timing,
                    whether the run finished or was stopped
  + automatic analysis at the end (analyze_actuator_session.py):
    timeseries.png       pressure, Z (or Cp) and theta (or D) vs time
    lcr_vs_pressure.png  Z / theta vs pressure, inflation vs deflation
    summary.csv          settled mean of every hold + change vs baseline
    (--show-plots opens the figures, --no-analysis skips it)

Protocol JSON
-------------
{
  "name": "my_test",
  "actuators": {"A1": "out1", "A2": "out2"},   # optional aliases
  "max_pressure_mbar": 400,                     # optional, CLI overrides
  "repeat": 1,
  "steps": [
    {"type": "hold",  "pressure": 0,   "duration": 5,  "valves": []},
    {"type": "ramp",  "to": 300,       "duration": 3,  "valves": ["A1"]},
    {"type": "hold",  "pressure": 300, "duration": 5},
    {"type": "ramp",  "from": 300, "to": 0, "duration": 3},
    {"type": "cycle", "low": 0, "high": 250, "ramp_up": 2, "hold_high": 3,
                      "ramp_down": 2, "hold_low": 3, "cycles": 5,
                      "valves": ["A1", "A2"]},
    {"type": "staircase", "levels": [50, 100, 150], "ramp": 1, "hold": 4,
                      "rest": 3, "valves": "all"}
  ]
}
  * "valves" sets EXACTLY which outputs are open for that step ([] = all
    closed, "all" = all open).  Omitted -> unchanged from previous step.
    Names: out1..out8, an alias from "actuators", or the integer 1..8.
  * "from" omitted -> starts from the current set-point.
  * staircase: "levels" list, or "from"/"to"/"n"; "rest" (optional) returns
    to 0 mbar between levels for that many seconds.
  * every step accepts an optional "label".

Asynchronous valves ("valve_timeline")
--------------------------------------
Instead of "valves" inside the steps, a protocol can give every valve its
own timeline, independent of the pressure steps (times in s from the start
of each run / repeat):

  "valve_timeline": [
    {"valves": "A1", "delay": 0,   "on": 2, "off": 1, "pulses": 0},
    {"valves": "A2", "delay": 1.5, "on": 2, "off": 1, "pulses": 4},
    {"valves": "A3", "delay": 6},                      # open at 6 s, stay open
    {"valves": "A4", "delay": 2, "until": 9},          # open 2..9 s
    {"t": 12, "close": "all"},                         # one-off event
    {"t": 13, "open": "A1+A2"}
  ]
  * pulse train: open "on" s, closed "off" s, starting at "delay";
    "pulses": N trains, 0 = repeat until the end of the pressure profile.
  * no "on": the valve opens at "delay" and stays open (or until "until").
  * all valves share the one Flow EZ pressure; the timeline decides which
    actuators receive it at every moment.
  * steps must not also set "valves" when a valve_timeline is given.

Safety
------
Every pressure is checked against the ceiling before anything moves.  On
the end of the protocol, Ctrl-C or any error: pressure -> 0 mbar with the
valves left as they were for --vent-time seconds (so inflated actuators
deflate through the Flow EZ), then all valves closed and the session closed.
"""

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'Capacitance_measurement'))   # lcr6100.py

FGT_SDK_PATH = os.environ.get(
    'FGT_SDK_PATH', '/Users/jonathantirado/Documents/GitHub/fgt-SDK/Python')

N_OUTPUTS = 8
ON_POSITION, OFF_POSITION = 1, 0
DEFAULT_MAX_PRESSURE = 500.0   # mbar — conservative default ceiling
LOG_DIR = os.path.join(HERE, 'logs')

# --measure presets -> LCR-6100 function
MEASURE_MODES = {'capacitance': 'Cp-D', 'impedance': 'Z-thd'}


# ═══════════════════════════════════════════════════════════════════════════
#  Protocol → flat list of segments
# ═══════════════════════════════════════════════════════════════════════════

class ProtocolError(ValueError):
    pass


def valve_resolver(actuators=None):
    """Return f(spec) -> sorted tuple of 0-based P-Switch indices."""
    names = {f'out{i + 1}': i for i in range(N_OUTPUTS)}
    for alias, target in (actuators or {}).items():
        t = str(target).lower()
        if t not in names:
            raise ProtocolError(f"actuator '{alias}' -> '{target}': target must be out1..out8")
        names[str(alias).lower()] = names[t]

    def resolve(spec):
        if spec is None:
            return None
        if isinstance(spec, str):
            if spec.strip().lower() == 'all':
                return tuple(range(N_OUTPUTS))
            spec = [s for s in spec.replace('+', ',').split(',') if s.strip()]
        out = set()
        for v in spec:
            if isinstance(v, int) or (isinstance(v, str) and v.strip().isdigit()):
                n = int(v)
                if not 1 <= n <= N_OUTPUTS:
                    raise ProtocolError(f'valve number {n} out of range 1..{N_OUTPUTS}')
                out.add(n - 1)
            else:
                key = str(v).strip().lower()
                if key not in names:
                    raise ProtocolError(f"unknown valve/actuator '{v}'. Known: "
                                        + ', '.join(sorted(names)))
                out.add(names[key])
        return tuple(sorted(out))
    return resolve


def valve_label(idx_tuple, actuators=None):
    if not idx_tuple:
        return 'none'
    rev = {}
    for alias, target in (actuators or {}).items():
        rev.setdefault(int(str(target).lower().replace('out', '')) - 1, alias)
    return '+'.join(rev.get(i, f'out{i + 1}') for i in idx_tuple)


def expand_protocol(protocol):
    """
    Turn the protocol dict into a list of segments:
        dict(step, label, type, phase, cycle, valves, p0, p1, duration)
    valves is a tuple of open outputs or None (= unchanged).
    """
    resolve = valve_resolver(protocol.get('actuators'))
    steps = protocol.get('steps')
    if not steps:
        raise ProtocolError('protocol has no steps')

    segs = []
    sp = 0.0

    def seg(i, step, phase, p0, p1, dur, valves=None, cycle=0):
        if dur < 0:
            raise ProtocolError(f'step {i}: negative duration')
        segs.append(dict(step=i, label=step.get('label', f"{step['type']} {i}"),
                         type=step['type'], phase=phase, cycle=cycle,
                         valves=valves, p0=float(p0), p1=float(p1),
                         duration=float(dur)))

    for i, step in enumerate(steps):
        t = step.get('type')
        valves = resolve(step.get('valves'))
        try:
            if t == 'hold':
                p = float(step.get('pressure', sp))
                seg(i, step, 'hold', p, p, step['duration'], valves)
                sp = p
            elif t == 'ramp':
                p0 = float(step.get('from', sp))
                p1 = float(step['to'])
                seg(i, step, 'inflate' if p1 >= p0 else 'deflate',
                    p0, p1, step['duration'], valves)
                sp = p1
            elif t == 'cycle':
                lo = float(step.get('low', 0.0))
                hi = float(step['high'])
                for c in range(int(step.get('cycles', 1))):
                    v = valves if c == 0 else None
                    seg(i, step, 'inflate', lo, hi, step.get('ramp_up', 1.0), v, c)
                    seg(i, step, 'hold_high', hi, hi, step.get('hold_high', 0.0), None, c)
                    seg(i, step, 'deflate', hi, lo, step.get('ramp_down', 1.0), None, c)
                    seg(i, step, 'hold_low', lo, lo, step.get('hold_low', 0.0), None, c)
                sp = lo
            elif t == 'staircase':
                if 'levels' in step:
                    levels = [float(x) for x in step['levels']]
                else:
                    a, b, n = float(step['from']), float(step['to']), int(step['n'])
                    levels = [a + (b - a) * k / max(n - 1, 1) for k in range(n)]
                ramp = float(step.get('ramp', 0.0))
                rest = step.get('rest')
                for c, lvl in enumerate(levels):
                    v = valves if c == 0 else None
                    seg(i, step, 'inflate' if lvl >= sp else 'deflate',
                        sp, lvl, ramp, v, c)
                    seg(i, step, 'hold', lvl, lvl, step['hold'], None, c)
                    sp = lvl
                    if rest is not None:
                        seg(i, step, 'deflate', lvl, 0.0, ramp, None, c)
                        seg(i, step, 'rest', 0.0, 0.0, float(rest), None, c)
                        sp = 0.0
            else:
                raise ProtocolError(f"step {i}: unknown type '{t}' "
                                    "(hold | ramp | cycle | staircase)")
        except KeyError as e:
            raise ProtocolError(f"step {i} ({t}): missing field {e}") from None
    # drop zero-length holds created by optional fields (keep 0-s ramps = steps)
    return [s for s in segs if s['duration'] > 0 or s['p0'] != s['p1']
            or s['valves'] is not None]


def expand_valve_timeline(protocol, total_s):
    """
    protocol['valve_timeline'] -> sorted list of events (t, valve_index, open)
    within one run of length total_s.  [] when there is no timeline.
    """
    items = protocol.get('valve_timeline') or []
    if not items:
        return []
    if any(st.get('valves') is not None for st in protocol.get('steps', [])):
        raise ProtocolError('use either "valves" inside the steps or a '
                            '"valve_timeline", not both')
    resolve = valve_resolver(protocol.get('actuators'))
    ev = []
    for n, it in enumerate(items):
        try:
            if 't' in it:                                   # one-off event
                t = float(it['t'])
                for key, on in (('close', False), ('open', True)):
                    if key in it:
                        for i in resolve(it[key]):
                            ev.append((t, i, on))
                continue
            idx = resolve(it['valves'])
            if not idx:
                raise ProtocolError('opens no valve')
            delay = float(it.get('delay', 0.0))
            if 'on' in it:
                on_s, off_s = float(it['on']), float(it.get('off', 0.0))
                if on_s <= 0 or off_s < 0:
                    raise ProtocolError('"on" must be > 0 and "off" >= 0')
                pulses = int(it.get('pulses', 0))
                end = float(it.get('until', total_s))
                t, k = delay, 0
                while t < end - 1e-9 and (pulses == 0 or k < pulses):
                    for i in idx:
                        ev.append((t, i, True))
                        ev.append((min(t + on_s, end), i, False))
                    t += on_s + off_s
                    k += 1
                    if on_s + off_s <= 0:
                        break
            else:
                for i in idx:
                    ev.append((delay, i, True))
                    if 'until' in it:
                        ev.append((float(it['until']), i, False))
        except KeyError as e:
            raise ProtocolError(f'valve_timeline item {n + 1}: missing field {e}') from None
        except ProtocolError as e:
            raise ProtocolError(f'valve_timeline item {n + 1}: {e}') from None
    for t, _, _ in ev:
        if t < 0:
            raise ProtocolError('valve_timeline: negative time')
    ev = [e for e in ev if e[0] < total_s - 1e-9 or not e[2]]
    ev.sort(key=lambda e: (e[0], e[2]))                # closes before opens
    return ev


def valve_state_at(events, t):
    """Set of open valve indices at time t (for previews)."""
    open_ = set()
    for te, i, on in events:
        if te > t:
            break
        (open_.add if on else open_.discard)(i)
    return open_


def print_valve_events(events, actuators=None):
    if not events:
        return
    print('\n  valve timeline (s from the start of each run):')
    # compact: time -> open set after all changes at that instant
    state, line, k = set(), [], 0
    while k < len(events):
        t = events[k][0]
        while k < len(events) and events[k][0] == t:
            _, i, on = events[k]
            (state.add if on else state.discard)(i)
            k += 1
        line.append(f'{t:7.2f}  open: {valve_label(tuple(sorted(state)), actuators)}')
    for ln in line[:40]:
        print('   ' + ln)
    if len(line) > 40:
        print(f'   … {len(line) - 40} more changes')


def validate(segs, max_p, min_p=0.0):
    for s in segs:
        for p in (s['p0'], s['p1']):
            if not (min_p <= p <= max_p):
                raise ProtocolError(
                    f"step {s['step']} '{s['label']}' ({s['phase']}) asks for "
                    f"{p:g} mbar, outside the allowed {min_p:g}..{max_p:g} mbar")


def sweep_protocol(args):
    """Build a protocol dict from the quick-sweep CLI options."""
    combos = [c.strip() for c in args.valves.split(';') if c.strip()]
    setpoints = [float(x) for x in args.setpoints.split(',') if x.strip()]
    steps = [{'type': 'hold', 'pressure': 0, 'duration': args.baseline,
              'valves': [], 'label': 'baseline (all closed, 0 mbar)'}]
    for combo in combos:
        for spt in setpoints:
            steps.append({'type': 'cycle', 'label': f'{combo} @ {spt:g} mbar',
                          'valves': combo, 'low': 0, 'high': spt,
                          'ramp_up': args.ramp, 'hold_high': args.hold,
                          'ramp_down': args.ramp, 'hold_low': args.rest,
                          'cycles': args.cycles})
    steps.append({'type': 'hold', 'pressure': 0, 'duration': args.baseline,
                  'valves': [], 'label': 'final baseline'})
    return {'name': args.name or 'sweep', 'actuators': parse_actuators(args.actuators),
            'steps': steps}


def parse_actuators(text):
    if not text:
        return {}
    out = {}
    for pair in text.split(','):
        if pair.strip():
            k, _, v = pair.partition('=')
            out[k.strip()] = v.strip()
    return out


def print_schedule(segs, actuators=None, repeat=1):
    t = 0.0
    print(f"{'t (s)':>8} {'dur':>6}  {'step':>4}  {'phase':<10} {'mbar':>15}  "
          f"{'valves':<14} label")
    for s in segs:
        v = '' if s['valves'] is None else valve_label(s['valves'], actuators)
        p = (f"{s['p0']:.0f}" if s['p0'] == s['p1']
             else f"{s['p0']:.0f} -> {s['p1']:.0f}")
        print(f"{t:8.1f} {s['duration']:6.1f}  {s['step']:>4}  {s['phase']:<10} "
              f"{p:>15}  {v:<14} {s['label']}"
              + (f"  [cycle {s['cycle']}]" if s['type'] in ('cycle', 'staircase') else ''))
        t += s['duration']
    print(f"{'total':>8} {t:6.1f} s per run"
          + (f"  x{repeat} = {t * repeat:.1f} s" if repeat > 1 else ''))
    return t


# ═══════════════════════════════════════════════════════════════════════════
#  Hardware back-ends (real + simulated)
# ═══════════════════════════════════════════════════════════════════════════

class FluigentRig:
    """Thin, thread-safe wrapper over the Fluigent SDK."""

    def __init__(self, pressure_index=0):
        sys.path.insert(0, FGT_SDK_PATH)
        import Fluigent.SDK as fgt
        self.fgt = fgt
        self.p_idx = pressure_index
        self.lock = threading.Lock()
        self.valve_indices = list(range(N_OUTPUTS))
        self.setpoint, self.open_set, self.info, self.n_valves = 0.0, set(), {}, N_OUTPUTS

    def open(self):
        fgt = self.fgt
        fgt.fgt_init()
        n_p = fgt.fgt_get_pressureChannelCount()
        n_v = fgt.fgt_get_valveChannelCount()
        if self.p_idx >= n_p:
            raise RuntimeError(f'pressure channel {self.p_idx} not found ({n_p} detected)')
        if n_v < N_OUTPUTS:
            print(f'[fgt] WARNING: only {n_v} valve channel(s) detected; '
                  f'outputs above {n_v} will be refused')
        self.n_valves = n_v
        pmin, pmax = fgt.fgt_get_pressureRange(self.p_idx)
        info = {'pressure_channels': n_p, 'valve_channels': n_v,
                'pressure_range_mbar': [pmin, pmax]}
        try:
            info['pressure_channel_info'] = str(fgt.fgt_get_pressureChannelsInfo())
            info['valve_channel_info'] = str(fgt.fgt_get_valveChannelsInfo())
        except Exception:
            pass
        self.info = info
        return info

    def set_pressure(self, mbar):
        with self.lock:
            self.fgt.fgt_set_pressure(self.p_idx, mbar)
        self.setpoint = mbar

    def get_pressure(self):
        with self.lock:
            return float(self.fgt.fgt_get_pressure(self.p_idx))

    def set_valves(self, open_set):
        for i in range(min(N_OUTPUTS, self.n_valves)):
            with self.lock:
                self.fgt.fgt_set_valvePosition(
                    i, ON_POSITION if i in open_set else OFF_POSITION)
        self.open_set = set(open_set)
        bad = [i for i in open_set if i >= self.n_valves]
        if bad:
            raise RuntimeError(f'valve output(s) {[b + 1 for b in bad]} not connected')

    def close(self):
        with self.lock:
            self.fgt.fgt_close()


class SimRig:
    """First-order pressure response, no hardware."""

    def __init__(self, tau_s=0.15):
        self.tau = tau_s
        self.sp = 0.0
        self.p = 0.0
        self.t = time.monotonic()
        self.open_set = set()
        self.lock = threading.Lock()

    def open(self):
        self.info = {'simulated': True, 'pressure_range_mbar': [0, 2000],
                     'valve_channels': N_OUTPUTS}
        return self.info

    @property
    def setpoint(self):
        return self.sp

    def _advance(self):
        now = time.monotonic()
        a = 1.0 - math.exp(-(now - self.t) / self.tau)
        self.p += (self.sp - self.p) * a
        self.t = now

    def set_pressure(self, mbar):
        with self.lock:
            self._advance()
            self.sp = mbar

    def get_pressure(self):
        with self.lock:
            self._advance()
            return self.p + 0.3 * math.sin(self.t * 50)

    def set_valves(self, open_set):
        self.open_set = set(open_set)

    def close(self):
        pass


class SimLCR:
    """Fake LCR: sensor capacitance rises with pressure × open actuators.
    Reports Cp-D / Cp-Rp / Z-theta etc. consistently for that capacitance."""

    function = 'Cp-Rp'
    labels = ('Cp', 'Rp')
    freq_hz = 20000.0

    def __init__(self, rig):
        self.rig, self.n = rig, 0
        self.applied_config = {'function': self.function, 'simulated': True}

    def connect(self):
        pass

    def start_polling(self):
        pass

    def describe(self):
        return f'Mode: {self.function} | 20 kHz (simulated)'

    def idn(self):
        return 'SIMULATED'

    def get_sample(self):
        self.n += 1
        p = self.rig.get_pressure() if self.rig else 0.0
        n_open = len(getattr(self.rig, 'open_set', ()))
        cp = (1.80 + 0.0012 * p * n_open) * 1e-12 * (1 + 0.0005 * math.sin(self.n))
        rp = 5e6
        w = 2 * math.pi * self.freq_hz
        y = complex(1 / rp, w * cp)
        z = 1 / y
        f = self.function
        if f.startswith('Z'):
            prim = abs(z)
            ang = math.atan2(z.imag, z.real)
            sec = {'Z-thd': math.degrees(ang), 'Z-thr': ang,
                   'Z-D': -z.real / z.imag, 'Z-Q': -z.imag / z.real}.get(f, 0.0)
        elif f == 'Cp-D':
            prim, sec = cp, 1 / (w * cp * rp)
        else:
            prim, sec = cp, rp
        return {'t': time.time(), 'primary': prim, 'secondary': sec,
                'ok': True, 'n': self.n}

    def disconnect(self):
        pass


# ═══════════════════════════════════════════════════════════════════════════
#  Session: runs the segments, logs in a background thread
# ═══════════════════════════════════════════════════════════════════════════

def format_lcr(lcr, smp):
    """'Cp=  1.234 pF  D=0.0123'  /  'Z= 1.234e+06 Ohm  theta= -89.10 deg'"""
    from lcr6100 import FUNCTIONS
    pn, sn = lcr.labels
    units = FUNCTIONS.get(getattr(lcr, 'function', ''), (0, 0, ('', '')))[2]
    P, S = smp['primary'], smp['secondary']
    if pn[0] in 'CL' and P == P:          # capacitance / inductance -> pF / uH
        txt = (f'{pn}={P * 1e12:8.3f} pF' if pn[0] == 'C' else f'{pn}={P * 1e6:8.3f} uH')
    else:
        txt = f'{pn}={P:10.4g} {units[0]}'
    if sn:
        txt += f'  {sn}={S:8.4g}' + (f' {units[1]}' if units[1] else '')
    return txt + ('' if smp['ok'] else ' (OUT)')


class ActuatorSession:
    """
    Reusable engine — the CLI and actuator_gui.py both drive this class.

        s = ActuatorSession(rig, lcr, protocol, out_dir, ...)
        s.run()            # blocking; s.stop() from another thread aborts
        s.run_manual()     # record until stop(); drive it with
                           #   s.set_pressure() / s.set_valves() / s.set_phase()
        s.state            # live dict: setpoint, pressure, lcr, step, ...

    manage_rig=False : the caller opened the rig (rig.info holds open()'s
                       result) and keeps it open after the session.
    on_row(dict)     : called from the logger thread for every CSV row.
    """

    def __init__(self, rig, lcr, protocol, out_dir, max_pressure,
                 log_rate_hz=50.0, control_rate_hz=50.0, vent_s=3.0,
                 repeat=1, status=True, manage_rig=True, on_row=None):
        self.rig, self.lcr = rig, lcr
        self.manual = protocol is None or protocol.get('manual', False)
        if protocol is None:
            protocol = {'name': 'manual', 'manual': True, 'steps': []}
        self.protocol = protocol
        self.actuators = protocol.get('actuators') or {}
        self.segments = [] if self.manual else expand_protocol(protocol)
        validate(self.segments, max_pressure)
        self.run_s = sum(sg['duration'] for sg in self.segments)
        self.valve_events = ([] if self.manual
                             else expand_valve_timeline(protocol, self.run_s))
        self.manage_rig = manage_rig
        self.on_row = on_row
        self.max_pressure = max_pressure
        self.out_dir = out_dir
        self.log_dt = 1.0 / log_rate_hz
        self.ctrl_dt = 1.0 / control_rate_hz
        self.vent_s = vent_s
        self.repeat = repeat
        self.show_status = status

        self._stop = threading.Event()
        self._state_lock = threading.Lock()
        self.state = dict(rep=0, seg=-1, step=-1, label='init', type='', phase='init',
                          cycle=0, setpoint=0.0, pressure=float('nan'),
                          valves=(), lcr=None, elapsed=0.0, done=False)
        self.rows = 0
        self.t0 = None

    # ── control ──────────────────────────────────────────────────────────
    def stop(self):
        self._stop.set()

    def _update(self, **kw):
        with self._state_lock:
            self.state.update(kw)

    def _set_valves(self, v):
        self.rig.set_valves(set(v))
        self._update(valves=tuple(v))

    def _set_sp(self, p):
        p = min(max(p, 0.0), self.max_pressure)    # belt and braces
        self.rig.set_pressure(p)
        self._update(setpoint=p)

    # public, thread-safe commands (manual mode / GUI)
    def set_pressure(self, p):
        self._set_sp(p)

    def set_valves(self, v):
        self._set_valves(v)

    def set_phase(self, phase, label=None):
        self._update(phase=phase, **({'label': label} if label else {}))

    def _sleep_until(self, t_target):
        while not self._stop.is_set():
            rem = t_target - time.monotonic()
            if rem <= 0:
                return True
            self._stop.wait(min(rem, 0.05))
        return False

    def _run_segment(self, i, s, rep):
        self._update(rep=rep, seg=i, step=s['step'], label=s['label'],
                     type=s['type'], phase=s['phase'], cycle=s['cycle'])
        if s['valves'] is not None:
            self._set_valves(s['valves'])
        start = time.monotonic()
        dur = s['duration']
        if dur <= 0 or s['p0'] == s['p1']:
            self._set_sp(s['p1'])
            return self._sleep_until(start + dur)
        k = 0
        while not self._stop.is_set():
            el = time.monotonic() - start
            if el >= dur:
                break
            self._set_sp(s['p0'] + (s['p1'] - s['p0']) * el / dur)
            k += 1
            if not self._sleep_until(start + k * self.ctrl_dt):
                return False
        self._set_sp(s['p1'])
        return not self._stop.is_set()

    # ── logging ──────────────────────────────────────────────────────────
    def _columns(self):
        pn, sn = getattr(self.lcr, 'labels', ('primary', 'secondary')) if self.lcr else (None, None)
        self.lcr_cols = []
        if self.lcr:
            from lcr6100 import FUNCTIONS
            units = FUNCTIONS.get(getattr(self.lcr, 'function', ''), (0, 0, ('', '')))[2]
            self.lcr_cols = [f'lcr_{pn}_{units[0]}'.rstrip('_')]
            if sn:
                self.lcr_cols.append(f'lcr_{sn}_{units[1]}'.rstrip('_'))
        cols = ['t_unix', 't_s', 'datetime', 'rep', 'segment', 'step', 'label',
                'step_type', 'phase', 'cycle', 'setpoint_mbar', 'pressure_mbar',
                'valves_open', 'valve_mask']
        if self.lcr:
            cols += self.lcr_cols + ['lcr_ok', 'lcr_n', 'lcr_age_s']
        return cols

    def _log_loop(self, path):
        cols = self._columns()
        with open(path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(cols)
            k, last_flush, last_print = 0, time.monotonic(), 0.0
            t_next = time.monotonic()
            while True:
                finished = self._log_finished.is_set()
                try:
                    p = self.rig.get_pressure()
                except Exception as e:
                    p = float('nan')
                    if k == 0:
                        print(f'\n[log] pressure read failed: {e}')
                now = time.time()
                with self._state_lock:
                    st = dict(self.state)
                st['pressure'] = p
                row = [f'{now:.4f}', f'{now - self.t0:.4f}',
                       datetime.fromtimestamp(now).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                       st['rep'], st['seg'], st['step'], st['label'], st['type'],
                       st['phase'], st['cycle'], f"{st['setpoint']:.2f}", f'{p:.2f}',
                       valve_label(st['valves'], self.actuators),
                       sum(1 << i for i in st['valves'])]
                if self.lcr:
                    smp = self.lcr.get_sample()
                    st['lcr'] = smp
                    row += [smp['primary']]
                    if len(self.lcr_cols) > 1:
                        row += [smp['secondary']]
                    age = now - smp['t'] if smp['t'] == smp['t'] else float('nan')
                    row += [int(bool(smp['ok'])), smp['n'], f'{age:.3f}']
                w.writerow(row)
                self.rows += 1
                if self.on_row:
                    smp = st.get('lcr') or {}
                    try:
                        self.on_row({'t': now - self.t0, 'setpoint': st['setpoint'],
                                     'pressure': p, 'phase': st['phase'],
                                     'valves': st['valves'], 'step': st['step'],
                                     'label': st['label'],
                                     'primary': smp.get('primary', float('nan')),
                                     'secondary': smp.get('secondary', float('nan')),
                                     'ok': smp.get('ok', False), 'n': smp.get('n', 0)})
                    except Exception:
                        pass
                self._update(pressure=p, lcr=st.get('lcr'), elapsed=now - self.t0)
                mono = time.monotonic()
                if mono - last_flush > 1.0:
                    fh.flush()
                    last_flush = mono
                if self.show_status and mono - last_print > 0.25:
                    self._print_status(st)
                    last_print = mono
                if finished:
                    break
                k += 1
                t_next += self.log_dt
                d = t_next - time.monotonic()
                if d > 0:
                    time.sleep(d)
                else:
                    t_next = time.monotonic()     # fell behind: don't burst

    def _print_status(self, st):
        lcr = ''
        if st.get('lcr'):
            lcr = '  ' + format_lcr(self.lcr, st['lcr'])
        line = (f"\r  t={st['elapsed']:7.1f}s  [{st['step']}] {st['phase']:<9} "
                f"sp={st['setpoint']:7.1f}  P={st['pressure']:7.1f} mbar  "
                f"valves={valve_label(st['valves'], self.actuators):<12}{lcr}   ")
        sys.stdout.write(line[:160])
        sys.stdout.flush()

    # ── main entry ───────────────────────────────────────────────────────
    def run(self):
        """Run the protocol (blocking).  Returns the status string."""
        return self._execute(self._protocol_body, reset=True, vent_at_end=True)

    def run_manual(self, vent_at_end=False):
        """Record until stop(); pressure/valves come from set_*()."""
        return self._execute(self._manual_body, reset=False, vent_at_end=vent_at_end)

    def _valve_loop(self, t_start, done):
        """Apply the asynchronous valve timeline for one run."""
        open_ = set()
        k, ev = 0, self.valve_events
        while k < len(ev) and not self._stop.is_set() and not done.is_set():
            t = ev[k][0]
            wait = t_start + t - time.monotonic()
            if wait > 0:
                if done.wait(min(wait, 0.02)) or self._stop.is_set():
                    break
                continue
            while k < len(ev) and ev[k][0] == t:      # all changes at this instant
                _, i, on = ev[k]
                (open_.add if on else open_.discard)(i)
                k += 1
            try:
                self._set_valves(tuple(sorted(open_)))
            except Exception as e:
                print(f'\n[valves] set failed: {e}')

    def _protocol_body(self):
        for rep in range(self.repeat):
            vthread = None
            if self.valve_events:
                done = threading.Event()
                self._set_valves(())
                vthread = threading.Thread(target=self._valve_loop, name='valves',
                                           daemon=True, args=(time.monotonic(), done))
                vthread.start()
            try:
                for i, s in enumerate(self.segments):
                    if not self._run_segment(i, s, rep):
                        return False
            finally:
                if vthread:
                    done.set()
                    vthread.join(timeout=1.0)
        return True

    def _manual_body(self):
        self._update(label='manual', type='manual', phase='hold', step=0, seg=0)
        while not self._stop.is_set():
            self._stop.wait(0.05)
        return True

    def _execute(self, body, reset, vent_at_end):
        os.makedirs(self.out_dir, exist_ok=True)
        meta = {'started': datetime.now().isoformat(timespec='seconds'),
                'max_pressure_mbar': self.max_pressure,
                'log_rate_hz': 1.0 / self.log_dt, 'repeat': self.repeat,
                'n_segments': len(self.segments)}
        with open(os.path.join(self.out_dir, 'protocol.json'), 'w') as fh:
            json.dump(self.protocol, fh, indent=2)

        meta['fluigent'] = self.rig.open() if self.manage_rig else getattr(self.rig, 'info', {})
        dev_max = meta['fluigent'].get('pressure_range_mbar', [0, 1e9])[1]
        if self.max_pressure > dev_max:
            raise ProtocolError(f'max pressure {self.max_pressure} > device range {dev_max}')
        if self.lcr:
            meta['lcr'] = {'function': getattr(self.lcr, 'function', None),
                           'readback': dict(getattr(self.lcr, 'applied_config', {})),
                           'describe': self.lcr.describe()}
            try:
                meta['lcr']['idn'] = self.lcr.idn()
            except Exception:
                pass

        self._log_finished = threading.Event()
        status = 'error'
        logger = None
        try:
            if reset:
                self._set_sp(0.0)
                self._set_valves(())
            else:
                self._update(setpoint=getattr(self.rig, 'setpoint', 0.0),
                             valves=tuple(sorted(getattr(self.rig, 'open_set', ()))))
            self.t0 = time.time()
            logger = threading.Thread(target=self._log_loop, name='logger', daemon=True,
                                      args=(os.path.join(self.out_dir, 'data.csv'),))
            logger.start()
            completed = body()
            status = 'completed' if completed else 'stopped'
        except KeyboardInterrupt:
            status = 'stopped (Ctrl-C)'
        except Exception as e:
            status = f'error: {e!r}'
            raise
        finally:
            if vent_at_end or status.startswith('error'):
                self._safe_shutdown()
            if logger:
                self._log_finished.set()
                logger.join(timeout=5)
            if self.manage_rig:
                try:
                    self.rig.close()
                except Exception as e:
                    print(f'[fgt] close failed: {e}')
            meta.update(finished=datetime.now().isoformat(timespec='seconds'),
                        status=status, rows=self.rows,
                        duration_s=(time.time() - self.t0) if self.t0 else 0)
            with open(os.path.join(self.out_dir, 'meta.json'), 'w') as fh:
                json.dump(meta, fh, indent=2, default=str)
            self._update(done=True)
            print(f'\n[session] {status}.  {self.rows} rows -> {self.out_dir}')
        return status

    def _safe_shutdown(self):
        self._update(label='shutdown', phase='vent', type='shutdown')
        try:
            self._set_sp(0.0)
        except Exception as e:
            print(f'\n[fgt] WARNING could not vent: {e}')
        # keep valves as they are so the actuators deflate through the Flow EZ
        t_end = time.monotonic() + self.vent_s
        while time.monotonic() < t_end:
            time.sleep(0.05)
        try:
            self._set_valves(())
        except Exception as e:
            print(f'\n[fgt] WARNING could not close valves: {e}')


# ═══════════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════════

def parse_args(argv=None):
    from lcr6100 import add_lcr_args
    p = argparse.ArgumentParser(
        description='Inflatable-actuator pressure protocols + LCR data logging',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    src = p.add_argument_group('what to run (choose one)')
    src.add_argument('--protocol', help='JSON protocol file')
    src.add_argument('--valves', help="quick sweep: combos separated by ';', "
                     "outputs inside a combo by '+', e.g. \"A1;A2;A1+A2\" or \"1;2;1+2\"")
    q = p.add_argument_group('quick-sweep options (with --valves)')
    q.add_argument('--setpoints', default='100,200', help='mbar list [100,200]')
    q.add_argument('--ramp', type=float, default=2.0, help='ramp up/down time s [2]')
    q.add_argument('--hold', type=float, default=5.0, help='hold at set-point s [5]')
    q.add_argument('--rest', type=float, default=5.0, help='rest at 0 mbar after each deflation s [5]')
    q.add_argument('--cycles', type=int, default=1, help='inflate/deflate cycles per set-point [1]')
    q.add_argument('--baseline', type=float, default=5.0, help='0-mbar baseline at start/end s [5]')
    q.add_argument('--actuators', default='', help='aliases, e.g. "A1=out1,A2=out2"')
    g = p.add_argument_group('run options')
    g.add_argument('--name', default=None, help='session name (folder prefix)')
    g.add_argument('--max-pressure', type=float, default=None,
                   help=f'safety ceiling mbar [protocol value or {DEFAULT_MAX_PRESSURE:g}]')
    g.add_argument('--repeat', type=int, default=None, help='repeat whole protocol N times')
    g.add_argument('--log-rate', type=float, default=50.0, help='CSV rows/s [50]')
    g.add_argument('--control-rate', type=float, default=50.0, help='ramp set-point updates/s [50]')
    g.add_argument('--vent-time', type=float, default=3.0, help='deflate time before closing valves at the end [3]')
    g.add_argument('--pressure-channel', type=int, default=0, help='Flow EZ channel index [0]')
    g.add_argument('--out', default=LOG_DIR, help='log root folder [Fluigent_control/logs]')
    g.add_argument('--dry-run', action='store_true', help='print schedule only')
    g.add_argument('--simulate', action='store_true', help='simulated Fluigent + LCR')
    g.add_argument('--yes', '-y', action='store_true', help='do not ask for confirmation')
    g.add_argument('--no-analysis', action='store_true',
                   help='skip the automatic plots + summary at the end')
    g.add_argument('--show-plots', action='store_true',
                   help='open the analysis figures in a window at the end')
    l = p.add_argument_group('LCR')
    l.add_argument('--no-lcr', action='store_true', help='do not use the LCR meter')
    l.add_argument('--lcr-port', default=None, help='LCR serial port [ask]')
    l.add_argument('--measure', choices=sorted(MEASURE_MODES), default='capacitance',
                   help='capacitance = Cp-D,  impedance = Z-theta(deg)  '
                        '[capacitance]; --lcr-func overrides')
    add_lcr_args(p)
    p.set_defaults(lcr_func=None)
    a = p.parse_args(argv)
    if a.lcr_func is None:
        a.lcr_func = MEASURE_MODES[a.measure]
    if bool(a.protocol) == bool(a.valves):
        p.error('give exactly one of --protocol FILE or --valves "..."')
    return a


def pick_lcr_port():
    from lcr6100 import list_ports
    ports = list_ports()
    if not ports:
        raise SystemExit('[LCR] no serial ports found — use --no-lcr or connect the meter')
    print('\n  Serial ports:')
    for i, (dev, desc) in enumerate(ports):
        print(f'    [{i}] {dev}  —  {desc}')
    raw = input(f'  LCR port [0-{len(ports) - 1}] or path > ').strip()
    return ports[int(raw)][0] if raw.isdigit() else raw


# ═══════════════════════════════════════════════════════════════════════════
#  Interactive set-up (no command-line arguments -> ask question by question)
# ═══════════════════════════════════════════════════════════════════════════

WIZARD_FILE = os.path.join(HERE, 'wizard_last.json')   # last answers = next defaults

WIZARD_DEFAULTS = {
    'simulate': False, 'source': 'sweep', 'protocol': '',
    'actuators': 'A1=out1', 'valves': 'A1', 'setpoints': '100,200',
    'ramp': 2.0, 'hold': 5.0, 'rest': 5.0, 'cycles': 1, 'baseline': 5.0,
    'name': 'test', 'max_pressure': 200.0, 'repeat': 1,
    'use_lcr': True, 'measure': 'capacitance', 'lcr_freq': 20000.0,
    'lcr_volt': 1.0, 'lcr_speed': 'FAST', 'lcr_port': '',
    'log_rate': 50.0, 'vent_time': 3.0, 'analysis': True, 'show_plots': True,
}


class _Quit(Exception):
    pass


def _ask(question, default=None, conv=str, check=None, hint=''):
    """Ask one question; Enter keeps the default; re-ask until valid. 'q' quits."""
    while True:
        d = '' if default is None or default == '' else f' [{default}]'
        h = f'  ({hint})' if hint else ''
        raw = input(f'\n  {question}{h}{d}\n  > ').strip()
        if raw.lower() in ('q', 'quit', 'exit'):
            raise _Quit()
        if not raw:
            if default is None or default == '':
                print('    ! an answer is needed')
                continue
            raw = str(default)
        try:
            val = conv(raw)
            if check:
                msg = check(val)
                if msg:
                    print(f'    ! {msg}')
                    continue
            return val
        except (ValueError, ProtocolError) as e:
            print(f'    ! {e}' if str(e) else f"    ! '{raw}' is not valid")


def _yes(question, default=True):
    def conv(s):
        s = s.lower()
        if s in ('y', 'yes', 's', 'si', 'sí', '1', 'true'):
            return True
        if s in ('n', 'no', '0', 'false'):
            return False
        raise ValueError('answer y or n')
    return _ask(question + ' [y/n]', 'y' if default else 'n', conv)


def _choose(question, options, default_key):
    """options: list of (key, text). Answer with the number or the key."""
    keys = [k for k, _ in options]
    lines = '\n'.join(f'      [{i + 1}] {t}' for i, (_, t) in enumerate(options))
    dflt = keys.index(default_key) + 1 if default_key in keys else 1

    def conv(s):
        if s.isdigit() and 1 <= int(s) <= len(keys):
            return keys[int(s) - 1]
        if s in keys:
            return s
        raise ValueError(f'choose 1..{len(keys)}')
    return _ask(f'{question}\n{lines}', dflt, conv)


def _num(lo=None, hi=None, integer=False):
    def conv(s):
        try:
            x = int(s) if integer else float(s)
        except ValueError:
            raise ValueError(f"'{s}' is not {'an integer' if integer else 'a number'}")
        if (lo is not None and x < lo) or (hi is not None and x > hi):
            raise ValueError(f'must be between {lo:g} and {hi:g}' if hi is not None
                             else f'must be ≥ {lo:g}')
        return x
    return conv


def interactive_args():
    """Ask every option one message at a time and return the equivalent argv."""
    last = dict(WIZARD_DEFAULTS)
    try:
        with open(WIZARD_FILE) as fh:
            last.update(json.load(fh))
    except (OSError, ValueError):
        pass
    a = {}
    print('\n=== Actuator + LCR session set-up ===')
    print('  Enter = keep the value in [brackets],  q = quit')
    try:
        # 1. hardware
        a['simulate'] = _yes('Simulate (no hardware, desk test)?', last['simulate'])

        # 2. safety ceiling (asked first so pressures can be checked against it)
        a['max_pressure'] = _ask('Maximum pressure — safety ceiling', last['max_pressure'],
                                 _num(1, 2000), hint='mbar')
        mp = a['max_pressure']

        # 3. what to run
        a['source'] = _choose('What do you want to run?',
                              [('sweep', 'Quick sweep (valve combos × set-points)'),
                               ('file', 'JSON protocol file')], last['source'])
        if a['source'] == 'file':
            pdir = os.path.join(HERE, 'protocols')
            files = sorted(f for f in os.listdir(pdir) if f.endswith('.json')) \
                if os.path.isdir(pdir) else []
            if files:
                print('\n  Protocols in protocols/:')
                for i, f in enumerate(files):
                    print(f'      [{i + 1}] {f}')

            def conv_file(s):
                if s.isdigit() and files and 1 <= int(s) <= len(files):
                    s = os.path.join(pdir, files[int(s) - 1])
                path = os.path.expanduser(s)
                if not os.path.isabs(path) and not os.path.exists(path):
                    path = os.path.join(pdir, path)
                with open(path) as fh:                        # must exist + parse
                    proto = json.load(fh)
                validate(expand_protocol(proto), mp)          # within the ceiling?
                return path

            def safe_file(s):
                try:
                    return conv_file(s)
                except OSError:
                    raise ValueError(f"cannot open '{s}'")
                except json.JSONDecodeError as e:
                    raise ValueError(f'not valid JSON: {e}')
            a['protocol'] = _ask('Protocol file (number or path)',
                                 last['protocol'] or (1 if files else None), safe_file)
        else:
            def conv_act(s):
                acts = parse_actuators('' if s.strip() == '-' else s)
                valve_resolver(acts)                          # raises if a target is bad
                return '' if s.strip() == '-' else s
            a['actuators'] = _ask('Actuator names for the P-Switch outputs',
                                  last['actuators'] or '-', conv_act,
                                  hint="e.g. A1=out1,A2=out2   '-' = none, use out1..out8")
            acts = parse_actuators(a['actuators'])
            known = ', '.join(list(acts) + [f'out{i + 1}' for i in range(N_OUTPUTS)][:3]) + ', …'

            def conv_valves(s):
                resolve = valve_resolver(acts)
                combos = [c.strip() for c in s.split(';') if c.strip()]
                if not combos:
                    raise ValueError('give at least one combination')
                for c in combos:
                    if not resolve(c):
                        raise ValueError(f"'{c}' opens no valve")
                return s
            a['valves'] = _ask('Valve combinations to test', last['valves'], conv_valves,
                               hint=f"';' between tests, '+' = together, e.g. A1;A2;A1+A2 — known: {known}")

            def conv_sp(s):
                vals = [float(x) for x in s.split(',') if x.strip()]
                if not vals:
                    raise ValueError('give at least one set-point')
                bad = [v for v in vals if not 0 < v <= mp]
                if bad:
                    raise ValueError(f'{bad[0]:g} mbar is outside 0..{mp:g} (the ceiling)')
                return ','.join(f'{v:g}' for v in vals)
            a['setpoints'] = _ask('Set-points', last['setpoints'], conv_sp,
                                  hint='mbar, comma separated')
            a['ramp'] = _ask('Ramp time (inflation and deflation)', last['ramp'], _num(0), hint='s')
            a['hold'] = _ask('Hold time at each set-point', last['hold'], _num(0), hint='s')
            a['rest'] = _ask('Rest at 0 mbar after each deflation', last['rest'], _num(0), hint='s')
            a['cycles'] = _ask('Cycles per set-point', last['cycles'], _num(1, 1000, True))
            a['baseline'] = _ask('Baseline at 0 mbar at start and end', last['baseline'], _num(0), hint='s')

        def conv_name(s):
            s = s.strip().replace(' ', '_')
            if any(c in s for c in '/\\:'):
                raise ValueError('no / \\ : in the name')
            return s
        a['name'] = _ask('Session name (folder prefix)', last['name'], conv_name)
        a['repeat'] = _ask('Repeat the whole protocol how many times?', last['repeat'],
                           _num(1, 1000, True))

        # 4. LCR
        a['use_lcr'] = _yes('Record the LCR-6100?', last['use_lcr'])
        if a['use_lcr']:
            a['measure'] = _choose('LCR measurement',
                                   [('capacitance', 'Capacitance  (Cp-D)'),
                                    ('impedance', 'Impedance    (|Z|-θ in degrees)')],
                                   last['measure'])
            a['lcr_freq'] = _ask('Test frequency', last['lcr_freq'], _num(10, 300000), hint='Hz')
            a['lcr_volt'] = _ask('Test level', last['lcr_volt'], _num(0.01, 2.0), hint='V')
            a['lcr_speed'] = _choose('Measurement speed',
                                     [('FAST', 'FAST'), ('MED', 'MED'), ('SLOW', 'SLOW')],
                                     last['lcr_speed'])
            if not a['simulate']:
                from lcr6100 import list_ports
                ports = [d for d, _ in list_ports()]
                if ports:
                    print('\n  Serial ports:')
                    for i, d in enumerate(ports):
                        print(f'      [{i + 1}] {d}')
                dflt = (ports.index(last['lcr_port']) + 1 if last['lcr_port'] in ports
                        else next((i + 1 for i, d in enumerate(ports) if 'usbserial' in d), 1)
                        if ports else last['lcr_port'] or None)

                def conv_port(s):
                    if s.isdigit() and 1 <= int(s) <= len(ports):
                        return ports[int(s) - 1]
                    if not os.path.exists(s):
                        raise ValueError(f"no such port '{s}'")
                    return s
                a['lcr_port'] = _ask('LCR serial port (number or path)', dflt, conv_port)

        # 5. logging / end of run
        a['log_rate'] = _ask('Log rate', last['log_rate'], _num(1, 200), hint='CSV rows per second')
        a['vent_time'] = _ask('Vent time at the end (0 mbar before valves close)',
                              last['vent_time'], _num(0, 60), hint='s')
        a['analysis'] = _yes('Analyse automatically at the end (plots + summary)?', last['analysis'])
        a['show_plots'] = _yes('Open the plots when done?', last['show_plots']) \
            if a['analysis'] else False
    except (_Quit, EOFError, KeyboardInterrupt):
        raise SystemExit('\nSet-up cancelled.')

    try:
        with open(WIZARD_FILE, 'w') as fh:
            json.dump({**last, **a}, fh, indent=2)
    except OSError:
        pass

    # ── equivalent command line ────────────────────────────────────────
    argv = []
    if a['source'] == 'file':
        argv += ['--protocol', a['protocol']]
    else:
        argv += ['--valves', a['valves'], '--setpoints', a['setpoints'],
                 '--ramp', f"{a['ramp']:g}", '--hold', f"{a['hold']:g}",
                 '--rest', f"{a['rest']:g}", '--cycles', str(a['cycles']),
                 '--baseline', f"{a['baseline']:g}"]
        if a['actuators']:
            argv += ['--actuators', a['actuators']]
    argv += ['--name', a['name'], '--max-pressure', f"{a['max_pressure']:g}",
             '--repeat', str(a['repeat']), '--log-rate', f"{a['log_rate']:g}",
             '--vent-time', f"{a['vent_time']:g}"]
    if a['use_lcr']:
        argv += ['--measure', a['measure'], '--lcr-freq', f"{a['lcr_freq']:g}",
                 '--lcr-volt', f"{a['lcr_volt']:g}", '--lcr-speed', a['lcr_speed']]
        if a.get('lcr_port'):
            argv += ['--lcr-port', a['lcr_port']]
    else:
        argv.append('--no-lcr')
    if a['simulate']:
        argv.append('--simulate')
    if not a['analysis']:
        argv.append('--no-analysis')
    if a['show_plots']:
        argv.append('--show-plots')

    import shlex
    print('\n' + '─' * 70)
    print('  Settings fixed for this run (same as running):')
    print('  python actuator_lcr_logger.py ' + ' '.join(shlex.quote(x) for x in argv))
    print('─' * 70)
    return argv


def main(argv=None):
    if argv is None and len(sys.argv) == 1:      # plain "python actuator_lcr_logger.py"
        argv = interactive_args()
    args = parse_args(argv)
    if args.protocol:
        with open(args.protocol) as fh:
            protocol = json.load(fh)
        if args.actuators:
            protocol.setdefault('actuators', {}).update(parse_actuators(args.actuators))
    else:
        protocol = sweep_protocol(args)
    if args.name:
        protocol['name'] = args.name
    name = protocol.get('name', 'session')
    max_p = (args.max_pressure if args.max_pressure is not None
             else float(protocol.get('max_pressure_mbar', DEFAULT_MAX_PRESSURE)))
    repeat = args.repeat if args.repeat is not None else int(protocol.get('repeat', 1))
    protocol['max_pressure_mbar'] = max_p
    protocol['repeat'] = repeat

    try:
        segs = expand_protocol(protocol)
        validate(segs, max_p)
    except ProtocolError as e:
        raise SystemExit(f'[protocol] {e}')

    print(f'\n=== {name}   (ceiling {max_p:g} mbar) ===')
    total = print_schedule(segs, protocol.get('actuators'), repeat)
    try:
        print_valve_events(expand_valve_timeline(protocol, total), protocol.get('actuators'))
    except ProtocolError as e:
        raise SystemExit(f'[protocol] {e}')
    if args.dry_run:
        return

    # ── hardware ─────────────────────────────────────────────────────────
    rig = SimRig() if args.simulate else FluigentRig(args.pressure_channel)
    lcr = None
    if not args.no_lcr:
        if args.simulate:
            from lcr6100 import FUNCTIONS, normalize_function
            lcr = SimLCR(rig)
            lcr.function = normalize_function(args.lcr_func)     # honour --measure
            lcr.labels = FUNCTIONS[lcr.function][:2]
            lcr.freq_hz = float(args.lcr_freq)
            lcr.applied_config = {'function': lcr.function, 'simulated': True}
        else:
            from lcr6100 import lcr_from_args, LCRConfigError
            port = args.lcr_port or pick_lcr_port()
            lcr = lcr_from_args(port, args, warn_columns=False)
            try:
                lcr.connect()
            except LCRConfigError as e:
                lcr.disconnect()
                raise SystemExit(f'{e}\n  -> check SYSTEM > TERMINATOR (try --lcr-eol lf)')
            lcr.start_polling()
            time.sleep(0.3)
            smp = lcr.get_sample()
            print(f'[LCR] {lcr.describe()}   first reading: {format_lcr(lcr, smp)}')

    out_dir = os.path.join(args.out, f"{name}_{datetime.now():%Y%m%d_%H%M%S}")
    if not args.yes and not args.simulate:
        if input('\nStart? Pressure will be applied. [y/N] > ').strip().lower() != 'y':
            if lcr:
                lcr.disconnect()
            print('Aborted.')
            return

    session = ActuatorSession(rig, lcr, protocol, out_dir, max_p,
                              log_rate_hz=args.log_rate,
                              control_rate_hz=args.control_rate,
                              vent_s=args.vent_time, repeat=repeat)
    try:
        session.run()
    finally:
        if lcr:
            lcr.disconnect()

    if not args.no_analysis and session.rows:
        try:
            from analyze_actuator_session import analyze
            analyze(out_dir, show=args.show_plots)
        except Exception as e:      # never lose a run because a plot failed
            print(f'[analysis] failed: {e!r}\n  re-run: python '
                  f'analyze_actuator_session.py "{out_dir}"')


if __name__ == '__main__':
    main()
    # The Fluigent dylib (x86, Rosetta) crashes with SIGILL during interpreter
    # tear-down; everything is already vented, closed and saved here.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
