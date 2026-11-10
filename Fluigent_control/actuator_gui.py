#!/usr/bin/env python3
"""
actuator_gui.py — interactive control of inflatable actuators + LCR logging
==========================================================================

Graphical front-end for actuator_lcr_logger.py (same engine, same CSV /
analysis output).

    conda activate fluigent-x86
    python -W ignore actuator_gui.py            # real hardware
    python -W ignore actuator_gui.py --simulate # no hardware (desk testing)

Window
------
  Top bar   live pressure / set-point / LCR values, recording state and the
            red STOP / VENT button (also: Esc key)
  Setup     connect Fluigent + LCR, LCR mode (capacitance Cp-D / impedance
            Z-theta / any function), frequency, level, speed, safety ceiling,
            actuator names for the 8 P-Switch outputs
  Manual    toggle valves, set a pressure or ramp to it, record everything
            you do to a session folder (analysed when you stop)
  Protocol  quick sweep or JSON protocol, preview the schedule, run / stop,
            automatic analysis + open the plots at the end
  Ramps + valves
            build a pressure profile from ramps / holds (or generate a ramp
            series) and give every valve its own asynchronous timing
            (closed / open from t / pulse train with delay, on, off, count,
            plus a stagger helper for sequential activation); preview the
            profile + valve chart, save / load as JSON, run
  Plots     pressure vs time, LCR primary vs time, LCR secondary vs time,
            LCR primary vs pressure (inflation blue / deflation orange)

Settings (ports, mode, names, limits) are remembered in gui_settings.json.
"""

import argparse
import collections
import io
import json
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from contextlib import redirect_stdout
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import actuator_lcr_logger as core            # also puts Capacitance_measurement on sys.path
import analyze_actuator_session as ana
import lcr6100

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(HERE, 'gui_settings.json')

C_INFLATE, C_DEFLATE, C_HOLD = ana.C_INFLATE, ana.C_DEFLATE, ana.C_HOLD
C_SET, C_GRID, C_SURFACE, C_TEXT2 = ana.C_SETPOINT, ana.C_GRID, ana.C_SURFACE, ana.C_TEXT2
C_PRESSURE, C_TEXT = ana.C_PRESSURE, ana.C_TEXT
C_GREEN, C_LGREEN, C_YELLOW = ana.SDU_GREEN, ana.SDU_LIGHT_GREEN, ana.SDU_YELLOW
C_BROWN, C_BEIGE = ana.SDU_BROWN, ana.SDU_BEIGE
C_STOP = '#C0392B'                  # safety red stays red

PLOT_MAX_POINTS = 900               # live lines are thinned to this many points
UI_READOUT_MS = 100                 # top-bar refresh
PLOT_MIN_MS = 250                   # fastest plot refresh (adapts to draw time)


def apply_theme(root):
    """SDU-coloured ttk theme ('clam' base: macOS 'aqua' ignores colours)."""
    st = ttk.Style(root)
    st.theme_use('clam')
    base = ('Helvetica', 12)
    root.configure(bg=C_BEIGE)
    st.configure('.', background=C_BEIGE, foreground=C_TEXT, font=base,
                 bordercolor='#CBBFAE', lightcolor=C_BEIGE, darkcolor=C_BEIGE,
                 troughcolor='#FFFFFF', focuscolor=C_YELLOW,
                 selectbackground=C_YELLOW, selectforeground=C_TEXT)
    st.configure('TFrame', background=C_BEIGE)
    st.configure('TLabel', background=C_BEIGE, foreground=C_TEXT)
    st.configure('Hint.TLabel', foreground=C_TEXT2, font=('Helvetica', 11))
    st.configure('TLabelframe', background=C_BEIGE, bordercolor='#CBBFAE', relief='solid',
                 borderwidth=1)
    st.configure('TLabelframe.Label', background=C_BEIGE, foreground=C_BROWN,
                 font=('Helvetica', 12, 'bold'))
    st.configure('TNotebook', background=C_BEIGE, borderwidth=0, tabmargins=(4, 4, 4, 0))
    st.configure('TNotebook.Tab', background='#E2D8C9', foreground=C_BROWN,
                 padding=(14, 6), font=('Helvetica', 12, 'bold'), borderwidth=0)
    st.map('TNotebook.Tab', background=[('selected', C_YELLOW), ('active', '#F5D27A')],
           foreground=[('selected', C_TEXT)])
    st.configure('TButton', background='#FFFFFF', foreground=C_BROWN, padding=(10, 4),
                 bordercolor='#BBAE9B', relief='flat')
    st.map('TButton', background=[('disabled', '#EEE8DE'), ('pressed', C_LGREEN),
                                  ('active', '#F6EFD9')],
           foreground=[('disabled', '#B5AA9B')])
    st.configure('Accent.TButton', background=C_GREEN, foreground='#FFFFFF',
                 bordercolor=C_GREEN, font=('Helvetica', 12, 'bold'))
    st.map('Accent.TButton', background=[('disabled', '#C9D3B8'), ('pressed', '#5E7D39'),
                                         ('active', '#86AB57')],
           foreground=[('disabled', '#F4F1EA')])
    st.configure('Stop.TButton', background=C_STOP, foreground='#FFFFFF', bordercolor=C_STOP,
                 font=('Helvetica', 16, 'bold'), padding=(18, 12))
    st.map('Stop.TButton', background=[('pressed', '#922B21'), ('active', '#D04A3B')])
    st.configure('Valve.Toolbutton', background='#FFFFFF', foreground=C_BROWN,
                 bordercolor='#BBAE9B', padding=(4, 8), anchor='center',
                 font=('Helvetica', 12, 'bold'))
    st.map('Valve.Toolbutton',
           background=[('disabled', '#EEE8DE'), ('selected', C_YELLOW), ('active', '#F6EFD9')],
           foreground=[('disabled', '#B5AA9B'), ('selected', C_TEXT)])
    for w in ('TEntry', 'TCombobox', 'TSpinbox'):
        st.configure(w, fieldbackground='#FFFFFF', foreground=C_TEXT, bordercolor='#BBAE9B',
                     arrowcolor=C_BROWN, padding=3)
    st.map('TCombobox', fieldbackground=[('readonly', '#FFFFFF'), ('disabled', '#EEE8DE')])
    for w in ('TCheckbutton', 'TRadiobutton'):
        st.configure(w, background=C_BEIGE, foreground=C_TEXT, indicatorbackground='#FFFFFF',
                     indicatorforeground=C_GREEN)
        st.map(w, background=[('active', C_BEIGE)],
               indicatorbackground=[('selected', C_GREEN), ('active', '#F6EFD9')],
               indicatorforeground=[('selected', '#FFFFFF')])
    st.configure('Horizontal.TProgressbar', background=C_GREEN, troughcolor='#FFFFFF',
                 bordercolor='#CBBFAE', lightcolor=C_GREEN, darkcolor=C_GREEN)
    st.configure('Treeview', background='#FFFFFF', fieldbackground='#FFFFFF',
                 foreground=C_TEXT, rowheight=22, font=('Helvetica', 11))
    st.configure('Treeview.Heading', background='#E2D8C9', foreground=C_BROWN,
                 font=('Helvetica', 11, 'bold'), relief='flat')
    st.map('Treeview', background=[('selected', C_YELLOW)], foreground=[('selected', C_TEXT)])
    st.configure('TPanedwindow', background=C_BEIGE)
    st.configure('Sash', sashthickness=6, background='#E2D8C9')


class ScrollFrame(ttk.Frame):
    """Vertically scrollable container: put widgets in .inner"""
    def __init__(self, parent, **kw):
        super().__init__(parent, **kw)
        self.canvas = tk.Canvas(self, bg=C_BEIGE, highlightthickness=0, bd=0)
        sb = ttk.Scrollbar(self, orient='vertical', command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas, padding=8)
        self.inner.bind('<Configure>', lambda e: self.canvas.configure(
            scrollregion=self.canvas.bbox('all')))
        self._win = self.canvas.create_window((0, 0), window=self.inner, anchor='nw')
        self.canvas.bind('<Configure>', lambda e: self.canvas.itemconfigure(
            self._win, width=e.width))
        self.canvas.configure(yscrollcommand=sb.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        for w in (self.canvas, self.inner):
            w.bind('<Enter>', lambda e: self.canvas.bind_all('<MouseWheel>', self._wheel))
            w.bind('<Leave>', lambda e: self.canvas.unbind_all('<MouseWheel>'))

    def _wheel(self, e):
        d = e.delta
        self.canvas.yview_scroll(int(-d if abs(d) < 10 else -d / 120), 'units')

MEASURE_PRESETS = {'Capacitance (Cp-D)': 'Cp-D', 'Impedance (Z-θ deg)': 'Z-thd',
                   'Other function…': None}

DEFAULTS = {
    'lcr_port': '', 'lcr_preset': 'Impedance (Z-θ deg)', 'lcr_func': 'Z-thd',
    'lcr_freq': '20000', 'lcr_volt': '1.0', 'lcr_speed': 'FAST', 'lcr_avg': '',
    'lcr_eol': 'crlf', 'max_p': '200', 'vent_s': '3', 'log_rate': '50',
    'aliases': ['A1', '', '', '', '', '', '', ''],
    'sweep_valves': 'A1', 'sweep_setpoints': '100,200', 'sweep_ramp': '3',
    'sweep_hold': '5', 'sweep_rest': '5', 'sweep_cycles': '1', 'sweep_baseline': '5',
    'name': 'gui_test', 'repeat': '1', 'protocol_file': '', 'source': 'sweep',
    'window_s': '60', 'ramp_target': '100', 'ramp_time': '3',
    'analyze': True, 'open_plots': True, 'simulate': False,
    # Ramps + valves builder
    'builder_steps': [['hold', 0, 5], ['ramp', 100, 3], ['hold', 100, 5],
                      ['ramp', 0, 3], ['hold', 0, 5]],
    'builder_valves': [{'mode': 'pulse', 'delay': '0', 'on': '2', 'off': '1', 'n': '0'}]
                      + [{'mode': 'closed', 'delay': '0', 'on': '2', 'off': '1', 'n': '0'}] * 7,
    'gen_peak': '100', 'gen_step': '0', 'gen_up': '3', 'gen_hold': '5', 'gen_down': '3',
    'gen_rest': '5', 'gen_n': '3', 'stagger_s': '1',
}
VALVE_MODES = ('closed', 'open', 'pulse')


# ═══════════════════════════════════════════════════════════════════════════
#  Hardware wrapper: one rig + one LCR shared by manual control and sessions
# ═══════════════════════════════════════════════════════════════════════════

class Hardware:
    def __init__(self, post):
        self.post = post                  # post(kind, payload) -> GUI queue
        self.rig = None
        self.lcr = None
        self.session = None               # ActuatorSession while recording/running
        self.max_p = 200.0
        self.phase = 'hold'
        self._ramp_stop = threading.Event()
        self._ramp_thread = None
        self.lock = threading.Lock()

    # connections -------------------------------------------------------
    def connect_rig(self, simulate):
        rig = core.SimRig() if simulate else core.FluigentRig(0)
        info = rig.open()
        rig.set_pressure(0.0)
        rig.set_valves(set())
        self.rig = rig
        return info

    def connect_lcr(self, simulate, port, cfg, eol):
        if self.lcr:
            self.disconnect_lcr()
        if simulate:
            if not self.rig:
                raise RuntimeError('connect the (simulated) Fluigent first')
            lcr = core.SimLCR(self.rig)
            lcr.function = cfg['function'] if cfg else 'Cp-Rp'
            lcr.labels = lcr6100.FUNCTIONS[lcr.function][:2]
        else:
            lcr = lcr6100.LCR6100(port, eol=eol, config=cfg, verbose=False)
            lcr.connect()
            lcr.start_polling()
        self.lcr = lcr
        return lcr.describe()

    def configure_lcr(self, cfg):
        if isinstance(self.lcr, core.SimLCR):
            self.lcr.function = cfg['function']
            self.lcr.labels = lcr6100.FUNCTIONS[cfg['function']][:2]
            return self.lcr.describe()
        self.lcr.configure(**cfg)
        return self.lcr.describe()

    def disconnect_lcr(self):
        if self.lcr:
            try:
                self.lcr.disconnect()
            except Exception:
                pass
        self.lcr = None

    # commands ----------------------------------------------------------
    def set_pressure(self, p):
        if not self.rig:
            raise RuntimeError('Fluigent not connected')
        p = min(max(float(p), 0.0), self.max_p)
        if self.session:
            self.session.set_pressure(p)
        else:
            self.rig.set_pressure(p)
        return p

    def set_valves(self, open_set):
        if not self.rig:
            raise RuntimeError('Fluigent not connected')
        if self.session:
            self.session.set_valves(tuple(sorted(open_set)))
        else:
            self.rig.set_valves(set(open_set))

    def set_phase(self, phase):
        self.phase = phase
        if self.session:
            self.session.set_phase(phase)

    @property
    def setpoint(self):
        return getattr(self.rig, 'setpoint', 0.0) if self.rig else 0.0

    @property
    def open_set(self):
        return set(getattr(self.rig, 'open_set', ())) if self.rig else set()

    def cancel_ramp(self):
        self._ramp_stop.set()
        t = self._ramp_thread
        if t and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=1.0)

    def ramp(self, target, duration, rate_hz=50.0):
        self.cancel_ramp()
        self._ramp_stop = threading.Event()
        stop = self._ramp_stop
        p0 = self.setpoint
        target = min(max(float(target), 0.0), self.max_p)

        def work():
            try:
                self.set_phase('inflate' if target >= p0 else 'deflate')
                t0 = time.monotonic()
                while not stop.is_set():
                    el = time.monotonic() - t0
                    if el >= duration:
                        break
                    self.set_pressure(p0 + (target - p0) * el / duration)
                    stop.wait(1.0 / rate_hz)
                if not stop.is_set():
                    self.set_pressure(target)
            except Exception as e:
                self.post('error', f'Ramp failed: {e}')
            finally:
                self.set_phase('hold')
        if duration <= 0:
            self.set_pressure(target)
            return
        self._ramp_thread = threading.Thread(target=work, daemon=True, name='ramp')
        self._ramp_thread.start()

    def vent(self, close_after_s=3.0):
        """Pressure -> 0 now, close all valves after close_after_s (deflation)."""
        self.cancel_ramp()
        if not self.rig:
            return
        try:
            self.rig.set_pressure(0.0)
            if self.session:
                self.session._update(setpoint=0.0)
        except Exception as e:
            self.post('error', f'Could not vent: {e}')

        def close_later():
            time.sleep(close_after_s)
            try:
                self.rig.set_valves(set())
                if self.session:
                    self.session._update(valves=())
                self.post('valves', set())
            except Exception as e:
                self.post('error', f'Could not close valves: {e}')
        threading.Thread(target=close_later, daemon=True).start()

    def shutdown(self, vent_s):
        if self.session:
            self.session.stop()
        self.cancel_ramp()
        if self.rig:
            try:
                self.rig.set_pressure(0.0)
                time.sleep(vent_s)
                self.rig.set_valves(set())
            except Exception:
                pass
            try:
                self.rig.close()
            except Exception:
                pass
        self.disconnect_lcr()


# ═══════════════════════════════════════════════════════════════════════════
#  GUI
# ═══════════════════════════════════════════════════════════════════════════

class App:
    def __init__(self, root, simulate=False):
        self.root = root
        self.q = queue.Queue()
        self.hw = Hardware(lambda k, v=None: self.q.put((k, v)))
        self.hist = collections.deque(maxlen=60000)
        self.t0 = time.monotonic()
        self.session_thread = None
        self.session_kind = None           # 'manual' | 'protocol'
        self.session_total_s = 0.0
        self.last_out_dir = None
        self.cfg = dict(DEFAULTS)
        self._load_settings()

        root.title('Actuator + LCR control')
        root.geometry('1500x900')
        root.minsize(1150, 720)
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        root.bind('<Escape>', lambda e: self.emergency_stop())

        self.v = {}                         # tk variables by name
        self._build_topbar()
        body = ttk.PanedWindow(root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=6, pady=(0, 6))
        left = ttk.Frame(body, width=500)
        right = ttk.Frame(body)
        body.add(left, weight=0)
        body.add(right, weight=1)
        nb = ttk.Notebook(left)
        nb.pack(fill='both', expand=True)
        self.tab_setup = ttk.Frame(nb, padding=8)
        self.tab_manual = ttk.Frame(nb, padding=8)
        self.tab_proto = ttk.Frame(nb, padding=8)
        self.tab_builder = ScrollFrame(nb)
        nb.add(self.tab_setup, text='Setup')
        nb.add(self.tab_manual, text='Manual')
        nb.add(self.tab_proto, text='Protocol')
        nb.add(self.tab_builder, text='Ramps + valves')
        self._build_setup(self.tab_setup)
        self._build_manual(self.tab_manual)
        self._build_protocol(self.tab_proto)
        self._build_builder(self.tab_builder.inner)
        self._build_plots(right)

        self.v['simulate'].set(simulate)
        self._monitor_stop = threading.Event()
        threading.Thread(target=self._monitor_loop, daemon=True, name='monitor').start()
        self.root.after(100, self._ui_tick)
        self.root.after(300, self._plot_tick)
        self.root.after(400, self.builder_preview)
        self._refresh_ports()
        self._set_controls_state()

    # ── settings ────────────────────────────────────────────────────────
    def _load_settings(self):
        try:
            with open(SETTINGS_FILE) as fh:
                self.cfg.update(json.load(fh))
        except (OSError, ValueError):
            pass

    def _save_settings(self):
        out = {}
        for k in DEFAULTS:
            if k == 'aliases':
                out[k] = [v.get().strip() for v in self.alias_vars]
            elif k == 'builder_steps':
                out[k] = self._builder_rows()
            elif k == 'builder_valves':
                out[k] = [{f: row[f].get() for f in row if not f.startswith('_')}
                          for row in self.bvalve]
            elif k in self.v:
                out[k] = self.v[k].get()
        try:
            with open(SETTINGS_FILE, 'w') as fh:
                json.dump(out, fh, indent=2)
        except OSError:
            pass

    def _var(self, name, kind=tk.StringVar):
        val = self.cfg.get(name, DEFAULTS.get(name, ''))
        if kind is tk.BooleanVar:
            val = bool(val)
        v = kind(value=val)
        self.v[name] = v
        return v

    # ── top bar ─────────────────────────────────────────────────────────
    def _build_topbar(self):
        bar = tk.Frame(self.root, bg=C_BROWN, padx=14, pady=8)
        bar.pack(fill='x')
        tk.Frame(self.root, bg=C_YELLOW, height=4).pack(fill='x')     # accent stripe
        big = ('Helvetica', 22, 'bold')
        small = ('Helvetica', 11)

        def readout(parent, title, width=9):
            f = tk.Frame(parent, bg=C_BROWN, padx=0)
            f.pack(side='left', padx=(0, 26))
            tk.Label(f, text=title, font=small, fg='#E2D3BE', bg=C_BROWN).pack(anchor='w')
            lab = tk.Label(f, text='—', font=big, fg='#FFFFFF', bg=C_BROWN,
                           width=width, anchor='w')
            lab.pack(anchor='w')
            return lab
        self.lab_p = readout(bar, 'Pressure (mbar)', 7)
        self.lab_sp = readout(bar, 'Set-point (mbar)', 7)
        self.lab_lcr1 = readout(bar, 'LCR primary', 14)
        self.lab_lcr2 = readout(bar, 'LCR secondary', 14)
        self.lab_p.configure(fg=C_YELLOW)
        right = tk.Frame(bar, bg=C_BROWN)
        right.pack(side='right')
        self.btn_stop = ttk.Button(right, text='STOP / VENT', style='Stop.TButton',
                                   command=self.emergency_stop)
        self.btn_stop.pack(side='right', padx=(14, 0))
        st = tk.Frame(right, bg=C_BROWN)
        st.pack(side='right')
        self.lab_conn = tk.Label(st, text='Fluigent: —   LCR: —', font=small,
                                 fg='#E2D3BE', bg=C_BROWN)
        self.lab_conn.pack(anchor='e')
        self.lab_state = tk.Label(st, text='idle', font=('Helvetica', 14, 'bold'),
                                  fg='#FFFFFF', bg=C_BROWN)
        self.lab_state.pack(anchor='e')

    # ── Setup tab ───────────────────────────────────────────────────────
    def _build_setup(self, f):
        g = ttk.LabelFrame(f, text='Fluigent (Flow EZ + P-Switch)', padding=8)
        g.pack(fill='x')
        ttk.Checkbutton(g, text='Simulate (no hardware)',
                        variable=self._var('simulate', tk.BooleanVar)).grid(row=0, column=0, sticky='w')
        self.btn_fgt = ttk.Button(g, text='Connect Fluigent', style='Accent.TButton',
                                  command=self.connect_fluigent)
        self.btn_fgt.grid(row=0, column=1, sticky='e', padx=4)
        self.lab_fgt = ttk.Label(g, text='not connected', foreground=C_TEXT2)
        self.lab_fgt.grid(row=1, column=0, columnspan=2, sticky='w', pady=(4, 0))
        g.columnconfigure(0, weight=1)

        g = ttk.LabelFrame(f, text='LCR-6100', padding=8)
        g.pack(fill='x', pady=8)
        r = 0
        ttk.Label(g, text='Port').grid(row=r, column=0, sticky='w')
        self.cb_port = ttk.Combobox(g, textvariable=self._var('lcr_port'), width=28)
        self.cb_port.grid(row=r, column=1, sticky='we')
        ttk.Button(g, text='↻', width=3, command=self._refresh_ports).grid(row=r, column=2)
        r += 1
        ttk.Label(g, text='Measure').grid(row=r, column=0, sticky='w', pady=(6, 0))
        cb = ttk.Combobox(g, textvariable=self._var('lcr_preset'), state='readonly',
                          values=list(MEASURE_PRESETS), width=26)
        cb.grid(row=r, column=1, columnspan=2, sticky='we', pady=(6, 0))
        cb.bind('<<ComboboxSelected>>', lambda e: self._preset_changed())
        r += 1
        ttk.Label(g, text='Function').grid(row=r, column=0, sticky='w')
        self.cb_func = ttk.Combobox(g, textvariable=self._var('lcr_func'), state='readonly',
                                    values=list(lcr6100.FUNCTIONS), width=12)
        self.cb_func.grid(row=r, column=1, sticky='w')
        r += 1
        for key, label, unit in (('lcr_freq', 'Frequency', 'Hz'), ('lcr_volt', 'Level', 'V'),
                                 ('lcr_avg', 'Averaging', '0–256, blank = keep')):
            ttk.Label(g, text=label).grid(row=r, column=0, sticky='w')
            ttk.Entry(g, textvariable=self._var(key), width=10).grid(row=r, column=1, sticky='w')
            ttk.Label(g, text=unit, foreground=C_TEXT2).grid(row=r, column=2, sticky='w')
            r += 1
        ttk.Label(g, text='Speed').grid(row=r, column=0, sticky='w')
        ttk.Combobox(g, textvariable=self._var('lcr_speed'), state='readonly',
                     values=['FAST', 'MED', 'SLOW'], width=8).grid(row=r, column=1, sticky='w')
        r += 1
        ttk.Label(g, text='Terminator').grid(row=r, column=0, sticky='w')
        ttk.Combobox(g, textvariable=self._var('lcr_eol'), state='readonly',
                     values=['crlf', 'lf', 'cr'], width=8).grid(row=r, column=1, sticky='w')
        r += 1
        bf = ttk.Frame(g)
        bf.grid(row=r, column=0, columnspan=3, sticky='we', pady=(6, 0))
        self.btn_lcr = ttk.Button(bf, text='Connect LCR', style='Accent.TButton',
                                  command=self.connect_lcr)
        self.btn_lcr.pack(side='left')
        self.btn_lcr_apply = ttk.Button(bf, text='Apply settings', command=self.apply_lcr)
        self.btn_lcr_apply.pack(side='left', padx=4)
        ttk.Button(bf, text='Disconnect', command=self.disconnect_lcr).pack(side='left')
        r += 1
        self.lab_lcr = ttk.Label(g, text='not connected', foreground=C_TEXT2, wraplength=380)
        self.lab_lcr.grid(row=r, column=0, columnspan=3, sticky='w', pady=(4, 0))
        g.columnconfigure(1, weight=1)
        self._preset_changed(init=True)

        g = ttk.LabelFrame(f, text='Safety', padding=8)
        g.pack(fill='x')
        ttk.Label(g, text='Max pressure').grid(row=0, column=0, sticky='w')
        e = ttk.Entry(g, textvariable=self._var('max_p'), width=8)
        e.grid(row=0, column=1, sticky='w')
        e.bind('<FocusOut>', lambda ev: self._apply_max_p())
        e.bind('<Return>', lambda ev: self._apply_max_p())
        ttk.Label(g, text='mbar (ceiling for everything)', foreground=C_TEXT2).grid(row=0, column=2, sticky='w')
        ttk.Label(g, text='Vent time').grid(row=1, column=0, sticky='w')
        ttk.Entry(g, textvariable=self._var('vent_s'), width=8).grid(row=1, column=1, sticky='w')
        ttk.Label(g, text='s at 0 mbar before valves close', foreground=C_TEXT2).grid(row=1, column=2, sticky='w')

        g = ttk.LabelFrame(f, text='Actuator names (P-Switch outputs)', padding=8)
        g.pack(fill='x', pady=8)
        self.alias_vars = []
        aliases = list(self.cfg.get('aliases', DEFAULTS['aliases'])) + [''] * 8
        for i in range(core.N_OUTPUTS):
            ttk.Label(g, text=f'out{i + 1}').grid(row=i // 4 * 2, column=i % 4, sticky='w', padx=4)
            v = tk.StringVar(value=aliases[i])
            v.trace_add('write', lambda *a: self._update_valve_labels())
            ttk.Entry(g, textvariable=v, width=9).grid(row=i // 4 * 2 + 1, column=i % 4, padx=4, pady=(0, 4))
            self.alias_vars.append(v)
        ttk.Label(g, text='Use these names in Manual and in sweep valve lists.',
                  foreground=C_TEXT2).grid(row=4, column=0, columnspan=4, sticky='w')

    def _preset_changed(self, init=False):
        func = MEASURE_PRESETS.get(self.v['lcr_preset'].get())
        if func:
            self.v['lcr_func'].set(func)
            self.cb_func.configure(state='disabled')
        else:
            self.cb_func.configure(state='readonly')

    def _refresh_ports(self):
        try:
            ports = lcr6100.list_ports()
        except Exception:
            ports = []
        devs = [d for d, _ in ports if 'usb' in d.lower()] or [d for d, _ in ports]
        self.cb_port.configure(values=devs)
        if not self.v['lcr_port'].get() and devs:
            self.v['lcr_port'].set(devs[0])

    def aliases(self):
        return {v.get().strip(): f'out{i + 1}' for i, v in enumerate(self.alias_vars)
                if v.get().strip()}

    def valve_name(self, i):
        a = self.alias_vars[i].get().strip()
        return f'{a}\n(out{i + 1})' if a else f'out{i + 1}'

    # ── Manual tab ──────────────────────────────────────────────────────
    def _build_manual(self, f):
        g = ttk.LabelFrame(f, text='Valves (P-Switch outputs) — click to open / close', padding=8)
        g.pack(fill='x')
        self.valve_btns = []
        self.valve_state = [tk.BooleanVar(value=False) for _ in range(core.N_OUTPUTS)]
        for i in range(core.N_OUTPUTS):
            b = ttk.Checkbutton(g, text=self.valve_name(i), variable=self.valve_state[i],
                                style='Valve.Toolbutton', width=9,
                                command=lambda i=i: self.toggle_valve(i))
            b.grid(row=i // 4, column=i % 4, padx=3, pady=3, sticky='we')
            self.valve_btns.append(b)
        bf = ttk.Frame(g)
        bf.grid(row=2, column=0, columnspan=4, sticky='we', pady=(4, 0))
        ttk.Button(bf, text='Close all', command=lambda: self.set_all_valves(False)).pack(side='left')
        ttk.Button(bf, text='Open all', command=lambda: self.set_all_valves(True)).pack(side='left', padx=4)
        for c in range(4):
            g.columnconfigure(c, weight=1)

        g = ttk.LabelFrame(f, text='Pressure', padding=8)
        g.pack(fill='x', pady=8)
        self.v['sp_slider'] = tk.DoubleVar(value=0.0)
        self.scale = tk.Scale(g, from_=0, to=self._max_p(), orient='horizontal',
                              variable=self.v['sp_slider'], resolution=1, length=300,
                              label='Set-point (mbar) — release to apply', showvalue=True,
                              bg=C_BEIGE, fg=C_TEXT, troughcolor='#FFFFFF',
                              activebackground=C_YELLOW, highlightthickness=0, bd=0,
                              sliderrelief='flat', font=('Helvetica', 11))
        self.scale.grid(row=0, column=0, columnspan=4, sticky='we')
        self.scale.bind('<ButtonRelease-1>', lambda e: self.set_pressure(self.v['sp_slider'].get()))
        ttk.Label(g, text='Set to').grid(row=1, column=0, sticky='w', pady=(6, 0))
        self.v['sp_entry'] = tk.StringVar(value='0')
        e = ttk.Entry(g, textvariable=self.v['sp_entry'], width=8)
        e.grid(row=1, column=1, sticky='w', pady=(6, 0))
        e.bind('<Return>', lambda ev: self.set_pressure(self.v['sp_entry'].get()))
        ttk.Button(g, text='Set', command=lambda: self.set_pressure(self.v['sp_entry'].get())).grid(
            row=1, column=2, sticky='w', pady=(6, 0))
        ttk.Label(g, text='Ramp to').grid(row=2, column=0, sticky='w', pady=(6, 0))
        ttk.Entry(g, textvariable=self._var('ramp_target'), width=8).grid(row=2, column=1, sticky='w', pady=(6, 0))
        rf = ttk.Frame(g)
        rf.grid(row=2, column=2, columnspan=2, sticky='w', pady=(6, 0))
        ttk.Label(rf, text='in').pack(side='left')
        ttk.Entry(rf, textvariable=self._var('ramp_time'), width=5).pack(side='left', padx=2)
        ttk.Label(rf, text='s').pack(side='left')
        ttk.Button(rf, text='Ramp', command=self.manual_ramp).pack(side='left', padx=6)
        bf = ttk.Frame(g)
        bf.grid(row=3, column=0, columnspan=4, sticky='w', pady=(8, 0))
        ttk.Button(bf, text='Deflate (ramp to 0)', command=lambda: self.manual_ramp(0)).pack(side='left')
        ttk.Button(bf, text='0 mbar now', command=lambda: self.set_pressure(0)).pack(side='left', padx=4)
        g.columnconfigure(3, weight=1)

        g = ttk.LabelFrame(f, text='Record manual session', padding=8)
        g.pack(fill='x')
        ttk.Label(g, text='Name').grid(row=0, column=0, sticky='w')
        self.v['manual_name'] = tk.StringVar(value='manual')
        ttk.Entry(g, textvariable=self.v['manual_name'], width=20).grid(row=0, column=1, sticky='w')
        self.btn_rec = ttk.Button(g, text='●  Start recording', command=self.toggle_recording)
        self.btn_rec.grid(row=1, column=0, columnspan=2, sticky='w', pady=(6, 0))
        self.lab_rec = ttk.Label(g, text='Everything you do here is logged to a session folder '
                                 'and analysed when you stop.', foreground=C_TEXT2, wraplength=380)
        self.lab_rec.grid(row=2, column=0, columnspan=2, sticky='w', pady=(4, 0))

    def _update_valve_labels(self):
        if hasattr(self, 'valve_btns'):
            for i, b in enumerate(self.valve_btns):
                b.configure(text=self.valve_name(i))
        if hasattr(self, 'bvalve_lab'):
            for i, lab in enumerate(self.bvalve_lab):
                lab.configure(text=self._bv_name(i))

    # ── Protocol tab ────────────────────────────────────────────────────
    def _build_protocol(self, f):
        src = self._var('source')
        g = ttk.LabelFrame(f, text='What to run', padding=8)
        g.pack(fill='x')
        ttk.Radiobutton(g, text='Quick sweep', value='sweep', variable=src).grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(g, text='JSON protocol file', value='file', variable=src).grid(row=0, column=1, sticky='w')
        ttk.Radiobutton(g, text='Ramps + valves tab', value='builder', variable=src).grid(row=0, column=2, sticky='w')

        s = ttk.LabelFrame(f, text='Quick sweep', padding=8)
        s.pack(fill='x', pady=6)
        rows = (('sweep_valves', 'Valve combos', "';' between tests, '+' together: A1;A2;A1+A2"),
                ('sweep_setpoints', 'Set-points', 'mbar, comma separated'),
                ('sweep_ramp', 'Ramp', 's up and down'),
                ('sweep_hold', 'Hold', 's at set-point'),
                ('sweep_rest', 'Rest', 's at 0 after deflation'),
                ('sweep_cycles', 'Cycles', 'per set-point'),
                ('sweep_baseline', 'Baseline', 's at 0 mbar start / end'))
        for r, (k, lab, hint) in enumerate(rows):
            ttk.Label(s, text=lab).grid(row=r, column=0, sticky='w')
            ttk.Entry(s, textvariable=self._var(k), width=16 if r else 22).grid(row=r, column=1, sticky='w')
            ttk.Label(s, text=hint, foreground=C_TEXT2).grid(row=r, column=2, sticky='w', padx=4)

        fl = ttk.LabelFrame(f, text='JSON protocol', padding=8)
        fl.pack(fill='x')
        ttk.Button(fl, text='Load…', command=self.pick_protocol).grid(row=0, column=0, sticky='w')
        self.lab_pfile = ttk.Label(fl, textvariable=self._var('protocol_file'),
                                   foreground=C_TEXT2, wraplength=320)
        self.lab_pfile.grid(row=0, column=1, sticky='w', padx=6)

        o = ttk.LabelFrame(f, text='Run', padding=8)
        o.pack(fill='x', pady=6)
        ttk.Label(o, text='Name').grid(row=0, column=0, sticky='w')
        ttk.Entry(o, textvariable=self._var('name'), width=18).grid(row=0, column=1, sticky='w')
        ttk.Label(o, text='Repeat').grid(row=0, column=2, sticky='w', padx=(8, 0))
        ttk.Entry(o, textvariable=self._var('repeat'), width=4).grid(row=0, column=3, sticky='w')
        ttk.Label(o, text='Log rate (Hz)').grid(row=1, column=0, sticky='w')
        ttk.Entry(o, textvariable=self._var('log_rate'), width=6).grid(row=1, column=1, sticky='w')
        ttk.Checkbutton(o, text='Analyse at end', variable=self._var('analyze', tk.BooleanVar)).grid(
            row=2, column=0, columnspan=2, sticky='w')
        ttk.Checkbutton(o, text='Open plots', variable=self._var('open_plots', tk.BooleanVar)).grid(
            row=2, column=2, columnspan=2, sticky='w')
        bf = ttk.Frame(o)
        bf.grid(row=3, column=0, columnspan=4, sticky='w', pady=(6, 0))
        ttk.Button(bf, text='Preview', command=self.preview).pack(side='left')
        self.btn_run = ttk.Button(bf, text='▶  Run', style='Accent.TButton',
                                  command=self.run_protocol)
        self.btn_run.pack(side='left', padx=4)
        self.btn_pstop = ttk.Button(bf, text='■  Stop', command=self.stop_session)
        self.btn_pstop.pack(side='left')
        self.btn_folder = ttk.Button(bf, text='Open results', command=self.open_results)
        self.btn_folder.pack(side='left', padx=4)
        self.prog = ttk.Progressbar(o, mode='determinate', length=340)
        self.prog.grid(row=4, column=0, columnspan=4, sticky='we', pady=(8, 0))
        self.lab_step = ttk.Label(o, text='', foreground=C_TEXT2, wraplength=380)
        self.lab_step.grid(row=5, column=0, columnspan=4, sticky='w')

        self.txt = tk.Text(f, height=10, font=('Menlo', 10), wrap='none', bg='#FFFFFF',
                           fg=C_TEXT, relief='flat', highlightthickness=1,
                           highlightbackground='#CBBFAE', padx=6, pady=4)
        self.txt.pack(fill='both', expand=True)

    # ── Ramps + valves tab ──────────────────────────────────────────────
    def _build_builder(self, f):
        # 1. pressure profile -------------------------------------------
        g = ttk.LabelFrame(f, text='1 · Pressure profile (runs top to bottom)', padding=8)
        g.pack(fill='x')
        cols = ('type', 'mbar', 'time')
        self.tree = ttk.Treeview(g, columns=cols, show='headings', height=7,
                                 selectmode='browse')
        for c, w, t in (('type', 90, 'Step'), ('mbar', 110, 'To / at (mbar)'),
                        ('time', 90, 'Time (s)')):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor='center')
        self.tree.grid(row=0, column=0, columnspan=6, sticky='we')
        sb = ttk.Scrollbar(g, orient='vertical', command=self.tree.yview)
        sb.grid(row=0, column=6, sticky='ns')
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.bind('<<TreeviewSelect>>', lambda e: self._builder_pick())
        for row in self.cfg.get('builder_steps', DEFAULTS['builder_steps']):
            self.tree.insert('', 'end', values=tuple(row))

        self.v['b_type'] = tk.StringVar(value='ramp')
        self.v['b_p'] = tk.StringVar(value='100')
        self.v['b_t'] = tk.StringVar(value='3')
        ef = ttk.Frame(g)
        ef.grid(row=1, column=0, columnspan=7, sticky='we', pady=(6, 0))
        ttk.Combobox(ef, textvariable=self.v['b_type'], values=('ramp', 'hold'),
                     state='readonly', width=6).pack(side='left')
        ttk.Label(ef, text=' to').pack(side='left')
        ttk.Entry(ef, textvariable=self.v['b_p'], width=6).pack(side='left', padx=2)
        ttk.Label(ef, text='mbar in').pack(side='left')
        ttk.Entry(ef, textvariable=self.v['b_t'], width=5).pack(side='left', padx=2)
        ttk.Label(ef, text='s').pack(side='left')
        ttk.Button(ef, text='Add', command=self._builder_add).pack(side='left', padx=(8, 2))
        ttk.Button(ef, text='Update', command=self._builder_update).pack(side='left')
        bf = ttk.Frame(g)
        bf.grid(row=2, column=0, columnspan=7, sticky='w', pady=(4, 0))
        for txt, cmd in (('Remove', self._builder_remove), ('↑', lambda: self._builder_move(-1)),
                         ('↓', lambda: self._builder_move(1)), ('Clear all', self._builder_clear)):
            ttk.Button(bf, text=txt, command=cmd, width=8 if len(txt) > 1 else 3).pack(
                side='left', padx=(0, 4))
        ttk.Label(g, text="ramp = linear change to the value · hold = stay at the value",
                  style='Hint.TLabel').grid(row=3, column=0, columnspan=7, sticky='w', pady=(4, 0))

        # 2. ramp generator ---------------------------------------------
        g = ttk.LabelFrame(f, text='2 · Generate a ramp series (appended to the profile)',
                           padding=8)
        g.pack(fill='x', pady=8)
        fields = (('gen_peak', 'Peak', 'mbar'), ('gen_step', '+ per ramp', 'mbar'),
                  ('gen_n', 'Ramps', '×'), ('gen_up', 'Up', 's'), ('gen_hold', 'Hold', 's'),
                  ('gen_down', 'Down', 's'), ('gen_rest', 'Rest at 0', 's'))
        for k, (key, lab, unit) in enumerate(fields):
            r, c = divmod(k, 4)
            cell = ttk.Frame(g)
            cell.grid(row=r, column=c, sticky='w', padx=(0, 10), pady=2)
            ttk.Label(cell, text=lab, style='Hint.TLabel').pack(anchor='w')
            e = ttk.Frame(cell)
            e.pack(anchor='w')
            ttk.Entry(e, textvariable=self._var(key), width=6).pack(side='left')
            ttk.Label(e, text=unit, style='Hint.TLabel').pack(side='left', padx=2)
        ttk.Button(g, text='Add ramps', command=self._builder_generate).grid(
            row=1, column=3, sticky='se', pady=2)
        ttk.Label(g, text="'+ per ramp' > 0 makes increasing peaks (e.g. 50, 100, 150 …)",
                  style='Hint.TLabel').grid(row=2, column=0, columnspan=4, sticky='w')

        # 3. asynchronous valves ----------------------------------------
        g = ttk.LabelFrame(f, text='3 · Valve timing — each valve on its own clock', padding=8)
        g.pack(fill='x')
        for c, t in enumerate(('Output', 'Mode', 'Start at', 'Open for', 'Closed for',
                               'Pulses')):
            ttk.Label(g, text=t, style='Hint.TLabel').grid(row=0, column=c, sticky='w', padx=2)
        ttk.Label(g, text='', style='Hint.TLabel').grid(row=0, column=6)
        saved = list(self.cfg.get('builder_valves', DEFAULTS['builder_valves']))
        saved += DEFAULTS['builder_valves'][len(saved):]
        self.bvalve = []
        self.bvalve_lab = []
        for i in range(core.N_OUTPUTS):
            sv = saved[i]
            row = {k: tk.StringVar(value=str(sv.get(k, DEFAULTS['builder_valves'][1][k])))
                   for k in ('mode', 'delay', 'on', 'off', 'n')}
            lab = ttk.Label(g, text=self._bv_name(i), width=10)
            lab.grid(row=i + 1, column=0, sticky='w', padx=2)
            self.bvalve_lab.append(lab)
            cb = ttk.Combobox(g, textvariable=row['mode'], values=VALVE_MODES,
                              state='readonly', width=7)
            cb.grid(row=i + 1, column=1, padx=2, pady=1)
            ents = []
            for c, k in enumerate(('delay', 'on', 'off', 'n')):
                e = ttk.Entry(g, textvariable=row[k], width=6)
                e.grid(row=i + 1, column=c + 2, padx=2, pady=1)
                ents.append(e)
            row['_ents'] = ents
            cb.bind('<<ComboboxSelected>>', lambda e, i=i: self._bv_mode_changed(i))
            self.bvalve.append(row)
            self._bv_mode_changed(i, init=True)
        ttk.Label(g, text='s', style='Hint.TLabel').grid(row=1, column=6, sticky='w')
        hint = ('closed = never opens · open = opens at "Start at" and stays open · '
                'pulse = open / closed cycles from "Start at"; Pulses 0 = until the end')
        ttk.Label(g, text=hint, style='Hint.TLabel', wraplength=440).grid(
            row=core.N_OUTPUTS + 1, column=0, columnspan=7, sticky='w', pady=(4, 0))
        sf = ttk.Frame(g)
        sf.grid(row=core.N_OUTPUTS + 2, column=0, columnspan=7, sticky='w', pady=(6, 0))
        ttk.Label(sf, text='Stagger: each next active valve starts').pack(side='left')
        ttk.Entry(sf, textvariable=self._var('stagger_s'), width=5).pack(side='left', padx=3)
        ttk.Label(sf, text='s later').pack(side='left')
        ttk.Button(sf, text='Apply', command=self._builder_stagger).pack(side='left', padx=6)
        ttk.Button(sf, text='All closed', command=lambda: [r['mode'].set('closed') or
                   self._bv_mode_changed(i) for i, r in enumerate(self.bvalve)]).pack(side='left')

        # 4. preview + run ----------------------------------------------
        g = ttk.LabelFrame(f, text='4 · Preview and run', padding=8)
        g.pack(fill='x', pady=8)
        self.bfig = Figure(figsize=(4.8, 2.9), layout='constrained', facecolor=C_SURFACE)
        self.bax_p, self.bax_v = self.bfig.subplots(
            2, 1, sharex=True, gridspec_kw={'height_ratios': [1.2, 1]})
        for ax in (self.bax_p, self.bax_v):
            ana._style(ax)
        self.bcanvas = FigureCanvasTkAgg(self.bfig, master=g)
        self.bcanvas.get_tk_widget().configure(bg=C_SURFACE, highlightthickness=0)
        self.bcanvas.get_tk_widget().pack(fill='x')
        self.lab_btotal = ttk.Label(g, text='', style='Hint.TLabel', wraplength=440)
        self.lab_btotal.pack(anchor='w', pady=(4, 0))
        rf = ttk.Frame(g)
        rf.pack(fill='x', pady=(6, 0))
        ttk.Label(rf, text='Name').pack(side='left')
        ttk.Entry(rf, textvariable=self.v['name'], width=14).pack(side='left', padx=(3, 8))
        ttk.Label(rf, text='Repeat').pack(side='left')
        ttk.Entry(rf, textvariable=self.v['repeat'], width=4).pack(side='left', padx=3)
        bf = ttk.Frame(g)
        bf.pack(fill='x', pady=(6, 0))
        ttk.Button(bf, text='Preview', command=self.builder_preview).pack(side='left')
        self.btn_brun = ttk.Button(bf, text='▶  Run', style='Accent.TButton',
                                   command=self.builder_run)
        self.btn_brun.pack(side='left', padx=4)
        self.btn_bstop = ttk.Button(bf, text='■  Stop', command=self.stop_session)
        self.btn_bstop.pack(side='left')
        ttk.Button(bf, text='Save…', command=self.builder_save).pack(side='left', padx=(12, 2))
        ttk.Button(bf, text='Load…', command=self.builder_load).pack(side='left')
        self.prog_b = ttk.Progressbar(g, mode='determinate')
        self.prog_b.pack(fill='x', pady=(8, 0))
        self.lab_bstep = ttk.Label(g, text='', style='Hint.TLabel', wraplength=440)
        self.lab_bstep.pack(anchor='w')

    def _bv_name(self, i):
        a = self.alias_vars[i].get().strip() if hasattr(self, 'alias_vars') else ''
        return f'{a} (out{i + 1})' if a else f'out{i + 1}'

    def _bv_mode_changed(self, i, init=False):
        row = self.bvalve[i]
        mode = row['mode'].get()
        states = {'closed': (0, 0, 0, 0), 'open': (1, 0, 0, 0), 'pulse': (1, 1, 1, 1)}[mode]
        for e, on in zip(row['_ents'], states):
            e.configure(state='normal' if on else 'disabled')

    def aliases_rev(self):
        return {a: o for a, o in self.aliases().items()}

    def _builder_rows(self):
        return [list(self.tree.item(i, 'values')) for i in self.tree.get_children()]

    def _builder_entry(self):
        typ = self.v['b_type'].get()
        p = self._num('b_p', lo=0)
        t = self._num('b_t', lo=0)
        if p > self._max_p():
            raise ValueError(f'{p:g} mbar is above the {self._max_p():g} mbar ceiling')
        if typ == 'hold' and t <= 0:
            raise ValueError('a hold needs a time > 0')
        return (typ, f'{p:g}', f'{t:g}')

    def _builder_add(self):
        try:
            vals = self._builder_entry()
        except ValueError as e:
            return messagebox.showerror('Profile', str(e))
        sel = self.tree.selection()
        idx = self.tree.index(sel[0]) + 1 if sel else 'end'
        self.tree.selection_set(self.tree.insert('', idx, values=vals))
        self.builder_preview()

    def _builder_update(self):
        sel = self.tree.selection()
        if not sel:
            return messagebox.showinfo('Profile', 'Select a row to update.')
        try:
            self.tree.item(sel[0], values=self._builder_entry())
        except ValueError as e:
            return messagebox.showerror('Profile', str(e))
        self.builder_preview()

    def _builder_pick(self):
        sel = self.tree.selection()
        if sel:
            typ, p, t = self.tree.item(sel[0], 'values')
            self.v['b_type'].set(typ)
            self.v['b_p'].set(p)
            self.v['b_t'].set(t)

    def _builder_remove(self):
        for i in self.tree.selection():
            self.tree.delete(i)
        self.builder_preview()

    def _builder_move(self, d):
        sel = self.tree.selection()
        if sel:
            i = self.tree.index(sel[0])
            self.tree.move(sel[0], '', max(0, i + d))
            self.builder_preview()

    def _builder_clear(self):
        if self.tree.get_children() and messagebox.askyesno('Profile', 'Remove all steps?'):
            self.tree.delete(*self.tree.get_children())
            self.builder_preview()

    def _builder_generate(self):
        try:
            peak = self._num('gen_peak', lo=0)
            inc = self._num('gen_step')
            n = self._num('gen_n', lo=1, hi=500, integer=True)
            up, hold = self._num('gen_up', lo=0), self._num('gen_hold', lo=0)
            down, rest = self._num('gen_down', lo=0), self._num('gen_rest', lo=0)
            top = max(peak, peak + inc * (n - 1))
            if top > self._max_p():
                raise ValueError(f'the series reaches {top:g} mbar, above the '
                                 f'{self._max_p():g} mbar ceiling')
            if min(peak, peak + inc * (n - 1)) <= 0:
                raise ValueError('every peak must be > 0 mbar')
        except ValueError as e:
            return messagebox.showerror('Ramp series', str(e))
        rows = self._builder_rows()
        if not rows or float(rows[-1][1]) != 0:
            self.tree.insert('', 'end', values=('ramp', '0', f'{down:g}') if rows
                             else ('hold', '0', f'{rest or 5:g}'))
        for k in range(n):
            pk = peak + inc * k
            self.tree.insert('', 'end', values=('ramp', f'{pk:g}', f'{up:g}'))
            if hold > 0:
                self.tree.insert('', 'end', values=('hold', f'{pk:g}', f'{hold:g}'))
            self.tree.insert('', 'end', values=('ramp', '0', f'{down:g}'))
            if rest > 0:
                self.tree.insert('', 'end', values=('hold', '0', f'{rest:g}'))
        self.builder_preview()

    def _builder_stagger(self):
        try:
            gap = self._num('stagger_s', lo=0)
        except ValueError as e:
            return messagebox.showerror('Stagger', str(e))
        active = [r for r in self.bvalve if r['mode'].get() != 'closed']
        if not active:
            return messagebox.showinfo('Stagger', 'Set at least one valve to open or pulse.')
        try:
            t0 = float(active[0]['delay'].get() or 0)
        except ValueError:
            t0 = 0.0
        for k, r in enumerate(active):
            r['delay'].set(f'{t0 + k * gap:g}')
        self.builder_preview()

    def builder_protocol(self):
        steps = []
        for k, (typ, p, t) in enumerate(self._builder_rows()):
            p, t = float(p), float(t)
            steps.append({'type': 'ramp', 'to': p, 'duration': t} if typ == 'ramp'
                         else {'type': 'hold', 'pressure': p, 'duration': t})
        if not steps:
            raise ValueError('The pressure profile is empty — add ramps / holds first.')
        timeline = []
        for i, r in enumerate(self.bvalve):
            mode = r['mode'].get()
            if mode == 'closed':
                continue
            name = self._bv_name(i)

            def num(key, lo=0.0, integer=False):
                raw = r[key].get().strip()
                try:
                    x = int(float(raw)) if integer else float(raw)
                except ValueError:
                    raise ValueError(f"{name}: '{raw}' is not a number") from None
                if x < lo:
                    raise ValueError(f'{name}: value must be ≥ {lo:g}')
                return x
            item = {'valves': f'out{i + 1}', 'delay': num('delay')}
            if mode == 'pulse':
                item.update(on=num('on'), off=num('off'), pulses=num('n', integer=True))
                if item['on'] <= 0:
                    raise ValueError(f'{name}: "Open for" must be > 0 s')
            timeline.append(item)
        if not timeline:
            raise ValueError('All valves are closed — set at least one valve to open or pulse.')
        return {'name': self.v['name'].get().strip() or 'ramps_valves',
                'actuators': self.aliases(), 'steps': steps, 'valve_timeline': timeline}

    def builder_preview(self):
        for lab, i in zip(self.bvalve_lab, range(core.N_OUTPUTS)):
            lab.configure(text=self._bv_name(i))
        for ax in (self.bax_p, self.bax_v):
            ax.clear()
            ana._style(ax)
        try:
            protocol = self.builder_protocol()
            segs = core.expand_protocol(protocol)
            core.validate(segs, self._max_p())
            total = sum(s['duration'] for s in segs)
            events = core.expand_valve_timeline(protocol, total)
        except Exception as e:
            self.lab_btotal.configure(text=f'⚠  {e}', foreground=C_STOP)
            self.bcanvas.draw_idle()
            return None
        # pressure profile
        t, p, tt = [0.0], [segs[0]['p0']], 0.0
        for s in segs:
            t += [tt, tt + s['duration']]
            p += [s['p0'], s['p1']]
            tt += s['duration']
        self.bax_p.fill_between(t, p, color=C_LGREEN, alpha=0.25, linewidth=0)
        self.bax_p.plot(t, p, color=C_PRESSURE, lw=1.8)
        self.bax_p.set_ylabel('mbar', fontsize=9)
        self.bax_p.set_title('Pressure profile', loc='left', fontsize=9, color=C_TEXT)
        # valve chart
        used = sorted({i for _, i, _ in events})
        for row, i in enumerate(used):
            spans, t_open = [], None
            for te, j, on in events:
                if j != i:
                    continue
                if on and t_open is None:
                    t_open = te
                elif not on and t_open is not None:
                    spans.append((t_open, te - t_open))
                    t_open = None
            if t_open is not None:
                spans.append((t_open, total - t_open))
            col = (C_GREEN, C_YELLOW)[row % 2]
            self.bax_v.broken_barh(spans, (row - 0.35, 0.7), facecolors=col, edgecolor='none')
        self.bax_v.set_yticks(range(len(used)))
        self.bax_v.set_yticklabels([self._bv_name(i).split(' ')[0] for i in used], fontsize=8)
        self.bax_v.set_ylim(-0.6, max(len(used), 1) - 0.4)
        self.bax_v.invert_yaxis()
        self.bax_v.set_xlim(0, max(total, 1e-3))
        self.bax_v.set_xlabel('Time (s)', fontsize=9)
        self.bax_v.set_title('Valves open (bars)', loc='left', fontsize=9, color=C_TEXT)
        late = [i for i, r in enumerate(self.bvalve) if r['mode'].get() != 'closed'
                and i not in used]
        rep = self._num('repeat', lo=1, integer=True) if self.v['repeat'].get().strip() else 1
        msg = (f'{total:.1f} s per run' + (f' × {rep} = {total * rep:.1f} s' if rep > 1 else '')
               + f' · peak {max(p):g} mbar · {len(events)} valve switches')
        if late:
            msg += ' · ⚠ never opens (starts after the end): ' + ', '.join(
                self._bv_name(i) for i in late)
        self.lab_btotal.configure(text=msg, foreground=C_STOP if late else C_TEXT2)
        self.bcanvas.draw_idle()
        return protocol

    def builder_run(self):
        if self.builder_preview() is None:
            return messagebox.showerror('Ramps + valves', self.lab_btotal.cget('text'))
        self.v['source'].set('builder')
        self.run_protocol()

    def builder_save(self):
        try:
            protocol = self.builder_protocol()
        except ValueError as e:
            return messagebox.showerror('Save', str(e))
        protocol['max_pressure_mbar'] = self._max_p()
        path = filedialog.asksaveasfilename(
            initialdir=os.path.join(HERE, 'protocols'), defaultextension='.json',
            initialfile=f"{protocol['name']}.json", filetypes=[('JSON protocol', '*.json')])
        if path:
            with open(path, 'w') as fh:
                json.dump(protocol, fh, indent=2)
            self.lab_btotal.configure(text=f'saved → {path}', foreground=C_TEXT2)

    def builder_load(self):
        path = filedialog.askopenfilename(initialdir=os.path.join(HERE, 'protocols'),
                                          filetypes=[('JSON protocol', '*.json')])
        if not path:
            return
        try:
            with open(path) as fh:
                pr = json.load(fh)
            rows, sp = [], 0.0
            for st in pr.get('steps', []):
                if st.get('type') == 'ramp':
                    sp = float(st['to'])
                    rows.append(('ramp', f'{sp:g}', f"{float(st['duration']):g}"))
                elif st.get('type') == 'hold':
                    sp = float(st.get('pressure', sp))
                    rows.append(('hold', f'{sp:g}', f"{float(st['duration']):g}"))
                else:
                    raise ValueError(f"step type '{st.get('type')}' can't be edited here — "
                                     "run this file from the Protocol tab instead")
            resolve = core.valve_resolver(pr.get('actuators'))
            modes = [{'mode': 'closed', 'delay': '0', 'on': '2', 'off': '1', 'n': '0'}
                     for _ in range(core.N_OUTPUTS)]
            for it in pr.get('valve_timeline', []):
                if 't' in it or 'until' in it:
                    raise ValueError('one-off events / "until" can\'t be edited here — '
                                     'run this file from the Protocol tab instead')
                for i in resolve(it['valves']):
                    m = modes[i]
                    m['delay'] = f"{float(it.get('delay', 0)):g}"
                    if 'on' in it:
                        m.update(mode='pulse', on=f"{float(it['on']):g}",
                                 off=f"{float(it.get('off', 0)):g}",
                                 n=str(int(it.get('pulses', 0))))
                    else:
                        m['mode'] = 'open'
        except Exception as e:
            return messagebox.showerror('Load', f'{type(e).__name__}: {e}')
        self.tree.delete(*self.tree.get_children())
        for r in rows:
            self.tree.insert('', 'end', values=r)
        for i, m in enumerate(modes):
            for k in ('mode', 'delay', 'on', 'off', 'n'):
                self.bvalve[i][k].set(m[k])
            self._bv_mode_changed(i)
        for a, o in (pr.get('actuators') or {}).items():
            try:
                self.alias_vars[int(str(o).lower().replace('out', '')) - 1].set(a)
            except (ValueError, IndexError):
                pass
        if pr.get('name'):
            self.v['name'].set(pr['name'])
        self.builder_preview()

    # ── plots ───────────────────────────────────────────────────────────
    def _build_plots(self, f):
        top = ttk.Frame(f)
        top.pack(fill='x')
        ttk.Label(top, text='Window').pack(side='left')
        cb = ttk.Combobox(top, textvariable=self._var('window_s'), width=6, state='readonly',
                          values=['15', '30', '60', '120', '300', '600'])
        cb.pack(side='left', padx=4)
        ttk.Label(top, text='s').pack(side='left')
        ttk.Button(top, text='Clear', command=self.hist.clear).pack(side='left', padx=8)

        self.fig = Figure(figsize=(9, 7), layout='constrained', facecolor=C_SURFACE)
        self._relayout = True
        gs = self.fig.add_gridspec(2, 2)
        self.ax_p = self.fig.add_subplot(gs[0, 0])
        self.ax_1 = self.fig.add_subplot(gs[0, 1], sharex=self.ax_p)
        self.ax_2 = self.fig.add_subplot(gs[1, 0], sharex=self.ax_p)
        self.ax_x = self.fig.add_subplot(gs[1, 1])
        for ax in (self.ax_p, self.ax_1, self.ax_2, self.ax_x):
            ana._style(ax)
        self.ln_sp, = self.ax_p.plot([], [], color=C_SET, ls='--', lw=1.2, label='set-point')
        self.ln_p, = self.ax_p.plot([], [], color=C_PRESSURE, lw=2, label='measured')
        self.ax_p.legend(loc='upper left', frameon=False, fontsize=8, labelcolor=C_TEXT2)
        self.ax_p.set_title('Pressure vs time', loc='left', fontsize=10)
        self.ax_p.set_ylabel('mbar')
        self.ax_p.set_xlabel('Time (s)')
        self.ln_1, = self.ax_1.plot([], [], color=C_INFLATE, lw=1.5)
        self.ln_2, = self.ax_2.plot([], [], color=C_INFLATE, lw=1.5)
        self.ax_2.set_xlabel('Time (s)')
        self.ax_1.set_xlabel('Time (s)')
        self.sc = {}
        for ph, col, lab in (('other', C_HOLD, 'hold'), ('inflate', C_INFLATE, 'inflation'),
                             ('deflate', C_DEFLATE, 'deflation')):
            self.sc[ph], = self.ax_x.plot([], [], ls='none', marker='o', ms=3, color=col,
                                          alpha=0.7, label=lab)
        self.ax_x.legend(loc='best', frameon=False, fontsize=8, labelcolor=C_TEXT2)
        self.ax_x.set_xlabel('Measured pressure (mbar)')
        self.canvas = FigureCanvasTkAgg(self.fig, master=f)
        self.canvas.get_tk_widget().configure(bg=C_SURFACE, highlightthickness=0)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.canvas.get_tk_widget().bind('<Configure>',
                                         lambda e: setattr(self, '_relayout', True), add='+')
        self._lcr_axes_key = None

    def _label_lcr_axes(self):
        lcr = self.hw.lcr
        func = getattr(lcr, 'function', None) if lcr else None
        key = (func,)
        if key == self._lcr_axes_key:
            return
        self._lcr_axes_key = key
        if not func:
            for ax, t in ((self.ax_1, 'LCR (not connected)'), (self.ax_2, 'LCR secondary'),
                          (self.ax_x, 'LCR vs pressure')):
                ax.set_title(t, loc='left', fontsize=10)
                ax.set_ylabel('')
            return
        pn, sn, units = lcr6100.FUNCTIONS[func]
        self._lcr_units = units
        p_pretty = ana.PRETTY.get(pn, pn)
        s_pretty = ana.PRETTY.get(sn, sn) if sn else '—'
        self.ax_1.set_title(f'{p_pretty} vs time', loc='left', fontsize=10)
        self.ax_2.set_title(f'{s_pretty} vs time', loc='left', fontsize=10)
        self.ax_x.set_title(f'{p_pretty} vs pressure', loc='left', fontsize=10)
        self._lcr_names = (p_pretty, s_pretty)

    # ── live data ───────────────────────────────────────────────────────
    def _on_row(self, r):
        r = dict(r)
        r['t'] = time.monotonic() - self.t0
        r['func'] = self._lcr_func()
        self.hist.append(r)

    def _lcr_func(self):
        return getattr(self.hw.lcr, 'function', None) if self.hw.lcr else None

    def _monitor_loop(self):
        """Reads pressure + LCR at 20 Hz whenever no session is logging."""
        while not self._monitor_stop.is_set():
            if self.hw.rig and not self.hw.session:
                try:
                    p = self.hw.rig.get_pressure()
                except Exception:
                    p = float('nan')
                smp = self.hw.lcr.get_sample() if self.hw.lcr else {}
                self.hist.append({'t': time.monotonic() - self.t0, 'setpoint': self.hw.setpoint,
                                  'pressure': p, 'phase': self.hw.phase,
                                  'valves': tuple(sorted(self.hw.open_set)),
                                  'primary': smp.get('primary', float('nan')),
                                  'secondary': smp.get('secondary', float('nan')),
                                  'ok': smp.get('ok', False), 'step': -1, 'label': '',
                                  'func': self._lcr_func()})
            elif self.hw.lcr and not self.hw.rig:
                smp = self.hw.lcr.get_sample()
                self.hist.append({'t': time.monotonic() - self.t0, 'setpoint': float('nan'),
                                  'pressure': float('nan'), 'phase': 'hold', 'valves': (),
                                  'primary': smp.get('primary', float('nan')),
                                  'secondary': smp.get('secondary', float('nan')),
                                  'ok': smp.get('ok', False), 'step': -1, 'label': '',
                                  'func': self._lcr_func()})
            self._monitor_stop.wait(0.05)

    def _ui_tick(self):
        """Fast loop: queue, numbers, progress (cheap)."""
        try:
            self._drain_queue()
            self._update_readouts()
            self._update_session_progress()
        except Exception as e:                    # never kill the UI loop
            print('[gui] tick error:', repr(e))
        self.root.after(UI_READOUT_MS, self._ui_tick)

    def _plot_tick(self):
        """Slow loop: redraw the live plots, pacing itself to the draw cost so
        the window never queues up behind matplotlib."""
        t = time.perf_counter()
        try:
            self._update_plots()
        except Exception as e:
            print('[gui] plot error:', repr(e))
        dt_ms = (time.perf_counter() - t) * 1000.0
        self.root.after(int(max(PLOT_MIN_MS, 1.5 * dt_ms)), self._plot_tick)

    def _drain_queue(self):
        while True:
            try:
                kind, val = self.q.get_nowait()
            except queue.Empty:
                return
            if kind == 'error':
                messagebox.showerror('Error', str(val))
            elif kind == 'info':
                messagebox.showinfo('Info', str(val))
            elif kind == 'fgt':
                self.lab_fgt.configure(text=val)
                self._set_controls_state()
            elif kind == 'lcr':
                self.lab_lcr.configure(text=val)
                self._lcr_axes_key = None
                self._set_controls_state()
            elif kind == 'valves':
                for i, v in enumerate(self.valve_state):
                    v.set(i in val)
            elif kind == 'session_done':
                self._session_finished(*val)
            elif kind == 'analysis':
                self._show_analysis(val)

    def _scale(self, idx, values):
        if not getattr(self, '_lcr_units', None):
            return 1.0, ''
        u = self._lcr_units[idx]
        if u is None:
            return 1.0, ''
        import numpy as np
        return ana.display_scale('x', u, np.asarray(values, dtype=float))

    def _update_readouts(self):
        h = self.hist[-1] if self.hist else None
        if h:
            self.lab_p.configure(text=f"{h['pressure']:.1f}" if h['pressure'] == h['pressure'] else '—')
            self.lab_sp.configure(text=f"{h['setpoint']:.1f}" if h['setpoint'] == h['setpoint'] else '—')
        lcr = self.hw.lcr
        if lcr and h and getattr(self, '_lcr_names', None):
            k1, u1 = self._scale(0, [h['primary']])
            k2, u2 = self._scale(1, [h['secondary']])
            v1 = h['primary'] * k1
            v2 = h['secondary'] * k2
            self.lab_lcr1.configure(text=f"{self._lcr_names[0]} {v1:.4g} {u1}" if v1 == v1 else '—')
            out = '' if h['ok'] else '  OUT'
            self.lab_lcr2.configure(text=(f"{self._lcr_names[1]} {v2:.4g} {u2}{out}"
                                          if v2 == v2 else out.strip() or '—'),
                                    fg='#FF8A7A' if out else '#FFFFFF')
        else:
            self.lab_lcr1.configure(text='—')
            self.lab_lcr2.configure(text='—', fg='#FFFFFF')
        fg = 'connected' if self.hw.rig else '—'
        lc = lcr.describe().replace('Mode: ', '') if lcr else '—'
        self.lab_conn.configure(text=f'Fluigent: {fg}    LCR: {lc}')
        if self.hw.session:
            kind = 'RECORDING (manual)' if self.session_kind == 'manual' else 'RUNNING protocol'
            self.lab_state.configure(text=f'● {kind}  {self.hw.session.rows} rows', fg=C_YELLOW)
        else:
            self.lab_state.configure(text='idle', fg='#FFFFFF')

    def _update_plots(self):
        self._label_lcr_axes()
        if not self.hist:
            if self._relayout:
                self._draw()
            return
        win = float(self.v['window_s'].get() or 60)
        t_now = self.hist[-1]['t']
        data = []
        for r in reversed(self.hist):               # newest first, stop at window start
            if r['t'] < t_now - win:
                break
            data.append(r)
        if not data:
            return
        data.reverse()
        if len(data) > PLOT_MAX_POINTS:              # thin for drawing speed
            step = math.ceil(len(data) / PLOT_MAX_POINTS)
            data = data[::step] + ([data[-1]] if (len(data) - 1) % step else [])
        t = [r['t'] for r in data]
        self.ln_sp.set_data(t, [r['setpoint'] for r in data])
        self.ln_p.set_data(t, [r['pressure'] for r in data])
        # only plot LCR samples taken in the current measurement function —
        # after switching e.g. Cp-D -> Z-thd the old values have other units
        func = self._lcr_func()
        nan = float('nan')
        prim = [r['primary'] if r.get('func') == func else nan for r in data]
        sec = [r['secondary'] if r.get('func') == func else nan for r in data]
        k1, u1 = self._scale(0, [x for x in prim if x == x] or [1.0])
        k2, u2 = self._scale(1, [x for x in sec if x == x] or [1.0])
        y1 = [x * k1 for x in prim]
        y2 = [x * k2 for x in sec]
        self.ln_1.set_data(t, y1)
        self.ln_2.set_data(t, y2)
        if getattr(self, '_lcr_names', None):
            self.ax_1.set_ylabel(f'{self._lcr_names[0]} ({u1})' if u1 else self._lcr_names[0])
            self.ax_2.set_ylabel(f'{self._lcr_names[1]} ({u2})' if u2 else self._lcr_names[1])
            self.ax_x.set_ylabel(self.ax_1.get_ylabel())
        cur = [r for r in data if r.get('func') == func]
        mostly_out = bool(cur) and sum(1 for r in cur if not r['ok']) > len(cur) / 2
        col = C_HOLD if mostly_out else C_INFLATE
        self.ln_1.set_color(col)
        self.ln_2.set_color(col)
        P = [r['pressure'] for r in data]
        for ph in ('other', 'inflate', 'deflate'):
            sel = [i for i, r in enumerate(data)
                   if (r['phase'] == ph) or (ph == 'other' and r['phase'] not in ('inflate', 'deflate'))]
            self.sc[ph].set_data([P[i] for i in sel], [y1[i] for i in sel])
        self.ax_p.set_xlim(max(0.0, t_now - win), max(t_now, 1.0))
        for ax in (self.ax_p, self.ax_1, self.ax_2, self.ax_x):
            ax.relim()
            ax.autoscale_view(scalex=(ax is self.ax_x), scaley=True)
        self._draw()

    def _draw(self):
        # constrained layout is expensive: solve it only after a resize or a
        # label change, then keep the axes where they are
        key = (self.ax_1.get_ylabel(), self.ax_2.get_ylabel(), self.ax_1.get_title(loc='left'))
        if key != getattr(self, '_layout_key', None):
            self._layout_key, self._relayout = key, True
        if self._relayout:
            self.fig.set_layout_engine('constrained')
            self.canvas.draw()
            self.fig.set_layout_engine('none')
            self._relayout = False
        else:
            self.canvas.draw()

    def _update_session_progress(self):
        s = self.hw.session
        if s and self.session_kind == 'protocol' and self.session_total_s > 0:
            el = s.state.get('elapsed', 0.0)
            self.prog['value'] = min(100.0, 100.0 * el / self.session_total_s)
            st = s.state
            txt = (f"{el:6.1f} / {self.session_total_s:.0f} s   "
                   f"step {st['step']}: {st['label']}  ({st['phase']})")
            self.lab_step.configure(text=txt)
            self.prog_b['value'] = self.prog['value']
            self.lab_bstep.configure(text=txt + '   open: ' + core.valve_label(
                tuple(st.get('valves', ())), self.aliases_rev()))
            for i, v in enumerate(self.valve_state):
                v.set(i in st.get('valves', ()))
        elif s and self.session_kind == 'manual':
            self.lab_rec.configure(text=f"Recording… {s.state.get('elapsed', 0):.1f} s, "
                                        f"{s.rows} rows → {s.out_dir}")

    # ── helpers ─────────────────────────────────────────────────────────
    def _num(self, name, value=None, lo=None, hi=None, integer=False):
        raw = self.v[name].get() if value is None else value
        try:
            x = int(float(raw)) if integer else float(raw)
        except (TypeError, ValueError):
            raise ValueError(f"'{raw}' is not a number ({name})")
        if (lo is not None and x < lo) or (hi is not None and x > hi):
            raise ValueError(f'{name} = {x} must be within {lo}..{hi}')
        return x

    def _max_p(self):
        try:
            return max(0.0, float(self.v['max_p'].get()))
        except (KeyError, ValueError):
            return 200.0

    def _apply_max_p(self):
        mp = self._max_p()
        self.hw.max_p = mp
        self.scale.configure(to=mp)

    def _busy(self, fn, *args, done=None):
        """Run fn in a thread; errors go to a message box."""
        def work():
            try:
                res = fn(*args)
                if done:
                    self.q.put(done(res))
            except Exception as e:
                self.q.put(('error', f'{type(e).__name__}: {e}'))
        threading.Thread(target=work, daemon=True).start()

    def _set_controls_state(self):
        running = self.hw.session is not None and self.session_kind == 'protocol'
        recording = self.hw.session is not None and self.session_kind == 'manual'
        rig = self.hw.rig is not None
        manual_ok = rig and not running
        for b in self.valve_btns:
            b.configure(state='normal' if manual_ok else 'disabled')
        self.scale.configure(state='normal' if manual_ok else 'disabled')
        self.btn_run.configure(state='normal' if rig and not self.hw.session else 'disabled')
        self.btn_pstop.configure(state='normal' if running else 'disabled')
        self.btn_brun.configure(state='normal' if rig and not self.hw.session else 'disabled')
        self.btn_bstop.configure(state='normal' if running else 'disabled')
        self.btn_rec.configure(state='normal' if rig and not running else 'disabled',
                               text='■  Stop recording' if recording else '●  Start recording')
        self.btn_fgt.configure(state='disabled' if rig else 'normal')
        self.btn_lcr_apply.configure(state='normal' if self.hw.lcr and not self.hw.session else 'disabled')
        self.btn_folder.configure(state='normal' if self.last_out_dir else 'disabled')

    # ── actions: connections ────────────────────────────────────────────
    def connect_fluigent(self):
        self._apply_max_p()
        sim = self.v['simulate'].get()
        self.lab_fgt.configure(text='connecting…')

        def done(info):
            rng = info.get('pressure_range_mbar', ['?', '?'])
            return ('fgt', f"connected{' (SIMULATED)' if sim else ''} — "
                           f"Flow EZ {rng[0]:g}–{rng[1]:g} mbar, "
                           f"{info.get('valve_channels', '?')} valve outputs")
        self._busy(self.hw.connect_rig, sim, done=done)

    def _lcr_cfg(self):
        func = lcr6100.normalize_function(self.v['lcr_func'].get())
        avg = self.v['lcr_avg'].get().strip()
        return {'function': func,
                'freq_hz': self._num('lcr_freq', lo=10, hi=300000),
                'volt_v': self._num('lcr_volt', lo=0.01, hi=2.0),
                'speed': self.v['lcr_speed'].get(),
                'avg': int(avg) if avg else None,
                'auto_range': True}

    def connect_lcr(self):
        try:
            cfg = self._lcr_cfg()
        except ValueError as e:
            return messagebox.showerror('LCR settings', str(e))
        sim = self.v['simulate'].get()
        port = self.v['lcr_port'].get().strip()
        if not sim and not port:
            return messagebox.showerror('LCR', 'Choose the LCR serial port first.')
        self.lab_lcr.configure(text='connecting + configuring…')
        self._busy(self.hw.connect_lcr, sim, port, cfg, self.v['lcr_eol'].get(),
                   done=lambda d: ('lcr', f'connected — {d}  (verified by read-back)'))

    def apply_lcr(self):
        try:
            cfg = self._lcr_cfg()
        except ValueError as e:
            return messagebox.showerror('LCR settings', str(e))
        self.lab_lcr.configure(text='applying…')
        self._busy(self.hw.configure_lcr, cfg,
                   done=lambda d: ('lcr', f'connected — {d}  (verified by read-back)'))

    def disconnect_lcr(self):
        if self.hw.session:
            return messagebox.showwarning('LCR', 'Stop the running session first.')
        self.hw.disconnect_lcr()
        self.q.put(('lcr', 'not connected'))

    # ── actions: manual ─────────────────────────────────────────────────
    def toggle_valve(self, i):
        open_set = {k for k, v in enumerate(self.valve_state) if v.get()}
        try:
            self.hw.set_valves(open_set)
        except Exception as e:
            self.valve_state[i].set(not self.valve_state[i].get())
            messagebox.showerror('Valve', str(e))

    def set_all_valves(self, on):
        for v in self.valve_state:
            v.set(on)
        try:
            self.hw.set_valves(set(range(core.N_OUTPUTS)) if on else set())
        except Exception as e:
            messagebox.showerror('Valve', str(e))

    def set_pressure(self, value):
        try:
            p = self._num('sp_entry', value=value, lo=0)
            mp = self._max_p()
            if p > mp:
                messagebox.showwarning('Limit', f'{p:g} mbar is above the {mp:g} mbar ceiling — '
                                                f'using {mp:g}.')
            self.hw.max_p = mp
            self.hw.cancel_ramp()
            p = self.hw.set_pressure(p)
            self.v['sp_slider'].set(p)
            self.v['sp_entry'].set(f'{p:g}')
        except Exception as e:
            messagebox.showerror('Pressure', str(e))

    def manual_ramp(self, target=None):
        try:
            tgt = self._num('ramp_target', value=target, lo=0)
            dur = self._num('ramp_time', lo=0)
            mp = self._max_p()
            if tgt > mp:
                return messagebox.showwarning('Limit', f'{tgt:g} mbar is above the {mp:g} mbar ceiling.')
            if not self.hw.rig:
                raise RuntimeError('Fluigent not connected')
            self.hw.max_p = mp
            self.hw.ramp(tgt, dur)
            self.v['sp_slider'].set(tgt)
            self.v['sp_entry'].set(f'{tgt:g}')
        except Exception as e:
            messagebox.showerror('Ramp', str(e))

    def toggle_recording(self):
        if self.hw.session and self.session_kind == 'manual':
            return self.stop_session()
        if self.hw.session:
            return
        name = self.v['manual_name'].get().strip() or 'manual'
        protocol = {'name': name, 'manual': True, 'steps': [], 'actuators': self.aliases()}
        self._start_session(protocol, 'manual', total_s=0.0)

    # ── actions: protocol ───────────────────────────────────────────────
    def pick_protocol(self):
        p = filedialog.askopenfilename(initialdir=os.path.join(HERE, 'protocols'),
                                       filetypes=[('JSON protocol', '*.json')])
        if p:
            self.v['protocol_file'].set(p)
            self.v['source'].set('file')

    def build_protocol(self):
        if self.v['source'].get() == 'builder':
            protocol = self.builder_protocol()
        elif self.v['source'].get() == 'file':
            path = self.v['protocol_file'].get()
            if not path:
                raise ValueError('Load a JSON protocol file first.')
            with open(path) as fh:
                protocol = json.load(fh)
            protocol.setdefault('actuators', {}).update(
                {k: v for k, v in self.aliases().items() if k not in protocol['actuators']})
        else:
            ns = argparse.Namespace(
                valves=self.v['sweep_valves'].get(),
                setpoints=self.v['sweep_setpoints'].get(),
                ramp=self._num('sweep_ramp', lo=0), hold=self._num('sweep_hold', lo=0),
                rest=self._num('sweep_rest', lo=0),
                cycles=self._num('sweep_cycles', lo=1, integer=True),
                baseline=self._num('sweep_baseline', lo=0),
                name=None, actuators='')
            protocol = core.sweep_protocol(ns)
            protocol['actuators'] = self.aliases()
        protocol['name'] = self.v['name'].get().strip() or protocol.get('name', 'gui')
        protocol['repeat'] = self._num('repeat', lo=1, integer=True)
        protocol['max_pressure_mbar'] = self._max_p()
        segs = core.expand_protocol(protocol)
        core.validate(segs, self._max_p())
        core.expand_valve_timeline(protocol, sum(s['duration'] for s in segs))  # check it
        return protocol, segs

    def preview(self):
        try:
            protocol, segs = self.build_protocol()
        except Exception as e:
            return messagebox.showerror('Protocol', str(e))
        buf = io.StringIO()
        with redirect_stdout(buf):
            total = core.print_schedule(segs, protocol.get('actuators'), protocol['repeat'])
            core.print_valve_events(core.expand_valve_timeline(protocol, total),
                                    protocol.get('actuators'))
        self.txt.delete('1.0', 'end')
        self.txt.insert('1.0', buf.getvalue())
        return protocol, total * protocol['repeat']

    def run_protocol(self):
        if self.hw.session:
            return
        res = self.preview()
        if not res:
            return
        protocol, total = res
        if not self.hw.lcr and not messagebox.askyesno(
                'No LCR', 'The LCR is not connected — run with pressure only?'):
            return
        if not messagebox.askokcancel(
                'Start protocol',
                f"Run '{protocol['name']}' ({total:.0f} s, max "
                f"{protocol['max_pressure_mbar']:g} mbar)?\nPressure will be applied."):
            return
        self.hw.cancel_ramp()
        self._start_session(protocol, 'protocol', total_s=total)

    def _start_session(self, protocol, kind, total_s):
        try:
            log_rate = self._num('log_rate', lo=1, hi=200)
            vent_s = self._num('vent_s', lo=0, hi=60)
        except ValueError as e:
            return messagebox.showerror('Settings', str(e))
        self._apply_max_p()
        name = protocol.get('name', kind)
        out_dir = os.path.join(core.LOG_DIR, f"{name}_{datetime.now():%Y%m%d_%H%M%S}")
        try:
            sess = core.ActuatorSession(
                self.hw.rig, self.hw.lcr, None if kind == 'manual' else protocol, out_dir,
                self._max_p(), log_rate_hz=log_rate, vent_s=vent_s,
                repeat=protocol.get('repeat', 1), status=False, manage_rig=False,
                on_row=self._on_row)
        except Exception as e:
            return messagebox.showerror('Protocol', str(e))
        if kind == 'manual':
            sess.protocol = protocol
            sess.actuators = protocol.get('actuators', {})
        self.hw.session = sess
        self.session_kind = kind
        self.session_total_s = total_s
        self.prog['value'] = 0
        analyze = self.v['analyze'].get()
        open_plots = self.v['open_plots'].get()
        self._set_controls_state()

        def work():
            status = 'error'
            try:
                status = sess.run_manual() if kind == 'manual' else sess.run()
            except Exception as e:
                status = f'error: {e}'
                self.q.put(('error', f'Session failed: {e}'))
            finally:
                self.hw.session = None
            self.q.put(('session_done', (status, out_dir, kind)))
            if analyze and sess.rows:
                try:
                    files = ana.analyze(out_dir, show=open_plots, verbose=False)
                    self.q.put(('analysis', (out_dir, files)))
                except Exception as e:
                    self.q.put(('error', f'Analysis failed: {e}'))
        self.session_thread = threading.Thread(target=work, daemon=True, name='session')
        self.session_thread.start()

    def stop_session(self):
        s = self.hw.session
        if s:
            s.stop()

    def _session_finished(self, status, out_dir, kind):
        self.last_out_dir = out_dir
        self.session_kind = None
        self.prog['value'] = 100 if status == 'completed' else self.prog['value']
        self.lab_step.configure(text=f'{status} → {out_dir}')
        self.lab_bstep.configure(text=f'{status} → {out_dir}')
        if status == 'completed':
            self.prog_b['value'] = 100
        if kind == 'manual':
            self.lab_rec.configure(text=f'Saved: {out_dir}')
        else:
            self.q.put(('valves', set()))
        self._set_controls_state()

    def _show_analysis(self, val):
        out_dir, files = val
        try:
            d, _ = ana.load_session(out_dir)
            rows = ana.summarize(d)
        except Exception:
            rows = []
        lines = [f'Analysis: {out_dir}', '']
        if rows:
            pc = d['lcr_cols'][0] if d['lcr_cols'] else None
            lines.append(f"{'step':>4} {'phase':<9} {'valves':<10} {'sp':>6} {'P mean':>8}"
                         + (f"  {ana.quantity(pc)[0] + ' mean':>12} {'Δ%':>8} {'ok':>5}" if pc else ''))
            for r in rows:
                line = (f"{r['step']:>4} {r['phase']:<9} {r['valves']:<10} "
                        f"{r['setpoint_mbar']:6.0f} {r['pressure_mean_mbar']:8.2f}")
                if pc:
                    line += (f"  {r[pc + '_mean']:12.5g} {r['primary_change_pct']:8.3f} "
                             f"{r['lcr_ok_frac']:5.0%}")
                lines.append(line)
        lines += [''] + files
        self.txt.delete('1.0', 'end')
        self.txt.insert('1.0', '\n'.join(lines))

    def open_results(self):
        if self.last_out_dir:
            ana.open_files([self.last_out_dir])

    # ── emergency / close ───────────────────────────────────────────────
    def emergency_stop(self):
        s = self.hw.session
        if s:
            s.stop()
        try:
            vent_s = float(self.v['vent_s'].get())
        except ValueError:
            vent_s = 3.0
        self.hw.vent(close_after_s=vent_s)
        self.v['sp_slider'].set(0)
        self.v['sp_entry'].set('0')
        self.lab_state.configure(text='VENTING', fg='#FF8A7A')

    def on_close(self):
        if self.hw.session and not messagebox.askyesno(
                'Quit', 'A session is running. Stop it, vent and quit?'):
            return
        try:
            self._save_settings()
        except Exception as e:                    # never block closing / venting
            print('[gui] could not save settings:', repr(e))
        self._monitor_stop.set()
        try:
            vent_s = float(self.v['vent_s'].get())
        except ValueError:
            vent_s = 3.0
        if self.session_thread and self.session_thread.is_alive():
            self.hw.session and self.hw.session.stop()
            self.session_thread.join(timeout=vent_s + 5)
        self.hw.shutdown(vent_s if self.hw.rig else 0)
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description='Actuator + LCR control GUI')
    ap.add_argument('--simulate', action='store_true', help='start with Simulate ticked')
    a = ap.parse_args()
    root = tk.Tk()
    apply_theme(root)
    App(root, simulate=a.simulate)
    root.mainloop()
    # on_close() has vented, closed the valves and saved everything.  Skip
    # interpreter tear-down: the x86 Fluigent dylib crashes there (SIGILL).
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
