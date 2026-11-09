"""
theme.py — a flat, dark "control console" theme for the control_robot GUI
========================================================================
A single :func:`apply_theme` sets up a cohesive dark ttk style (built on the
``clam`` theme, the only stock one that lets you fully recolour widgets) and
returns the palette + a handful of named fonts. The look is deliberately flat
and product-like: no beveled/grooved borders, flat accent/danger buttons, card
surfaces, subtle 1-px borders, and a monospace family for live numbers.

Usage:
    from theme import apply_theme
    COLORS, FONTS = apply_theme(root)
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont


# Two palettes, same flat "instrument console" style. 'light' is a clean white
# product look; 'dark' is the GitHub-dark variant.
PALETTES = {
    'dark': {
        'bg':        '#0d1117',   # window background
        'surface':   '#161b22',   # card background
        'surface2':  '#1c2430',   # inset (spinbox / value chips)
        'border':    '#30363d',   # hairline borders
        'text':      '#e6edf3',   # primary text
        'muted':     '#8b949e',   # secondary text / card titles
        'accent':    '#2f81f7',   # primary action (blue)
        'accent_hi': '#4c93ff',
        'danger':    '#f85149',   # STOP / destructive
        'danger_hi': '#ff6a60',
        'success':   '#3fb950',   # OK badges
        'warn':      '#d29922',   # warnings / contact
        'track':     '#283039',   # slider troughs / disabled
        'on_accent': '#ffffff',
    },
    'light': {
        'bg':        '#eceff3',   # window background (soft grey-white)
        'surface':   '#ffffff',   # card background (white)
        'surface2':  '#f1f4f8',   # inset (spinbox / value chips)
        'border':    '#d0d7de',   # hairline borders
        'text':      '#1f2328',   # primary text (near-black)
        'muted':     '#656d76',   # secondary text / card titles
        'accent':    '#0969da',   # primary action (blue)
        'accent_hi': '#1f6feb',
        'danger':    '#cf222e',   # STOP / destructive
        'danger_hi': '#e5484d',
        'success':   '#1a7f37',   # OK badges
        'warn':      '#9a6700',   # warnings / contact
        'track':     '#e4e8ec',   # slider troughs / disabled
        'on_accent': '#ffffff',
    },
}

# Active palette (filled in by apply_theme; defaults to light).
COLORS = dict(PALETTES['light'])


def _pick(root, candidates, size, weight='normal'):
    fams = set(tkfont.families(root))
    for c in candidates:
        if c in fams:
            return tkfont.Font(root=root, family=c, size=size, weight=weight)
    return tkfont.Font(root=root, family=candidates[-1], size=size, weight=weight)


def apply_theme(root, variant='light'):
    """Apply the theme (`'light'` or `'dark'`) to `root`; return (COLORS, FONTS)."""
    COLORS.clear()
    COLORS.update(PALETTES.get(variant, PALETTES['light']))
    ui_fam = ['Inter', 'Segoe UI', 'Roboto', 'Helvetica Neue', 'DejaVu Sans']
    mono_fam = ['JetBrains Mono', 'Roboto Mono', 'Consolas', 'DejaVu Sans Mono']
    FONTS = {
        'title':  _pick(root, ui_fam, 15, 'bold'),
        'h':      _pick(root, ui_fam, 10, 'bold'),
        'ui':     _pick(root, ui_fam, 10),
        'small':  _pick(root, ui_fam, 9),
        'btn':    _pick(root, ui_fam, 10, 'bold'),
        'mono':   _pick(root, mono_fam, 11),
        'mono_sm': _pick(root, mono_fam, 9),
        'mono_big': _pick(root, mono_fam, 13, 'bold'),
    }

    C = COLORS
    root.configure(bg=C['bg'])
    st = ttk.Style(root)
    st.theme_use('clam')

    # base
    st.configure('.', background=C['bg'], foreground=C['text'], font=FONTS['ui'],
                 borderwidth=0, focuscolor=C['bg'], troughcolor=C['track'])
    st.configure('TFrame', background=C['bg'])
    st.configure('Card.TFrame', background=C['surface'])
    st.configure('Border.TFrame', background=C['border'])
    st.configure('Bar.TFrame', background=C['surface'])

    # labels
    st.configure('TLabel', background=C['bg'], foreground=C['text'], font=FONTS['ui'])
    st.configure('Card.TLabel', background=C['surface'], foreground=C['text'], font=FONTS['ui'])
    st.configure('CardTitle.TLabel', background=C['surface'], foreground=C['muted'], font=FONTS['h'])
    st.configure('Muted.TLabel', background=C['surface'], foreground=C['muted'], font=FONTS['small'])
    st.configure('Value.TLabel', background=C['surface'], foreground=C['text'], font=FONTS['mono'])
    st.configure('ValueBig.TLabel', background=C['surface'], foreground=C['text'], font=FONTS['mono_big'])
    st.configure('AppTitle.TLabel', background=C['surface'], foreground=C['text'], font=FONTS['title'])
    st.configure('Bar.TLabel', background=C['surface'], foreground=C['muted'], font=FONTS['small'])

    # buttons — flat, padded, hover via state map
    def _button(name, bg, fg, hover):
        st.configure(name, background=bg, foreground=fg, font=FONTS['btn'],
                     padding=(14, 9), relief='flat', borderwidth=0, anchor='center')
        st.map(name,
               background=[('disabled', C['track']), ('pressed', hover), ('active', hover)],
               foreground=[('disabled', C['muted'])])

    _button('Primary.TButton', C['accent'], C['on_accent'], C['accent_hi'])
    _button('Danger.TButton', C['danger'], C['on_accent'], C['danger_hi'])
    _button('Ghost.TButton', C['surface2'], C['text'], C['border'])
    # jog pad keys: inset chip that lights up accent while held/hover
    st.configure('Jog.TButton', background=C['surface2'], foreground=C['text'],
                 font=FONTS['btn'], padding=(10, 12), relief='flat', borderwidth=0)
    st.map('Jog.TButton',
           background=[('pressed', C['accent']), ('active', C['border'])],
           foreground=[('pressed', C['on_accent'])])

    # scales
    st.configure('Horizontal.TScale', background=C['surface'],
                 troughcolor=C['track'], borderwidth=0)

    # spinboxes
    st.configure('TSpinbox', fieldbackground=C['surface2'], background=C['surface2'],
                 foreground=C['text'], bordercolor=C['border'], arrowcolor=C['muted'],
                 insertcolor=C['text'], padding=4, relief='flat')
    st.map('TSpinbox', fieldbackground=[('readonly', C['surface2'])])

    # separators
    st.configure('TSeparator', background=C['border'])

    # checkbutton
    st.configure('TCheckbutton', background=C['surface'], foreground=C['text'],
                 font=FONTS['ui'], focuscolor=C['surface'])
    st.map('TCheckbutton', background=[('active', C['surface'])])

    # notebook (tabs) — flat, underline-accent on the selected tab
    st.configure('TNotebook', background=C['bg'], borderwidth=0, tabmargins=(8, 6, 8, 0))
    st.configure('TNotebook.Tab', background=C['bg'], foreground=C['muted'],
                 font=FONTS['btn'], padding=(18, 9), borderwidth=0)
    st.map('TNotebook.Tab',
           background=[('selected', C['surface']), ('active', C['surface2'])],
           foreground=[('selected', C['accent']), ('active', C['text'])])

    # treeview (routine table)
    st.configure('Treeview', background=C['surface'], fieldbackground=C['surface'],
                 foreground=C['text'], borderwidth=0, rowheight=26, font=FONTS['ui'])
    st.configure('Treeview.Heading', background=C['surface2'], foreground=C['muted'],
                 font=FONTS['h'], borderwidth=0, relief='flat')
    st.map('Treeview', background=[('selected', C['accent'])],
           foreground=[('selected', C['on_accent'])])

    return C, FONTS


def badge(parent, text, color, bg):
    """A small pill-style status chip (a tk.Label so we can round-pad it)."""
    lbl = tk.Label(parent, text=text, fg=color, bg=bg, font=('TkDefaultFont', 9, 'bold'),
                   padx=10, pady=3)
    return lbl
