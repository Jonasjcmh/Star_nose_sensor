#!/usr/bin/env python3
"""
Timed pressure + valve sequencer for Flow EZ (pressure) and P-Switch (valve)
modules.

Define named pressure channels (Flow EZ outputs) and named valve channels
(P-Switch outputs) below, then describe a SEQUENCE of timed steps. Each step
sets whichever channels you list to a target pressure / on-off state and
holds that state for `duration` seconds before moving to the next step.
Channels you don't mention in a step keep their previous value.

A single P-Switch exposes 8 independently addressable outputs, so any subset
of them can be open at once. Use only(*names) to express "activate exactly
these outputs, close all the rest" for a given step — see PSWITCH_OUTPUTS,
COMBINATIONS and the generated SEQUENCE below for examples.

Run with --dry-run to print the resolved schedule without touching hardware.

Usage:
    python sequence_control.py                # run once on real hardware
    python sequence_control.py --dry-run       # print schedule, no hardware
    python sequence_control.py --loop 3        # run the sequence 3 times
    python sequence_control.py --loop 0        # loop forever (Ctrl+C to stop)
"""
import argparse
import sys
import time

fgt = None  # bound to the Fluigent.SDK module by run(); stays None in --dry-run

# ---------------------------------------------------------------------------
# Channel map — edit to match your setup.
# Index is the Fluigent channel index as reported by fgt_get_pressureChannelsInfo()
# / fgt_get_valveChannelsInfo() (0-based, in detection order).
# ---------------------------------------------------------------------------
PRESSURE_CHANNELS = {
    "flow_ez": 0,   # single Flow EZ, 2000 mbar full-scale
}

# A P-Switch exposes 8 independently addressable outputs (each a 2-way
# valve: closed/open). With only one P-Switch connected they occupy valve
# indices 0-7 in detection order.
PSWITCH_OUTPUTS = [f"pswitch_out{i + 1}" for i in range(8)]
VALVE_CHANNELS = {name: i for i, name in enumerate(PSWITCH_OUTPUTS)}

ON_POSITION = 1    # P-Switch output position considered "activated" / open
OFF_POSITION = 0   # P-Switch output position considered "deactivated" / closed

# Hard safety ceiling enforced below, independent of device full-scale (2000
# mbar). Keeps this test sequence — and any edits to it — from accidentally
# commanding more than intended.
SAFETY_MAX_PRESSURE_MBAR = 240

TEST_PRESSURE_MBAR = 240  # common pressure applied while exercising the valves


def only(*active_outputs):
    """Build a `valves` dict that turns on exactly the given P-Switch
    outputs and turns off every other one, so each step is a fully
    deterministic state (no output left open from a previous step).

        only()                              -> all 8 outputs closed
        only("pswitch_out1")                -> output 1 alone
        only("pswitch_out1", "pswitch_out3") -> outputs 1 and 3 together
    """
    unknown = set(active_outputs) - set(VALVE_CHANNELS)
    if unknown:
        raise ValueError(f"unknown P-Switch output(s): {sorted(unknown)}")
    return {name: (name in active_outputs) for name in VALVE_CHANNELS}


# ---------------------------------------------------------------------------
# Sequence — ordered list of timed steps.
#   duration : seconds to hold this state before advancing
#   pressures: {channel_name: mbar}  (only listed channels change)
#   valves   : {channel_name: True/False}  (only listed channels change)
# ---------------------------------------------------------------------------
# Exercises every P-Switch output individually, then a few combinations,
# all at a constant, safety-capped test pressure.
SEQUENCE = [
    {
        "label": "baseline: 0 mbar, all outputs closed",
        "duration": 2.0,
        "pressures": {"flow_ez": 0},
        "valves": only(),
    },
    {
        "label": f"pressurize to {TEST_PRESSURE_MBAR} mbar, outputs still closed",
        "duration": 1.0,
        "pressures": {"flow_ez": TEST_PRESSURE_MBAR},
    },
]

# One output at a time.
for _name in PSWITCH_OUTPUTS:
    SEQUENCE.append({
        "label": f"{_name} only",
        "duration": 1.5,
        "valves": only(_name),
    })

SEQUENCE.append({"label": "all outputs closed", "duration": 1.0, "valves": only()})

# A few example combinations — edit/extend this list for whatever
# combos you actually want to test (any subset of PSWITCH_OUTPUTS works).
COMBINATIONS = [
    ("pswitch_out1", "pswitch_out2"),
    ("pswitch_out3", "pswitch_out4"),
    ("pswitch_out1", "pswitch_out3", "pswitch_out5", "pswitch_out7"),
    tuple(PSWITCH_OUTPUTS),  # all 8 at once
]
for _combo in COMBINATIONS:
    SEQUENCE.append({
        "label": " + ".join(_combo),
        "duration": 1.5,
        "valves": only(*_combo),
    })

SEQUENCE.append({
    "label": "vent to 0 mbar, all outputs closed",
    "duration": 2.0,
    "pressures": {"flow_ez": 0},
    "valves": only(),
})


def validate_sequence():
    for step in SEQUENCE:
        for name, mbar in step.get("pressures", {}).items():
            if mbar > SAFETY_MAX_PRESSURE_MBAR:
                raise SystemExit(
                    f"step '{step.get('label', '')}' requests {mbar} mbar on "
                    f"'{name}', exceeding SAFETY_MAX_PRESSURE_MBAR="
                    f"{SAFETY_MAX_PRESSURE_MBAR}"
                )
            if name not in PRESSURE_CHANNELS:
                raise SystemExit(f"unknown pressure channel '{name}' in sequence")
        for name in step.get("valves", {}):
            if name not in VALVE_CHANNELS:
                raise SystemExit(f"unknown valve channel '{name}' in sequence")


def resolve_step(step, pressure_state, valve_state):
    """Apply a step's requested changes onto the running state dicts."""
    pressure_state.update(step.get("pressures", {}))
    valve_state.update(step.get("valves", {}))
    return pressure_state, valve_state


def print_schedule():
    validate_sequence()
    pressure_state = {name: None for name in PRESSURE_CHANNELS}
    valve_state = {name: None for name in VALVE_CHANNELS}
    t = 0.0
    print(f"{'t (s)':>8}  {'dur':>6}  label")
    for step in SEQUENCE:
        resolve_step(step, pressure_state, valve_state)
        print(f"{t:8.2f}  {step['duration']:6.2f}  {step.get('label', '')}")
        for name, val in step.get("pressures", {}).items():
            print(f"           pressure  {name:12s} -> {val} mbar")
        for name, val in step.get("valves", {}).items():
            print(f"           valve     {name:12s} -> {'ON' if val else 'OFF'}")
        t += step["duration"]
    print(f"{'total':>8}  {t:6.2f}s")


def apply_step(step):
    for name, mbar in step.get("pressures", {}).items():
        idx = PRESSURE_CHANNELS[name]
        fgt.fgt_set_pressure(idx, mbar)
    for name, on in step.get("valves", {}).items():
        idx = VALVE_CHANNELS[name]
        fgt.fgt_set_valvePosition(idx, ON_POSITION if on else OFF_POSITION)


def all_off():
    for idx in PRESSURE_CHANNELS.values():
        try:
            fgt.fgt_set_pressure(idx, 0)
        except Exception as e:
            print(f"warning: could not vent pressure channel {idx}: {e}")
    for idx in VALVE_CHANNELS.values():
        try:
            fgt.fgt_set_valvePosition(idx, OFF_POSITION)
        except Exception as e:
            print(f"warning: could not close valve channel {idx}: {e}")


def run(loop_count):
    global fgt
    validate_sequence()

    sys.path.insert(0, "/Users/jonathantirado/Documents/GitHub/fgt-SDK/Python")
    import Fluigent.SDK as fgt

    fgt.fgt_init()
    try:
        n_pressure = fgt.fgt_get_pressureChannelCount()
        n_valve = fgt.fgt_get_valveChannelCount()
        for name, idx in PRESSURE_CHANNELS.items():
            if idx >= n_pressure:
                raise SystemExit(
                    f"pressure channel '{name}' index {idx} out of range "
                    f"(only {n_pressure} pressure channel(s) detected)"
                )
        for name, idx in VALVE_CHANNELS.items():
            if idx >= n_valve:
                raise SystemExit(
                    f"valve channel '{name}' index {idx} out of range "
                    f"(only {n_valve} valve channel(s) detected)"
                )

        cycle = 0
        while loop_count == 0 or cycle < loop_count:
            for step in SEQUENCE:
                print(f"[cycle {cycle}] {step.get('label', '')}")
                apply_step(step)
                time.sleep(step["duration"])
            cycle += 1

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        all_off()
        fgt.fgt_close()
        print("All outputs vented/closed and session closed.")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dry-run", action="store_true",
                    help="print the resolved schedule and exit, no hardware access")
    p.add_argument("--loop", type=int, default=1,
                    help="number of times to run the sequence (0 = loop forever, default 1)")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.dry_run:
        print_schedule()
    else:
        run(args.loop)
