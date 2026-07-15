#!/usr/bin/env python3
"""
visualize_tcp_force.py — live view of the UR5's TCP force/torque wrench,
with an on-demand F/T zero reset and a live weight estimate from Fz.

Connects to the real robot (ur_rtde), same connection pattern as
collect_single.py: rtde_control.RTDEControlInterface for zeroFtSensor(),
rtde_receive.RTDEReceiveInterface for getActualTCPForce()/getActualTCPPose().

What it shows
-------------
Top panel:    Fx, Fy, Fz (N) over a rolling time window.
Bottom panel: estimated weight (g) over the same window, computed from Fz
              alone (weight_g = |Fz| / G * 1000) -- this assumes the load
              is applied along the tool Z axis, same convention as the
              rest of this project's fzcal_* calibration logs. Fx/Fy are
              still plotted above so you can see if that assumption is
              actually holding (large Fx/Fy means the load isn't purely
              axial and the weight estimate will be off).
Live text:    current Fx/Fy/Fz and the current weight estimate, updated
              every frame.

Zeroing the F/T sensor
-----------------------
The whole point of this script is to make zeroing easy to get right: with
NOTHING (or only the hardware you want zeroed out) resting on the sensor,
press 'z' in the plot window at any time to call rtde_c.zeroFtSensor().
This is a live re-zero, not a one-shot prompt -- so you can watch the
trace settle, zero it, watch it settle to zero, and keep watching before
deciding your baseline is good, without restarting the script.

Usage
-----
    python visualize_tcp_force.py
    python visualize_tcp_force.py --robot-ip 127.0.0.1     # URSim
    python visualize_tcp_force.py --window-s 15 --rate 20

Press 'z' in the plot window to zero the F/T sensor. Close the window
(or Ctrl+C in the terminal) to disconnect and exit.
"""

import argparse
import os
import sys
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ur5_control lives in Integration_2 (ROBOT_IP), same as collect_single.py.
INTEGRATION_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "Integration_2")
sys.path.insert(0, os.path.abspath(INTEGRATION_DIR))

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None

G = 9.80665  # m/s^2


def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default=None,
                     help="override UR5 IP (default: ur5_control.ROBOT_IP)")
    ap.add_argument("--rate", type=float, default=20.0,
                     help="sample rate (Hz)")
    ap.add_argument("--window-s", type=float, default=10.0,
                     help="rolling display window (s)")
    ap.add_argument("--no-zero-on-start", action="store_true",
                     help="skip the zeroFtSensor() call at startup "
                          "(you can still press 'z' any time)")
    return ap


def main():
    args = build_parser().parse_args()

    if rtde_control is None or rtde_receive is None:
        sys.exit("ur_rtde is not installed — "
                  "pip install ur-rtde --break-system-packages")

    import ur5_control
    robot_ip = args.robot_ip or ur5_control.ROBOT_IP

    print(f"[viz] Connecting to {robot_ip} ...")
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    print("[viz] Connected.")

    if not args.no_zero_on_start:
        input("[viz] Press Enter to zero the F/T sensor before starting "
              "(make sure only the hardware you want zeroed is on it)...")
        rtde_c.zeroFtSensor()
        print("[viz] Zeroed.")
        time.sleep(0.5)

    n = max(2, int(args.window_s * args.rate))
    t_buf = np.linspace(-args.window_s, 0, n)
    fx_buf = np.zeros(n)
    fy_buf = np.zeros(n)
    fz_buf = np.zeros(n)
    w_buf = np.zeros(n)

    state = {"last_zero_ago": None}

    fig, (ax_f, ax_w) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    fx_line, = ax_f.plot(t_buf, fx_buf, color="#1f77b4", linewidth=1.2, label="Fx")
    fy_line, = ax_f.plot(t_buf, fy_buf, color="#2ca02c", linewidth=1.2, label="Fy")
    fz_line, = ax_f.plot(t_buf, fz_buf, color="#d62728", linewidth=1.4, label="Fz")
    ax_f.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax_f.set_ylabel("Force (N)")
    ax_f.set_title("TCP force/torque — press 'z' to zero the F/T sensor")
    ax_f.legend(fontsize=9, loc="upper left")
    ax_f.grid(alpha=0.25)
    live_text = ax_f.text(0.99, 0.02, "", transform=ax_f.transAxes,
                           fontsize=10, ha="right", va="bottom", family="monospace",
                           bbox=dict(facecolor="white", alpha=0.8, edgecolor="#888"))

    w_line, = ax_w.plot(t_buf, w_buf, color="#333333", linewidth=1.4)
    ax_w.axhline(0, color="gray", lw=0.8, alpha=0.5)
    ax_w.set_ylabel("estimated weight (g)")
    ax_w.set_xlabel("time (s)")
    ax_w.set_title("weight_g = |Fz| / g * 1000  (assumes axial load along tool Z)")
    ax_w.grid(alpha=0.25)
    weight_text = ax_w.text(0.99, 0.90, "", transform=ax_w.transAxes,
                             fontsize=12, ha="right", va="top", fontweight="bold",
                             bbox=dict(facecolor="white", alpha=0.8, edgecolor="#888"))

    def on_key(event):
        if event.key == "z":
            print("[viz] Zeroing F/T sensor...")
            rtde_c.zeroFtSensor()
            state["last_zero_ago"] = 0.0
            print("[viz] Zeroed.")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        ft = rtde_r.getActualTCPForce()
        fx, fy, fz = ft[0], ft[1], ft[2]
        weight_g = abs(fz) / G * 1000.0

        for buf, val in ((fx_buf, fx), (fy_buf, fy), (fz_buf, fz), (w_buf, weight_g)):
            buf[:-1] = buf[1:]
            buf[-1] = val

        fx_line.set_ydata(fx_buf)
        fy_line.set_ydata(fy_buf)
        fz_line.set_ydata(fz_buf)
        w_line.set_ydata(w_buf)

        ax_f.set_ylim(min(fx_buf.min(), fy_buf.min(), fz_buf.min()) - 0.1,
                      max(fx_buf.max(), fy_buf.max(), fz_buf.max()) + 0.1)
        ax_w.set_ylim(0, max(w_buf.max(), 10.0) * 1.1)

        live_text.set_text(f"Fx={fx:+7.3f} N\nFy={fy:+7.3f} N\nFz={fz:+7.3f} N")
        weight_text.set_text(f"{weight_g:6.1f} g")

        if state["last_zero_ago"] is not None:
            state["last_zero_ago"] += 1.0 / args.rate

        return fx_line, fy_line, fz_line, w_line, live_text, weight_text

    anim = FuncAnimation(fig, update, interval=1000.0 / args.rate, blit=False)
    fig.tight_layout()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        print("[viz] Disconnecting...")
        try:
            rtde_c.stopScript()
        except Exception:
            pass


if __name__ == "__main__":
    main()
