#!/usr/bin/env python3
"""
Live serial monitor for the ESP32-C6 pressure sensor project.

Reads CSV lines "millis,voltage_V,pressure_mbar" from the board and
plots them live. Click the "Toggle V/P" button, or press 'v' / 'p',
to switch between voltage and pressure views.

Install deps:
    pip install pyserial matplotlib

Usage:
    python pressure_monitor.py --port /dev/ttyACM0
    python pressure_monitor.py --port COM5 --baud 115200
"""
import argparse
import collections
import threading

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0 or COM5")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--window", type=int, default=300, help="Number of samples kept on screen")
    return p.parse_args()


class SerialReader(threading.Thread):
    def __init__(self, port, baud, maxlen):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=1)
        self.t = collections.deque(maxlen=maxlen)
        self.voltage = collections.deque(maxlen=maxlen)
        self.pressure = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.running = True
        self.t0 = None

    def run(self):
        self.ser.reset_input_buffer()
        while self.running:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException:
                break
            if not line or line.startswith("millis"):
                continue
            parts = line.split(",")
            if len(parts) != 3:
                continue
            try:
                ms, v, p = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                continue
            if self.t0 is None:
                self.t0 = ms
            with self.lock:
                self.t.append((ms - self.t0) / 1000.0)
                self.voltage.append(v)
                self.pressure.append(p)

    def snapshot(self):
        with self.lock:
            return list(self.t), list(self.voltage), list(self.pressure)

    def stop(self):
        self.running = False
        self.ser.close()


def main():
    args = parse_args()
    reader = SerialReader(args.port, args.baud, args.window)
    reader.start()

    mode = {"value": "pressure"}  # or "voltage"

    fig, ax = plt.subplots(figsize=(9, 5))
    plt.subplots_adjust(bottom=0.2)
    line, = ax.plot([], [], lw=1.5)
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)

    def set_labels():
        if mode["value"] == "pressure":
            ax.set_ylabel("Pressure (mbar)")
            ax.set_title("Live Pressure Reading (100MDAA5)")
        else:
            ax.set_ylabel("Voltage (V)")
            ax.set_title("Live Sensor Output Voltage")
    set_labels()

    def toggle(event=None):
        mode["value"] = "voltage" if mode["value"] == "pressure" else "pressure"
        set_labels()

    ax_button = plt.axes([0.42, 0.05, 0.18, 0.075])
    button = Button(ax_button, "Toggle V/P")
    button.on_clicked(toggle)

    def on_key(event):
        if event.key == "v":
            mode["value"] = "voltage"
            set_labels()
        elif event.key == "p":
            mode["value"] = "pressure"
            set_labels()
    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(frame):
        t, v, p = reader.snapshot()
        if not t:
            return line,
        y = p if mode["value"] == "pressure" else v
        line.set_data(t, y)
        ax.set_xlim(max(0, t[-1] - 30), t[-1] + 0.5 if t[-1] > 0 else 1)
        ymin, ymax = min(y), max(y)
        pad = (ymax - ymin) * 0.1 or 0.1
        ax.set_ylim(ymin - pad, ymax + pad)
        return line,

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        reader.stop()


if __name__ == "__main__":
    main()