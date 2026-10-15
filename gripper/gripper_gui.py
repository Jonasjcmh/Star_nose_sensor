"""
Simple GUI to control a Robotiq 2F-85 connected to a UR robot.

Run:   python gripper_gui.py
       python gripper_gui.py --ip 192.168.1.10
       python gripper_gui.py --sim        (no hardware, for testing the GUI)

Slider: target position (sent when you release it). Speed/force sliders set
the next command. The panel shows live position and tells you when an object
is gripped.
"""
import argparse
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

from robotiq_gripper import RobotiqGripper, SimulatedGripper, pos_to_mm

POLL_S = 0.1


class GripperApp:
    def __init__(self, root, ip, sim):
        self.root = root
        self.gripper = None
        self.state = None
        self.stop_poll = threading.Event()
        self.poll_thread = None
        self.error = None

        root.title("Robotiq 2F-85 control")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        frm = ttk.Frame(root, padding=12)
        frm.grid()

        # --- connection ---
        ttk.Label(frm, text="Robot IP:").grid(row=0, column=0, sticky="w")
        self.ip_var = tk.StringVar(value=ip)
        ttk.Entry(frm, textvariable=self.ip_var, width=16).grid(row=0, column=1)
        self.sim_var = tk.BooleanVar(value=sim)
        ttk.Checkbutton(frm, text="Simulation", variable=self.sim_var).grid(row=0, column=2)
        self.conn_btn = ttk.Button(frm, text="Connect", command=self.toggle_connect)
        self.conn_btn.grid(row=0, column=3, padx=4)
        self.act_btn = ttk.Button(frm, text="Activate", command=self.activate, state="disabled")
        self.act_btn.grid(row=0, column=4)

        # --- position slider ---
        ttk.Label(frm, text="Target position (0 open - 255 closed)").grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(12, 0))
        self.pos_var = tk.IntVar(value=0)
        self.pos_slider = ttk.Scale(frm, from_=0, to=255, orient="horizontal", length=420,
                                    command=lambda v: self._update_target_label())
        self.pos_slider.grid(row=2, column=0, columnspan=4, sticky="we")
        self.pos_slider.bind("<ButtonRelease-1>", lambda e: self.send_target())
        self.target_lbl = ttk.Label(frm, text="")
        self.target_lbl.grid(row=2, column=4, padx=6)

        # --- speed / force ---
        ttk.Label(frm, text="Speed").grid(row=3, column=0, sticky="w", pady=(8, 0))
        self.speed = tk.Scale(frm, from_=0, to=255, orient="horizontal", length=180)
        self.speed.set(100)
        self.speed.grid(row=3, column=1, columnspan=2, sticky="w")
        ttk.Label(frm, text="Force").grid(row=4, column=0, sticky="w")
        self.force = tk.Scale(frm, from_=0, to=255, orient="horizontal", length=180)
        self.force.set(80)
        self.force.grid(row=4, column=1, columnspan=2, sticky="w")

        # --- quick buttons ---
        btns = ttk.Frame(frm)
        btns.grid(row=3, column=3, rowspan=2, columnspan=2)
        ttk.Button(btns, text="Open", command=lambda: self.go(0)).grid(row=0, column=0, padx=3)
        ttk.Button(btns, text="Close", command=lambda: self.go(255)).grid(row=0, column=1, padx=3)

        # --- feedback ---
        ttk.Separator(frm).grid(row=5, column=0, columnspan=5, sticky="we", pady=10)
        ttk.Label(frm, text="Current position").grid(row=6, column=0, sticky="w")
        self.bar = ttk.Progressbar(frm, maximum=255, length=420)
        self.bar.grid(row=6, column=1, columnspan=3, sticky="we")
        self.cur_lbl = ttk.Label(frm, text="-")
        self.cur_lbl.grid(row=6, column=4)

        self.obj_lbl = tk.Label(frm, text="Not connected", width=34, height=2,
                                bg="#888888", fg="white", font=("Helvetica", 13, "bold"))
        self.obj_lbl.grid(row=7, column=0, columnspan=5, pady=(10, 4))
        self.info_lbl = ttk.Label(frm, text="")
        self.info_lbl.grid(row=8, column=0, columnspan=5, sticky="w")

        # simulation: fake object position
        self.sim_obj_var = tk.IntVar(value=0)
        self.sim_obj_chk = ttk.Checkbutton(
            frm, text="Sim: object between fingers (~pos 140)", variable=self.sim_obj_var,
            command=self._sim_obj_changed)
        self.sim_obj_chk.grid(row=9, column=0, columnspan=5, sticky="w")

        self._update_target_label()
        self.root.after(100, self.refresh_ui)

    # ---------- helpers ----------
    def _update_target_label(self):
        p = int(self.pos_slider.get())
        self.target_lbl.config(text=f"{p}  ({pos_to_mm(p):.0f} mm)")

    def _sim_obj_changed(self):
        if isinstance(self.gripper, SimulatedGripper):
            self.gripper.object_pos = 140 if self.sim_obj_var.get() else None

    # ---------- connection ----------
    def toggle_connect(self):
        if self.gripper:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        try:
            g = SimulatedGripper() if self.sim_var.get() else RobotiqGripper(self.ip_var.get())
            g.connect()
        except Exception as e:
            messagebox.showerror("Connection failed", f"{e}\n\nCheck the robot IP, that the "
                                 "Robotiq URCap is installed and the gripper is connected.")
            return
        self.gripper = g
        self._sim_obj_changed()
        self.conn_btn.config(text="Disconnect")
        self.act_btn.config(state="normal")
        self.stop_poll.clear()
        self.poll_thread = threading.Thread(target=self._poll, daemon=True)
        self.poll_thread.start()

    def disconnect(self):
        self.stop_poll.set()
        if self.poll_thread:
            self.poll_thread.join(timeout=1)
        if self.gripper:
            self.gripper.disconnect()
        self.gripper = None
        self.state = None
        self.conn_btn.config(text="Connect")
        self.act_btn.config(state="disabled")

    def activate(self):
        def work():
            try:
                self.gripper.activate()
            except Exception as e:
                self.error = f"Activation failed: {e}"
        threading.Thread(target=work, daemon=True).start()

    # ---------- commands ----------
    def send_target(self):
        self.go(int(self.pos_slider.get()))

    def go(self, pos):
        if not self.gripper:
            return
        self.pos_slider.set(pos)
        g, sp, fo = self.gripper, self.speed.get(), self.force.get()

        def work():
            try:
                g.move(pos, sp, fo)
            except Exception as e:
                self.error = f"Command failed: {e}"
        threading.Thread(target=work, daemon=True).start()

    # ---------- polling ----------
    def _poll(self):
        while not self.stop_poll.is_set():
            try:
                self.state = self.gripper.get_state()
            except Exception as e:
                self.error = f"Lost connection: {e}"
                self.state = None
                return
            time.sleep(POLL_S)

    def refresh_ui(self):
        if self.error:
            msg, self.error = self.error, None
            if msg.startswith("Lost connection"):
                self.disconnect()
            messagebox.showerror("Gripper", msg)
        st = self.state
        if st:
            self.bar["value"] = st["position"]
            self.cur_lbl.config(text=f"{st['position']}  ({pos_to_mm(st['position']):.0f} mm)")
            if st["fault"]:
                self.obj_lbl.config(text=f"FAULT code {st['fault']}", bg="#b00020")
            elif not st["activated"]:
                self.obj_lbl.config(text="Not activated - press Activate", bg="#888888")
            elif st["moving"]:
                self.obj_lbl.config(text="Moving...", bg="#1e6fd9")
            elif st["object_detected"]:
                self.obj_lbl.config(text="OBJECT GRIPPED", bg="#1b9e3e")
            else:
                self.obj_lbl.config(text="No object (reached position)", bg="#c77d00")
            self.info_lbl.config(text=f"OBJ status = {st['obj_status']}   requested = {st['requested']}")
        elif not self.gripper:
            self.obj_lbl.config(text="Not connected", bg="#888888")
        self.root.after(100, self.refresh_ui)

    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.10", help="UR robot IP")
    ap.add_argument("--sim", action="store_true", help="simulated gripper, no hardware")
    args = ap.parse_args()
    root = tk.Tk()
    GripperApp(root, args.ip, args.sim)
    root.mainloop()


if __name__ == "__main__":
    main()
