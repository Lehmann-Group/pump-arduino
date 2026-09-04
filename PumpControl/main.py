import csv
import datetime as dt
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial

PROBE_PORT = "COM5"
PUMP_PORT = "COM4"
BAUD = 9600
SAMPLE_INTERVAL = 2.0
SERIAL_TIMEOUT = 1.0

DEFAULT_CALIBRATION = [
    {"slope": 0.15000, "intercept": -52.13303},
    {"slope": 0.15000, "intercept": -52.13303},
]

DEFAULT_SETTINGS = [
    {"target": 11.0, "dose": 0.50, "delay": 10.0, "speed": 255, "mode": "off"},
    {"target": 11.0, "dose": 0.50, "delay": 10.0, "speed": 255, "mode": "off"},
]


def send_line(ser, command):
    ser.write((command + "\n").encode())
    ser.flush()
    return ser.readline().decode(errors="replace").strip()


class Controller(threading.Thread):
    def __init__(self, commands, events):
        super().__init__(daemon=True)
        self.commands = commands
        self.events = events
        self.stop_event = threading.Event()
        self.settings = [dict(x) for x in DEFAULT_SETTINGS]
        self.calibration = [dict(x) for x in DEFAULT_CALIBRATION]
        self.next_dose = [0.0, 0.0]
        self.pump_on = [False, False]
        self.latest = [{"raw": None, "ph": None}, {"raw": None, "ph": None}]
        self.pump_ser = None
        self.log_file = None

    def emit(self, kind, **data):
        self.events.put({"type": kind, **data})

    def stop_pumps(self):
        if self.pump_ser and self.pump_ser.is_open:
            try:
                send_line(self.pump_ser, "x")
            except (serial.SerialException, OSError):
                pass
        self.pump_on = [False, False]

    def set_speed(self, pump, speed):
        speed = max(0, min(255, int(speed)))
        send_line(self.pump_ser, f"p{pump}:{speed}")
        self.pump_on[pump] = speed > 0

    def handle_commands(self):
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return

            kind = command["type"]
            if kind == "shutdown":
                self.stop_event.set()
                return

            if kind == "settings":
                pump = command["pump"]
                self.settings[pump].update(command["values"])
                self.emit("log", text=f"Pump {pump + 1} settings updated.")

            elif kind == "calibration":
                pump = command["pump"]
                self.calibration[pump].update(command["values"])
                self.emit("log", text=f"Probe {pump + 1} calibration updated.")

            elif kind == "mode":
                pump = command["pump"]
                mode = command["mode"]
                self.settings[pump]["mode"] = mode
                if mode == "off":
                    self.set_speed(pump, 0)
                elif mode == "on":
                    self.set_speed(pump, self.settings[pump]["speed"])
                else:
                    self.set_speed(pump, 0)
                self.emit("log", text=f"Pump {pump + 1} mode: {mode.upper()}")

            elif kind == "test":
                pump = command["pump"]
                self.set_speed(pump, self.settings[pump]["speed"])
                self.emit("log", text=f"Pump {pump + 1} test: PWM {self.settings[pump]['speed']}.")

            elif kind == "stop_all":
                self.stop_pumps()
                self.emit("log", text="Both pumps stopped.")

    def read_probes(self, ser):
        ser.reset_input_buffer()
        line = send_line(ser, "a")
        parts = line.split(",")
        if len(parts) != 2:
            raise RuntimeError(f"Unexpected probe response: {line!r}")
        return [float(parts[0]), float(parts[1])]

    def prepare_log(self):
        os.makedirs("data", exist_ok=True)
        self.log_file = os.path.join(
            "data", f"pump_log_{dt.datetime.now():%Y-%m-%d}.csv"
        )
        if not os.path.exists(self.log_file):
            with open(self.log_file, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow([
                    "time", "raw0", "ph0", "raw1", "ph1",
                    "pump0_mode", "pump0_speed", "pump0_target",
                    "pump1_mode", "pump1_speed", "pump1_target",
                ])

    def log_reading(self):
        with open(self.log_file, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([
                dt.datetime.now().isoformat(timespec="seconds"),
                self.latest[0]["raw"], self.latest[0]["ph"],
                self.latest[1]["raw"], self.latest[1]["ph"],
                self.settings[0]["mode"], self.settings[0]["speed"], self.settings[0]["target"],
                self.settings[1]["mode"], self.settings[1]["speed"], self.settings[1]["target"],
            ])

    def auto_dose(self, pump):
        s = self.settings[pump]
        ph = self.latest[pump]["ph"]
        now = time.monotonic()
        if s["mode"] != "auto" or ph is None or ph >= s["target"] or now < self.next_dose[pump]:
            return

        self.set_speed(pump, s["speed"])
        self.emit("log", text=(
            f"Pump {pump + 1} dosing for {s['dose']:.2f} s at PWM {s['speed']}."
        ))
        end = now + s["dose"]
        while time.monotonic() < end and not self.stop_event.is_set():
            self.handle_commands()
            if self.settings[pump]["mode"] != "auto":
                break
            time.sleep(0.02)

        if self.settings[pump]["mode"] != "on":
            self.set_speed(pump, 0)
        self.next_dose[pump] = time.monotonic() + s["delay"]

    def run(self):
        try:
            with serial.Serial(PROBE_PORT, BAUD, timeout=SERIAL_TIMEOUT) as probe_ser, \
                 serial.Serial(PUMP_PORT, BAUD, timeout=SERIAL_TIMEOUT) as self.pump_ser:
                time.sleep(2.0)
                self.stop_pumps()
                self.prepare_log()
                self.emit("connected", text=(
                    f"Connected: probes {PROBE_PORT}; pumps {PUMP_PORT}."
                ))

                while not self.stop_event.is_set():
                    cycle_start = time.monotonic()
                    self.handle_commands()
                    raw = self.read_probes(probe_ser)

                    for i in range(2):
                        c = self.calibration[i]
                        self.latest[i]["raw"] = raw[i]
                        self.latest[i]["ph"] = raw[i] * c["slope"] + c["intercept"]

                    self.auto_dose(0)
                    self.auto_dose(1)
                    self.log_reading()
                    self.emit("reading", latest=self.latest, settings=self.settings)

                    remaining = SAMPLE_INTERVAL - (time.monotonic() - cycle_start)
                    if remaining > 0:
                        self.stop_event.wait(remaining)

        except Exception as exc:
            self.emit("error", text=str(exc))
        finally:
            self.stop_pumps()
            self.emit("stopped", text="Controller stopped; both pumps commanded off.")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Two-Pump pH Controller")
        self.geometry("980x720")
        self.protocol("WM_DELETE_WINDOW", self.close)

        self.commands = queue.Queue()
        self.events = queue.Queue()
        self.controller = None

        self.status = tk.StringVar(value="Not connected")
        self.log = tk.StringVar(value="Configure settings, then click Start.")
        self.vars = []
        self.history = []

        self.build()
        self.after(100, self.process_events)

    def build(self):
        outer = ttk.Frame(self, padding=12)
        outer.pack(fill="both", expand=True)

        ttk.Label(outer, text="Two-Pump pH Dosing Controller", font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(outer, textvariable=self.status).pack(anchor="w", pady=(3, 10))

        panels = ttk.Frame(outer)
        panels.pack(fill="x")
        for pump in range(2):
            self.build_pump_panel(panels, pump)

        buttons = ttk.Frame(outer)
        buttons.pack(fill="x", pady=10)
        ttk.Button(buttons, text="Start", command=self.start).pack(side="left")
        ttk.Button(buttons, text="Stop both pumps", command=lambda: self.send({"type": "stop_all"})).pack(side="left", padx=8)
        ttk.Label(buttons, textvariable=self.log).pack(side="left", padx=10)

        history_box = ttk.LabelFrame(outer, text="Probe readings", padding=6)
        history_box.pack(fill="both", expand=True)
        columns = ("time", "raw0", "ph0", "raw1", "ph1")
        self.table = ttk.Treeview(history_box, columns=columns, show="headings", height=12)
        headings = ("Time", "Probe 1 raw", "Probe 1 pH", "Probe 2 raw", "Probe 2 pH")
        for col, heading in zip(columns, headings):
            self.table.heading(col, text=heading)
            self.table.column(col, anchor="center", width=140)
        scrollbar = ttk.Scrollbar(history_box, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=scrollbar.set)
        self.table.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def build_pump_panel(self, parent, pump):
        frame = ttk.LabelFrame(parent, text=f"Pump {pump + 1} / Probe {pump + 1}", padding=10)
        frame.pack(side="left", fill="both", expand=True, padx=5)

        values = {
            "target": tk.StringVar(value=str(DEFAULT_SETTINGS[pump]["target"])),
            "dose": tk.StringVar(value=str(DEFAULT_SETTINGS[pump]["dose"])),
            "delay": tk.StringVar(value=str(DEFAULT_SETTINGS[pump]["delay"])),
            "speed": tk.StringVar(value=str(DEFAULT_SETTINGS[pump]["speed"])),
            "slope": tk.StringVar(value=str(DEFAULT_CALIBRATION[pump]["slope"])),
            "intercept": tk.StringVar(value=str(DEFAULT_CALIBRATION[pump]["intercept"])),
            "ph": tk.StringVar(value="--"),
            "raw": tk.StringVar(value="--"),
        }
        self.vars.append(values)

        ttk.Label(frame, text="Current pH:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=values["ph"], font=("Segoe UI", 12, "bold")).grid(row=0, column=1, sticky="w")
        ttk.Label(frame, text="Raw analog:").grid(row=1, column=0, sticky="w")
        ttk.Label(frame, textvariable=values["raw"]).grid(row=1, column=1, sticky="w")

        fields = [
            ("pH target", "target"),
            ("Dose time (s)", "dose"),
            ("Redose delay (s)", "delay"),
            ("PWM speed (0-255)", "speed"),
            ("Calibration slope", "slope"),
            ("Calibration intercept", "intercept"),
        ]
        for row, (label, key) in enumerate(fields, start=2):
            ttk.Label(frame, text=label + ":").grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(frame, textvariable=values[key], width=14).grid(row=row, column=1, sticky="w", pady=2)

        ttk.Button(frame, text="Apply settings", command=lambda p=pump: self.apply(p)).grid(
            row=8, column=0, columnspan=2, sticky="ew", pady=(8, 4)
        )

        actions = ttk.Frame(frame)
        actions.grid(row=9, column=0, columnspan=2, sticky="ew")
        for col, mode in enumerate(("on", "off", "auto")):
            ttk.Button(actions, text=mode.upper(), command=lambda m=mode, p=pump: self.set_mode(p, m)).grid(
                row=0, column=col, padx=2
            )
        ttk.Button(frame, text="Test at selected speed", command=lambda p=pump: self.test(p)).grid(
            row=10, column=0, columnspan=2, sticky="ew", pady=(4, 0)
        )

    def numeric_values(self, pump):
        v = self.vars[pump]
        values = {
            "target": float(v["target"].get()),
            "dose": float(v["dose"].get()),
            "delay": float(v["delay"].get()),
            "speed": int(v["speed"].get()),
            "slope": float(v["slope"].get()),
            "intercept": float(v["intercept"].get()),
        }
        if not all(math.isfinite(x) for x in values.values() if isinstance(x, float)):
            raise ValueError("Values must be finite.")
        if values["dose"] <= 0 or values["delay"] < 0 or not 0 <= values["speed"] <= 255:
            raise ValueError("Dose must be > 0, delay >= 0, and PWM speed must be 0-255.")
        return values

    def apply(self, pump):
        try:
            values = self.numeric_values(pump)
        except ValueError as exc:
            messagebox.showerror("Invalid settings", str(exc))
            return
        self.send({"type": "settings", "pump": pump, "values": {
            key: values[key] for key in ("target", "dose", "delay", "speed")
        }})
        self.send({"type": "calibration", "pump": pump, "values": {
            key: values[key] for key in ("slope", "intercept")
        }})
        self.log.set(f"Pump {pump + 1} settings saved.")

    def set_mode(self, pump, mode):
        self.apply(pump)
        self.send({"type": "mode", "pump": pump, "mode": mode})

    def test(self, pump):
        self.apply(pump)
        self.send({"type": "test", "pump": pump})

    def send(self, command):
        if self.controller and self.controller.is_alive():
            self.commands.put(command)
        else:
            self.log.set("Start the controller first.")

    def start(self):
        if self.controller and self.controller.is_alive():
            return
        for pump in range(2):
            try:
                self.numeric_values(pump)
            except ValueError as exc:
                messagebox.showerror("Invalid settings", f"Pump {pump + 1}: {exc}")
                return
        self.controller = Controller(self.commands, self.events)
        self.controller.start()
        for pump in range(2):
            self.apply(pump)
        self.status.set("Opening serial ports...")

    def process_events(self):
        try:
            while True:
                event = self.events.get_nowait()
                kind = event["type"]
                if kind == "connected":
                    self.status.set(event["text"])
                elif kind == "log":
                    self.log.set(event["text"])
                elif kind == "error":
                    self.status.set("Controller error")
                    self.log.set(event["text"])
                    messagebox.showerror("Controller error", event["text"])
                elif kind == "stopped":
                    self.status.set(event["text"])
                elif kind == "reading":
                    latest = event["latest"]
                    now = dt.datetime.now().strftime("%H:%M:%S")
                    row = [now]
                    for i in range(2):
                        self.vars[i]["raw"].set(f"{latest[i]['raw']:.1f}")
                        self.vars[i]["ph"].set(f"{latest[i]['ph']:.3f}")
                        row += [f"{latest[i]['raw']:.1f}", f"{latest[i]['ph']:.3f}"]
                    self.table.insert("", "end", values=row)
                    children = self.table.get_children()
                    if len(children) > 300:
                        self.table.delete(children[0])
                    self.table.yview_moveto(1)
        except queue.Empty:
            pass
        self.after(100, self.process_events)

    def close(self):
        if self.controller and self.controller.is_alive():
            self.commands.put({"type": "shutdown"})
            self.after(300, self.destroy)
        else:
            self.destroy()


if __name__ == "__main__":
    App().mainloop()