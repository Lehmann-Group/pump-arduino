import csv
import datetime as dt
import json
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
SETTINGS_FILE = "pump_controller_settings.json"

DEFAULT_CALIBRATION = [
    {"x_calibration": "217, 400, 572", "y_calibration": "4, 7, 10"},
    {"x_calibration": "217, 400, 572", "y_calibration": "4, 7, 10"},
]

DEFAULT_SETTINGS = [
    {"target": 11.0, "dose": 0.50, "delay": 10.0, "speed": 255, "mode": "off"},
    {"target": 11.0, "dose": 0.50, "delay": 10.0, "speed": 255, "mode": "off"},
]

LOG_COLUMNS = [
    "time", "raw0", "ph0", "raw1", "ph1",
    "pump0_mode", "pump0_speed", "pump0_target",
    "pump0_dose_s", "pump0_redose_delay_s",
    "pump0_x_calibration", "pump0_y_calibration",
    "pump0_slope", "pump0_intercept",
    "pump1_mode", "pump1_speed", "pump1_target",
    "pump1_dose_s", "pump1_redose_delay_s",
    "pump1_x_calibration", "pump1_y_calibration",
    "pump1_slope", "pump1_intercept",
    "pump0_experimental_run", "pump1_experimental_run",
]


def send_line(ser, command):
    ser.write((command + "\n").encode())
    ser.flush()
    return ser.readline().decode(errors="replace").strip()


def parse_calibration_values(text, field_name):
    try:
        values = [float(item.strip()) for item in text.split(",")]
    except ValueError as exc:
        raise ValueError(f"{field_name} must be comma-separated numbers.") from exc
    if len(values) < 2 or any(not math.isfinite(value) for value in values):
        raise ValueError(f"{field_name} must contain at least two finite numbers.")
    return values


def calculate_linear_calibration(x_values, y_values):
    if len(x_values) != len(y_values):
        raise ValueError("X calibration and Y calibration must contain the same number of values.")
    n = len(x_values)
    x_mean = sum(x_values) / n
    y_mean = sum(y_values) / n
    denominator = sum((x - x_mean) ** 2 for x in x_values)
    if denominator == 0:
        raise ValueError("X calibration values must not all be identical.")
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_values, y_values)) / denominator
    intercept = y_mean - slope * x_mean
    if not all(math.isfinite(value) for value in (slope, intercept)):
        raise ValueError("Calibration coefficients must be finite.")
    return slope, intercept


def load_saved_gui_settings():
    defaults = {
        "geometry": "980x720",
        "pumps": [{**DEFAULT_SETTINGS[i], **DEFAULT_CALIBRATION[i]} for i in range(2)],
    }
    if not os.path.exists(SETTINGS_FILE):
        return defaults
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
            saved = json.load(f)
        if not isinstance(saved, dict) or not isinstance(saved.get("pumps"), list):
            raise ValueError("Invalid settings file structure")
        pumps = []
        for i in range(2):
            pump_data = saved["pumps"][i] if i < len(saved["pumps"]) else {}
            if not isinstance(pump_data, dict):
                pump_data = {}
            pumps.append({**defaults["pumps"][i], **pump_data})
        geometry = saved.get("geometry", defaults["geometry"])
        return {"geometry": geometry if isinstance(geometry, str) else defaults["geometry"], "pumps": pumps}
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return defaults


class Controller(threading.Thread):
    def __init__(self, commands, events, settings, calibration):
        super().__init__(daemon=True)
        self.commands = commands
        self.events = events
        self.stop_event = threading.Event()
        self.settings = [dict(x) for x in settings]
        self.calibration = [dict(x) for x in calibration]
        self.next_dose = [0.0, 0.0]
        self.pump_on = [False, False]
        self.experiment_active = [False, False]
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
                self.emit("log", text=(
                    f"Probe {pump + 1} calibration updated: "
                    f"slope {self.calibration[pump]['slope']:.6f}, "
                    f"intercept {self.calibration[pump]['intercept']:.6f}."
                ))
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
            elif kind == "stop_all":
                self.stop_pumps()
                self.emit("log", text="Both pumps stopped.")
            elif kind == "experiment":
                pump = command["pump"]
                self.experiment_active[pump] = bool(command["active"])
                label = "STARTED" if self.experiment_active[pump] else "ENDED"
                self.log_experiment_event(pump, label)
                self.emit("experiment", pump=pump, active=self.experiment_active[pump])
                self.emit("log", text=f"Experimental run {label.lower()} for probe/pump {pump + 1}.")

    def read_probes(self, ser):
        ser.reset_input_buffer()
        line = send_line(ser, "a")
        parts = line.split(",")
        if len(parts) != 2:
            raise RuntimeError(f"Unexpected probe response: {line!r}")
        return [float(parts[0]), float(parts[1])]

    def prepare_log(self):
        os.makedirs("data", exist_ok=True)
        date = dt.datetime.now().strftime("%Y-%m-%d")
        base = os.path.join("data", f"pump_log_{date}")
        path = base + ".csv"
        version = 2
        while os.path.isfile(path):
            with open(path, "r", newline="", encoding="utf-8") as f:
                header = next(csv.reader(f), [])
            if header == LOG_COLUMNS:
                break
            path = f"{base}_v{version}.csv"
            version += 1
        self.log_file = path
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            with open(path, "w", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(LOG_COLUMNS)

    def log_row(self, timestamp=None):
        c0, c1 = self.calibration
        s0, s1 = self.settings
        return [
            timestamp or dt.datetime.now().isoformat(timespec="seconds"),
            self.latest[0]["raw"], self.latest[0]["ph"],
            self.latest[1]["raw"], self.latest[1]["ph"],
            s0["mode"], s0["speed"], s0["target"], s0["dose"], s0["delay"],
            c0["x_calibration"], c0["y_calibration"], c0["slope"], c0["intercept"],
            s1["mode"], s1["speed"], s1["target"], s1["dose"], s1["delay"],
            c1["x_calibration"], c1["y_calibration"], c1["slope"], c1["intercept"],
            int(self.experiment_active[0]), int(self.experiment_active[1]),
        ]

    def log_reading(self):
        with open(self.log_file, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(self.log_row())

    def log_experiment_event(self, pump, label):
        os.makedirs("data", exist_ok=True)
        path = os.path.join("data", f"experiment_events_{dt.datetime.now():%Y-%m-%d}.csv")
        create_header = not os.path.isfile(path) or os.path.getsize(path) == 0
        with open(path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if create_header:
                writer.writerow(["time", "probe_pump_pair", "event", "main_log_file"])
            writer.writerow([dt.datetime.now().isoformat(timespec="seconds"), pump + 1, label, self.log_file])
        self.log_reading()

    def auto_dose(self, pump):
        s = self.settings[pump]
        ph = self.latest[pump]["ph"]
        now = time.monotonic()
        if s["mode"] != "auto" or ph is None or ph >= s["target"] or now < self.next_dose[pump]:
            return
        self.set_speed(pump, s["speed"])
        self.emit("log", text=f"Pump {pump + 1} dosing for {s['dose']:.2f} s at PWM {s['speed']}.")
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
                self.emit("connected", text=f"Connected: probes {PROBE_PORT}; pumps {PUMP_PORT}.")
                while not self.stop_event.is_set():
                    cycle_start = time.monotonic()
                    self.handle_commands()
                    if self.stop_event.is_set():
                        break
                    raw = self.read_probes(probe_ser)
                    for i in range(2):
                        c = self.calibration[i]
                        self.latest[i]["raw"] = raw[i]
                        self.latest[i]["ph"] = raw[i] * c["slope"] + c["intercept"]
                    self.auto_dose(0)
                    self.auto_dose(1)
                    if self.stop_event.is_set():
                        break
                    self.log_reading()
                    self.emit("reading", latest=[dict(x) for x in self.latest])
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
        saved = load_saved_gui_settings()
        self.title("Two-Pump pH Controller")
        try:
            self.geometry(saved["geometry"])
        except tk.TclError:
            self.geometry("980x720")
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.commands = queue.Queue()
        self.events = queue.Queue()
        self.controller = None
        self.saved_pumps = saved["pumps"]
        self.experiment_active = [False, False]
        self.experiment_buttons = []
        self.status = tk.StringVar(value="Not connected")
        self.log = tk.StringVar(value="Configure settings, then click Start.")
        self.vars = []
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
        saved = self.saved_pumps[pump]
        values = {
            "target": tk.StringVar(value=str(saved["target"])),
            "dose": tk.StringVar(value=str(saved["dose"])),
            "delay": tk.StringVar(value=str(saved["delay"])),
            "speed": tk.StringVar(value=str(saved["speed"])),
            "y_calibration": tk.StringVar(value=str(saved["y_calibration"])),
            "x_calibration": tk.StringVar(value=str(saved["x_calibration"])),
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
            ("Y calibration (pH)", "y_calibration"),
            ("X calibration (raw)", "x_calibration"),
        ]
        for row, (label, key) in enumerate(fields, start=2):
            ttk.Label(frame, text=label + ":").grid(row=row, column=0, sticky="w", pady=2)
            width = 24 if key in ("x_calibration", "y_calibration") else 14
            ttk.Entry(frame, textvariable=values[key], width=width).grid(row=row, column=1, sticky="w", pady=2)
        ttk.Button(frame, text="Apply settings", command=lambda p=pump: self.apply(p)).grid(
            row=8, column=0, columnspan=2, sticky="ew", pady=(8, 4)
        )
        actions = ttk.Frame(frame)
        actions.grid(row=9, column=0, columnspan=2, sticky="ew")
        for col, mode in enumerate(("on", "off", "auto")):
            ttk.Button(actions, text=mode.upper(), command=lambda m=mode, p=pump: self.set_mode(p, m)).grid(
                row=0, column=col, padx=2
            )
        button = ttk.Button(frame, text="Start experimental run", command=lambda p=pump: self.toggle_experiment(p))
        button.grid(row=10, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        button.state(["disabled"])
        self.experiment_buttons.append(button)

    def numeric_values(self, pump):
        v = self.vars[pump]
        try:
            values = {
                "target": float(v["target"].get()),
                "dose": float(v["dose"].get()),
                "delay": float(v["delay"].get()),
                "speed": int(v["speed"].get()),
            }
        except ValueError as exc:
            raise ValueError("Target, dose, delay, and speed must be numeric.") from exc
        x_values = parse_calibration_values(v["x_calibration"].get(), "X calibration")
        y_values = parse_calibration_values(v["y_calibration"].get(), "Y calibration")
        slope, intercept = calculate_linear_calibration(x_values, y_values)
        if not all(math.isfinite(x) for x in values.values() if isinstance(x, float)):
            raise ValueError("Values must be finite.")
        if values["dose"] <= 0 or values["delay"] < 0 or not 0 <= values["speed"] <= 255:
            raise ValueError("Dose must be > 0, delay >= 0, and PWM speed must be 0-255.")
        values.update({
            "x_calibration": v["x_calibration"].get().strip(),
            "y_calibration": v["y_calibration"].get().strip(),
            "slope": slope,
            "intercept": intercept,
        })
        return values

    def collect_saved_pump_settings(self):
        pumps = []
        for pump in range(2):
            v = self.vars[pump]
            pumps.append({
                "target": v["target"].get(),
                "dose": v["dose"].get(),
                "delay": v["delay"].get(),
                "speed": v["speed"].get(),
                "mode": self.saved_pumps[pump].get("mode", "off"),
                "x_calibration": v["x_calibration"].get(),
                "y_calibration": v["y_calibration"].get(),
            })
        return pumps

    def save_gui_settings(self):
        payload = {"geometry": self.geometry(), "pumps": self.collect_saved_pump_settings()}
        try:
            with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
        except OSError as exc:
            messagebox.showwarning("Settings not saved", f"Could not save GUI settings:\n{exc}")

    def apply(self, pump):
        try:
            values = self.numeric_values(pump)
        except ValueError as exc:
            messagebox.showerror("Invalid settings", str(exc))
            return False
        self.saved_pumps[pump].update({
            key: values[key]
            for key in ("target", "dose", "delay", "speed", "x_calibration", "y_calibration")
        })
        self.send({"type": "settings", "pump": pump, "values": {
            key: values[key] for key in ("target", "dose", "delay", "speed")
        }})
        self.send({"type": "calibration", "pump": pump, "values": {
            key: values[key] for key in ("x_calibration", "y_calibration", "slope", "intercept")
        }})
        self.log.set(f"Pump {pump + 1} settings saved; pH = {values['slope']:.6f} × raw + {values['intercept']:.6f}.")
        return True

    def set_mode(self, pump, mode):
        if self.apply(pump):
            self.saved_pumps[pump]["mode"] = mode
            self.send({"type": "mode", "pump": pump, "mode": mode})

    def toggle_experiment(self, pump):
        if not self.controller or not self.controller.is_alive():
            self.log.set("Start the controller before marking an experimental run.")
            return
        self.experiment_buttons[pump].state(["disabled"])
        self.send({"type": "experiment", "pump": pump, "active": not self.experiment_active[pump]})

    def send(self, command):
        if self.controller and self.controller.is_alive():
            self.commands.put(command)
        else:
            self.log.set("Start the controller first.")

    def start(self):
        if self.controller and self.controller.is_alive():
            return
        settings = []
        calibration = []
        for pump in range(2):
            try:
                values = self.numeric_values(pump)
            except ValueError as exc:
                messagebox.showerror("Invalid settings", f"Pump {pump + 1}: {exc}")
                return
            settings.append({key: values[key] for key in ("target", "dose", "delay", "speed")})
            calibration.append({
                key: values[key]
                for key in ("x_calibration", "y_calibration", "slope", "intercept")
            })
            self.saved_pumps[pump].update({**settings[-1], **calibration[-1]})
        self.controller = Controller(self.commands, self.events, settings, calibration)
        self.controller.start()
        self.status.set("Opening serial ports...")

    def process_events(self):
        try:
            while True:
                event = self.events.get_nowait()
                kind = event["type"]
                if kind == "connected":
                    self.status.set(event["text"])
                    for button in self.experiment_buttons:
                        button.state(["!disabled"])
                elif kind == "log":
                    self.log.set(event["text"])
                elif kind == "error":
                    self.status.set("Controller error")
                    self.log.set(event["text"])
                    messagebox.showerror("Controller error", event["text"])
                elif kind == "stopped":
                    self.status.set(event["text"])
                    self.experiment_active = [False, False]
                    for button in self.experiment_buttons:
                        button.configure(text="Start experimental run")
                        button.state(["disabled"])
                elif kind == "experiment":
                    pump = event["pump"]
                    self.experiment_active[pump] = event["active"]
                    self.experiment_buttons[pump].configure(
                        text="End experimental run" if event["active"] else "Start experimental run"
                    )
                    self.experiment_buttons[pump].state(["!disabled"])
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
        self.save_gui_settings()
        if self.controller and self.controller.is_alive():
            self.commands.put({"type": "shutdown"})
            self.after(300, self.destroy)
        else:
            self.destroy()


if __name__ == "__main__":
    App().mainloop()
