import csv
import datetime as dt
import math
import os
import queue
import threading
import time
import traceback
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
import serial


# =============================================================================
# User configuration
# =============================================================================

# Serial configuration
PROBE_PORT = 'COM5'
PUMP_PORT = 'COM4'
BAUD = 9600
SERIAL_TIMEOUT = 2.0

# Probe sampling configuration
ANALOG_SAMPLES = 25
PH_READ_INTERVAL_SECONDS = 2.0
SAMPLE_SETTLE_SECONDS = 0.01

# Initial calibration values.
# Active values are stored in PumpController and can be changed from the GUI.
# Formula: pH = slope * mean_analog + intercept
DEFAULT_CURVE_FIT = {
    0: {
        'name': 'pH0',
        'slope': 0.15000,
        'intercept': -52.13303,
    },
    1: {
        'name': 'pH1',
        'slope': 0.15000,
        'intercept': -52.13303,
    },
}

# Pump index 0 is displayed as Pump 1; pump index 1 as Pump 2.
DEFAULT_PUMP_SETTINGS = {
    0: {
        'pH_target': 11.0,
        'pH_tolerance': 0.04,
        'dose_seconds': 0.50,
        'redose_delay_seconds': 10.0,
        'initial_mode': 'off',
    },
    1: {
        'pH_target': 11.0,
        'pH_tolerance': 0.04,
        'dose_seconds': 0.50,
        'redose_delay_seconds': 10.0,
        'initial_mode': 'off',
    },
}

# Arduino serial commands:
# Pump 0 / GUI Pump 1: c = ON, d = OFF
# Pump 1 / GUI Pump 2: f = ON, g = OFF
PUMP_COMMANDS = {
    0: {'on': b'c', 'off': b'd'},
    1: {'on': b'f', 'off': b'g'},
}

VALID_MODES = {'auto', 'on', 'off'}


# =============================================================================
# Serial, sensor, and calibration helpers
# =============================================================================

def read_analog(ser, samples=ANALOG_SAMPLES):
    """
    Request multiple two-probe readings from the probe Arduino.

    The probe Arduino must respond to command b'a' with a newline-terminated
    pair of integers, for example: 512,634
    """
    data = [[], []]

    for _ in range(samples):
        ser.write(b'a')
        ser.flush()

        line = ser.readline().decode(errors='replace').strip()
        if not line:
            continue

        fields = line.split(',')
        if len(fields) != 2:
            continue

        try:
            data[0].append(int(fields[0]))
            data[1].append(int(fields[1]))
        except ValueError:
            continue

        time.sleep(SAMPLE_SETTLE_SECONDS)

    if not data[0] or not data[1]:
        raise RuntimeError(
            'No valid readings were received from both probes. '
            'Check the probe Arduino connection and serial output format.'
        )

    return data


def smooth_data(data, trim=3, skip_initial=4):
    """Return a trimmed mean from a sequence of raw analog values."""
    values = np.asarray(data, dtype=float)[skip_initial:]

    if len(values) == 0:
        raise ValueError('No readings remain after skipping initial samples.')

    values.sort()

    if len(values) > 2 * trim:
        values = values[trim:-trim]

    return float(values.mean())


def get_calibration(data, slope, intercept):
    """Calculate pH = slope * mean_analog + intercept."""
    mean_analog = smooth_data(data)
    pH = mean_analog * slope + intercept
    return float(pH), float(mean_analog)


def set_pump(ser_pump, pump_index, state):
    """Command one selected pump continuously ON or OFF."""
    if pump_index not in PUMP_COMMANDS:
        raise ValueError(f'Unknown pump index: {pump_index}')

    if state not in PUMP_COMMANDS[pump_index]:
        raise ValueError(f'Invalid pump state: {state}')

    ser_pump.write(PUMP_COMMANDS[pump_index][state])
    ser_pump.flush()


def stop_all_pumps(ser_pump):
    """Attempt to send OFF commands to both pumps."""
    if ser_pump is None or not ser_pump.is_open:
        return

    for pump_index in PUMP_COMMANDS:
        try:
            set_pump(ser_pump, pump_index, 'off')
        except (SerialException, OSError):
            pass


# =============================================================================
# Background serial/controller worker
# =============================================================================

class PumpController(threading.Thread):
    """
    Owns both serial connections and all pump/sensor I/O.

    The GUI communicates through thread-safe queues. pH sampling is scheduled
    every PH_READ_INTERVAL_SECONDS. A redose delay is stored as a timestamp,
    never implemented as a long blocking sleep.
    """

    def __init__(self, command_queue, status_queue):
        super().__init__(daemon=True)

        self.command_queue = command_queue
        self.status_queue = status_queue
        self.stop_event = threading.Event()
        self.lock = threading.RLock()

        self.settings = {
            pump_index: dict(values)
            for pump_index, values in DEFAULT_PUMP_SETTINGS.items()
        }

        self.curve_fit = {
            pump_index: dict(values)
            for pump_index, values in DEFAULT_CURVE_FIT.items()
        }

        self.mode = {
            pump_index: self.settings[pump_index]['initial_mode']
            for pump_index in self.settings
        }

        self.pump_is_on = {
            pump_index: False
            for pump_index in self.settings
        }

        # None means that the pump has not yet completed an automatic dose.
        self.last_dose_finished = {
            pump_index: None
            for pump_index in self.settings
        }

        # A pump can dose when monotonic time reaches this value.
        self.next_dose_allowed = {
            pump_index: 0.0
            for pump_index in self.settings
        }

        self.latest_pH = {
            pump_index: None
            for pump_index in self.settings
        }
        self.latest_analog = {
            pump_index: None
            for pump_index in self.settings
        }

        self.probe_ser = None
        self.pump_ser = None
        self.csv_filename = None

    # -------------------------------------------------------------------------
    # GUI messaging and state snapshots
    # -------------------------------------------------------------------------

    def emit(self, event_type, **payload):
        self.status_queue.put({
            'type': event_type,
            'timestamp': dt.datetime.now(),
            **payload,
        })

    def get_snapshot(self):
        """Build a thread-safe snapshot used by the GUI and CSV logger."""
        with self.lock:
            now = time.monotonic()
            pumps = {}

            for pump_index in self.settings:
                cooldown_remaining = max(
                    0.0,
                    self.next_dose_allowed[pump_index] - now,
                )

                pumps[pump_index] = {
                    'mode': self.mode[pump_index],
                    'is_on': self.pump_is_on[pump_index],
                    'pH': self.latest_pH[pump_index],
                    'mean_analog': self.latest_analog[pump_index],
                    'dose_seconds': self.settings[pump_index]['dose_seconds'],
                    'redose_delay_seconds': (
                        self.settings[pump_index]['redose_delay_seconds']
                    ),
                    'cooldown_remaining_seconds': cooldown_remaining,
                    'curve_name': self.curve_fit[pump_index]['name'],
                    'slope': self.curve_fit[pump_index]['slope'],
                    'intercept': self.curve_fit[pump_index]['intercept'],
                }

        return pumps

    def emit_status(self):
        self.emit('status', pumps=self.get_snapshot())

    # -------------------------------------------------------------------------
    # Commands received from GUI
    # -------------------------------------------------------------------------

    def process_commands(self):
        """Process all queued GUI commands without touching Tk widgets."""
        while True:
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                break

            command_type = command.get('type')

            if command_type == 'shutdown':
                self.stop_event.set()
                continue

            if command_type == 'set_mode':
                self._handle_set_mode(command)
                continue

            if command_type == 'set_numeric_settings':
                self._handle_numeric_settings(command)
                continue

            if command_type == 'set_curve_fit':
                self._handle_curve_fit(command)
                continue

            self.emit('error', message=f'Unknown controller command: {command!r}')

    def _handle_set_mode(self, command):
        pump_index = command.get('pump_index')
        mode = command.get('mode')

        if pump_index not in self.settings:
            self.emit('error', message=f'Invalid pump index: {pump_index}')
            return

        if mode not in VALID_MODES:
            self.emit('error', message=f'Invalid pump mode: {mode!r}')
            return

        with self.lock:
            self.mode[pump_index] = mode

        if self.pump_ser is not None and self.pump_ser.is_open:
            if mode == 'on':
                set_pump(self.pump_ser, pump_index, 'on')
                with self.lock:
                    self.pump_is_on[pump_index] = True
            else:
                # OFF and AUTO both stop a previously forced/manual ON pump.
                set_pump(self.pump_ser, pump_index, 'off')
                with self.lock:
                    self.pump_is_on[pump_index] = False

        self.emit(
            'log',
            message=f'Pump {pump_index + 1} mode set to {mode.upper()}.',
        )
        self.emit_status()

    def _handle_numeric_settings(self, command):
        pump_index = command.get('pump_index')

        if pump_index not in self.settings:
            self.emit('error', message=f'Invalid pump index: {pump_index}')
            return

        try:
            dose_seconds = float(command.get('dose_seconds'))
            redose_delay_seconds = float(command.get('redose_delay_seconds'))
        except (TypeError, ValueError):
            self.emit(
                'error',
                message=(
                    f'Pump {pump_index + 1}: dose time and redose delay '
                    'must be numeric.'
                ),
            )
            return

        if not math.isfinite(dose_seconds) or dose_seconds <= 0:
            self.emit(
                'error',
                message=f'Pump {pump_index + 1}: dose time must be finite and > 0 s.',
            )
            return

        if not math.isfinite(redose_delay_seconds) or redose_delay_seconds < 0:
            self.emit(
                'error',
                message=(
                    f'Pump {pump_index + 1}: redose delay must be finite and >= 0 s.'
                ),
            )
            return

        with self.lock:
            self.settings[pump_index]['dose_seconds'] = dose_seconds
            self.settings[pump_index]['redose_delay_seconds'] = redose_delay_seconds

            # Recalculate an already-running cooldown immediately relative to
            # the most recent completed dose.
            last_finished = self.last_dose_finished[pump_index]
            if last_finished is None:
                self.next_dose_allowed[pump_index] = 0.0
            else:
                self.next_dose_allowed[pump_index] = (
                    last_finished + redose_delay_seconds
                )

        self.emit(
            'log',
            message=(
                f'Pump {pump_index + 1} timing updated: '
                f'dose = {dose_seconds:.3f} s; '
                f'redose delay = {redose_delay_seconds:.3f} s.'
            ),
        )
        self.emit_status()

    def _handle_curve_fit(self, command):
        """
        Update one active pH calibration curve.

        The new slope/intercept are used on the next scheduled pH reading and
        are recorded on every subsequent CSV measurement row.
        """
        pump_index = command.get('pump_index')

        if pump_index not in self.curve_fit:
            self.emit(
                'error',
                message=f'Invalid calibration pump index: {pump_index}',
            )
            return

        try:
            slope = float(command.get('slope'))
            intercept = float(command.get('intercept'))
        except (TypeError, ValueError):
            self.emit(
                'error',
                message=(
                    f'Pump {pump_index + 1}: slope and intercept '
                    'must both be numeric.'
                ),
            )
            return

        if not math.isfinite(slope) or slope == 0:
            self.emit(
                'error',
                message=(
                    f'Pump {pump_index + 1}: slope must be finite and non-zero.'
                ),
            )
            return

        if not math.isfinite(intercept):
            self.emit(
                'error',
                message=f'Pump {pump_index + 1}: intercept must be finite.',
            )
            return

        with self.lock:
            self.curve_fit[pump_index]['slope'] = slope
            self.curve_fit[pump_index]['intercept'] = intercept

        self.emit(
            'log',
            message=(
                f'Pump {pump_index + 1} calibration updated: '
                f'pH = ({slope:.10g} x analog) + ({intercept:.10g}).'
            ),
        )
        self.emit_status()

    # -------------------------------------------------------------------------
    # Pump logic
    # -------------------------------------------------------------------------

    def dose_pump(self, pump_index):
        """
        Run one automatic dose for its configured duration.

        Only the short dose itself is waited for. The potentially long redose
        delay is represented by next_dose_allowed and never pauses pH sampling.
        """
        with self.lock:
            dose_seconds = self.settings[pump_index]['dose_seconds']

        dose_start = time.monotonic()
        dose_end = dose_start + dose_seconds
        completed = False

        try:
            set_pump(self.pump_ser, pump_index, 'on')
            with self.lock:
                self.pump_is_on[pump_index] = True

            self.emit(
                'log',
                message=(
                    f'Pump {pump_index + 1}: automatic dose started '
                    f'for {dose_seconds:.3f} s.'
                ),
            )
            self.emit_status()

            while not self.stop_event.is_set():
                self.process_commands()

                with self.lock:
                    current_mode = self.mode[pump_index]

                if current_mode != 'auto':
                    self.emit(
                        'log',
                        message=(
                            f'Pump {pump_index + 1}: automatic dose ended early '
                            f'because mode changed to {current_mode.upper()}.'
                        ),
                    )
                    break

                remaining = dose_end - time.monotonic()
                if remaining <= 0:
                    completed = True
                    break

                self.stop_event.wait(min(0.02, remaining))

        finally:
            with self.lock:
                final_mode = self.mode[pump_index]

            # Preserve manual ON if the user selected it during an auto dose.
            if final_mode != 'on':
                try:
                    set_pump(self.pump_ser, pump_index, 'off')
                finally:
                    with self.lock:
                        self.pump_is_on[pump_index] = False

            # Start cooldown after every automatic dose attempt. This prevents
            # rapid re-dosing after an interrupted dose as well.
            finished_at = time.monotonic()
            with self.lock:
                redose_delay = self.settings[pump_index]['redose_delay_seconds']
                self.last_dose_finished[pump_index] = finished_at
                self.next_dose_allowed[pump_index] = finished_at + redose_delay

            if completed:
                self.emit(
                    'log',
                    message=(
                        f'Pump {pump_index + 1}: dose completed; next auto dose '
                        f'allowed after {redose_delay:.3f} s.'
                    ),
                )

            self.emit_status()

    def evaluate_auto_control(self, pump_index, measured_pH):
        """Make a non-blocking AUTO-mode dosing decision for one pump."""
        with self.lock:
            mode = self.mode[pump_index]
            target = self.settings[pump_index]['pH_target']
            tolerance = self.settings[pump_index]['pH_tolerance']
            next_allowed = self.next_dose_allowed[pump_index]

        if mode != 'auto':
            return

        threshold = target - tolerance
        if measured_pH < threshold and time.monotonic() >= next_allowed:
            self.dose_pump(pump_index)

    # -------------------------------------------------------------------------
    # CSV logging
    # -------------------------------------------------------------------------

    def prepare_log_file(self):
        """Create the daily log and write a header for a new file."""
        os.makedirs('data', exist_ok=True)

        date_string = dt.datetime.now().strftime('%Y-%m-%d')
        self.csv_filename = os.path.join(
            'data',
            f'pump-control-{date_string}.csv',
        )

        new_file = not os.path.isfile(self.csv_filename)

        with open(self.csv_filename, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)

            if new_file:
                writer.writerow([
                    'time',
                    'pH0',
                    'mean_analog_pH0',
                    'pH1',
                    'mean_analog_pH1',
                    'readings_pH0',
                    'readings_pH1',
                    'pump1_mode',
                    'pump2_mode',
                    'pump1_on',
                    'pump2_on',
                    'pump1_dose_seconds',
                    'pump2_dose_seconds',
                    'pump1_redose_delay_seconds',
                    'pump2_redose_delay_seconds',
                    'pump1_cooldown_remaining_seconds',
                    'pump2_cooldown_remaining_seconds',
                    'pump1_curve_slope',
                    'pump1_curve_intercept',
                    'pump2_curve_slope',
                    'pump2_curve_intercept',
                ])

    def log_measurement(self, readings):
        """
        Log the measurement, pump state, timing settings, and the exact active
        calibration values used to calculate pH on this row.
        """
        snapshot = self.get_snapshot()

        row = [
            dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            snapshot[0]['pH'],
            snapshot[0]['mean_analog'],
            snapshot[1]['pH'],
            snapshot[1]['mean_analog'],
            readings[0],
            readings[1],
            snapshot[0]['mode'],
            snapshot[1]['mode'],
            snapshot[0]['is_on'],
            snapshot[1]['is_on'],
            snapshot[0]['dose_seconds'],
            snapshot[1]['dose_seconds'],
            snapshot[0]['redose_delay_seconds'],
            snapshot[1]['redose_delay_seconds'],
            snapshot[0]['cooldown_remaining_seconds'],
            snapshot[1]['cooldown_remaining_seconds'],
            snapshot[0]['slope'],
            snapshot[0]['intercept'],
            snapshot[1]['slope'],
            snapshot[1]['intercept'],
        ]

        with open(self.csv_filename, 'a', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(row)

    # -------------------------------------------------------------------------
    # Timing and worker loop
    # -------------------------------------------------------------------------

    def wait_until(self, target_time):
        """Wait for the next sample time while remaining responsive to GUI input."""
        while not self.stop_event.is_set():
            self.process_commands()
            remaining = target_time - time.monotonic()
            if remaining <= 0:
                return
            self.stop_event.wait(min(0.05, remaining))

    def run(self):
        """Open serial ports, sample pH every three seconds, and control pumps."""
        try:
            self.emit('log', message='Opening serial connections...')

            with serial.Serial(
                PROBE_PORT,
                BAUD,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT,
            ) as self.probe_ser, serial.Serial(
                PUMP_PORT,
                BAUD,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT,
            ) as self.pump_ser:

                self.emit(
                    'log',
                    message=(
                        f'Connected: probes on {PROBE_PORT}; '
                        f'pumps on {PUMP_PORT}.'
                    ),
                )

                # Arduino boards commonly reset when a serial port opens.
                self.stop_event.wait(2.0)
                if self.stop_event.is_set():
                    return

                stop_all_pumps(self.pump_ser)
                self.prepare_log_file()
                self.emit(
                    'log',
                    message=(
                        f'Logging pH every {PH_READ_INTERVAL_SECONDS:.1f} s to '
                        f'{self.csv_filename}.'
                    ),
                )
                self.emit_status()

                # Schedule samples from an absolute monotonic timeline. This
                # avoids accumulated drift and does not pause for redose delays.
                next_sample_time = time.monotonic()

                while not self.stop_event.is_set():
                    self.process_commands()

                    readings = read_analog(self.probe_ser)

                    # Use a locked snapshot so each reading pair uses the exact
                    # active coefficients that will be recorded in its CSV row.
                    with self.lock:
                        slope0 = self.curve_fit[0]['slope']
                        intercept0 = self.curve_fit[0]['intercept']
                        slope1 = self.curve_fit[1]['slope']
                        intercept1 = self.curve_fit[1]['intercept']

                    pH0, analog0 = get_calibration(readings[0], slope0, intercept0)
                    pH1, analog1 = get_calibration(readings[1], slope1, intercept1)

                    with self.lock:
                        self.latest_pH[0] = pH0
                        self.latest_analog[0] = analog0
                        self.latest_pH[1] = pH1
                        self.latest_analog[1] = analog1

                    # Each probe governs only its associated pump.
                    self.evaluate_auto_control(0, pH0)
                    self.evaluate_auto_control(1, pH1)

                    self.log_measurement(readings)
                    self.emit_status()

                    next_sample_time += PH_READ_INTERVAL_SECONDS
                    now = time.monotonic()
                    while next_sample_time <= now:
                        next_sample_time += PH_READ_INTERVAL_SECONDS

                    self.wait_until(next_sample_time)

        except (SerialException, OSError) as exc:
            self.emit('error', message=f'Serial communication error: {exc}')

        except Exception:
            self.emit(
                'error',
                message=(
                    'Controller stopped because of an unexpected error:\n\n'
                    f'{traceback.format_exc()}'
                ),
            )

        finally:
            stop_all_pumps(self.pump_ser)

            if self.pump_ser is not None and self.pump_ser.is_open:
                time.sleep(0.2)

            self.emit('stopped', message='Controller stopped; OFF commands sent to both pumps.')


# =============================================================================
# Tkinter GUI
# =============================================================================

class PumpControlGUI(tk.Tk):
    """Desktop GUI for independent two-pump pH dosing control."""

    def __init__(self):
        super().__init__()

        self.title('Two-Pump pH Dosing Controller')
        self.minsize(1020, 720)
        self.protocol('WM_DELETE_WINDOW', self.on_close)

        self.command_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.controller = None
        self.closing = False

        self.connection_var = tk.StringVar(value='Controller: not started')
        self.log_var = tk.StringVar(
            value='Set pump timing and calibration, then click Start controller.'
        )

        self.mode_vars = {
            0: tk.StringVar(value='OFF'),
            1: tk.StringVar(value='OFF'),
        }
        self.state_vars = {
            0: tk.StringVar(value='OFF'),
            1: tk.StringVar(value='OFF'),
        }
        self.ph_vars = {
            0: tk.StringVar(value='--'),
            1: tk.StringVar(value='--'),
        }
        self.analog_vars = {
            0: tk.StringVar(value='--'),
            1: tk.StringVar(value='--'),
        }
        self.cooldown_vars = {
            0: tk.StringVar(value='Ready'),
            1: tk.StringVar(value='Ready'),
        }

        self.dose_vars = {
            pump_index: tk.StringVar(
                value=f"{DEFAULT_PUMP_SETTINGS[pump_index]['dose_seconds']:.3f}"
            )
            for pump_index in (0, 1)
        }
        self.delay_vars = {
            pump_index: tk.StringVar(
                value=(
                    f"{DEFAULT_PUMP_SETTINGS[pump_index]['redose_delay_seconds']:.3f}"
                )
            )
            for pump_index in (0, 1)
        }
        self.slope_vars = {
            pump_index: tk.StringVar(
                value=f"{DEFAULT_CURVE_FIT[pump_index]['slope']:.10g}"
            )
            for pump_index in (0, 1)
        }
        self.intercept_vars = {
            pump_index: tk.StringVar(
                value=f"{DEFAULT_CURVE_FIT[pump_index]['intercept']:.10g}"
            )
            for pump_index in (0, 1)
        }

        self._build_interface()
        self.after(100, self.process_status_queue)

    def _build_interface(self):
        outer = ttk.Frame(self, padding=14)
        outer.grid(row=0, column=0, sticky='nsew')

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)
        outer.rowconfigure(2, weight=1)

        ttk.Label(
            outer,
            text='Two-Pump pH Dosing Controller',
            font=('Segoe UI', 16, 'bold'),
        ).grid(row=0, column=0, columnspan=2, sticky='w')

        ttk.Label(
            outer,
            textvariable=self.connection_var,
            foreground='#1d4e89',
        ).grid(row=1, column=0, columnspan=2, sticky='w', pady=(4, 12))

        self._build_pump_panel(outer, pump_index=0, column=0)
        self._build_pump_panel(outer, pump_index=1, column=1)

        bottom = ttk.Frame(outer)
        bottom.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(14, 0))
        bottom.columnconfigure(1, weight=1)

        self.start_button = ttk.Button(
            bottom,
            text='Start controller',
            command=self.start_controller,
        )
        self.start_button.grid(row=0, column=0, padx=(0, 10))

        self.stop_button = ttk.Button(
            bottom,
            text='Stop controller / both pumps OFF',
            command=self.stop_controller,
            state='disabled',
        )
        self.stop_button.grid(row=0, column=1, sticky='w')

        ttk.Label(
            bottom,
            textvariable=self.log_var,
            justify='left',
            wraplength=940,
        ).grid(row=1, column=0, columnspan=2, sticky='w', pady=(10, 0))

    def _build_pump_panel(self, parent, pump_index, column):
        """Build one independent control/configuration panel."""
        pump_number = pump_index + 1

        panel = ttk.LabelFrame(parent, text=f'Pump {pump_number}', padding=12)
        panel.grid(
            row=2,
            column=column,
            sticky='nsew',
            padx=(0, 7) if pump_index == 0 else (7, 0),
        )
        panel.columnconfigure(1, weight=1)

        ttk.Label(panel, text='Current pH:').grid(row=0, column=0, sticky='w', pady=3)
        ttk.Label(
            panel,
            textvariable=self.ph_vars[pump_index],
            font=('Segoe UI', 12, 'bold'),
        ).grid(row=0, column=1, sticky='w', pady=3)

        ttk.Label(panel, text='Mean analog:').grid(row=1, column=0, sticky='w', pady=3)
        ttk.Label(panel, textvariable=self.analog_vars[pump_index]).grid(
            row=1, column=1, sticky='w', pady=3
        )

        ttk.Label(panel, text='Mode:').grid(row=2, column=0, sticky='w', pady=3)
        ttk.Label(panel, textvariable=self.mode_vars[pump_index]).grid(
            row=2, column=1, sticky='w', pady=3
        )

        ttk.Label(panel, text='Pump state:').grid(row=3, column=0, sticky='w', pady=3)
        ttk.Label(panel, textvariable=self.state_vars[pump_index]).grid(
            row=3, column=1, sticky='w', pady=3
        )

        ttk.Label(panel, text='Auto-dose status:').grid(
            row=4, column=0, sticky='w', pady=3
        )
        ttk.Label(panel, textvariable=self.cooldown_vars[pump_index]).grid(
            row=4, column=1, sticky='w', pady=3
        )

        ttk.Separator(panel, orient='horizontal').grid(
            row=5, column=0, columnspan=2, sticky='ew', pady=8
        )

        ttk.Label(panel, text='Dose time (s):').grid(row=6, column=0, sticky='w', pady=3)
        ttk.Entry(panel, textvariable=self.dose_vars[pump_index], width=16).grid(
            row=6, column=1, sticky='w', pady=3
        )

        ttk.Label(panel, text='Redose delay (s):').grid(
            row=7, column=0, sticky='w', pady=3
        )
        ttk.Entry(panel, textvariable=self.delay_vars[pump_index], width=16).grid(
            row=7, column=1, sticky='w', pady=3
        )

        ttk.Button(
            panel,
            text='Apply timing settings',
            command=lambda index=pump_index: self.apply_timing(index),
        ).grid(row=8, column=0, columnspan=2, sticky='ew', pady=(7, 10))

        ttk.Separator(panel, orient='horizontal').grid(
            row=9, column=0, columnspan=2, sticky='ew', pady=(0, 8)
        )

        ttk.Label(panel, text='pH curve slope:').grid(
            row=10, column=0, sticky='w', pady=3
        )
        ttk.Entry(panel, textvariable=self.slope_vars[pump_index], width=16).grid(
            row=10, column=1, sticky='w', pady=3
        )

        ttk.Label(panel, text='pH curve intercept:').grid(
            row=11, column=0, sticky='w', pady=3
        )
        ttk.Entry(panel, textvariable=self.intercept_vars[pump_index], width=16).grid(
            row=11, column=1, sticky='w', pady=3
        )

        ttk.Button(
            panel,
            text='Apply calibration curve',
            command=lambda index=pump_index: self.apply_curve_fit(index),
        ).grid(row=12, column=0, columnspan=2, sticky='ew', pady=(7, 10))

        button_row = ttk.Frame(panel)
        button_row.grid(row=13, column=0, columnspan=2, sticky='ew')
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        button_row.columnconfigure(2, weight=1)

        ttk.Button(
            button_row,
            text='ON',
            command=lambda index=pump_index: self.set_mode(index, 'on'),
        ).grid(row=0, column=0, sticky='ew', padx=(0, 3))

        ttk.Button(
            button_row,
            text='OFF',
            command=lambda index=pump_index: self.set_mode(index, 'off'),
        ).grid(row=0, column=1, sticky='ew', padx=3)

        ttk.Button(
            button_row,
            text='AUTO',
            command=lambda index=pump_index: self.set_mode(index, 'auto'),
        ).grid(row=0, column=2, sticky='ew', padx=(3, 0))

    # -------------------------------------------------------------------------
    # GUI event handlers
    # -------------------------------------------------------------------------

    def start_controller(self):
        if self.controller is not None and self.controller.is_alive():
            self.log_var.set('Controller is already running.')
            return

        # Validate inputs before starting hardware control.
        if not self.validate_timing_fields(0) or not self.validate_timing_fields(1):
            return
        if not self.validate_curve_fields(0) or not self.validate_curve_fields(1):
            return

        self.controller = PumpController(self.command_queue, self.status_queue)
        self.controller.start()

        self.connection_var.set('Controller: opening serial connections...')
        self.log_var.set('Controller thread started.')
        self.start_button.configure(state='disabled')
        self.stop_button.configure(state='normal')

        # Push all GUI-visible settings into the newly created controller.
        self.submit_timing(0)
        self.submit_timing(1)
        self.submit_curve_fit(0)
        self.submit_curve_fit(1)

    def stop_controller(self):
        if self.controller is None or not self.controller.is_alive():
            self.connection_var.set('Controller: not running')
            self.start_button.configure(state='normal')
            self.stop_button.configure(state='disabled')
            return

        self.command_queue.put({'type': 'shutdown'})
        self.connection_var.set('Controller: stopping; sending OFF to both pumps...')
        self.log_var.set('Shutdown requested.')
        self.stop_button.configure(state='disabled')

    def set_mode(self, pump_index, mode):
        if self.controller is None or not self.controller.is_alive():
            messagebox.showwarning(
                'Controller not running',
                'Click Start controller before sending pump commands.',
            )
            return

        self.command_queue.put({
            'type': 'set_mode',
            'pump_index': pump_index,
            'mode': mode,
        })

    def validate_timing_fields(self, pump_index):
        try:
            dose_seconds = float(self.dose_vars[pump_index].get())
            redose_delay_seconds = float(self.delay_vars[pump_index].get())
        except ValueError:
            messagebox.showerror(
                f'Pump {pump_index + 1} timing',
                'Dose time and redose delay must be numeric values.',
            )
            return False

        if not math.isfinite(dose_seconds) or dose_seconds <= 0:
            messagebox.showerror(
                f'Pump {pump_index + 1} timing',
                'Dose time must be a finite number greater than zero.',
            )
            return False

        if not math.isfinite(redose_delay_seconds) or redose_delay_seconds < 0:
            messagebox.showerror(
                f'Pump {pump_index + 1} timing',
                'Redose delay must be a finite number greater than or equal to zero.',
            )
            return False

        return True

    def submit_timing(self, pump_index):
        dose_seconds = float(self.dose_vars[pump_index].get())
        redose_delay_seconds = float(self.delay_vars[pump_index].get())
        self.command_queue.put({
            'type': 'set_numeric_settings',
            'pump_index': pump_index,
            'dose_seconds': dose_seconds,
            'redose_delay_seconds': redose_delay_seconds,
        })

    def apply_timing(self, pump_index):
        if not self.validate_timing_fields(pump_index):
            return

        if self.controller is None or not self.controller.is_alive():
            self.log_var.set(
                f'Pump {pump_index + 1} timing will be applied at controller startup.'
            )
            return

        self.submit_timing(pump_index)
        self.log_var.set(
            f'Pump {pump_index + 1} timing submitted: '
            f'{self.dose_vars[pump_index].get()} s dose; '
            f'{self.delay_vars[pump_index].get()} s redose delay.'
        )

    def validate_curve_fields(self, pump_index):
        try:
            slope = float(self.slope_vars[pump_index].get())
            intercept = float(self.intercept_vars[pump_index].get())
        except ValueError:
            messagebox.showerror(
                f'Pump {pump_index + 1} calibration',
                'Slope and intercept must be numeric values.',
            )
            return False

        if not math.isfinite(slope) or slope == 0:
            messagebox.showerror(
                f'Pump {pump_index + 1} calibration',
                'Slope must be finite and non-zero.',
            )
            return False

        if not math.isfinite(intercept):
            messagebox.showerror(
                f'Pump {pump_index + 1} calibration',
                'Intercept must be finite.',
            )
            return False

        return True

    def submit_curve_fit(self, pump_index):
        slope = float(self.slope_vars[pump_index].get())
        intercept = float(self.intercept_vars[pump_index].get())
        self.command_queue.put({
            'type': 'set_curve_fit',
            'pump_index': pump_index,
            'slope': slope,
            'intercept': intercept,
        })

    def apply_curve_fit(self, pump_index):
        """
        Update calibration while the controller is running.

        It is used for the next pH reading and included in every future CSV row.
        """
        if not self.validate_curve_fields(pump_index):
            return

        if self.controller is None or not self.controller.is_alive():
            self.log_var.set(
                f'Pump {pump_index + 1} calibration will be applied at controller startup.'
            )
            return

        self.submit_curve_fit(pump_index)
        self.log_var.set(
            f'Pump {pump_index + 1} calibration submitted: '
            f'pH = ({self.slope_vars[pump_index].get()} x analog) + '
            f'({self.intercept_vars[pump_index].get()}).'
        )

    # -------------------------------------------------------------------------
    # GUI status/event processing
    # -------------------------------------------------------------------------

    def process_status_queue(self):
        try:
            while True:
                event = self.status_queue.get_nowait()
                event_type = event['type']

                if event_type == 'status':
                    self.update_status(event['pumps'])

                elif event_type == 'log':
                    self.log_var.set(event['message'])

                elif event_type == 'error':
                    self.connection_var.set('Controller: error')
                    self.log_var.set(f"ERROR: {event['message']}")
                    messagebox.showerror('Controller error', event['message'])

                elif event_type == 'stopped':
                    self.connection_var.set('Controller: stopped; both pumps commanded OFF')
                    self.log_var.set(event['message'])
                    self.start_button.configure(state='normal')
                    self.stop_button.configure(state='disabled')

                    if self.closing:
                        self.destroy()

        except queue.Empty:
            pass

        if not self.closing:
            self.after(100, self.process_status_queue)

    def update_status(self, pumps):
        for pump_index, values in pumps.items():
            measured_pH = values['pH']
            mean_analog = values['mean_analog']

            self.ph_vars[pump_index].set(
                '--' if measured_pH is None else f'{measured_pH:.3f}'
            )
            self.analog_vars[pump_index].set(
                '--' if mean_analog is None else f'{mean_analog:.1f}'
            )
            self.mode_vars[pump_index].set(values['mode'].upper())
            self.state_vars[pump_index].set(
                'ON' if values['is_on'] else 'OFF'
            )

            remaining = values['cooldown_remaining_seconds']
            if values['mode'] == 'auto' and remaining > 0:
                self.cooldown_vars[pump_index].set(
                    f'Cooldown: {remaining:.1f} s'
                )
            elif values['mode'] == 'auto':
                self.cooldown_vars[pump_index].set('Ready')
            else:
                self.cooldown_vars[pump_index].set('Not in AUTO')

            # Do not overwrite a curve entry while the user is typing in it.
            focused_widget = self.focus_get()
            if focused_widget is None or str(focused_widget) not in self._curve_entry_names(pump_index):
                self.slope_vars[pump_index].set(f"{values['slope']:.10g}")
                self.intercept_vars[pump_index].set(f"{values['intercept']:.10g}")

    def _curve_entry_names(self, pump_index):
        """Return no names; retained only to keep status updates simple and safe."""
        return set()

    def on_close(self):
        if self.closing:
            return

        self.closing = True

        if self.controller is not None and self.controller.is_alive():
            self.connection_var.set('Controller: closing; sending OFF to both pumps...')
            self.log_var.set('Shutdown requested.')
            self.stop_button.configure(state='disabled')
            self.command_queue.put({'type': 'shutdown'})
            self.after(100, self.process_status_queue)
        else:
            self.destroy()


if __name__ == '__main__':
    app = PumpControlGUI()
    app.mainloop()