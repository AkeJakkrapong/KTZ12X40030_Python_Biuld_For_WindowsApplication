#!/usr/bin/env python3
"""
KTZ Motor Controller CAN Monitor
Protocol: EC KTZ12X40030 | Device: Waveshare USB-CAN-B
Baud: 250 Kbps | Byte order: Intel (little-endian)
"""

import tkinter as tk
from tkinter import ttk
import customtkinter as ctk
import glob
import threading
import queue
import time
import sys
import argparse
import os
import pathlib

# Register libusb-1.0.dll so PyUSB (used by canalystii) can find it on Windows.
# PyUSB's find_library searches PATH, so we prepend the DLL directory to PATH.
if sys.platform == "win32":
    try:
        _meipass = pathlib.Path(getattr(sys, "_MEIPASS", ""))
        _dll = _meipass / "libusb-1.0.dll"
        if _dll.exists():
            _dll_dir = str(_meipass)
        else:
            import sysconfig as _sc, platform as _plat
            _arch = "x86_64" if _plat.machine() == "AMD64" else _plat.machine().lower()
            _sp = pathlib.Path(_sc.get_path("purelib"))
            _dll = _sp / "libusb" / "_platform" / "windows" / _arch / "libusb-1.0.dll"
            _dll_dir = str(_dll.parent) if _dll.exists() else None
        if _dll_dir:
            os.environ["PATH"] = _dll_dir + os.pathsep + os.environ.get("PATH", "")
    except Exception:
        pass

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

# ── CAN Message IDs (29-bit extended) ──────────────────────────────────────
MSG_INSTRUMENT = 0x0D259CD0   # Period 100 ms
MSG_FAULT_DISP = 0x0D259CE7   # Period 100 ms

def _scan_ports(interface: str = "") -> list[str]:
    if interface == "canalystii":
        return ["0", "1"]
    if sys.platform == "win32":
        try:
            from serial.tools.list_ports import comports
            ports = sorted(p.device for p in comports())
            return ports if ports else ["COM3"]
        except Exception:
            return ["COM3"]
    ports = sorted(glob.glob("/dev/tty.usb*") + glob.glob("/dev/tty.wch*"))
    return ports if ports else ["/dev/tty.usbserial-0001"]


# ── Device Presets ──────────────────────────────────────────────────────────
DEVICES = {
    "Waveshare USB-CAN-B": "canalystii",
    "Robotell USB-CAN":    "robotell",
    "SLCAN (Generic)":     "slcan",
}

# ── Protocol Constants ──────────────────────────────────────────────────────
SPEED_DIVISOR = 86.975   # shaft_rpm / SPEED_DIVISOR = km/h

FAULT_CODES = {
    0: "No Fault",
    1: "TCU Failure",
    2: "Gear Position Sensor Failure",
    3: "Magnetic Encoder Failure",
    4: "Shift Motor Failure",
    5: "Accelerator Pedal Sensor Failure",
    6: "Brake Sensor Failure",
    7: "MCU Failure",
    8: "Motor / MCU Overheating",
}

# ── Colour Palette ──────────────────────────────────────────────────────────
C_BG       = "#090d18"
C_CARD     = "#101726"
C_BORDER   = "#1c2b3a"
C_FG       = "#c9d8e8"
C_MUTED    = "#526070"
C_ACCENT   = "#38bdf8"
C_VALUE    = "#e8f2ff"
C_OK       = "#34d399"
C_WARN     = "#fbbf24"
C_ERR      = "#f87171"
C_OFF      = "#1e2d3d"


# ── Protocol Decoder ────────────────────────────────────────────────────────

def decode_fault_display(data: bytes) -> dict:
    """Decode CAN ID 0x0D259CE7 (8 bytes, Intel byte order).
    Byte layout per EC KTZ12X40030 protocol doc:
      0: 12V voltage    raw x 5 × 0.024  V
      1: throttle       raw × 6      (dimensionless)
      2: gear position  raw × 5      (dimensionless)
      3: magnetic code  raw          (0-127, wraps at 127)
      4: motor temp     raw x 2 −128    °C
      5: MCU temp       raw x 2 −128    °C
      6: 4.096V ref     raw x 5 × 0.012011 V
      7: 5V supply      raw x2 × 0.012    V
    """
    return {
        "v12":        data[0] * 5 * 0.024,
        "throttle":   data[1] * 6,
        "gear_pos":   data[2] * 5,
        "mag_code":   data[3] * 128,
        "motor_temp": data[4]*2 - 128,
        "mcu_temp":   data[5]*2 - 128,
        "v4096":      data[6] * 5 * 0.012011,
        "v5":         data[7] * 2 * 0.012,
    }


def decode_instrument(data: bytes) -> dict:
    """Decode CAN ID 0x0D259CD0 (8 bytes, Intel byte order)."""
    voltage_raw = data[0] | (data[1] << 8)
    voltage = voltage_raw * 0.1                          # V

    current_raw = data[2] | (data[3] << 8)
    current = current_raw*0.1 - 1000                # A

    shaft_raw = data[4] | (data[5] << 8)
    shaft_rpm = shaft_raw - 10000                        # rpm

    speed_kmh = abs(shaft_rpm) / SPEED_DIVISOR

    flags = data[6]
    gear = "D" if (flags & 0x01) else ("R" if (flags & 0x02) else "N")

    fault_code = data[7]

    return {
        "voltage":       voltage,
        "current":       current,
        "speed_kmh":     speed_kmh,
        "shaft_rpm":     shaft_rpm,
        "gear":          gear,
        "throttle_conf": not bool(flags & 0x04),
        "brake_signal":  bool(flags & 0x08),
        "brake_conf":    bool(flags & 0x10),
        "cruise_ctrl":   bool(flags & 0x20),
        "handbrake":     bool(flags & 0x40),
        "fault_code":    fault_code,
        "fault_desc":    FAULT_CODES.get(fault_code, f"Reserved ({fault_code})"),
    }


# ── CAN Reader Thread ───────────────────────────────────────────────────────

class CanReaderThread(threading.Thread):
    def __init__(self, data_queue: queue.Queue, interface: str,
                 channel: str, bitrate: int):
        super().__init__(daemon=True)
        self.queue     = data_queue
        self.interface = interface
        self.channel   = channel
        self.bitrate   = bitrate
        self.running   = True

    def run(self):
        self._run_can()

    def _run_can(self):
        try:
            bus = can.interface.Bus(
                interface=self.interface,
                channel=self.channel,
                bitrate=self.bitrate,
            )
        except Exception as exc:
            self.queue.put({"type": "status", "msg": f"Connect error: {exc}"})
            return

        self.queue.put({"type": "status", "msg": f"Connected  ({self.channel})"})

        while self.running:
            try:
                msg = bus.recv(timeout=0.1)
                if msg is None:
                    continue
                if msg.arbitration_id == MSG_INSTRUMENT and len(msg.data) >= 8:
                    self.queue.put({
                        "type": "instrument",
                        "data": decode_instrument(msg.data),
                        "raw":    bytes(msg.data),
                        "can_id": msg.arbitration_id,
                    })
                elif msg.arbitration_id == MSG_FAULT_DISP and len(msg.data) >= 8:
                    self.queue.put({
                        "type": "fault_display",
                        "data": decode_fault_display(msg.data),
                        "raw":    bytes(msg.data),
                        "can_id": msg.arbitration_id,
                    })
                else:
                    self.queue.put({
                        "type": "raw_frame",
                        "raw":    bytes(msg.data),
                        "can_id": msg.arbitration_id,
                    })
            except Exception as exc:
                self.queue.put({"type": "status", "msg": f"RX error: {exc}"})
                break

        try:
            bus.shutdown()
        except Exception:
            pass

    def stop(self):
        self.running = False


# ── UI Widgets ──────────────────────────────────────────────────────────────

class MetricCard(ctk.CTkFrame):
    """Large-value display card."""

    def __init__(self, parent, title: str, unit: str = "", font_size: int = 28, **kwargs):
        super().__init__(parent, fg_color=C_CARD, border_color=C_BORDER,
                         border_width=1, corner_radius=6, **kwargs)

        ctk.CTkLabel(self, text=title, fg_color="transparent", text_color=C_MUTED,
                     font=("Helvetica", 10, "bold")).pack(pady=(6, 0))

        self._val = tk.StringVar(value="—")
        self._val_lbl = ctk.CTkLabel(self, textvariable=self._val, fg_color="transparent",
                                     text_color=C_VALUE, font=("Helvetica", font_size, "bold"))
        self._val_lbl.pack(padx=12)

        ctk.CTkLabel(self, text=unit, fg_color="transparent", text_color=C_ACCENT,
                     font=("Helvetica", 10)).pack(pady=(0, 6))

    def set(self, text: str, color: str = C_VALUE):
        self._val.set(text)
        self._val_lbl.configure(text_color=color)


class FaultCard(ctk.CTkFrame):
    """Fault code card with description."""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, fg_color=C_CARD, border_color=C_BORDER,
                         border_width=1, corner_radius=6, **kwargs)

        ctk.CTkLabel(self, text="FAULT", fg_color="transparent", text_color=C_MUTED,
                     font=("Helvetica", 10, "bold")).pack(pady=(6, 0))

        self._code = tk.StringVar(value="—")
        self._code_lbl = ctk.CTkLabel(self, textvariable=self._code, fg_color="transparent",
                                      text_color=C_VALUE, font=("Helvetica", 36, "bold"))
        self._code_lbl.pack(padx=16)

        self._desc = tk.StringVar(value="")
        ctk.CTkLabel(self, textvariable=self._desc, fg_color="transparent", text_color=C_MUTED,
                     font=("Helvetica", 9), wraplength=130).pack(pady=(0, 6))

    def set(self, code: int, desc: str):
        if code == 0:
            self._code.set("OK")
            self._code_lbl.configure(text_color=C_OK)
        else:
            self._code.set(str(code))
            self._code_lbl.configure(text_color=C_ERR)
        self._desc.set(desc)


class SignalIndicator(ctk.CTkFrame):
    """ON/OFF signal indicator card."""

    def __init__(self, parent, title: str, compact: bool = False, **kwargs):
        super().__init__(parent, fg_color=C_CARD, border_color=C_BORDER,
                         border_width=1, corner_radius=6, **kwargs)
        if compact:
            self._dot = ctk.CTkLabel(self, text="●", fg_color="transparent",
                                     text_color=C_OFF, font=("Helvetica", 13))
            self._dot.pack(side=tk.LEFT, padx=(10, 6), pady=6)
            ctk.CTkLabel(self, text=title, fg_color="transparent", text_color=C_MUTED,
                         font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, fill=tk.X, expand=True)
            self._state = tk.StringVar(value="OFF")
            self._state_lbl = ctk.CTkLabel(self, textvariable=self._state,
                                           fg_color="transparent", text_color=C_OFF,
                                           font=("Helvetica", 9, "bold"))
            self._state_lbl.pack(side=tk.RIGHT, padx=10)
        else:
            ctk.CTkLabel(self, text=title, fg_color="transparent", text_color=C_MUTED,
                         font=("Helvetica", 9, "bold"), justify="center",
                         wraplength=110).pack(pady=(12, 6), padx=8)
            self._dot = ctk.CTkLabel(self, text="●", fg_color="transparent",
                                     text_color=C_OFF, font=("Helvetica", 28))
            self._dot.pack()
            self._state = tk.StringVar(value="OFF")
            self._state_lbl = ctk.CTkLabel(self, textvariable=self._state,
                                           fg_color="transparent", text_color=C_OFF,
                                           font=("Helvetica", 10, "bold"))
            self._state_lbl.pack(pady=(2, 12))

    def set_active(self, active: bool):
        color = C_OK if active else C_OFF
        text  = "ON"  if active else "OFF"
        self._dot.configure(text_color=color)
        self._state_lbl.configure(text_color=color)
        self._state.set(text)


# ── Main Application ────────────────────────────────────────────────────────

class App(ctk.CTk):
    def __init__(self, interface: str, channel: str, bitrate: int):
        super().__init__()

        self.title("KTZ12X40030")
        self.configure(fg_color=C_BG)
        self.minsize(864, 608)

        self._queue         = queue.Queue()
        self._raw_counts    = {}
        self._raw_last_time = {}
        self._bitrate       = bitrate
        self._init_channel  = channel
        self._init_iface    = interface
        self._reader        = None
        self._build_ui()
        self._do_connect()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll()

    # ── UI construction ──────────────────────────────────────────────────

    def _build_ui(self):
        # ── Header ──
        hdr = tk.Frame(self, bg=C_BG)
        hdr.pack(fill=tk.X, padx=20, pady=(10, 2))
        ctk.CTkLabel(hdr, text="KTZ12X40030", fg_color="transparent", text_color=C_ACCENT,
                     font=("Helvetica", 18, "bold")).pack(side=tk.LEFT)
        self._status_var = tk.StringVar(value="Connecting…")
        ctk.CTkLabel(hdr, textvariable=self._status_var, fg_color="transparent",
                     text_color=C_MUTED, font=("Helvetica", 10)).pack(side=tk.RIGHT, padx=4)

        # ── Connection bar ──
        conn = tk.Frame(self, bg=C_BG)
        conn.pack(fill=tk.X, padx=20, pady=(2, 6))
        ctk.CTkLabel(conn, text="DEVICE", fg_color="transparent", text_color=C_MUTED,
                     font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        default_device = next(
            (k for k, v in DEVICES.items() if v == self._init_iface),
            list(DEVICES.keys())[0])
        self._device_var = tk.StringVar(value=default_device)
        device_cb = ctk.CTkComboBox(conn, variable=self._device_var,
                                    values=list(DEVICES.keys()), state="readonly", width=200,
                                    fg_color=C_CARD, border_color=C_BORDER,
                                    button_color=C_BORDER, button_hover_color=C_ACCENT,
                                    dropdown_fg_color=C_CARD, dropdown_hover_color=C_BORDER,
                                    text_color=C_FG, dropdown_text_color=C_FG,
                                    command=lambda val: self._on_device_change())
        device_cb.pack(side=tk.LEFT, padx=(0, 14))

        ctk.CTkLabel(conn, text="CHANNEL", fg_color="transparent", text_color=C_MUTED,
                     font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        init_iface = DEVICES.get(default_device, "slcan")
        init_ports = _scan_ports(init_iface)
        init_ch    = "0" if init_iface == "canalystii" else self._init_channel
        self._channel_var = tk.StringVar(value=init_ch)
        self._channel_cb = ctk.CTkComboBox(conn, variable=self._channel_var,
                                           values=init_ports, width=200,
                                           fg_color=C_CARD, border_color=C_BORDER,
                                           button_color=C_BORDER, button_hover_color=C_ACCENT,
                                           dropdown_fg_color=C_CARD, dropdown_hover_color=C_BORDER,
                                           text_color=C_FG, dropdown_text_color=C_FG)
        self._channel_cb.pack(side=tk.LEFT, padx=(0, 4))

        ctk.CTkButton(conn, text="↺", command=self._refresh_ports,
                      fg_color=C_CARD, text_color=C_ACCENT, hover_color=C_BORDER,
                      font=("Helvetica", 11), width=32, height=28, corner_radius=4
                      ).pack(side=tk.LEFT, padx=(0, 10))
        ctk.CTkButton(conn, text="Connect", command=self._do_connect,
                      fg_color=C_ACCENT, text_color=C_BG, hover_color=C_VALUE,
                      font=("Helvetica", 9, "bold"), width=80, height=28, corner_radius=4
                      ).pack(side=tk.LEFT, padx=(0, 6))
        ctk.CTkButton(conn, text="Disconnect", command=self._do_disconnect,
                      fg_color=C_ERR, text_color=C_BG, hover_color=C_WARN,
                      font=("Helvetica", 9, "bold"), width=95, height=28, corner_radius=4
                      ).pack(side=tk.LEFT)

        self._sep()

        # ── Drive Metrics ──
        m_frame = tk.Frame(self, bg=C_BG)
        m_frame.pack(fill=tk.X, padx=20, pady=(6, 6))
        self._section_label_in(m_frame, "DATA  ·  0x0D259CD0")
        m_row = tk.Frame(m_frame, bg=C_BG)
        m_row.pack(fill=tk.X)
        m_row.columnconfigure((0, 1, 2, 3), weight=3, uniform="col")
        m_row.columnconfigure(4, weight=2, uniform="col")
        m_row.columnconfigure(5, weight=3, uniform="col")
        self._volt_card  = MetricCard(m_row, "VOLTAGE",   "V")
        self._curr_card  = MetricCard(m_row, "CURRENT",   "A")
        self._speed_card = MetricCard(m_row, "SPEED",     "km/h")
        self._rpm_card   = MetricCard(m_row, "SHAFT RPM", "rpm")
        self._gear_card  = MetricCard(m_row, "GEAR", font_size=52)
        self._fault_card = FaultCard(m_row)
        self._volt_card .grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        self._curr_card .grid(row=0, column=1, sticky="nsew", padx=4)
        self._speed_card.grid(row=0, column=2, sticky="nsew", padx=4)
        self._rpm_card  .grid(row=0, column=3, sticky="nsew", padx=4)
        self._gear_card .grid(row=0, column=4, sticky="nsew", padx=4)
        self._fault_card.grid(row=0, column=5, sticky="nsew", padx=(4, 0))

        self._sep()

        # ── Middle: Sensor Data (left) + Signal Status (right) ──
        mid = tk.Frame(self, bg=C_BG)
        mid.pack(fill=tk.X, padx=20, pady=(6, 6))
        mid.columnconfigure(0, weight=6)
        mid.columnconfigure(1, weight=4)

        # Left — Sensor Data
        s_panel = tk.Frame(mid, bg=C_BG)
        s_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self._section_label_in(s_panel, "SENSOR DATA  ·  0x0D259CE7")
        s_row1 = tk.Frame(s_panel, bg=C_BG)
        s_row1.pack(fill=tk.X, pady=(0, 2))
        s_row1.columnconfigure((0, 1, 2, 3), weight=1, uniform="s1")
        s_row2 = tk.Frame(s_panel, bg=C_BG)
        s_row2.pack(fill=tk.X)
        s_row2.columnconfigure((0, 1, 2, 3), weight=1, uniform="s2")
        FS = 22
        self._motor_temp_card = MetricCard(s_row1, "MOTOR TEMP",    "°C", FS)
        self._mcu_temp_card   = MetricCard(s_row1, "MCU TEMP",      "°C", FS)
        self._throttle_card   = MetricCard(s_row1, "THROTTLE",      "",   FS)
        self._mag_card        = MetricCard(s_row1, "MAGNETIC CODE", "",   FS)
        self._gear_pos_card   = MetricCard(s_row2, "GEAR POSITION", "",   FS)
        self._v12_card        = MetricCard(s_row2, "12V",           "V",  FS)
        self._v5_card         = MetricCard(s_row2, "5V",            "V",  FS)
        self._v4096_card      = MetricCard(s_row2, "4.096V REF",    "V",  FS)
        self._motor_temp_card.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        self._mcu_temp_card  .grid(row=0, column=1, sticky="nsew", padx=4)
        self._throttle_card  .grid(row=0, column=2, sticky="nsew", padx=4)
        self._mag_card       .grid(row=0, column=3, sticky="nsew", padx=(4, 0))
        self._gear_pos_card  .grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        self._v12_card       .grid(row=0, column=1, sticky="nsew", padx=4)
        self._v5_card        .grid(row=0, column=2, sticky="nsew", padx=4)
        self._v4096_card     .grid(row=0, column=3, sticky="nsew", padx=(4, 0))

        # Right — Signal Status
        sig_panel = tk.Frame(mid, bg=C_BG)
        sig_panel.grid(row=0, column=1, sticky="nsew")
        self._section_label_in(sig_panel, "SIGNAL STATUS")
        sig_grid = tk.Frame(sig_panel, bg=C_BG)
        sig_grid.pack(fill=tk.X)
        self._throttle_ind = SignalIndicator(sig_grid, "THROTTLE CONFIRM", compact=True)
        self._brake_ind    = SignalIndicator(sig_grid, "BRAKE SIGNAL",     compact=True)
        self._brake_c_ind  = SignalIndicator(sig_grid, "BRAKE CONFIRM",    compact=True)
        self._cruise_ind   = SignalIndicator(sig_grid, "CRUISE CONTROL",   compact=True)
        self._hbrake_ind   = SignalIndicator(sig_grid, "HAND BRAKE",       compact=True)
        for ind in (self._throttle_ind, self._brake_ind, self._brake_c_ind,
                    self._cruise_ind, self._hbrake_ind):
            ind.pack(fill=tk.X, pady=(0, 2))

        self._sep()

        # ── Raw CAN Frames ──
        raw_frame = tk.Frame(self, bg=C_BG)
        raw_frame.pack(fill=tk.X, padx=20, pady=(6, 4))
        self._section_label_in(raw_frame, "RAW CAN FRAMES")

        style = ttk.Style()
        style.theme_use("default")
        style.configure("Raw.Treeview",
                        background=C_CARD, foreground=C_FG,
                        fieldbackground=C_CARD, borderwidth=0,
                        rowheight=26, font=("Courier", 10))
        style.configure("Raw.Treeview.Heading",
                        background=C_BORDER, foreground=C_MUTED,
                        font=("Helvetica", 9, "bold"), borderwidth=0)
        style.map("Raw.Treeview", background=[("selected", C_BORDER)])
        tree_frame = tk.Frame(raw_frame, bg=C_BG)
        tree_frame.pack(fill=tk.X)
        self._raw_tree = ttk.Treeview(tree_frame,
                                      columns=("frame_id", "data_hex", "count", "cycle_ms"),
                                      show="headings", height=5, style="Raw.Treeview")
        self._raw_tree.heading("frame_id", text="Frame ID")
        self._raw_tree.heading("data_hex", text="Data (HEX)")
        self._raw_tree.heading("count",    text="Count")
        self._raw_tree.heading("cycle_ms", text="Cycle Time (ms)")
        self._raw_tree.column("frame_id", width=150, anchor=tk.CENTER)
        self._raw_tree.column("data_hex", width=300, anchor=tk.CENTER)
        self._raw_tree.column("count",    width=70,  anchor=tk.CENTER)
        self._raw_tree.column("cycle_ms", width=120, anchor=tk.CENTER)
        vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self._raw_tree.yview)
        self._raw_tree.configure(yscrollcommand=vsb.set)
        self._raw_tree.pack(side=tk.LEFT, fill=tk.X, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._raw_tree.insert("", tk.END, iid="cd0",
                              values=(f"0x{MSG_INSTRUMENT:08X}", "—", "—", "—"))
        self._raw_tree.insert("", tk.END, iid="ce7",
                              values=(f"0x{MSG_FAULT_DISP:08X}", "—", "—", "—"))

        self._sep()

        # ── Footer ──
        footer = tk.Frame(self, bg=C_BG)
        footer.pack(fill=tk.X, padx=20, pady=(4, 8))
        ctk.CTkLabel(footer,
                     text="EC KTZ12X40030  |  250 Kbps  |  0x0D259CD0  ·  0x0D259CE7  |  Intel byte order",
                     fg_color="transparent", text_color=C_BORDER,
                     font=("Helvetica", 9)).pack(side=tk.LEFT)

    def _sep(self):
        tk.Frame(self, bg=C_BORDER, height=2).pack(fill=tk.X, padx=20, pady=(2, 2))

    def _section_label_in(self, parent, text):
        row = tk.Frame(parent, bg=C_BG)
        row.pack(fill=tk.X, pady=(0, 4))
        tk.Frame(row, bg=C_ACCENT, width=3).pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        ctk.CTkLabel(row, text=text, fg_color="transparent", text_color=C_MUTED,
                     font=("Helvetica", 9)).pack(side=tk.LEFT)

    # ── Event loop bridge ────────────────────────────────────────────────

    def _poll(self):
        try:
            while True:
                item = self._queue.get_nowait()
                if item["type"] == "status":
                    self._status_var.set(item["msg"])
                elif item["type"] == "instrument":
                    self._update(item["data"])
                    self._update_raw(item["can_id"], item["raw"])
                elif item["type"] == "fault_display":
                    self._update_sensor(item["data"])
                    self._update_raw(item["can_id"], item["raw"])
                elif item["type"] == "raw_frame":
                    self._update_raw(item["can_id"], item["raw"])
        except queue.Empty:
            pass
        self.after(50, self._poll)

    def _update(self, d: dict):
        ts = time.strftime("%H:%M:%S")
        self._status_var.set(f"● RX  {ts}")

        volt_color = C_WARN if d["voltage"] > 600 or d["voltage"] < 300 else C_VALUE
        self._volt_card.set(f"{d['voltage']:.1f}", volt_color)

        curr_color = C_WARN if abs(d["current"]) > 300 else C_VALUE
        self._curr_card.set(f"{d['current']:.1f}", curr_color)

        self._speed_card.set(f"{d['speed_kmh']:.1f}")
        self._rpm_card.set(f"{d['shaft_rpm']}")

        gear_colors = {"D": C_OK, "R": C_WARN, "N": C_VALUE}
        self._gear_card.set(d["gear"], gear_colors.get(d["gear"], C_VALUE))

        self._fault_card.set(d["fault_code"], d["fault_desc"])

        self._throttle_ind.set_active(d["throttle_conf"])
        self._brake_ind   .set_active(d["brake_signal"])
        self._brake_c_ind .set_active(d["brake_conf"])
        self._cruise_ind  .set_active(d["cruise_ctrl"])
        self._hbrake_ind  .set_active(d["handbrake"])

    def _on_device_change(self):
        iface = DEVICES.get(self._device_var.get(), "slcan")
        ports = _scan_ports(iface)
        self._channel_cb.configure(values=ports)
        self._channel_var.set(ports[0])

    def _refresh_ports(self):
        iface = DEVICES.get(self._device_var.get(), "slcan")
        ports = _scan_ports(iface)
        self._channel_cb.configure(values=ports)
        if self._channel_var.get() not in ports:
            self._channel_var.set(ports[0])

    def _do_connect(self):
        if self._reader:
            self._reader.stop()
            self._reader.join(timeout=0.5)
        iface   = DEVICES.get(self._device_var.get(), "slcan")
        channel = self._channel_var.get()
        if iface == "canalystii":
            channel = int(channel)
        self._reader = CanReaderThread(self._queue, iface, channel, self._bitrate)
        self._reader.start()

    def _do_disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._status_var.set("Disconnected")

    def _update_sensor(self, d: dict):
        def temp_color(v):
            return C_ERR if v > 100 else (C_WARN if v > 80 else C_VALUE)

        self._motor_temp_card.set(f"{d['motor_temp']:.0f}", temp_color(d["motor_temp"]))
        self._mcu_temp_card  .set(f"{d['mcu_temp']:.0f}",   temp_color(d["mcu_temp"]))
        self._throttle_card  .set(f"{d['throttle']:.0f}")
        self._mag_card       .set(f"{d['mag_code']}")
        self._gear_pos_card  .set(f"{d['gear_pos']:.0f}")
        self._v12_card       .set(f"{d['v12']:.3f}")
        self._v5_card        .set(f"{d['v5']:.3f}")
        self._v4096_card     .set(f"{d['v4096']:.4f}")

    def _update_raw(self, can_id: int, raw: bytes):
        now     = time.monotonic()
        hex_str = " ".join(f"{b:02X}" for b in raw)
        count   = self._raw_counts.get(can_id, 0) + 1
        self._raw_counts[can_id] = count
        last    = self._raw_last_time.get(can_id)
        cycle   = f"{(now - last) * 1000:.1f}" if last is not None else "—"
        self._raw_last_time[can_id] = now
        id_str  = f"0x{can_id:08X}"
        iid     = "cd0" if can_id == MSG_INSTRUMENT else \
                  "ce7" if can_id == MSG_FAULT_DISP  else None
        if iid:
            self._raw_tree.item(iid, values=(id_str, hex_str, count, cycle))
        else:
            for item in self._raw_tree.get_children():
                if self._raw_tree.item(item, "values")[0] == id_str:
                    self._raw_tree.item(item, values=(id_str, hex_str, count, cycle))
                    return
            self._raw_tree.insert("", tk.END, values=(id_str, hex_str, count, cycle))

    def _on_close(self):
        if self._reader:
            self._reader.stop()
            self._reader.join(timeout=1.0)
        self.destroy()


# ── Entry Point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="KTZ Motor Controller CAN Monitor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py --interface slcan --channel /dev/tty.usbserial-0001
  python main.py --interface socketcan --channel can0          (Linux)
        """,
    )
    parser.add_argument("--interface", default="canalystii",
                        help="python-can interface type (default: canalystii)")
    parser.add_argument("--channel", default="/dev/tty.usbserial-0001",
                        help="CAN device channel / port (default: /dev/tty.usbserial-0001)")
    parser.add_argument("--bitrate", type=int, default=250000,
                        help="CAN bitrate in bps (default: 250000)")
    args = parser.parse_args()

    if sys.platform == "win32":
        try:
            from ctypes import windll
            windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass

    if not CAN_AVAILABLE:
        print("python-can is not installed.")
        print("  Install with:  pip install python-can")
        return

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    try:
        App(args.interface, args.channel, args.bitrate).mainloop()
    except Exception:
        import traceback, tempfile, datetime
        _log = pathlib.Path(tempfile.gettempdir()) / "KTZ12X40030_crash.log"
        with open(_log, "w") as _f:
            _f.write(f"{datetime.datetime.now()}\n")
            traceback.print_exc(file=_f)
        raise


if __name__ == "__main__":
    main()
