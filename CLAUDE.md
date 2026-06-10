# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Running the app

```bash
pip install python-can customtkinter canalystii libusb   # one-time setup (Windows)

python main.py
python main.py --interface slcan --channel /dev/tty.usbserial-XXXX --bitrate 250000
python main.py --interface socketcan --channel can0    # Linux SocketCAN
```

Find the USB-CAN-B serial port on macOS: `ls /dev/tty.usb* /dev/tty.wch*`

> **Note:** python-can must be installed — the app will print an error and exit if it is missing. There is no demo/simulation mode.

## Windows setup for Waveshare USB-CAN-B (one-time per PC)

The device uses `VID=0x04D8 PID=0x0053` (Microchip, identified as "Chuangxin Tech USBCAN/CANalyst-II").

1. **Install WinUSB driver via Zadig** (zadig.akeo.ie):
   - Options → List All Devices
   - Select the USB-CAN device → set driver to **WinUSB** → Install Driver
2. **Install Python packages:**
   ```
   pip install canalystii libusb
   ```
   - `canalystii` — python-can backend for this device
   - `libusb` — bundles `libusb-1.0.dll`; PyUSB needs this to access WinUSB devices (searches via `PATH`, not system DLL search path)

`main.py` prepends the `libusb` package's DLL directory to `PATH` at startup so PyUSB can find it. The built exe bundles `libusb-1.0.dll` directly (via `KTZ12X40030.spec` `binaries`).

If you see **"Connect error: [Errno 13] Access denied"**, another process has the device open — close it and retry.

## Architecture

Single file (`main.py`) with four layers:

1. **Decoder** — two pure functions at module level:
   - `decode_instrument(data)` → parses CAN ID `0x0D259CD0` (voltage, current, shaft RPM, gear flags, fault code)
   - `decode_fault_display(data)` → parses CAN ID `0x0D259CE7` (motor temp, MCU temp, throttle analog, 12 V rail, gear position, magnetic code, voltage refs)

2. **`CanReaderThread`** — daemon thread that pushes dicts onto a `queue.Queue`. Message types:
   - `"instrument"` — decoded data from `0x0D259CD0`
   - `"fault_display"` — decoded data from `0x0D259CE7`
   - `"raw_frame"` — any other CAN ID (raw bytes only, no decoding)
   - `"status"` — connection status string
   Runs `_run_can()` only (no demo mode). Stops cleanly via `stop()`.

3. **Widgets** — three `customtkinter.CTkFrame` subclasses: `MetricCard` (large value + unit), `FaultCard` (code + description text), `SignalIndicator` (coloured dot ON/OFF). Layout-only frames inside `App` use plain `tk.Frame` to avoid CTkFrame minimum-height side effects.

4. **`App(ctk.CTk)`** — builds the layout, starts the thread on launch, polls the queue every 50 ms with `after()`. All widget updates happen on the main thread via `_update()`. The connection bar has three controls: **Connect** (starts/restarts thread), **Disconnect** (stops thread, clears status), and **↺** (refreshes port list).

## Top row layout (0x0D259CD0)

Six `MetricCard` widgets in a single grid row:

| Column | Label | Unit | Value |
|--------|-------|------|-------|
| 0 | VOLTAGE | V | `voltage` |
| 1 | CURRENT | A | `current` |
| 2 | SPEED | km/h | `speed_kmh` = `\|shaft_rpm\| / 86.975` |
| 3 | SHAFT RPM | rpm | `shaft_rpm` (signed) |
| 4 | GEAR | — | D / R / N |
| 5 | FAULT | — | code + description |

## UI notes

- **Framework:** `customtkinter` (dark mode), with `ttk.Treeview` for the raw CAN frames table (no CTk equivalent).
- **Window size:** 864 × 608 px minimum (fits 1366 × 768 at 125% DPI scaling on Windows).
- **Windows DPI:** `SetProcessDpiAwareness(1)` is called at startup on Win32 so the app renders at physical pixels without Windows blurring it.
- **Raw CAN Frames table** — rows are ordered:
  1. `0x0D259CD0` (pinned, `iid="cd0"`)
  2. `0x0D259CE7` (pinned, `iid="ce7"`)
  3+ Any other IDs received, appended in order of first appearance, updated in-place on subsequent messages.

## Building for Windows

```bat
build_windows.bat
```

Output: `dist\KTZ12X40030.exe` (single-file, no console window). Requires Python 3.10+, PyInstaller, and all packages in `requirements.txt` plus `canalystii` and `libusb`.

The spec (`KTZ12X40030.spec`) auto-detects and bundles `libusb-1.0.dll` from the `libusb` PyPI package so the exe works on machines that have only had the Zadig WinUSB driver installed (no extra Python packages needed).

Crash logs (unhandled exceptions at startup) are written to `%TEMP%\KTZ12X40030_crash.log`.

## Device presets

`DEVICES` dict (top of `main.py`) maps UI dropdown labels to python-can interface strings:

| Label | Interface |
|-------|-----------|
| Waveshare USB-CAN-B | `canalystii` |
| Robotell USB-CAN | `robotell` |
| SLCAN (Generic) | `slcan` |

For `canalystii`, `_scan_ports()` returns `["0", "1"]` and the channel is cast to `int` before passing to python-can. For all other interfaces it scans serial ports.

## Fault codes dict

`FAULT_CODES` (module-level dict) maps byte 7 of `0x0D259CD0` to a description string. Code `0` = "No Fault"; codes 1–8 cover specific subsystem failures. Unknown codes fall back to `f"Reserved ({fault_code})"` in `decode_instrument()`.

## Protocol reference (EC KTZ12X40030)

- **Baud rate:** 250 Kbps, **byte order:** Intel (little-endian), **frame type:** extended 29-bit
- **CAN ID `0x0D259CD0`** (Instrument display, 100 ms):
  - Bytes 0-1: bus voltage → `raw × 0.1` V
  - Bytes 2-3: bus current → `(raw − 1000) × 0.1` A
  - Bytes 4-5: shaft RPM → `raw − 10000` rpm
  - Byte 6 bit 0: gear_fwd → "D"
  - Byte 6 bit 1: gear_rev → "R" (N ถ้าไม่มีทั้งคู่)
  - Byte 6 bit 2: throttle_confirm (active-low)
  - Byte 6 bit 3: brake_signal
  - Byte 6 bit 4: brake_confirm
  - Byte 6 bit 5: cruise_ctrl
  - Byte 6 bit 6: handbrake
  - Byte 7: fault code (0 = no fault, see `FAULT_CODES` dict)
- **Speed formula:** `|shaft_rpm| / 86.975` km/h
- **CAN ID `0x0D259CE7`** (Fault display, 100 ms):
  - Byte 0: 12V rail → `raw × 5 × 0.024` V
  - Byte 1: throttle analog → `raw × 6`
  - Byte 2: gear position → `raw × 5`
  - Byte 3: magnetic code → `raw × 128` (raw 0–127)
  - Byte 4: motor temp → `(raw − 128) × 2` °C
  - Byte 5: MCU temp → `(raw − 128) × 2` °C
  - Byte 6: 4.096V ref → `raw × 5 × 0.012011` V
  - Byte 7: 5V supply → `raw × 2 × 0.012` V
