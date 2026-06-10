# KTZ12X40030 CAN Monitor

A desktop GUI application for real-time monitoring of the EC KTZ12X40030 motor controller via CAN bus.

## Features

- Live decoding of drive metrics: voltage, current, speed, shaft RPM, gear, fault code
- Sensor data panel: motor temp, MCU temp, throttle, gear position, magnetic code, voltage rails
- Signal status indicators: throttle confirm, brake signal/confirm, cruise control, handbrake
- Raw CAN frames table with cycle time measurement for all received IDs
- Supports Waveshare USB-CAN-B, Robotell USB-CAN, and any SLCAN-compatible adapter

## Requirements

- Python 3.10+
- Windows / macOS / Linux

## Installation

```bash
pip install python-can customtkinter canalystii libusb pyserial
```

## Running

```bash
python main.py
```

By default connects to the **Waveshare USB-CAN-B** (canalystii interface, channel 0) at 250 Kbps.

Override via CLI flags:

```bash
python main.py --interface slcan    --channel COM3                    # Windows SLCAN
python main.py --interface slcan    --channel /dev/tty.usbserial-XXXX # macOS SLCAN
python main.py --interface socketcan --channel can0                   # Linux SocketCAN
```

## Windows Setup (Waveshare USB-CAN-B)

The device uses `VID=0x04D8 PID=0x0053` and requires the WinUSB driver:

1. Download and run **Zadig** from [zadig.akeo.ie](https://zadig.akeo.ie)
2. Options → List All Devices
3. Select the USB-CAN device → set driver to **WinUSB** → Install Driver
4. Install Python packages:
   ```
   pip install canalystii libusb
   ```

> If you see **"Connect error: [Errno 13] Access denied"**, another application has the device open — close it and retry.

## Building a standalone Windows executable

```bat
build_windows.bat
```

Output: `dist\KTZ12X40030.exe` — single file, no console, no Python required on the target machine.

Crash logs are written to `%TEMP%\KTZ12X40030_crash.log` on unhandled startup exceptions.

## CAN Protocol Reference

| Parameter | CAN ID | Rate |
|-----------|--------|------|
| Instrument display (voltage, current, RPM, gear, fault) | `0x0D259CD0` | 100 ms |
| Fault/sensor display (temps, throttle, voltage rails) | `0x0D259CE7` | 100 ms |

- **Baud rate:** 250 Kbps
- **Byte order:** Intel (little-endian)
- **Frame type:** Extended 29-bit

### 0x0D259CD0 — Instrument display

| Bytes | Signal | Formula | Unit |
|-------|--------|---------|------|
| 0–1 | Bus voltage | `raw × 0.1` | V |
| 2–3 | Bus current | `(raw − 1000) × 0.1` | A |
| 4–5 | Shaft RPM | `raw − 10000` | rpm |
| 6[0] | Gear forward | bit set → D | — |
| 6[1] | Gear reverse | bit set → R | — |
| 6[2] | Throttle confirm | active-low | — |
| 6[3] | Brake signal | — | — |
| 6[4] | Brake confirm | — | — |
| 6[5] | Cruise control | — | — |
| 6[6] | Handbrake | — | — |
| 7 | Fault code | 0 = no fault | — |

Speed: `|shaft_rpm| / 86.975` km/h

### 0x0D259CE7 — Fault/sensor display

| Byte | Signal | Formula | Unit |
|------|--------|---------|------|
| 0 | 12 V rail | `raw × 5 × 0.024` | V |
| 1 | Throttle analog | `raw × 6` | — |
| 2 | Gear position | `raw × 5` | — |
| 3 | Magnetic code | `raw × 128` (0–127) | — |
| 4 | Motor temp | `(raw − 128) × 2` | °C |
| 5 | MCU temp | `(raw − 128) × 2` | °C |
| 6 | 4.096 V ref | `raw × 5 × 0.012011` | V |
| 7 | 5 V supply | `raw × 2 × 0.012` | V |

### Fault codes

| Code | Description |
|------|-------------|
| 0 | No Fault |
| 1 | TCU Failure |
| 2 | Gear Position Sensor Failure |
| 3 | Magnetic Encoder Failure |
| 4 | Shift Motor Failure |
| 5 | Accelerator Pedal Sensor Failure |
| 6 | Brake Sensor Failure |
| 7 | MCU Failure |
| 8 | Motor / MCU Overheating |
