# Peeling Thin Sheet Using Stepper Motor — ESP32 Controller

Firmware for an automated thin-sheet peeling machine driven by a NEMA 17 stepper motor and controlled via a Wemos D1 R32 (ESP32).

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Configuration](#configuration)
- [Serial Command Interface](#serial-command-interface)
- [Serial Output](#serial-output)
- [State Machine](#state-machine)

---

## Overview

This firmware:
- Controls a NEMA 17 stepper motor through a DM542T driver to drive a peeling stage.
- Drives a 240×320 ST7789 TFT display (landscape) with a live status UI and settings menu.
- Supports 4 physical buttons for UI navigation, start/stop, homing, and calibration.
- Reads two independent limit switches (home end and far end).
- Reads temperature via a MAX31856 thermocouple amplifier (K-type) and drives a heater MOSFET with a PID controller.
- Connects to an existing Wi-Fi network (station mode) and hosts a WebSocket-based status/control UI on port 80.
- Exposes a serial command interface (115 200 baud) for external software control via a USB-serial bridge.
- Emits a JSON heartbeat every 100 ms so a host application can monitor position, state, and temperature in real time.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Wemos D1 R32 (ESP32 dual-core, 240 MHz) |
| Stepper motor | NEMA 17 — rated 0.4 A |
| Stepper driver | DM542T (V4.0) — Leadshine digital driver, active-low ENA- |
| Display | 240×320 ST7789 TFT (VSPI, landscape rotation) |
| Limit switches | 2× microswitch — home end (X) and far end (Y) |
| Buttons | 4× push-button — A (start/stop), B (settings/home), X (increment/CAL), Y (decrement) |
| Thermocouple amplifier | Adafruit MAX31856 — K-type thermocouple, VSPI bus |
| Heater MOSFET | N-channel FET driven by 1 kHz PWM (LEDC) on GPIO 32 |

### Driver Configuration (DIP Switches)

| Switches | Function | Recommendation |
|---|---|---|
| SW1–SW3 | Peak output current | Match to motor rated current |
| SW4–SW6 | Microstep resolution | Match to firmware steps-per-rev setting (default 25600 steps/rev) |
| SW7 | Standby current reduction | ON = 50 % current at rest |
| SW8 | Pulse active edge | ON = rising edge (default) |

Refer to the DM542T datasheet for the exact SW1–SW3 current table.

---

## Wiring

### Stepper Driver

| ESP32 GPIO | Signal | Connected To |
|---|---|---|
| GPIO 26 | ENA+ | DM542T ENA+ (ENA- tied to GND) |
| GPIO 25 | DIR+ | DM542T DIR+ (DIR- tied to GND) |
| GPIO 27 | PUL+ | DM542T PUL+ (PUL- tied to GND) |

> The DM542T minimum HIGH is 3.5 V; ESP32 GPIO outputs 3.3 V — use a level shifter or voltage divider on step/dir/enable lines if the driver does not trigger reliably.

### Display (ST7789, VSPI)

| ESP32 GPIO | Signal |
|---|---|
| GPIO 18 | SPI SCK  (shared VSPI bus) |
| GPIO 23 | SPI MOSI (shared VSPI bus) |
| GPIO 19 | SPI MISO (shared VSPI bus) |
| GPIO  5 | TFT CS |
| GPIO 17 | TFT DC |
| GPIO  2 | TFT RST |
| GPIO 16 | TFT backlight |

### Thermocouple Amplifier (MAX31856, shares VSPI)

| ESP32 GPIO | Signal |
|---|---|
| GPIO 21 | MAX31856 CS |
| GPIO 22 | MAX31856 DRDY (data-ready, active-low) |

### Heater MOSFET

| ESP32 GPIO | Signal |
|---|---|
| GPIO 32 | Gate (1 kHz PWM via LEDC, 8-bit duty) |

### Buttons (active-low, internal pull-up)

| ESP32 GPIO | Button | Function |
|---|---|---|
| GPIO  4 | BTN_A | Start / stop; in settings: navigate up |
| GPIO 33 | BTN_B | Settings / home; in settings: navigate down / exit+save |
| GPIO 14 | BTN_X | In settings: increment (+) or trigger CAL |
| GPIO 12 | BTN_Y | In settings: decrement (−) |

### Limit Switches (active-low, internal pull-up)

| ESP32 GPIO | Switch | Triggers |
|---|---|---|
| GPIO 13 | Home / X end | HOMING → stop and zero position |
| GPIO 15 | Far / Y end  | CAL_RUNNING → record travel distance; PEELING/MOVING → safety abort |

---

## Software Dependencies

Install via the Arduino Library Manager before compiling:

| Library | Notes |
|---|---|
| **FastAccelStepper** | Hardware-timer step generation |
| **Adafruit ST7789** | TFT display driver |
| **Adafruit GFX** | Graphics primitives |
| **Adafruit MAX31856** | Thermocouple amplifier driver |
| **ESP Async WebServer** ≥ 3.3.x | Use the mathieucarbou fork — the me-no-dev fork has mbedtls API breakage with ESP32 core ≥ 3.x |
| **AsyncTCP** | Required by ESP Async WebServer |
| **ESPmDNS** | Bundled with the ESP32 core — no separate install needed |

---

## Installation

```bash
# Install ESP32 core (once)
arduino-cli core install esp32:esp32

# Compile
arduino-cli compile --fqbn esp32:esp32:d1_mini32 Peeling_Automation_Stepper_esp32/

# Upload (replace /dev/ttyUSB0 with the actual port)
# UploadSpeed=460800 is required — the CH340 on the D1 R32 drops at the default 921600
arduino-cli upload --fqbn esp32:esp32:d1_mini32:UploadSpeed=460800 --port /dev/ttyUSB0 Peeling_Automation_Stepper_esp32/

# Monitor serial output
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
```

Add your Wi-Fi credentials in `Peeling_Automation_Stepper_esp32/wifi_credentials.h` before compiling.

### Serial Bridge (optional)

A Python serial bridge lets you control the instrument over USB without Wi-Fi:

```bash
cd serial_bridge
pip install -r requirements.txt
python serial_bridge.py          # auto-detect CH340
python serial_bridge.py /dev/ttyUSB0  # explicit port
```

Then open **http://localhost:8080** in any browser.

---

## Configuration

Key settings adjustable via the on-device settings menu (BTN_B → SETTINGS):

| Parameter | Description |
|---|---|
| Speed (µm/s) | Peel speed — adjustable at runtime via BTN_X/BTN_Y in settings |
| Angle (°) | Peeling arm half-angle θ — used for µm/step conversion |
| Start position (µm) | Distance from home to peel start point |
| CAL | Runs a full home→far-end traversal to measure travel distance |
| TEMP settings | PID setpoint, Kp/Ki/Kd, and controller on/off |
| Wait for temp | Hold peel at start position until temperature reaches setpoint |

Calibration (CAL) runs a full home→far-end traversal to measure travel distance automatically.

---

## Serial Command Interface

Connect at **115 200 baud**. Send a single character followed immediately by any numeric argument.

| Command | Argument | Description |
|---|---|---|
| `m` | `<int32>` | Move to absolute step position. Example: `m200000` |
| `s` | — | Stop motor immediately |
| `v` | `<int>` | Set motor max speed in Hz. Example: `v500` |
| `h` | `<0–255>` | Set heater duty cycle (blocked when PID control is active). Example: `h128` |
| `b` | `<A\|B\|X\|Y><1\|0>` | Virtual button press (1) or release (0). Example: `bA1` |

---

## Serial Output

Every **100 ms** the firmware emits a JSON heartbeat on Serial (and WebSocket). Key fields:

| Field | Description |
|---|---|
| `state` | State machine name: `"IDLE"`, `"MOVING"`, `"TO_START"`, `"WAIT_TEMP"`, `"PEELING"`, `"HOMING"`, `"SETTINGS"`, `"CAL_HOME"`, `"CAL_RUN"` |
| `position` | Current position in steps |
| `speed` | Current step rate (Hz) |
| `pos_um` | Current position in µm (peel distance) |
| `speed_um` | Current speed in µm/s |
| `speed_set` | Configured target speed in µm/s |
| `angle` | Configured arm half-angle (°) |
| `dist_xa_steps` | Calibrated X→A travel in steps (0 = not calibrated) |
| `dist_xa_um` | Calibrated X→A travel in µm |
| `peel_elapsed_ms` | Elapsed peel time in ms (non-zero during PEELING only) |
| `temp_c` | Thermocouple temperature in °C (`null` on fault) |
| `heater_duty` | Heater PWM duty 0–255 |
| `temp_setpoint` | Target temperature in °C |
| `temp_ctrl_active` | `true` when PID controller is running |
| `wait_for_temp` | `true` when peel waits for temperature to reach setpoint |
| `rssi` | Wi-Fi signal strength in dBm (0 = disconnected) |
| `clients` | Number of connected WebSocket clients |
| `ip` | Current Wi-Fi IP address string |

---

## State Machine

```
IDLE → [A, dist_xa > 0]           → MOVING_TO_START
IDLE → [A, dist_xa = 0]           → warning "RUN CAL FIRST" (stays IDLE)
IDLE → [B, pos = 0]               → SETTINGS
IDLE → [B, pos > 0]               → HOMING

MOVING_TO_START → [arrival + 100 ms, wait_for_temp = false] → PEELING
MOVING_TO_START → [arrival + 100 ms, wait_for_temp = true]  → WAITING_FOR_TEMP
WAITING_FOR_TEMP → [temp within ±2 °C of setpoint]          → PEELING

PEELING   → [motor reaches dist_xa or A or limit switch] → IDLE
HOMING    → [X limit switch]     → IDLE  (position := 0)
SETTINGS/CAL field → [X button]  → CAL_HOMING → CAL_RUNNING → IDLE (saves dist_xa)
Any moving state   → [A button]  → IDLE  (abort)

SETTINGS: A = navigate up, B = navigate down / exit+save, X = +/CAL/OPEN, Y = −
```

---

## Author

Lior Segev — Version 4.4.0-esp32, June 2026
