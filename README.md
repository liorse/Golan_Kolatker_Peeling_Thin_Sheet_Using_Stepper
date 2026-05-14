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
- Drives a 240×240 ST7789 TFT display with a live status UI and settings menu.
- Supports 4 physical buttons for UI navigation, start/stop, homing, and calibration.
- Reads two independent limit switches (home end and far end).
- Hosts a Wi-Fi access point with a WebSocket-based status feed.
- Exposes a serial command interface (115 200 baud) for external software control.
- Emits a JSON heartbeat every 100 ms so a host application can monitor position and state in real time.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Wemos D1 R32 (ESP32 dual-core, 240 MHz) |
| Stepper motor | NEMA 17 — rated 0.4 A |
| Stepper driver | DM542T (V4.0) — Leadshine digital driver, active-low ENA- |
| Display | 240×240 ST7789 TFT (SPI) |
| Limit switches | 2× microswitch — home end (X) and far end (Y) |

### Driver Configuration (DIP Switches)

| Switches | Function | Recommendation |
|---|---|---|
| SW1–SW3 | Peak output current | Match to motor rated current |
| SW4–SW6 | Microstep resolution | Match to firmware steps-per-rev setting |
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
| GPIO 18 | SPI SCK |
| GPIO 23 | SPI MOSI |
| GPIO  5 | TFT CS |
| GPIO 17 | TFT DC |
| GPIO 16 | TFT backlight |

### Buttons (active-low, internal pull-up)

| ESP32 GPIO | Button | Function |
|---|---|---|
| GPIO  4 | BTN_A | Start / stop; in settings: navigate up |
| GPIO 19 | BTN_B | Settings / home; in settings: navigate down / exit+save |
| GPIO 14 | BTN_X | In settings: increment (+) or trigger CAL |
| GPIO 12 | BTN_Y | In settings: decrement (−) |

### Limit Switches (active-low, internal pull-up)

| ESP32 GPIO | Switch | Triggers |
|---|---|---|
| GPIO 13 | Home / X end | HOMING → stop and zero position |
| GPIO 36 | Far / Y end  | CAL_RUNNING → record travel distance; PEELING/MOVING → safety abort. Input-only pin — requires external 10 kΩ pull-up to 3.3 V |

---

## Software Dependencies

Install via the Arduino Library Manager before compiling:

| Library | Notes |
|---|---|
| **FastAccelStepper** | Hardware-timer step generation |
| **Adafruit ST7789** | TFT display driver |
| **Adafruit GFX** | Graphics primitives |
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

---

## Configuration

Key settings adjustable in the sketch or via the on-device settings menu:

| Parameter | Description |
|---|---|
| Speed (Hz) | Motor step rate — adjustable at runtime via BTN_X/BTN_Y in settings |
| Angle (°) | Peeling arm half-angle θ — used for µm/step conversion |
| Steps per rev | Microstep resolution matching DM542T DIP switch setting |
| Start position | Distance from home to peel start point (set by calibration) |

Calibration (CAL) runs a full home→far-end traversal to measure travel distance automatically.

---

## Serial Command Interface

Connect at **115 200 baud**. Send a single character followed immediately by any numeric argument.

| Command | Argument | Description |
|---|---|---|
| `m` | `<int32>` | Move to absolute step position. Example: `m200000` |
| `s` | — | Stop motor immediately |
| `v` | `<int>` | Set motor max speed in Hz. Example: `v500` |

---

## Serial Output

Every **100 ms** the firmware emits a JSON heartbeat:

```json
{"state":N,"position":N,"speed":N,"pos_um":F,"speed_um":F,"angle":N,"spr":N}
```

| Field | Description |
|---|---|
| `state` | State machine code (see below) |
| `position` | Current position in steps |
| `speed` | Current step rate (Hz) |
| `pos_um` | Current position in µm |
| `speed_um` | Current speed in µm/s |
| `angle` | Configured arm half-angle (°) |
| `spr` | Steps per revolution |

### State Codes

| Code | State |
|---|---|
| 0 | IDLE |
| 1 | MOVING |
| 2 | HOMING |
| 3 | MOVING_TO_START |
| 4 | PEELING |
| 5 | SETTINGS |
| 6 | CAL_HOMING |
| 7 | CAL_RUNNING |

---

## State Machine

```
IDLE → [A, dist_xa > 0] → MOVING_TO_START → [arrival + 100 ms] → PEELING
IDLE → [A, dist_xa = 0] → show warning "RUN CAL FIRST"
IDLE → [B, pos = 0]     → SETTINGS
IDLE → [B, pos > 0]     → HOMING

PEELING        → [A or limit switch]  → IDLE
HOMING         → [X limit switch]     → IDLE  (position := 0)
SETTINGS/CAL   → [X button]          → CAL_HOMING → CAL_RUNNING → IDLE (saves dist_xa)
Any moving state → [A]               → IDLE  (abort)
```

---

## Author

Lior Segev — Version 4.1.0-esp32, May 2026
