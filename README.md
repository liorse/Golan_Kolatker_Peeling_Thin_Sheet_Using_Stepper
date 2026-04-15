# Peeling Thin Sheet Using Stepper Motor — Arduino Uno Controller

Firmware for an automated thin-sheet peeling machine driven by a NEMA 17 stepper motor and controlled via an Arduino Uno.

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
- Controls a NEMA 17 stepper motor through a TB6600 driver to drive a peeling stage up and down.
- Exposes a simple single-character serial command interface (115 200 baud) for external software control.
- Emits a JSON status object every 100 ms so a host application can monitor position and state in real time.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino Uno (ATmega328P) |
| Stepper motor | NEMA 17 — rated 0.4 A |
| Stepper driver | DM542T (V4.0) — Leadshine digital driver |
| Power supply | 12 V, 1 A minimum |

The Arduino and driver are powered separately — the DM542T requires **24–48 V** for the motor supply (12 V minimum).

### Driver Configuration (DIP Switches)

The DM542T uses DIP switches instead of a trim-pot:

| Switches | Function | Recommendation |
|---|---|---|
| SW1–SW3 | Peak output current | Match to motor rated current |
| SW4–SW6 | Microstep resolution | Match to firmware step counts |
| SW7 | Standby current reduction | ON = 50 % current at rest (saves heat) |
| SW8 | Pulse active edge | ON = rising edge (default) |

Refer to the DM542T datasheet for the exact SW1–SW3 current table.

---

## Wiring

| Arduino Pin | Signal | Connected To |
|---|---|---|
| 8 | `dirPinStepper` | DM542T DIR- |
| 9 | `stepPinStepper` | DM542T PUL- |
| 12 | `enablePinStepper` | DM542T ENA- (active-low) |
| GND | — | DM542T DIR+, PUL+, ENA+ tied to +5 V via 1 kΩ, or use differential mode |

---

## Software Dependencies

Install the following libraries through the Arduino Library Manager before compiling:

| Library | Notes |
|---|---|
| **FastAccelStepper** | Hardware-timer step generation; much higher step rates than AccelStepper |
| **AVRStepperPins** | AVR pin constants (bundled with FastAccelStepper) |

---

## Installation

1. Clone or download this repository.
2. Open `Peeling_Automation_Stepper_Control_Uno/Peeling_Automation_Stepper_Control_Uno.ino` in the Arduino IDE.
3. Install the libraries listed above via **Sketch → Include Library → Manage Libraries**.
4. Select **Board: Arduino Uno** and the correct COM port.
5. Click **Upload**.

---

## Configuration

The following constants can be adjusted at the top of the sketch:

| Constant | Default | Description |
|---|---|---|
| `POS_MIDDLE` | 239 042 | Mid-travel position (steps) |
| `POS_TOP` | 478 085 | Full-travel top position (steps) |
| `SPEED_MAX` | 30 000 | Maximum step rate (Hz) — use ~1 250 for full-step mode |

Position 0 is wherever the motor is when the Arduino powers on. Move the stage to the desired home position before powering on, or issue move commands relative to that starting point.

---

## Serial Command Interface

Connect at **115 200 baud** (no line ending required). Send a single character followed immediately by any numeric argument.

| Command | Argument | Description |
|---|---|---|
| `m` | `<int32>` | Move to absolute step position. Clamped to `[0 … POS_TOP]`. Example: `m200000` |
| `s` | — | Stop motor immediately. |
| `v` | `<int>` | Set motor max speed in Hz. Example: `v15000` |

---

## Serial Output

Every **100 ms** the firmware emits a JSON object on the serial port:

```json
{"state":<int>,"position":<int32>}
```

| Field | Type | Description |
|---|---|---|
| `state` | int | Current state machine state (see table below) |
| `position` | int32 | Current motor position in steps (0 = power-on position) |

### State Codes

| Code | State | Description |
|---|---|---|
| 1 | `INIT` | Entry state at startup |
| 2 | `MOVING` | Motor executing a move command |
| 3 | `WAITING` | Motor idle |

---

## State Machine

```
Power-on / reset
      │
      ▼
   WAITING ◄────────────(target reached)────────── MOVING
      │                                                ▲
      └──────────────────('m' command)─────────────────┘
      │
      └──────────────────('s' command while moving)──► WAITING
```

---

## Author

Lior Segev — Version 2.0.0, April 15, 2026

