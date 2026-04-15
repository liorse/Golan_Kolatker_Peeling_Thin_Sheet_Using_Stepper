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
| Stepper driver | TB6600 |
| Power supply | 12 V, 1 A minimum |

The Arduino and driver logic are both powered from the 12 V supply.

### Driver Current Limit

Set the TB6600 current-limit trim-pot so that the test-point on the board reads **2.3 V** while the driver is powered. This limits the motor current to its nominal 0.4 A rating.

---

## Wiring

| Arduino Pin | Signal | Connected To |
|---|---|---|
| 8 | `dirPinStepper` | TB6600 DIR input |
| 9 | `stepPinStepper` | TB6600 PUL input |
| 12 | `enablePinStepper` | TB6600 ENA input |
| GND | — | Common ground (Arduino + driver) |

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

