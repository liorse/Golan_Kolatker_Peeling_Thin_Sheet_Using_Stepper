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
- [Calibration](#calibration)
- [Serial Command Interface](#serial-command-interface)
- [Serial Output](#serial-output)
- [State Machine](#state-machine)
- [LED Behaviour](#led-behaviour)
- [Known Issues](#known-issues)

---

## Overview

This firmware:
- Controls a NEMA 17 stepper motor through a TB6600 driver to drive a peeling stage up and down.
- Homes the stage automatically at power-on using a normally-open microswitch.
- Exposes a simple single-character serial command interface (115 200 baud) for external software control.
- Emits a JSON status object every 100 ms so a host application can monitor position and state in real time.
- Drives a sync strobe LED at a randomised interval to signal the peeling rhythm to external equipment (e.g. a camera trigger).
- Drives a position-indicator LED that blinks 1 / 2 / 3 times to acknowledge bottom / middle / top positions.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino Uno (ATmega328P) |
| Stepper motor | NEMA 17 — rated 0.4 A |
| Stepper driver | TB6600 |
| Home switch | Normally-Open microswitch |
| Sync LED | Any 5 V LED on pin 10 |
| Position LED | Any 5 V LED on pin 11 |
| Power supply | 12 V, 1 A minimum |

The Arduino and driver logic are both powered from the 12 V supply. No separate power supply is needed.

### Driver Current Limit

Set the TB6600 current-limit trim-pot so that the test-point on the board reads **2.3 V** while the driver is powered. This limits the motor current to its nominal 0.4 A rating.

---

## Wiring

| Arduino Pin | Signal | Connected To |
|---|---|---|
| 7 | `MICROSWITCH_PIN` | Home/limit switch (other leg to GND) |
| 8 | `dirPinStepper` | TB6600 DIR input |
| 9 | `stepPinStepper` | TB6600 PUL input |
| 10 | `LED_PIN` | Sync strobe LED (+ resistor to GND) |
| 11 | `POSITION_LED_PIN` | Position-indicator LED (+ resistor to GND) |
| 12 | `enablePinStepper` | TB6600 ENA input |
| GND | — | Common ground (Arduino, driver, switch) |

The microswitch is wired between pin 7 and GND. The internal pull-up resistor is enabled, so the pin reads HIGH when open and LOW when the switch is pressed (stage at home).

---

## Software Dependencies

Install the following libraries through the Arduino Library Manager before compiling:

| Library | Version tested | Notes |
|---|---|---|
| **FastAccelStepper** | latest | Hardware-timer step generation; much higher step rates than AccelStepper |
| **AVRStepperPins** | (bundled with FastAccelStepper) | AVR pin constants |
| **Countimer** | latest | Lightweight interval timer for LED sync |
| Wire | built-in | Reserved for future MPR121 touch-interface support |

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
| `POS_GROUND` | 3 500 | Lowest safe resting position (steps above home switch) |
| `POS_MIDDLE` | 239 042 | Mid-travel position (steps) |
| `POS_TOP` | 478 085 | Full-travel top position (steps) |
| `SPEED_MAX` | 30 000 | Maximum step rate (Hz) — use ~1 250 for full-step mode |
| `LED_delay_between_pulses_ms` | 1 000 | Nominal sync LED inter-pulse period (ms); also settable via `w` command |
| `LED_pulse_duration_ms` | 30 | Sync LED on-time per pulse (ms); also settable via `p` command |
| `BOTTOM_PULSE_DURATION` | 2 | Minimum allowed pulse duration (ms) |
| `TOP_PULSE_DURATION` | 500 | Maximum allowed pulse duration (ms) |
| `BOTTOM_PULSE_WAIT` | 50 | Minimum allowed inter-pulse wait (ms) |
| `TOP_PULSE_WAIT` | 3 600 000 | Maximum allowed inter-pulse wait (ms) — 1 hour |

---

## Calibration

The firmware starts in the `CALIBRATING` state every time it powers on.

1. The motor drives downward (large negative step target).
2. When the microswitch closes (pin 7 goes LOW), the driver immediately stops and declares that position as **step 0**.
3. The motor then rises to `POS_GROUND` (3 500 steps) as the safe resting height.

Calibration can also be triggered at any time by sending the `c` serial command.

If the microswitch closes unexpectedly while the motor is in the `MOVING` state, the firmware re-zeros and rises to `POS_GROUND` automatically as a safety measure.

---

## Serial Command Interface

Connect at **115 200 baud** (no line ending required). Send a single character followed immediately by any numeric argument.

| Command | Argument | Description |
|---|---|---|
| `m` | `<int32>` | Move to absolute step position. Clamped to `[POS_GROUND … POS_TOP]`. Example: `m200000` |
| `s` | — | Stop motor immediately. |
| `c` | — | Trigger homing / calibration sequence. |
| `t` | — | Blink position LED **3 times** (top position acknowledge). |
| `h` | — | Blink position LED **2 times** (middle position acknowledge). |
| `b` | — | Blink position LED **1 time** (bottom position acknowledge). |
| `p` | `<int>` | Set sync LED pulse duration in ms. Clamped to `[2 … 500]`. |
| `w` | `<int>` | Set sync LED inter-pulse wait in ms. Clamped to `[50 … 3 600 000]`. |
| `v` | `<int>` | Set motor max speed in Hz. Also starts LED sync (see Known Issues). |
| `1` | — | Start LED sync pulsing. |
| `0` | — | Stop LED sync pulsing. |

---

## Serial Output

Every **100 ms** the firmware emits a JSON object on the serial port:

```json
{"state":<int>,"position":<int32>}
```

| Field | Type | Description |
|---|---|---|
| `state` | int | Current state machine state (see table below) |
| `position` | int32 | Current motor position in steps (0 = home switch) |

### State Codes

| Code | State | Description |
|---|---|---|
| 1 | `INIT` | Entry state; transitions immediately at startup |
| 2 | `MOVING` | Motor executing a move command |
| 3 | `WAITING` | Motor idle; LED sync active |
| 4 | `CALIBRATING` | Homing sequence in progress |
| 5 | `ON_MICROSWITCH` | Reserved; not yet used |

---

## State Machine

```
Power-on / reset
      │
      ▼
 CALIBRATING ──(home switch found)──► MOVING ──(target reached)──► WAITING
      ▲                                  │                             │
      └──────────────('c' command)───────┘◄────────('m' command)──────┘
                                         │
                              (home switch hit during move)
                                         │
                                      re-zeros → MOVING → WAITING
```

---

## LED Behaviour

### Sync LED (pin 10)

- Activated by the `1` serial command; deactivated by `0`.
- While active, emits a short pulse (`LED_pulse_duration_ms` wide) at a **randomised** interval of ±50 % around `LED_delay_between_pulses_ms`.
- Randomisation avoids stroboscopic locking artefacts with camera frame rates.
- Pulsing is automatically **suppressed** while the motor is moving and **resumed** when the motor is idle.

### Position LED (pin 11)

Blinks synchronously in response to `t` / `h` / `b` commands to acknowledge that a position was reached:

| Blinks | Command | Position |
|---|---|---|
| 1 | `b` | Bottom |
| 2 | `h` | Middle |
| 3 | `t` | Top |

---

## Known Issues

- **`v` command fall-through**: The `case 'v'` block is missing a `break` statement, causing it to fall through into `case '1'`. As a result, setting the motor speed via `v` also automatically starts LED sync. Send `0` after `v` if LED sync is not desired.

---

## Author

Lior Segev — Version 1.0.0, July 17, 2023
