# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Upload

This is an Arduino/Pico firmware project. Use `arduino-cli` or the Arduino IDE.

```bash
# Compile (arduino-pico core by Earle Philhower must be installed)
# os=freertos is required — FastAccelStepper v1.2.5 on RP2040 uses FreeRTOS internally
arduino-cli compile --fqbn rp2040:rp2040:rpipico:os=freertos Peeling_Automation_Stepper_Control_Uno/

# Upload (replace /dev/ttyACM0 with the actual port)
arduino-cli upload --fqbn rp2040:rp2040:rpipico:os=freertos --port /dev/ttyACM0 Peeling_Automation_Stepper_Control_Uno/

# Monitor serial output (115200 baud)
arduino-cli monitor --port /dev/ttyACM0 --config baudrate=115200
```

Required library: **FastAccelStepper** — install via Arduino Library Manager or `arduino-cli lib install "FastAccelStepper"`.

## Architecture

The entire firmware is a single file: `Peeling_Automation_Stepper_Control_Uno/Peeling_Automation_Stepper_Control_Uno.ino`.

`loop()` has three sequential sections on every iteration:
1. **Serial parser** — reads one character, dispatches `m` / `s` / `v` commands
2. **State machine** — `WAITING ↔ MOVING`; transitions to WAITING when `stepper->isRunning()` goes false
3. **100 ms JSON heartbeat** — emits `{"state":N,"position":N,"speed":N}` for host monitoring

## Key Constants

| Constant | Default | Notes |
|---|---|---|
| `POS_TOP` | 478 085 steps | Full peel stroke; `m` commands are clamped to `[0, POS_TOP]` |
| `POS_MIDDLE` | `POS_TOP / 2` | Not used in logic; available for convenience |
| `SPEED_MAX` | 100 Hz | Very conservative default; change at runtime with `v<Hz>` |

## Hardware & Timing Constraints

- **MCU**: Raspberry Pi Pico (RP2040) — despite the `.ino` filename referencing "Uno"
- **Driver**: DM542T (active-low ENA-). Timing from datasheet Fig. 15:
  - ENA- asserted LOW → first PUL ≥ 200 ms (firmware uses 500 ms `delay()`)
  - DIR stable before PUL ≥ 5 µs — enforced via `setDirectionPin(..., 40)` (40 µs)
- Pico GPIO is 3.3 V; DM542T minimum HIGH is 3.5 V — a level shifter or voltage divider may be needed on step/dir/enable lines.
- `setAcceleration(2147483647)` disables ramping — motor starts at full speed immediately.

## README Mismatch

`README.md` documents the v2.0.0 Arduino Uno build. The current firmware (v3.0.0) targets the Raspberry Pi Pico with different pin numbers. When updating the README, use the pin assignments in the `.ino` file header (GPIO 3/4/5), not the README wiring table.
