# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Upload

This is an Arduino firmware project. Use `arduino-cli` or the Arduino IDE.

### Raspberry Pi Pico (branch: master)

```bash
# Compile (arduino-pico core by Earle Philhower must be installed)
# os=freertos is required — FastAccelStepper v1.2.5 on RP2040 uses FreeRTOS internally
arduino-cli compile --fqbn rp2040:rp2040:rpipico:os=freertos Peeling_Automation_Stepper_Control_Uno/

# Upload (replace /dev/ttyACM0 with the actual port)
arduino-cli upload --fqbn rp2040:rp2040:rpipico:os=freertos --port /dev/ttyACM0 Peeling_Automation_Stepper_Control_Uno/

# Monitor serial output (115200 baud)
arduino-cli monitor --port /dev/ttyACM0 --config baudrate=115200
```

### Wemos D1 R32 / ESP32 (branch: esp32)

Sketch: `Peeling_Automation_Stepper_esp32/Peeling_Automation_Stepper_esp32.ino`

```bash
# Install ESP32 core (once)
arduino-cli core install esp32:esp32

# Compile
arduino-cli compile --fqbn esp32:esp32:d1_mini32 Peeling_Automation_Stepper_esp32/

# Upload (replace /dev/ttyUSB0 with the actual port)
# UploadSpeed=460800 is required — the CH340 on the D1 R32 drops at the default 921600
arduino-cli upload --fqbn esp32:esp32:d1_mini32:UploadSpeed=460800 --port /dev/ttyUSB0 Peeling_Automation_Stepper_esp32/

# Monitor serial output (115200 baud)
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
```

Required libraries: **FastAccelStepper**, **Adafruit ST7789**, **Adafruit GFX**, **ESP Async WebServer** (v3.3.x by mathieucarbou), **AsyncTCP** — install via Arduino Library Manager. (`ESPmDNS` is bundled with the ESP32 core — no separate install needed.)

> Note: use `ESP Async WebServer` ≥ 3.3.x (mathieucarbou fork). The older `ESPAsyncWebServer` by me-no-dev has mbedtls API breakage with ESP32 core ≥ 3.x and will not compile.

The web UI (`index.html`) is stored as a PROGMEM string inside the sketch.  When editing
the UI, update **both** the PROGMEM string in the `.ino` file and the canonical copy at
`serial_bridge/index.html` (used by the Python bridge).

### Serial bridge (Python)

Run the serial bridge on your computer to control the instrument over USB without WiFi:

```bash
cd serial_bridge
pip install -r requirements.txt

# Auto-detect CH340 and start
python serial_bridge.py

# Specify port explicitly
python serial_bridge.py /dev/ttyUSB0          # Linux / macOS
python serial_bridge.py COM3                   # Windows

# Custom HTTP port
python serial_bridge.py --http-port 9090
```

Then open **http://localhost:8080** in any browser.  The 🔌 SERIAL badge confirms the
serial transport.  WiFi WebSocket and physical buttons remain active simultaneously.

## Architecture

The ESP32 firmware is a single file: `Peeling_Automation_Stepper_esp32/Peeling_Automation_Stepper_esp32.ino`.

`loop()` has four sequential sections on every iteration:
1. **Serial parser** — reads one character, dispatches `m` / `s` / `v` / `b` commands
2. **Button polling** — physical + virtual (WebSocket / serial-bridge) buttons merged
3. **State machine** — `IDLE ↔ MOVING_TO_START ↔ PEELING` etc.
4. **100 ms JSON heartbeat** — emits full state JSON on both `Serial` and WebSocket

### Serial commands

| Command | Example | Effect |
|---------|---------|--------|
| `m<pos>` | `m50000` | Move to step position |
| `s` | `s` | Stop / abort |
| `v<hz>` | `v200` | Set speed in Hz |
| `bXY` | `bA1` / `bA0` | Virtual button press / release (serial bridge) |

`b` commands use the same `virtualBtn[]` mechanism as the WiFi WebSocket, so the entire
state machine (settings, homing, calibration) is reachable over serial.

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
