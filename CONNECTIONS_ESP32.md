# ESP32 (Wemos D1 R32) — Wiring Reference

## Display — ST7789 via Pico Explorer Base header

The Pico Explorer Base display is permanently wired to the Pico's GPIO header.
Plug into those header pins directly; do **not** insert a Pico.

| Signal | ESP32 GPIO | Wemos D1 R32 label | Pico Explorer header pin | Pico GPIO |
|--------|-----------:|---------------------|:------------------------:|-----------|
| DC     | 17         | D4                  | Pin 21                   | GP16      |
| CS     | 5          | D10 (SS)            | Pin 22                   | GP17      |
| SCK    | 18         | D13                 | Pin 24                   | GP18      |
| MOSI   | 23         | D11                 | Pin 25                   | GP19      |
| BL     | 16         | D5                  | Pin 26                   | GP20      |
| GND    | GND        | GND                 | Pin 23                   | GND       |
| 3V3    | 3V3        | 3V3                 | Pin 36                   | 3V3\_OUT  |

> **RST** is not connected (firmware sets `TFT_RST = -1`). The display resets
> via the board's power-on rail.

> **Power**: with no Pico inserted, supply 3.3 V from the ESP32's 3V3 pin to
> Pico header **pin 36** to power the display circuitry.

---

## Buttons

All four Pico Explorer Base buttons are used. The X and Y buttons simulate the
home and far-end limit switches (microswitches) during bench testing.

| Function                       | ESP32 GPIO | Wemos D1 R32 label | Pico Explorer header pin | Pico GPIO |
|--------------------------------|-----------:|--------------------|:------------------------:|-----------|
| BTN_A (start / stop / +)       | 4          | A1                 | Pin 16                   | GP12      |
| BTN_B (settings / home / −)    | 19         | D12                | Pin 17                   | GP13      |
| BTN_X (home limit / simulate)  | 21         | SDA                | Pin 19                   | GP14      |
| BTN_Y (far limit / simulate)   | 22         | SCL                | Pin 20                   | GP15      |

> All buttons are active-low. The ESP32 enables internal pull-ups so no
> external resistor is needed.

---

## Stepper Driver (DM542T)

| Signal | ESP32 GPIO | Wemos D1 R32 label | DM542T terminal |
|--------|------------|---------------------|-----------------|
| ENA+   | 26         | D2                  | ENA+            |
| DIR+   | 25         | D3                  | DIR+            |
| PUL+   | 27         | D6                  | PUL+            |
| GND    | GND        | GND                 | ENA− / DIR− / PUL− (all three) |

> Active-high wiring: the − terminals are tied to GND, and the MCU drives the
> + terminals. ENA+ must go HIGH ≥ 200 ms before the first pulse (firmware
> uses 500 ms).
