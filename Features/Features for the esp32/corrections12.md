# Temperature Control — Future Feature

A future firmware update will add closed-loop temperature control of a heat pad.

## Hardware

- **Sensor**: CJMCU MAX31856 thermocouple amplifier with K-type thermocouple (SPI interface)
- **Actuator**: N-channel MOSFET module controlled via PWM (resistive heat pad load)

---

## Pin Conflict Resolution

GPIO 19 is currently `BTN_B` in firmware but is also declared as the VSPI MISO pin in
`SPI.begin(18, 19, 23, -1)`. The ST7789 display is write-only so this conflict was
harmless — until now. Wiring MAX31856 SDO to GPIO 19 would cause the sensor to fight the
button pull-up on every SPI read.

**Fix**: move BTN_B from GPIO 19 → GPIO 33 (one firmware line, one rewired breadboard wire).
GPIO 19 becomes a clean MISO. `SPI.begin()` is unchanged.

```cpp
// Peeling_Automation_Stepper_esp32.ino
#define BTN_B   33   // was 19
```

---

## MAX31856 Wiring

| MAX31856 Pin | Purpose                        | Connect to       | Notes                                      |
|--------------|--------------------------------|------------------|--------------------------------------------|
| VIN          | Power input                    | ESP32 **3.3 V**  | Breakout has onboard regulator             |
| 3Vo          | Regulated 3.3 V output         | **NC**           | Output from breakout — leave unconnected   |
| GND          | Ground                         | **GND**          |                                            |
| SCK          | SPI clock                      | **GPIO 18**      | Shared VSPI SCK (display uses it too)      |
| SDI          | SPI data in (MOSI)             | **GPIO 23**      | Shared VSPI MOSI                           |
| SDO          | SPI data out (MISO)            | **GPIO 19**      | Clean MISO now that BTN_B has moved        |
| CS           | Chip select (active-low)       | **GPIO 21**      |                                            |
| DRDY         | Data-ready interrupt           | **GPIO 22**      | New conversion ready ≈ every 143 ms        |
| FLT          | Fault output                   | **NC**           | Read fault register via SPI instead        |

---

## MOSFET PWM Wiring

| Signal          | GPIO        | Notes                    |
|-----------------|-------------|--------------------------|
| PWM → MOSFET gate | **GPIO 32** | ESP32 LEDC, **1 kHz**, 8-bit duty |

---

## BTN_B Change Summary

| Signal | Old GPIO | New GPIO |
|--------|----------|----------|
| BTN_B  | 19       | **33**   |

---

## Remaining Free GPIOs After This Addition

| GPIO | Type        | Status |
|------|-------------|--------|
| 34   | Input-only  | Spare  |
| 35   | Input-only  | Spare  |
| 39   | Input-only  | Spare  |

---

## Future Firmware Implementation Notes

When implementing the temperature control loop:

1. **Library**: `Adafruit_MAX31856` — declare with `cs=21, drdy=22`
2. **PWM setup**:
   ```cpp
   ledcSetup(channel, 1000, 8);      // 1 kHz, 8-bit
   ledcAttachPin(32, channel);
   ledcWrite(channel, duty);          // 0–255
   ```
3. **EEPROM**: add `float target_temp_c` to `SavedSettings`; bump `EEPROM_MAGIC` to `"PEL5"`
4. **Settings field**: add `FIELD_TEMP` after `FIELD_CAL` in the `SettingsField` enum
5. **Safety**: on any MAX31856 fault (open/short thermocouple), immediately set PWM duty = 0
6. **Control**: start with bang-bang (on/off at setpoint ± hysteresis); upgrade to PID later if needed
