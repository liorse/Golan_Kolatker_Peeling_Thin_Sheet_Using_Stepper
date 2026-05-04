# Peeling Instrument — Build Specification

## Original Requirements

1. A professional-looking box containing the power supply, Pico W (WiFi), and stepper driver.
2. Box inputs: power button (24V rail + 5V Pico), two UI buttons, USB connector for firmware updates, 220V socket.
3. Box outputs: stepper motor connector, display.
4. Minimal soldering.
5. Power the Pico from the 24V DC rail (no battery).

---

## Decisions Made

| Topic | Decision |
|---|---|
| MCU | Raspberry Pi Pico W (RP2040 + WiFi) — NOT Pico 2W |
| Pico power | TSR 1-2450 buck regulator (24V → 5V, SIP-3), feeds VSYS pin |
| Pico programming | Panel-mount micro-USB on rear; coexists with VSYS power (internal OR-diode) |
| WiFi | Status monitoring + remote control (firmware TBD separately) |
| Enclosure | Hammond 1455T2601 — 260×122×78mm aluminum extruded |
| AC switching | Illuminated rocker switch on front panel (mains-rated, 250V/10A) |
| IEC inlet | Rear panel, IEC C14 + integrated fuse holder, 1A T slow-blow fuse |
| Display | Standalone ST7789 1.3" 240×240 SPI module, mounted in 3D-printed bezel |
| UI buttons | 2× 16mm metal illuminated momentary pushbutton (5V LED): black (B), green (Y) |
| Button illumination | State-controlled via GPIO 6 (B-LED) and GPIO 7 (Y-LED) through 2N2222 transistors |
| Stepper output | GX16-4 aviation connector, 4-pin, female panel-mount on rear |
| Motor cable end | GX16-4 male plug (A+/A−/B+/B−) |
| Labels | Front Panel Express custom-printed aluminum panel |
| Display bezel | 3D-printed (design after ST7789 module arrives and PCB is measured) |
| E-stop | None |

---

## Complete Parts List

### Enclosure
| Part | Spec | Source | ~Cost |
|---|---|---|---|
| Hammond 1455T2601 | 260×122×78mm aluminum | Mouser / DigiKey | $40 |

### Power
| Part | Spec | Source | ~Cost |
|---|---|---|---|
| Mean Well LRS-75-24A | 24V / 3.2A, 75W | (already owned) | — |
| TSR 1-2450 | 24V → 5V / 1A, SIP-3 | Mouser / DigiKey | $6 |
| Schurter 4301.1400 | IEC C14 + fuse holder, panel-mount | Mouser | $5 |
| Fuse T1A 5×20mm | 1A slow-blow | Any | $1 |

### Front Panel
| Part | Spec | Source | ~Cost |
|---|---|---|---|
| KCD3 rocker switch | 22×30mm cutout, 250V/10A, illuminated | AliExpress | $3 |
| ST7789 1.3" 240×240 module | SPI, documented PCB dimensions | Waveshare / Adafruit equiv. | $10 |
| 16mm metal pushbutton, black | Momentary, 5V LED, illuminated (Button B) | AliExpress | $5 |
| 16mm metal pushbutton, green | Momentary, 5V LED, illuminated (Button Y) | AliExpress | $5 |
| Front Panel Express panel | 122×78mm custom-printed aluminum | frontpanelexpress.com | ~$40 |

### Rear Panel
| Part | Spec | Source | ~Cost |
|---|---|---|---|
| GX16-4 female panel-mount | 4-pin aviation socket | AliExpress | $2 |
| GX16-4 male cable-end | 4-pin aviation plug (for motor cable) | AliExpress | $2 |
| Panel-mount micro-USB | Female, solder terminals | AliExpress | $4 |

### Internal Electronics
| Part | Qty | Source | ~Cost |
|---|---|---|---|
| 2N2222 NPN transistor (TO-92) | 2 | Any | <$1 |
| 1kΩ resistor 1/4W | 2 | Any | <$1 |
| Perfboard strip | 1 small | Any | $1 |

### Wiring & Connectors
| Item | Notes |
|---|---|
| 1.5mm² mains wire (L/N/PE) | Brown, blue, green-yellow — IEC inlet → rocker switch → PSU |
| Bootlace ferrules 1.5mm² | Crimp on all mains wire ends |
| 0.5mm² hookup wire, assorted | DC signals and LED wiring |
| Dupont female-to-female cables 20cm | Pico GPIO → display (7 wires) and GPIO → buttons |
| Short micro-B to micro-B USB cable | Internal: panel-mount USB socket → Pico W USB port |

---

## Front Panel Layout (122mm × 78mm)

```
|←8→|←22→|←7→|←30→|←7→|←16→|←8→|←16→|←8→|
     PWR       DISPLAY     BTN-B       BTN-Y
```

All items vertically centered on the 78mm face.

---

## Internal Wiring Summary (minimal soldering)

| Connection | Method | Solder joints |
|---|---|---|
| 220V inlet → front rocker → PSU | Ferrule-crimped wire | 0 |
| PSU 24V → DM542T power | Screw terminals both ends | 0 |
| DM542T step/dir/ena → Pico GPIO | DM542T screw terminals + Dupont on Pico header | 0 |
| PSU 24V → TSR 1-2450 → Pico VSYS | Short wires, SIP-3 on perfboard | ~3 |
| Pico GPIO → display | Dupont female-to-female harness (7 wires) | 0 |
| Pico GPIO → buttons (signal) | Dupont female per button | 0 |
| Pico GPIO 6/7 → 2N2222 → button LEDs | Transistor circuit on perfboard | ~4 |
| DM542T output → GX16-4 panel socket | 4 wires soldered to GX16-4 | 4 |

**Total soldering: ~11 joints.**

---

## Firmware Change Required

State-controlled button LEDs require two new GPIO outputs:
- **GPIO 6** → B-button LED (on during SETTINGS / HOMING)
- **GPIO 7** → Y-button LED (on during MOVING_TO_START / PEELING)

Approximately 10 lines added to the state machine. No other firmware changes needed for the enclosure build.

---

## Action Items (ordered)

1. Order all parts except Front Panel Express panel and display bezel.
2. Receive ST7789 module → measure PCB dimensions.
3. Design and print display bezel.
4. Design Front Panel Express file (122×78mm) → order panel.
5. Wire internal components.
6. Add GPIO 6/7 LED control to firmware.
