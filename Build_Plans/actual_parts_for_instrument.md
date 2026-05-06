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
| Enclosure | Hammond 1455Q2202 — 220×125×52mm aluminum body, plastic end panels (RF-transparent for Pico W WiFi) |
| AC switching | Illuminated rocker switch on front panel (mains-rated, 250V/10A) |
| IEC inlet | Rear panel, IEC C14 + integrated fuse holder, 1A T slow-blow fuse |
| Display | Waveshare Pico-ResTouch-LCD-3.5 (ILI9488, 480×320, ~73×49mm active area) — plugs directly onto Pico 40-pin header via SPI1 (GP8–GP13, GP16–GP17); **firmware pin reassignment + driver change required (not yet done)** |
| UI buttons | 2× Gateron KS-33 LP Brown (tactile) + low-profile MX keycaps, 14×14mm plate cutout, snap into 1.5mm aluminum panel |
| Button illumination | State-controlled via GPIO 0 (B-LED) and GPIO 1 (Y-LED) through 2N2222 transistors (reassigned from GP6/7 which conflict with Waveshare) |
| AC switching | Rear panel — illuminated rocker switch (mains-rated, 250V/10A); moved from front panel to accommodate 3.5" display width |
| Stepper output | GX16-4 aviation connector, 4-pin, female panel-mount on rear |
| Motor cable end | GX16-4 male plug (A+/A−/B+/B−) |
| Labels | Front Panel Express custom-printed aluminum panel |
| Display bezel | 3D-printed (design after ST7789 module arrives and PCB is measured) |
| E-stop | None |

---

## Complete Parts List

### Enclosure
| Part | MPN | Spec | Source | ~Cost |
|---|---|---|---|---|
| [Hammond 1455Q2202](https://www.hammfg.com/part/1455Q2202) | 1455Q2202 | 220×125×52mm, plastic end panels (WiFi-transparent) | [Mouser](https://www.mouser.com/ProductDetail/Hammond-Manufacturing/1455Q2202?qs=CUcEH/sN5tIHhQeXecWocA%3D%3D) / [DigiKey](https://www.digikey.com/en/products/detail/hammond-manufacturing/1455Q2202/2094863) | ~$35 |

### Power
| Part | MPN | Spec | Source | ~Cost |
|---|---|---|---|---|
| [Mean Well LRS-75-24](https://www.meanwell.com/productPdf.aspx?i=403) | LRS-75-24 | 24V / 3.2A, 75W | (already owned) — [DigiKey](https://www.digikey.com/en/products/detail/mean-well-usa-inc/LRS-75-24/7705055) / [Amazon](https://www.amazon.com/MeanWell-LRS-75-24-Power-Supply-75W/dp/B077B3X7S3) | — |
| [TSR 1-2450](https://www.tracopower.com/model/tsr-1-2450) | TSR 1-2450 | 24V → 5V / 1A, SIP-3 | [Mouser](https://www.mouser.com/ProductDetail/TRACO-Power/TSR-1-2450?qs=ckJk83FOD0XFKqda0Mzkgw%3D%3D) / [DigiKey](https://www.digikey.ca/en/products/detail/traco-power/TSR-1-2450/9383780) | $6 |
| [Schurter 4301.1400](https://www.schurter.com/en/products-and-solutions/components/connectors/power-entry-modules) | 4301.1400 | IEC C14 + fuse holder, panel-mount | [Mouser](https://www.mouser.com/c/connectors/power-entry-connectors/?m=Schurter) | $5 |
| Fuse T1A 5×20mm | — | 1A slow-blow | Any (DigiKey, Mouser, Amazon) | $1 |

### Front Panel
| Part | MPN | Spec | Source | ~Cost |
|---|---|---|---|---|
| [KCD3 rocker switch](https://www.aliexpress.com/w/wholesale-kcd3-rocker-switch.html) | KCD3 | 22×30mm cutout, 250V/10A, illuminated | [AliExpress](https://www.aliexpress.com/w/wholesale-kcd3-rocker-switch.html) | $3 |
| [Waveshare Pico-ResTouch-LCD-3.5](https://www.waveshare.com/pico-restouch-lcd-3.5.htm) | Pico-ResTouch-LCD-3.5 | ILI9488, 480×320, ~73×49mm active area, plug-on Pico board (SPI1 GP8–GP13, GP16–GP17) | [Waveshare](https://www.waveshare.com/pico-restouch-lcd-3.5.htm) | ~$25 |
| [Gateron KS-33 LP 2.0 Brown (35pcs)](https://www.gateron.com/products/gateron-ks-33-low-profile-switch-set) | KS-33 Brown | Low-profile tactile, 14×14mm plate cutout, SMD LED slot, 1.5mm plate snap-in | [Gateron](https://www.gateron.com/products/gateron-ks-33-low-profile-switch-set) / [Amazon](https://www.amazon.com/GATERON-Switches-Pre-lubed-Mechanical-Keyboard/dp/B0BZYHJ9XB) | ~$15 (35pcs, need 2) |
| Low-profile MX keycaps 1U blank (×2) | — (generic) | LP MX stem, fits KS-33 — e.g. Nuphy or Keychron LP caps | [Amazon](https://www.amazon.com/s?k=low+profile+mx+keycap+1u+blank) / [Keychron](https://www.keychron.com) | ~$5 |
| [Front Panel Express panel](https://www.frontpanelexpress.com/) | — (custom) | 122×78mm, **1.5mm aluminum** (required for MX snap-in mount) | [frontpanelexpress.com](https://www.frontpanelexpress.com/) | ~$40 |

### Rear Panel
| Part | MPN | Spec | Source | ~Cost |
|---|---|---|---|---|
| [GX16-4 female panel-mount](https://www.aliexpress.com/item/32830722292.html) | GX16-4 | 4-pin aviation socket | [AliExpress](https://www.aliexpress.com/item/32830722292.html) | $2 |
| [GX16-4 male cable-end](https://www.aliexpress.com/item/32830722292.html) | GX16-4 | 4-pin aviation plug (for motor cable) | [AliExpress](https://www.aliexpress.com/item/32830722292.html) | $2 |
| [Panel-mount micro-USB](https://www.aliexpress.com/w/wholesale-micro-usb-panel-mount.html) | — (generic) | Female, solder terminals | [AliExpress](https://www.aliexpress.com/w/wholesale-micro-usb-panel-mount.html) | $4 |

### MCU
| Part | MPN | Spec | Source | ~Cost |
|---|---|---|---|---|
| [Raspberry Pi Pico W](https://www.raspberrypi.com/products/raspberry-pi-pico/) | SC0918 | RP2040, 2.4GHz WiFi, micro-USB | [Raspberry Pi](https://www.raspberrypi.com/products/raspberry-pi-pico/) / [Adafruit](https://www.adafruit.com/product/5526) / [DigiKey](https://www.digikey.com/en/products/detail/raspberry-pi/SC0918/16608263) | $6 |

### Internal Electronics
| Part | MPN | Qty | Source | ~Cost |
|---|---|---|---|---|
| 2N2222 NPN transistor (TO-92) | 2N2222A | 2 | Any | <$1 |
| 1kΩ resistor 1/4W | — (generic) | 2 | Any | <$1 |
| Perfboard strip | — (generic) | 1 small | Any | $1 |

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
|←13→|←14→|←8→|←────73────→|←14→|
      BTN-B       DISPLAY
      BTN-Y
```

PWR rocker is on the rear panel (3.5" display fills available front width).
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

## Firmware Changes Required

The Waveshare Pico-ResTouch-LCD-3.5 occupies GP8–GP13 and GP15–GP17 (display + touch). Several of these conflict with the current firmware. Required changes before hardware assembly:

| Change | Current | New |
|---|---|---|
| Display driver | `Adafruit_ST7789` | Waveshare ILI9488 library |
| SPI bus | SPI0 (GP18/19) | SPI1 (GP10/11, internal to Waveshare board) |
| Screen resolution | 240×240 | 480×320 + landscape UI layout |
| BTN_A pin | GP12 | **GP6** |
| BTN_B pin | GP13 | **GP7** |
| BTN_Y limit pin | GP15 | **GP2** |
| B-LED pin | GP6 | **GP0** |
| Y-LED pin | GP7 | **GP1** |

BTN_X (GP14) and stepper pins (GP3/4/5) are unaffected.
Estimated scope: ~20 lines for pin reassignment + display driver swap + UI layout recalculation.

---

## Action Items (ordered)

1. Order all parts except Front Panel Express panel and display bezel.
2. Receive ST7789 module → measure PCB dimensions.
3. Design and print display bezel.
4. Design Front Panel Express file (122×78mm) → order panel.
5. Wire internal components.
6. Add GPIO 6/7 LED control to firmware.
