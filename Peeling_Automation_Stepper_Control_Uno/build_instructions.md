# Peeling Instrument — Build Instructions

> **MAINS VOLTAGE HAZARD.** This instrument contains 220 V AC wiring. Work with the power cord unplugged at all times during assembly. Double-check every mains connection before first power-on.

---

## Tools Required

- Drill + step drill bit (for panel holes)
- File or deburring tool
- Crimping tool for bootlace ferrules (1.5 mm²)
- Soldering iron + solder
- Multimeter
- Small flat and Phillips screwdrivers
- Callipers (for panel layout)

---

## Step 1 — Verify Component Fit Before Any Drilling

Before cutting anything, dry-fit the three largest components inside the bare enclosure shell (Hammond 1455Q2202, interior ~215 × 120 × 48 mm):

| Component | Approx. dimensions |
|---|---|
| Mean Well LRS-75-24 PSU | 129 × 98 × 38 mm |
| DM542T stepper driver | 118 × 75 × 34 mm |
| Raspberry Pi Pico W | 51 × 21 × 4 mm |

A layout that works: PSU and DM542T side-by-side along the 215 mm axis, with the Pico W on a small perfboard between or above them. If they do not fit comfortably, consider mounting the PSU externally or using a larger enclosure before proceeding.

---

## Step 2 — Prepare the Front Panel

The front panel is a custom-printed aluminum panel from Front Panel Express (122 × 78 mm). Layout (left → right): buttons stacked left of the display, PWR rocker on the right.

```
|←7→|←14→|←5→|←57→|←5→|←22→|←12→|
     BTN-B       DISP        PWR
     BTN-Y
```

| Cutout | Size / shape | Notes |
|---|---|---|
| BTN-B | 14 × 14 mm square | Gateron KS-33 snap-in — top-left of display (GO / STOP / +) |
| BTN-Y | 14 × 14 mm square | Gateron KS-33 snap-in — bottom-left of display (SET / HOME / NEXT) |
| DISPLAY | ~57 × 43 mm rectangle | Adafruit 2.8" active area — measure module before ordering |
| PWR (rocker switch) | 22 × 30 mm rectangle | KCD3 footprint |

BTN-B centre: x = 14 mm, y = 20 mm. BTN-Y centre: x = 14 mm, y = 58 mm. Display centre: x = 54.5 mm, y = 39 mm. PWR centre: x = 99 mm, y = 39 mm.

> **Panel must be ordered at 1.5 mm aluminum** — this is the standard plate thickness Gateron KS-33 switch clips are designed to snap into. Measure the Adafruit 2.8" PCB before finalising the display cutout. Design and print the 3D display bezel after the module arrives.

Mount components into the front panel:
1. Snap the KCD3 rocker switch into its rectangular cutout.
2. Push each Gateron KS-33 switch through its 14×14 mm square hole from the rear until the clips snap onto the panel face. Fit the LP keycap onto the stem.
3. Fit the 3D-printed display bezel and Adafruit 2.0" module.

---

## Step 3 — Prepare the Rear Panel

Drill the following holes in the plastic rear end panel:

| Component | Hole size | Notes |
|---|---|---|
| Schurter 4301.1400 IEC inlet | 28 × 22 mm rectangle | Per Schurter datasheet |
| GX16-4 panel socket (motor) | 16 mm round | Aviation connector |
| Panel-mount micro-USB | Per part datasheet | Rear programming port |

Mount each component:
1. Press-fit / screw the Schurter IEC inlet. Insert a T1A 5×20 mm slow-blow fuse into the holder.
2. Thread the GX16-4 female socket through its hole and tighten the locking nut.
3. Mount the panel-mount micro-USB socket.

---

## Step 4 — Mains Wiring (IEC inlet → Rocker switch → PSU)

> Use 1.5 mm² wire (brown = L, blue = N, green-yellow = PE). Crimp a bootlace ferrule on every wire end before inserting into a screw terminal.

```
IEC C14 inlet (with fuse)
  L (fused) ──► Rocker switch IN ──► Rocker switch OUT ──► PSU L
  N         ──────────────────────────────────────────────► PSU N
  PE        ──────────────────────────────────────────────► PSU PE (earth)
```

- The Schurter 4301.1400 has the fuse inline on the L conductor.
- The rocker switch interrupts the L conductor only.
- Leave 30–40 mm of slack on each wire for routing.
- After wiring, tug each ferrule firmly to confirm it is seated.

---

## Step 5 — DC Power Chain (24 V and 5 V)

### 5a — PSU → DM542T (24 V)

Connect the Mean Well LRS-75-24 +V and -V output screw terminals directly to the DM542T VCC+ and VCC− power terminals using 0.5 mm² hookup wire.

### 5b — PSU → TSR 1-2450 → Pico W VSYS (5 V)

The TSR 1-2450 is a SIP-3 buck regulator (24 V in → 5 V out). Solder it to a small piece of perfboard:

```
PSU +24V ──► TSR pin 1 (Vin)
PSU  GND ──► TSR pin 2 (GND)
TSR pin 3 (Vout, 5V) ──► Pico W VSYS (pin 39)
PSU  GND ──► Pico W GND (pin 38)
```

The Pico W has an internal OR-diode between VSYS and the USB 5 V rail, so the panel-mount USB cable and the TSR 1-2450 can coexist safely.

---

## Step 6 — Stepper Driver Wiring (DM542T → Pico W)

The DM542T uses differential signal inputs (PUL+/−, DIR+/−, ENA+/−). Wire single-ended from the Pico W by tying the + side to 5 V and the − side to the GPIO:

| DM542T terminal | Connect to |
|---|---|
| ENA+ | Pico 5 V rail |
| ENA− | GPIO 3 |
| DIR+ | Pico 5 V rail |
| DIR− | GPIO 4 |
| PUL+ | Pico 5 V rail |
| PUL− | GPIO 5 |
| VCC+ | PSU +24 V |
| VCC− | PSU GND |
| A+, A−, B+, B− | GX16-4 panel socket (pins 1–4) |

> The Pico W GPIO outputs 3.3 V. With the + side tied to 5 V, the differential voltage seen by the DM542T is 5 V in the active state, which meets its input specification.

**Motor output (DM542T → GX16-4 panel socket):**

Solder four wires from the DM542T output terminals (A+, A−, B+, B−) to the GX16-4 female panel socket pins 1–4 respectively. Use the same pin mapping on the GX16-4 male cable-end plug.

---

## Step 7 — Display Wiring (Adafruit 2.8" ILI9341 → Pico W)

Connect using Dupont female-to-female jumper cables (no soldering):

| Adafruit #1770 pin | Pico W GPIO |
|---|---|
| VIN | 3.3 V (pin 36) |
| GND | GND |
| CLK | GPIO 18 |
| MOSI | GPIO 19 |
| CS | GPIO 17 |
| D/C | GPIO 16 |
| RST | not connected (firmware sets RST = −1) |
| LITE | GPIO 20 (backlight) |
| T_CLK, T_CS, T_DI, T_DO, T_IRQ | leave disconnected (touchscreen not used) |

Use the SPI0 bus (GPIO 18/19).

---

## Step 8 — Button Wiring

Both UI buttons are active-low with INPUT_PULLUP in firmware. The Gateron KS-33 has two signal pins and two SMD LED pins on its underside.

| Button | Signal GPIO | LED GPIO | Notes |
|---|---|---|---|
| BTN-B (top, keycap) | GPIO 12 | GPIO 6 (via transistor) | GO / STOP / + |
| BTN-Y (bottom, keycap) | GPIO 13 | GPIO 7 (via transistor) | SET / HOME / NEXT |

**Signal wiring:**
- Solder a short wire from one switch pin → GPIO (Dupont to Pico header)
- Solder a short wire from the other switch pin → GND
- The KS-33 is non-polar for signal; either pin is fine

**LED wiring (transistor circuit on perfboard):**

The KS-33 has two SMD LED pads underneath (+ and −). Drive each via a 2N2222:

```
Pico GPIO 6 ──► 1 kΩ ──► 2N2222 Base
                          2N2222 Collector ──► KS-33 BTN-B LED − pad
                          KS-33 BTN-B LED + pad ──► +5 V
                          2N2222 Emitter   ──► GND

Pico GPIO 7 ──► 1 kΩ ──► 2N2222 Base
                          2N2222 Collector ──► KS-33 BTN-Y LED − pad
                          KS-33 BTN-Y LED + pad ──► +5 V
                          2N2222 Emitter   ──► GND
```

Solder thin wires directly to the SMD LED pads on the underside of each switch.

> This LED control is not yet in the firmware. Add GPIO 6 and GPIO 7 outputs to the state machine (~10 lines) before testing button illumination.

---

## Step 9 — Programming Port

Connect a short micro-B to micro-B USB cable from the panel-mount micro-USB socket (rear panel) to the Pico W USB port. This allows firmware uploads via USB without opening the enclosure.

---

## Step 10 — Firmware Upload

With the enclosure open and the Pico W accessible:

```bash
# Compile
arduino-cli compile --fqbn rp2040:rp2040:rpipico:os=freertos \
  Peeling_Automation_Stepper_Control_Uno/

# Upload (replace port as needed)
arduino-cli upload --fqbn rp2040:rp2040:rpipico:os=freertos \
  --port /dev/ttyACM0 \
  Peeling_Automation_Stepper_Control_Uno/

# Monitor
arduino-cli monitor --port /dev/ttyACM0 --config baudrate=115200
```

Requires the **arduino-pico** core (Earle Philhower) and the **FastAccelStepper** library installed in Arduino CLI.

---

## Step 11 — First Power-On Test (bench, before closing)

Perform these checks with the enclosure open and the PSU connected but the stepper motor not yet plugged in.

1. **Mains safety check:** With power off, use a multimeter to verify continuity L→switch→PSU L, and that PE is connected to the PSU earth terminal.
2. **No-load voltage check:** Power on. Measure PSU output: should be 24 V ±1 V. Measure TSR 1-2450 output: should be 5 V ±0.25 V. Measure Pico W VSYS: should be ~5 V.
3. **Display check:** Display should light up and show the instrument UI within a few seconds of power-on.
4. **Button check:** Press BTN-B (GPIO 12) and BTN-Y (GPIO 13) and confirm the UI responds.
5. **Serial heartbeat:** Connect via USB, open monitor at 115200 baud, confirm `{"state":N,"position":N,"speed":N}` appears every 100 ms.
6. **Stepper check:** Plug in the motor cable. Send `v50` then `m100000` via serial. Motor should move; send `s` to stop.

---

## Step 12 — Close and Label

1. Orient the Pico W so its PCB antenna faces the plastic rear end panel for best WiFi reception.
2. Secure all internal components (PSU feet, DM542T DIN rail or screw mount, perfboard standoffs).
3. Route mains and DC wiring separately where possible.
4. Slide the enclosure body over the internal assembly and attach both end panels.
5. Attach the Front Panel Express aluminum panel to the front end cap.
