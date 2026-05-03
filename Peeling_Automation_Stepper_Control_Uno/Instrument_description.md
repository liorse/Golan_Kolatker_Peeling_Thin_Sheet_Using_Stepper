# Peeling Instrument — Design Specification

## Hardware

- **MCU:** Raspberry Pi Pico (RP2040) on Pimoroni Pico Explorer Base
- **Motor:** NEMA 17 stepper driven by DM542T driver
- **Display:** 1.54" 240×240 IPS ST7789
- **Buttons:** A (GPIO 12), B (GPIO 13), X (GPIO 14), Y (GPIO 15) — active-low, INPUT_PULLUP

### Button Physical Roles

| Button | Physical role |
|--------|--------------|
| X | Limit switch — home end (simulated by button, wired to real microswitch) |
| A | Limit switch — far end (simulated by button, wired to real microswitch) |
| Y | User interface — START / STOP / increment setting |
| B | User interface — SETTINGS / HOME / cycle setting field |

---

## Unit Conversion

- **Step size:** 1 step = 0.9375 µm (measured)
- **Angle:** user-set tilt of stage in degrees (0–89°; 90° disallowed — cos(90°) = 0)
- **Distance formula:** `distance_µm = steps × 0.9375 × cos(angle_rad)`
- **Speed formula:** `speed_µm_s = steps_per_s × 0.9375 × cos(angle_rad)`
- **Inverse (µm → steps):** `steps = distance_µm / (0.9375 × cos(angle_rad))`

---

## Coordinate Convention

- **Position 0** = microswitch X (home end)
- **Positive direction** = toward microswitch A (far end)
- **distance_XA** = calibrated total travel in steps (X→A); stored in flash

---

## State Machine

```
IDLE ──[Y, dist_XA > 0]──────────────► MOVING_TO_START
IDLE ──[Y, dist_XA == 0]─────────────► show "RUN CAL FIRST" (no movement)
IDLE ──[B, position == 0]────────────► SETTINGS
IDLE ──[B, position > 0]─────────────► HOMING

MOVING_TO_START ──[arrival]──────────► 100 ms pause ──► PEELING
MOVING_TO_START ──[Y]────────────────► IDLE (abort)
MOVING_TO_START ──[X triggered]──────► HOMING (safety)
MOVING_TO_START ──[A triggered]──────► IDLE (safety stop)

PEELING ──[Y]────────────────────────► IDLE (user stop)
PEELING ──[A triggered]──────────────► IDLE (end stop reached)
PEELING ──[X triggered]──────────────► IDLE (safety stop)

HOMING ──[X triggered]───────────────► IDLE (forceStop, setCurrentPosition(0), disable outputs)
HOMING ──[Y]─────────────────────────► IDLE (abort)

SETTINGS ──[B cycles fields]─────────► angle → speed → start_pos → CAL → (exit to IDLE, auto-save)
SETTINGS ──[Y short]─────────────────► increment current field
SETTINGS ──[Y hold 500 ms]───────────► fast-increment current field
SETTINGS ──[B long 500 ms]───────────► decrement current field
SETTINGS (on CAL field) ──[Y]────────► CAL sequence

CAL ──[homing complete]──────────────► run at 100 steps/s toward A
CAL ──[A triggered]──────────────────► record steps, save dist_XA, → IDLE
CAL ──[Y]────────────────────────────► abort, → IDLE (no save)
```

---

## Button Labels (dynamic per state)

| State | Y label | B label |
|-------|---------|---------|
| IDLE (position == 0) | START | SET |
| IDLE (position > 0) | START | HOME |
| MOVING_TO_START | STOP | — |
| PEELING | STOP | — |
| HOMING | STOP | — |
| SETTINGS | (increment) | (cycle/decrement) |
| CAL | START/ABORT | — |

---

## Settings Fields (cycled by B)

| Field | Range | Increment (short Y) | Fast increment (hold Y) |
|-------|-------|---------------------|------------------------|
| Angle | 0–89° | 1° | — (small range) |
| Speed | 1–1000 µm/s | 1 µm/s | 10 µm/s |
| Start position | 0–dist_XA µm | 10 µm | 100 µm |
| CAL | — | triggers calibration | — |

Long-press B (500 ms) decrements the current field (angle/speed/start_pos).
Exiting CAL field (B cycles past it) returns to IDLE and auto-saves all settings.

---

## Motor Control Details

- **Homing speed:** 100 steps/s (≈ 93.75 µm/s at 0°)
- **MOVING_TO_START speed:** 100 steps/s
- **Peeling speed:** set by user (1–1000 µm/s), converted to steps/s
- **Transition at start position:** motor stops, 100 ms pause, then begins peeling
- **Acceleration:** `setAcceleration(2147483647)` — instant (no ramp)
- **ENA- timing:** `enableOutputs()` + 500 ms delay before first move (DM542T datasheet requirement); not repeated if motor is already enabled
- **Limit detection:** polling in `loop()` (reaction time ~1 ms at 100 steps/s → < 0.1 µm overshoot)

---

## Calibration Sequence (CAL)

1. Run homing sequence (motor runs negative at 100 steps/s until X triggers → set position = 0)
2. Run at 100 steps/s in positive direction toward A
3. When A triggers: record `dist_XA_steps = getCurrentPosition()`
4. Save `dist_XA_steps` to `Preferences`
5. Return to IDLE

If Y is pressed during CAL: abort immediately, do not save, return to IDLE.

---

## Persistent Storage (`Preferences` library)

| Key | Type | First-boot default |
|-----|------|--------------------|
| `angle` | int | 30 |
| `speed_um` | float | 1.0 µm/s |
| `start_um` | float | 0.0 µm |
| `dist_xa` | int32 | 0 (not calibrated) |

All values saved when settings menu exits (B cycles past CAL back to IDLE).
`dist_xa` saved only after successful calibration run.

---

## Display Layout (240×240, content area y=34–206)

### Run screen

```
┌─[Y: START/STOP]──────────────[X: LIMIT]─┐
├──────────────────────────────────────────┤
│            PEELING           (size 2)    │  state
│  POS:   1234.5 µm            (size 2)    │  current position
│  SET:    93.8 µm/s           (size 1)    │  set speed
│  RUN:    93.8 µm/s           (size 1)    │  actual running speed
│  ANGLE:  30°                 (size 1)    │  tilt angle
│  TO END: 12.3 s              (size 1)    │  time to microswitch A (-- if not peeling)
│  PEEL T: 00:05               (size 1)    │  elapsed peel time (-- if not peeling)
│  [█████████████░░░░░░░░░░░]             │  progress bar (pos / dist_XA)
├──────────────────────────────────────────┤
└─[B: SET/HOME]────────────────[A: LIMIT]─┘
```

### Settings screen

Active field highlighted in yellow; others in cyan. Value shown large (size 2) with field name above it (size 1).

---

## Serial Interface (preserved)

**Commands (115200 baud):**
- `m<int32>` — move to absolute step position (clamped to [0, dist_XA])
- `s` — stop immediately
- `v<int>` — set speed in Hz (steps/s)

**JSON heartbeat (every 100 ms):**
```json
{"state":N,"position":N,"speed":N,"pos_um":F,"speed_um":F,"angle":N}
```

| Field | Description |
|-------|-------------|
| `state` | 1=INIT, 2=MOVING, 3=WAITING |
| `position` | current position in steps |
| `speed` | current speed in steps/s |
| `pos_um` | current position in µm |
| `speed_um` | current speed in µm/s |
| `angle` | current angle in degrees |

---

## Safety Rules

1. If `dist_XA == 0` (never calibrated): Y (START) is blocked; display shows `RUN CAL FIRST`
2. Start position is clamped to `[0, dist_XA_µm]` at input time
3. Angle capped at 89° (prevents division by zero in µm↔steps conversion)
4. Both limit switches (X and A) stop the motor in any MOVING or PEELING state
