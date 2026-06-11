# Feature: Rotary Encoder EC11 With Push Button

## Hardware
- **Component**: 360° Rotary Encoder EC11 with push button — bare 5-pin metal component (no breakout board)
- **Pin layout**:

```
3-pin side (encoder):        2-pin side (push button):
  [CLK]  [GND]  [DT]              [SW]  [SW]
    1      2      3                 4     5
```

- **Wiring**:
  - Pin 1 (CLK) → GPIO 14 (was BTN_X)
  - Pin 2 (middle/common) → GND
  - Pin 3 (DT) → GPIO 4 (was BTN_A)
  - Pin 4 or 5 (SW) → GPIO 33 (was BTN_B)
  - Pin 4 or 5 (SW, other) → GND
  - GPIO 12 (was BTN_Y) — left unused
  - **VCC not needed** — ESP32 INPUT_PULLUP provides pull-up; encoder shorts CLK/DT to GND

- **Identify pins**: use a multimeter in continuity mode — pressing the shaft should beep across the 2-pin side (push button). The 3-pin row is always the encoder.

## Compile-time switch

```cpp
#define ROTARY  1
#define BUTTONS 2
#define INPUT_MODE  ROTARY   // change to BUTTONS for 4-button mode
```

Place at the top of the `.ino` file. Comment/change `INPUT_MODE` to switch modes. No other code changes needed.

## Behaviour

### Main screen (IDLE)
- Rotate CW / CCW → move cyan highlight between **GO** and **SET/HOME** buttons (wraps)
- Push (SW) → activates the highlighted button
- Virtual buttons (WiFi / serial bridge) → bypass encoder, trigger actions directly (unchanged)

### Settings screen
- Button boxes are drawn **empty** (no labels) in rotary mode
- Two navigation modes:

  **Browse mode** (default on entry):
  - Rotate CW / CCW → move cyan highlight between settings fields (circular wrap)
  - Fields in order: SPEED → ANGLE → START → CAL → TEMP → WAIT_TEMP → EXIT
  - Push on a numeric field → enter **Edit mode**
  - Push on **CAL** → triggers calibration immediately (no edit mode)
  - Push on **WAIT_TEMP** → toggles value immediately (no edit mode)
  - Push on **EXIT** → saves settings and returns to main screen

  **Edit mode** (entered by pushing on a numeric field):
  - Rotate CW → increment value (1 click = 1 button-press step, same as existing X button)
  - Rotate CCW → decrement value (same as existing Y button)
  - Push → confirm value, return to Browse mode

### Temp sub-menu
Follows the exact same pattern as main settings:
- Browse mode: rotate between fields, push to enter edit or trigger action
- Fields: BACK (direct exit), SETPOINT, KP, KI, KD (all edit mode), STARTSTOP (direct toggle)

### Focus reset
Focus always resets to the **first field / button** (index 0) on every screen transition.

## Visual indicator
Focused button / field gets a **cyan background** with black text — distinct from idle (black bg) and pressed (white border) states.

## Encoder reading
- Polled in `loop()` — no interrupts
- **Software debounce**: ignore CLK transitions faster than 5 ms
- SW push button: existing 500 ms press/release logic, no long-press action

## What does NOT change
- The TFT display layout
- The WiFi web UI
- The serial bridge UI
- Virtual button protocol (`bA1`/`bA0` etc.)
- All existing logic when `INPUT_MODE BUTTONS` is selected
