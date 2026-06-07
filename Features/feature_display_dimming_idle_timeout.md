# Display Dimming — Feature Spec

## Request
Dim the display after 20 seconds of idle activity (no external button pressed, no WiFi/WebSocket command received). Light it back up if a button is pressed or a WiFi command is sent.
Dim to 20% brightness.

## Decisions

| # | Question | Decision |
|---|----------|----------|
| 1 | Wake-on-press behaviour | **Wake + act** — the button press both restores brightness and executes its action (no double-press required) |
| 2 | Motor running = idle? | **Dim even during motor movement** — active PEELING/HOMING/MOVING states do not reset the timer |
| 3 | Serial commands | **Yes** — serial `m`, `s`, `v` commands reset the idle timer and wake the display |
| 4 | Timeout configurable? | **Hardcoded** `#define DIM_TIMEOUT_MS 20000` — not added to the Settings screen |
| 5 | Transition style | **Gradual fade** 255 → 51 over 1 second (linear, every loop iteration) |
| 6 | Wake transition | **Instant snap** to full brightness — no fade-up |

## Implementation Notes
- Brightness controlled via ESP32 LEDC PWM on GPIO 16 (TFT_BL), core 3.x pin-based API (`ledcAttach` / `ledcWrite`)
- `BL_FULL = 255`, `BL_DIM = 51` (20% of 255)
- Activity sources: physical buttons, WebSocket messages (via `wsActivityFlag` volatile bool), serial commands
- `wakeDisplay()` helper: snaps to full brightness + resets `lastActivityMs`
- Dim state machine: `displayFading` → linear interpolation → `displayDimmed`
