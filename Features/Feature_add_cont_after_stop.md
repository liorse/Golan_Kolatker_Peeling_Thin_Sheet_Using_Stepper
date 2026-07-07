# Feature: Continue After Stop

## Original Request

Change the behaviour when a user presses stop. Add an option to continue the process from exactly where it stopped. Available as a button in all interfaces: TFT display, electron app, and WiFi web UI.

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Pauseable state | PEELING only | HOMING and CAL need clean end positions; MOVING_TO_START is cheap to re-run |
| New state | `PAUSED` added to `AppState` enum | Explicit state, no ambiguity between "idle" and "paused mid-peel" |
| Stop during PEELING | New `pauseAndWait()` | Stops + disables motor, keeps `hasHomed = true`, keeps temp control active |
| Trigger for CONT | A button (same slot as STOP) | STOP label transforms to CONT when paused — no confusion with GO |
| A in PAUSED | **CONT** — resume from current position to `dist_xa_steps` | Re-enables motor (500 ms ENA- delay built in), calls `moveTo(dist_xa_steps)` at `speed_um_s` |
| B in PAUSED | **HOME** — calls `abortAndIdle()` + `startHoming()` | Only way to abandon peel and start fresh; turns off temp control |
| X, Y in PAUSED | no-op | Not needed |
| `hasHomed` on pause | Keep `true` | FastAccelStepper position counter stays valid; CONT can resume directly |
| Temp control on pause | Stay active | Sheet is still on the heated surface; maintain setpoint during pause |
| Go (fresh start) from PAUSED | Not available | Must HOME first, then GO — prevents accidental overwrite of paused position |
| TFT display text | "PAUSED" | Explicit, unambiguous |
| Web UI CONT button position | Same slot as STOP (A-slot) | Mirrors physical button exactly |
| Web UI CONT button color | Orange | Distinct from GO (green) and STOP (red); signals "resume a paused operation" |

## State Machine Change

```
PEELING → [A pressed] → PAUSED   (new: pauseAndWait)
PAUSED  → [A pressed] → PEELING  (new: resumePeeling)
PAUSED  → [B pressed] → HOMING   (abortAndIdle + startHoming)
```

## Implementation Notes

- `pauseAndWait()`: `stepper->stopMove()` + wait + `disableMotor()` — does NOT set `hasHomed = false`, does NOT touch temp control
- `resumePeeling()`: `enableMotor()` (includes `delay(500)`) + `stepper->setSpeedInMilliHz(...)` + `stepper->moveTo(dist_xa_steps)` → `appState = PEELING`
- JSON heartbeat emits `"PAUSED"` so web/electron renders correct button states
- Web UI reads state and swaps A-slot button: red STOP → orange CONT
