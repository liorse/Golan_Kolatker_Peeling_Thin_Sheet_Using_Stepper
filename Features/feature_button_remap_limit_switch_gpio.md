# Corrections 9 — Button Remapping & Limit Switch GPIO

## Changes implemented

### New pin assignments
- `BTN_X` (GPIO 14) — now a UI button: **+** in settings, triggers CAL on CAL field, no-op outside settings
- `BTN_Y` (GPIO 12) — now a UI button: **−** in settings, no-op outside settings
- `LIMIT_SW` (GPIO 13) — **new**: both limit switches wired in parallel to this single pin (active-low, INPUT_PULLUP). Will be split to GPIO 32 and GPIO 33 in a future revision.

### Settings menu button mapping (new)

| Button | Action |
|--------|--------|
| A | Navigate **up** through fields (no-op on first field: SPEED) |
| B (short press) | Navigate **down**; exits and saves when leaving the last field (CAL) |
| X (press / hold) | **+** increment with auto-repeat; on CAL field → triggers calibration run |
| Y (press / hold) | **−** decrement with auto-repeat; no-op on CAL field |

### Outside settings
- X and Y are **no-ops** in all non-settings states.
- A and B behaviour in IDLE / moving states is unchanged.

### Display
- TFT button labels in settings: A=`UP`, B=`DOWN`, X=`+`/`CAL`, Y=`-`
- Hint line (`tap=NEXT  hold=-`) **removed** from both TFT and web UI.
- Web UI button labels are dynamic: show `+`/`-` in settings, `X`/`Y` otherwise.
