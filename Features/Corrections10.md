# Corrections 10 — UI Polish & Microstep Simplification

## Changes implemented

### 1. X and Y buttons hidden on main screen
- Outside the settings menu, the X and Y button boxes are erased (filled solid black, no border).
- They only appear when in settings, showing `+` / `CAL` and `-`.

### 2. UP / DOWN button font size
- In the settings menu, the A (`UP`) and B (`DOWN`) button labels now use text size 2, matching the size of `GO`, `STOP`, and `SET`/`HOME` on the main screen.

### 3. Microstep setting removed from UI; value fixed at 25600
- `FIELD_STEPS` removed from the `SettingsField` enum.
- Settings navigation is now: **Speed → Angle → Start → CAL → exit+save**.
- `steps_per_rev` is a compile-time constant (`25600`). It is no longer stored in or loaded from EEPROM.
- EEPROM magic bumped from `PEL3` → `PEL4`; existing saved settings are cleared on first boot. **Calibration must be re-run after flashing.**
- Settings field Y-positions spread slightly (24 px spacing instead of 22 px) now that there are only 4 rows.
- Web UI updated to match: 4-field loop, `settings_field===3` is CAL, X/Y boxes blank outside settings.
