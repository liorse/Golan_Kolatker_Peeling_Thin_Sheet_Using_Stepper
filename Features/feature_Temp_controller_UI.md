# Temperature Controller UI

## Resolved Design (grilling session 2026-06-07)

---

## Layout

- Canvas expands to **320×240** — the physical ST7789 display is already 320 wide; the firmware previously used only the centre 240 px.
- **Left column** (x 0–199, 200 px): existing stepper UI — run screen and settings screen, content unchanged except minor horizontal squeeze.
- **Right column** (x 200–319, 120 px): temperature bar, visible on **both** Run and Settings screens at all times.
- Vertical cyan divider separates the two columns.
- The existing `T: xx.x C` text row is **removed** from the left column (replaced by the right-column bar).

---

## Right Column — Temperature Bar

Top to bottom:

1. **Status indicator** (y ≈ 4)
   - `CTRL:OFF` in gray — loop not running
   - `CTRL: ON` blinking yellow at 500 ms — loop active, not yet stable
   - `STABLE` solid green — loop active and within ±2 °C of setpoint
2. **Power %** (y ≈ 18) — `PWR: xx%` (heater_duty / 255 × 100), cyan
3. **Vertical thermometer bar** (y 46–220, height 174 px)
   - Fills **bottom-to-top** (0 °C at bottom, 120 °C at top)
   - **Red** fill when not stable, **green** fill when stable
   - On fault (`temp_c = null`): entire bar red, shows `FLT`
4. **Setpoint line** — white horizontal line across the bar at the setpoint temperature, **always visible** regardless of control state
5. **Current temperature label** — floats to the right of the bar, near the top of the filled portion (`xx.x` format)

---

## Settings Screen

Main settings gains a 5th entry: **TEMP SETTINGS** (press `+`/`OPEN` to enter sub-menu).

### TEMP sub-menu fields (6 entries, navigated with A=up / B=down)

| # | Field | X button | Y button |
|---|-------|----------|----------|
| 0 | `< BACK` | `BACK` — exits sub-menu | no-op |
| 1 | Setpoint | `+` / increment | `−` / decrement |
| 2 | Kp | `+` | `−` |
| 3 | Ki | `+` | `−` |
| 4 | Kd | `+` | `−` |
| 5 | Start/Stop | `ON` — enable control | `OFF` — disable control |

Pressing B on the last field (Start/Stop) exits the sub-menu back to main settings on the TEMP entry.

---

## Defaults, Ranges, and Steps

| Parameter | Default | Range | Step |
|-----------|---------|-------|------|
| Setpoint | 50 °C | 0–120 °C | 1 °C |
| Kp | 60 | 0–200 | 1 |
| Ki | 0.15 | 0–2.0 | 0.01 |
| Kd | 900 | 0–2000 | 10 |

PID rationale: slow thermal system with large thermal mass. Kp=60 avoids slamming. Ki=0.15 (Ti ≈ 400 s) is patient. Kd=900 (Td ≈ 15 s) applies early braking to prevent overshoot.

**Stability threshold**: ±2 °C (matches the planned bang-bang hysteresis; also tolerates MAX31856 noise floor).

---

## Firmware Additions (this step — control loop deferred)

| Item | Detail |
|------|--------|
| New variables | `float tempSetpoint`, `bool tempControlActive`, `float kp`, `float ki`, `float kd` |
| EEPROM | Added to `SavedSettings`; magic bumped to `"PEL5"` |
| JSON heartbeat | `temp_setpoint`, `temp_ctrl_active`, `kp`, `ki`, `kd`, `in_temp_sub`, `temp_field` |
| Control loop | **Deferred to next step** |

---

## What is NOT in this step

- PID control loop execution (no output calculation, no integral/derivative accumulation)
- Automatic heater PWM from the loop (heater still driven by `h<duty>` serial command or manual override)


