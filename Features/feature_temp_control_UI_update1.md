# Temperature Column UI Update 1

## Original Problems

1. Flickering in the temp part of the display
2. Residual lines when changing setpoint
3. Setpoint line flickering
4. Bar too small; requested bigger and centered
5. Show setpoint temperature above the setpoint line
6. Make temp area as small as possible (more room for peeling window)
7. Fonts in the temp column too small

## Design Decisions (resolved via grilling)

### Flicker fix (items 1, 3)
**Decision:** dirty-state tracking — `updateTempColumn()` now stores `prevFillH`,
`prevSpY`, `prevBarColor`, `prevHeaterDuty`, `prevCtrlStatus`, `prevTempDrawn`,
`prevTempLabelY`, and `prevFault` as static locals. Each section only repaints
pixels when the relevant value changes.

### Residual setpoint lines (item 2)
**Decision:** track `prevSpY`. On setpoint change: erase old line at full width
(`TBAR_X - 4`, length `TBAR_W + 8` — includes the 4 px overhang that caused the
ghost lines), clear the entire label column (`fillRect` from `labelX` to screen
right), then redraw the new line and label. Force bar redraw to restore any fill
pixels overwritten by the erase.

### Bar size and position (item 4)
**Decision:** `TBAR_W` widened from 22 → 40 px. `TBAR_X` shifted from 205 → 209
to center the bar+label block within the 119 px right column
(margin = (119 − 103) / 2 = 8 px; 103 = 40 bar + 3 gap + 60 label).

### Item 6 — column width
**Decision:** keep floating temp label to the right of the bar (user preference).
Column width unchanged at 119 px. The wider bar (40 px) plus size-2 label
consumes 103 px, leaving 8 px margin each side — visually centered.

### Setpoint label (item 5)
**Decision:** show setpoint value (e.g. `"50C"`) in size-2 to the right of the
bar at `y = spY − 17` (one text-row above the setpoint line), clamped to
`[TBAR_TOP, TBAR_BOT − 16]`.

### Temp label vs setpoint label collision
When temperature approaches setpoint the two labels would overlap. Resolution:
if `|tempLabelY − spY| < 12`, push the temp label to `spY + 2` (just below the
setpoint line). Both labels remain visible; setpoint label stays above the line.

### Fonts (item 7)
**Decision:** `setTextSize(2)` for all right-column text (status, PWR%, temp
label, setpoint label). All text fits within the 119 px column at size-2.

## Files Changed

- `Peeling_Automation_Stepper_esp32/Peeling_Automation_Stepper_esp32.ino`
  - `#define TBAR_X 209` (was 205)
  - `#define TBAR_W  40` (was 22)
  - `updateTempColumn()` fully rewritten with dirty-state tracking
  - JS constants in PROGMEM string updated: `TBAR_X=209,TBAR_W=40`
- `serial_bridge/index.html`
  - JS constants updated: `TBAR_X=209,TBAR_W=40`
