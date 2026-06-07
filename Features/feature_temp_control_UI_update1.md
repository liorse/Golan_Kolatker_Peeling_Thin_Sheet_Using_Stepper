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

---

## Follow-up Session — Bug Fixes & Layout Polish

### Bug: Black screen after exiting settings

**Root cause:** `updateRunContent()` uses static `prev*` variables to skip unchanged
rows. When B navigates from FIELD_TEMP back to IDLE, `clearContent()` wipes the left
panel, but all statics still matched the old IDLE values → every `strcmp`/`!=` check
passed and nothing was redrawn.

**Fix:** Added global `bool runScreenDirty = false;`. In the 100 ms heartbeat, when
the screen transitions settings → run, set `runScreenDirty = true`. At the top of
`updateRunContent()`, if the flag is set, reset all statics to their sentinel values
and clear the flag — forcing a full redraw in the same tick.

### Bug: Flicker and erasure of vertical divider during run screen

**Root cause:** Two rows overflowed past `LEFT_W = 200 px` into the right temperature
column on every heartbeat tick:

| Row | Start x | Chars × 12 px | End x | Overflow |
|-----|---------|--------------|-------|---------|
| State label | 0 | 20 × 12 = 240 | 240 | 40 px |
| POS / warning | 6 | 19 × 12 = 228 | 234 | 34 px |

The overflow overwrote the vertical divider line and portions of the temperature bar.
Because `updateTempColumn()` uses dirty-state tracking (not a full redraw), the damage
persisted until the next partial repaint, causing visible flicker and ghosting.

**Fix:**
- State label reduced to **16 chars** centred in the left column
  (`16 × 12 = 192 px` from x = `(200 − 192) / 2 = 4`; ends at x = 196).
- POS normal row: `"um      "` (8 chars) → `"um   "` (5 chars);
  total 4 + 7 + 5 = 16 chars from x = 6; ends at x = 198.
- POS warning row: `"!CAL FIRST!        "` (19 chars) → `"!CAL FIRST!     "` (16 chars);
  from x = 6, ends at x = 198.
- `prevSb` buffer shrunk from 22 → 18 bytes to match.

### Layout: Centred state label

The 16-char state string is now drawn starting at x = 4 (formula:
`(LEFT_W − 16 × 12) / 2 = 4`) instead of x = 0. Both firmware and JS updated.

### Layout: Centred WiFi icon and client count

**Problem:** WiFi icon was at `cx = 120`, client count at x = 136. With 2-digit counts
the text reached x = 148, overlapping the top-right button box (x = 145..197).

**Decision:** Move icon to `cx = 100` — the geometric centre of the 90 px gap between
the two top button boxes (left button ends at x = 55, right button starts at x = 145;
centre = (55 + 145) / 2 = 100).

| Element | Old | New |
|---------|-----|-----|
| WiFi icon centre | cx = 120 | cx = 100 |
| Icon span | x = 106..134 | x = 86..114 |
| Client count x | 136 | 116 |
| Client count max (2 digits) | 148 (clips button) | 128 (clear of button at 145) |

Both firmware (`drawWifiIcon()` + heartbeat client-count draw) and JS updated.

## All Files Changed (cumulative)

- `Peeling_Automation_Stepper_esp32/Peeling_Automation_Stepper_esp32.ino`
  - `#define TBAR_X 209`, `#define TBAR_W 40`
  - `updateTempColumn()` — dirty-state tracking
  - `updateRunContent()` — `runScreenDirty` reset, 16-char state label at x=4,
    shortened POS/warning rows, `prevSb[18]`
  - `drawWifiIcon()` — `cx = 100`
  - Heartbeat client-count draw — `fillRect(114,...)`, `setCursor(116,...)`
  - Global `bool runScreenDirty`
  - JS in PROGMEM — `cx=100`, client count at x=116, state label 16 chars at x=4,
    POS/warning rows shortened to 16 chars
- `serial_bridge/index.html` — same JS changes as PROGMEM copy
- `serial_bridge_electron/index.html` — same JS changes (WiFi icon cx=100,
  client count at x=116, state label 16 chars at x=4, POS/warning rows shortened)
