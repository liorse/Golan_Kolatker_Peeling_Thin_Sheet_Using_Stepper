## New 2.8" 240×320 ST7789 display — decisions

**Goal:** swap in the 2.8" 240×320 RGB display (same ST7789 interface), rotate it CW 90°, and fix the inverted colors.

### Decisions

| Question | Decision |
|----------|----------|
| Orientation after rotation | Landscape (320 wide × 240 tall) |
| Connector edge in landscape | Right edge → `setRotation(3)` |
| Layout changes | None for now; content stays in left 240×240 region |
| Panel offset | No offset — start with `tft.init(240, 320)` and check |

### Color inversion explanation

The ST7789 chip has a display-inversion register (INVON/INVOFF). Different panel manufacturers ship with it in different states. The old 240×240 module had it off; the new 240×320 module has it on, causing black↔white swap. Fix: `tft.invertDisplay(false)` after `tft.init()` — the Adafruit ST7789 `init()` already calls `invertDisplay(true)` internally, so we override it with `false`.

### Code changes (setup())

```cpp
// Before
tft.init(240, 240);
tft.setRotation(2);

// After
tft.init(240, 320);
tft.setRotation(3);
tft.invertDisplay(false);
```

### Future work
- Extend layout to use the full 320px width (extra 80px currently blank on the right).
