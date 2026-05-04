1. when going in to setting and trying to change values the display flickers. can you fix that?
   FIXED: replaced full-area fillRect+redraw with targeted per-row updates. Only the active
   field row is redrawn on value changes; only the old and new rows update on field transitions.
   Title and cal status are drawn once on entry.

2. buttons text are too small. can you make them as large as POS text for example
   FIXED: BTN_W widened 28→52 px; drawButtonBox switched to textSize 2 (12×16 px per char)
   with corrected centering math.

3. when going to start please use the same speed as you do for homing the device.
   FIXED: startMoveToStart() now uses CAL_SPEED_HZ (1067 Hz ≈ 1 mm/s) instead of 100 Hz.
