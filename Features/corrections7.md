In the UI, when I press set the lower button shows the text next and the upper button shows in the specific situation + and in the cal option cal. all is fine. But, I want to alert the user that he can also press a long press on the lower button to function as a - and not just a next for a short press.

IMPLEMENTED: Two-pronged discoverability fix for the B button long-press decrement in Settings:

1. Button label — the lower-left button (B) now shows "NEXT/-" (textSize 1) instead of "NEXT" (textSize 2), communicating both the short-press (NEXT) and long-press (-) actions directly on the button.

2. Hint line — "tap=NEXT  hold=-" is drawn at y=56 (between the "SETTINGS" title and the first field) in cyan at textSize 1. It disappears when the CAL field is active (where long-press B has no effect) and reappears on any other field.

Implementation: `drawButtonBox` gained an optional `sz` parameter (default 2) for textSize. `updateButtons` passes `bSz=1` and label `"NEXT/-"` for the B button in SETTINGS. A new `drawSettingsHint(fieldIdx)` helper draws or clears the hint line; called from `updateSettingsContent` on first draw and on every field change.