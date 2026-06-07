# Feature: wait_for_temp — Start Peeling When Temperature is Reached

## Summary

Add a `wait_for_temp` yes/no setting. When enabled, pressing GO moves the motor to the
start position and then holds in a new `WAITING_FOR_TEMP` state until the thermocouple
reads within ±2 °C of the setpoint before starting the peel stroke.

## Agreed Design Decisions

| # | Question | Decision |
|---|---|---|
| 1 | Auto-activate temp control on GO? | No — user controls it manually; GO only auto-enables if `wait_for_temp=YES` and control is OFF |
| 2 | `wait_for_temp=YES` but `tempControlActive=OFF`? | Auto-enable temp control (reset PID), set `tempCtrlAutoEnabled=true`, proceed |
| 3 | Timeout if temp never arrives? | None — A-button abort works as today |
| 4 | State machine | New `WAITING_FOR_TEMP` state between `MOVING_TO_START` and `PEELING` |
| 5 | "Reached" threshold | ±2 °C — reuses existing `stable` definition |
| 6 | Settings placement | Top-level field after `FIELD_TEMP`: `SPEED → ANGLE → START → CAL → TEMP → WAIT_TEMP → [exit+save]` |
| 7 | UI during wait | State label = `"WAIT TEMP"` (yellow); right-column temp bar already shows progress |
| 8 | On peel end | Auto-disable heater only if `tempCtrlAutoEnabled`; leave on if user enabled it manually |

## Implementation Checklist

- [x] `AppState` enum: add `WAITING_FOR_TEMP`
- [x] `SettingsField` enum: add `FIELD_WAIT_TEMP`
- [x] New globals: `bool waitForTemp`, `bool tempCtrlAutoEnabled`
- [x] `cycleSettingsField()`: insert `FIELD_WAIT_TEMP` before exit
- [x] Button handler `SETTINGS`: `X=YES`, `Y=NO` for `FIELD_WAIT_TEMP`
- [x] `startMoveToStart()`: if `waitForTemp && !tempControlActive` → auto-enable PID + set flag
- [x] `MOVING_TO_START` case: on motor stop, go to `WAITING_FOR_TEMP` instead of calling `startPeeling()`
- [x] New `WAITING_FOR_TEMP` case: check ±2 °C → call `startPeeling()`
- [x] `abortAndIdle()` + `PEELING → IDLE`: if `tempCtrlAutoEnabled`, turn heater off + clear flag
- [x] EEPROM: bump magic `PEL5 → PEL6`, add `waitForTemp` field
- [x] Settings display: render `FIELD_WAIT_TEMP` row as `"WAIT:YES"` / `"WAIT:NO"`
- [x] Heartbeat JSON: add `"wait_for_temp"` bool field
- [x] Web UI (PROGMEM + `serial_bridge/index.html`): add `WAIT_TEMP` to `STATE_COL` (yellow), render state label
