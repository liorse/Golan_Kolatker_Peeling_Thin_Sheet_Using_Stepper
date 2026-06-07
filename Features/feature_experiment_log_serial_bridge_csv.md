# Feature: Experiment Log in the Serial Bridge

## Request

Add a logging function to the Python serial bridge so that when GO is pressed (i.e. the
instrument enters the PEELING state) the position-vs-time data is automatically recorded
along with all experiment metadata (angle, speed).  The web UI should show a live
indicator of the active log filename and save folder.

---

## Design Decisions (resolved)

| # | Topic | Decision |
|---|-------|----------|
| 1 | Log trigger — start | First heartbeat where `state == "PEELING"` → open new CSV file. No firmware change needed. |
| 2 | Log trigger — stop | First heartbeat where state *leaves* `"PEELING"` → write final row then close. |
| 3 | Re-trigger | If PEELING arrives while a file is already open, close the old file first then open a new one. |
| 4 | Disconnect safety | Serial disconnection also calls `_log_close()` so no data is lost mid-run. |
| 5 | File format | **CSV** (stdlib `csv` module — no new dependency). |
| 6 | File location | `serial_bridge/logs/` subdirectory (auto-created). |
| 7 | Filename | `peel_YYYYMMDD_HHMMSS_angNN_spdXpX.csv` — timestamp + key experiment params encoded in the name. |
| 8 | CSV columns | `timestamp` (ISO-8601 wall clock), `time_ms` (`peel_elapsed_ms` from firmware), `pos_um`, `speed_um_s` (actual), `state`. |
| 9 | Metadata | Angle and set-speed live in the filename only; no redundant header or extra columns. |
| 10 | Sample rate | Every heartbeat while in PEELING state ≈ 10 Hz (100 ms interval). |
| 11 | Console output | One line on open (`📄 Logging → …`), one line on close (`✓ Log closed — N rows written`). |
| 12 | UI indicator | Python bridge injects `log_info` into every WebSocket heartbeat. JS renders a `#log-status` div below the serial badge: filename in yellow + save folder in grey. Hidden when not logging. |
| 13 | PROGMEM sync | `serial_bridge/index.html` and the PROGMEM HTML block in the `.ino` kept identical. |

---

## CSV Layout

```
timestamp,time_ms,pos_um,speed_um_s,state
2026-05-27T14:30:22.143,0,500.00,1.02,PEELING
2026-05-27T14:30:22.243,100,501.02,1.03,PEELING
…
2026-05-27T14:30:45.843,23700,1987.44,0.00,IDLE
```

## Example Filename

```
logs/peel_20260527_143022_ang30_spd1p0.csv
```

## UI Indicator (serial bridge only)

While PEELING, below the "🔌 SERIAL connected" line:

```
📝 peel_20260527_143022_ang30_spd1p0.csv
→ logs
```

Filename shown in yellow; folder path in grey.  Element is empty (zero height) when not logging.
Hidden on WiFi path (ESP32 never sends `log_info`).

---

## Files Modified

| File | Change |
|------|--------|
| `serial_bridge/serial_bridge.py` | `_log_open/row/close/dispatch` helpers; `log_info` injected into every broadcast; dispatch reordered before broadcast so status is accurate on the same frame |
| `serial_bridge/index.html` | `#log-status` CSS + div + JS in `render()` |
| `Peeling_Automation_Stepper_esp32/Peeling_Automation_Stepper_esp32.ino` | PROGMEM block kept in sync with `index.html` |
