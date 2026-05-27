# Feature: Experiment Log in the Serial Bridge

## Request

Add a logging function to the Python serial bridge so that when GO is pressed (i.e. the
instrument enters the PEELING state) the position-vs-time data is automatically recorded
along with all experiment metadata (angle, speed).

---

## Design Decisions (resolved)

| # | Topic | Decision |
|---|-------|----------|
| 1 | Log trigger — start | First heartbeat where `state == "PEELING"` → open new CSV file. No firmware change needed. |
| 2 | Log trigger — stop | First heartbeat where state *leaves* `"PEELING"` → close file, print summary. |
| 3 | Re-trigger | If PEELING arrives while a file is already open, close the old file first then open a new one. |
| 4 | File format | **CSV** (stdlib `csv` module — no new dependency). |
| 5 | File location | `serial_bridge/logs/` subdirectory (auto-created). |
| 6 | Filename | `peel_YYYYMMDD_HHMMSS_angNN_spdX.X.csv` — timestamp + key experiment params encoded in the name. |
| 7 | CSV columns | `timestamp` (ISO-8601 wall clock), `time_ms` (`peel_elapsed_ms` from firmware), `pos_um`, `speed_um_s` (actual), `state`. |
| 8 | Metadata | Angle and set-speed live in the filename only; no redundant header or extra columns. |
| 9 | Sample rate | Every heartbeat while in PEELING state ≈ 10 Hz (100 ms interval). |
| 10 | Console output | One line on open (`📄 Logging → …`), one line on close (`✓ Log closed — N rows written`). |
| 11 | Scope | `serial_bridge.py` only — no firmware change, no UI change. |

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
logs/peel_20260527_143022_ang30_spd1.0.csv
```

---

## Files Modified

| File | Change |
|------|--------|
| `serial_bridge/serial_bridge.py` | Add `_log_*` helpers; hook into heartbeat dispatch loop |
