# Feature: Serial-Bridge Web UI

## Request

Serve the same web UI that is currently served by the ESP32 over WiFi, but this time the
communication channel is the USB serial port.  The user runs a Python program on their
computer which:

1. Connects to the ESP32 over serial (USB/UART).
2. Serves the same webpage on `http://localhost:8080`.
3. Lets the user control the instrument through the identical web UI.

Parallel operation must be preserved: WiFi WebSocket and serial bridge can be active at the
same time.  Physical buttons + TFT display continue to function normally.

---

## Design Decisions (resolved)

| # | Topic | Decision |
|---|-------|----------|
| 1 | ESP32 firmware change | Add `bA1`/`bA0` serial commands for virtual button press/release injection.  Format: `b` + letter (`A`/`B`/`X`/`Y`) + `1` (press) or `0` (release).  Parsed with two `Serial.read()` calls after the `b` command character.  Keeps the entire state machine (settings, homing, CAL) in the firmware — full parity with WiFi mode. |
| 2 | Python stack | `asyncio` + `websockets` + `pyserial`.  Single Python file.  Two pip packages beyond stdlib. |
| 3 | Serial port selection | Auto-detect CH340 USB-serial adapter by VID `1A86` / PID `7523`.  If multiple CH340 ports are found, pick the first and print the full list as info.  User can override with a CLI positional argument: `python serial_bridge.py /dev/ttyUSB1`. |
| 4 | File layout | `serial_bridge/serial_bridge.py` + `serial_bridge/requirements.txt`. |
| 5 | HTTP / WS port | 8080 (default).  Can be overridden with a CLI argument. |
| 6 | HTML source of truth | `serial_bridge/index.html` — one canonical copy. |
| 7 | ESP32 HTML serving | Keep PROGMEM string in the sketch (reverted from LittleFS).  HTML is stored as `static const char HTML_PAGE[] PROGMEM = R"rawhtml(...)rawhtml"` in the `.ino` file.  Served via `req->send_P(200, "text/html", HTML_PAGE)`.  Python bridge reads `serial_bridge/index.html` directly from disk.  Both copies must be kept in sync when editing the UI. |
| 8 | LittleFS upload method | ~~Not needed — reverted to PROGMEM.~~  No separate filesystem flash step required. |
| 9 | Serial disconnection | Auto-reconnect.  Bridge polls for the CH340 port every 2 s.  Browser status line shows "SERIAL — reconnecting…" during the gap. |
| 10 | Button command format | `bA1` / `bA0` — three ASCII bytes.  `b` = button-inject command, next byte = button letter (`A`/`B`/`X`/`Y`), next byte = `1` (press) or `0` (release). |
| 11 | Transport indicator | Python injects `"transport":"serial"` into every JSON heartbeat it relays to the browser.  The shared `index.html` JavaScript checks for this field and shows a 🔌 SERIAL badge in the status line.  WiFi path is completely unchanged (field absent → badge hidden). |
| 12 | Listen address | `127.0.0.1` (localhost only).  Not exposed to the LAN by default. |
| 13 | Multi-CH340 behaviour | Pick the first found; print all detected ports as informational output.  User specifies a port explicitly via CLI arg to override. |

---

## Data Flow

```
ESP32 serial TX  →  Python asyncio reader  →  inject "transport":"serial"  →  ws.send(json)  →  browser
browser click    →  ws.recv({"btn":"A","action":"press"})  →  serial.write("bA1")  →  ESP32 onButtonPress()
```

The 100 ms JSON heartbeat that the firmware already emits on `Serial` is the sole state source;
the Python bridge passes it through verbatim (plus the injected `transport` field).

---

## Files to Create / Modify

| File | Action |
|------|--------|
| `serial_bridge/serial_bridge.py` | **Create** — asyncio bridge |
| `serial_bridge/requirements.txt` | **Create** — `websockets`, `pyserial` |
| `serial_bridge/index.html` | **Create** — extracted from PROGMEM + SERIAL badge JS |
| ~~`Peeling_Automation_Stepper_esp32/data/index.html`~~ | ~~symlink to serial_bridge/index.html~~ — not needed (PROGMEM approach) |
| `Peeling_Automation_Stepper_esp32/Peeling_Automation_Stepper_esp32.ino` | **Modify** — (a) add `b` command to serial parser, (b) embed HTML as `PROGMEM` raw-string `HTML_PAGE`, served via `req->send_P()` |
| ~~`tools/upload_fs.py`~~ | ~~mklittlefs + esptool.py wrapper~~ — not needed (PROGMEM approach) |
| `CLAUDE.md` | **Update** — serial bridge usage; note about keeping PROGMEM + serial_bridge/index.html in sync |
