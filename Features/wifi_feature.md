# WiFi Feature — Design Specification

## Goal
Connect to the instrument via WiFi and get a browser UI that:
1. Pixel-perfectly replicates the physical 240×240 ST7789 display
2. Updates in real-time as the physical display changes
3. Exposes all four buttons (A, B, X, Y) as remotely actuatable controls

---

## Design Decisions

### WiFi Mode
- **Station mode** — ESP32 joins `WIS_Hotspot` (open network, no password)
- Credentials stored in `wifi_credentials.h` (gitignored), not hardcoded in `.ino`
- Boot is **non-blocking**: if the hotspot is unavailable the instrument works normally as a physical device
- Settings screen shows `"WiFi: connecting…"`, `"WiFi: 192.168.x.y"`, or `"WiFi: offline"` in real time
- mDNS advertises `http://peeling.local` for a permanent bookmark

### Real-Time Protocol
- **WebSocket** — ESP32 pushes a JSON heartbeat to all connected clients every 100 ms
- Library: `ESPAsyncWebServer` + `AsyncTCP`

### Browser UI
- **Pixel-perfect HTML canvas**, 240×240 internal coordinates, **CSS-scaled** to fit the viewport
- Both the **run screen and settings screen** are mirrored exactly
- No browser-only controls — strict mirror of the physical UI
- Canvas click/touch events on button regions send press/release WebSocket commands

### Button Semantics
- All four buttons (A, B, X, Y) are remotely actuatable
- **Press + Release events** — browser sends `{"btn":"A","action":"press"}` on mousedown/touchstart and `{"btn":"A","action":"release"}` on mouseup/touchend
- X and Y function as **limit switch simulators** — identical behavior to the physical microswitches
- Firmware injects virtual button state via `volatile bool virtualBtn[4]`, ORed with `digitalRead()` in the loop

### Concurrency
- `AsyncWebServer`/`AsyncTCP` run on core 0; `loop()` runs on core 1
- Shared variables protected with `portENTER_CRITICAL` / `portEXIT_CRITICAL` spinlock
- WebSocket callbacks write only to the virtual button booleans and a broadcast-pending flag; `loop()` owns all state machine logic

### Heartbeat Expansion
The 100 ms heartbeat JSON is expanded from ~70 bytes to ~200 bytes to support full canvas rendering:

```json
{
  "state": "PEELING",
  "position": 12345,
  "speed": 100,
  "pos_um": 11.58,
  "speed_um": 15.20,
  "speed_set": 15.0,
  "angle": 30,
  "spr": 1600,
  "dist_xa_steps": 478085,
  "dist_xa_um": 448.2,
  "start_pos_um": 0.0,
  "peel_elapsed_ms": 3200,
  "warning_active": false,
  "settings_field": 0,
  "btn": [false, false, false, false],
  "ip": "192.168.1.47"
}
```

### HTML Delivery
- Entire page (HTML + CSS + JS) stored as an **inline PROGMEM string** in the `.ino`
- No separate filesystem upload step required — same `arduino-cli compile/upload` as today

### IP Display
- IP address shown in the **settings screen** on the TFT (below the cal status line)
- Also accessible via mDNS at `http://peeling.local`

---

## Required Libraries
Install via Arduino Library Manager:
- `ESPAsyncWebServer` (by me-no-dev or mathieucarbou fork)
- `AsyncTCP` (by me-no-dev)
- `ESPmDNS` (built into ESP32 core — no install needed)

---

## Known Limitations
- If two browsers are open simultaneously, both can send button events. The firmware processes them in arrival order — no inter-client locking. Acceptable for single-lab use.
- The `enableMotor()` call blocks `loop()` for 500 ms (hardware DM542T requirement). During this window heartbeats are delayed; the browser will show a brief stale state. Harmless in practice.
