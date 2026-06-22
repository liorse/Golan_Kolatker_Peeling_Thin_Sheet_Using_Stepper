#!/usr/bin/env python3
"""
serial_bridge.py — Serial ↔ WebSocket bridge for the Peeling Thin-Sheet Controller.

Serves the web UI at http://localhost:8080 and bridges:
  ESP32 serial TX  →  browser WebSocket  (JSON heartbeat + injected "transport":"serial")
  browser WS click →  ESP32 serial TX    (bA1 / bA0 button-inject commands)

The ESP32 continues to serve the same UI over WiFi independently; serial and WiFi
can be active at the same time.

Experiment logging
  When the firmware enters the PEELING state a CSV file is automatically opened in
  logs/ (next to this script).  Each 100 ms heartbeat is written as one row.  The
  file is closed when the state leaves PEELING.

  Filename:  peel_YYYYMMDD_HHMMSS_angNN_spdXpX.csv  (decimal point → 'p')
  Columns:   timestamp, time_ms, pos_um, speed_um_s, state

Usage:
    python serial_bridge.py [port] [--http-port PORT]

    port          Serial port, e.g. /dev/ttyUSB0 or COM3.
                  If omitted the first CH340 USB-serial adapter is used
                  (VID 1A86 / PID 7523 — the chip on the Wemos D1 R32).

    --http-port   HTTP port for the web UI (default: 8080).

Requirements:
    pip install websockets pyserial
    (Python 3.8+)
"""

import argparse
import asyncio
import csv
import datetime
import json
import logging
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Optional, Set

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial not installed.  Run:  pip install pyserial")

try:
    import websockets
except ImportError:
    sys.exit("websockets not installed.  Run:  pip install websockets")

# ── Fixed addresses ───────────────────────────────────────────────────────────
HTTP_HOST = "127.0.0.1"
WS_HOST   = "127.0.0.1"
WS_PORT   = 8082           # browser connects to ws://localhost:8082/ws

# CH340 USB-serial converter (Wemos D1 R32 / most cheap ESP32 dev boards)
CH340_VID = 0x1A86
CH340_PID = 0x7523

BAUD_RATE          = 115200
RECONNECT_INTERVAL = 2.0   # seconds between reconnect attempts

HTML_PATH = Path(__file__).parent / "index.html"

# ── Runtime globals (all asyncio-thread accesses except where noted) ──────────
_clients: Set           = set()   # active browser WebSocket connections
_latest:  Optional[str] = None    # last heartbeat JSON (sent to new clients on connect)
_ser:     Optional[serial.Serial] = None   # current open serial port (None = disconnected)
_ser_lock = threading.Lock()      # guards _ser for cross-thread safety

# ── Experiment log state ───────────────────────────────────────────────────────
LOG_DIR = Path(__file__).parent / "logs"
_log_file   = None          # open file object (None = not logging)
_log_writer = None          # csv.writer for _log_file
_log_rows   = 0             # rows written in current log
_prev_state = ""            # last seen state string (edge detection)

LOG_COLUMNS = ["timestamp", "time_ms", "pos_um", "speed_um_s", "state"]


def _log_open(obj: dict) -> None:
    """Open a new CSV log file for a peeling run."""
    global _log_file, _log_writer, _log_rows
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts      = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    ang     = int(obj.get("angle", 0))
    spd     = float(obj.get("speed_set",     0.0))
    tmp     = float(obj.get("temp_setpoint", 0.0))
    spd_str = f"{spd:.1f}".replace('.', 'p')   # e.g. 1.5 → "1p5" (no dots in stem)
    tmp_str = f"{tmp:.1f}".replace('.', 'p')
    fname   = LOG_DIR / f"peel_{ts}_ang{ang}_spd{spd_str}_tmp{tmp_str}.csv"
    _log_file   = open(fname, "w", newline="", encoding="utf-8")
    _log_writer = csv.writer(_log_file)
    _log_writer.writerow(LOG_COLUMNS)
    _log_rows   = 0
    print(f"📄 Logging → {fname}")


def _log_row(obj: dict) -> None:
    """Append one heartbeat row to the open log."""
    global _log_rows
    if _log_writer is None:
        return
    ts     = datetime.datetime.now().isoformat(timespec="milliseconds")
    row = [
        ts,
        obj.get("peel_elapsed_ms", 0),
        round(float(obj.get("pos_um",    0.0)), 2),
        round(float(obj.get("speed_um",  0.0)), 3),
        obj.get("state", ""),
    ]
    _log_writer.writerow(row)
    _log_rows += 1


def _log_close() -> None:
    """Flush and close the current log file."""
    global _log_file, _log_writer, _log_rows
    if _log_file is None:
        return
    _log_file.flush()
    _log_file.close()
    print(f"✓ Log closed — {_log_rows} rows written")
    _log_file   = None
    _log_writer = None
    _log_rows   = 0


def _log_dispatch(obj: dict) -> None:
    """
    Called for every parsed JSON heartbeat.
    Opens / appends / closes the log based on state transitions.
    """
    global _prev_state
    state = obj.get("state", "")

    if state == "PEELING":
        if _log_file is None:
            _log_open(obj)
        _log_row(obj)
    else:
        if _log_file is not None:
            # Write the final non-PEELING row so the transition is visible
            _log_row(obj)
            _log_close()

    _prev_state = state


# ─────────────────────────────────────────────────────────────────────────────
# HTTP server  — serves index.html from this directory
# Runs in a daemon thread so it never blocks the asyncio event loop.
# ─────────────────────────────────────────────────────────────────────────────
class _HttpHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path in ('/', '/index.html'):
            body = HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Not found\n')

    def log_message(self, *_args) -> None:
        pass   # suppress per-request log noise


def _start_http(http_port: int) -> None:
    """Blocking HTTP server — called in a daemon thread."""
    server = HTTPServer((HTTP_HOST, http_port), _HttpHandler)
    server.serve_forever()


# ─────────────────────────────────────────────────────────────────────────────
# WebSocket handler  — browser → Python
# Each browser tab gets one call to this coroutine.
# ─────────────────────────────────────────────────────────────────────────────
async def _ws_handler(websocket, path: str = '/ws') -> None:
    global _clients, _latest
    _clients.add(websocket)
    # Replay the most recent heartbeat so the UI is populated immediately.
    if _latest:
        try:
            await websocket.send(_latest)
        except Exception:
            pass
    try:
        async for raw in websocket:
            await _forward_to_serial(raw)
    except Exception:
        pass
    finally:
        _clients.discard(websocket)


async def _forward_to_serial(raw: str) -> None:
    """
    Translate a browser button-event JSON → bXY serial command.

    Browser sends: {"btn":"A","action":"press"}
    Serial output: b A 1   (three ASCII bytes, no newline)
    """
    try:
        msg    = json.loads(raw)
        btn    = msg.get('btn')
        action = msg.get('action')
    except (json.JSONDecodeError, AttributeError):
        return
    if btn not in ('A', 'B', 'X', 'Y') or action not in ('press', 'release'):
        return

    cmd = ('b' + btn + ('1' if action == 'press' else '0')).encode()

    with _ser_lock:
        s = _ser
    if s is None:
        return
    loop = asyncio.get_event_loop()
    try:
        await loop.run_in_executor(None, s.write, cmd)
    except Exception as exc:
        logging.warning("Serial write error: %s", exc)


async def _broadcast(text: str) -> None:
    """Send a text frame to every connected browser; remove dead sockets."""
    dead: Set = set()
    for ws in set(_clients):
        try:
            await ws.send(text)
        except Exception:
            dead.add(ws)
    _clients.difference_update(dead)


# ─────────────────────────────────────────────────────────────────────────────
# Serial auto-detect
# ─────────────────────────────────────────────────────────────────────────────
def _find_ch340(port_arg: Optional[str]) -> Optional[str]:
    if port_arg:
        return port_arg
    ports = list(serial.tools.list_ports.comports())
    found = [p for p in ports if p.vid == CH340_VID and p.pid == CH340_PID]
    if not found:
        return None
    if len(found) > 1:
        print("Multiple CH340 ports detected:")
        for p in found:
            print(f"  {p.device}  —  {p.description}")
        print(f"Using first: {found[0].device}  "
              "(pass a port argument to override)\n")
    return found[0].device


# ─────────────────────────────────────────────────────────────────────────────
# Serial read loop with auto-reconnect
# ─────────────────────────────────────────────────────────────────────────────
async def _serial_loop(port_arg: Optional[str], http_port: int) -> None:
    """
    Maintain the serial connection.  On disconnect, poll every
    RECONNECT_INTERVAL seconds until the device reappears.
    """
    global _ser, _latest
    loop = asyncio.get_event_loop()

    while True:
        port = _find_ch340(port_arg)
        if port is None:
            print("Searching for CH340 device…", end='\r', flush=True)
            await asyncio.sleep(RECONNECT_INTERVAL)
            continue

        try:
            s = serial.Serial(port, BAUD_RATE, timeout=0.5, dtr=False, rts=False)
        except serial.SerialException as exc:
            print(f"Cannot open {port}: {exc}")
            await asyncio.sleep(RECONNECT_INTERVAL)
            continue

        with _ser_lock:
            _ser = s

        print(f"\nConnected: {port}  @  {BAUD_RATE} baud")
        print(f"Open  →  http://localhost:{http_port}\n")

        try:
            while True:
                # readline() is blocking — run in thread pool to avoid blocking asyncio
                line: bytes = await loop.run_in_executor(None, s.readline)
                if not line:
                    if not s.is_open:
                        break       # port closed externally
                    continue        # timeout (0.5 s) — device is quiet, keep waiting

                text = line.decode('utf-8', errors='replace').strip()
                if not text.startswith('{'):
                    continue        # ignore non-JSON lines (e.g. boot messages)

                try:
                    obj = json.loads(text)
                except json.JSONDecodeError:
                    continue

                # Experiment logging — dispatch first so log_info reflects
                # the current open/close state in the same broadcast frame.
                _log_dispatch(obj)

                # Inject transport marker + live log status for the browser UI
                obj['transport'] = 'serial'
                obj['log_info'] = {
                    'active':   _log_file is not None,
                    'filename': Path(_log_file.name).name if _log_file is not None else '',
                    'folder':   'logs',
                }
                text = json.dumps(obj, separators=(',', ':'))
                _latest = text
                await _broadcast(text)

        except Exception as exc:
            print(f"\nSerial lost ({port}): {exc}")
        finally:
            with _ser_lock:
                _ser = None
            try:
                s.close()
            except Exception:
                pass
            # Close any open experiment log so data is not lost on disconnect
            _log_close()
            # Tell every open browser tab that serial is gone
            await _broadcast('{"serial_lost":true,"transport":"serial"}')

        print(f"Reconnecting in {RECONNECT_INTERVAL:.0f}s…")
        await asyncio.sleep(RECONNECT_INTERVAL)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
async def _async_main(port_arg: Optional[str], http_port: int, logs_dir: Optional[str]) -> None:
    global LOG_DIR
    if logs_dir:
        LOG_DIR = Path(logs_dir)

    if not HTML_PATH.exists():
        sys.exit(f"HTML file not found: {HTML_PATH}\n"
                 "Run from the serial_bridge/ directory or re-check the path.")

    # HTTP server in a daemon thread (serves index.html)
    threading.Thread(
        target=_start_http, args=(http_port,), daemon=True, name="http"
    ).start()
    print(f"HTTP  ←  http://{HTTP_HOST}:{http_port}")

    # WebSocket server (asyncio)
    async with websockets.serve(_ws_handler, WS_HOST, WS_PORT):
        print(f"WS    ←  ws://{WS_HOST}:{WS_PORT}/ws")
        await _serial_loop(port_arg, http_port)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Serial ↔ WebSocket bridge for the Peeling Controller")
    ap.add_argument(
        'port', nargs='?',
        help='Serial port (default: auto-detect CH340 VID 1A86 / PID 7523)')
    ap.add_argument(
        '--http-port', type=int, default=8080, metavar='PORT',
        help='HTTP port for the web UI (default: 8080)')
    ap.add_argument(
        '--logs-dir', default=None, metavar='DIR',
        help='Directory for CSV logs (default: logs/ next to this script)')
    args = ap.parse_args()

    try:
        asyncio.run(_async_main(args.port, args.http_port, args.logs_dir))
    except KeyboardInterrupt:
        print("\nBridge stopped.")


if __name__ == '__main__':
    main()
