#!/usr/bin/env python3
"""
upload_fs.py — Build and flash the LittleFS filesystem image to the ESP32.

Packages the web UI (index.html) into a LittleFS binary and writes it to
the ESP32's flash.  Run this once after first flashing the firmware, and
again whenever index.html changes.

Requirements:
    - mklittlefs  in PATH  (https://github.com/earlephilhower/mklittlefs/releases)
    - esptool.py           (pip install esptool)
    - pyserial             (pip install pyserial)

Usage:
    python tools/upload_fs.py [--port /dev/ttyUSB0] [--offset 0x290000] [--size 0x170000]

Default offsets match the ESP32 default partition table (4 MB flash):
    spiffs   0x290000   size 0x170000 (1.44 MB)
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# ── Defaults matching the ESP32 default.csv partition table (4 MB flash) ──────
DEFAULT_OFFSET = 0x290000   # spiffs partition start
DEFAULT_SIZE   = 0x170000   # spiffs partition size (1.44 MB)
BAUD_RATE      = 460800     # matches existing CLAUDE.md upload speed for CH340

CH340_VID = 0x1A86
CH340_PID = 0x7523

# Paths are relative to this script's location (tools/)
REPO_ROOT = Path(__file__).parent.parent
DATA_DIR  = REPO_ROOT / "Peeling_Automation_Stepper_esp32" / "data"


# ─────────────────────────────────────────────────────────────────────────────
def _find_ch340() -> str:
    try:
        import serial.tools.list_ports
    except ImportError:
        sys.exit("pyserial not installed.  Run:  pip install pyserial")
    found = [p for p in serial.tools.list_ports.comports()
             if p.vid == CH340_VID and p.pid == CH340_PID]
    if not found:
        sys.exit("No CH340 device found.\n"
                 "Connect the ESP32 and try again, or pass --port explicitly.")
    if len(found) > 1:
        print("Multiple CH340 ports found:")
        for p in found:
            print(f"  {p.device}  —  {p.description}")
        print(f"Using first: {found[0].device}")
    return found[0].device


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Build and flash LittleFS image to the ESP32")
    ap.add_argument('--port',
                    help='Serial port (default: auto-detect CH340)')
    ap.add_argument('--offset', type=lambda x: int(x, 0),
                    default=DEFAULT_OFFSET,
                    help=f'Flash offset in hex (default: {DEFAULT_OFFSET:#x})')
    ap.add_argument('--size',   type=lambda x: int(x, 0),
                    default=DEFAULT_SIZE,
                    help=f'Partition size in hex (default: {DEFAULT_SIZE:#x})')
    ap.add_argument('--baud', type=int, default=BAUD_RATE,
                    help=f'Upload baud rate (default: {BAUD_RATE})')
    args = ap.parse_args()

    # ── Validate tools ────────────────────────────────────────────────────────
    mklittlefs = shutil.which('mklittlefs')
    if mklittlefs is None:
        sys.exit(
            "mklittlefs not found in PATH.\n"
            "Download from: https://github.com/earlephilhower/mklittlefs/releases\n"
            "and place the binary somewhere on your PATH.")

    if shutil.which('esptool.py') is None and shutil.which('esptool') is None:
        sys.exit(
            "esptool not found.  Install with:  pip install esptool")
    esptool_cmd = 'esptool.py' if shutil.which('esptool.py') else 'esptool'

    # ── Validate data directory ───────────────────────────────────────────────
    if not DATA_DIR.exists():
        sys.exit(f"Data directory not found: {DATA_DIR}")
    files = list(DATA_DIR.iterdir())
    if not files:
        sys.exit(f"Data directory is empty: {DATA_DIR}")
    print(f"Data directory: {DATA_DIR}")
    for f in files:
        if not f.name.startswith('.'):   # skip .gitkeep etc.
            print(f"  {f.name}  ({f.stat().st_size:,} bytes)")

    # ── Serial port ───────────────────────────────────────────────────────────
    port = args.port or _find_ch340()
    print(f"Port: {port}")

    # ── Build LittleFS image ──────────────────────────────────────────────────
    fd, img_path = tempfile.mkstemp(suffix='.bin', prefix='littlefs_')
    os.close(fd)
    try:
        print(f"\nBuilding LittleFS image  (size={args.size:#x}) …")
        subprocess.run(
            [mklittlefs,
             '-c', str(DATA_DIR),
             '-b', '4096',
             '-p', '256',
             '-s', str(args.size),
             img_path],
            check=True)
        img_size = Path(img_path).stat().st_size
        print(f"Image built: {img_size:,} bytes  →  {img_path}")

        # ── Flash ─────────────────────────────────────────────────────────────
        print(f"\nFlashing to {port}  offset={args.offset:#x}  baud={args.baud} …")
        subprocess.run(
            [esptool_cmd,
             '--chip', 'esp32',
             '--port', port,
             '--baud', str(args.baud),
             'write_flash',
             str(args.offset), img_path],
            check=True)

        print("\nDone — LittleFS image flashed successfully.")
        print("You may now open the web UI at http://<ESP32-IP>/")

    except subprocess.CalledProcessError as exc:
        sys.exit(f"Command failed (exit {exc.returncode})")
    finally:
        try:
            os.unlink(img_path)
        except OSError:
            pass


if __name__ == '__main__':
    main()
