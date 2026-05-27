# Peeling Controller — Electron App

Node.js / Electron port of `serial_bridge.py`.  
Double-click the app → serial bridge starts → window opens automatically.

## Development

**Requires Node ≥ 16** (use nvm if your system node is older):

```bash
# If you have nvm:
nvm use 17      # or any ≥ 16

npm install
npm start                         # open app window
npm start -- /dev/ttyUSB0        # specify serial port (Linux)
npm start -- COM3                 # specify serial port (Windows)
```

The first `npm install` runs `electron-rebuild` automatically to compile
`serialport`'s native bindings against the bundled Electron Node version.

## Bridge-only (no Electron window)

```bash
node bridge.js                    # auto-detect CH340
node bridge.js /dev/ttyUSB0      # explicit port
node bridge.js --http-port 9090  # custom HTTP port
```

Then open **http://localhost:8080** in any browser — same as the Python bridge.

## Build distributable

```bash
npm run dist:linux   # → dist/  (.AppImage + .deb, x64)
npm run dist:win     # → dist/  (.exe installer + portable, x64)
```

**Cross-compilation note**: `serialport` uses native C++ bindings.  
To build the Windows `.exe` you must run `npm run dist:win` **on Windows**.  
To build the Linux `.AppImage` run `npm run dist:linux` **on Linux**.

Experiment logs (CSV) are written to:
- **Development**: `serial_bridge_electron/logs/`
- **Packaged app**: `%APPDATA%\peeling-controller\logs\` (Windows)  
  or `~/.config/peeling-controller/logs/` (Linux)

## How it differs from the Python bridge

| | Python bridge | Electron app |
|---|---|---|
| Runtime needed | Python 3.8 + pip | none (self-contained) |
| Opens browser | manually | automatic window |
| Log location | `serial_bridge/logs/` | user data dir (packaged) |
| Platforms | any with Python | Windows, Linux |
