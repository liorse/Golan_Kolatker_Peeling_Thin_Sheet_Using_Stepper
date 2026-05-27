'use strict';
/**
 * main.js — Electron entry point for the Peeling Controller desktop app.
 *
 * Starts the HTTP + WebSocket + serial bridge, then opens a BrowserWindow
 * pointing at http://127.0.0.1:<HTTP_PORT>.  The bridge logic lives in
 * bridge.js; this file only handles Electron lifecycle.
 *
 * Usage (development):
 *   npm start [serialPort]
 *   e.g.  npm start -- /dev/ttyUSB0      (Linux)
 *         npm start -- COM3              (Windows)
 *   Omit the port to auto-detect the CH340 adapter.
 *
 * Build distributable:
 *   npm run dist:linux     → dist/  (.AppImage, .deb)
 *   npm run dist:win       → dist/  (.exe installer + portable)
 */

const { app, BrowserWindow } = require('electron');
const path = require('path');
const { startBridge, stopBridge } = require('./bridge');

const HTTP_PORT = 8080;

// Electron argv when run as "electron . [userArgs]":
//   process.argv = ['electron', '/path/to/main.js', ...userArgs]
// When packaged:
//   process.argv = ['/path/to/Peeling Controller', ...userArgs]
// We take the first positional arg (if any) as the serial port.
function getPortArg() {
  const raw = process.argv.slice(2).filter(a =>
    !a.startsWith('-') &&          // not a flag
    !a.endsWith('.js') &&          // not the main.js path (dev mode)
    !a.endsWith('.asar') &&        // not the asar path (packaged)
    !a.includes('\\') &&           // not a windows path to the exe
    !/^\d+$/.test(a)               // not a bare number
  );
  return raw[0] || null;
}

let win = null;

function createWindow() {
  win = new BrowserWindow({
    width: 560,
    height: 640,
    minWidth: 400,
    minHeight: 500,
    title: 'Peeling Controller',
    backgroundColor: '#111111',
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
    },
  });

  win.setMenuBarVisibility(false);
  win.loadURL(`http://127.0.0.1:${HTTP_PORT}`);

  win.on('closed', () => { win = null; });
}

app.whenReady().then(async () => {
  const portArg = getPortArg();
  const logsDir = path.join(app.getPath('userData'), 'logs');

  try {
    await startBridge({ httpPort: HTTP_PORT, portArg, logsDir });
  } catch (err) {
    console.error('Bridge failed to start:', err);
    app.quit();
    return;
  }

  createWindow();

  // macOS: re-open window when dock icon is clicked and no windows are open
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', () => {
  stopBridge();
  app.quit();   // quit on all platforms (this is a lab instrument, not a typical macOS app)
});
