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

const { app, BrowserWindow, ipcMain, dialog } = require('electron');
const path = require('path');
const fs   = require('fs');
const { startBridge, stopBridge, setLogsDir, listPorts, connectToPort, disconnectFromPort } = require('./bridge');

const SETTINGS_KEY = 'logsDir';

// Persist a single settings.json in userData (no extra npm dependency needed)
function loadSettings(settingsPath) {
  try {
    return JSON.parse(fs.readFileSync(settingsPath, 'utf8'));
  } catch (_) {
    return {};
  }
}

function saveSettings(settingsPath, data) {
  try {
    fs.writeFileSync(settingsPath, JSON.stringify(data, null, 2), 'utf8');
  } catch (e) {
    console.warn('Could not save settings:', e.message);
  }
}

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

let win          = null;
let currentLogsDir = null;
let settingsPath   = null;
let httpPort       = null;   // assigned by startBridge (OS-picked free port)

function createWindow() {
  win = new BrowserWindow({
    width: 560,
    height: 660,
    minWidth: 400,
    minHeight: 520,
    title: 'Peeling Controller',
    backgroundColor: '#111111',
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.js'),
    },
  });

  win.setMenuBarVisibility(false);
  win.loadURL(`http://127.0.0.1:${httpPort}`);

  win.on('closed', () => { win = null; });
}

app.whenReady().then(async () => {
  const portArg = getPortArg();
  settingsPath  = path.join(app.getPath('userData'), 'settings.json');

  const settings = loadSettings(settingsPath);
  currentLogsDir = settings[SETTINGS_KEY] || path.join(app.getPath('userData'), 'logs');

  // IPC: serial port management
  ipcMain.handle('list-ports',      ()         => listPorts());
  ipcMain.handle('connect-port',    (_, path)  => connectToPort(path));
  ipcMain.handle('disconnect-port', ()         => disconnectFromPort());

  // IPC: return current logs directory
  ipcMain.handle('get-logs-dir', () => currentLogsDir);

  // IPC: open native folder picker, persist choice, notify bridge
  ipcMain.handle('choose-logs-dir', async () => {
    const result = await dialog.showOpenDialog(win, {
      title:       'Choose log folder',
      defaultPath: currentLogsDir,
      properties:  ['openDirectory', 'createDirectory'],
    });
    if (result.canceled || result.filePaths.length === 0) return null;

    const chosen = result.filePaths[0];
    currentLogsDir = chosen;
    setLogsDir(chosen);

    const s = loadSettings(settingsPath);
    s[SETTINGS_KEY] = chosen;
    saveSettings(settingsPath, s);

    return chosen;
  });

  try {
    ({ httpPort } = await startBridge({ portArg, logsDir: currentLogsDir, autoConnect: false }));
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
