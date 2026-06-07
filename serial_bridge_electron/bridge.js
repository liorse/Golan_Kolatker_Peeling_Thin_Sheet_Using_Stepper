'use strict';
/**
 * bridge.js — Node.js port of serial_bridge.py
 *
 * HTTP server  : serves index.html at http://127.0.0.1:<httpPort>
 * WebSocket    : ws://127.0.0.1:8082/ws  ↔  serial port
 * Serial       : auto-detects CH340 adapter (VID 1A86 / PID 7523), 115200 baud,
 *                with automatic reconnect every 2 s on disconnect.
 * Logging      : CSV files written to <logsDir>/  on every PEELING run.
 *
 * Public API (used by main.js):
 *   startBridge({ httpPort, portArg, logsDir }) → Promise<void>
 *   stopBridge()
 *
 * Standalone CLI (no Electron):
 *   node bridge.js [serialPort] [--http-port PORT]
 */

const fs   = require('fs');
const http = require('http');
const path = require('path');

const { SerialPort }     = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const { WebSocketServer } = require('ws');

// ── Constants ─────────────────────────────────────────────────────────────────
const WS_PORT            = 8082;   // browser connects to ws://localhost:8082/ws
const BAUD_RATE          = 115200;
const RECONNECT_INTERVAL = 2000;   // ms between reconnect attempts

// CH340 USB-serial converter (Wemos D1 R32 / most cheap ESP32 dev boards)
const CH340_VID = '1a86';
const CH340_PID = '7523';

const HTML_PATH = path.join(__dirname, 'index.html');
const LOG_COLUMNS = ['timestamp', 'time_ms', 'pos_um', 'speed_um_s', 'temp_c', 'heater_duty', 'state'];

// ── Runtime state ─────────────────────────────────────────────────────────────
let clients    = new Set();   // active WebSocket connections
let latestMsg  = null;        // last heartbeat JSON (replayed to new clients)
let currentPort = null;       // open SerialPort instance (null = disconnected)
let loopActive  = false;      // false → serialLoop exits cleanly

// Logging state
let logStream  = null;        // fs.WriteStream, or null
let logRows    = 0;
let logsDir    = path.join(__dirname, 'logs');   // override via startBridge()

// Servers
let httpServer = null;
let wss        = null;

// ── Timestamp helpers ─────────────────────────────────────────────────────────

/** "20250527_143022" — used in log file names (local time). */
function stampFile(d) {
  const p = n => String(n).padStart(2, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_`
       + `${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

/** "2025-05-27T14:30:22.123" — row timestamps in logs (local time, ms precision). */
function stampRow(d) {
  const p  = n => String(n).padStart(2, '0');
  const ms = String(d.getMilliseconds()).padStart(3, '0');
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}T`
       + `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${ms}`;
}

// ── Experiment logging ────────────────────────────────────────────────────────

function logOpen(obj) {
  if (!fs.existsSync(logsDir)) fs.mkdirSync(logsDir, { recursive: true });

  const ts     = stampFile(new Date());
  const ang    = parseInt(obj.angle || 0);
  const spd    = parseFloat(obj.speed_set    || 0).toFixed(1).replace('.', 'p');
  const tmp    = parseFloat(obj.temp_setpoint || 0).toFixed(1).replace('.', 'p');
  const fname  = path.join(logsDir, `peel_${ts}_ang${ang}_spd${spd}_tmp${tmp}.csv`);

  logStream = fs.createWriteStream(fname, { encoding: 'utf8' });
  logStream.write(LOG_COLUMNS.join(',') + '\n');
  logRows = 0;
  console.log(`📄 Logging → ${fname}`);
}

function logRow(obj) {
  if (!logStream) return;
  const row = [
    stampRow(new Date()),
    obj.peel_elapsed_ms || 0,
    Number(obj.pos_um   || 0).toFixed(2),
    Number(obj.speed_um || 0).toFixed(3),
    obj.temp_c != null ? Number(obj.temp_c).toFixed(1) : '',
    obj.heater_duty != null ? obj.heater_duty : '',
    obj.state || '',
  ];
  logStream.write(row.join(',') + '\n');
  logRows++;
}

function logClose() {
  if (!logStream) return;
  const rows   = logRows;
  const stream = logStream;
  logStream = null;   // prevent further writes immediately
  logRows   = 0;
  stream.end(() => {
    console.log(`✓ Log closed — ${rows} rows written`);
  });
}

/**
 * Called for every parsed JSON heartbeat.
 * Opens / appends / closes the CSV log based on PEELING state transitions.
 */
function logDispatch(obj) {
  const state = obj.state || '';
  if (state === 'PEELING') {
    if (!logStream) logOpen(obj);
    logRow(obj);
  } else {
    if (logStream) {
      logRow(obj);   // write final row so the state transition is visible
      logClose();
    }
  }
}

// ── WebSocket broadcast ───────────────────────────────────────────────────────

function broadcast(text) {
  const dead = [];
  for (const ws of clients) {
    if (ws.readyState === ws.OPEN) {
      ws.send(text);
    } else {
      dead.push(ws);
    }
  }
  dead.forEach(ws => clients.delete(ws));
}

// ── Serial auto-detect ────────────────────────────────────────────────────────

async function findCH340(portArg) {
  if (portArg) return portArg;
  const ports = await SerialPort.list();
  const found = ports.filter(p =>
    p.vendorId?.toLowerCase()  === CH340_VID &&
    p.productId?.toLowerCase() === CH340_PID
  );
  if (found.length === 0) return null;
  if (found.length > 1) {
    console.log('Multiple CH340 ports detected:');
    found.forEach(p => console.log(`  ${p.path}  —  ${p.manufacturer || 'CH340'}`));
    console.log(`Using first: ${found[0].path}  (pass a port argument to override)\n`);
  }
  return found[0].path;
}

// ── Serial reconnect loop ─────────────────────────────────────────────────────

async function serialLoop(portArg) {
  while (loopActive) {
    // --- find device ---
    let portPath;
    try {
      portPath = await findCH340(portArg);
    } catch (e) {
      portPath = null;
    }

    if (!portPath) {
      process.stdout.write('Searching for CH340 device…\r');
      await sleep(RECONNECT_INTERVAL);
      continue;
    }

    // --- open port ---
    let port;
    try {
      port = new SerialPort({ path: portPath, baudRate: BAUD_RATE, autoOpen: false });
    } catch (e) {
      console.log(`Cannot create SerialPort for ${portPath}: ${e.message}`);
      await sleep(RECONNECT_INTERVAL);
      continue;
    }

    // Wait until the port closes (error or disconnect), then loop
    await new Promise(resolve => {
      const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

      port.open(err => {
        if (err) {
          console.log(`Cannot open ${portPath}: ${err.message}`);
          resolve();
          return;
        }
        currentPort = port;
        console.log(`\nConnected: ${portPath}  @  ${BAUD_RATE} baud`);
      });

      // ── incoming serial data ──────────────────────────────────────────────
      parser.on('data', line => {
        line = line.trim();
        if (!line.startsWith('{')) return;

        let obj;
        try { obj = JSON.parse(line); } catch (_) { return; }

        // Experiment logging — update first so log_info reflects current state
        logDispatch(obj);

        // Inject transport + live log status for the browser UI
        obj.transport = 'serial';
        obj.log_info  = {
          active:   logStream !== null,
          filename: logStream ? path.basename(String(logStream.path)) : '',
          folder:   logsDir,
        };

        const text = JSON.stringify(obj);
        latestMsg  = text;
        broadcast(text);
      });

      // ── port closed (cable pulled, reset, etc.) ───────────────────────────
      port.on('close', () => {
        console.log(`\nSerial lost (${portPath})`);
        currentPort = null;
        logClose();
        broadcast('{"serial_lost":true,"transport":"serial"}');
        resolve();
      });

      port.on('error', err => {
        console.error(`Serial error: ${err.message}`);
        try { port.close(); } catch (_) { /* already closed */ }
      });
    });

    if (!loopActive) break;
    console.log(`Reconnecting in ${RECONNECT_INTERVAL / 1000} s…`);
    await sleep(RECONNECT_INTERVAL);
  }
}

// ── HTTP server ───────────────────────────────────────────────────────────────

function startHTTP(httpPort) {
  return new Promise((resolve, reject) => {
    const html = fs.readFileSync(HTML_PATH);

    httpServer = http.createServer((req, res) => {
      if (req.url === '/' || req.url === '/index.html') {
        res.writeHead(200, {
          'Content-Type':   'text/html; charset=utf-8',
          'Content-Length': html.length,
        });
        res.end(html);
      } else {
        res.writeHead(404);
        res.end('Not found\n');
      }
    });

    httpServer.listen(httpPort, '127.0.0.1', () => {
      console.log(`HTTP  ←  http://127.0.0.1:${httpPort}`);
      resolve();
    });

    httpServer.on('error', reject);
  });
}

// ── WebSocket server ──────────────────────────────────────────────────────────

function startWS() {
  wss = new WebSocketServer({ host: '127.0.0.1', port: WS_PORT });
  console.log(`WS    ←  ws://127.0.0.1:${WS_PORT}/ws`);

  wss.on('connection', ws => {
    clients.add(ws);

    // Replay the latest heartbeat so the UI is populated immediately on connect
    if (latestMsg) ws.send(latestMsg);

    ws.on('message', raw => {
      // Translate browser button-event JSON → bXY serial command
      // Browser sends: {"btn":"A","action":"press"}
      // Serial output: bA1  (3 bytes, no newline)
      let msg;
      try { msg = JSON.parse(raw); } catch (_) { return; }

      const { btn, action } = msg;
      if (!['A', 'B', 'X', 'Y'].includes(btn))                return;
      if (!['press', 'release'].includes(action))              return;

      const cmd = Buffer.from('b' + btn + (action === 'press' ? '1' : '0'));
      if (currentPort && currentPort.isOpen) {
        currentPort.write(cmd, err => {
          if (err) console.warn('Serial write error:', err.message);
        });
      }
    });

    ws.on('close', () => clients.delete(ws));
    ws.on('error', () => clients.delete(ws));
  });
}

// ── Public API ────────────────────────────────────────────────────────────────

/**
 * Start the bridge.  Returns a Promise that resolves once the HTTP server
 * is listening (so Electron can call loadURL() straight away).
 *
 * @param {object} opts
 * @param {number}      opts.httpPort  HTTP port for the UI (default 8080)
 * @param {string|null} opts.portArg   Serial port path, or null for auto-detect
 * @param {string}      opts.logsDir   Directory for CSV logs
 */
async function startBridge({ httpPort = 8080, portArg = null, logsDir: logsPath } = {}) {
  if (logsPath) logsDir = logsPath;
  loopActive = true;

  await startHTTP(httpPort);
  startWS();
  serialLoop(portArg);   // runs forever; errors are caught internally
}

function stopBridge() {
  loopActive = false;
  logClose();
  if (currentPort && currentPort.isOpen) {
    try { currentPort.close(); } catch (_) { /* ignore */ }
  }
  if (wss)        { try { wss.close(); }        catch (_) { /* ignore */ } }
  if (httpServer) { try { httpServer.close(); } catch (_) { /* ignore */ } }
}

function setLogsDir(newDir) {
  logsDir = newDir;
}

module.exports = { startBridge, stopBridge, setLogsDir };

// ── Standalone CLI (node bridge.js) ──────────────────────────────────────────
if (require.main === module) {
  const argv = process.argv.slice(2);

  let portArg  = null;
  let httpPort = 8080;

  for (let i = 0; i < argv.length; i++) {
    if (argv[i] === '--http-port' && argv[i + 1]) {
      httpPort = parseInt(argv[++i], 10);
    } else if (!argv[i].startsWith('-')) {
      portArg = argv[i];
    }
  }

  startBridge({ httpPort, portArg, logsDir: path.join(__dirname, 'logs') })
    .then(() => {
      console.log(`\nOpen  →  http://localhost:${httpPort}`);
      console.log('Press Ctrl+C to stop.\n');
    })
    .catch(err => {
      console.error('Bridge failed to start:', err.message);
      process.exit(1);
    });

  process.on('SIGINT', () => {
    console.log('\nBridge stopped.');
    stopBridge();
    process.exit(0);
  });
}

// ── Utility ───────────────────────────────────────────────────────────────────
function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}
