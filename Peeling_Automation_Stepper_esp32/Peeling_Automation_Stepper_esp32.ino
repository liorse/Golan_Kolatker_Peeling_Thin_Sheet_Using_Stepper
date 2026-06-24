// =============================================================================
// Project   : Peeling Thin Sheet Using Stepper Motor — ESP32 Controller
// File      : Peeling_Automation_Stepper_esp32.ino
// Author    : Lior Segev
// Version   : 4.4.0-esp32
// Date      : June 24, 2026
// =============================================================================
//
// OVERVIEW
// --------
// Firmware for a stepper-motor-driven thin-sheet peeling instrument.
// Controls a NEMA 17 via DM542T driver on a Wemos D1 R32 (ESP32)
// with a 2.8″ 240×320 ST7789 display and 4 external buttons.
//
// HARDWARE
// --------
//   • Wemos D1 R32 (ESP32 dual-core)
//   • NEMA 17 stepper motor (0.4 A rated)
//   • DM542T (V4.0) stepper driver — active-low ENA-
//       – t1: ENA→first PUL ≥ 200 ms (firmware uses 500 ms delay)
//       – t2: DIR stable before PUL ≥ 5 µs (setDirectionPin 40 µs)
//   • Microswitches: X = home end, Y = far end
//   • 2.8″ ST7789 240×320 TFT display (SPI/VSPI; RST → GPIO 2)
//   • 4 external push-buttons: A (start/stop), B (settings/home), X (increment/CAL), Y (decrement)
//
// PIN ASSIGNMENT
// ---------------
//   GPIO 26  — ENA+ (active-high; ENA- tied to GND)
//   GPIO 25  — DIR+ (DIR- tied to GND)
//   GPIO 27  — PUL+ (PUL- tied to GND)
//   GPIO  4  — BTN_A  (UI: start/stop; settings: navigate up)
//   GPIO 33  — BTN_B  (UI: settings/home; settings: navigate down / exit+save)
//   GPIO 14  — BTN_X  (UI: settings: increment/CAL trigger; no-op outside settings)
//   GPIO 12  — BTN_Y  (UI: settings: decrement; no-op outside settings)
//   GPIO 15  — LIMIT_SW_X (home/X limit switch, active-low)
//   GPIO 13  — LIMIT_SW_Y (far/Y limit switch, active-low, INPUT_PULLUP)
//   GPIO  2  — TFT_RST
//   GPIO 17  — TFT_DC
//   GPIO  5  — TFT_CS
//   GPIO 18  — SPI SCK  (VSPI default, shared with MAX31856)
//   GPIO 23  — SPI MOSI (VSPI default, shared with MAX31856)
//   GPIO 19  — SPI MISO (VSPI default; MAX31856 SDO)
//   GPIO 16  — TFT_BL
//   GPIO 21  — MAX31856 CS  (thermocouple amplifier)
//   GPIO 22  — MAX31856 DRDY (data-ready; LOW when conversion complete)
//   GPIO 32  — HEATER_PIN (LEDC auto-ch, 1 kHz PWM → N-channel MOSFET gate)
//
// UNIT CONVERSION
// ---------------
//   d = 2 × L × cos(θ/2)  →  L = d / (2 × cos(θ/2))
//   d (motor)    = steps × (1500 / steps_per_rev) µm
//   L (peel)     = steps × (1500 / steps_per_rev) / (2 × cos(θ/2))
//   speed_µm_s   = steps/s × (1500 / steps_per_rev) / (2 × cos(θ/2))
//
// STATE MACHINE
// -------------
//   IDLE → [A, dist_xa>0] → MOVING_TO_START → [arrival+100ms] → PEELING
//   IDLE → [A, dist_xa=0] → show warning "RUN CAL FIRST"
//   IDLE → [B, pos=0]     → SETTINGS  (B navigates down: speed→angle→start→steps→CAL→IDLE+save)
//   IDLE → [B, pos>0]     → HOMING
//   PEELING → [A or LIMIT_SW] → IDLE
//   HOMING  → [LIMIT_SW]      → IDLE (pos := 0)
//   SETTINGS/CAL field → [X] → CAL_HOMING → CAL_RUNNING → IDLE (saves dist_xa)
//   Any moving state → [A]   → IDLE (abort)
//   SETTINGS: A=up, B=down/exit+save, X=+/CAL, Y=-
//
// SERIAL INTERFACE  (115200 baud)
// --------------------------------
//   Commands: 'm'<int32> move to step pos, 's' stop, 'v'<int> set Hz,
//             'h'<0-255> set heater duty, 'b'<A|B|X|Y><1|0> virtual button press/release
//   Heartbeat every 100 ms: JSON with state, position, speed, temperatures, heater,
//     WiFi/WebSocket info, and all settings (see sendWsJson() for full field list)
// =============================================================================

#include <FastAccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MAX31856.h>
#include <SPI.h>
#include <EEPROM.h>
#include <math.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include "wifi_credentials.h"

// ---- Stepper driver pins ----------------------------------------------------
#define enablePinStepper  26
#define dirPinStepper     25
#define stepPinStepper    27

// ---- Display pins -----------------------------------------------------------
#define TFT_CS    5
#define TFT_DC   17
#define TFT_RST   2
#define TFT_BL   16

// ---- Button pins (active-low, INPUT_PULLUP) ---------------------------------
#define BTN_A    4   // UI: start / stop; in settings: navigate up
#define BTN_B   33   // UI: settings / home; in settings: navigate down / exit+save
#define BTN_X   14   // UI: in settings: increment (+) or trigger CAL; no-op outside settings
#define BTN_Y   12   // UI: in settings: decrement (−); no-op outside settings  GPIO12: strapping pin — WROOM pull-down holds it LOW at boot (safe)
#define LIMIT_SW_X 15  // home/X limit switch (active-low, INPUT_PULLUP)
#define LIMIT_SW_Y 13  // far/Y  limit switch (active-low, INPUT_PULLUP)

// ---- MAX31856 thermocouple amplifier (VSPI shared with display) ---------------
#define MAX_CS    21
#define MAX_DRDY  22

// ---- Heater MOSFET (LEDC, 1 kHz, 8-bit) ------------------------------------
#define HEATER_PIN  32

// ---- Display geometry -------------------------------------------------------
#define SCREEN_W   320
#define SCREEN_H   240
#define X_OFF        0
#define LEFT_W     200   // stepper UI column width (px)
#define TCOL_X     201   // temperature bar column start (1 px gap = vertical divider)

// Temperature bar geometry (right column)
#define TBAR_X     209   // bar left edge (centered with size-2 label in right column)
#define TBAR_W      40   // bar width in px
#define TBAR_TOP    46   // top of bar = 120 °C
#define TBAR_BOT   220   // bottom of bar = 0 °C
#define TBAR_H     (TBAR_BOT - TBAR_TOP)   // 174 px

#define BTN_W       52
#define BTN_H       28
#define BTN_LEFT_X   3
#define BTN_RIGHT_X (LEFT_W - BTN_W - 3)   // = 145; both button pairs stay in left column
#define BTN_TOP_Y    3
#define BTN_BOT_Y   (SCREEN_H - BTN_H - 3)

#define DIV_TOP_Y   33
#define DIV_BOT_Y  207

// Content y positions (run screen)
#define STATE_Y    36   // textSize 2 (16 px) → ends 52
#define POS_Y      56   //                    → ends 72
#define SETSPD_Y   76   //                    → ends 92
#define RUNSPD_Y   96   //                    → ends 112
#define ANGLE_Y   116   //                    → ends 132
#define TOEND_Y   136   //                    → ends 152
#define PEELT_Y   156   //                    → ends 172
#define BAR_X       20
#define BAR_Y     178   // 12 px bar → ends 190; bottom divider at 207
#define BAR_W     168   // shortened to stay in left column (20 + 168 = 188 < 200)
#define BAR_H      12
// TEMP_Y removed — temperature is shown in the right-column bar

// ---- Physical constants -----------------------------------------------------
// 1 full motor revolution = 1.5 mm linear travel.
// Step size = 1500 / steps_per_rev µm  (runtime variable: microns_per_step).
// CAL speed targets ≈1 mm/s at motor; calSpeedHz() computes this dynamically.

// ---- FastAccelStepper -------------------------------------------------------
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper       *stepper = NULL;

// ---- Display ----------------------------------------------------------------
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);

// ---- Thermocouple -----------------------------------------------------------
Adafruit_MAX31856 thermo(MAX_CS);   // hardware VSPI; DRDY read via digitalRead(MAX_DRDY)
float         lastTempC      = NAN; // NAN until first valid reading
unsigned long lastTempReadMs = 0;   // millis() of last SPI read (rate-limit to one per ~130 ms)
unsigned long lastDrdyHighMs = 0;   // millis() when DRDY was last seen HIGH; init in setup()

// ---- Heater state -----------------------------------------------------------
uint8_t heaterDuty = 0;            // current LEDC duty (0–255); updated by 'h' serial command

// ---- Temperature controller state ------------------------------------------
float tempSetpoint       = 50.0f;  // target °C (0–120)
bool  tempControlActive  = false;  // PID loop enabled (control loop deferred to next step)
bool  waitForTemp        = false;  // hold peel until temp reaches setpoint
bool  tempCtrlAutoEnabled = false; // true when GO auto-enabled temp control (so we auto-disable on end)
float kp                 = 60.0f;  // proportional gain
float ki                 =  0.15f; // integral gain
float kd                 = 900.0f; // derivative gain

// ---- PID runtime state (reset on activation) --------------------------------
#define PID_BAND  4.0f             // °C — bang-bang outside, PID inside
float         pidIntegral      = 0.0f;
float         pidLastError     = 0.0f;
float         pidFilteredDeriv = 0.0f;
unsigned long lastPidRunMs     = 0;    // 0 = first tick, skip computation

enum TempField { TFIELD_BACK, TFIELD_SETPOINT, TFIELD_KP, TFIELD_KI, TFIELD_KD, TFIELD_STARTSTOP };

bool      inTempSubMenu    = false;
TempField tempField        = TFIELD_BACK;
int       prevTempFieldIdx = -1;   // -1 = first draw needed in temp sub-menu

// ---- Persistent storage -----------------------------------------------------
#define EEPROM_MAGIC  0x50454C36u   // "PEL6" — wait_for_temp setting added
#define EEPROM_ADDR   0
#define EEPROM_SIZE   128

struct SavedSettings {
  uint32_t magic;
  int      angle_deg;
  float    speed_um_s;
  float    start_pos_um;
  int32_t  dist_xa_steps;
  float    tempSetpoint;
  float    kp;
  float    ki;
  float    kd;
  bool     tempControlActive;
  bool     waitForTemp;
};

// ---- Application state ------------------------------------------------------
enum AppState {
  IDLE,
  MOVING,            // serial-commanded move (no auto-peel)
  MOVING_TO_START,   // button-triggered: moves to start_pos, then auto-peels
  WAITING_FOR_TEMP,  // motor at start pos, waiting for temperature to reach setpoint
  PEELING,
  HOMING,
  SETTINGS,
  CAL_HOMING,
  CAL_RUNNING
};
AppState appState = IDLE;

enum SettingsField { FIELD_SPEED, FIELD_ANGLE, FIELD_START, FIELD_CAL, FIELD_TEMP, FIELD_WAIT_TEMP };
SettingsField settingsField = FIELD_SPEED;

// ---- User-configurable values -----------------------------------------------
const int steps_per_rev  = 25600;      // fixed — matches DM542T DIP switches; not user-adjustable
float   microns_per_step = 1500.0f / steps_per_rev;  // 0.05859375 µm/step
int     angle_deg      = 30;
float   speed_um_s     = 1.0f;
float   start_pos_um   = 0.0f;
int32_t dist_xa_steps  = 0;          // calibrated X→A distance in steps (0 = uncalibrated)

// ---- Motor state ------------------------------------------------------------
bool motorEnabled = false;

// ---- Peel sequencing --------------------------------------------------------
unsigned long startPeelAt   = 0;   // millis() target for MOVING_TO_START→PEELING (0=not armed)
unsigned long peel_start_ms = 0;

// ---- Button tracking --------------------------------------------------------
#define IDX_A 0
#define IDX_B 1
#define IDX_X 2
#define IDX_Y 3

bool          btnDown[4]      = {};
unsigned long btnPressAt[4]   = {};
bool          btnLongFired[4] = {};
unsigned long btnRepeatAt[4]  = {};

const unsigned long LONG_PRESS_MS = 500;
const unsigned long REPEAT_MS     = 100;

// ---- Limit switch edge detection (for safety abort in moving states) --------
bool          limitXPrev     = false;
bool          limitYPrev     = false;
bool          hasHomed       = false;
unsigned long limitXStableAt = 0;
unsigned long limitYStableAt = 0;
const unsigned long LIMIT_DEBOUNCE_MS = 20;

// ---- Warning overlay --------------------------------------------------------
unsigned long warningUntil = 0;

// ---- Periodic update --------------------------------------------------------
unsigned long previousMillis = 0;
const unsigned long HEARTBEAT_MS = 100;

// ---- WiFi / WebSocket -------------------------------------------------------
AsyncWebServer  webServer(80);
AsyncWebSocket  ws("/ws");
static portMUX_TYPE wsMux        = portMUX_INITIALIZER_UNLOCKED;
volatile bool   virtualBtn[4]    = {};   // written by WS callback (core 0), read by loop() (core 1)
char            wifiIpStr[40]    = "WiFi: connecting";
static bool     serverStarted    = false;
static unsigned long wifiStartMs = 0;
static int      prevRssiBars     = -1;   // tracks last drawn WiFi icon level
static int      prevClientCount  = -1;   // tracks last drawn WebSocket client count
static bool     ipStripDirty    = true;  // forces IP strip redraw after updateButtons()
static bool     runScreenDirty  = false; // forces full run-screen redraw after settings exit

// ESPAsyncWebServer (mathieucarbou ≥ 3.3.x) is thread-safe: ws.textAll() and
// webServer.begin() may be called directly from loop() on core 1.
void onWsEvent(AsyncWebSocket *, AsyncWebSocketClient *, AwsEventType, void *, uint8_t *, size_t);  // forward decl


// ---- Screen mode tracking (for clean transitions) ---------------------------
bool inSettingsScreen     = false;
bool settingsDirty        = true;
bool justEnteredSettings  = false;
int  prevSettingsFieldIdx = -1;   // -1 = first draw needed


// =============================================================================
// Unit conversion
// =============================================================================
int calSpeedHz() {
  // Targets ≈10 mm/s (10000 µm/s) at the motor → ≈5000 µm/s at the peel point.
  int hz = (int)(10000.0f / microns_per_step);
  return hz < 1 ? 1 : hz;
}

float stepToUmFactor() {
  return 1.0f / (2.0f * cosf((float)angle_deg * (float)M_PI / 360.0f));
}

float stepsToUm(int32_t steps) {
  return (float)steps * microns_per_step * stepToUmFactor();
}

int32_t umToSteps(float um) {
  float d = microns_per_step * stepToUmFactor();
  if (d < 1e-4f) d = 1e-4f;
  return (int32_t)(um / d);
}

uint32_t speedUmToMilliHz(float um_s) {
  float d = microns_per_step * stepToUmFactor();
  if (d < 1e-4f) d = 1e-4f;
  uint32_t mhz = (uint32_t)(um_s / d * 1000.0f);
  return mhz < 1 ? 1 : mhz;
}


// =============================================================================
// EEPROM persistence
// =============================================================================
void loadPrefs() {
  EEPROM.begin(EEPROM_SIZE);
  SavedSettings s;
  EEPROM.get(EEPROM_ADDR, s);
  if (s.magic == EEPROM_MAGIC) {
    angle_deg          = s.angle_deg;
    speed_um_s         = s.speed_um_s;
    start_pos_um       = s.start_pos_um;
    dist_xa_steps      = s.dist_xa_steps;
    tempSetpoint       = s.tempSetpoint;
    kp                 = s.kp;
    ki                 = s.ki;
    kd                 = s.kd;
    tempControlActive  = false;  // always boot with control off — safety gate
    waitForTemp        = s.waitForTemp;
  }
}

void saveAll() {
  SavedSettings s;
  s.magic             = EEPROM_MAGIC;
  s.angle_deg         = angle_deg;
  s.speed_um_s        = speed_um_s;
  s.start_pos_um      = start_pos_um;
  s.dist_xa_steps     = dist_xa_steps;
  s.tempSetpoint      = tempSetpoint;
  s.kp                = kp;
  s.ki                = ki;
  s.kd                = kd;
  s.tempControlActive = tempControlActive;
  s.waitForTemp       = waitForTemp;
  EEPROM.put(EEPROM_ADDR, s);
  EEPROM.commit();
}

void saveSettings()    { saveAll(); }
void saveCalibration() { saveAll(); }


// HTML page served from PROGMEM.  The same file is also used by the Python
// serial bridge (serial_bridge/index.html).  Keep both in sync when editing.
static const char HTML_PAGE[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Peeling Controller</title>
<style>
body{margin:0;background:#111;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;}
canvas{image-rendering:pixelated;width:640px;aspect-ratio:4/3;max-width:100vw;}
#ws-status{color:#888;font-size:12px;margin-top:6px;font-family:sans-serif;}
#serial-badge{display:none;color:#00fc00;font-weight:bold;margin-left:8px;font-size:13px;}
#log-status{font-family:monospace;font-size:11px;margin-top:4px;text-align:center;line-height:1.6;min-height:1.6em;}
</style>
</head>
<body>
<canvas id="c" width="320" height="240"></canvas>
<div id="ws-status">connecting... <span id="serial-badge">&#128268; SERIAL</span></div>
<div id="log-status"></div>
<script>
const SW=320,SH=240;
const LEFT_W=200;
const BTN_W=52,BTN_H=28,BTN_LEFT_X=3,BTN_RIGHT_X=LEFT_W-BTN_W-3,BTN_TOP_Y=3,BTN_BOT_Y=SH-BTN_H-3;
const DIV_TOP_Y=33,DIV_BOT_Y=207;
const STATE_Y=36,POS_Y=56,SETSPD_Y=76,RUNSPD_Y=96,ANGLE_Y=116,TOEND_Y=136,PEELT_Y=156;
const BAR_X=20,BAR_Y=178,BAR_W=168,BAR_H=12;
const fieldY=[60,82,104,126,148,170];
const tempFieldY=[60,82,104,126,148,170];
const TCOL_X=200,TCOL_W=120;
const TBAR_X=209,TBAR_W=40,TBAR_TOP=46,TBAR_BOT=220,TBAR_H=TBAR_BOT-TBAR_TOP;
const C={BK:'#000000',WH:'#ffffff',CY:'#00f8ff',GR:'#00fc00',YE:'#f8fc00',RE:'#f80000',GY:'#848484'};
const STATE_COL={IDLE:C.GR,MOVING:C.YE,TO_START:C.YE,WAIT_TEMP:C.YE,PEELING:C.YE,HOMING:C.CY,CAL_HOME:C.CY,CAL_RUN:C.CY,SETTINGS:C.GR};

const cv=document.getElementById('c');
const ctx=cv.getContext('2d');

function setFont(sz){ctx.font='bold '+(8*sz)+'px monospace';ctx.textBaseline='top';}
function cw(sz){return 6*sz;}
function ch(sz){return 8*sz;}

function drawText(text,x,y,sz,col,bg){
  setFont(sz);
  if(bg){ctx.fillStyle=bg;ctx.fillRect(x,y,text.length*cw(sz),ch(sz));}
  ctx.fillStyle=col;
  for(let i=0;i<text.length;i++) ctx.fillText(text[i],x+i*cw(sz),y);
}

function roundRect(x,y,w,h,r,fill,stroke){
  ctx.beginPath();
  ctx.moveTo(x+r,y);ctx.lineTo(x+w-r,y);ctx.quadraticCurveTo(x+w,y,x+w,y+r);
  ctx.lineTo(x+w,y+h-r);ctx.quadraticCurveTo(x+w,y+h,x+w-r,y+h);
  ctx.lineTo(x+r,y+h);ctx.quadraticCurveTo(x,y+h,x,y+h-r);
  ctx.lineTo(x,y+r);ctx.quadraticCurveTo(x,y,x+r,y);ctx.closePath();
  if(fill){ctx.fillStyle=fill;ctx.fill();}
  if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=1;ctx.stroke();}
}

function drawButtonBox(x,y,label,pressed,sz){
  sz=sz||2;
  const bg=pressed?C.WH:C.BK, fg=pressed?C.BK:C.CY;
  roundRect(x,y,BTN_W,BTN_H,4,bg,C.CY);
  const tx=x+Math.floor((BTN_W-label.length*cw(sz))/2);
  const ty=y+Math.floor((BTN_H-ch(sz))/2);
  drawText(label,tx,ty,sz,fg,bg);
}

function drawDividers(){
  ctx.strokeStyle=C.CY;ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(0,DIV_TOP_Y);ctx.lineTo(LEFT_W,DIV_TOP_Y);ctx.stroke();
  ctx.beginPath();ctx.moveTo(0,DIV_BOT_Y);ctx.lineTo(LEFT_W,DIV_BOT_Y);ctx.stroke();
  ctx.beginPath();ctx.moveTo(TCOL_X-1,0);ctx.lineTo(TCOL_X-1,SH);ctx.stroke();
}

function rssiToBars(rssi){
  if(!rssi||rssi===0) return 0;
  if(rssi>-60) return 4;
  if(rssi>-70) return 3;
  if(rssi>-80) return 2;
  return 1;
}

function drawWifiIcon(rssi,clients){
  const cx=100,cy=26,bars=rssiToBars(rssi);  // centred in gap (x=55..145) between buttons
  ctx.fillStyle=C.BK;ctx.fillRect(cx-14,cy-14,28,18);
  ctx.fillStyle=C.BK;ctx.fillRect(114,18,22,10);

  let color;
  if(bars>=3) color=C.GR;
  else if(bars===2) color=C.YE;
  else if(bars===1) color=C.RE;
  else color=C.GY;

  if(bars===0){
    ctx.strokeStyle=color;ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(cx-7,cy-11);ctx.lineTo(cx+7,cy-1);ctx.stroke();
    ctx.beginPath();ctx.moveTo(cx+7,cy-11);ctx.lineTo(cx-7,cy-1);ctx.stroke();
    setFont(1);ctx.fillStyle=C.CY;ctx.fillText(String(clients||0),116,20);
    return;
  }
  // dot
  ctx.fillStyle=color;ctx.beginPath();ctx.arc(cx,cy,2,0,Math.PI*2);ctx.fill();
  // arcs: upper semicircle (Math.PI to 0 = left to right over top)
  const radii=[];
  if(bars>=2) radii.push(5);
  if(bars>=3) radii.push(9);
  if(bars>=4) radii.push(13);
  ctx.strokeStyle=color;ctx.lineWidth=1;
  radii.forEach(r=>{
    ctx.beginPath();ctx.arc(cx,cy,r,Math.PI,0);ctx.stroke();
  });
  setFont(1);ctx.fillStyle=C.CY;ctx.fillText(String(clients||0),116,20);
}

function drawButtons(d){
  const btn=d.btn||[false,false,false,false];
  if(d.state==='SETTINGS'){
    drawButtonBox(BTN_LEFT_X,BTN_TOP_Y,'UP',btn[0],2);
    drawButtonBox(BTN_LEFT_X,BTN_BOT_Y,'DOWN',btn[1],2);
    let xLbl='+',yLbl='-';
    if(d.in_temp_sub){
      if(d.temp_field===0) xLbl='BACK';
      else if(d.temp_field===5) xLbl='ON';
      yLbl=(d.temp_field===5)?'OFF':'-';
    }else{
      if(d.settings_field===3) xLbl='CAL';
      else if(d.settings_field===4) xLbl='OPEN';
      else if(d.settings_field===5){xLbl='YES';yLbl='NO';}
    }
    drawButtonBox(BTN_RIGHT_X,BTN_TOP_Y,xLbl,btn[2],2);
    drawButtonBox(BTN_RIGHT_X,BTN_BOT_Y,yLbl,btn[3],2);
  }else{
    let aLbl,bLbl;
    if(d.state==='IDLE'){aLbl=d.dist_xa_steps>0?'GO':'!CAL';bLbl=d.has_homed?'SET':'HOME';}
    else{aLbl='STOP';bLbl='----';}
    drawButtonBox(BTN_LEFT_X,BTN_TOP_Y,aLbl,btn[0],2);
    drawButtonBox(BTN_LEFT_X,BTN_BOT_Y,bLbl,btn[1],2);
    roundRect(BTN_RIGHT_X,BTN_TOP_Y,BTN_W,BTN_H,4,C.BK,null);
    roundRect(BTN_RIGHT_X,BTN_BOT_Y,BTN_W,BTN_H,4,C.BK,null);
  }
}

function pad(v,n){return String(v).padStart(n);}
function padEnd(v,n){return String(v).padEnd(n);}

function drawRunScreen(d){
  const st=d.state||'IDLE';
  const col=STATE_COL[st]||C.GR;
  const padded=st.padStart(Math.floor((16+st.length)/2)).padEnd(16);
  drawText(padded,4,STATE_Y,2,col,C.BK);  // 16 chars centred: (200-192)/2=4

  if(d.warning_active){
    drawText('!CAL FIRST!        ',6,POS_Y,2,C.RE,C.BK);
  } else {
    drawText('POS:',6,POS_Y,2,C.CY,C.BK);
    drawText(pad(d.pos_um.toFixed(1),7),6+4*12,POS_Y,2,C.WH,C.BK);
    drawText('um      ',6+11*12,POS_Y,2,C.CY,C.BK);
  }
  drawText('SET:'+pad(d.speed_set.toFixed(1),7)+'um/s',6,SETSPD_Y,2,C.CY,C.BK);
  drawText('RUN:'+pad(d.speed_um.toFixed(1),7)+'um/s',6,RUNSPD_Y,2,C.WH,C.BK);
  drawText('ANG:'+pad(d.angle,7)+' deg',6,ANGLE_Y,2,C.CY,C.BK);

  let endStr;
  if(d.state==='PEELING'&&d.speed_set>0&&d.pos_um<d.dist_xa_um){
    endStr='END:'+pad(((d.dist_xa_um-d.pos_um)/d.speed_set).toFixed(1),7)+' s  ';
  } else {
    endStr='END:     -- s  ';
  }
  drawText(endStr,6,PEELT_Y,2,C.CY,C.BK);

  let ts='--';
  if(d.state==='PEELING'){
    const ts_=Math.floor(d.peel_elapsed_ms/1000);
    const sec=ts_%60,tot_m=Math.floor(ts_/60),mn=tot_m%60,hr=Math.floor(tot_m/60)%24,days=Math.floor(tot_m/1440);
    if(days>=10) ts=days+'d '+pad(hr,2)+':'+pad(mn,2);
    else if(days>=1) ts=days+'d '+pad(hr,2)+':'+pad(mn,2)+':'+pad(sec,2);
    else if(hr>=1) ts=pad(hr,2)+':'+pad(mn,2)+':'+pad(sec,2);
    else ts=pad(mn,2)+':'+pad(sec,2);
  }
  const lp=Math.floor((11-ts.length)/2);
  const cb=ts.padStart(lp+ts.length).padEnd(11);
  drawText('PLT:'+cb,6,TOEND_Y,2,C.CY,C.BK);

  ctx.strokeStyle=C.CY;ctx.lineWidth=1;ctx.strokeRect(BAR_X,BAR_Y,BAR_W,BAR_H);
  let filled=0;
  if(d.dist_xa_steps>0&&d.position>0){
    filled=Math.max(0,Math.min(BAR_W-2,Math.floor((BAR_W-2)*d.position/d.dist_xa_steps)));
  }
  ctx.fillStyle=C.CY;ctx.fillRect(BAR_X+1,BAR_Y+1,filled,BAR_H-2);
  ctx.fillStyle=C.BK;ctx.fillRect(BAR_X+1+filled,BAR_Y+1,BAR_W-2-filled,BAR_H-2);

}

function drawSettingsField(d,idx,active){
  ctx.fillStyle=C.BK;ctx.fillRect(0,fieldY[idx],LEFT_W,20);
  let vbuf;
  if(active){
    drawText('>',6,fieldY[idx],2,C.YE,C.BK);
    switch(idx){
      case 1:vbuf='ANG: '+pad(d.angle,2)+' deg  ';break;
      case 0:vbuf='SPD:'+d.speed_set.toFixed(1)+'um/s ';break;
      case 2:vbuf='ST: '+d.start_pos_um.toFixed(0)+'um   ';break;
      case 3:vbuf='CAL:press CAL';break;
      case 4:vbuf='TEMP SETTINGS';break;
      case 5:vbuf=d.wait_for_temp?'WAIT:YES     ':'WAIT:NO      ';break;
    }
    drawText(vbuf,22,fieldY[idx],2,C.YE,C.BK);
  }else{
    switch(idx){
      case 1:vbuf='ANG: '+d.angle+' deg';break;
      case 0:vbuf='SPD: '+d.speed_set.toFixed(1)+' um/s';break;
      case 2:vbuf='START: '+d.start_pos_um.toFixed(0)+' um';break;
      case 3:vbuf='CAL (press CAL)';break;
      case 4:vbuf='TEMP (press OPEN)';break;
      case 5:vbuf=d.wait_for_temp?'WAIT TEMP: YES':'WAIT TEMP: NO';break;
    }
    drawText(vbuf,16,fieldY[idx],1,C.GY,C.BK);
  }
}

function drawTempSubField(d,idx,active){
  ctx.fillStyle=C.BK;ctx.fillRect(0,tempFieldY[idx],LEFT_W,20);
  const sp=d.temp_setpoint!==undefined?d.temp_setpoint:50;
  const kp_=d.kp!==undefined?d.kp:60;
  const ki_=d.ki!==undefined?d.ki:0.15;
  const kd_=d.kd!==undefined?d.kd:900;
  const ctrlOn=!!d.temp_ctrl_active;
  let vbuf;
  if(active){
    drawText('>',6,tempFieldY[idx],2,C.YE,C.BK);
    switch(idx){
      case 0:vbuf='< BACK       ';break;
      case 1:vbuf='SP: '+pad(sp.toFixed(1),5)+' C  ';break;
      case 2:vbuf='Kp: '+pad(kp_.toFixed(1),5)+'   ';break;
      case 3:vbuf='Ki: '+pad(ki_.toFixed(2),5)+'   ';break;
      case 4:vbuf='Kd: '+pad(kd_.toFixed(0),5)+'   ';break;
      case 5:vbuf=ctrlOn?'CTRL: ON     ':'CTRL: OFF    ';break;
    }
    drawText(vbuf,22,tempFieldY[idx],2,C.YE,C.BK);
  }else{
    switch(idx){
      case 0:vbuf='< BACK';break;
      case 1:vbuf='SP: '+sp.toFixed(1)+' C';break;
      case 2:vbuf='Kp: '+kp_.toFixed(1);break;
      case 3:vbuf='Ki: '+ki_.toFixed(2);break;
      case 4:vbuf='Kd: '+kd_.toFixed(0);break;
      case 5:vbuf=ctrlOn?'CTRL: ON':'CTRL: OFF';break;
    }
    drawText(vbuf,16,tempFieldY[idx],1,C.GY,C.BK);
  }
}

function drawSettingsScreen(d){
  if(d.in_temp_sub){
    drawText('TEMP SET',Math.floor((LEFT_W-8*12)/2),STATE_Y,2,C.WH,C.BK);
    ctx.fillStyle=C.BK;ctx.fillRect(0,54,LEFT_W,6);
    for(let i=0;i<6;i++) drawTempSubField(d,i,i===d.temp_field);
  }else{
    drawText('SETTINGS',Math.floor((LEFT_W-8*12)/2),STATE_Y,2,C.WH,C.BK);
    ctx.fillStyle=C.BK;ctx.fillRect(0,54,LEFT_W,6);
    for(let i=0;i<6;i++) drawSettingsField(d,i,i===d.settings_field);
    ctx.fillStyle=C.BK;ctx.fillRect(0,190,LEFT_W,4);
    if(d.dist_xa_steps>0){
      drawText(padEnd('X-A: '+d.dist_xa_um.toFixed(1)+' um',22),6,194,1,C.GR,C.BK);
    }else{
      drawText('NOT CALIBRATED      ',6,194,1,C.RE,C.BK);
    }
  }
}

function drawTempColumn(d){
  const tempC=d.temp_c;
  const setpoint=d.temp_setpoint!==undefined?d.temp_setpoint:50;
  const ctrlActive=!!d.temp_ctrl_active;
  const heaterPct=Math.round((d.heater_duty||0)/255*100);
  const fault=(tempC===null||tempC===undefined);
  const stable=!fault&&ctrlActive&&Math.abs(tempC-setpoint)<=2.0;

  ctx.fillStyle=C.BK;ctx.fillRect(TCOL_X,0,TCOL_W,SH);

  let statusText,statusColor;
  if(!ctrlActive){
    statusText='OFF ';statusColor=C.GY;
  }else if(stable){
    statusText='OK  ';statusColor=C.GR;
  }else{
    const blink=Math.floor(Date.now()/500)%2;
    statusText='HEAT';statusColor=blink?C.YE:C.BK;
  }
  drawText(statusText,TCOL_X+2,2,2,statusColor,C.BK);

  const pwrStr='PWR:'+String(heaterPct).padStart(3)+'%';
  drawText(pwrStr,TCOL_X+2,19,2,C.CY,C.BK);

  ctx.strokeStyle=C.CY;ctx.lineWidth=1;ctx.strokeRect(TBAR_X,TBAR_TOP,TBAR_W,TBAR_H);

  const labelX=TBAR_X+TBAR_W+3;
  const spFrac=Math.max(0,Math.min(1,setpoint/120));
  const spY=TBAR_BOT-1-Math.floor((TBAR_H-2)*spFrac);

  if(fault){
    ctx.fillStyle=C.RE;ctx.fillRect(TBAR_X+1,TBAR_TOP+1,TBAR_W-2,TBAR_H-2);
    const ftx=TBAR_X+Math.floor((TBAR_W-3*6)/2);
    drawText('FLT',ftx,TBAR_TOP+Math.floor(TBAR_H/2)-4,1,C.WH,C.RE);
    ctx.fillStyle=C.BK;ctx.fillRect(labelX,TBAR_TOP,TCOL_X+TCOL_W-labelX-1,TBAR_H);
  }else{
    const clampedT=Math.max(0,Math.min(120,tempC));
    const fillH=Math.max(0,Math.min(TBAR_H-2,Math.floor((TBAR_H-2)*clampedT/120)));
    const fillY=TBAR_BOT-1-fillH;
    const barColor=stable?C.GR:C.RE;
    ctx.fillStyle=C.BK;ctx.fillRect(TBAR_X+1,TBAR_TOP+1,TBAR_W-2,TBAR_H-2-fillH);
    if(fillH>0){ctx.fillStyle=barColor;ctx.fillRect(TBAR_X+1,fillY,TBAR_W-2,fillH);}

    ctx.fillStyle=C.BK;ctx.fillRect(labelX,TBAR_TOP,TCOL_X+TCOL_W-labelX-1,TBAR_H);

    // Temp label centered in fill region
    const fillCenter=fillY+Math.floor(fillH/2);
    let lY=Math.max(TBAR_TOP,Math.min(TBAR_BOT-16,fillCenter-8));
    if(Math.abs(lY-spY)<12) lY=Math.min(TBAR_BOT-16,spY+2);
    drawText(tempC.toFixed(1),labelX,lY,2,C.WH,C.BK);

    // Setpoint label above the line
    const spLabelY=Math.max(TBAR_TOP,Math.min(TBAR_BOT-16,spY-17));
    drawText(setpoint.toFixed(1),labelX,spLabelY,2,C.WH,C.BK);
  }

  ctx.strokeStyle=C.WH;ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(TBAR_X-4,spY);ctx.lineTo(TBAR_X+TBAR_W+4,spY);ctx.stroke();

  ctx.strokeStyle=C.CY;ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(TCOL_X-1,0);ctx.lineTo(TCOL_X-1,SH);ctx.stroke();
}

let lastInSettings=null;
let lastInTempSub=null;
const serialBadge=document.getElementById('serial-badge');
const logStatusEl=document.getElementById('log-status');

function render(d){
  if(d.transport==='serial') serialBadge.style.display='inline';

  if(d.serial_lost){
    statusEl.firstChild.textContent='reconnecting... ';
    logStatusEl.innerHTML='';
    return;
  }

  if(d.log_info&&d.log_info.active){
    logStatusEl.innerHTML=
      '&#x1F4DD;&nbsp;<span style="color:#f8fc00">'+d.log_info.filename+'</span><br>'
      +'<span style="color:#888">&#8594;&nbsp;'+d.log_info.folder+'</span>';
  }else{
    logStatusEl.innerHTML='';
  }

  statusEl.firstChild.textContent='connected ';
  const inSettings=d.state==='SETTINGS';
  const inTempSub=!!d.in_temp_sub;
  if(inSettings!==lastInSettings||inTempSub!==lastInTempSub){
    ctx.fillStyle=C.BK;ctx.fillRect(0,DIV_TOP_Y+1,LEFT_W,DIV_BOT_Y-DIV_TOP_Y-1);
    lastInSettings=inSettings;
    lastInTempSub=inTempSub;
  }
  if(inSettings) drawSettingsScreen(d); else drawRunScreen(d);
  drawButtons(d);
  drawTempColumn(d);
  drawWifiIcon(d.rssi||0,d.clients||0);
  drawDividers();
}

ctx.fillStyle=C.BK;ctx.fillRect(0,0,SW,SH);
drawDividers();
// Draw initial temp column so bar outline is visible before first WebSocket frame

// WebSocket URL: use port 8081 when served from Python bridge on localhost;
// use the page host (port 80) when served directly from the ESP32.
const wsUrl=(location.hostname==='localhost'||location.hostname==='127.0.0.1')
  ? 'ws://'+location.hostname+':8082/ws'
  : 'ws://'+location.host+'/ws';

let sock;
const statusEl=document.getElementById('ws-status');
function connect(){
  sock=new WebSocket(wsUrl);
  sock.onopen=()=>{statusEl.firstChild.textContent='connected ';};
  sock.onclose=()=>{statusEl.firstChild.textContent='disconnected — reconnecting... ';setTimeout(connect,2000);};
  sock.onerror=()=>{sock.close();};
  sock.onmessage=e=>{try{render(JSON.parse(e.data));}catch(_){}};
}
connect();

function hitRegion(x,y){
  if(x>=BTN_LEFT_X&&x<=BTN_LEFT_X+BTN_W&&y>=BTN_TOP_Y&&y<=BTN_TOP_Y+BTN_H) return 'A';
  if(x>=BTN_RIGHT_X&&x<=BTN_RIGHT_X+BTN_W&&y>=BTN_TOP_Y&&y<=BTN_TOP_Y+BTN_H) return 'X';
  if(x>=BTN_LEFT_X&&x<=BTN_LEFT_X+BTN_W&&y>=BTN_BOT_Y&&y<=BTN_BOT_Y+BTN_H) return 'B';
  if(x>=BTN_RIGHT_X&&x<=BTN_RIGHT_X+BTN_W&&y>=BTN_BOT_Y&&y<=BTN_BOT_Y+BTN_H) return 'Y';
  return null;
}
function cvCoords(e){
  const r=cv.getBoundingClientRect();
  const t=e.touches?e.touches[0]:e;
  return{x:(t.clientX-r.left)*SW/r.width,y:(t.clientY-r.top)*SH/r.height};
}
function sendBtn(btn,action){if(sock&&sock.readyState===1)sock.send(JSON.stringify({btn,action}));}
cv.addEventListener('mousedown',e=>{const{x,y}=cvCoords(e);const b=hitRegion(x,y);if(b)sendBtn(b,'press');});
cv.addEventListener('mouseup',  e=>{const{x,y}=cvCoords(e);const b=hitRegion(x,y);if(b)sendBtn(b,'release');});
cv.addEventListener('touchstart',e=>{e.preventDefault();const{x,y}=cvCoords(e);const b=hitRegion(x,y);if(b)sendBtn(b,'press');},{passive:false});
cv.addEventListener('touchend',e=>{
  e.preventDefault();
  const r=cv.getBoundingClientRect(),t=e.changedTouches[0];
  const x=(t.clientX-r.left)*SW/r.width,y=(t.clientY-r.top)*SH/r.height;
  const b=hitRegion(x,y);if(b)sendBtn(b,'release');
},{passive:false});
</script>
</body>
</html>)rawhtml";


// =============================================================================
// WebSocket event handler
// =============================================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (!info->final || info->index != 0 || info->len != len) return;
  if (info->opcode != WS_TEXT) return;

  char buf[64];
  size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
  memcpy(buf, data, n);
  buf[n] = '\0';

  int idx = -1;
  if      (strstr(buf, "\"A\"")) idx = IDX_A;
  else if (strstr(buf, "\"B\"")) idx = IDX_B;
  else if (strstr(buf, "\"X\"")) idx = IDX_X;
  else if (strstr(buf, "\"Y\"")) idx = IDX_Y;
  if (idx < 0) return;

  bool pressed = strstr(buf, "\"press\"") != NULL;
  portENTER_CRITICAL(&wsMux);
  virtualBtn[idx] = pressed;
  portEXIT_CRITICAL(&wsMux);
}


// =============================================================================
// Settings field increment / decrement
// =============================================================================
void doIncrement(int dir, bool fast) {
  float dist_xa_um = stepsToUm(dist_xa_steps);
  switch (settingsField) {
    case FIELD_ANGLE:
      angle_deg = constrain(angle_deg + dir, 0, 89);
      break;
    case FIELD_SPEED:
      speed_um_s = constrain(speed_um_s + (float)dir * (fast ? 10.0f : 1.0f),
                             1.0f, 5000.0f);
      break;
    case FIELD_START: {
      float maxStart = (dist_xa_um > 0.0f) ? dist_xa_um : 1e6f;
      start_pos_um = constrain(start_pos_um + (float)dir * (fast ? 100.0f : 10.0f),
                               0.0f, maxStart);
      break;
    }
    case FIELD_CAL:
    case FIELD_TEMP:
    case FIELD_WAIT_TEMP:
      break;
  }
  settingsDirty = true;
}

void doTempIncrement(int dir, bool fast) {
  switch (tempField) {
    case TFIELD_SETPOINT:
      tempSetpoint = constrain(tempSetpoint + (float)dir, 0.0f, 120.0f);
      break;
    case TFIELD_KP:
      kp = constrain(kp + (float)dir * (fast ? 10.0f : 1.0f), 0.0f, 200.0f);
      break;
    case TFIELD_KI:
      ki = constrain(ki + (float)dir * (fast ? 0.1f : 0.01f), 0.0f, 2.0f);
      ki = roundf(ki * 100.0f) / 100.0f;  // clamp float drift to 2 decimal places
      break;
    case TFIELD_KD:
      kd = constrain(kd + (float)dir * (fast ? 100.0f : 10.0f), 0.0f, 2000.0f);
      break;
    default: break;
  }
  settingsDirty = true;
}

void clearContent();  // forward declaration

void enterTempSubMenu() {
  inTempSubMenu    = true;
  tempField        = TFIELD_BACK;
  prevTempFieldIdx = -1;
  clearContent();
  settingsDirty    = true;
}

void exitTempSubMenu() {
  inTempSubMenu        = false;
  prevSettingsFieldIdx = -1;  // force full main-settings redraw
  clearContent();
  settingsDirty        = true;
}

void cycleTempField() {
  switch (tempField) {
    case TFIELD_BACK:      tempField = TFIELD_SETPOINT;  break;
    case TFIELD_SETPOINT:  tempField = TFIELD_KP;        break;
    case TFIELD_KP:        tempField = TFIELD_KI;        break;
    case TFIELD_KI:        tempField = TFIELD_KD;        break;
    case TFIELD_KD:        tempField = TFIELD_STARTSTOP; break;
    case TFIELD_STARTSTOP:
      exitTempSubMenu();
      return;
  }
  settingsDirty = true;
}


// =============================================================================
// Motor helpers
// =============================================================================
void enableMotor() {
  if (!motorEnabled) {
    stepper->enableOutputs();
    delay(500);
    motorEnabled = true;
  }
}

void disableMotor() {
  stepper->disableOutputs();
  motorEnabled = false;
}

void abortAndIdle() {
  if (stepper->isRunning()) {
    stepper->forceStop();
  }
  disableMotor();
  startPeelAt = 0;
  if (tempCtrlAutoEnabled) {
    tempControlActive   = false;
    heaterDuty          = 0;
    ledcWrite(HEATER_PIN, 0);
    tempCtrlAutoEnabled = false;
  }
  appState = IDLE;
}

void startMoveToStart() {
  if (waitForTemp && !tempControlActive) {
    pidIntegral         = 0.0f;
    pidLastError        = 0.0f;
    pidFilteredDeriv    = 0.0f;
    lastPidRunMs        = 0;
    tempControlActive   = true;
    tempCtrlAutoEnabled = true;
  } else {
    tempCtrlAutoEnabled = false;
  }
  appState = MOVING_TO_START;
  enableMotor();
  stepper->setSpeedInHz(calSpeedHz() + 1); // Force FastAccelStepper ramp update
  stepper->setSpeedInHz(calSpeedHz());
  stepper->moveTo(umToSteps(start_pos_um));
  startPeelAt = 0;
}

void startPeeling() {
  // Motor stays enabled from startMoveToStart — no re-enable needed.
  stepper->setSpeedInMilliHz(speedUmToMilliHz(speed_um_s));
  stepper->moveTo(dist_xa_steps);
  peel_start_ms = millis();
  appState = PEELING;
}

void startHoming() {
  appState = HOMING;
  enableMotor();
  if (!digitalRead(LIMIT_SW_X)) {          // already at home switch — no need to move
    stepper->setCurrentPosition(0);
    hasHomed = true;
    disableMotor();
    appState = IDLE;
    updateButtons();
    return;
  }
  stepper->setSpeedInHz(calSpeedHz() + 1); // Force FastAccelStepper ramp update
  stepper->setSpeedInHz(calSpeedHz());
  stepper->runBackward();
}

void startCal() {
  enableMotor();
  stepper->setSpeedInHz(calSpeedHz() + 1); // Force FastAccelStepper ramp update
  stepper->setSpeedInHz(calSpeedHz());
  if (!digitalRead(LIMIT_SW_X)) {          // already at home switch — skip homing
    stepper->setCurrentPosition(0);
    limitYStableAt = 0;
    limitYPrev     = false;
    stepper->runForward();
    appState = CAL_RUNNING;
  } else {
    stepper->runBackward();
    appState = CAL_HOMING;
  }
}

void cycleSettingsField() {
  switch (settingsField) {
    case FIELD_SPEED:     settingsField = FIELD_ANGLE;     break;
    case FIELD_ANGLE:     settingsField = FIELD_START;     break;
    case FIELD_START:     settingsField = FIELD_CAL;       break;
    case FIELD_CAL:       settingsField = FIELD_TEMP;      break;
    case FIELD_TEMP:      settingsField = FIELD_WAIT_TEMP; break;
    case FIELD_WAIT_TEMP:
      saveSettings();
      appState = IDLE;
      return;
  }
  settingsDirty = true;
}


// =============================================================================
// Button event handlers
// =============================================================================
void onButtonPress(int idx) {
  switch (appState) {
    case IDLE:
      if (idx == IDX_A) {          // A = UI start button
        if (dist_xa_steps == 0) {
          warningUntil = millis() + 3000;
        } else {
          startMoveToStart();
        }
      } else if (idx == IDX_B) {
        if (hasHomed) {
          appState            = SETTINGS;
          settingsField       = FIELD_SPEED;
          settingsDirty       = true;
          justEnteredSettings = true;
        } else {
          startHoming();
        }
      }
      break;

    case SETTINGS:
      if (inTempSubMenu) {
        if (idx == IDX_A) {
          switch (tempField) {
            case TFIELD_SETPOINT:  tempField = TFIELD_BACK;      break;
            case TFIELD_KP:        tempField = TFIELD_SETPOINT;  break;
            case TFIELD_KI:        tempField = TFIELD_KP;        break;
            case TFIELD_KD:        tempField = TFIELD_KI;        break;
            case TFIELD_STARTSTOP: tempField = TFIELD_KD;        break;
            default: break;  // TFIELD_BACK: already at top, no-op
          }
          settingsDirty = true;
        } else if (idx == IDX_X) {
          if (tempField == TFIELD_BACK) {
            exitTempSubMenu();
          } else if (tempField == TFIELD_STARTSTOP) {
            if (!tempControlActive) {
              pidIntegral      = 0.0f;
              pidLastError     = 0.0f;
              pidFilteredDeriv = 0.0f;
              lastPidRunMs     = 0;
              tempControlActive = true;
            }
            settingsDirty = true;
          } else {
            doTempIncrement(+1, false);
          }
        } else if (idx == IDX_Y) {
          if (tempField == TFIELD_STARTSTOP) {
            if (tempControlActive) {
              tempControlActive = false;
              heaterDuty = 0;
              ledcWrite(HEATER_PIN, 0);
            }
            settingsDirty = true;
          } else if (tempField != TFIELD_BACK) {
            doTempIncrement(-1, false);
          }
        }
      } else {
        if (idx == IDX_A) {          // A = navigate up
          switch (settingsField) {
            case FIELD_ANGLE:     settingsField = FIELD_SPEED;     break;
            case FIELD_START:     settingsField = FIELD_ANGLE;     break;
            case FIELD_CAL:       settingsField = FIELD_START;     break;
            case FIELD_TEMP:      settingsField = FIELD_CAL;       break;
            case FIELD_WAIT_TEMP: settingsField = FIELD_TEMP;      break;
            default: break;          // FIELD_SPEED: already at top, no-op
          }
          settingsDirty = true;
        } else if (idx == IDX_X) {   // X = increment, CAL trigger, TEMP sub-menu, or WAIT YES
          if (settingsField == FIELD_CAL) {
            startCal();
          } else if (settingsField == FIELD_TEMP) {
            enterTempSubMenu();
          } else if (settingsField == FIELD_WAIT_TEMP) {
            waitForTemp   = true;
            settingsDirty = true;
          } else {
            doIncrement(+1, false);
          }
        } else if (idx == IDX_Y) {   // Y = decrement (no-op on CAL/TEMP; NO on WAIT_TEMP)
          if (settingsField == FIELD_WAIT_TEMP) {
            waitForTemp   = false;
            settingsDirty = true;
          } else if (settingsField != FIELD_CAL && settingsField != FIELD_TEMP) {
            doIncrement(-1, false);
          }
        }
      }
      // B: handled in onButtonRelease (navigate down / exit+save)
      break;

    case MOVING:
    case MOVING_TO_START:
    case WAITING_FOR_TEMP:
    case PEELING:
    case HOMING:
    case CAL_HOMING:
    case CAL_RUNNING:
      if (idx == IDX_A) {          // A = UI stop button
        hasHomed = false;
        abortAndIdle();
      }
      break;
  }
}

void onButtonRelease(int idx) {
  if (appState == SETTINGS && idx == IDX_B && !btnLongFired[IDX_B]) {
    if (justEnteredSettings) {
      justEnteredSettings = false;  // swallow the release that opened settings
    } else if (inTempSubMenu) {
      cycleTempField();
    } else {
      cycleSettingsField();
    }
  }
}

void onButtonLong(int idx) {
  if (appState != SETTINGS) return;
  if (inTempSubMenu) {
    if (tempField == TFIELD_BACK || tempField == TFIELD_STARTSTOP) return;
    if (idx == IDX_X) doTempIncrement(+1, false);
    else if (idx == IDX_Y) doTempIncrement(-1, false);
  } else {
    if (settingsField == FIELD_CAL || settingsField == FIELD_TEMP || settingsField == FIELD_WAIT_TEMP) return;
    if (idx == IDX_X) doIncrement(+1, false);
    else if (idx == IDX_Y) doIncrement(-1, false);
  }
}

void onButtonRepeat(int idx) {
  if (appState != SETTINGS) return;
  if (inTempSubMenu) {
    if (tempField == TFIELD_BACK || tempField == TFIELD_STARTSTOP) return;
    if (idx == IDX_X) doTempIncrement(+1, true);
    else if (idx == IDX_Y) doTempIncrement(-1, true);
  } else {
    if (settingsField == FIELD_CAL || settingsField == FIELD_TEMP || settingsField == FIELD_WAIT_TEMP) return;
    if (idx == IDX_X) doIncrement(+1, true);
    else if (idx == IDX_Y) doIncrement(-1, true);
  }
}


// =============================================================================
// Display helpers
// =============================================================================
void drawButtonBox(int16_t x, int16_t y, const char *label, bool pressed, uint8_t sz = 2) {
  uint16_t bg = pressed ? ST77XX_WHITE : ST77XX_BLACK;
  uint16_t fg = pressed ? ST77XX_BLACK : ST77XX_CYAN;
  tft.fillRoundRect(x, y, BTN_W, BTN_H, 4, bg);
  tft.drawRoundRect(x, y, BTN_W, BTN_H, 4, ST77XX_CYAN);
  tft.setTextSize(sz);
  tft.setTextColor(fg, bg);
  int len = strlen(label);
  int cw  = (sz == 1) ? 6 : 12;
  int ch  = (sz == 1) ? 8 : 16;
  tft.setCursor(x + (BTN_W - len * cw) / 2, y + (BTN_H - ch) / 2);
  tft.print(label);
}

void updateButtons() {
  char aLbl[8], bLbl[8];

  if (appState == SETTINGS) {
    drawButtonBox(BTN_LEFT_X,  BTN_TOP_Y, "UP",   btnDown[IDX_A]);
    drawButtonBox(BTN_LEFT_X,  BTN_BOT_Y, "DOWN", btnDown[IDX_B]);
    const char *xLbl, *yLbl;
    if (inTempSubMenu) {
      if      (tempField == TFIELD_BACK)      xLbl = "BACK";
      else if (tempField == TFIELD_STARTSTOP) xLbl = "ON";
      else                                    xLbl = "+";
      yLbl = (tempField == TFIELD_STARTSTOP) ? "OFF" : "-";
    } else {
      if      (settingsField == FIELD_CAL)       xLbl = "CAL";
      else if (settingsField == FIELD_TEMP)      xLbl = "OPEN";
      else if (settingsField == FIELD_WAIT_TEMP) xLbl = "YES";
      else                                       xLbl = "+";
      yLbl = (settingsField == FIELD_WAIT_TEMP) ? "NO" : "-";
    }
    drawButtonBox(BTN_RIGHT_X, BTN_TOP_Y, xLbl, btnDown[IDX_X]);
    drawButtonBox(BTN_RIGHT_X, BTN_BOT_Y, yLbl, btnDown[IDX_Y]);
  } else {
    if (appState == IDLE) {
      strcpy(aLbl, dist_xa_steps > 0 ? "GO" : "!CAL");
      strcpy(bLbl, hasHomed ? "SET" : "HOME");
    } else {
      strcpy(aLbl, "STOP");
      strcpy(bLbl, "----");
    }
    // Physical layout (setRotation 2): A=top-left, B=bottom-left, X/Y right side (blank)
    drawButtonBox(BTN_LEFT_X, BTN_TOP_Y, aLbl, btnDown[IDX_A]);
    drawButtonBox(BTN_LEFT_X, BTN_BOT_Y, bLbl, btnDown[IDX_B]);
    tft.fillRoundRect(BTN_RIGHT_X, BTN_TOP_Y, BTN_W, BTN_H, 4, ST77XX_BLACK);
    tft.fillRoundRect(BTN_RIGHT_X, BTN_BOT_Y, BTN_W, BTN_H, 4, ST77XX_BLACK);
  }
  ipStripDirty = true;  // button backgrounds may have overwritten IP text
}

void clearContent() {
  tft.fillRect(0, DIV_TOP_Y + 1, LEFT_W, DIV_BOT_Y - DIV_TOP_Y - 1, ST77XX_BLACK);
}


// =============================================================================
// Run screen
// =============================================================================
void updateRunContent() {
  static char    prevSb[18]    = {0};
  static int32_t prevPosSteps  = INT32_MIN;
  static bool    prevWarning   = false;
  static char    prevSetBuf[20]= {0};
  static char    prevRunBuf[20]= {0};
  static int     prevAngle     = INT_MIN;
  static char    prevEndBuf[20]= {0};
  static char    prevPltBuf[20]= {0};
  static int32_t prevFilled    = -1;

  if (runScreenDirty) {
    prevSb[0]      = '\0';
    prevPosSteps   = INT32_MIN;
    prevWarning    = true;   // force mismatch — actual warning is computed fresh below
    prevSetBuf[0]  = '\0';
    prevRunBuf[0]  = '\0';
    prevAngle      = INT_MIN;
    prevEndBuf[0]  = '\0';
    prevPltBuf[0]  = '\0';
    prevFilled     = -1;
    runScreenDirty = false;
  }

  char    buf[32];
  int32_t pos_steps   = stepper->getCurrentPosition();
  float   pos_um      = stepsToUm(pos_steps);
  float   actual_hz   = fabsf(stepper->getCurrentSpeedInMilliHz() / 1000.0f);
  float   actual_um_s = actual_hz * microns_per_step * stepToUmFactor();
  float   dist_xa_um  = stepsToUm(dist_xa_steps);

  // ---- State label ----
  const char *stateStr = "IDLE";
  uint16_t    stateCol = ST77XX_GREEN;
  switch (appState) {
    case MOVING:           stateStr = "MOVING";    stateCol = ST77XX_YELLOW; break;
    case MOVING_TO_START:  stateStr = "TO START";  stateCol = ST77XX_YELLOW; break;
    case WAITING_FOR_TEMP: stateStr = "WAIT TMP";  stateCol = ST77XX_YELLOW; break;
    case PEELING:          stateStr = "PEELING";   stateCol = ST77XX_YELLOW; break;
    case HOMING:           stateStr = "HOMING";    stateCol = ST77XX_CYAN;   break;
    case CAL_HOMING:       stateStr = "CAL HOME";  stateCol = ST77XX_CYAN;   break;
    case CAL_RUNNING:      stateStr = "CAL RUN";   stateCol = ST77XX_CYAN;   break;
    default: break;
  }
  {
    // 16 chars × 12 px = 192 px — fits within LEFT_W=200 without overflowing into temp column
    char sb[17];
    int  len = strlen(stateStr);
    int  lp  = (16 - len) / 2;
    int  i   = 0;
    while (i < lp)       sb[i++] = ' ';
    for (int j = 0; j < len; j++) sb[i++] = stateStr[j];
    while (i < 16)       sb[i++] = ' ';
    sb[i] = '\0';
    if (strcmp(sb, prevSb) != 0) {
      tft.setTextSize(2);
      tft.setTextColor(stateCol, ST77XX_BLACK);
      tft.setCursor((LEFT_W - 16 * 12) / 2, STATE_Y);  // centre 192 px in 200 px left column
      tft.print(sb);
      memcpy(prevSb, sb, sizeof(sb));
    }
  }

  tft.setTextSize(2);

  // ---- Position (or warning) ----
  bool warning = (millis() < warningUntil);
  bool needPos = (warning != prevWarning) || (!warning && pos_steps != prevPosSteps);
  if (needPos) {
    // 16 chars from x=6 → ends at x=198, stays within LEFT_W=200
    tft.setCursor(6 + X_OFF, POS_Y);
    if (warning) {
      tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
      tft.print("!CAL FIRST!     ");   // 16 chars
    } else {
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.print("POS:");
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      snprintf(buf, sizeof(buf), "%7.1f", pos_um);
      tft.print(buf);
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.print("um   ");              // 5 chars: total 4+7+5=16
    }
    prevPosSteps = pos_steps;
    prevWarning  = warning;
  }

  // ---- Set speed ----
  snprintf(buf, sizeof(buf), "SET:%7.1fum/s", speed_um_s);
  if (strcmp(buf, prevSetBuf) != 0) {
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setCursor(6 + X_OFF, SETSPD_Y);
    tft.print(buf);
    memcpy(prevSetBuf, buf, sizeof(prevSetBuf));
  }

  // ---- Run speed ----
  snprintf(buf, sizeof(buf), "RUN:%7.1fum/s", actual_um_s);
  if (strcmp(buf, prevRunBuf) != 0) {
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(6 + X_OFF, RUNSPD_Y);
    tft.print(buf);
    memcpy(prevRunBuf, buf, sizeof(prevRunBuf));
  }

  // ---- Angle ----
  if (angle_deg != prevAngle) {
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setCursor(6 + X_OFF, ANGLE_Y);
    snprintf(buf, sizeof(buf), "ANG:%7d deg", angle_deg);
    tft.print(buf);
    prevAngle = angle_deg;
  }

  // ---- Time to end ----
  {
    char endBuf[20];
    if (appState == PEELING && speed_um_s > 0.0f && pos_um < dist_xa_um) {
      snprintf(endBuf, sizeof(endBuf), "END:%7.1f s  ", (dist_xa_um - pos_um) / speed_um_s);
    } else {
      snprintf(endBuf, sizeof(endBuf), "END:     -- s  ");
    }
    if (strcmp(endBuf, prevEndBuf) != 0) {
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.setCursor(6 + X_OFF, PEELT_Y);
      tft.print(endBuf);
      memcpy(prevEndBuf, endBuf, sizeof(prevEndBuf));
    }
  }

  // ---- Peel elapsed time ----
  {
    char ts[14];
    if (appState == PEELING) {
      unsigned long total_s = (millis() - peel_start_ms) / 1000UL;
      int sec   = (int)(total_s % 60);
      int tot_m = (int)(total_s / 60);
      int mn    = tot_m % 60;
      int hr    = (tot_m / 60) % 24;
      int days  = tot_m / 1440;
      if (days >= 10)     snprintf(ts, sizeof(ts), "%dd %02d:%02d",      days, hr, mn);
      else if (days >= 1) snprintf(ts, sizeof(ts), "%dd %02d:%02d:%02d", days, hr, mn, sec);
      else if (hr >= 1)   snprintf(ts, sizeof(ts), "%02d:%02d:%02d",     hr, mn, sec);
      else                snprintf(ts, sizeof(ts), "%02d:%02d",          mn, sec);
    } else {
      snprintf(ts, sizeof(ts), "--");
    }
    int  tslen = strlen(ts);
    int  lpad  = (11 - tslen) / 2;
    char cb[16]; int ci = 0;
    for (int j = 0; j < lpad; j++)  cb[ci++] = ' ';
    for (int j = 0; j < tslen; j++) cb[ci++] = ts[j];
    while (ci < 11)                  cb[ci++] = ' ';
    cb[ci] = '\0';
    char pltBuf[20];
    snprintf(pltBuf, sizeof(pltBuf), "PLT:%s", cb);
    if (strcmp(pltBuf, prevPltBuf) != 0) {
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.setCursor(6 + X_OFF, TOEND_Y);
      tft.print(pltBuf);
      memcpy(prevPltBuf, pltBuf, sizeof(prevPltBuf));
    }
  }

  // ---- Progress bar ----
  int32_t filled = 0;
  if (dist_xa_steps > 0 && pos_steps > 0) {
    filled = (int32_t)(BAR_W - 2) * pos_steps / dist_xa_steps;
    filled = constrain(filled, 0L, (int32_t)(BAR_W - 2));
  }
  if (filled != prevFilled) {
    tft.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, ST77XX_CYAN);
    tft.fillRect(BAR_X + 1,          BAR_Y + 1, filled,              BAR_H - 2, ST77XX_CYAN);
    tft.fillRect(BAR_X + 1 + filled, BAR_Y + 1, BAR_W - 2 - filled, BAR_H - 2, ST77XX_BLACK);
    prevFilled = filled;
  }
}


// =============================================================================
// Settings screen
// =============================================================================
void drawSettingsField(int idx, bool active) {
  // 6 fields, 22 px apart; last field ends at 170+20=190, cal status at 192.
  const int fieldY[6] = { 60, 82, 104, 126, 148, 170 };
  tft.fillRect(0, fieldY[idx], LEFT_W, 20, ST77XX_BLACK);
  char vbuf[24];
  if (active) {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(6, fieldY[idx]);
    tft.print(">");
    tft.setCursor(22, fieldY[idx]);
    switch (idx) {
      case FIELD_ANGLE:     snprintf(vbuf, sizeof(vbuf), "ANG: %2d deg  ", angle_deg);                       break;
      case FIELD_SPEED:     snprintf(vbuf, sizeof(vbuf), "SPD:%.1fum/s ", speed_um_s);                       break;
      case FIELD_START:     snprintf(vbuf, sizeof(vbuf), "ST: %.0fum   ", start_pos_um);                     break;
      case FIELD_CAL:       snprintf(vbuf, sizeof(vbuf), "CAL:press CAL");                                   break;
      case FIELD_TEMP:      snprintf(vbuf, sizeof(vbuf), "TEMP SETTINGS");                                   break;
      case FIELD_WAIT_TEMP: snprintf(vbuf, sizeof(vbuf), waitForTemp ? "WAIT:YES     " : "WAIT:NO      ");   break;
    }
    tft.print(vbuf);
  } else {
    tft.setTextSize(1);
    tft.setTextColor(0x8410, ST77XX_BLACK);
    tft.setCursor(16, fieldY[idx]);
    switch (idx) {
      case FIELD_ANGLE:     snprintf(vbuf, sizeof(vbuf), "ANG: %d deg", angle_deg);                      break;
      case FIELD_SPEED:     snprintf(vbuf, sizeof(vbuf), "SPD: %.1f um/s", speed_um_s);                  break;
      case FIELD_START:     snprintf(vbuf, sizeof(vbuf), "START: %.0f um", start_pos_um);                break;
      case FIELD_CAL:       snprintf(vbuf, sizeof(vbuf), "CAL (press CAL)");                             break;
      case FIELD_TEMP:      snprintf(vbuf, sizeof(vbuf), "TEMP (press OPEN)");                           break;
      case FIELD_WAIT_TEMP: snprintf(vbuf, sizeof(vbuf), waitForTemp ? "WAIT TEMP: YES" : "WAIT TEMP: NO"); break;
    }
    tft.print(vbuf);
  }
}

void drawTempSubField(int idx, bool active) {
  const int fieldY[6] = { 60, 82, 104, 126, 148, 170 };
  tft.fillRect(0, fieldY[idx], LEFT_W, 20, ST77XX_BLACK);
  char vbuf[24];
  if (active) {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(6, fieldY[idx]);
    tft.print(">");
    tft.setCursor(22, fieldY[idx]);
    switch (idx) {
      case TFIELD_BACK:      snprintf(vbuf, sizeof(vbuf), "< BACK       ");                              break;
      case TFIELD_SETPOINT:  snprintf(vbuf, sizeof(vbuf), "SP: %5.1f C  ", tempSetpoint);               break;
      case TFIELD_KP:        snprintf(vbuf, sizeof(vbuf), "Kp: %5.1f    ", kp);                         break;
      case TFIELD_KI:        snprintf(vbuf, sizeof(vbuf), "Ki: %5.2f    ", ki);                          break;
      case TFIELD_KD:        snprintf(vbuf, sizeof(vbuf), "Kd: %5.0f    ", kd);                         break;
      case TFIELD_STARTSTOP: snprintf(vbuf, sizeof(vbuf), tempControlActive ? "CTRL: ON     " : "CTRL: OFF    "); break;
    }
    tft.print(vbuf);
  } else {
    tft.setTextSize(1);
    tft.setTextColor(0x8410, ST77XX_BLACK);
    tft.setCursor(16, fieldY[idx]);
    switch (idx) {
      case TFIELD_BACK:      snprintf(vbuf, sizeof(vbuf), "< BACK");                                    break;
      case TFIELD_SETPOINT:  snprintf(vbuf, sizeof(vbuf), "SP: %.1f C", tempSetpoint);                  break;
      case TFIELD_KP:        snprintf(vbuf, sizeof(vbuf), "Kp: %.1f", kp);                              break;
      case TFIELD_KI:        snprintf(vbuf, sizeof(vbuf), "Ki: %.2f", ki);                               break;
      case TFIELD_KD:        snprintf(vbuf, sizeof(vbuf), "Kd: %.0f", kd);                              break;
      case TFIELD_STARTSTOP: snprintf(vbuf, sizeof(vbuf), tempControlActive ? "CTRL: ON" : "CTRL: OFF"); break;
    }
    tft.print(vbuf);
  }
}

void drawSettingsHint(int) {
  tft.fillRect(0, 52, LEFT_W, 8, ST77XX_BLACK);
}


// =============================================================================
// Temperature bar column (right column, x = TCOL_X..SCREEN_W-1)
// Called every heartbeat tick (100 ms) from both run and settings screens.
// Dirty-state tracking: only repaints pixels that actually changed.
// =============================================================================
void updateTempColumn() {
  // Dirty-state: all statics start at sentinel values that force first draw.
  static int      prevFillH      = -1;
  static int      prevSpY        = -1;
  static uint16_t prevBarColor   = 0;
  static int      prevHeaterDuty = -1;
  static int      prevCtrlStatus = -1;  // 0=off 1=heat-dim 2=heat-blink 3=stable
  static float    prevTempDrawn  = NAN;
  static int      prevTempLabelY = -1;
  static bool     prevFault      = false;

  const int labelX = TBAR_X + TBAR_W + 3;  // label column starts here (right of bar)
  bool forceBarRedraw = false;

  // ---- Ctrl status (size-2, y=2) — only repaint when state changes or blink ticks ----
  int ctrlStatus;
  static bool stableState = false;
  
  if (!tempControlActive || isnan(lastTempC)) {
    stableState = false;
  } else {
    float err = fabsf(lastTempC - tempSetpoint);
    if (stableState && err > 3.0f) stableState = false;
    else if (!stableState && err <= 1.5f) stableState = true;
  }

  if (!tempControlActive) {
    ctrlStatus = 0;
  } else if (stableState) {
    ctrlStatus = 3;
  } else {
    ctrlStatus = ((millis() / 500) % 2) ? 2 : 1;
  }
  if (ctrlStatus != prevCtrlStatus) {
    tft.fillRect(TCOL_X + 1, 2, SCREEN_W - TCOL_X - 2, 16, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setCursor(TCOL_X + 2, 2);
    switch (ctrlStatus) {
      case 0: tft.setTextColor(0x8410,        ST77XX_BLACK); tft.print("OFF "); break;
      case 1: tft.setTextColor(ST77XX_BLACK,   ST77XX_BLACK); tft.print("HEAT"); break;
      case 2: tft.setTextColor(ST77XX_YELLOW,  ST77XX_BLACK); tft.print("HEAT"); break;
      case 3: tft.setTextColor(ST77XX_GREEN,   ST77XX_BLACK); tft.print("OK  "); break;
    }
    prevCtrlStatus = ctrlStatus;
  }

  // ---- Power % (size-2, y=19) — only repaint when duty changes ----
  int duty = (int)heaterDuty;
  if (duty != prevHeaterDuty) {
    char buf[12];
    snprintf(buf, sizeof(buf), "PWR:%3d%%", (int)((float)duty / 255.0f * 100.0f + 0.5f));
    tft.fillRect(TCOL_X + 1, 19, SCREEN_W - TCOL_X - 2, 16, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setCursor(TCOL_X + 2, 19);
    tft.print(buf);
    prevHeaterDuty = duty;
  }

  // ---- Setpoint line + label — only repaint when spY changes ----
  int spY = TBAR_BOT - 1 - (int)((float)(TBAR_H - 2) * constrain(tempSetpoint, 0.0f, 120.0f) / 120.0f);
  if (spY != prevSpY) {
    // Erase old line (full overhang width)
    if (prevSpY >= 0)
      tft.drawFastHLine(TBAR_X - 4, prevSpY, TBAR_W + 8, ST77XX_BLACK);
    // Clear entire label column so old temp + setpoint labels are gone
    tft.fillRect(labelX, TBAR_TOP, SCREEN_W - labelX - 1, TBAR_H, ST77XX_BLACK);
    prevTempLabelY = -1;   // force temp label redraw with updated collision check
    prevTempDrawn  = NAN;
    forceBarRedraw = true; // restore bar fill pixels overwritten by line erase

    // Draw new setpoint line (white, extends 4 px beyond bar each side)
    tft.drawFastHLine(TBAR_X - 4, spY, TBAR_W + 8, ST77XX_WHITE);

    // Setpoint label: value above the line, right of bar
    int spLabelY = constrain(spY - 17, TBAR_TOP, TBAR_BOT - 16);
    char buf[8];
    snprintf(buf, sizeof(buf), "%5.1f", tempSetpoint);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(labelX, spLabelY);
    tft.print(buf);

    prevSpY = spY;
  }

  // ---- Fault transition ----
  bool fault = isnan(lastTempC);
  if (fault != prevFault) {
    forceBarRedraw = true;
    prevFault      = fault;
    prevFillH      = -1;
    prevTempLabelY = -1;
    prevTempDrawn  = NAN;
  }

  // ---- Bar fill ----
  if (!fault) {
    float    clampedT = constrain(lastTempC, 0.0f, 120.0f);
    int      fillH    = (int)((float)(TBAR_H - 2) * clampedT / 120.0f);
    int      fillY    = TBAR_BOT - 1 - fillH;
    uint16_t barColor = stableState ? ST77XX_GREEN : ST77XX_RED;

    bool needFullBar = (barColor != prevBarColor) || forceBarRedraw || (prevFillH < 0);
    if (needFullBar) {
      tft.drawRect(TBAR_X, TBAR_TOP, TBAR_W, TBAR_H, ST77XX_CYAN);
      if (fillH < TBAR_H - 2)
        tft.fillRect(TBAR_X + 1, TBAR_TOP + 1, TBAR_W - 2, TBAR_H - 2 - fillH, ST77XX_BLACK);
      if (fillH > 0)
        tft.fillRect(TBAR_X + 1, fillY, TBAR_W - 2, fillH, barColor);
      tft.drawFastHLine(TBAR_X - 4, spY, TBAR_W + 8, ST77XX_WHITE);
      prevFillH    = fillH;
      prevBarColor = barColor;
    } else if (fillH != prevFillH) {
      // Delta draw: only repaint the rows that changed
      int prevFillY = TBAR_BOT - 1 - prevFillH;
      if (fillH > prevFillH) {
        tft.fillRect(TBAR_X + 1, fillY, TBAR_W - 2, prevFillY - fillY, barColor);
      } else {
        tft.fillRect(TBAR_X + 1, prevFillY, TBAR_W - 2, fillY - prevFillY, ST77XX_BLACK);
      }
      tft.drawFastHLine(TBAR_X - 4, spY, TBAR_W + 8, ST77XX_WHITE);
      prevFillH = fillH;
    }

    // ---- Temp label (size-2, centered vertically in fill region) ----
    char tBuf[8];
    snprintf(tBuf, sizeof(tBuf), "%5.1f", lastTempC);
    bool needsRedraw = (prevTempLabelY < 0);
    if (!needsRedraw) {
      char pBuf[8];
      snprintf(pBuf, sizeof(pBuf), "%5.1f", prevTempDrawn);
      needsRedraw = (strcmp(tBuf, pBuf) != 0);
    }
    if (needsRedraw) {
      int fillCenter = fillY + fillH / 2;
      int lY = constrain(fillCenter - 8, TBAR_TOP, TBAR_BOT - 16);
      if (abs(lY - spY) < 12)
        lY = constrain(spY + 2, TBAR_TOP, TBAR_BOT - 16);

      if (prevTempLabelY >= 0 && prevTempLabelY != lY)
        tft.fillRect(labelX, prevTempLabelY, SCREEN_W - labelX - 1, 16, ST77XX_BLACK);

      tft.setTextSize(2);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor(labelX, lY);
      tft.print(tBuf);
      prevTempDrawn  = lastTempC;
      prevTempLabelY = lY;
    }
  } else {
    // Fault: red fill + "FLT"
    if (forceBarRedraw) {
      tft.drawRect(TBAR_X, TBAR_TOP, TBAR_W, TBAR_H, ST77XX_CYAN);
      tft.fillRect(TBAR_X + 1, TBAR_TOP + 1, TBAR_W - 2, TBAR_H - 2, ST77XX_RED);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
      tft.setCursor(TBAR_X + 2, TBAR_TOP + TBAR_H / 2 - 4);
      tft.print("FLT");
      tft.fillRect(labelX, TBAR_TOP, SCREEN_W - labelX, TBAR_H, ST77XX_BLACK);
      tft.drawFastHLine(TBAR_X - 4, spY, TBAR_W + 8, ST77XX_WHITE);
      // Restore setpoint label (erased by the label area clear above)
      {
        char buf[8];
        snprintf(buf, sizeof(buf), "%5.1f", tempSetpoint);
        int spLabelY = constrain(spY - 17, TBAR_TOP, TBAR_BOT - 16);
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(labelX, spLabelY);
        tft.print(buf);
      }
    }
  }
}

void updateSettingsContent() {
  if (!settingsDirty) return;
  settingsDirty = false;

  if (inTempSubMenu) {
    int  curIdx    = (int)tempField;
    bool firstDraw = (prevTempFieldIdx < 0);
    if (firstDraw) {
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor((LEFT_W - 8 * 12) / 2, STATE_Y);
      tft.print("TEMP SET");
      drawSettingsHint(0);
      for (int i = 0; i < 6; i++) drawTempSubField(i, i == curIdx);
    } else if (prevTempFieldIdx != curIdx) {
      drawTempSubField(prevTempFieldIdx, false);
      drawTempSubField(curIdx, true);
      drawSettingsHint(0);
    } else {
      drawTempSubField(curIdx, true);
    }
    prevTempFieldIdx = curIdx;
  } else {
    int  curIdx    = (int)settingsField;
    bool firstDraw = (prevSettingsFieldIdx < 0);
    if (firstDraw) {
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor((LEFT_W - 8 * 12) / 2, STATE_Y);
      tft.print("SETTINGS");
      drawSettingsHint(curIdx);
      for (int i = 0; i < 6; i++) drawSettingsField(i, i == curIdx);
      char  vbuf[24];
      float dist_xa_um = stepsToUm(dist_xa_steps);
      tft.setTextSize(1);
      tft.setCursor(6, 192);
      if (dist_xa_steps > 0) {
        tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        snprintf(vbuf, sizeof(vbuf), "X-A: %.1f um    ", dist_xa_um);
      } else {
        tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        snprintf(vbuf, sizeof(vbuf), "NOT CALIBRATED      ");
      }
      tft.print(vbuf);
    } else if (prevSettingsFieldIdx != curIdx) {
      drawSettingsField(prevSettingsFieldIdx, false);
      drawSettingsField(curIdx, true);
      drawSettingsHint(curIdx);
    } else {
      drawSettingsField(curIdx, true);
    }
    prevSettingsFieldIdx = curIdx;
  }
}


// =============================================================================
// WiFi signal icon  (between A and X buttons, above top divider)
//   cx=100, cy=26: centred in the 90 px gap (x=55..145) between button boxes
//   bars: 0=disconnected(X), 1=poor, 2=fair, 3=good, 4=excellent
// =============================================================================
static int rssiToBars(int rssi) {
  if (rssi == 0)  return 0;
  if (rssi > -60) return 4;
  if (rssi > -70) return 3;
  if (rssi > -80) return 2;
  return 1;
}

static void drawTopArc(int16_t cx, int16_t cy, int16_t r, uint16_t color) {
  // Bresenham upper-semicircle: draws pixels where pixel_y <= cy
  int16_t x = 0, y = r, d = 3 - 2 * r;
  while (x <= y) {
    tft.drawPixel(cx + x, cy - y, color);
    tft.drawPixel(cx - x, cy - y, color);
    tft.drawPixel(cx + y, cy - x, color);
    tft.drawPixel(cx - y, cy - x, color);
    if (d < 0) d += 4 * x + 6;
    else { d += 4 * (x - y) + 10; y--; }
    x++;
  }
}

void drawWifiIcon(int bars) {
  const int16_t cx = 100, cy = 26;  // centred between buttons (gap x=55..145, centre=100)
  tft.fillRect(cx - 14, cy - 14, 28, 18, ST77XX_BLACK);  // erase old icon

  uint16_t color;
  switch (bars) {
    case 4: case 3: color = ST77XX_GREEN;  break;
    case 2:         color = ST77XX_YELLOW; break;
    case 1:         color = ST77XX_RED;    break;
    default:        color = 0x8410;        break;  // gray = disconnected
  }

  if (bars == 0) {
    // X mark — disconnected
    tft.drawLine(cx - 7, cy - 11, cx + 7, cy - 1, color);
    tft.drawLine(cx + 7, cy - 11, cx - 7, cy - 1, color);
    return;
  }
  tft.fillCircle(cx, cy, 2, color);
  if (bars >= 2) drawTopArc(cx, cy,  5, color);
  if (bars >= 3) drawTopArc(cx, cy,  9, color);
  if (bars >= 4) drawTopArc(cx, cy, 13, color);
}


// =============================================================================
// Full UI init (called once in setup)
// =============================================================================
void initUI() {
  tft.fillScreen(ST77XX_BLACK);
  tft.drawFastHLine(0, DIV_TOP_Y, LEFT_W, ST77XX_CYAN);
  tft.drawFastHLine(0, DIV_BOT_Y, LEFT_W, ST77XX_CYAN);
  tft.drawFastVLine(TCOL_X - 1, 0, SCREEN_H, ST77XX_CYAN);  // column divider
  updateButtons();
  drawWifiIcon(0);
  updateRunContent();
  updateTempColumn();
}


// =============================================================================
// setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);  // keep parseInt from blocking 1 s per 'm'/'v' command

  // Display — VSPI defaults: SCK=18, MISO=19, MOSI=23
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);   // keep backlight OFF until display is fully initialized
  SPI.begin(18, 19, 23, -1);
  tft.init(240, 320);
  tft.setRotation(3);
  tft.invertDisplay(false);

  // MAX31856 thermocouple amplifier (shares VSPI; SPI.begin() already called above)
  pinMode(MAX_DRDY, INPUT);
  thermo.begin();
  thermo.setThermocoupleType(MAX31856_TCTYPE_K);
  thermo.setConversionMode(MAX31856_CONTINUOUS);
  lastDrdyHighMs = millis();   // prevent false watchdog trigger during boot

  // Heater MOSFET — LEDC 1 kHz, 8-bit, starts off (10Hz is too low for ESP32 hardware timer divider and causes interrupt/timer stalls!)
  bool heaterOk = ledcAttach(HEATER_PIN, 1000, 8);
  ledcWrite(HEATER_PIN, 0);
  Serial.printf("[heater] ledcAttach GPIO%d: %s\n", HEATER_PIN, heaterOk ? "OK" : "FAILED");

  // Buttons
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_X, INPUT_PULLUP);
  pinMode(BTN_Y, INPUT_PULLUP);
  pinMode(LIMIT_SW_X, INPUT_PULLUP);
  pinMode(LIMIT_SW_Y, INPUT_PULLUP);
  hasHomed = !digitalRead(LIMIT_SW_X);  // already at home position on boot

  // Load saved settings (also calls updateMicronsPerStep internally)
  loadPrefs();

  // Stepper (must be before initUI so getCurrentPosition() reads 0, not garbage)
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper, true, 40);
    stepper->setEnablePin(enablePinStepper, true);  // active HIGH: ENA- tied to GND, MCU drives ENA+
    stepper->setAutoEnable(false);
    stepper->disableOutputs();
    stepper->setCurrentPosition(0);
    stepper->setSpeedInMilliHz(speedUmToMilliHz(speed_um_s));
    stepper->setAcceleration(2147483647);
  }

  // Draw UI, then enable backlight so users never see the white uninitialized screen
  initUI();
  digitalWrite(TFT_BL, HIGH);

  // WiFi — non-blocking station mode; server starts once connected (see loop)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);           // disable modem sleep — prevents periodic disconnections
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // 19.5 dBm: maximum TX power
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiStartMs = millis();

  delay(500);  // ESP32 UART is always ready; brief pause for USB-CDC enumeration
}


// =============================================================================
// Immediate WebSocket status push (also called by the 100 ms heartbeat)
// =============================================================================
static void sendWsJson() {
  if (ws.count() == 0) return;

  unsigned long now   = millis();
  float pos_um        = stepsToUm(stepper->getCurrentPosition());
  float actual_hz     = stepper->getCurrentSpeedInMilliHz() / 1000.0f;
  float actual_um_s   = actual_hz * microns_per_step * stepToUmFactor();
  float dist_xa_um    = stepsToUm(dist_xa_steps);
  unsigned long peel_elapsed = (appState == PEELING) ? (now - peel_start_ms) : 0UL;
  int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;

  const char *stateStr;
  switch (appState) {
    case MOVING:           stateStr = "MOVING";     break;
    case MOVING_TO_START:  stateStr = "TO_START";   break;
    case WAITING_FOR_TEMP: stateStr = "WAIT_TEMP";  break;
    case PEELING:          stateStr = "PEELING";    break;
    case HOMING:           stateStr = "HOMING";     break;
    case SETTINGS:         stateStr = "SETTINGS";   break;
    case CAL_HOMING:       stateStr = "CAL_HOME";   break;
    case CAL_RUNNING:      stateStr = "CAL_RUN";    break;
    default:               stateStr = "IDLE";       break;
  }

  char tempBuf[12];
  if (isnan(lastTempC)) snprintf(tempBuf, sizeof(tempBuf), "null");
  else                  snprintf(tempBuf, sizeof(tempBuf), "%.1f", lastTempC);

  char json[660];
  snprintf(json, sizeof(json),
    "{\"state\":\"%s\","
    "\"position\":%d,"
    "\"speed\":%d,"
    "\"pos_um\":%.2f,"
    "\"speed_um\":%.2f,"
    "\"speed_set\":%.1f,"
    "\"angle\":%d,"
    "\"spr\":%d,"
    "\"dist_xa_steps\":%d,"
    "\"dist_xa_um\":%.2f,"
    "\"start_pos_um\":%.1f,"
    "\"peel_elapsed_ms\":%lu,"
    "\"warning_active\":%s,"
    "\"settings_field\":%d,"
    "\"in_temp_sub\":%s,"
    "\"temp_field\":%d,"
    "\"btn\":[%s,%s,%s,%s],"
    "\"rssi\":%d,"
    "\"clients\":%d,"
    "\"temp_c\":%s,"
    "\"heater_duty\":%d,"
    "\"temp_setpoint\":%.1f,"
    "\"temp_ctrl_active\":%s,"
    "\"wait_for_temp\":%s,"
    "\"kp\":%.2f,"
    "\"ki\":%.3f,"
    "\"kd\":%.1f,"
    "\"ip\":\"%s\","
    "\"has_homed\":%s}",
    stateStr,
    (int)stepper->getCurrentPosition(),
    (int)actual_hz,
    pos_um,
    actual_um_s,
    speed_um_s,
    angle_deg,
    steps_per_rev,
    (int)dist_xa_steps,
    dist_xa_um,
    start_pos_um,
    peel_elapsed,
    (millis() < warningUntil) ? "true" : "false",
    (int)settingsField,
    inTempSubMenu ? "true" : "false",
    (int)tempField,
    btnDown[IDX_A] ? "true" : "false",
    btnDown[IDX_B] ? "true" : "false",
    btnDown[IDX_X] ? "true" : "false",
    btnDown[IDX_Y] ? "true" : "false",
    rssi,
    (int)ws.count(),
    tempBuf,
    (int)heaterDuty,
    tempSetpoint,
    tempControlActive ? "true" : "false",
    waitForTemp ? "true" : "false",
    kp,
    ki,
    kd,
    wifiIpStr,
    hasHomed ? "true" : "false"
  );

  if (Serial) {
    if (Serial.availableForWrite() >= strlen(json)) {
      Serial.println(json);
    }
  }
  ws.cleanupClients();
  if (ws.count() > 0) {
    bool canSend = true;
    for (AsyncWebSocketClient& c : ws.getClients()) {
      if (c.queueIsFull()) {
        canSend = false;
        break;
      }
    }
    if (canSend) ws.textAll(json);
  }
}


// =============================================================================
// loop
// =============================================================================
void loop() {
  unsigned long now = millis();

  // ---- Non-blocking WiFi / web server start ------------------------------------
  static bool webServerRegistered = false;
  static wl_status_t prevWifiStatus = WL_IDLE_STATUS;
  wl_status_t wifiStatus = WiFi.status();

  if (wifiStatus != prevWifiStatus) {
    if (wifiStatus == WL_CONNECTED) {
      String ip = WiFi.localIP().toString();
      snprintf(wifiIpStr, sizeof(wifiIpStr), "WiFi: %s", ip.c_str());
      MDNS.begin("peeling");
      if (!webServerRegistered) {
        ws.onEvent(onWsEvent);
        webServer.addHandler(&ws);
        webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
          req->send_P(200, "text/html", HTML_PAGE);
        });
        webServer.begin();
        webServerRegistered = true;
      }
      serverStarted = true;
    } else if (prevWifiStatus == WL_CONNECTED) {
      snprintf(wifiIpStr, sizeof(wifiIpStr), "WiFi: reconnecting");
    }
    prevWifiStatus = wifiStatus;
  }

  if (!serverStarted && millis() - wifiStartMs > 10000) {
    serverStarted = true;  // give up — run offline
    snprintf(wifiIpStr, sizeof(wifiIpStr), "WiFi: offline");
  }

  // ---- Button processing -------------------------------------------------------
  bool virtDown[4];
  portENTER_CRITICAL(&wsMux);
  for (int i = 0; i < 4; i++) virtDown[i] = virtualBtn[i];
  portEXIT_CRITICAL(&wsMux);

  bool curDown[4] = {
    !digitalRead(BTN_A) || virtDown[IDX_A],
    !digitalRead(BTN_B) || virtDown[IDX_B],
    !digitalRead(BTN_X) || virtDown[IDX_X],
    !digitalRead(BTN_Y) || virtDown[IDX_Y]
  };

  bool btnChanged = false;
  for (int i = 0; i < 4; i++) {
    if (curDown[i] && !btnDown[i]) {
      // Press
      btnDown[i]      = true;
      btnPressAt[i]   = now;
      btnLongFired[i] = false;
      btnRepeatAt[i]  = now + LONG_PRESS_MS;
      onButtonPress(i);
      btnChanged = true;
    } else if (!curDown[i] && btnDown[i]) {
      // Release
      btnDown[i] = false;
      onButtonRelease(i);
      btnChanged = true;
    } else if (curDown[i] && btnDown[i]) {
      if (!btnLongFired[i] && now >= btnPressAt[i] + LONG_PRESS_MS) {
        // Long press fires once
        btnLongFired[i] = true;
        btnRepeatAt[i]  = now + REPEAT_MS;
        onButtonLong(i);
        btnChanged = true;
      } else if (btnLongFired[i] && now >= btnRepeatAt[i]) {
        // Repeat
        btnRepeatAt[i] = now + REPEAT_MS;
        onButtonRepeat(i);
        btnChanged = true;
      }
    }
  }
  if (btnChanged) {
    updateButtons();
    sendWsJson();   // immediate push — don't wait for 100 ms heartbeat
  }

  // ---- Limit switch polling ---------------------------------------------------
  bool curLimX = !digitalRead(LIMIT_SW_X);
  bool curLimY = !digitalRead(LIMIT_SW_Y);

  // Edge + debounce for MOVING safety abort: a switch may already be pressed when
  // a new move is commanded (e.g. right after homing), so level-only detection
  // would abort the move before the motor escapes the switch.
  if  (curLimX && !limitXPrev) limitXStableAt = now;
  else if (!curLimX)            limitXStableAt = 0;
  if  (curLimY && !limitYPrev) limitYStableAt = now;
  else if (!curLimY)            limitYStableAt = 0;
  bool xNewPress = curLimX && limitXStableAt > 0 && (now - limitXStableAt) >= LIMIT_DEBOUNCE_MS;
  bool yNewPress = curLimY && limitYStableAt > 0 && (now - limitYStableAt) >= LIMIT_DEBOUNCE_MS;
  if (xNewPress) limitXStableAt = 0;   // consume: fire once per press
  if (yNewPress) limitYStableAt = 0;
  limitXPrev = curLimX;
  limitYPrev = curLimY;

  if (appState == HOMING || appState == CAL_HOMING) {
    if (xNewPress) {                            // home-end switch triggered (edge+debounce, same as MOVING)
      stepper->forceStop();
      while (stepper->isRunning()) {}           // wait for PIO to flush buffered steps
      stepper->setCurrentPosition(0);
      if (appState == CAL_HOMING) {
        stepper->setSpeedInHz(calSpeedHz());
        stepper->runForward();
        limitYStableAt = 0;   // discard any Y noise accumulated during homing
        limitYPrev     = false;
        appState = CAL_RUNNING;
      } else {
        hasHomed = true;
        disableMotor();
        appState = IDLE;
      }
      updateButtons();
    }
  } else if (appState == CAL_RUNNING) {
    if (yNewPress) {                            // far-end switch (GPIO 15): edge+debounce guards against motor-startup transients
      dist_xa_steps = stepper->getCurrentPosition();
      stepper->forceStop();
      disableMotor();
      saveCalibration();
      hasHomed = false;
      appState = IDLE;
      updateButtons();
    }
  } else if (appState == MOVING || appState == MOVING_TO_START || appState == WAITING_FOR_TEMP || appState == PEELING) {
    if (xNewPress || yNewPress) {               // new contact only — not a stale press
      if (yNewPress) {
        hasHomed = false;          // at far end — need to home before settings
      } else if (xNewPress) {
        hasHomed = true;
        stepper->forceStop();
        while (stepper->isRunning()) {}
        stepper->setCurrentPosition(0);
      }
      abortAndIdle();
      updateButtons();
    }
  } else if (appState == IDLE) {
    if (xNewPress || yNewPress) {
      if (yNewPress) hasHomed = false;
      if (xNewPress) {
        hasHomed = true;
        stepper->setCurrentPosition(0);
      }
      updateButtons();
    }
  }

  // ---- Serial command parser ---------------------------------------------------
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case 's':
        hasHomed = false;
        abortAndIdle();
        updateButtons();
        break;
      case 'v': {
        int32_t hz = Serial.parseInt();
        if (hz > 0) stepper->setSpeedInHz(hz);
        break;
      }
      case 'm': {
        int32_t pos = Serial.parseInt();
        int32_t maxPos = dist_xa_steps > 0 ? dist_xa_steps : 478085;
        pos = constrain(pos, (int32_t)0, maxPos);
        appState = MOVING;
        enableMotor();
        stepper->setSpeedInHz(100);
        stepper->moveTo(pos);
        updateButtons();
        break;
      }
      case 'h': {
        // h<duty>  — set heater duty 0–255 (e.g. h128 = 50%)
        int32_t duty = Serial.parseInt();
        if (tempControlActive) {
          Serial.println("{\"error\":\"heater manual override blocked — temp control is active\"}");
        } else {
          duty = constrain(duty, 0, 255);
          heaterDuty = (uint8_t)duty;
          ledcWrite(HEATER_PIN, heaterDuty);
          Serial.printf("{\"heater_duty\":%d,\"heater_freq_hz\":%u}\n",
                        (int)heaterDuty, (unsigned)ledcReadFreq(HEATER_PIN));
        }
        break;
      }
      case 'b': {
        // Virtual button inject from serial bridge: bA1=pressA  bA0=releaseA
        // Format: 'b' <letter A/B/X/Y> <'1'=press / '0'=release>
        unsigned long t0 = millis();
        while (Serial.available() < 2 && millis() - t0 < 10) { /* spin ≤10 ms */ }
        if (Serial.available() >= 2) {
          char letter = (char)Serial.read();
          char state  = (char)Serial.read();
          int  idx    = -1;
          if      (letter == 'A') idx = IDX_A;
          else if (letter == 'B') idx = IDX_B;
          else if (letter == 'X') idx = IDX_X;
          else if (letter == 'Y') idx = IDX_Y;
          if (idx >= 0) {
            bool pressed = (state == '1');
            portENTER_CRITICAL(&wsMux);
            virtualBtn[idx] = pressed;
            portEXIT_CRITICAL(&wsMux);
          }
        }
        break;
      }
    }
  }

  // ---- State machine -----------------------------------------------------------
  switch (appState) {
    case MOVING:
      if (!stepper->isRunning()) {
        disableMotor();
        if (stepper->getCurrentPosition() != 0) {
          hasHomed = false;
        }
        appState = IDLE;
        updateButtons();
      }
      break;

    case HOMING:
      // Motor runs backward until limit switch triggers (handled in switch debounce section above)
      break;

    case MOVING_TO_START:
      // Arm peel timer on arrival
      if (startPeelAt == 0 && !stepper->isRunning()) {
        startPeelAt = now + 100;
      }
      // After 100 ms pause: enter WAITING_FOR_TEMP or start peeling immediately
      if (startPeelAt > 0 && now >= startPeelAt) {
        startPeelAt = 0;
        if (waitForTemp) {
          appState = WAITING_FOR_TEMP;
        } else {
          startPeeling();
        }
        updateButtons();
      }
      break;

    case WAITING_FOR_TEMP:
      // Start peel once temperature is within ±2 °C of setpoint
      if (!isnan(lastTempC) && fabsf(lastTempC - tempSetpoint) <= 2.0f) {
        startPeeling();
        updateButtons();
      }
      break;

    case PEELING:
      // Motor reached dist_xa_steps
      if (!stepper->isRunning()) {
        disableMotor();
        if (tempCtrlAutoEnabled) {
          tempControlActive   = false;
          heaterDuty          = 0;
          ledcWrite(HEATER_PIN, 0);
          tempCtrlAutoEnabled = false;
        }
        hasHomed = false;
        appState = IDLE;
        updateButtons();
      }
      break;

    default:
      break;
  }

  // ---- Periodic display + serial heartbeat (100 ms) ---------------------------
  if (now - previousMillis >= HEARTBEAT_MS) {
    previousMillis = now;

    // Screen mode transition: clear content area on mode switch
    bool needSettings = (appState == SETTINGS);
    if (needSettings != inSettingsScreen) {
      clearContent();
      inSettingsScreen = needSettings;
      settingsDirty    = true;
      if (needSettings) prevSettingsFieldIdx = -1;
      else              runScreenDirty = true;  // force full run-screen redraw
    }

    if (inSettingsScreen) {
      updateSettingsContent();
    } else {
      updateRunContent();
    }

    // Redraw dividers and temperature column (may be overwritten by clearContent)
    tft.drawFastHLine(0, DIV_TOP_Y, LEFT_W, ST77XX_CYAN);
    tft.drawFastHLine(0, DIV_BOT_Y, LEFT_W, ST77XX_CYAN);
    tft.drawFastVLine(TCOL_X - 1, 0, SCREEN_H, ST77XX_CYAN);
    updateTempColumn();

    // IP address centered between bottom buttons (run screen only, redraws only on change).
    {
      static char prevIpDrawn[32] = "\x01";  // sentinel — forces first draw
      char ipDisp[32];
      if (!inSettingsScreen) {
        const char *src = (strncmp(wifiIpStr, "WiFi: ", 6) == 0)
                          ? wifiIpStr + 6 : wifiIpStr;
        strncpy(ipDisp, src, sizeof(ipDisp) - 1);
        ipDisp[sizeof(ipDisp) - 1] = '\0';
      } else {
        ipDisp[0] = '\0';
      }
      if (strcmp(ipDisp, prevIpDrawn) != 0 || ipStripDirty) {
        ipStripDirty = false;
        // Clear only the space between the two button boxes (left column)
        const int areaX = BTN_LEFT_X + BTN_W + 1;            // 56
        const int areaW = BTN_RIGHT_X - areaX - 1;            // 89 px
        const int ipY   = BTN_BOT_Y + (BTN_H - 16) / 2;     // vertically centred (215)
        tft.fillRect(areaX, BTN_BOT_Y, areaW, BTN_H, ST77XX_BLACK);
        if (ipDisp[0] != '\0') {
          int textW = (int)strlen(ipDisp) * 12;               // textSize 2 → 12 px/char
          int textX = (BTN_LEFT_X + BTN_W + BTN_RIGHT_X - textW) / 2;
          if (textX < areaX) textX = areaX;                   // left clamp
          tft.setTextSize(2);
          tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
          tft.setCursor(textX, ipY);
          tft.print(ipDisp);
        }
        strncpy(prevIpDrawn, ipDisp, sizeof(prevIpDrawn) - 1);
        prevIpDrawn[sizeof(prevIpDrawn) - 1] = '\0';
      }
    }

    // ---- Temperature reading (non-blocking, rate-limited, with watchdog) -------
    {
      bool drdyLow = (digitalRead(MAX_DRDY) == LOW);
      if (drdyLow) {
        unsigned long nowMs = millis();
        if (nowMs - lastTempReadMs >= 130) {       // one read per ~143 ms conversion window
          float t      = thermo.readThermocoupleTemperature();
          lastTempC    = thermo.readFault() ? NAN : t;
          lastTempReadMs = nowMs;
          lastDrdyHighMs = nowMs;                  // successful read = chip alive

          // ---- Temperature PID control ----------------------------------------
          if (tempControlActive) {
            // Safety cutoffs — force heater off and skip PID
            if (isnan(lastTempC) || lastTempC >= 120.0f) {
              heaterDuty = 0;
              ledcWrite(HEATER_PIN, 0);
            } else if (lastPidRunMs == 0) {
              // First tick: record timestamp, skip computation to get a valid dt next tick
              lastPidRunMs = nowMs;
            } else {
              float dt    = (nowMs - lastPidRunMs) / 1000.0f;
              lastPidRunMs = nowMs;
              float error = tempSetpoint - lastTempC;
              float duty_pct;

              if (error > PID_BAND) {
                duty_pct = 100.0f;                 // bang-bang: full power
              } else if (error < -PID_BAND) {
                duty_pct = 0.0f;                   // bang-bang: off
              } else {
                // PID zone
                float p_term = kp * error;

                pidIntegral += error * dt;
                float i_limit = 50.0f / ki;
                pidIntegral  = constrain(pidIntegral, -i_limit, i_limit);
                float i_term = ki * pidIntegral;

                float raw_deriv     = (error - pidLastError) / dt;
                pidFilteredDeriv    = 0.7f * pidFilteredDeriv + 0.3f * raw_deriv;
                float d_term        = kd * pidFilteredDeriv;
                pidLastError        = error;

                duty_pct = constrain(p_term + i_term + d_term, 0.0f, 100.0f);
              }

              heaterDuty = (uint8_t)(duty_pct * 255.0f / 100.0f + 0.5f);
              ledcWrite(HEATER_PIN, heaterDuty);
            }
          }
        }
        // Watchdog: DRDY stuck LOW for >3 s means chip is confused — reinit
        if (millis() - lastDrdyHighMs > 3000) {
          thermo.begin();
          thermo.setThermocoupleType(MAX31856_TCTYPE_K);
          thermo.setConversionMode(MAX31856_CONTINUOUS);
          lastTempC      = NAN;
          lastDrdyHighMs = millis();
        }
      } else {
        lastDrdyHighMs = millis();                 // DRDY HIGH = chip healthy
      }
    }

    // Periodic JSON heartbeat — Serial + WebSocket broadcast
    {
      float pos_um      = stepsToUm(stepper->getCurrentPosition());
      float actual_hz   = stepper->getCurrentSpeedInMilliHz() / 1000.0f;
      float actual_um_s = actual_hz * microns_per_step * stepToUmFactor();
      float dist_xa_um  = stepsToUm(dist_xa_steps);
      unsigned long peel_elapsed = (appState == PEELING) ? (now - peel_start_ms) : 0UL;
      int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
      int currBars    = rssiToBars(rssi);
      int clientCount = (int)ws.count();
      if (currBars != prevRssiBars || clientCount != prevClientCount) {
        drawWifiIcon(currBars);
        // client count to the right of WiFi icon (icon centre cx=100, ends ~x=113)
        tft.fillRect(114, 18, 22, 10, ST77XX_BLACK);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.setCursor(116, 20);
        tft.print(clientCount);
        prevRssiBars   = currBars;
        prevClientCount = clientCount;
      }

      const char *stateStr;
      switch (appState) {
        case MOVING:           stateStr = "MOVING";     break;
        case MOVING_TO_START:  stateStr = "TO_START";   break;
        case WAITING_FOR_TEMP: stateStr = "WAIT_TEMP";  break;
        case PEELING:          stateStr = "PEELING";    break;
        case HOMING:           stateStr = "HOMING";     break;
        case SETTINGS:         stateStr = "SETTINGS";   break;
        case CAL_HOMING:       stateStr = "CAL_HOME";   break;
        case CAL_RUNNING:      stateStr = "CAL_RUN";    break;
        default:               stateStr = "IDLE";       break;
      }

      char tempBuf[12];
      if (isnan(lastTempC)) snprintf(tempBuf, sizeof(tempBuf), "null");
      else                  snprintf(tempBuf, sizeof(tempBuf), "%.1f", lastTempC);

      char json[660];
      snprintf(json, sizeof(json),
        "{\"state\":\"%s\","
        "\"position\":%d,"
        "\"speed\":%d,"
        "\"pos_um\":%.2f,"
        "\"speed_um\":%.2f,"
        "\"speed_set\":%.1f,"
        "\"angle\":%d,"
        "\"spr\":%d,"
        "\"dist_xa_steps\":%d,"
        "\"dist_xa_um\":%.2f,"
        "\"start_pos_um\":%.1f,"
        "\"peel_elapsed_ms\":%lu,"
        "\"warning_active\":%s,"
        "\"settings_field\":%d,"
        "\"in_temp_sub\":%s,"
        "\"temp_field\":%d,"
        "\"btn\":[%s,%s,%s,%s],"
        "\"rssi\":%d,"
        "\"clients\":%d,"
        "\"temp_c\":%s,"
        "\"heater_duty\":%d,"
        "\"temp_setpoint\":%.1f,"
        "\"temp_ctrl_active\":%s,"
        "\"wait_for_temp\":%s,"
        "\"kp\":%.2f,"
        "\"ki\":%.3f,"
        "\"kd\":%.1f,"
        "\"ip\":\"%s\","
        "\"has_homed\":%s}",
        stateStr,
        (int)stepper->getCurrentPosition(),
        (int)actual_hz,
        pos_um,
        actual_um_s,
        speed_um_s,
        angle_deg,
        steps_per_rev,
        (int)dist_xa_steps,
        dist_xa_um,
        start_pos_um,
        peel_elapsed,
        (millis() < warningUntil) ? "true" : "false",
        (int)settingsField,
        inTempSubMenu ? "true" : "false",
        (int)tempField,
        btnDown[IDX_A] ? "true" : "false",
        btnDown[IDX_B] ? "true" : "false",
        btnDown[IDX_X] ? "true" : "false",
        btnDown[IDX_Y] ? "true" : "false",
        rssi,
        (int)ws.count(),
        tempBuf,
        (int)heaterDuty,
        tempSetpoint,
        tempControlActive ? "true" : "false",
        waitForTemp ? "true" : "false",
        kp,
        ki,
        kd,
        wifiIpStr,
        hasHomed ? "true" : "false"
      );

      if (Serial) Serial.println(json);
      ws.cleanupClients();
      if (ws.count() > 0) ws.textAll(json);
    }
  }
}
