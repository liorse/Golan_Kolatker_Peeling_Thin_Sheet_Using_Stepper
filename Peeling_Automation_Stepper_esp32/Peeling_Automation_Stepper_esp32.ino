// =============================================================================
// Project   : Peeling Thin Sheet Using Stepper Motor — ESP32 Controller
// File      : Peeling_Automation_Stepper_esp32.ino
// Author    : Lior Segev
// Version   : 4.3.0-esp32
// Date      : June 3, 2026
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
//   GPIO 13  — LIMIT_SW_X (home/X limit switch, active-low)
//   GPIO 36  — LIMIT_SW_Y (far/Y limit switch, active-low, external 10kΩ pull-up to 3.3V)
//   GPIO  2  — TFT_RST
//   GPIO 17  — TFT_DC
//   GPIO  5  — TFT_CS
//   GPIO 18  — SPI SCK  (VSPI default, shared with MAX31856)
//   GPIO 23  — SPI MOSI (VSPI default, shared with MAX31856)
//   GPIO 19  — SPI MISO (VSPI default; MAX31856 SDO)
//   GPIO 16  — TFT_BL
//   GPIO 21  — MAX31856 CS  (thermocouple amplifier)
//   GPIO 22  — MAX31856 DRDY (data-ready; LOW when conversion complete)
//   GPIO 32  — HEATER_PIN (LEDC ch0, 1 kHz PWM → N-channel MOSFET gate)
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
//   Commands: 'm'<int32> move to step pos, 's' stop, 'v'<int> set Hz
//   Heartbeat every 100 ms:
//     {"state":N,"position":N,"speed":N,"pos_um":F,"speed_um":F,"angle":N,"spr":N}
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
#define LIMIT_SW_X 13  // home/X limit switch (active-low, INPUT_PULLUP)
#define LIMIT_SW_Y 36  // far/Y  limit switch (active-low, external 10kΩ pull-up) — input-only pin, no internal pull-up

// ---- MAX31856 thermocouple amplifier (VSPI shared with display) ---------------
#define MAX_CS    21
#define MAX_DRDY  22

// ---- Heater MOSFET (LEDC, 10 Hz, 8-bit) ------------------------------------
#define HEATER_PIN  32

// ---- Display geometry -------------------------------------------------------
#define SCREEN_W   240
#define SCREEN_H   240
#define X_OFF      ((320 - SCREEN_W) / 2)   // centers 240-px content in 320-px landscape

#define BTN_W       52
#define BTN_H       28
#define BTN_LEFT_X   (3 + X_OFF)
#define BTN_RIGHT_X (SCREEN_W - BTN_W - 3 + X_OFF)
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
#define BAR_X      (20 + X_OFF)
#define BAR_Y     178   // 12 px bar → ends 190; bottom divider at 207
#define BAR_W     200
#define BAR_H      12
#define TEMP_Y    193   // textSize 1 (8 px) → ends 201; fits gap between bar and divider at 207

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

// ---- Persistent storage -----------------------------------------------------
#define EEPROM_MAGIC  0x50454C34u   // "PEL4" — microstep field removed
#define EEPROM_ADDR   0
#define EEPROM_SIZE   64

struct SavedSettings {
  uint32_t magic;
  int      angle_deg;
  float    speed_um_s;
  float    start_pos_um;
  int32_t  dist_xa_steps;
};

// ---- Application state ------------------------------------------------------
enum AppState {
  IDLE,
  MOVING,            // serial-commanded move (no auto-peel)
  MOVING_TO_START,   // button-triggered: moves to start_pos, then auto-peels
  PEELING,
  HOMING,
  SETTINGS,
  CAL_HOMING,
  CAL_RUNNING
};
AppState appState = IDLE;

enum SettingsField { FIELD_SPEED, FIELD_ANGLE, FIELD_START, FIELD_CAL };
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
unsigned long limitXStableAt = 0;
unsigned long limitYStableAt = 0;
const unsigned long LIMIT_DEBOUNCE_MS = 2;

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
  // Targets ≈1 mm/s (1000 µm/s) at the motor regardless of microstep setting.
  int hz = (int)(1000.0f / microns_per_step);
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
    angle_deg     = s.angle_deg;
    speed_um_s    = s.speed_um_s;
    start_pos_um  = s.start_pos_um;
    dist_xa_steps = s.dist_xa_steps;
  }
}

void saveAll() {
  SavedSettings s;
  s.magic         = EEPROM_MAGIC;
  s.angle_deg     = angle_deg;
  s.speed_um_s    = speed_um_s;
  s.start_pos_um  = start_pos_um;
  s.dist_xa_steps = dist_xa_steps;
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
canvas{image-rendering:pixelated;max-width:min(100vw,100vh);max-height:min(100vw,100vh);width:480px;height:480px;}
#ws-status{color:#888;font-size:12px;margin-top:6px;font-family:sans-serif;}
#serial-badge{display:none;color:#00fc00;font-weight:bold;margin-left:8px;font-size:13px;}
#log-status{font-family:monospace;font-size:11px;margin-top:4px;text-align:center;line-height:1.6;min-height:1.6em;}
</style>
</head>
<body>
<canvas id="c" width="240" height="240"></canvas>
<div id="ws-status">connecting... <span id="serial-badge">&#128268; SERIAL</span></div>
<div id="log-status"></div>
<script>
const SW=240,SH=240;
const BTN_W=52,BTN_H=28,BTN_LEFT_X=3,BTN_RIGHT_X=SW-BTN_W-3,BTN_TOP_Y=3,BTN_BOT_Y=SH-BTN_H-3;
const DIV_TOP_Y=33,DIV_BOT_Y=207;
const STATE_Y=36,POS_Y=56,SETSPD_Y=76,RUNSPD_Y=96,ANGLE_Y=116,TOEND_Y=136,PEELT_Y=156;
const BAR_X=20,BAR_Y=178,BAR_W=200,BAR_H=12;
const TEMP_Y=193;
const fieldY=[68,92,116,140];
const C={BK:'#000000',WH:'#ffffff',CY:'#00f8ff',GR:'#00fc00',YE:'#f8fc00',RE:'#f80000',GY:'#848484'};
const STATE_COL={IDLE:C.GR,MOVING:C.YE,TO_START:C.YE,PEELING:C.YE,HOMING:C.CY,CAL_HOME:C.CY,CAL_RUN:C.CY,SETTINGS:C.GR};

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
  ctx.beginPath();ctx.moveTo(0,DIV_TOP_Y);ctx.lineTo(SW,DIV_TOP_Y);ctx.stroke();
  ctx.beginPath();ctx.moveTo(0,DIV_BOT_Y);ctx.lineTo(SW,DIV_BOT_Y);ctx.stroke();
}

function rssiToBars(rssi){
  if(!rssi||rssi===0) return 0;
  if(rssi>-60) return 4;
  if(rssi>-70) return 3;
  if(rssi>-80) return 2;
  return 1;
}

function drawWifiIcon(rssi,clients){
  const cx=120,cy=26,bars=rssiToBars(rssi);
  ctx.fillStyle=C.BK;ctx.fillRect(cx-14,cy-14,28,18);
  ctx.fillStyle=C.BK;ctx.fillRect(134,18,22,10);

  let color;
  if(bars>=3) color=C.GR;
  else if(bars===2) color=C.YE;
  else if(bars===1) color=C.RE;
  else color=C.GY;

  if(bars===0){
    ctx.strokeStyle=color;ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(cx-7,cy-11);ctx.lineTo(cx+7,cy-1);ctx.stroke();
    ctx.beginPath();ctx.moveTo(cx+7,cy-11);ctx.lineTo(cx-7,cy-1);ctx.stroke();
    setFont(1);ctx.fillStyle=C.CY;ctx.fillText(String(clients||0),136,20);
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
  setFont(1);ctx.fillStyle=C.CY;ctx.fillText(String(clients||0),136,20);
}

function drawButtons(d){
  const btn=d.btn||[false,false,false,false];
  if(d.state==='SETTINGS'){
    drawButtonBox(BTN_LEFT_X, BTN_TOP_Y,'UP',  btn[0],2);
    drawButtonBox(BTN_LEFT_X, BTN_BOT_Y,'DOWN',btn[1],2);
    drawButtonBox(BTN_RIGHT_X,BTN_TOP_Y,d.settings_field===3?'CAL':'+',btn[2],2);
    drawButtonBox(BTN_RIGHT_X,BTN_BOT_Y,'-',btn[3],2);
  } else {
    let aLbl,bLbl;
    if(d.state==='IDLE'){aLbl=d.dist_xa_steps>0?'GO':'!CAL';bLbl=d.position===0?'SET':'HOME';}
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
  const padded=st.padStart(Math.floor((20+st.length)/2)).padEnd(20);
  drawText(padded,0,STATE_Y,2,col,C.BK);

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

  if(d.temp_c===null||d.temp_c===undefined){
    drawText('T:  -- FAULT --     ',6,TEMP_Y,1,C.RE,C.BK);
  } else {
    drawText(('T: '+d.temp_c.toFixed(1)+' C').padEnd(20),6,TEMP_Y,1,C.GR,C.BK);
  }
}

function drawSettingsField(d,idx,active){
  ctx.fillStyle=C.BK;ctx.fillRect(0,fieldY[idx],SW,20);
  let vbuf;
  if(active){
    drawText('>',6,fieldY[idx],2,C.YE,C.BK);
    switch(idx){
      case 1:vbuf='ANG: '+pad(d.angle,2)+' deg   ';break;
      case 0:vbuf='SPD:'+d.speed_set.toFixed(1)+'um/s  ';break;
      case 2:vbuf='ST: '+d.start_pos_um.toFixed(0)+'um    ';break;
      case 3:vbuf='CAL: press CAL';break;
    }
    drawText(vbuf,22,fieldY[idx],2,C.YE,C.BK);
  } else {
    switch(idx){
      case 1:vbuf='ANG: '+d.angle+' deg';break;
      case 0:vbuf='SPD: '+d.speed_set.toFixed(1)+' um/s';break;
      case 2:vbuf='START: '+d.start_pos_um.toFixed(0)+' um';break;
      case 3:vbuf='CAL (press CAL)';break;
    }
    drawText(vbuf,16,fieldY[idx],1,C.GY,C.BK);
  }
}

function drawSettingsScreen(d){
  drawText('SETTINGS',Math.floor((SW-8*12)/2),STATE_Y,2,C.WH,C.BK);
  ctx.fillStyle=C.BK;ctx.fillRect(0,54,SW,12);
  for(let i=0;i<4;i++) drawSettingsField(d,i,i===d.settings_field);
  ctx.fillStyle=C.BK;ctx.fillRect(0,178,SW,12);
  if(d.dist_xa_steps>0){
    drawText(padEnd('X-A: '+d.dist_xa_um.toFixed(1)+' um',22),6,178,1,C.GR,C.BK);
  } else {
    drawText('NOT CALIBRATED      ',6,178,1,C.RE,C.BK);
  }
  ctx.fillStyle=C.BK;ctx.fillRect(0,190,SW,12);
  drawText(padEnd(d.ip||'',28),6,190,1,C.CY,C.BK);
}

let lastInSettings=null;
const serialBadge=document.getElementById('serial-badge');
const logStatusEl=document.getElementById('log-status');

function render(d){
  // Reveal the SERIAL badge once we receive a transport:serial frame (stays visible)
  if(d.transport==='serial') serialBadge.style.display='inline';

  // Serial cable disconnected — show reconnecting notice without touching the canvas
  if(d.serial_lost){
    statusEl.firstChild.textContent='reconnecting... ';
    logStatusEl.innerHTML='';
    return;
  }

  // Log status indicator — only shown when Python bridge is active and logging
  if(d.log_info&&d.log_info.active){
    logStatusEl.innerHTML=
      '&#x1F4DD;&nbsp;<span style="color:#f8fc00">'+d.log_info.filename+'</span><br>'
      +'<span style="color:#888">→&nbsp;'+d.log_info.folder+'</span>';
  } else {
    logStatusEl.innerHTML='';
  }

  statusEl.firstChild.textContent='connected ';
  const inSettings=d.state==='SETTINGS';
  if(inSettings!==lastInSettings){
    ctx.fillStyle=C.BK;ctx.fillRect(0,DIV_TOP_Y+1,SW,DIV_BOT_Y-DIV_TOP_Y-1);
    lastInSettings=inSettings;
  }
  if(inSettings) drawSettingsScreen(d); else drawRunScreen(d);
  drawButtons(d);
  drawWifiIcon(d.rssi||0,d.clients||0);
  drawDividers();
}

ctx.fillStyle=C.BK;ctx.fillRect(0,0,SW,SH);
drawDividers();

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
                             1.0f, 1000.0f);
      break;
    case FIELD_START: {
      float maxStart = (dist_xa_um > 0.0f) ? dist_xa_um : 1e6f;
      start_pos_um = constrain(start_pos_um + (float)dir * (fast ? 100.0f : 10.0f),
                               0.0f, maxStart);
      break;
    }
    case FIELD_CAL:
      break;
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
  stepper->forceStop();
  disableMotor();
  startPeelAt = 0;
  appState    = IDLE;
}

void startMoveToStart() {
  appState = MOVING_TO_START;
  enableMotor();
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
  stepper->setSpeedInHz(calSpeedHz());
  stepper->runBackward();
}

void startCal() {
  appState = CAL_HOMING;
  enableMotor();
  stepper->setSpeedInHz(calSpeedHz());
  stepper->runBackward();
}

void cycleSettingsField() {
  switch (settingsField) {
    case FIELD_SPEED: settingsField = FIELD_ANGLE; break;
    case FIELD_ANGLE: settingsField = FIELD_START; break;
    case FIELD_START: settingsField = FIELD_CAL;   break;
    case FIELD_CAL:
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
        if (stepper->getCurrentPosition() == 0) {
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
      if (idx == IDX_A) {          // A = navigate up
        switch (settingsField) {
          case FIELD_ANGLE: settingsField = FIELD_SPEED; break;
          case FIELD_START: settingsField = FIELD_ANGLE; break;
          case FIELD_CAL:   settingsField = FIELD_START; break;
          default: break;          // FIELD_SPEED: already at top, no-op
        }
        settingsDirty = true;
      } else if (idx == IDX_X) {   // X = increment or CAL trigger
        if (settingsField == FIELD_CAL) {
          startCal();
        } else {
          doIncrement(+1, false);
        }
      } else if (idx == IDX_Y) {   // Y = decrement (no-op on CAL field)
        if (settingsField != FIELD_CAL) {
          doIncrement(-1, false);
        }
      }
      // B: handled in onButtonRelease (navigate down / exit+save)
      break;

    case MOVING:
    case MOVING_TO_START:
    case PEELING:
    case HOMING:
    case CAL_HOMING:
    case CAL_RUNNING:
      if (idx == IDX_A) {          // A = UI stop button
        abortAndIdle();
      }
      break;
  }
}

void onButtonRelease(int idx) {
  if (appState == SETTINGS && idx == IDX_B && !btnLongFired[IDX_B]) {
    if (justEnteredSettings) {
      justEnteredSettings = false;  // swallow the release that opened settings
    } else {
      cycleSettingsField();
    }
  }
}

void onButtonLong(int idx) {
  if (appState == SETTINGS && settingsField != FIELD_CAL) {
    if (idx == IDX_X) doIncrement(+1, false);
    else if (idx == IDX_Y) doIncrement(-1, false);
  }
}

void onButtonRepeat(int idx) {
  if (appState != SETTINGS) return;
  if (idx == IDX_X && settingsField != FIELD_CAL) {
    doIncrement(+1, true);
  } else if (idx == IDX_Y && settingsField != FIELD_CAL) {
    doIncrement(-1, true);
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
    drawButtonBox(BTN_RIGHT_X, BTN_TOP_Y, settingsField == FIELD_CAL ? "CAL" : "+", btnDown[IDX_X]);
    drawButtonBox(BTN_RIGHT_X, BTN_BOT_Y, "-",    btnDown[IDX_Y]);
  } else {
    if (appState == IDLE) {
      strcpy(aLbl, dist_xa_steps > 0 ? "GO" : "!CAL");
      strcpy(bLbl, stepper->getCurrentPosition() == 0 ? "SET" : "HOME");
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
  tft.fillRect(X_OFF, DIV_TOP_Y + 1, SCREEN_W, DIV_BOT_Y - DIV_TOP_Y - 1, ST77XX_BLACK);
}


// =============================================================================
// Run screen
// =============================================================================
void updateRunContent() {
  // All data rows: label(4) + value(%7) + unit(4) = 15 chars = 180 px from x=6
  char    buf[32];
  int32_t pos_steps   = stepper->getCurrentPosition();
  float   pos_um      = stepsToUm(pos_steps);
  float   actual_hz   = fabsf(stepper->getCurrentSpeedInMilliHz() / 1000.0f);
  float   actual_um_s = actual_hz * microns_per_step * stepToUmFactor();
  float   dist_xa_um  = stepsToUm(dist_xa_steps);

  // ---- State label (centered by exact char count) ----
  const char *stateStr = "IDLE";
  uint16_t    stateCol = ST77XX_GREEN;
  switch (appState) {
    case MOVING:          stateStr = "MOVING";    stateCol = ST77XX_YELLOW; break;
    case MOVING_TO_START: stateStr = "TO START";  stateCol = ST77XX_YELLOW; break;
    case PEELING:         stateStr = "PEELING";   stateCol = ST77XX_YELLOW; break;
    case HOMING:          stateStr = "HOMING";    stateCol = ST77XX_CYAN;   break;
    case CAL_HOMING:      stateStr = "CAL HOME";  stateCol = ST77XX_CYAN;   break;
    case CAL_RUNNING:     stateStr = "CAL RUN";   stateCol = ST77XX_CYAN;   break;
    default: break;
  }
  // Pad to 20 chars so background overwrites old text without a separate erase.
  {
    char sb[22];
    int  len = strlen(stateStr);
    int  lp  = (20 - len) / 2;
    int  i   = 0;
    while (i < lp)        sb[i++] = ' ';
    for (int j = 0; j < len; j++) sb[i++] = stateStr[j];
    while (i < 20)        sb[i++] = ' ';
    sb[i] = '\0';
    tft.setTextSize(2);
    tft.setTextColor(stateCol, ST77XX_BLACK);
    tft.setCursor(X_OFF, STATE_Y);
    tft.print(sb);
  }

  tft.setTextSize(2);

  // ---- Position (or warning) ----
  tft.setCursor(6 + X_OFF, POS_Y);
  if (millis() < warningUntil) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("!CAL FIRST!        ");   // 19 chars
  } else {
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.print("POS:");
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    snprintf(buf, sizeof(buf), "%7.1f", pos_um);
    tft.print(buf);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.print("um      ");              // "um" + 6 spaces = 8 chars → total 4+7+8=19 ✓
  }

  // ---- Set speed — "SET:%7.1fum/s" = 4+7+4 = 15 chars ----
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(6 + X_OFF, SETSPD_Y);
  snprintf(buf, sizeof(buf), "SET:%7.1fum/s", speed_um_s);
  tft.print(buf);

  // ---- Run speed — "RUN:%7.1fum/s" = 4+7+4 = 15 chars ----
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(6 + X_OFF, RUNSPD_Y);
  snprintf(buf, sizeof(buf), "RUN:%7.1fum/s", actual_um_s);
  tft.print(buf);

  // ---- Angle — "ANG:%7d deg" = 4+7+4 = 15 chars ----
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(6 + X_OFF, ANGLE_Y);
  snprintf(buf, sizeof(buf), "ANG:%7d deg", angle_deg);
  tft.print(buf);

  // ---- Time to end ----
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(6 + X_OFF, PEELT_Y);
  if (appState == PEELING && speed_um_s > 0.0f && pos_um < dist_xa_um) {
    float t = (dist_xa_um - pos_um) / speed_um_s;
    snprintf(buf, sizeof(buf), "END:%7.1f s  ", t);
  } else {
    snprintf(buf, sizeof(buf), "END:     -- s  ");
  }
  tft.print(buf);

  // ---- Peel elapsed time ----
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(6 + X_OFF, TOEND_Y);
  {
    char ts[14];
    if (appState == PEELING) {
      unsigned long total_s = (millis() - peel_start_ms) / 1000UL;
      int sec   = (int)(total_s % 60);
      int tot_m = (int)(total_s / 60);
      int mn    = tot_m % 60;
      int hr    = (tot_m / 60) % 24;
      int days  = tot_m / 1440;
      if (days >= 10) {
        snprintf(ts, sizeof(ts), "%dd %02d:%02d",      days, hr, mn);
      } else if (days >= 1) {
        snprintf(ts, sizeof(ts), "%dd %02d:%02d:%02d", days, hr, mn, sec);
      } else if (hr >= 1) {
        snprintf(ts, sizeof(ts), "%02d:%02d:%02d",     hr, mn, sec);
      } else {
        snprintf(ts, sizeof(ts), "%02d:%02d",          mn, sec);
      }
    } else {
      snprintf(ts, sizeof(ts), "--");
    }
    // Center ts within the 11-char value field
    int   tslen = strlen(ts);
    int   lpad  = (11 - tslen) / 2;
    char  cb[16];
    int   ci = 0;
    for (int j = 0; j < lpad; j++)   cb[ci++] = ' ';
    for (int j = 0; j < tslen; j++)  cb[ci++] = ts[j];
    while (ci < 11)                   cb[ci++] = ' ';
    cb[ci] = '\0';
    snprintf(buf, sizeof(buf), "PLT:%s", cb);
  }
  tft.print(buf);

  // ---- Progress bar ----
  tft.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, ST77XX_CYAN);
  int32_t filled = 0;
  if (dist_xa_steps > 0 && pos_steps > 0) {
    filled = (int32_t)(BAR_W - 2) * pos_steps / dist_xa_steps;
    if (filled > BAR_W - 2) filled = BAR_W - 2;
    if (filled < 0)          filled = 0;
  }
  tft.fillRect(BAR_X + 1,          BAR_Y + 1, filled,              BAR_H - 2, ST77XX_CYAN);
  tft.fillRect(BAR_X + 1 + filled, BAR_Y + 1, BAR_W - 2 - filled, BAR_H - 2, ST77XX_BLACK);

  // ---- Temperature (textSize 1 fits in 8 px gap between bar and divider) ----
  tft.setTextSize(1);
  tft.setCursor(6 + X_OFF, TEMP_Y);
  if (isnan(lastTempC)) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("T: -- FAULT --      ");
  } else {
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    snprintf(buf, sizeof(buf), "T: %6.1f C         ", lastTempC);
    tft.print(buf);
  }
}


// =============================================================================
// Settings screen
// =============================================================================
void drawSettingsField(int idx, bool active) {
  // 4 fields spaced 24 px apart; fieldY[3]+20=158 leaves room for cal status at y=178.
  const int fieldY[4] = { 68, 92, 116, 140 };
  tft.fillRect(X_OFF, fieldY[idx], SCREEN_W, 20, ST77XX_BLACK);
  char vbuf[24];
  if (active) {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(6 + X_OFF, fieldY[idx]);
    tft.print(">");
    tft.setCursor(22 + X_OFF, fieldY[idx]);
    switch (idx) {
      case FIELD_ANGLE: snprintf(vbuf, sizeof(vbuf), "ANG: %2d deg   ", angle_deg);    break;
      case FIELD_SPEED: snprintf(vbuf, sizeof(vbuf), "SPD:%.1fum/s  ", speed_um_s);   break;
      case FIELD_START: snprintf(vbuf, sizeof(vbuf), "ST: %.0fum    ", start_pos_um);  break;
      case FIELD_CAL:   snprintf(vbuf, sizeof(vbuf), "CAL: press CAL");                break;
    }
    tft.print(vbuf);
  } else {
    tft.setTextSize(1);
    tft.setTextColor(0x8410 /* mid-gray */, ST77XX_BLACK);
    tft.setCursor(16 + X_OFF, fieldY[idx]);
    switch (idx) {
      case FIELD_ANGLE: snprintf(vbuf, sizeof(vbuf), "ANG: %d deg", angle_deg);        break;
      case FIELD_SPEED: snprintf(vbuf, sizeof(vbuf), "SPD: %.1f um/s", speed_um_s);    break;
      case FIELD_START: snprintf(vbuf, sizeof(vbuf), "START: %.0f um", start_pos_um);  break;
      case FIELD_CAL:   snprintf(vbuf, sizeof(vbuf), "CAL (press CAL)");               break;
    }
    tft.print(vbuf);
  }
}

void drawSettingsHint(int) {
  tft.fillRect(X_OFF, 54, SCREEN_W, 12, ST77XX_BLACK);  // clear hint area
}

void updateSettingsContent() {
  if (!settingsDirty) return;
  settingsDirty = false;

  int  curIdx    = (int)settingsField;
  bool firstDraw = (prevSettingsFieldIdx < 0);

  if (firstDraw) {
    // Full initial draw: title, all fields, cal status
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(X_OFF + (SCREEN_W - 8 * 12) / 2, STATE_Y);
    tft.print("SETTINGS");
    drawSettingsHint(curIdx);
    for (int i = 0; i < 4; i++) drawSettingsField(i, i == curIdx);
    char  vbuf[24];
    float dist_xa_um = stepsToUm(dist_xa_steps);
    tft.setTextSize(1);
    tft.setCursor(6 + X_OFF, 178);
    if (dist_xa_steps > 0) {
      tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      snprintf(vbuf, sizeof(vbuf), "X-A: %.1f um    ", dist_xa_um);
    } else {
      tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
      snprintf(vbuf, sizeof(vbuf), "NOT CALIBRATED      ");
    }
    tft.print(vbuf);
  } else if (prevSettingsFieldIdx != curIdx) {
    // Active field changed: redraw old (now inactive) and new (now active) rows only
    drawSettingsField(prevSettingsFieldIdx, false);
    drawSettingsField(curIdx, true);
    drawSettingsHint(curIdx);
  } else {
    // Same field, value changed: redraw active row only
    drawSettingsField(curIdx, true);
  }

  prevSettingsFieldIdx = curIdx;
}


// =============================================================================
// WiFi signal icon  (between A and X buttons, above top divider)
//   cx=120, cy=26: dot at bottom, arcs open upward
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
  const int16_t cx = 120 + X_OFF, cy = 26;
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
  tft.drawFastHLine(X_OFF, DIV_TOP_Y, SCREEN_W, ST77XX_CYAN);
  tft.drawFastHLine(X_OFF, DIV_BOT_Y, SCREEN_W, ST77XX_CYAN);
  updateButtons();
  drawWifiIcon(0);
  updateRunContent();
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

  // Heater MOSFET — LEDC 10 Hz, 8-bit, starts off
  bool heaterOk = ledcAttach(HEATER_PIN, 10, 8);
  ledcWrite(HEATER_PIN, 0);
  Serial.printf("[heater] ledcAttach GPIO%d: %s\n", HEATER_PIN, heaterOk ? "OK" : "FAILED");

  // Buttons
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_X, INPUT_PULLUP);
  pinMode(BTN_Y, INPUT_PULLUP);
  pinMode(LIMIT_SW_X, INPUT_PULLUP);
  pinMode(LIMIT_SW_Y, INPUT);  // input-only pin — pull-up is external 10kΩ to 3.3V

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
    case MOVING:          stateStr = "MOVING";    break;
    case MOVING_TO_START: stateStr = "TO_START";  break;
    case PEELING:         stateStr = "PEELING";   break;
    case HOMING:          stateStr = "HOMING";    break;
    case SETTINGS:        stateStr = "SETTINGS";  break;
    case CAL_HOMING:      stateStr = "CAL_HOME";  break;
    case CAL_RUNNING:     stateStr = "CAL_RUN";   break;
    default:              stateStr = "IDLE";      break;
  }

  char tempBuf[12];
  if (isnan(lastTempC)) snprintf(tempBuf, sizeof(tempBuf), "null");
  else                  snprintf(tempBuf, sizeof(tempBuf), "%.1f", lastTempC);

  char json[430];
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
    "\"btn\":[%s,%s,%s,%s],"
    "\"rssi\":%d,"
    "\"clients\":%d,"
    "\"temp_c\":%s,"
    "\"heater_duty\":%d,"
    "\"ip\":\"%s\"}",
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
    btnDown[IDX_A] ? "true" : "false",
    btnDown[IDX_B] ? "true" : "false",
    btnDown[IDX_X] ? "true" : "false",
    btnDown[IDX_Y] ? "true" : "false",
    rssi,
    (int)ws.count(),
    tempBuf,
    (int)heaterDuty,
    wifiIpStr
  );

  ws.textAll(json);
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
    if (curLimX) {                              // home-end switch triggered
      stepper->forceStop();
      while (stepper->isRunning()) {}           // wait for PIO to flush buffered steps
      stepper->setCurrentPosition(0);
      if (appState == CAL_HOMING) {
        stepper->setSpeedInHz(calSpeedHz());
        stepper->runForward();
        appState = CAL_RUNNING;
      } else {
        disableMotor();
        appState = IDLE;
      }
      updateButtons();
    }
  } else if (appState == CAL_RUNNING) {
    if (curLimY) {                              // far-end switch triggered
      dist_xa_steps = stepper->getCurrentPosition();
      stepper->forceStop();
      disableMotor();
      saveCalibration();
      appState = IDLE;
      updateButtons();
    }
  } else if (appState == MOVING || appState == MOVING_TO_START || appState == PEELING) {
    if (xNewPress || yNewPress) {               // new contact only — not a stale press
      abortAndIdle();
      updateButtons();
    }
  }

  // ---- Serial command parser ---------------------------------------------------
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case 's':
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
        duty = constrain(duty, 0, 255);
        heaterDuty = (uint8_t)duty;
        ledcWrite(HEATER_PIN, heaterDuty);
        Serial.printf("{\"heater_duty\":%d,\"heater_freq_hz\":%u}\n",
                      (int)heaterDuty, (unsigned)ledcReadFreq(HEATER_PIN));
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
        appState = IDLE;
        updateButtons();
      }
      break;

    case MOVING_TO_START:
      // Arm peel timer on arrival
      if (startPeelAt == 0 && !stepper->isRunning()) {
        startPeelAt = now + 100;
      }
      // Fire peel after 100 ms pause
      if (startPeelAt > 0 && now >= startPeelAt) {
        startPeelAt = 0;
        startPeeling();
        updateButtons();
      }
      break;

    case PEELING:
      // Motor reached dist_xa_steps
      if (!stepper->isRunning()) {
        disableMotor();
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
    }

    if (inSettingsScreen) {
      updateSettingsContent();
    } else {
      updateRunContent();
    }

    // Redraw dividers (may be overwritten by fillRect in clearContent)
    tft.drawFastHLine(X_OFF, DIV_TOP_Y, SCREEN_W, ST77XX_CYAN);
    tft.drawFastHLine(X_OFF, DIV_BOT_Y, SCREEN_W, ST77XX_CYAN);

    // IP line in settings screen (redrawn every tick so it updates when WiFi connects)
    if (inSettingsScreen) {
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
      tft.setCursor(6 + X_OFF, 190);
      char ipBuf[30];
      snprintf(ipBuf, sizeof(ipBuf), "%-28s", wifiIpStr);
      tft.print(ipBuf);
    }

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
        // Clear only the space between the two button boxes
        const int areaX = BTN_LEFT_X + BTN_W + 1;           // 56
        const int areaW = BTN_RIGHT_X - areaX - 1;           // 128 px
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
        // client count to the right of WiFi icon (icon centre cx=120, ends ~x=133)
        tft.fillRect(134 + X_OFF, 18, 22, 10, ST77XX_BLACK);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.setCursor(136 + X_OFF, 20);
        tft.print(clientCount);
        prevRssiBars   = currBars;
        prevClientCount = clientCount;
      }

      const char *stateStr;
      switch (appState) {
        case MOVING:          stateStr = "MOVING";    break;
        case MOVING_TO_START: stateStr = "TO_START";  break;
        case PEELING:         stateStr = "PEELING";   break;
        case HOMING:          stateStr = "HOMING";    break;
        case SETTINGS:        stateStr = "SETTINGS";  break;
        case CAL_HOMING:      stateStr = "CAL_HOME";  break;
        case CAL_RUNNING:     stateStr = "CAL_RUN";   break;
        default:              stateStr = "IDLE";      break;
      }

      char tempBuf[12];
      if (isnan(lastTempC)) snprintf(tempBuf, sizeof(tempBuf), "null");
      else                  snprintf(tempBuf, sizeof(tempBuf), "%.1f", lastTempC);

      char json[430];
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
        "\"btn\":[%s,%s,%s,%s],"
        "\"rssi\":%d,"
        "\"clients\":%d,"
        "\"temp_c\":%s,"
        "\"heater_duty\":%d,"
        "\"ip\":\"%s\"}",
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
        btnDown[IDX_A] ? "true" : "false",
        btnDown[IDX_B] ? "true" : "false",
        btnDown[IDX_X] ? "true" : "false",
        btnDown[IDX_Y] ? "true" : "false",
        rssi,
        (int)ws.count(),
        tempBuf,
        (int)heaterDuty,
        wifiIpStr
      );

      if (Serial) Serial.println(json);
      ws.cleanupClients();
      if (ws.count() > 0) ws.textAll(json);
    }
  }
}
