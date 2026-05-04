// =============================================================================
// Project   : Peeling Thin Sheet Using Stepper Motor — Raspberry Pi Pico Controller
// File      : Peeling_Automation_Stepper_Control_Uno.ino
// Author    : Lior Segev
// Version   : 4.0.0
// Date      : May 3, 2026
// =============================================================================
//
// OVERVIEW
// --------
// Firmware for a stepper-motor-driven thin-sheet peeling instrument.
// Controls a NEMA 17 via DM542T driver on a Raspberry Pi Pico (RP2040)
// with a Pimoroni Pico Explorer Base (240×240 ST7789 display, 4 buttons).
//
// HARDWARE
// --------
//   • Raspberry Pi Pico (RP2040) on Pimoroni Pico Explorer Base
//   • NEMA 17 stepper motor (0.4 A rated)
//   • DM542T (V4.0) stepper driver — active-low ENA-
//       – t1: ENA→first PUL ≥ 200 ms (firmware uses 500 ms delay)
//       – t2: DIR stable before PUL ≥ 5 µs (setDirectionPin 40 µs)
//   • Microswitches: X = home end, A = far end (also Pico Explorer buttons)
//
// PIN ASSIGNMENT
// ---------------
//   GPIO  3  — ENA- (active-low)
//   GPIO  4  — DIR-
//   GPIO  5  — PUL-
//   GPIO 12  — BTN_A  (microswitch far end / Pico Explorer A)
//   GPIO 13  — BTN_B  (UI: settings / home / cycle / decrement)
//   GPIO 14  — BTN_X  (microswitch home end / Pico Explorer X)
//   GPIO 15  — BTN_Y  (UI: start / stop / increment)
//   GPIO 16  — TFT_DC
//   GPIO 17  — TFT_CS
//   GPIO 18  — SPI SCK
//   GPIO 19  — SPI MOSI
//   GPIO 20  — TFT_BL
//
// UNIT CONVERSION
// ---------------
//   1 step = 0.9375 µm × cos(angle_deg)
//   distance_µm  = steps    × 0.9375 × cos(angle_rad)
//   speed_µm_s   = steps/s  × 0.9375 × cos(angle_rad)
//
// STATE MACHINE
// -------------
//   IDLE → [Y, dist_xa>0] → MOVING_TO_START → [arrival+100ms] → PEELING
//   IDLE → [Y, dist_xa=0] → show warning "RUN CAL FIRST"
//   IDLE → [B, pos=0]     → SETTINGS  (cycles: angle→speed→start→CAL→IDLE+save)
//   IDLE → [B, pos>0]     → HOMING
//   PEELING → [Y or A limit] → IDLE
//   HOMING  → [X limit]      → IDLE (pos := 0)
//   SETTINGS/CAL field → [Y] → CAL_HOMING → CAL_RUNNING → IDLE (saves dist_xa)
//   Any moving state → [Y]   → IDLE (abort)
//
// SERIAL INTERFACE  (115200 baud)
// --------------------------------
//   Commands: 'm'<int32> move to step pos, 's' stop, 'v'<int> set Hz
//   Heartbeat every 100 ms:
//     {"state":N,"position":N,"speed":N,"pos_um":F,"speed_um":F,"angle":N}
// =============================================================================

#include <FastAccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <EEPROM.h>
#include <math.h>

// ---- Stepper driver pins ----------------------------------------------------
#define enablePinStepper  3
#define dirPinStepper     4
#define stepPinStepper    5

// ---- Display pins -----------------------------------------------------------
#define TFT_CS   17
#define TFT_DC   16
#define TFT_RST  -1
#define TFT_BL   20

// ---- Button pins (active-low, INPUT_PULLUP) ---------------------------------
#define BTN_A  12   // UI: start / stop / increment
#define BTN_B  13   // UI: settings / home / cycle / decrement
#define BTN_X  14   // microswitch: home end  (limit switch)
#define BTN_Y  15   // microswitch: far end   (limit switch)

// ---- Display geometry -------------------------------------------------------
#define SCREEN_W   240
#define SCREEN_H   240

#define BTN_W       52
#define BTN_H       28
#define BTN_LEFT_X   3
#define BTN_RIGHT_X (SCREEN_W - BTN_W - 3)
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
#define BAR_X      20
#define BAR_Y     178   // 12 px bar → ends 190; bottom divider at 207
#define BAR_W     200
#define BAR_H      12

// ---- Physical constants -----------------------------------------------------
#define MICRONS_PER_STEP  0.9375f
#define CAL_SPEED_HZ      1067    // ≈ 1 mm/s at 0° (both CAL phases)

// ---- FastAccelStepper -------------------------------------------------------
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper       *stepper = NULL;

// ---- Display ----------------------------------------------------------------
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);

// ---- Persistent storage -----------------------------------------------------
#define EEPROM_MAGIC  0x50454C31u   // "PEL1"
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

enum SettingsField { FIELD_ANGLE, FIELD_SPEED, FIELD_START, FIELD_CAL };
SettingsField settingsField = FIELD_ANGLE;

// ---- User-configurable values -----------------------------------------------
int     angle_deg     = 30;
float   speed_um_s    = 1.0f;
float   start_pos_um  = 0.0f;
int32_t dist_xa_steps = 0;      // calibrated X→A distance in steps (0 = uncalibrated)

// ---- Motor state ------------------------------------------------------------
bool motorEnabled = false;

// ---- Peel sequencing --------------------------------------------------------
unsigned long startPeelAt = 0;  // millis() target for MOVING_TO_START→PEELING (0=not armed)
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

// ---- Warning overlay --------------------------------------------------------
unsigned long warningUntil = 0;

// ---- Periodic update --------------------------------------------------------
unsigned long previousMillis = 0;
const unsigned long HEARTBEAT_MS = 100;

// ---- Screen mode tracking (for clean transitions) ---------------------------
bool inSettingsScreen    = false;
bool settingsDirty       = true;
bool justEnteredSettings = false;
int  prevSettingsFieldIdx = -1;  // -1 = first draw needed


// =============================================================================
// Unit conversion
// =============================================================================
float cosAngle() {
  return cosf((float)angle_deg * (float)M_PI / 180.0f);
}

float stepsToUm(int32_t steps) {
  return (float)steps * MICRONS_PER_STEP * cosAngle();
}

int32_t umToSteps(float um) {
  float d = MICRONS_PER_STEP * cosAngle();
  if (d < 1e-4f) d = 1e-4f;
  return (int32_t)(um / d);
}

int32_t speedUmToHz(float um_s) {
  float d = MICRONS_PER_STEP * cosAngle();
  if (d < 1e-4f) d = 1e-4f;
  int32_t hz = (int32_t)(um_s / d);
  return hz < 1 ? 1 : hz;
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
  // else keep firmware defaults
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
  stepper->setSpeedInHz(CAL_SPEED_HZ);
  stepper->moveTo(umToSteps(start_pos_um));
  startPeelAt = 0;
}

void startPeeling() {
  // Motor stays enabled from startMoveToStart — no re-enable needed.
  stepper->setSpeedInHz(speedUmToHz(speed_um_s));
  stepper->moveTo(dist_xa_steps);
  peel_start_ms = millis();
  appState = PEELING;
}

void startHoming() {
  appState = HOMING;
  enableMotor();
  stepper->setSpeedInHz(CAL_SPEED_HZ);
  stepper->runBackward();
}

void startCal() {
  appState = CAL_HOMING;
  enableMotor();
  stepper->setSpeedInHz(CAL_SPEED_HZ);
  stepper->runBackward();
}

void cycleSettingsField() {
  switch (settingsField) {
    case FIELD_ANGLE: settingsField = FIELD_SPEED; break;
    case FIELD_SPEED: settingsField = FIELD_START; break;
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
          appState             = SETTINGS;
          settingsField        = FIELD_ANGLE;
          settingsDirty        = true;
          justEnteredSettings  = true;
        } else {
          startHoming();
        }
      }
      break;

    case SETTINGS:
      if (idx == IDX_A) {          // A = UI increment / confirm CAL
        if (settingsField == FIELD_CAL) {
          startCal();
        } else {
          doIncrement(+1, false);
        }
      }
      // B: handled in onButtonRelease (short) and onButtonLong (long)
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
  if (appState == SETTINGS && idx == IDX_B && settingsField != FIELD_CAL) {
    doIncrement(-1, false);
  }
}

void onButtonRepeat(int idx) {
  if (appState != SETTINGS) return;
  if (idx == IDX_A && settingsField != FIELD_CAL) {
    doIncrement(+1, true);
  } else if (idx == IDX_B && settingsField != FIELD_CAL) {
    doIncrement(-1, true);
  }
}


// =============================================================================
// Display helpers
// =============================================================================
void drawButtonBox(int16_t x, int16_t y, const char *label, bool pressed) {
  uint16_t bg = pressed ? ST77XX_WHITE : ST77XX_BLACK;
  uint16_t fg = pressed ? ST77XX_BLACK : ST77XX_CYAN;
  tft.fillRoundRect(x, y, BTN_W, BTN_H, 4, bg);
  tft.drawRoundRect(x, y, BTN_W, BTN_H, 4, ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setTextColor(fg, bg);
  int len = strlen(label);
  tft.setCursor(x + (BTN_W - len * 12) / 2, y + (BTN_H - 16) / 2);
  tft.print(label);
}

void updateButtons() {
  char aLbl[8], bLbl[8];

  switch (appState) {
    case IDLE:
      strcpy(aLbl, dist_xa_steps > 0 ? "GO" : "!CAL");
      strcpy(bLbl, stepper->getCurrentPosition() == 0 ? "SET" : "HOME");
      break;
    case SETTINGS:
      strcpy(aLbl, settingsField == FIELD_CAL ? "CAL" : "+");
      strcpy(bLbl, "NEXT");
      break;
    default:
      strcpy(aLbl, "STOP");
      strcpy(bLbl, "----");
      break;
  }

  // Physical layout (setRotation 2): A=top-left, X=top-right, B=bottom-left, Y=bottom-right
  drawButtonBox(BTN_LEFT_X,  BTN_TOP_Y, aLbl,  btnDown[IDX_A]);  // top-left  = A (UI)
  drawButtonBox(BTN_RIGHT_X, BTN_TOP_Y, "X",   btnDown[IDX_X]);  // top-right = X (limit)
  drawButtonBox(BTN_LEFT_X,  BTN_BOT_Y, bLbl,  btnDown[IDX_B]);  // bot-left  = B (UI)
  drawButtonBox(BTN_RIGHT_X, BTN_BOT_Y, "Y",   btnDown[IDX_Y]);  // bot-right = Y (limit)
}

void clearContent() {
  tft.fillRect(0, DIV_TOP_Y + 1, SCREEN_W, DIV_BOT_Y - DIV_TOP_Y - 1, ST77XX_BLACK);
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
  float   actual_um_s = actual_hz * MICRONS_PER_STEP * cosAngle();
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
  // Pad to 20 chars (full screen width at textSize 2) so background overwrites old text
  // without a separate fillRect erase step — prevents flicker.
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
    tft.setCursor(0, STATE_Y);
    tft.print(sb);
  }

  tft.setTextSize(2);

  // ---- Position (or warning) ----
  // 19 chars from x=6 → covers x=6..234, clearing any stale content to the right
  tft.setCursor(6, POS_Y);
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
  tft.setCursor(6, SETSPD_Y);
  snprintf(buf, sizeof(buf), "SET:%7.1fum/s", speed_um_s);
  tft.print(buf);

  // ---- Run speed — "RUN:%7.1fum/s" = 4+7+4 = 15 chars ----
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(6, RUNSPD_Y);
  snprintf(buf, sizeof(buf), "RUN:%7.1fum/s", actual_um_s);
  tft.print(buf);

  // ---- Angle — "ANG:%7d deg" = 4+7+4 = 15 chars ----
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(6, ANGLE_Y);
  snprintf(buf, sizeof(buf), "ANG:%7d deg", angle_deg);
  tft.print(buf);

  // ---- Time to end — "END:%7.1f s  " or "END:     -- s  " = 15 chars ----
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(6, PEELT_Y);
  if (appState == PEELING && speed_um_s > 0.0f && pos_um < dist_xa_um) {
    float t = (dist_xa_um - pos_um) / speed_um_s;
    snprintf(buf, sizeof(buf), "END:%7.1f s  ", t);
  } else {
    snprintf(buf, sizeof(buf), "END:     -- s  ");
  }
  tft.print(buf);

  // ---- Peel elapsed time — value centered in 11-char field after "PLT:" ----
  // Formats: MM:SS | HH:MM:SS | Xd HH:MM:SS (≤9d) | XXd HH:MM (≥10d)
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(6, TOEND_Y);
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
}


// =============================================================================
// Settings screen
// =============================================================================
void drawSettingsField(int idx, bool active) {
  const int fieldY[4] = { 72, 104, 128, 152 };
  tft.fillRect(0, fieldY[idx], SCREEN_W, 20, ST77XX_BLACK);
  char vbuf[24];
  if (active) {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(6, fieldY[idx]);
    tft.print(">");
    tft.setCursor(22, fieldY[idx]);
    switch (idx) {
      case FIELD_ANGLE: snprintf(vbuf, sizeof(vbuf), "ANG: %2d deg   ", angle_deg);   break;
      case FIELD_SPEED: snprintf(vbuf, sizeof(vbuf), "SPD:%.1fum/s  ", speed_um_s);  break;
      case FIELD_START: snprintf(vbuf, sizeof(vbuf), "ST: %.0fum    ", start_pos_um); break;
      case FIELD_CAL:   snprintf(vbuf, sizeof(vbuf), "CAL: press CAL");               break;
    }
    tft.print(vbuf);
  } else {
    tft.setTextSize(1);
    tft.setTextColor(0x8410 /* mid-gray */, ST77XX_BLACK);
    tft.setCursor(16, fieldY[idx]);
    switch (idx) {
      case FIELD_ANGLE: snprintf(vbuf, sizeof(vbuf), "ANG: %d deg", angle_deg);       break;
      case FIELD_SPEED: snprintf(vbuf, sizeof(vbuf), "SPD: %.1f um/s", speed_um_s);   break;
      case FIELD_START: snprintf(vbuf, sizeof(vbuf), "START: %.0f um", start_pos_um); break;
      case FIELD_CAL:   snprintf(vbuf, sizeof(vbuf), "CAL (press CAL)");              break;
    }
    tft.print(vbuf);
  }
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
    tft.setCursor((SCREEN_W - 8 * 12) / 2, STATE_Y);
    tft.print("SETTINGS");
    for (int i = 0; i < 4; i++) drawSettingsField(i, i == curIdx);
    char  vbuf[24];
    float dist_xa_um = stepsToUm(dist_xa_steps);
    tft.setTextSize(1);
    tft.setCursor(6, 178);
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
  } else {
    // Same field, value changed: redraw active row only
    drawSettingsField(curIdx, true);
  }

  prevSettingsFieldIdx = curIdx;
}


// =============================================================================
// Full UI init (called once in setup)
// =============================================================================
void initUI() {
  tft.fillScreen(ST77XX_BLACK);
  tft.drawFastHLine(0, DIV_TOP_Y, SCREEN_W, ST77XX_CYAN);
  tft.drawFastHLine(0, DIV_BOT_Y, SCREEN_W, ST77XX_CYAN);
  updateButtons();
  updateRunContent();
}


// =============================================================================
// setup
// =============================================================================
void setup() {
  Serial.begin(115200);

  // Display
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.setRX(0);
  SPI.setTX(19);
  SPI.setSCK(18);
  SPI.begin();
  tft.init(240, 240);
  tft.setRotation(2);

  // Buttons
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_X, INPUT_PULLUP);
  pinMode(BTN_Y, INPUT_PULLUP);

  // Load saved settings
  loadPrefs();

  // Draw UI
  initUI();

  // Stepper
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper, true, 40);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(false);
    stepper->disableOutputs();
    stepper->setCurrentPosition(0);
    stepper->setSpeedInHz(speedUmToHz(speed_um_s));
    stepper->setAcceleration(2147483647);
  }

  unsigned long t = millis();
  while (!Serial && millis() - t < 2000) delay(10);
}


// =============================================================================
// loop
// =============================================================================
void loop() {
  unsigned long now = millis();

  // ---- Button processing -------------------------------------------------------
  bool curDown[4] = {
    !digitalRead(BTN_A),
    !digitalRead(BTN_B),
    !digitalRead(BTN_X),
    !digitalRead(BTN_Y)
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
  if (btnChanged) updateButtons();

  // ---- Limit switch polling (X = home end, Y = far end) -----------------------
  if (appState == HOMING || appState == CAL_HOMING) {
    if (!digitalRead(BTN_X)) {                  // home-end switch triggered
      stepper->forceStop();
      while (stepper->isRunning()) {}           // wait for PIO to flush buffered steps
      stepper->setCurrentPosition(0);
      if (appState == CAL_HOMING) {
        stepper->setSpeedInHz(CAL_SPEED_HZ);    // 5 mm/s toward far end
        stepper->runForward();
        appState = CAL_RUNNING;
      } else {
        disableMotor();
        appState = IDLE;
      }
      updateButtons();
    }
  } else if (appState == CAL_RUNNING) {
    if (!digitalRead(BTN_Y)) {                  // far-end switch triggered
      dist_xa_steps = stepper->getCurrentPosition();
      stepper->forceStop();
      disableMotor();
      saveCalibration();
      appState = IDLE;
      updateButtons();
    }
  } else if (appState == MOVING || appState == MOVING_TO_START || appState == PEELING) {
    if (!digitalRead(BTN_X) || !digitalRead(BTN_Y)) {  // either limit hit = safety stop
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
    tft.drawFastHLine(0, DIV_TOP_Y, SCREEN_W, ST77XX_CYAN);
    tft.drawFastHLine(0, DIV_BOT_Y, SCREEN_W, ST77XX_CYAN);

    // Serial JSON heartbeat
    float pos_um      = stepsToUm(stepper->getCurrentPosition());
    float actual_hz   = stepper->getCurrentSpeedInMilliHz() / 1000.0f;
    float actual_um_s = actual_hz * MICRONS_PER_STEP * cosAngle();
    int   stateCode   = (appState == IDLE || appState == SETTINGS) ? 3 : 2;

    Serial.print("{\"state\":");   Serial.print(stateCode);
    Serial.print(",\"position\":"); Serial.print(stepper->getCurrentPosition());
    Serial.print(",\"speed\":");   Serial.print((int32_t)actual_hz);
    Serial.print(",\"pos_um\":");  Serial.print(pos_um, 2);
    Serial.print(",\"speed_um\":"); Serial.print(actual_um_s, 2);
    Serial.print(",\"angle\":");   Serial.print(angle_deg);
    Serial.println("}");
  }
}
