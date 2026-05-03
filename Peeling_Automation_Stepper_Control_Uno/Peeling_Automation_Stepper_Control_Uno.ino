// =============================================================================
// Project   : Peeling Thin Sheet Using Stepper Motor — Raspberry Pi Pico Controller
// File      : Peeling_Automation_Stepper_Control_Uno.ino
// Author    : Lior Segev
// Version   : 3.3.0
// Date      : May 3, 2026
// =============================================================================
//
// OVERVIEW
// --------
// This firmware controls a NEMA 17 stepper motor used to automate the peeling
// of thin sheets. It manages motor position via a serial command interface and
// emits a JSON status object every 100 ms for external software monitoring.
// Runs on a Pimoroni Pico Explorer Base (1.54" 240×240 IPS ST7789 display,
// four buttons A/B/X/Y).
//
// HARDWARE
// --------
//   • Raspberry Pi Pico (RP2040) on Pimoroni Pico Explorer Base
//   • NEMA 17 stepper motor (0.4 A rated)
//   • DM542T (V4.0) stepper driver — Leadshine digital driver
//       – ENA active-low; t1: ENA→first PUL ≥ 200 ms (datasheet Fig.15)
//       – t2: DIR stable before PUL ≥ 5 µs; t3/t4: PUL high/low width ≥ 2.5 µs
//       – Signal levels: HIGH > 3.5 V, LOW < 0.5 V — level shifter may be needed
//   • Power supply: 24–48 V recommended (min 12 V)
//
// PIN ASSIGNMENT  (Pico GPIO numbers)
// ------------------------------------
//   GPIO  3  — enablePinStepper : DM542T ENA- (active-low; driven manually)
//   GPIO  4  — dirPinStepper    : DM542T DIR-
//   GPIO  5  — stepPinStepper   : DM542T PUL-
//   GPIO 12  — BTN_A            : Pico Explorer button A (INPUT_PULLUP, active-low)
//   GPIO 13  — BTN_B            : Pico Explorer button B
//   GPIO 14  — BTN_X            : Pico Explorer button X
//   GPIO 15  — BTN_Y            : Pico Explorer button Y
//   GPIO 16  — TFT_DC           : ST7789 data/command
//   GPIO 17  — TFT_CS           : ST7789 SPI chip select
//   GPIO 18  — SPI SCK          : ST7789 clock (SPI0)
//   GPIO 19  — SPI MOSI         : ST7789 data  (SPI0)
//   GPIO 20  — TFT_BL           : ST7789 backlight
//
// DISPLAY LAYOUT (240 × 240 square, setRotation(2))
// --------------------------------------------------
//   ┌─[Y]──────────────────────────────────[X]─┐  y =  3–31   button row
//   ├──────────────────────────────────────────┤  y = 33      cyan divider
//   │                                          │
//   │              WAITING / MOVING            │  y =  50–74  state (textSize 3)
//   │                                          │
//   │              POSITION                    │  y =  88–96  label (textSize 1)
//   │               478085                     │  y =  98–122 value (textSize 3)
//   │                                          │
//   │               SPEED                      │  y = 132–140 label (textSize 1)
//   │             100    Hz                    │  y = 142–158 value (textSize 2)
//   │                                          │
//   │   [██████████████░░░░░░░░░░░░░░░░░░░░]   │  y = 168–180 progress bar
//   │                                          │
//   ├──────────────────────────────────────────┤  y = 207     cyan divider
//   └─[B]──────────────────────────────────[A]─┘  y = 209–237 button row
//
//   Button corners are for setRotation(2); adjust labels if physical board differs.
//
// SOFTWARE DEPENDENCIES
// ---------------------
//   • FastAccelStepper             — PIO-based step generation for RP2040
//   • Adafruit ST7735 and ST7789   — display driver
//   • Adafruit GFX Library         — graphics primitives
//   (Board support: arduino-pico by Earle Philhower)
//
// STATE MACHINE
// -------------
//   MOVING  — motor executing moveTo(); → WAITING when isRunning() is false.
//   WAITING — motor idle.
//
// SERIAL COMMAND INTERFACE  (115 200 baud, no line ending required)
// -----------------------------------------------------------------
//   'm' <int32>   Move to absolute step position (clamped to [0 … POS_TOP]).
//   's'           Stop the motor immediately.
//   'v' <int>     Set motor max speed in Hz.
//
// SERIAL OUTPUT FORMAT
// --------------------
//   Every 100 ms: {"state":<int>,"position":<int32>,"speed":<int32>}
//   State codes: INIT=1, MOVING=2, WAITING=3
// =============================================================================

#include <FastAccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// --- Stepper driver signal pins -----------------------------------------------
#define enablePinStepper   3
#define dirPinStepper      4
#define stepPinStepper     5

// --- Display pins (Pico Explorer Base) ----------------------------------------
#define TFT_CS   17
#define TFT_DC   16
#define TFT_RST  -1
#define TFT_BL   20

// --- Button pins (Pico Explorer Base, active-low with INPUT_PULLUP) -----------
#define BTN_A  12
#define BTN_B  13
#define BTN_X  14
#define BTN_Y  15

// --- UI layout constants (240 × 240 square display) ---------------------------
#define SCREEN_W   240
#define SCREEN_H   240

#define BTN_W       28
#define BTN_H       28
#define BTN_LEFT_X   3
#define BTN_RIGHT_X (SCREEN_W - BTN_W - 3)   // 209
#define BTN_TOP_Y    3
#define BTN_BOT_Y   (SCREEN_H - BTN_H - 3)   // 209

#define DIV_TOP_Y   33
#define DIV_BOT_Y  207

// Content y positions
#define STATE_Y      50   // textSize 3 (24 px) → ends y = 74
#define POS_LBL_Y    88   // textSize 1  (8 px) → ends y = 96
#define POS_VAL_Y    98   // textSize 3 (24 px) → ends y = 122
#define SPD_LBL_Y   132   // textSize 1  (8 px) → ends y = 140
#define SPD_VAL_Y   142   // textSize 2 (16 px) → ends y = 158
#define BAR_X        20
#define BAR_Y       168
#define BAR_W       200
#define BAR_H        12

// Content x positions (centred on 240 px)
// textSize 3: char = 18 px wide
//   "WAITING"/"MOVING " → 7 × 18 = 126 px  →  x = (240-126)/2 = 57
#define STATE_X      57
//   pos value "%-6ld"  → 6 × 18 = 108 px  →  x = (240-108)/2 = 66
#define POS_VAL_X    66
// textSize 2: char = 12 px wide
//   spd value "%-6ld Hz" → 9 × 12 = 108 px  →  x = (240-108)/2 = 66
#define SPD_VAL_X    66
// textSize 1: char = 6 px wide
//   "POSITION" → 8 × 6 = 48 px  →  x = (240-48)/2 = 96
#define POS_LBL_X    96
//   "SPEED"    → 5 × 6 = 30 px  →  x = (240-30)/2 = 105
#define SPD_LBL_X   105

// --- FastAccelStepper objects -------------------------------------------------
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// --- Display object -----------------------------------------------------------
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);

// --- Motor travel positions (in microsteps) -----------------------------------
int32_t POS_MIDDLE = 478085 / 2;
int32_t POS_TOP    = 478085;
int32_t SPEED_MAX  = 100;

// --- State machine -----------------------------------------------------------
enum state { INIT = 1, MOVING, WAITING };
enum state current_state  = WAITING;
enum state previous_state = INIT;

unsigned long previousMillis = 0;
const unsigned long interval  = 100;

// --- Button state tracking ---------------------------------------------------
uint8_t prevBtnState = 0xFF;

// ---------------------------------------------------------------------------
// drawButtonBox
// ---------------------------------------------------------------------------
void drawButtonBox(int16_t x, int16_t y, const char *label, bool pressed)
{
  uint16_t bg = pressed ? ST77XX_WHITE : ST77XX_BLACK;
  uint16_t fg = pressed ? ST77XX_BLACK : ST77XX_CYAN;
  tft.fillRoundRect(x, y, BTN_W, BTN_H, 4, bg);
  tft.drawRoundRect(x, y, BTN_W, BTN_H, 4, ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setTextColor(fg, bg);
  // Centre single char in box (char is 12 × 16 px at textSize 2)
  tft.setCursor(x + (BTN_W - 12) / 2, y + (BTN_H - 16) / 2);
  tft.print(label);
}

// ---------------------------------------------------------------------------
// updateButtons — redraws only the four corner button boxes on state change.
// Corner mapping is for setRotation(2); adjust labels if physical board differs.
// ---------------------------------------------------------------------------
void updateButtons(uint8_t btnState)
{
  drawButtonBox(BTN_LEFT_X,  BTN_TOP_Y, "Y", !(btnState & 0x08));  // top-left
  drawButtonBox(BTN_RIGHT_X, BTN_TOP_Y, "X", !(btnState & 0x04));  // top-right
  drawButtonBox(BTN_LEFT_X,  BTN_BOT_Y, "B", !(btnState & 0x02));  // bottom-left
  drawButtonBox(BTN_RIGHT_X, BTN_BOT_Y, "A", !(btnState & 0x01));  // bottom-right
}

// ---------------------------------------------------------------------------
// updateContent — rewrites dynamic fields in-place (no full-screen clear).
// Text rendered with background colour so old digits are cleanly overwritten.
// ---------------------------------------------------------------------------
void updateContent()
{
  char buf[16];
  int32_t pos = stepper->getCurrentPosition();

  // ---- State ----
  uint16_t stateColor = (current_state == MOVING) ? ST77XX_YELLOW : ST77XX_GREEN;
  tft.setTextSize(3);
  tft.setTextColor(stateColor, ST77XX_BLACK);
  tft.setCursor(STATE_X, STATE_Y);
  tft.print(current_state == MOVING ? "MOVING " : "WAITING");  // padded to 7 chars

  // ---- Position ----
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(POS_LBL_X, POS_LBL_Y);
  tft.print("POSITION");

  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  snprintf(buf, sizeof(buf), "%-6ld", (long)pos);  // fixed 6-char width clears old digits
  tft.setCursor(POS_VAL_X, POS_VAL_Y);
  tft.print(buf);

  // ---- Speed ----
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(SPD_LBL_X, SPD_LBL_Y);
  tft.print("SPEED");

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  snprintf(buf, sizeof(buf), "%-6ld Hz", (long)SPEED_MAX);  // fixed 9-char width
  tft.setCursor(SPD_VAL_X, SPD_VAL_Y);
  tft.print(buf);

  // ---- Position progress bar ----
  tft.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, ST77XX_CYAN);
  int32_t filled = (pos <= 0)       ? 0
                 : (pos >= POS_TOP) ? BAR_W - 2
                 : (int32_t)(BAR_W - 2) * pos / POS_TOP;
  tft.fillRect(BAR_X + 1,          BAR_Y + 1,  filled,               BAR_H - 2, ST77XX_CYAN);
  tft.fillRect(BAR_X + 1 + filled, BAR_Y + 1,  BAR_W - 2 - filled,  BAR_H - 2, ST77XX_BLACK);
}

// ---------------------------------------------------------------------------
// initUI — one-time full-screen draw.
// ---------------------------------------------------------------------------
void initUI()
{
  tft.fillScreen(ST77XX_BLACK);
  tft.drawFastHLine(0, DIV_TOP_Y, SCREEN_W, ST77XX_CYAN);
  tft.drawFastHLine(0, DIV_BOT_Y, SCREEN_W, ST77XX_CYAN);
  updateButtons(0xFF);
  updateContent();
}

void setup()
{
  Serial.begin(115200);

  // --- Display init ---
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  SPI.setRX(0);    // MISO — not connected to display; keeps SPI0 config valid
  SPI.setTX(19);
  SPI.setSCK(18);
  SPI.begin();

  tft.init(240, 240);          // 1.54" 240×240 IPS panel
  tft.setRotation(2);          // 90° CW from landscape (rotation 1)
  initUI();

  // --- Button init ---
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_X, INPUT_PULLUP);
  pinMode(BTN_Y, INPUT_PULLUP);

  // --- Stepper init ---
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper, true, 40);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(false);
    stepper->disableOutputs();
    stepper->setCurrentPosition(0);
    stepper->setSpeedInHz(SPEED_MAX);
    stepper->setAcceleration(2147483647);
  }

  // Wait up to 2 s for USB serial — proceed regardless so display works standalone
  unsigned long t = millis();
  while (!Serial && millis() - t < 2000) {
    delay(10);
  }
}

void loop()
{
  // ---- Button handler (edge-detected) ----------------------------------------
  uint8_t btnState = (digitalRead(BTN_A) << 0)
                   | (digitalRead(BTN_B) << 1)
                   | (digitalRead(BTN_X) << 2)
                   | (digitalRead(BTN_Y) << 3);
  if (btnState != prevBtnState) {
    updateButtons(btnState);
    prevBtnState = btnState;
  }

  // ---- Serial command parser --------------------------------------------------
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {

      case 's':
        stepper->stopMove();
        stepper->forceStop();
        stepper->disableOutputs();
        current_state = WAITING;
        break;

      case 'v':
        SPEED_MAX = Serial.parseInt();
        stepper->setSpeedInHz(SPEED_MAX);
        break;

      case 'm':
        int32_t position = Serial.parseInt();
        if (position < 0)            position = 0;
        else if (position > POS_TOP) position = POS_TOP;
        stepper->enableOutputs();
        delay(500);
        stepper->moveTo(position);
        current_state = MOVING;
        break;
    }
  }

  // ---- State machine ----------------------------------------------------------
  switch (current_state) {
    case MOVING:
      if (!stepper->isRunning()) {
        stepper->disableOutputs();
        current_state = WAITING;
      }
      break;
    case WAITING:
      break;
  }

  // ---- Periodic status update (every 100 ms) ----------------------------------
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    updateContent();

    Serial.print("{\"state\":");
    Serial.print(current_state);
    Serial.print(",\"position\":");
    Serial.print(stepper->getCurrentPosition());
    Serial.print(",\"speed\":");
    Serial.print(SPEED_MAX);
    Serial.println("}");
  }

  previous_state = current_state;
}
