// =============================================================================
// Project   : Peeling Thin Sheet Using Stepper Motor — Arduino Uno Controller
// File      : Peeling_Automation_Stepper_Control_Uno.ino
// Author    : Lior Segev
// Version   : 1.0.0
// Date      : July 17, 2023
// =============================================================================
//
// OVERVIEW
// --------
// This firmware controls a NEMA 17 stepper motor used to automate the peeling
// of thin sheets. It manages motor position, homing (calibration), LED
// synchronisation signals, and a serial command interface for external control.
//
// HARDWARE
// --------
//   • Arduino Uno (ATmega328P)
//   • NEMA 17 stepper motor (0.4 A rated)
//   • TB6600 stepper driver  (replaces the originally documented EasyDriver)
//       – Current-limit trim-pot target: 2.3 V on the board test-point
//   • Microswitch (Normally Open) on pin 7 — used as the home / zero switch
//   • Sync LED      on pin 10 — flashes at a randomised interval to signal
//                               the peeling rhythm to external equipment
//   • Position LED  on pin 11 — blinks 1 / 2 / 3 times to signal bottom /
//                               middle / top position acknowledge
//   • Power supply: 12 V, 1 A minimum
//       – Arduino and driver logic both powered from this supply
//
// PIN ASSIGNMENT
// --------------
//   Pin  7  — MICROSWITCH_PIN  : home switch (INPUT_PULLUP, LOW = triggered)
//   Pin  8  — dirPinStepper    : TB6600 direction input
//   Pin  9  — stepPinStepper   : TB6600 step pulse input
//   Pin 10  — LED_PIN          : sync / strobe LED output
//   Pin 11  — POSITION_LED_PIN : position-indicator LED output
//   Pin 12  — enablePinStepper : TB6600 enable input (managed by FastAccelStepper)
//
// SOFTWARE DEPENDENCIES
// ---------------------
//   • FastAccelStepper  (replaces AccelStepper) — hardware-timer-based step
//     generation for the ATmega328P, much higher step-rates than software
//     implementations.
//   • AVRStepperPins    — pin constant definitions for FastAccelStepper on AVR.
//   • Countimer         — lightweight interval/timeout timer helper.
//   • Wire              — I²C bus (reserved; MPR121 touch controller support,
//     currently unused in this revision).
//
// MOTOR POSITION CONSTANTS
// ------------------------
//   POS_GROUND  =     3 500 steps  — safe resting height above the zero switch
//   POS_MIDDLE  =   239 042 steps  — mid-travel position  (~POS_TOP / 2)
//   POS_TOP     =   478 085 steps  — full-travel top position
//   SPEED_MAX   =    30 000 Hz     — maximum step rate (full-step: ~1 250 Hz)
//
// CALIBRATION PROCEDURE
// ---------------------
//   1. Send serial command 'c' (or power-on in CALIBRATING state).
//   2. The motor drives downward indefinitely (target = -1 000 000 steps).
//   3. When the microswitch closes (pin 7 reads LOW), the driver force-stops
//      and re-labels the current encoder position as 0.
//   4. The motor then moves to POS_GROUND as the safe resting height.
//   Calibration also triggers automatically if the switch closes while the
//   motor is in MOVING state.
//
// STATE MACHINE
// -------------
//   INIT          — entry state; not used for logic, transitions immediately.
//   CALIBRATING   — drives the motor down to find the home switch (see above).
//   MOVING        — motor is executing a moveTo() command; transitions to
//                   WAITING when the target is reached.
//   WAITING       — motor is idle; the LED sync timer is active.
//   ON_MICROSWITCH— declared but not yet used in this revision.
//
// SERIAL COMMAND INTERFACE  (115 200 baud, no line ending required)
// -----------------------------------------------------------------
//   Command  Argument     Description
//   -------  -----------  ------------------------------------------------
//   'm'      <int32>      Move to absolute step position (clamped to
//                         [POS_GROUND … POS_TOP]).
//   's'      —            Stop the motor immediately (forceStop).
//   'c'      —            Enter CALIBRATING state (home sequence).
//   't'      —            Flash position LED 3× (top position acknowledge).
//   'h'      —            Flash position LED 2× (middle position acknowledge).
//   'b'      —            Flash position LED 1× (bottom position acknowledge).
//   'p'      <int>        Set LED pulse duration in ms
//                         (clamp: BOTTOM_PULSE_DURATION … TOP_PULSE_DURATION).
//   'w'      <int>        Set LED inter-pulse wait in ms
//                         (clamp: BOTTOM_PULSE_WAIT … TOP_PULSE_WAIT).
//   'v'      <int>        Set motor max speed in Hz (falls through to '1').
//   '1'      —            Start LED sync pulsing.
//   '0'      —            Stop  LED sync pulsing.
//
// SERIAL OUTPUT FORMAT
// --------------------
//   Every 100 ms the firmware emits a JSON object:
//       {"state":<int>,"position":<int32>}
//   State codes: INIT=1, MOVING=2, WAITING=3, CALIBRATING=4, ON_MICROSWITCH=5
//
// LED SYNC BEHAVIOUR
// ------------------
//   While LED_sync_timer_running is true the sync LED emits a short pulse
//   (LED_pulse_duration_ms wide) at a randomised interval centred on
//   LED_delay_between_pulses_ms (±50 %). The randomisation simulates natural
//   variation to avoid stroboscopic locking artefacts.
//   The timer is paused while the motor is MOVING and restarted on WAITING.
//
// NOTE: The 'v' command falls through to '1' (missing break). This means
//       setting speed also starts LED sync. This is likely unintentional and
//       should be reviewed.
// =============================================================================

// FastAccelStepper uses hardware timers (Timer1/Timer2) for precise step
// generation, allowing much higher step rates than a software AccelStepper.
//#include <AccelStepper.h>        // original library — replaced by FastAccelStepper
#include <FastAccelStepper.h>
#include <AVRStepperPins.h>         // AVR pin-number constants for FastAccelStepper

#include <Wire.h>                   // I²C — reserved for future MPR121 touch interface
#ifndef _BV
#define _BV(bit) (1 << (bit))       // bit-value helper required by some I²C headers
#endif
#include <Countimer.h>              // lightweight interval timer used for LED sync

// --- Pin definitions -----------------------------------------------------------
#define enablePinStepper  12  // TB6600 /EN pin; managed automatically by FastAccelStepper
//#define ENABLE_MOTOR    12  // alternate name — kept for reference
#define MICROSWITCH_PIN    7  // Home/limit switch (INPUT_PULLUP; LOW = switch closed)
#define LED_PIN           10  // Sync strobe LED output
#define POSITION_LED_PIN  11  // Position-indicator LED output

// --- LED parameter clamp limits -----------------------------------------------
#define BOTTOM_PULSE_DURATION    2  // Minimum LED pulse width (ms)
#define TOP_PULSE_DURATION     500  // Maximum LED pulse width (ms)
#define BOTTOM_PULSE_WAIT       50  // Minimum inter-pulse delay (ms)
#define TOP_PULSE_WAIT     3600000  // Maximum inter-pulse delay (ms) — 1 hour

// --- Stepper driver signal pins -----------------------------------------------
#define dirPinStepper   8   // TB6600 direction input
#define stepPinStepper  9   // TB6600 step pulse input

// --- LED sync timer -----------------------------------------------------------
// Countimer fires refreshClock() at a randomised interval to strobe LED_PIN.
Countimer LED_sync_timer;
bool LED_sync_timer_running = false;  // true = sync LED actively pulsing

// --- FastAccelStepper objects -------------------------------------------------
// 'engine' manages the hardware-timer resources; 'stepper' is the motor instance.
//AccelStepper stepper(AccelStepper::DRIVER, 9, 8);  // original — replaced
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// --- Motor travel positions (in microsteps) -----------------------------------
uint16_t POS_GROUND = 3500;           // Lowest safe resting position (above home switch)
int32_t  POS_MIDDLE = 478085 / 2;    // Mid-travel (~239 042 steps)
int32_t  POS_TOP    = 478085;         // Full travel — top of peel stroke
int32_t  SPEED_MAX  = 30000;          // Max step rate in Hz (full-step mode: ~1 250 Hz)

// --- LED timing parameters (runtime-adjustable via serial commands 'p'/'w') ---
unsigned long LED_delay_between_pulses_ms = 1000; // Nominal inter-pulse period (ms)
unsigned long LED_pulse_duration_ms       =   30; // Sync LED on-time per pulse (ms)

// --- Microswitch state --------------------------------------------------------
bool microswitch_state;  // current debounce state (not yet used for debounce logic)

/**
 * setup() — Arduino initialisation routine (runs once at power-on / reset).
 *
 * Sequence:
 *   1. Opens the serial port at 115 200 baud for command input and JSON output.
 *   2. Configures LED output pins (both start LOW / off).
 *   3. Configures the home switch pin with the internal pull-up resistor.
 *   4. Initialises FastAccelStepper: connects to the step pin, sets direction
 *      pin, enable pin (auto-managed), initial position 0, max speed, and a
 *      very high acceleration so the motor ramps quickly.
 *   5. Waits for the serial port to be ready (Leonardo/Micro compatibility —
 *      harmless on Uno but kept for portability).
 *   6. Arms the LED sync Countimer with the initial interval and starts it.
 *      Note: pulsing only occurs once LED_sync_timer_running is set to true
 *      via the '1' serial command.
 *   7. Because current_state is initialised to CALIBRATING, the first loop()
 *      iteration will immediately begin the homing sequence.
 */
void setup()
{
  Serial.begin(115200);

  // Sync strobe LED — driven HIGH only inside refreshClock() when sync is on
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Position-indicator LED — driven by gen_position_pulse() on 't'/'h'/'b' commands
  pinMode(POSITION_LED_PIN, OUTPUT);
  digitalWrite(POSITION_LED_PIN, LOW);

  // Home / limit switch — INPUT_PULLUP; switch closure pulls pin LOW
  pinMode(MICROSWITCH_PIN, INPUT_PULLUP);

  // FastAccelStepper setup: uses Timer1/Timer2 for hardware step generation
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);       // driver automatically enabled/disabled on motion
    stepper->setCurrentPosition(0);     // treat power-on position as temporary zero
    stepper->setSpeedInHz(SPEED_MAX);   // 30 000 steps/s
    stepper->setAcceleration(10000000); // very high — near-instant ramp for this application
  }

  // Wait for USB serial enumeration (no-op on Uno, needed on Leonardo/Micro)
  while (!Serial) {
    delay(10);
  }

  // Arm the LED sync timer; LED will not pulse until '1' command is received
  LED_sync_timer.setInterval(refreshClock, LED_delay_between_pulses_ms);
  LED_sync_timer.start();
}

/**
 * refreshClock() — LED sync timer callback.
 *
 * Called by LED_sync_timer at each interval expiry.
 * If sync is enabled (LED_sync_timer_running), emits one pulse on LED_PIN.
 * After each call it re-sets the next interval to a random value in the range
 * [0.5 × LED_delay_between_pulses_ms … 1.5 × LED_delay_between_pulses_ms],
 * introducing ±50 % jitter to avoid stroboscopic locking artefacts.
 */
void refreshClock() {
  if (LED_sync_timer_running) {
    digitalWrite(LED_PIN, HIGH);
    delay(LED_pulse_duration_ms);   // blocking delay — acceptable: pulse is very short
    digitalWrite(LED_PIN, LOW);
  }
  // Randomise next interval: ±50 % of the nominal period
  int minRange = LED_delay_between_pulses_ms * 0.5;
  int maxRange = LED_delay_between_pulses_ms * 1.5;
  LED_sync_timer.setInterval(refreshClock, random(minRange, maxRange + 1));
}

/**
 * onComplete() — placeholder callback for Countimer countdown completion.
 * Currently a stub (no-op). Reserved for future use (e.g., safety timeout).
 */
void onComplete() {
  int c = 1;
  c++;  // no-op
}

/**
 * gen_position_pulse(n) — blink POSITION_LED_PIN n times.
 *
 * Used to give visual feedback about which position the motor has reached:
 *   n = 1 → bottom (b command)
 *   n = 2 → middle (h command)
 *   n = 3 → top    (t command)
 *
 * Pulse width and gap both equal LED_pulse_duration_ms.
 */
void gen_position_pulse(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(POSITION_LED_PIN, HIGH);
    delay(LED_pulse_duration_ms);
    digitalWrite(POSITION_LED_PIN, LOW);
    delay(LED_pulse_duration_ms);
  }
}

// --- State machine -----------------------------------------------------------
// State codes are transmitted in the JSON output so external software can track
// the controller lifecycle without polling extra registers.
enum state { INIT = 1, MOVING, WAITING, CALIBRATING, ON_MICROSWITCH };
//   INIT=1  MOVING=2  WAITING=3  CALIBRATING=4  ON_MICROSWITCH=5 (reserved)

enum state current_state  = CALIBRATING; // Start with a homing sequence on power-up
enum state previous_state = INIT;

unsigned long previousMillis = 0;         // Timestamp for 100 ms JSON report cadence
const unsigned long interval  = 100;      // JSON status report interval (ms)
bool previous_switch_state = HIGH;        // Used to detect the falling edge of the home switch

/**
 * loop() — Arduino main loop (runs repeatedly after setup()).
 *
 * Each iteration performs three jobs:
 *   1. Drain the serial RX buffer and dispatch any pending command.
 *   2. Evaluate the current state machine state and take appropriate action.
 *   3. Emit a JSON status object every 100 ms, service the LED sync timer,
 *      and save state for edge-detection on the next iteration.
 */
void loop()
{
  // ---- Serial command parser ------------------------------------------------
  // Single-character commands; numeric arguments read immediately after via parseInt().
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {

      case 's':  // STOP — abort any in-progress move immediately
        stepper->stopMove();
        stepper->forceStop();
        current_state = WAITING;
        break;

      case 'c':  // CALIBRATE — trigger homing sequence
        current_state = CALIBRATING;
        break;

      case 't':  // TOP acknowledge — blink position LED 3 times
        gen_position_pulse(3);
        break;

      case 'h':  // HALF / middle acknowledge — blink position LED 2 times
        gen_position_pulse(2);
        break;

      case 'b':  // BOTTOM acknowledge — blink position LED 1 time
        gen_position_pulse(1);
        break;

      case 'p':  // PULSE duration — set LED on-time in ms (clamped)
        LED_pulse_duration_ms = Serial.parseInt();
        if (LED_pulse_duration_ms < BOTTOM_PULSE_DURATION) {
          LED_pulse_duration_ms = BOTTOM_PULSE_DURATION;
        } else if (LED_pulse_duration_ms > TOP_PULSE_DURATION) {
          LED_pulse_duration_ms = TOP_PULSE_DURATION;
        }
        break;

      case 'w':  // WAIT — set inter-pulse delay in ms (clamped); restarts timer
        LED_delay_between_pulses_ms = Serial.parseInt();
        if (LED_delay_between_pulses_ms < BOTTOM_PULSE_WAIT) {
          LED_delay_between_pulses_ms = BOTTOM_PULSE_WAIT;
        } else if (LED_delay_between_pulses_ms > TOP_PULSE_WAIT) {
          LED_delay_between_pulses_ms = TOP_PULSE_WAIT;
        }
        LED_sync_timer.setInterval(refreshClock, LED_delay_between_pulses_ms);
        break;

      case 'v':  // VELOCITY — set motor speed in Hz
        // NOTE: missing 'break' causes fall-through into case '1';
        // setting speed will also start LED sync. Review if unintended.
        SPEED_MAX = Serial.parseInt();
        stepper->setSpeedInHz(SPEED_MAX);
        /* fall through */

      case '1':  // LED sync ON
        LED_sync_timer_running = true;
        break;

      case '0':  // LED sync OFF
        LED_sync_timer_running = false;
        break;

      case 'm':  // MOVE — absolute position in steps (clamped to travel limits)
        int32_t position = Serial.parseInt();
        if (position < POS_GROUND) {
          position = POS_GROUND;  // never drive into the home switch
        } else if (position > POS_TOP) {
          position = POS_TOP;     // never exceed travel range
        }
        delay(100);               // brief settle before commanding move
        stepper->moveTo(position);
        current_state = MOVING;
        break;
    }
  }

  // ---- State machine --------------------------------------------------------
  switch (current_state) {

    case MOVING:
      // Transition to WAITING once the target step count is reached
      if (stepper->getCurrentPosition() == stepper->targetPos()) {
        current_state = WAITING;
      }
      // Safety: if home switch closes unexpectedly during a move, re-home and
      // then rise to POS_GROUND (falling-edge detection via previous_switch_state)
      if (digitalRead(MICROSWITCH_PIN) == LOW && previous_switch_state == HIGH) {
        stepper->setCurrentPosition(0);  // declare this as position zero
        stepper->moveTo(POS_GROUND);     // immediately rise to safe height
        current_state = MOVING;
      }
      LED_sync_timer.stop(); // suppress sync LED during motor motion
      break;

    case WAITING:
      // Motor idle — re-enable the sync LED timer so pulsing resumes
      LED_sync_timer.restart();
      break;

    case CALIBRATING:
      if (digitalRead(MICROSWITCH_PIN) == HIGH) {
        // Switch not yet triggered — begin (or continue) driving downward.
        // Only issue moveTo() on the first loop iteration in this state to
        // avoid re-issuing the command every loop cycle.
        if (previous_state != current_state) {
          stepper->moveTo(-1000000); // large negative target guarantees the switch is hit
        }
      } else {
        // Home switch closed — zero the encoder and rise to safe height
        stepper->forceStopAndNewPosition(0);
        Serial.println(stepper->getCurrentPosition()); // confirm new zero position
        stepper->moveTo(POS_GROUND);
        current_state = MOVING;
      }
      break;
  }

  
  // ---- Periodic JSON status report (every 100 ms) --------------------------
  // Format: {"state":<1-5>,"position":<steps>}
  // State codes: INIT=1 MOVING=2 WAITING=3 CALIBRATING=4 ON_MICROSWITCH=5
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("{\"state\":");
    Serial.print(current_state);
    Serial.print(",\"position\":");
    Serial.print(stepper->getCurrentPosition());
    Serial.println("}");
  }

  // ---- Save previous state for edge detection next loop --------------------
  previous_state        = current_state;
  previous_switch_state = digitalRead(MICROSWITCH_PIN);

  // ---- Service the LED sync timer ------------------------------------------
  LED_sync_timer.run();

}


