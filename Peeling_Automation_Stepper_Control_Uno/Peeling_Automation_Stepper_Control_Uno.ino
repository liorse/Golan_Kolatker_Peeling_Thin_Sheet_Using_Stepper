// =============================================================================
// Project   : Peeling Thin Sheet Using Stepper Motor — Arduino Uno Controller
// File      : Peeling_Automation_Stepper_Control_Uno.ino
// Author    : Lior Segev
// Version   : 2.0.0
// Date      : April 15, 2026
// =============================================================================
//
// OVERVIEW
// --------
// This firmware controls a NEMA 17 stepper motor used to automate the peeling
// of thin sheets. It manages motor position via a serial command interface and
// emits a JSON status object every 100 ms for external software monitoring.
//
// HARDWARE
// --------
//   • Arduino Uno (ATmega328P)
//   • NEMA 17 stepper motor (0.4 A rated)
//   • TB6600 stepper driver
//       – Current-limit trim-pot target: 2.3 V on the board test-point
//   • Power supply: 12 V, 1 A minimum
//
// PIN ASSIGNMENT
// --------------
//   Pin  8  — dirPinStepper    : TB6600 direction input
//   Pin  9  — stepPinStepper   : TB6600 step pulse input
//   Pin 12  — enablePinStepper : TB6600 enable input (managed by FastAccelStepper)
//
// SOFTWARE DEPENDENCIES
// ---------------------
//   • FastAccelStepper — hardware-timer-based step generation for ATmega328P.
//   • AVRStepperPins   — pin constant definitions for FastAccelStepper on AVR.
//
// MOTOR POSITION CONSTANTS
// ------------------------
//   POS_MIDDLE =   239 042 steps  — mid-travel position (~POS_TOP / 2)
//   POS_TOP    =   478 085 steps  — full-travel top position
//   SPEED_MAX  =    30 000 Hz     — maximum step rate (full-step mode: ~1 250 Hz)
//
// STATE MACHINE
// -------------
//   INIT    — entry state; transitions to WAITING immediately.
//   MOVING  — motor is executing a moveTo() command; transitions to WAITING
//             when the target position is reached.
//   WAITING — motor is idle.
//
// SERIAL COMMAND INTERFACE  (115 200 baud, no line ending required)
// -----------------------------------------------------------------
//   Command  Argument     Description
//   -------  -----------  ------------------------------------------------
//   'm'      <int32>      Move to absolute step position (clamped to [0 … POS_TOP]).
//   's'      —            Stop the motor immediately.
//   'v'      <int>        Set motor max speed in Hz.
//
// SERIAL OUTPUT FORMAT
// --------------------
//   Every 100 ms the firmware emits a JSON object:
//       {"state":<int>,"position":<int32>}
//   State codes: INIT=1, MOVING=2, WAITING=3
// =============================================================================

// FastAccelStepper uses hardware timers (Timer1/Timer2) for precise step
// generation, allowing much higher step rates than a software AccelStepper.
#include <FastAccelStepper.h>
#include <AVRStepperPins.h>   // AVR pin-number constants for FastAccelStepper

// --- Stepper driver signal pins -----------------------------------------------
#define enablePinStepper  12  // TB6600 /EN pin; managed automatically by FastAccelStepper
#define dirPinStepper      8  // TB6600 direction input
#define stepPinStepper     9  // TB6600 step pulse input

// --- FastAccelStepper objects -------------------------------------------------
// 'engine' manages the hardware-timer resources; 'stepper' is the motor instance.
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// --- Motor travel positions (in microsteps) -----------------------------------
int32_t POS_MIDDLE = 478085 / 2;  // Mid-travel (~239 042 steps)
int32_t POS_TOP    = 478085;       // Full travel — top of peel stroke
int32_t SPEED_MAX  = 30000;        // Max step rate in Hz (full-step mode: ~1 250 Hz)

// --- State machine -----------------------------------------------------------
// State codes are transmitted in the JSON output so external software can track
// the controller lifecycle without polling extra registers.
enum state { INIT = 1, MOVING, WAITING };
//   INIT=1  MOVING=2  WAITING=3

enum state current_state  = WAITING;  // Start idle; position 0 is power-on position
enum state previous_state = INIT;

unsigned long previousMillis = 0;    // Timestamp for 100 ms JSON report cadence
const unsigned long interval  = 100; // JSON status report interval (ms)

/**
 * setup() — Arduino initialisation routine (runs once at power-on / reset).
 *
 * Sequence:
 *   1. Opens the serial port at 115 200 baud for command input and JSON output.
 *   2. Initialises FastAccelStepper: connects to the step pin, sets direction
 *      pin, enable pin (auto-managed), initial position 0, max speed, and a
 *      very high acceleration so the motor ramps quickly.
 *   3. Waits for the serial port to be ready (Leonardo/Micro compatibility —
 *      harmless on Uno but kept for portability).
 */
void setup()
{
  Serial.begin(115200);

  // FastAccelStepper setup: uses Timer1/Timer2 for hardware step generation
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);       // driver automatically enabled/disabled on motion
    stepper->setCurrentPosition(0);     // treat power-on position as zero
    stepper->setSpeedInHz(SPEED_MAX);   // 30 000 steps/s
    stepper->setAcceleration(10000000); // very high — near-instant ramp for this application
  }

  // Wait for USB serial enumeration (no-op on Uno, needed on Leonardo/Micro)
  while (!Serial) {
    delay(10);
  }
}

/**
 * loop() — Arduino main loop (runs repeatedly after setup()).
 *
 * Each iteration performs three jobs:
 *   1. Drain the serial RX buffer and dispatch any pending command.
 *   2. Evaluate the current state machine state and take appropriate action.
 *   3. Emit a JSON status object every 100 ms and save state for next iteration.
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

      case 'v':  // VELOCITY — set motor max speed in Hz
        SPEED_MAX = Serial.parseInt();
        stepper->setSpeedInHz(SPEED_MAX);
        break;

      case 'm':  // MOVE — absolute position in steps (clamped to travel limits)
        int32_t position = Serial.parseInt();
        if (position < 0) {
          position = 0;        // clamp to minimum
        } else if (position > POS_TOP) {
          position = POS_TOP;  // clamp to maximum travel
        }
        delay(100);            // brief settle before commanding move
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
      break;

    case WAITING:
      // Motor idle — nothing to do
      break;
  }

  // ---- Periodic JSON status report (every 100 ms) --------------------------
  // Format: {"state":<1-3>,"position":<steps>}
  // State codes: INIT=1  MOVING=2  WAITING=3
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("{\"state\":");
    Serial.print(current_state);
    Serial.print(",\"position\":");
    Serial.print(stepper->getCurrentPosition());
    Serial.println("}");
  }

  // ---- Save previous state for next loop -----------------------------------
  previous_state = current_state;
}
