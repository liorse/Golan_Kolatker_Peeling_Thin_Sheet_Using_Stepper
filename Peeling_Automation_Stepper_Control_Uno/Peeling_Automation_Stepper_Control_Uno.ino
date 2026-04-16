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
//   • DM542T (V4.0) stepper driver — Leadshine digital driver
//       – Peak current set via DIP switches SW1–SW3
//       – Microstepping resolution set via DIP switches SW4–SW6
//       – ENA active-low; t1: ENA→first PUL ≥ 200 ms (datasheet Fig.15)
//       – t2: DIR stable before PUL ≥ 5 µs; t3/t4: PUL high/low width ≥ 2.5 µs
//       – Signal levels: HIGH > 3.5 V, LOW < 0.5 V — Arduino 5 V compatible directly
//   • Power supply: 24–48 V recommended (min 12 V)
//
// PIN ASSIGNMENT
// --------------
//   Pin  8  — dirPinStepper    : DM542T DIR- input
//   Pin  9  — stepPinStepper   : DM542T PUL- input
//   Pin 12  — enablePinStepper : DM542T ENA- input (active-low; driven manually)
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
//       {"state":<int>,"position":<int32>,"speed":<int32>}
//   State codes: INIT=1, MOVING=2, WAITING=3
//   speed: current configured max speed in Hz (SPEED_MAX)
// =============================================================================

// FastAccelStepper uses hardware timers (Timer1/Timer2) for precise step
// generation, allowing much higher step rates than a software AccelStepper.
#include <FastAccelStepper.h>
#include <AVRStepperPins.h>   // AVR pin-number constants for FastAccelStepper

// --- Stepper driver signal pins -----------------------------------------------
#define enablePinStepper  12  // DM542T ENA- pin (active-low); driven manually — HIGH = disabled, LOW = enabled
#define dirPinStepper      8  // DM542T DIR- input
#define stepPinStepper     9  // DM542T PUL- input

// --- FastAccelStepper objects -------------------------------------------------
// 'engine' manages the hardware-timer resources; 'stepper' is the motor instance.
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// --- Motor travel positions (in microsteps) -----------------------------------
int32_t POS_MIDDLE = 478085 / 2;  // Mid-travel (~239 042 steps)
int32_t POS_TOP    = 478085;       // Full travel — top of peel stroke
int32_t SPEED_MAX  = 100;        // Max step rate in Hz (full-step mode: ~1 250 Hz)

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
 *   2. Configures the ENA- pin as OUTPUT and starts with the driver disabled
 *      (HIGH = disabled for active-low ENA-).
 *   3. Initialises FastAccelStepper: connects to the step pin, sets direction
 *      pin, initial position 0, max speed, and a very high acceleration.
 *   4. Waits for the serial port to be ready (Leonardo/Micro compatibility —
 *      harmless on Uno but kept for portability).
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
    stepper->setDirectionPin(dirPinStepper, true, 40); // dirHighCountsUp=true; dir_change_delay_us=40 µs (≥ t2 per DM542T datasheet)
    stepper->setEnablePin(enablePinStepper); // register ENA- pin so enableOutputs()/disableOutputs() control it
    stepper->setAutoEnable(false);           // do NOT auto-assert ENA- on move; we call enableOutputs() manually
    stepper->disableOutputs();               // start with driver disabled (ENA- HIGH)
    stepper->setCurrentPosition(0);          // treat power-on position as zero
    stepper->setSpeedInHz(SPEED_MAX);        // steps/s
    stepper->setAcceleration(2147483647); // effectively no acceleration
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
        stepper->disableOutputs(); // assert ENA- HIGH — driver disabled
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
        // Assert ENA- LOW then wait t1 > 200 ms before first pulse
        // (DM542T datasheet Fig.15) — setDelayToEnable() is unreliable on Uno.
        stepper->enableOutputs(); // assert ENA- LOW — driver enabled
        delay(500);
        stepper->moveTo(position);
        current_state = MOVING;
        break;
    }
  }

  // ---- State machine --------------------------------------------------------
  switch (current_state) {

    case MOVING:
      // Disable driver and transition to WAITING only once FastAccelStepper has
      // finished generating pulses (isRunning() is false after full deceleration).
      // Using getCurrentPosition()==targetPos() is unreliable — it can fire before
      // the hardware timer ISR has even started the move.
      if (!stepper->isRunning()) {
        stepper->disableOutputs(); // assert ENA- HIGH — driver disabled
        current_state = WAITING;
      }
      break;

    case WAITING:
      // Motor idle — nothing to do
      break;
  }

  // ---- Periodic JSON status report (every 100 ms) --------------------------
  // Format: {"state":<1-3>,"position":<steps>,"speed":<Hz>}
  // State codes: INIT=1  MOVING=2  WAITING=3
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("{\"state\":");
    Serial.print(current_state);
    Serial.print(",\"position\":");
    Serial.print(stepper->getCurrentPosition());
    Serial.print(",\"speed\":");
    Serial.print(SPEED_MAX);
    Serial.println("}");
  }

  // ---- Save previous state for next loop -----------------------------------
  previous_state = current_state;
}
