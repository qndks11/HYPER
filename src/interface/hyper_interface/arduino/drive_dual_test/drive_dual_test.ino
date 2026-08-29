// drive_dual_test.ino
//
// Standalone bring-up sketch for the NEW drive setup: two BTS7960 drive
// motors (front + rear), sharing ONE drive encoder between them (see
// hyper_motor_interface.ino's header comment for why -- no independent
// front/rear feedback). No ROS 2 involved.
//
// This test pulses front and rear EACH ALONE first (so a wiring/direction
// mistake on either one shows up on its own, not masked by the other), then
// both TOGETHER at once (mirroring exactly how hyper_motor_interface.ino
// drives them in normal operation) so you can also confirm they agree with
// each other and don't fight/cancel out.
//
// SAFETY: lift the drive wheels off the ground before running this.
//
// What it tests, once, in this order (each phase ~800ms of PWM then stop):
//   1. FRONT alone at +TEST_PWM
//   2. FRONT alone at -TEST_PWM
//   3. REAR alone at +TEST_PWM
//   4. REAR alone at -TEST_PWM
//   5. BOTH together at +TEST_PWM
//   6. BOTH together at -TEST_PWM
// For each phase it prints the raw encoder count delta measured during
// that phase.
//
// How to read the results:
//   - Phases 1-2 (FRONT alone): watch which way the front wheel spins, note
//     the sign of encoder_delta for each. This tells you FRONT's own
//     RPWM-to-direction sign, independent of rear.
//   - Phases 3-4 (REAR alone): same, but for the rear wheel. IMPORTANT: the
//     rear motor is on a completely separate BTS7960 -- there's no
//     guarantee "RPWM > 0" means the same physical direction for it as it
//     does for front just because the firmware sends them the same PWM.
//     Wire (or note down a sign flip for) whichever one disagrees so both
//     actually drive the vehicle the same way when combined in phases 5-6.
//   - Phases 5-6 (BOTH together): if front and rear individually agreed
//     (phases 1-4 gave consistent signs), this should just show roughly
//     double the wheel-spin/encoder_delta magnitude of a single-motor
//     phase (since the encoder is upstream of the front motor's own gear
//     reduction -- rear's contribution to what the encoder sees depends on
//     how the drivetrain couples them, so don't expect an exact 2x, just
//     "clearly more than one motor alone, same sign"). If instead it looks
//     WEAKER or reversed relative to a single-motor phase, front and rear
//     are fighting each other -- go fix whichever one had the wrong sign
//     in phases 1-4 before trusting hyper_motor_interface.ino's closed loop.

#include <avr/io.h> // PIND -- direct port read in on_drive_encoder_edge()

// ---- Pins -- must match hyper_motor_interface.ino's new drive wiring ----
const uint8_t DRIVE_FRONT_RPWM_PIN = 9;
const uint8_t DRIVE_FRONT_LPWM_PIN = 6;
const uint8_t DRIVE_REAR_RPWM_PIN = 5;
const uint8_t DRIVE_REAR_LPWM_PIN = 3;
const uint8_t DRIVE_ENC_A_PIN = 2;
const uint8_t DRIVE_ENC_B_PIN = 4;

// ---- Test parameters ----
const int16_t TEST_PWM = 90;             // matches DRIVE_MIN_PWM in the real sketch
const unsigned long TEST_DURATION_MS = 800;
const unsigned long SETTLE_MS = 500;     // pause between phases so motion fully stops

volatile long drive_encoder_count = 0;

// Mirrors on_drive_encoder_edge() (x1 decoding) in hyper_motor_interface.ino.
void on_drive_encoder_edge() {
  if ((PIND >> 4) & 0x1) {
    drive_encoder_count--;
  } else {
    drive_encoder_count++;
  }
}

// Mirrors set_bts7960() in hyper_motor_interface.ino.
void set_bts7960(uint8_t rpwm_pin, uint8_t lpwm_pin, int16_t pwm) {
  if (pwm > 0) {
    analogWrite(rpwm_pin, pwm);
    analogWrite(lpwm_pin, 0);
  } else if (pwm < 0) {
    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, -pwm);
  } else {
    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, 0);
  }
}

void stop_all() {
  set_bts7960(DRIVE_FRONT_RPWM_PIN, DRIVE_FRONT_LPWM_PIN, 0);
  set_bts7960(DRIVE_REAR_RPWM_PIN, DRIVE_REAR_LPWM_PIN, 0);
}

long read_drive_count() {
  long count;
  noInterrupts();
  count = drive_encoder_count;
  interrupts();
  return count;
}

// front_pwm/rear_pwm: pass 0 for whichever motor should stay off this phase.
void run_drive_phase(const char *label, int16_t front_pwm, int16_t rear_pwm) {
  long before = read_drive_count();
  set_bts7960(DRIVE_FRONT_RPWM_PIN, DRIVE_FRONT_LPWM_PIN, front_pwm);
  set_bts7960(DRIVE_REAR_RPWM_PIN, DRIVE_REAR_LPWM_PIN, rear_pwm);
  delay(TEST_DURATION_MS);
  stop_all();
  long after = read_drive_count();

  Serial.print(label);
  Serial.print(": front_pwm=");
  Serial.print(front_pwm);
  Serial.print(", rear_pwm=");
  Serial.print(rear_pwm);
  Serial.print(", encoder_delta=");
  Serial.println(after - before);

  delay(SETTLE_MS);
}

void setup() {
  Serial.begin(115200);

  pinMode(DRIVE_FRONT_RPWM_PIN, OUTPUT);
  pinMode(DRIVE_FRONT_LPWM_PIN, OUTPUT);
  pinMode(DRIVE_REAR_RPWM_PIN, OUTPUT);
  pinMode(DRIVE_REAR_LPWM_PIN, OUTPUT);
  stop_all();

  pinMode(DRIVE_ENC_A_PIN, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A_PIN), on_drive_encoder_edge, RISING);

  Serial.println("DRIVE_DUAL_TEST_READY -- lift the drive wheels off the ground!");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  Serial.println("--- FRONT alone ---");
  run_drive_phase("FRONT +PWM", TEST_PWM, 0);
  run_drive_phase("FRONT -PWM", -TEST_PWM, 0);

  Serial.println("--- REAR alone ---");
  run_drive_phase("REAR +PWM", 0, TEST_PWM);
  run_drive_phase("REAR -PWM", 0, -TEST_PWM);

  Serial.println("--- BOTH together ---");
  run_drive_phase("BOTH +PWM", TEST_PWM, TEST_PWM);
  run_drive_phase("BOTH -PWM", -TEST_PWM, -TEST_PWM);

  Serial.println("TEST COMPLETE. See the notes at the top of this file for how to");
  Serial.println("read these six deltas.");
}

void loop() {
  // Nothing -- setup() runs the whole sequence once. Press reset to rerun.
}
