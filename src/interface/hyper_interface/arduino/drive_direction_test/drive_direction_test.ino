// drive_direction_test.ino
//
// Standalone bring-up sketch to directly correlate a positive drive PWM
// command with (a) which way the vehicle actually moves and (b) which sign
// the drive encoder reports for that motion -- on the ground, at a low
// enough PWM to watch safely, not lifted like ../drive_dual_test. No ROS 2
// involved.
//
// Why this exists: hyper_motor_interface.ino's apply_drive() computes a
// PI-controller pwm sign from (target - measured) error, then sends
// DRIVE_MOTOR_SIGN * pwm to both drive motors -- so what actually matters
// for getting DRIVE_MOTOR_SIGN right is the real-world relationship
// between "positive PWM sent to both motors" and "which way the vehicle
// physically goes", confirmed by eye on the ground -- not just whether
// front/rear agree with each other (../drive_dual_test already confirmed
// that) or what the encoder reports in isolation.
//
// SAFETY: do this somewhere the vehicle has a few meters of clear room to
// roll in either direction. TEST_PWM is low (matches DRIVE_MIN_PWM) but the
// vehicle WILL move -- don't stand in front of/behind it.
//
// What it does: drives both front and rear motors together at +TEST_PWM
// (mirroring exactly how apply_drive() drives them for a positive
// /velocity command) for TEST_DURATION_MS, printing the running encoder
// count continuously the whole time so you can watch the count change in
// real time alongside watching the vehicle move, then stops and reports
// the total delta.
//
// How to read the result:
//   - Watch which way the vehicle actually rolls while this runs.
//   - If it rolled FORWARD and encoder_delta came out POSITIVE: no sign
//     flip needed anywhere -- DRIVE_ENCODER_SIGN and DRIVE_MOTOR_SIGN in
//     hyper_motor_interface.ino should both be +1... but they're
//     currently -1/-1 (tuned against earlier tests), so if you see this
//     result, something about this test's setup differs from those
//     earlier ones (e.g. wiring changed again) -- don't just flip signs
//     blindly, figure out what's actually different first.
//   - If it rolled BACKWARD and encoder_delta came out POSITIVE: this
//     matches the real-world symptom that motivated adding
//     DRIVE_MOTOR_SIGN (commanding positive /velocity drove backward) --
//     DRIVE_MOTOR_SIGN = -1 (already set) corrects the output direction,
//     and DRIVE_ENCODER_SIGN should be whatever makes the CONTROLLER
//     see this backward motion as negative once DRIVE_MOTOR_SIGN flips
//     the output -- see the note in hyper_motor_interface.ino's
//     DRIVE_MOTOR_SIGN comment; this test's raw (pre-DRIVE_MOTOR_SIGN,
//     pre-DRIVE_ENCODER_SIGN) delta sign is the input to that reasoning,
//     not something to plug in directly.
//   - Report both the rolled direction AND the printed delta sign back
//     rather than guessing -- this determines whether DRIVE_ENCODER_SIGN
//     also needs to change alongside DRIVE_MOTOR_SIGN.

#include <avr/io.h> // PIND -- direct port read in on_drive_encoder_edge()

// ---- Pins -- must match hyper_motor_interface.ino's current drive wiring ----
const uint8_t DRIVE_FRONT_RPWM_PIN = 9;
const uint8_t DRIVE_FRONT_LPWM_PIN = 6;
const uint8_t DRIVE_REAR_RPWM_PIN = 5;
const uint8_t DRIVE_REAR_LPWM_PIN = 3;
const uint8_t DRIVE_ENC_A_PIN = 2;
const uint8_t DRIVE_ENC_B_PIN = 4;

// ---- Test parameters ----
const int16_t TEST_PWM = 90;              // matches DRIVE_MIN_PWM in the real sketch
const unsigned long TEST_DURATION_MS = 2000; // longer than ../drive_dual_test's phases --
                                              // gives enough time to actually watch it roll
const unsigned long PRINT_PERIOD_MS = 100;   // 10 Hz running count while it drives

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

  Serial.println("DRIVE_DIRECTION_TEST_READY -- make sure the vehicle has room to roll!");
  Serial.println("Starting in 3 seconds -- watch which way it goes...");
  delay(3000);

  long start_count = read_drive_count();
  unsigned long start_ms = millis();
  unsigned long last_print_ms = 0;

  set_bts7960(DRIVE_FRONT_RPWM_PIN, DRIVE_FRONT_LPWM_PIN, TEST_PWM);
  set_bts7960(DRIVE_REAR_RPWM_PIN, DRIVE_REAR_LPWM_PIN, TEST_PWM);

  while (millis() - start_ms < TEST_DURATION_MS) {
    if (millis() - last_print_ms >= PRINT_PERIOD_MS) {
      last_print_ms = millis();
      Serial.print("count=");
      Serial.println(read_drive_count());
    }
  }

  stop_all();
  long total_delta = read_drive_count() - start_count;

  Serial.print("TEST COMPLETE. pwm=+");
  Serial.print(TEST_PWM);
  Serial.print(", total encoder_delta=");
  Serial.println(total_delta);
  Serial.println("Which way did the vehicle actually roll -- forward or backward?");
  Serial.println("See the notes at the top of this file for what to do with both answers.");
}

void loop() {
  // Nothing -- setup() runs the whole sequence once. Press reset to rerun.
}
