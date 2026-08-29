// steering_bts_test.ino
//
// Standalone bring-up sketch for the NEW steering setup: BTS7960 (replacing
// the old L298N) driving the steering motor, position read back from the
// existing 3-wire analog sensor. No ROS 2 involved -- this just pulses the
// steering BTS7960 briefly at a small fixed PWM in both directions and
// prints how far the sensor moved, mirroring what ../motor_sign_test did
// for the old L298N steering, but for the new pins/driver.
//
// SAFETY: keep hands clear of the steering linkage while this runs.
//
// What it tests, once, in this order (each phase ~800ms of PWM then stop):
//   1. STEER at +TEST_PWM
//   2. STEER at -TEST_PWM
// For each phase it prints the raw sensor delta measured during that phase.
//
// How to read the results:
//   - Watch which way the steering actually turns, and note the printed
//     steer_adc_delta sign. hyper_motor_interface.ino's STEER_RAW_RIGHT/
//     STEER_RAW_LEFT already define which raw ADC value means which
//     physical lock, so as long as those two constants are still correct
//     (unaffected by this driver swap -- the sensor itself didn't change),
//     the steering P controller's sign is already consistent -- this test
//     is mainly to confirm the BTS7960 is wired so a positive command
//     actually turns the wheels (not to "fix" a sign constant for).
//     If a +PWM phase visibly drives the steering AWAY from the direction
//     that increases the measured angle, swap the steering motor's two
//     wires at the BTS7960 M+/M- terminals.
//   - Also sanity-check the motor doesn't buzz/stall at TEST_PWM -- if it
//     does, the BTS7960 may need R_EN/L_EN wired to 5V (see
//     hyper_motor_interface.ino's wiring comment) or the motor's supply
//     voltage checked.

// ---- Pins -- must match hyper_motor_interface.ino's new steering wiring ----
const uint8_t STEER_RPWM_PIN = 10;
const uint8_t STEER_LPWM_PIN = 11;
const uint8_t STEER_SENSOR_PIN = A0;

// ---- Test parameters ----
const int16_t TEST_PWM = 90;             // matches STEER_MIN_PWM in the real sketch
const unsigned long TEST_DURATION_MS = 800;
const unsigned long SETTLE_MS = 500;     // pause after the phase so motion fully stops

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

void run_steer_phase(const char *label, int16_t pwm) {
  int before = analogRead(STEER_SENSOR_PIN);
  set_bts7960(STEER_RPWM_PIN, STEER_LPWM_PIN, pwm);
  delay(TEST_DURATION_MS);
  set_bts7960(STEER_RPWM_PIN, STEER_LPWM_PIN, 0);
  int after = analogRead(STEER_SENSOR_PIN);

  Serial.print(label);
  Serial.print(": pwm=");
  Serial.print(pwm);
  Serial.print(", steer_adc_delta=");
  Serial.println(after - before);

  delay(SETTLE_MS);
}

void setup() {
  Serial.begin(115200);

  pinMode(STEER_RPWM_PIN, OUTPUT);
  pinMode(STEER_LPWM_PIN, OUTPUT);
  set_bts7960(STEER_RPWM_PIN, STEER_LPWM_PIN, 0);

  Serial.println("STEERING_BTS_TEST_READY");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  run_steer_phase("STEER +PWM", TEST_PWM);
  run_steer_phase("STEER -PWM", -TEST_PWM);

  Serial.println("TEST COMPLETE. Compare the printed signs to which way steering");
  Serial.println("actually moved -- see the notes at the top of this file.");
}

void loop() {
  // Nothing -- setup() runs the whole sequence once. Press reset to rerun.
}
