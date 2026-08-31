// motor_sign_test.ino
//
// Standalone bring-up sketch to find the sign convention for both motor
// channels before trusting hyper_motor_interface.ino's closed loops with
// them. No ROS 2, no PI/P controller involved -- this just pulses each
// channel briefly at a small fixed PWM in both directions and prints how
// far the corresponding sensor moved, so the direction can be read off by
// eye + the printed numbers instead of trial-and-error editing the real
// control sketch.
//
// SAFETY: lift the drive wheels off the ground before running this. Pins,
// gains-free PWM level, and wiring match hyper_motor_interface.ino -- see
// that file's wiring comments for the full BTS7960/L298N/encoder/sensor
// hookup (drive is on a BTS7960, steering stayed on the L298N).
//
// What it tests, once, in this order (each phase ~800ms of PWM then stop):
//   1. DRIVE at +TEST_PWM
//   2. DRIVE at -TEST_PWM
//   3. STEER at +TEST_PWM
//   4. STEER at -TEST_PWM
// For each phase it prints the raw sensor delta measured during that phase.
//
// How to read the results:
//   - DRIVE phases: watch which way the wheel actually spins, and note the
//     printed encoder_delta sign for each. If "+PWM" phase's wheel spin
//     direction isn't the one you want to call "forward", swap the drive
//     motor's two wires at the BTS7960 M+/M- terminals (a wiring fix, not
//     a code fix). Separately: DRIVE_ENCODER_SIGN in hyper_motor_interface.ino
//     should be set so that (+PWM phase's encoder_delta sign) x
//     DRIVE_ENCODER_SIGN comes out positive -- that is what keeps the PI
//     controller's feedback loop stable (prevents the runaway/overheating
//     failure mode), independent of which physical direction ends up being
//     "forward".
//   - STEER phases: watch which way the steering actually turns, and note
//     the printed steer_adc_delta sign. hyper_motor_interface.ino's
//     STEER_RAW_RIGHT/STEER_RAW_LEFT already define which raw ADC value
//     means which physical lock, so as long as those two constants were
//     measured correctly (via steering_sensor_test.ino), the steering P
//     controller's sign is already consistent -- this test is mainly to
//     confirm that (not something you need to "fix" a sign constant for).
//     If a +PWM phase visibly drives the steering AWAY from the direction
//     that increases the measured angle, swap the steering motor's two
//     wires at the L298N OUT3/OUT4 terminals.

#include <avr/io.h> // PIND -- direct port read in on_drive_encoder_edge()

// ---- Pins -- must match hyper_motor_interface.ino ----
const uint8_t DRIVE_RPWM_PIN = 9;
const uint8_t DRIVE_LPWM_PIN = 6;
// BTS7960 R_EN/L_EN are wired straight to 5V, not to Arduino pins.
const uint8_t STEER_EN_PIN = 10;
const uint8_t STEER_IN1_PIN = 12;
const uint8_t STEER_IN2_PIN = 11;
const uint8_t DRIVE_ENC_A_PIN = 2;
const uint8_t DRIVE_ENC_B_PIN = 3;
const uint8_t STEER_SENSOR_PIN = A0;

// ---- Test parameters ----
const int16_t TEST_PWM = 60;             // small, safe -- matches *_MIN_PWM in the real sketch
const unsigned long TEST_DURATION_MS = 800;
const unsigned long SETTLE_MS = 500;     // pause between phases so motion fully stops

volatile long drive_encoder_count = 0;
volatile uint8_t drive_last_ab_state = 0;

const int8_t QUADRATURE_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void on_drive_encoder_edge() {
  uint8_t port_d = PIND;
  uint8_t a = (port_d >> 2) & 0x1;
  uint8_t b = (port_d >> 3) & 0x1;
  uint8_t current_state = (a << 1) | b;
  uint8_t index = (drive_last_ab_state << 2) | current_state;
  drive_encoder_count += QUADRATURE_TABLE[index];
  drive_last_ab_state = current_state;
}

void set_channel(uint8_t en_pin, uint8_t in1_pin, uint8_t in2_pin, int16_t pwm) {
  digitalWrite(in1_pin, pwm > 0 ? HIGH : LOW);
  digitalWrite(in2_pin, pwm < 0 ? HIGH : LOW);
  analogWrite(en_pin, abs(pwm));
}

// Mirrors set_drive_bts7960() in hyper_motor_interface.ino.
void set_drive_bts7960(int16_t pwm) {
  if (pwm > 0) {
    analogWrite(DRIVE_RPWM_PIN, pwm);
    analogWrite(DRIVE_LPWM_PIN, 0);
  } else if (pwm < 0) {
    analogWrite(DRIVE_RPWM_PIN, 0);
    analogWrite(DRIVE_LPWM_PIN, -pwm);
  } else {
    analogWrite(DRIVE_RPWM_PIN, 0);
    analogWrite(DRIVE_LPWM_PIN, 0);
  }
}

void stop_all() {
  set_drive_bts7960(0);
  set_channel(STEER_EN_PIN, STEER_IN1_PIN, STEER_IN2_PIN, 0);
}

long read_drive_count() {
  long count;
  noInterrupts();
  count = drive_encoder_count;
  interrupts();
  return count;
}

// Runs one phase: drives one channel at the given PWM for TEST_DURATION_MS,
// stops, then prints how far the given sensor moved during that phase.
void run_drive_phase(const char *label, int16_t pwm) {
  long before = read_drive_count();
  set_drive_bts7960(pwm);
  delay(TEST_DURATION_MS);
  set_drive_bts7960(0);
  long after = read_drive_count();

  Serial.print(label);
  Serial.print(": pwm=");
  Serial.print(pwm);
  Serial.print(", encoder_delta=");
  Serial.println(after - before);

  delay(SETTLE_MS);
}

void run_steer_phase(const char *label, int16_t pwm) {
  int before = analogRead(STEER_SENSOR_PIN);
  set_channel(STEER_EN_PIN, STEER_IN1_PIN, STEER_IN2_PIN, pwm);
  delay(TEST_DURATION_MS);
  set_channel(STEER_EN_PIN, STEER_IN1_PIN, STEER_IN2_PIN, 0);
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

  pinMode(DRIVE_RPWM_PIN, OUTPUT);
  pinMode(DRIVE_LPWM_PIN, OUTPUT);
  pinMode(STEER_EN_PIN, OUTPUT);
  pinMode(STEER_IN1_PIN, OUTPUT);
  pinMode(STEER_IN2_PIN, OUTPUT);
  stop_all();

  pinMode(DRIVE_ENC_A_PIN, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B_PIN, INPUT_PULLUP);
  drive_last_ab_state = (digitalRead(DRIVE_ENC_A_PIN) << 1) | digitalRead(DRIVE_ENC_B_PIN);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A_PIN), on_drive_encoder_edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B_PIN), on_drive_encoder_edge, CHANGE);

  Serial.println("MOTOR_SIGN_TEST_READY -- lift the drive wheels off the ground!");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  Serial.println("--- DRIVE ---");
  run_drive_phase("DRIVE +PWM", TEST_PWM);
  run_drive_phase("DRIVE -PWM", -TEST_PWM);

  Serial.println("--- STEER ---");
  run_steer_phase("STEER +PWM", TEST_PWM);
  run_steer_phase("STEER -PWM", -TEST_PWM);

  Serial.println("TEST COMPLETE. Compare the printed signs to which way each");
  Serial.println("channel actually moved, then see the notes at the top of");
  Serial.println("this file for what to fix and where.");
}

void loop() {
  // Nothing -- setup() runs the whole sequence once. Press the Uno's reset
  // button to run it again.
}
