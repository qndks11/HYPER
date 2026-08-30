// hyper_motor_interface.ino
//
// Receives {velocity [m/s], steering_angle [rad]} commands from
// hyper_interface arduino_interface_node over USB serial and drives THREE
// motors, all BTS7960-driven: two drive motors (front + rear -- both wired
// in parallel off the same PI output, see apply_drive(), since this vehicle
// has only one drive encoder shared between them, not one per motor) and
// one steering motor. This is an Uno/Nano pin budget squeeze: 3x BTS7960
// needs 6 PWM pins (RPWM+LPWM each), which is every PWM-capable pin this
// board has (3, 5, 6, 9, 10, 11) -- see the pin assignment block below for
// how they're divided up, and don't reassign a pin here without checking
// it's still PWM-capable and not needed elsewhere.
//
// Drive has a magnetic ABI (quadrature) encoder (see ../magnetic_encoder_test)
// mounted on the (front) drive motor shaft -- upstream of the drivetrain's
// gear reduction, not on the wheel itself -- so both drive motors run
// closed loop together off it: a PI controller compares measured wheel
// velocity (derived from the encoder via DRIVE_GEAR_RATIO and
// WHEEL_CIRCUMFERENCE_M below) to the commanded velocity and drives both
// motors' RPWM/LPWM identically toward it (see apply_drive()). There is no
// independent feedback for the rear motor -- it's assumed to track the
// front motor closely enough driving the same PWM open-loop between them;
// if front/rear ever need independently tuned behavior (e.g. one wheel
// slipping more than the other), this single-encoder/mirrored-PWM scheme
// can't express that.
// DRIVE_GEAR_RATIO and WHEEL_DIAMETER_M were measured by hand on this
// vehicle -- see ../magnetic_encoder_test's README/comments for the
// measurement procedure (spin the drive wheel a known number of turns,
// read the raw encoder count delta, divide). Re-measure and update them if
// the drivetrain, wheels, or this encoder's mounting change.
// DRIVE_ENCODER_SIGN needs re-checking after any drive motor/driver swap --
// which physical direction RPWM vs. LPWM drives the motor may have changed
// even if the wiring "looks the same", so don't assume the old sign still
// holds; re-verify with ../motor_sign_test before trusting the closed loop
// again.
//
// Steering has a 3-wire analog position sensor (see ../steering_sensor_test)
// geared to the steering shaft, so it runs closed loop: a simple P
// controller reads the sensor, compares it to the commanded steering_angle,
// and drives STEER_RPWM/LPWM toward it, stopping once it is within
// STEER_DEADBAND_RAD. STEER_RAW_RIGHT/STEER_RAW_LEFT below are the raw
// analogRead() values measured at each steering lock -- if the sensor, its
// gearing, or its supply voltage change, re-run steering_sensor_test.ino and
// update those two constants. Steering just moved from an L298N to a
// BTS7960 (more current/torque, and different electrical noise
// characteristics) -- STEER_KP/STEER_MIN_PWM/STEER_DEADBAND_RAD below were
// tuned for the old L298N and will likely need re-tuning the same way the
// drive motor's did after ITS BTS7960 swap (see that history in this file)
// -- watch for the same trembling/oscillation symptoms and lower
// STEER_KP/raise STEER_AVG_SAMPLES first if so.
//
// Wire format (11 bytes, little-endian) -- must match _make_packet() in
// hyper_interface/arduino_interface_node.py:
//   byte 0    : 0xAA              (start-of-frame 1)
//   byte 1    : 0x55              (start-of-frame 2)
//   byte 2-5  : float32           velocity        [m/s]
//   byte 6-9  : float32           steering_angle  [rad]
//   byte 10   : uint8             checksum = XOR of bytes 2-9
//
// BTS7960 wiring (x3 -- drive front, drive rear, steering; identical
// pattern for each, just different pins):
//   *_RPWM_PIN (PWM) -> BTS7960 RPWM
//   *_LPWM_PIN (PWM) -> BTS7960 LPWM
//   BTS7960 R_EN, L_EN   -> Arduino 5V directly (not an Arduino pin -- this
//                           sketch never toggles them, so there's no reason
//                           to spend a GPIO pin on them)
// BTS7960 R_IS/L_IS (current sense) are left unconnected -- not read by
// this sketch. BTS7960 M+/M- go to that motor; B+/B- to its power supply,
// VCC/GND to 5V logic.
// Only one of RPWM/LPWM is ever driven at a time per motor (see
// set_bts7960()): RPWM > 0 for one direction, LPWM > 0 for the other, both
// 0 = coast. The two drive motors' BTS7960 units get identical RPWM/LPWM
// values every tick (see apply_drive()) -- wire drive front and drive rear
// motors so that "RPWM > 0" means "forward" on BOTH (i.e. check each
// motor's rotation direction independently with ../motor_sign_test, don't
// assume they match just because the PWM is identical).
//
// Drive encoder wiring (see ../magnetic_encoder_test for the ABI-mode board
// details -- pin 2 must be interrupt-capable):
//   encoder A -> Arduino pin 2 (INT0 -- x1 decoding, see on_drive_encoder_edge())
//   encoder B -> Arduino pin 4 (plain digital read, not an interrupt source
//               -- moved off pin 3 when steering's BTS7960 swap needed pin 3
//               as a 6th PWM output; x1 decoding only needs A to be
//               interrupt-capable, B can be any free digital pin)
//   encoder 5V/GND -> Arduino 5V/GND (JP1 set to 5V on the AS5047P board)
// DRIVE_ENCODER_SIGN below may need flipping: after uploading, command a
// small positive /velocity and confirm the VEL telemetry line's "current"
// value comes out positive too -- if it's negative instead, flip the sign.

#include <string.h>   // memcpy
#include <math.h>     // fabs
#include <avr/io.h>   // PIND -- direct port read in on_drive_encoder_edge()

// ---- Pin assignment (change to match your wiring) ----
// All three BTS7960 RPWM/LPWM pins below must be PWM-capable -- on an
// Uno/Nano that's only pins 3, 5, 6, 9, 10, 11, and all six are spoken for
// here. R_EN/L_EN on every BTS7960 are wired straight to 5V (see the wiring
// comment above), not to Arduino pins.
const uint8_t DRIVE_FRONT_RPWM_PIN = 9;   // unchanged from the single-drive-motor wiring
const uint8_t DRIVE_FRONT_LPWM_PIN = 6;   // unchanged from the single-drive-motor wiring
const uint8_t DRIVE_REAR_RPWM_PIN = 5;
const uint8_t DRIVE_REAR_LPWM_PIN = 3;
const uint8_t STEER_RPWM_PIN = 10;
const uint8_t STEER_LPWM_PIN = 11;

// ---- Drive encoder (see ../magnetic_encoder_test) ----
const uint8_t DRIVE_ENC_A_PIN = 2;  // must be interrupt-capable (INT0 on Uno/Nano)
const uint8_t DRIVE_ENC_B_PIN = 4;  // read only, not an interrupt source (x1 decoding)

// Encoder's rated pulses-per-revolution (see ../magnetic_encoder_test,
// measured empirically via its index/Z pulse: 1000 PPR on this unit).
const long DRIVE_ENCODER_PULSES_PER_REV = 1000;
// x1 decoding (only A's rising edge counts, direction recovered from B) --
// x4 (both A and B, every edge) was CPU-expensive enough at this encoder's
// resolution x gear ratio to saturate the CPU and stall loop() at higher
// drive speeds (symptom: steering stops responding once driving fast
// enough). A first x1 attempt combined this decode-scheme change with also
// switching to a raw ISR(INT0_vect)/EICRA/EIMSK register setup (instead of
// attachInterrupt()) and left drive_encoder_count stuck at 0 permanently --
// never diagnosed which of the two changes broke it, so both were reverted
// together back to x4/attachInterrupt() to get back to a known-working
// state. This x1 attempt changes only the decode scheme, keeping
// attachInterrupt() (the same proven mechanism x4 used) -- verified working
// standalone first in ../encoder_x1_test (count changes correctly, forward/
// backward give opposite signs matching x4's existing DRIVE_ENCODER_SIGN
// convention) before porting the logic here.
const long DRIVE_COUNTS_PER_REV = DRIVE_ENCODER_PULSES_PER_REV;

// Encoder is mounted on the drive MOTOR shaft, upstream of the drivetrain's
// gear reduction -- not on the wheel itself. Measured by hand: spin the
// drive wheel a known number of turns, read the raw encoder count delta,
// divide by (DRIVE_COUNTS_PER_REV x turns). Re-measure if the drivetrain or
// this encoder's mounting changes.
//
// NOT the true mechanical gear ratio anymore -- direct hand-turn
// measurement (see ../gear_ratio_test) gave 43.46, close to the original
// 42.24 (~2.9% off, normal hand-measurement noise). But /odom during actual
// driving consistently overestimated distance by ~70-78% regardless of
// gear ratio, wheel diameter, or driving speed/style (jerky, smooth, slow
// -- slow was WORSE, which rules out wheel slip and instead points at
// BTS7960 PWM switching noise injecting spurious edges onto the drive
// encoder lines, the same class of issue STEER_AVG_SAMPLES works around on
// the steering sensor -- only present while the motor is actually being
// driven, which is why the hand-turn test (motor off) came out clean.
// Rather than fix the actual noise (shielding/filtering the encoder
// wiring, or debouncing in software), this value was inflated to
// compensate empirically: 43.46 * (3.5605m odom / 2m actual) = 77.37.
// Re-corrected again after 77.37 turned out to now UNDER-estimate by
// ~11% (two repeat runs: 1.775m and 1.778m odom for ~2m actual, quite
// repeatable) -- 77.37 * (1.7765m avg / 2m actual) = 68.72.
//
// This is a BAND-AID, not a real gear ratio, and it also feeds
// update_measured_drive_velocity() -- inflating/deflating it doesn't just
// change /odom's displayed distance, it also changes how fast the PI
// drive loop believes the vehicle is going at any given encoder count
// rate, so the real-world speed for a given /velocity command shifts by
// roughly the same factor. If that's a problem, fix the actual EMI
// instead and put this back to 43.46 (the direct hand-turn measurement).
const float DRIVE_GEAR_RATIO = 68.72f; // NOT a real gear ratio -- see comment above

// Drive wheel diameter, measured by hand (tire included).
const float WHEEL_DIAMETER_M = 0.27f;
const float WHEEL_CIRCUMFERENCE_M = PI * WHEEL_DIAMETER_M;

// Flips the sign of the measured velocity if the encoder's count direction
// turns out to be opposite the commanded-positive (forward) direction on
// this vehicle -- see the wiring comment at the top of this file for how to
// check it.
const int8_t DRIVE_ENCODER_SIGN = -1;

// Flips the sign of the PWM actually sent to the drive motors (see
// apply_drive()) -- separate from DRIVE_ENCODER_SIGN above, which only
// corrects the FEEDBACK reading; this corrects the OUTPUT direction.
// Confirmed with ../drive_direction_test: +90 raw PWM (no sign correction
// applied), motor-driven (not pushed by hand) -- the vehicle drove itself
// BACKWARD. So a positive controller output needs to be inverted before
// it reaches the motors for positive /velocity to mean forward -- hence
// -1. (A separate hand-push test, done to check DRIVE_ENCODER_SIGN, showed
// forward motion gives a negative raw encoder count regardless of what
// turned the wheel -- that one's unaffected by this constant and still
// correctly implies DRIVE_ENCODER_SIGN=-1 above.) Re-verify with
// ../drive_direction_test (letting the sketch itself drive the motor, not
// pushing by hand) after any future drive motor/driver/wiring change.
const int8_t DRIVE_MOTOR_SIGN = -1;

// ---- Steering position sensor (see ../steering_sensor_test) ----
const uint8_t STEER_SENSOR_PIN = A0;

// Raw analogRead() values measured at each physical steering lock (5V supply).
// Re-measure with steering_sensor_test.ino if the sensor or its wiring changes.
// Re-measured after the L298N->BTS7960 steering swap (also caught and fixed
// a ~100-count center offset from a first pass at this remeasurement --
// this final pass's center (~520 raw) lines up with the geometric midpoint
// ((18+1018)/2 = 518) within noise, so that's resolved).
const int STEER_RAW_RIGHT = 18;   // -MAX_STEERING_ANGLE
const int STEER_RAW_LEFT = 1018;  // +MAX_STEERING_ANGLE

// P-controller gain: PWM duty per radian of (target - current) error. Tuned by
// trial -- start low and raise it until the steering responds promptly
// without overshooting/oscillating around the target.
// Lowered from 400 -- combined with the BTS7960's added electrical noise on
// the sensor line, 400 was aggressive enough to turn that noise into visible
// trembling even with STEER_AVG_SAMPLES averaging. Raise it again (in small
// steps) if steering now feels sluggish.
const float STEER_KP = 150.0f;

// Ignore errors smaller than this (radians) and stop the steering motor --
// without a deadband, sensor noise makes the P controller hunt back and
// forth forever instead of settling. Widened from 0.015 (~0.86 deg) for the
// same reason as STEER_KP above -- shrink it again if this is now leaving a
// visibly off-center resting angle.
const float STEER_DEADBAND_RAD = 0.03f; // ~1.7 deg

// Minimum PWM magnitude applied whenever outside the deadband, so small
// errors are not commanded at a PWM too low to overcome the motor/gearbox
// static friction (stiction). Tune by trial: raise it if the steering stalls
// on small corrections, lower it if it overshoots/oscillates at the target.
const int16_t STEER_MIN_PWM = 90;

// PI-controller gains for drive velocity: PWM duty per (m/s) of
// (target - measured) error, and per (m/s * s) of accumulated error.
// Starting point only -- tune by trial like STEER_KP above. A P-only
// controller leaves a persistent speed offset (needs the integral term to
// actually reach the commanded speed against motor/gearbox friction), but
// too much DRIVE_KI causes overshoot/oscillation -- raise it just enough to
// close that gap without hunting.
// Lowered from 25.5/2.5 -- still a slight tremble while driving at the
// BTS7960's stronger/faster response, same class of issue as
// STEER_KP/STEER_AVG_SAMPLES above (see read_steering_angle()). Combined
// with the measured_drive_velocity smoothing filter below. Raise again in
// small steps if the drive now feels sluggish to reach target speed.
const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 1.2f;

// Anti-windup clamp on the integral term's contribution to PWM -- without
// this, the integral keeps accumulating while the motor is stalled/at its
// PWM ceiling and then overshoots badly once it's able to move again.
const float DRIVE_INTEGRAL_LIMIT_PWM = 150.0f;

// Minimum PWM magnitude applied only while breaking away from a near-zero
// measured velocity (see apply_drive()) -- overcomes static friction to get
// the vehicle moving, without forcing this floor once already near the
// target speed (that caused oscillation -- see apply_drive()'s comment).
// Tune by trial.
const int16_t DRIVE_MIN_PWM = 60;

// ---- Serial link ----
const unsigned long BAUD_RATE = 115200;

// ---- Command scaling -- keep in sync with hyper_interface parameters.yaml ----
const float MAX_VELOCITY = 1.0f;          // [m/s] -- not read by the PI drive loop
                                           // (see apply_drive()), kept only as a
                                           // doc reference matching hyper_control /
                                           // hyper_interface's parameters.yaml.
const float MAX_STEERING_ANGLE = 0.5061455f; // [rad] (29 deg, measured), matches the sensor calibration above

// ---- Fail-safe ----
// If no valid packet arrives within this long, cut power to both motors
// (fail passive) rather than have the steering P controller keep actively
// holding a stale target with no upstream link to correct it.
const unsigned long COMMAND_TIMEOUT_MS = 300;

// ---- Control loop ----
const unsigned long CONTROL_PERIOD_MS = 20; // 50 Hz, independent of packet arrival

// ---- Telemetry ----
// Periodic "STEER,<target_rad>,<current_rad>\n" and "VEL,<target_mps>,
// <current_mps>\n" lines sent back over the same serial link so
// arduino_interface_node.py can republish the actual steering angle and
// drive velocity to ROS 2 (/steering_angle_actual, /velocity_actual, and
// from there into /odom) -- lets you verify with `ros2 topic echo` that the
// commanded values are actually being reached, without needing the Serial
// Monitor (which cannot be open at the same time as the ROS 2 node anyway).
const unsigned long TELEMETRY_PERIOD_MS = 100; // 10 Hz
unsigned long last_telemetry_ms = 0;

// ---- Drive encoder state ----
// x1 decoding: DRIVE_ENC_A_PIN triggers attachInterrupt() on RISING only;
// DRIVE_ENC_B_PIN is just read (not an interrupt source) inside the handler
// to recover direction. See DRIVE_COUNTS_PER_REV's comment above for why
// (and how this was verified before landing here).
volatile long drive_encoder_count = 0;

// NOTE: reads PIND directly instead of digitalRead(DRIVE_ENC_B_PIN) -- this
// handler should stay as fast as possible since it still fires often (up
// to ~250k/s at this encoder's resolution/gear ratio at higher drive
// speeds -- 1/4 of x4's rate, but still a lot). PIND is a direct register
// read of pins 0-7, and DRIVE_ENC_B_PIN (4, moved from 3 when steering's
// BTS7960 swap needed pin 3 as a PWM output -- see this file's header
// comment) is bit 4 of it on an Uno/Nano (ATmega328P) -- if this pin or the
// board changes, this must be updated to match.
//
// Which raw sign means "forward" doesn't matter here -- DRIVE_ENCODER_SIGN
// corrects for it, and was verified to still hold for this decode scheme
// (see ../encoder_x1_test: forward/backward hand-spins gave the same sign
// convention as x4 did) -- re-verify again after moving B to pin 4, since
// that verification predates the pin move.
void on_drive_encoder_edge() {
  if ((PIND >> 4) & 0x1) {
    drive_encoder_count--;
  } else {
    drive_encoder_count++;
  }
}

// Updated once per control loop tick by update_measured_drive_velocity().
float measured_drive_velocity = 0.0f;
// PI controller's accumulated error, maintained across apply_drive() calls.
float drive_integral = 0.0f;

// ---- Wire protocol ----
const uint8_t SOF1 = 0xAA;
const uint8_t SOF2 = 0x55;
const uint8_t PAYLOAD_LEN = 8;   // 2x float32
const uint8_t PACKET_LEN = 2 + PAYLOAD_LEN + 1; // SOF x2 + payload + checksum

enum RxState { WAIT_SOF1, WAIT_SOF2, READ_PAYLOAD };
RxState rx_state = WAIT_SOF1;
uint8_t rx_buf[PAYLOAD_LEN + 1]; // payload + checksum
uint8_t rx_index = 0;

unsigned long last_packet_ms = 0;
unsigned long last_control_ms = 0;

// Latest targets from ROS 2, applied by the control loop below at its own
// fixed rate rather than directly from process_packet() -- this keeps the
// steering P controller running smoothly even if packets arrive jittery.
float target_velocity = 0.0f;
float target_steering_angle = 0.0f;

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(DRIVE_FRONT_RPWM_PIN, OUTPUT);
  pinMode(DRIVE_FRONT_LPWM_PIN, OUTPUT);
  pinMode(DRIVE_REAR_RPWM_PIN, OUTPUT);
  pinMode(DRIVE_REAR_LPWM_PIN, OUTPUT);
  pinMode(STEER_RPWM_PIN, OUTPUT);
  pinMode(STEER_LPWM_PIN, OUTPUT);

  pinMode(DRIVE_ENC_A_PIN, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A_PIN), on_drive_encoder_edge, RISING);

  stop_all();
  last_packet_ms = millis();

  Serial.println("HYPER_ARDUINO_READY");
}

void loop() {
  while (Serial.available() > 0) {
    handle_byte((uint8_t)Serial.read());
  }

  if (millis() - last_control_ms < CONTROL_PERIOD_MS) {
    return;
  }
  last_control_ms = millis();

  update_measured_drive_velocity();

  if (millis() - last_packet_ms > COMMAND_TIMEOUT_MS) {
    stop_all();
    return;
  }

  apply_drive(target_velocity);
  apply_steering(target_steering_angle);
  send_telemetry();
}

void send_telemetry() {
  if (millis() - last_telemetry_ms < TELEMETRY_PERIOD_MS) {
    return;
  }
  last_telemetry_ms = millis();

  Serial.print("STEER,");
  Serial.print(target_steering_angle, 4);
  Serial.print(",");
  Serial.println(read_steering_angle(), 4);

  Serial.print("VEL,");
  Serial.print(target_velocity, 4);
  Serial.print(",");
  Serial.println(measured_drive_velocity, 4);
}

void handle_byte(uint8_t byte_in) {
  switch (rx_state) {
    case WAIT_SOF1:
      if (byte_in == SOF1) {
        rx_state = WAIT_SOF2;
      }
      break;

    case WAIT_SOF2:
      rx_state = (byte_in == SOF2) ? READ_PAYLOAD : WAIT_SOF1;
      rx_index = 0;
      break;

    case READ_PAYLOAD:
      rx_buf[rx_index++] = byte_in;
      if (rx_index >= sizeof(rx_buf)) {
        process_packet();
        rx_state = WAIT_SOF1;
      }
      break;
  }
}

void process_packet() {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < PAYLOAD_LEN; i++) {
    checksum ^= rx_buf[i];
  }
  if (checksum != rx_buf[PAYLOAD_LEN]) {
    return; // corrupt packet, drop it -- watchdog will stop the motors if this persists
  }

  memcpy(&target_velocity, &rx_buf[0], sizeof(float));
  memcpy(&target_steering_angle, &rx_buf[4], sizeof(float));
  last_packet_ms = millis();
}

// Drives one BTS7960: pwm > 0 -> RPWM=pwm/LPWM=0 (one direction),
// pwm < 0 -> RPWM=0/LPWM=-pwm (the other), pwm == 0 -> both 0 (coast/stop).
// Only one of RPWM/LPWM is ever nonzero at a time -- driving both
// simultaneously is invalid for this driver (shoot-through risk). Shared by
// all three BTS7960 units (drive front, drive rear, steering) -- see
// apply_drive()/apply_steering() for how each calls this with its own pins.
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

// Low-pass filter weight for measured_drive_velocity: each control tick's
// raw instantaneous estimate is blended with the previous filtered value as
// filtered = ALPHA*raw + (1-ALPHA)*filtered. Added alongside the DRIVE_KP/KI
// reduction above to settle the slight tremble while driving -- a noisy
// raw estimate feeding the P term directly turns into PWM ripple. Lower
// ALPHA = smoother but slower to track real speed changes; raise it if the
// drive now feels sluggish to respond.
const float DRIVE_VELOCITY_FILTER_ALPHA = 0.2f;

// Recomputes measured_drive_velocity from the encoder count delta since the
// last call, using the actual elapsed time rather than assuming exactly
// CONTROL_PERIOD_MS -- called once per loop() pass alongside the rest of
// the control loop.
void update_measured_drive_velocity() {
  static unsigned long last_calc_ms = 0;
  static long last_count = 0;

  unsigned long now = millis();
  unsigned long dt_ms = now - last_calc_ms;
  if (dt_ms == 0) {
    return; // called twice within the same millis() tick, nothing to update
  }

  long count;
  noInterrupts();
  count = drive_encoder_count;
  interrupts();

  long delta_count = count - last_count;
  float wheel_rev = (delta_count / (float)DRIVE_COUNTS_PER_REV) / DRIVE_GEAR_RATIO;
  float distance_m = wheel_rev * WHEEL_CIRCUMFERENCE_M;
  float raw_velocity = DRIVE_ENCODER_SIGN * (distance_m / (dt_ms / 1000.0f));
  measured_drive_velocity = DRIVE_VELOCITY_FILTER_ALPHA * raw_velocity
      + (1.0f - DRIVE_VELOCITY_FILTER_ALPHA) * measured_drive_velocity;

  last_count = count;
  last_calc_ms = now;
}

// Closed-loop drive: PI controller drives toward velocity_target using
// measured_drive_velocity from the encoder (see update_measured_drive_velocity()).
void apply_drive(float velocity_target) {
  // No coast-to-stop shortcut here (an earlier version had one: go to PWM 0
  // once both velocity_target and measured_drive_velocity were near zero).
  // That let the vehicle roll freely under gravity/a push whenever
  // commanded to stop -- zero PWM means zero holding force, so anything
  // (a slope, a shove) could move it with nothing pushing back. Instead,
  // velocity_target=0 is treated like any other target: the PI loop below
  // (plus the DRIVE_MIN_PWM floor once error is large enough) keeps
  // actively correcting toward it, so a push away from a standstill gets
  // resisted immediately rather than only once it drifts past a threshold.
  float error = velocity_target - measured_drive_velocity;

  // Reset the integral immediately if it's now pointing the WRONG way (its
  // accumulated sign opposes the current error) instead of letting it
  // unwind naturally. A long sustained drive (e.g. 15+ seconds forward)
  // winds the integral up toward its clamp (DRIVE_INTEGRAL_LIMIT_PWM); once
  // the target then drops to 0/reverses, natural unwinding (only
  // drive_integral += error*dt per tick) is far too slow -- at the clamp,
  // fully discharging it can take on the order of 100+ seconds, during
  // which the stale integral's contribution (up to +-DRIVE_INTEGRAL_LIMIT_PWM,
  // 150 here -- more than STEER_MIN_PWM-scale forces) can outweigh the
  // fresh (now negative) error entirely, so the vehicle keeps driving
  // forward on old momentum regardless of the new target. Symptom: stop
  // command does nothing for a long stretch after a sustained drive.
  if ((drive_integral > 0.0f && error < 0.0f) || (drive_integral < 0.0f && error > 0.0f)) {
    drive_integral = 0.0f;
  }
  drive_integral = constrain(
      drive_integral + error * (CONTROL_PERIOD_MS / 1000.0f),
      -DRIVE_INTEGRAL_LIMIT_PWM / DRIVE_KI, DRIVE_INTEGRAL_LIMIT_PWM / DRIVE_KI);

  float output = DRIVE_KP * error + DRIVE_KI * drive_integral;
  int16_t pwm = (int16_t)constrain(output, -255.0f, 255.0f);

  // Only force the PWM floor while there's a substantial error to close --
  // starting from rest (needs to overcome static friction) or braking hard
  // from speed toward a much lower target (needs real reverse force, not
  // just a weak few-PWM nudge) -- NOT unconditionally on every nonzero
  // error like STEER_MIN_PWM does for steering. Steering's floor is safe
  // because it only applies outside STEER_DEADBAND_RAD (i.e. only far from
  // the target); this loop has no such deadband once velocity_target itself
  // is nonzero, so forcing +-DRIVE_MIN_PWM on every small residual error
  // made it snap-overshoot-snap back and forth once near the target speed
  // (visible as trembling/oscillation) instead of settling smoothly.
  //
  // Gating on |measured_drive_velocity| alone (as an earlier version of
  // this did) missed the braking case: releasing the throttle from speed
  // gives a large negative error while measured_drive_velocity is still
  // large too, so that check never applied the floor and braking was left
  // to a PWM in the single digits/tens -- barely more than rolling
  // resistance, so the vehicle coasted for a long time instead of actually
  // slowing down. Gating on |error| instead covers both cases the same way.
  //
  // But |error| alone missed a different case: commanding a LOW target
  // (e.g. 0.15 m/s) from a standstill starts with error == that target, so
  // for any target at or below this threshold the floor never engages and
  // the raw KP*error (a couple PWM at low targets) never overcomes static
  // friction -- the vehicle just sits there. OR in the same breakaway
  // check an earlier version used (measured velocity near zero while a
  // nonzero target is commanded) so low-target starts still get a kick.
  bool large_error = fabs(error) > 0.15f;
  bool breaking_away = fabs(measured_drive_velocity) < 0.05f && fabs(velocity_target) > 0.01f;
  bool needs_floor = large_error || breaking_away;
  if (needs_floor) {
    if (pwm > 0 && pwm < DRIVE_MIN_PWM) {
      pwm = DRIVE_MIN_PWM;
    } else if (pwm < 0 && pwm > -DRIVE_MIN_PWM) {
      pwm = -DRIVE_MIN_PWM;
    }
  }

  // Both drive motors get the identical PWM -- see this file's header
  // comment for why there's no independent front/rear feedback.
  // DRIVE_MOTOR_SIGN applied here (not earlier) so drive_integral/pwm still
  // reflect the controller's own view of the error -- only the final motor
  // command direction is corrected.
  int16_t motor_pwm = (int16_t)(DRIVE_MOTOR_SIGN * pwm);
  set_bts7960(DRIVE_FRONT_RPWM_PIN, DRIVE_FRONT_LPWM_PIN, motor_pwm);
  set_bts7960(DRIVE_REAR_RPWM_PIN, DRIVE_REAR_LPWM_PIN, motor_pwm);
}

// Number of analogRead() samples averaged together in read_steering_angle().
// Added after the BTS7960 swap started injecting enough electrical noise
// into the steering sensor line to make the P controller hunt around the
// deadband. Averaging costs a little latency (STEER_AVG_SAMPLES x ~100us,
// negligible next to the 20ms control period) for a much steadier reading.
const uint8_t STEER_AVG_SAMPLES = 8;

// Converts a raw analogRead() from the steering position sensor into an
// angle using the two lock-to-lock calibration points, clamped in case
// sensor noise or slight over-travel pushes it past either raw endpoint.
float read_steering_angle() {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < STEER_AVG_SAMPLES; i++) {
    sum += analogRead(STEER_SENSOR_PIN);
  }
  float raw = (float)sum / STEER_AVG_SAMPLES;

  float t = (raw - STEER_RAW_RIGHT) / (float)(STEER_RAW_LEFT - STEER_RAW_RIGHT);
  float angle = -MAX_STEERING_ANGLE + t * (2.0f * MAX_STEERING_ANGLE);
  return constrain(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
}

// Closed-loop steering: P controller drives toward steering_angle_target
// using the position sensor, stopping once within STEER_DEADBAND_RAD.
void apply_steering(float steering_angle_target) {
  float current_angle = read_steering_angle();
  float error = steering_angle_target - current_angle;

  int16_t pwm;
  if (fabs(error) < STEER_DEADBAND_RAD) {
    pwm = 0;
  } else {
    pwm = (int16_t)constrain(STEER_KP * error, -255.0f, 255.0f);
    if (pwm > 0 && pwm < STEER_MIN_PWM) {
      pwm = STEER_MIN_PWM;
    } else if (pwm < 0 && pwm > -STEER_MIN_PWM) {
      pwm = -STEER_MIN_PWM;
    }
  }

  set_bts7960(STEER_RPWM_PIN, STEER_LPWM_PIN, pwm);
}

void stop_all() {
  drive_integral = 0.0f; // don't carry windup across a stop into the next command
  set_bts7960(DRIVE_FRONT_RPWM_PIN, DRIVE_FRONT_LPWM_PIN, 0);
  set_bts7960(DRIVE_REAR_RPWM_PIN, DRIVE_REAR_LPWM_PIN, 0);
  set_bts7960(STEER_RPWM_PIN, STEER_LPWM_PIN, 0);
}
