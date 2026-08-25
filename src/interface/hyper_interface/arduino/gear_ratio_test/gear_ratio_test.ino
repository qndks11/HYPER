// gear_ratio_test.ino
//
// Standalone bring-up sketch to re-measure DRIVE_GEAR_RATIO in
// hyper_motor_interface.ino. No ROS 2, no motor driver involved -- this
// only decodes the drive encoder (A/B, same x4 quadrature scheme as
// hyper_motor_interface.ino and ../magnetic_encoder_test) and prints the
// running raw count, so the drive wheel can be spun by hand a known number
// of turns and the resulting count delta read off directly, instead of
// trying to read it through hyper_motor_interface.ino's normal telemetry
// (which reports derived velocity, not the raw count).
//
// SAFETY: no motor driver code here at all, so there is no need to lift
// the wheels -- nothing will move on its own. Wiring matches
// hyper_motor_interface.ino's encoder pins exactly (see that file's wiring
// comment for the full hookup) -- only A/B/5V/GND need to be connected for
// this test.
//
// How to use:
//   1. Upload this sketch, open Serial Monitor (or `arduino-cli monitor`)
//      at 115200 baud.
//   2. Note the printed count.
//   3. By hand, spin the DRIVE WHEEL (not the motor shaft directly) a
//      known number of full turns -- 10 is a good default, more turns
//      means less relative error from any turn-counting mistake. Turning
//      the wheel through the drivetrain's gear reduction is the whole
//      point -- that's what DRIVE_GEAR_RATIO is measuring.
//   4. Note the printed count again.
//   5. Compute:
//        count_delta = count_after - count_before
//        DRIVE_GEAR_RATIO = abs(count_delta) / (COUNTS_PER_REV * turns)
//      where COUNTS_PER_REV below matches
//      hyper_motor_interface.ino's DRIVE_COUNTS_PER_REV (x4 decoding,
//      1000 PPR x 4 = 4000) -- keep the two in sync if either changes.
//   6. Update DRIVE_GEAR_RATIO in hyper_motor_interface.ino with the
//      result.

#include <avr/io.h> // PIND -- direct port read in on_drive_encoder_edge()

// ---- Pins -- must match hyper_motor_interface.ino's drive encoder wiring ----
const uint8_t DRIVE_ENC_A_PIN = 2;
const uint8_t DRIVE_ENC_B_PIN = 3;

// Must match hyper_motor_interface.ino's DRIVE_COUNTS_PER_REV.
const long PULSES_PER_REV = 1000;
const long COUNTS_PER_REV = PULSES_PER_REV * 4; // x4 decoding

const unsigned long PRINT_PERIOD_MS = 200; // 5 Hz -- plenty for a hand-spin test

volatile long drive_encoder_count = 0;
volatile uint8_t drive_last_ab_state = 0; // bit1 = A, bit0 = B

const int8_t QUADRATURE_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// See hyper_motor_interface.ino's on_drive_encoder_edge() for why this
// reads PIND directly instead of digitalRead().
void on_drive_encoder_edge() {
  uint8_t port_d = PIND;
  uint8_t a = (port_d >> 2) & 0x1;
  uint8_t b = (port_d >> 3) & 0x1;
  uint8_t current_state = (a << 1) | b;
  uint8_t index = (drive_last_ab_state << 2) | current_state;
  drive_encoder_count += QUADRATURE_TABLE[index];
  drive_last_ab_state = current_state;
}

void setup() {
  Serial.begin(115200);

  pinMode(DRIVE_ENC_A_PIN, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B_PIN, INPUT_PULLUP);
  drive_last_ab_state = (digitalRead(DRIVE_ENC_A_PIN) << 1) | digitalRead(DRIVE_ENC_B_PIN);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A_PIN), on_drive_encoder_edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B_PIN), on_drive_encoder_edge, CHANGE);

  Serial.println("GEAR_RATIO_TEST_READY");
  Serial.println("Note the count below, spin the drive wheel N known turns by hand,");
  Serial.println("then note the count again -- see the notes at the top of this file");
  Serial.println("for how to turn that into DRIVE_GEAR_RATIO.");
}

void loop() {
  static unsigned long last_print_ms = 0;
  unsigned long now = millis();
  if (now - last_print_ms < PRINT_PERIOD_MS) {
    return;
  }
  last_print_ms = now;

  long count;
  noInterrupts();
  count = drive_encoder_count;
  interrupts();

  Serial.print("count=");
  Serial.println(count);
}
