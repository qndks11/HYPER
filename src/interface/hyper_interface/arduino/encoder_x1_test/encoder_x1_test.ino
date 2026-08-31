// encoder_x1_test.ino
//
// Standalone bring-up sketch to verify x1 quadrature decoding (only A's
// rising edge counted, direction recovered from B) BEFORE it gets ported
// into hyper_motor_interface.ino. No ROS 2, no motor driver involved.
//
// Why this exists: hyper_motor_interface.ino currently uses x4 decoding
// (both A and B, every edge), which is CPU-expensive enough at this
// encoder's resolution x gear ratio that at higher drive speeds the
// interrupt rate saturates the CPU and stalls loop() -- symptom: steering
// stops responding once driving fast enough. x1 (only A's rising edge)
// cuts the interrupt rate to 1/4, which should fix that. An earlier
// attempt at x1 (combined with a raw ISR(INT0_vect)/EICRA/EIMSK register
// setup instead of attachInterrupt(), to also shave off attachInterrupt's
// call overhead) left drive_encoder_count stuck at 0 permanently -- never
// diagnosed exactly why, and combining two changes at once (decode scheme
// AND interrupt mechanism) made it hard to tell which one broke. This
// sketch isolates decode scheme as the only variable: attachInterrupt()
// is the same proven-working mechanism as x4 uses today, just with RISING
// instead of CHANGE and only on pin A.
//
// SAFETY: no motor driver code here, nothing moves on its own.
//
// How to use:
//   1. Upload, open Serial Monitor (or `arduino-cli monitor`) at 115200.
//   2. Note the printed count (should be changing if the wheel/shaft is
//      already moving, otherwise steady).
//   3. By hand, slowly turn the encoder (drive wheel, or motor shaft
//      directly, whichever is easier to access) one way, then the other.
//      Confirm count increases in one direction and decreases in the
//      other, and that it changes AT ALL (the earlier x1 attempt's bug was
//      it never counted anything).
//   4. Spin several turns and sanity-check the count roughly matches what
//      x4 would give divided by 4 (compare against ../gear_ratio_test if
//      unsure) -- confirms no double-counting or missed edges.
//
// Only once this checks out should on_drive_encoder_edge()'s logic here be
// ported into hyper_motor_interface.ino (replacing its x4 ISR + both
// attachInterrupt() calls with this single-pin RISING version), together
// with DRIVE_COUNTS_PER_REV there dropping from x4 to x1 (PULSES_PER_REV,
// not x4).

#include <avr/io.h> // PIND -- direct port read in on_drive_encoder_edge()

// ---- Pins -- must match hyper_motor_interface.ino's drive encoder wiring ----
const uint8_t DRIVE_ENC_A_PIN = 2;
const uint8_t DRIVE_ENC_B_PIN = 3;

const unsigned long PRINT_PERIOD_MS = 200; // 5 Hz -- plenty for a hand-spin test

volatile long drive_encoder_count = 0;

// x1 decoding: A just rose. If B is currently low, A leads B -> one
// direction; if B is high, B leads A -> the other. Which raw sign means
// "forward" doesn't matter for this test -- just confirm it changes AT ALL
// and that the two hand-spin directions give opposite signs.
void on_drive_encoder_edge() {
  if ((PIND >> 3) & 0x1) {
    drive_encoder_count--;
  } else {
    drive_encoder_count++;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(DRIVE_ENC_A_PIN, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A_PIN), on_drive_encoder_edge, RISING);

  Serial.println("ENCODER_X1_TEST_READY");
  Serial.println("Spin the encoder by hand both ways and watch count change below.");
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
