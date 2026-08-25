// magnetic_encoder_test.ino
//
// Standalone bring-up sketch for an AS5047P-based magnetic rotary encoder
// module wired in ABI (quadrature) mode -- specifically tested against an
// MKS AS5047P-TS_EK_AB breakout, but works with any ABI-mode incremental
// output. No ROS 2, no motor driver involved -- this only decodes A/B/I
// pulses and prints running count, angle, and RPM to the Serial Monitor,
// so the encoder can be verified (wiring, direction sense, and pulses-
// per-revolution) by hand before it gets wired into a real closed-loop
// feedback path (e.g. wheel odometry).
//
// This board is a bare sensor breakout (chip + header only) -- it has NO
// power/status LED, so the absence of any light on the board is normal
// and not itself a sign of failure.
//
// Wiring (Arduino Uno/Nano), MKS AS5047P-TS_EK_AB ABI header:
//   board A  -> Arduino pin 2 (INT0, must be interrupt-capable)
//   board B  -> Arduino pin 3 (INT1, must be interrupt-capable)
//   board I  -> Arduino pin 4 (index/Z pulse, once per revolution; polled,
//               does not need to be interrupt-capable)
//   board 5V -> Arduino 5V
//   board GND -> Arduino GND
// IMPORTANT: this board has a JP1 jumper selecting the ABI output logic
// level, 5V or 3V3. Set JP1 to 5V when wiring directly into a 5V-logic
// Uno/Nano -- left on 3V3, the Uno may not read the signal as a reliable
// HIGH and count will appear stuck at 0. This is the first thing to check
// if count never moves.
// On boards other than Uno/Nano the interrupt-capable pins for A/B differ
// (Mega: 2, 3, 18, 19, 20, 21; 32u4 boards: 0, 1, 2, 3, 7) -- update
// ENCODER_A_PIN / ENCODER_B_PIN to match; ENCODER_I_PIN can be any free
// digital pin since it is only polled.
//
// Decoding: A/B are decoded with full x4 quadrature decoding (every A/B
// edge, via a standard state-transition lookup table). PULSES_PER_REV
// below is only used to compute angle_deg/rpm -- if it is wrong, count
// itself is still accurate, only the derived angle/rpm will be off. The
// index pulse (I) measures the actual pulses-per-revolution directly
// (see step 3 below), so it does not need to be guessed from a datasheet.
//
// How to use:
//   1. Wire as above (JP1 set to 5V), upload this sketch, open Serial
//      Monitor at 115200 baud.
//   2. By hand, slowly turn the encoder shaft one way, then the other.
//      Confirm count increases in one direction and decreases in the
//      other -- if the sense is backwards for your application, swap the
//      A/B wires.
//   3. Keep turning the shaft in one direction past a full revolution.
//      Each time it passes the index position, an "INDEX ..." line is
//      printed showing counts_since_last_index and the pulses-per-rev it
//      implies (counts_since_last_index / 4). That measured value is the
//      real PPR of this module/config -- update PULSES_PER_REV with it so
//      angle_deg/rpm are accurate.
//   4. Spin the shaft quickly and confirm count keeps up smoothly (no
//      jumps or stalls) -- jumps usually mean a wiring/contact problem,
//      or JP1 set to the wrong logic level for your board.

#include <avr/io.h> // PIND -- direct port read in on_encoder_edge()

const uint8_t ENCODER_A_PIN = 2;
const uint8_t ENCODER_B_PIN = 3;
const uint8_t ENCODER_I_PIN = 4;

// Encoder's pulses-per-revolution. Only used to compute angle_deg/rpm --
// measured via the index pulse (see step 3 above): consistently -4000
// counts / 4 = 1000 PPR across many revolutions on this unit.
const long PULSES_PER_REV = 1000;
const long COUNTS_PER_REV = PULSES_PER_REV * 4; // x4 decoding

const unsigned long PRINT_PERIOD_MS = 100; // 10 Hz

volatile long encoder_count = 0;
volatile uint8_t last_ab_state = 0; // bit1 = A, bit0 = B

// Quadrature state-transition table: index = (previous_state << 2) |
// current_state, value = direction (+1 / -1) or 0 for an invalid/repeated
// transition (bounce or a missed edge).
const int8_t QUADRATURE_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// NOTE: reads PIND directly instead of digitalRead(ENCODER_A_PIN) /
// digitalRead(ENCODER_B_PIN) -- this ISR needs to be fast since it fires on
// every A/B edge (tens of thousands of times/sec at speed); digitalRead()'s
// pin->port lookup is slow enough that at higher RPM the ISRs can saturate
// the CPU and stall loop() (and therefore millis()/Serial output). PIND is a
// direct register read of pins 0-7, and ENCODER_A_PIN/ENCODER_B_PIN (2, 3)
// are bits 2/3 of it on an Uno/Nano (ATmega328P) -- if these pins or the
// board change, update this to match.
void on_encoder_edge() {
  uint8_t port_d = PIND;
  uint8_t a = (port_d >> 2) & 0x1;
  uint8_t b = (port_d >> 3) & 0x1;
  uint8_t current_state = (a << 1) | b;
  uint8_t index = (last_ab_state << 2) | current_state;
  encoder_count += QUADRATURE_TABLE[index];
  last_ab_state = current_state;
}

unsigned long last_print_ms = 0;
long last_count_for_rpm = 0;

// Index (Z) pulse tracking -- polled in loop() since Uno only has two
// hardware interrupt pins, both already used by A/B.
uint8_t last_i_state = LOW;
long count_at_last_index = 0;
bool have_last_index = false;

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_I_PIN, INPUT_PULLUP);

  last_ab_state = (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);
  last_i_state = digitalRead(ENCODER_I_PIN);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), on_encoder_edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), on_encoder_edge, CHANGE);

  Serial.println("MAGNETIC_ENCODER_TEST_READY");
  Serial.println("count, angle_deg, rpm");
}

void check_index_pulse() {
  uint8_t i_state = digitalRead(ENCODER_I_PIN);
  if (i_state == HIGH && last_i_state == LOW) {
    long count;
    noInterrupts();
    count = encoder_count;
    interrupts();

    if (have_last_index) {
      long counts_since_last_index = count - count_at_last_index;
      Serial.print("INDEX, counts_since_last_index=");
      Serial.print(counts_since_last_index);
      Serial.print(", implied_pulses_per_rev=");
      Serial.println(counts_since_last_index / 4.0f, 1);
    }
    count_at_last_index = count;
    have_last_index = true;
  }
  last_i_state = i_state;
}

void loop() {
  check_index_pulse();

  unsigned long now = millis();
  if (now - last_print_ms < PRINT_PERIOD_MS) {
    return;
  }

  long count;
  noInterrupts();
  count = encoder_count;
  interrupts();

  long delta_count = count - last_count_for_rpm;
  float dt_sec = (now - last_print_ms) / 1000.0f;

  // Wrap into [0, COUNTS_PER_REV) before converting to degrees; C's %
  // can return negative for a negative count, so normalize first.
  long wrapped_count = count % COUNTS_PER_REV;
  if (wrapped_count < 0) {
    wrapped_count += COUNTS_PER_REV;
  }
  float angle_deg = (float)wrapped_count * 360.0f / COUNTS_PER_REV;
  float rpm = (dt_sec > 0) ? (delta_count / (float)COUNTS_PER_REV) / (dt_sec / 60.0f) : 0.0f;

  Serial.print(count);
  Serial.print(", ");
  Serial.print(angle_deg, 2);
  Serial.print(", ");
  Serial.println(rpm, 2);

  last_count_for_rpm = count;
  last_print_ms = now;
}
