// steering_sensor_test.ino
//
// Standalone bring-up sketch to characterize the steering angle sensor
// (assumed to be a 3-wire analog potentiometer-style sensor: power, GND,
// signal). No ROS 2, no motor driver involved -- this only reads the
// sensor and prints raw ADC + voltage + a running min/max to the Serial
// Monitor, so the sensor full-scale range can be found by hand before it
// gets wired into a real feedback control loop.
//
// Wiring:
//   sensor power  -> Arduino 5V
//   sensor GND    -> Arduino GND
//   sensor signal -> Arduino A0
// If the sensor is rated for 3.3V logic instead of 5V, power it from 3.3V
// instead and do not feed a 5V signal back into an Uno's A0 -- the Uno's
// ADC reference below assumes a 5V signal range.
//
// How to use:
//   1. Wire as above, upload this sketch, open Serial Monitor at 115200 baud.
//   2. By hand (motor unpowered), slowly turn the steering through its full
//      mechanical range, lock to lock.
//   3. Watch the printed min/max -- those are the raw ADC values at full
//      lock each direction. Write them down; they are what
//      hyper_motor_interface.ino (once feedback control is added) will
//      need to convert a raw reading into an actual angle.

const uint8_t SENSOR_PIN = A0;
const unsigned long PRINT_PERIOD_MS = 100; // 10 Hz

int min_reading = 1023;
int max_reading = 0;
unsigned long last_print_ms = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("STEERING_SENSOR_TEST_READY");
  Serial.println("raw_adc, voltage_v, min_seen, max_seen");
}

void loop() {
  int raw = analogRead(SENSOR_PIN);
  if (raw < min_reading) {
    min_reading = raw;
  }
  if (raw > max_reading) {
    max_reading = raw;
  }

  if (millis() - last_print_ms >= PRINT_PERIOD_MS) {
    last_print_ms = millis();
    float voltage = raw / 1023.0f * 5.0f;
    Serial.print(raw);
    Serial.print(", ");
    Serial.print(voltage, 3);
    Serial.print(", ");
    Serial.print(min_reading);
    Serial.print(", ");
    Serial.println(max_reading);
  }
}
