#ifndef HYPER_CONTROL__STM32_PROTOCOL_HPP_
#define HYPER_CONTROL__STM32_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace hyper_control
{

// Wire format sent from Stm32SystemInterface::write() to the STM32 board, once per control
// loop cycle. Fields are native little-endian (both the host and the STM32 Cortex-M core are
// little-endian, so floats are copied byte-for-byte with no re-encoding).
#pragma pack(push, 1)
struct Stm32CommandPacket
{
  uint8_t header{0xAA};
  float steering_angle{0.0F};       // [rad], positive = left, matches the "position" command interface
  float rear_wheel_velocity{0.0F};  // [rad/s], matches the "velocity" command interface
  uint8_t crc{0};                   // CRC-8/SMBUS (poly 0x07, init 0x00) over steering_angle and rear_wheel_velocity
};
#pragma pack(pop)

inline uint8_t compute_crc8(const uint8_t * data, std::size_t length)
{
  uint8_t crc = 0x00;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

}  // namespace hyper_control

#endif  // HYPER_CONTROL__STM32_PROTOCOL_HPP_
