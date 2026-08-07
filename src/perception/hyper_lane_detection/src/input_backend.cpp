#include "hyper_lane_detection/input_backend.hpp"

namespace hyper_lane_detection
{

std::optional<InputBackend> parse_input_backend(const std::string & value)
{
  if (value == "direct_usb") {
    return InputBackend::kDirectUsb;
  }
  if (value == "ros_raw") {
    return InputBackend::kRosRaw;
  }
  return std::nullopt;
}

std::string to_string(InputBackend backend)
{
  switch (backend) {
    case InputBackend::kDirectUsb:
      return "direct_usb";
    case InputBackend::kRosRaw:
      return "ros_raw";
  }
  return "unknown";
}

}  // namespace hyper_lane_detection
