#ifndef HYPER_LANE_DETECTION__INPUT_BACKEND_HPP_
#define HYPER_LANE_DETECTION__INPUT_BACKEND_HPP_

#include <optional>
#include <string>

namespace hyper_lane_detection
{

/**
 * @brief Selects where lane_detection_node gets its camera frames from, set via the
 * "input_backend" ROS parameter.
 */
enum class InputBackend
{
  /// Real vehicle: lane_detection_node opens the physical ELP USB camera device itself
  /// (see ElpCameraCapture) and never touches an intermediate ROS image topic.
  kDirectUsb,
  /// Gazebo simulation: plain sensor_msgs/Image subscription on the raw topic, no
  /// image_transport, no rectification (the simulated camera output is already the expected
  /// input).
  kRosRaw,
  /// Legacy real-car path, kept temporarily for A/B comparison and rollback: subscribes over
  /// image_transport's "compressed" transport, same as before this refactor.
  kRosCompressed,
};

/**
 * @brief Parses the "input_backend" parameter string into its enum value.
 * @return The parsed backend, or std::nullopt if `value` doesn't match any known backend.
 */
std::optional<InputBackend> parse_input_backend(const std::string & value);

/// Human-readable name for logging -- the inverse of parse_input_backend().
std::string to_string(InputBackend backend);

}  // namespace hyper_lane_detection

#endif  // HYPER_LANE_DETECTION__INPUT_BACKEND_HPP_
