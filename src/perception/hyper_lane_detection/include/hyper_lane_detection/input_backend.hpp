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
  /// Real vehicle: plain sensor_msgs/Image subscription, same as kRosRaw, but fed by
  /// hyper_camera's ElpCameraPublisherNode component loaded into the same
  /// ComposableNodeContainer as this node -- rclcpp's intra-process manager hands the frame
  /// straight to the subscription callback instead of serializing it over the topic. Selects the
  /// real-ELP RoiConfig (see lane_detection_node.cpp).
  kIntraProcess,
  /// Gazebo simulation: plain sensor_msgs/Image subscription on the raw topic, fed by
  /// ros_gz_bridge. Selects the sim-camera RoiConfig.
  kRosRaw,
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
