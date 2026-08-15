#ifndef HYPER_CAMERA__ELP_CAMERA_PUBLISHER_NODE_HPP_
#define HYPER_CAMERA__ELP_CAMERA_PUBLISHER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hyper_camera/elp_camera_capture.hpp"

namespace hyper_camera
{

/**
 * @brief Owns the front ELP camera and publishes each rectified frame as a plain
 * sensor_msgs/Image on "image_raw". Replaces what used to be hyper_lane_detection's own
 * input_backend:=direct_usb capture loop -- pulling capture out into its own node lets it be
 * loaded into the same ComposableNodeContainer as LaneDetection, so
 * rclcpp::Publisher::publish(std::move(unique_ptr)) hands the frame to LaneDetection's
 * subscription by pointer instead of serializing it over a topic (input_backend:=intra_process;
 * see hyper_lane_detection/lane_detection_node.cpp and hyper_object_detection's
 * perception.launch.py for the container that loads both).
 */
class ElpCameraPublisherNode : public rclcpp::Node
{
public:
  explicit ElpCameraPublisherNode(const rclcpp::NodeOptions & options);

private:
  void capture_timer_callback();

  std::unique_ptr<ElpCameraCapture> elp_capture_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  std::string frame_id_;
};

}  // namespace hyper_camera

#endif  // HYPER_CAMERA__ELP_CAMERA_PUBLISHER_NODE_HPP_
