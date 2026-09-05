#ifndef HYPER_CAMERA__LOGITECH_CAMERA_PUBLISHER_NODE_HPP_
#define HYPER_CAMERA__LOGITECH_CAMERA_PUBLISHER_NODE_HPP_

#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hyper_camera
{

/**
 * @brief Owns the vehicle's one physical camera -- the Logitech C920 -- and publishes each frame
 * as a plain sensor_msgs/Image on "image_raw".
 *
 * @details This is the single camera source for the whole perception stage: lane_detection warps
 * it into a bird's-eye view and object_detection runs YOLO on it. It replaced a pair of cameras
 * (the front ELP fisheye for lanes, this C920 for objects), which is why it is a C++ rclcpp
 * component rather than the rclpy node it started life as: lane_detection loads it into its own
 * ComposableNodeContainer, so publish(std::move(unique_ptr)) hands the frame to that
 * subscription by pointer instead of serializing it (input_backend:=intra_process). The same
 * publish still goes out over DDS for object_detection_node, which is a separate rclpy process
 * and has no zero-copy path available to it either way.
 *
 * No rectification happens here, unlike the ELP node this replaces: the C920 is a normal ~70 deg
 * lens with no meaningful fisheye distortion and no calibration file in this repo, so
 * hyper_lane_detection's BEV homography is built from a plain pinhole model of it (see
 * hyper_lane_detection/config/bev_real.yaml).
 */
class LogitechCameraPublisherNode : public rclcpp::Node
{
public:
  explicit LogitechCameraPublisherNode(const rclcpp::NodeOptions & options);

private:
  void capture_timer_callback();

  cv::VideoCapture capture_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  std::string frame_id_;
};

}  // namespace hyper_camera

#endif  // HYPER_CAMERA__LOGITECH_CAMERA_PUBLISHER_NODE_HPP_
