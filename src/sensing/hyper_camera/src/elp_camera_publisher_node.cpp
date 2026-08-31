#include "hyper_camera/elp_camera_publisher_node.hpp"

#include <chrono>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace hyper_camera
{

ElpCameraPublisherNode::ElpCameraPublisherNode(const rclcpp::NodeOptions & options)
: Node("elp_camera_publisher", options)
{
  ElpCameraCapture::Config config;
  config.device = declare_parameter<std::string>("video_device", "/dev/video_elp");
  // 640x360, not the sensor's 1280x720: half the linear resolution is a quarter of the pixels
  // to pull over USB, MJPEG-decode and remap every frame, which is the single biggest lever on
  // this camera's power draw. Kept at the calibration's 16:9 so ElpCameraCapture can rescale the
  // intrinsics rather than crop the field of view (see elp_camera_capture.cpp). Raising this
  // means rescaling hyper_lane_detection's bev_real.yaml intrinsics to match.
  config.width = declare_parameter<int>("image_width", 640);
  config.height = declare_parameter<int>("image_height", 360);
  config.framerate = declare_parameter<double>("framerate", 30.0);
  config.calibration_file = declare_parameter<std::string>(
    "calibration_file",
    ament_index_cpp::get_package_share_directory("hyper_camera") +
      "/config/ELP-USBGS1200P01-KL170.yaml");
  frame_id_ = declare_parameter<std::string>("frame_id", "camera");

  elp_capture_ = std::make_unique<ElpCameraCapture>(get_logger());
  if (!elp_capture_->open(config)) {
    // ElpCameraCapture::open() has already logged the specific failure; a node that "starts"
    // with no working camera would otherwise spin forever silently publishing nothing.
    throw std::runtime_error("elp_camera_publisher: camera setup failed");
  }

  // Depth 10, matching LaneDetection's subscription QoS on the other end -- this topic is meant
  // to be loaded into the same container as LaneDetection (intra-process), not used stand-alone.
  image_publisher_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

  const auto period_s = std::chrono::duration<double>(1.0 / config.framerate);
  capture_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period_s),
    std::bind(&ElpCameraPublisherNode::capture_timer_callback, this));

  RCLCPP_INFO(
    get_logger(), "elp_camera_publisher: publishing rectified frames on '%s'",
    image_publisher_->get_topic_name());
}

void ElpCameraPublisherNode::capture_timer_callback()
{
  cv::Mat rectified;
  if (!elp_capture_->read(rectified)) {
    // Read failure is already logged inside ElpCameraCapture::read(); skip this tick rather than
    // aborting the node over what may be a transient USB glitch.
    return;
  }

  // Published by unique_ptr -- when a subscriber sits in the same intra-process container (the
  // usual case for this node, see the class doc comment), rclcpp's intra-process manager hands
  // this pointer straight to the subscription's callback instead of copying/serializing it.
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = now();
  cv_image.header.frame_id = frame_id_;
  cv_image.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image.image = rectified;
  cv_image.toImageMsg(*msg);

  image_publisher_->publish(std::move(msg));
}

}  // namespace hyper_camera

RCLCPP_COMPONENTS_REGISTER_NODE(hyper_camera::ElpCameraPublisherNode)
