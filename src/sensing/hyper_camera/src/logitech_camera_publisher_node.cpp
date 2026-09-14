#include "hyper_camera/logitech_camera_publisher_node.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
// How often to repeat a capture failure [ms]. An unplugged camera fails on every tick, and at 30
// fps that would bury the rest of the log within seconds.
constexpr int kReadErrorThrottleMs = 5000;
}  // namespace

namespace hyper_camera
{

LogitechCameraPublisherNode::LogitechCameraPublisherNode(const rclcpp::NodeOptions & options)
: Node("logitech_camera_publisher", options)
{
  const auto device = declare_parameter<std::string>("video_device", "/dev/video_logitech");
  // 640x360, not the C920's 1280x720: half the linear resolution is a quarter of the pixels to
  // pull over USB and MJPEG-decode every frame, which is the single biggest lever on this
  // camera's power draw. object_detection_node letterboxes into the YOLO model's own input size
  // anyway, so the extra pixels buy nothing downstream.
  //
  // !! The BEV geometry in hyper_lane_detection/config/bev_real.yaml derives its focal length
  // from this width and the lens's horizontal FOV, and its near edge from this height. Changing
  // either here without revisiting that file silently mis-scales every distance the BEV reports.
  const int width = static_cast<int>(declare_parameter<int>("image_width", 640));
  const int height = static_cast<int>(declare_parameter<int>("image_height", 360));
  const auto framerate = declare_parameter<double>("framerate", 30.0);
  frame_id_ = declare_parameter<std::string>("frame_id", "camera");

  if (!capture_.open(device, cv::CAP_V4L2)) {
    // A node that "starts" with no working camera would otherwise spin forever publishing
    // nothing, and the whole perception stage would come up looking merely idle.
    RCLCPP_FATAL(get_logger(), "Failed to open camera device '%s'", device.c_str());
    throw std::runtime_error(
      "logitech_camera_publisher: failed to open camera device '" + device + "'");
  }

  // MJPEG in, decoded to BGR8 out -- OpenCV's V4L2 backend does the JPEG decode internally on
  // read() (CAP_PROP_CONVERT_RGB stays at its default true).
  capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  capture_.set(cv::CAP_PROP_FPS, framerate);

  // Depth 10, matching LaneDetection's and object_detection_node's subscription QoS on the other
  // end.
  image_publisher_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

  const auto period_s = std::chrono::duration<double>(1.0 / framerate);
  capture_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period_s),
    std::bind(&LogitechCameraPublisherNode::capture_timer_callback, this));

  RCLCPP_INFO(
    get_logger(), "logitech_camera_publisher: opened '%s' (%dx%d @ %.0f fps), publishing on '%s'",
    device.c_str(), width, height, framerate, image_publisher_->get_topic_name());
}

void LogitechCameraPublisherNode::capture_timer_callback()
{
  cv::Mat frame;
  if (!capture_.read(frame) || frame.empty()) {
    // Skip this tick rather than aborting the node over what may be a transient USB glitch.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kReadErrorThrottleMs,
      "Camera stream read failed (device disconnected?)");
    return;
  }

  // Published by unique_ptr -- for the subscriber sitting in the same intra-process container
  // (LaneDetection, see the class doc comment), rclcpp's intra-process manager hands this
  // pointer straight to the subscription callback instead of copying/serializing it.
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = now();
  cv_image.header.frame_id = frame_id_;
  cv_image.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image.image = frame;
  cv_image.toImageMsg(*msg);

  image_publisher_->publish(std::move(msg));
}

}  // namespace hyper_camera

RCLCPP_COMPONENTS_REGISTER_NODE(hyper_camera::LogitechCameraPublisherNode)
