#ifndef HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
#define HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "hyper_lane_detection/elp_camera_capture.hpp"
#include "hyper_lane_detection/input_backend.hpp"
#include "hyper_lane_detection/lane_detector.hpp"
#include "hyper_lane_detection/stopline_detector.hpp"

class LaneDetection : public rclcpp::Node
{
public:
  LaneDetection();

private:
  /// Which physical camera a frame came from -- selects the publishers/window/fit parameters
  /// process_frame() runs it through. Orthogonal to InputBackend: the backend decides how a frame
  /// arrives (topic vs. direct device), CameraSide decides whose lane it is.
  enum class CameraSide { kFront, kRear };

  /**
   * @brief input_backend:=ros_raw callback for the front camera: a plain sensor_msgs/Image
   * subscription. Gazebo's bridged camera image is already the expected simulation input, so this
   * does not rectify -- it decodes via cv_bridge and hands off to process_frame() directly.
   *
   * @param msg Incoming raw camera image.
   */
  void raw_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /// input_backend:=ros_raw callback for the rear camera -- see raw_image_callback().
  void raw_rear_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /**
   * @brief input_backend:=direct_usb setup: opens the physical ELP camera via ElpCameraCapture
   * (device, resolution, framerate, and calibration file all read from ROS parameters) and starts
   * capture_timer_ polling it at the configured framerate. Front camera only -- direct_usb has no
   * rear-camera equivalent, so nothing here waits on or fails without one.
   *
   * @throws std::runtime_error if the camera device can't be opened or its calibration can't be
   * loaded; this is a startup failure and must not be swallowed into a degraded-but-running node.
   */
  void setup_direct_usb_capture();

  /// capture_timer_'s callback: pulls one rectified frame from elp_capture_ and, on success, hands
  /// it to process_frame(). A read failure is already logged inside ElpCameraCapture::read(), so
  /// this just skips the tick rather than treating a transient glitch as fatal.
  void capture_timer_callback();

  /**
   * @brief The full per-frame pipeline shared by every input backend and both cameras: bird's-eye
   * warp, lane and stop-line detection (delegated to lane_detector_/stopline_detector_), message
   * publish, and debug view. Factored out so the algorithm itself never depends on how the frame
   * arrived (ROS topic vs. direct device) or which camera it's from -- `side` alone selects the
   * publishers/window/fit parameters below.
   *
   * @param image Camera frame (BGR8) -- already rectified for the front camera under
   * input_backend:=direct_usb; passed through as-is for every other backend/camera combination
   * (ros_raw's simulated frames need no rectification at all).
   * @param stamp Capture timestamp of `image`. Threaded through for future header-stamped outputs
   * and latency logging; the current output messages (std_msgs/Float64MultiArray) carry no header
   * of their own to stamp.
   * @param side Which camera `image` is from -- selects the publishers/window and, in
   * LaneDetector::detect_and_draw(), whether the front's straight-fit/lane-center-bias or the
   * rear's curve-fit/parking-standoff-bias applies (see bird_eye()).
   */
  void process_frame(const cv::Mat & image, const rclcpp::Time & stamp, CameraSide side);

  /**
   * @brief Builds the perspective transform mapping the trapezoidal ROI (sampled from the
   * source image's own dimensions) to a bird's-eye view of the given destination size.
   *
   * @param src_height Source image height [px], used only to locate the ROI corners.
   * @param src_width Source image width [px], used only to locate the ROI corners.
   * @param dst_height Output (bird's-eye) image height [px].
   * @param dst_width Output (bird's-eye) image width [px].
   * @param use_rear_roi Which camera side's ROI shape (front vs. rear) -- see bird_eye().
   * @param use_sim_roi Which camera *model*'s ROI (real ELP vs. Gazebo sim) -- see bird_eye().
   * @return The 3x3 perspective transform matrix.
   */
  cv::Mat build_transform(
    int src_height, int src_width, int dst_height, int dst_width, bool use_rear_roi,
    bool use_sim_roi) const;

  /**
   * @brief Warps the input image to a bird's-eye view using build_transform().
   *
   * @param use_rear_roi False: front camera. True: rear camera -- reaches farther toward the
   * horizon and stretched across more output rows, for a clearer view of the (comparatively slow,
   * close-range) parking maneuver than the front's ordinary-road-speed settings need.
   * @param use_sim_roi False: the real ELP camera, rectified from a ~170 deg fisheye lens down to
   * a much narrower rectilinear projection (see ELP-USBGS1200P01-KL170.yaml's P vs. K). True: the
   * Gazebo sim camera, a plain rectilinear model at its own, differently proportioned FOV and
   * resolution (153 deg HFOV, 640x400 16:10 vs. the real camera's 1280x720 16:9). A homography
   * tuned for one camera's FOV/geometry does not transfer to the other, so each combination of
   * `use_rear_roi`/`use_sim_roi` gets its own ROI/BEV-height-scale constants (see the anonymous
   * namespace in the .cpp) rather than sharing one set across cameras that don't actually see the
   * same ground-plane geometry.
   */
  cv::Mat bird_eye(const cv::Mat & image, bool use_rear_roi, bool use_sim_roi) const;

  hyper_lane_detection::InputBackend input_backend_{hyper_lane_detection::InputBackend::kDirectUsb};

  hyper_lane_detection::LaneDetector lane_detector_;
  hyper_lane_detection::StoplineDetector stopline_detector_;

  // input_backend:=ros_raw only.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_rear_image_subscriber_;

  // input_backend:=direct_usb only -- front camera exclusively, see setup_direct_usb_capture().
  std::unique_ptr<hyper_lane_detection::ElpCameraCapture> elp_capture_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stopline_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_lane_center_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_stopline_publisher_;
};

#endif  // HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
