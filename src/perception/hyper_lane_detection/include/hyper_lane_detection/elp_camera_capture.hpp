#ifndef HYPER_LANE_DETECTION__ELP_CAMERA_CAPTURE_HPP_
#define HYPER_LANE_DETECTION__ELP_CAMERA_CAPTURE_HPP_

#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hyper_lane_detection
{

/**
 * @brief Owns the physical ELP USB camera device for input_backend:=direct_usb: opens the V4L2
 * device, captures+decodes MJPEG frames, and rectifies each one against a fisheye (equidistant)
 * calibration loaded once at startup.
 *
 * @details Deliberately has no knowledge of lane/stop-line detection -- callers only ever see the
 * rectified cv::Mat this class hands back, the same shape of image lane_detection_node previously
 * received from image_rect over ROS (hyper_camera's RectifyNode), just produced in-process now
 * instead of over an intermediate topic.
 */
class ElpCameraCapture
{
public:
  struct Config
  {
    std::string device;
    int width{1280};
    int height{720};
    double framerate{30.0};
    std::string calibration_file;
  };

  explicit ElpCameraCapture(rclcpp::Logger logger);

  /**
   * @brief Opens the V4L2 device in MJPEG mode at the configured resolution/framerate, and builds
   * the fisheye rectification maps once from `config.calibration_file`.
   *
   * @return False if the device can't be opened or the calibration file can't be parsed --
   * callers must treat this as a fatal startup failure rather than continuing without a feed.
   */
  bool open(const Config & config);

  /**
   * @brief Grabs and decodes the next MJPEG frame and rectifies it.
   *
   * @param[out] rectified_out The rectified BGR8 frame on success.
   * @return False on a capture/decode failure (e.g. the device was unplugged) -- callers must not
   * treat a false return as merely "no frame this tick".
   */
  bool read(cv::Mat & rectified_out);

private:
  rclcpp::Logger logger_;
  cv::VideoCapture capture_;
  cv::Mat map1_;
  cv::Mat map2_;
};

}  // namespace hyper_lane_detection

#endif  // HYPER_LANE_DETECTION__ELP_CAMERA_CAPTURE_HPP_
