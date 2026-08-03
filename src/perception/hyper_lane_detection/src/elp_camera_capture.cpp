#include "hyper_lane_detection/elp_camera_capture.hpp"

#include <algorithm>

#include <camera_calibration_parsers/parse.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace hyper_lane_detection
{

ElpCameraCapture::ElpCameraCapture(rclcpp::Logger logger) : logger_(logger) {}

bool ElpCameraCapture::open(const Config & config)
{
  capture_.open(config.device, cv::CAP_V4L2);
  if (!capture_.isOpened()) {
    RCLCPP_FATAL(
      logger_, "direct_usb: failed to open camera device '%s'", config.device.c_str());
    return false;
  }

  // MJPEG in, decoded to BGR8 out -- OpenCV's V4L2 backend does the JPEG decode internally on
  // read()/retrieve() when CAP_PROP_CONVERT_RGB (default true) is left enabled.
  capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  capture_.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
  capture_.set(cv::CAP_PROP_FPS, config.framerate);

  std::string camera_name;
  sensor_msgs::msg::CameraInfo cam_info;
  if (!camera_calibration_parsers::readCalibration(
      config.calibration_file, camera_name, cam_info)) {
    RCLCPP_FATAL(
      logger_, "direct_usb: failed to read calibration file '%s'",
      config.calibration_file.c_str());
    return false;
  }

  cv::Mat k(3, 3, CV_64F);
  std::copy(cam_info.k.begin(), cam_info.k.end(), k.ptr<double>());

  cv::Mat d(1, static_cast<int>(cam_info.d.size()), CV_64F);
  std::copy(cam_info.d.begin(), cam_info.d.end(), d.ptr<double>());

  cv::Mat r(3, 3, CV_64F);
  std::copy(cam_info.r.begin(), cam_info.r.end(), r.ptr<double>());

  // cam_info.p is the 3x4 [K'|0] projection matrix; only its leading 3x3 block is the rectified
  // camera matrix cv::fisheye::initUndistortRectifyMap expects as `P`.
  cv::Mat p(3, 4, CV_64F);
  std::copy(cam_info.p.begin(), cam_info.p.end(), p.ptr<double>());
  const cv::Mat new_k = p(cv::Rect(0, 0, 3, 3)).clone();

  const cv::Size image_size(config.width, config.height);
  cv::fisheye::initUndistortRectifyMap(k, d, r, new_k, image_size, CV_16SC2, map1_, map2_);

  RCLCPP_INFO(
    logger_, "direct_usb: opened '%s' (%dx%d @ %.0f fps), calibration '%s' loaded",
    config.device.c_str(), config.width, config.height, config.framerate,
    config.calibration_file.c_str());
  return true;
}

bool ElpCameraCapture::read(cv::Mat & rectified_out)
{
  cv::Mat frame;
  if (!capture_.read(frame) || frame.empty()) {
    RCLCPP_ERROR(logger_, "direct_usb: camera stream read failed (device disconnected?)");
    return false;
  }
  cv::remap(frame, rectified_out, map1_, map2_, cv::INTER_LINEAR);
  return true;
}

}  // namespace hyper_lane_detection
