#include "auto_vehicle/stopline_detection.hpp"

#include <cstdio>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
constexpr double kLaneWidthMeters = 3.7;
constexpr double kNumLaneInScreen = 3.2;  // how many lanes fit across the BEV image width

// A stop-line bar spans most of the lane, so its bounding box is much wider than it is tall; a
// single zebra-crossing stripe is comparatively close to square. This floor separates the two.
constexpr double kMinStoplineAspectRatio = 3.0;
// The stop-line bar must span at least this fraction of the BEV image width to count -- rules
// out narrower marks (e.g. a single crossing stripe) that happen to pass the aspect ratio test.
constexpr double kMinStoplineWidthFraction = 0.5;
// Floor on contour area [px^2] to reject small mask noise before the shape checks run.
constexpr double kMinStoplineAreaPx = 200.0;

constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;
}  // namespace

StoplineDetection::StoplineDetection() : Node{"stopline_detection"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/bev/image", 10, std::bind(&StoplineDetection::image_callback, this, std::placeholders::_1));

  stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/detection", 10);

  cv::namedWindow("Stopline Detection", cv::WINDOW_NORMAL);
  cv::resizeWindow("Stopline Detection", kWindowWidth, kWindowHeight);

  RCLCPP_INFO(get_logger(), "StoplineDetection started");
}

cv::Mat StoplineDetection::white_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 60, 255), mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

StoplineDetection::StoplineResult StoplineDetection::find_stopline(
  const cv::Mat & mask, const cv::Point2d & origin, double meters_per_pixel) const
{
  StoplineResult result;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  const double min_width_px = mask.cols * kMinStoplineWidthFraction;

  bool found = false;
  cv::Rect best_box;
  for (const auto & contour : contours) {
    if (cv::contourArea(contour) < kMinStoplineAreaPx) {
      continue;
    }
    const cv::Rect box = cv::boundingRect(contour);
    const double aspect_ratio = static_cast<double>(box.width) / box.height;
    if (aspect_ratio < kMinStoplineAspectRatio || box.width < min_width_px) {
      continue;
    }
    // Closest to the vehicle wins: the stop line that actually governs the next stop.
    if (!found || box.y + box.height > best_box.y + best_box.height) {
      best_box = box;
      found = true;
    }
  }

  if (!found) {
    return result;
  }

  result.valid = true;
  result.bounding_box = best_box;
  const double stopline_row = best_box.y + best_box.height;  // bottom edge, closest to vehicle
  result.distance_m = (origin.y - stopline_row) * meters_per_pixel;
  return result;
}

void StoplineDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat & warped = cv_ptr->image;
  const cv::Mat mask = white_mask(warped);

  cv::Mat view = warped.clone();
  view.setTo(cv::Scalar(0, 255, 0), mask);

  const cv::Point2d origin(warped.cols / 2.0, warped.rows - 1.0);
  const double meters_per_pixel =
    kNumLaneInScreen * kLaneWidthMeters / static_cast<double>(warped.cols);

  const StoplineResult result = find_stopline(mask, origin, meters_per_pixel);

  if (result.valid) {
    cv::rectangle(view, result.bounding_box, cv::Scalar(0, 0, 255), 3);
  }

  std_msgs::msg::Float64MultiArray stopline_msg;
  stopline_msg.data = {result.distance_m, result.valid ? 1.0 : 0.0};
  stopline_publisher_->publish(stopline_msg);

  const cv::Scalar text_color = result.valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  char distance_text[64];
  std::snprintf(distance_text, sizeof(distance_text), "Distance: %.2f m", result.distance_m);
  cv::putText(
    view, distance_text, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

  if (!result.valid) {
    cv::putText(
      view, "Stopline not detected", cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color,
      2);
  }

  cv::imshow("Stopline Detection", view);
  cv::waitKey(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StoplineDetection>());
  rclcpp::shutdown();
  return 0;
}
