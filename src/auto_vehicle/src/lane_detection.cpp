#include "auto_vehicle/lane_detection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
// ROI trapezoid corners as (row_ratio, col_ratio) of the source image
constexpr double kRoiTopLeftRow = 0.6, kRoiTopLeftCol = 0.1;
constexpr double kRoiTopRightRow = 0.6, kRoiTopRightCol = 0.9;
constexpr double kRoiBottomLeftRow = 1.0, kRoiBottomLeftCol = 0.0;
constexpr double kRoiBottomRightRow = 1.0, kRoiBottomRightCol = 1.0;

constexpr int kBirdEyeWidth = 400;
constexpr int kBirdEyeHeight = 600;

constexpr int kNumWindows = 9;
constexpr int kMargin = 60;
constexpr int kMinPix = 30;

// Fits x = a*y^2 + b*y + c to the given pixels using least squares.
cv::Vec3d fit_quadratic(const std::vector<cv::Point> & pixels)
{
  const int n = static_cast<int>(pixels.size());
  cv::Mat A(n, 3, CV_64F);
  cv::Mat b(n, 1, CV_64F);

  for (int i = 0; i < n; ++i) {
    const double y = pixels[i].y;
    A.at<double>(i, 0) = y * y;
    A.at<double>(i, 1) = y;
    A.at<double>(i, 2) = 1.0;
    b.at<double>(i, 0) = pixels[i].x;
  }

  cv::Mat coeffs;
  cv::solve(A, b, coeffs, cv::DECOMP_SVD);
  return cv::Vec3d(coeffs.at<double>(0), coeffs.at<double>(1), coeffs.at<double>(2));
}

double eval_quadratic(const cv::Vec3d & fit, double y)
{
  return fit[0] * y * y + fit[1] * y + fit[2];
}
}  // namespace

LaneDetection::LaneDetection() : Node{"lane_detection"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&LaneDetection::image_callback, this, std::placeholders::_1));

  lane_center_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);

  cv::namedWindow("Lane Detection", cv::WINDOW_AUTOSIZE);

  RCLCPP_INFO(get_logger(), "LaneDetection started");
}

cv::Mat LaneDetection::build_transform(int height, int width) const
{
  const std::vector<cv::Point2f> src{
    {static_cast<float>(width * kRoiTopLeftCol), static_cast<float>(height * kRoiTopLeftRow)},
    {static_cast<float>(width * kRoiTopRightCol), static_cast<float>(height * kRoiTopRightRow)},
    {static_cast<float>(width * kRoiBottomRightCol),
     static_cast<float>(height * kRoiBottomRightRow)},
    {static_cast<float>(width * kRoiBottomLeftCol), static_cast<float>(height * kRoiBottomLeftRow)}};

  const std::vector<cv::Point2f> dst{
    {0.0f, 0.0f},
    {static_cast<float>(kBirdEyeWidth), 0.0f},
    {static_cast<float>(kBirdEyeWidth), static_cast<float>(kBirdEyeHeight)},
    {0.0f, static_cast<float>(kBirdEyeHeight)}};

  return cv::getPerspectiveTransform(src, dst);
}

cv::Mat LaneDetection::bird_eye(const cv::Mat & image) const
{
  const cv::Mat transform = build_transform(image.rows, image.cols);
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, cv::Size(kBirdEyeWidth, kBirdEyeHeight));
  return warped;
}

cv::Mat LaneDetection::binary_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat white_mask;
  cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 40, 255), white_mask);

  cv::Mat yellow_mask;
  cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(35, 255, 255), yellow_mask);

  cv::Mat mask;
  cv::bitwise_or(white_mask, yellow_mask, mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

LaneDetection::LaneFitResult LaneDetection::sliding_window_search(
  const cv::Mat & binary, cv::Mat & overlay) const
{
  LaneFitResult result;

  const int height = binary.rows;
  const int width = binary.cols;
  const int mid = width / 2;

  // Histogram (column-wise pixel value sum) of the bottom half locates the initial bases
  const cv::Mat lower_half = binary(cv::Range(height / 2, height), cv::Range::all());
  cv::Mat column_sums;
  cv::reduce(lower_half, column_sums, 0, cv::REDUCE_SUM, CV_32S);

  int left_x = 0;
  int right_x = mid;
  int left_max = -1;
  int right_max = -1;
  for (int col = 0; col < mid; ++col) {
    const int value = column_sums.at<int>(0, col);
    if (value > left_max) {
      left_max = value;
      left_x = col;
    }
  }
  for (int col = mid; col < width; ++col) {
    const int value = column_sums.at<int>(0, col);
    if (value > right_max) {
      right_max = value;
      right_x = col;
    }
  }

  const int window_height = height / kNumWindows;

  std::vector<cv::Point> left_pixels;
  std::vector<cv::Point> right_pixels;

  for (int window = 0; window < kNumWindows; ++window) {
    const int y_low = height - (window + 1) * window_height;
    const int y_high = height - window * window_height;

    const int left_x_low = std::clamp(left_x - kMargin, 0, width);
    const int left_x_high = std::clamp(left_x + kMargin, 0, width);
    const int right_x_low = std::clamp(right_x - kMargin, 0, width);
    const int right_x_high = std::clamp(right_x + kMargin, 0, width);

    cv::rectangle(
      overlay, cv::Point(left_x_low, y_low), cv::Point(left_x_high, y_high),
      cv::Scalar(0, 255, 255), 2);
    cv::rectangle(
      overlay, cv::Point(right_x_low, y_low), cv::Point(right_x_high, y_high),
      cv::Scalar(0, 255, 255), 2);

    std::vector<cv::Point> left_found;
    std::vector<cv::Point> right_found;

    for (int y = y_low; y < y_high; ++y) {
      const uchar * row = binary.ptr<uchar>(y);
      for (int x = left_x_low; x < left_x_high; ++x) {
        if (row[x] != 0) left_found.emplace_back(x, y);
      }
      for (int x = right_x_low; x < right_x_high; ++x) {
        if (row[x] != 0) right_found.emplace_back(x, y);
      }
    }

    left_pixels.insert(left_pixels.end(), left_found.begin(), left_found.end());
    right_pixels.insert(right_pixels.end(), right_found.begin(), right_found.end());

    if (static_cast<int>(left_found.size()) >= kMinPix) {
      double sum = 0.0;
      for (const auto & p : left_found) sum += p.x;
      left_x = static_cast<int>(sum / left_found.size());
    }
    if (static_cast<int>(right_found.size()) >= kMinPix) {
      double sum = 0.0;
      for (const auto & p : right_found) sum += p.x;
      right_x = static_cast<int>(sum / right_found.size());
    }
  }

  // Paint the detected lane pixels green
  for (const auto & p : left_pixels) overlay.at<cv::Vec3b>(p) = cv::Vec3b(0, 255, 0);
  for (const auto & p : right_pixels) overlay.at<cv::Vec3b>(p) = cv::Vec3b(0, 255, 0);

  if (static_cast<int>(left_pixels.size()) < kMinPix ||
      static_cast<int>(right_pixels.size()) < kMinPix) {
    return result;
  }

  const cv::Vec3d left_fit = fit_quadratic(left_pixels);
  const cv::Vec3d right_fit = fit_quadratic(right_pixels);

  const double y_eval = static_cast<double>(height);
  const double left_x_bottom = eval_quadratic(left_fit, y_eval);
  const double right_x_bottom = eval_quadratic(right_fit, y_eval);
  const double center_x = (left_x_bottom + right_x_bottom) / 2.0;

  // Pixel offset from the image center, positive meaning right of the lane center
  result.offset = (center_x - width / 2.0) / (width / 2.0);

  // Heading error: average slope (dx/dy) of both lane fits at the bottom of the image
  const double left_slope = 2.0 * left_fit[0] * y_eval + left_fit[1];
  const double right_slope = 2.0 * right_fit[0] * y_eval + right_fit[1];
  result.heading_error = (left_slope + right_slope) / 2.0;
  result.valid = true;

  // Draw the fitted lane curves
  std::vector<cv::Point> left_curve;
  std::vector<cv::Point> right_curve;
  for (int y = 0; y < height; y += 4) {
    left_curve.emplace_back(static_cast<int>(eval_quadratic(left_fit, y)), y);
    right_curve.emplace_back(static_cast<int>(eval_quadratic(right_fit, y)), y);
  }
  cv::polylines(overlay, left_curve, false, cv::Scalar(0, 200, 0), 3);
  cv::polylines(overlay, right_curve, false, cv::Scalar(0, 200, 0), 3);

  return result;
}

void LaneDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat warped = bird_eye(cv_ptr->image);
  const cv::Mat binary = binary_mask(warped);

  cv::Mat overlay = warped.clone();
  const LaneFitResult fit = sliding_window_search(binary, overlay);

  std_msgs::msg::Float64MultiArray lane_msg;
  lane_msg.data = {fit.offset, fit.heading_error, fit.valid ? 1.0 : 0.0};
  lane_center_publisher_->publish(lane_msg);

  const cv::Scalar text_color = fit.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

  char offset_text[64];
  std::snprintf(offset_text, sizeof(offset_text), "Offset: %.3f", fit.offset);
  cv::putText(
    overlay, offset_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);

  char angle_text[64];
  std::snprintf(
    angle_text, sizeof(angle_text), "Angle: %.1f deg",
    std::atan(fit.heading_error) * 180.0 / CV_PI);
  cv::putText(
    overlay, angle_text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);

  if (!fit.valid) {
    cv::putText(
      overlay, "Lane not detected", cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color,
      2);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Lane not detected");
  }

  cv::imshow("Lane Detection", overlay);
  cv::waitKey(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetection>());
  rclcpp::shutdown();
  return 0;
}
