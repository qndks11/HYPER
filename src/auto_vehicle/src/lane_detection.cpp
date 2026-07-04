#include "auto_vehicle/lane_detection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
// ROI trapezoid corners as (row_ratio, col_ratio) of the source image
constexpr double kRoiTopLeftRow = 0.6,  kRoiTopLeftCol = 0.25;
constexpr double kRoiTopRightRow = 0.6, kRoiTopRightCol = 0.75;
constexpr double kRoiBottomLeftRow = 1,  kRoiBottomLeftCol = -0.6;
constexpr double kRoiBottomRightRow = 1, kRoiBottomRightCol = 1.6;


constexpr int kNumWindows = 12;
constexpr int kMargin = 50;
constexpr int kMinPix = 50;

constexpr double kLaneWidthMeters = 3.7;
constexpr double kArrowLength = 100.0;

constexpr int kThumbWidth = 480;
constexpr int kThumbHeight = 360;

// Fits x = a*y^2 + b*y + c to the given points using least squares.
cv::Vec3d fit_quadratic(const std::vector<cv::Point> & points)
{
  const int n = static_cast<int>(points.size());
  cv::Mat A(n, 3, CV_64F);
  cv::Mat b(n, 1, CV_64F);

  for (int i = 0; i < n; ++i) {
    const double y = points[i].y;
    A.at<double>(i, 0) = y * y;
    A.at<double>(i, 1) = y;
    A.at<double>(i, 2) = 1.0;
    b.at<double>(i, 0) = points[i].x;
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

  cv::namedWindow("Dashboard", cv::WINDOW_NORMAL);
  cv::resizeWindow("Dashboard", kThumbWidth * 3, kThumbHeight * 2);

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
    {static_cast<float>(width), 0.0f},
    {static_cast<float>(width), static_cast<float>(height)},
    {0.0f, static_cast<float>(height)}};

  return cv::getPerspectiveTransform(src, dst);
}

cv::Mat LaneDetection::bird_eye(const cv::Mat & image) const
{
  const cv::Mat transform = build_transform(image.rows, image.cols);
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, image.size());
  return warped;
}

cv::Mat LaneDetection::binary_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat white_mask;
  cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 30, 255), white_mask);

  cv::Mat yellow_mask;
  cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(35, 255, 255), yellow_mask);

  cv::Mat mask;
  cv::bitwise_or(white_mask, yellow_mask, mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

void LaneDetection::sliding_window_search(
  const cv::Mat & binary, cv::Mat & windows_view, std::vector<cv::Point> & left_points,
  std::vector<cv::Point> & right_points) const
{
  const int height = binary.rows;
  const int width = binary.cols;
  const int mid = width / 2;

  // Histogram (column-wise pixel value sum) of the bottom half locates the initial bases
  const cv::Mat lower_half = binary(cv::Range(height / 2, height), cv::Range::all());
  cv::Mat column_sums;
  cv::reduce(lower_half, column_sums, 0, cv::REDUCE_SUM, CV_32S);

  int left_base = 0;
  int right_base = mid;
  int left_max = -1;
  int right_max = -1;
  for (int col = 0; col < mid; ++col) {
    const int value = column_sums.at<int>(0, col);
    if (value > left_max) {
      left_max = value;
      left_base = col;
    }
  }
  for (int col = mid; col < width; ++col) {
    const int value = column_sums.at<int>(0, col);
    if (value > right_max) {
      right_max = value;
      right_base = col;
    }
  }

  const int window_height = std::max(1, height / kNumWindows);

  for (int y_high = height; y_high > 0; y_high -= window_height) {
    const int y_low = std::max(0, y_high - window_height);

    const int left_low = std::clamp(left_base - kMargin, 0, width);
    const int left_high = std::clamp(left_base + kMargin, 0, width);
    const int right_low = std::clamp(right_base - kMargin, 0, width);
    const int right_high = std::clamp(right_base + kMargin, 0, width);

    cv::rectangle(
      windows_view, cv::Point(left_low, y_low), cv::Point(left_high, y_high),
      cv::Scalar(255, 255, 255), 2);
    cv::rectangle(
      windows_view, cv::Point(right_low, y_low), cv::Point(right_high, y_high),
      cv::Scalar(255, 255, 255), 2);

    // Left window: find the largest contour's centroid, if any clears the pixel-count threshold
    const cv::Mat left_window = binary(cv::Range(y_low, y_high), cv::Range(left_low, left_high)).clone();
    if (cv::countNonZero(left_window) > kMinPix) {
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(left_window, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      if (!contours.empty()) {
        const auto & largest = *std::max_element(
          contours.begin(), contours.end(), [](const auto & a, const auto & b) {
            return cv::contourArea(a) < cv::contourArea(b);
          });
        const cv::Moments m = cv::moments(largest);
        if (m.m00 != 0) {
          const int cx = static_cast<int>(m.m10 / m.m00) + left_low;
          const int cy = static_cast<int>(m.m01 / m.m00) + y_low;
          left_points.emplace_back(cx, cy);
          left_base = cx;
          cv::circle(windows_view, cv::Point(cx, cy), 4, cv::Scalar(0, 255, 0), -1);
        }
      }
    }

    // Right window: mirror of the left window search
    const cv::Mat right_window =
      binary(cv::Range(y_low, y_high), cv::Range(right_low, right_high)).clone();
    if (cv::countNonZero(right_window) > kMinPix) {
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(right_window, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      if (!contours.empty()) {
        const auto & largest = *std::max_element(
          contours.begin(), contours.end(), [](const auto & a, const auto & b) {
            return cv::contourArea(a) < cv::contourArea(b);
          });
        const cv::Moments m = cv::moments(largest);
        if (m.m00 != 0) {
          const int cx = static_cast<int>(m.m10 / m.m00) + right_low;
          const int cy = static_cast<int>(m.m01 / m.m00) + y_low;
          right_points.emplace_back(cx, cy);
          right_base = cx;
          cv::circle(windows_view, cv::Point(cx, cy), 4, cv::Scalar(0, 255, 0), -1);
        }
      }
    }
  }
}

LaneDetection::LaneFitResult LaneDetection::evaluate_lane(
  const std::vector<cv::Point> & left_points, const std::vector<cv::Point> & right_points,
  int width, int height) const
{
  LaneFitResult result;
  if (left_points.size() < 3 || right_points.size() < 3) {
    return result;
  }

  result.left_fit = fit_quadratic(left_points);
  result.right_fit = fit_quadratic(right_points);

  const double y_eval = static_cast<double>(height);

  const double left_slope = 2.0 * result.left_fit[0] * y_eval + result.left_fit[1];
  const double right_slope = 2.0 * result.right_fit[0] * y_eval + result.right_fit[1];
  const double left_curvature =
    std::pow(1.0 + left_slope * left_slope, 1.5) / std::abs(2.0 * result.left_fit[0]);
  const double right_curvature =
    std::pow(1.0 + right_slope * right_slope, 1.5) / std::abs(2.0 * result.right_fit[0]);
  result.curvature_px = (left_curvature + right_curvature) / 2.0;

  // Offset uses the fit evaluated at the bottom of the image (closest to the vehicle)
  const double left_x = eval_quadratic(result.left_fit, y_eval);
  const double right_x = eval_quadratic(result.right_fit, y_eval);
  const double lane_center = (left_x + right_x) / 2.0;
  const double meters_per_pixel = kLaneWidthMeters / static_cast<double>(width);
  result.offset_m = (width / 2.0 - lane_center) * meters_per_pixel;

  // Heading error: average slope (dx/dy) of both lane fits at the vehicle position,
  // converted to an angle from the forward (vertical) axis.
  const double heading_slope = (left_slope + right_slope) / 2.0;
  result.steering_angle_deg = std::atan(heading_slope) * 180.0 / CV_PI;
  result.valid = true;

  return result;
}
cv::Mat LaneDetection::make_thumbnail(const cv::Mat & image, const std::string & label) const
{
  cv::Mat bgr = image;
  if (image.channels() == 1) {
    cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  }

  cv::Mat thumb;
  cv::resize(bgr, thumb, cv::Size(kThumbWidth, kThumbHeight));
  cv::rectangle(thumb, cv::Point(0, 0), cv::Point(kThumbWidth, 28), cv::Scalar(0, 0, 0), -1);
  cv::putText(
    thumb, label, cv::Point(6, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
  return thumb;
}

cv::Mat LaneDetection::build_dashboard(
  const std::vector<std::pair<std::string, cv::Mat>> & views) const
{
  std::vector<cv::Mat> thumbs;
  thumbs.reserve(views.size());
  for (const auto & [label, image] : views) {
    thumbs.push_back(make_thumbnail(image, label));
  }

  cv::Mat row1;
  cv::Mat row2;
  cv::hconcat(std::vector<cv::Mat>(thumbs.begin(), thumbs.begin() + 3), row1);
  cv::hconcat(std::vector<cv::Mat>(thumbs.begin() + 3, thumbs.begin() + 6), row2);

  cv::Mat dashboard;
  cv::vconcat(row1, row2, dashboard);
  return dashboard;
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

  const cv::Mat frame = cv_ptr->image;
  const int height = frame.rows;
  const int width = frame.cols;

  // Mark the ROI trapezoid corners on the original frame for the dashboard
  cv::Mat original_view = frame.clone();
  {
    const std::vector<cv::Point> roi_corners{
      {static_cast<int>(width * kRoiTopLeftCol), static_cast<int>(height * kRoiTopLeftRow)},
      {static_cast<int>(width * kRoiBottomLeftCol), static_cast<int>(height * kRoiBottomLeftRow)},
      {static_cast<int>(width * kRoiTopRightCol), static_cast<int>(height * kRoiTopRightRow)},
      {static_cast<int>(width * kRoiBottomRightCol),
       static_cast<int>(height * kRoiBottomRightRow)}};
    for (const auto & p : roi_corners) cv::circle(original_view, p, 5, cv::Scalar(0, 0, 255), -1);
  }

  const cv::Mat warped = bird_eye(frame);
  const cv::Mat binary = binary_mask(warped);

  cv::Mat windows_view;
  cv::cvtColor(binary, windows_view, cv::COLOR_GRAY2BGR);

  std::vector<cv::Point> left_points;
  std::vector<cv::Point> right_points;
  sliding_window_search(binary, windows_view, left_points, right_points);

  // Fall back to the previous frame's lane when nothing clears the threshold this frame
  if (left_points.empty()) {
    left_points = prev_left_points_;
  } else {
    prev_left_points_ = left_points;
  }
  if (right_points.empty()) {
    right_points = prev_right_points_;
  } else {
    prev_right_points_ = right_points;
  }

  const LaneFitResult fit = evaluate_lane(left_points, right_points, width, height);

  cv::Mat fitted_view = warped.clone();
  cv::Mat result = frame.clone();

  if (fit.valid) {
    const double left_bottom = eval_quadratic(fit.left_fit, height);
    const double left_top = eval_quadratic(fit.left_fit, 0);
    const double right_bottom = eval_quadratic(fit.right_fit, height);
    const double right_top = eval_quadratic(fit.right_fit, 0);

    const std::vector<cv::Point> quad{
      {static_cast<int>(left_bottom), height},
      {static_cast<int>(left_top), 0},
      {static_cast<int>(right_top), 0},
      {static_cast<int>(right_bottom), height}};

    cv::Mat overlay = warped.clone();
    cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{quad}, cv::Scalar(0, 255, 0));
    cv::addWeighted(overlay, 0.2, warped, 0.8, 0.0, fitted_view);

    const cv::Mat inverse_transform = build_transform(height, width).inv();
    cv::Mat unwarped_overlay;
    cv::warpPerspective(fitted_view, unwarped_overlay, inverse_transform, frame.size());
    cv::addWeighted(frame, 1.0, unwarped_overlay, 0.5, 0.0, result);

    const cv::Point arrow_start(width / 2, height);
    const cv::Point arrow_end(
      static_cast<int>(width / 2 + kArrowLength * std::sin(fit.steering_angle_deg * CV_PI / 180.0)),
      static_cast<int>(height - kArrowLength * std::cos(fit.steering_angle_deg * CV_PI / 180.0)));
    cv::line(result, arrow_start, arrow_end, cv::Scalar(255, 0, 0), 2);
  }

  std_msgs::msg::Float64MultiArray lane_msg;
  lane_msg.data = {fit.offset_m, fit.steering_angle_deg, fit.curvature_px, fit.valid ? 1.0 : 0.0};

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "[/lane/center] offset=%.3f m, angle=%.2f deg, curvature=%.1f px, valid=%s",
    fit.offset_m, fit.steering_angle_deg, fit.curvature_px, fit.valid ? "true" : "false");

  const cv::Scalar text_color = fit.valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  char curvature_text[64];
  std::snprintf(curvature_text, sizeof(curvature_text), "Curvature: %.2f px", fit.curvature_px);
  cv::putText(
    result, curvature_text, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

  char offset_text[64];
  std::snprintf(offset_text, sizeof(offset_text), "Offset: %.2f m", fit.offset_m);
  cv::putText(
    result, offset_text, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

  char angle_text[64];
  std::snprintf(angle_text, sizeof(angle_text), "Angle: %.2f deg", fit.steering_angle_deg);
  cv::putText(
    result, angle_text, cv::Point(30, 110), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

  if (!fit.valid) {
    cv::putText(
      result, "Lane not detected", cv::Point(30, 150), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color,
      2);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Lane not detected");
  }

  const cv::Mat dashboard = build_dashboard({
    {"Original", original_view},
    {"Bird's Eye View", warped},
    {"Thresholding", binary},
    {"Sliding Windows", windows_view},
    {"Lane Detection", result},
    {"Fitted Lane (Bird's Eye)", fitted_view}});

  cv::imshow("Dashboard", dashboard);
  cv::waitKey(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetection>());
  rclcpp::shutdown();
  return 0;
}
