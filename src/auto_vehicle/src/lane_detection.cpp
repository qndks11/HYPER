#include "auto_vehicle/lane_detection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
// ROI trapezoid corners as (row_ratio, col_ratio) of the source image
constexpr double kRoiTopLeftRow = 0.40,  kRoiTopLeftCol = 0.25;
constexpr double kRoiTopRightRow = 0.40, kRoiTopRightCol = 0.75;
constexpr double kRoiBottomLeftRow = 1,  kRoiBottomLeftCol = -1;
constexpr double kRoiBottomRightRow = 1, kRoiBottomRightCol = 2;

constexpr double kMaxAngleRad = CV_PI;    // fallback/clamp bound for the search domain
// Target angular resolution of the polar image, i.e. how many raw points feed the spline fit
// per frame. Finer (smaller) gives more detail but also a denser, more numerically twitchy
// spline that visibly jitters/lags frame to frame; coarser trades some curve detail for a
// steadier fit.
constexpr double kAngleStepRad = 0.05;

// Sliding window over the polar (angle-row x radius-col) image. Deliberately wide in radius and
// narrow in angle: "horizontally long" so a window survives local gaps/deviations in the mask
// without losing the lane, while staying narrow enough in angle to not blur the search.
constexpr int kWindowRadiusSpan = 25;    // columns (px along the radius axis)
constexpr int kWindowAngleSpanRows = 3;  // rows (angle bins)
constexpr double kWindowFillThreshold = 0.30;

constexpr int kMinLanePoints = 6;   // a few points of slack past the 4 a cubic needs exactly
constexpr int kCurveSamples = 40;   // density of the drawn/unwarped curve, not the fit itself
constexpr double kMinSecondDerivative = 1e-6;

constexpr double kLaneWidthMeters = 3.7;
constexpr double kArrowLength = 100.0;

constexpr int kThumbWidth = 420;
constexpr int kThumbHeight = 300;

// Fits x = a*y^3 + b*y^2 + c*y + d to the given points by least squares.
cv::Vec4d fit_cubic(const std::vector<cv::Point> & points)
{
  const int n = static_cast<int>(points.size());
  cv::Mat A(n, 4, CV_64F);
  cv::Mat b(n, 1, CV_64F);

  for (int i = 0; i < n; ++i) {
    const double y = points[i].y;
    A.at<double>(i, 0) = y * y * y;
    A.at<double>(i, 1) = y * y;
    A.at<double>(i, 2) = y;
    A.at<double>(i, 3) = 1.0;
    b.at<double>(i, 0) = points[i].x;
  }

  cv::Mat coeffs;
  cv::solve(A, b, coeffs, cv::DECOMP_SVD);
  return cv::Vec4d(
    coeffs.at<double>(0), coeffs.at<double>(1), coeffs.at<double>(2), coeffs.at<double>(3));
}

double eval_cubic(const cv::Vec4d & fit, double y)
{
  return fit[0] * y * y * y + fit[1] * y * y + fit[2] * y + fit[3];
}

double eval_cubic_first_derivative(const cv::Vec4d & fit, double y)
{
  return 3.0 * fit[0] * y * y + 2.0 * fit[1] * y + fit[2];
}

double eval_cubic_second_derivative(const cv::Vec4d & fit, double y)
{
  return 6.0 * fit[0] * y + 2.0 * fit[1];
}

// Sum of `image`'s pixels within the half-open rect [row0,row1) x [col0,col1), via a precomputed
// integral image (as returned by cv::integral). O(1) regardless of rect size.
int box_sum(const cv::Mat & integral, int row0, int col0, int row1, int col1)
{
  return integral.at<int>(row1, col1) - integral.at<int>(row0, col1) -
         integral.at<int>(row1, col0) + integral.at<int>(row0, col0);
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

cv::Mat LaneDetection::yellow_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(35, 255, 255), mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

LaneDetection::PolarSearchResult LaneDetection::polar_search(
  const cv::Mat & binary, const cv::Point2d & origin) const
{
  PolarSearchResult result;

  constexpr double kAlpha = 0.0;           // due right
  constexpr double kBeta = kMaxAngleRad;   // due forward
  const int theta_samples =
    std::max(2, static_cast<int>(std::round((kBeta - kAlpha) / kAngleStepRad)) + 1);
  const int max_radius = static_cast<int>(std::ceil(std::hypot(binary.cols, binary.rows)));

  // Remap the mask into polar space: row = angle in [0, pi/2], col = radius in [0, max_radius).
  cv::Mat map_x(theta_samples, max_radius, CV_32F);
  cv::Mat map_y(theta_samples, max_radius, CV_32F);
  for (int row = 0; row < theta_samples; ++row) {
    const double theta = kAlpha + (kBeta - kAlpha) * row / (theta_samples - 1);
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    for (int col = 0; col < max_radius; ++col) {
      map_x.at<float>(row, col) = static_cast<float>(origin.x + col * cos_t);
      map_y.at<float>(row, col) = static_cast<float>(origin.y - col * sin_t);
    }
  }
  cv::remap(binary, result.polar_image, map_x, map_y, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);

  cv::Mat polar_01;
  cv::threshold(result.polar_image, polar_01, 0, 1, cv::THRESH_BINARY);
  cv::Mat integral;
  cv::integral(polar_01, integral, CV_32S);

  bool found_any = false;
  for (int row = 0; row < theta_samples; ++row) {
    const int row0 = std::max(0, row - kWindowAngleSpanRows / 2);
    const int row1 = std::min(theta_samples, row0 + kWindowAngleSpanRows);
    const int window_area = (row1 - row0) * kWindowRadiusSpan;

    bool found = false;
    int found_col = 0;
    for (int col0 = 0; col0 + kWindowRadiusSpan <= max_radius; ++col0) {
      const int col1 = col0 + kWindowRadiusSpan;
      const double ratio = static_cast<double>(box_sum(integral, row0, col0, row1, col1)) /
                            window_area;
      if (ratio >= kWindowFillThreshold) {
        found = true;
        found_col = col0;
        break;
      }
    }

    if (!found) {
      // Small angles walk through the ROI trapezoid's extrapolated near-field dead zone (see
      // build_transform) before ever reaching real image content, so a miss there is just "not
      // yet visible," not a disconnect. Once the lane has actually been picked up, though, a
      // miss means it genuinely ended -- stop the sweep.
      if (found_any) {
        break;
      }
      continue;
    }
    found_any = true;

    // Refine to the exact nearest yellow column within the qualifying window, so a wide window
    // never reports a coarser radius than what is actually there.
    int refined_col = found_col + kWindowRadiusSpan - 1;
    for (int col = found_col; col < found_col + kWindowRadiusSpan; ++col) {
      if (box_sum(integral, row0, col, row1, col + 1) > 0) {
        refined_col = col;
        break;
      }
    }

    const double theta = kAlpha + (kBeta - kAlpha) * row / (theta_samples - 1);
    result.polar_points.emplace_back(refined_col, row);
    result.polar_boxes.emplace_back(found_col, row0, kWindowRadiusSpan, row1 - row0);
    result.points.emplace_back(
      static_cast<int>(std::lround(origin.x + refined_col * std::cos(theta))),
      static_cast<int>(std::lround(origin.y - refined_col * std::sin(theta))));
  }

  return result;
}

LaneDetection::LaneFitResult LaneDetection::fit_lane(
  const std::vector<cv::Point> & points, const cv::Point2d & origin, int width) const
{
  LaneFitResult result;
  if (static_cast<int>(points.size()) < kMinLanePoints) {
    return result;
  }

  const cv::Vec4d fit = fit_cubic(points);

  // Sample the fitted curve across the y range the points actually span, for drawing.
  const auto minmax_y = std::minmax_element(
    points.begin(), points.end(),
    [](const cv::Point & a, const cv::Point & b) { return a.y < b.y; });
  const double y_near = minmax_y.second->y;  // closest to the vehicle
  const double y_far = minmax_y.first->y;    // farthest point found
  result.curve_points.reserve(kCurveSamples);
  for (int i = 0; i < kCurveSamples; ++i) {
    const double y = y_near + (y_far - y_near) * i / (kCurveSamples - 1);
    result.curve_points.emplace_back(
      static_cast<int>(std::lround(eval_cubic(fit, y))), static_cast<int>(std::lround(y)));
  }

  // Curvature and heading, evaluated at the vehicle's row.
  const double y_eval = origin.y;
  const double slope = eval_cubic_first_derivative(fit, y_eval);
  const double second_derivative = eval_cubic_second_derivative(fit, y_eval);
  result.curvature_radius_px =
    std::pow(1.0 + slope * slope, 1.5) /
    std::max(std::abs(second_derivative), kMinSecondDerivative);
  result.steering_angle_deg = std::atan(slope) * 180.0 / CV_PI;

  const double meters_per_pixel = kLaneWidthMeters / static_cast<double>(width);
  result.offset_m = (eval_cubic(fit, y_eval) - origin.x) * meters_per_pixel;

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
  cv::rectangle(thumb, cv::Point(0, 0), cv::Point(kThumbWidth, 24), cv::Scalar(0, 0, 0), -1);
  cv::putText(
    thumb, label, cv::Point(6, 17), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
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

  cv::Mat row1, row2, dashboard;
  cv::hconcat(std::vector<cv::Mat>(thumbs.begin(), thumbs.begin() + 3), row1);
  cv::hconcat(std::vector<cv::Mat>(thumbs.begin() + 3, thumbs.begin() + 6), row2);
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

  // Panel 1: mark the ROI trapezoid corners on the original frame.
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
  const cv::Mat mask = yellow_mask(warped);

  // Panel 2: BEV with the yellow mask highlighted.
  cv::Mat masked_view = warped.clone();
  masked_view.setTo(cv::Scalar(0, 255, 0), mask);

  const cv::Point2d origin(warped.cols / 2.0, warped.rows - 1.0);
  const PolarSearchResult search = polar_search(mask, origin);

  // Fall back to the previous frame's lane when nothing is found this frame
  std::vector<cv::Point> points = search.points;
  if (points.empty()) {
    points = prev_points_;
  } else {
    prev_points_ = points;
  }

  const LaneFitResult fit = fit_lane(points, origin, warped.cols);

  // Panel 3: the raw polar-remapped mask.
  cv::Mat polar_view;
  cv::cvtColor(search.polar_image, polar_view, cv::COLOR_GRAY2BGR);

  // Panel 4: the polar mask annotated with the sliding windows and refined hits.
  cv::Mat polar_search_view = polar_view.clone();
  for (const auto & box : search.polar_boxes) {
    cv::rectangle(polar_search_view, box, cv::Scalar(255, 255, 255), 1);
  }
  for (const auto & p : search.polar_points) {
    cv::circle(polar_search_view, p, 2, cv::Scalar(0, 255, 0), -1);
  }

  // Panel 5: the fitted cubic polynomial back in BEV coordinates.
  cv::Mat bev_curve_view;
  cv::cvtColor(mask, bev_curve_view, cv::COLOR_GRAY2BGR);
  cv::circle(
    bev_curve_view, cv::Point(static_cast<int>(origin.x), static_cast<int>(origin.y)), 5,
    cv::Scalar(0, 0, 255), -1);
  for (const auto & p : points) {
    cv::circle(bev_curve_view, p, 3, cv::Scalar(0, 255, 0), -1);
  }
  if (fit.valid) {
    cv::polylines(bev_curve_view, fit.curve_points, false, cv::Scalar(255, 0, 0), 2);
  }

  // Panel 6: the fitted curve unwarped back onto the original frame, with the fit stats.
  cv::Mat result = frame.clone();
  if (fit.valid) {
    std::vector<cv::Point2f> bev_points;
    bev_points.reserve(fit.curve_points.size());
    for (const auto & p : fit.curve_points) bev_points.emplace_back(p);

    std::vector<cv::Point2f> original_points;
    cv::perspectiveTransform(bev_points, original_points, build_transform(height, width).inv());

    std::vector<cv::Point> original_points_i;
    original_points_i.reserve(original_points.size());
    for (const auto & p : original_points) {
      original_points_i.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
    }
    cv::polylines(result, original_points_i, false, cv::Scalar(255, 0, 255), 3);

    const cv::Point arrow_start(width / 2, height);
    const cv::Point arrow_end(
      static_cast<int>(width / 2 + kArrowLength * std::sin(fit.steering_angle_deg * CV_PI / 180.0)),
      static_cast<int>(height - kArrowLength * std::cos(fit.steering_angle_deg * CV_PI / 180.0)));
    cv::line(result, arrow_start, arrow_end, cv::Scalar(255, 0, 0), 2);
  }

  std_msgs::msg::Float64MultiArray lane_msg;
  lane_msg.data = {fit.offset_m, fit.steering_angle_deg, fit.curvature_radius_px,
                    fit.valid ? 1.0 : 0.0};
  lane_center_publisher_->publish(lane_msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "[/lane/center] offset=%.3f m, angle=%.2f deg, radius=%.1f px, valid=%s", fit.offset_m,
    fit.steering_angle_deg, fit.curvature_radius_px, fit.valid ? "true" : "false");

  const cv::Scalar text_color = fit.valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  char radius_text[64];
  std::snprintf(radius_text, sizeof(radius_text), "Radius: %.1f px", fit.curvature_radius_px);
  cv::putText(
    result, radius_text, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

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
    {"BEV Yellow Mask", masked_view},
    {"Polar Coordinates", polar_view},
    {"Closest Yellow Box", polar_search_view},
    {"Polynomial (BEV)", bev_curve_view},
    {"Result", result}});

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
