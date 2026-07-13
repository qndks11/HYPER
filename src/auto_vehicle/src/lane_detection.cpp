#include "auto_vehicle/lane_detection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
<<<<<<< Updated upstream
// ROI trapezoid corners as (row_ratio, col_ratio) of the source image
constexpr double kRoiTopLeftRow = 0.40,  kRoiTopLeftCol = 0.25;
constexpr double kRoiTopRightRow = 0.40, kRoiTopRightCol = 0.75;
constexpr double kRoiBottomLeftRow = 1,  kRoiBottomLeftCol = -1;
constexpr double kRoiBottomRightRow = 1, kRoiBottomRightCol = 2;


constexpr int kNumWindows = 12;
constexpr int kMargin = 50;
constexpr int kMinPix = 50;
=======
// ROI trapezoid corners as (row_ratio, col_ratio) of the source image.
constexpr double kRoiTopLeftRow = 0.40;
constexpr double kRoiTopLeftCol = 0.25;
constexpr double kRoiTopRightRow = 0.40;
constexpr double kRoiTopRightCol = 0.75;
constexpr double kRoiBottomLeftRow = 1.0;
constexpr double kRoiBottomLeftCol = -1.4;
constexpr double kRoiBottomRightRow = 1.0;
constexpr double kRoiBottomRightCol = 2.4;

// Lane-chain settings.
constexpr double kChainStepRadius = 30.0;
constexpr int kMaxChainSteps = 50;
constexpr double kTopRowMargin = 20.0;
constexpr int kMinLanePoints = 6;
constexpr double kMinForwardSpanPx = 60.0;

// A practically straight lane is represented by this finite radius.
constexpr double kMaxCurvatureRadiusPx = 1e6;
constexpr double kMinCurvature = 1e-6;

// The current BEV calibration assumes the image width corresponds to 3.7 m.
// This is inherited from the existing implementation and should eventually be
// replaced with a measured BEV calibration value.
constexpr double kBevWidthMeters = 3.7;

// Distance from the detected right lane boundary to the desired vehicle path.
// Straight sections keep the existing 0.60 m offset. As curvature increases,
// the target path is moved slightly farther away from the right boundary.
constexpr double kStraightRightLaneToTargetMeters = 0.60;
constexpr double kMaxCurveExtraOffsetMeters = 0.05;

// Adaptive lookahead: use a farther target on a straight and a nearer target
// on a curve. Ratios are relative to the BEV image height.
constexpr double kStraightLookaheadRatio = 0.45;
constexpr double kCurveLookaheadRatio = 0.22;
constexpr double kMinLookaheadPx = 35.0;

// Curvature-radius thresholds used to blend between straight and curved
// lookahead distances.
constexpr double kStraightRadiusThresholdPx = 1400.0;
constexpr double kSharpCurveRadiusThresholdPx = 280.0;

// Robust quadratic-fit residual threshold.
constexpr double kMinResidualThresholdPx = 8.0;
constexpr double kResidualScale = 2.5;

// Keep the previous lane for only a very short camera dropout. Unlike the old
// implementation, the previous lane is not reused indefinitely.
constexpr int kMaxPreviousFrameReuse = 2;

// Output smoothing. A larger value follows changes faster; a smaller value is
// smoother. This is applied only while valid detections continue.
constexpr double kOutputFilterAlpha = 0.35;
>>>>>>> Stashed changes

constexpr double kArrowLength = 100.0;
<<<<<<< Updated upstream

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
=======
constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;
constexpr double kBevHeightScale = 1.35;


double clamp_value(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}


double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }

  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];

  if (values.size() % 2 == 1) {
    return upper;
  }

  std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
  return 0.5 * (values[middle - 1] + upper);
}

>>>>>>> Stashed changes
}  // namespace


LaneDetection::LaneDetection() : Node{"lane_detection"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10,
    std::bind(&LaneDetection::image_callback, this, std::placeholders::_1));

  lane_center_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);

<<<<<<< Updated upstream
  cv::namedWindow("Dashboard", cv::WINDOW_NORMAL);
  cv::resizeWindow("Dashboard", kThumbWidth * 3, kThumbHeight * 2);
=======
  cv::namedWindow("Lane Detection", cv::WINDOW_NORMAL);
  cv::resizeWindow(
    "Lane Detection", kWindowWidth,
    static_cast<int>(kWindowHeight * kBevHeightScale));
>>>>>>> Stashed changes

  RCLCPP_INFO(get_logger(), "LaneDetection started (quadratic curve fit)");
}

<<<<<<< Updated upstream
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
=======

cv::Mat LaneDetection::build_transform(
  int src_height, int src_width, int dst_height, int dst_width) const
{
  const std::vector<cv::Point2f> src{
    {
      static_cast<float>(src_width * kRoiTopLeftCol),
      static_cast<float>(src_height * kRoiTopLeftRow)
    },
    {
      static_cast<float>(src_width * kRoiTopRightCol),
      static_cast<float>(src_height * kRoiTopRightRow)
    },
    {
      static_cast<float>(src_width * kRoiBottomRightCol),
      static_cast<float>(src_height * kRoiBottomRightRow)
    },
    {
      static_cast<float>(src_width * kRoiBottomLeftCol),
      static_cast<float>(src_height * kRoiBottomLeftRow)
    }
  };

  const std::vector<cv::Point2f> dst{
    {0.0f, 0.0f},
    {static_cast<float>(dst_width), 0.0f},
    {static_cast<float>(dst_width), static_cast<float>(dst_height)},
    {0.0f, static_cast<float>(dst_height)}
  };
>>>>>>> Stashed changes

  return cv::getPerspectiveTransform(src, dst);
}


cv::Mat LaneDetection::bird_eye(const cv::Mat & image) const
{
<<<<<<< Updated upstream
  const cv::Mat transform = build_transform(image.rows, image.cols);
=======
  const cv::Size dst_size(
    image.cols,
    static_cast<int>(image.rows * kBevHeightScale));

  const cv::Mat transform = build_transform(
    image.rows, image.cols, dst_size.height, dst_size.width);

>>>>>>> Stashed changes
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, image.size());
  return warped;
}

<<<<<<< Updated upstream
cv::Mat LaneDetection::binary_mask(const cv::Mat & image) const
=======

cv::Mat LaneDetection::yellow_mask(const cv::Mat & image) const
>>>>>>> Stashed changes
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(
    hsv,
    cv::Scalar(15, 80, 80),
    cv::Scalar(35, 255, 255),
    mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

<<<<<<< Updated upstream
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
=======

std::vector<cv::Point> LaneDetection::walk_lane_chain(
  const std::vector<cv::Point> & yellow_points,
  const cv::Point2d & origin) const
{
  std::vector<cv::Point> chain;

  if (yellow_points.empty()) {
    return chain;
  }

  // Start from the bottommost yellow pixel on the right side of the vehicle.
  const cv::Point * seed = nullptr;

  for (const auto & point : yellow_points) {
    if (point.x < origin.x) {
      continue;
    }

    if (
      seed == nullptr || point.y > seed->y ||
      (
        point.y == seed->y &&
        std::abs(point.x - origin.x) < std::abs(seed->x - origin.x)
      )
    ) {
      seed = &point;
    }
  }

  if (seed == nullptr) {
    return chain;
  }

  std::vector<cv::Point> pool = yellow_points;
  cv::Point current = *seed;

  // Coordinates use flipped y for forward motion: dy > 0 means toward the
  // top of the image.
  cv::Point2d direction(0.0, 1.0);

  chain.push_back(current);
  pool.erase(std::remove(pool.begin(), pool.end(), current), pool.end());

  for (
    int step = 0;
    step < kMaxChainSteps && current.y > kTopRowMargin;
    ++step
  ) {
    std::vector<cv::Point> distance_passed;
    std::vector<cv::Point> remaining_pool;

    distance_passed.reserve(pool.size());
    remaining_pool.reserve(pool.size());

    for (const auto & point : pool) {
      const double dx = point.x - current.x;
      const double dy = current.y - point.y;
      const double distance_squared = dx * dx + dy * dy;

      if (distance_squared < kChainStepRadius * kChainStepRadius) {
        distance_passed.push_back(point);
      } else {
        remaining_pool.push_back(point);
      }
    }

    pool = std::move(remaining_pool);

    std::vector<cv::Point> candidates;
    candidates.reserve(distance_passed.size());

    for (const auto & point : distance_passed) {
      const double dx = point.x - current.x;
      const double dy = current.y - point.y;

      // Do not walk backward relative to the current chain direction.
      if (dx * direction.x + dy * direction.y >= 0.0) {
        candidates.push_back(point);
      }
    }

    if (candidates.empty()) {
      break;
    }

    // Prefer the smallest absolute angle from the current direction. The old
    // implementation omitted abs(), which could bias left and right curves
    // differently.
    std::sort(
      candidates.begin(), candidates.end(),
      [&](const cv::Point & a, const cv::Point & b) {
        const double a_dx = a.x - current.x;
        const double a_dy = current.y - a.y;
        const double b_dx = b.x - current.x;
        const double b_dy = current.y - b.y;

        const double a_distance = std::hypot(a_dx, a_dy);
        const double b_distance = std::hypot(b_dx, b_dy);

        const double a_cross = direction.x * a_dy - direction.y * a_dx;
        const double b_cross = direction.x * b_dy - direction.y * b_dx;

        const double lhs = std::abs(a_cross) * b_distance;
        const double rhs = std::abs(b_cross) * a_distance;

        if (std::abs(lhs - rhs) > 1e-9) {
          return lhs < rhs;
        }

        // If the angle is effectively the same, take the farther point so the
        // chain progresses efficiently.
        return a_distance > b_distance;
      });

    const cv::Point next = candidates.front();

    direction = cv::Point2d(
      next.x - current.x,
      current.y - next.y);

    current = next;
    chain.push_back(current);
>>>>>>> Stashed changes
  }
}

<<<<<<< Updated upstream
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

=======

LaneDetection::LaneFitResult LaneDetection::fit_lane(
  const std::vector<cv::Point> & points,
  const cv::Point2d & origin,
  int width) const
{
  LaneFitResult result;

  if (static_cast<int>(points.size()) < kMinLanePoints || width <= 0) {
    return result;
  }

  // Forward coordinate s is measured upward from the vehicle origin.
  std::vector<double> forward;
  forward.reserve(points.size());

  double min_forward = 1e9;
  double max_forward = -1e9;

  for (const auto & point : points) {
    const double s = origin.y - static_cast<double>(point.y);
    forward.push_back(s);
    min_forward = std::min(min_forward, s);
    max_forward = std::max(max_forward, s);
  }

  const double forward_span = max_forward - min_forward;

  if (forward_span < kMinForwardSpanPx) {
    return result;
  }

  // Weighted quadratic fit:
  //   x(s) = a*s^2 + b*s + c
  // Near-field points receive more weight because they are more important for
  // immediate vehicle control.
  auto solve_quadratic = [&](const std::vector<int> & indices, cv::Vec3d & coeffs) {
      if (static_cast<int>(indices.size()) < kMinLanePoints) {
        return false;
      }

      cv::Mat design(
        static_cast<int>(indices.size()), 3, CV_64F);
      cv::Mat target(
        static_cast<int>(indices.size()), 1, CV_64F);

      for (std::size_t row = 0; row < indices.size(); ++row) {
        const int index = indices[row];
        const double s = forward[index];

        const double normalized = clamp_value(
          (s - min_forward) / std::max(forward_span, 1.0),
          0.0,
          1.0);

        // 5 near the vehicle, 1 at the far end.
        const double weight = 1.0 + 4.0 * (1.0 - normalized);
        const double scale = std::sqrt(weight);

        design.at<double>(static_cast<int>(row), 0) = scale * s * s;
        design.at<double>(static_cast<int>(row), 1) = scale * s;
        design.at<double>(static_cast<int>(row), 2) = scale;
        target.at<double>(static_cast<int>(row), 0) =
          scale * static_cast<double>(points[index].x);
      }

      cv::Mat solution;

      if (!cv::solve(design, target, solution, cv::DECOMP_SVD)) {
        return false;
      }

      coeffs = cv::Vec3d(
        solution.at<double>(0, 0),
        solution.at<double>(1, 0),
        solution.at<double>(2, 0));

      return std::isfinite(coeffs[0]) &&
             std::isfinite(coeffs[1]) &&
             std::isfinite(coeffs[2]);
    };

  std::vector<int> all_indices(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    all_indices[i] = static_cast<int>(i);
  }

  cv::Vec3d coefficients;

  if (!solve_quadratic(all_indices, coefficients)) {
    return result;
  }

  auto evaluate_x = [&](double s) {
      return coefficients[0] * s * s +
             coefficients[1] * s +
             coefficients[2];
    };

  // One robust-refit pass removes isolated yellow-mask noise.
  std::vector<double> residuals;
  residuals.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    residuals.push_back(
      std::abs(static_cast<double>(points[i].x) - evaluate_x(forward[i])));
  }

  const double residual_median = median(residuals);
  const double residual_threshold = std::max(
    kMinResidualThresholdPx,
    kResidualScale * residual_median);

  std::vector<int> inlier_indices;
  inlier_indices.reserve(points.size());

  for (std::size_t i = 0; i < residuals.size(); ++i) {
    if (residuals[i] <= residual_threshold) {
      inlier_indices.push_back(static_cast<int>(i));
    }
  }

  if (static_cast<int>(inlier_indices.size()) >= kMinLanePoints) {
    cv::Vec3d robust_coefficients;

    if (solve_quadratic(inlier_indices, robust_coefficients)) {
      coefficients = robust_coefficients;
    }
  }

  const auto lane_x = [&](double s) {
      return coefficients[0] * s * s +
             coefficients[1] * s +
             coefficients[2];
    };

  const auto lane_slope = [&](double s) {
      return 2.0 * coefficients[0] * s + coefficients[1];
    };

  const auto curvature_at = [&](double s) {
      const double slope = lane_slope(s);
      const double second_derivative = 2.0 * coefficients[0];
      const double denominator = std::pow(1.0 + slope * slope, 1.5);

      if (denominator <= 0.0) {
        return 0.0;
      }

      return std::abs(second_derivative) / denominator;
    };

  // Estimate curve strength at a point between the vehicle and the far end.
  const double curvature_probe_s = clamp_value(
    min_forward + 0.35 * forward_span,
    min_forward,
    max_forward);

  const double probe_curvature = curvature_at(curvature_probe_s);
  const double probe_radius =
    probe_curvature > kMinCurvature ?
    1.0 / probe_curvature :
    kMaxCurvatureRadiusPx;

  double curve_strength = 0.0;

  if (probe_radius <= kSharpCurveRadiusThresholdPx) {
    curve_strength = 1.0;
  } else if (probe_radius < kStraightRadiusThresholdPx) {
    curve_strength =
      (kStraightRadiusThresholdPx - probe_radius) /
      (kStraightRadiusThresholdPx - kSharpCurveRadiusThresholdPx);
  }

  curve_strength = clamp_value(curve_strength, 0.0, 1.0);

  const double straight_lookahead =
    kStraightLookaheadRatio * origin.y;
  const double curve_lookahead =
    kCurveLookaheadRatio * origin.y;

  const double requested_lookahead =
    (1.0 - curve_strength) * straight_lookahead +
    curve_strength * curve_lookahead;

  const double lookahead_s = clamp_value(
    requested_lookahead,
    std::max(kMinLookaheadPx, min_forward),
    max_forward);

  const double meters_per_pixel =
    kBevWidthMeters / static_cast<double>(width);

  // Smoothly increase the distance from the right lane only in curves.
  // curve_strength: 0.0 on a straight, 1.0 on a sharp curve.
  const double target_distance_m =
    kStraightRightLaneToTargetMeters +
    curve_strength * kMaxCurveExtraOffsetMeters;

  const double target_shift_px =
    target_distance_m / meters_per_pixel;

  // The offset remains a near-field cross-track error, preserving the meaning
  // expected by the existing controller. The target path, however, is now the
  // quadratic center curve rather than a straight approximation.
  const double near_s = min_forward;
  const double center_x_near = lane_x(near_s) - target_shift_px;

  result.offset_m =
    (center_x_near - origin.x) * meters_per_pixel;

  // Use the tangent of the quadratic curve at an adaptive lookahead point.
  // Positive dx/ds means the lane moves right as it goes forward. The minus
  // sign preserves the steering-angle convention of the previous code.
  const double target_slope = lane_slope(lookahead_s);
  result.steering_angle_deg =
    -std::atan(target_slope) * 180.0 / CV_PI;

  const double target_curvature = curvature_at(lookahead_s);
  result.curvature_radius_px =
    target_curvature > kMinCurvature ?
    1.0 / target_curvature :
    kMaxCurvatureRadiusPx;

  // Draw the estimated center path, not merely the detected right boundary.
  constexpr int kCurveDrawSamples = 32;
  result.curve_points.clear();
  result.curve_points.reserve(kCurveDrawSamples);

  for (int i = 0; i < kCurveDrawSamples; ++i) {
    const double ratio =
      static_cast<double>(i) /
      static_cast<double>(kCurveDrawSamples - 1);

    const double s = min_forward + ratio * forward_span;
    const double center_x = lane_x(s) - target_shift_px;
    const double image_y = origin.y - s;

    result.curve_points.emplace_back(
      static_cast<int>(std::lround(center_x)),
      static_cast<int>(std::lround(image_y)));
  }

  result.valid =
    std::isfinite(result.offset_m) &&
    std::isfinite(result.steering_angle_deg) &&
    std::isfinite(result.curvature_radius_px);

>>>>>>> Stashed changes
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


void LaneDetection::image_callback(
  const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvShare(
      msg,
      sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & exception) {
    RCLCPP_ERROR(
      get_logger(),
      "cv_bridge exception: %s",
      exception.what());
    return;
  }

  const cv::Mat frame = cv_ptr->image;
<<<<<<< Updated upstream
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

=======
>>>>>>> Stashed changes
  const cv::Mat warped = bird_eye(frame);
  const cv::Mat binary = binary_mask(warped);

<<<<<<< Updated upstream
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
=======
  cv::Mat view = warped.clone();
  view.setTo(cv::Scalar(0, 255, 0), mask);

  const cv::Point2d origin(
    warped.cols / 2.0,
    warped.rows - 1.0);

  std::vector<cv::Point> yellow_points;
  cv::findNonZero(mask, yellow_points);

  const std::vector<cv::Point> chain =
    walk_lane_chain(yellow_points, origin);

  static int missed_frames = 0;

  std::vector<cv::Point> points;
  bool using_previous_frame = false;

  if (!chain.empty()) {
    points = chain;
    prev_points_ = chain;
    missed_frames = 0;
  } else {
    ++missed_frames;

    if (
      missed_frames <= kMaxPreviousFrameReuse &&
      !prev_points_.empty()
    ) {
      points = prev_points_;
      using_previous_frame = true;
    }
  }

  for (const auto & point : points) {
    cv::circle(
      view,
      point,
      3,
      using_previous_frame ?
      cv::Scalar(0, 120, 255) :
      cv::Scalar(0, 165, 255),
      -1);
  }

  LaneFitResult fit = fit_lane(points, origin, warped.cols);

  // Smooth valid outputs to reduce frame-to-frame steering jitter.
  static bool filter_initialized = false;
  static double filtered_offset = 0.0;
  static double filtered_angle = 0.0;
  static double filtered_curvature = 1.0 / kMaxCurvatureRadiusPx;

  if (fit.valid) {
    if (!filter_initialized) {
      filtered_offset = fit.offset_m;
      filtered_angle = fit.steering_angle_deg;
      filtered_curvature =
        1.0 / std::max(fit.curvature_radius_px, 1.0);
      filter_initialized = true;
    } else {
      filtered_offset =
        kOutputFilterAlpha * fit.offset_m +
        (1.0 - kOutputFilterAlpha) * filtered_offset;

      filtered_angle =
        kOutputFilterAlpha * fit.steering_angle_deg +
        (1.0 - kOutputFilterAlpha) * filtered_angle;

      const double measured_curvature =
        1.0 / std::max(fit.curvature_radius_px, 1.0);

      filtered_curvature =
        kOutputFilterAlpha * measured_curvature +
        (1.0 - kOutputFilterAlpha) * filtered_curvature;
    }

    fit.offset_m = filtered_offset;
    fit.steering_angle_deg = filtered_angle;
    fit.curvature_radius_px =
      filtered_curvature > kMinCurvature ?
      1.0 / filtered_curvature :
      kMaxCurvatureRadiusPx;
  } else {
    filter_initialized = false;
  }

  if (fit.valid) {
    cv::polylines(
      view,
      fit.curve_points,
      false,
      cv::Scalar(255, 0, 255),
      3);

    const cv::Point arrow_start(
      static_cast<int>(origin.x),
      static_cast<int>(origin.y));

    const cv::Point arrow_end(
      static_cast<int>(
        origin.x +
        kArrowLength *
        std::sin(fit.steering_angle_deg * CV_PI / 180.0)),
      static_cast<int>(
        origin.y -
        kArrowLength *
        std::cos(fit.steering_angle_deg * CV_PI / 180.0)));

    cv::line(
      view,
      arrow_start,
      arrow_end,
      cv::Scalar(255, 0, 0),
      2);
>>>>>>> Stashed changes
  }

  // /lane/center message layout:
  //   data[0] = center offset [m]
  //   data[1] = center heading [deg]
  //   data[2] = curvature radius [px]
  //   data[3] = valid (1.0 / 0.0)
  //   data[4:] = magenta center-path points as
  //              [forward_m, left_m, forward_m, left_m, ...]
  //
  // Vehicle-local convention:
  //   forward_m > 0 : ahead of the vehicle
  //   left_m    > 0 : left of the vehicle
  //
  // The first four fields remain unchanged, so old subscribers remain
  // compatible. The updated controller consumes the extra point pairs and
  // directly follows the magenta center path using local Pure Pursuit.
  std_msgs::msg::Float64MultiArray lane_msg;
<<<<<<< Updated upstream
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
=======
  lane_msg.data.reserve(4 + fit.curve_points.size() * 2);

  lane_msg.data.push_back(fit.offset_m);
  lane_msg.data.push_back(fit.steering_angle_deg);
  lane_msg.data.push_back(fit.curvature_radius_px);
  lane_msg.data.push_back(fit.valid ? 1.0 : 0.0);

  if (fit.valid && warped.cols > 0) {
    // The existing BEV approximation uses one common scale for both image
    // axes. For accurate metric tracking this value should eventually be
    // replaced by a measured BEV calibration.
    const double meters_per_pixel =
      kBevWidthMeters / static_cast<double>(warped.cols);

    for (const auto & point : fit.curve_points) {
      const double forward_m =
        (origin.y - static_cast<double>(point.y)) * meters_per_pixel;

      const double left_m =
        (origin.x - static_cast<double>(point.x)) * meters_per_pixel;

      if (
        forward_m > 0.0 &&
        std::isfinite(forward_m) &&
        std::isfinite(left_m)
      ) {
        lane_msg.data.push_back(forward_m);
        lane_msg.data.push_back(left_m);
      }
    }
  }

  lane_center_publisher_->publish(lane_msg);

  const cv::Scalar text_color =
    fit.valid ?
    cv::Scalar(255, 255, 255) :
    cv::Scalar(0, 0, 255);

  char offset_text[96];
  std::snprintf(
    offset_text,
    sizeof(offset_text),
    "Center offset: %.2f m",
    fit.offset_m);

  cv::putText(
    view,
    offset_text,
    cv::Point(30, 30),
    cv::FONT_HERSHEY_SIMPLEX,
    0.8,
    text_color,
    2);

  char angle_text[96];
  std::snprintf(
    angle_text,
    sizeof(angle_text),
    "Curve heading: %.2f deg",
    fit.steering_angle_deg);

  cv::putText(
    view,
    angle_text,
    cv::Point(30, 65),
    cv::FONT_HERSHEY_SIMPLEX,
    0.8,
    text_color,
    2);

  char radius_text[96];
  std::snprintf(
    radius_text,
    sizeof(radius_text),
    "Radius: %.1f px",
    fit.curvature_radius_px);

  cv::putText(
    view,
    radius_text,
    cv::Point(30, 100),
    cv::FONT_HERSHEY_SIMPLEX,
    0.8,
    text_color,
    2);

  if (using_previous_frame && fit.valid) {
    cv::putText(
      view,
      "Using previous lane (temporary)",
      cv::Point(30, 135),
      cv::FONT_HERSHEY_SIMPLEX,
      0.7,
      cv::Scalar(0, 200, 255),
      2);
  }

  if (!fit.valid) {
    cv::putText(
      view,
      "Lane center unavailable",
      cv::Point(30, 135),
      cv::FONT_HERSHEY_SIMPLEX,
      0.8,
      text_color,
      2);
  }
>>>>>>> Stashed changes

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