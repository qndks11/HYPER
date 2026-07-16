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
// Perspective-transform settings retained from the original lane-detection node.
constexpr double kRoiTopLeftRow = 0.40;
constexpr double kRoiTopLeftCol = 0.25;
constexpr double kRoiTopRightRow = 0.40;
constexpr double kRoiTopRightCol = 0.75;
constexpr double kRoiBottomLeftRow = 1.0;
constexpr double kRoiBottomLeftCol = -1.4;
constexpr double kRoiBottomRightRow = 1.0;
constexpr double kRoiBottomRightCol = 2.4;
constexpr double kBevHeightScale = 1.35;

// Lane-chain detection: both left and right lanes are tracked separately.
constexpr double kChainStepRadius = 30.0;
constexpr int kMaxChainSteps = 50;
constexpr double kTopRowMargin = 20.0;

// Robust quadratic fit x(s) = a*s^2 + b*s + c.
constexpr int kMinLanePoints = 6;
constexpr double kMinForwardSpanPx = 60.0;
constexpr double kMinResidualThresholdPx = 8.0;
constexpr double kResidualScale = 2.5;

constexpr double kMaxCurvatureRadiusPx = 1e6;
constexpr double kMinCurvature = 1e-6;

// Lane metric calibration inherited from the second lane-detection code.
// Tune this value after measuring the actual BEV width represented by the image.
constexpr double kLaneBevWidthMeters = 3.7;

// Single-side fallback. When only one boundary is visible, move from that
// boundary toward the lane center by this amount. Curves add a small margin.
constexpr double kStraightBoundaryToTargetMeters = 0.60;
constexpr double kMaxCurveExtraOffsetMeters = 0.05;

// Adaptive lookahead from the quadratic lane implementation.
constexpr double kStraightLookaheadRatio = 0.45;
constexpr double kCurveLookaheadRatio = 0.22;
constexpr double kMinLookaheadPx = 35.0;
constexpr double kStraightRadiusThresholdPx = 1400.0;
constexpr double kSharpCurveRadiusThresholdPx = 280.0;

// Only bridge very short camera dropouts; do not reuse an old lane indefinitely.
constexpr int kMaxPreviousFrameReuse = 2;

// Low-pass filter for published offset, heading and curvature.
constexpr double kOutputFilterAlpha = 0.35;

// Original dual-lane plausibility check and stop-line metric calibration.
constexpr double kLaneWidthMeters = 3.7;
constexpr double kNumLaneInScreen = 3.2;

// 두 차선을 실제 한 차로의 좌/우 경계로 인정하기 위한 엄격한 기준.
// 기준을 통과하지 못하면 왼쪽 선은 무시하고 오른쪽 선을 사용한다.
constexpr double kMinPlausibleLaneWidthMeters = kLaneWidthMeters * 0.65;
constexpr double kMaxPlausibleLaneWidthMeters = kLaneWidthMeters * 1.35;
constexpr double kMaxLaneWidthVariationMeters = kLaneWidthMeters * 0.25;
constexpr double kMaxBoundaryHeadingDifferenceDeg = 15.0;
constexpr double kMinDualLaneCommonSpanPx = 100.0;
constexpr int kDualLaneValidationSamples = 5;

// Original stop-line detector settings.
constexpr double kMinStoplineAspectRatio = 3.0;
constexpr double kMinStoplineWidthFraction = 0.25;
constexpr double kMinStoplineAreaPx = 500.0;

constexpr int kCurveDrawSamples = 32;
constexpr double kArrowLength = 100.0;
constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;


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


bool is_spurious_cross_lane(
  const std::vector<cv::Point> & chain,
  const cv::Point2d & origin)
{
  if (chain.size() < 2) {
    return false;
  }

  const bool start_left = chain.front().x < origin.x;
  const bool end_left = chain.back().x < origin.x;

  if (start_left == end_left) {
    return false;
  }

  // A genuine curved lane gets farther from the vehicle while crossing the
  // image center. A chain that crosses without moving forward is likely the
  // same physical line claimed by both seeds.
  return chain.front().y <= chain.back().y;
}


double model_x(const cv::Vec3d & coefficients, double forward_px)
{
  return coefficients[0] * forward_px * forward_px +
         coefficients[1] * forward_px +
         coefficients[2];
}


double model_slope(const cv::Vec3d & coefficients, double forward_px)
{
  return 2.0 * coefficients[0] * forward_px + coefficients[1];
}


double model_curvature(const cv::Vec3d & coefficients, double forward_px)
{
  const double slope = model_slope(coefficients, forward_px);
  const double second_derivative = 2.0 * coefficients[0];
  const double denominator = std::pow(1.0 + slope * slope, 1.5);

  if (denominator <= 0.0) {
    return 0.0;
  }

  return std::abs(second_derivative) / denominator;
}


double curve_strength_for_model(
  const cv::Vec3d & coefficients,
  double min_forward_px,
  double max_forward_px)
{
  const double probe_forward = clamp_value(
    min_forward_px + 0.35 * (max_forward_px - min_forward_px),
    min_forward_px,
    max_forward_px);

  const double curvature = model_curvature(coefficients, probe_forward);
  const double radius =
    curvature > kMinCurvature ? 1.0 / curvature : kMaxCurvatureRadiusPx;

  if (radius <= kSharpCurveRadiusThresholdPx) {
    return 1.0;
  }

  if (radius >= kStraightRadiusThresholdPx) {
    return 0.0;
  }

  return clamp_value(
    (kStraightRadiusThresholdPx - radius) /
    (kStraightRadiusThresholdPx - kSharpCurveRadiusThresholdPx),
    0.0,
    1.0);
}


// 현재 프레임의 두 2차 곡선이 실제 한 차로의 좌/우 경계인지 검사한다.
// 폭, 폭의 일관성, 두 경계의 진행 방향, 공통 검출 구간을 모두 검사한다.
bool is_clear_dual_lane_pair(
  const cv::Vec3d & left_coefficients,
  const cv::Vec3d & right_coefficients,
  double common_min_forward,
  double common_max_forward,
  double meters_per_pixel)
{
  if (
    !std::isfinite(common_min_forward) ||
    !std::isfinite(common_max_forward) ||
    common_max_forward - common_min_forward < kMinDualLaneCommonSpanPx)
  {
    return false;
  }

  double min_width_m = 1e9;
  double max_width_m = -1e9;
  double width_sum_m = 0.0;
  double max_heading_difference_deg = 0.0;

  for (int i = 0; i < kDualLaneValidationSamples; ++i) {
    const double ratio =
      kDualLaneValidationSamples <= 1 ?
      0.0 :
      static_cast<double>(i) /
      static_cast<double>(kDualLaneValidationSamples - 1);

    const double forward_px =
      common_min_forward +
      ratio * (common_max_forward - common_min_forward);

    const double left_x = model_x(left_coefficients, forward_px);
    const double right_x = model_x(right_coefficients, forward_px);

    if (
      !std::isfinite(left_x) ||
      !std::isfinite(right_x) ||
      right_x <= left_x)
    {
      return false;
    }

    const double width_m =
      (right_x - left_x) * meters_per_pixel;

    if (
      !std::isfinite(width_m) ||
      width_m < kMinPlausibleLaneWidthMeters ||
      width_m > kMaxPlausibleLaneWidthMeters)
    {
      return false;
    }

    min_width_m = std::min(min_width_m, width_m);
    max_width_m = std::max(max_width_m, width_m);
    width_sum_m += width_m;

    const double left_heading =
      std::atan(model_slope(left_coefficients, forward_px));

    const double right_heading =
      std::atan(model_slope(right_coefficients, forward_px));

    const double heading_difference_deg =
      std::abs(left_heading - right_heading) * 180.0 / CV_PI;

    max_heading_difference_deg = std::max(
      max_heading_difference_deg,
      heading_difference_deg);
  }

  const double average_width_m =
    width_sum_m / static_cast<double>(kDualLaneValidationSamples);

  return
    average_width_m >= kMinPlausibleLaneWidthMeters &&
    average_width_m <= kMaxPlausibleLaneWidthMeters &&
    max_width_m - min_width_m <= kMaxLaneWidthVariationMeters &&
    max_heading_difference_deg <= kMaxBoundaryHeadingDifferenceDeg;
}

}  // namespace


LaneDetection::LaneDetection() : Node{"lane_detection"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10,
    std::bind(&LaneDetection::image_callback, this, std::placeholders::_1));

  lane_center_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);

  stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/detection", 10);

  cv::namedWindow("Lane Detection", cv::WINDOW_NORMAL);
  cv::resizeWindow(
    "Lane Detection", kWindowWidth,
    static_cast<int>(kWindowHeight * kBevHeightScale));

  RCLCPP_INFO(
    get_logger(),
    "LaneDetection started (right-priority lane tracking + clear dual-lane center + stop-line)");
}


cv::Mat LaneDetection::build_transform(
  int src_height, int src_width,
  int dst_height, int dst_width) const
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

  return cv::getPerspectiveTransform(src, dst);
}


cv::Mat LaneDetection::bird_eye(const cv::Mat & image) const
{
  const cv::Size dst_size(
    image.cols,
    static_cast<int>(image.rows * kBevHeightScale));

  const cv::Mat transform = build_transform(
    image.rows, image.cols,
    dst_size.height, dst_size.width);

  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, dst_size);
  return warped;
}


cv::Mat LaneDetection::yellow_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  // Keep the original lane-detection node's tighter yellow threshold.
  cv::inRange(
    hsv,
    cv::Scalar(22, 80, 80),
    cv::Scalar(35, 255, 255),
    mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  return mask;
}


cv::Mat LaneDetection::white_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(
    hsv,
    cv::Scalar(0, 0, 180),
    cv::Scalar(180, 60, 255),
    mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  return mask;
}


std::vector<cv::Point> LaneDetection::walk_lane_chain(
  const std::vector<cv::Point> & yellow_points,
  const cv::Point2d & origin,
  LaneSide side) const
{
  std::vector<cv::Point> chain;

  if (yellow_points.empty()) {
    return chain;
  }

  // Select the bottommost pixel on the requested side as the chain seed.
  const cv::Point * seed = nullptr;

  for (const auto & point : yellow_points) {
    const bool wrong_side =
      side == LaneSide::kRight ?
      point.x < origin.x :
      point.x >= origin.x;

    if (wrong_side) {
      continue;
    }

    if (
      seed == nullptr ||
      point.y > seed->y ||
      (
        point.y == seed->y &&
        std::abs(point.x - origin.x) < std::abs(seed->x - origin.x)
      ))
    {
      seed = &point;
    }
  }

  if (seed == nullptr) {
    return chain;
  }

  std::vector<cv::Point> pool = yellow_points;
  cv::Point current = *seed;
  cv::Point2d direction(0.0, 1.0);

  chain.push_back(current);
  pool.erase(std::remove(pool.begin(), pool.end(), current), pool.end());

  for (
    int step = 0;
    step < kMaxChainSteps && current.y > kTopRowMargin;
    ++step)
  {
    std::vector<cv::Point> distance_passed;
    std::vector<cv::Point> remaining_pool;

    distance_passed.reserve(pool.size());
    remaining_pool.reserve(pool.size());

    for (const auto & point : pool) {
      const double dx = point.x - current.x;
      const double dy = current.y - point.y;

      if (dx * dx + dy * dy < kChainStepRadius * kChainStepRadius) {
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

      if (dx * direction.x + dy * direction.y >= 0.0) {
        candidates.push_back(point);
      }
    }

    if (candidates.empty()) {
      break;
    }

    // Preserve the original dual-side chain ordering: the preferred turn
    // direction is mirrored between the right and left boundaries.
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

        const double lhs = a_cross * b_distance;
        const double rhs = b_cross * a_distance;

        if (std::abs(lhs - rhs) > 1e-9) {
          return side == LaneSide::kRight ? lhs > rhs : lhs < rhs;
        }

        return a_distance > b_distance;
      });

    const cv::Point next = candidates.front();
    direction = cv::Point2d(next.x - current.x, current.y - next.y);
    current = next;
    chain.push_back(current);
  }

  return chain;
}


LaneDetection::QuadraticLaneModel LaneDetection::fit_lane_model(
  const std::vector<cv::Point> & points,
  const cv::Point2d & origin) const
{
  QuadraticLaneModel model;

  if (static_cast<int>(points.size()) < kMinLanePoints) {
    return model;
  }

  std::vector<double> forward;
  forward.reserve(points.size());

  double min_forward = 1e9;
  double max_forward = -1e9;

  for (const auto & point : points) {
    const double value = origin.y - static_cast<double>(point.y);
    forward.push_back(value);
    min_forward = std::min(min_forward, value);
    max_forward = std::max(max_forward, value);
  }

  const double forward_span = max_forward - min_forward;

  if (forward_span < kMinForwardSpanPx) {
    return model;
  }

  auto solve_quadratic =
    [&](const std::vector<int> & indices, cv::Vec3d & coefficients) {
      if (static_cast<int>(indices.size()) < kMinLanePoints) {
        return false;
      }

      cv::Mat design(static_cast<int>(indices.size()), 3, CV_64F);
      cv::Mat target(static_cast<int>(indices.size()), 1, CV_64F);

      for (std::size_t row = 0; row < indices.size(); ++row) {
        const int index = indices[row];
        const double s = forward[index];

        const double normalized = clamp_value(
          (s - min_forward) / std::max(forward_span, 1.0),
          0.0,
          1.0);

        // Near-field points matter more for immediate steering control.
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

      coefficients = cv::Vec3d(
        solution.at<double>(0, 0),
        solution.at<double>(1, 0),
        solution.at<double>(2, 0));

      return std::isfinite(coefficients[0]) &&
             std::isfinite(coefficients[1]) &&
             std::isfinite(coefficients[2]);
    };

  std::vector<int> all_indices(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    all_indices[i] = static_cast<int>(i);
  }

  cv::Vec3d coefficients;

  if (!solve_quadratic(all_indices, coefficients)) {
    return model;
  }

  // One robust-refit pass rejects isolated mask noise.
  std::vector<double> residuals;
  residuals.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    residuals.push_back(
      std::abs(
        static_cast<double>(points[i].x) -
        model_x(coefficients, forward[i])));
  }

  const double residual_threshold = std::max(
    kMinResidualThresholdPx,
    kResidualScale * median(residuals));

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

  model.valid = true;
  model.coefficients = coefficients;
  model.min_forward_px = min_forward;
  model.max_forward_px = max_forward;
  return model;
}


LaneDetection::LaneFitResult LaneDetection::build_center_path(
  const QuadraticLaneModel & left_model,
  const QuadraticLaneModel & right_model,
  const std::vector<cv::Point> & left_points,
  const std::vector<cv::Point> & right_points,
  const cv::Point2d & origin,
  int width,
  bool allow_dual_lane_center) const
{
  LaneFitResult result;

  (void)left_points;
  (void)right_points;

  // 오른쪽 경계가 없으면 왼쪽 경계만으로는 경로를 만들지 않는다.
  // 단, 오른쪽 경계는 최대 kMaxPreviousFrameReuse 프레임까지 재사용될 수 있다.
  if (width <= 0 || !right_model.valid) {
    return result;
  }

  const double lane_meters_per_pixel =
    kLaneBevWidthMeters / static_cast<double>(width);

  const double lane_width_check_meters_per_pixel =
    kNumLaneInScreen * kLaneWidthMeters / static_cast<double>(width);

  cv::Vec3d center_coefficients{0.0, 0.0, 0.0};
  double min_forward = 0.0;
  double max_forward = 0.0;
  bool center_model_ready = false;

  // 양쪽 차선 중앙은 다음 조건을 모두 만족할 때만 사용한다.
  // 1) 왼쪽/오른쪽 모두 현재 프레임 검출값
  // 2) 두 곡선의 공통 전방 구간이 충분함
  // 3) 여러 지점에서 차선 폭이 정상적이고 일정함
  // 4) 두 경계의 진행 방향 차이가 작음
  if (
    allow_dual_lane_center &&
    left_model.valid &&
    right_model.valid)
  {
    const double common_min = std::max(
      left_model.min_forward_px,
      right_model.min_forward_px);

    const double common_max = std::min(
      left_model.max_forward_px,
      right_model.max_forward_px);

    const bool clear_pair = is_clear_dual_lane_pair(
      left_model.coefficients,
      right_model.coefficients,
      common_min,
      common_max,
      lane_width_check_meters_per_pixel);

    if (clear_pair) {
      center_coefficients =
        0.5 * (left_model.coefficients + right_model.coefficients);

      min_forward = common_min;
      max_forward = common_max;
      center_model_ready = true;
      result.used_dual_lane_center = true;
    }
  }

  // 두 차선이 명확하지 않으면 무조건 오른쪽 차선 기준으로 추종한다.
  if (!center_model_ready) {
    center_coefficients = right_model.coefficients;
    min_forward = right_model.min_forward_px;
    max_forward = right_model.max_forward_px;

    const double curve_strength = curve_strength_for_model(
      right_model.coefficients,
      min_forward,
      max_forward);

    const double target_distance_m =
      kStraightBoundaryToTargetMeters +
      curve_strength * kMaxCurveExtraOffsetMeters;

    const double target_shift_px =
      target_distance_m / lane_meters_per_pixel;

    // 이미지 x는 오른쪽으로 증가한다.
    // 오른쪽 경계에서 차로 중앙으로 이동하려면 왼쪽 방향으로 이동한다.
    center_coefficients[2] -= target_shift_px;

    center_model_ready = true;
    result.used_right_boundary = true;
  }

  if (!center_model_ready || max_forward - min_forward < kMinForwardSpanPx) {
    return result;
  }

  const double curve_strength = curve_strength_for_model(
    center_coefficients,
    min_forward,
    max_forward);

  const double straight_lookahead =
    kStraightLookaheadRatio * origin.y;

  const double curve_lookahead =
    kCurveLookaheadRatio * origin.y;

  const double requested_lookahead =
    (1.0 - curve_strength) * straight_lookahead +
    curve_strength * curve_lookahead;

  const double lookahead_forward = clamp_value(
    requested_lookahead,
    std::max(kMinLookaheadPx, min_forward),
    max_forward);

  const double center_x_near =
    model_x(center_coefficients, min_forward);

  result.offset_m =
    (center_x_near - origin.x) * lane_meters_per_pixel;

  result.steering_angle_deg =
    -std::atan(
      model_slope(center_coefficients, lookahead_forward)) *
    180.0 / CV_PI;

  const double curvature =
    model_curvature(center_coefficients, lookahead_forward);

  result.curvature_radius_px =
    curvature > kMinCurvature ?
    1.0 / curvature :
    kMaxCurvatureRadiusPx;

  result.curve_points.clear();
  result.curve_points.reserve(kCurveDrawSamples);

  for (int i = 0; i < kCurveDrawSamples; ++i) {
    const double ratio =
      static_cast<double>(i) /
      static_cast<double>(kCurveDrawSamples - 1);

    const double forward_px =
      min_forward +
      ratio * (max_forward - min_forward);

    result.curve_points.emplace_back(
      static_cast<int>(
        std::lround(model_x(center_coefficients, forward_px))),
      static_cast<int>(
        std::lround(origin.y - forward_px)));
  }

  result.valid =
    std::isfinite(result.offset_m) &&
    std::isfinite(result.steering_angle_deg) &&
    std::isfinite(result.curvature_radius_px) &&
    !result.curve_points.empty();

  return result;
}


LaneDetection::StoplineResult LaneDetection::find_stopline(
  const cv::Mat & mask,
  const cv::Point2d & origin,
  double meters_per_pixel) const
{
  StoplineResult result;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  const double min_width_px = mask.cols * kMinStoplineWidthFraction;
  const double lane_width_px = mask.cols / kNumLaneInScreen;
  const double band_min_x = origin.x - 1.2 * lane_width_px / 2.0;
  const double band_max_x = origin.x + 1.2 * lane_width_px / 2.0;

  bool found = false;
  cv::Rect best_box;

  for (const auto & contour : contours) {
    if (cv::contourArea(contour) < kMinStoplineAreaPx) {
      continue;
    }

    const cv::Rect box = cv::boundingRect(contour);
    const double aspect_ratio =
      static_cast<double>(box.width) / std::max(box.height, 1);

    if (aspect_ratio < kMinStoplineAspectRatio || box.width < min_width_px) {
      continue;
    }

    const double box_center_x = box.x + box.width / 2.0;

    if (box_center_x < band_min_x || box_center_x > band_max_x) {
      continue;
    }

    // The closest valid stop line governs the vehicle.
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

  const double stopline_row = best_box.y + best_box.height;
  result.distance_m = (origin.y - stopline_row) * meters_per_pixel;
  return result;
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

  const cv::Mat warped = bird_eye(cv_ptr->image);
  const cv::Mat yellow = yellow_mask(warped);
  const cv::Mat white = white_mask(warped);

  const cv::Point2d origin(
    warped.cols / 2.0,
    warped.rows - 1.0);

  cv::Mat view = warped.clone();
  view.setTo(cv::Scalar(0, 255, 0), yellow);
  view.setTo(cv::Scalar(180, 180, 180), white);

  // ------------------------------------------------------------------
  // 오른쪽 차선 우선 검출:
  // - 두 경계가 현재 프레임에서 모두 명확할 때만 중앙 경로 사용
  // - 그 외에는 오른쪽 경계를 기준으로 목표 경로 생성
  // ------------------------------------------------------------------
  std::vector<cv::Point> yellow_points;
  cv::findNonZero(yellow, yellow_points);

  std::vector<cv::Point> right_chain =
    walk_lane_chain(
      yellow_points,
      origin,
      LaneSide::kRight);

  std::vector<cv::Point> left_chain =
    walk_lane_chain(
      yellow_points,
      origin,
      LaneSide::kLeft);

  if (is_spurious_cross_lane(right_chain, origin)) {
    right_chain.clear();
  }

  if (is_spurious_cross_lane(left_chain, origin)) {
    left_chain.clear();
  }

  const bool right_detected_current = !right_chain.empty();
  const bool left_detected_current = !left_chain.empty();

  std::vector<cv::Point> right_points;
  bool using_previous_right = false;

  if (right_detected_current) {
    right_points = right_chain;
    prev_right_points_ = right_chain;
    right_missed_frames_ = 0;
  } else {
    ++right_missed_frames_;

    if (
      right_missed_frames_ <= kMaxPreviousFrameReuse &&
      !prev_right_points_.empty())
    {
      right_points = prev_right_points_;
      using_previous_right = true;
    }
  }

  // 왼쪽 차선은 현재 프레임 값만 사용한다.
  // 이전 왼쪽 차선을 양쪽 중앙 계산에 섞으면 실제로 두 선이 보이지 않는
  // 상황에서도 잘못된 중앙 경로가 생성될 수 있기 때문이다.
  const std::vector<cv::Point> left_points = left_chain;

  for (const auto & point : right_points) {
    cv::circle(
      view,
      point,
      3,
      using_previous_right ?
      cv::Scalar(0, 120, 255) :
      cv::Scalar(0, 165, 255),
      -1);
  }

  for (const auto & point : left_points) {
    cv::circle(
      view,
      point,
      3,
      cv::Scalar(255, 0, 0),
      -1);
  }

  const QuadraticLaneModel right_model =
    fit_lane_model(right_points, origin);

  const QuadraticLaneModel left_model =
    fit_lane_model(left_points, origin);

  // 이전 프레임 차선이 하나라도 섞였으면 양쪽 차선 중앙을 사용하지 않는다.
  const bool allow_dual_lane_center =
    right_detected_current &&
    left_detected_current &&
    !using_previous_right;

  LaneFitResult fit = build_center_path(
    left_model,
    right_model,
    left_points,
    right_points,
    origin,
    warped.cols,
    allow_dual_lane_center);

  // Same scalar output filter as the second lane-detection implementation.
  if (fit.valid) {
    if (!filter_initialized_) {
      filtered_offset_ = fit.offset_m;
      filtered_angle_ = fit.steering_angle_deg;
      filtered_curvature_ =
        1.0 / std::max(fit.curvature_radius_px, 1.0);
      filter_initialized_ = true;
    } else {
      filtered_offset_ =
        kOutputFilterAlpha * fit.offset_m +
        (1.0 - kOutputFilterAlpha) * filtered_offset_;

      filtered_angle_ =
        kOutputFilterAlpha * fit.steering_angle_deg +
        (1.0 - kOutputFilterAlpha) * filtered_angle_;

      const double measured_curvature =
        1.0 / std::max(fit.curvature_radius_px, 1.0);

      filtered_curvature_ =
        kOutputFilterAlpha * measured_curvature +
        (1.0 - kOutputFilterAlpha) * filtered_curvature_;
    }

    fit.offset_m = filtered_offset_;
    fit.steering_angle_deg = filtered_angle_;
    fit.curvature_radius_px =
      filtered_curvature_ > kMinCurvature ?
      1.0 / filtered_curvature_ :
      kMaxCurvatureRadiusPx;
  } else {
    filter_initialized_ = false;
  }

  if (fit.valid) {
    // Magenta is the actual center path published for the controller.
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
  }

  // /lane/center:
  // data[0] = center offset [m]
  // data[1] = center heading [deg]
  // data[2] = curvature radius [px]
  // data[3] = valid (1.0 / 0.0)
  // data[4:] = center-path pairs [forward_m, left_m, ...]
  std_msgs::msg::Float64MultiArray lane_msg;
  lane_msg.data.reserve(4 + fit.curve_points.size() * 2);

  lane_msg.data.push_back(fit.offset_m);
  lane_msg.data.push_back(fit.steering_angle_deg);
  lane_msg.data.push_back(fit.curvature_radius_px);
  lane_msg.data.push_back(fit.valid ? 1.0 : 0.0);

  if (fit.valid && warped.cols > 0) {
    const double meters_per_pixel =
      kLaneBevWidthMeters / static_cast<double>(warped.cols);

    for (const auto & point : fit.curve_points) {
      const double forward_m =
        (origin.y - static_cast<double>(point.y)) * meters_per_pixel;

      const double left_m =
        (origin.x - static_cast<double>(point.x)) * meters_per_pixel;

      if (
        forward_m > 0.0 &&
        std::isfinite(forward_m) &&
        std::isfinite(left_m))
      {
        lane_msg.data.push_back(forward_m);
        lane_msg.data.push_back(left_m);
      }
    }
  }

  lane_center_publisher_->publish(lane_msg);

  const cv::Scalar lane_text_color =
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
    lane_text_color,
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
    lane_text_color,
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
    lane_text_color,
    2);

  char sides_text[96];
  std::snprintf(
    sides_text,
    sizeof(sides_text),
    "L: %s  R: %s%s",
    left_model.valid && left_detected_current ? "OK" : "--",
    right_model.valid ? "OK" : "--",
    using_previous_right ? "(prev)" : "");
  cv::putText(
    view,
    sides_text,
    cv::Point(30, 135),
    cv::FONT_HERSHEY_SIMPLEX,
    0.7,
    lane_text_color,
    2);

  const char * tracking_mode = "NO RIGHT LANE";

  if (fit.used_dual_lane_center) {
    tracking_mode = "DUAL-LANE CENTER";
  } else if (fit.used_right_boundary) {
    tracking_mode = using_previous_right ?
      "RIGHT LANE (PREVIOUS)" :
      "RIGHT-LANE PRIORITY";
  }

  cv::putText(
    view,
    tracking_mode,
    cv::Point(30, 170),
    cv::FONT_HERSHEY_SIMPLEX,
    0.7,
    lane_text_color,
    2);

  if (!fit.valid) {
    cv::putText(
      view,
      "Lane center unavailable",
      cv::Point(30, 205),
      cv::FONT_HERSHEY_SIMPLEX,
      0.8,
      lane_text_color,
      2);
  }

  // ------------------------------------------------------------------
  // Original stop-line detector retained unchanged in behavior.
  // ------------------------------------------------------------------
  const double stopline_meters_per_pixel =
    kNumLaneInScreen * kLaneWidthMeters /
    static_cast<double>(warped.cols);

  const StoplineResult stopline =
    find_stopline(white, origin, stopline_meters_per_pixel);

  if (stopline.valid) {
    cv::rectangle(
      view,
      stopline.bounding_box,
      cv::Scalar(0, 0, 255),
      3);
  }

  std_msgs::msg::Float64MultiArray stopline_msg;
  stopline_msg.data = {
    stopline.distance_m,
    stopline.valid ? 1.0 : 0.0
  };
  stopline_publisher_->publish(stopline_msg);

  const cv::Scalar stopline_text_color =
    stopline.valid ?
    cv::Scalar(255, 255, 255) :
    cv::Scalar(0, 0, 255);

  char distance_text[96];
  std::snprintf(
    distance_text,
    sizeof(distance_text),
    "Stopline distance: %.2f m",
    stopline.distance_m);
  cv::putText(
    view,
    distance_text,
    cv::Point(30, view.rows - 60),
    cv::FONT_HERSHEY_SIMPLEX,
    0.8,
    stopline_text_color,
    2);

  if (!stopline.valid) {
    cv::putText(
      view,
      "Stopline not detected",
      cv::Point(30, view.rows - 20),
      cv::FONT_HERSHEY_SIMPLEX,
      0.8,
      stopline_text_color,
      2);
  }

  cv::imshow("Lane Detection", view);
  cv::waitKey(1);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetection>());
  rclcpp::shutdown();
  return 0;
}