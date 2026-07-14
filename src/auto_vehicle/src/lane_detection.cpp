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

constexpr double kArrowLength = 100.0;
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

}  // namespace


LaneDetection::LaneDetection() : Node{"lane_detection"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10,
    std::bind(&LaneDetection::image_callback, this, std::placeholders::_1));

  lane_center_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);

  cv::namedWindow("Lane Detection", cv::WINDOW_NORMAL);
  cv::resizeWindow(
    "Lane Detection", kWindowWidth,
    static_cast<int>(kWindowHeight * kBevHeightScale));

  RCLCPP_INFO(get_logger(), "LaneDetection started (quadratic curve fit)");
}


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

  return cv::getPerspectiveTransform(src, dst);
}


cv::Mat LaneDetection::bird_eye(const cv::Mat & image) const
{
  const cv::Size dst_size(
    image.cols,
    static_cast<int>(image.rows * kBevHeightScale));

  const cv::Mat transform = build_transform(
    image.rows, image.cols, dst_size.height, dst_size.width);

  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, dst_size);
  return warped;
}


cv::Mat LaneDetection::yellow_mask(const cv::Mat & image) const
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
  }

  return chain;
}


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

  const cv::Mat frame = cv_ptr->image;
  const cv::Mat warped = bird_eye(frame);
  const cv::Mat mask = yellow_mask(warped);

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