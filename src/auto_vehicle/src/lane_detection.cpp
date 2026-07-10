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
constexpr double kRoiBottomLeftRow = 1,  kRoiBottomLeftCol = -1.4;
constexpr double kRoiBottomRightRow = 1, kRoiBottomRightCol = 2.4;

// Neighborhood radius for each step of the lane chain walk [px]. Also sets the rough spacing
// between consecutive chain points, since each step prefers the farthest qualifying pixel within
// this radius.
constexpr double kChainStepRadius = 30.0;
// Safety cap on chain length. Real chains are bounded by image size / kChainStepRadius (tens of
// steps); this just guards against runaway loops.
constexpr int kMaxChainSteps = 50;
// Row [px] near the top of the BEV image at which the chain walk stops, since rows this close to
// the horizon are unreliable and there's nowhere further to walk to anyway.
constexpr double kTopRowMargin = 20.0;

// A chain point whose turn angle (between the segment entering it and the segment leaving it)
// exceeds this is treated as an outlier -- e.g. the walk jumping onto a stray yellow pixel off
// the true lane -- and pruned along with kSharpTurnPruneRadius neighbors on each side.
constexpr double kSharpTurnAngleDeg = 60.0;
constexpr int kSharpTurnPruneRadius = 5;

constexpr int kMinLanePoints = 6;   // a few points of slack past the 4 a cubic needs exactly
constexpr int kCurveSamples = 40;   // density of the drawn/unwarped curve, not the fit itself
constexpr double kMinCurvatureDenominator = 1e-6;
// Floor on |dy/ds| at the chain's near end when inverting the tangent to locate the vehicle's
// row (see fit_lane): below this the near end is running ~horizontal, so that row can't be
// reached by extrapolating along the curve and the near point's x is used directly instead.
constexpr double kMinNearTangentDy = 1e-6;

constexpr double kLaneWidthMeters = 3.7;
constexpr double kArrowLength = 100.0;

constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;

// Bird's-eye output height as a multiple of the source frame height, i.e. how much farther
// down the road the BEV view looks. Width is left unscaled.
constexpr double kBevHeightScale = 1.4;

// Fits v = a*t^3 + b*t^2 + c*t + d to the given (t, v) pairs by weighted least squares. Used
// twice per lane fit -- once for x(s), once for y(s), both against the same arc-length
// parameter t=s and weight -- so it takes t/v/weight as plain vectors rather than cv::Points.
cv::Vec4d fit_cubic(
  const std::vector<double> & t, const std::vector<double> & v,
  const std::vector<double> & weight)
{
  const int n = static_cast<int>(t.size());
  cv::Mat A(n, 4, CV_64F);
  cv::Mat b(n, 1, CV_64F);

  for (int i = 0; i < n; ++i) {
    const double w = weight[i];
    A.at<double>(i, 0) = w * t[i] * t[i] * t[i];
    A.at<double>(i, 1) = w * t[i] * t[i];
    A.at<double>(i, 2) = w * t[i];
    A.at<double>(i, 3) = w;
    b.at<double>(i, 0) = w * v[i];
  }

  cv::Mat coeffs;
  cv::solve(A, b, coeffs, cv::DECOMP_SVD);
  return cv::Vec4d(
    coeffs.at<double>(0), coeffs.at<double>(1), coeffs.at<double>(2), coeffs.at<double>(3));
}

double eval_cubic(const cv::Vec4d & fit, double t)
{
  return fit[0] * t * t * t + fit[1] * t * t + fit[2] * t + fit[3];
}

double eval_cubic_first_derivative(const cv::Vec4d & fit, double t)
{
  return 3.0 * fit[0] * t * t + 2.0 * fit[1] * t + fit[2];
}

double eval_cubic_second_derivative(const cv::Vec4d & fit, double t)
{
  return 6.0 * fit[0] * t + 2.0 * fit[1];
}

}  // namespace

LaneDetection::LaneDetection() : Node{"lane_detection"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&LaneDetection::image_callback, this, std::placeholders::_1));

  lane_center_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);

  cv::namedWindow("Lane Detection", cv::WINDOW_NORMAL);
  cv::resizeWindow(
    "Lane Detection", kWindowWidth, static_cast<int>(kWindowHeight * kBevHeightScale));

  RCLCPP_INFO(get_logger(), "LaneDetection started");
}

cv::Mat LaneDetection::build_transform(
  int src_height, int src_width, int dst_height, int dst_width) const
{
  const std::vector<cv::Point2f> src{
    {static_cast<float>(src_width * kRoiTopLeftCol), static_cast<float>(src_height * kRoiTopLeftRow)},
    {static_cast<float>(src_width * kRoiTopRightCol), static_cast<float>(src_height * kRoiTopRightRow)},
    {static_cast<float>(src_width * kRoiBottomRightCol), static_cast<float>(src_height * kRoiBottomRightRow)},
    {static_cast<float>(src_width * kRoiBottomLeftCol), static_cast<float>(src_height * kRoiBottomLeftRow)}};

  const std::vector<cv::Point2f> dst{
    {0.0f, 0.0f},
    {static_cast<float>(dst_width), 0.0f},
    {static_cast<float>(dst_width), static_cast<float>(dst_height)},
    {0.0f, static_cast<float>(dst_height)}};

  return cv::getPerspectiveTransform(src, dst);
}

cv::Mat LaneDetection::bird_eye(const cv::Mat & image) const
{
  const cv::Size dst_size(image.cols, static_cast<int>(image.rows * kBevHeightScale));
  const cv::Mat transform = build_transform(image.rows, image.cols, dst_size.height, dst_size.width);
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, dst_size);
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

std::vector<cv::Point> LaneDetection::walk_lane_chain(
  const std::vector<cv::Point> & yellow_points, const cv::Point2d & origin) const
{
  std::vector<cv::Point> chain;
  if (yellow_points.empty()) {
    return chain;
  }

  // Seed: the bottommost yellow pixel on the right half of the image (closest to the vehicle,
  // on the right lane).
  const cv::Point * seed = nullptr;
  for (const auto & p : yellow_points) {
    if (p.x < origin.x) {
      continue;
    }
    if (!seed || p.y > seed->y ||
        (p.y == seed->y && std::abs(p.x - origin.x) < std::abs(seed->x - origin.x))) {
      seed = &p;
    }
  }
  if (!seed) {
    return chain;
  }

  std::vector<cv::Point> pool = yellow_points;
  cv::Point current = *seed;
  // Direction is tracked in "flipped-y" terms (dy = current.y - candidate.y), so that (0, 1)
  // means straight up the image, i.e. toward the horizon/forward, matching how a lane naturally
  // extends away from the vehicle.
  cv::Point2d direction(0.0, 1.0);

  chain.push_back(current);
  pool.erase(std::remove(pool.begin(), pool.end(), current), pool.end());

  for (int step = 0; step < kMaxChainSteps && current.y > kTopRowMargin; ++step) {
    // Split the pool into pixels within kChainStepRadius of the current point ("passed the
    // distance test") and everything else. Every pixel that passed the distance test is
    // consumed this step -- win or lose -- so a pixel the walk swept past but didn't choose can
    // never be picked up again later and pull the chain backward.
    std::vector<cv::Point> distance_passed;
    std::vector<cv::Point> remaining_pool;
    distance_passed.reserve(pool.size());
    remaining_pool.reserve(pool.size());
    for (const auto & p : pool) {
      const double dx = p.x - current.x;
      const double dy = current.y - p.y;
      if (dx * dx + dy * dy < kChainStepRadius * kChainStepRadius) {
        distance_passed.push_back(p);
      } else {
        remaining_pool.push_back(p);
      }
    }
    pool = std::move(remaining_pool);

    // Among those, keep only pixels within +/-90 degrees of the current direction (non-negative
    // projection onto it) -- the chain never doubles back on itself.
    std::vector<cv::Point> candidates;
    candidates.reserve(distance_passed.size());
    for (const auto & p : distance_passed) {
      const double dx = p.x - current.x;
      const double dy = current.y - p.y;
      if (dx * direction.x + dy * direction.y >= 0.0) {
        candidates.push_back(p);
      }
    }
    if (candidates.empty()) {
      break;
    }

    // Rank candidates by how closely aligned they are with the current direction (smallest
    // angle first, via the signed sine from the cross product -- valid since candidates are
    // already restricted to +/-90 degrees), then by distance from the current point, farthest
    // first.
    std::sort(
      candidates.begin(), candidates.end(), [&](const cv::Point & a, const cv::Point & b) {
        const double adx = a.x - current.x, ady = current.y - a.y;
        const double bdx = b.x - current.x, bdy = current.y - b.y;
        const double a_dist = std::hypot(adx, ady);
        const double b_dist = std::hypot(bdx, bdy);
        // |sin(a)| < |sin(b)|  <=>  |cross_a| / a_dist < |cross_b| / b_dist  <=>
        // |cross_a| * b_dist < |cross_b| * a_dist (dir_norm cancels; distances are positive).
        const double a_cross = direction.x * ady - direction.y * adx;
        const double b_cross = direction.x * bdy - direction.y * bdx;
        const double lhs = a_cross * b_dist;
        const double rhs = b_cross * a_dist;
        if (lhs != rhs) {
          return lhs > rhs;
        }
        return a_dist > b_dist;
      });

    const cv::Point next = candidates.front();
    direction = cv::Point2d(next.x - current.x, current.y - next.y);
    current = next;
    chain.push_back(current);
  }

  return chain;
}

std::vector<cv::Point> LaneDetection::prune_sharp_turns(const std::vector<cv::Point> & points) const
{
  const int n = static_cast<int>(points.size());
  std::vector<bool> remove(n, false);

  for (int i = 1; i + 1 < n; ++i) {
    const cv::Point prev_dir = points[i] - points[i - 1];
    const cv::Point next_dir = points[i + 1] - points[i];
    const double prev_norm = cv::norm(prev_dir);
    const double next_norm = cv::norm(next_dir);
    if (prev_norm < 1e-6 || next_norm < 1e-6) {
      continue;
    }

    const double dot = static_cast<double>(prev_dir.x) * next_dir.x +
                        static_cast<double>(prev_dir.y) * next_dir.y;
    const double cos_angle = std::clamp(dot / (prev_norm * next_norm), -1.0, 1.0);
    const double turn_angle_deg = std::acos(cos_angle) * 180.0 / CV_PI;

    if (turn_angle_deg > kSharpTurnAngleDeg) {
      const int lo = std::max(0, i - kSharpTurnPruneRadius);
      const int hi = std::min(n - 1, i + kSharpTurnPruneRadius);
      std::fill(remove.begin() + lo, remove.begin() + hi + 1, true);
    }
  }

  std::vector<cv::Point> pruned;
  pruned.reserve(n);
  for (int i = 0; i < n; ++i) {
    if (!remove[i]) pruned.push_back(points[i]);
  }
  return pruned;
}

LaneDetection::LaneFitResult LaneDetection::fit_lane(
  const std::vector<cv::Point> & points, const cv::Point2d & origin, int width) const
{
  LaneFitResult result;
  if (static_cast<int>(points.size()) < kMinLanePoints) {
    return result;
  }

  // Arc length s along the chain, from the near (vehicle-adjacent) point at s=0 outward. s is
  // monotonic by construction -- walk_lane_chain only ever extends within +/-90 degrees of the
  // current heading, never backward -- so unlike y, it stays a valid independent variable through
  // a sharp turn where many x (or y) values repeat.
  const int n = static_cast<int>(points.size());
  std::vector<double> s(n), xs(n), ys(n);
  s[0] = 0.0;
  xs[0] = points[0].x;
  ys[0] = points[0].y;
  for (int i = 1; i < n; ++i) {
    s[i] = s[i - 1] + cv::norm(points[i] - points[i - 1]);
    xs[i] = points[i].x;
    ys[i] = points[i].y;
  }
  const double s_far = s.back();

  // Weight points by distance from the far end (mirrors the old fit's row-based weighting:
  // points near the vehicle pull the fit harder than far-field ones). The +1 keeps the farthest
  // point's weight off exactly zero.
  std::vector<double> weight(n);
  for (int i = 0; i < n; ++i) weight[i] = (s_far - s[i]) + 1.0;

  const cv::Vec4d fit_x = fit_cubic(s, xs, weight);
  const cv::Vec4d fit_y = fit_cubic(s, ys, weight);

  // Sample the fitted curve across the arc length the points actually span, for drawing.
  result.curve_points.reserve(kCurveSamples);
  for (int i = 0; i < kCurveSamples; ++i) {
    const double si = s_far * i / (kCurveSamples - 1);
    result.curve_points.emplace_back(
      static_cast<int>(std::lround(eval_cubic(fit_x, si))),
      static_cast<int>(std::lround(eval_cubic(fit_y, si))));
  }

  // Curvature and heading, evaluated at s=0 (the closest point the chain actually reached)
  // rather than the vehicle's exact row. With dashed lane markings especially, a gap can leave no
  // detected pixel anywhere near the vehicle's row on a given frame; evaluating past s=0 would
  // extrapolate the cubics past the domain they were fit to, and a little unwarranted
  // extrapolation swings a derivative far more than it would a position.
  const double dxds = eval_cubic_first_derivative(fit_x, 0.0);
  const double dyds = eval_cubic_first_derivative(fit_y, 0.0);
  const double d2xds2 = eval_cubic_second_derivative(fit_x, 0.0);
  const double d2yds2 = eval_cubic_second_derivative(fit_y, 0.0);
  result.curvature_radius_px =
    std::pow(dxds * dxds + dyds * dyds, 1.5) /
    std::max(std::abs(dxds * d2yds2 - dyds * d2xds2), kMinCurvatureDenominator);
  // Heading relative to straight ahead (up the image, i.e. -y). atan2 stays well-defined even
  // when the tangent is running flat (dyds ~ 0, a 90-degree turn), unlike atan(dx/dy) on a
  // single-valued-in-y fit, which blows up right where a turn gets sharp.
  result.steering_angle_deg = -std::atan2(dxds, -dyds) * 180.0 / CV_PI;

  // Offset stays anchored to the vehicle's actual row (origin.y): the Stanley controller
  // downstream expects cross-track error at the vehicle, not at s=0. Since y is no longer the fit
  // parameter, find it by linearly extrapolating from the near tangent -- a position lookup is
  // far less sensitive to that extrapolation than a derivative is. If the near end is running
  // ~horizontal (mid-turn), that row can't be reached this way, so fall back to the near point's
  // x directly.
  double lane_x_at_vehicle = xs[0];
  if (std::abs(dyds) > kMinNearTangentDy) {
    const double s_at_origin = (origin.y - ys[0]) / dyds;
    lane_x_at_vehicle = eval_cubic(fit_x, s_at_origin);
  }
  const double meters_per_pixel = kLaneWidthMeters / static_cast<double>(width);
  result.offset_m = (lane_x_at_vehicle - origin.x) * meters_per_pixel - 0.5;

  result.valid = true;
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

  const cv::Mat frame = cv_ptr->image;

  const cv::Mat warped = bird_eye(frame);
  const cv::Mat mask = yellow_mask(warped);

  // BEV with the yellow mask highlighted; this is also the single debug view, so the chain,
  // fitted curve, and fit stats all get drawn on top of it below.
  cv::Mat view = warped.clone();
  view.setTo(cv::Scalar(0, 255, 0), mask);

  const cv::Point2d origin(warped.cols / 2.0, warped.rows - 1.0);

  std::vector<cv::Point> yellow_points;
  cv::findNonZero(mask, yellow_points);
  const std::vector<cv::Point> chain = prune_sharp_turns(walk_lane_chain(yellow_points, origin));

  // Fall back to the previous frame's lane when nothing is found this frame
  std::vector<cv::Point> points = chain;
  if (points.empty()) {
    points = prev_points_;
  } else {
    prev_points_ = points;
  }

  for (const auto & p : points) cv::circle(view, p, 3, cv::Scalar(0, 165, 255), -1);

  const LaneFitResult fit = fit_lane(points, origin, warped.cols);

  if (fit.valid) {
    cv::polylines(view, fit.curve_points, false, cv::Scalar(255, 0, 255), 3);

    const cv::Point arrow_start(static_cast<int>(origin.x), static_cast<int>(origin.y));
    const cv::Point arrow_end(
      static_cast<int>(
        origin.x + kArrowLength * std::sin(fit.steering_angle_deg * CV_PI / 180.0)),
      static_cast<int>(
        origin.y - kArrowLength * std::cos(fit.steering_angle_deg * CV_PI / 180.0)));
    cv::line(view, arrow_start, arrow_end, cv::Scalar(255, 0, 0), 2);
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

  char offset_text[64];
  std::snprintf(offset_text, sizeof(offset_text), "Offset: %.2f m", fit.offset_m);
  cv::putText(view, offset_text, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

  char angle_text[64];
  std::snprintf(angle_text, sizeof(angle_text), "Angle: %.2f deg", fit.steering_angle_deg);
  cv::putText(view, angle_text, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

  if (!fit.valid) {
    cv::putText(
      view, "Lane not detected", cv::Point(30, 110), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Lane not detected");
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
