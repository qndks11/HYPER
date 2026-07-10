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

// A few points of slack past the minimum a stable tangent-scale fit needs: at least
// kTangentLookahead+1 points to estimate each end's tangent direction, and at least one interior
// point to fit the two shared scalars against.
constexpr int kMinLanePoints = 6;
constexpr int kCurveSamples = 40;   // density of the drawn/unwarped curve, not the fit itself
constexpr double kMinCurvatureDenominator = 1e-6;
// How many points in from each end of the chain to look when estimating that end's tangent
// direction, so a single noisy pixel doesn't dominate the direction the fitted curve gets locked
// to (see fit_lane).
constexpr int kTangentLookahead = 3;

constexpr double kLaneWidthMeters = 3.7;
constexpr double kArrowLength = 100.0;

constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;

// Bird's-eye output height as a multiple of the source frame height, i.e. how much farther
// down the road the BEV view looks. Width is left unscaled.
constexpr double kBevHeightScale = 1.4;

// Fits the two tangent-scale unknowns (k1, k2) of a cubic Bezier curve whose control points are
// constrained to lie along the chain's own tangent directions at each end --
// P1 = p0 + k1*dir0, P2 = p3 - k2*dir_end -- rather than left free in the plane. Locking each
// control point's *direction* to the observed chain tangent keeps the curve's shape tied to what
// was actually seen: B'(0) = 3*k1*dir0 and B'(1) = 3*k2*dir_end always point along dir0/dir_end
// for any k1, k2 > 0, so the fit can no longer swing the curve's start/end heading off in some
// direction the data didn't suggest -- which is what let a fully free control-point fit produce
// unrealistically sharp curvature on a noisy chain. k1 and k2 (returned clamped to >= 0, since a
// negative scale would fold the curve back on itself) are still found by weighted least squares,
// with the x and y residual of every interior chain point stacked into one system, since a
// single pair of scalars is shared between both axes.
cv::Vec2d fit_bezier_tangent_scales(
  const std::vector<double> & t, const std::vector<cv::Point2d> & pts,
  const std::vector<double> & weight, const cv::Point2d & p0, const cv::Point2d & p3,
  const cv::Point2d & dir0, const cv::Point2d & dir_end)
{
  const int n = static_cast<int>(t.size());
  cv::Mat A(2 * n, 2, CV_64F);
  cv::Mat b(2 * n, 1, CV_64F);

  for (int i = 0; i < n; ++i) {
    const double w = weight[i];
    const double u = 1.0 - t[i];
    const double basis1 = 3.0 * u * u * t[i];       // k1's coefficient
    const double basis2 = 3.0 * u * t[i] * t[i];     // k2's coefficient (subtracted, see below)
    // p0 and p3's Bezier weights collapse to the cubic Hermite blend functions once P1 and P2 are
    // expressed via dir0/dir_end instead of standing on their own.
    const double blend0 = u * u * (1.0 + 2.0 * t[i]);
    const double blend3 = t[i] * t[i] * (3.0 - 2.0 * t[i]);
    const cv::Point2d residual = pts[i] - (blend0 * p0 + blend3 * p3);

    const int row_x = 2 * i, row_y = 2 * i + 1;
    A.at<double>(row_x, 0) = w * basis1 * dir0.x;
    A.at<double>(row_x, 1) = -w * basis2 * dir_end.x;
    A.at<double>(row_y, 0) = w * basis1 * dir0.y;
    A.at<double>(row_y, 1) = -w * basis2 * dir_end.y;
    b.at<double>(row_x, 0) = w * residual.x;
    b.at<double>(row_y, 0) = w * residual.y;
  }

  cv::Mat coeffs;
  cv::solve(A, b, coeffs, cv::DECOMP_SVD);
  return cv::Vec2d(std::max(0.0, coeffs.at<double>(0)), std::max(0.0, coeffs.at<double>(1)));
}

double eval_bezier(double p0, double p1, double p2, double p3, double t)
{
  const double u = 1.0 - t;
  return u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3;
}

double eval_bezier_first_derivative(double p0, double p1, double p2, double p3, double t)
{
  const double u = 1.0 - t;
  return 3.0 * u * u * (p1 - p0) + 6.0 * u * t * (p2 - p1) + 3.0 * t * t * (p3 - p2);
}

double eval_bezier_second_derivative(double p0, double p1, double p2, double p3, double t)
{
  const double u = 1.0 - t;
  return 6.0 * u * (p2 - 2.0 * p1 + p0) + 6.0 * t * (p3 - 2.0 * p2 + p1);
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
  // s_far == 0 would make t undefined below; in practice this can't happen (each chain step
  // moves at least kChainStepRadius), but this keeps a degenerate chain from producing NaNs.
  if (s_far < kMinCurvatureDenominator) {
    return result;
  }

  // Bezier parameter t = s / s_far, so the chain's near and far points land exactly on t=0 and
  // t=1 -- the curve's fixed endpoints.
  std::vector<double> t(n);
  for (int i = 0; i < n; ++i) t[i] = s[i] / s_far;

  // Tangent directions at each end, estimated a few points in from the endpoint rather than from
  // a single adjacent segment, so one noisy pixel doesn't dominate the direction the curve gets
  // locked to. kTangentLookahead is clamped to n-1 so this stays valid for a chain right at
  // kMinLanePoints.
  const int lookahead = std::min(kTangentLookahead, n - 1);
  const cv::Point2d dir0(
    points[lookahead].x - points[0].x, points[lookahead].y - points[0].y);
  const cv::Point2d dir_end(
    points[n - 1].x - points[n - 1 - lookahead].x, points[n - 1].y - points[n - 1 - lookahead].y);
  const double dir0_norm = cv::norm(dir0);
  const double dir_end_norm = cv::norm(dir_end);
  if (dir0_norm < kMinCurvatureDenominator || dir_end_norm < kMinCurvatureDenominator) {
    return result;
  }

  // Weight points by distance from the far end (mirrors the old fit's row-based weighting:
  // points near the vehicle pull the fit harder than far-field ones). The +1 keeps the farthest
  // point's weight off exactly zero. Only interior points are passed to the tangent-scale fit --
  // the endpoints contribute nothing to it, since the Bezier basis functions for P1 and P2
  // vanish at t=0 and t=1.
  std::vector<double> weight(n);
  for (int i = 0; i < n; ++i) weight[i] = (s_far - s[i]) + 1.0;
  const std::vector<double> t_interior(t.begin() + 1, t.end() - 1);
  const std::vector<double> weight_interior(weight.begin() + 1, weight.end() - 1);
  std::vector<cv::Point2d> pts_interior(n - 2);
  for (int i = 1; i < n - 1; ++i) pts_interior[i - 1] = cv::Point2d(xs[i], ys[i]);

  const cv::Point2d p0(xs[0], ys[0]);
  const cv::Point2d p3(xs[n - 1], ys[n - 1]);
  const cv::Vec2d tangent_scales = fit_bezier_tangent_scales(
    t_interior, pts_interior, weight_interior, p0, p3, dir0 / dir0_norm, dir_end / dir_end_norm);
  const cv::Point2d p1 = p0 + tangent_scales[0] * (dir0 / dir0_norm);
  const cv::Point2d p2 = p3 - tangent_scales[1] * (dir_end / dir_end_norm);

  // Sample the fitted curve across its full [0, 1] domain, for drawing.
  result.curve_points.reserve(kCurveSamples);
  for (int i = 0; i < kCurveSamples; ++i) {
    const double ti = static_cast<double>(i) / (kCurveSamples - 1);
    result.curve_points.emplace_back(
      static_cast<int>(std::lround(eval_bezier(p0.x, p1.x, p2.x, p3.x, ti))),
      static_cast<int>(std::lround(eval_bezier(p0.y, p1.y, p2.y, p3.y, ti))));
  }

  // Curvature and heading, evaluated at t=0 (the closest point the chain actually reached, since
  // the curve is pinned to it exactly) rather than the vehicle's exact row. With dashed lane
  // markings especially, a gap can leave no detected pixel anywhere near the vehicle's row on a
  // given frame; evaluating past t=0 would extrapolate past the curve's start, and a little
  // unwarranted extrapolation swings a derivative far more than it would a position. The
  // curvature formula below is parameterization-invariant, so using the raw t-derivatives here
  // (rather than rescaling them to arc length) still gives the true geometric curvature.
  const double dxdt = eval_bezier_first_derivative(p0.x, p1.x, p2.x, p3.x, 0.0);
  const double dydt = eval_bezier_first_derivative(p0.y, p1.y, p2.y, p3.y, 0.0);
  const double d2xdt2 = eval_bezier_second_derivative(p0.x, p1.x, p2.x, p3.x, 0.0);
  const double d2ydt2 = eval_bezier_second_derivative(p0.y, p1.y, p2.y, p3.y, 0.0);
  result.curvature_radius_px =
    std::pow(dxdt * dxdt + dydt * dydt, 1.5) /
    std::max(std::abs(dxdt * d2ydt2 - dydt * d2xdt2), kMinCurvatureDenominator);
  // Heading relative to straight ahead (up the image, i.e. -y). atan2 stays well-defined even
  // when the tangent is running flat (dydt ~ 0, a 90-degree turn), unlike atan(dx/dy) on a
  // single-valued-in-y fit, which blows up right where a turn gets sharp.
  result.steering_angle_deg = -std::atan2(dxdt, -dydt) * 180.0 / CV_PI;

  // Offset is taken at the chain's near point (t=0, pinned exactly to points[0]) rather than
  // extrapolated out to the vehicle's actual row (origin.y). The camera sits back from the front
  // wheels now, so the visible lane can start well short of the vehicle row; extrapolating that
  // gap with the near tangent would swing the offset far more than the position error it's meant
  // to fix, especially mid-turn. The near point is the closest actual observation, so it's used
  // as-is.
  const double lane_x_at_vehicle = xs[0];
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
  const std::vector<cv::Point> chain = walk_lane_chain(yellow_points, origin);

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
