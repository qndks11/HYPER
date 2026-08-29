#include "hyper_lane_detection/lane_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "hyper_lane_detection/course_geometry.hpp"

namespace hyper_lane_detection
{

namespace
{
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

// A few points of slack past the minimum a well-conditioned line-direction fit needs (2 points
// determine a direction exactly; more give the weighted eigen solve something to average over).
constexpr int kMinLanePoints = 3;
// Floor on the fitted line direction's weighted variance (see fit_lane): below this the chain is
// numerically degenerate (points effectively coincident), so the eigen solve isn't trustworthy.
constexpr double kMinLineFitVariance = 1e-6;
// fit_lane() models the lane as a straight line, which has zero curvature everywhere; this
// sentinel radius is reported in its place rather than infinity, so downstream consumers (e.g.
// the controller's curvature-based speed throttle) see "very straight" instead of a non-finite
// value. It is the only value Fit::curvature_radius_px ever takes now that the rear camera's
// quadratic fit is gone with the camera itself.
constexpr double kMaxCurvatureRadiusPx = 1e6;

constexpr double kArrowLength = 100.0;

// Half-lane-width offset [m] from a single tracked lane line to the estimated lane center.
// Baked into fit_lane()'s offset_m, signed per side (subtracted for the right lane, added for the
// left, since the center sits on opposite sides of each line), so each side's own biased offset
// reports a centered estimate even though the two sides are no longer combined into one fit.
// Now 1.5 m rather than the 1.85 m it was: kLaneWidthMeters used to carry a 3.7 m public-highway
// figure, while the course this drives is built with 3.0 m lanes, so every published lane offset
// stood 0.35 m off center in whichever direction the tracked line happened to be on.
constexpr double kLaneCenterOffsetBiasM = kLaneWidthMeters / 2.0;

// Empirical gain [m per degree] converting a tracked lane line's own outward lean --
// steering_angle_deg's magnitude, in the direction that means the line is angling away from the
// vehicle rather than toward it -- into extra lateral offset correction. kLaneCenterOffsetBiasM
// alone assumes the visible line runs parallel to the vehicle's heading; once it's visibly
// leaning outward that assumption undershoots the true center offset, and more so the more it
// leans. Applied to both sides unconditionally, since each side is always published as its own
// independent single-line estimate now. Untuned placeholder -- adjust against real footage.
constexpr double kOutwardLeanGainMPerDeg = 0.04;

constexpr int kTextLinePitch = 36;

// True if `chain` starts on one side of origin.x and ends on the other, but its endpoint isn't
// farther from the vehicle (smaller row) than its start point. A chain that genuinely walks a
// curving lane across the centerline still gets farther from the vehicle as it goes; a chain that
// fails that check instead indicates the left- and right-side seeds latched onto the same
// physical lane line (e.g. a single lane straddling origin.x), so the caller should discard it
// rather than report it as a second, distinct lane.
bool is_spurious_cross_lane(const std::vector<cv::Point> & chain, const cv::Point2d & origin)
{
  if (chain.size() < 2) {
    return false;
  }
  const bool start_left = chain.front().x < origin.x;
  const bool end_left = chain.back().x < origin.x;
  if (start_left == end_left) {
    return false;
  }
  return chain.front().y <= chain.back().y;
}

}  // namespace

cv::Mat LaneDetector::yellow_mask(const cv::Mat & warped) const
{
  cv::Mat hsv;
  cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(22, 80, 80), cv::Scalar(35, 255, 255), mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

std::vector<cv::Point> LaneDetector::walk_lane_chain(
  const std::vector<cv::Point> & yellow_points, const cv::Point2d & origin, Side side) const
{
  std::vector<cv::Point> chain;
  if (yellow_points.empty()) {
    return chain;
  }

  // Seed: the bottommost yellow pixel on the requested half of the image (closest to the
  // vehicle, on that side's lane).
  const cv::Point * seed = nullptr;
  for (const auto & p : yellow_points) {
    const bool wrong_side = side == Side::kRight ? (p.x < origin.x) : (p.x >= origin.x);
    if (wrong_side) {
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

    // Rank candidates by their signed angle to the current direction (cross product over the
    // product of norms -- valid without abs() since candidates are already restricted to +/-90
    // degrees, so the sign alone orders them). The right lane is walked most-positive-angle
    // first; the left lane is walked in the opposite order, most-negative first (see Side).
    // Ties are broken by distance from the current point, farthest first.
    std::sort(
      candidates.begin(), candidates.end(), [&](const cv::Point & a, const cv::Point & b) {
        const double adx = a.x - current.x, ady = current.y - a.y;
        const double bdx = b.x - current.x, bdy = current.y - b.y;
        const double a_dist = std::hypot(adx, ady);
        const double b_dist = std::hypot(bdx, bdy);
        // sin(a) vs sin(b): cross_a / a_dist vs cross_b / b_dist <=> cross_a * b_dist vs
        // cross_b * a_dist (dir_norm cancels; distances are positive).
        const double a_cross = direction.x * ady - direction.y * adx;
        const double b_cross = direction.x * bdy - direction.y * bdx;
        const double lhs = a_cross * b_dist;
        const double rhs = b_cross * a_dist;
        if (lhs != rhs) {
          return side == Side::kRight ? lhs > rhs : lhs < rhs;
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

LaneDetector::Fit LaneDetector::fit_lane(
  const std::vector<cv::Point> & points, const cv::Point2d & origin, double meters_per_pixel,
  Side side, double center_bias_m) const
{
  Fit result;
  if (static_cast<int>(points.size()) < kMinLanePoints) {
    return result;
  }

  const int n = static_cast<int>(points.size());
  const cv::Point2d p0(points[0].x, points[0].y);
  const cv::Point2d p_far(points[n - 1].x, points[n - 1].y);

  // Arc length s along the chain, used only to weight near points more heavily below (mirrors
  // the old fit's weighting: near-field accuracy matters more to the Stanley controller than
  // far-field noise) and to size how far the drawn line segment extends.
  std::vector<double> s(n);
  s[0] = 0.0;
  for (int i = 1; i < n; ++i) s[i] = s[i - 1] + cv::norm(points[i] - points[i - 1]);
  const double s_far = s.back();

  // Fit a line through the fixed point p0 by weighted total least squares: the best-fit
  // direction is the eigenvector, of the weighted scatter matrix of (points - p0), with the
  // largest eigenvalue -- the axis the points are most spread along, which is exactly the
  // direction minimizing the weighted sum of squared perpendicular distances to the line. Unlike
  // ordinary least squares (regressing x on y or y on x), this has no axis it breaks down along,
  // so it stays well-conditioned for a lane running in any direction, including near-horizontal.
  double mxx = 0.0, myy = 0.0, mxy = 0.0;
  for (int i = 0; i < n; ++i) {
    const double w = (s_far - s[i]) + 1.0;
    const double qx = points[i].x - p0.x;
    const double qy = points[i].y - p0.y;
    mxx += w * qx * qx;
    myy += w * qy * qy;
    mxy += w * qx * qy;
  }

  const cv::Mat scatter = (cv::Mat_<double>(2, 2) << mxx, mxy, mxy, myy);
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(scatter, eigenvalues, eigenvectors);
  if (eigenvalues.at<double>(0) < kMinLineFitVariance) {
    return result;
  }
  cv::Point2d direction(eigenvectors.at<double>(0, 0), eigenvectors.at<double>(0, 1));
  // The eigenvector's sign is arbitrary; flip it to point from the near point toward the far
  // point, so steering_angle_deg comes out with the expected sign convention below.
  if (direction.x * (p_far.x - p0.x) + direction.y * (p_far.y - p0.y) < 0.0) {
    direction = -direction;
  }

  // Drawn as a single segment spanning the same reach as the observed chain.
  result.curve_points = {
    cv::Point(static_cast<int>(std::lround(p0.x)), static_cast<int>(std::lround(p0.y))),
    cv::Point(
      static_cast<int>(std::lround(p0.x + direction.x * s_far)),
      static_cast<int>(std::lround(p0.y + direction.y * s_far)))};

  // Heading relative to straight ahead (up the image, i.e. -y). atan2 stays well-defined even
  // when the line is running flat (direction.y ~ 0), unlike atan(dx/dy).
  result.steering_angle_deg = -std::atan2(direction.x, -direction.y) * 180.0 / CV_PI;

  // A straight line has zero curvature everywhere; see kMaxCurvatureRadiusPx.
  result.curvature_radius_px = kMaxCurvatureRadiusPx;

  // Signed perpendicular distance from the vehicle to the fitted line itself, rather than just to
  // its anchor point p0 -- for a steeply angled fit, p0's own x-position is an increasingly poor
  // stand-in for where the line actually sits at the vehicle's row. This is the cross product of
  // (origin - p0) with the unit direction vector; it reduces to the old (p0.x - origin.x) formula
  // exactly when direction is vertical (a straight-ahead lane).
  const double perp_px = (origin.x - p0.x) * direction.y - (origin.y - p0.y) * direction.x;
  // The target sits on the vehicle's side of a right line, but on the far side of a left line, so
  // the bias flips sign between the two -- see the call sites for what center_bias_m represents.
  const double signed_bias_m = side == Side::kRight ? -center_bias_m : center_bias_m;
  result.offset_m = perp_px * meters_per_pixel + signed_bias_m;

  result.valid = true;
  return result;
}

LaneDetector::Result LaneDetector::detect_and_draw(
  const cv::Mat & yellow, const cv::Point2d & origin, double meters_per_pixel,
  cv::Mat & view) const
{
  std::vector<cv::Point> yellow_points;
  cv::findNonZero(yellow, yellow_points);
  std::vector<cv::Point> right_points = walk_lane_chain(yellow_points, origin, Side::kRight);
  std::vector<cv::Point> left_points = walk_lane_chain(yellow_points, origin, Side::kLeft);

  // A chain that crosses to the other side without getting farther from the vehicle is most
  // likely the same physical lane line the other side's seed already claimed; see
  // is_spurious_cross_lane().
  if (is_spurious_cross_lane(right_points, origin)) right_points.clear();
  if (is_spurious_cross_lane(left_points, origin)) left_points.clear();

  for (const auto & p : right_points) cv::circle(view, p, 3, cv::Scalar(0, 165, 255), -1);
  for (const auto & p : left_points) cv::circle(view, p, 3, cv::Scalar(255, 0, 0), -1);

  Result result;
  result.right =
    fit_lane(right_points, origin, meters_per_pixel, Side::kRight, kLaneCenterOffsetBiasM);
  result.left =
    fit_lane(left_points, origin, meters_per_pixel, Side::kLeft, kLaneCenterOffsetBiasM);

  // Each side is published as its own independent estimate now (no more width-based decision
  // between left-only / right-only / averaged-both). If a line is angling outward -- away from
  // the vehicle rather than parallel to it -- kLaneCenterOffsetBiasM alone (baked into
  // fit_lane()'s offset_m) undershoots the true center offset, more so the more it leans; correct
  // for that on whichever side(s) are valid. See kOutwardLeanGainMPerDeg.
  auto apply_outward_lean_correction = [](Fit & f, Side side) {
    const double outward_lean_deg =
      side == Side::kRight ? -f.steering_angle_deg : f.steering_angle_deg;
    if (outward_lean_deg <= 0.0) {
      return;
    }
    const double extra_m = outward_lean_deg * kOutwardLeanGainMPerDeg;
    f.offset_m += side == Side::kRight ? -extra_m : extra_m;
  };
  if (result.right.valid) apply_outward_lean_correction(result.right, Side::kRight);
  if (result.left.valid) apply_outward_lean_correction(result.left, Side::kLeft);

  if (result.right.valid) {
    cv::polylines(view, result.right.curve_points, false, cv::Scalar(255, 0, 255), 3);
  }
  if (result.left.valid) {
    cv::polylines(view, result.left.curve_points, false, cv::Scalar(0, 255, 255), 3);
  }

  // steering_angle_deg is defined as -atan2(direction.x, -direction.y) in fit_lane, so a
  // rightward-tilting fitted line comes out as a *negative* angle; reconstructing a direction
  // vector from the angle therefore needs sin() negated too (-sin, not sin), or the drawn
  // direction ends up mirrored left/right from the line it's supposed to represent.
  auto draw_side_overlay = [&](const Fit & f, cv::Scalar color) {
    if (!f.valid) return;
    const double heading_rad = f.steering_angle_deg * CV_PI / 180.0;
    const cv::Point2d heading_dir(-std::sin(heading_rad), -std::cos(heading_rad));

    const cv::Point arrow_start(static_cast<int>(origin.x), static_cast<int>(origin.y));
    const cv::Point arrow_end(
      static_cast<int>(std::lround(origin.x + heading_dir.x * kArrowLength)),
      static_cast<int>(std::lround(origin.y + heading_dir.y * kArrowLength)));
    cv::line(view, arrow_start, arrow_end, color, 2);

    // The target line this side is actually tracking: unlike the heading arrow above (which
    // starts at the vehicle's own position), this starts at the vehicle's lateral offset from
    // that side's estimated center (f.offset_m converted back to px) and extends along the same
    // heading all the way to the top of the frame, so it reads as the path being followed rather
    // than just a direction.
    const double offset_px = f.offset_m / meters_per_pixel;
    const cv::Point target_start(
      static_cast<int>(std::lround(origin.x + offset_px)), static_cast<int>(origin.y));
    const cv::Point target_end(
      static_cast<int>(std::lround(target_start.x + heading_dir.x * view.rows)),
      static_cast<int>(std::lround(target_start.y + heading_dir.y * view.rows)));
    cv::line(view, target_start, target_end, color, 2);
  };
  draw_side_overlay(result.left, cv::Scalar(255, 0, 0));
  draw_side_overlay(result.right, cv::Scalar(0, 255, 0));

  return result;
}

void LaneDetector::draw_text(const Result & result, cv::Mat & view, int panel_top) const
{
  const cv::Scalar lane_text_color = (result.left.valid || result.right.valid) ?
    cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  char left_text[64];
  if (result.left.valid) {
    std::snprintf(left_text, sizeof(left_text), "L: %.2f m / %.2f deg",
      result.left.offset_m, result.left.steering_angle_deg);
  } else {
    std::snprintf(left_text, sizeof(left_text), "L: --");
  }
  cv::putText(
    view, left_text, cv::Point(30, panel_top + kTextLinePitch * 1), cv::FONT_HERSHEY_SIMPLEX,
    1.0, lane_text_color, 2);

  char right_text[64];
  if (result.right.valid) {
    std::snprintf(right_text, sizeof(right_text), "R: %.2f m / %.2f deg",
      result.right.offset_m, result.right.steering_angle_deg);
  } else {
    std::snprintf(right_text, sizeof(right_text), "R: --");
  }
  cv::putText(
    view, right_text, cv::Point(30, panel_top + kTextLinePitch * 2), cv::FONT_HERSHEY_SIMPLEX,
    1.0, lane_text_color, 2);

  if (!result.left.valid && !result.right.valid) {
    cv::putText(
      view, "Lane not detected", cv::Point(30, panel_top + kTextLinePitch * 3),
      cv::FONT_HERSHEY_SIMPLEX, 1.0, lane_text_color, 2);
  }
}

}  // namespace hyper_lane_detection
