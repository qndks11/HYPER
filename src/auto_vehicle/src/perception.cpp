#include "auto_vehicle/perception.hpp"

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

// Bird's-eye output height as a multiple of the source frame height, i.e. how much farther
// down the road the BEV view looks. Width is left unscaled.
constexpr double kBevHeightScale = 1.35;

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
// A straight-line fit has zero curvature everywhere; this sentinel radius is reported in its
// place rather than infinity, so downstream consumers (e.g. the controller's curvature-based
// speed throttle) see "very straight" instead of a non-finite value.
constexpr double kMaxCurvatureRadiusPx = 1e6;

// Shared by both lane and stop-line meters-per-pixel scaling -- previously duplicated across
// lane_detection.cpp and stopline_detection.cpp.
constexpr double kLaneWidthMeters = 3.7;
constexpr double kNumLaneInScreen = 3.2;  // how many lanes fit across the BEV image width
constexpr double kArrowLength = 100.0;

// Tolerance above kLaneWidthMeters before two simultaneously-detected lines are treated as
// implausibly far apart to be the same lane's pair (e.g. one side actually latched onto an
// adjacent lane's paint). Loose enough to tolerate normal fit noise on a real lane.
constexpr double kMaxPlausibleLaneWidthMeters = kLaneWidthMeters * 1.4;

// Empirical half-lane-width offset [m] from a single tracked lane line to the estimated lane
// center, calibrated back when only the right lane was tracked. Baked into fit_lane()'s offset_m,
// signed per side (subtracted for the right lane, added for the left, since the center sits on
// opposite sides of each line), so the single-lane fallback (only one side detected this frame)
// still reports a centered estimate. When both sides are valid, the opposite signs cancel when
// averaged, so image_callback's combined offset is just the plain average -- no bias to add back.
constexpr double kLaneCenterOffsetBiasM = kLaneWidthMeters / 2.0;

// A stop-line bar spans most of the lane, so its bounding box is much wider than it is tall; a
// single zebra-crossing stripe is comparatively close to square. This floor separates the two.
constexpr double kMinStoplineAspectRatio = 3.0;
// The stop-line bar must span at least this fraction of the BEV image width to count -- rules
// out narrower marks (e.g. a single crossing stripe) that happen to pass the aspect ratio test.
constexpr double kMinStoplineWidthFraction = 0.25;
// Floor on contour area [px^2] to reject small mask noise before the shape checks run.
constexpr double kMinStoplineAreaPx = 500.0;

constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;

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

Perception::Perception() : Node{"perception"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&Perception::image_callback, this, std::placeholders::_1));

  lane_center_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);
  stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/detection", 10);

  cv::namedWindow("Perception", cv::WINDOW_NORMAL);
  cv::resizeWindow("Perception", kWindowWidth, static_cast<int>(kWindowHeight * kBevHeightScale));

  RCLCPP_INFO(get_logger(), "Perception started");
}

cv::Mat Perception::build_transform(
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

cv::Mat Perception::bird_eye(const cv::Mat & image) const
{
  const cv::Size dst_size(image.cols, static_cast<int>(image.rows * kBevHeightScale));
  const cv::Mat transform = build_transform(image.rows, image.cols, dst_size.height, dst_size.width);
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, dst_size);
  return warped;
}

cv::Mat Perception::yellow_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  // Hue floor is 22, not the more permissive 15 a plain "yellow-ish" range might suggest: sampling
  // the course texture's own pixels showed lane paint (including its anti-aliased blends against
  // the gray road, which only shift S/V, not hue) clustering at H 26-31, while brown dirt/curb
  // pixels sit in a separate cluster at H 16-19 with a clean, essentially empty gap at H 20-25 --
  // so 22 excludes the brown without narrowing the yellow paint's own hue range at all.
  cv::inRange(hsv, cv::Scalar(22, 80, 80), cv::Scalar(35, 255, 255), mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

cv::Mat Perception::white_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 60, 255), mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

std::vector<cv::Point> Perception::walk_lane_chain(
  const std::vector<cv::Point> & yellow_points, const cv::Point2d & origin, LaneSide side) const
{
  std::vector<cv::Point> chain;
  if (yellow_points.empty()) {
    return chain;
  }

  // Seed: the bottommost yellow pixel on the requested half of the image (closest to the
  // vehicle, on that side's lane).
  const cv::Point * seed = nullptr;
  for (const auto & p : yellow_points) {
    const bool wrong_side = side == LaneSide::kRight ? (p.x < origin.x) : (p.x >= origin.x);
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
    // first; the left lane is walked in the opposite order, most-negative first (see LaneSide).
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
          return side == LaneSide::kRight ? lhs > rhs : lhs < rhs;
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

Perception::LaneFitResult Perception::fit_lane(
  const std::vector<cv::Point> & points, const cv::Point2d & origin, int width,
  LaneSide side) const
{
  LaneFitResult result;
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

  const double meters_per_pixel = kNumLaneInScreen * kLaneWidthMeters / static_cast<double>(width);
  // The lane center sits on the vehicle's side of a right lane line, but on the far side of a
  // left lane line, so the bias flips sign between the two -- see kLaneCenterOffsetBiasM.
  const double center_bias_m =
    side == LaneSide::kRight ? -kLaneCenterOffsetBiasM : kLaneCenterOffsetBiasM;
  result.offset_m = (p0.x - origin.x) * meters_per_pixel + center_bias_m;

  result.valid = true;
  return result;
}

Perception::StoplineResult Perception::find_stopline(
  const cv::Mat & mask, const cv::Point2d & origin, double meters_per_pixel) const
{
  StoplineResult result;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  const double min_width_px = mask.cols * kMinStoplineWidthFraction;

  // A stop-line bar belonging to some other lane at a multi-lane intersection can still pass the
  // aspect-ratio and min-width checks above, so it's not enough to look at a candidate's shape --
  // gate on lateral position too: reject anything centered more than half a lane away from the
  // vehicle's own lane center (origin.x). One lane spans mask.cols / kNumLaneInScreen px, the same
  // scale used everywhere else in this file.
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
    const double aspect_ratio = static_cast<double>(box.width) / box.height;
    if (aspect_ratio < kMinStoplineAspectRatio || box.width < min_width_px) {
      continue;
    }
    const double box_center_x = box.x + box.width / 2.0;
    if (box_center_x < band_min_x || box_center_x > band_max_x) {
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

void Perception::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat warped = bird_eye(cv_ptr->image);
  const cv::Point2d origin(warped.cols / 2.0, warped.rows - 1.0);
  // Shared by the lane-width plausibility check and stop-line distance below; fit_lane() also
  // derives this internally per side, since it only receives `width` rather than this node's
  // full image_callback scope.
  const double meters_per_pixel = kNumLaneInScreen * kLaneWidthMeters / static_cast<double>(warped.cols);

  const cv::Mat yellow = yellow_mask(warped);
  const cv::Mat white = white_mask(warped);

  // Single shared debug view: both masks are highlighted first (different colors so the two
  // don't read as one blob), before either detector's annotations are drawn on top -- so a mask
  // fill never overwrites an annotation drawn earlier.
  cv::Mat view = warped.clone();
  view.setTo(cv::Scalar(0, 255, 0), yellow);
  view.setTo(cv::Scalar(180, 180, 180), white);

  // --- Lane detection ---

  std::vector<cv::Point> yellow_points;
  cv::findNonZero(yellow, yellow_points);
  std::vector<cv::Point> right_chain = walk_lane_chain(yellow_points, origin, LaneSide::kRight);
  std::vector<cv::Point> left_chain = walk_lane_chain(yellow_points, origin, LaneSide::kLeft);

  // A chain that crosses to the other side without getting farther from the vehicle is most
  // likely the same physical lane line the other side's seed already claimed; see
  // is_spurious_cross_lane().
  if (is_spurious_cross_lane(right_chain, origin)) right_chain.clear();
  if (is_spurious_cross_lane(left_chain, origin)) left_chain.clear();

  // Fall back to the previous frame's lane, per side, when nothing is found this frame.
  std::vector<cv::Point> right_points = right_chain;
  if (right_points.empty()) {
    right_points = prev_right_points_;
  } else {
    prev_right_points_ = right_points;
  }

  std::vector<cv::Point> left_points = left_chain;
  if (left_points.empty()) {
    left_points = prev_left_points_;
  } else {
    prev_left_points_ = left_points;
  }

  for (const auto & p : right_points) cv::circle(view, p, 3, cv::Scalar(0, 165, 255), -1);
  for (const auto & p : left_points) cv::circle(view, p, 3, cv::Scalar(255, 0, 0), -1);

  const LaneFitResult right_fit = fit_lane(right_points, origin, warped.cols, LaneSide::kRight);
  const LaneFitResult left_fit = fit_lane(left_points, origin, warped.cols, LaneSide::kLeft);

  // Combine both sides into a single published estimate: when both are valid, average their
  // offsets into a true lane-center estimate -- each fit's kLaneCenterOffsetBiasM is signed
  // opposite to the other's, so the average needs no further correction; otherwise fall back to
  // whichever single side is valid, unchanged from the single-lane behavior.
  LaneFitResult fit;
  if (right_fit.valid && left_fit.valid) {
    const double lane_width_m =
      (right_points.front().x - left_points.front().x) * meters_per_pixel;
    if (lane_width_m > kMaxPlausibleLaneWidthMeters) {
      // The two lines are farther apart than a real lane, so at least one of them isn't this
      // lane's own line -- trust whichever side is physically closer to the vehicle rather than
      // averaging in a bogus far side.
      const bool right_closer =
        std::abs(right_points.front().x - origin.x) < std::abs(left_points.front().x - origin.x);
      fit = right_closer ? right_fit : left_fit;
    } else {
      fit.valid = true;
      fit.offset_m = (right_fit.offset_m + left_fit.offset_m) / 2.0;
      fit.steering_angle_deg = (right_fit.steering_angle_deg + left_fit.steering_angle_deg) / 2.0;
      fit.curvature_radius_px = (right_fit.curvature_radius_px + left_fit.curvature_radius_px) / 2.0;
    }
  } else if (right_fit.valid) {
    fit = right_fit;
  } else if (left_fit.valid) {
    fit = left_fit;
  }

  if (right_fit.valid) cv::polylines(view, right_fit.curve_points, false, cv::Scalar(255, 0, 255), 3);
  if (left_fit.valid) cv::polylines(view, left_fit.curve_points, false, cv::Scalar(0, 255, 255), 3);

  if (fit.valid) {
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

  const cv::Scalar lane_text_color = fit.valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  char offset_text[64];
  std::snprintf(offset_text, sizeof(offset_text), "Offset: %.2f m", fit.offset_m);
  cv::putText(
    view, offset_text, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, lane_text_color, 2);

  char angle_text[64];
  std::snprintf(angle_text, sizeof(angle_text), "Angle: %.2f deg", fit.steering_angle_deg);
  cv::putText(
    view, angle_text, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 1.0, lane_text_color, 2);

  if (!fit.valid) {
    cv::putText(
      view, "Lane not detected", cv::Point(30, 110), cv::FONT_HERSHEY_SIMPLEX, 1.0,
      lane_text_color, 2);
  }

  char sides_text[64];
  std::snprintf(
    sides_text, sizeof(sides_text), "L: %s  R: %s", left_fit.valid ? "OK" : "--",
    right_fit.valid ? "OK" : "--");
  cv::putText(
    view, sides_text, cv::Point(30, 150), cv::FONT_HERSHEY_SIMPLEX, 1.0, lane_text_color, 2);

  // --- Stop-line detection ---

  const StoplineResult stopline = find_stopline(white, origin, meters_per_pixel);

  if (stopline.valid) {
    cv::rectangle(view, stopline.bounding_box, cv::Scalar(0, 0, 255), 3);
  }

  std_msgs::msg::Float64MultiArray stopline_msg;
  stopline_msg.data = {stopline.distance_m, stopline.valid ? 1.0 : 0.0};
  stopline_publisher_->publish(stopline_msg);

  const cv::Scalar stopline_text_color =
    stopline.valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  // Anchored to the bottom of the frame so it doesn't collide with the lane section's
  // top-anchored text above.
  char distance_text[64];
  std::snprintf(distance_text, sizeof(distance_text), "Distance: %.2f m", stopline.distance_m);
  cv::putText(
    view, distance_text, cv::Point(30, view.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 1.0,
    stopline_text_color, 2);

  if (!stopline.valid) {
    cv::putText(
      view, "Stopline not detected", cv::Point(30, view.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 1.0,
      stopline_text_color, 2);
  }

  cv::imshow("Perception", view);
  cv::waitKey(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Perception>());
  rclcpp::shutdown();
  return 0;
}
