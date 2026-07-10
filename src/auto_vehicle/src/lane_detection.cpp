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

// A few points of slack past the minimum a well-conditioned line-direction fit needs (2 points
// determine a direction exactly; more give the weighted eigen solve something to average over).
constexpr int kMinLanePoints = 6;
// Floor on the fitted line direction's weighted variance (see fit_lane): below this the chain is
// numerically degenerate (points effectively coincident), so the eigen solve isn't trustworthy.
constexpr double kMinLineFitVariance = 1e-6;
// A straight-line fit has zero curvature everywhere; this sentinel radius is reported in its
// place rather than infinity, so downstream consumers (e.g. the controller's curvature-based
// speed throttle) see "very straight" instead of a non-finite value.
constexpr double kMaxCurvatureRadiusPx = 1e6;

constexpr double kLaneWidthMeters = 3.7;
constexpr double kArrowLength = 100.0;

constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;

// Bird's-eye output height as a multiple of the source frame height, i.e. how much farther
// down the road the BEV view looks. Width is left unscaled.
constexpr double kBevHeightScale = 1.35;

// How far right (in degrees) the chain's start has to already be curving before the right lane
// is judged about to sweep out of frame -- see is_right_turn().
constexpr double kRightTurnAngleDeg = 15.0;

// A lane chain's start is judged to be curving right if the direction toward its last point (cl)
// sits more than kRightTurnAngleDeg to the right of the direction toward its 2nd point (c1),
// both measured from the chain's first point (c0) -- i.e. the chain has already swung right
// between those two points, not just angled that way from the start. This looks at only the
// chain's first three points -- right where the vehicle is -- so it catches the turn as early as
// possible, before the rest of the right lane has actually left the frame.
bool is_right_turn(const std::vector<cv::Point> & chain)
{
  if (chain.size() < 4) {
    return false;
  }
  const cv::Point baseline = chain[3] - chain[0];  // c3 -> c0
  const cv::Point test = chain[chain.size()-1] - chain[0];      // c[l-1] -> c0
  const double cross =
    static_cast<double>(baseline.x) * test.y - static_cast<double>(baseline.y) * test.x;
  const double dot =
    static_cast<double>(baseline.x) * test.x + static_cast<double>(baseline.y) * test.y;
  return std::atan2(cross, dot) * 180.0 / CV_PI > kRightTurnAngleDeg;
}

// Reflects every point's x coordinate about axis_x, leaving y unchanged; applying it twice with
// the same axis returns the original points exactly. Used to walk the left lane with the exact
// same seed and ranking logic that normally walks the right one: reflection reverses
// orientation, so on the mirrored cloud, walk_lane_chain()'s right-half seed condition picks up
// what were originally left-half pixels, and its candidate ranking's handedness comes out
// reversed too -- exactly the two changes needed to walk the other lane, with no changes to
// walk_lane_chain() itself.
std::vector<cv::Point> mirror_points_x(const std::vector<cv::Point> & points, double axis_x)
{
  const int axis = static_cast<int>(std::lround(2.0 * axis_x));
  std::vector<cv::Point> mirrored;
  mirrored.reserve(points.size());
  for (const auto & p : points) mirrored.emplace_back(axis - p.x, p.y);
  return mirrored;
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
    if (p.x < origin.x || p.y < 1.2 * kWindowHeight) {
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
  const std::vector<cv::Point> & points, const cv::Point2d & origin, int width,
  bool tracking_left_lane) const
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

  // Offset is taken at the chain's near point (p0 itself) rather than extrapolated out to the
  // vehicle's actual row (origin.y). The camera sits back from the front wheels, so the visible
  // lane can start well short of the vehicle row; extrapolating that gap isn't safe, especially
  // mid-turn. The near point is the closest actual observation, so it's used as-is.
  //
  // The chain tracks whichever lane boundary is currently visible -- normally the right one, but
  // the left one when the right lane has swept out of frame in a turn (see image_callback) -- and
  // since the two boundaries sit on opposite sides of lane center, the constant term converting
  // "distance from the tracked line to the vehicle" into "distance from lane center to the
  // vehicle" flips sign between them.
  const double meters_per_pixel = kLaneWidthMeters / static_cast<double>(width);
  const double lane_center_bias = tracking_left_lane ? 0.55 : -0.55;
  result.offset_m = (p0.x - origin.x) * meters_per_pixel + lane_center_bias;

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
  std::vector<cv::Point> chain = walk_lane_chain(yellow_points, origin);
  bool tracking_left_lane = false;

  // The right lane sweeps out of frame fast in a right turn, so fall back to the left lane
  // boundary as soon as the right chain is either unusable or already curving sharply right at
  // its start. Walking the left lane reuses walk_lane_chain() unmodified on a mirrored point
  // cloud -- see mirror_points_x() -- so the seed and ranking both come out flipped to the left
  // side for free.
  if (static_cast<int>(chain.size()) < kMinLanePoints || is_right_turn(chain)) {
    const std::vector<cv::Point> mirrored_yellow = mirror_points_x(yellow_points, origin.x);
    const std::vector<cv::Point> mirrored_chain = walk_lane_chain(mirrored_yellow, origin);
    if (!mirrored_chain.empty()) {
      chain = mirror_points_x(mirrored_chain, origin.x);
      tracking_left_lane = true;
    }
  }

  // Fall back to the previous frame's lane (and which boundary it tracked) when nothing is found
  // this frame.
  std::vector<cv::Point> points = chain;
  if (points.empty()) {
    points = prev_points_;
    tracking_left_lane = prev_tracking_left_lane_;
  } else {
    prev_points_ = points;
    prev_tracking_left_lane_ = tracking_left_lane;
  }

  for (const auto & p : points) cv::circle(view, p, 3, cv::Scalar(0, 165, 255), -1);

  const LaneFitResult fit = fit_lane(points, origin, warped.cols, tracking_left_lane);

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


  const cv::Scalar text_color = fit.valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  char offset_text[64];
  std::snprintf(offset_text, sizeof(offset_text), "Offset: %.2f m", fit.offset_m);
  cv::putText(view, offset_text, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

  char angle_text[64];
  std::snprintf(angle_text, sizeof(angle_text), "Angle: %.2f deg", fit.steering_angle_deg);
  cv::putText(view, angle_text, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

  if (!fit.valid) cv::putText(view, "Lane not detected", cv::Point(30, 110), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);

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
