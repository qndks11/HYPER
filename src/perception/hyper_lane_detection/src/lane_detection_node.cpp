#include "hyper_lane_detection/lane_detection_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

using hyper_lane_detection::ElpCameraCapture;
using hyper_lane_detection::InputBackend;
using hyper_lane_detection::parse_input_backend;
using hyper_lane_detection::to_string;

namespace
{
// ROI trapezoid corners as (row_ratio, col_ratio) of the source image. Bottom corner ratios
// outside [0, 1] flare past the frame's own edges to capture more near-field width than the
// frame alone shows at that row -- but warpPerspective can only sample pixels that actually exist
// in the source frame, so the wider that flare, the more of bird_eye()'s output is unavoidably
// black (see its corner triangles). Tune against the actual camera feed.
constexpr double kRoiTopLeftRow = 0.48,  kRoiTopLeftCol = 0.35;
constexpr double kRoiTopRightRow = 0.48, kRoiTopRightCol = 0.65;
constexpr double kRoiBottomLeftRow = 1,  kRoiBottomLeftCol = -2;
constexpr double kRoiBottomRightRow = 1, kRoiBottomRightCol = 3;

// Bird's-eye output height as a multiple of the source frame height, i.e. how much farther
// down the road the BEV view looks. Width is left unscaled.
constexpr double kBevHeightScale = 0.5;

// Rear-camera-only ROI/BEV equivalents of the two constant groups above -- the rear camera needs
// to see farther and more clearly than the front's road-lane ROI, to pick up the parking bay's
// side line earlier while backing in. Top row ratio lowered (0.48 -> 0.30) to pull in ground
// closer to the horizon than the front ROI reaches; height scale raised (0.5 -> 0.8) so that
// captured range is stretched across more output pixels, giving walk_lane_chain()/fit_lane() finer
// pixel resolution to work with instead of the same detail compressed into fewer rows. Top col
// ratios and the bottom flare are left matching the front's, since the concern here is depth, not
// width. Front camera (kRoi* /kBevHeightScale above) is untouched. Tune against the actual rear
// camera feed, same as the front ROI's own comment says.
constexpr double kRearRoiTopLeftRow = 0.48,  kRearRoiTopLeftCol = 0.35;
constexpr double kRearRoiTopRightRow = 0.48, kRearRoiTopRightCol = 0.65;
constexpr double kRearRoiBottomLeftRow = 1,  kRearRoiBottomLeftCol = -2;
constexpr double kRearRoiBottomRightRow = 1, kRearRoiBottomRightCol = 3;
constexpr double kRearBevHeightScale = 0.8;

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
// fit_lane_curve() needs more points than fit_lane() -- estimating a curvature term (2 unknowns:
// a, b) from noisy pixel data needs more spread than estimating a direction alone.
constexpr int kMinCurveFitPoints = 5;
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
constexpr double kNumLaneInScreen = 6.2;  // how many lanes fit across the BEV image width
constexpr double kArrowLength = 100.0;

// Empirical half-lane-width offset [m] from a single tracked lane line to the estimated lane
// center, calibrated back when only the right lane was tracked. Baked into fit_lane()'s offset_m,
// signed per side (subtracted for the right lane, added for the left, since the center sits on
// opposite sides of each line), so each side's own biased offset reports a centered estimate
// even though the two sides are no longer combined into one fit.
constexpr double kLaneCenterOffsetBiasM = kLaneWidthMeters / 2.0;

// Rear camera equivalent of kLaneCenterOffsetBiasM: while backing into a parking bay, only one
// side line is visible and it is not the near side of a same-width paired lane, so the
// lane-center assumption above doesn't apply. Instead the target is a fixed standoff from that
// one line, measured to the vehicle's own centerline -- body_width/2 (~0.5m, see vehicle.xacro)
// plus a small margin, so the body's near-side edge clears the line instead of crossing it.
constexpr double kRearParkingLineStandoffM = 0.85;

// Empirical gain [m per degree] converting a tracked lane line's own outward lean --
// steering_angle_deg's magnitude, in the direction that means the line is angling away from the
// vehicle rather than toward it -- into extra lateral offset correction. kLaneCenterOffsetBiasM
// alone assumes the visible line runs parallel to the vehicle's heading; once it's visibly
// leaning outward that assumption undershoots the true center offset, and more so the more it
// leans. Applied to both sides unconditionally, since each side is always published as its own
// independent single-line estimate now. Untuned placeholder -- adjust against real footage.
constexpr double kOutwardLeanGainMPerDeg = 0.04;

// A stop-line bar spans most of the lane, so its bounding box is much wider than it is tall; a
// single zebra-crossing stripe is comparatively close to square. This floor separates the two.
constexpr double kMinStoplineAspectRatio = 3.0;
// The stop-line bar must span at least this fraction of the BEV image width to count -- rules
// out narrower marks (e.g. a single crossing stripe) that happen to pass the aspect ratio test.
constexpr double kMinStoplineWidthFraction = 0.10;
// Floor on contour area [px^2] to reject small mask noise before the shape checks run.
constexpr double kMinStoplineAreaPx = 500.0;

constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;

// Height [px] of the black strip appended below the BEV image to hold the overlay text. Fixed
// regardless of the incoming frame size so the 8 possible lines (6 top-anchored, 2 bottom-
// anchored) always have enough pitch between them -- the overlap this fixes came from anchoring
// text directly onto the BEV image, whose height shrinks with kBevHeightScale and the source
// frame size and can end up shorter than the text block itself.
constexpr int kTextPanelHeight = 360;
constexpr int kTextLinePitch = 36;

// Row margin [px] pushing the vehicle's reference point (origin, in image_callback) below the
// BEV frame's bottom edge, rather than pinning it exactly to the last visible row. The camera's
// visible bottom row isn't necessarily where the vehicle physically sits (e.g. a hood/bumper gap
// the camera doesn't capture), so this treats that row as the zero point and moves the true
// reference point this many pixels closer to the vehicle (i.e. "behind" the frame).
constexpr double kOriginBelowFrameMarginPx = 0.0;

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

LaneDetection::LaneDetection() : Node{"lane_detection"}
{
  const std::string backend_param =
    declare_parameter<std::string>("input_backend", "ros_compressed");
  const auto backend = parse_input_backend(backend_param);
  if (!backend) {
    RCLCPP_FATAL(
      get_logger(),
      "Invalid input_backend '%s' -- expected one of: direct_usb, ros_raw, ros_compressed",
      backend_param.c_str());
    throw std::invalid_argument("lane_detection_node: invalid input_backend '" + backend_param + "'");
  }
  input_backend_ = *backend;

  lane_center_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);
  stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/detection", 10);
  rear_lane_center_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/lane/rear_center", 10);
  rear_stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/rear_detection", 10);

  cv::namedWindow("LaneDetection", cv::WINDOW_NORMAL);
  cv::resizeWindow(
    "LaneDetection", kWindowWidth,
    static_cast<int>(kWindowHeight * kBevHeightScale) + kTextPanelHeight);

  switch (input_backend_) {
    case InputBackend::kRosCompressed:
      // Both cameras arrive over the "compressed" transport (JPEG) rather than raw -- the
      // vehicle's link to wherever these frames are encoded/decoded is bandwidth-constrained.
      // image_transport negotiates the matching .../compressed topic and hands the callback an
      // already-decoded Image, same as a plain subscription would. Kept only for A/B comparison
      // and rollback against input_backend:=direct_usb.
      image_subscriber_ = image_transport::create_subscription(
        this, "/image_raw",
        std::bind(&LaneDetection::image_callback, this, std::placeholders::_1), "compressed");
      rear_image_subscriber_ = image_transport::create_subscription(
        this, "/rear_image_raw",
        std::bind(&LaneDetection::rear_image_callback, this, std::placeholders::_1),
        "compressed");
      break;

    case InputBackend::kRosRaw:
      // Gazebo simulation: ros_gz_bridge already publishes plain sensor_msgs/Image, so no
      // image_transport/compressed subscription is needed here at all.
      raw_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&LaneDetection::raw_image_callback, this, std::placeholders::_1));
      raw_rear_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/rear_image_raw", 10,
        std::bind(&LaneDetection::raw_rear_image_callback, this, std::placeholders::_1));
      break;

    case InputBackend::kDirectUsb:
      // Real vehicle: this node owns the physical ELP camera directly (see
      // setup_direct_usb_capture()) -- no ROS image topic involved for the front camera, and no
      // rear-camera equivalent.
      setup_direct_usb_capture();
      break;
  }

  // The rear window only ever receives frames under ros_compressed/ros_raw -- direct_usb has no
  // rear-camera path, so skip creating a window that would otherwise just sit blank.
  if (input_backend_ != InputBackend::kDirectUsb) {
    cv::namedWindow("LaneDetectionRear", cv::WINDOW_NORMAL);
    cv::resizeWindow(
      "LaneDetectionRear", kWindowWidth,
      static_cast<int>(kWindowHeight * kRearBevHeightScale) + kTextPanelHeight);
  }

  RCLCPP_INFO(
    get_logger(), "LaneDetection started (input_backend=%s)", to_string(input_backend_).c_str());
}

void LaneDetection::setup_direct_usb_capture()
{
  ElpCameraCapture::Config config;
  config.device = declare_parameter<std::string>("video_device", "/dev/video_elp");
  config.width = declare_parameter<int>("image_width", 1280);
  config.height = declare_parameter<int>("image_height", 720);
  config.framerate = declare_parameter<double>("framerate", 30.0);
  config.calibration_file = declare_parameter<std::string>(
    "calibration_file",
    ament_index_cpp::get_package_share_directory("hyper_camera") +
      "/config/ELP-USBGS1200P01-KL170.yaml");

  elp_capture_ = std::make_unique<ElpCameraCapture>(get_logger());
  if (!elp_capture_->open(config)) {
    // ElpCameraCapture::open() has already logged the specific failure; a node that "starts" with
    // no working camera would otherwise spin forever silently doing nothing.
    throw std::runtime_error("lane_detection_node: direct_usb camera setup failed");
  }

  const auto period_s = std::chrono::duration<double>(1.0 / config.framerate);
  capture_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period_s),
    std::bind(&LaneDetection::capture_timer_callback, this));
}

void LaneDetection::capture_timer_callback()
{
  cv::Mat rectified;
  if (!elp_capture_->read(rectified)) {
    // Read failure is already logged inside ElpCameraCapture::read(); skip this tick rather than
    // aborting the node over what may be a transient USB glitch.
    return;
  }
  process_frame(rectified, now(), CameraSide::kFront);
}

cv::Mat LaneDetection::build_transform(
  int src_height, int src_width, int dst_height, int dst_width, bool use_rear_roi) const
{
  const double top_left_row = use_rear_roi ? kRearRoiTopLeftRow : kRoiTopLeftRow;
  const double top_left_col = use_rear_roi ? kRearRoiTopLeftCol : kRoiTopLeftCol;
  const double top_right_row = use_rear_roi ? kRearRoiTopRightRow : kRoiTopRightRow;
  const double top_right_col = use_rear_roi ? kRearRoiTopRightCol : kRoiTopRightCol;
  const double bottom_left_row = use_rear_roi ? kRearRoiBottomLeftRow : kRoiBottomLeftRow;
  const double bottom_left_col = use_rear_roi ? kRearRoiBottomLeftCol : kRoiBottomLeftCol;
  const double bottom_right_row = use_rear_roi ? kRearRoiBottomRightRow : kRoiBottomRightRow;
  const double bottom_right_col = use_rear_roi ? kRearRoiBottomRightCol : kRoiBottomRightCol;

  const std::vector<cv::Point2f> src{
    {static_cast<float>(src_width * top_left_col), static_cast<float>(src_height * top_left_row)},
    {static_cast<float>(src_width * top_right_col), static_cast<float>(src_height * top_right_row)},
    {static_cast<float>(src_width * bottom_right_col), static_cast<float>(src_height * bottom_right_row)},
    {static_cast<float>(src_width * bottom_left_col), static_cast<float>(src_height * bottom_left_row)}};

  const std::vector<cv::Point2f> dst{
    {0.0f, 0.0f},
    {static_cast<float>(dst_width), 0.0f},
    {static_cast<float>(dst_width), static_cast<float>(dst_height)},
    {0.0f, static_cast<float>(dst_height)}};

  return cv::getPerspectiveTransform(src, dst);
}

cv::Mat LaneDetection::bird_eye(const cv::Mat & image, bool use_rear_roi) const
{
  const double height_scale = use_rear_roi ? kRearBevHeightScale : kBevHeightScale;
  const cv::Size dst_size(image.cols, static_cast<int>(image.rows * height_scale));
  const cv::Mat transform =
    build_transform(image.rows, image.cols, dst_size.height, dst_size.width, use_rear_roi);
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, dst_size);
  return warped;
}

cv::Mat LaneDetection::yellow_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  // Hue floor is 22, not the more permissive 15 a plain "yellow-ish" range might suggest: sampling
  // the course texture's own pixels showed lane paint (including its anti-aliased blends against
  // the gray road, which only shift S/V, not hue) clustering at H 26-31, while brown dirt/curb
  // pixels sit in a separate cluster at H 16-19 with a clean, essentially empty gap at H 20-25 --
  // so 22 excludes the brown without narrowing the   yellow paint's own hue range at all.
  cv::inRange(hsv, cv::Scalar(22, 80, 80), cv::Scalar(35, 255, 255), mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

cv::Mat LaneDetection::white_mask(const cv::Mat & image) const
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 60, 255), mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

std::vector<cv::Point> LaneDetection::walk_lane_chain(
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

LaneDetection::LaneFitResult LaneDetection::fit_lane(
  const std::vector<cv::Point> & points, const cv::Point2d & origin, int width,
  LaneSide side, double center_bias_m) const
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
  // Signed perpendicular distance from the vehicle to the fitted line itself, rather than just to
  // its anchor point p0 -- for a steeply angled fit, p0's own x-position is an increasingly poor
  // stand-in for where the line actually sits at the vehicle's row. This is the cross product of
  // (origin - p0) with the unit direction vector; it reduces to the old (p0.x - origin.x) formula
  // exactly when direction is vertical (a straight-ahead lane).
  const double perp_px = (origin.x - p0.x) * direction.y - (origin.y - p0.y) * direction.x;
  // The target sits on the vehicle's side of a right line, but on the far side of a left line, so
  // the bias flips sign between the two -- see the call sites for what center_bias_m represents.
  const double signed_bias_m = side == LaneSide::kRight ? -center_bias_m : center_bias_m;
  result.offset_m = perp_px * meters_per_pixel + signed_bias_m;

  result.valid = true;
  return result;
}

LaneDetection::LaneFitResult LaneDetection::fit_lane_curve(
  const std::vector<cv::Point> & points, const cv::Point2d & origin, int width,
  LaneSide side, double center_bias_m) const
{
  LaneFitResult result;
  if (static_cast<int>(points.size()) < kMinCurveFitPoints) {
    return result;
  }

  const int n = static_cast<int>(points.size());
  const cv::Point2d p0(points[0].x, points[0].y);

  // Same near-field-weighted arc length as fit_lane(), and the same anchor-at-p0 shift (x', y') --
  // fitting x' = a*y'^2 + b*y' (no free constant term) forces the curve through p0 exactly.
  std::vector<double> s(n);
  s[0] = 0.0;
  for (int i = 1; i < n; ++i) s[i] = s[i - 1] + cv::norm(points[i] - points[i - 1]);
  const double s_far = s.back();

  double sy2 = 0.0, sy3 = 0.0, sy4 = 0.0, sxy = 0.0, sxy2 = 0.0;
  for (int i = 0; i < n; ++i) {
    const double w = (s_far - s[i]) + 1.0;
    const double yp = points[i].y - p0.y;
    const double xp = points[i].x - p0.x;
    const double yp2 = yp * yp;
    sy2 += w * yp2;
    sy3 += w * yp2 * yp;
    sy4 += w * yp2 * yp2;
    sxy += w * xp * yp;
    sxy2 += w * xp * yp2;
  }
  // Degenerate if the chain barely spreads away from its own anchor -- same conceptual role as
  // fit_lane()'s eigenvalue floor, just measured directly on y' spread here instead.
  if (sy2 < kMinLineFitVariance) {
    return result;
  }

  const cv::Mat normal_matrix = (cv::Mat_<double>(2, 2) << sy4, sy3, sy3, sy2);
  const cv::Mat rhs = (cv::Mat_<double>(2, 1) << sxy2, sxy);
  cv::Mat solution;
  // DECOMP_SVD degrades gracefully on a near-singular system (e.g. too few distinct depths to
  // pin down curvature confidently) instead of failing outright the way a strict LU solve would.
  cv::solve(normal_matrix, rhs, solution, cv::DECOMP_SVD);
  const double a = solution.at<double>(0);
  const double b = solution.at<double>(1);

  // Tangent at the near point (y'=0): dx/dy = b. A step of dy<0 (away from the vehicle, up the
  // image) gives dx = b*dy, so the direction vector is proportional to (-b, -1) -- matching
  // fit_lane()'s convention that direction.y is negative when pointing away from the vehicle.
  const double direction_norm = std::sqrt(b * b + 1.0);
  const cv::Point2d direction(-b / direction_norm, -1.0 / direction_norm);

  // Sampled across the chain's actual observed reach (p0 down to the farthest point), for drawing
  // a real curve instead of fit_lane()'s two-point straight segment.
  constexpr int kCurveDrawSamples = 12;
  const cv::Point2d p_far(points[n - 1].x, points[n - 1].y);
  result.curve_points.reserve(kCurveDrawSamples + 1);
  for (int i = 0; i <= kCurveDrawSamples; ++i) {
    const double t = static_cast<double>(i) / kCurveDrawSamples;
    const double yp = t * (p_far.y - p0.y);
    const double xp = a * yp * yp + b * yp;
    result.curve_points.push_back(cv::Point(
      static_cast<int>(std::lround(p0.x + xp)), static_cast<int>(std::lround(p0.y + yp))));
  }

  result.steering_angle_deg = -std::atan2(direction.x, -direction.y) * 180.0 / CV_PI;

  // Curvature from the second derivative at the near point (f''(0) = 2a): kappa = f'' / (1+f'^2)^1.5,
  // radius = 1/kappa. Clamped to the same flat-line sentinel fit_lane() always reports, since a
  // near-zero `a` (an essentially straight chain) would otherwise blow up toward infinity.
  const double curvature = std::abs(2.0 * a) / std::pow(1.0 + b * b, 1.5);
  result.curvature_radius_px = curvature > 1.0 / kMaxCurvatureRadiusPx ?
    1.0 / curvature : kMaxCurvatureRadiusPx;

  const double meters_per_pixel = kNumLaneInScreen * kLaneWidthMeters / static_cast<double>(width);
  // Same perpendicular-distance-to-tangent-line formula as fit_lane(), just fed this curve's
  // near-point tangent direction instead of the global straight-line direction (standard
  // Frenet-style local approximation for lateral offset to a curve).
  const double perp_px = (origin.x - p0.x) * direction.y - (origin.y - p0.y) * direction.x;
  const double signed_bias_m = side == LaneSide::kRight ? -center_bias_m : center_bias_m;
  result.offset_m = perp_px * meters_per_pixel + signed_bias_m;

  result.valid = true;
  return result;
}

LaneDetection::StoplineResult LaneDetection::find_stopline(
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

void LaneDetection::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  process_frame(cv_ptr->image, msg->header.stamp, CameraSide::kFront);
}

void LaneDetection::rear_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception (rear): %s", e.what());
    return;
  }
  process_frame(cv_ptr->image, msg->header.stamp, CameraSide::kRear);
}

void LaneDetection::raw_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  process_frame(cv_ptr->image, msg->header.stamp, CameraSide::kFront);
}

void LaneDetection::raw_rear_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception (rear): %s", e.what());
    return;
  }
  process_frame(cv_ptr->image, msg->header.stamp, CameraSide::kRear);
}

void LaneDetection::process_frame(
  const cv::Mat & image, const rclcpp::Time & stamp, CameraSide side)
{
  // Reserved for future header-stamped outputs / latency logging -- the current output messages
  // (std_msgs/Float64MultiArray) carry no header of their own to stamp.
  (void)stamp;

  const bool is_rear = side == CameraSide::kRear;
  const auto & lane_publisher = is_rear ? rear_lane_center_publisher_ : lane_center_publisher_;
  const auto & stopline_publisher = is_rear ? rear_stopline_publisher_ : stopline_publisher_;
  const std::string window_name = is_rear ? "LaneDetectionRear" : "LaneDetection";
  const double lane_center_bias_m = is_rear ? kRearParkingLineStandoffM : kLaneCenterOffsetBiasM;
  const bool use_curve_fit = is_rear;
  const bool use_rear_roi = is_rear;

  const cv::Mat warped = bird_eye(image, use_rear_roi);
  const cv::Point2d origin(warped.cols / 2.0, warped.rows - 1.0 + kOriginBelowFrameMarginPx);
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
  std::vector<cv::Point> left_points = left_chain;

  for (const auto & p : right_points) cv::circle(view, p, 3, cv::Scalar(0, 165, 255), -1);
  for (const auto & p : left_points) cv::circle(view, p, 3, cv::Scalar(255, 0, 0), -1);

  LaneFitResult right_fit = use_curve_fit ?
    fit_lane_curve(right_points, origin, warped.cols, LaneSide::kRight, lane_center_bias_m) :
    fit_lane(right_points, origin, warped.cols, LaneSide::kRight, lane_center_bias_m);
  LaneFitResult left_fit = use_curve_fit ?
    fit_lane_curve(left_points, origin, warped.cols, LaneSide::kLeft, lane_center_bias_m) :
    fit_lane(left_points, origin, warped.cols, LaneSide::kLeft, lane_center_bias_m);

  // Each side is published as its own independent estimate now (no more width-based decision
  // between left-only / right-only / averaged-both). If a line is angling outward -- away from
  // the vehicle rather than parallel to it -- kLaneCenterOffsetBiasM alone (baked into
  // fit_lane()'s offset_m) undershoots the true center offset, more so the more it leans; correct
  // for that on whichever side(s) are valid. See kOutwardLeanGainMPerDeg.
  auto apply_outward_lean_correction = [](LaneFitResult & f, LaneSide side) {
    const double outward_lean_deg =
      side == LaneSide::kRight ? -f.steering_angle_deg : f.steering_angle_deg;
    if (outward_lean_deg <= 0.0) {
      return;
    }
    const double extra_m = outward_lean_deg * kOutwardLeanGainMPerDeg;
    f.offset_m += side == LaneSide::kRight ? -extra_m : extra_m;
  };
  if (right_fit.valid) apply_outward_lean_correction(right_fit, LaneSide::kRight);
  if (left_fit.valid) apply_outward_lean_correction(left_fit, LaneSide::kLeft);

  if (right_fit.valid) cv::polylines(view, right_fit.curve_points, false, cv::Scalar(255, 0, 255), 3);
  if (left_fit.valid) cv::polylines(view, left_fit.curve_points, false, cv::Scalar(0, 255, 255), 3);

  // steering_angle_deg is defined as -atan2(direction.x, -direction.y) in fit_lane, so a
  // rightward-tilting fitted line comes out as a *negative* angle; reconstructing a direction
  // vector from the angle therefore needs sin() negated too (-sin, not sin), or the drawn
  // direction ends up mirrored left/right from the line it's supposed to represent.
  auto draw_side_overlay = [&](const LaneFitResult & f, cv::Scalar color) {
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
  draw_side_overlay(left_fit, cv::Scalar(255, 0, 0));
  draw_side_overlay(right_fit, cv::Scalar(0, 255, 0));

  std_msgs::msg::Float64MultiArray lane_msg;
  lane_msg.data = {
    left_fit.steering_angle_deg, left_fit.offset_m, left_fit.valid ? 1.0 : 0.0,
    right_fit.steering_angle_deg, right_fit.offset_m, right_fit.valid ? 1.0 : 0.0};
  lane_publisher->publish(lane_msg);

  const cv::Scalar lane_text_color = (left_fit.valid || right_fit.valid) ?
    cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  // Dedicated black strip below the BEV image for all overlay text, sized independently of the
  // BEV image's own (config-dependent) height so the lines below never run out of room.
  cv::copyMakeBorder(
    view, view, 0, kTextPanelHeight, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  const int panel_top = view.rows - kTextPanelHeight;

  char left_text[64];
  if (left_fit.valid) {
    std::snprintf(left_text, sizeof(left_text), "L: %.2f m / %.2f deg",
      left_fit.offset_m, left_fit.steering_angle_deg);
  } else {
    std::snprintf(left_text, sizeof(left_text), "L: --");
  }
  cv::putText(
    view, left_text, cv::Point(30, panel_top + kTextLinePitch * 1), cv::FONT_HERSHEY_SIMPLEX,
    1.0, lane_text_color, 2);

  char right_text[64];
  if (right_fit.valid) {
    std::snprintf(right_text, sizeof(right_text), "R: %.2f m / %.2f deg",
      right_fit.offset_m, right_fit.steering_angle_deg);
  } else {
    std::snprintf(right_text, sizeof(right_text), "R: --");
  }
  cv::putText(
    view, right_text, cv::Point(30, panel_top + kTextLinePitch * 2), cv::FONT_HERSHEY_SIMPLEX,
    1.0, lane_text_color, 2);

  if (!left_fit.valid && !right_fit.valid) {
    cv::putText(
      view, "Lane not detected", cv::Point(30, panel_top + kTextLinePitch * 3),
      cv::FONT_HERSHEY_SIMPLEX, 1.0, lane_text_color, 2);
  }

  // --- Stop-line detection ---

  const StoplineResult stopline = find_stopline(white, origin, meters_per_pixel);

  if (stopline.valid) {
    cv::rectangle(view, stopline.bounding_box, cv::Scalar(0, 0, 255), 3);
  }

  std_msgs::msg::Float64MultiArray stopline_msg;
  stopline_msg.data = {stopline.distance_m, stopline.valid ? 1.0 : 0.0};
  stopline_publisher->publish(stopline_msg);

  const cv::Scalar stopline_text_color =
    stopline.valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  // Anchored below the lane section's own lines (rows 1-3 above), with a blank row of padding
  // between them, so it never collides with that block regardless of how many of those lines
  // are showing.
  char distance_text[64];
  std::snprintf(distance_text, sizeof(distance_text), "Distance: %.2f m", stopline.distance_m);
  cv::putText(
    view, distance_text, cv::Point(30, panel_top + kTextLinePitch * 8), cv::FONT_HERSHEY_SIMPLEX,
    1.0, stopline_text_color, 2);

  if (!stopline.valid) {
    cv::putText(
      view, "Stopline not detected", cv::Point(30, panel_top + kTextLinePitch * 9),
      cv::FONT_HERSHEY_SIMPLEX, 1.0, stopline_text_color, 2);
  }

  cv::imshow(window_name, view);
  cv::waitKey(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LaneDetection>());
  } catch (const std::exception & e) {
    // Startup failures (bad input_backend, direct_usb camera/calibration failure) throw rather
    // than leaving a half-initialized node spinning; surface them clearly instead of an opaque
    // uncaught-exception crash.
    RCLCPP_FATAL(rclcpp::get_logger("lane_detection"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
