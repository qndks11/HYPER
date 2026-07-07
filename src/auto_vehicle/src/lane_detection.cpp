#include "auto_vehicle/lane_detection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
// ROI trapezoid corners as (row_ratio, col_ratio) of the source image
constexpr double kRoiTopLeftRow = 0.40,  kRoiTopLeftCol = 0.25;
constexpr double kRoiTopRightRow = 0.40, kRoiTopRightCol = 0.75;
constexpr double kRoiBottomLeftRow = 1,  kRoiBottomLeftCol = -1;
constexpr double kRoiBottomRightRow = 1, kRoiBottomRightCol = 2;

// Polar right-lane search: from the bottom-center origin, sweep angle 0 (due right, along the
// bottom edge) to pi/2 (due forward) looking for the closest box of yellow pixels at each angle.
constexpr int kSearchBoxSize = 6;                  // px; the search box is kSearchBoxSize^2
constexpr double kSearchBoxFillThreshold = 0.30;   // fraction of the box that must be yellow
constexpr double kAngleStepRad = 0.1;
constexpr double kMaxAngleRad = CV_PI / 2.0;
constexpr double kBinarySearchTolerancePx = 1.0;

constexpr int kMinLanePoints = 4;
constexpr int kSplineSamples = 120;
constexpr double kMinCurvatureKappa = 1e-6;

constexpr double kLaneWidthMeters = 3.7;
constexpr double kArrowLength = 100.0;

constexpr int kThumbWidth = 480;
constexpr int kThumbHeight = 360;

// A natural cubic spline through (t_i, y_i), t strictly increasing. Used to fit x(t) and y(t)
// independently for a parametric curve, so unlike a single-valued x = f(y) fit, the curve can
// bend back on itself in x for a given y -- the case a fixed-column vertical search cannot
// represent at all.
class CubicSpline1D
{
public:
  CubicSpline1D(const std::vector<double> & t, const std::vector<double> & y) : t_(t), y_(y)
  {
    const int n = static_cast<int>(t_.size()) - 1;
    std::vector<double> h(std::max(n, 1));
    for (int i = 0; i < n; ++i) h[i] = t_[i + 1] - t_[i];

    // Thomas algorithm for the tridiagonal natural-spline system (M_0 = M_n = 0).
    std::vector<double> a(n + 1, 0.0), b(n + 1, 1.0), c(n + 1, 0.0), d(n + 1, 0.0);
    for (int i = 1; i < n; ++i) {
      a[i] = h[i - 1];
      b[i] = 2.0 * (h[i - 1] + h[i]);
      c[i] = h[i];
      d[i] = 6.0 * ((y_[i + 1] - y_[i]) / h[i] - (y_[i] - y_[i - 1]) / h[i - 1]);
    }

    std::vector<double> cp(n + 1, 0.0), dp(n + 1, 0.0);
    for (int i = 1; i <= n; ++i) {
      const double pivot = b[i] - a[i] * cp[i - 1];
      cp[i] = (i < n) ? c[i] / pivot : 0.0;
      dp[i] = (d[i] - a[i] * dp[i - 1]) / pivot;
    }

    m_.assign(n + 1, 0.0);
    m_[n] = dp[n];
    for (int i = n - 1; i >= 0; --i) m_[i] = dp[i] - cp[i] * m_[i + 1];
  }

  // Evaluates the spline and its first/second derivatives at parameter `t`.
  void evaluate(double t, double & value, double & first, double & second) const
  {
    const int n = static_cast<int>(t_.size()) - 1;
    const int i = std::clamp(
      static_cast<int>(std::upper_bound(t_.begin(), t_.end(), t) - t_.begin()) - 1, 0,
      std::max(n - 1, 0));

    const double h = t_[i + 1] - t_[i];
    const double a = t_[i + 1] - t;
    const double bb = t - t_[i];

    value = m_[i] * a * a * a / (6.0 * h) + m_[i + 1] * bb * bb * bb / (6.0 * h) +
            (y_[i] / h - m_[i] * h / 6.0) * a + (y_[i + 1] / h - m_[i + 1] * h / 6.0) * bb;
    first = -m_[i] * a * a / (2.0 * h) + m_[i + 1] * bb * bb / (2.0 * h) -
            (y_[i] / h - m_[i] * h / 6.0) + (y_[i + 1] / h - m_[i + 1] * h / 6.0);
    second = m_[i] * a / h + m_[i + 1] * bb / h;
  }

private:
  std::vector<double> t_;
  std::vector<double> y_;
  std::vector<double> m_;  // second derivatives at knots
};

std::vector<cv::Point> sample_spline(
  const CubicSpline1D & spline_x, const CubicSpline1D & spline_y, double t_max, int num_samples)
{
  std::vector<cv::Point> points;
  points.reserve(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    const double t = t_max * static_cast<double>(i) / static_cast<double>(num_samples - 1);
    double x, x1, x2, y, y1, y2;
    spline_x.evaluate(t, x, x1, x2);
    spline_y.evaluate(t, y, y1, y2);
    points.emplace_back(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)));
  }
  return points;
}

struct BoxCheck
{
  bool valid{false};
  double fill_ratio{0.0};
};

// Fraction of set pixels within a `size`x`size` box centered at `center`, clipped to the image.
// `valid` is false once the box has walked entirely off the image.
BoxCheck evaluate_box(const cv::Mat & binary, const cv::Point2d & center, int size)
{
  const int half = size / 2;
  const cv::Rect box(
    static_cast<int>(std::lround(center.x)) - half,
    static_cast<int>(std::lround(center.y)) - half, size, size);
  const cv::Rect clipped = box & cv::Rect(0, 0, binary.cols, binary.rows);
  if (clipped.area() <= 0) {
    return {};
  }
  const double ratio = static_cast<double>(cv::countNonZero(binary(clipped))) / clipped.area();
  return {true, ratio};
}

struct PolarHit
{
  cv::Point pixel;
  cv::Rect box;
};

// From `origin`, searches along the ray at angle `theta` (0 = due right, pi/2 = due forward) for
// the closest box that clears kSearchBoxFillThreshold. Exponentially doubles the radius outward
// (close to far) to bracket the first hit, then binary-searches the bracket down to within
// kBinarySearchTolerancePx. Returns nullopt if the ray leaves the image before any box qualifies.
std::optional<PolarHit> find_closest_box(
  const cv::Mat & binary, const cv::Point2d & origin, double theta)
{
  const cv::Point2d direction(std::cos(theta), -std::sin(theta));
  const auto sample = [&](double r) {
    return evaluate_box(binary, origin + direction * r, kSearchBoxSize);
  };

  double lo = 0.0;
  double hi = kSearchBoxSize;
  BoxCheck hit;
  while (true) {
    const BoxCheck check = sample(hi);
    if (!check.valid) {
      return std::nullopt;  // walked off the image without finding anything
    }
    if (check.fill_ratio >= kSearchBoxFillThreshold) {
      hit = check;
      break;
    }
    lo = hi;
    hi *= 2.0;
  }

  while (hi - lo > kBinarySearchTolerancePx) {
    const double mid = 0.5 * (lo + hi);
    const BoxCheck check = sample(mid);
    if (check.valid && check.fill_ratio >= kSearchBoxFillThreshold) {
      hi = mid;
      hit = check;
    } else {
      lo = mid;
    }
  }

  const cv::Point2d center = origin + direction * hi;
  const cv::Point pixel(
    static_cast<int>(std::lround(center.x)), static_cast<int>(std::lround(center.y)));
  const cv::Rect box(
    pixel.x - kSearchBoxSize / 2, pixel.y - kSearchBoxSize / 2, kSearchBoxSize, kSearchBoxSize);
  return PolarHit{pixel, box};
}
}  // namespace

LaneDetection::LaneDetection() : Node{"lane_detection"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&LaneDetection::image_callback, this, std::placeholders::_1));

  lane_center_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);

  cv::namedWindow("Dashboard", cv::WINDOW_NORMAL);
  cv::resizeWindow("Dashboard", kThumbWidth * 2, kThumbHeight * 2);

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

LaneDetection::PolarSearchResult LaneDetection::polar_search(const cv::Mat & binary) const
{
  PolarSearchResult result;
  const cv::Point2d origin(binary.cols / 2.0, binary.rows - 1.0);
  bool found = 0;

  for (double theta = 0.0; theta <= kMaxAngleRad + 1e-6; theta += kAngleStepRad) {
    const std::optional<PolarHit> hit = find_closest_box(binary, origin, theta);
    if (!hit) {
      if (found) break;  // disconnect: the lane trail stops here
      continue;
    }
    found = 1;
    result.points.push_back(hit->pixel);
    result.boxes.push_back(hit->box);
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

  std::vector<double> t(points.size()), xs(points.size()), ys(points.size());
  t[0] = 0.0;
  xs[0] = points[0].x;
  ys[0] = points[0].y;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    t[i] = t[i - 1] + std::max(std::hypot(dx, dy), 1e-3);  // keep t strictly increasing
    xs[i] = points[i].x;
    ys[i] = points[i].y;
  }

  const CubicSpline1D spline_x(t, xs);
  const CubicSpline1D spline_y(t, ys);

  result.spline_points = sample_spline(spline_x, spline_y, t.back(), kSplineSamples);

  // The natural boundary condition forces zero curvature exactly at t=0, so evaluate the
  // vehicle-side heading/curvature one knot in rather than at the true endpoint.
  const double t_eval = t[1];
  double x_val, x1, x2, y_val, y1, y2;
  spline_x.evaluate(t_eval, x_val, x1, x2);
  spline_y.evaluate(t_eval, y_val, y1, y2);

  const double speed_sq = x1 * x1 + y1 * y1;
  const double kappa =
    speed_sq > 1e-9 ? (x1 * y2 - y1 * x2) / std::pow(speed_sq, 1.5) : 0.0;
  result.curvature_radius_px = 1.0 / std::max(std::abs(kappa), kMinCurvatureKappa);

  // Heading of travel relative to straight ahead (the -y axis); atan2 stays well-behaved even
  // when the lane runs nearly horizontal, which is exactly the steep-curve case this is for.
  result.steering_angle_deg = std::atan2(x1, -y1) * 180.0 / CV_PI;

  const double meters_per_pixel = kLaneWidthMeters / static_cast<double>(width);
  result.offset_m = (points[0].x - origin.x) * meters_per_pixel;

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

  cv::Mat row1, row2, dashboard;
  cv::hconcat(std::vector<cv::Mat>{thumbs[0], thumbs[1]}, row1);
  cv::hconcat(std::vector<cv::Mat>{thumbs[2], thumbs[3]}, row2);
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

  const cv::Mat warped = bird_eye(frame);
  const cv::Mat mask = yellow_mask(warped);

  cv::Mat masked_view = warped.clone();
  masked_view.setTo(cv::Scalar(0, 255, 0), mask);

  const cv::Point2d origin(warped.cols / 2.0, warped.rows - 1.0);
  const PolarSearchResult search = polar_search(mask);

  // Fall back to the previous frame's lane when nothing is found this frame
  std::vector<cv::Point> points = search.points;
  if (points.empty()) {
    points = prev_points_;
  } else {
    prev_points_ = points;
  }

  const LaneFitResult fit = fit_lane(points, origin, warped.cols);

  cv::Mat search_view;
  cv::cvtColor(mask, search_view, cv::COLOR_GRAY2BGR);
  cv::circle(
    search_view, cv::Point(static_cast<int>(origin.x), static_cast<int>(origin.y)), 5,
    cv::Scalar(0, 0, 255), -1);
  for (const auto & box : search.boxes) {
    cv::rectangle(search_view, box, cv::Scalar(255, 255, 255), 1);
  }
  for (const auto & p : points) {
    cv::circle(search_view, p, 3, cv::Scalar(0, 255, 0), -1);
  }
  if (fit.valid) {
    cv::polylines(search_view, fit.spline_points, false, cv::Scalar(255, 0, 0), 2);
  }

  cv::Mat result = frame.clone();
  if (fit.valid) {
    std::vector<cv::Point2f> bev_points;
    bev_points.reserve(fit.spline_points.size());
    for (const auto & p : fit.spline_points) bev_points.emplace_back(p);

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
    {"Polar Search + Spline", search_view},
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
