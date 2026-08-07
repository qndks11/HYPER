#include "hyper_lane_detection/stopline_detector.hpp"

#include <cstdio>
#include <vector>

#include "hyper_lane_detection/bev_scale.hpp"

namespace hyper_lane_detection
{

namespace
{
// A stop-line bar spans most of the lane, so its bounding box is much wider than it is tall; a
// single zebra-crossing stripe is comparatively close to square. This floor separates the two.
constexpr double kMinStoplineAspectRatio = 3.0;
// The stop-line bar must span at least this fraction of the BEV image width to count -- rules
// out narrower marks (e.g. a single crossing stripe) that happen to pass the aspect ratio test.
constexpr double kMinStoplineWidthFraction = 0.10;
// Floor on contour area [px^2] to reject small mask noise before the shape checks run.
constexpr double kMinStoplineAreaPx = 500.0;

constexpr int kTextLinePitch = 36;

}  // namespace

cv::Mat StoplineDetector::white_mask(const cv::Mat & warped) const
{
  cv::Mat hsv;
  cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 60, 255), mask);

  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

StoplineDetector::Result StoplineDetector::find_stopline(
  const cv::Mat & mask, const cv::Point2d & origin, double meters_per_pixel) const
{
  Result result;

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

StoplineDetector::Result StoplineDetector::detect_and_draw(
  const cv::Mat & white, const cv::Point2d & origin, cv::Mat & view) const
{
  const double meters_per_pixel =
    kNumLaneInScreen * kLaneWidthMeters / static_cast<double>(white.cols);
  const Result result = find_stopline(white, origin, meters_per_pixel);

  if (result.valid) {
    cv::rectangle(view, result.bounding_box, cv::Scalar(0, 0, 255), 3);
  }

  return result;
}

void StoplineDetector::draw_text(const Result & result, cv::Mat & view, int panel_top) const
{
  const cv::Scalar stopline_text_color =
    result.valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);

  // Anchored below the lane section's own lines (rows 1-3), with a blank row of padding
  // between them, so it never collides with that block regardless of how many of those lines
  // are showing.
  char distance_text[64];
  std::snprintf(distance_text, sizeof(distance_text), "Distance: %.2f m", result.distance_m);
  cv::putText(
    view, distance_text, cv::Point(30, panel_top + kTextLinePitch * 8), cv::FONT_HERSHEY_SIMPLEX,
    1.0, stopline_text_color, 2);

  if (!result.valid) {
    cv::putText(
      view, "Stopline not detected", cv::Point(30, panel_top + kTextLinePitch * 9),
      cv::FONT_HERSHEY_SIMPLEX, 1.0, stopline_text_color, 2);
  }
}

}  // namespace hyper_lane_detection
