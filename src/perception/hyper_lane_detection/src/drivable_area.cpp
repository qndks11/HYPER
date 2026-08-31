#include "hyper_lane_detection/drivable_area.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace hyper_lane_detection
{

namespace
{

/// Converts a radius in meters to an odd-sided structuring element, or an empty Mat when the
/// radius is under half a pixel (in which case the caller skips the morphology entirely rather
/// than running a 1x1 kernel that does nothing but cost a pass over the image).
cv::Mat metric_kernel(double radius_m, double meters_per_pixel)
{
  const int radius_px = static_cast<int>(std::lround(radius_m / meters_per_pixel));
  if (radius_px < 1) {
    return cv::Mat();
  }
  const int side = 2 * radius_px + 1;
  return cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(side, side));
}

}  // namespace

cv::Rect DrivableAreaDetector::seed_rect(
  const cv::Size & size, const cv::Point2d & origin_px, double meters_per_pixel) const
{
  const int half_width_px =
    std::max(1, static_cast<int>(std::lround(0.5 * settings_.seed_width_m / meters_per_pixel)));
  const int depth_px =
    std::max(1, static_cast<int>(std::lround(settings_.seed_depth_m / meters_per_pixel)));

  // Measured from the bottom row up. The bottom row is the nearest ground the camera can actually
  // see (GroundRegion::near_m), which is not the vehicle itself -- origin_px normally sits below
  // the raster entirely -- so the seed starts at the closest ground there is rather than at a
  // vehicle-relative distance that may not be in frame.
  const int x = static_cast<int>(std::lround(origin_px.x)) - half_width_px;
  const int y = size.height - depth_px;
  return cv::Rect(x, y, 2 * half_width_px, depth_px) & cv::Rect(cv::Point(0, 0), size);
}

DrivableAreaDetector::Result DrivableAreaDetector::detect(
  const cv::Mat & bev, const cv::Point2d & origin_px, double meters_per_pixel) const
{
  Result result;
  result.classes = cv::Mat(bev.size(), CV_8S, cv::Scalar(kUnknown));
  if (bev.empty() || meters_per_pixel <= 0.0) {
    return result;
  }

  // No-data: the warp's un-sampled corners, left at exactly zero. Same convention as
  // publish_bev_cloud() -- black is "the homography had no source pixel here", not dark ground.
  cv::Mat gray;
  cv::cvtColor(bev, gray, cv::COLOR_BGR2GRAY);
  cv::Mat valid = gray > 0;

  // ---- Stage 1: color ----
  cv::Mat hsv;
  cv::cvtColor(bev, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);

  cv::Mat surface = channels[1] < settings_.surface_max_saturation;

  // Yellow paint is saturated, so the threshold above has already rejected it; this mask exists
  // to keep it rejected through the morphology below, which would otherwise close straight over a
  // line only a few pixels wide.
  cv::Mat paint;
  cv::inRange(
    hsv,
    cv::Scalar(settings_.paint_hue_min, settings_.paint_min_saturation, 0),
    cv::Scalar(settings_.paint_hue_max, 255, 255),
    paint);

  cv::bitwise_and(surface, valid, surface);

  // ---- Stage 2: morphology ----
  // Close before open. Closing first seals the dashes and glare specks that would otherwise let
  // the flood fill leak off the road or split the road in two; opening afterwards removes the
  // gray specks out in the grass that closing may have just fattened.
  const cv::Mat close_kernel = metric_kernel(settings_.close_radius_m, meters_per_pixel);
  if (!close_kernel.empty()) {
    cv::morphologyEx(surface, surface, cv::MORPH_CLOSE, close_kernel);
  }
  const cv::Mat open_kernel = metric_kernel(settings_.open_radius_m, meters_per_pixel);
  if (!open_kernel.empty()) {
    cv::morphologyEx(surface, surface, cv::MORPH_OPEN, open_kernel);
  }
  // Closing dilates across the no-data border, so re-clip. Without this the mask grows into the
  // warp's black corners and those pixels stop being reported as unknown.
  cv::bitwise_and(surface, valid, surface);

  // Re-cut the yellow paint, after the morphology rather than before it. Closing at
  // close_radius_m is wide enough to bridge a lane line outright, so a boundary subtracted before
  // that pass would simply be filled back in; subtracting here is what makes the line survive as
  // a hole in the surface mask, and therefore as a barrier the reachability stage below cannot
  // flood through.
  cv::subtract(surface, paint, surface);

  // ---- Stage 3: reachability ----
  const cv::Rect seed = seed_rect(bev.size(), origin_px, meters_per_pixel);
  if (seed.empty()) {
    return result;  // seeded stays false
  }

  const double coverage =
    static_cast<double>(cv::countNonZero(surface(seed))) / static_cast<double>(seed.area());
  if (coverage < settings_.min_seed_coverage) {
    // Deliberately all-unknown, not all-undrivable. See DrivableAreaSettings::min_seed_coverage:
    // the vehicle's way out of this state is to drive, and a lethal ring would forbid exactly that.
    return result;
  }
  result.seeded = true;

  cv::Mat labels;
  const int label_count = cv::connectedComponents(surface, labels, 8, CV_32S);

  // Every label the seed patch touches, not just the one under its center -- the patch can
  // straddle a paint line the closing failed to bridge, and dropping the half the center missed
  // would carve a lethal stripe up the middle of the road.
  std::vector<bool> reachable(static_cast<size_t>(label_count), false);
  for (int row = seed.y; row < seed.y + seed.height; ++row) {
    const int32_t * label_row = labels.ptr<int32_t>(row);
    for (int col = seed.x; col < seed.x + seed.width; ++col) {
      const int32_t label = label_row[col];
      if (label != 0) {
        reachable[static_cast<size_t>(label)] = true;
      }
    }
  }

  // ---- Stage 4: labelling ----
  // Everything the fill did not reach comes out kOffLimits, at one strength. Nothing this
  // detector emits is lethal, deliberately: the mask is a homography onto the ground plane with
  // no height information in it, so a misread frame -- glare, a shadow edge, one interpolated far
  // row -- must never be able to declare a collision or grow an inflation halo in front of the
  // car. kOffLimits still costs enough for the controller to steer around it (see the layer's
  // off_limits_cost), and what is genuinely solid is the lidar obstacle layer's job to say.
  //
  // The distinction the earlier version drew here -- paint and unreachable asphalt as a lane
  // rule, everything else as lethal ground -- is therefore gone, along with the paint dilation
  // and the minimum-width opening that existed only to keep line-shaped things out of the lethal
  // class. Stage 3's reachability is still what decides drivable from not.
  for (int row = 0; row < bev.rows; ++row) {
    const int32_t * label_row = labels.ptr<int32_t>(row);
    const uint8_t * valid_row = valid.ptr<uint8_t>(row);
    int8_t * out_row = result.classes.ptr<int8_t>(row);
    for (int col = 0; col < bev.cols; ++col) {
      if (valid_row[col] == 0) {
        continue;  // stays kUnknown
      }
      const int32_t label = label_row[col];
      const bool drivable = label != 0 && reachable[static_cast<size_t>(label)];
      out_row[col] = drivable ? kDrivable : kOffLimits;
    }
  }

  return result;
}

void DrivableAreaDetector::draw(
  const Result & result, const cv::Point2d & origin_px, double meters_per_pixel,
  cv::Mat & view) const
{
  if (result.classes.empty() || view.size() != result.classes.size()) {
    return;
  }

  // Tint rather than paint, so the underlying view stays readable -- the point of looking at this
  // debug image is to see *what the camera saw* where the classifier decided something.
  cv::Mat tint(view.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  tint.setTo(cv::Scalar(0, 165, 255), result.classes == kOffLimits);
  tint.setTo(cv::Scalar(0, 255, 0), result.classes == kDrivable);
  cv::addWeighted(view, 0.65, tint, 0.35, 0.0, view);

  const cv::Rect seed = seed_rect(view.size(), origin_px, meters_per_pixel);
  if (!seed.empty()) {
    cv::rectangle(view, seed, result.seeded ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255), 2);
  }
}

}  // namespace hyper_lane_detection
