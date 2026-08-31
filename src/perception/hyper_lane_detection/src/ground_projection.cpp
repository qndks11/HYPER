#include "hyper_lane_detection/ground_projection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace hyper_lane_detection
{

namespace
{
// Floor on a ground point's depth along the optical axis [m]. A point at or behind the camera
// plane has no image projection at all, and one barely in front of it projects arbitrarily far
// out of frame; either way the homography built from it is meaningless, so create() rejects the
// region rather than returning a transform that silently produces garbage.
constexpr double kMinProjectionDepthM = 1e-3;

// A far edge closer to the horizon than this many rows is drawing its topmost BEV rows from a
// vanishingly thin band of source pixels -- geometrically valid, but interpolated from so little
// real data that anything detected up there is noise. Reported by describe(), not rejected: how
// much far-field blur is acceptable is a tuning call, not a correctness one.
constexpr double kHorizonProximityWarnRows = 5.0;

}  // namespace

CameraIntrinsics CameraIntrinsics::from_horizontal_fov(
  double horizontal_fov_rad, const cv::Size & image)
{
  CameraIntrinsics intrinsics;
  if (horizontal_fov_rad <= 0.0 || horizontal_fov_rad >= CV_PI || image.width <= 0 ||
    image.height <= 0)
  {
    return intrinsics;  // left unset; is_set() reports the failure to the caller
  }
  // Square pixels: one focal length serves both axes, so the vertical FOV follows from the
  // image's aspect ratio rather than being independently specifiable -- exactly Gazebo's model.
  intrinsics.fx = (image.width / 2.0) / std::tan(horizontal_fov_rad / 2.0);
  intrinsics.fy = intrinsics.fx;
  intrinsics.cx = image.width / 2.0;
  intrinsics.cy = image.height / 2.0;
  return intrinsics;
}

GroundProjection::GroundProjection(
  const CameraIntrinsics & intrinsics, const CameraExtrinsics & extrinsics,
  const GroundRegion & region, const cv::Size & bev_size, const cv::Point2d & origin_px,
  const cv::Mat & homography)
: intrinsics_{intrinsics}, extrinsics_{extrinsics}, region_{region}, bev_size_{bev_size},
  origin_px_{origin_px}, homography_{homography}
{
}

cv::Point2d GroundProjection::project_to_image(double x_m, double y_m) const
{
  // Ground point relative to the camera, still in the looking-direction frame (x along the view
  // direction, y to its left, z up). The ground plane is height_m below the optical center.
  const double dx = x_m - extrinsics_.longitudinal_offset_m;
  const double dy = y_m;
  const double dz = -extrinsics_.height_m;

  const double sin_pitch = std::sin(extrinsics_.pitch_rad);
  const double cos_pitch = std::cos(extrinsics_.pitch_rad);

  // Into the optical frame (z forward along the optical axis, x right, y down), which is the
  // looking-direction frame pitched down by pitch_rad and then relabelled to the camera
  // convention. Written out rather than assembled as a matrix because it is three lines and
  // this way the pitch rotation and the axis relabelling stay visible as separate steps.
  const double depth = dx * cos_pitch - dz * sin_pitch;
  const double right = -dy;
  const double down = -(dx * sin_pitch + dz * cos_pitch);

  return cv::Point2d(
    intrinsics_.cx + intrinsics_.fx * right / depth,
    intrinsics_.cy + intrinsics_.fy * down / depth);
}

double GroundProjection::horizon_row() const
{
  // Limit of project_to_image()'s row as x -> infinity: the ground point's depth and its drop
  // below the optical axis both grow linearly with distance, so their ratio converges to
  // -tan(pitch) and the row converges to cy - fy*tan(pitch), independent of height.
  return intrinsics_.cy - intrinsics_.fy * std::tan(extrinsics_.pitch_rad);
}

std::optional<GroundProjection> GroundProjection::create(
  const CameraIntrinsics & intrinsics, const CameraExtrinsics & extrinsics,
  const GroundRegion & region, std::string & error)
{
  if (!intrinsics.is_set()) {
    error = "camera intrinsics are unset (fx/fy must be positive)";
    return std::nullopt;
  }
  if (extrinsics.height_m <= 0.0) {
    error = "camera height_m must be positive (the camera has to be above the ground plane)";
    return std::nullopt;
  }
  if (region.meters_per_pixel <= 0.0) {
    error = "meters_per_pixel must be positive";
    return std::nullopt;
  }
  if (region.half_width_m <= 0.0) {
    error = "half_width_m must be positive";
    return std::nullopt;
  }
  if (region.far_m <= region.near_m) {
    error = "far_m must be greater than near_m";
    return std::nullopt;
  }

  // Snap the requested extents outward to a whole number of pixels at exactly the requested
  // scale, rather than keeping the extents and deriving the scale from a rounded image size --
  // the latter would give the two axes slightly different meters-per-pixel, reintroducing in
  // miniature the anisotropy this class exists to remove.
  const int width = std::max(1, static_cast<int>(std::lround(2.0 * region.half_width_m /
      region.meters_per_pixel)));
  const int height = std::max(1, static_cast<int>(std::lround((region.far_m - region.near_m) /
      region.meters_per_pixel)));

  GroundRegion realized = region;
  realized.half_width_m = width * region.meters_per_pixel / 2.0;
  realized.far_m = region.near_m + height * region.meters_per_pixel;

  const GroundProjection probe(
    intrinsics, extrinsics, realized, cv::Size(width, height), cv::Point2d(), cv::Mat());

  // BEV corner order matches the ground corner order below: (0,0) is the far-left of the region,
  // and rows increase toward the vehicle. Corners are placed at 0 and width/height (not
  // width-1/height-1) so that col = (half_width - y)/mpp and row = (far - x)/mpp hold exactly,
  // which is what makes origin_px() below an exact expression rather than an approximation.
  const double x_far = realized.far_m;
  const double x_near = realized.near_m;
  const double y_left = realized.half_width_m;
  const double y_right = -realized.half_width_m;

  const std::vector<std::pair<double, double>> ground_corners{
    {x_far, y_left}, {x_far, y_right}, {x_near, y_right}, {x_near, y_left}};
  const std::vector<cv::Point2f> bev_corners{
    {0.0f, 0.0f},
    {static_cast<float>(width), 0.0f},
    {static_cast<float>(width), static_cast<float>(height)},
    {0.0f, static_cast<float>(height)}};

  std::vector<cv::Point2f> source_corners;
  source_corners.reserve(ground_corners.size());
  for (const auto & [x_m, y_m] : ground_corners) {
    // Depth check before projecting: project_to_image() divides by this, so a corner at or
    // behind the camera plane would otherwise come back as an infinity that getPerspectiveTransform
    // turns into a silently unusable matrix.
    const double dx = x_m - extrinsics.longitudinal_offset_m;
    const double depth = dx * std::cos(extrinsics.pitch_rad) +
      extrinsics.height_m * std::sin(extrinsics.pitch_rad);
    if (depth < kMinProjectionDepthM) {
      char detail[192];
      std::snprintf(
        detail, sizeof(detail),
        "ground corner (x=%.2f m, y=%.2f m) is not in front of the camera (depth %.3f m) -- "
        "near_m is too close for a camera %.2f m up at %.1f deg of pitch",
        x_m, y_m, depth, extrinsics.height_m, extrinsics.pitch_rad * 180.0 / CV_PI);
      error = detail;
      return std::nullopt;
    }
    const cv::Point2d pixel = probe.project_to_image(x_m, y_m);
    source_corners.emplace_back(
      static_cast<float>(pixel.x), static_cast<float>(pixel.y));
  }

  const cv::Mat homography = cv::getPerspectiveTransform(source_corners, bev_corners);

  // The vehicle frame origin sits at ground (0, 0): horizontally centered, and far_m/mpp rows
  // down from the BEV's top row -- which is height + near_m/mpp, i.e. below the last row by
  // however much ground the camera cannot see at its own feet.
  const cv::Point2d origin_px(
    width / 2.0, realized.far_m / realized.meters_per_pixel);

  return GroundProjection(
    intrinsics, extrinsics, realized, cv::Size(width, height), origin_px, homography);
}

std::string GroundProjection::describe(const cv::Size & source) const
{
  char summary[512];
  std::snprintf(
    summary, sizeof(summary),
    "ground x %.2f..%.2f m, y +/-%.2f m at %.4f m/px -> BEV %dx%d px; "
    "vehicle origin at BEV (%.1f, %.1f), horizon at source row %.1f",
    region_.near_m, region_.far_m, region_.half_width_m, region_.meters_per_pixel,
    bev_size_.width, bev_size_.height, origin_px_.x, origin_px_.y, horizon_row());
  std::string text{summary};

  // The near corners of a wide region routinely fall outside the frame: the camera simply does
  // not see that far sideways at that little distance. warpPerspective leaves those BEV corners
  // black and publish_bev_cloud drops them, so this is a note about lost coverage rather than a
  // fault -- but it is the first thing to look at if the overlay's bottom corners are missing.
  int outside = 0;
  const std::vector<std::pair<double, double>> corners{
    {region_.far_m, region_.half_width_m}, {region_.far_m, -region_.half_width_m},
    {region_.near_m, -region_.half_width_m}, {region_.near_m, region_.half_width_m}};
  for (const auto & [x_m, y_m] : corners) {
    const cv::Point2d pixel = project_to_image(x_m, y_m);
    if (pixel.x < 0.0 || pixel.x > source.width || pixel.y < 0.0 || pixel.y > source.height) {
      ++outside;
    }
  }
  if (outside > 0) {
    char note[160];
    std::snprintf(
      note, sizeof(note),
      "; %d of 4 region corners fall outside the %dx%d source frame (those BEV corners stay black)",
      outside, source.width, source.height);
    text += note;
  }

  const double far_row = project_to_image(region_.far_m, 0.0).y;
  if (far_row - horizon_row() < kHorizonProximityWarnRows) {
    char note[192];
    std::snprintf(
      note, sizeof(note),
      "; far_m projects to source row %.1f, within %.1f rows of the horizon -- the BEV's top rows "
      "are interpolated from almost no source pixels",
      far_row, kHorizonProximityWarnRows);
    text += note;
  }

  return text;
}

}  // namespace hyper_lane_detection
