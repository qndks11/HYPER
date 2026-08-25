#ifndef HYPER_LANE_DETECTION__GROUND_PROJECTION_HPP_
#define HYPER_LANE_DETECTION__GROUND_PROJECTION_HPP_

#include <optional>
#include <string>

#include <opencv2/opencv.hpp>

namespace hyper_lane_detection
{

/// Pinhole intrinsics of the frame the homography is built against, in that frame's own pixels.
/// This must be the *rectified* camera matrix whenever the incoming frames are rectified (the
/// leading 3x3 block of CameraInfo's P, which is what hyper_camera's ElpCameraCapture already
/// remaps every real ELP frame into), not the raw K -- a homography derived from raw intrinsics
/// applied to rectified pixels is wrong by exactly the rectification it ignores.
struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  /// Intrinsics of an ideal centered pinhole with square pixels covering `horizontal_fov_rad`
  /// across `image.width`. This is exactly the model Gazebo's <camera> renders -- its
  /// <horizontal_fov> plus <image> width/height fully determine its projection, with no
  /// distortion and the principal point at the image center -- so the sim needs nothing more
  /// than the FOV it was already configured with. A real lens does not generally satisfy any of
  /// those three assumptions, which is why the real camera path passes fx/fy/cx/cy explicitly
  /// instead of coming through here.
  static CameraIntrinsics from_horizontal_fov(double horizontal_fov_rad, const cv::Size & image);

  bool is_set() const { return fx > 0.0 && fy > 0.0; }
};

/// Where the camera sits relative to the flat ground plane and the vehicle frame the BEV is
/// published in, expressed along the camera's own looking direction rather than the vehicle's
/// +x -- so the rear camera, which looks backward, describes its mounting with the same positive
/// numbers the front one does and needs no sign conventions of its own here. Converting that
/// looking-direction frame back to the vehicle's is a single 180 deg yaw, applied downstream
/// where the ground overlay is emitted (see LaneDetection::publish_bev_cloud's `facing`).
struct CameraExtrinsics
{
  /// Height of the camera's optical center above the ground plane [m].
  double height_m{0.0};
  /// Downward tilt from horizontal [rad]. Positive tips the optical axis toward the ground.
  double pitch_rad{0.0};
  /// Distance from the vehicle frame's origin to the camera, measured along the direction the
  /// camera looks [m]. Positive means the camera sits ahead of the origin in its own view
  /// direction, which is the usual case for both a forward- and a rearward-facing mount.
  double longitudinal_offset_m{0.0};
};

/// The patch of ground the bird's-eye view should show, in the camera's looking-direction frame
/// (x along the view direction, y to that direction's left). Unlike the trapezoidal pixel ROI
/// this replaced, every field here is a real distance -- which is the whole point: the output's
/// meters-per-pixel is chosen up front rather than being an emergent property of four corner
/// ratios that nobody can read a scale off of.
struct GroundRegion
{
  /// Nearest ground distance the BEV's bottom row shows [m]. Must be in front of the camera.
  /// A camera can only see ground beyond wherever its bottom image row strikes the plane, so
  /// setting this nearer than that simply produces black rows; GroundProjection::describe()
  /// reports when that is happening.
  double near_m{0.0};
  /// Farthest ground distance the BEV's top row shows [m].
  double far_m{0.0};
  /// Half the lateral extent the BEV spans, each side of the camera's view direction [m].
  double half_width_m{0.0};
  /// Ground distance one BEV pixel covers [m/px] -- the same value along both axes, since the
  /// output grid is a metric raster by construction. This is the number the detectors' distance
  /// and offset outputs are scaled by, so it is a real measurement, not a display preference.
  double meters_per_pixel{0.0};
};

/**
 * @brief The bird's-eye-view homography, derived from where the camera actually is rather than
 * hand-picked in image space.
 *
 * @details The previous approach picked four trapezoid corners as ratios of the source frame and
 * warped them to fill the output, then separately declared a meters-per-pixel constant. Nothing
 * connected the two: the corners implied one ground geometry, the constant asserted another, and
 * the output was neither isotropic nor metric. This class inverts that. Given the camera's
 * intrinsics and its pose over a flat ground plane, it takes the ground rectangle you *want* to
 * see and projects that rectangle's four corners into the image to obtain the source quad. The
 * warp therefore maps a genuine ground rectangle onto a uniform metric raster, so:
 *
 * - one meters-per-pixel is correct along both axes, because the output grid is defined in meters
 *   before any pixel is touched;
 * - the vehicle frame's origin has an exact, computable position in BEV pixels (origin_px()),
 *   including the usual case where it falls below the image because the camera cannot see the
 *   ground at its own feet;
 * - changing a camera's FOV, resolution, height or pitch changes the homography automatically,
 *   instead of silently invalidating constants tuned against the old geometry.
 *
 * The flat-ground assumption is the one thing this shares with the ROI approach and cannot escape:
 * any single homography between an image and a plane assumes the world *is* that plane. Ground
 * that rises or falls (the course's hill, notably) projects with an error that grows with
 * distance, which is a reason to keep GroundRegion::far_m modest rather than a reason to distrust
 * the near field.
 */
class GroundProjection
{
public:
  /**
   * @brief Builds the projection, or fails with a reason.
   *
   * @param intrinsics Rectified pinhole intrinsics of the source frame.
   * @param extrinsics Camera pose over the ground plane, in looking-direction convention.
   * @param region Requested ground patch and scale. The realized region is snapped to a whole
   * number of pixels at exactly the requested meters_per_pixel (see region()), so the output
   * grid stays exactly metric rather than the scale being bent to fit a rounded image size.
   * @param error Set to a human-readable reason when construction fails.
   * @return The projection, or std::nullopt if the request does not describe a visible ground
   * patch (e.g. a corner at or behind the camera plane, or a non-positive extent/scale).
   */
  static std::optional<GroundProjection> create(
    const CameraIntrinsics & intrinsics, const CameraExtrinsics & extrinsics,
    const GroundRegion & region, std::string & error);

  /// Projects a ground point, in the camera's looking-direction frame, to source-image pixels.
  /// Only meaningful for points in front of the camera plane; see create()'s validation.
  cv::Point2d project_to_image(double x_m, double y_m) const;

  /// Source-image row the ground horizon falls on -- where a ground point infinitely far away
  /// projects. Ground below this row is visible, ground above it is sky. Reported by describe()
  /// because a region whose far edge crowds this row is one whose far rows are interpolated from
  /// almost no source pixels.
  double horizon_row() const;

  /// Homography mapping source-image pixels to BEV pixels; feed straight to cv::warpPerspective
  /// together with bev_size().
  const cv::Mat & homography() const { return homography_; }

  /// Output raster size [px], derived from the realized region and the scale.
  cv::Size bev_size() const { return bev_size_; }

  /// The vehicle frame origin's position in BEV pixel coordinates. Its column is the image's
  /// horizontal center; its row is normally *below* the last image row, since the BEV starts at
  /// region().near_m rather than at the vehicle itself. Distances therefore come out as
  /// (origin.y - row) * meters_per_pixel and lateral offsets as (origin.x - col) *
  /// meters_per_pixel, both exact -- this is what the old hand-set kOriginBelowFrameMarginPx was
  /// approximating by eye.
  cv::Point2d origin_px() const { return origin_px_; }

  /// Ground distance one BEV pixel covers [m/px], both axes.
  double meters_per_pixel() const { return region_.meters_per_pixel; }

  /// The realized ground region -- as requested, except far_m and half_width_m snapped outward
  /// to whole pixels at meters_per_pixel.
  const GroundRegion & region() const { return region_; }

  /// One-line summary of the realized geometry plus anything worth knowing about it (source
  /// corners that fall outside `source`, i.e. BEV corners that will be black for want of pixels
  /// to sample; a far edge close to the horizon). Logged once per camera at startup so a wrong
  /// mounting parameter is visible in the log rather than only in RViz.
  std::string describe(const cv::Size & source) const;

private:
  GroundProjection(
    const CameraIntrinsics & intrinsics, const CameraExtrinsics & extrinsics,
    const GroundRegion & region, const cv::Size & bev_size, const cv::Point2d & origin_px,
    const cv::Mat & homography);

  CameraIntrinsics intrinsics_;
  CameraExtrinsics extrinsics_;
  GroundRegion region_;
  cv::Size bev_size_;
  cv::Point2d origin_px_;
  cv::Mat homography_;
};

}  // namespace hyper_lane_detection

#endif  // HYPER_LANE_DETECTION__GROUND_PROJECTION_HPP_
