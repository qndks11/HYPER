#ifndef HYPER_LANE_DETECTION__LANE_DETECTOR_HPP_
#define HYPER_LANE_DETECTION__LANE_DETECTOR_HPP_

#include <vector>

#include <opencv2/opencv.hpp>

namespace hyper_lane_detection
{

/// Detects the left/right lane lines in a bird's-eye-warped frame and draws the debug overlay
/// for both. Operates purely on an already-warped frame and an already-computed yellow mask --
/// it knows nothing about cameras, ROS, or how the frame arrived; see lane_detection_node.hpp for
/// that.
class LaneDetector
{
public:
  enum class Side { kLeft, kRight };

  struct Fit
  {
    bool valid{false};
    double offset_m{0.0};
    double steering_angle_deg{0.0};
    double curvature_radius_px{0.0};
    std::vector<cv::Point> curve_points;  // the fitted line's/curve's points, for drawing
  };

  struct Result
  {
    Fit left;
    Fit right;
  };

  /**
   * @brief Produces a binary mask isolating yellow lane paint in HSV space.
   */
  cv::Mat yellow_mask(const cv::Mat & warped) const;

  /**
   * @brief Walks both lane chains from `yellow`, fits each side (curve or straight per
   * `is_rear`), applies the outward-lean offset correction, and draws the chain points / fitted
   * line-or-curve / heading arrow / target line into `view`.
   *
   * @details Does NOT highlight the mask into `view` -- the caller highlights both the lane and
   * stop-line masks into `view` once, before calling either detector, so both detectors'
   * annotations always draw on top of both mask highlights (matching this package's original,
   * single-file draw order).
   *
   * `is_rear` alone selects between the front camera's paired-lane assumption (straight-line fit,
   * lane-center bias) and the rear camera's single parking-bay line (curve fit, fixed standoff
   * bias) -- both constants live entirely inside this class now, so the caller doesn't need to
   * know either one.
   *
   * @param yellow Yellow paint mask (BEV), from yellow_mask().
   * @param origin The vehicle's position in BEV coordinates (GroundProjection::origin_px()).
   * Normally sits below the image's last row, since the BEV starts at the nearest ground the
   * camera can actually see rather than at the vehicle itself.
   * @param meters_per_pixel Ground distance one BEV pixel covers [m/px], from
   * GroundProjection::meters_per_pixel(). Isotropic by construction, so it applies to both axes.
   * @param is_rear False: front camera (straight fit, lane-center bias). True: rear camera (curve
   * fit, parking-line standoff bias).
   * @param view Debug image to draw into; must already carry both mask highlights and be BEV-
   * sized (not yet bordered with the text panel).
   * @return Both sides' fit results, for publishing and draw_text().
   */
  Result detect_and_draw(
    const cv::Mat & yellow, const cv::Point2d & origin, double meters_per_pixel, bool is_rear,
    cv::Mat & view) const;

  /// Draws the "L: .. / R: .. / Lane not detected" text block onto view's text panel, starting at
  /// panel_top.
  void draw_text(const Result & result, cv::Mat & view, int panel_top) const;

private:
  /**
   * @brief Walks a chain of yellow pixels along one lane side, starting from its bottom point
   * and stepping outward one nearby pixel at a time.
   *
   * @details The seed point is the bottommost yellow pixel on the requested half of the image
   * (x >= origin.x for kRight, x < origin.x for kLeft), and the initial direction is straight up
   * the image (toward the horizon). Each step: pixels within kNeighborRadius of the current point
   * are filtered to those whose displacement from it has a non-negative dot product with the
   * current direction (i.e. within +/-90 degrees of it); if none remain, the chain stops. The
   * survivors are ranked by the signed sine of their angle to the current direction (cross
   * product over the product of norms, valid without abs() since candidates are already
   * restricted to +/-90 degrees): kRight walks most-positive-angle first, kLeft walks in the
   * opposite order, most-negative first -- with remaining ties broken by preferring the pixel
   * farthest from the current point. The winner becomes the new current point, and the vector
   * from the old point to it becomes the new direction, so the chain keeps following whatever
   * heading the lane is actually curving toward.
   *
   * Nothing stops this walk from drifting across the centerline as it follows a curving lane, so
   * a chain seeded on one side can legitimately end up on the other. When there's really only one
   * lane line straddling origin.x, though, both the left and right seeds can latch onto it and
   * walk out along essentially the same pixels -- detect_and_draw() distinguishes a genuine
   * crossing lane from this spurious duplicate by checking that the chain still gets farther from
   * the vehicle from start to end (see is_spurious_cross_lane() in the .cpp).
   *
   * @param yellow_points Every yellow mask pixel coordinate (BEV), e.g. from cv::findNonZero.
   * @param origin The bottom-center point used to pick the seed (x >= origin.x for kRight,
   * x < origin.x for kLeft).
   * @param side Which half of the image to seed the chain from.
   * @return The walked chain's original BEV pixel coordinates, near (bottom) to far.
   */
  std::vector<cv::Point> walk_lane_chain(
    const std::vector<cv::Point> & yellow_points, const cv::Point2d & origin, Side side) const;

  /**
   * @brief Fits a straight line through the walked lane chain by weighted total least squares,
   * pinned to the chain's near point, and derives the heading angle and lateral offset from it.
   *
   * @details The line is anchored exactly at the chain's first (near) point; its direction is
   * the eigenvector, of the weighted scatter matrix of the other chain points relative to that
   * anchor, with the largest eigenvalue -- the axis the points are most spread along, which
   * minimizes the weighted sum of squared perpendicular distances to the line. This total
   * least squares formulation (as opposed to regressing x on y or y on x) has no axis it breaks
   * down along, so it stays well-conditioned for a lane running in any direction, including
   * near-horizontal. A straight-line model has no curvature to derive, so curvature_radius_px is
   * always a large sentinel value. `center_bias_m` is applied with a sign matching `side`: the
   * target sits on the vehicle's side of a right line, but on the far side of a left line -- so
   * for the front camera's paired-lane assumption this places the target at the lane's center,
   * while for the rear camera's single parking-bay line it places the target at a fixed standoff
   * from it.
   *
   * @param points The lane chain from walk_lane_chain(), in BEV coordinates, near to far.
   * @param origin The vehicle position the chain was anchored to (BEV coordinates).
   * @param meters_per_pixel Ground distance one BEV pixel covers [m/px], used to scale the fitted
   * line's perpendicular pixel offset to meters.
   * @param side Which lane line `points` traces, so the center-offset bias is signed correctly.
   * @param center_bias_m Distance from the detected line to the target, signed per `side`.
   * @return The lane fit result; `valid` is false if there are too few points.
   */
  Fit fit_lane(
    const std::vector<cv::Point> & points, const cv::Point2d & origin, double meters_per_pixel,
    Side side, double center_bias_m) const;

  /**
   * @brief Fits a quadratic curve (x' = a*y'^2 + b*y', in coordinates shifted so the chain's near
   * point is the origin) through the walked lane chain by weighted least squares, and derives the
   * heading angle, lateral offset, and curvature from its near-point tangent -- the rear-camera
   * counterpart to fit_lane()'s straight-line model, for lines that are genuinely curved (e.g. a
   * parking bay's entrance) rather than running straight, where a global straight-line direction
   * systematically mis-estimates heading/offset away from the near point.
   *
   * @details Anchored at the chain's near point p0 exactly (no free constant term), same as
   * fit_lane(). The 2x2 weighted normal-equations system for (a, b) uses the same per-point
   * near-field weighting as fit_lane(), solved via cv::solve(..., cv::DECOMP_SVD) rather than a
   * strict LU decomposition, so a near-singular system degrades gracefully instead of failing
   * outright. Requires more points than fit_lane() (kMinCurveFitPoints) since estimating a
   * curvature term needs more spread than estimating a single direction. steering_angle_deg and
   * offset_m are evaluated from the curve's tangent at the near point (f'(0) = b), using the same
   * formulas as fit_lane() so both fits are drop-in interchangeable for callers; unlike fit_lane(),
   * curvature_radius_px is a real value derived from the curve's second derivative (f''(0) = 2a)
   * rather than always the flat-line sentinel. curve_points samples the fitted parabola across the
   * chain's observed reach, for drawing, rather than fit_lane()'s two-point straight segment.
   *
   * @param points The lane chain from walk_lane_chain(), in BEV coordinates, near to far.
   * @param origin The vehicle position the chain was anchored to (BEV coordinates).
   * @param meters_per_pixel Ground distance one BEV pixel covers [m/px], used to scale the fitted
   * curve's perpendicular pixel offset to meters.
   * @param side Which lane line `points` traces, so the center-offset bias is signed correctly.
   * @param center_bias_m Distance from the detected line to the target, signed per `side` -- same
   * meaning as fit_lane()'s parameter of the same name.
   * @return The curve fit result; `valid` is false if there are too few points or the chain is too
   * straight/short for a stable curvature estimate.
   */
  Fit fit_lane_curve(
    const std::vector<cv::Point> & points, const cv::Point2d & origin, double meters_per_pixel,
    Side side, double center_bias_m) const;
};

}  // namespace hyper_lane_detection

#endif  // HYPER_LANE_DETECTION__LANE_DETECTOR_HPP_
