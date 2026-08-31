#ifndef HYPER_LANE_DETECTION__DRIVABLE_AREA_HPP_
#define HYPER_LANE_DETECTION__DRIVABLE_AREA_HPP_

#include <opencv2/opencv.hpp>

namespace hyper_lane_detection
{

/// Occupancy values the detector emits, chosen to be nav_msgs/OccupancyGrid's own encoding so the
/// classification raster can be copied into a grid message without a translation table.
///
/// Everything the detector rejects comes out kOffLimits, at one strength -- grass and the
/// oncoming lane alike. Nothing here is lethal: this is a homography onto the ground plane with
/// no height information in it, so one misread frame must not be able to declare a collision or
/// grow an inflation halo in front of the car. Height is the lidar obstacle layer's subject, and
/// that is the layer allowed to be lethal.
///
/// kUndrivable is kept for the consumer's benefit: DrivableAreaLayer still splits on its
/// lethal_threshold, and the value is what a stronger class would have to be if this detector
/// ever emits one again. Nothing in this file produces it today.
constexpr int8_t kDrivable = 0;
constexpr int8_t kOffLimits = 60;
constexpr int8_t kUndrivable = 100;
constexpr int8_t kUnknown = -1;

/// Tuning for DrivableAreaDetector. Every field is a ROS parameter on the node (prefix
/// `drivable.`); the defaults describe the simulated course, whose surface is flat gray asphalt
/// (HSV S = 12), whose curbs are lighter gray (S = 1), whose lane paint is yellow (H = 30,
/// S = 255) and white (S = 0), and whose off-course area is grass (H = 74, S = 205).
struct DrivableAreaSettings
{
  /// A pixel this unsaturated is road surface. This is the whole classifier: asphalt, concrete,
  /// curbs and white paint are all achromatic, and everything that bounds the course -- grass,
  /// dirt, painted run-off -- is not. Saturation is used rather than value/brightness because it
  /// barely moves under shadow, which is exactly where a brightness threshold fails: a shadowed
  /// patch of asphalt is dark but still gray.
  int surface_max_saturation{40};

  /// Yellow lane paint. Yellow marks the course boundary and the centre line, i.e. exactly the
  /// lines the vehicle is not allowed to cross, so it is cut out of the surface mask to stop the
  /// reachability flood fill below from spilling into the oncoming lane or off the course through
  /// a line the morphology would otherwise have bridged. White paint is left alone: it is
  /// achromatic, stays part of the surface, and is a lane hint rather than a boundary.
  ///
  /// Cutting the fill is all it does -- and since every rejected pixel is kOffLimits now, what
  /// the paint walls off is the only thing it changes.
  ///
  /// The range matches LaneDetector::yellow_mask() exactly -- if you retune the lane detector's
  /// yellow, retune this with it, or the barriers drift out from under the paint.
  int paint_hue_min{22};
  int paint_hue_max{35};
  int paint_min_saturation{80};

  /// Morphological closing radius [m], applied to the surface mask. Bridges the gaps that dashed
  /// paint, specular glare and warp interpolation punch through the road, so the flood fill below
  /// doesn't leak or fragment. Sized in meters rather than pixels so it means the same thing at
  /// any meters_per_pixel.
  double close_radius_m{0.12};

  /// Morphological opening radius [m], applied after the closing. Deletes isolated gray specks
  /// out in the grass, which would otherwise survive as tiny drivable islands.
  double open_radius_m{0.06};

  /// The seed patch, directly ahead of the vehicle, that the flood fill starts from -- the
  /// assumption being that the ground the vehicle is about to drive over is drivable. Width
  /// defaults to the vehicle's own track width; depth is measured forward from the nearest ground
  /// row the BEV actually covers.
  double seed_width_m{0.81};
  double seed_depth_m{0.60};

  /// Fraction of the seed patch that must classify as surface before the result is trusted. Below
  /// it the frame is published as all-unknown instead of all-undrivable: a camera staring at a
  /// wall, a blown exposure or a genuinely off-course vehicle must not fence the car in with
  /// cost, because the recovery from that state is to drive.
  double min_seed_coverage{0.25};
};

/**
 * @brief Classifies a bird's-eye-warped frame into drivable surface / off-limits road /
 * undrivable ground / no-data, without a learned model.
 *
 * @details Four stages, in order:
 *
 * 1. **Color.** Per pixel, `surface = (S < surface_max_saturation)`. See
 *    DrivableAreaSettings::surface_max_saturation for why saturation and not brightness. Yellow
 *    paint is saturated and so falls out of the surface on its own; stage 2 then takes care not
 *    to put it back.
 * 2. **Morphology.** Close then open, at radii given in meters, and then re-subtract the yellow
 *    paint -- the closing is wide enough to bridge a lane line, so the barrier has to be cut
 *    after it rather than before.
 * 3. **Reachability.** Keep only the connected components of the surface mask that the seed patch
 *    in front of the vehicle touches. This is the stage that earns its keep: color alone happily
 *    labels the gray service road on the far side of the grass as drivable, and a controller that
 *    believes it will plan straight across the infield. Connectivity in the BEV *is* connectivity
 *    on the ground plane, so "reachable" here means what it says -- and because stage 2 leaves the
 *    yellow paint as a hole, an unbroken yellow line bounds the flood fill the same way the edge
 *    of the asphalt does.
 * 4. **Labelling.** Whatever the fill reached is kDrivable; everything else in frame is
 *    kOffLimits, at one strength, whether it is grass, the paint, or the asphalt beyond it. The
 *    detector does not grade how badly you must not go somewhere -- see the constants above for
 *    why nothing a ground-plane color rule produces should be lethal.
 *
 * Runs on the BEV rather than the raw frame on purpose. A learned segmenter would want the raw
 * perspective view (that is what it was trained on), but a color rule does not care about
 * perspective, and doing it here makes stage 3 metric and reuses the warp process_frame() has
 * already paid for.
 *
 * The standing limitation is the one every homography-based ground projection has: it assumes the
 * world is the ground plane. Anything with height -- a cone, a wall, another car -- is smeared
 * along the ray away from the camera rather than sitting at its true footprint. That smear is
 * *conservative* for this layer's purpose (an obstacle's shadow is marked undrivable, and it
 * extends away from the vehicle, never toward it), but it is why this is a complement to the lidar
 * obstacle layer and not a replacement for it. The sim course's hill breaks the flat-world
 * assumption outright; expect this mask to be wrong on it.
 */
class DrivableAreaDetector
{
public:
  explicit DrivableAreaDetector(const DrivableAreaSettings & settings = {})
  : settings_{settings} {}

  struct Result
  {
    /// CV_8S, BEV-sized, one of kDrivable / kOffLimits / kUnknown per pixel.
    cv::Mat classes;

    /// False when the seed patch held too little surface to trust (see min_seed_coverage). The
    /// classes raster is then uniformly kUnknown, and the caller should say so out loud -- this
    /// is the "I have no idea where the road is" state, and it is silent otherwise.
    bool seeded{false};
  };

  /**
   * @brief Runs the four stages above over one warped frame.
   *
   * @param bev The bird's-eye view (BGR8), exactly as cv::warpPerspective produced it. Pixels
   * left pure black by the warp are treated as no-data, matching the convention
   * LaneDetection::publish_bev_cloud() already uses -- they are the corners the homography had no
   * source pixels for, not dark ground, and they come out kUnknown rather than kUndrivable.
   * @param origin_px The vehicle frame origin in BEV pixels (GroundProjection::origin_px()), used
   * to place the seed patch. Normally below the last row.
   * @param meters_per_pixel Ground distance one BEV pixel covers, for converting the settings'
   * metric radii into kernel sizes.
   */
  Result detect(
    const cv::Mat & bev, const cv::Point2d & origin_px, double meters_per_pixel) const;

  /// Tints `view` (BEV-sized, BGR8, modified in place) by classification -- amber over
  /// off-limits, green over drivable, untouched over no-data -- and outlines the seed patch.
  /// Debug only.
  void draw(
    const Result & result, const cv::Point2d & origin_px, double meters_per_pixel,
    cv::Mat & view) const;

  const DrivableAreaSettings & settings() const { return settings_; }

private:
  /// The seed patch as a BEV rectangle, clipped to `size`. Empty if it falls entirely outside.
  cv::Rect seed_rect(
    const cv::Size & size, const cv::Point2d & origin_px, double meters_per_pixel) const;

  DrivableAreaSettings settings_;
};

}  // namespace hyper_lane_detection

#endif  // HYPER_LANE_DETECTION__DRIVABLE_AREA_HPP_
