#ifndef HYPER_LANE_DETECTION__STOPLINE_DETECTOR_HPP_
#define HYPER_LANE_DETECTION__STOPLINE_DETECTOR_HPP_

#include <opencv2/opencv.hpp>

namespace hyper_lane_detection
{

/// Detects the stop-line bar in a bird's-eye-warped frame and draws the debug overlay for it.
/// Operates purely on an already-warped frame and an already-computed white mask -- it knows
/// nothing about cameras, ROS, or how the frame arrived; see lane_detection_node.hpp for that.
class StoplineDetector
{
public:
  struct Result
  {
    bool valid{false};
    double distance_m{0.0};
    cv::Rect bounding_box;  // the matched contour's bounding box, for drawing
  };

  /**
   * @brief Produces a binary mask isolating white paint in HSV space (low saturation, high
   * value) -- unlike yellow_mask (LaneDetector), which targets the lane paint's hue band instead,
   * since stop lines and lane lines are painted different colors in this course.
   */
  cv::Mat white_mask(const cv::Mat & warped) const;

  /**
   * @brief Finds the stop-line bar in `white` and draws its bounding box into `view`.
   *
   * @details Computes its own meters-per-pixel scale internally from `white.cols` (via
   * bev_scale.hpp) -- the caller doesn't need to know or pass this scale factor. Does NOT
   * highlight the mask into `view` -- the caller highlights both the lane and stop-line masks
   * into `view` once, before calling either detector; see LaneDetector::detect_and_draw() for the
   * shared rationale.
   *
   * @param white White paint mask (BEV), from white_mask().
   * @param origin The bottom-center point representing the vehicle's position (BEV coordinates).
   * @param view Debug image to draw into; must already carry both mask highlights and be BEV-
   * sized (not yet bordered with the text panel).
   * @return The stop-line detection result, for publishing and draw_text().
   */
  Result detect_and_draw(const cv::Mat & white, const cv::Point2d & origin, cv::Mat & view) const;

  /// Draws the "Distance: .. / Stopline not detected" text block onto view's text panel, starting
  /// at panel_top.
  void draw_text(const Result & result, cv::Mat & view, int panel_top) const;

private:
  /**
   * @brief Finds the stop-line bar in a white mask, if any.
   *
   * @details Runs cv::findContours() on the mask and keeps only contours whose bounding box is
   * shaped like a stop-line bar: wide relative to its height (aspect ratio >=
   * kMinStoplineAspectRatio) and wide relative to the frame (spans at least
   * kMinStoplineWidthFraction of the image width). The width-fraction check is what rules out a
   * zebra crossing's individual stripes, which can pass the aspect-ratio test on their own but
   * don't individually span the lane the way an unbroken stop-line bar does. A shape match alone
   * isn't enough at a multi-lane intersection, though, since an adjacent lane's stop-line bar can
   * look identical -- candidates are also rejected if their horizontal center falls outside a
   * one-lane-wide band centered on `origin`'s x, so only a bar actually in the vehicle's own lane
   * survives. Among what's left, the one closest to the vehicle (largest row) is returned, since
   * that's the stop line that actually governs the vehicle's next stop.
   *
   * @param mask White paint mask (BEV, from white_mask()).
   * @param origin The bottom-center point representing the vehicle's position.
   * @param meters_per_pixel Scale factor from BEV pixels to meters.
   * @return The stop-line detection result; `valid` is false if nothing qualifies.
   */
  Result find_stopline(
    const cv::Mat & mask, const cv::Point2d & origin, double meters_per_pixel) const;
};

}  // namespace hyper_lane_detection

#endif  // HYPER_LANE_DETECTION__STOPLINE_DETECTOR_HPP_
