#ifndef LANE_DETECTION_HPP
#define LANE_DETECTION_HPP

#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class LaneDetection : public rclcpp::Node
{
public:
  LaneDetection();

private:
  struct LaneFitResult
  {
    bool valid{false};
    double offset_m{0.0};
    double steering_angle_deg{0.0};
    double curvature_radius_px{0.0};
    std::vector<cv::Point> curve_points;  // the fitted line's two endpoints, for drawing
  };

  /**
   * @brief Callback invoked for every incoming camera frame.
   *
   * @details Masks yellow lane paint, warps to a bird's-eye view, extracts the yellow pixels as
   * a bare (x, y) point cloud, and walks a chain along the right lane from its bottom point via
   * walk_lane_chain(). If that chain is too short to fit or is_right_turn() judges it to already
   * be curving sharply right (a sign the right lane is about to sweep out of frame), the yellow
   * cloud is mirrored about the vehicle's x position and walked again -- picking up the left lane
   * instead, since mirroring flips both walk_lane_chain()'s right-half seed and the handedness of
   * its candidate ranking -- and the result mirrored back before use. Whichever chain is settled
   * on is fit to a straight line via fit_lane(), publishing offset/steering angle/curvature-radius
   * on `/lane/center`, and shown on a single BEV debug view (yellow mask, walked chain, fitted
   * line, and offset/angle stats).
   *
   * @param msg Incoming camera image.
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Builds the perspective transform mapping the trapezoidal ROI (sampled from the
   * source image's own dimensions) to a bird's-eye view of the given destination size.
   *
   * @param src_height Source image height [px], used only to locate the ROI corners.
   * @param src_width Source image width [px], used only to locate the ROI corners.
   * @param dst_height Output (bird's-eye) image height [px].
   * @param dst_width Output (bird's-eye) image width [px].
   * @return The 3x3 perspective transform matrix.
   */
  cv::Mat build_transform(int src_height, int src_width, int dst_height, int dst_width) const;

  /**
   * @brief Warps the input image to a bird's-eye view using build_transform().
   */
  cv::Mat bird_eye(const cv::Mat & image) const;

  /**
   * @brief Produces a binary mask isolating yellow lane paint in HSV space.
   */
  cv::Mat yellow_mask(const cv::Mat & image) const;

  /**
   * @brief Walks a chain of yellow pixels along the right lane, starting from its bottom point
   * and stepping outward one nearby pixel at a time.
   *
   * @details The seed point is the bottommost yellow pixel on the right half of the image
   * (x >= origin.x), and the initial direction is straight up the image (toward the horizon).
   * Each step: pixels within kNeighborRadius of the current point are filtered to those whose
   * displacement from it has a non-negative dot product with the current direction (i.e. within
   * +/-90 degrees of it); if none remain, the chain stops. The survivors are ranked by the
   * signed sine of their angle to the current direction (cross product over the product of
   * norms), smallest magnitude first -- since candidates are already restricted to +/-90
   * degrees, this orders identically to angle magnitude -- with remaining ties broken by
   * preferring the pixel farthest from the current point. The winner becomes the new current
   * point, and the vector from the old point to it becomes the new direction, so the chain
   * keeps following whatever heading the lane is actually curving toward.
   *
   * @param yellow_points Every yellow mask pixel coordinate (BEV), e.g. from cv::findNonZero.
   * @param origin The bottom-center point used to pick the right-lane seed (x >= origin.x).
   * @return The walked chain's original BEV pixel coordinates, near (bottom) to far.
   */
  std::vector<cv::Point> walk_lane_chain(
    const std::vector<cv::Point> & yellow_points, const cv::Point2d & origin) const;

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
   * always a large sentinel value. Offset is extrapolated from the fitted line out to the
   * vehicle's actual row (origin.y) rather than read at the anchor point: unlike the old curve
   * fits, which were only valid within the span of points they were fit to, a straight line has
   * the same direction everywhere, so walking it out to any row is exact rather than an
   * approximation that degrades with distance. The constant term converting "distance from the
   * tracked line to the vehicle" into "distance from lane center to the vehicle" flips sign
   * depending on which of the two lane boundaries is being tracked.
   *
   * @param points The lane chain from walk_lane_chain() (possibly mirrored back from a left-lane
   * walk -- see image_callback), in BEV coordinates, near to far.
   * @param origin The bottom-center point the chain was anchored to.
   * @param width Bird's-eye image width [px], used to scale pixel offset to meters.
   * @param tracking_left_lane Whether `points` came from the left lane boundary instead of the
   * right one, so the offset's lane-center bias is applied with the correct sign.
   * @return The lane fit result; `valid` is false if there are too few points.
   */
  LaneFitResult fit_lane(
    const std::vector<cv::Point> & points, const cv::Point2d & origin, int width,
    bool tracking_left_lane) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;

  std::vector<cv::Point> prev_points_;
  // Which lane boundary prev_points_ came from, so a frame that falls back to it (see
  // image_callback) still applies the matching offset bias.
  bool prev_tracking_left_lane_{false};
};

#endif  // LANE_DETECTION_HPP
