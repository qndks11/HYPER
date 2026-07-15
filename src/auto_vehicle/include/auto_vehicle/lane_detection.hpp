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
  enum class LaneSide { kLeft, kRight };

  struct LaneFitResult
  {
    bool valid{false};
    double offset_m{0.0};
    double steering_angle_deg{0.0};
    double curvature_radius_px{0.0};
    std::vector<cv::Point> curve_points;  // the fitted line's two endpoints, for drawing
  };

  /**
   * @brief Callback invoked for every incoming bird's-eye-view frame (published by
   * bev_producer, which owns the raw-camera-to-BEV warp).
   *
   * @details Masks yellow lane paint, extracts the yellow pixels as a bare (x, y) point cloud,
   * and walks a chain from each side (left and right of the vehicle's assumed centerline) via
   * walk_lane_chain(), dropping any chain that crosses to the other side without getting farther
   * from the vehicle -- see walk_lane_chain()'s doc for why that indicates both sides latched
   * onto the same physical lane line. Each surviving chain is fit to a straight line via
   * fit_lane(); when both are valid the two are averaged into a true lane-center estimate (see
   * kLaneCenterOffsetBiasM), otherwise whichever single side is valid is used as-is. Publishes
   * offset/steering angle/curvature-radius on `/lane/center`, and shows a single BEV debug view
   * (yellow mask, both walked chains, both fitted lines, and offset/angle stats).
   *
   * @param msg Incoming BEV image.
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Produces a binary mask isolating yellow lane paint in HSV space.
   */
  cv::Mat yellow_mask(const cv::Mat & image) const;

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
   * walk out along essentially the same pixels -- the caller distinguishes a genuine crossing lane
   * from this spurious duplicate by checking that the chain still gets farther from the vehicle
   * from start to end (see image_callback).
   *
   * @param yellow_points Every yellow mask pixel coordinate (BEV), e.g. from cv::findNonZero.
   * @param origin The bottom-center point used to pick the seed (x >= origin.x for kRight,
   * x < origin.x for kLeft).
   * @param side Which half of the image to seed the chain from.
   * @return The walked chain's original BEV pixel coordinates, near (bottom) to far.
   */
  std::vector<cv::Point> walk_lane_chain(
    const std::vector<cv::Point> & yellow_points, const cv::Point2d & origin, LaneSide side) const;

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
   * approximation that degrades with distance. The lane-center bias (kLaneCenterOffsetBiasM) is
   * applied with a sign matching `side`: the center sits on the vehicle's side of a right lane
   * line, but on the far side of a left lane line.
   *
   * @param points The lane chain from walk_lane_chain(), in BEV coordinates, near to far.
   * @param origin The bottom-center point the chain was anchored to.
   * @param width Bird's-eye image width [px], used to scale pixel offset to meters.
   * @param side Which lane line `points` traces, so the center-offset bias is signed correctly.
   * @return The lane fit result; `valid` is false if there are too few points.
   */
  LaneFitResult fit_lane(
    const std::vector<cv::Point> & points, const cv::Point2d & origin, int width,
    LaneSide side) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;

  std::vector<cv::Point> prev_left_points_;
  std::vector<cv::Point> prev_right_points_;
};

#endif  // LANE_DETECTION_HPP
