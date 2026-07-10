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
    std::vector<cv::Point> curve_points;  // densely resampled fitted curve, for drawing
  };

  /**
   * @brief Callback invoked for every incoming camera frame.
   *
   * @details Masks yellow lane paint, warps to a bird's-eye view, extracts the yellow pixels as
   * a bare (x, y) point cloud, walks a chain along the right lane from its bottom point via
   * walk_lane_chain(), fits a pair of third-degree polynomials (x and y, both parameterized by
   * arc length along the chain) through the walked chain's original BEV coordinates, publishes
   * offset/steering angle/curvature-radius on `/lane/center`, and shows a single BEV debug view
   * (yellow mask, walked chain, fitted curve, and offset/angle stats).
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
   * @brief Fits parametric cubics x(s) and y(s), both against arc length s along the walked lane
   * chain, by least squares, and derives the curvature radius, heading angle, and lateral offset
   * at the vehicle's row.
   *
   * @details Unlike a per-point interpolating fit, a global least-squares polynomial averages
   * out point-to-point noise instead of passing through every point exactly, so the fitted curve
   * is steadier frame to frame. Parameterizing by arc length rather than fitting x = f(y)
   * directly means the fit stays well-defined -- and curvature/heading stay numerically stable --
   * through a sharp or even 90-degree turn, where x = f(y) would stop being single-valued.
   *
   * @param points The lane chain from walk_lane_chain(), in BEV coordinates, near to far.
   * @param origin The bottom-center point the chain was anchored to.
   * @param width Bird's-eye image width [px], used to scale pixel offset to meters.
   * @return The lane fit result; `valid` is false if there are too few points.
   */
  LaneFitResult fit_lane(
    const std::vector<cv::Point> & points, const cv::Point2d & origin, int width) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;

  std::vector<cv::Point> prev_points_;
};

#endif  // LANE_DETECTION_HPP
