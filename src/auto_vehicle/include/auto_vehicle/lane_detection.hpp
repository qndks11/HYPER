#ifndef LANE_DETECTION_HPP
#define LANE_DETECTION_HPP

#include <string>
#include <utility>
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
  struct PolarSearchResult
  {
    std::vector<cv::Point> points;  // detected right-lane pixel centers, ordered near to far
    std::vector<cv::Rect> boxes;    // the search box that found each point, for visualization
  };

  struct LaneFitResult
  {
    bool valid{false};
    double offset_m{0.0};
    double steering_angle_deg{0.0};
    double curvature_radius_px{0.0};
    std::vector<cv::Point> spline_points;  // densely resampled curve, for drawing
  };

  /**
   * @brief Callback invoked for every incoming camera frame.
   *
   * @details Masks yellow lane paint, warps to a bird's-eye view, runs the polar right-lane
   * search, fits a parametric cubic spline through the result, publishes offset/steering
   * angle/curvature-radius on `/lane/center`, and shows a 4-panel debug dashboard (original,
   * bird's-eye yellow mask, polar search + spline, final overlay).
   *
   * @param msg Incoming camera image.
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Builds the perspective transform mapping the trapezoidal ROI to a bird's-eye view
   * of the same size as the source image.
   *
   * @param height Source image height [px].
   * @param width Source image width [px].
   * @return The 3x3 perspective transform matrix.
   */
  cv::Mat build_transform(int height, int width) const;

  /**
   * @brief Warps the input image to a bird's-eye view using build_transform().
   */
  cv::Mat bird_eye(const cv::Mat & image) const;

  /**
   * @brief Produces a binary mask isolating yellow lane paint in HSV space.
   */
  cv::Mat yellow_mask(const cv::Mat & image) const;

  /**
   * @brief Searches for the right lane in polar coordinates, anchored at the bottom-center of
   * the bird's-eye mask.
   *
   * @details For each angle from 0 (due right, along the bottom edge) to pi/2 (due forward) in
   * kAngleStepRad steps, finds the closest small box along that ray whose fill ratio clears
   * kSearchBoxFillThreshold, using an exponential-then-binary-search sweep over the radius so
   * each ray is O(log r) instead of a linear scan. Walking by angle rather than by image row
   * lets the search follow the lane even where it curves sharply sideways, which a
   * fixed-direction vertical search cannot. The sweep stops the moment an angle turns up
   * nothing (a disconnect) rather than continuing on to pi/2 regardless.
   *
   * @param binary Binary yellow lane mask (bird's-eye view).
   * @return The points found (near to far) and the boxes that found them.
   */
  PolarSearchResult polar_search(const cv::Mat & binary) const;

  /**
   * @brief Fits a parametric cubic spline through the polar search points and derives the
   * curvature radius, heading angle, and lateral offset near the vehicle.
   *
   * @param points Right-lane points from polar_search(), ordered near to far.
   * @param origin The bottom-center point the polar search was anchored to.
   * @param width Bird's-eye image width [px], used to scale pixel offset to meters.
   * @return The lane fit result; `valid` is false if there are too few points.
   */
  LaneFitResult fit_lane(
    const std::vector<cv::Point> & points, const cv::Point2d & origin, int width) const;

  /**
   * @brief Resizes an image to the dashboard thumbnail size and stamps a label on it.
   */
  cv::Mat make_thumbnail(const cv::Mat & image, const std::string & label) const;

  /**
   * @brief Arranges exactly four labeled views into a 2x2 grid for the debug dashboard.
   */
  cv::Mat build_dashboard(const std::vector<std::pair<std::string, cv::Mat>> & views) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;

  std::vector<cv::Point> prev_points_;
};

#endif  // LANE_DETECTION_HPP
