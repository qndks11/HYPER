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
    cv::Mat polar_image;                 // yellow mask remapped into (angle row, radius col)
    std::vector<cv::Point> polar_points;  // hits in polar-image (col=radius, row=angle) coords
    std::vector<cv::Rect> polar_boxes;    // the qualifying sliding window for each hit
    std::vector<cv::Point> points;        // the same hits, converted back to BEV (x, y)
  };

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
   * @details Masks yellow lane paint, warps to a bird's-eye view, remaps the mask into polar
   * coordinates anchored at the bottom-center of the BEV image, runs a sliding-window search
   * for the right lane in that polar image, converts the result back to BEV coordinates, fits
   * a third-degree polynomial through it, publishes offset/steering angle/curvature-radius on
   * `/lane/center`, and shows a 6-panel debug dashboard (original, BEV mask, polar mask, polar
   * search result, BEV curve, final overlay).
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
   * @brief Searches for the right lane in polar coordinates over the fixed domain [0, pi/2].
   *
   * @details Remaps `binary` into a polar image anchored at `origin`, with rows spanning
   * [0, pi/2] (angle, 0 = due right, pi/2 = due forward) and columns spanning [0, max radius]
   * (radius) -- so radius increases left to right along a row. For each row, a
   * horizontally-elongated sliding window (wide in radius, narrow in angle) scans left to
   * right; the first window whose fill ratio clears kWindowFillThreshold is refined down to the
   * exact nearest yellow column within it, so the result always tracks the true minimum radius
   * rather than just the coarse window position. The elongated shape lets a window survive
   * small local deviations without losing the lane.
   *
   * Rows before the first hit are skipped rather than treated as a disconnect -- the ROI
   * trapezoid used for the bird's-eye warp is intentionally wider at its near edge than the
   * camera's actual field of view (see build_transform), so the small-angle rows walk through
   * an extrapolated black dead zone before ever reaching real image content. Once the first hit
   * is found, a row with no qualifying window (a genuine disconnect) stops the sweep.
   *
   * @param binary Binary yellow lane mask (bird's-eye view).
   * @param origin The bottom-center point to anchor the polar transform to.
   * @return The polar image, the hits in both polar and BEV coordinates, and the windows found.
   */
  PolarSearchResult polar_search(const cv::Mat & binary, const cv::Point2d & origin) const;

  /**
   * @brief Fits a third-degree polynomial x = f(y) through the polar search points by least
   * squares, and derives the curvature radius, heading angle, and lateral offset at the
   * vehicle's row.
   *
   * @details Unlike a per-point interpolating fit, a global least-squares polynomial averages
   * out point-to-point noise instead of passing through every point exactly, so the fitted
   * curve is steadier frame to frame. Being single-valued in y, it cannot represent a curve
   * that folds back in x for a given y (an extremely sharp/hairpin turn) -- a deliberate
   * simplicity/stability trade-off.
   *
   * @param points Right-lane points from polar_search(), in BEV coordinates, near to far.
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
   * @brief Arranges exactly six labeled views into a 3x2 grid for the debug dashboard.
   */
  cv::Mat build_dashboard(const std::vector<std::pair<std::string, cv::Mat>> & views) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;

  std::vector<cv::Point> prev_points_;
};

#endif  // LANE_DETECTION_HPP
