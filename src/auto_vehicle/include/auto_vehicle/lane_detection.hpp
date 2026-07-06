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
  struct LaneFitResult
  {
    bool valid{false};
    double offset_m{0.0};
    double steering_angle_deg{0.0};
    double curvature_px{0.0};
    cv::Vec3d left_fit{0.0, 0.0, 0.0};
    cv::Vec3d right_fit{0.0, 0.0, 0.0};
  };

  /**
   * @brief Callback invoked for every incoming camera frame.
   *
   * @details Warps the frame to a bird's-eye view, extracts the lane binary mask, runs the
   * contour-based sliding-window search, fits a quadratic to each lane, publishes the lane
   * offset/steering angle/curvature on `/lane/center`, and shows a 6-panel debug dashboard
   * (original, bird's-eye, threshold mask, sliding windows, final overlay, fitted lane).
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
   * @brief Produces a binary mask isolating white and yellow lane markings in HSV space.
   */
  cv::Mat binary_mask(const cv::Mat & image) const;

  /**
   * @brief Runs the contour-based sliding-window lane search over the binary mask.
   *
   * @details A histogram of the bottom half of the mask locates the initial left/right base
   * positions. Stacked windows re-center on the largest contour's centroid found within a
   * margin of the previous window (falling back to the previous base when no contour clears
   * the pixel-count threshold). Windows are drawn in white and centroids in green on
   * `windows_view` for the debug dashboard.
   *
   * @param binary Binary lane mask (bird's-eye view).
   * @param windows_view BGR bird's-eye mask annotated in place with windows and centroids.
   * @param left_points Output centroids found for the left lane.
   * @param right_points Output centroids found for the right lane.
   */
  void sliding_window_search(
    const cv::Mat & binary, cv::Mat & windows_view, std::vector<cv::Point> & left_points,
    std::vector<cv::Point> & right_points) const;

  /**
   * @brief Fits a quadratic to each lane's centroids and derives curvature, offset, and
   * steering angle.
   *
   * @param left_points Left lane centroids (bird's-eye view).
   * @param right_points Right lane centroids (bird's-eye view).
   * @param width Bird's-eye image width [px].
   * @param height Bird's-eye image height [px].
   * @return The lane fit result; `valid` is false if either side has too few points.
   */
  LaneFitResult evaluate_lane(
    const std::vector<cv::Point> & left_points, const std::vector<cv::Point> & right_points,
    int width, int height) const;

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

  std::vector<cv::Point> prev_left_points_;
  std::vector<cv::Point> prev_right_points_;
};

#endif  // LANE_DETECTION_HPP
