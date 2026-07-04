#ifndef LANE_DETECTION_HPP
#define LANE_DETECTION_HPP

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
    double offset{0.0};
    double heading_error{0.0};
  };

  /**
   * @brief Callback invoked for every incoming camera frame.
   *
   * @details Converts the ROS image to OpenCV, warps it to a bird's-eye view, extracts the
   * lane binary mask, runs the sliding-window search, publishes the lane center offset and
   * heading error on `/lane/center`, and shows the annotated bird's-eye view in a debug window.
   *
   * @param msg Incoming camera image.
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Builds the perspective transform mapping the trapezoidal ROI to a bird's-eye view.
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
   * @brief Runs the sliding-window lane search over the binary mask.
   *
   * @details Mirrors the Python reference implementation: a histogram of the bottom half of
   * the mask locates the initial left/right base positions, then the stacked windows re-center
   * on the mean pixel position within a margin of the previous window. Detected lane pixels are
   * drawn in green and the sliding windows in yellow on `overlay`.
   *
   * @param binary Binary lane mask (bird's-eye view).
   * @param overlay Bird's-eye color image annotated in place with the detected lane pixels.
   * @return The lane fit result (validity, offset, and heading error).
   */
  LaneFitResult sliding_window_search(const cv::Mat & binary, cv::Mat & overlay) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;
};

#endif  // LANE_DETECTION_HPP
