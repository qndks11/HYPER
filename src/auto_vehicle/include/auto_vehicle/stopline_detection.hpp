#ifndef STOPLINE_DETECTION_HPP
#define STOPLINE_DETECTION_HPP

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class StoplineDetection : public rclcpp::Node
{
public:
  StoplineDetection();

private:
  struct StoplineResult
  {
    bool valid{false};
    double distance_m{0.0};
    cv::Rect bounding_box;  // the matched contour's bounding box, for drawing
  };

  /**
   * @brief Callback invoked for every incoming bird's-eye-view frame (published by
   * bev_producer).
   *
   * @details Masks white paint, then hands the mask to find_stopline() to pick out the stop-line
   * bar by shape. Publishes whether a stop line is visible and, if so, its distance ahead of the
   * vehicle on `/stopline/detection`, and shows a debug view (white mask, matched bounding box,
   * distance readout).
   *
   * @param msg Incoming BEV image.
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Produces a binary mask isolating white paint in HSV space (low saturation, high
   * value) -- unlike lane_detection's yellow_mask, which targets the lane paint's hue band
   * instead, since stop lines and lane lines are painted different colors in this course.
   */
  cv::Mat white_mask(const cv::Mat & image) const;

  /**
   * @brief Finds the stop-line bar in a white mask, if any.
   *
   * @details Runs cv::findContours() on the mask and keeps only contours whose bounding box is
   * shaped like a stop-line bar: wide relative to its height (aspect ratio >=
   * kMinStoplineAspectRatio) and wide relative to the frame (spans at least
   * kMinStoplineWidthFraction of the image width). The width-fraction check is what rules out a
   * zebra crossing's individual stripes, which can pass the aspect-ratio test on their own but
   * don't individually span the lane the way an unbroken stop-line bar does. Among survivors, the
   * one closest to the vehicle (largest row) is returned, since that's the stop line that
   * actually governs the vehicle's next stop.
   *
   * @param mask White paint mask (BEV, from white_mask()).
   * @param origin The bottom-center point representing the vehicle's position.
   * @param meters_per_pixel Scale factor from BEV pixels to meters.
   * @return The stop-line detection result; `valid` is false if nothing qualifies.
   */
  StoplineResult find_stopline(
    const cv::Mat & mask, const cv::Point2d & origin, double meters_per_pixel) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stopline_publisher_;
};

#endif  // STOPLINE_DETECTION_HPP
