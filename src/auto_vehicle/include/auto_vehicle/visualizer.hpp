#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class Visualizer : public rclcpp::Node
{
public:
  Visualizer();

private:
  /**
   * @brief Callback invoked for every incoming BEV frame: redraws the combined debug view using
   * this frame as the base, with the latest lane and stopline overlays (cached by
   * lane_overlay_callback() / stopline_overlay_callback()) stacked on top via compose_overlay().
   *
   * @details The lane and stopline overlays arrive on their own topics, asynchronously from this
   * one and from each other; rather than time-synchronizing all three, this just reuses whatever
   * each detector's most recently received overlay is. That can be a frame or two stale relative
   * to the base image, an acceptable tradeoff for a debug view over the complexity of exact sync.
   *
   * @param msg Incoming BEV image.
   */
  void bev_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Caches the latest lane_detection overlay (annotations on an otherwise-black canvas,
   * see lane_detection's doc) for bev_callback() to stack onto the next BEV frame.
   */
  void lane_overlay_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Caches the latest stopline_detection overlay, mirroring lane_overlay_callback().
   */
  void stopline_overlay_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bev_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr lane_overlay_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stopline_overlay_subscriber_;

  cv::Mat lane_overlay_;
  cv::Mat stopline_overlay_;
};

#endif  // VISUALIZER_HPP
