#ifndef BEV_PRODUCER_HPP
#define BEV_PRODUCER_HPP

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class BevProducer : public rclcpp::Node
{
public:
  BevProducer();

private:
  /**
   * @brief Callback invoked for every incoming camera frame: warps it to a bird's-eye view via
   * bird_eye() and republishes it on `/bev/image`, so downstream perception nodes
   * (lane_detection, stopline_detection) share one BEV transform instead of each computing their
   * own.
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

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bev_publisher_;
};

#endif  // BEV_PRODUCER_HPP
