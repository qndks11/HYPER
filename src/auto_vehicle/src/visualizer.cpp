#include "auto_vehicle/visualizer.hpp"

#include "auto_vehicle/image_overlay.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;
}  // namespace

Visualizer::Visualizer() : Node{"visualizer"}
{
  bev_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/bev/image", 10, std::bind(&Visualizer::bev_callback, this, std::placeholders::_1));
  lane_overlay_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/lane/overlay", 10,
    std::bind(&Visualizer::lane_overlay_callback, this, std::placeholders::_1));
  stopline_overlay_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/stopline/overlay", 10,
    std::bind(&Visualizer::stopline_overlay_callback, this, std::placeholders::_1));

  cv::namedWindow("Visualizer", cv::WINDOW_NORMAL);
  cv::resizeWindow("Visualizer", kWindowWidth, kWindowHeight);

  RCLCPP_INFO(get_logger(), "Visualizer started");
}

void Visualizer::lane_overlay_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  lane_overlay_ = cv_ptr->image.clone();
}

void Visualizer::stopline_overlay_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  stopline_overlay_ = cv_ptr->image.clone();
}

void Visualizer::bev_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat view = cv_ptr->image.clone();
  if (!lane_overlay_.empty() && lane_overlay_.size() == view.size()) {
    compose_overlay(view, lane_overlay_);
  }
  if (!stopline_overlay_.empty() && stopline_overlay_.size() == view.size()) {
    compose_overlay(view, stopline_overlay_);
  }

  cv::imshow("Visualizer", view);
  cv::waitKey(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualizer>());
  rclcpp::shutdown();
  return 0;
}
