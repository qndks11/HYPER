#include "auto_vehicle/bev_producer.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
// ROI trapezoid corners as (row_ratio, col_ratio) of the source image
constexpr double kRoiTopLeftRow = 0.40,  kRoiTopLeftCol = 0.25;
constexpr double kRoiTopRightRow = 0.40, kRoiTopRightCol = 0.75;
constexpr double kRoiBottomLeftRow = 1,  kRoiBottomLeftCol = -1.4;
constexpr double kRoiBottomRightRow = 1, kRoiBottomRightCol = 2.4;

// Bird's-eye output height as a multiple of the source frame height, i.e. how much farther
// down the road the BEV view looks. Width is left unscaled.
constexpr double kBevHeightScale = 1.35;

constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;
}  // namespace

BevProducer::BevProducer() : Node{"bev_producer"}
{
  image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&BevProducer::image_callback, this, std::placeholders::_1));

  bev_publisher_ = create_publisher<sensor_msgs::msg::Image>("/bev/image", 10);

  cv::namedWindow("BEV", cv::WINDOW_NORMAL);
  cv::resizeWindow("BEV", kWindowWidth, static_cast<int>(kWindowHeight * kBevHeightScale));

  RCLCPP_INFO(get_logger(), "BevProducer started");
}

cv::Mat BevProducer::build_transform(
  int src_height, int src_width, int dst_height, int dst_width) const
{
  const std::vector<cv::Point2f> src{
    {static_cast<float>(src_width * kRoiTopLeftCol), static_cast<float>(src_height * kRoiTopLeftRow)},
    {static_cast<float>(src_width * kRoiTopRightCol), static_cast<float>(src_height * kRoiTopRightRow)},
    {static_cast<float>(src_width * kRoiBottomRightCol), static_cast<float>(src_height * kRoiBottomRightRow)},
    {static_cast<float>(src_width * kRoiBottomLeftCol), static_cast<float>(src_height * kRoiBottomLeftRow)}};

  const std::vector<cv::Point2f> dst{
    {0.0f, 0.0f},
    {static_cast<float>(dst_width), 0.0f},
    {static_cast<float>(dst_width), static_cast<float>(dst_height)},
    {0.0f, static_cast<float>(dst_height)}};

  return cv::getPerspectiveTransform(src, dst);
}

cv::Mat BevProducer::bird_eye(const cv::Mat & image) const
{
  const cv::Size dst_size(image.cols, static_cast<int>(image.rows * kBevHeightScale));
  const cv::Mat transform = build_transform(image.rows, image.cols, dst_size.height, dst_size.width);
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, dst_size);
  return warped;
}

void BevProducer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat warped = bird_eye(cv_ptr->image);

  const sensor_msgs::msg::Image::SharedPtr bev_msg =
    cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, warped).toImageMsg();
  bev_publisher_->publish(*bev_msg);

  cv::imshow("BEV", warped);
  cv::waitKey(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BevProducer>());
  rclcpp::shutdown();
  return 0;
}
