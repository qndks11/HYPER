#ifndef AUTO_VEHICLE__LANE_DETECTION_HPP_
#define AUTO_VEHICLE__LANE_DETECTION_HPP_

#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class LaneDetection : public rclcpp::Node
{
public:
  LaneDetection();

private:
  enum class LaneSide
  {
    kLeft,
    kRight
  };

  struct QuadraticLaneModel
  {
    bool valid{false};
    cv::Vec3d coefficients{0.0, 0.0, 0.0};
    double min_forward_px{0.0};
    double max_forward_px{0.0};
  };

  struct LaneFitResult
  {
    bool valid{false};

    // true: 현재 프레임의 왼쪽/오른쪽 경계가 모두 명확해서
    //       두 경계의 중간을 중심 경로로 사용했다.
    bool used_dual_lane_center{false};

    // true: 두 경계가 명확하지 않아 오른쪽 경계를 기준으로
    //       목표 경로를 생성했다.
    bool used_right_boundary{false};

    double offset_m{0.0};
    double steering_angle_deg{0.0};
    double curvature_radius_px{1e6};
    std::vector<cv::Point> curve_points;
  };

  struct StoplineResult
  {
    bool valid{false};
    double distance_m{0.0};
    cv::Rect bounding_box;
  };

  cv::Mat build_transform(
    int src_height, int src_width,
    int dst_height, int dst_width) const;

  cv::Mat bird_eye(const cv::Mat & image) const;
  cv::Mat yellow_mask(const cv::Mat & image) const;
  cv::Mat white_mask(const cv::Mat & image) const;

  std::vector<cv::Point> walk_lane_chain(
    const std::vector<cv::Point> & yellow_points,
    const cv::Point2d & origin,
    LaneSide side) const;

  QuadraticLaneModel fit_lane_model(
    const std::vector<cv::Point> & points,
    const cv::Point2d & origin) const;

  LaneFitResult build_center_path(
    const QuadraticLaneModel & left_model,
    const QuadraticLaneModel & right_model,
    const std::vector<cv::Point> & left_points,
    const std::vector<cv::Point> & right_points,
    const cv::Point2d & origin,
    int width,
    bool allow_dual_lane_center) const;

  StoplineResult find_stopline(
    const cv::Mat & mask,
    const cv::Point2d & origin,
    double meters_per_pixel) const;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stopline_publisher_;

  // 불명확한 양쪽 차선 판정을 막기 위해 이전 프레임의 왼쪽
  // 차선은 중앙 계산에 재사용하지 않는다. 오른쪽 차선만 짧게 재사용한다.
  std::vector<cv::Point> prev_right_points_;
  int right_missed_frames_{0};

  bool filter_initialized_{false};
  double filtered_offset_{0.0};
  double filtered_angle_{0.0};
  double filtered_curvature_{0.0};
};

#endif  // AUTO_VEHICLE__LANE_DETECTION_HPP_