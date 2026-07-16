#include "parking_cpp/common.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

}  // namespace

class ParkingScanCostmap : public rclcpp::Node
{
public:
  ParkingScanCostmap()
  : Node("parking_scan_costmap"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered_map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/parking/local_costmap");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    base_frame_id_ = strip_slash(declare_parameter<std::string>("base_frame_id", "base_link"));

    resolution_ = declare_parameter<double>("resolution", 0.10);
    grid_width_m_ = declare_parameter<double>("grid_width_m", 16.0);
    grid_height_m_ = declare_parameter<double>("grid_height_m", 16.0);
    scan_min_range_ = declare_parameter<double>("scan_min_range", 0.08);
    scan_max_range_ = declare_parameter<double>("scan_max_range", 8.0);
    obstacle_inflation_m_ = declare_parameter<double>("obstacle_inflation_m", 0.10);
    occupied_value_ = declare_parameter<int>("occupied_value", 100);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.50);

    body_length_ = declare_parameter<double>("body_length", 1.34);
    body_width_ = declare_parameter<double>("body_width", 0.81);
    body_center_x_ = declare_parameter<double>("body_center_x_from_base_link", 0.0);
    body_center_y_ = declare_parameter<double>("body_center_y_from_base_link", 0.0);
    self_filter_margin_m_ = declare_parameter<double>("self_filter_margin_m", 0.03);
    self_filter_enabled_ = declare_parameter<bool>("self_filter_enabled", true);

    use_laser_tf_ = declare_parameter<bool>("use_laser_tf", true);
    require_laser_tf_ = declare_parameter<bool>("require_laser_tf", true);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.05);
    manual_laser_x_ = declare_parameter<double>("laser_x", 0.0);
    manual_laser_y_ = declare_parameter<double>("laser_y", 0.0);
    manual_laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);

    if (resolution_ <= 0.0 || grid_width_m_ <= 0.0 || grid_height_m_ <= 0.0) {
      throw std::runtime_error("Costmap resolution and dimensions must be positive.");
    }
    if (body_length_ <= 0.0 || body_width_ <= 0.0) {
      throw std::runtime_error("body_length and body_width must be positive.");
    }

    const double half_length = 0.5 * body_length_;
    const double half_width = 0.5 * body_width_;
    body_min_x_ = body_center_x_ - half_length;
    body_max_x_ = body_center_x_ + half_length;
    body_min_y_ = body_center_y_ - half_width;
    body_max_y_ = body_center_y_ + half_width;

    inflation_offsets_ = make_inflation_offsets();

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, map_qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&ParkingScanCostmap::on_odom, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ParkingScanCostmap::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "C++ parking scan costmap started: body=%.2fx%.2fm, body_x=[%.2f, %.2f], body_y=[%.2f, %.2f]",
      body_length_, body_width_, body_min_x_, body_max_x_, body_min_y_, body_max_y_);
  }

private:
  static std::string strip_slash(std::string value)
  {
    while (!value.empty() && value.front() == '/') {
      value.erase(value.begin());
    }
    return value;
  }

  double now_s() const
  {
    return now().seconds();
  }

  std::vector<std::pair<int, int>> make_inflation_offsets() const
  {
    std::vector<std::pair<int, int>> offsets;
    const int cells = std::max(0, static_cast<int>(std::ceil(obstacle_inflation_m_ / resolution_)));
    for (int dx = -cells; dx <= cells; ++dx) {
      for (int dy = -cells; dy <= cells; ++dy) {
        const double distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution_;
        if (distance <= obstacle_inflation_m_ + 0.5 * resolution_) {
          offsets.emplace_back(dx, dy);
        }
      }
    }
    if (offsets.empty()) {
      offsets.emplace_back(0, 0);
    }
    return offsets;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    vehicle_pose_ = Pose2D{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      parking_cpp::yaw_from_quaternion(msg->pose.pose.orientation)};
    odom_time_s_ = now_s();
  }

  bool point_inside_vehicle(double x, double y) const
  {
    if (!self_filter_enabled_) {
      return false;
    }
    return x >= body_min_x_ - self_filter_margin_m_ &&
           x <= body_max_x_ + self_filter_margin_m_ &&
           y >= body_min_y_ - self_filter_margin_m_ &&
           y <= body_max_y_ + self_filter_margin_m_;
  }

  std::optional<Pose2D> laser_pose_in_base(const sensor_msgs::msg::LaserScan & scan)
  {
    if (!use_laser_tf_) {
      return Pose2D{manual_laser_x_, manual_laser_y_, manual_laser_yaw_};
    }

    const std::string source_frame = strip_slash(scan.header.frame_id);
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/scan header.frame_id is empty.");
      if (require_laser_tf_) {
        return std::nullopt;
      }
      return Pose2D{manual_laser_x_, manual_laser_y_, manual_laser_yaw_};
    }

    if (source_frame == base_frame_id_) {
      return Pose2D{};
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_id_, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_s_));
      return Pose2D{
        transform.transform.translation.x,
        transform.transform.translation.y,
        parking_cpp::yaw_from_quaternion(transform.transform.rotation)};
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform %s -> %s: %s",
        source_frame.c_str(), base_frame_id_.c_str(), exception.what());
      if (require_laser_tf_) {
        return std::nullopt;
      }
      return Pose2D{manual_laser_x_, manual_laser_y_, manual_laser_yaw_};
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!vehicle_pose_.has_value()) {
      return;
    }
    if (now_s() - odom_time_s_ > odom_timeout_s_) {
      return;
    }

    const auto laser_pose = laser_pose_in_base(*msg);
    if (!laser_pose.has_value()) {
      return;
    }

    const int width = std::max(1, static_cast<int>(std::ceil(grid_width_m_ / resolution_)));
    const int height = std::max(1, static_cast<int>(std::ceil(grid_height_m_ / resolution_)));
    const double origin_x = vehicle_pose_->x - 0.5 * static_cast<double>(width) * resolution_;
    const double origin_y = vehicle_pose_->y - 0.5 * static_cast<double>(height) * resolution_;
    std::vector<int8_t> data(static_cast<std::size_t>(width * height), 0);

    const double c_laser = std::cos(laser_pose->yaw);
    const double s_laser = std::sin(laser_pose->yaw);
    const double c_vehicle = std::cos(vehicle_pose_->yaw);
    const double s_vehicle = std::sin(vehicle_pose_->yaw);
    const double minimum = std::max(static_cast<double>(msg->range_min), scan_min_range_);
    const double maximum = std::min(static_cast<double>(msg->range_max), scan_max_range_);

    double angle = msg->angle_min;
    for (const float raw_range : msg->ranges) {
      const double range = static_cast<double>(raw_range);
      if (std::isfinite(range) && range >= minimum && range <= maximum) {
        const double laser_px = range * std::cos(angle);
        const double laser_py = range * std::sin(angle);
        const double base_px = laser_pose->x + c_laser * laser_px - s_laser * laser_py;
        const double base_py = laser_pose->y + s_laser * laser_px + c_laser * laser_py;

        if (!point_inside_vehicle(base_px, base_py)) {
          const double map_px = vehicle_pose_->x + c_vehicle * base_px - s_vehicle * base_py;
          const double map_py = vehicle_pose_->y + s_vehicle * base_px + c_vehicle * base_py;
          const int mx = static_cast<int>(std::floor((map_px - origin_x) / resolution_));
          const int my = static_cast<int>(std::floor((map_py - origin_y) / resolution_));

          if (mx >= 0 && my >= 0 && mx < width && my < height) {
            for (const auto & [dx, dy] : inflation_offsets_) {
              const int gx = mx + dx;
              const int gy = my + dy;
              if (gx >= 0 && gy >= 0 && gx < width && gy < height) {
                data[static_cast<std::size_t>(gy * width + gx)] =
                  static_cast<int8_t>(parking_cpp::clamp(occupied_value_, 0, 100));
              }
            }
          }
        }
      }
      angle += msg->angle_increment;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = msg->header.stamp;
    grid.header.frame_id = frame_id_;
    grid.info.map_load_time = now();
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width);
    grid.info.height = static_cast<uint32_t>(height);
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data = std::move(data);
    costmap_pub_->publish(grid);
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string base_frame_id_;

  double resolution_{0.10};
  double grid_width_m_{16.0};
  double grid_height_m_{16.0};
  double scan_min_range_{0.08};
  double scan_max_range_{8.0};
  double obstacle_inflation_m_{0.10};
  int occupied_value_{100};
  double odom_timeout_s_{0.50};

  double body_length_{1.34};
  double body_width_{0.81};
  double body_center_x_{0.0};
  double body_center_y_{0.0};
  double body_min_x_{-0.67};
  double body_max_x_{0.67};
  double body_min_y_{-0.405};
  double body_max_y_{0.405};
  double self_filter_margin_m_{0.03};
  bool self_filter_enabled_{true};

  bool use_laser_tf_{true};
  bool require_laser_tf_{true};
  double tf_timeout_s_{0.05};
  double manual_laser_x_{0.0};
  double manual_laser_y_{0.0};
  double manual_laser_yaw_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<Pose2D> vehicle_pose_;
  double odom_time_s_{-1e9};
  std::vector<std::pair<int, int>> inflation_offsets_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingScanCostmap>());
  rclcpp::shutdown();
  return 0;
}
