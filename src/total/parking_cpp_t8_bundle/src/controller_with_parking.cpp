#include "parking_cpp/common.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Command
{
  double steering{0.0};
  double speed{0.0};
  bool valid{false};
};

}  // namespace

class ControllerWithParking : public rclcpp::Node
{
public:
  ControllerWithParking()
  : Node("controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    wheelbase_ = declare_parameter<double>("wheelbase", 1.005);
    control_hz_ = declare_parameter<double>("control_hz", 40.0);
    max_steer_ = declare_parameter<double>("max_steering_angle", 0.5235988);
    steering_rate_ = declare_parameter<double>("max_steering_angular_velocity", 2.0);
    max_velocity_ = declare_parameter<double>("max_velocity", 4.44);

    cruise_speed_ = declare_parameter<double>("cruise_speed", 3.75);
    approach_speed_ = declare_parameter<double>("approach_speed", 0.50);
    bridge_speed_ = declare_parameter<double>("bridge_speed", 2.50);
    waypoint_speed_ = declare_parameter<double>("waypoint_speed", 2.50);
    minimum_speed_ = declare_parameter<double>("min_speed", 1.25);
    acceleration_limit_ = declare_parameter<double>("accel_limit", 2.0);
    deceleration_limit_ = declare_parameter<double>("decel_limit", 4.0);
    curve_slow_k_ = declare_parameter<double>("curve_slow_k", 1.2);
    lane_curve_speed_reduction_ = declare_parameter<double>("lane_curve_speed_reduction", 0.40);

    lane_lookahead_straight_ = declare_parameter<double>("lane_lookahead_straight", 1.0);
    lane_lookahead_curve_ = declare_parameter<double>("lane_lookahead_curve", 0.60);
    lane_straight_radius_px_ = declare_parameter<double>("lane_straight_radius_px", 1400.0);
    lane_sharp_curve_radius_px_ = declare_parameter<double>("lane_sharp_curve_radius_px", 280.0);
    lane_min_forward_ = declare_parameter<double>("lane_min_forward", 0.10);
    lane_path_min_points_ = declare_parameter<int>("lane_path_min_points", 3);
    lane_target_filter_alpha_ = declare_parameter<double>("lane_target_filter_alpha", 0.35);

    stanley_k_ = declare_parameter<double>("stanley_k", 0.8);
    stanley_v_min_ = declare_parameter<double>("stanley_v_min", 0.5);
    lane_offset_sign_ = declare_parameter<double>("lane_offset_sign", -1.0);
    lane_heading_sign_ = declare_parameter<double>("lane_heading_sign", 1.0);
    lane_filter_alpha_ = declare_parameter<double>("lane_filter_alpha", 0.3);
    avoid_offset_bias_ = declare_parameter<double>("avoid_offset_bias", 0.45);
    map_lookahead_ = declare_parameter<double>("lookahead", 1.0);

    parking_forward_speed_ = declare_parameter<double>("parking_forward_speed", 0.25);
    parking_reverse_speed_ = declare_parameter<double>("parking_reverse_speed", 0.20);
    parking_lookahead_ = declare_parameter<double>("parking_lookahead", 0.45);
    parking_search_window_ = declare_parameter<int>("parking_search_window", 35);
    gear_change_pause_s_ = declare_parameter<double>("gear_change_pause_s", 0.60);
    gear_change_stop_speed_ = declare_parameter<double>("gear_change_stop_speed", 0.03);
    parking_goal_tolerance_m_ = declare_parameter<double>("parking_goal_tolerance_m", 0.20);
    parking_goal_yaw_tolerance_ =
      declare_parameter<double>("parking_goal_yaw_tolerance_deg", 8.0) * parking_cpp::kPi / 180.0;
    parking_index_end_margin_ = declare_parameter<int>("parking_index_end_margin", 2);

    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    require_scan_for_parking_ = declare_parameter<bool>("require_scan_for_parking", true);
    scan_timeout_s_ = declare_parameter<double>("scan_timeout_s", 0.50);
    require_forward_scan_coverage_ = declare_parameter<bool>("require_forward_scan_coverage", true);
    require_reverse_scan_coverage_ = declare_parameter<bool>("require_reverse_scan_coverage", true);

    base_frame_id_ = strip_slash(declare_parameter<std::string>("base_frame_id", "base_link"));
    body_length_ = declare_parameter<double>("body_length", 1.34);
    body_width_ = declare_parameter<double>("body_width", 0.81);
    body_center_x_ = declare_parameter<double>("body_center_x_from_base_link", 0.0);
    body_center_y_ = declare_parameter<double>("body_center_y_from_base_link", 0.0);
    self_filter_enabled_ = declare_parameter<bool>("self_filter_enabled", true);
    self_filter_margin_m_ = declare_parameter<double>("self_filter_margin_m", 0.03);
    parking_obstacle_clearance_m_ = declare_parameter<double>("parking_obstacle_clearance_m", 0.25);
    const double corridor_margin =
      declare_parameter<double>("parking_obstacle_corridor_margin_m", 0.10);

    use_laser_tf_ = declare_parameter<bool>("use_laser_tf", true);
    require_laser_tf_ = declare_parameter<bool>("require_laser_tf", true);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.05);
    manual_laser_x_ = declare_parameter<double>("laser_x", 0.0);
    manual_laser_y_ = declare_parameter<double>("laser_y", 0.0);
    manual_laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);
    input_timeout_s_ = declare_parameter<double>("input_timeout", 0.30);

    const double half_length = 0.5 * body_length_;
    const double half_width = 0.5 * body_width_;
    body_min_x_ = body_center_x_ - half_length;
    body_max_x_ = body_center_x_ + half_length;
    body_min_y_ = body_center_y_ - half_width;
    body_max_y_ = body_center_y_ + half_width;
    vehicle_front_m_ = std::max(0.0, body_max_x_);
    vehicle_rear_m_ = std::max(0.0, -body_min_x_);
    parking_corridor_min_y_ = body_min_y_ - corridor_margin;
    parking_corridor_max_y_ = body_max_y_ + corridor_margin;

    const auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/driving_mode", 10,
      std::bind(&ControllerWithParking::on_mode, this, std::placeholders::_1));
    lane_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/lane/center", 10,
      std::bind(&ControllerWithParking::on_lane, this, std::placeholders::_1));
    bridge_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/bridge_path", 10,
      std::bind(&ControllerWithParking::on_bridge, this, std::placeholders::_1));
    waypoint_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/waypoint_path", 10,
      std::bind(&ControllerWithParking::on_waypoint_path, this, std::placeholders::_1));
    parking_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/parking/path", transient_qos,
      std::bind(&ControllerWithParking::on_parking_path, this, std::placeholders::_1));
    parking_directions_sub_ = create_subscription<std_msgs::msg::Int8MultiArray>(
      "/parking/directions", transient_qos,
      std::bind(&ControllerWithParking::on_parking_directions, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map", 10,
      std::bind(&ControllerWithParking::on_odom, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ControllerWithParking::on_scan, this, std::placeholders::_1));

    command_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
    velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/velocity", 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);
    parking_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/parking/controller_status", transient_qos);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ControllerWithParking::control_step, this));
    last_control_time_s_ = now_s();
    publish_parking_status("IDLE");

    RCLCPP_INFO(
      get_logger(),
      "C++ controller started: parking F=%.2f, R=%.2f m/s, T8 body=%.2fx%.2fm",
      parking_forward_speed_, parking_reverse_speed_, body_length_, body_width_);
  }

private:
  static std::string strip_slash(std::string value)
  {
    while (!value.empty() && value.front() == '/') {
      value.erase(value.begin());
    }
    return value;
  }

  double now_s() const {return now().seconds();}

  void publish_parking_status(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    parking_status_pub_->publish(message);
  }

  void on_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string new_mode = msg->data;
    if (new_mode != mode_) {
      filtered_lane_target_.reset();
      if (new_mode == "PARKING") {
        parking_index_ = 0;
        current_gear_ = parking_directions_.empty() ? 1 : parking_directions_.front();
        pending_gear_.reset();
        parking_done_published_ = false;
        scan_stop_reason_.clear();
        publish_parking_status("TRACKING");
      } else if (mode_ == "PARKING") {
        pending_gear_.reset();
      }
    }
    mode_ = new_mode;
  }

  void on_lane(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    lane_time_s_ = now_s();
    if (msg->data.size() < 4U) {
      lane_valid_ = false;
      lane_path_local_.clear();
      filtered_lane_target_.reset();
      return;
    }

    const double raw_offset = msg->data[0];
    const double raw_heading = msg->data[1] * parking_cpp::kPi / 180.0;
    lane_radius_px_ = msg->data[2];
    const bool valid = msg->data[3] > 0.5;

    if (valid) {
      if (!lane_initialized_) {
        lane_offset_ = raw_offset;
        lane_heading_ = raw_heading;
        lane_initialized_ = true;
      } else {
        const double alpha = parking_cpp::clamp(lane_filter_alpha_, 0.0, 1.0);
        lane_offset_ = alpha * raw_offset + (1.0 - alpha) * lane_offset_;
        lane_heading_ = alpha * raw_heading + (1.0 - alpha) * lane_heading_;
      }

      lane_path_local_.clear();
      for (std::size_t index = 4; index + 1U < msg->data.size(); index += 2U) {
        const double forward = msg->data[index];
        const double left = msg->data[index + 1U];
        if (std::isfinite(forward) && std::isfinite(left) && forward >= lane_min_forward_) {
          lane_path_local_.push_back(Point2D{forward, left});
        }
      }
      std::sort(
        lane_path_local_.begin(), lane_path_local_.end(),
        [](const Point2D & lhs, const Point2D & rhs) {return lhs.x < rhs.x;});
      if (static_cast<int>(lane_path_local_.size()) < lane_path_min_points_) {
        lane_path_local_.clear();
        filtered_lane_target_.reset();
      }
    } else {
      lane_path_local_.clear();
      filtered_lane_target_.reset();
      lane_initialized_ = false;
    }
    lane_valid_ = valid;
  }

  void on_bridge(const nav_msgs::msg::Path::SharedPtr msg)
  {
    bridge_path_.clear();
    bridge_path_.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      bridge_path_.push_back(Point2D{pose.pose.position.x, pose.pose.position.y});
    }
  }

  void on_waypoint_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    waypoint_path_.clear();
    waypoint_path_.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      waypoint_path_.push_back(Point2D{pose.pose.position.x, pose.pose.position.y});
    }
  }

  void on_parking_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    parking_path_.clear();
    parking_path_.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      parking_path_.push_back(Pose2D{
        pose.pose.position.x,
        pose.pose.position.y,
        parking_cpp::yaw_from_quaternion(pose.pose.orientation)});
    }
    parking_index_ = 0;
    parking_done_published_ = false;
    validate_parking_inputs();
  }

  void on_parking_directions(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
  {
    parking_directions_.clear();
    parking_directions_.reserve(msg->data.size());
    for (const auto value : msg->data) {
      parking_directions_.push_back(value >= 0 ? 1 : -1);
    }
    parking_index_ = 0;
    parking_done_published_ = false;
    validate_parking_inputs();
  }

  void validate_parking_inputs()
  {
    if (parking_path_.empty() || parking_directions_.empty()) {
      return;
    }
    if (parking_path_.size() != parking_directions_.size()) {
      RCLCPP_ERROR(
        get_logger(), "Parking path/directions mismatch: %zu vs %zu",
        parking_path_.size(), parking_directions_.size());
      publish_parking_status("ERROR");
      return;
    }
    publish_parking_status("IDLE");
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = parking_cpp::yaw_from_quaternion(msg->pose.pose.orientation);
    velocity_ = msg->twist.twist.linear.x;
    odom_time_s_ = now_s();
  }

  std::optional<Pose2D> laser_pose_in_base(const sensor_msgs::msg::LaserScan & scan)
  {
    if (!use_laser_tf_) {
      return Pose2D{manual_laser_x_, manual_laser_y_, manual_laser_yaw_};
    }
    const std::string source_frame = strip_slash(scan.header.frame_id);
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/scan frame_id is empty.");
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

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto laser_pose = laser_pose_in_base(*msg);
    if (!laser_pose.has_value()) {
      return;
    }

    double min_forward = std::numeric_limits<double>::infinity();
    double min_reverse = std::numeric_limits<double>::infinity();
    bool forward_coverage = false;
    bool reverse_coverage = false;
    const double c_laser = std::cos(laser_pose->yaw);
    const double s_laser = std::sin(laser_pose->yaw);
    const double minimum = std::max(0.01, static_cast<double>(msg->range_min));
    const double maximum = static_cast<double>(msg->range_max);

    double angle = msg->angle_min;
    for (const float raw_range : msg->ranges) {
      const double beam_x = c_laser * std::cos(angle) - s_laser * std::sin(angle);
      const double beam_y = s_laser * std::cos(angle) + c_laser * std::sin(angle);
      if (beam_x >= 0.25) {
        forward_coverage = true;
      } else if (beam_x <= -0.25) {
        reverse_coverage = true;
      }

      const double range = static_cast<double>(raw_range);
      if (std::isfinite(range) && range >= minimum && range <= maximum) {
        const double base_x = laser_pose->x + range * beam_x;
        const double base_y = laser_pose->y + range * beam_y;
        if (!point_inside_vehicle(base_x, base_y) &&
          base_y >= parking_corridor_min_y_ && base_y <= parking_corridor_max_y_)
        {
          if (base_x >= 0.0) {
            min_forward = std::min(min_forward, base_x);
          } else {
            min_reverse = std::min(min_reverse, std::abs(base_x));
          }
        }
      }
      angle += msg->angle_increment;
    }

    scan_min_forward_m_ = min_forward;
    scan_min_reverse_m_ = min_reverse;
    scan_has_forward_coverage_ = forward_coverage;
    scan_has_reverse_coverage_ = reverse_coverage;
    scan_time_s_ = now_s();
  }

  std::pair<bool, std::string> parking_scan_blocked(int direction) const
  {
    if (!require_scan_for_parking_) {
      return {false, ""};
    }
    if (now_s() - scan_time_s_ > scan_timeout_s_) {
      return {true, "SCAN_STALE"};
    }
    if (direction >= 0) {
      if (require_forward_scan_coverage_ && !scan_has_forward_coverage_) {
        return {true, "NO_FORWARD_SCAN_COVERAGE"};
      }
      const double threshold = vehicle_front_m_ + parking_obstacle_clearance_m_;
      if (scan_min_forward_m_ <= threshold) {
        return {true, "FORWARD_OBSTACLE"};
      }
    } else {
      if (require_reverse_scan_coverage_ && !scan_has_reverse_coverage_) {
        return {true, "NO_REVERSE_SCAN_COVERAGE"};
      }
      const double threshold = vehicle_rear_m_ + parking_obstacle_clearance_m_;
      if (scan_min_reverse_m_ <= threshold) {
        return {true, "REVERSE_OBSTACLE"};
      }
    }
    return {false, ""};
  }

  double lane_curve_strength() const
  {
    const double radius = std::abs(lane_radius_px_);
    if (!std::isfinite(radius) || radius >= lane_straight_radius_px_) {
      return 0.0;
    }
    if (radius <= lane_sharp_curve_radius_px_) {
      return 1.0;
    }
    const double denominator = std::max(1.0, lane_straight_radius_px_ - lane_sharp_curve_radius_px_);
    return parking_cpp::clamp((lane_straight_radius_px_ - radius) / denominator, 0.0, 1.0);
  }

  double lane_lookahead() const
  {
    const double strength = lane_curve_strength();
    return lane_lookahead_straight_ * (1.0 - strength) + lane_lookahead_curve_ * strength;
  }

  Command lane_path_pure_pursuit(double lateral_bias)
  {
    if (lane_path_local_.empty()) {
      return Command{};
    }
    const double lookahead = lane_lookahead();
    Point2D target = lane_path_local_.back();
    for (const auto & point : lane_path_local_) {
      if (std::hypot(point.x, point.y) >= lookahead) {
        target = point;
        break;
      }
    }
    target.y += lateral_bias;
    if (!filtered_lane_target_.has_value()) {
      filtered_lane_target_ = target;
    } else {
      const double alpha = parking_cpp::clamp(lane_target_filter_alpha_, 0.0, 1.0);
      filtered_lane_target_->x = alpha * target.x + (1.0 - alpha) * filtered_lane_target_->x;
      filtered_lane_target_->y = alpha * target.y + (1.0 - alpha) * filtered_lane_target_->y;
    }
    const double distance_squared =
      filtered_lane_target_->x * filtered_lane_target_->x +
      filtered_lane_target_->y * filtered_lane_target_->y;
    if (filtered_lane_target_->x <= lane_min_forward_ || distance_squared < 1e-6) {
      return Command{};
    }
    return Command{
      std::atan2(2.0 * wheelbase_ * filtered_lane_target_->y, distance_squared),
      0.0,
      true};
  }

  double lane_follow_stanley(double bias) const
  {
    const double error = (lane_offset_ - bias) * lane_offset_sign_;
    const double heading = lane_heading_ * lane_heading_sign_;
    const double speed = std::max(std::abs(velocity_), stanley_v_min_);
    return heading + std::atan2(stanley_k_ * error, speed);
  }

  double lane_follow_command(double bias)
  {
    const Command pursuit = lane_path_pure_pursuit(bias);
    return pursuit.valid ? pursuit.steering : lane_follow_stanley(bias);
  }

  double speed_for_curve(double steering) const
  {
    const double steering_ratio = parking_cpp::clamp(
      std::abs(steering) / std::max(max_steer_, 1e-6), 0.0, 1.0);
    const double steering_speed = cruise_speed_ / (1.0 + curve_slow_k_ * steering_ratio);
    const double curve_scale = 1.0 - lane_curve_speed_reduction_ * lane_curve_strength();
    return std::max(minimum_speed_, steering_speed * curve_scale);
  }

  Command pure_pursuit_on_map_path(const std::vector<Point2D> & path, double lookahead) const
  {
    if (path.empty()) {
      return Command{};
    }
    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < path.size(); ++index) {
      const double distance = std::hypot(path[index].x - x_, path[index].y - y_);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = index;
      }
    }

    std::optional<Point2D> selected;
    std::optional<Point2D> last_forward;
    for (std::size_t index = nearest; index < path.size(); ++index) {
      const double dx = path[index].x - x_;
      const double dy = path[index].y - y_;
      const double local_x = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
      const double local_y = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
      if (local_x <= 0.0) {
        continue;
      }
      last_forward = Point2D{local_x, local_y};
      if (std::hypot(local_x, local_y) >= lookahead) {
        selected = last_forward;
        break;
      }
    }
    if (!selected.has_value()) {
      selected = last_forward;
    }
    if (!selected.has_value()) {
      return Command{};
    }
    const double distance_squared = selected->x * selected->x + selected->y * selected->y;
    if (distance_squared < 1e-6) {
      return Command{};
    }
    return Command{
      std::atan2(2.0 * wheelbase_ * selected->y, distance_squared),
      0.0,
      true};
  }

  std::size_t nearest_parking_index() const
  {
    if (parking_path_.empty()) {
      return 0;
    }
    const std::size_t start = parking_index_ > 3U ? parking_index_ - 3U : 0U;
    const std::size_t end = std::min(
      parking_path_.size(), parking_index_ + static_cast<std::size_t>(std::max(1, parking_search_window_)));
    std::size_t best = std::min(parking_index_, parking_path_.size() - 1U);
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = start; index < end; ++index) {
      const double distance = std::hypot(parking_path_[index].x - x_, parking_path_[index].y - y_);
      if (distance < best_distance) {
        best_distance = distance;
        best = index;
      }
    }
    return std::max(parking_index_, best);
  }

  std::size_t parking_target_index(std::size_t start) const
  {
    double travelled = 0.0;
    std::size_t index = start;
    Pose2D previous = parking_path_[start];
    while (index + 1U < parking_path_.size()) {
      ++index;
      travelled += std::hypot(
        parking_path_[index].x - previous.x,
        parking_path_[index].y - previous.y);
      previous = parking_path_[index];
      if (travelled >= parking_lookahead_) {
        break;
      }
    }
    return index;
  }

  bool parking_goal_reached() const
  {
    if (parking_path_.empty()) {
      return false;
    }
    const Pose2D & goal = parking_path_.back();
    const double distance = std::hypot(goal.x - x_, goal.y - y_);
    const double yaw_error = std::abs(parking_cpp::normalize_angle(goal.yaw - yaw_));
    const std::size_t end_margin = static_cast<std::size_t>(std::max(0, parking_index_end_margin_));
    const std::size_t minimum_index =
      parking_path_.size() > end_margin + 1U ? parking_path_.size() - 1U - end_margin : 0U;
    return parking_index_ >= minimum_index &&
           distance <= parking_goal_tolerance_m_ &&
           yaw_error <= parking_goal_yaw_tolerance_;
  }

  Command parking_command()
  {
    if (parking_path_.empty() || parking_path_.size() != parking_directions_.size()) {
      publish_parking_status("ERROR");
      return Command{0.0, 0.0, true};
    }

    parking_index_ = nearest_parking_index();
    if (parking_goal_reached()) {
      if (!parking_done_published_) {
        publish_parking_status("DONE");
        parking_done_published_ = true;
      }
      return Command{0.0, 0.0, true};
    }

    const std::size_t target_index = parking_target_index(parking_index_);
    const int desired_gear = parking_directions_[target_index];
    if (desired_gear != current_gear_) {
      if (!pending_gear_.has_value() || *pending_gear_ != desired_gear) {
        pending_gear_ = desired_gear;
        gear_change_started_ = std::chrono::steady_clock::now();
        publish_parking_status("GEAR_CHANGE");
        return Command{0.0, 0.0, true};
      }
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - gear_change_started_).count();
      if (std::abs(velocity_) > gear_change_stop_speed_ || elapsed < gear_change_pause_s_) {
        return Command{0.0, 0.0, true};
      }
      current_gear_ = desired_gear;
      pending_gear_.reset();
      publish_parking_status("TRACKING");
    }

    auto compute_local = [&](std::size_t index) {
        const Pose2D & target = parking_path_[index];
        const double motion_yaw = current_gear_ > 0 ? yaw_ : parking_cpp::normalize_angle(yaw_ + parking_cpp::kPi);
        const double dx = target.x - x_;
        const double dy = target.y - y_;
        return Point2D{
          std::cos(motion_yaw) * dx + std::sin(motion_yaw) * dy,
          -std::sin(motion_yaw) * dx + std::cos(motion_yaw) * dy};
      };

    Point2D local = compute_local(target_index);
    double distance_squared = local.x * local.x + local.y * local.y;
    if (local.x <= 0.0 || distance_squared < 1e-6) {
      local = compute_local(parking_index_);
      distance_squared = local.x * local.x + local.y * local.y;
      if (local.x <= 0.0 || distance_squared < 1e-6) {
        return Command{0.0, 0.0, true};
      }
    }

    const double steering_motion = std::atan2(2.0 * wheelbase_ * local.y, distance_squared);
    const double steering = current_gear_ > 0 ? steering_motion : -steering_motion;
    const double speed = current_gear_ > 0 ? parking_forward_speed_ : -parking_reverse_speed_;
    return Command{steering, speed, true};
  }

  void control_step()
  {
    const double current_time = now_s();
    double dt = current_time - last_control_time_s_;
    if (dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / std::max(1.0, control_hz_);
    }
    last_control_time_s_ = current_time;

    const bool odom_ok = current_time - odom_time_s_ < input_timeout_s_;
    const bool lane_ok = current_time - lane_time_s_ < input_timeout_s_ && lane_valid_;
    double target_steering = commanded_steering_;
    double target_speed = 0.0;

    if (!odom_ok) {
      target_steering = 0.0;
      target_speed = 0.0;
    } else if (mode_ == "TURN_BRIDGE") {
      const Command command = pure_pursuit_on_map_path(bridge_path_, map_lookahead_);
      target_steering = command.steering;
      target_speed = command.valid ? bridge_speed_ : 0.0;
    } else if (mode_ == "APPROACH" || mode_ == "PARKING_APPROACH") {
      if (lane_ok) {
        target_steering = lane_follow_command(0.0);
        target_speed = std::min(approach_speed_, speed_for_curve(target_steering));
      } else {
        target_speed = 0.0;
      }
    } else if (
      mode_ == "STOP_AT_LIGHT" || mode_ == "PARKING_PLAN" ||
      mode_ == "PARKED" || mode_ == "PARKING_FAILED")
    {
      target_steering = mode_ == "STOP_AT_LIGHT" && lane_ok ? lane_follow_command(0.0) : 0.0;
      target_speed = 0.0;
    } else if (mode_ == "PARKING") {
      const Command command = parking_command();
      target_steering = command.steering;
      target_speed = command.speed;
      const auto [blocked, reason] = parking_scan_blocked(current_gear_);
      if (blocked) {
        target_steering = 0.0;
        target_speed = 0.0;
        if (reason != scan_stop_reason_) {
          scan_stop_reason_ = reason;
          publish_parking_status("OBSTACLE_STOP");
          RCLCPP_WARN(get_logger(), "Parking stopped by /scan: %s", reason.c_str());
        }
      } else if (!scan_stop_reason_.empty()) {
        scan_stop_reason_.clear();
        publish_parking_status("TRACKING");
        RCLCPP_INFO(get_logger(), "Parking scan safety released.");
      }
    } else if (mode_ == "AVOID") {
      if (lane_ok) {
        target_steering = lane_follow_command(avoid_offset_bias_);
        target_speed = 0.6 * speed_for_curve(target_steering);
      }
    } else if (mode_ == "WAYPOINT_FOLLOW") {
      const Command command = pure_pursuit_on_map_path(waypoint_path_, map_lookahead_);
      target_steering = command.steering;
      target_speed = command.valid ? waypoint_speed_ : 0.0;
    } else {
      if (lane_ok) {
        target_steering = lane_follow_command(0.0);
        target_speed = speed_for_curve(target_steering);
      }
    }

    target_steering = parking_cpp::clamp(target_steering, -max_steer_, max_steer_);
    target_speed = parking_cpp::clamp(target_speed, -max_velocity_, max_velocity_);
    commanded_steering_ = parking_cpp::rate_limit(
      commanded_steering_, target_steering, steering_rate_ * dt);
    commanded_speed_ = parking_cpp::ramp(
      commanded_speed_, target_speed,
      acceleration_limit_ * dt, deceleration_limit_ * dt);
    if (mode_ == "PARKING" && !scan_stop_reason_.empty()) {
      commanded_speed_ = 0.0;
    }

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = now();
    command.header.frame_id = base_frame_id_;
    command.drive.steering_angle = commanded_steering_;
    command.drive.speed = commanded_speed_;
    command_pub_->publish(command);

    std_msgs::msg::Float64 velocity;
    velocity.data = commanded_speed_;
    velocity_pub_->publish(velocity);
    std_msgs::msg::Float64 steering;
    steering.data = commanded_steering_;
    steering_pub_->publish(steering);
  }

  double wheelbase_{1.005};
  double control_hz_{40.0};
  double max_steer_{0.5235988};
  double steering_rate_{2.0};
  double max_velocity_{4.44};
  double cruise_speed_{3.75};
  double approach_speed_{0.50};
  double bridge_speed_{2.50};
  double waypoint_speed_{2.50};
  double minimum_speed_{1.25};
  double acceleration_limit_{2.0};
  double deceleration_limit_{4.0};
  double curve_slow_k_{1.2};
  double lane_curve_speed_reduction_{0.40};
  double lane_lookahead_straight_{1.0};
  double lane_lookahead_curve_{0.60};
  double lane_straight_radius_px_{1400.0};
  double lane_sharp_curve_radius_px_{280.0};
  double lane_min_forward_{0.10};
  int lane_path_min_points_{3};
  double lane_target_filter_alpha_{0.35};
  double stanley_k_{0.8};
  double stanley_v_min_{0.5};
  double lane_offset_sign_{-1.0};
  double lane_heading_sign_{1.0};
  double lane_filter_alpha_{0.3};
  double avoid_offset_bias_{0.45};
  double map_lookahead_{1.0};

  double parking_forward_speed_{0.25};
  double parking_reverse_speed_{0.20};
  double parking_lookahead_{0.45};
  int parking_search_window_{35};
  double gear_change_pause_s_{0.60};
  double gear_change_stop_speed_{0.03};
  double parking_goal_tolerance_m_{0.20};
  double parking_goal_yaw_tolerance_{8.0 * parking_cpp::kPi / 180.0};
  int parking_index_end_margin_{2};

  std::string scan_topic_;
  bool require_scan_for_parking_{true};
  double scan_timeout_s_{0.50};
  bool require_forward_scan_coverage_{true};
  bool require_reverse_scan_coverage_{true};
  std::string base_frame_id_{"base_link"};
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
  double vehicle_front_m_{0.67};
  double vehicle_rear_m_{0.67};
  double parking_obstacle_clearance_m_{0.25};
  double parking_corridor_min_y_{-0.505};
  double parking_corridor_max_y_{0.505};
  bool use_laser_tf_{true};
  bool require_laser_tf_{true};
  double tf_timeout_s_{0.05};
  double manual_laser_x_{0.0};
  double manual_laser_y_{0.0};
  double manual_laser_yaw_{0.0};
  double input_timeout_s_{0.30};

  std::string mode_{"LANE_FOLLOW"};
  double lane_offset_{0.0};
  double lane_heading_{0.0};
  double lane_radius_px_{1e6};
  bool lane_valid_{false};
  bool lane_initialized_{false};
  std::vector<Point2D> lane_path_local_;
  std::optional<Point2D> filtered_lane_target_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double velocity_{0.0};
  std::vector<Point2D> bridge_path_;
  std::vector<Point2D> waypoint_path_;
  std::vector<Pose2D> parking_path_;
  std::vector<int> parking_directions_;
  std::size_t parking_index_{0};
  int current_gear_{1};
  std::optional<int> pending_gear_;
  std::chrono::steady_clock::time_point gear_change_started_{};
  bool parking_done_published_{false};
  double commanded_steering_{0.0};
  double commanded_speed_{0.0};
  double last_control_time_s_{0.0};
  double odom_time_s_{-1e9};
  double lane_time_s_{-1e9};
  double scan_time_s_{-1e9};
  double scan_min_forward_m_{std::numeric_limits<double>::infinity()};
  double scan_min_reverse_m_{std::numeric_limits<double>::infinity()};
  bool scan_has_forward_coverage_{false};
  bool scan_has_reverse_coverage_{false};
  std::string scan_stop_reason_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lane_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr bridge_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr parking_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr parking_directions_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parking_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerWithParking>());
  rclcpp::shutdown();
  return 0;
}
