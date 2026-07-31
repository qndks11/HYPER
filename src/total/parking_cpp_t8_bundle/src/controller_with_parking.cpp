#include "parking_cpp/common.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
struct Point2D {double x{0.0}; double y{0.0};};
struct Command {double steering{0.0}; double speed{0.0}; bool valid{false};};

bool has_suffix(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
    value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}

class ControllerWithParking : public rclcpp::Node
{
public:
  ControllerWithParking() : Node("controller")
  {
    wheelbase_ = declare_parameter<double>("wheelbase", 1.005);
    control_hz_ = declare_parameter<double>("control_hz", 40.0);
    max_steer_ = declare_parameter<double>("max_steering_angle", 0.5235988);
    steering_rate_ = declare_parameter<double>("max_steering_angular_velocity", 2.0);
    max_velocity_ = declare_parameter<double>("max_velocity", 4.44);
    cruise_speed_ = declare_parameter<double>("cruise_speed", 3.75);
    approach_speed_ = declare_parameter<double>("approach_speed", 0.50);
    hill_approach_speed_ = declare_parameter<double>("hill_approach_speed", 0.40);
    accel_zone_speed_ = declare_parameter<double>("accel_zone_speed", 4.0);
    emergency_deceleration_limit_ = declare_parameter<double>("emergency_decel_limit", 8.0);
    bridge_speed_ = declare_parameter<double>("bridge_speed", 2.50);
    waypoint_speed_ = declare_parameter<double>("waypoint_speed", 2.50);
    minimum_speed_ = declare_parameter<double>("min_speed", 1.25);
    acceleration_limit_ = declare_parameter<double>("accel_limit", 2.0);
    deceleration_limit_ = declare_parameter<double>("decel_limit", 4.0);
    curve_slow_k_ = declare_parameter<double>("curve_slow_k", 1.2);
    lane_curve_speed_reduction_ = declare_parameter<double>("lane_curve_speed_reduction", 0.40);
    stanley_k_ = declare_parameter<double>("stanley_k", 0.8);
    stanley_v_min_ = declare_parameter<double>("stanley_v_min", 0.5);
    lane_offset_sign_ = declare_parameter<double>("lane_offset_sign", -1.0);
    lane_heading_sign_ = declare_parameter<double>("lane_heading_sign", 1.0);
    lane_filter_alpha_ = declare_parameter<double>("lane_filter_alpha", 0.3);
    avoid_offset_bias_ = declare_parameter<double>("avoid_offset_bias", 0.45);
    map_lookahead_ = declare_parameter<double>("lookahead", 1.0);
    input_timeout_s_ = declare_parameter<double>("input_timeout", 0.30);
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "body_link");

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/driving_mode", 10, std::bind(&ControllerWithParking::on_mode, this, std::placeholders::_1));
    lane_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/lane/center", 10, std::bind(&ControllerWithParking::on_lane, this, std::placeholders::_1));
    bridge_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/bridge_path", 10, std::bind(&ControllerWithParking::on_bridge, this, std::placeholders::_1));
    waypoint_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/waypoint_path", 10,
      std::bind(&ControllerWithParking::on_waypoint_path, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map", 10,
      std::bind(&ControllerWithParking::on_odom, this, std::placeholders::_1));

    command_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
    velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/velocity", 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ControllerWithParking::control_step, this));
    last_control_time_s_ = now_s();
    RCLCPP_INFO(get_logger(), "Controller started (parking removed, hill stop enabled).");
  }

private:
  double now_s() const {return now().seconds();}

  void on_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    mode_ = msg->data;
  }

  void on_lane(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    lane_time_s_ = now_s();
    if (msg->data.size() < 6U) {
      left_valid_ = false; right_valid_ = false; return;
    }
    const double raw_left_angle = msg->data[0] * parking_cpp::kPi / 180.0;
    const double raw_left_offset = msg->data[1];
    left_valid_ = msg->data[2] > 0.5;
    const double raw_right_angle = msg->data[3] * parking_cpp::kPi / 180.0;
    const double raw_right_offset = msg->data[4];
    right_valid_ = msg->data[5] > 0.5;

    const double alpha = parking_cpp::clamp(lane_filter_alpha_, 0.0, 1.0);
    if (!left_valid_) {left_initialized_ = false;}
    else if (!left_initialized_) {left_angle_ = raw_left_angle; left_offset_ = raw_left_offset; left_initialized_ = true;}
    else {
      left_angle_ = alpha * raw_left_angle + (1.0 - alpha) * left_angle_;
      left_offset_ = alpha * raw_left_offset + (1.0 - alpha) * left_offset_;
    }

    if (!right_valid_) {right_initialized_ = false;}
    else if (!right_initialized_) {right_angle_ = raw_right_angle; right_offset_ = raw_right_offset; right_initialized_ = true;}
    else {
      right_angle_ = alpha * raw_right_angle + (1.0 - alpha) * right_angle_;
      right_offset_ = alpha * raw_right_offset + (1.0 - alpha) * right_offset_;
    }
  }

  void on_bridge(const nav_msgs::msg::Path::SharedPtr msg)
  {
    bridge_path_.clear();
    for (const auto & pose : msg->poses) {bridge_path_.push_back({pose.pose.position.x, pose.pose.position.y});}
  }

  void on_waypoint_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    waypoint_path_.clear();
    for (const auto & pose : msg->poses) {waypoint_path_.push_back({pose.pose.position.x, pose.pose.position.y});}
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = parking_cpp::yaw_from_quaternion(msg->pose.pose.orientation);
    velocity_ = msg->twist.twist.linear.x;
    odom_time_s_ = now_s();
  }

  double lane_follow_command(double offset_bias, bool use_right)
  {
    const double angle = use_right ? right_angle_ : left_angle_;
    const double offset = use_right ? right_offset_ : left_offset_;
    const double heading = lane_heading_sign_ * angle;
    const double effective_offset = lane_offset_sign_ * offset + offset_bias;
    return heading +
      std::atan2(stanley_k_ * effective_offset, std::max(stanley_v_min_, std::abs(velocity_)));
  }

  double speed_for_curve(double steering) const
  {
    const double normalized = parking_cpp::clamp(std::abs(steering) / std::max(1e-6, max_steer_), 0.0, 1.0);
    const double reduced = cruise_speed_ * (1.0 - lane_curve_speed_reduction_ *
      std::pow(normalized, curve_slow_k_));
    return parking_cpp::clamp(reduced, minimum_speed_, cruise_speed_);
  }

  // `reverse` targets a point behind the vehicle instead of ahead of it, for driving a path
  // backward (e.g. a back-up-then-arc parking maneuver). The steering law is unchanged: the
  // circular arc tangent to the vehicle's heading through the target point is the same arc
  // whether it's driven forward or backward, only the direction of travel along it (and hence
  // the sign of speed the caller applies) differs.
  Command pure_pursuit_on_map_path(const std::vector<Point2D> & path, double lookahead, bool reverse = false) const
  {
    if (path.empty()) {return {};}
    std::size_t nearest = 0;
    double nearest_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double dx = path[i].x - x_; const double dy = path[i].y - y_;
      const double d2 = dx * dx + dy * dy;
      if (d2 < nearest_d2) {nearest_d2 = d2; nearest = i;}
    }
    std::size_t target = nearest;
    while (target + 1U < path.size() &&
      std::hypot(path[target].x - x_, path[target].y - y_) < lookahead) {++target;}
    const double dx = path[target].x - x_; const double dy = path[target].y - y_;
    const double lx = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    const double ly = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
    const double ld2 = lx * lx + ly * ly;
    if (ld2 < 1e-6) {return {};}
    if (reverse) {if (lx >= 0.0) {return {};}} else {if (lx <= 0.0) {return {};}}
    return Command{std::atan2(2.0 * wheelbase_ * ly, ld2), 0.0, true};
  }

  enum class Phase
  {
    kLeftLaneFollow, kRightLaneFollow, kApproach, kHillApproach, kStopAtLight, kHillStop,
    kTurnBridge, kAccelZone, kObstacleStop, kOther
  };

  // Maps the supervisor's slot-scoped /driving_mode strings (e.g. INTERSECTION_A_APPROACH,
  // INTERSECTION_B_APPROACH, ...) down to the same generic phases this controller has always
  // branched on, so control_step doesn't need a near-duplicate branch per intersection. Exact
  // matches are checked before the generic "_APPROACH" suffix so HILL_APPROACH doesn't fall
  // through to the intersection-approach speed profile.
  Phase classify_mode(const std::string & mode) const
  {
    if (mode == "LEFT_LANE_FOLLOW") {return Phase::kLeftLaneFollow;}
    if (mode == "RIGHT_LANE_FOLLOW") {return Phase::kRightLaneFollow;}
    if (mode == "HILL_APPROACH") {return Phase::kHillApproach;}
    if (mode == "HILL_STOP") {return Phase::kHillStop;}
    if (mode == "ACCEL_OBSTACLE_ZONE") {return Phase::kAccelZone;}
    if (mode == "OBSTACLE_STOP") {return Phase::kObstacleStop;}
    if (has_suffix(mode, "_APPROACH")) {return Phase::kApproach;}
    if (has_suffix(mode, "_STOP_AT_LIGHT")) {return Phase::kStopAtLight;}
    if (has_suffix(mode, "_TURN_BRIDGE")) {return Phase::kTurnBridge;}
    return Phase::kOther;
  }

  void control_step()
  {
    const double t = now_s();
    double dt = t - last_control_time_s_;
    if (dt <= 0.0 || dt > 1.0) {dt = 1.0 / std::max(1.0, control_hz_);}
    last_control_time_s_ = t;
    const bool odom_ok = t - odom_time_s_ < input_timeout_s_;

    const Phase phase = classify_mode(mode_);
    // Cruise phases set which side to track; every other phase keeps steering with whichever
    // side was active immediately before entering it, since the supervisor doesn't (and doesn't
    // need to) publish a left/right variant of every approach/stop/turn-bridge phase.
    if (phase == Phase::kLeftLaneFollow) {use_right_lane_ = false;}
    else if (phase == Phase::kRightLaneFollow) {use_right_lane_ = true;}
    const bool lane_ok = t - lane_time_s_ < input_timeout_s_ &&
      (use_right_lane_ ? right_valid_ : left_valid_);

    double target_steering = commanded_steering_;
    double target_speed = 0.0;

    if (!odom_ok) {target_steering = 0.0; target_speed = 0.0;}
    else if (phase == Phase::kTurnBridge) {
      const bool reverse_bridge = mode_.find("REVERSE") != std::string::npos;
      const Command c = pure_pursuit_on_map_path(bridge_path_, map_lookahead_, reverse_bridge);
      target_steering = c.steering;
      target_speed = c.valid ? (reverse_bridge ? -bridge_speed_ : bridge_speed_) : 0.0;
    } else if (phase == Phase::kApproach) {
      if (lane_ok) {
        target_steering = lane_follow_command(0.0, use_right_lane_);
        target_speed = std::min(approach_speed_, speed_for_curve(target_steering));
      }
    } else if (phase == Phase::kHillApproach) {
      if (lane_ok) {
        target_steering = lane_follow_command(0.0, use_right_lane_);
        target_speed = std::min(hill_approach_speed_, speed_for_curve(target_steering));
      }
    } else if (phase == Phase::kAccelZone) {
      if (lane_ok) {
        target_steering = lane_follow_command(0.0, use_right_lane_);
        target_speed = std::min(accel_zone_speed_, speed_for_curve(target_steering));
      }
    } else if (phase == Phase::kObstacleStop) {
      target_steering = lane_ok ? lane_follow_command(0.0, use_right_lane_) : 0.0;
      target_speed = 0.0;
    } else if (phase == Phase::kStopAtLight || phase == Phase::kHillStop) {
      target_steering = lane_ok ? lane_follow_command(0.0, use_right_lane_) : 0.0;
      target_speed = 0.0;
    } else if (mode_ == "AVOID") {
      if (lane_ok) {
        target_steering = lane_follow_command(avoid_offset_bias_, use_right_lane_);
        target_speed = 0.6 * speed_for_curve(target_steering);
      }
    } else if (mode_ == "WAYPOINT_FOLLOW") {
      const Command c = pure_pursuit_on_map_path(waypoint_path_, map_lookahead_);
      target_steering = c.steering; target_speed = c.valid ? waypoint_speed_ : 0.0;
    } else if (lane_ok) {
      target_steering = lane_follow_command(0.0, use_right_lane_);
      target_speed = speed_for_curve(target_steering);
    }

    target_steering = parking_cpp::clamp(target_steering, -max_steer_, max_steer_);
    target_speed = parking_cpp::clamp(target_speed, -max_velocity_, max_velocity_);
    commanded_steering_ = parking_cpp::rate_limit(commanded_steering_, target_steering, steering_rate_ * dt);
    const double active_decel_limit = phase == Phase::kObstacleStop ?
      emergency_deceleration_limit_ : deceleration_limit_;
    commanded_speed_ = parking_cpp::ramp(commanded_speed_, target_speed,
      acceleration_limit_ * dt, active_decel_limit * dt);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now(); cmd.header.frame_id = base_frame_id_;
    cmd.drive.steering_angle = commanded_steering_; cmd.drive.speed = commanded_speed_;
    command_pub_->publish(cmd);
    std_msgs::msg::Float64 velocity; velocity.data = commanded_speed_; velocity_pub_->publish(velocity);
    std_msgs::msg::Float64 steering; steering.data = commanded_steering_; steering_pub_->publish(steering);
  }

  double wheelbase_{1.005}, control_hz_{40.0}, max_steer_{0.5235988}, steering_rate_{2.0};
  double max_velocity_{4.44}, cruise_speed_{3.75}, approach_speed_{0.5}, hill_approach_speed_{0.4};
  double accel_zone_speed_{4.0}, emergency_deceleration_limit_{8.0};
  double bridge_speed_{2.5}, waypoint_speed_{2.5}, minimum_speed_{1.25};
  double acceleration_limit_{2.0}, deceleration_limit_{4.0}, curve_slow_k_{1.2};
  double lane_curve_speed_reduction_{0.4};
  double stanley_k_{0.8}, stanley_v_min_{0.5};
  double lane_offset_sign_{-1.0}, lane_heading_sign_{1.0}, lane_filter_alpha_{0.3};
  double avoid_offset_bias_{0.45}, map_lookahead_{1.0}, input_timeout_s_{0.3};
  std::string base_frame_id_{"body_link"}, mode_{"LEFT_LANE_FOLLOW"};
  bool use_right_lane_{false};
  double left_angle_{0.0}, left_offset_{0.0}, right_angle_{0.0}, right_offset_{0.0};
  bool left_valid_{false}, right_valid_{false}, left_initialized_{false}, right_initialized_{false};
  std::vector<Point2D> bridge_path_, waypoint_path_;
  double x_{0.0}, y_{0.0}, yaw_{0.0}, velocity_{0.0};
  double commanded_steering_{0.0}, commanded_speed_{0.0}, last_control_time_s_{0.0};
  double odom_time_s_{-1e9}, lane_time_s_{-1e9};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lane_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr bridge_sub_, waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_, steering_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerWithParking>());
  rclcpp::shutdown();
  return 0;
}
