#include "hyper_waypoint_tracker/common.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
struct Point2D {double x{0.0}; double y{0.0};};
struct Command {double steering{0.0}; bool valid{false};};
struct LatLon {double lat{0.0}; double lon{0.0};};
}  // namespace

class WaypointTracker : public rclcpp::Node
{
public:
  WaypointTracker() : Node("waypoint_tracker")
  {
    waypoints_yaml_ = declare_parameter<std::string>("waypoints_yaml", "");
    wheelbase_ = declare_parameter<double>("wheelbase", 1.005);
    control_hz_ = declare_parameter<double>("control_hz", 40.0);
    lookahead_ = declare_parameter<double>("lookahead", 1.0);
    cruise_speed_ = declare_parameter<double>("cruise_speed", 2.0);
    max_steer_ = declare_parameter<double>("max_steering_angle", 0.5235988);
    steering_rate_ = declare_parameter<double>("max_steering_angular_velocity", 2.0);
    max_velocity_ = declare_parameter<double>("max_velocity", 4.44);
    acceleration_limit_ = declare_parameter<double>("accel_limit", 2.0);
    deceleration_limit_ = declare_parameter<double>("decel_limit", 4.0);
    goal_radius_m_ = declare_parameter<double>("goal_radius_m", 0.5);
    calibration_window_s_ = declare_parameter<double>("calibration_window_s", 2.0);
    input_timeout_s_ = declare_parameter<double>("input_timeout", 0.30);

    if (!load_waypoints(waypoints_yaml_)) {
      RCLCPP_ERROR(get_logger(),
        "No usable waypoints loaded from '%s' -- halting immediately.", waypoints_yaml_.c_str());
      state_ = State::kDone;
    }

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&WaypointTracker::on_gps, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map", 10, std::bind(&WaypointTracker::on_odom, this, std::placeholders::_1));

    velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/velocity", 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/waypoint_tracker/path", rclcpp::QoS(1).transient_local());

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_hz_));
    timer_ = create_wall_timer(period, std::bind(&WaypointTracker::control_step, this));
  }

private:
  enum class State {kCalibrating, kTracking, kDone};

  bool load_waypoints(const std::string & path)
  {
    if (path.empty()) {return false;}
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load waypoints_yaml '%s': %s", path.c_str(), e.what());
      return false;
    }
    if (!root["waypoints"] || !root["waypoints"].IsSequence()) {return false;}
    for (const auto & entry : root["waypoints"]) {
      if (!entry["latitude"] || !entry["longitude"]) {continue;}
      raw_waypoints_.push_back({entry["latitude"].as<double>(), entry["longitude"].as<double>()});
    }
    if (raw_waypoints_.empty()) {return false;}

    origin_lat_ = raw_waypoints_.front().lat;
    origin_lon_ = raw_waypoints_.front().lon;
    raw_enu_waypoints_.reserve(raw_waypoints_.size());
    for (const auto & ll : raw_waypoints_) {
      const auto enu = hyper_waypoint_tracker::equirect_local_xy(origin_lat_, origin_lon_, ll.lat, ll.lon);
      raw_enu_waypoints_.push_back({enu.x, enu.y});
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from '%s'.", raw_waypoints_.size(), path.c_str());
    return true;
  }

  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    gps_lat_ = msg->latitude;
    gps_lon_ = msg->longitude;
    gps_time_s_ = now_s();
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = hyper_waypoint_tracker::yaw_from_quaternion(msg->pose.pose.orientation);
    odom_time_s_ = now_s();
  }

  double now_s() const {return this->now().seconds();}

  // A path can loop back spatially close to an earlier segment even though those points are far
  // apart in arc-length. A global nearest-point search would then jump tracking backward right as
  // the vehicle nears the goal. Restricting the search to a forward-looking window from the last
  // known index keeps progress monotonic along the path instead of by raw distance -- mirrors
  // hyper_planner's controller_with_parking_node.cpp pure_pursuit_windowed().
  static constexpr std::size_t kPursuitSearchWindow = 60U;

  std::size_t windowed_nearest_index(const std::vector<Point2D> & path, std::size_t last_index) const
  {
    if (path.empty()) {return 0U;}
    if (last_index >= path.size()) {last_index = path.size() - 1U;}
    const std::size_t window_end = std::min(path.size(), last_index + kPursuitSearchWindow);
    std::size_t nearest = last_index;
    double nearest_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = last_index; i < window_end; ++i) {
      const double dx = path[i].x - x_; const double dy = path[i].y - y_;
      const double d2 = dx * dx + dy * dy;
      if (d2 < nearest_d2) {nearest_d2 = d2; nearest = i;}
    }
    return nearest;
  }

  Command pure_pursuit_windowed(
    const std::vector<Point2D> & path, double lookahead, std::size_t & last_index) const
  {
    if (path.empty()) {return {};}
    const std::size_t nearest = windowed_nearest_index(path, last_index);
    last_index = nearest;
    std::size_t target = nearest;
    while (target + 1U < path.size() &&
      std::hypot(path[target].x - x_, path[target].y - y_) < lookahead) {++target;}
    const double dx = path[target].x - x_; const double dy = path[target].y - y_;
    const double lx = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    const double ly = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
    const double ld2 = lx * lx + ly * ly;
    if (lx <= 0.0 || ld2 < 1e-6) {return {};}
    return Command{std::atan2(2.0 * wheelbase_ * ly, ld2), true};
  }

  void finish_calibration()
  {
    const double offset_x = calib_offset_sum_x_ / static_cast<double>(calib_sample_count_);
    const double offset_y = calib_offset_sum_y_ / static_cast<double>(calib_sample_count_);
    path_.clear();
    path_.reserve(raw_enu_waypoints_.size());
    for (const auto & p : raw_enu_waypoints_) {
      path_.push_back({p.x + offset_x, p.y + offset_y});
    }
    publish_path();
    state_ = State::kTracking;
    RCLCPP_INFO(get_logger(),
      "Calibration complete (%zu samples): offset=(%.2f, %.2f) m. Tracking %zu waypoints.",
      calib_sample_count_, offset_x, offset_y, path_.size());
  }

  void publish_path() const
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.poses.reserve(path_.size());
    for (const auto & p : path_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    path_pub_->publish(msg);
  }

  void control_step()
  {
    const double t = now_s();
    double dt = t - last_control_time_s_;
    if (dt <= 0.0 || dt > 1.0) {dt = 1.0 / std::max(1.0, control_hz_);}
    last_control_time_s_ = t;
    const bool odom_ok = t - odom_time_s_ < input_timeout_s_;
    const bool gps_ok = t - gps_time_s_ < input_timeout_s_;

    double target_steering = 0.0;
    double target_speed = 0.0;

    if (state_ == State::kCalibrating) {
      if (odom_ok && gps_ok) {
        const auto enu = hyper_waypoint_tracker::equirect_local_xy(
          origin_lat_, origin_lon_, gps_lat_, gps_lon_);
        calib_offset_sum_x_ += (x_ - enu.x);
        calib_offset_sum_y_ += (y_ - enu.y);
        ++calib_sample_count_;
        if (calib_start_time_s_ < 0.0) {calib_start_time_s_ = t;}
        if (t - calib_start_time_s_ >= calibration_window_s_ && calib_sample_count_ > 0U) {
          finish_calibration();
        }
      }
    } else if (state_ == State::kTracking) {
      if (!odom_ok || path_.empty()) {
        target_steering = 0.0; target_speed = 0.0;
      } else {
        const Command c = pure_pursuit_windowed(path_, lookahead_, last_index_);
        const bool at_last_index = (last_index_ + 1U >= path_.size());
        const double dist_to_goal = std::hypot(path_.back().x - x_, path_.back().y - y_);
        if (at_last_index && (dist_to_goal <= goal_radius_m_ || !c.valid)) {
          state_ = State::kDone;
          RCLCPP_INFO(get_logger(),
            "Waypoint tracking complete (%.2f m from final waypoint) -- halted.", dist_to_goal);
          target_steering = 0.0; target_speed = 0.0;
        } else {
          target_steering = c.valid ? c.steering : 0.0;
          target_speed = c.valid ? cruise_speed_ : 0.0;
        }
      }
    }
    // State::kDone: target_steering/target_speed stay at 0.

    target_steering = hyper_waypoint_tracker::clamp(target_steering, -max_steer_, max_steer_);
    target_speed = hyper_waypoint_tracker::clamp(target_speed, -max_velocity_, max_velocity_);
    commanded_steering_ = hyper_waypoint_tracker::rate_limit(
      commanded_steering_, target_steering, steering_rate_ * dt);
    commanded_speed_ = hyper_waypoint_tracker::ramp(
      commanded_speed_, target_speed, acceleration_limit_ * dt, deceleration_limit_ * dt);

    std_msgs::msg::Float64 velocity; velocity.data = commanded_speed_; velocity_pub_->publish(velocity);
    std_msgs::msg::Float64 steering; steering.data = commanded_steering_; steering_pub_->publish(steering);
  }

  // Params
  std::string waypoints_yaml_;
  double wheelbase_{1.005}, control_hz_{40.0}, lookahead_{1.0}, cruise_speed_{2.0};
  double max_steer_{0.5235988}, steering_rate_{2.0}, max_velocity_{4.44};
  double acceleration_limit_{2.0}, deceleration_limit_{4.0};
  double goal_radius_m_{0.5}, calibration_window_s_{2.0}, input_timeout_s_{0.3};

  // Loaded course, in lat/lon and in ENU meters relative to the first waypoint.
  std::vector<LatLon> raw_waypoints_;
  std::vector<Point2D> raw_enu_waypoints_;
  double origin_lat_{0.0}, origin_lon_{0.0};

  // Calibration accumulator: live (gps, odom) sample pairs -> average translation offset from the
  // waypoints' ENU frame into this session's live map frame. Translation-only: assumes map/odom
  // axes are ENU-aligned, which is navsat_transform_node's standard behavior given this stack's
  // magnetic_declination_radians/yaw_offset config (see dual_ekf_navsat.yaml).
  double calib_offset_sum_x_{0.0}, calib_offset_sum_y_{0.0};
  std::size_t calib_sample_count_{0U};
  double calib_start_time_s_{-1.0};

  // Calibrated path (map frame) and pursuit progress.
  std::vector<Point2D> path_;
  std::size_t last_index_{0U};

  State state_{State::kCalibrating};

  // Live inputs
  double gps_lat_{0.0}, gps_lon_{0.0}, gps_time_s_{-1e9};
  double x_{0.0}, y_{0.0}, yaw_{0.0}, odom_time_s_{-1e9};

  double last_control_time_s_{0.0};
  double commanded_steering_{0.0}, commanded_speed_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointTracker>());
  rclcpp::shutdown();
  return 0;
}
