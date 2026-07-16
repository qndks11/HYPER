#include "parking_cpp/common.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

constexpr const char * kLaneFollow = "LANE_FOLLOW";
constexpr const char * kApproach = "APPROACH";
constexpr const char * kStopAtLight = "STOP_AT_LIGHT";
constexpr const char * kTurnBridge = "TURN_BRIDGE";
constexpr const char * kParkingApproach = "PARKING_APPROACH";
constexpr const char * kParkingPlan = "PARKING_PLAN";
constexpr const char * kParking = "PARKING";
constexpr const char * kParked = "PARKED";
constexpr const char * kParkingFailed = "PARKING_FAILED";

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct GpsPoint
{
  double latitude{0.0};
  double longitude{0.0};
};

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

std::filesystem::path expand_home(const std::string & value)
{
  if (!value.empty() && value.front() == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      if (value == "~") {
        return std::filesystem::path(home);
      }
      if (value.size() >= 2U && value[1] == '/') {
        return std::filesystem::path(home) / value.substr(2U);
      }
    }
  }
  return std::filesystem::path(value);
}

}  // namespace

class BehaviorSupervisorWithParking : public rclcpp::Node
{
public:
  BehaviorSupervisorWithParking()
  : Node("behavior_supervisor")
  {
    yaml_path_ = expand_home(declare_parameter<std::string>("event_path_yaml", "course.yaml"));
    tick_hz_ = declare_parameter<double>("tick_hz", 10.0);
    gps_timeout_s_ = declare_parameter<double>("gps_timeout", 1.5);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout", 0.5);
    perception_timeout_s_ = declare_parameter<double>("perception_timeout", 0.5);
    sign_timeout_s_ = declare_parameter<double>("sign_timeout", 1.5);
    stop_distance_m_ = declare_parameter<double>("stop_line_trigger_distance_m", 0.50);
    bridge_exit_tolerance_m_ = declare_parameter<double>("bridge_exit_tolerance_m", 0.50);
    minimum_bridge_time_s_ = declare_parameter<double>("min_bridge_time_s", 1.0);
    event_rearm_margin_m_ = declare_parameter<double>("event_rearm_margin_m", 3.0);
    approach_cancel_margin_m_ = declare_parameter<double>("approach_cancel_margin_m", 1.5);
    default_parking_plan_trigger_m_ = declare_parameter<double>("parking_plan_trigger_m", 0.80);
    default_mission_ = lower(declare_parameter<std::string>("default_mission", "straight"));
    mission_ = default_mission_;
    left_on_green_allowed_ = declare_parameter<bool>("left_on_green_allowed", false);
    right_on_green_allowed_ = declare_parameter<bool>("right_on_green_allowed", true);
    arrow_signal_selects_path_ = declare_parameter<bool>("arrow_signal_selects_path", true);

    load_yaml();

    const auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10,
      std::bind(&BehaviorSupervisorWithParking::on_gps, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map", 10,
      std::bind(&BehaviorSupervisorWithParking::on_odom, this, std::placeholders::_1));
    stopline_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/stopline/detection", 10,
      std::bind(&BehaviorSupervisorWithParking::on_stopline, this, std::placeholders::_1));
    sign_sub_ = create_subscription<std_msgs::msg::String>(
      "/perception/sign", 10,
      std::bind(&BehaviorSupervisorWithParking::on_sign, this, std::placeholders::_1));
    mission_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/turn", 10,
      std::bind(&BehaviorSupervisorWithParking::on_mission, this, std::placeholders::_1));
    event_command_sub_ = create_subscription<std_msgs::msg::String>(
      "/event/cmd", 10,
      std::bind(&BehaviorSupervisorWithParking::on_event_command, this, std::placeholders::_1));
    parking_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/parking/status", transient_qos,
      std::bind(&BehaviorSupervisorWithParking::on_parking_status, this, std::placeholders::_1));
    parking_controller_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/parking/controller_status", transient_qos,
      std::bind(
        &BehaviorSupervisorWithParking::on_parking_controller_status,
        this, std::placeholders::_1));

    mode_pub_ = create_publisher<std_msgs::msg::String>("/driving_mode", 10);
    bridge_pub_ = create_publisher<nav_msgs::msg::Path>("/bridge_path", 10);
    active_event_pub_ = create_publisher<std_msgs::msg::String>("/active_event", 10);
    parking_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/parking/goal", transient_qos);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tick_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BehaviorSupervisorWithParking::tick, this));

    RCLCPP_INFO(
      get_logger(), "C++ behavior supervisor started: yaml=%s",
      yaml_path_.string().c_str());
  }

private:
  double now_s() const {return now().seconds();}

  template<typename T>
  T yaml_value(const YAML::Node & node, const std::string & key, const T & fallback) const
  {
    try {
      if (node && node[key]) {
        return node[key].as<T>();
      }
    } catch (const YAML::Exception &) {
    }
    return fallback;
  }

  void load_yaml()
  {
    try {
      const YAML::Node root = YAML::LoadFile(yaml_path_.string());
      events_ = root["events"];
      paths_ = root["paths"];
      if (!events_ || !events_.IsMap()) {
        events_ = YAML::Node(YAML::NodeType::Map);
      }
      if (!paths_ || !paths_.IsMap()) {
        paths_ = YAML::Node(YAML::NodeType::Map);
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu events and %zu paths.", events_.size(), paths_.size());
    } catch (const std::exception & exception) {
      events_ = YAML::Node(YAML::NodeType::Map);
      paths_ = YAML::Node(YAML::NodeType::Map);
      RCLCPP_ERROR(get_logger(), "Failed to load event YAML: %s", exception.what());
    }
  }

  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < 0 || !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
      std::abs(msg->latitude) > 90.0 || std::abs(msg->longitude) > 180.0)
    {
      return;
    }
    current_gps_ = GpsPoint{msg->latitude, msg->longitude};
    gps_time_s_ = now_s();
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = Pose2D{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      parking_cpp::yaw_from_quaternion(msg->pose.pose.orientation)};
    odom_time_s_ = now_s();
  }

  void on_stopline(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2U) {
      return;
    }
    stop_line_detected_ = msg->data[1] >= 0.5;
    stop_line_distance_m_ =
      stop_line_detected_ && std::isfinite(msg->data[0]) && msg->data[0] >= 0.0 ?
      msg->data[0] : std::numeric_limits<double>::infinity();
    stopline_time_s_ = now_s();
  }

  void on_sign(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string value = lower(trim(msg->data));
    if (value != "none" && value != "red" && value != "green" &&
      value != "left_arrow" && value != "right_arrow")
    {
      value = "none";
    }
    sign_ = value;
    sign_time_s_ = now_s();
  }

  void on_mission(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string value = lower(trim(msg->data));
    if (!value.empty()) {
      mission_ = value;
      RCLCPP_INFO(get_logger(), "Next event direction: %s", mission_.c_str());
    }
  }

  void on_parking_status(const std_msgs::msg::String::SharedPtr msg)
  {
    parking_planner_status_ = upper(trim(msg->data));
  }

  void on_parking_controller_status(const std_msgs::msg::String::SharedPtr msg)
  {
    parking_controller_status_ = upper(trim(msg->data));
  }

  static std::string upper(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char character) {return static_cast<char>(std::toupper(character));});
    return value;
  }

  void on_event_command(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string command = trim(msg->data);
    if (command == "reset") {
      reset_to_lane_follow("manual reset");
      return;
    }
    if (command == "reload") {
      load_yaml();
      return;
    }
    if (command == "clear_completed") {
      completed_events_.clear();
      RCLCPP_INFO(get_logger(), "Completed event memory cleared.");
      return;
    }
    if (command == "force_parking_plan") {
      if (!active_event_id_.has_value() || event_type(*active_event_id_) != "parking") {
        RCLCPP_WARN(get_logger(), "No active parking event.");
        return;
      }
      enter_parking_plan();
      return;
    }

    const auto separator = command.find(':');
    if (separator != std::string::npos && command.substr(0, separator) == "force_event") {
      const std::string event_id = command.substr(separator + 1U);
      if (!events_[event_id]) {
        RCLCPP_WARN(get_logger(), "Unknown event: %s", event_id.c_str());
        return;
      }
      activate_event(event_id, "manual force_event");
    }
  }

  bool gps_fresh(double time) const
  {
    return current_gps_.has_value() && time - gps_time_s_ <= gps_timeout_s_;
  }

  bool odom_fresh(double time) const
  {
    return current_pose_.has_value() && time - odom_time_s_ <= odom_timeout_s_;
  }

  bool stop_line_fresh(double time) const
  {
    return stop_line_detected_ && time - stopline_time_s_ <= perception_timeout_s_ &&
           std::isfinite(stop_line_distance_m_);
  }

  std::string current_sign(double time) const
  {
    return time - sign_time_s_ <= sign_timeout_s_ ? sign_ : "none";
  }

  std::string event_type(const std::string & event_id) const
  {
    const YAML::Node event = events_[event_id];
    return lower(yaml_value<std::string>(event, "event_type", "intersection")) == "parking" ?
      "parking" : "intersection";
  }

  void set_state(const std::string & state, const std::string & reason = "")
  {
    if (state == state_) {
      return;
    }
    const std::string previous = state_;
    state_ = state;
    RCLCPP_INFO(
      get_logger(), "STATE: %s -> %s%s%s",
      previous.c_str(), state_.c_str(), reason.empty() ? "" : " | ", reason.c_str());
  }

  double distance_to_event(const std::string & event_id) const
  {
    if (!current_gps_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    try {
      const YAML::Node event = events_[event_id];
      const double latitude = event["latitude"].as<double>();
      const double longitude = event["longitude"].as<double>();
      return parking_cpp::haversine_m(
        current_gps_->latitude, current_gps_->longitude, latitude, longitude);
    } catch (const YAML::Exception &) {
      return std::numeric_limits<double>::infinity();
    }
  }

  std::optional<std::string> find_event_in_range()
  {
    std::optional<std::string> best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (auto iterator = events_.begin(); iterator != events_.end(); ++iterator) {
      const std::string event_id = iterator->first.as<std::string>();
      const YAML::Node event = iterator->second;
      const double radius = yaml_value<double>(event, "approach_radius_m", 2.5);
      const double distance = distance_to_event(event_id);

      if (completed_events_.count(event_id) > 0U) {
        if (distance > radius + event_rearm_margin_m_) {
          completed_events_.erase(event_id);
        } else {
          continue;
        }
      }
      if (distance <= radius && distance < best_distance) {
        best = event_id;
        best_distance = distance;
      }
    }
    return best;
  }

  void activate_event(const std::string & event_id, const std::string & reason)
  {
    active_event_id_ = event_id;
    if (event_type(event_id) == "parking") {
      parking_goal_sent_ = false;
      parking_planner_status_ = "IDLE";
      parking_controller_status_ = "IDLE";
      set_state(kParkingApproach, reason);
    } else {
      set_state(kApproach, reason);
    }
  }

  std::string selected_direction(double time) const
  {
    const std::string signal = current_sign(time);
    if (arrow_signal_selects_path_) {
      if (signal == "left_arrow") {
        return "left";
      }
      if (signal == "right_arrow") {
        return "right";
      }
      if (signal == "green") {
        return "straight";
      }
    }
    return mission_;
  }

  bool signal_allows_entry(double time) const
  {
    if (!active_event_id_.has_value()) {
      return false;
    }
    const YAML::Node event = events_[*active_event_id_];
    if (!yaml_value<bool>(event, "signal_required", true)) {
      return true;
    }
    const std::string signal = current_sign(time);
    const std::string direction = selected_direction(time);
    if (direction == "straight") {
      return signal == "green";
    }
    if (direction == "left") {
      return signal == "left_arrow" || (signal == "green" && left_on_green_allowed_);
    }
    if (direction == "right") {
      return signal == "right_arrow" || (signal == "green" && right_on_green_allowed_);
    }
    return false;
  }

  std::optional<nav_msgs::msg::Path> transform_relative_path(const YAML::Node & points)
  {
    if (!current_pose_.has_value() || !points || !points.IsSequence()) {
      return std::nullopt;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    const double c = std::cos(current_pose_->yaw);
    const double s = std::sin(current_pose_->yaw);

    for (const auto & point : points) {
      try {
        const double local_x = point["x"].as<double>();
        const double local_y = point["y"].as<double>();
        const double local_yaw = point["yaw"] ? point["yaw"].as<double>() : 0.0;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = current_pose_->x + c * local_x - s * local_y;
        pose.pose.position.y = current_pose_->y + s * local_x + c * local_y;
        pose.pose.orientation = parking_cpp::quaternion_from_yaw(
          parking_cpp::normalize_angle(current_pose_->yaw + local_yaw));
        path.poses.push_back(pose);
      } catch (const YAML::Exception &) {
      }
    }
    if (path.poses.empty()) {
      return std::nullopt;
    }
    return path;
  }

  void enter_bridge()
  {
    if (!active_event_id_.has_value()) {
      reset_to_lane_follow("bridge requested without event");
      return;
    }
    const YAML::Node event = events_[*active_event_id_];
    const std::string direction = selected_direction(now_s());
    const YAML::Node path_key_node = event["paths"] ? event["paths"][direction] : YAML::Node();
    if (!path_key_node) {
      RCLCPP_ERROR(
        get_logger(), "No path for %s:%s",
        active_event_id_->c_str(), direction.c_str());
      reset_to_lane_follow("missing event path");
      return;
    }
    const std::string path_key = path_key_node.as<std::string>();
    const auto transformed = transform_relative_path(paths_[path_key]);
    if (!transformed.has_value()) {
      reset_to_lane_follow("empty event path");
      return;
    }
    active_path_key_ = path_key;
    transformed_path_ = *transformed;
    const auto & final_pose = transformed_path_->poses.back().pose.position;
    bridge_end_ = std::make_pair(final_pose.x, final_pose.y);
    bridge_started_ = std::chrono::steady_clock::now();
    set_state(kTurnBridge, "path=" + path_key);
  }

  double distance_to_bridge_end() const
  {
    if (!current_pose_.has_value() || !bridge_end_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    return std::hypot(current_pose_->x - bridge_end_->first, current_pose_->y - bridge_end_->second);
  }

  void finish_bridge()
  {
    if (active_event_id_.has_value()) {
      completed_events_.insert(*active_event_id_);
    }
    reset_to_lane_follow("event path completed");
  }

  void publish_bridge_path()
  {
    if (!transformed_path_.has_value()) {
      return;
    }
    transformed_path_->header.stamp = now();
    for (auto & pose : transformed_path_->poses) {
      pose.header = transformed_path_->header;
    }
    bridge_pub_->publish(*transformed_path_);
  }

  std::optional<geometry_msgs::msg::PoseStamped> build_parking_goal()
  {
    if (!active_event_id_.has_value() || !current_pose_.has_value()) {
      return std::nullopt;
    }
    try {
      const YAML::Node goal_local = events_[*active_event_id_]["parking_goal_local"];
      const double local_x = goal_local["x"].as<double>();
      const double local_y = goal_local["y"].as<double>();
      const double local_yaw = goal_local["yaw"].as<double>();
      const double c = std::cos(current_pose_->yaw);
      const double s = std::sin(current_pose_->yaw);
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "map";
      goal.header.stamp = now();
      goal.pose.position.x = current_pose_->x + c * local_x - s * local_y;
      goal.pose.position.y = current_pose_->y + s * local_x + c * local_y;
      goal.pose.orientation = parking_cpp::quaternion_from_yaw(
        parking_cpp::normalize_angle(current_pose_->yaw + local_yaw));
      return goal;
    } catch (const YAML::Exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "%s has no valid parking_goal_local: %s",
        active_event_id_->c_str(), exception.what());
      return std::nullopt;
    }
  }

  void enter_parking_plan()
  {
    if (parking_goal_sent_) {
      return;
    }
    const auto goal = build_parking_goal();
    if (!goal.has_value()) {
      set_state(kParkingFailed, "invalid parking goal");
      return;
    }
    parking_goal_pub_->publish(*goal);
    parking_goal_sent_ = true;
    set_state(kParkingPlan, "parking goal published");
  }

  void finish_parking()
  {
    if (active_event_id_.has_value()) {
      completed_events_.insert(*active_event_id_);
    }
    set_state(kParked, "parking controller DONE");
  }

  void reset_to_lane_follow(const std::string & reason)
  {
    active_event_id_.reset();
    active_path_key_.reset();
    transformed_path_.reset();
    bridge_end_.reset();
    parking_goal_sent_ = false;
    parking_planner_status_ = "IDLE";
    parking_controller_status_ = "IDLE";
    set_state(kLaneFollow, reason);
  }

  void publish_state()
  {
    std_msgs::msg::String mode;
    mode.data = state_;
    mode_pub_->publish(mode);
    std_msgs::msg::String active;
    active.data = active_event_id_.value_or("");
    active_event_pub_->publish(active);
  }

  void tick()
  {
    const double time = now_s();
    if (!odom_fresh(time)) {
      std_msgs::msg::String stop;
      stop.data = kStopAtLight;
      mode_pub_->publish(stop);
      return;
    }

    if (state_ == kLaneFollow) {
      if (gps_fresh(time)) {
        const auto event = find_event_in_range();
        if (event.has_value()) {
          activate_event(*event, "entered GPS event radius");
        }
      }
    } else if (state_ == kApproach) {
      if (!active_event_id_.has_value()) {
        reset_to_lane_follow("no active event");
      } else {
        const YAML::Node event = events_[*active_event_id_];
        const double radius = yaml_value<double>(event, "approach_radius_m", 2.5);
        const double distance = distance_to_event(*active_event_id_);
        if (distance > radius + approach_cancel_margin_m_) {
          reset_to_lane_follow("left event approach radius");
        } else if (stop_line_fresh(time) && stop_line_distance_m_ <= stop_distance_m_) {
          if (signal_allows_entry(time)) {
            enter_bridge();
          } else {
            set_state(kStopAtLight, "sign=" + current_sign(time));
          }
        }
      }
    } else if (state_ == kStopAtLight) {
      if (signal_allows_entry(time)) {
        enter_bridge();
      }
    } else if (state_ == kTurnBridge) {
      publish_bridge_path();
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - bridge_started_).count();
      if (elapsed >= minimum_bridge_time_s_ &&
        distance_to_bridge_end() <= bridge_exit_tolerance_m_)
      {
        finish_bridge();
      }
    } else if (state_ == kParkingApproach) {
      if (!active_event_id_.has_value()) {
        reset_to_lane_follow("no active parking event");
      } else {
        const YAML::Node event = events_[*active_event_id_];
        const double radius = yaml_value<double>(event, "approach_radius_m", 2.5);
        const double distance = distance_to_event(*active_event_id_);
        if (distance > radius + approach_cancel_margin_m_) {
          reset_to_lane_follow("left parking radius");
        } else {
          const double trigger = yaml_value<double>(
            event, "parking_plan_trigger_m", default_parking_plan_trigger_m_);
          if (distance <= trigger) {
            enter_parking_plan();
          }
        }
      }
    } else if (state_ == kParkingPlan) {
      if (parking_planner_status_ == "READY") {
        set_state(kParking, "Hybrid A* path ready");
      } else if (parking_planner_status_ == "FAILED") {
        set_state(kParkingFailed, "Hybrid A* planning failed");
      }
    } else if (state_ == kParking) {
      if (parking_controller_status_ == "DONE") {
        finish_parking();
      } else if (parking_controller_status_ == "ERROR") {
        set_state(kParkingFailed, "parking controller error");
      }
    }

    publish_state();
  }

  std::filesystem::path yaml_path_;
  double tick_hz_{10.0};
  double gps_timeout_s_{1.5};
  double odom_timeout_s_{0.5};
  double perception_timeout_s_{0.5};
  double sign_timeout_s_{1.5};
  double stop_distance_m_{0.50};
  double bridge_exit_tolerance_m_{0.50};
  double minimum_bridge_time_s_{1.0};
  double event_rearm_margin_m_{3.0};
  double approach_cancel_margin_m_{1.5};
  double default_parking_plan_trigger_m_{0.80};
  std::string default_mission_{"straight"};
  std::string mission_{"straight"};
  bool left_on_green_allowed_{false};
  bool right_on_green_allowed_{true};
  bool arrow_signal_selects_path_{true};

  YAML::Node events_;
  YAML::Node paths_;
  std::string state_{kLaneFollow};
  std::optional<std::string> active_event_id_;
  std::optional<std::string> active_path_key_;
  std::optional<nav_msgs::msg::Path> transformed_path_;
  std::optional<std::pair<double, double>> bridge_end_;
  std::chrono::steady_clock::time_point bridge_started_{};
  std::set<std::string> completed_events_;

  std::optional<GpsPoint> current_gps_;
  std::optional<Pose2D> current_pose_;
  double gps_time_s_{-1e9};
  double odom_time_s_{-1e9};
  bool stop_line_detected_{false};
  double stop_line_distance_m_{std::numeric_limits<double>::infinity()};
  double stopline_time_s_{-1e9};
  std::string sign_{"none"};
  double sign_time_s_{-1e9};
  std::string parking_planner_status_{"IDLE"};
  std::string parking_controller_status_{"IDLE"};
  bool parking_goal_sent_{false};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stopline_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sign_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parking_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parking_controller_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bridge_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_event_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr parking_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorSupervisorWithParking>());
  rclcpp::shutdown();
  return 0;
}
