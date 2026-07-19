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
#include <utility>

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
constexpr const char * kHillApproach = "HILL_APPROACH";
constexpr const char * kHillStop = "HILL_STOP";

struct Pose2D {double x{0.0}; double y{0.0}; double yaw{0.0};};
struct GpsPoint {double latitude{0.0}; double longitude{0.0};};

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return "";}
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

std::filesystem::path expand_home(const std::string & value)
{
  if (!value.empty() && value.front() == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      if (value == "~") {return std::filesystem::path(home);}
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
  BehaviorSupervisorWithParking() : Node("behavior_supervisor")
  {
    yaml_path_ = expand_home(declare_parameter<std::string>("event_path_yaml", "course.yaml"));
    tick_hz_ = declare_parameter<double>("tick_hz", 10.0);
    gps_timeout_s_ = declare_parameter<double>("gps_timeout", 1.5);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout", 0.5);
    perception_timeout_s_ = declare_parameter<double>("perception_timeout", 0.5);
    sign_timeout_s_ = declare_parameter<double>("sign_timeout", 1.5);
    stop_distance_m_ = declare_parameter<double>("stop_line_trigger_distance_m", 3.0);
    bridge_exit_tolerance_m_ = declare_parameter<double>("bridge_exit_tolerance_m", 0.50);
    minimum_bridge_time_s_ = declare_parameter<double>("min_bridge_time_s", 1.0);
    event_rearm_margin_m_ = declare_parameter<double>("event_rearm_margin_m", 3.0);
    approach_cancel_margin_m_ = declare_parameter<double>("approach_cancel_margin_m", 1.5);
    default_hill_stop_duration_s_ = declare_parameter<double>("hill_stop_duration_s", 5.0);
    default_mission_ = lower(declare_parameter<std::string>("default_mission", "straight"));
    mission_ = default_mission_;
    left_on_green_allowed_ = declare_parameter<bool>("left_on_green_allowed", false);
    right_on_green_allowed_ = declare_parameter<bool>("right_on_green_allowed", true);
    arrow_signal_selects_path_ = declare_parameter<bool>("arrow_signal_selects_path", true);

    load_yaml();

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&BehaviorSupervisorWithParking::on_gps, this, std::placeholders::_1));
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

    mode_pub_ = create_publisher<std_msgs::msg::String>("/driving_mode", 10);
    bridge_pub_ = create_publisher<nav_msgs::msg::Path>("/bridge_path", 10);
    active_event_pub_ = create_publisher<std_msgs::msg::String>("/active_event", 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tick_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BehaviorSupervisorWithParking::tick, this));

    RCLCPP_INFO(get_logger(), "Behavior supervisor started (intersection + hill stop): yaml=%s",
      yaml_path_.string().c_str());
  }

private:
  double now_s() const {return now().seconds();}

  template<typename T>
  T yaml_value(const YAML::Node & node, const std::string & key, const T & fallback) const
  {
    try {
      if (node && node[key]) {return node[key].as<T>();}
    } catch (const YAML::Exception &) {}
    return fallback;
  }

  void load_yaml()
  {
    try {
      const YAML::Node root = YAML::LoadFile(yaml_path_.string());
      events_ = root["events"];
      paths_ = root["paths"];
      if (!events_ || !events_.IsMap()) {events_ = YAML::Node(YAML::NodeType::Map);}
      if (!paths_ || !paths_.IsMap()) {paths_ = YAML::Node(YAML::NodeType::Map);}
      RCLCPP_INFO(get_logger(), "Loaded %zu events and %zu paths.", events_.size(), paths_.size());
    } catch (const std::exception & e) {
      events_ = YAML::Node(YAML::NodeType::Map);
      paths_ = YAML::Node(YAML::NodeType::Map);
      RCLCPP_ERROR(get_logger(), "Failed to load event YAML: %s", e.what());
    }
  }

  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < 0 || !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
      std::abs(msg->latitude) > 90.0 || std::abs(msg->longitude) > 180.0) {return;}
    current_gps_ = GpsPoint{msg->latitude, msg->longitude};
    gps_time_s_ = now_s();
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = Pose2D{msg->pose.pose.position.x, msg->pose.pose.position.y,
      parking_cpp::yaw_from_quaternion(msg->pose.pose.orientation)};
    odom_time_s_ = now_s();
  }

  void on_stopline(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2U) {return;}
    stop_line_detected_ = msg->data[1] >= 0.5;
    stop_line_distance_m_ = stop_line_detected_ && std::isfinite(msg->data[0]) && msg->data[0] >= 0.0 ?
      msg->data[0] : std::numeric_limits<double>::infinity();
    stopline_time_s_ = now_s();
  }

  void on_sign(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string value = lower(trim(msg->data));
    if (value != "none" && value != "red" && value != "green" &&
      value != "left_arrow" && value != "right_arrow") {value = "none";}
    sign_ = value;
    sign_time_s_ = now_s();
  }

  void on_mission(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string value = lower(trim(msg->data));
    if (!value.empty()) {mission_ = value;}
  }

  void on_event_command(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string command = trim(msg->data);
    if (command == "reset") {reset_to_lane_follow("manual reset"); return;}
    if (command == "reload") {load_yaml(); return;}
    if (command == "clear_completed") {completed_events_.clear(); return;}

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

  bool gps_fresh(double t) const {return current_gps_ && t - gps_time_s_ <= gps_timeout_s_;}
  bool odom_fresh(double t) const {return current_pose_ && t - odom_time_s_ <= odom_timeout_s_;}
  bool stop_line_fresh(double t) const
  {
    return stop_line_detected_ && t - stopline_time_s_ <= perception_timeout_s_ &&
           std::isfinite(stop_line_distance_m_);
  }
  std::string current_sign(double t) const {return t - sign_time_s_ <= sign_timeout_s_ ? sign_ : "none";}

  std::string event_type(const std::string & event_id) const
  {
    const std::string type = lower(yaml_value<std::string>(events_[event_id], "event_type", "intersection"));
    return (type == "hill_stop" || type == "stopline" || type == "slope_stop") ? "hill_stop" : "intersection";
  }

  void set_state(const std::string & state, const std::string & reason = "")
  {
    if (state == state_) {return;}
    const std::string previous = state_;
    state_ = state;
    RCLCPP_INFO(get_logger(), "STATE: %s -> %s%s%s", previous.c_str(), state_.c_str(),
      reason.empty() ? "" : " | ", reason.c_str());
  }

  double distance_to_event(const std::string & event_id) const
  {
    if (!current_gps_) {return std::numeric_limits<double>::infinity();}
    try {
      const YAML::Node event = events_[event_id];
      return parking_cpp::haversine_m(current_gps_->latitude, current_gps_->longitude,
        event["latitude"].as<double>(), event["longitude"].as<double>());
    } catch (const YAML::Exception &) {return std::numeric_limits<double>::infinity();}
  }

  std::optional<std::string> find_event_in_range()
  {
    std::optional<std::string> best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (auto it = events_.begin(); it != events_.end(); ++it) {
      const std::string id = it->first.as<std::string>();
      const double radius = yaml_value<double>(it->second, "approach_radius_m", 2.5);
      const double distance = distance_to_event(id);
      if (completed_events_.count(id) > 0U) {
        if (distance > radius + event_rearm_margin_m_) {completed_events_.erase(id);} else {continue;}
      }
      if (distance <= radius && distance < best_distance) {best = id; best_distance = distance;}
    }
    return best;
  }

  void activate_event(const std::string & id, const std::string & reason)
  {
    active_event_id_ = id;
    if (event_type(id) == "hill_stop") {set_state(kHillApproach, reason);} else {set_state(kApproach, reason);}
  }

  std::string selected_direction(double t) const
  {
    const std::string signal = current_sign(t);
    if (arrow_signal_selects_path_) {
      if (signal == "left_arrow") {return "left";}
      if (signal == "right_arrow") {return "right";}
      if (signal == "green") {return "straight";}
    }
    return mission_;
  }

  bool signal_allows_entry(double t) const
  {
    if (!active_event_id_) {return false;}
    const YAML::Node event = events_[*active_event_id_];
    if (!yaml_value<bool>(event, "signal_required", true)) {return true;}
    const std::string signal = current_sign(t);
    const std::string direction = selected_direction(t);
    if (direction == "straight") {return signal == "green";}
    if (direction == "left") {return signal == "left_arrow" || (signal == "green" && left_on_green_allowed_);}
    if (direction == "right") {return signal == "right_arrow" || (signal == "green" && right_on_green_allowed_);}
    return false;
  }

  std::optional<nav_msgs::msg::Path> transform_relative_path(const YAML::Node & points)
  {
    if (!current_pose_ || !points || !points.IsSequence()) {return std::nullopt;}
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    const double c = std::cos(current_pose_->yaw);
    const double s = std::sin(current_pose_->yaw);
    for (const auto & point : points) {
      try {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        const double lx = point["x"].as<double>();
        const double ly = point["y"].as<double>();
        const double lyaw = point["yaw"] ? point["yaw"].as<double>() : 0.0;
        pose.pose.position.x = current_pose_->x + c * lx - s * ly;
        pose.pose.position.y = current_pose_->y + s * lx + c * ly;
        pose.pose.orientation = parking_cpp::quaternion_from_yaw(
          parking_cpp::normalize_angle(current_pose_->yaw + lyaw));
        path.poses.push_back(pose);
      } catch (const YAML::Exception &) {}
    }
    return path.poses.empty() ? std::nullopt : std::optional<nav_msgs::msg::Path>(path);
  }

  void enter_bridge()
  {
    if (!active_event_id_) {reset_to_lane_follow("bridge requested without event"); return;}
    const YAML::Node event = events_[*active_event_id_];
    const std::string direction = selected_direction(now_s());
    const YAML::Node key_node = event["paths"] ? event["paths"][direction] : YAML::Node();
    if (!key_node) {reset_to_lane_follow("missing event path"); return;}
    const std::string key = key_node.as<std::string>();
    const auto transformed = transform_relative_path(paths_[key]);
    if (!transformed) {reset_to_lane_follow("empty event path"); return;}
    transformed_path_ = *transformed;
    const auto & p = transformed_path_->poses.back().pose.position;
    bridge_end_ = std::make_pair(p.x, p.y);
    bridge_started_ = std::chrono::steady_clock::now();
    set_state(kTurnBridge, "path=" + key);
  }

  void publish_bridge_path()
  {
    if (!transformed_path_) {return;}
    transformed_path_->header.stamp = now();
    for (auto & pose : transformed_path_->poses) {pose.header = transformed_path_->header;}
    bridge_pub_->publish(*transformed_path_);
  }

  void begin_hill_stop()
  {
    if (!active_event_id_) {return;}
    hill_stop_duration_current_s_ = yaml_value<double>(
      events_[*active_event_id_], "stop_duration_s", default_hill_stop_duration_s_);
    hill_stop_started_ = std::chrono::steady_clock::now();
    set_state(kHillStop, "stop_duration=" + std::to_string(hill_stop_duration_current_s_) + "s");
  }

  void complete_active_event(const std::string & reason)
  {
    if (active_event_id_) {completed_events_.insert(*active_event_id_);}
    reset_to_lane_follow(reason);
  }

  void reset_to_lane_follow(const std::string & reason)
  {
    active_event_id_.reset();
    transformed_path_.reset();
    bridge_end_.reset();
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
    const double t = now_s();
    if (!odom_fresh(t)) {
      std_msgs::msg::String stop;
      stop.data = kStopAtLight;
      mode_pub_->publish(stop);
      return;
    }

    if (state_ == kLaneFollow) {
      if (gps_fresh(t)) {
        const auto event = find_event_in_range();
        if (event) {activate_event(*event, "entered GPS event radius");}
      }
    } else if (state_ == kApproach) {
      if (!active_event_id_) {reset_to_lane_follow("no active event");}
      else {
        const YAML::Node event = events_[*active_event_id_];
        const double radius = yaml_value<double>(event, "approach_radius_m", 2.5);
        const double distance = distance_to_event(*active_event_id_);
        if (distance > radius + approach_cancel_margin_m_) {reset_to_lane_follow("left event radius");}
        else if (stop_line_fresh(t) && stop_line_distance_m_ <= stop_distance_m_) {
          if (signal_allows_entry(t)) {enter_bridge();}
          else {set_state(kStopAtLight, "sign=" + current_sign(t));}
        }
      }
    } else if (state_ == kStopAtLight) {
      if (signal_allows_entry(t)) {enter_bridge();}
    } else if (state_ == kTurnBridge) {
      publish_bridge_path();
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - bridge_started_).count();
      if (elapsed >= minimum_bridge_time_s_ && current_pose_ && bridge_end_ &&
        std::hypot(current_pose_->x - bridge_end_->first, current_pose_->y - bridge_end_->second) <=
        bridge_exit_tolerance_m_) {complete_active_event("event path completed");}
    } else if (state_ == kHillApproach) {
      if (!active_event_id_) {reset_to_lane_follow("no active hill event");}
      else {
        const YAML::Node event = events_[*active_event_id_];
        const double radius = yaml_value<double>(event, "approach_radius_m", 2.5);
        const double distance = distance_to_event(*active_event_id_);
        if (distance > radius + approach_cancel_margin_m_) {reset_to_lane_follow("left hill event radius");}
        else if (stop_line_fresh(t) && stop_line_distance_m_ <= stop_distance_m_) {begin_hill_stop();}
      }
    } else if (state_ == kHillStop) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - hill_stop_started_).count();
      if (elapsed >= hill_stop_duration_current_s_) {complete_active_event("hill stop completed");}
    }

    publish_state();
  }

  std::filesystem::path yaml_path_;
  double tick_hz_{10.0};
  double gps_timeout_s_{1.5};
  double odom_timeout_s_{0.5};
  double perception_timeout_s_{0.5};
  double sign_timeout_s_{1.5};
  double stop_distance_m_{0.5};
  double bridge_exit_tolerance_m_{0.5};
  double minimum_bridge_time_s_{1.0};
  double event_rearm_margin_m_{3.0};
  double approach_cancel_margin_m_{1.5};
  double default_hill_stop_duration_s_{5.0};
  double hill_stop_duration_current_s_{5.0};
  std::string default_mission_{"straight"};
  std::string mission_{"straight"};
  bool left_on_green_allowed_{false};
  bool right_on_green_allowed_{true};
  bool arrow_signal_selects_path_{true};

  YAML::Node events_;
  YAML::Node paths_;
  std::string state_{kLaneFollow};
  std::optional<std::string> active_event_id_;
  std::optional<nav_msgs::msg::Path> transformed_path_;
  std::optional<std::pair<double, double>> bridge_end_;
  std::chrono::steady_clock::time_point bridge_started_{};
  std::chrono::steady_clock::time_point hill_stop_started_{};
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

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stopline_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sign_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bridge_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_event_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorSupervisorWithParking>());
  rclcpp::shutdown();
  return 0;
}
