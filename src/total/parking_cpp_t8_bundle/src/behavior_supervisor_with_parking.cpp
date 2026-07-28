#include "parking_cpp/common.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
// Default cruising states: follow the left or right lane and scan for GPS-radius entry into the
// next scripted event in kCourseSequence.
constexpr const char * kLeftLaneFollow = "LEFT_LANE_FOLLOW";
constexpr const char * kRightLaneFollow = "RIGHT_LANE_FOLLOW";
// Inside an intersection event's approach radius, closing on the stop line before requesting
// entry. Split per intersection (rather than one shared generic state) so /driving_mode always
// says which of A/B/C is active, instead of leaving that only inferable from /active_event.
constexpr const char * kIntersectionAApproach = "INTERSECTION_A_APPROACH";
constexpr const char * kIntersectionAStopAtLight = "INTERSECTION_A_STOP_AT_LIGHT";
constexpr const char * kIntersectionATurnBridge = "INTERSECTION_A_TURN_BRIDGE";
constexpr const char * kIntersectionBApproach = "INTERSECTION_B_APPROACH";
constexpr const char * kIntersectionBStopAtLight = "INTERSECTION_B_STOP_AT_LIGHT";
constexpr const char * kIntersectionBTurnBridge = "INTERSECTION_B_TURN_BRIDGE";
constexpr const char * kIntersectionCApproach = "INTERSECTION_C_APPROACH";
constexpr const char * kIntersectionCStopAtLight = "INTERSECTION_C_STOP_AT_LIGHT";
constexpr const char * kIntersectionCTurnBridge = "INTERSECTION_C_TURN_BRIDGE";
// Inside a hill_stop event's approach radius, closing on the stop line before beginning the timed
// hill stop. Only one hill-stop event exists in the course, so unlike intersections this doesn't
// need a per-event variant to stay unambiguous.
constexpr const char * kHillApproach = "HILL_APPROACH";
// Stopped at the hill for a fixed duration (stop_duration_s), then the event completes and control returns to lane following.
constexpr const char * kHillStop = "HILL_STOP";
// Inside an accel/obstacle zone, driving while monitoring LIDAR freshness and front obstacle
// distance until the end radius is reached. Only one such event exists in the course.
constexpr const char * kAccelObstacleZone = "ACCEL_OBSTACLE_ZONE";
// Stopped in an accel/obstacle zone because an obstacle is too close (or the scan timed out); resumes once clear for obstacle_clear_hold_s.
constexpr const char * kObstacleStop = "OBSTACLE_STOP";
// Terminal state once the course-end GPS zone is reached: holds a full stop indefinitely. Unlike
// every other event there's nothing after it in kCourseSequence, so this is never left.
constexpr const char * kEndStop = "END_STOP_AT_LIGHT";
// Safety fallback published when odometry is stale, regardless of what was happening before.
constexpr const char * kOdomStaleStop = "STOP_AT_LIGHT";
// Parking: drives the recorded entry_path (relative to the GPS-trigger pose) across the lane-less
// gap to the physical parking-start point, then re-anchors spot_path to wherever the vehicle
// actually stopped and drives it (gear changes included) into the spot, then optionally holds for
// hold_duration_s before the slot completes like any other event. Split per parking style (rather
// than one shared PARKING_* name) for the same reason intersections are split per letter: so
// /driving_mode always says which parking maneuver is active.
constexpr const char * kParkingTZoneApproach = "PARKING_T_ZONE_APPROACH";
constexpr const char * kParkingTZoneManeuver = "PARKING_T_ZONE_MANEUVER";
constexpr const char * kParkingTZoneHold = "PARKING_T_ZONE_HOLD";
constexpr const char * kParkingParallelApproach = "PARKING_PARALLEL_APPROACH";
constexpr const char * kParkingParallelManeuver = "PARKING_PARALLEL_MANEUVER";
constexpr const char * kParkingParallelHold = "PARKING_PARALLEL_HOLD";

// The course's fixed slot order. Cruise slots carry the lane side to follow; event slots name
// which scripted event (looked up in course.yaml via event_id_for()) is active. Replaces what
// used to be a separate event_sequence_ (YAML list of event ids) + sequence_index_ pair -- the
// ordering now lives directly in this vector instead of alongside-but-separate-from the FSM
// state, so there's one place, not two, that says "what's next."
enum class SlotKind
{
  kCruiseLeft,
  kCruiseRight,
  kHillstop, 
  kLaneChange1, // Left lane -> Right lane
  kLaneChange2, // Right lane -> Left lane
  kIntersectionA,
  kIntersectionB,
  kIntersectionC,
  kAccelObstacle,
  kLaneChange4, // Left lane -> Right lane
  kLaneChange5, // Left lane -> Right lane
  kParkingTZone,
  kParkingParallel,
  kEnd,
};

// NOTE: kParkingParallel's slot position below is still a placeholder (near course end) pending
// its actual course location; kParkingTZone has been placed at its real spot (right after
// intersection_B, which goes directly into right-lane cruise). Repositioning either is a one-line
// reorder -- nothing else depends on position other than "cruise slot right before" (the lane
// side the approach starts from) and "cruise slot right after" (the lane side driving resumes in
// once parked).
const std::vector<SlotKind> kCourseSequence = {
  SlotKind::kCruiseLeft,
  SlotKind::kHillstop,
  SlotKind::kCruiseLeft,
  SlotKind::kLaneChange1,
  SlotKind::kCruiseRight,
  SlotKind::kLaneChange2,
  SlotKind::kCruiseLeft,
  SlotKind::kIntersectionA,
  SlotKind::kCruiseRight,
  SlotKind::kIntersectionB,
  // T-zone parking: intersection_B straight goes directly into right-lane cruise (no left-lane
  // segment/lane-change in between) -- that's where it's reached, per course layout.
  SlotKind::kCruiseRight,
  SlotKind::kParkingTZone,
  SlotKind::kCruiseRight,
  SlotKind::kIntersectionC,
  SlotKind::kCruiseRight,
  SlotKind::kAccelObstacle,
  SlotKind::kCruiseRight,
  SlotKind::kLaneChange4,
  SlotKind::kCruiseLeft,
  SlotKind::kLaneChange5,
  SlotKind::kCruiseRight,
  // NOTE: kParkingParallel's position is still a placeholder (before course end) -- move it once
  // its actual location on the course is known, same one-line-reorder caveat as kParkingTZone had.
  SlotKind::kParkingParallel,
  SlotKind::kCruiseRight,
  SlotKind::kEnd
};

// Maps a course slot to its course.yaml event id, i.e. the key under `events:` (and, for
// intersections, transitively under `paths:`) that carries that slot's GPS/radius/path data.
// Returns an empty string for the cruise slots, which have no associated YAML event.
std::string event_id_for(SlotKind kind)
{
  switch (kind) {
    case SlotKind::kHillstop: return "slope_A";
    case SlotKind::kIntersectionA: return "intersection_A";
    case SlotKind::kIntersectionB: return "intersection_B";
    case SlotKind::kIntersectionC: return "intersection_C";
    case SlotKind::kAccelObstacle: return "accel_A";
    case SlotKind::kLaneChange1: return "lane_change_1";
    case SlotKind::kLaneChange2: return "lane_change_2";
    case SlotKind::kLaneChange4: return "lane_change_4";
    case SlotKind::kLaneChange5: return "lane_change_5";
    case SlotKind::kParkingTZone: return "parking_t1";
    case SlotKind::kParkingParallel: return "parking_p1";
    case SlotKind::kEnd: return "end";
    default: return "";
  }
}

bool is_cruise(SlotKind kind)
{
  return kind == SlotKind::kCruiseLeft || kind == SlotKind::kCruiseRight;
}

const char * cruise_state(SlotKind kind)
{
  return kind == SlotKind::kCruiseRight ? kRightLaneFollow : kLeftLaneFollow;
}

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
    approach_cancel_margin_m_ = declare_parameter<double>("approach_cancel_margin_m", 1.5);
    default_hill_stop_duration_s_ = declare_parameter<double>("hill_stop_duration_s", 5.0);
    default_mission_ = lower(declare_parameter<std::string>("default_mission", "straight"));
    mission_ = default_mission_;
    left_on_green_allowed_ = declare_parameter<bool>("left_on_green_allowed", false);
    right_on_green_allowed_ = declare_parameter<bool>("right_on_green_allowed", true);
    arrow_signal_selects_path_ = declare_parameter<bool>("arrow_signal_selects_path", true);
    scan_timeout_s_ = declare_parameter<double>("scan_timeout", 2.0);
    scan_startup_grace_s_ = declare_parameter<double>("scan_startup_grace_s", 3.0);
    stop_on_lidar_timeout_ = declare_parameter<bool>("stop_on_lidar_timeout", false);
    obstacle_min_distance_m_ = declare_parameter<double>("obstacle_min_distance_m", 0.40);
    default_accel_zone_speed_mps_ = declare_parameter<double>("accel_zone_speed_mps", 4.0);
    default_obstacle_stop_distance_m_ = declare_parameter<double>("obstacle_stop_distance_m", 2.0);
    default_obstacle_clear_distance_m_ = declare_parameter<double>("obstacle_clear_distance_m", 2.5);
    default_obstacle_half_width_m_ = declare_parameter<double>("obstacle_half_width_m", 0.55);
    default_obstacle_clear_hold_s_ = declare_parameter<double>("obstacle_clear_hold_s", 1.0);
    parking_entry_tolerance_m_ = declare_parameter<double>("parking_entry_tolerance_m", 0.35);
    parking_entry_heading_tolerance_deg_ =
      declare_parameter<double>("parking_entry_heading_tolerance_deg", 15.0);
    parking_exit_tolerance_m_ = declare_parameter<double>("parking_exit_tolerance_m", 0.25);
    parking_rear_stop_distance_m_ = declare_parameter<double>("parking_rear_stop_distance_m", 0.30);
    default_parking_hold_duration_s_ = declare_parameter<double>("parking_hold_duration_s", 0.0);

    load_yaml();

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&BehaviorSupervisorWithParking::on_gps, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map", 10,
      std::bind(&BehaviorSupervisorWithParking::on_odom, this, std::placeholders::_1));
    stopline_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/stopline/detection", 10,
      std::bind(&BehaviorSupervisorWithParking::on_stopline, this, std::placeholders::_1));
    rear_stopline_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/stopline/rear_detection", 10,
      std::bind(&BehaviorSupervisorWithParking::on_rear_stopline, this, std::placeholders::_1));
    sign_sub_ = create_subscription<std_msgs::msg::String>(
      "/perception/sign", 10,
      std::bind(&BehaviorSupervisorWithParking::on_sign, this, std::placeholders::_1));
    mission_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/turn", 10,
      std::bind(&BehaviorSupervisorWithParking::on_mission, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&BehaviorSupervisorWithParking::on_scan, this, std::placeholders::_1));
    event_command_sub_ = create_subscription<std_msgs::msg::String>(
      "/event/cmd", 10,
      std::bind(&BehaviorSupervisorWithParking::on_event_command, this, std::placeholders::_1));

    mode_pub_ = create_publisher<std_msgs::msg::String>("/driving_mode", 10);
    bridge_pub_ = create_publisher<nav_msgs::msg::Path>("/bridge_path", 10);
    active_event_pub_ = create_publisher<std_msgs::msg::String>("/active_event", 10);
    parking_entry_pub_ = create_publisher<nav_msgs::msg::Path>("/parking/entry_path", 10);
    parking_spot_pub_ = create_publisher<nav_msgs::msg::Path>("/parking/spot_path", 10);
    parking_gear_pub_ = create_publisher<std_msgs::msg::Int8MultiArray>("/parking/spot_gear", 10);

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

      // The course order itself is hardcoded (kCourseSequence), not read from YAML anymore --
      // just sanity-check that every event slot it references actually exists in this file.
      for (const auto kind : kCourseSequence) {
        if (is_cruise(kind)) {continue;}
        const std::string id = event_id_for(kind);
        if (!events_[id]) {
          RCLCPP_WARN(get_logger(),
            "kCourseSequence references event id '%s', which is missing from course.yaml.",
            id.c_str());
        }
      }

      RCLCPP_INFO(get_logger(), "Loaded %zu events, %zu paths.", events_.size(), paths_.size());
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

  void on_rear_stopline(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2U) {return;}
    rear_stop_line_detected_ = msg->data[1] >= 0.5;
    rear_stop_line_distance_m_ =
      rear_stop_line_detected_ && std::isfinite(msg->data[0]) && msg->data[0] >= 0.0 ?
      msg->data[0] : std::numeric_limits<double>::infinity();
    rear_stopline_time_s_ = now_s();
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


  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_received_ = true;
    scan_time_s_ = now_s();
    double closest_x = std::numeric_limits<double>::infinity();
    const double half_width = active_event_index_ ?
      yaml_value<double>(events_[event_id_for(kCourseSequence[*active_event_index_])],
        "obstacle_half_width_m", default_obstacle_half_width_m_) : default_obstacle_half_width_m_;

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double range = msg->ranges[i];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {continue;}
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const double x = range * std::cos(angle);
      const double y = range * std::sin(angle);
      if (x >= obstacle_min_distance_m_ && std::abs(y) <= half_width) {
        closest_x = std::min(closest_x, x);
      }
    }
    front_obstacle_distance_m_ = closest_x;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "SCAN callback OK | nearest_front=%.3f m | samples=%zu",
      front_obstacle_distance_m_, msg->ranges.size());
  }

  void on_event_command(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string command = trim(msg->data);
    if (command == "reset") {reset_to_lane_follow("manual reset"); return;}
    if (command == "reload") {load_yaml(); return;}

    const auto separator = command.find(':');
    if (separator != std::string::npos && command.substr(0, separator) == "force_event") {
      const std::string event_id = command.substr(separator + 1U);
      if (!events_[event_id]) {
        RCLCPP_WARN(get_logger(), "Unknown event: %s", event_id.c_str());
        return;
      }
      for (std::size_t i = 0; i < kCourseSequence.size(); ++i) {
        if (!is_cruise(kCourseSequence[i]) && event_id_for(kCourseSequence[i]) == event_id) {
          activate_event(i, "manual force_event");
          return;
        }
      }
      RCLCPP_WARN(get_logger(),
        "Event '%s' exists in course.yaml but is not part of the hardcoded course sequence.",
        event_id.c_str());
    }
  }

  bool gps_fresh(double t) const {return current_gps_ && t - gps_time_s_ <= gps_timeout_s_;}
  bool odom_fresh(double t) const {return current_pose_ && t - odom_time_s_ <= odom_timeout_s_;}
  bool stop_line_fresh(double t) const
  {
    return stop_line_detected_ && t - stopline_time_s_ <= perception_timeout_s_ &&
           std::isfinite(stop_line_distance_m_);
  }
  bool rear_stop_line_fresh(double t) const
  {
    return rear_stop_line_detected_ && t - rear_stopline_time_s_ <= perception_timeout_s_ &&
           std::isfinite(rear_stop_line_distance_m_);
  }
  std::string current_sign(double t) const {return t - sign_time_s_ <= sign_timeout_s_ ? sign_ : "none";}

  std::string event_type(const std::string & event_id) const
  {
    const std::string type = lower(yaml_value<std::string>(events_[event_id], "event_type", "intersection"));
    if (type == "hill_stop" || type == "stopline" || type == "slope_stop") {return "hill_stop";}
    if (type == "accel_obstacle" || type == "obstacle_zone") {return "accel_obstacle";}
    return "intersection";
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
      const YAML::Node point = event_type(event_id) == "accel_obstacle" && event["start"] ?
        event["start"] : event;
      return parking_cpp::haversine_m(current_gps_->latitude, current_gps_->longitude,
        point["latitude"].as<double>(), point["longitude"].as<double>());
    } catch (const YAML::Exception &) {return std::numeric_limits<double>::infinity();}
  }

  double distance_to_accel_end(const std::string & event_id) const
  {
    if (!current_gps_) {return std::numeric_limits<double>::infinity();}
    try {
      const YAML::Node end = events_[event_id]["end"];
      if (!end) {return std::numeric_limits<double>::infinity();}
      return parking_cpp::haversine_m(current_gps_->latitude, current_gps_->longitude,
        end["latitude"].as<double>(), end["longitude"].as<double>());
    } catch (const YAML::Exception &) {return std::numeric_limits<double>::infinity();}
  }

  // The next event to watch for is always deterministic now -- whatever immediately follows the
  // current cruise slot in kCourseSequence -- so this just checks that one candidate's GPS
  // radius, replacing what used to be a scan over every unsequenced event in the YAML map.
  std::optional<std::size_t> next_event_index_in_range() const
  {
    const std::size_t next_index = cruise_index_ + 1U;
    if (next_index >= kCourseSequence.size()) {return std::nullopt;}
    const SlotKind kind = kCourseSequence[next_index];
    const std::string id = event_id_for(kind);
    const bool accel_event = kind == SlotKind::kAccelObstacle;
    const double radius = accel_event ?
      yaml_value<double>(events_[id], "start_radius_m",
        yaml_value<double>(events_[id], "approach_radius_m", 2.5)) :
      yaml_value<double>(events_[id], "approach_radius_m", 2.5);
    return distance_to_event(id) <= radius ? std::optional<std::size_t>(next_index) : std::nullopt;
  }

  void activate_event(std::size_t index, const std::string & reason)
  {
    const SlotKind kind = kCourseSequence[index];
    const std::string id = event_id_for(kind);
    const std::string type = event_type(id);
    if (type == "accel_obstacle") {
      const YAML::Node event = events_[id];
      if (!event["start"] || !event["end"]) {
        RCLCPP_WARN(get_logger(), "Accel event '%s' requires both start and end GPS points", id.c_str());
        return;
      }
    }

    active_event_index_ = index;

    if (kind == SlotKind::kHillstop) {set_state(kHillApproach, reason);}
    else if (kind == SlotKind::kAccelObstacle) {
      obstacle_clear_started_.reset();
      accel_zone_enter_time_s_ = now_s();
      set_state(kAccelObstacleZone, reason);
    } else if (kind == SlotKind::kIntersectionA) {set_state(kIntersectionAApproach, reason);}
    else if (kind == SlotKind::kIntersectionB) {set_state(kIntersectionBApproach, reason);}
    else if (kind == SlotKind::kIntersectionC) {set_state(kIntersectionCApproach, reason);}
    else if (kind == SlotKind::kLaneChange1 || kind == SlotKind::kLaneChange5) {
      complete_active_event(reason);
      set_state(kRightLaneFollow, reason);
    }
    else if (kind == SlotKind::kLaneChange2 || kind == SlotKind::kLaneChange4) {
      complete_active_event(reason);
      set_state(kLeftLaneFollow, reason);
    }
    else if (kind == SlotKind::kParkingTZone) {enter_parking_approach(kParkingTZoneApproach);}
    else if (kind == SlotKind::kParkingParallel) {enter_parking_approach(kParkingParallelApproach);}
    else if (kind == SlotKind::kEnd) {set_state(kEndStop, reason);}
    else {RCLCPP_WARN(get_logger(), "Unknown event kind %d", static_cast<int>(kind));}
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
    if (!active_event_index_) {return false;}
    const YAML::Node event = events_[event_id_for(kCourseSequence[*active_event_index_])];
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
    if (!active_event_index_) {reset_to_lane_follow("bridge requested without event"); return;}
    const SlotKind kind = kCourseSequence[*active_event_index_];
    const YAML::Node event = events_[event_id_for(kind)];
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
    const char * turn_state = kind == SlotKind::kIntersectionA ? kIntersectionATurnBridge :
      kind == SlotKind::kIntersectionB ? kIntersectionBTurnBridge : kIntersectionCTurnBridge;
    set_state(turn_state, "path=" + key);
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
    if (!active_event_index_) {return;}
    hill_stop_duration_current_s_ = yaml_value<double>(
      events_[event_id_for(kCourseSequence[*active_event_index_])],
      "stop_duration_s", default_hill_stop_duration_s_);
    hill_stop_started_ = std::chrono::steady_clock::now();
    set_state(kHillStop, "stop_duration=" + std::to_string(hill_stop_duration_current_s_) + "s");
  }

  std::vector<int8_t> extract_gear_array(const YAML::Node & points) const
  {
    std::vector<int8_t> gears;
    if (!points || !points.IsSequence()) {return gears;}
    for (const auto & point : points) {
      int8_t gear = 1;
      if (point["gear"]) {
        try {
          const std::string raw = lower(trim(point["gear"].as<std::string>()));
          if (raw == "reverse" || raw == "back" || raw == "-1") {gear = -1;}
        } catch (const YAML::Exception &) {}
      }
      gears.push_back(gear);
    }
    return gears;
  }

  // Drives the recorded entry_path (relative to the pose at GPS-trigger time) across the
  // lane-less gap to the physical parking-start point. This segment tolerates GNSS/odometry
  // anchor error because spot_path is re-anchored to wherever the vehicle actually ends up here,
  // not to this trigger pose.
  void enter_parking_approach(const char * approach_state)
  {
    if (!active_event_index_) {reset_to_lane_follow("parking requested without event"); return;}
    const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
    const YAML::Node event = events_[id];
    const YAML::Node key_node = event["entry_path"];
    if (!key_node) {reset_to_lane_follow("missing parking entry_path"); return;}
    const std::string key = key_node.as<std::string>();
    const auto transformed = transform_relative_path(paths_[key]);
    if (!transformed) {reset_to_lane_follow("empty parking entry_path"); return;}
    parking_entry_path_ = *transformed;
    const auto & last = parking_entry_path_->poses.back();
    parking_entry_end_ = std::make_pair(last.pose.position.x, last.pose.position.y);
    parking_entry_end_yaw_ = parking_cpp::yaw_from_quaternion(last.pose.orientation);
    set_state(approach_state, "path=" + key);
  }

  void publish_parking_entry_path()
  {
    if (!parking_entry_path_) {return;}
    parking_entry_path_->header.stamp = now();
    for (auto & pose : parking_entry_path_->poses) {pose.header = parking_entry_path_->header;}
    parking_entry_pub_->publish(*parking_entry_path_);
  }

  // Re-anchors spot_path to the vehicle's actual pose right now (arrival at the parking-start
  // point), not to the original GPS-trigger pose, so drift accumulated during entry_path does not
  // propagate into the reverse maneuver.
  void enter_parking_maneuver(const char * maneuver_state)
  {
    if (!active_event_index_) {reset_to_lane_follow("parking maneuver without event"); return;}
    const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
    const YAML::Node event = events_[id];
    const YAML::Node key_node = event["spot_path"];
    if (!key_node) {reset_to_lane_follow("missing parking spot_path"); return;}
    const std::string key = key_node.as<std::string>();
    const auto transformed = transform_relative_path(paths_[key]);
    if (!transformed) {reset_to_lane_follow("empty parking spot_path"); return;}
    parking_spot_path_ = *transformed;
    parking_spot_gear_ = extract_gear_array(paths_[key]);
    const auto & last = parking_spot_path_->poses.back();
    parking_spot_end_ = std::make_pair(last.pose.position.x, last.pose.position.y);
    set_state(maneuver_state, "path=" + key);
  }

  void publish_parking_maneuver()
  {
    if (!parking_spot_path_) {return;}
    parking_spot_path_->header.stamp = now();
    for (auto & pose : parking_spot_path_->poses) {pose.header = parking_spot_path_->header;}
    parking_spot_pub_->publish(*parking_spot_path_);
    std_msgs::msg::Int8MultiArray gear_msg;
    gear_msg.data = parking_spot_gear_;
    parking_gear_pub_->publish(gear_msg);
  }

  void begin_parking_hold(const char * hold_state)
  {
    if (!active_event_index_) {reset_to_lane_follow("no active event"); return;}
    const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
    parking_hold_duration_current_s_ = yaml_value<double>(
      events_[id], "hold_duration_s", default_parking_hold_duration_s_);
    if (parking_hold_duration_current_s_ <= 0.0) {complete_active_event("parking completed"); return;}
    parking_hold_started_ = std::chrono::steady_clock::now();
    set_state(hold_state, "hold_duration=" + std::to_string(parking_hold_duration_current_s_) + "s");
  }

  void complete_active_event(const std::string & reason)
  {
    if (active_event_index_) {
      // The event slot's index is always cruise_index_ + 1 (see next_event_index_in_range()), so
      // the cruise slot right after it is simply the next index.
      cruise_index_ = std::min(*active_event_index_ + 1U, kCourseSequence.size() - 1U);
      RCLCPP_INFO(get_logger(), "Sequence advanced: '%s' complete -> next cruise slot is %s",
        event_id_for(kCourseSequence[*active_event_index_]).c_str(),
        cruise_state(kCourseSequence[cruise_index_]));
    }
    reset_to_lane_follow(reason);
  }

  void reset_to_lane_follow(const std::string & reason)
  {
    active_event_index_.reset();
    transformed_path_.reset();
    bridge_end_.reset();
    obstacle_clear_started_.reset();
    accel_zone_enter_time_s_ = -1e9;
    parking_entry_path_.reset();
    parking_entry_end_.reset();
    parking_spot_path_.reset();
    parking_spot_gear_.clear();
    parking_spot_end_.reset();
    set_state(cruise_state(kCourseSequence[cruise_index_]), reason);
  }

  void publish_state()
  {
    std_msgs::msg::String mode;
    mode.data = state_;
    mode_pub_->publish(mode);
    std_msgs::msg::String active;
    active.data = active_event_index_ ? event_id_for(kCourseSequence[*active_event_index_]) : "";
    active_event_pub_->publish(active);
  }

  void tick()
  {
    const double t = now_s();
    if (!odom_fresh(t)) {
      std_msgs::msg::String stop;
      stop.data = kOdomStaleStop;
      mode_pub_->publish(stop);
      return;
    }

    if (!active_event_index_) {
      // Cruising: publish this slot's lane side and watch the single deterministic next event
      // (whatever immediately follows this cruise slot in kCourseSequence) for GPS entry.
      set_state(cruise_state(kCourseSequence[cruise_index_]));
      if (gps_fresh(t)) {
        const auto next_index = next_event_index_in_range();
        if (next_index) {activate_event(*next_index, "entered GPS event radius");}
      }
    } else if (state_ == kIntersectionAApproach || state_ == kIntersectionBApproach ||
      state_ == kIntersectionCApproach)
    {
      const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
      const YAML::Node event = events_[id];
      const double radius = yaml_value<double>(event, "approach_radius_m", 2.5);
      const double distance = distance_to_event(id);
      if (distance > radius + approach_cancel_margin_m_) {reset_to_lane_follow("left event radius");}
      else if (stop_line_fresh(t) && stop_line_distance_m_ <= stop_distance_m_) {
        if (signal_allows_entry(t)) {enter_bridge();}
        else {
          const SlotKind kind = kCourseSequence[*active_event_index_];
          const char * stop_state = kind == SlotKind::kIntersectionA ? kIntersectionAStopAtLight :
            kind == SlotKind::kIntersectionB ? kIntersectionBStopAtLight : kIntersectionCStopAtLight;
          set_state(stop_state, "sign=" + current_sign(t));
        }
      }
    } else if (state_ == kIntersectionAStopAtLight || state_ == kIntersectionBStopAtLight ||
      state_ == kIntersectionCStopAtLight)
    {
      if (signal_allows_entry(t)) {enter_bridge();}
    } else if (state_ == kIntersectionATurnBridge || state_ == kIntersectionBTurnBridge ||
      state_ == kIntersectionCTurnBridge)
    {
      publish_bridge_path();
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - bridge_started_).count();
      if (elapsed >= minimum_bridge_time_s_ && current_pose_ && bridge_end_ &&
        std::hypot(current_pose_->x - bridge_end_->first, current_pose_->y - bridge_end_->second) <=
        bridge_exit_tolerance_m_) {complete_active_event("event path completed");}
    } else if (state_ == kHillApproach) {
      const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
      const YAML::Node event = events_[id];
      const double radius = yaml_value<double>(event, "approach_radius_m", 2.5);
      const double distance = distance_to_event(id);
      if (distance > radius + approach_cancel_margin_m_) {reset_to_lane_follow("left hill event radius");}
      else if (stop_line_fresh(t) && stop_line_distance_m_ <= stop_distance_m_) {begin_hill_stop();}
    } else if (state_ == kHillStop) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - hill_stop_started_).count();
      if (elapsed >= hill_stop_duration_current_s_) {complete_active_event("hill stop completed");}
    } else if (state_ == kAccelObstacleZone) {
      const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
      const YAML::Node event = events_[id];
      const double stop_distance = yaml_value<double>(
        event, "obstacle_stop_distance_m", default_obstacle_stop_distance_m_);
      const double end_radius = yaml_value<double>(event, "end_radius_m", 2.5);
      const double end_distance = distance_to_accel_end(id);
      const bool scan_fresh =
        scan_received_ && (t - scan_time_s_ <= scan_timeout_s_);
      const double time_in_zone = t - accel_zone_enter_time_s_;

      if (end_distance <= end_radius) {
        complete_active_event("accel obstacle zone end reached");
      } else if (!scan_fresh) {
        if (time_in_zone >= scan_startup_grace_s_) {
          if (stop_on_lidar_timeout_) {
            set_state(kObstacleStop, "lidar timeout");
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Lidar data is stale, but stop_on_lidar_timeout=false. "
              "scan_received=%d, scan_age=%.3f s, timeout=%.3f s",
              scan_received_,
              scan_received_ ? (t - scan_time_s_) : -1.0,
              scan_timeout_s_);
          }
        }
      } else if (front_obstacle_distance_m_ <= stop_distance) {
        obstacle_clear_started_.reset();
        set_state(kObstacleStop, "front obstacle detected");
      }
    } else if (state_ == kObstacleStop) {
      const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
      const YAML::Node event = events_[id];
      const double clear_distance = yaml_value<double>(
        event, "obstacle_clear_distance_m", default_obstacle_clear_distance_m_);
      const double hold_s = yaml_value<double>(
        event, "obstacle_clear_hold_s", default_obstacle_clear_hold_s_);
      const bool scan_fresh =
        scan_received_ && (t - scan_time_s_ <= scan_timeout_s_);
      if (scan_fresh && front_obstacle_distance_m_ >= clear_distance) {
        if (!obstacle_clear_started_) {obstacle_clear_started_ = std::chrono::steady_clock::now();}
        const double clear_elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - *obstacle_clear_started_).count();
        if (clear_elapsed >= hold_s) {
          obstacle_clear_started_.reset();
          set_state(kAccelObstacleZone, "obstacle cleared");
        }
      } else {obstacle_clear_started_.reset();}
    } else if (state_ == kParkingTZoneApproach || state_ == kParkingParallelApproach) {
      if (!active_event_index_ || !parking_entry_path_) {reset_to_lane_follow("no active parking event");}
      else {
        publish_parking_entry_path();
        const double heading_tolerance_rad = parking_entry_heading_tolerance_deg_ * parking_cpp::kPi / 180.0;
        if (current_pose_ && parking_entry_end_ &&
          std::hypot(current_pose_->x - parking_entry_end_->first,
            current_pose_->y - parking_entry_end_->second) <= parking_entry_tolerance_m_ &&
          std::abs(parking_cpp::normalize_angle(current_pose_->yaw - parking_entry_end_yaw_)) <=
            heading_tolerance_rad) {
          const char * maneuver_state =
            state_ == kParkingTZoneApproach ? kParkingTZoneManeuver : kParkingParallelManeuver;
          enter_parking_maneuver(maneuver_state);
        }
      }
    } else if (state_ == kParkingTZoneManeuver || state_ == kParkingParallelManeuver) {
      if (!active_event_index_ || !parking_spot_path_) {reset_to_lane_follow("no active parking maneuver");}
      else {
        publish_parking_maneuver();
        // Rear stop-line detection (the bay's back wall, seen while backing toward it) is a
        // direct physical measurement, not a replay of wherever spot_path's last recorded point
        // happened to be (which drifts with anchor/GNSS error) -- so whenever it's actively
        // seeing the wall, it alone governs completion, not just "whichever check passes first".
        // The position-based check only applies as a fallback for whenever the camera currently
        // isn't seeing anything (never detected yet, view lost, or this event has no rear
        // stop-line at all), so the maneuver still has a way to complete without the camera.
        const bool rear_fresh = rear_stop_line_fresh(t);
        const bool rear_stop_reached = rear_fresh && rear_stop_line_distance_m_ <= parking_rear_stop_distance_m_;
        const bool position_reached = current_pose_ && parking_spot_end_ &&
          std::hypot(current_pose_->x - parking_spot_end_->first,
            current_pose_->y - parking_spot_end_->second) <= parking_exit_tolerance_m_;
        if (rear_fresh ? rear_stop_reached : position_reached) {
          const char * hold_state =
            state_ == kParkingTZoneManeuver ? kParkingTZoneHold : kParkingParallelHold;
          begin_parking_hold(hold_state);
        }
      }
    } else if (state_ == kParkingTZoneHold || state_ == kParkingParallelHold) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - parking_hold_started_).count();
      if (elapsed >= parking_hold_duration_current_s_) {complete_active_event("parking completed");}
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
  double approach_cancel_margin_m_{1.5};
  double default_hill_stop_duration_s_{5.0};
  double hill_stop_duration_current_s_{5.0};
  double scan_timeout_s_{2.0};
  double scan_startup_grace_s_{3.0};
  bool stop_on_lidar_timeout_{false};
  double obstacle_min_distance_m_{0.40};
  double default_accel_zone_speed_mps_{4.0};
  double default_obstacle_stop_distance_m_{2.0};
  double default_obstacle_clear_distance_m_{2.5};
  double default_obstacle_half_width_m_{0.55};
  double default_obstacle_clear_hold_s_{1.0};
  double parking_entry_tolerance_m_{0.35};
  double parking_entry_heading_tolerance_deg_{15.0};
  double parking_exit_tolerance_m_{0.25};
  double parking_rear_stop_distance_m_{0.30};
  double default_parking_hold_duration_s_{0.0};
  double parking_hold_duration_current_s_{0.0};
  std::string default_mission_{"straight"};
  std::string mission_{"straight"};
  bool left_on_green_allowed_{false};
  bool right_on_green_allowed_{true};
  bool arrow_signal_selects_path_{true};

  YAML::Node events_;
  YAML::Node paths_;
  std::string state_{kLeftLaneFollow};
  std::size_t cruise_index_{0};
  std::optional<std::size_t> active_event_index_;
  std::optional<nav_msgs::msg::Path> transformed_path_;
  std::optional<std::pair<double, double>> bridge_end_;
  std::chrono::steady_clock::time_point bridge_started_{};
  std::chrono::steady_clock::time_point hill_stop_started_{};
  std::optional<std::chrono::steady_clock::time_point> obstacle_clear_started_;

  std::optional<nav_msgs::msg::Path> parking_entry_path_;
  std::optional<std::pair<double, double>> parking_entry_end_;
  double parking_entry_end_yaw_{0.0};
  std::optional<nav_msgs::msg::Path> parking_spot_path_;
  std::vector<int8_t> parking_spot_gear_;
  std::optional<std::pair<double, double>> parking_spot_end_;
  std::chrono::steady_clock::time_point parking_hold_started_{};

  std::optional<GpsPoint> current_gps_;
  std::optional<Pose2D> current_pose_;
  double gps_time_s_{-1e9};
  double odom_time_s_{-1e9};
  bool stop_line_detected_{false};
  double stop_line_distance_m_{std::numeric_limits<double>::infinity()};
  double stopline_time_s_{-1e9};
  bool rear_stop_line_detected_{false};
  double rear_stop_line_distance_m_{std::numeric_limits<double>::infinity()};
  double rear_stopline_time_s_{-1e9};
  std::string sign_{"none"};
  double sign_time_s_{-1e9};
  bool scan_received_{false};
  double scan_time_s_{-1e9};
  double accel_zone_enter_time_s_{-1e9};
  double front_obstacle_distance_m_{std::numeric_limits<double>::infinity()};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stopline_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rear_stopline_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sign_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bridge_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_event_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr parking_entry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr parking_spot_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr parking_gear_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorSupervisorWithParking>());
  rclcpp::shutdown();
  return 0;
}