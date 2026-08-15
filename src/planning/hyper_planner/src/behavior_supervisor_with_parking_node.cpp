#include "hyper_planner/common.hpp"

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
#include <std_msgs/msg/float64.hpp>
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
// Lane-following through a static-obstacle zone (e.g. an S-curve on the course) while laterally
// biasing off lane-center by whatever compute_avoid_offset() computes from /scan each tick, enough
// to clear the vehicle's own half-width plus a margin. Deliberately reuses the literal "AVOID"
// string controller_with_parking.cpp's control_step() already special-cases (mode_ == "AVOID"),
// so no controller-side mode-classification change was needed -- only a dynamic offset topic feeding
// what used to be that branch's fixed avoid_offset_bias parameter.
constexpr const char * kAvoidObstacleZone = "AVOID";
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
  kAvoidObstacle,
  kLaneChange4, // Left lane -> Right lane
  kLaneChange5, // Left lane -> Right lane
  kParkingTZone,
  kParkingParallel,
  kEnd,
};

// TEST ORDERING: kAvoidObstacle and kParkingTZone (plus the right-lane cruise slots they expect to
// be approached from) have both been pulled to the very front, kAvoidObstacle first, so a test run
// hits the AVOID trigger immediately instead of driving the whole course first -- kParkingTZone was
// already here for the same reason (see its own note below) and is left second. Their real
// competition positions are elsewhere (kParkingTZone: right after intersection_B; kAvoidObstacle:
// wherever the actual S-curve is on the course, still unsurveyed) -- move both back there (and drop
// the duplicated kCruiseRight/kAvoidObstacle/kCruiseRight/kParkingTZone at the front, restoring
// kCruiseLeft as the first slot and the default state_ back to kLeftLaneFollow) before the actual
// run. Repositioning either slot is otherwise a one-line reorder -- nothing else depends on position
// beyond "cruise slot right before" (the lane side the approach starts from) and "cruise slot right
// after" (the lane side driving resumes in once the event completes).
//
// NOTE: kParkingParallel's slot position below is still a placeholder (near course end) pending its
// actual course location.
const std::vector<SlotKind> kCourseSequence = {
  SlotKind::kCruiseRight,
  SlotKind::kAvoidObstacle,
  SlotKind::kCruiseRight,
  SlotKind::kParkingTZone,
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
    case SlotKind::kAvoidObstacle: return "avoid_obstacle_1";
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

// Result of compute_avoid_offset(): whether an obstacle is currently in the scan corridor at all,
// whether it spans too much of the corridor to safely dodge either way, and (if not) how far to
// bias off lane-center to clear it.
// nearest_x/y_min/y_max are only meaningful when obstacle_present -- carried along purely so
// tick()'s AVOID branch can log what was actually seen, for diagnosing "is it even detecting
// anything" separately from "is it choosing not to dodge."
struct AvoidResult
{
  bool obstacle_present{false};
  bool blocked{false};
  double offset_bias{0.0};
  double nearest_x{0.0};
  double y_min{0.0};
  double y_max{0.0};
};

// Applies a local (dx, dy, dyaw) offset -- expressed in `base`'s own frame -- on top of `base`,
// the same rigid-body transform transform_relative_path() applies per-point. Used to correct the
// parking entry_path anchor: `base` is the closest-GPS-approach pose (an approximation of where
// the vehicle is when passing the mark_parking GPS fix), and the offset is the recorded, exact
// displacement from that mark_parking pose to wherever record_start:entry was actually called
// during recording -- since those two points are generally not the same physical location.
Pose2D compose_pose(const Pose2D & base, double dx, double dy, double dyaw)
{
  const double c = std::cos(base.yaw);
  const double s = std::sin(base.yaw);
  return Pose2D{
    base.x + c * dx - s * dy,
    base.y + s * dx + c * dy,
    hyper_planner::normalize_angle(base.yaw + dyaw),
  };
}

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
    // Minimum odom-measured distance backed up since the maneuver started before the rear-camera
    // check above is trusted -- without this, a misdetection right at the bay entrance (e.g. the
    // entry threshold marking, seen the instant the rear camera is pointed backward) reads as
    // "arrived" on the maneuver's very first tick, before any real reverse progress.
    parking_rear_stop_min_progress_m_ =
      declare_parameter<double>("parking_rear_stop_min_progress_m", 1.0);
    default_parking_hold_duration_s_ = declare_parameter<double>("parking_hold_duration_s", 0.0);

    // AVOID zone: lane-following through a static-obstacle section (e.g. an S-curve) while
    // steering laterally around whatever compute_avoid_offset() finds in /scan. See that function's
    // own comment for the corridor/clustering/Minkowski-inflation details.
    // Wide on purpose, two reasons. (1) Through an S-curve the vehicle's heading is constantly
    // changing relative to the road, so a static obstacle can sit at a large angle off the *current*
    // heading while still being squarely in the upcoming path -- narrowing this (tried 30 deg during
    // tuning) caused the obstacle to only enter the filter once already close, leaving too little
    // distance/time left to actually execute the lateral dodge (looked like "not detecting it" but
    // was really "detecting it too late"). (2) Geometrically, fully covering the avoid_lookahead_m_ x
    // avoid_corridor_half_width_m_ detection rectangle at its nearest edge (x = obstacle_min_distance_m_)
    // requires atan2(avoid_corridor_half_width_m_, obstacle_min_distance_m_) ~= 79 deg with the
    // current defaults (2.0m / 0.4m) -- anything narrower leaves a blind wedge at the near corners
    // of that rectangle. 100 deg covers that with margin; going wider than needed doesn't add false
    // detections since obstacle_min_distance_m_/avoid_lookahead_m_ already reject any point whose x
    // falls outside the rectangle regardless of angle.
    avoid_scan_half_angle_deg_ = declare_parameter<double>("avoid_scan_half_angle_deg", 100.0);
    // Raised from 4.0m for more reaction distance/time to actually complete a lateral dodge before
    // reaching the obstacle, now that the LIDAR itself also refreshes faster (see vehicle.xacro's
    // lidar_sensor update_rate).
    avoid_lookahead_m_ = declare_parameter<double>("avoid_lookahead_m", 6.0);
    // Widened from 1.3m: measured live against a wide obstacle, y_max sat pinned at ~1.3 (the old
    // filter edge) for the entire approach while y_min kept growing more negative as the vehicle
    // closed in -- the filter was clipping the obstacle's true far edge, so compute_avoid_offset()
    // never saw its real extent and kept picking/re-picking a dodge direction off incomplete data,
    // undershooting as the "hidden" side turned out to need more room than it looked like from far
    // away. 2.0m reaches past the ~3m lane's own half-width (1.5m, see driving_course/model.sdf's
    // "3 m lanes" comment) so a genuinely wide obstacle's far edge is visible well before close range
    // -- note this also makes it more likely to pick up off-lane clutter (e.g. track-edge decoration)
    // as part of a cluster; if that becomes the next problem, it needs its own fix (e.g. discounting
    // points beyond the lane's own edge) rather than narrowing this back down.
    avoid_corridor_half_width_m_ = declare_parameter<double>("avoid_corridor_half_width_m", 2.0);
    avoid_margin_m_ = declare_parameter<double>("avoid_margin_m", 0.05);
    // How much earlier (in meters of lateral clearance) to start steering away, before the obstacle
    // is *literally* already overlapping the vehicle's straight-ahead footprint -- see its use in
    // compute_avoid_offset()'s overlap check. 0.3 was tried first and measured live: the obstacle
    // was visible from x=2.88m out, but attention_half_width (vehicle_half_width_m_ + this) wasn't
    // crossed until x=1.04m -- under 1m of travel left to actually execute the lateral dodge, too
    // late. Raised so the dodge starts reacting as soon as a cluster is anywhere near the vehicle's
    // path, instead of waiting for it to swing close to center first. Independent of
    // avoid_corridor_half_width_m_ (which only controls how far off-center a point is even looked
    // at, not when a detected one triggers a reaction).
    avoid_anticipation_m_ = declare_parameter<double>("avoid_anticipation_m", 0.9);
    // How long to keep holding the last commanded dodge offset after a tick's raw geometry first
    // says "clear," before actually releasing back to lane-center -- see its use in the AVOID tick
    // branch. Without this, the dodge cuts off the moment the obstacle's LIDAR-frame bearing swings
    // wide (which happens quickly once the vehicle itself starts turning away, well before it has
    // actually passed the obstacle in the real world), producing a brief steer-then-snap-back
    // instead of a full pass.
    avoid_hold_s_ = declare_parameter<double>("avoid_hold_s", 1.2);
    // How fast (m/s of offset_bias change) to ramp back toward 0 once avoid_hold_s_ has elapsed --
    // see the release branch in the AVOID tick for why an instant jump to 0 is unsafe. At the
    // default 0.5 m/s, releasing from avoid_decisive_offset_m_ (0.7m) takes ~1.4s.
    avoid_release_rate_m_s_ = declare_parameter<double>("avoid_release_rate_m_s", 0.5);
    // Must be large enough that a dodge using the corridor's full available width (up to
    // avoid_corridor_half_width_m_ minus the vehicle's own inflated half-width) isn't needlessly
    // rejected as "blocked" by this ceiling alone -- see the clamp-vs-blocked comment in
    // compute_avoid_offset() below for why this isn't just clamped instead. 1.0m leaves the vehicle
    // body ~0.1m clear of the opposite lane line (lane half-width 1.5m minus vehicle half-width
    // 0.405m minus this) even if a dodge uses the full ceiling -- tight, but real obstacles observed
    // during tuning needed close to this much.
    avoid_max_offset_m_ = declare_parameter<double>("avoid_max_offset_m", 1.0);
    // Once compute_avoid_offset() decides a dodge is warranted, it commits to at least this much
    // clearance in the chosen direction (hugging closer to that side's lane line) instead of the bare
    // minimum -- see the comment at its use for why a marginal nudge is fragile. Kept comfortably
    // under avoid_max_offset_m_ so it's never itself the reason a genuinely wide obstacle reports
    // blocked.
    avoid_decisive_offset_m_ = declare_parameter<double>("avoid_decisive_offset_m", 0.7);
    // Real half-width used to inflate the obstacle (Minkowski-sum style) so the *body* clears it,
    // not just its centerline -- matches hyper_control/config/parameters.yaml's body_width/2 (0.81/2).
    vehicle_half_width_m_ = declare_parameter<double>("vehicle_half_width_m", 0.405);
    avoid_cluster_break_m_ = declare_parameter<double>("avoid_cluster_break_m", 0.3);

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
    avoid_offset_pub_ = create_publisher<std_msgs::msg::Float64>("/avoid/offset_bias", 10);

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
      hyper_planner::yaw_from_quaternion(msg->pose.pose.orientation)};
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
    // Cached whole so compute_avoid_offset() can re-scan a wider window (and cluster by angle) once
    // per tick, independent of the single symmetric-corridor scalar this callback also computes
    // below for the accel/obstacle zone.
    last_scan_ = msg;
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

  // Finds the nearest obstacle ahead within a wide forward corridor and decides how far to bias
  // off lane-center to clear it -- unlike on_scan()'s single symmetric-corridor scalar (which only
  // answers "is anything within a fixed half-width," used for the accel zone's stop/go decision),
  // this needs the obstacle's actual left/right extent to steer *around* it, not just detect it.
  //
  // Points within +/-avoid_scan_half_angle_deg_ and avoid_lookahead_m_/avoid_corridor_half_width_m_
  // are grouped into clusters by consecutive-point gap (a jump > avoid_cluster_break_m_ starts a new
  // cluster) and the closest cluster ahead is treated as *the* obstacle -- adequate for a single
  // static obstacle in one S-curve, not a general multi-obstacle planner. avoid_corridor_half_width_m_
  // here is *only* a point filter (how far off-center a return still counts as "worth looking at") --
  // it is not a real wall, so it must not also gate whether a dodge is feasible (see below).
  //
  // Required offset is sized like a Minkowski-sum inflation: the vehicle is treated as a point
  // steering around an obstacle inflated by the vehicle's own half-width plus avoid_margin_m_, so
  // the *body* (not just its centerline) clears the obstacle by that margin. The only real limit on
  // how far that offset can be is avoid_max_offset_m_ (how far off lane-center the course/lane
  // actually allows) -- an earlier version of this function also rejected a dodge whenever neither
  // side had enough clearance *within avoid_corridor_half_width_m_*, which silently treated that
  // arbitrary point-filter width as if it were the edge of the drivable lane and reported `blocked`
  // for perfectly dodgeable obstacles. Fixed by dropping that check entirely.
  AvoidResult compute_avoid_offset() const
  {
    AvoidResult result;
    if (!last_scan_) {return result;}
    const sensor_msgs::msg::LaserScan & scan = *last_scan_;
    const double half_angle = avoid_scan_half_angle_deg_ * hyper_planner::kPi / 180.0;

    struct Pt {double x; double y;};
    std::vector<Pt> points;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {continue;}
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (std::abs(angle) > half_angle) {continue;}
      const double x = range * std::cos(angle);
      const double y = range * std::sin(angle);
      if (x < obstacle_min_distance_m_ || x > avoid_lookahead_m_) {continue;}
      if (std::abs(y) > avoid_corridor_half_width_m_) {continue;}
      points.push_back({x, y});
    }
    if (points.empty()) {return result;}

    // scan.ranges is already angle-ordered, and the filter loop above preserves that order, so
    // consecutive entries in `points` are consecutive scan angles -- a real gap between them (not
    // just an angular one) means a break between separate objects.
    std::size_t cluster_start = 0, best_start = 0, best_end = 0;
    double best_min_x = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i <= points.size(); ++i) {
      const bool boundary = i == points.size() ||
        std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y) > avoid_cluster_break_m_;
      if (!boundary) {continue;}
      double min_x = std::numeric_limits<double>::infinity();
      for (std::size_t j = cluster_start; j < i; ++j) {min_x = std::min(min_x, points[j].x);}
      if (min_x < best_min_x) {best_min_x = min_x; best_start = cluster_start; best_end = i;}
      cluster_start = i;
    }

    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for (std::size_t j = best_start; j < best_end; ++j) {
      y_min = std::min(y_min, points[j].y);
      y_max = std::max(y_max, points[j].y);
    }
    result.obstacle_present = true;
    result.nearest_x = best_min_x;
    result.y_min = y_min;
    result.y_max = y_max;

    // Does the obstacle overlap the vehicle's current (unbiased) straight-line footprint, widened by
    // avoid_anticipation_m_ so the dodge starts a bit before the obstacle is *literally* already in
    // the way -- waiting for exact overlap means reacting later (and closer) than it sounds once
    // LIDAR refresh latency and stopping/steering distance are accounted for. If it doesn't even
    // overlap this widened zone, no dodge is needed yet -- offset stays 0 (e.g. still far off to one
    // side, or already fully passed).
    const double attention_half_width = vehicle_half_width_m_ + avoid_anticipation_m_;
    if (y_min > attention_half_width || y_max < -attention_half_width) {return result;}

    const double required_half_width = vehicle_half_width_m_ + avoid_margin_m_;
    const double offset_if_right = y_min - required_half_width;  // dodge right (toward -y)
    const double offset_if_left = y_max + required_half_width;   // dodge left (toward +y)
    // Prefer whichever direction needs the smaller bias -- equivalently, whichever side the
    // obstacle occupies less of.
    const bool go_right = std::abs(offset_if_right) <= std::abs(offset_if_left);
    double offset = go_right ? offset_if_right : offset_if_left;

    // Once a dodge is actually warranted, commit to at least avoid_decisive_offset_m_ of clearance
    // in the chosen direction -- hugging closer to that side's lane line -- rather than the bare
    // minimum required_half_width. A minimal nudge is fragile: it rides right at the edge of the
    // margin, so any LIDAR noise, clustering error, or the obstacle's own extent being clipped by
    // avoid_corridor_half_width_m_ can leave it under-cleared. This only ever widens the offset, never
    // shrinks it below what's actually needed for a wider obstacle -- the overlap check above already
    // decided a dodge is warranted; this just makes the magnitude decisive instead of marginal.
    if (go_right) {offset = std::min(offset, -avoid_decisive_offset_m_);}
    else {offset = std::max(offset, avoid_decisive_offset_m_);}

    // avoid_max_offset_m_ is the one real limit on how far off lane-center to bias (how much the
    // course/lane actually allows), so exceeding it means neither direction is safely dodgeable --
    // clamping instead would silently under-clear the obstacle (drive the vehicle body closer than
    // avoid_margin_m_ intends) rather than failing safe.
    if (std::abs(offset) > avoid_max_offset_m_) {
      result.blocked = true;
      return result;
    }
    result.offset_bias = offset;
    return result;
  }

  void publish_avoid_offset()
  {
    std_msgs::msg::Float64 msg;
    msg.data = avoid_offset_bias_;
    avoid_offset_pub_->publish(msg);
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
    if (type == "avoid_obstacle" || type == "s_curve_avoid") {return "avoid_obstacle";}
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
      // Any zone with a "start" GPS point (currently accel_obstacle and avoid_obstacle) is entered
      // via that point rather than the event's own top-level lat/lon -- keyed off the node's actual
      // presence rather than event_type() so this doesn't need a new branch per zone type added.
      const YAML::Node point = event["start"] ? event["start"] : event;
      return hyper_planner::haversine_m(current_gps_->latitude, current_gps_->longitude,
        point["latitude"].as<double>(), point["longitude"].as<double>());
    } catch (const YAML::Exception &) {return std::numeric_limits<double>::infinity();}
  }

  double distance_to_accel_end(const std::string & event_id) const
  {
    if (!current_gps_) {return std::numeric_limits<double>::infinity();}
    try {
      const YAML::Node end = events_[event_id]["end"];
      if (!end) {return std::numeric_limits<double>::infinity();}
      return hyper_planner::haversine_m(current_gps_->latitude, current_gps_->longitude,
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
    const bool zone_event = kind == SlotKind::kAccelObstacle || kind == SlotKind::kAvoidObstacle;
    const double radius = zone_event ?
      yaml_value<double>(events_[id], "start_radius_m",
        yaml_value<double>(events_[id], "approach_radius_m", 2.5)) :
      yaml_value<double>(events_[id], "approach_radius_m", 2.5);
    return distance_to_event(id) <= radius ? std::optional<std::size_t>(next_index) : std::nullopt;
  }

  // Unlike other events (triggered the instant GPS first enters approach_radius_m), parking
  // instead tracks the closest GPS approach within that radius and activates once the vehicle has
  // passed it (distance starts increasing again) -- this closest-approach pose approximates
  // wherever the vehicle physically is when passing the mark_parking GPS fix.
  //
  // That approximates the mark_parking *pose*, but the recorded path's local origin is wherever the
  // human operator separately issued record_start during recording -- a different physical point,
  // in an arbitrary direction/distance from mark_parking, entirely up to when the operator happened
  // to call each command. Treating the closest-approach pose itself as that origin (as if the two
  // coincided) would bake that whole gap into every replay -- not just GPS/odometry noise, but a
  // systematic bias from however far apart those two commands happened to be issued during
  // recording. That's what `entry_trigger_offset`/`spot_trigger_offset` (dx/dy/dyaw, recorded by
  // waypoint_recorder.py at record_start time, relative to the mark_parking pose) correct for:
  // compose it onto the closest-approach pose to get the actual path anchor, leaving only
  // GPS/odometry noise as the remaining error.
  //
  // Which offset applies depends on which path this slot actually starts with: if entry_path is
  // recorded, entry_trigger_offset anchors it (see enter_parking_slot() below). If entry_path is
  // absent -- the approach has a real lane, so ordinary lane-following gets the vehicle to the
  // parking-start point without a replayed path -- this trigger jumps straight into spot_path, so
  // spot_trigger_offset anchors that instead. Events recorded before these fields existed simply
  // have no such node, so yaml_value's fallback keeps the old (uncorrected) behavior for them.
  void check_parking_gps_approach(std::size_t next_index)
  {
    const SlotKind kind = kCourseSequence[next_index];
    const std::string id = event_id_for(kind);
    const double radius = yaml_value<double>(events_[id], "approach_radius_m", 2.5);
    const double distance = distance_to_event(id);

    if (distance > radius) {
      parking_closest_distance_m_.reset();
      parking_closest_anchor_pose_.reset();
      return;
    }

    if (!parking_closest_distance_m_ || distance < *parking_closest_distance_m_) {
      parking_closest_distance_m_ = distance;
      parking_closest_anchor_pose_ = current_pose_;
      return;
    }

    // Distance increased relative to the closest point seen so far -- we've now passed it.
    const std::optional<Pose2D> anchor = parking_closest_anchor_pose_;
    parking_closest_distance_m_.reset();
    parking_closest_anchor_pose_.reset();

    std::optional<Pose2D> corrected_anchor = anchor;
    if (anchor) {
      const bool has_entry_path = static_cast<bool>(events_[id]["entry_path"]);
      const YAML::Node offset =
        events_[id][has_entry_path ? "entry_trigger_offset" : "spot_trigger_offset"];
      const double dx = yaml_value<double>(offset, "dx", 0.0);
      const double dy = yaml_value<double>(offset, "dy", 0.0);
      const double dyaw = yaml_value<double>(offset, "dyaw", 0.0);
      corrected_anchor = compose_pose(*anchor, dx, dy, dyaw);
    }
    activate_event(next_index, "closest GPS approach", corrected_anchor);
  }

  // `anchor_pose`, when given, overrides current_pose_ as the parking entry_path's local origin --
  // used for the closest-GPS-approach pose captured by check_parking_gps_approach() above. Ignored
  // by every non-parking event kind.
  void activate_event(
    std::size_t index, const std::string & reason,
    const std::optional<Pose2D> & anchor_pose = std::nullopt)
  {
    const SlotKind kind = kCourseSequence[index];
    const std::string id = event_id_for(kind);
    const std::string type = event_type(id);
    if (type == "accel_obstacle" || type == "avoid_obstacle") {
      const YAML::Node event = events_[id];
      if (!event["start"] || !event["end"]) {
        RCLCPP_WARN(get_logger(), "Zone event '%s' requires both start and end GPS points", id.c_str());
        return;
      }
    }

    active_event_index_ = index;

    if (kind == SlotKind::kHillstop) {set_state(kHillApproach, reason);}
    else if (kind == SlotKind::kAccelObstacle) {
      obstacle_clear_started_.reset();
      accel_zone_enter_time_s_ = now_s();
      set_state(kAccelObstacleZone, reason);
    } else if (kind == SlotKind::kAvoidObstacle) {
      avoid_offset_bias_ = 0.0;
      avoid_hold_until_s_ = -1e9;
      accel_zone_enter_time_s_ = now_s();
      set_state(kAvoidObstacleZone, reason);
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
    else if (kind == SlotKind::kParkingTZone) {
      enter_parking_slot(kParkingTZoneApproach, kParkingTZoneManeuver, anchor_pose);
    }
    else if (kind == SlotKind::kParkingParallel) {
      enter_parking_slot(kParkingParallelApproach, kParkingParallelManeuver, anchor_pose);
    }
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

  // `anchor_override`, when given, is used as the path's local origin instead of current_pose_ --
  // for parking events this is the pose captured at closest GPS approach (see
  // next_parking_trigger_pose()), not wherever current_pose_ happens to be when this function is
  // called, since by the time an event actually activates the vehicle has typically already moved
  // past that closest point.
  std::optional<nav_msgs::msg::Path> transform_relative_path(
    const YAML::Node & points, const std::optional<Pose2D> & anchor_override = std::nullopt)
  {
    const std::optional<Pose2D> & anchor = anchor_override ? anchor_override : current_pose_;
    if (!anchor || !points || !points.IsSequence()) {return std::nullopt;}
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    const double c = std::cos(anchor->yaw);
    const double s = std::sin(anchor->yaw);
    for (const auto & point : points) {
      try {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        const double lx = point["x"].as<double>();
        const double ly = point["y"].as<double>();
        const double lyaw = point["yaw"] ? point["yaw"].as<double>() : 0.0;
        pose.pose.position.x = anchor->x + c * lx - s * ly;
        pose.pose.position.y = anchor->y + s * lx + c * ly;
        pose.pose.orientation = hyper_planner::quaternion_from_yaw(
          hyper_planner::normalize_angle(anchor->yaw + lyaw));
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
  void enter_parking_approach(const char * approach_state, const std::optional<Pose2D> & anchor_pose)
  {
    if (!active_event_index_) {reset_to_lane_follow("parking requested without event"); return;}
    const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
    const YAML::Node event = events_[id];
    const YAML::Node key_node = event["entry_path"];
    if (!key_node) {reset_to_lane_follow("missing parking entry_path"); return;}
    const std::string key = key_node.as<std::string>();
    const auto transformed = transform_relative_path(paths_[key], anchor_pose);
    if (!transformed) {reset_to_lane_follow("empty parking entry_path"); return;}
    parking_entry_path_ = *transformed;
    const auto & last = parking_entry_path_->poses.back();
    parking_entry_end_ = std::make_pair(last.pose.position.x, last.pose.position.y);
    parking_entry_end_yaw_ = hyper_planner::yaw_from_quaternion(last.pose.orientation);
    set_state(approach_state, "path=" + key);
  }

  // Dispatches a just-triggered parking slot to whichever of the two flows it's recorded for. If
  // entry_path exists, drives it open-loop first (enter_parking_approach, as above). If it doesn't
  // -- because the approach to this spot has a real lane, so ordinary lane-following (whatever
  // cruise state was already active) gets the vehicle there without needing a replayed path --
  // skips straight into the maneuver, anchoring spot_path directly off the corrected GPS-trigger
  // pose (`anchor_pose`, already composed with spot_trigger_offset by check_parking_gps_approach)
  // instead of the live arrival pose enter_parking_maneuver() otherwise uses.
  void enter_parking_slot(
    const char * approach_state, const char * maneuver_state,
    const std::optional<Pose2D> & anchor_pose)
  {
    if (!active_event_index_) {reset_to_lane_follow("parking requested without event"); return;}
    const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
    if (events_[id]["entry_path"]) {
      enter_parking_approach(approach_state, anchor_pose);
    } else {
      enter_parking_maneuver(maneuver_state, anchor_pose);
    }
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
  // propagate into the reverse maneuver. `anchor_pose`, when given, overrides that live-pose anchor
  // -- used only by enter_parking_slot() below when there is no entry_path to arrive via (the
  // approach to this spot is ordinary lane-following instead), so the vehicle jumps straight from
  // the GPS trigger into the maneuver with no arrival step to re-anchor from.
  void enter_parking_maneuver(
    const char * maneuver_state, const std::optional<Pose2D> & anchor_pose = std::nullopt)
  {
    if (!active_event_index_) {reset_to_lane_follow("parking maneuver without event"); return;}
    const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
    const YAML::Node event = events_[id];
    const YAML::Node key_node = event["spot_path"];
    if (!key_node) {reset_to_lane_follow("missing parking spot_path"); return;}
    const std::string key = key_node.as<std::string>();
    const auto transformed = transform_relative_path(paths_[key], anchor_pose);
    if (!transformed) {reset_to_lane_follow("empty parking spot_path"); return;}
    parking_spot_path_ = *transformed;
    parking_spot_gear_ = extract_gear_array(paths_[key]);
    const auto & last = parking_spot_path_->poses.back();
    parking_spot_end_ = std::make_pair(last.pose.position.x, last.pose.position.y);
    // Captured so the rear-camera completion check (below) can require real reverse progress
    // before trusting it -- otherwise a misdetection right at the bay entrance (e.g. the entry
    // threshold marking itself, seen by the rear camera the instant it's pointed backward) reads as
    // "arrived" on literally the first tick of the maneuver, before the vehicle has moved at all.
    parking_maneuver_start_pos_ = current_pose_ ?
      std::make_optional(std::make_pair(current_pose_->x, current_pose_->y)) : std::nullopt;
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
    parking_maneuver_start_pos_.reset();
    parking_closest_distance_m_.reset();
    parking_closest_anchor_pose_.reset();
    // Defense in depth alongside the controller's own freshness timeout on this topic (see
    // on_avoid_offset() there) -- don't leave a stale nonzero bias published once this event is no
    // longer active, even though a non-AVOID state would eventually be ignored on the controller
    // side too.
    avoid_offset_bias_ = 0.0;
    avoid_hold_until_s_ = -1e9;
    publish_avoid_offset();
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
        const std::size_t next_index = cruise_index_ + 1U;
        const bool next_is_parking = next_index < kCourseSequence.size() &&
          (kCourseSequence[next_index] == SlotKind::kParkingTZone ||
            kCourseSequence[next_index] == SlotKind::kParkingParallel);
        if (next_is_parking) {
          check_parking_gps_approach(next_index);
        } else {
          const auto ready_index = next_event_index_in_range();
          if (ready_index) {activate_event(*ready_index, "entered GPS event radius");}
        }
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
          // AVOID zones reach OBSTACLE_STOP only via compute_avoid_offset() reporting `blocked`
          // (see the AVOID branch below) -- resume there instead of the accel zone's state so
          // lane-following-with-dodge continues, rather than getting stuck in a state that expects
          // the accel zone's own stop/clear-distance signal.
          const char * resume_state =
            event_type(id) == "avoid_obstacle" ? kAvoidObstacleZone : kAccelObstacleZone;
          set_state(resume_state, "obstacle cleared");
        }
      } else {obstacle_clear_started_.reset();}
    } else if (state_ == kAvoidObstacleZone) {
      const std::string id = event_id_for(kCourseSequence[*active_event_index_]);
      const YAML::Node event = events_[id];
      const double end_radius = yaml_value<double>(event, "end_radius_m", 2.5);
      const double end_distance = distance_to_accel_end(id);  // generic "end" GPS lookup, reused as-is
      const bool scan_fresh = scan_received_ && (t - scan_time_s_ <= scan_timeout_s_);

      if (end_distance <= end_radius) {
        avoid_offset_bias_ = 0.0;
        publish_avoid_offset();
        complete_active_event("avoid obstacle zone end reached");
      } else if (!scan_fresh) {
        // No fresh LIDAR to steer around anything safely -- fall back to plain lane-center tracking
        // rather than holding a stale offset_bias.
        avoid_offset_bias_ = 0.0;
        publish_avoid_offset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "AVOID: no fresh /scan data.");
      } else {
        const AvoidResult result = compute_avoid_offset();
        if (!result.obstacle_present) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "AVOID: no obstacle in scan corridor (angle=+-%.0fdeg, range=%.1f..%.1fm, y<=%.2fm).",
            avoid_scan_half_angle_deg_, obstacle_min_distance_m_, avoid_lookahead_m_,
            avoid_corridor_half_width_m_);
        } else if (result.blocked) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "AVOID: obstacle at x=%.2fm y=[%.2f,%.2f] -- BLOCKED, needed offset exceeds "
            "avoid_max_offset_m (%.2fm).",
            result.nearest_x, result.y_min, result.y_max, avoid_max_offset_m_);
        } else if (std::abs(result.offset_bias) > 1e-6) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "AVOID: obstacle at x=%.2fm y=[%.2f,%.2f] -- dodging, offset_bias=%.3fm.",
            result.nearest_x, result.y_min, result.y_max, result.offset_bias);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "AVOID: obstacle at x=%.2fm y=[%.2f,%.2f] -- not yet in path, no dodge needed.",
            result.nearest_x, result.y_min, result.y_max);
        }
        if (result.blocked) {
          avoid_offset_bias_ = 0.0;
          publish_avoid_offset();
          obstacle_clear_started_.reset();
          set_state(kObstacleStop, "avoid obstacle blocks entire corridor, cannot dodge safely");
        } else if (std::abs(result.offset_bias) > 1e-6) {
          // Actively needed this tick -- keep the hold window refreshed so releasing (below) always
          // waits avoid_hold_s_ from the *last* tick a dodge was actually required, not from
          // whenever the dodge first started.
          avoid_offset_bias_ = result.offset_bias;
          avoid_hold_until_s_ = t + avoid_hold_s_;
          publish_avoid_offset();
        } else if (t < avoid_hold_until_s_) {
          // This tick's raw geometry says "clear," but we were dodging recently -- once the vehicle
          // itself is turning away, the obstacle's bearing in the vehicle-relative LIDAR frame swings
          // wide well before the vehicle has actually finished passing it in the real world, so
          // dropping straight to 0 here would cut the maneuver short (steer briefly, then snap back
          // to lane-center while still alongside the obstacle). Hold the last commanded offset
          // instead until avoid_hold_s_ has elapsed since the last tick that actually needed one.
          publish_avoid_offset();
        } else {
          // Ramp back toward lane-center instead of snapping straight to 0 -- the vehicle has
          // actually been riding avoid_decisive_offset_m_-ish off true lane-center for a while, so an
          // instant jump to bias=0 here creates a large sudden cross-track error against the *real*
          // lane, which ordinary Stanley correction then reads as needing a hard correction. Measured
          // live: this pinned steering at max_steering_angle_ for over a second right after release,
          // and that violent swing-back is itself what caused a second, closely-following obstacle to
          // be missed -- not a detection or dodge-direction problem at all.
          const double step = avoid_release_rate_m_s_ / std::max(1.0, tick_hz_);
          if (avoid_offset_bias_ > step) {avoid_offset_bias_ -= step;}
          else if (avoid_offset_bias_ < -step) {avoid_offset_bias_ += step;}
          else {avoid_offset_bias_ = 0.0;}
          publish_avoid_offset();
        }
      }
    } else if (state_ == kParkingTZoneApproach || state_ == kParkingParallelApproach) {
      if (!active_event_index_ || !parking_entry_path_) {reset_to_lane_follow("no active parking event");}
      else {
        publish_parking_entry_path();
        const double dist_to_entry_end = current_pose_ && parking_entry_end_ ?
          std::hypot(current_pose_->x - parking_entry_end_->first,
            current_pose_->y - parking_entry_end_->second) :
          std::numeric_limits<double>::infinity();
        const double heading_err_deg = current_pose_ ?
          std::abs(hyper_planner::normalize_angle(current_pose_->yaw - parking_entry_end_yaw_)) *
            180.0 / hyper_planner::kPi :
          std::numeric_limits<double>::infinity();
        // Debug aid for diagnosing "approach times out without ever reaching entry tolerance" --
        // watch this to see whether distance keeps shrinking (still driving, just needs more time
        // or a looser tolerance) or plateaus somewhere (pursuit stuck/stalled on the entry path).
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "parking entry approach: dist_to_end=%.3f m (tol=%.2f) heading_err=%.1f deg (tol=%.1f)",
          dist_to_entry_end, parking_entry_tolerance_m_, heading_err_deg,
          parking_entry_heading_tolerance_deg_);
        if (dist_to_entry_end <= parking_entry_tolerance_m_ && heading_err_deg <= parking_entry_heading_tolerance_deg_) {
          const char * maneuver_state =
            state_ == kParkingTZoneApproach ? kParkingTZoneManeuver : kParkingParallelManeuver;
          enter_parking_maneuver(maneuver_state);
        }
      }
    } else if (state_ == kParkingTZoneManeuver || state_ == kParkingParallelManeuver) {
      if (!active_event_index_ || !parking_spot_path_) {reset_to_lane_follow("no active parking maneuver");}
      else {
        publish_parking_maneuver();
        // Either check completes the maneuver -- spot_path's last point IS the actual intended
        // parking spot, so physically reaching it is sufficient on its own, not just a fallback for
        // whenever the camera "currently isn't seeing anything". Making the rear camera the
        // *exclusive* source of truth whenever it reports anything fresh (the previous if/else
        // priority scheme) meant that if the vehicle drove all the way to the recorded destination
        // while the camera kept reporting a fresh-but-not-yet-close-enough detection, position_reached
        // would never even be consulted -- the maneuver could get stuck exactly at its own goal.
        // The camera still independently lets the maneuver finish *early/more precisely* than the
        // recorded endpoint (which can drift with anchor/GNSS error) when it's confidently close.
        // How far the vehicle has actually moved (odom-measured) since the maneuver started --
        // gates the rear-camera check below so a misdetection right at the entrance can't complete
        // the maneuver before any real reverse progress has happened.
        const double progress_m = current_pose_ && parking_maneuver_start_pos_ ?
          std::hypot(current_pose_->x - parking_maneuver_start_pos_->first,
            current_pose_->y - parking_maneuver_start_pos_->second) :
          0.0;
        const bool rear_fresh = rear_stop_line_fresh(t);
        const bool rear_stop_reached = rear_fresh &&
          rear_stop_line_distance_m_ <= parking_rear_stop_distance_m_ &&
          progress_m >= parking_rear_stop_min_progress_m_;
        const bool position_reached = current_pose_ && parking_spot_end_ &&
          std::hypot(current_pose_->x - parking_spot_end_->first,
            current_pose_->y - parking_spot_end_->second) <= parking_exit_tolerance_m_;
        if (rear_stop_reached || position_reached) {
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
  double parking_rear_stop_min_progress_m_{1.0};
  double default_parking_hold_duration_s_{0.0};
  double parking_hold_duration_current_s_{0.0};
  std::string default_mission_{"straight"};
  std::string mission_{"straight"};
  bool left_on_green_allowed_{false};
  bool right_on_green_allowed_{true};
  bool arrow_signal_selects_path_{true};

  YAML::Node events_;
  YAML::Node paths_;
  // TEST ORDERING: matches kCourseSequence's front slot (kCruiseRight) above -- revert to
  // kLeftLaneFollow together with that reorder before the actual competition run.
  std::string state_{kRightLaneFollow};
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
  std::optional<std::pair<double, double>> parking_maneuver_start_pos_;
  std::chrono::steady_clock::time_point parking_hold_started_{};
  std::optional<double> parking_closest_distance_m_;
  std::optional<Pose2D> parking_closest_anchor_pose_;

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
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  double avoid_offset_bias_{0.0};
  double avoid_scan_half_angle_deg_{100.0};
  double avoid_lookahead_m_{6.0};
  double avoid_corridor_half_width_m_{2.0};
  double avoid_margin_m_{0.05};
  double avoid_anticipation_m_{0.9};
  double avoid_hold_s_{1.2};
  double avoid_release_rate_m_s_{0.5};
  double avoid_hold_until_s_{-1e9};
  double avoid_max_offset_m_{1.0};
  double avoid_decisive_offset_m_{0.7};
  double vehicle_half_width_m_{0.405};
  double avoid_cluster_break_m_{0.3};

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
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr avoid_offset_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorSupervisorWithParking>());
  rclcpp::shutdown();
  return 0;
}