#include "hyper_planner/common.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
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
    // Reverted to 1.0 (i.e. inert) -- this was meant to just turn the dodge *response* sharper, but
    // Stanley control drives effective_offset (measured lane offset + this bias) toward zero, so
    // scaling bias alone shifts the *steady-state target itself*, not just how fast it's reached.
    // At 1.8, a 0.7m dodge (avoid_decisive_offset_m_) actually settled at 1.26m off lane-center --
    // past avoid_max_offset_m_'s 1.0m safety ceiling entirely, measured live via the long, hard
    // (max-steering-angle) correction needed to recover from that overshoot once released, and
    // implicated in a second obstacle being missed while still recovering from it. Now redundant
    // with avoid_decisive_offset_m_ anyway (added later, in compute_avoid_offset(), which sizes the
    // dodge correctly instead of scaling the control signal) -- kept as a parameter rather than
    // deleted only so it's easy to find if "sharper dodge" comes up again, but do not raise this
    // above 1.0 without separately re-deriving what steady-state offset it actually produces.
    avoid_offset_gain_ = declare_parameter<double>("avoid_offset_gain", 1.0);
    map_lookahead_ = declare_parameter<double>("lookahead", 1.0);
    input_timeout_s_ = declare_parameter<double>("input_timeout", 0.30);
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "body_link");
    parking_approach_speed_ = declare_parameter<double>("parking_approach_speed", 0.35);
    parking_forward_speed_ = declare_parameter<double>("parking_forward_speed", 0.25);
    parking_reverse_speed_ = declare_parameter<double>("parking_reverse_speed", 0.20);
    parking_lookahead_ = declare_parameter<double>("parking_lookahead", 0.60);
    gear_change_pause_s_ = declare_parameter<double>("gear_change_pause_s", 0.6);
    // Sign relating "rear-camera-reported lateral offset/heading" to "steering direction" -- the
    // rear camera is yaw-rotated 180 deg from the front one (see vehicle.xacro), which mirrors its
    // image left/right relative to the vehicle body, so these are very likely NOT simply
    // lane_offset_sign_/lane_heading_sign_ carried over. Verify empirically (watch the
    // LaneDetectionRear debug window vs. which way the vehicle actually steers) and flip if it
    // corrects the wrong way.
    parking_rear_offset_sign_ = declare_parameter<double>("parking_rear_offset_sign", -1.0);
    parking_rear_heading_sign_ = declare_parameter<double>("parking_rear_heading_sign", 1.0);
    // Separate from stanley_k_ (shared with front lane-following, which is not the thing being
    // tuned here) so this can be turned down independently -- observed live: oscillation ("와리가리")
    // once inside the bay, right where rear-camera correction starts dominating the blend, matching
    // classic Stanley overcorrection when the gain is too high for how noisy the reading is this
    // close to the target.
    parking_rear_stanley_k_ = declare_parameter<double>("parking_rear_stanley_k", 0.4);
    // How long a fresh, plausible rear-line reading takes to fully replace pursuit steering.
    parking_rear_blend_time_s_ = declare_parameter<double>("parking_rear_blend_time_s", 0.3);
    // A rear-line reading whose offset exceeds this is treated as a misdetection (wrong object,
    // shadow, mask noise) and ignored in favor of spot_path pursuit, even though its valid flag
    // is set -- real bay boundary lines don't appear meters away from the recorded maneuver.
    parking_rear_offset_plausible_max_m_ =
      declare_parameter<double>("parking_rear_offset_plausible_max_m", 1.5);
    // Offset alone isn't enough: a misdetection can land on a small, plausible-looking offset by
    // coincidence while its fitted angle is nowhere near parallel to the vehicle (observed live:
    // offset -0.23m, angle -54 deg). A real bay boundary line runs roughly parallel to the
    // reverse path, so reject readings whose angle magnitude exceeds this bound even if their
    // offset alone would have passed.
    parking_rear_angle_plausible_max_deg_ =
      declare_parameter<double>("parking_rear_angle_plausible_max_deg", 30.0);
    // During PARKING_*_MANEUVER only, x_/y_/yaw_ are driven from wheel-encoder distance + IMU yaw
    // instead of /odometry/filtered_map -- short-duration relative accuracy from encoder+IMU beats
    // GPS-corrected absolute position for backing precisely into a tight bay, since a GPS/EKF
    // correction landing mid-maneuver would otherwise show up as a steering jump that has nothing
    // to do with how the vehicle actually moved. Real joint/wheel values below are confirmed from
    // vehicle.xacro + gz_ros2_control.yaml, not guessed: only rear_left/right_wheel_joint publish
    // position on /joint_states (front wheel spin joints aren't exposed), and Gazebo's configured
    // wheel_radius is 0.2m regardless of any real-hardware value used later.
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.2);
    left_wheel_joint_ = declare_parameter<std::string>("left_wheel_joint", "rear_left_wheel_joint");
    right_wheel_joint_ = declare_parameter<std::string>("right_wheel_joint", "rear_right_wheel_joint");
    // Sign relating wheel rotation direction to signed distance -- not yet empirically verified.
    // Drive forward and confirm delta_s (see the throttled maneuver log) is positive; flip to -1.0
    // if it reads backwards, same as the parking_rear_*_sign parameters above.
    wheel_encoder_distance_sign_ = declare_parameter<double>("wheel_encoder_distance_sign", 1.0);
    // Once the rear stop-line (bay's back wall) is this close, stop steering off the rear-camera
    // lane/pursuit correction and hold the wheels straight instead -- backing the last stretch on a
    // steering correction risks swinging the tail into the wall at the last moment; straight is the
    // safer final approach regardless of what the lane correction would otherwise say.
    parking_rear_stopline_align_distance_m_ =
      declare_parameter<double>("parking_rear_stopline_align_distance_m", 1.0);

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/driving_mode", 10, std::bind(&ControllerWithParking::on_mode, this, std::placeholders::_1));
    lane_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/lane/center", 10, std::bind(&ControllerWithParking::on_lane, this, std::placeholders::_1));
    rear_lane_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/lane/rear_center", 10,
      std::bind(&ControllerWithParking::on_rear_lane, this, std::placeholders::_1));
    rear_stopline_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/stopline/rear_detection", 10,
      std::bind(&ControllerWithParking::on_rear_stopline, this, std::placeholders::_1));
    bridge_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/bridge_path", 10, std::bind(&ControllerWithParking::on_bridge, this, std::placeholders::_1));
    waypoint_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/waypoint_path", 10,
      std::bind(&ControllerWithParking::on_waypoint_path, this, std::placeholders::_1));
    parking_entry_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/parking/entry_path", 10,
      std::bind(&ControllerWithParking::on_parking_entry, this, std::placeholders::_1));
    parking_spot_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/parking/spot_path", 10,
      std::bind(&ControllerWithParking::on_parking_spot, this, std::placeholders::_1));
    parking_gear_sub_ = create_subscription<std_msgs::msg::Int8MultiArray>(
      "/parking/spot_gear", 10,
      std::bind(&ControllerWithParking::on_parking_gear, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map", 10,
      std::bind(&ControllerWithParking::on_odom, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ControllerWithParking::on_joint_states, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&ControllerWithParking::on_imu, this, std::placeholders::_1));
    avoid_offset_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/avoid/offset_bias", 10,
      std::bind(&ControllerWithParking::on_avoid_offset, this, std::placeholders::_1));

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
    if (msg->data != mode_) {
      last_parking_gear_ = 0;
      gear_pause_until_s_ = 0.0;
      parking_entry_last_index_ = 0U;
      parking_spot_last_index_ = 0U;
      rear_tracking_ = false;
    }
    mode_ = msg->data;
  }

  void on_lane(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    lane_time_s_ = now_s();
    if (msg->data.size() < 6U) {
      left_valid_ = false; right_valid_ = false; return;
    }
    const double raw_left_angle = msg->data[0] * hyper_planner::kPi / 180.0;
    const double raw_left_offset = msg->data[1];
    left_valid_ = msg->data[2] > 0.5;
    const double raw_right_angle = msg->data[3] * hyper_planner::kPi / 180.0;
    const double raw_right_offset = msg->data[4];
    right_valid_ = msg->data[5] > 0.5;

    const double alpha = hyper_planner::clamp(lane_filter_alpha_, 0.0, 1.0);
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

  // Unfiltered (unlike on_lane's alpha-smoothed left/right_offset_) -- during PARKING_MANEUVER
  // this becomes the primary steering reference the moment it's valid and plausible, so
  // responsiveness matters more than smoothing here.
  void on_rear_lane(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    rear_lane_time_s_ = now_s();
    if (msg->data.size() < 6U) {rear_left_valid_ = false; rear_right_valid_ = false; return;}
    rear_left_angle_ = msg->data[0] * hyper_planner::kPi / 180.0;
    rear_left_offset_ = msg->data[1];
    rear_left_valid_ = msg->data[2] > 0.5;
    rear_right_angle_ = msg->data[3] * hyper_planner::kPi / 180.0;
    rear_right_offset_ = msg->data[4];
    rear_right_valid_ = msg->data[5] > 0.5;
  }

  void on_rear_stopline(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    rear_stopline_time_s_ = now_s();
    if (msg->data.size() < 2U) {rear_stopline_valid_ = false; return;}
    rear_stopline_distance_m_ = msg->data[0];
    rear_stopline_valid_ = msg->data[1] > 0.5;
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

  void on_parking_entry(const nav_msgs::msg::Path::SharedPtr msg)
  {
    parking_entry_path_.clear();
    for (const auto & pose : msg->poses) {
      parking_entry_path_.push_back({pose.pose.position.x, pose.pose.position.y});
    }
  }

  void on_parking_spot(const nav_msgs::msg::Path::SharedPtr msg)
  {
    parking_spot_path_.clear();
    for (const auto & pose : msg->poses) {
      parking_spot_path_.push_back({pose.pose.position.x, pose.pose.position.y});
    }
  }

  void on_parking_gear(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
  {
    parking_spot_gear_.assign(msg->data.begin(), msg->data.end());
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = hyper_planner::yaw_from_quaternion(msg->pose.pose.orientation);
    velocity_ = msg->twist.twist.linear.x;
    odom_time_s_ = now_s();
  }

  // Just caches the two drive-wheel joint positions -- the actual delta/integration math runs once
  // per control tick in control_step(), same pattern as on_lane()/on_rear_lane() above.
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool have_left = false, have_right = false;
    double left_angle = 0.0, right_angle = 0.0;
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      if (msg->name[i] == left_wheel_joint_) {left_angle = msg->position[i]; have_left = true;}
      else if (msg->name[i] == right_wheel_joint_) {right_angle = msg->position[i]; have_right = true;}
    }
    if (!have_left || !have_right || !std::isfinite(left_angle) || !std::isfinite(right_angle)) {return;}
    left_wheel_angle_ = left_angle;
    right_wheel_angle_ = right_angle;
    joint_state_time_s_ = now_s();
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double yaw = hyper_planner::yaw_from_quaternion(msg->orientation);
    if (!std::isfinite(yaw)) {return;}
    imu_yaw_ = yaw;
    imu_time_s_ = now_s();
  }

  // Published by behavior_supervisor_with_parking.cpp's AVOID-zone LIDAR clustering (see
  // compute_avoid_offset() there) -- how far off lane-center to bias to clear a detected obstacle,
  // already sized for the vehicle's own half-width plus a safety margin. Only trusted while fresh
  // (see the AVOID branch in control_step()); a stale value falls back to 0 rather than freezing an
  // old bias, since a stopped/crashed supervisor should not leave the vehicle permanently offset.
  void on_avoid_offset(const std_msgs::msg::Float64::SharedPtr msg)
  {
    avoid_offset_bias_ = msg->data;
    avoid_offset_time_s_ = now_s();
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
    const double normalized = hyper_planner::clamp(std::abs(steering) / std::max(1e-6, max_steer_), 0.0, 1.0);
    const double reduced = cruise_speed_ * (1.0 - lane_curve_speed_reduction_ *
      std::pow(normalized, curve_slow_k_));
    return hyper_planner::clamp(reduced, minimum_speed_, cruise_speed_);
  }

  Command pure_pursuit_on_map_path(const std::vector<Point2D> & path, double lookahead) const
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
    if (lx <= 0.0 || ld2 < 1e-6) {return {};}
    return Command{std::atan2(2.0 * wheelbase_ * ly, ld2), 0.0, true};
  }

  // A path with a turn (e.g. parking entry_path cutting off the road into the zone) can loop
  // back spatially close to its own earlier segment even though those points are far apart in
  // arc-length. A *global* nearest-point search picks whichever is closer in raw Euclidean
  // distance, which can jump tracking backward to an already-passed point right as the vehicle
  // nears the end of the path -- that point then reads as "behind" the vehicle and pursuit stalls
  // just short of the goal. Restricting the search to a forward-looking window from the last
  // known index keeps progress monotonic along the path instead of by raw distance.
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
    return Command{std::atan2(2.0 * wheelbase_ * ly, ld2), 0.0, true};
  }

  // Pure-pursuit curvature is derived from the circle tangent to the vehicle heading at the rear
  // axle passing through the target point; that geometry -- and therefore this formula -- holds
  // whether the target is ahead (forward gear) or behind (reverse gear), so no sign flip is
  // needed beyond commanding a negative speed for reverse segments. On a gear change the vehicle
  // pauses for gear_change_pause_s_ before moving in the new direction.
  Command gear_aware_pursuit_on_map_path(
    const std::vector<Point2D> & path, const std::vector<int8_t> & gears, double lookahead,
    std::size_t & last_index)
  {
    if (path.empty() || gears.size() != path.size()) {return {};}
    const std::size_t nearest = windowed_nearest_index(path, last_index);
    last_index = nearest;

    std::size_t target = nearest;
    while (target + 1U < path.size() &&
      std::hypot(path[target].x - x_, path[target].y - y_) < lookahead) {++target;}

    // Gear is read from the point actually being steered toward (target), not the nearest point.
    // A near-zero-length segment (e.g. set_gear called immediately at record_start, leaving a
    // single-point "forward" stub right on top of the first "reverse" point) can tie the
    // nearest-point search, locking the gear onto the wrong segment while steering already
    // targets a point further down the correctly-tagged path.
    const int8_t gear = gears[target];
    const double t = now_s();
    if (gear != last_parking_gear_) {
      gear_pause_until_s_ = t + gear_change_pause_s_;
      last_parking_gear_ = gear;
    }
    if (t < gear_pause_until_s_) {return Command{commanded_steering_, 0.0, true};}

    const double dx = path[target].x - x_; const double dy = path[target].y - y_;
    const double lx = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    const double ly = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
    const double ld2 = lx * lx + ly * ly;
    const bool pursuit_valid = ld2 >= 1e-6;
    const double pursuit_steering =
      pursuit_valid ? std::atan2(2.0 * wheelbase_ * ly, ld2) : commanded_steering_;
    const double speed = gear >= 0 ? parking_forward_speed_ : -parking_reverse_speed_;

    // Once the rear camera has a fresh, plausible line reading while backing up, it becomes the
    // primary steering reference -- it measures the real bay boundary every frame, so unlike
    // spot_path (a fixed replay anchored to wherever the vehicle happened to be at maneuver
    // start) it isn't subject to anchor/GNSS drift at all. "Plausible" rejects a reading whose
    // offset is too large to be the real line (a misdetected shadow, the wrong object, mask
    // noise) even though its valid flag is set, falling back to pursuit instead of trusting it.
    // Eased in over parking_rear_blend_time_s_ so a fresh reading doesn't step-change the
    // steering; dropped instantly (no easing) the moment the reading is lost or implausible,
    // since a stale/wrong line is worse than the recorded-path fallback.
    bool rear_plausible = false;
    double rear_steering = pursuit_steering;
    if (gear < 0 && t - rear_lane_time_s_ < input_timeout_s_) {
      // Plausibility is checked per side, before any averaging -- averaging first would let two
      // individually-bad readings (e.g. one side's offset way out, the other's angle way off)
      // cancel out into a combined number that happens to land inside both thresholds, even
      // though neither side alone was trustworthy. Observed live: left angle -3 deg (fine) with
      // right angle -47 deg (garbage) averaged to -25 deg, passing the old post-average check.
      const double max_angle_rad = parking_rear_angle_plausible_max_deg_ * hyper_planner::kPi / 180.0;
      const bool left_ok = rear_left_valid_ &&
        std::abs(rear_left_offset_) <= parking_rear_offset_plausible_max_m_ &&
        std::abs(rear_left_angle_) <= max_angle_rad;
      const bool right_ok = rear_right_valid_ &&
        std::abs(rear_right_offset_) <= parking_rear_offset_plausible_max_m_ &&
        std::abs(rear_right_angle_) <= max_angle_rad;
      // Prefer the left line whenever it's valid, rather than averaging with the right one --
      // rear_parking_perception is designed around "only one side line is visible/relevant while
      // backing into the bay" (see kRearParkingLineStandoffM's comment in lane_detection.cpp), so a
      // simultaneous right-side reading is not necessarily the bay's other boundary; averaging it
      // in can pull the target toward an unrelated feature instead of the line actually being
      // tracked (observed live: left 1.71m/24.5deg vs. right 4.92m/25.4deg -- far too wide apart to
      // be the two sides of the same bay).
      if (left_ok || right_ok) {
        const double rear_offset_m = left_ok ? rear_left_offset_ : rear_right_offset_;
        const double rear_angle_rad = left_ok ? rear_left_angle_ : rear_right_angle_;
        rear_plausible = true;
        rear_steering = parking_rear_heading_sign_ * rear_angle_rad + std::atan2(
          parking_rear_stanley_k_ * parking_rear_offset_sign_ * rear_offset_m,
          std::max(stanley_v_min_, std::abs(velocity_)));
      }
    }

    double steering;
    if (rear_plausible) {
      if (!rear_tracking_) {rear_track_start_s_ = t; rear_tracking_ = true;}
      const double blend = hyper_planner::clamp(
        (t - rear_track_start_s_) / std::max(1e-3, parking_rear_blend_time_s_), 0.0, 1.0);
      steering = (1.0 - blend) * pursuit_steering + blend * rear_steering;
    } else {
      rear_tracking_ = false;
      steering = pursuit_steering;
    }

    // Once the rear stop-line is close enough, straighten the wheels for the final stretch
    // regardless of what pursuit/lane correction above computed -- overrides everything else here,
    // since swinging the tail on a steering correction is exactly the wrong thing to do this close
    // to the wall.
    const bool near_rear_stopline = gear < 0 && rear_stopline_valid_ &&
      t - rear_stopline_time_s_ < input_timeout_s_ &&
      rear_stopline_distance_m_ <= parking_rear_stopline_align_distance_m_;
    if (near_rear_stopline) {steering = 0.0;}

    if (!pursuit_valid && !rear_plausible) {return {};}
    return Command{steering, speed, true};
  }

  enum class Phase
  {
    kLeftLaneFollow, kRightLaneFollow, kApproach, kHillApproach, kStopAtLight, kHillStop,
    kTurnBridge, kAccelZone, kObstacleStop, kParkingApproach, kParkingManeuver, kParkingHold, kOther
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
    // Parking approach/maneuver need path-replay (entry_path / gear-tagged spot_path), not the
    // generic lane-following the "_APPROACH" suffix implies elsewhere, so these are matched
    // exactly and ahead of the suffix fallbacks below.
    if (mode == "PARKING_T_ZONE_APPROACH" || mode == "PARKING_PARALLEL_APPROACH") {
      return Phase::kParkingApproach;
    }
    if (mode == "PARKING_T_ZONE_MANEUVER" || mode == "PARKING_PARALLEL_MANEUVER") {
      return Phase::kParkingManeuver;
    }
    if (mode == "PARKING_T_ZONE_HOLD" || mode == "PARKING_PARALLEL_HOLD") {return Phase::kParkingHold;}
    if (has_suffix(mode, "_APPROACH")) {return Phase::kApproach;}
    if (has_suffix(mode, "_STOP_AT_LIGHT")) {return Phase::kStopAtLight;}
    if (has_suffix(mode, "_TURN_BRIDGE")) {return Phase::kTurnBridge;}
    return Phase::kOther;
  }

  // Integrates parking_local_{x,y,yaw}_ from wheel-encoder distance + IMU yaw, but only while
  // actually in a parking maneuver -- outside of it this just keeps the "previous" trackers current
  // so that the *next* maneuver entry computes a clean first delta instead of a backlogged jump.
  // Called once per control tick, before the phase dispatch below, same as every other cached-input
  // freshness check in this function.
  void update_parking_local_pose(Phase phase, double t)
  {
    const bool encoder_fresh = t - joint_state_time_s_ < input_timeout_s_;
    const bool imu_fresh = t - imu_time_s_ < input_timeout_s_;

    if (!encoder_fresh || !imu_fresh) {
      // Sensors not ready (or stale): don't integrate a garbage/backlogged delta, and force a
      // reseed once they return rather than resuming from a now-stale "previous" reading.
      parking_local_seeded_ = false;
      parking_pose_initialized_ = false;
      return;
    }

    if (!parking_pose_initialized_) {
      prev_left_wheel_angle_ = left_wheel_angle_;
      prev_right_wheel_angle_ = right_wheel_angle_;
      prev_imu_yaw_ = imu_yaw_;
      parking_pose_initialized_ = true;
      return;
    }

    const double delta_left = hyper_planner::normalize_angle(left_wheel_angle_ - prev_left_wheel_angle_);
    const double delta_right = hyper_planner::normalize_angle(right_wheel_angle_ - prev_right_wheel_angle_);
    const double delta_q = 0.5 * (delta_left + delta_right);
    const double delta_s = wheel_encoder_distance_sign_ * wheel_radius_ * delta_q;
    const double delta_yaw = hyper_planner::normalize_angle(imu_yaw_ - prev_imu_yaw_);

    if (phase == Phase::kParkingManeuver) {
      if (!parking_local_seeded_) {
        parking_local_x_ = x_; parking_local_y_ = y_; parking_local_yaw_ = yaw_;
        parking_local_seeded_ = true;
      } else {
        const double yaw_mid = parking_local_yaw_ + 0.5 * delta_yaw;
        parking_local_x_ += delta_s * std::cos(yaw_mid);
        parking_local_y_ += delta_s * std::sin(yaw_mid);
        parking_local_yaw_ = hyper_planner::normalize_angle(parking_local_yaw_ + delta_yaw);
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "parking local pose: x=%.3f y=%.3f yaw=%.3f delta_s=%.4f",
        parking_local_x_, parking_local_y_, parking_local_yaw_, delta_s);
    } else {
      // Not maneuvering: reseed next time it's entered, from wherever the vehicle actually is then.
      parking_local_seeded_ = false;
    }

    prev_left_wheel_angle_ = left_wheel_angle_;
    prev_right_wheel_angle_ = right_wheel_angle_;
    prev_imu_yaw_ = imu_yaw_;
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
    update_parking_local_pose(phase, t);

    double target_steering = commanded_steering_;
    double target_speed = 0.0;

    if (!odom_ok) {target_steering = 0.0; target_speed = 0.0;}
    else if (phase == Phase::kTurnBridge) {
      const Command c = pure_pursuit_on_map_path(bridge_path_, map_lookahead_);
      target_steering = c.steering; target_speed = c.valid ? bridge_speed_ : 0.0;
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
      // Don't rigidly stick to use_right_lane_ (inherited from whichever cruise state preceded this
      // zone) the way every other phase does -- the obstacle actually being dodged sits right beside
      // the vehicle on one side, and can occlude the camera's view of *that side's* lane line right
      // as a dodge needs it most, at which point lane_ok (computed off use_right_lane_ alone) would
      // go false and freeze target_steering/target_speed exactly when a reaction matters most. Prefer
      // whichever side this zone was already tracking, but fall back to the other one if it's
      // currently the only one valid, rather than doing nothing.
      const bool right_fresh = t - lane_time_s_ < input_timeout_s_ && right_valid_;
      const bool left_fresh = t - lane_time_s_ < input_timeout_s_ && left_valid_;
      if (right_fresh || left_fresh) {
        const bool use_right_for_avoid = right_fresh && (use_right_lane_ || !left_fresh);
        const bool avoid_offset_fresh = (t - avoid_offset_time_s_) < input_timeout_s_;
        // avoid_offset_gain_ only scales the dodge component (bias), not ordinary lane-following
        // (every other mode still calls lane_follow_command(0.0, ...) or with unscaled offset) --
        // deliberately separate from stanley_k_ so this can be tuned more aggressively without
        // making normal lane tracking twitchier everywhere else.
        const double bias = avoid_offset_fresh ? avoid_offset_bias_ * avoid_offset_gain_ : 0.0;
        target_steering = lane_follow_command(bias, use_right_for_avoid);
        target_speed = 0.6 * speed_for_curve(target_steering);
      }
    } else if (mode_ == "WAYPOINT_FOLLOW") {
      const Command c = pure_pursuit_on_map_path(waypoint_path_, map_lookahead_);
      target_steering = c.steering; target_speed = c.valid ? waypoint_speed_ : 0.0;
    } else if (phase == Phase::kParkingApproach) {
      const Command c = pure_pursuit_windowed(parking_entry_path_, map_lookahead_, parking_entry_last_index_);
      target_steering = c.steering; target_speed = c.valid ? parking_approach_speed_ : 0.0;
    } else if (phase == Phase::kParkingManeuver) {
      // Drive pursuit off the encoder+IMU local pose instead of raw odom, if it's seeded -- safe to
      // overwrite these members here since control_step() runs synchronously (single-threaded
      // executor, no callback can interleave mid-tick) and on_odom() will refresh x_/y_/yaw_ from
      // real odom again the moment the phase changes, before any non-parking phase reads them.
      if (parking_local_seeded_) {x_ = parking_local_x_; y_ = parking_local_y_; yaw_ = parking_local_yaw_;}
      const Command c = gear_aware_pursuit_on_map_path(
        parking_spot_path_, parking_spot_gear_, parking_lookahead_, parking_spot_last_index_);
      target_steering = c.valid ? c.steering : 0.0; target_speed = c.valid ? c.speed : 0.0;
    } else if (phase == Phase::kParkingHold) {
      target_steering = 0.0; target_speed = 0.0;
    } else if (lane_ok) {
      target_steering = lane_follow_command(0.0, use_right_lane_);
      target_speed = speed_for_curve(target_steering);
    }

    target_steering = hyper_planner::clamp(target_steering, -max_steer_, max_steer_);
    target_speed = hyper_planner::clamp(target_speed, -max_velocity_, max_velocity_);
    commanded_steering_ = hyper_planner::rate_limit(commanded_steering_, target_steering, steering_rate_ * dt);
    const double active_decel_limit = phase == Phase::kObstacleStop ?
      emergency_deceleration_limit_ : deceleration_limit_;
    commanded_speed_ = hyper_planner::ramp(commanded_speed_, target_speed,
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
  double avoid_offset_bias_{0.0}, avoid_offset_time_s_{-1e9}, avoid_offset_gain_{1.0};
  double map_lookahead_{1.0}, input_timeout_s_{0.3};
  std::string base_frame_id_{"body_link"}, mode_{"LEFT_LANE_FOLLOW"};
  bool use_right_lane_{false};
  double left_angle_{0.0}, left_offset_{0.0}, right_angle_{0.0}, right_offset_{0.0};
  bool left_valid_{false}, right_valid_{false}, left_initialized_{false}, right_initialized_{false};
  std::vector<Point2D> bridge_path_, waypoint_path_;
  std::vector<Point2D> parking_entry_path_, parking_spot_path_;
  std::vector<int8_t> parking_spot_gear_;
  double parking_approach_speed_{0.35}, parking_forward_speed_{0.25}, parking_reverse_speed_{0.20};
  double parking_lookahead_{0.60}, gear_change_pause_s_{0.6};
  int8_t last_parking_gear_{0};
  double gear_pause_until_s_{0.0};
  std::size_t parking_entry_last_index_{0U};
  std::size_t parking_spot_last_index_{0U};
  double parking_rear_offset_sign_{-1.0}, parking_rear_heading_sign_{1.0};
  double parking_rear_stanley_k_{0.4};
  double parking_rear_blend_time_s_{0.3}, parking_rear_offset_plausible_max_m_{1.5};
  double parking_rear_angle_plausible_max_deg_{30.0};
  double rear_left_angle_{0.0}, rear_left_offset_{0.0}, rear_right_angle_{0.0}, rear_right_offset_{0.0};
  bool rear_left_valid_{false}, rear_right_valid_{false};
  double rear_lane_time_s_{-1e9};
  bool rear_tracking_{false};
  double rear_track_start_s_{0.0};
  double parking_rear_stopline_align_distance_m_{1.0};
  double rear_stopline_distance_m_{0.0}, rear_stopline_time_s_{-1e9};
  bool rear_stopline_valid_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0}, velocity_{0.0};
  double commanded_steering_{0.0}, commanded_speed_{0.0}, last_control_time_s_{0.0};
  double odom_time_s_{-1e9}, lane_time_s_{-1e9};

  // Wheel-encoder + IMU local pose, used only during PARKING_*_MANEUVER (see update_parking_local_pose).
  double wheel_radius_{0.2}, wheel_encoder_distance_sign_{1.0};
  std::string left_wheel_joint_{"rear_left_wheel_joint"}, right_wheel_joint_{"rear_right_wheel_joint"};
  double left_wheel_angle_{0.0}, right_wheel_angle_{0.0}, joint_state_time_s_{-1e9};
  double prev_left_wheel_angle_{0.0}, prev_right_wheel_angle_{0.0};
  double imu_yaw_{0.0}, imu_time_s_{-1e9}, prev_imu_yaw_{0.0};
  bool parking_pose_initialized_{false};
  bool parking_local_seeded_{false};
  double parking_local_x_{0.0}, parking_local_y_{0.0}, parking_local_yaw_{0.0};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lane_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rear_lane_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rear_stopline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr bridge_sub_, waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr parking_entry_sub_, parking_spot_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr parking_gear_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr avoid_offset_sub_;
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
