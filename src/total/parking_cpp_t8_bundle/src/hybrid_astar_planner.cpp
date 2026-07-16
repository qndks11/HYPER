#include "parking_cpp/common.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Key
{
  int x{0};
  int y{0};
  int yaw{0};
  int direction{1};

  bool operator==(const Key & other) const
  {
    return x == other.x && y == other.y && yaw == other.yaw && direction == other.direction;
  }
};

struct KeyHash
{
  std::size_t operator()(const Key & key) const noexcept
  {
    std::size_t seed = 0;
    auto mix = [&seed](int value) {
        seed ^= std::hash<int>{}(value) + 0x9e3779b9 + (seed << 6U) + (seed >> 2U);
      };
    mix(key.x);
    mix(key.y);
    mix(key.yaw);
    mix(key.direction);
    return seed;
  }
};

struct SearchNode
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  int direction{1};
  double steering{0.0};
  double g{0.0};
  double h{0.0};
  std::optional<Key> parent;

  double f() const {return g + h;}
};

struct QueueEntry
{
  double f{0.0};
  std::uint64_t serial{0};
  Key key;
};

struct QueueCompare
{
  bool operator()(const QueueEntry & lhs, const QueueEntry & rhs) const
  {
    if (lhs.f == rhs.f) {
      return lhs.serial > rhs.serial;
    }
    return lhs.f > rhs.f;
  }
};

class GridCollisionChecker
{
public:
  GridCollisionChecker(
    const std::optional<nav_msgs::msg::OccupancyGrid> & grid,
    int occupied_threshold,
    bool unknown_is_occupied,
    double vehicle_front_m,
    double vehicle_rear_m,
    double vehicle_half_width_m,
    double footprint_sample_m,
    const std::optional<std::tuple<double, double, double, double>> & empty_bounds)
  : grid_(grid),
    occupied_threshold_(occupied_threshold),
    unknown_is_occupied_(unknown_is_occupied),
    vehicle_front_m_(vehicle_front_m),
    vehicle_rear_m_(vehicle_rear_m),
    vehicle_half_width_m_(vehicle_half_width_m),
    footprint_sample_m_(std::max(0.05, footprint_sample_m)),
    empty_bounds_(empty_bounds)
  {
    if (grid_.has_value()) {
      resolution_ = grid_->info.resolution;
      width_ = static_cast<int>(grid_->info.width);
      height_ = static_cast<int>(grid_->info.height);
      origin_x_ = grid_->info.origin.position.x;
      origin_y_ = grid_->info.origin.position.y;
    }
    make_footprint_samples();
  }

  bool collision(double x, double y, double yaw) const
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (const auto & sample : local_samples_) {
      const double world_x = x + c * sample.first - s * sample.second;
      const double world_y = y + s * sample.first + c * sample.second;
      if (point_occupied(world_x, world_y)) {
        return true;
      }
    }
    return false;
  }

private:
  void make_footprint_samples()
  {
    for (double x = -vehicle_rear_m_; x <= vehicle_front_m_ + 1e-9; x += footprint_sample_m_) {
      for (double y = -vehicle_half_width_m_; y <= vehicle_half_width_m_ + 1e-9;
        y += footprint_sample_m_)
      {
        local_samples_.emplace_back(x, y);
      }
    }
    local_samples_.emplace_back(0.0, 0.0);
    local_samples_.emplace_back(vehicle_front_m_, vehicle_half_width_m_);
    local_samples_.emplace_back(vehicle_front_m_, -vehicle_half_width_m_);
    local_samples_.emplace_back(-vehicle_rear_m_, vehicle_half_width_m_);
    local_samples_.emplace_back(-vehicle_rear_m_, -vehicle_half_width_m_);
  }

  bool point_occupied(double x, double y) const
  {
    if (!grid_.has_value()) {
      if (!empty_bounds_.has_value()) {
        return false;
      }
      const auto [min_x, max_x, min_y, max_y] = *empty_bounds_;
      return !(x >= min_x && x <= max_x && y >= min_y && y <= max_y);
    }

    const int mx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    const int my = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    if (mx < 0 || my < 0 || mx >= width_ || my >= height_) {
      return true;
    }

    const auto index = static_cast<std::size_t>(my * width_ + mx);
    if (index >= grid_->data.size()) {
      return true;
    }
    const int value = static_cast<int>(grid_->data[index]);
    if (value < 0) {
      return unknown_is_occupied_;
    }
    return value >= occupied_threshold_;
  }

  std::optional<nav_msgs::msg::OccupancyGrid> grid_;
  int occupied_threshold_{50};
  bool unknown_is_occupied_{true};
  double vehicle_front_m_{0.77};
  double vehicle_rear_m_{0.77};
  double vehicle_half_width_m_{0.505};
  double footprint_sample_m_{0.10};
  std::optional<std::tuple<double, double, double, double>> empty_bounds_;
  double resolution_{0.10};
  int width_{0};
  int height_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<std::pair<double, double>> local_samples_;
};

struct PlannerParameters
{
  double xy_resolution{0.20};
  double yaw_resolution{15.0 * parking_cpp::kPi / 180.0};
  double motion_length{0.30};
  double integration_step{0.10};
  double wheelbase{1.005};
  double max_steer{0.5235988};
  int steering_samples{5};
  bool allow_reverse{true};
  double reverse_penalty{1.20};
  double gear_switch_penalty{4.0};
  double steering_penalty{0.15};
  double steering_change_penalty{0.35};
  double heading_heuristic_weight{0.50};
  double goal_xy_tolerance{0.30};
  double goal_yaw_tolerance{12.0 * parking_cpp::kPi / 180.0};
  int max_iterations{120000};
};

class HybridAStar
{
public:
  HybridAStar(GridCollisionChecker checker, PlannerParameters parameters)
  : checker_(std::move(checker)), parameters_(parameters) {}

  std::optional<std::vector<SearchNode>> plan(const Pose2D & start, const Pose2D & goal)
  {
    if (checker_.collision(start.x, start.y, start.yaw) ||
      checker_.collision(goal.x, goal.y, goal.yaw))
    {
      return std::nullopt;
    }

    SearchNode start_node;
    start_node.x = start.x;
    start_node.y = start.y;
    start_node.yaw = start.yaw;
    start_node.direction = 1;
    start_node.h = heuristic(start_node.x, start_node.y, start_node.yaw, goal);
    const Key start_key = key(start_node.x, start_node.y, start_node.yaw, 1);

    std::unordered_map<Key, SearchNode, KeyHash> nodes;
    std::unordered_map<Key, double, KeyHash> best_g;
    nodes.emplace(start_key, start_node);
    best_g.emplace(start_key, 0.0);

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
    std::uint64_t serial = 0;
    open.push(QueueEntry{start_node.f(), serial++, start_key});

    const std::vector<int> directions = parameters_.allow_reverse ?
      std::vector<int>{1, -1} : std::vector<int>{1};
    const auto steering_values = make_steering_values();

    int iterations = 0;
    while (!open.empty() && iterations < parameters_.max_iterations) {
      ++iterations;
      const QueueEntry entry = open.top();
      open.pop();
      const auto node_iterator = nodes.find(entry.key);
      if (node_iterator == nodes.end()) {
        continue;
      }
      const SearchNode current = node_iterator->second;
      const auto best_iterator = best_g.find(entry.key);
      if (best_iterator != best_g.end() && current.g > best_iterator->second + 1e-9) {
        continue;
      }

      if (goal_reached(current, goal)) {
        auto path = reconstruct(entry.key, nodes);
        SearchNode exact_goal;
        exact_goal.x = goal.x;
        exact_goal.y = goal.y;
        exact_goal.yaw = goal.yaw;
        exact_goal.direction = current.direction;
        exact_goal.steering = current.steering;
        exact_goal.g = current.g;
        exact_goal.parent = entry.key;
        path.push_back(exact_goal);
        return path;
      }

      for (const int direction : directions) {
        for (const double steering : steering_values) {
          const auto propagated = propagate(current, steering, direction);
          if (!propagated.has_value()) {
            continue;
          }
          const Key child_key = key(propagated->x, propagated->y, propagated->yaw, direction);
          const double new_g = current.g + transition_cost(current, steering, direction);
          const auto known = best_g.find(child_key);
          if (known != best_g.end() && new_g >= known->second - 1e-9) {
            continue;
          }

          SearchNode child;
          child.x = propagated->x;
          child.y = propagated->y;
          child.yaw = propagated->yaw;
          child.direction = direction;
          child.steering = steering;
          child.g = new_g;
          child.h = heuristic(child.x, child.y, child.yaw, goal);
          child.parent = entry.key;
          nodes[child_key] = child;
          best_g[child_key] = new_g;
          open.push(QueueEntry{child.f(), serial++, child_key});
        }
      }
    }
    return std::nullopt;
  }

private:
  Key key(double x, double y, double yaw, int direction) const
  {
    return Key{
      static_cast<int>(std::llround(x / parameters_.xy_resolution)),
      static_cast<int>(std::llround(y / parameters_.xy_resolution)),
      static_cast<int>(std::llround(parking_cpp::normalize_angle(yaw) / parameters_.yaw_resolution)),
      direction};
  }

  double heuristic(double x, double y, double yaw, const Pose2D & goal) const
  {
    return std::hypot(goal.x - x, goal.y - y) +
           parameters_.heading_heuristic_weight *
           std::abs(parking_cpp::normalize_angle(goal.yaw - yaw));
  }

  bool goal_reached(const SearchNode & node, const Pose2D & goal) const
  {
    return std::hypot(node.x - goal.x, node.y - goal.y) <= parameters_.goal_xy_tolerance &&
           std::abs(parking_cpp::normalize_angle(node.yaw - goal.yaw)) <=
           parameters_.goal_yaw_tolerance;
  }

  std::vector<double> make_steering_values() const
  {
    const int count = std::max(3, parameters_.steering_samples);
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      values.push_back(
        -parameters_.max_steer +
        2.0 * parameters_.max_steer * static_cast<double>(index) /
        static_cast<double>(count - 1));
    }
    return values;
  }

  std::optional<Pose2D> propagate(
    const SearchNode & node, double steering, int direction) const
  {
    Pose2D pose{node.x, node.y, node.yaw};
    double remaining = parameters_.motion_length;
    const double step = std::min(parameters_.integration_step, parameters_.motion_length);
    while (remaining > 1e-9) {
      const double distance = std::min(step, remaining) * static_cast<double>(direction);
      pose.x += distance * std::cos(pose.yaw);
      pose.y += distance * std::sin(pose.yaw);
      pose.yaw = parking_cpp::normalize_angle(
        pose.yaw + distance / parameters_.wheelbase * std::tan(steering));
      remaining -= std::abs(distance);
      if (checker_.collision(pose.x, pose.y, pose.yaw)) {
        return std::nullopt;
      }
    }
    return pose;
  }

  double transition_cost(const SearchNode & current, double steering, int direction) const
  {
    double cost = parameters_.motion_length;
    if (direction < 0) {
      cost *= parameters_.reverse_penalty;
    }
    if (direction != current.direction) {
      cost += parameters_.gear_switch_penalty;
    }
    cost += parameters_.steering_penalty *
      std::abs(steering) / std::max(parameters_.max_steer, 1e-6);
    cost += parameters_.steering_change_penalty *
      std::abs(steering - current.steering) / std::max(parameters_.max_steer, 1e-6);
    return cost;
  }

  static std::vector<SearchNode> reconstruct(
    Key goal_key, const std::unordered_map<Key, SearchNode, KeyHash> & nodes)
  {
    std::vector<SearchNode> reverse_path;
    std::optional<Key> current = goal_key;
    while (current.has_value()) {
      const auto iterator = nodes.find(*current);
      if (iterator == nodes.end()) {
        break;
      }
      reverse_path.push_back(iterator->second);
      current = iterator->second.parent;
    }
    return std::vector<SearchNode>(reverse_path.rbegin(), reverse_path.rend());
  }

  GridCollisionChecker checker_;
  PlannerParameters parameters_;
};

}  // namespace

class HybridAStarPlannerNode : public rclcpp::Node
{
public:
  HybridAStarPlannerNode()
  : Node("hybrid_astar_planner")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    map_topic_ = declare_parameter<std::string>("map_topic", "/parking/local_costmap");
    allow_empty_map_ = declare_parameter<bool>("allow_empty_map", false);
    map_timeout_s_ = declare_parameter<double>("map_timeout_s", 0.75);
    empty_map_margin_m_ = declare_parameter<double>("empty_map_margin_m", 8.0);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
    unknown_is_occupied_ = declare_parameter<bool>("unknown_is_occupied", false);

    const double safety_margin = declare_parameter<double>("footprint_margin_m", 0.10);
    vehicle_front_m_ = declare_parameter<double>("vehicle_front_m", 0.67) + safety_margin;
    vehicle_rear_m_ = declare_parameter<double>("vehicle_rear_m", 0.67) + safety_margin;
    vehicle_half_width_m_ = 0.5 * declare_parameter<double>("vehicle_width_m", 0.81) + safety_margin;
    footprint_sample_m_ = declare_parameter<double>("footprint_sample_m", 0.10);

    parameters_.xy_resolution = declare_parameter<double>("xy_resolution", 0.20);
    parameters_.yaw_resolution = declare_parameter<double>("yaw_resolution_deg", 15.0) * parking_cpp::kPi / 180.0;
    parameters_.motion_length = declare_parameter<double>("motion_length", 0.30);
    parameters_.integration_step = declare_parameter<double>("integration_step", 0.10);
    parameters_.wheelbase = declare_parameter<double>("wheelbase", 1.005);
    parameters_.max_steer = declare_parameter<double>("max_steering_angle", 0.5235988);
    parameters_.steering_samples = declare_parameter<int>("steering_samples", 5);
    parameters_.allow_reverse = declare_parameter<bool>("allow_reverse", true);
    parameters_.reverse_penalty = declare_parameter<double>("reverse_penalty", 1.20);
    parameters_.gear_switch_penalty = declare_parameter<double>("gear_switch_penalty", 4.0);
    parameters_.steering_penalty = declare_parameter<double>("steering_penalty", 0.15);
    parameters_.steering_change_penalty = declare_parameter<double>("steering_change_penalty", 0.35);
    parameters_.heading_heuristic_weight = declare_parameter<double>("heading_heuristic_weight", 0.50);
    parameters_.goal_xy_tolerance = declare_parameter<double>("goal_xy_tolerance", 0.30);
    parameters_.goal_yaw_tolerance = declare_parameter<double>("goal_yaw_tolerance_deg", 12.0) * parking_cpp::kPi / 180.0;
    parameters_.max_iterations = declare_parameter<int>("max_iterations", 120000);

    const auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map", 10,
      std::bind(&HybridAStarPlannerNode::on_odom, this, std::placeholders::_1));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, transient_qos,
      std::bind(&HybridAStarPlannerNode::on_map, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/parking/goal", transient_qos,
      std::bind(&HybridAStarPlannerNode::on_goal, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/parking/path", transient_qos);
    directions_pub_ = create_publisher<std_msgs::msg::Int8MultiArray>(
      "/parking/directions", transient_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("/parking/status", transient_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/parking/debug_markers", 10);

    publish_status("IDLE");
    RCLCPP_INFO(
      get_logger(),
      "C++ Hybrid A* started: wheelbase=%.3fm, max_steer=%.1fdeg, map=%s",
      parameters_.wheelbase, parameters_.max_steer * 180.0 / parking_cpp::kPi, map_topic_.c_str());
  }

  ~HybridAStarPlannerNode() override
  {
    if (planning_thread_.joinable()) {
      planning_thread_.join();
    }
  }

private:
  void publish_status(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::scoped_lock lock(state_mutex_);
    current_pose_ = Pose2D{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      parking_cpp::yaw_from_quaternion(msg->pose.pose.orientation)};
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::scoped_lock lock(state_mutex_);
    grid_ = *msg;
    grid_time_s_ = now().seconds();
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Pose2D start;
    std::optional<nav_msgs::msg::OccupancyGrid> grid_snapshot;
    double grid_age = std::numeric_limits<double>::infinity();
    {
      std::scoped_lock lock(state_mutex_);
      if (!current_pose_.has_value()) {
        RCLCPP_ERROR(get_logger(), "No /odometry/filtered_map; cannot plan.");
        publish_status("FAILED");
        return;
      }
      start = *current_pose_;
      grid_snapshot = grid_;
      if (grid_.has_value()) {
        grid_age = now().seconds() - grid_time_s_;
      }
    }

    if (!grid_snapshot.has_value() && !allow_empty_map_) {
      RCLCPP_ERROR(get_logger(), "No /parking/local_costmap; planning rejected.");
      publish_status("FAILED");
      return;
    }
    if (grid_snapshot.has_value() && map_timeout_s_ > 0.0 && grid_age > map_timeout_s_) {
      RCLCPP_ERROR(get_logger(), "Parking costmap is stale (%.3fs).", grid_age);
      publish_status("FAILED");
      return;
    }
    if (planning_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Planner is already running; new goal ignored.");
      return;
    }

    if (planning_thread_.joinable()) {
      planning_thread_.join();
    }

    const Pose2D goal{
      msg->pose.position.x,
      msg->pose.position.y,
      parking_cpp::yaw_from_quaternion(msg->pose.orientation)};
    planning_thread_ = std::thread(
      &HybridAStarPlannerNode::run_plan, this, start, goal, grid_snapshot);
  }

  void run_plan(
    Pose2D start, Pose2D goal,
    std::optional<nav_msgs::msg::OccupancyGrid> grid_snapshot)
  {
    publish_status("PLANNING");
    try {
      const auto bounds = std::make_tuple(
        std::min(start.x, goal.x) - empty_map_margin_m_,
        std::max(start.x, goal.x) + empty_map_margin_m_,
        std::min(start.y, goal.y) - empty_map_margin_m_,
        std::max(start.y, goal.y) + empty_map_margin_m_);
      GridCollisionChecker checker(
        grid_snapshot, occupied_threshold_, unknown_is_occupied_,
        vehicle_front_m_, vehicle_rear_m_, vehicle_half_width_m_,
        footprint_sample_m_, bounds);
      HybridAStar planner(std::move(checker), parameters_);
      const auto result = planner.plan(start, goal);
      if (!result.has_value() || result->size() < 2U) {
        RCLCPP_ERROR(get_logger(), "Hybrid A* failed to find a path.");
        publish_status("FAILED");
      } else {
        publish_result(*result, goal);
        RCLCPP_INFO(
          get_logger(), "Hybrid A* path ready: %zu points", result->size());
        publish_status("READY");
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Planner exception: %s", exception.what());
      publish_status("FAILED");
    }
    planning_ = false;
  }

  void publish_result(const std::vector<SearchNode> & result, const Pose2D & goal)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    std_msgs::msg::Int8MultiArray directions;
    directions.data.reserve(result.size());

    for (const auto & node : result) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = node.x;
      pose.pose.position.y = node.y;
      pose.pose.orientation = parking_cpp::quaternion_from_yaw(node.yaw);
      path.poses.push_back(pose);
      directions.data.push_back(static_cast<int8_t>(node.direction >= 0 ? 1 : -1));
    }
    path_pub_->publish(path);
    directions_pub_->publish(directions);
    publish_markers(result, goal);
  }

  void publish_markers(const std::vector<SearchNode> & result, const Pose2D & goal)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = now();
    line.ns = "hybrid_astar_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.06;
    line.color.a = 1.0;
    line.color.g = 1.0;
    for (const auto & node : result) {
      geometry_msgs::msg::Point point;
      point.x = node.x;
      point.y = node.y;
      point.z = 0.05;
      line.points.push_back(point);
    }
    array.markers.push_back(line);

    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header = line.header;
    goal_marker.ns = "hybrid_astar_goal";
    goal_marker.id = 1;
    goal_marker.type = visualization_msgs::msg::Marker::ARROW;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position.x = goal.x;
    goal_marker.pose.position.y = goal.y;
    goal_marker.pose.orientation = parking_cpp::quaternion_from_yaw(goal.yaw);
    goal_marker.scale.x = 0.8;
    goal_marker.scale.y = 0.15;
    goal_marker.scale.z = 0.15;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 1.0;
    array.markers.push_back(goal_marker);
    marker_pub_->publish(array);
  }

  std::string frame_id_;
  std::string map_topic_;
  bool allow_empty_map_{false};
  double map_timeout_s_{0.75};
  double empty_map_margin_m_{8.0};
  int occupied_threshold_{50};
  bool unknown_is_occupied_{false};
  double vehicle_front_m_{0.77};
  double vehicle_rear_m_{0.77};
  double vehicle_half_width_m_{0.505};
  double footprint_sample_m_{0.10};
  PlannerParameters parameters_;

  std::mutex state_mutex_;
  std::optional<Pose2D> current_pose_;
  std::optional<nav_msgs::msg::OccupancyGrid> grid_;
  double grid_time_s_{-1e9};
  std::atomic_bool planning_{false};
  std::thread planning_thread_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr directions_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridAStarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
