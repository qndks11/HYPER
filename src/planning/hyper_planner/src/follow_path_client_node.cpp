#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    // Strip whitespace and a trailing '\r' from CRLF files.
    const auto begin = field.find_first_not_of(" \t\r\n");
    const auto end = field.find_last_not_of(" \t\r\n");
    fields.push_back(begin == std::string::npos ? "" : field.substr(begin, end - begin + 1));
  }
  return fields;
}
}  // namespace

// Loads a waypoint CSV recorded by hyper_waypoint, converts it to a nav_msgs/Path
// and sends it to the nav2 controller server's `follow_path` action.
class FollowPathClient : public rclcpp::Node
{
public:
  FollowPathClient() : Node("follow_path_client")
  {
    waypoint_csv_ = declare_parameter<std::string>("waypoint_csv", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    action_name_ = declare_parameter<std::string>("action_name", "follow_path");
    controller_id_ = declare_parameter<std::string>("controller_id", "FollowPath");
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
    min_spacing_m_ = declare_parameter<double>("min_spacing_m", 0.0);
    server_wait_timeout_s_ = declare_parameter<double>("server_wait_timeout_sec", 20.0);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    shutdown_on_finish_ = declare_parameter<bool>("shutdown_on_finish", false);
    start_from_nearest_ = declare_parameter<bool>("start_from_nearest", true);
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "body_link");
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_sec", 5.0);
    max_start_distance_m_ = declare_parameter<double>("max_start_distance_m", 0.0);
    lead_in_spacing_m_ = declare_parameter<double>("lead_in_spacing_m", 0.5);

    if (start_from_nearest_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      // spin_thread = true: lookupTransform() below blocks with a timeout, so the listener
      // needs its own thread to keep filling the buffer while we wait.
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    }

    // Latched so RViz picks the path up whenever it subscribes.
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/path", rclcpp::QoS(1).transient_local());

    client_ = rclcpp_action::create_client<FollowPath>(this, action_name_);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = load_and_send();
        response->message = response->success
          ? "Goal sent to '" + action_name_ + "'."
          : "Failed to send goal; see node log.";
      });

    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (!goal_handle_) {
          response->success = false;
          response->message = "No active goal.";
          return;
        }
        client_->async_cancel_goal(goal_handle_);
        response->success = true;
        response->message = "Cancel requested.";
      });

    if (auto_start_) {
      // Fire once from the executor so the constructor does not block on the action server.
      start_timer_ = create_wall_timer(
        std::chrono::milliseconds(0), [this]() {
          start_timer_->cancel();
          if (!load_and_send() && shutdown_on_finish_) {
            rclcpp::shutdown();
          }
        });
    } else {
      RCLCPP_INFO(
        get_logger(), "auto_start is false; call '%s/start' to send the path.",
        get_name());
    }
  }

private:
  bool load_and_send()
  {
    nav_msgs::msg::Path path;
    if (!load_path(path)) {
      return false;
    }
    if (!start_near_robot(path)) {
      return false;
    }

    path_pub_->publish(path);

    if (!client_->wait_for_action_server(
        std::chrono::duration<double>(server_wait_timeout_s_)))
    {
      RCLCPP_ERROR(
        get_logger(), "Action server '%s' not available after %.1f s. Is nav2's "
        "controller_server running and activated?", action_name_.c_str(), server_wait_timeout_s_);
      return false;
    }

    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by '%s'.", action_name_.c_str());
        if (shutdown_on_finish_) {
          rclcpp::shutdown();
        }
        return;
      }
      goal_handle_ = handle;
      RCLCPP_INFO(get_logger(), "Goal accepted; following path.");
    };
    options.feedback_callback = [this](
      GoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "distance_to_goal=%.2f m, speed=%.2f m/s",
        feedback->distance_to_goal, feedback->speed);
    };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
      goal_handle_.reset();
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Path following succeeded.");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "Path following was aborted by the controller.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(get_logger(), "Path following was canceled.");
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Path following returned an unknown result code.");
          break;
      }
      if (shutdown_on_finish_) {
        rclcpp::shutdown();
      }
    };

    RCLCPP_INFO(
      get_logger(), "Sending %zu-pose path to '%s' (controller_id='%s', goal_checker_id='%s').",
      path.poses.size(), action_name_.c_str(), controller_id_.c_str(), goal_checker_id_.c_str());
    client_->async_send_goal(goal, options);
    return true;
  }

  // nav2's controller only searches for the closest path pose within a costmap-sized window
  // from the head of the path, then clips the result to the local costmap. So a path whose
  // first pose sits outside that 20 x 20 m window leaves MPPI with an empty reference and the
  // action aborts on the first tick ("Resulting plan has 0 poses in it"). Here we drop the
  // poses the vehicle has already passed and, if it is sitting off the path, prepend a
  // straight lead-in from its current pose so the path always starts under the vehicle.
  bool start_near_robot(nav_msgs::msg::Path & path)
  {
    if (!start_from_nearest_) {
      return true;
    }

    geometry_msgs::msg::PoseStamped robot;
    if (!lookup_robot_pose(path.header.frame_id, robot)) {
      return false;
    }

    size_t nearest = 0;
    double nearest_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const double d = std::hypot(
        path.poses[i].pose.position.x - robot.pose.position.x,
        path.poses[i].pose.position.y - robot.pose.position.y);
      if (d < nearest_dist) {
        nearest_dist = d;
        nearest = i;
      }
    }

    if (max_start_distance_m_ > 0.0 && nearest_dist > max_start_distance_m_) {
      RCLCPP_ERROR(
        get_logger(),
        "Nearest waypoint (#%zu) is %.1f m away, beyond max_start_distance_m (%.1f). Refusing "
        "to send. Move the vehicle onto the course, or raise the limit.",
        nearest, nearest_dist, max_start_distance_m_);
      return false;
    }

    if (nearest > 0) {
      path.poses.erase(
        path.poses.begin(), path.poses.begin() + static_cast<std::ptrdiff_t>(nearest));
    }
    const size_t lead_in = insert_lead_in(path, robot, nearest_dist);

    RCLCPP_INFO(
      get_logger(),
      "Starting at waypoint #%zu, %.2f m from the vehicle (%zu passed pose(s) dropped, "
      "%zu lead-in pose(s) prepended); %zu poses remain.",
      nearest, nearest_dist, nearest, lead_in, path.poses.size());
    return true;
  }

  // Straight line from the vehicle to the first remaining path pose, sampled every
  // lead_in_spacing_m so no gap can drop the reference out of the local costmap.
  // Note this ignores the vehicle's heading and turning radius -- if the path starts behind
  // the vehicle, MPPI has to reverse or loop around to pick it up.
  size_t insert_lead_in(
    nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & robot, double distance)
  {
    if (lead_in_spacing_m_ <= 0.0 || path.poses.empty() || distance <= lead_in_spacing_m_) {
      return 0;
    }

    const auto target = path.poses.front().pose.position;
    const double dx = target.x - robot.pose.position.x;
    const double dy = target.y - robot.pose.position.y;
    const auto orientation = quaternion_from_yaw(std::atan2(dy, dx));
    const auto steps = static_cast<size_t>(std::ceil(distance / lead_in_spacing_m_));

    std::vector<geometry_msgs::msg::PoseStamped> lead_in;
    lead_in.reserve(steps);
    for (size_t i = 0; i < steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = robot.pose.position.x + t * dx;
      pose.pose.position.y = robot.pose.position.y + t * dy;
      pose.pose.orientation = orientation;
      lead_in.push_back(pose);
    }

    path.poses.insert(path.poses.begin(), lead_in.begin(), lead_in.end());
    return lead_in.size();
  }

  bool lookup_robot_pose(const std::string & frame, geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      // TimePointZero = latest available. The vehicle is standing still when a goal is sent,
      // so the small staleness that allows does not matter.
      const auto tf = tf_buffer_->lookupTransform(
        frame, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_s_));
      pose.header = tf.header;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        get_logger(), "No '%s' -> '%s' transform after %.1f s: %s. Is odometry running?",
        frame.c_str(), robot_base_frame_.c_str(), tf_timeout_s_, ex.what());
      return false;
    }
  }

  // Reads the CSV by header name, so both the minimal `idx,x,y,yaw,frame_id` layout
  // and the full recorder layout (with GPS/IMU columns) work.
  bool load_path(nav_msgs::msg::Path & path)
  {
    if (waypoint_csv_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'waypoint_csv' is empty.");
      return false;
    }

    std::ifstream file(waypoint_csv_);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open '%s'.", waypoint_csv_.c_str());
      return false;
    }

    std::string line;
    if (!std::getline(file, line)) {
      RCLCPP_ERROR(get_logger(), "'%s' is empty.", waypoint_csv_.c_str());
      return false;
    }

    std::unordered_map<std::string, size_t> column;
    const auto header = split_csv_line(line);
    for (size_t i = 0; i < header.size(); ++i) {
      column[header[i]] = i;
    }
    if (column.count("x") == 0 || column.count("y") == 0) {
      RCLCPP_ERROR(
        get_logger(), "'%s' has no 'x'/'y' header columns.", waypoint_csv_.c_str());
      return false;
    }
    const size_t x_col = column["x"];
    const size_t y_col = column["y"];
    const bool has_yaw = column.count("yaw") > 0;
    const bool has_frame = column.count("frame_id") > 0;

    std::string csv_frame;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> yaws;   // NaN where the CSV has no usable yaw.
    size_t line_no = 1;
    size_t skipped = 0;

    while (std::getline(file, line)) {
      ++line_no;
      if (line.empty()) {
        continue;
      }
      const auto fields = split_csv_line(line);
      if (fields.size() <= std::max(x_col, y_col) ||
        fields[x_col].empty() || fields[y_col].empty())
      {
        ++skipped;
        continue;
      }

      double x = 0.0;
      double y = 0.0;
      try {
        x = std::stod(fields[x_col]);
        y = std::stod(fields[y_col]);
      } catch (const std::exception &) {
        RCLCPP_WARN(get_logger(), "Line %zu: unparsable x/y, skipping.", line_no);
        ++skipped;
        continue;
      }

      if (min_spacing_m_ > 0.0 && !xs.empty()) {
        if (std::hypot(x - xs.back(), y - ys.back()) < min_spacing_m_) {
          ++skipped;
          continue;
        }
      }

      double yaw = std::numeric_limits<double>::quiet_NaN();
      if (has_yaw && fields.size() > column["yaw"] && !fields[column["yaw"]].empty()) {
        try {
          yaw = std::stod(fields[column["yaw"]]);
        } catch (const std::exception &) {
          // Leave as NaN; it gets derived from the neighbouring points below.
        }
      }

      if (csv_frame.empty() && has_frame && fields.size() > column["frame_id"]) {
        csv_frame = fields[column["frame_id"]];
      }

      xs.push_back(x);
      ys.push_back(y);
      yaws.push_back(yaw);
    }

    if (xs.empty()) {
      RCLCPP_ERROR(get_logger(), "'%s' contains no usable waypoints.", waypoint_csv_.c_str());
      return false;
    }

    path.header.frame_id = csv_frame.empty() ? frame_id_ : csv_frame;
    path.header.stamp = now();
    path.poses.reserve(xs.size());

    for (size_t i = 0; i < xs.size(); ++i) {
      double yaw = yaws[i];
      if (std::isnan(yaw)) {
        // Point the pose at the next waypoint (or back along the last segment at the end).
        if (i + 1 < xs.size()) {
          yaw = std::atan2(ys[i + 1] - ys[i], xs[i + 1] - xs[i]);
        } else if (i > 0) {
          yaw = std::atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]);
        } else {
          yaw = 0.0;
        }
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = xs[i];
      pose.pose.position.y = ys[i];
      pose.pose.orientation = quaternion_from_yaw(yaw);
      path.poses.push_back(pose);
    }

    RCLCPP_INFO(
      get_logger(), "Loaded %zu waypoints from '%s' in frame '%s' (%zu rows skipped).",
      path.poses.size(), waypoint_csv_.c_str(), path.header.frame_id.c_str(), skipped);
    return true;
  }

  std::string waypoint_csv_;
  std::string frame_id_;
  std::string action_name_;
  std::string controller_id_;
  std::string goal_checker_id_;
  double min_spacing_m_{0.0};
  double server_wait_timeout_s_{20.0};
  bool auto_start_{true};
  bool shutdown_on_finish_{false};
  bool start_from_nearest_{true};
  std::string robot_base_frame_;
  double tf_timeout_s_{5.0};
  double max_start_distance_m_{0.0};
  double lead_in_spacing_m_{0.5};

  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowPathClient>());
  rclcpp::shutdown();
  return 0;
}
