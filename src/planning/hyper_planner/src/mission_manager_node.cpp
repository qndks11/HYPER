// HYPER 미션 매니저.
//
// config/mission.yaml의 스텝 큐를 순서대로 실행합니다. 핵심 아이디어는
// "한 스텝 = FollowPath 골 하나"입니다 -- 정지선/신호등/주차 지점이 곧 세그먼트의 끝이므로
// "도착했는가?"를 따로 판정할 필요 없이 nav2의 goal checker가 알려 줍니다.
//
// 정지에는 별도의 정지 명령이 필요 없습니다. 골을 보내지 않으면 /cmd_vel이 끊기고
// cmd_vel_to_ackermann의 워치독(input_timeout 0.3초)이 차를 세웁니다.
//
// 장애물 회피는 스텝이 아닙니다 -- MPPI가 local costmap을 보며 해당 drive 스텝 안에서
// 알아서 처리합니다.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
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
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "hyper_planner/common.hpp"
#include "hyper_planner/path_loader.hpp"

namespace
{
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

enum class StepType
{
  kDrive,
  kStop,
  kWaitSignal,
};

struct Step
{
  StepType type{StepType::kDrive};

  // drive
  std::string label;                 // mission.yaml의 `until`
  std::size_t begin_index{0};        // CSV 인덱스 (양끝 포함)
  std::size_t end_index{0};
  std::string controller_id;
  std::string goal_checker_id;
  bool reverse{false};

  // stop
  double duration_s{0.0};

  // wait_signal
  std::vector<std::string> accepted;
  double timeout_s{60.0};
  int debounce_frames{3};
};

// 스텝 사이를 오가는 상태. kStarting은 "이번 drive 스텝의 골을 아직 못 보냈다"는
// 뜻이고, 실제 전송은 항상 타이머에서 일어납니다(액션 콜백 안에서 새 골을 보내지
// 않으려고 일부러 한 단계 끼워 둔 것입니다).
enum class Phase
{
  kIdle,
  kStarting,
  kDriving,
  kHolding,
  kWaiting,
  kFinished,
  kFailed,
};

std::vector<std::string> split_values(const std::string & text)
{
  std::vector<std::string> values;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto begin = item.find_first_not_of(" \t");
    const auto end = item.find_last_not_of(" \t");
    if (begin != std::string::npos) {
      values.push_back(item.substr(begin, end - begin + 1));
    }
  }
  return values;
}

const char * type_name(StepType type)
{
  switch (type) {
    case StepType::kDrive: return "drive";
    case StepType::kStop: return "stop";
    case StepType::kWaitSignal: return "wait_signal";
  }
  return "?";
}
}  // namespace

class MissionManager : public rclcpp::Node
{
public:
  MissionManager() : Node("mission_manager")
  {
    mission_yaml_ = declare_parameter<std::string>("mission_yaml", "");
    waypoint_csv_ = declare_parameter<std::string>("waypoint_csv", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    action_name_ = declare_parameter<std::string>("action_name", "follow_path");
    sign_topic_ = declare_parameter<std::string>("sign_topic", "/perception/sign");
    default_controller_id_ = declare_parameter<std::string>("controller_id", "FollowPath");
    default_goal_checker_id_ =
      declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
    min_spacing_m_ = declare_parameter<double>("min_spacing_m", 0.0);
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "body_link");
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_sec", 5.0);
    lead_in_spacing_m_ = declare_parameter<double>("lead_in_spacing_m", 0.5);
    // 골로 보내기 직전에 경로를 이 간격으로 다시 깝니다(0이면 끔). 녹화 CSV의 약
    // 0.59 m 간격은 MPPI PathAlignCritic이 "가장 가까운 경로 점까지의 거리"를 벌점으로
    // 쓰기 때문에 ±0.3 m의 코너 컷 사각지대를 만듭니다. 자세한 설명은 path_loader.hpp의
    // resample_path 주석을 보세요.
    //
    // 이 값을 바꾸면 nav2_controller.yaml에서 "점 개수"로 세는 파라미터
    // (PathFollow/PathAngle/PathAlign의 offset_from_furthest)도 같이 조정해야 합니다.
    path_resample_spacing_m_ = declare_parameter<double>("path_resample_spacing_m", 0.25);
    max_start_distance_m_ = declare_parameter<double>("max_start_distance_m", 0.0);
    server_wait_timeout_s_ = declare_parameter<double>("server_wait_timeout_sec", 30.0);
    auto_start_ = declare_parameter<bool>("auto_start", false);
    // 컨트롤러가 abort 했지만 차가 이미 세그먼트 끝 이 거리 안에 있으면 도착으로 칩니다.
    // goal checker 공차를 조금 못 맞춰 progress checker에 걸리는 흔한 경우에서,
    // 같은 골을 무한 재전송하며 미션이 죽는 걸 막습니다.
    arrival_slack_m_ = declare_parameter<double>("arrival_slack_m", 0.6);
    goal_retry_limit_ = declare_parameter<int>("goal_retry_limit", 2);
    // 신호를 timeout_s 안에 못 보면: true면 그냥 출발(대회에서 멈춰 서는 것보다 낫다),
    // false면 미션 실패로 정지.
    proceed_on_signal_timeout_ = declare_parameter<bool>("proceed_on_signal_timeout", true);

    // 기본 생성된 rclcpp::Time은 시스템 시계라, use_sim_time일 때 now()와 비교하면
    // 시간 소스가 달라 예외가 납니다. 노드 시계로 초기화해 둡니다.
    hold_until_ = now();
    wait_until_ = now();
    starting_since_ = now();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // spin_thread = true: 아래 lookupTransform()이 타임아웃까지 블록하므로 리스너는
    // 버퍼를 계속 채울 자기 스레드가 필요합니다.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    // 나중에 붙는 RViz/툴이 받을 수 있도록 latch 합니다.
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).transient_local());

    sign_sub_ = create_subscription<std_msgs::msg::String>(
      sign_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {on_sign(msg->data);});

    client_ = rclcpp_action::create_client<FollowPath>(this, action_name_);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_start(response);
      });
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_cancel(response);
      });
    skip_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/skip",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_skip(response);
      });
    restart_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/restart",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_restart(response);
      });

    if (!load_mission()) {
      phase_ = Phase::kFailed;
      // 노드는 살려 둡니다 -- 죽으면 launch 로그에서 이유를 놓치기 쉽습니다.
      RCLCPP_FATAL(get_logger(), "Mission not loaded; refusing to run.");
      publish_status("failed: mission not loaded");
      return;
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {tick();});

    publish_status("idle");
    if (auto_start_) {
      RCLCPP_INFO(get_logger(), "auto_start is true; starting the mission.");
      begin_step();
    } else {
      RCLCPP_INFO(
        get_logger(), "Ready. Call '%s/start' to run the mission (%zu steps).",
        get_name(), steps_.size());
    }
  }

private:
  // ---------------------------------------------------------------- 로드/검증

  bool load_mission()
  {
    std::string error;
    if (!hyper_planner::load_waypoint_csv(waypoint_csv_, min_spacing_m_, waypoints_, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }
    if (waypoints_.frame_id.empty()) {
      waypoints_.frame_id = frame_id_;
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %zu waypoints from '%s' in frame '%s' (%zu rows skipped).",
      waypoints_.points.size(), waypoint_csv_.c_str(), waypoints_.frame_id.c_str(),
      waypoints_.skipped_rows);

    if (mission_yaml_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'mission_yaml' is empty.");
      return false;
    }

    YAML::Node root;
    try {
      root = YAML::LoadFile(mission_yaml_);
    } catch (const YAML::Exception & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to parse '%s': %s", mission_yaml_.c_str(), ex.what());
      return false;
    }

    const double snap_tolerance_m = root["label_snap_tolerance_m"]
      ? root["label_snap_tolerance_m"].as<double>() : 1.0;

    if (!root["labels"] || !root["labels"].IsMap() || root["labels"].size() == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "'%s' has no 'labels'. Place them first:\n"
        "  python3 src/planning/hyper_waypoint/scripts/label_waypoints.py %s",
        mission_yaml_.c_str(), waypoint_csv_.c_str());
      return false;
    }
    if (!root["steps"] || !root["steps"].IsSequence() || root["steps"].size() == 0) {
      RCLCPP_ERROR(get_logger(), "'%s' has no 'steps'.", mission_yaml_.c_str());
      return false;
    }

    // 라벨을 최근접 웨이포인트로 스냅합니다. 라벨은 좌표로 저장되어 있어 코스를 다시
    // 녹화해도 살아남지만, 그만큼 엉뚱한 데 붙을 수도 있으므로 여기서 걸러냅니다.
    for (const auto & entry : root["labels"]) {
      const auto name = entry.first.as<std::string>();
      const YAML::Node & point = entry.second;
      if (!point["x"] || !point["y"]) {
        RCLCPP_ERROR(get_logger(), "Label '%s' has no x/y.", name.c_str());
        return false;
      }
      const double x = point["x"].as<double>();
      const double y = point["y"].as<double>();

      std::size_t nearest = 0;
      double best = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < waypoints_.points.size(); ++i) {
        const double d = std::hypot(waypoints_.points[i].x - x, waypoints_.points[i].y - y);
        if (d < best) {
          best = d;
          nearest = i;
        }
      }
      if (best > snap_tolerance_m) {
        RCLCPP_ERROR(
          get_logger(),
          "Label '%s' (%.3f, %.3f) is %.2f m from the nearest waypoint (#%zu), beyond "
          "label_snap_tolerance_m (%.2f). Re-place it with label_waypoints.py, or check that "
          "mission.yaml and '%s' describe the same course.",
          name.c_str(), x, y, best, nearest, snap_tolerance_m, waypoint_csv_.c_str());
        return false;
      }
      label_index_[name] = nearest;
      RCLCPP_INFO(
        get_logger(), "Label '%s' -> waypoint #%zu (%.2f m away).", name.c_str(), nearest, best);
    }

    // 스텝을 읽으면서 drive 세그먼트를 잇습니다. 세그먼트 시작점은 직전 drive 스텝의
    // 도착점이고, 첫 세그먼트만 CSV 처음부터 시작합니다.
    std::size_t cursor = 0;
    bool seen_drive = false;
    for (std::size_t i = 0; i < root["steps"].size(); ++i) {
      const YAML::Node & node = root["steps"][i];
      const auto type_text = node["type"] ? node["type"].as<std::string>() : std::string();

      Step step;
      if (type_text == "drive") {
        step.type = StepType::kDrive;
        if (!node["until"]) {
          RCLCPP_ERROR(get_logger(), "Step %zu (drive) has no 'until'.", i);
          return false;
        }
        step.label = node["until"].as<std::string>();
        const auto found = label_index_.find(step.label);
        if (found == label_index_.end()) {
          RCLCPP_ERROR(
            get_logger(), "Step %zu references label '%s', which is not in 'labels'.",
            i, step.label.c_str());
          return false;
        }
        const std::size_t target = found->second;
        // 코스는 한 번 주행해 녹화한 것이므로 라벨은 CSV를 따라 단조 증가해야 합니다.
        // 그렇지 않으면 세그먼트가 비거나 거꾸로 뒤집힙니다.
        if (seen_drive && target <= cursor) {
          RCLCPP_ERROR(
            get_logger(),
            "Step %zu: label '%s' is at waypoint #%zu, which is not past the previous step's "
            "#%zu. Labels must advance along the recorded course; re-place '%s'.",
            i, step.label.c_str(), target, cursor, step.label.c_str());
          return false;
        }
        step.begin_index = seen_drive ? cursor : 0;
        step.end_index = target;
        step.reverse = node["reverse"] ? node["reverse"].as<bool>() : false;
        step.controller_id = node["controller"]
          ? node["controller"].as<std::string>() : default_controller_id_;
        step.goal_checker_id = node["goal_checker"]
          ? node["goal_checker"].as<std::string>() : default_goal_checker_id_;

        // reverse 플래그가 녹화된 실제 주행 방향과 맞는지 확인합니다. 틀리면 RPP가
        // 엉뚱한 방향으로 당기므로 바로 알아채는 편이 낫습니다.
        const double opposing = hyper_planner::reverse_fraction(
          waypoints_.points, step.begin_index, step.end_index);
        if (step.reverse && opposing < 0.5) {
          RCLCPP_WARN(
            get_logger(),
            "Step %zu (until '%s') is marked reverse, but only %.0f%% of the recorded segment "
            "has the body heading opposing travel. Was this stretch actually recorded driving "
            "backwards?", i, step.label.c_str(), 100.0 * opposing);
        } else if (!step.reverse && opposing > 0.5) {
          RCLCPP_WARN(
            get_logger(),
            "Step %zu (until '%s') is NOT marked reverse, but %.0f%% of the recorded segment "
            "has the body heading opposing travel. Add 'reverse: true' and a reversing "
            "controller, or MPPI will try to drive it nose-first.",
            i, step.label.c_str(), 100.0 * opposing);
        }

        cursor = target;
        seen_drive = true;
      } else if (type_text == "stop") {
        step.type = StepType::kStop;
        step.duration_s = node["duration_s"] ? node["duration_s"].as<double>() : 0.0;
      } else if (type_text == "wait_signal") {
        step.type = StepType::kWaitSignal;
        step.accepted = split_values(
          node["value"] ? node["value"].as<std::string>() : std::string("green"));
        step.timeout_s = node["timeout_s"] ? node["timeout_s"].as<double>() : 60.0;
        step.debounce_frames = node["debounce_frames"]
          ? node["debounce_frames"].as<int>() : 3;
        if (step.accepted.empty()) {
          RCLCPP_ERROR(get_logger(), "Step %zu (wait_signal) has an empty 'value'.", i);
          return false;
        }
      } else {
        RCLCPP_ERROR(
          get_logger(), "Step %zu has unknown type '%s' (expected drive/stop/wait_signal).",
          i, type_text.c_str());
        return false;
      }
      steps_.push_back(step);
    }

    RCLCPP_INFO(
      get_logger(), "Mission '%s' loaded: %zu steps, %zu labels.",
      mission_yaml_.c_str(), steps_.size(), label_index_.size());
    return true;
  }

  // ---------------------------------------------------------------- 상태 진행

  void begin_step()
  {
    if (step_index_ >= steps_.size()) {
      phase_ = Phase::kFinished;
      RCLCPP_INFO(get_logger(), "Mission complete (%zu steps).", steps_.size());
      publish_status("finished");
      return;
    }

    const Step & step = steps_[step_index_];
    retries_ = 0;
    switch (step.type) {
      case StepType::kDrive:
        phase_ = Phase::kStarting;
        starting_since_ = now();
        RCLCPP_INFO(
          get_logger(), "%s drive -> '%s' (wp #%zu..#%zu%s, controller='%s', goal_checker='%s')",
          progress().c_str(), step.label.c_str(), step.begin_index, step.end_index,
          step.reverse ? ", reverse" : "", step.controller_id.c_str(),
          step.goal_checker_id.c_str());
        break;
      case StepType::kStop:
        phase_ = Phase::kHolding;
        hold_until_ = now() + rclcpp::Duration::from_seconds(step.duration_s);
        RCLCPP_INFO(
          get_logger(), "%s stop for %.1f s.", progress().c_str(), step.duration_s);
        break;
      case StepType::kWaitSignal:
        phase_ = Phase::kWaiting;
        wait_until_ = now() + rclcpp::Duration::from_seconds(step.timeout_s);
        sign_streak_ = 0;
        RCLCPP_INFO(
          get_logger(), "%s waiting for '%s' on %s (%d frame(s), timeout %.0f s).",
          progress().c_str(), join(step.accepted).c_str(), sign_topic_.c_str(),
          step.debounce_frames, step.timeout_s);
        break;
    }
    publish_status(status_text());
  }

  void advance()
  {
    ++step_index_;
    begin_step();
  }

  void fail(const std::string & reason)
  {
    phase_ = Phase::kFailed;
    RCLCPP_ERROR(get_logger(), "Mission failed at step %zu: %s", step_index_, reason.c_str());
    publish_status("failed: " + reason);
  }

  void tick()
  {
    switch (phase_) {
      case Phase::kStarting:
        try_send_goal();
        break;
      case Phase::kHolding:
        if (now() >= hold_until_) {
          advance();
        }
        break;
      case Phase::kWaiting: {
        const Step & step = steps_[step_index_];
        if (sign_streak_ >= step.debounce_frames) {
          RCLCPP_INFO(get_logger(), "Signal '%s' confirmed; going.", last_sign_.c_str());
          advance();
        } else if (now() >= wait_until_) {
          if (proceed_on_signal_timeout_) {
            RCLCPP_WARN(
              get_logger(),
              "No '%s' within %.0f s (last saw '%s'). Proceeding anyway -- check the traffic "
              "light detector.", join(step.accepted).c_str(), step.timeout_s,
              last_sign_.empty() ? "nothing" : last_sign_.c_str());
            advance();
          } else {
            fail("timed out waiting for signal '" + join(step.accepted) + "'");
          }
        }
        break;
      }
      case Phase::kIdle:
      case Phase::kDriving:
      case Phase::kFinished:
      case Phase::kFailed:
        break;
    }
  }

  // ---------------------------------------------------------------- drive 스텝

  void try_send_goal()
  {
    if (!client_->action_server_is_ready()) {
      if ((now() - starting_since_).seconds() > server_wait_timeout_s_) {
        fail(
          "action server '" + action_name_ + "' never became available. Is nav2's "
          "controller_server running and activated?");
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for action server '%s'...",
        action_name_.c_str());
      return;
    }

    const Step & step = steps_[step_index_];
    nav_msgs::msg::Path path = hyper_planner::make_path(
      waypoints_.points, step.begin_index, step.end_index, waypoints_.frame_id, now(),
      step.reverse);
    if (path.poses.size() < 2) {
      fail("segment for '" + step.label + "' has fewer than 2 poses");
      return;
    }
    if (!trim_to_robot(path, step.reverse)) {
      // 사유는 trim_to_robot이 이미 로그로 남겼습니다. 다음 tick에 다시 시도합니다
      // (오도메트리가 아직 안 올라온 것뿐일 수 있으므로).
      if ((now() - starting_since_).seconds() > server_wait_timeout_s_) {
        fail("could not locate the vehicle in frame '" + path.header.frame_id + "'");
      }
      return;
    }

    path_pub_->publish(path);

    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = step.controller_id;
    goal.goal_checker_id = step.goal_checker_id;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
      if (!handle) {
        fail("goal rejected by '" + action_name_ + "'");
        return;
      }
      goal_handle_ = handle;
      phase_ = Phase::kDriving;
      publish_status(status_text());
    };
    options.feedback_callback = [this](
      GoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s distance_to_goal=%.2f m, speed=%.2f m/s",
        progress().c_str(), feedback->distance_to_goal, feedback->speed);
    };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
      on_result(result);
    };

    RCLCPP_INFO(
      get_logger(), "%s sending %zu-pose path to '%s'.",
      progress().c_str(), path.poses.size(), action_name_.c_str());
    phase_ = Phase::kDriving;   // 응답이 올 때까지 재전송을 막습니다.
    client_->async_send_goal(goal, options);
  }

  void on_result(const GoalHandle::WrappedResult & result)
  {
    goal_handle_.reset();
    if (phase_ == Phase::kFinished || phase_ == Phase::kFailed || phase_ == Phase::kIdle) {
      return;   // 취소로 이미 정리된 뒤 도착한 결과.
    }

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "%s arrived.", progress().c_str());
        advance();
        return;

      case rclcpp_action::ResultCode::CANCELED:
        if (skip_requested_) {
          skip_requested_ = false;
          RCLCPP_WARN(get_logger(), "%s skipped.", progress().c_str());
          advance();
        } else {
          phase_ = Phase::kIdle;
          RCLCPP_WARN(
            get_logger(), "%s canceled; call '%s/start' to resume.",
            progress().c_str(), get_name());
          publish_status("canceled");
        }
        return;

      case rclcpp_action::ResultCode::ABORTED:
      default: {
        const Step & step = steps_[step_index_];
        double distance = 0.0;
        // goal checker 공차를 아슬아슬하게 못 맞춰 progress checker에 걸린 경우
        // 차는 사실상 도착해 있습니다. 같은 골을 반복해 보내며 시간을 버리지 않습니다.
        if (distance_to_waypoint(step.end_index, distance) && distance <= arrival_slack_m_) {
          RCLCPP_WARN(
            get_logger(),
            "%s aborted, but the vehicle is %.2f m from '%s' (within arrival_slack_m %.2f). "
            "Counting it as arrived.",
            progress().c_str(), distance, step.label.c_str(), arrival_slack_m_);
          advance();
          return;
        }
        if (retries_ < goal_retry_limit_) {
          ++retries_;
          RCLCPP_WARN(
            get_logger(), "%s aborted by the controller; retrying (%d/%d).",
            progress().c_str(), retries_, goal_retry_limit_);
          phase_ = Phase::kStarting;
          starting_since_ = now();
          return;
        }
        fail("controller aborted '" + step.label + "' " + std::to_string(retries_ + 1) + " times");
        return;
      }
    }
  }

  // nav2 컨트롤러는 경로 앞머리에서 costmap 크기만큼만 최근접점을 찾고 결과를 local
  // costmap으로 자릅니다. 그래서 경로가 항상 차량 아래에서 시작하도록, 이미 지난 점을
  // 버리고 필요하면 진입 경로를 덧붙입니다.
  bool trim_to_robot(nav_msgs::msg::Path & path, bool reverse)
  {
    geometry_msgs::msg::PoseStamped robot;
    if (!lookup_robot_pose(path.header.frame_id, robot)) {
      return false;
    }

    double distance = 0.0;
    const std::size_t nearest = hyper_planner::nearest_pose_index(
      path, robot.pose.position.x, robot.pose.position.y, distance);

    if (max_start_distance_m_ > 0.0 && distance > max_start_distance_m_) {
      RCLCPP_ERROR(
        get_logger(),
        "Vehicle is %.1f m from the nearest pose of this segment, beyond max_start_distance_m "
        "(%.1f).", distance, max_start_distance_m_);
      return false;
    }

    // 지나온 점을 버리되, 최소 두 점은 남겨야 경로가 방향을 잃지 않습니다.
    const std::size_t dropped = nearest + 2 <= path.poses.size()
      ? nearest : (path.poses.size() >= 2 ? path.poses.size() - 2 : 0);
    if (dropped > 0) {
      path.poses.erase(
        path.poses.begin(), path.poses.begin() + static_cast<std::ptrdiff_t>(dropped));
    }

    // 진입 경로는 차량 헤딩과 회전 반경을 무시한 직선이라, 후진 세그먼트에 붙이면
    // RPP가 방향을 반대로 읽습니다. 전진 세그먼트에서만 씁니다.
    std::size_t lead_in = 0;
    if (!reverse) {
      lead_in = hyper_planner::insert_lead_in(
        path, robot.pose.position.x, robot.pose.position.y, lead_in_spacing_m_);
    }

    // 진입 경로까지 붙인 다음에 다시 깝니다 -- 그래야 경로 전체가 균일한 간격이 됩니다.
    const std::size_t before_resample = path.poses.size();
    hyper_planner::resample_path(path, path_resample_spacing_m_);

    RCLCPP_INFO(
      get_logger(),
      "Segment starts %.2f m from the vehicle (%zu passed pose(s) dropped, %zu lead-in "
      "pose(s) prepended); %zu poses -> %zu after resampling at %.2f m.",
      distance, dropped, lead_in, before_resample, path.poses.size(),
      path_resample_spacing_m_);
    return true;
  }

  bool lookup_robot_pose(const std::string & frame, geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      // TimePointZero = 가장 최근 값. 골을 보내는 시점에는 차가 서 있으므로
      // 이 정도 지연은 문제가 되지 않습니다.
      const auto tf = tf_buffer_->lookupTransform(
        frame, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_s_));
      pose.header = tf.header;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No '%s' -> '%s' transform after %.1f s: %s. Is odometry running?",
        frame.c_str(), robot_base_frame_.c_str(), tf_timeout_s_, ex.what());
      return false;
    }
  }

  bool distance_to_waypoint(std::size_t index, double & distance_m)
  {
    geometry_msgs::msg::PoseStamped robot;
    if (index >= waypoints_.points.size() || !lookup_robot_pose(waypoints_.frame_id, robot)) {
      return false;
    }
    distance_m = std::hypot(
      waypoints_.points[index].x - robot.pose.position.x,
      waypoints_.points[index].y - robot.pose.position.y);
    return true;
  }

  // ---------------------------------------------------------------- 신호 대기

  void on_sign(const std::string & value)
  {
    last_sign_ = value;
    if (phase_ != Phase::kWaiting) {
      return;
    }
    const Step & step = steps_[step_index_];
    const bool match =
      std::find(step.accepted.begin(), step.accepted.end(), value) != step.accepted.end();
    // 연속 프레임만 셉니다 -- 한 프레임짜리 오검출로 빨간불에 출발하지 않도록.
    sign_streak_ = match ? sign_streak_ + 1 : 0;
  }

  // ---------------------------------------------------------------- 서비스

  void handle_start(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (phase_ == Phase::kFailed && steps_.empty()) {
      response->success = false;
      response->message = "Mission was not loaded; see the node log.";
      return;
    }
    if (phase_ == Phase::kDriving || phase_ == Phase::kStarting ||
      phase_ == Phase::kHolding || phase_ == Phase::kWaiting)
    {
      response->success = false;
      response->message = "Mission is already running (step " + std::to_string(step_index_) + ").";
      return;
    }
    if (step_index_ >= steps_.size()) {
      response->success = false;
      response->message = "Mission is finished; call '~/restart' first.";
      return;
    }
    begin_step();
    response->success = true;
    response->message = "Started at step " + std::to_string(step_index_) + ".";
  }

  void handle_cancel(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    skip_requested_ = false;
    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
    } else {
      phase_ = Phase::kIdle;
      publish_status("canceled");
    }
    response->success = true;
    response->message = "Cancel requested; the mission holds at step " +
      std::to_string(step_index_) + ".";
  }

  void handle_skip(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (step_index_ >= steps_.size()) {
      response->success = false;
      response->message = "Nothing to skip; the mission is finished.";
      return;
    }
    const std::string skipped = std::to_string(step_index_);
    if (goal_handle_) {
      // 취소 결과가 돌아왔을 때 advance() 하도록 표시해 둡니다.
      skip_requested_ = true;
      client_->async_cancel_goal(goal_handle_);
    } else {
      advance();
    }
    response->success = true;
    response->message = "Skipping step " + skipped + ".";
  }

  void handle_restart(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    skip_requested_ = false;
    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
      goal_handle_.reset();
    }
    step_index_ = 0;
    retries_ = 0;
    phase_ = Phase::kIdle;
    publish_status("idle");
    response->success = true;
    response->message = "Reset to step 0; call '~/start' to run.";
  }

  // ---------------------------------------------------------------- 표시

  std::string progress() const
  {
    return "[" + std::to_string(step_index_ + 1) + "/" + std::to_string(steps_.size()) + "]";
  }

  std::string status_text() const
  {
    if (step_index_ >= steps_.size()) {
      return "finished";
    }
    const Step & step = steps_[step_index_];
    std::string text = progress() + " " + type_name(step.type);
    if (step.type == StepType::kDrive) {
      text += " until=" + step.label;
    } else if (step.type == StepType::kWaitSignal) {
      text += " value=" + join(step.accepted);
    }
    return text;
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  static std::string join(const std::vector<std::string> & values)
  {
    std::string text;
    for (const auto & value : values) {
      if (!text.empty()) {
        text += "|";
      }
      text += value;
    }
    return text;
  }

  // ---------------------------------------------------------------- 멤버

  std::string mission_yaml_;
  std::string waypoint_csv_;
  std::string frame_id_;
  std::string action_name_;
  std::string sign_topic_;
  std::string default_controller_id_;
  std::string default_goal_checker_id_;
  std::string robot_base_frame_;
  double min_spacing_m_{0.0};
  double tf_timeout_s_{5.0};
  double lead_in_spacing_m_{0.5};
  double path_resample_spacing_m_{0.25};
  double max_start_distance_m_{0.0};
  double server_wait_timeout_s_{30.0};
  double arrival_slack_m_{0.6};
  int goal_retry_limit_{2};
  bool auto_start_{false};
  bool proceed_on_signal_timeout_{true};

  hyper_planner::WaypointFile waypoints_;
  std::unordered_map<std::string, std::size_t> label_index_;
  std::vector<Step> steps_;

  Phase phase_{Phase::kIdle};
  std::size_t step_index_{0};
  int retries_{0};
  bool skip_requested_{false};
  rclcpp::Time hold_until_;
  rclcpp::Time wait_until_;
  rclcpp::Time starting_since_;
  int sign_streak_{0};
  std::string last_sign_;

  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sign_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr skip_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
