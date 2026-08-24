// HYPER 미션 매니저.
//
// config/mission.yaml의 스텝 큐를 순서대로 실행합니다. 핵심 아이디어는
// "한 스텝 = FollowPath 골 하나"입니다 -- 정지선/신호등/주차 지점이 곧 세그먼트의 끝이므로
// "도착했는가?"를 따로 판정할 필요 없이 nav2의 goal checker가 알려 줍니다.
//
// 정지에는 별도의 정지 명령이 필요 없습니다. 골을 보내지 않으면 /cmd_vel이 끊기고
// cmd_vel_to_ackermann의 워치독(input_timeout 0.3초)이 차를 세웁니다.
//
// "한 스텝 = 골 하나"에는 예외가 둘 있고, 둘 다 신호등에서 시간을 버리지 않으려는 것입니다.
//
//   prearm -- 초록불이면 서지 않고 통과
//     drive 스텝의 골에 다가가는 동안(남은 거리 <= prearm_distance_m) 다음 wait_signal의
//     신호를 미리 봅니다. 통과 신호가 확인되면 그 자리에서 "지금 위치 -> 그 다음 drive
//     스텝의 끝"까지를 새 골로 보냅니다. nav2의 SimpleActionServer는 실행 중에 들어온 골을
//     pending 슬롯에 받아 두었다가 다음 제어 주기에 갈아끼우므로
//     (controller_server::updateGlobalPath) /cmd_vel이 끊기지 않습니다 -- 감속 없이 그대로
//     통과합니다. 확인이 안 되면 아무 일도 일어나지 않고 원래 골 그대로 정지선에 섭니다.
//     기본 동작이 "정지"이고 "통과"가 명시적 예외이므로, 인식이 끊기거나 흔들려도 안전한
//     쪽으로 실패합니다.
//
//   cancel-on-arrival -- 도착 직전에 골을 취소
//     MPPI는 골 공차 안으로 들어가는 마지막 수십 cm를 기어갑니다(nav2_controller.yaml의
//     precise_goal_checker 주석 참고). 뒤에 stop/wait_signal이 붙는 스텝이라면 골 판정을
//     기다릴 이유가 없으므로, 남은 거리와 속도가 충분히 작아지면 골을 취소하고 도착으로
//     칩니다. 취소 시점에 controller_server가 0 속도를 내보내고 워치독이 그 뒤를 받칩니다.
//
//   decel 프로파일 -- 등감속으로 세운다 (drive 스텝의 decel_profile_a, 0이면 끔)
//     MPPI에는 가속도 제약이 없고(models/constraints.hpp는 vx/vy/wz 상한뿐입니다),
//     감속은 PathFollowCritic이 "궤적의 마지막 점을 경로의 끝점에 맞춘다"는 항에서
//     부수적으로 나옵니다. 그 결과 속도가 v = (남은거리) / (time_steps * model_dt)라는
//     지수 감쇠가 되어, 수학적으로 영영 도착하지 않고 마지막 몇 미터를 기어갑니다.
//
//     그래서 두 가지를 같이 합니다.
//       1. 골 경로를 라벨보다 decel_profile_lookahead_m만큼 뒤까지 보냅니다. MPPI가 보는
//          경로 끝이 local costmap 밖에 있으면 위의 감속 항 자체가 켜지지 않아, 차는
//          라벨 직전까지 vx_max로 달립니다. 이 꼬리는 녹화 코스가 아니라 라벨의 진행
//          방향으로 뻗은 직선입니다(path_loader.hpp의 append_straight_tail). 코스를 이어
//          붙이면 주차 진입처럼 라벨 뒤가 후진 구간인 곳에서 꼬리가 되돌아와 버리고,
//          코스 끝에서는 이어 붙일 코스 자체가 모자랍니다. 어차피 이 꼬리는 주행되지
//          않으므로(항상 라벨에서 취소합니다) 실제 코스일 이유가 없습니다.
//       2. 대신 우리가 /speed_limit(nav2_msgs/SpeedLimit)으로 v = sqrt(2*a*d)를 실어
//          보내 MPPI의 vx_max를 직접 깎습니다. 이게 등감속 프로파일입니다.
//     정지는 여전히 cancel-on-arrival이 합니다. 경로 끝이 라벨보다 뒤에 있으므로 goal
//     checker는 라벨에서 절대 만족되지 않고, 도착 판정은 오직 취소로만 일어납니다.
//     그래서 아래 두 가지가 "있으면 좋은 것"이 아니라 필수입니다.
//       - 커서가 정지점을 지나면 속도 조건과 무관하게 무조건 취소(하드 백스톱).
//       - 진행도(tf)가 progress_stale_cancel_s 동안 끊기면 취소. 속도 제한은 마지막 값이
//         그대로 남는데, 감속 중의 "낡은 제한"은 항상 실제로 필요한 값보다 빠르기 때문에
//         이쪽만은 안전한 방향으로 실패하지 않습니다.
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
#include <nav2_msgs/msg/speed_limit.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "hyper_planner/mission_manager_parameters.hpp"
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
  // 골까지 남은 거리가 이 값 이하이고 속도도 cancel_on_arrival_speed 이하로 떨어지면,
  // 골 판정을 기다리지 않고 취소해 도착으로 칩니다. 0 = 끔.
  double cancel_on_arrival_m{0.0};
  // 로드 시점에 이어 둔 prearm 링크(아래 link_prearm_steps 참고). prearm_enabled면 이
  // drive 스텝을 달리는 동안 steps_[prearm_wait_step]의 신호를 미리 보고, 확인되면
  // steps_[prearm_merge_step]의 끝까지 골을 이어 보냅니다.
  bool prearm_enabled{false};
  std::size_t prearm_wait_step{0};
  std::size_t prearm_merge_step{0};
  // 0보다 크면 이 스텝을 등감속(m/s^2)으로 세웁니다. 파일 머리의 "decel 프로파일" 참고.
  double decel_profile_a{0.0};
  // 라벨(end_index) 뒤로 덧붙일 직선 꼬리의 길이(0 = 없음). resolve_decel_tails가 정합니다.
  // 보낸 경로 위에서 라벨이 어디인지 찾을 때도 이 값을 씁니다 -- 좌표로 최근접점을 찾으면
  // 같은 길을 되짚는 구간에서 엉뚱한 점에 붙을 수 있지만, "경로 끝에서 남은 길이"로 찾으면
  // 그런 모호함이 없습니다.
  double tail_after_label_m{0.0};

  // stop
  double duration_s{0.0};

  // wait_signal
  std::vector<std::string> accepted;
  double timeout_s{60.0};
  int debounce_frames{3};

  // 두 종류가 같이 쓰는 필드. Step은 종류별로 나뉘지 않은 평평한 구조체입니다.
  //   wait_signal에서: mission.yaml이 적어 준 값. 0보다 크면 prearm을 켭니다.
  //   drive에서:       link_prearm_steps가 뒤의 wait_signal에서 복사해 온 값. 골까지 남은
  //                    거리가 이 값 이하로 들어오면 신호를 미리 보기 시작합니다.
  double prearm_distance_m{0.0};
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

// FollowPath 피드백의 speed가 이 시간보다 오래되면 못 믿습니다.
constexpr double kFeedbackStaleSeconds = 0.5;

// 진행도 계산용 tf 조회 타임아웃. tick마다 부르므로 골을 보낼 때(tf_timeout_sec, 기본 5초)와
// 달리 짧아야 합니다 -- 길게 잡으면 tf가 잠깐 비는 동안 타이머가 통째로 멈춥니다.
constexpr double kProgressTfTimeoutSeconds = 0.1;

// 경로를 만들다 실패했을 때, 다시 시도해 볼 만한 실패(kRetry: 아직 tf가 없다 등)와
// 설정이 틀려서 영영 안 될 실패(kInvalid)를 구분합니다.
enum class PathBuild
{
  kOk,
  kRetry,
  kInvalid,
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
    param_listener_ = std::make_shared<mission_manager::ParamListener>(
      get_node_parameters_interface());
    params_ = param_listener_->get_params();

    if (params_.decel_profile_min_speed >= params_.cancel_on_arrival_speed) {
      RCLCPP_WARN(
        get_logger(),
        "decel_profile_min_speed (%.2f) is not below cancel_on_arrival_speed (%.2f). The "
        "profile will never let the vehicle slow past the cancel threshold, so cancel-on-"
        "arrival can only fire from the hard backstop at the stop point.",
        params_.decel_profile_min_speed, params_.cancel_on_arrival_speed);
    }

    // 기본 생성된 rclcpp::Time은 시스템 시계라, use_sim_time일 때 now()와 비교하면
    // 시간 소스가 달라 예외가 납니다. 노드 시계로 초기화해 둡니다.
    hold_until_ = now();
    wait_until_ = now();
    starting_since_ = now();
    last_feedback_time_ = now();
    progress_ok_since_ = now();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // spin_thread = true: 아래 lookupTransform()이 타임아웃까지 블록하므로 리스너는
    // 버퍼를 계속 채울 자기 스레드가 필요합니다.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    // 나중에 붙는 RViz/툴이 받을 수 있도록 latch 합니다.
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).transient_local());
    // controller_server가 QoS(10)으로 구독합니다.
    speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
      params_.speed_limit_topic, rclcpp::QoS(10));

    sign_sub_ = create_subscription<std_msgs::msg::String>(
      params_.sign_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {on_sign(msg->data);});

    client_ = rclcpp_action::create_client<FollowPath>(this, params_.action_name);

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
    if (params_.auto_start) {
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
    if (!hyper_planner::load_waypoint_csv(params_.waypoint_csv, params_.min_spacing_m, waypoints_, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }
    if (waypoints_.frame_id.empty()) {
      waypoints_.frame_id = params_.frame_id;
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %zu waypoints from '%s' in frame '%s' (%zu rows skipped).",
      waypoints_.points.size(), params_.waypoint_csv.c_str(), waypoints_.frame_id.c_str(),
      waypoints_.skipped_rows);

    if (params_.mission_yaml.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'mission_yaml' is empty.");
      return false;
    }

    YAML::Node root;
    try {
      root = YAML::LoadFile(params_.mission_yaml);
    } catch (const YAML::Exception & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to parse '%s': %s", params_.mission_yaml.c_str(), ex.what());
      return false;
    }

    const double snap_tolerance_m = root["label_snap_tolerance_m"]
      ? root["label_snap_tolerance_m"].as<double>() : 1.0;

    if (!root["labels"] || !root["labels"].IsMap() || root["labels"].size() == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "'%s' has no 'labels'. Place them first:\n"
        "  python3 src/planning/hyper_waypoint/scripts/label_waypoints.py %s",
        params_.mission_yaml.c_str(), params_.waypoint_csv.c_str());
      return false;
    }
    if (!root["steps"] || !root["steps"].IsSequence() || root["steps"].size() == 0) {
      RCLCPP_ERROR(get_logger(), "'%s' has no 'steps'.", params_.mission_yaml.c_str());
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
          name.c_str(), x, y, best, nearest, snap_tolerance_m, params_.waypoint_csv.c_str());
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
          ? node["controller"].as<std::string>() : params_.controller_id;
        step.goal_checker_id = node["goal_checker"]
          ? node["goal_checker"].as<std::string>() : params_.goal_checker_id;
        step.cancel_on_arrival_m = node["cancel_on_arrival_m"]
          ? node["cancel_on_arrival_m"].as<double>() : params_.cancel_on_arrival_m;
        step.decel_profile_a = node["decel_profile_a"]
          ? node["decel_profile_a"].as<double>() : params_.decel_profile_a;

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
        step.prearm_distance_m = node["prearm_distance_m"]
          ? node["prearm_distance_m"].as<double>() : 0.0;
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

    resolve_decel_tails();
    link_prearm_steps();

    RCLCPP_INFO(
      get_logger(), "Mission '%s' loaded: %zu steps, %zu labels.",
      params_.mission_yaml.c_str(), steps_.size(), label_index_.size());
    return true;
  }

  // decel 프로파일을 쓰는 drive 스텝에, 라벨 뒤로 덧붙일 직선 꼬리의 길이를 정해 둡니다.
  //
  // MPPI의 감속은 "경로의 마지막 점"에서 나오므로(파일 머리 주석), 그 점을 local costmap
  // 밖으로 밀어내면 감속 항이 아예 켜지지 않습니다. 감속은 대신 /speed_limit이 맡습니다.
  //
  // 꼬리는 녹화 코스가 아니라 직선이므로(append_straight_tail) 길이가 항상 정확히
  // decel_profile_lookahead_m입니다. 코스가 라벨 뒤에서 무엇을 하든 -- 주차 진입처럼
  // 후진으로 되돌아오든, finish처럼 아예 끝나든 -- 상관없습니다.
  void resolve_decel_tails()
  {
    for (std::size_t i = 0; i < steps_.size(); ++i) {
      Step & step = steps_[i];
      if (step.type != StepType::kDrive) {
        continue;
      }
      step.tail_after_label_m = 0.0;
      if (step.decel_profile_a <= 0.0) {
        continue;
      }
      if (step.reverse) {
        RCLCPP_WARN(
          get_logger(),
          "Step %zu (until '%s') is a reverse segment; decel_profile_a is ignored there. "
          "Reverse parking runs on RPP and has to reach the goal checker.",
          i, step.label.c_str());
        step.decel_profile_a = 0.0;
        continue;
      }

      step.tail_after_label_m = params_.decel_profile_lookahead_m;
      RCLCPP_INFO(
        get_logger(),
        "Step %zu (until '%s'): decel profile at %.1f m/s^2; the goal path runs %.1f m of "
        "straight extrapolation past the label (wp #%zu) so MPPI does not brake on its own.",
        i, step.label.c_str(), step.decel_profile_a, step.tail_after_label_m, step.end_index);
    }
  }

  // wait_signal의 prearm_distance_m를 그 앞뒤 drive 스텝에 이어 줍니다. 패턴은 반드시
  // [drive -> wait_signal -> drive]여야 합니다: 신호를 미리 보는 것은 앞의 drive이고,
  // 통과하면 골을 이어 붙일 끝점은 뒤의 drive이기 때문입니다.
  //
  // 조건이 안 맞으면 경고만 남기고 그 자리의 prearm을 끕니다 -- 미션을 거부하지 않는 이유는,
  // prearm이 꺼진 결과가 곧 "예전처럼 정지선에 서서 기다린다"라서 안전하기 때문입니다.
  void link_prearm_steps()
  {
    for (std::size_t i = 0; i < steps_.size(); ++i) {
      if (steps_[i].type != StepType::kDrive || i + 1 >= steps_.size()) {
        continue;
      }
      const Step & wait_step = steps_[i + 1];
      if (wait_step.type != StepType::kWaitSignal || wait_step.prearm_distance_m <= 0.0) {
        continue;
      }

      const char * reason = nullptr;
      if (i + 2 >= steps_.size()) {
        reason = "there is no step after the wait_signal to merge into";
      } else if (steps_[i + 2].type != StepType::kDrive) {
        reason = "the step after the wait_signal is not a drive step";
      } else if (steps_[i].reverse || steps_[i + 2].reverse) {
        // 두 세그먼트를 한 골로 합치면 그 안에 방향 전환이 들어갑니다. RPP는 이를 처리하지
        // 못하고, MPPI도 PreferForwardCritic 때문에 안정적으로 못 냅니다.
        reason = "merging would put a direction change inside a single goal";
      } else if (steps_[i].controller_id != steps_[i + 2].controller_id) {
        reason = "the two drive steps use different controllers";
      }
      if (reason != nullptr) {
        RCLCPP_WARN(
          get_logger(),
          "Step %zu's wait_signal has prearm_distance_m %.1f, but %s. Prearm is off here: the "
          "vehicle will stop at '%s' and wait as usual.",
          i + 1, wait_step.prearm_distance_m, reason, steps_[i].label.c_str());
        continue;
      }

      steps_[i].prearm_enabled = true;
      steps_[i].prearm_wait_step = i + 1;
      steps_[i].prearm_merge_step = i + 2;
      steps_[i].prearm_distance_m = wait_step.prearm_distance_m;
      RCLCPP_INFO(
        get_logger(),
        "Prearm: while driving to '%s', watch %s for '%s' from %.1f m out; if confirmed, roll "
        "straight through to '%s' without stopping.",
        steps_[i].label.c_str(), params_.sign_topic.c_str(), join(wait_step.accepted).c_str(),
        steps_[i].prearm_distance_m, steps_[i + 2].label.c_str());
    }
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
    prearmed_ = false;
    arrival_requested_ = false;
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
          progress().c_str(), join(step.accepted).c_str(), params_.sign_topic.c_str(),
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
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
      if (params_logged_once_) {
        RCLCPP_INFO(get_logger(), "Parameters updated at runtime.");
      }
      params_logged_once_ = true;
    }

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
          if (params_.proceed_on_signal_timeout) {
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
      case Phase::kDriving:
        update_progress();
        update_prearm();
        // 통과 신호가 확인되면 설 이유가 없으므로 cancel-on-arrival보다 먼저 봅니다.
        if (!try_preempt_for_signal()) {
          check_cancel_on_arrival();
        }
        break;

      case Phase::kIdle:
      case Phase::kFinished:
      case Phase::kFailed:
        break;
    }

    // switch가 끝난 뒤에 부릅니다. kDriving에서는 update_progress()가 방금 갱신한 거리로
    // 계산해야 하고, 그 밖의 어느 분기로 빠져나갔든(도착/취소/실패/완료) 여기서 제한이
    // 해제됩니다. 스텝 전환 경로마다 해제를 끼워 넣는 것보다 빠뜨릴 구멍이 적습니다.
    update_speed_limit();
  }

  // ---------------------------------------------------------------- drive 스텝

  void try_send_goal()
  {
    if (!client_->action_server_is_ready()) {
      if ((now() - starting_since_).seconds() > params_.server_wait_timeout_sec) {
        fail(
          "action server '" + params_.action_name + "' never became available. Is nav2's "
          "controller_server running and activated?");
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for action server '%s'...",
        params_.action_name.c_str());
      return;
    }

    const Step & step = steps_[step_index_];
    nav_msgs::msg::Path path;
    switch (
      build_path(
        step.begin_index, step.end_index, step.tail_after_label_m, step.reverse, step.label,
        path))
    {
      case PathBuild::kOk:
        break;
      case PathBuild::kInvalid:
        fail("segment for '" + step.label + "' is unusable; see the log above");
        return;
      case PathBuild::kRetry:
        // 사유는 build_path/trim_to_robot이 이미 로그로 남겼습니다. 다음 tick에 다시
        // 시도합니다(오도메트리가 아직 안 올라온 것뿐일 수 있으므로).
        if ((now() - starting_since_).seconds() > params_.server_wait_timeout_sec) {
          fail("could not locate the vehicle in frame '" + waypoints_.frame_id + "'");
        }
        return;
    }

    RCLCPP_INFO(
      get_logger(), "%s sending %zu-pose path to '%s'.",
      progress().c_str(), path.poses.size(), params_.action_name.c_str());
    phase_ = Phase::kDriving;   // 응답이 올 때까지 재전송을 막습니다.
    send_goal(step, path);
  }

  // 세그먼트 경로를 만들어 차량의 현재 위치에 맞춰 다듬습니다. 실패 사유는 여기서 로그로
  // 남기므로, 호출자는 kRetry / kInvalid만 보고 어떻게 할지 정하면 됩니다.
  //
  // tail_m > 0이면 라벨 뒤로 그만큼 직선 꼬리를 덧붙입니다(decel 프로파일). 꼬리는
  // trim_to_robot보다 먼저 붙입니다 -- 그래야 그 안의 resample이 꼬리까지 같은 간격으로
  // 다시 깔아 주고(MPPI의 경로 critic은 '미터'가 아니라 '점 개수'로 셉니다), 그러면서도
  // 꼭짓점을 옮기지 않으므로 꼬리 길이는 tail_m 그대로 남습니다. set_stop_point가 라벨을
  // 되찾을 때 그 길이에 의존합니다.
  PathBuild build_path(
    std::size_t begin_index, std::size_t end_index, double tail_m, bool reverse,
    const std::string & label, nav_msgs::msg::Path & path)
  {
    path = hyper_planner::make_path(
      waypoints_.points, begin_index, end_index, waypoints_.frame_id, now(), reverse);
    if (path.poses.size() < 2) {
      RCLCPP_ERROR(
        get_logger(), "Segment for '%s' (wp #%zu..#%zu) has fewer than 2 poses.",
        label.c_str(), begin_index, end_index);
      return PathBuild::kInvalid;
    }

    if (tail_m > 0.0) {
      // resample이 꺼져 있으면(path_resample_spacing_m == 0) 꼬리도 녹화 CSV와 비슷한
      // 간격으로 깝니다. 꼬리만 촘촘하면 점 개수로 세는 critic들이 꼬리 쪽으로 치우칩니다.
      const double spacing = params_.path_resample_spacing_m > 0.0
        ? params_.path_resample_spacing_m : 0.5;
      const std::size_t added = hyper_planner::append_straight_tail(path, tail_m, spacing);
      if (added == 0) {
        RCLCPP_ERROR(
          get_logger(),
          "Could not extrapolate the %.1f m tail past '%s' -- the segment ends with a "
          "zero-length step, so there is no direction to extend along.", tail_m, label.c_str());
        return PathBuild::kInvalid;
      }
      RCLCPP_INFO(
        get_logger(), "Extended the path %.1f m past '%s' with %zu straight pose(s).",
        tail_m, label.c_str(), added);
    }

    return trim_to_robot(path, reverse) ? PathBuild::kOk : PathBuild::kRetry;
  }

  // 골 하나를 보냅니다. 이미 실행 중인 골이 있으면 nav2가 그것을 pending 슬롯의 새 골로
  // 갈아끼웁니다(= prearm 통과). 갈아끼울 때 옛 골은 abort로 끝나므로, 그 결과가 뒤늦게
  // 와도 재시도 로직이 돌지 않게 호출자가 mark_superseded()로 표시해 둡니다.
  void send_goal(const Step & step, const nav_msgs::msg::Path & path)
  {
    path_pub_->publish(path);

    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = step.controller_id;
    goal.goal_checker_id = step.goal_checker_id;

    // 진행도는 이 경로를 기준으로 새로 셉니다. 속도도 새 골의 피드백이 올 때까지는
    // 직전 골의 값으로 판단하지 않습니다.
    reset_progress(path);
    set_stop_point(step, path);
    have_speed_ = false;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
      if (!handle) {
        fail("goal rejected by '" + params_.action_name + "'");
        return;
      }
      goal_handle_ = handle;
      phase_ = Phase::kDriving;
      publish_status(status_text());
    };
    options.feedback_callback = [this](
      GoalHandle::SharedPtr handle, const std::shared_ptr<const FollowPath::Feedback> feedback) {
      if (is_superseded(handle)) {
        return;   // 갈아끼우기 직전에 출발한 옛 골의 피드백.
      }
      // speed만 씁니다. feedback->distance_to_goal은 프레임이 맞지 않아 못 씁니다
      // (update_progress의 주석 참고). 거리는 remaining_path_m_로 직접 셉니다.
      last_speed_ = feedback->speed;
      last_feedback_time_ = now();
      have_speed_ = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        // limit 0.00 = 제한 없음(nav2의 NO_SPEED_LIMIT과 같은 뜻).
        "%s %.2f m to '%s' (%.2f m direct), speed=%.2f m/s, limit=%.2f m/s",
        progress().c_str(), distance_to_stop_m_, steps_[step_index_].label.c_str(),
        stop_point_distance_m_, feedback->speed, published_speed_limit_);
    };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
      on_result(result);
    };

    client_->async_send_goal(goal, options);
  }

  // ------------------------------------------------- prearm / cancel-on-arrival

  bool speed_is_fresh() const
  {
    return have_speed_ && (now() - last_feedback_time_).seconds() <= kFeedbackStaleSeconds;
  }

  // 우리가 보낸 경로 위에서 차량이 어디까지 왔는지 직접 셉니다.
  //
  // nav2의 FollowPath 피드백에 distance_to_goal이 있지만 이 스택에서는 쓸 수 없습니다.
  // controller_server는 그 값을 이렇게 계산합니다(controller_server.cpp):
  //
  //     feedback->distance_to_goal =
  //       calculate_path_length(current_path_, find_closest_pose_idx());
  //
  // 그리고 find_closest_pose_idx는 차량 포즈와 경로 점의 좌표를 프레임 변환 없이 그대로
  // 뺍니다. 차량 포즈는 costmap의 global_frame으로 오는데(nav2_controller.yaml의
  // local_costmap: global_frame: odom) 우리가 보내는 경로는 map 프레임입니다. 이 스택의
  // odom 원점은 차량 출발 지점이고 그 지점의 map 좌표는 (41.08, -45.70)이므로, 두 좌표계는
  // 약 61 m 어긋나 있습니다. 그래서 "가장 가까운 점"이 늘 경로 끝쪽으로 잡히고,
  // distance_to_goal은 골을 보낸 순간부터 0.00 m으로 나옵니다.
  //
  // 참고로 틀리는 것은 이 피드백 숫자 하나뿐입니다. MPPI는 경로를 제대로 TF 변환해 쓰고
  // (transformGlobalPlan), 골 판정도 end_pose_를 costmap 프레임으로 옮긴 뒤 비교하므로
  // (ControllerServer::isGoalReached) 주행과 도착 판정에는 영향이 없습니다.
  //
  // 커서는 앞으로만 움직입니다. 그래서 코스가 자기 자신 근처로 돌아오는 구간에서도, 차가
  // 경로에서 잠깐 벗어나도 진행도가 뒤로 튀지 않습니다.
  void update_progress()
  {
    progress_valid_ = false;
    if (active_path_.poses.size() < 2) {
      return;
    }

    geometry_msgs::msg::PoseStamped robot;
    if (!lookup_robot_pose(active_path_.header.frame_id, robot, kProgressTfTimeoutSeconds)) {
      return;
    }
    const double rx = robot.pose.position.x;
    const double ry = robot.pose.position.y;

    std::size_t best = path_cursor_;
    double best_distance = std::numeric_limits<double>::max();
    double scanned = 0.0;
    for (std::size_t i = path_cursor_; i < active_path_.poses.size(); ++i) {
      const auto & point = active_path_.poses[i].pose.position;
      const double d = std::hypot(point.x - rx, point.y - ry);
      if (d < best_distance) {
        best_distance = d;
        best = i;
      }
      if (i + 1 >= active_path_.poses.size()) {
        break;
      }
      const auto & next = active_path_.poses[i + 1].pose.position;
      scanned += std::hypot(next.x - point.x, next.y - point.y);
      if (scanned > params_.progress_search_window_m) {
        break;
      }
    }
    path_cursor_ = best;

    remaining_path_m_ = path_suffix_len_[path_cursor_];
    const auto & goal_point = active_path_.poses.back().pose.position;
    goal_point_distance_m_ = std::hypot(goal_point.x - rx, goal_point.y - ry);

    // 정지점은 경로 끝보다 앞에 있을 수 있습니다(decel 프로파일). 커서가 정지점을
    // 지나가면 음수가 되고, 그 부호가 cancel-on-arrival의 하드 백스톱이 됩니다.
    distance_to_stop_m_ = remaining_path_m_ - path_suffix_len_[stop_index_];
    stop_point_distance_m_ = std::hypot(stop_x_ - rx, stop_y_ - ry);

    progress_valid_ = true;
    progress_ok_since_ = now();
  }

  // 보낸 경로 위에서 "여기서 서야 한다"는 지점을 정합니다.
  //
  // 좌표로 최근접점을 찾지 않는 이유: 주차 진입/출차처럼 같은 길을 되짚는 구간에서는
  // 라벨 좌표 근처를 경로가 두 번 지나므로 엉뚱한 인덱스에 붙을 수 있습니다. 대신
  // "경로 끝에서 남은 길이"로 찾습니다 -- trim/lead-in은 경로 앞쪽만 건드리고, resample은
  // 점을 다시 깔 뿐 꼭짓점을 옮기지 않으므로 꼬리 길이는 tail_after_label_m 그대로입니다.
  void set_stop_point(const Step & step, const nav_msgs::msg::Path & path)
  {
    stop_index_ = path.poses.empty() ? 0 : path.poses.size() - 1;
    for (std::size_t i = 0; i < path_suffix_len_.size(); ++i) {
      if (path_suffix_len_[i] <= step.tail_after_label_m) {
        stop_index_ = i;
        break;
      }
    }

    if (step.end_index < waypoints_.points.size()) {
      stop_x_ = waypoints_.points[step.end_index].x;
      stop_y_ = waypoints_.points[step.end_index].y;
    } else if (!path.poses.empty()) {
      stop_x_ = path.poses.back().pose.position.x;
      stop_y_ = path.poses.back().pose.position.y;
    }

    if (step.tail_after_label_m > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "Stop point for '%s' is pose %zu/%zu of the sent path (%.1f m of path runs past it).",
        step.label.c_str(), stop_index_, path.poses.size(),
        path_suffix_len_.empty() ? 0.0 : path_suffix_len_[stop_index_]);
    }
  }

  // 경로를 보낼 때 각 인덱스에서 끝까지 남은 길이를 미리 재 둡니다(tick마다 다시 세지 않게).
  void reset_progress(const nav_msgs::msg::Path & path)
  {
    active_path_ = path;
    path_cursor_ = 0;
    progress_valid_ = false;
    remaining_path_m_ = 0.0;
    goal_point_distance_m_ = 0.0;
    stop_index_ = 0;
    distance_to_stop_m_ = 0.0;
    stop_point_distance_m_ = 0.0;
    progress_ok_since_ = now();

    path_suffix_len_.assign(path.poses.size(), 0.0);
    if (path.poses.size() < 2) {
      return;
    }
    for (std::size_t i = path.poses.size() - 2; ; --i) {
      const auto & a = path.poses[i].pose.position;
      const auto & b = path.poses[i + 1].pose.position;
      path_suffix_len_[i] = path_suffix_len_[i + 1] + std::hypot(b.x - a.x, b.y - a.y);
      if (i == 0) {
        break;
      }
    }
  }

  bool is_superseded(const GoalHandle::SharedPtr & handle) const
  {
    return has_superseded_goal_ && handle && handle->get_goal_id() == superseded_goal_id_;
  }

  // 지금 실행 중인 골에 다가가는 동안 다음 wait_signal의 신호를 미리 볼지 정합니다.
  void update_prearm()
  {
    const Step & step = steps_[step_index_];
    const bool in_range = step.prearm_enabled && progress_valid_ &&
      distance_to_stop_m_ <= step.prearm_distance_m;

    if (in_range && !prearmed_) {
      // 진입하는 순간 streak을 비웁니다. 멀리서 -- 신호등이 아직 몇 픽셀일 때 -- 우연히
      // 쌓인 연속 프레임이 그대로 통과 판정으로 이어지지 않게 하려는 것입니다.
      sign_streak_ = 0;
      RCLCPP_INFO(
        get_logger(), "%s %.1f m from '%s'; watching %s for '%s'.",
        progress().c_str(), distance_to_stop_m_, step.label.c_str(), params_.sign_topic.c_str(),
        join(steps_[step.prearm_wait_step].accepted).c_str());
    }
    prearmed_ = in_range;
  }

  // 통과 신호가 확인되면 정지 없이 그대로 통과합니다. 실행 중인 골을 "지금 위치 -> 다음
  // drive 스텝의 끝"까지의 골로 갈아끼우고, wait_signal과 그 drive 스텝을 건너뜁니다.
  // 갈아끼웠으면 true를 돌려줍니다.
  bool try_preempt_for_signal()
  {
    const Step & step = steps_[step_index_];
    // arrival_requested_: cancel-on-arrival로 이미 취소를 걸어 둔 뒤라면 골을 갈아끼우지
    // 않습니다. 취소와 새 골이 동시에 날아가는 상황을 피하려는 것입니다. 이때 늦게 들어온
    // 초록불은 잠깐 섰다가 바로 뒤 wait_signal이 처리하므로 잃는 시간은 정차 한 번뿐입니다.
    if (!prearmed_ || !goal_handle_ || arrival_requested_) {
      return false;
    }
    if (sign_streak_ < steps_[step.prearm_wait_step].debounce_frames) {
      return false;
    }

    const std::size_t merge_index = step.prearm_merge_step;
    const Step & merge_step = steps_[merge_index];
    nav_msgs::msg::Path path;
    if (build_path(
        step.begin_index, merge_step.end_index, merge_step.tail_after_label_m, false,
        merge_step.label, path) != PathBuild::kOk)
    {
      // 다음 tick에 다시 시도합니다. 끝내 못 만들면 prearm이 그냥 안 일어나고, 원래 골
      // 그대로 정지선에 서서 wait_signal이 처리합니다 -- 안전한 쪽으로 실패합니다.
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s '%s' confirmed %.1f m before '%s'; skipping the stop and continuing to '%s' "
      "(%zu-pose path preempts the running goal).",
      progress().c_str(), last_sign_.c_str(), distance_to_stop_m_, step.label.c_str(),
      merge_step.label.c_str(), path.poses.size());

    // 갈아끼워진 옛 골은 nav2가 abort로 끝냅니다. 그 결과가 뒤늦게 도착했을 때 ABORTED
    // 재시도 로직이 돌면 방금 보낸 골을 덮어쓰므로, id를 적어 두고 무시합니다.
    superseded_goal_id_ = goal_handle_->get_goal_id();
    has_superseded_goal_ = true;

    step_index_ = merge_index;
    retries_ = 0;
    prearmed_ = false;
    sign_streak_ = 0;
    send_goal(merge_step, path);
    return true;
  }

  // MPPI는 골 공차 안으로 들어가는 마지막 수십 cm를 기어갑니다. 어차피 여기서 서야 하는
  // 스텝이라면 골 판정을 기다릴 이유가 없으므로, 충분히 가깝고 충분히 느려졌으면 골을
  // 취소하고 도착으로 칩니다.
  //
  // 속도 조건이 있는 이유: 아직 빠를 때 취소하면 0 속도가 실제로 나가기까지의 짧은 지연
  // 동안 그만큼 굴러가 정지선을 넘습니다. 느려질 때까지는 MPPI가 계속 줄이도록 둡니다.
  void check_cancel_on_arrival()
  {
    const Step & step = steps_[step_index_];
    if (step.cancel_on_arrival_m <= 0.0 || !goal_handle_ || arrival_requested_) {
      return;
    }

    // 하드 백스톱 1 -- 진행도가 끊긴 채로 달리는 경우.
    // decel 프로파일을 쓰는 스텝은 경로 끝이 라벨보다 뒤에 있어 goal checker가 받쳐 주지
    // 않습니다. 여기서 tf를 놓치면 남은 안전장치가 없고, 속도 제한도 마지막 값이 그대로
    // 남아 실제 필요한 값보다 빠릅니다. 그러니 차라리 세웁니다.
    if (step.decel_profile_a > 0.0 && !progress_valid_ &&
      (now() - progress_ok_since_).seconds() > params_.progress_stale_cancel_sec)
    {
      RCLCPP_ERROR(
        get_logger(),
        "%s no valid progress for %.1f s while running the decel profile to '%s'. Canceling "
        "to stop the vehicle -- the goal runs past the label, so nothing else would stop it.",
        progress().c_str(), (now() - progress_ok_since_).seconds(), step.label.c_str());
      arrival_requested_ = true;
      client_->async_cancel_goal(goal_handle_);
      return;
    }

    if (!progress_valid_) {
      return;
    }

    // 하드 백스톱 2 -- 정지점을 지났으면 속도와 무관하게 취소합니다. 프로파일 스텝에서는
    // 이것이 "정지선을 넘지 않는다"의 마지막 보루입니다.
    if (distance_to_stop_m_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "%s reached '%s' at %.2f m/s, above cancel_on_arrival_speed %.2f. Canceling anyway; "
        "the vehicle will coast a little past the label.",
        progress().c_str(), step.label.c_str(), std::fabs(last_speed_),
        params_.cancel_on_arrival_speed);
      arrival_requested_ = true;
      client_->async_cancel_goal(goal_handle_);
      return;
    }

    if (!speed_is_fresh()) {
      return;
    }
    // 두 거리를 모두 봅니다. distance_to_stop_m_는 "경로를 정지점까지 달렸는가"이고
    // stop_point_distance_m_는 "지금 정지점 옆에 있는가"입니다. 하나만 보면, 경로를 벗어난
    // 채 커서만 끝까지 간 경우나 코스가 정지점 근처를 스쳐 지나가는 경우에 잘못 걸립니다.
    if (distance_to_stop_m_ > step.cancel_on_arrival_m ||
      stop_point_distance_m_ > step.cancel_on_arrival_m ||
      std::fabs(last_speed_) > params_.cancel_on_arrival_speed)
    {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s %.2f m from '%s' at %.2f m/s; canceling instead of crawling into the goal checker.",
      progress().c_str(), stop_point_distance_m_, step.label.c_str(), std::fabs(last_speed_));
    arrival_requested_ = true;
    client_->async_cancel_goal(goal_handle_);
  }

  // ------------------------------------------------------------- decel 프로파일

  // 등감속 프로파일을 /speed_limit으로 내보냅니다. 파일 머리의 "decel 프로파일" 참고.
  //
  //     v = sqrt(2 * a * (정지점까지 남은 거리 - cancel_on_arrival_m))
  //
  // cancel_on_arrival_m를 빼는 이유: 그래야 취소 지점에서 속도가 정확히 하한까지 내려와
  // cancel-on-arrival의 속도 조건이 열립니다. 안 빼면 프로파일이 정지점에서야 0이 되는데,
  // 그 전에는 계속 속도 조건에 걸려 취소가 안 되고 결국 백스톱까지 밀립니다.
  void update_speed_limit()
  {
    double limit = 0.0;   // 0.0 = NO_SPEED_LIMIT

    if (phase_ == Phase::kDriving && step_index_ < steps_.size()) {
      const Step & step = steps_[step_index_];
      if (step.decel_profile_a > 0.0) {
        if (!progress_valid_) {
          // tf를 잠깐 놓친 것뿐일 수 있습니다. 제한을 해제하면 그 순간 vx_max로 튀어
          // 나가므로, 마지막 값을 그대로 둡니다. 오래 끊기면 위 백스톱이 골을 취소합니다.
          return;
        }
        const double braking = std::max(
          0.0, distance_to_stop_m_ - step.cancel_on_arrival_m);
        const double profile = std::sqrt(2.0 * step.decel_profile_a * braking);
        // vx_max 이상이면 제한할 게 없습니다. 그대로 실어 보내면 MPPI의 setSpeedLimit이
        // ratio > 1로 오히려 vx_max를 올리므로 반드시 해제(0.0)해야 합니다.
        if (profile < params_.controller_vx_max) {
          limit = std::max(profile, params_.decel_profile_min_speed);
        }
      }
    }

    // 값이 실제로 바뀔 때만 보냅니다(해제 -> 해제는 아무것도 안 보냄).
    if (std::fabs(limit - published_speed_limit_) < 1e-3) {
      return;
    }
    published_speed_limit_ = limit;

    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = now();
    msg.header.frame_id = params_.frame_id;
    msg.percentage = false;
    msg.speed_limit = limit;
    speed_limit_pub_->publish(msg);
  }

  // ---------------------------------------------------------------- 액션 결과

  void on_result(const GoalHandle::WrappedResult & result)
  {
    if (has_superseded_goal_ && result.goal_id == superseded_goal_id_) {
      // prearm으로 갈아끼운 옛 골의 결과입니다(nav2가 abort로 끝냅니다). 그냥 두면 아래
      // ABORTED 분기가 방금 보낸 골 위에 재시도를 얹습니다.
      has_superseded_goal_ = false;
      RCLCPP_DEBUG(get_logger(), "Ignoring the result of the goal we preempted.");
      return;
    }
    goal_handle_.reset();
    if (phase_ == Phase::kFinished || phase_ == Phase::kFailed || phase_ == Phase::kIdle) {
      return;   // 취소로 이미 정리된 뒤 도착한 결과.
    }

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (steps_[step_index_].decel_profile_a > 0.0) {
          // 프로파일 스텝의 골은 라벨보다 뒤에 있으므로, 여기까지 왔다는 것은 백스톱이
          // 전부 실패해 차가 라벨을 그대로 지나쳤다는 뜻입니다.
          RCLCPP_ERROR(
            get_logger(),
            "%s reached the END OF THE EXTENDED PATH for '%s' -- cancel-on-arrival never "
            "fired, so the vehicle drove past the label. Check tf and cancel_on_arrival_m.",
            progress().c_str(), steps_[step_index_].label.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s arrived.", progress().c_str());
        }
        advance();
        return;

      case rclcpp_action::ResultCode::CANCELED:
        if (arrival_requested_) {
          arrival_requested_ = false;
          RCLCPP_INFO(
            get_logger(), "%s arrived (canceled on arrival).", progress().c_str());
          advance();
        } else if (skip_requested_) {
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
        if (distance_to_waypoint(step.end_index, distance) && distance <= params_.arrival_slack_m) {
          RCLCPP_WARN(
            get_logger(),
            "%s aborted, but the vehicle is %.2f m from '%s' (within arrival_slack_m %.2f). "
            "Counting it as arrived.",
            progress().c_str(), distance, step.label.c_str(), params_.arrival_slack_m);
          advance();
          return;
        }
        if (retries_ < params_.goal_retry_limit) {
          ++retries_;
          RCLCPP_WARN(
            get_logger(), "%s aborted by the controller; retrying (%d/%ld).",
            progress().c_str(), retries_, params_.goal_retry_limit);
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
    if (!lookup_robot_pose(path.header.frame_id, robot, params_.tf_timeout_sec)) {
      return false;
    }

    double distance = 0.0;
    const std::size_t nearest = hyper_planner::nearest_pose_index(
      path, robot.pose.position.x, robot.pose.position.y, distance);

    if (params_.max_start_distance_m > 0.0 && distance > params_.max_start_distance_m) {
      RCLCPP_ERROR(
        get_logger(),
        "Vehicle is %.1f m from the nearest pose of this segment, beyond max_start_distance_m "
        "(%.1f).", distance, params_.max_start_distance_m);
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
        path, robot.pose.position.x, robot.pose.position.y, params_.lead_in_spacing_m);
    }

    // 진입 경로까지 붙인 다음에 다시 깝니다 -- 그래야 경로 전체가 균일한 간격이 됩니다.
    const std::size_t before_resample = path.poses.size();
    hyper_planner::resample_path(path, params_.path_resample_spacing_m);

    RCLCPP_INFO(
      get_logger(),
      "Segment starts %.2f m from the vehicle (%zu passed pose(s) dropped, %zu lead-in "
      "pose(s) prepended); %zu poses -> %zu after resampling at %.2f m.",
      distance, dropped, lead_in, before_resample, path.poses.size(),
      params_.path_resample_spacing_m);
    return true;
  }

  bool lookup_robot_pose(
    const std::string & frame, geometry_msgs::msg::PoseStamped & pose, double timeout_s)
  {
    try {
      // TimePointZero = 가장 최근 값. 골을 보내는 시점에는 차가 서 있으므로
      // 이 정도 지연은 문제가 되지 않습니다.
      const auto tf = tf_buffer_->lookupTransform(
        frame, params_.robot_base_frame, tf2::TimePointZero, tf2::durationFromSec(timeout_s));
      pose.header = tf.header;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No '%s' -> '%s' transform after %.2f s: %s. Is odometry running?",
        frame.c_str(), params_.robot_base_frame.c_str(), timeout_s, ex.what());
      return false;
    }
  }

  bool distance_to_waypoint(std::size_t index, double & distance_m)
  {
    geometry_msgs::msg::PoseStamped robot;
    if (index >= waypoints_.points.size() ||
      !lookup_robot_pose(waypoints_.frame_id, robot, params_.tf_timeout_sec))
    {
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

    // 신호를 세는 상황은 둘입니다. kWaiting은 정지선에 서서 기다리는 중이고,
    // kDriving + prearmed_는 정지선으로 다가가며 미리 보는 중입니다. 어느 쪽이든
    // 기준이 되는 값은 wait_signal 스텝에 적힌 accepted입니다.
    const Step * wait_step = nullptr;
    if (phase_ == Phase::kWaiting) {
      wait_step = &steps_[step_index_];
    } else if (phase_ == Phase::kDriving && prearmed_) {
      wait_step = &steps_[steps_[step_index_].prearm_wait_step];
    }
    if (wait_step == nullptr) {
      return;
    }

    const bool match =
      std::find(wait_step->accepted.begin(), wait_step->accepted.end(), value) !=
      wait_step->accepted.end();
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
    arrival_requested_ = false;
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
    arrival_requested_ = false;
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
    arrival_requested_ = false;
    prearmed_ = false;
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

  // 파라미터는 전부 여기에 있습니다(src/mission_manager_parameters.yaml에서 생성).
  std::shared_ptr<mission_manager::ParamListener> param_listener_;
  mission_manager::Params params_;
  bool params_logged_once_{false};

  hyper_planner::WaypointFile waypoints_;
  std::unordered_map<std::string, std::size_t> label_index_;
  std::vector<Step> steps_;

  Phase phase_{Phase::kIdle};
  std::size_t step_index_{0};
  int retries_{0};
  bool skip_requested_{false};
  // 도착으로 치려고 우리가 건 취소인지(true), 사람이 부른 '~/cancel'인지 구분합니다.
  bool arrival_requested_{false};
  // prearm으로 갈아끼운 옛 골. 그 골의 결과와 피드백은 무시해야 합니다.
  bool has_superseded_goal_{false};
  rclcpp_action::GoalUUID superseded_goal_id_{};
  rclcpp::Time hold_until_;
  rclcpp::Time wait_until_;
  rclcpp::Time starting_since_;
  int sign_streak_{0};
  std::string last_sign_;
  // 지금 실행 중인 drive 스텝에서 신호를 미리 보는 중인지.
  bool prearmed_{false};

  // 지금 보낸 경로와 그 위에서의 진행도. prearm과 cancel-on-arrival이 이 값으로 판단합니다.
  // nav2 피드백의 distance_to_goal은 쓰지 않습니다(update_progress의 주석 참고).
  nav_msgs::msg::Path active_path_;
  std::vector<double> path_suffix_len_;   // 각 인덱스에서 경로 끝까지 남은 길이
  std::size_t path_cursor_{0};            // 앞으로만 움직입니다
  bool progress_valid_{false};
  double remaining_path_m_{0.0};          // 커서에서 경로 끝까지
  double goal_point_distance_m_{0.0};     // 차량에서 경로 마지막 점까지 직선거리

  // "여기서 서야 한다"는 지점(= 스텝의 라벨). decel 프로파일을 안 쓰면 경로 끝과 같습니다.
  std::size_t stop_index_{0};             // 보낸 경로 위에서의 인덱스
  double stop_x_{0.0};
  double stop_y_{0.0};
  double distance_to_stop_m_{0.0};        // 커서에서 정지점까지 (지나가면 음수)
  double stop_point_distance_m_{0.0};     // 차량에서 정지점까지 직선거리
  rclcpp::Time progress_ok_since_;        // 진행도가 마지막으로 유효했던 시각

  // 마지막으로 내보낸 속도 제한. 0.0 = NO_SPEED_LIMIT(해제).
  double published_speed_limit_{0.0};

  // FollowPath 피드백에서 가져오는 유일한 값. |cmd_vel|이라 프레임과 무관합니다.
  bool have_speed_{false};
  double last_speed_{0.0};
  rclcpp::Time last_feedback_time_;

  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
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
