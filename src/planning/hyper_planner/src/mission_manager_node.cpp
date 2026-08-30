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
// 분기 -- 차선 안내 표지에 따라 두 경로 중 하나 (branch 스텝)
//   코스 끝의 갈림길처럼 "표지가 알려 주는 대로 가야 하는" 곳에 씁니다. 웨이포인트 CSV
//   하나는 한 번 주행해 녹화한 것이라 갈림길을 표현할 수 없으므로(라벨이 CSV를 따라 단조
//   증가해야 합니다), 갈래마다 CSV를 따로 녹화해 courses로 등록하고 branch가 고릅니다.
//
//   판정은 "같은 값이 debounce_frames 연속"입니다. wait_signal의 "허용 목록 안이기만 하면
//   됨"과 다른데, 분기는 어느 값이 나왔는지가 곧 어느 길이기 때문입니다(pick_branch).
//
//   wait_signal과 결정적으로 다른 점은 "못 봤다"의 안전한 답이 없다는 것입니다. 신호등은
//   못 보면 서 있으면 되지만 갈림길에서는 어디로든 가야 하므로, timeout이 지나면 반드시
//   default 갈래로 갑니다. 그래서 default가 필수이고 timeout_s는 짧습니다.
//
//   prearm도 됩니다. 확인되면 서지 않고 그대로 갈래로 들어가는데, 이때 골 경로는 "지금
//   위치 -> 분기 지점(지금 코스) -> 고른 갈래의 끝(갈래 코스)"이라 두 코스에 걸칩니다.
//   그것이 신호등 prearm과의 유일한 차이입니다(try_preempt_for_branch).
//
// 장애물 회피는 스텝이 아닙니다 -- MPPI가 local costmap을 보며 해당 drive 스텝 안에서
// 알아서 처리합니다.
//
//   막힘 유지 -- 피하지 말고 서서 기다린다 (drive 스텝의 obstacle_hold_s, 0이면 끔)
//     반대로 "피하지 말고 서야 하는" 구간(가속 구간)은 경로를 벗어나지 않는 RPP로 달립니다.
//     RPP는 앞이 막히면 제어를 포기하고 액션이 abort 되는데, 이 abort는 설정 오류가 아니라
//     "지금은 못 간다"입니다. 그래서 goal_retry_limit을 태우지 않고 그 자리에 섰다가
//     주기적으로 같은 골을 다시 보내, 장애물이 치워지면 스스로 이어서 갑니다.
//     자세한 것은 아래 enter_blocked() 위의 주석을 보세요.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
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

#include "hyper_planner/mission_manager_parameters.hpp"
#include "hyper_planner/common.hpp"
#include "hyper_planner/mission_loader.hpp"
#include "hyper_planner/path_loader.hpp"
#include "hyper_planner/path_progress.hpp"
#include "hyper_planner/speed_limit.hpp"

namespace
{
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

// 스텝의 정의와 mission.yaml 로드는 mission_loader.hpp에 있습니다.
using hyper_planner::Step;
using hyper_planner::StepType;
using hyper_planner::join_values;
using hyper_planner::type_name;

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
  // 앞이 막혀 선 상태(obstacle_hold_s를 켠 스텝에서만). 골이 없으므로 차는 워치독이
  // 세우고 있고, obstacle_retry_period_sec마다 같은 골을 다시 보내 봅니다.
  kBlocked,
  kFinished,
  kFailed,
};

// FollowPath 피드백의 speed가 이 시간보다 오래되면 못 믿습니다.
constexpr double kFeedbackStaleSeconds = 0.5;

// 진행도 계산용 tf 조회 타임아웃. tick마다 부르므로 골을 보낼 때(tf_timeout_sec, 기본 5초)와
// 달리 짧아야 합니다 -- 길게 잡으면 tf가 잠깐 비는 동안 타이머가 통째로 멈춥니다.
constexpr double kProgressTfTimeoutSeconds = 0.1;

// 골 경로를 이룰 구간 하나 = 어느 코스의 어느 웨이포인트 범위(양끝 포함).
// 보통은 하나지만, 분기를 서지 않고 통과할 때는 main 코스와 갈래 코스 둘이 됩니다.
struct PathSegment
{
  const hyper_planner::Course * course{nullptr};
  std::size_t begin_index{0};
  std::size_t end_index{0};
};

// 경로를 만들다 실패했을 때, 다시 시도해 볼 만한 실패(kRetry: 아직 tf가 없다 등)와
// 설정이 틀려서 영영 안 될 실패(kInvalid)를 구분합니다.
enum class PathBuild
{
  kOk,
  kRetry,
  kInvalid,
};

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
    // rclcpp::Time의 기본 생성자는 시스템 클록이라, use_sim_time인 now()와 비교하면
    // 던집니다. 처음 읽히기 전에 반드시 대입되지만 그래도 여기서 맞춰 둡니다.
    blocked_since_ = now();
    blocked_retry_at_ = now();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // spin_thread = true: 아래 lookupTransform()이 타임아웃까지 블록하므로 리스너는
    // 버퍼를 계속 채울 자기 스레드가 필요합니다.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    // 나중에 붙는 RViz/툴이 받을 수 있도록 latch 합니다.
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).transient_local());
    // controller_server가 QoS(10)으로 구독합니다.
    speed_limit_ = std::make_unique<hyper_planner::SpeedLimitPublisher>(
      create_publisher<nav2_msgs::msg::SpeedLimit>(params_.speed_limit_topic, rclcpp::QoS(10)),
      params_.frame_id);

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
    hyper_planner::MissionLoadConfig config;
    config.waypoint_csv = params_.waypoint_csv;
    config.min_spacing_m = params_.min_spacing_m;
    config.frame_id = params_.frame_id;
    config.mission_yaml = params_.mission_yaml;
    config.controller_id = params_.controller_id;
    config.goal_checker_id = params_.goal_checker_id;
    config.cancel_on_arrival_m = params_.cancel_on_arrival_m;
    config.decel_profile_a = params_.decel_profile_a;
    config.decel_profile_lookahead_m = params_.decel_profile_lookahead_m;
    config.sign_topic = params_.sign_topic;

    hyper_planner::MissionLoader loader(get_logger(), config);
    if (!loader.load()) {
      return false;
    }
    courses_ = std::move(loader.courses());
    steps_ = std::move(loader.steps());
    return true;
  }

  // ---------------------------------------------------------------- 코스 접근

  const hyper_planner::Course & course_of(const Step & step) const
  {
    return courses_[step.course_id];
  }

  PathSegment segment_of(const Step & step) const
  {
    return PathSegment{&course_of(step), step.begin_index, step.end_index};
  }

  // ---------------------------------------------------------------- 분기 판정

  // 로그용. "allow -> left_route, ban -> right_route".
  static std::string branch_values(const Step & branch)
  {
    std::string text;
    for (const auto & branch_case : branch.cases) {
      if (!text.empty()) {
        text += ", ";
      }
      text += join_values(branch_case.accepted) + " -> " + branch_case.route;
    }
    return text;
  }

  static std::string route_name(const Step & branch, std::size_t target)
  {
    for (const auto & branch_case : branch.cases) {
      if (branch_case.target == target) {
        return branch_case.route;
      }
    }
    return branch.default_route;
  }

  // 지금까지 본 신호로 갈래를 고를 수 있으면 true.
  //
  // wait_signal의 sign_streak_이 아니라 value_streak_을 쓰는 이유: 신호등은 "허용 목록
  // 안이기만 하면" 되지만(green이든 left_arrow든 결과가 같습니다), 분기는 *어느 값이*
  // 확인됐는지가 곧 어느 길로 가느냐입니다. 그래서 같은 값이 연속으로 몇 프레임 나왔는지를
  // 따로 셉니다. 두 갈래가 번갈아 보이면 어느 쪽도 확정되지 않고, timeout 뒤 default로
  // 갑니다 -- 애매할 때 찍지 않는 쪽이 맞습니다.
  bool pick_branch(const Step & branch, std::size_t & target, std::string & matched) const
  {
    if (value_streak_ < branch.debounce_frames) {
      return false;
    }
    for (const auto & branch_case : branch.cases) {
      if (std::find(branch_case.accepted.begin(), branch_case.accepted.end(), streak_value_) !=
        branch_case.accepted.end())
      {
        target = branch_case.target;
        matched = streak_value_;
        return true;
      }
    }
    return false;
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
    blocked_ = false;
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
        value_streak_ = 0;
        RCLCPP_INFO(
          get_logger(), "%s waiting for '%s' on %s (%d frame(s), timeout %.0f s).",
          progress().c_str(), join_values(step.accepted).c_str(), params_.sign_topic.c_str(),
          step.debounce_frames, step.timeout_s);
        break;
      case StepType::kBranch:
        // 분기도 "서서 신호를 본다"는 점은 wait_signal과 같으므로 같은 phase를 씁니다.
        // 다른 것은 판정 결과입니다 -- 다음 스텝으로 가는 게 아니라 갈래를 고릅니다.
        phase_ = Phase::kWaiting;
        wait_until_ = now() + rclcpp::Duration::from_seconds(step.timeout_s);
        sign_streak_ = 0;
        value_streak_ = 0;
        RCLCPP_INFO(
          get_logger(), "%s branching on %s (%s; %d frame(s), timeout %.0f s -> '%s').",
          progress().c_str(), params_.sign_topic.c_str(), branch_values(step).c_str(),
          step.debounce_frames, step.timeout_s, step.default_route.c_str());
        break;
    }
    publish_status(status_text());
  }

  // 다음 스텝은 항상 Step::next_index입니다. 보통은 바로 다음 스텝이지만, 분기의 갈래
  // (route)는 마지막에 합류 지점으로 되돌아가거나 미션을 끝냅니다(kEndOfMission).
  void advance()
  {
    goto_step(
      step_index_ < steps_.size() ? steps_[step_index_].next_index : steps_.size());
  }

  void goto_step(std::size_t index)
  {
    step_index_ = index;
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
      case Phase::kWaiting:
        if (steps_[step_index_].type == StepType::kBranch) {
          tick_branch();
        } else {
          tick_wait_signal();
        }
        break;
      case Phase::kBlocked:
        tick_blocked();
        break;
      case Phase::kDriving:
        update_progress();
        update_prearm();
        // 통과 신호가 확인되면 설 이유가 없으므로 cancel-on-arrival보다 먼저 봅니다.
        if (!try_preempt()) {
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
        {segment_of(step)}, step.tail_after_label_m, step.reverse, step.label, path))
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
          fail(
            "could not locate the vehicle in frame '" +
            course_of(step).waypoints.frame_id + "'");
        }
        return;
    }

    RCLCPP_INFO(
      get_logger(), "%s sending %zu-pose path to '%s'.",
      progress().c_str(), path.poses.size(), params_.action_name.c_str());
    phase_ = Phase::kDriving;   // 응답이 올 때까지 재전송을 막습니다.
    send_goal(step, path);
  }

  // ------------------------------------------------------------- 신호 대기/분기

  void tick_wait_signal()
  {
    const Step & step = steps_[step_index_];
    if (sign_streak_ >= step.debounce_frames) {
      RCLCPP_INFO(get_logger(), "Signal '%s' confirmed; going.", last_sign_.c_str());
      advance();
    } else if (now() >= wait_until_) {
      if (params_.proceed_on_signal_timeout) {
        RCLCPP_WARN(
          get_logger(),
          "No '%s' within %.0f s (last saw '%s'). Proceeding anyway -- check the traffic "
          "light detector.", join_values(step.accepted).c_str(), step.timeout_s,
          last_sign_.empty() ? "nothing" : last_sign_.c_str());
        advance();
      } else {
        fail("timed out waiting for signal '" + join_values(step.accepted) + "'");
      }
    }
  }

  // 분기 지점에 서서 차선 안내 신호를 읽고 갈래를 고릅니다.
  //
  // wait_signal과 결정적으로 다른 점은 "못 봤다"의 안전한 답이 없다는 것입니다. 신호등은
  // 못 보면 서 있는 게 안전하지만, 갈림길에서는 어디로든 가야 합니다. 그래서 여기에는
  // proceed_on_signal_timeout 같은 선택지가 없고 default 갈래로 반드시 갑니다 -- 대신
  // timeout_s를 짧게 잡습니다(더 기다린다고 더 나은 답이 나오지 않습니다).
  void tick_branch()
  {
    const Step & step = steps_[step_index_];
    std::size_t target = 0;
    std::string matched;
    if (pick_branch(step, target, matched)) {
      RCLCPP_INFO(
        get_logger(), "%s branch: '%s' confirmed; taking route '%s'.",
        progress().c_str(), matched.c_str(), route_name(step, target).c_str());
      goto_step(target);
      return;
    }
    if (now() >= wait_until_) {
      RCLCPP_WARN(
        get_logger(),
        "%s branch: no lane sign confirmed within %.0f s (last saw '%s'). Taking the default "
        "route '%s' -- check the sign detector.",
        progress().c_str(), step.timeout_s, last_sign_.empty() ? "nothing" : last_sign_.c_str(),
        step.default_route.c_str());
      goto_step(step.default_target);
    }
  }

  // ------------------------------------------------------------------ 막힘 처리
  //
  // 경로를 벗어나지 않는 컨트롤러(ForwardFollowPath/ReverseFollowPath = RPP)는 앞이
  // 막히면 회피하지 않고 제어를 포기합니다. controller_server는 그동안 0 속도를 내보내다
  // failure_tolerance가 지나면 액션을 abort 합니다. 그 abort는 "설정이 틀렸다"가 아니라
  // "지금 못 간다"이므로, goal_retry_limit을 태워 미션을 죽일 이유가 없습니다.
  //
  // 그래서 obstacle_hold_s를 켠 스텝에서는 abort를 이렇게 다룹니다.
  //   1. 골을 안 보낸 채로 섭니다. 정지 명령은 따로 필요 없습니다 -- /cmd_vel이 끊기면
  //      cmd_vel_to_ackermann의 워치독(0.3초)이 차를 세웁니다.
  //   2. obstacle_retry_period_sec마다 같은 골을 다시 보냅니다. 경로는 매번 현재 위치에서
  //      다시 잘리므로(trim_to_robot) 선 자리에서 그대로 이어집니다. 아직 막혀 있으면
  //      컨트롤러가 곧바로 다시 abort 하고 여기로 돌아옵니다.
  //   3. 다시 굴러가기 시작하면(피드백 속도 > unblocked_speed) 타이머를 0으로 되돌립니다.
  //   4. 계속 막힌 채로 obstacle_hold_s가 지나면 그때는 미션을 실패로 끝냅니다 -- 영영
  //      서 있는 것보다는 사람이 알아채는 편이 낫습니다.
  void enter_blocked(const Step & step)
  {
    if (!blocked_) {
      blocked_ = true;
      blocked_since_ = now();
    }
    phase_ = Phase::kBlocked;
    blocked_retry_at_ = now() + rclcpp::Duration::from_seconds(params_.obstacle_retry_period_sec);
    RCLCPP_WARN(
      get_logger(),
      "%s blocked on the way to '%s' (the controller could not find a valid command -- most "
      "likely an obstacle ahead). Holding; retrying every %.1f s, giving up after %.0f s "
      "(blocked for %.1f s so far).",
      progress().c_str(), step.label.c_str(), params_.obstacle_retry_period_sec,
      step.obstacle_hold_s, (now() - blocked_since_).seconds());
    publish_status("blocked " + status_text());
  }

  void tick_blocked()
  {
    const Step & step = steps_[step_index_];
    const double held = (now() - blocked_since_).seconds();
    if (held > step.obstacle_hold_s) {
      fail(
        "blocked in front of '" + step.label + "' for " + std::to_string(static_cast<int>(held)) +
        " s (obstacle_hold_s " + std::to_string(static_cast<int>(step.obstacle_hold_s)) + ")");
      return;
    }
    if (now() >= blocked_retry_at_) {
      RCLCPP_INFO(
        get_logger(), "%s blocked for %.1f s; re-sending the goal to see if the way is clear.",
        progress().c_str(), held);
      phase_ = Phase::kStarting;
      starting_since_ = now();
    }
  }

  // 세그먼트 경로를 만들어 차량의 현재 위치에 맞춰 다듬습니다. 실패 사유는 여기서 로그로
  // 남기므로, 호출자는 kRetry / kInvalid만 보고 어떻게 할지 정하면 됩니다.
  //
  // 세그먼트를 여러 개 받는 이유는 분기 prearm 때문입니다. 갈래로 서지 않고 들어갈 때의
  // 골은 "지금 위치 -> 분기 지점(main 코스) -> 고른 갈래의 끝(갈래 코스)"이라 두 코스에
  // 걸칩니다. 세그먼트가 하나면 예전과 똑같이 동작합니다.
  //
  // tail_m > 0이면 라벨 뒤로 그만큼 직선 꼬리를 덧붙입니다(decel 프로파일). 꼬리는
  // trim_to_robot보다 먼저 붙입니다 -- 그래야 그 안의 resample이 꼬리까지 같은 간격으로
  // 다시 깔아 주고(MPPI의 경로 critic은 '미터'가 아니라 '점 개수'로 셉니다), 그러면서도
  // 꼭짓점을 옮기지 않으므로 꼬리 길이는 tail_m 그대로 남습니다. set_stop_point가 라벨을
  // 되찾을 때 그 길이에 의존합니다.
  PathBuild build_path(
    const std::vector<PathSegment> & segments, double tail_m, bool reverse,
    const std::string & label, nav_msgs::msg::Path & path)
  {
    path = nav_msgs::msg::Path{};
    path.header.frame_id = segments.front().course->waypoints.frame_id;
    path.header.stamp = now();

    for (const PathSegment & segment : segments) {
      const nav_msgs::msg::Path part = hyper_planner::make_path(
        segment.course->waypoints.points, segment.begin_index, segment.end_index,
        path.header.frame_id, path.header.stamp, reverse);
      if (part.poses.size() < 2) {
        RCLCPP_ERROR(
          get_logger(), "Segment for '%s' (course '%s', wp #%zu..#%zu) has fewer than 2 poses.",
          label.c_str(), segment.course->name.c_str(), segment.begin_index, segment.end_index);
        return PathBuild::kInvalid;
      }
      // 이음매의 중복점 하나를 버립니다. 같은 코스를 이어 붙일 때 앞 세그먼트의 끝과 뒤
      // 세그먼트의 시작이 같은 웨이포인트이고, 갈래 CSV를 분기 지점에서 정확히 시작해
      // 녹화했을 때도 그렇습니다. 떨어져 있으면(이음매가 있으면) 아무것도 안 버립니다.
      std::size_t first = 0;
      if (!path.poses.empty()) {
        const auto & tail = path.poses.back().pose.position;
        const auto & head = part.poses.front().pose.position;
        if (std::hypot(head.x - tail.x, head.y - tail.y) < 1e-6) {
          first = 1;
        }
      }
      path.poses.insert(
        path.poses.end(), part.poses.begin() + static_cast<std::ptrdiff_t>(first),
        part.poses.end());
    }
    if (path.poses.size() < 2) {
      RCLCPP_ERROR(
        get_logger(), "Path for '%s' has fewer than 2 poses.", label.c_str());
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
    progress_.reset(path);
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
      // (path_progress.hpp의 주석 참고). 거리는 PathProgress가 직접 셉니다.
      last_speed_ = feedback->speed;
      last_feedback_time_ = now();
      have_speed_ = true;
      // 다시 굴러가기 시작했으면 막힘 타이머를 되돌립니다. 한 스텝 안에서 장애물을
      // 여러 번 만나도 매번 obstacle_hold_s를 처음부터 씁니다.
      if (blocked_ && std::fabs(last_speed_) > params_.unblocked_speed) {
        blocked_ = false;
        RCLCPP_INFO(
          get_logger(), "%s moving again at %.2f m/s; the way ahead is clear.",
          progress().c_str(), std::fabs(last_speed_));
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        // limit 0.00 = 제한 없음(nav2의 NO_SPEED_LIMIT과 같은 뜻).
        "%s %.2f m to '%s' (%.2f m direct), speed=%.2f m/s, limit=%.2f m/s",
        progress().c_str(), progress_.distance_to_stop_m(), steps_[step_index_].label.c_str(),
        progress_.stop_point_distance_m(), feedback->speed, speed_limit_->last());
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

  // 진행도를 한 tick 갱신합니다. 기하 계산은 PathProgress가 하고, 여기서는 tf에서 차량
  // 좌표를 얻어 넣어 주는 것과 "마지막으로 유효했던 시각"을 적어 두는 것만 합니다.
  void update_progress()
  {
    geometry_msgs::msg::PoseStamped robot;
    if (!lookup_robot_pose(
        progress_.path().header.frame_id, robot, kProgressTfTimeoutSeconds))
    {
      progress_.invalidate();
      return;
    }
    if (progress_.update(
        robot.pose.position.x, robot.pose.position.y, params_.progress_search_window_m))
    {
      progress_ok_since_ = now();
    }
  }

  // 이 스텝에서 "여기서 서야 한다"는 지점을 PathProgress에 알려 줍니다. 라벨의 웨이포인트
  // 좌표를 쓰고, 그게 없으면(끝 인덱스가 CSV 범위 밖) 경로의 마지막 점으로 대신합니다.
  void set_stop_point(const Step & step, const nav_msgs::msg::Path & path)
  {
    const auto & points = course_of(step).waypoints.points;
    double stop_x = 0.0;
    double stop_y = 0.0;
    if (step.end_index < points.size()) {
      stop_x = points[step.end_index].x;
      stop_y = points[step.end_index].y;
    } else if (!path.poses.empty()) {
      stop_x = path.poses.back().pose.position.x;
      stop_y = path.poses.back().pose.position.y;
    }
    progress_.set_stop_point(step.tail_after_label_m, stop_x, stop_y);

    if (step.tail_after_label_m > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "Stop point for '%s' is pose %zu/%zu of the sent path (%.1f m of path runs past it).",
        step.label.c_str(), progress_.stop_index(), path.poses.size(), progress_.stop_tail_m());
    }
  }

  bool is_superseded(const GoalHandle::SharedPtr & handle) const
  {
    return has_superseded_goal_ && handle && handle->get_goal_id() == superseded_goal_id_;
  }

  // 지금 실행 중인 골에 다가가는 동안 다음 wait_signal/branch의 신호를 미리 볼지 정합니다.
  void update_prearm()
  {
    const Step & step = steps_[step_index_];
    const bool in_range = step.prearm_enabled && progress_.valid() &&
      progress_.distance_to_stop_m() <= step.prearm_distance_m;

    if (in_range && !prearmed_) {
      // 진입하는 순간 streak을 비웁니다. 멀리서 -- 표지가 아직 몇 픽셀일 때 -- 우연히
      // 쌓인 연속 프레임이 그대로 통과/분기 판정으로 이어지지 않게 하려는 것입니다.
      sign_streak_ = 0;
      value_streak_ = 0;
      const Step & watched = steps_[step.prearm_wait_step];
      const std::string looking_for = watched.type == StepType::kBranch
        ? branch_values(watched) : join_values(watched.accepted);
      RCLCPP_INFO(
        get_logger(), "%s %.1f m from '%s'; watching %s for '%s'.",
        progress().c_str(), progress_.distance_to_stop_m(), step.label.c_str(),
        params_.sign_topic.c_str(), looking_for.c_str());
    }
    prearmed_ = in_range;
  }

  // 지금 스텝이 미리 보고 있는 것이 신호등인지 분기인지에 따라 갈라 줍니다.
  bool try_preempt()
  {
    const Step & step = steps_[step_index_];
    if (!step.prearm_enabled) {
      return false;
    }
    return steps_[step.prearm_wait_step].type == StepType::kBranch
      ? try_preempt_for_branch() : try_preempt_for_signal();
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

    // 두 세그먼트가 같은 코스에서 이어지는 것이 보통입니다(정지선 앞뒤). 그때는 예전처럼
    // 하나의 연속 구간으로 만듭니다.
    std::vector<PathSegment> segments;
    if (step.course_id == merge_step.course_id) {
      segments.push_back({&course_of(step), step.begin_index, merge_step.end_index});
    } else {
      segments.push_back(segment_of(step));
      segments.push_back(segment_of(merge_step));
    }

    nav_msgs::msg::Path path;
    if (build_path(
        segments, merge_step.tail_after_label_m, false, merge_step.label, path) != PathBuild::kOk)
    {
      // 다음 tick에 다시 시도합니다. 끝내 못 만들면 prearm이 그냥 안 일어나고, 원래 골
      // 그대로 정지선에 서서 wait_signal이 처리합니다 -- 안전한 쪽으로 실패합니다.
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s '%s' confirmed %.1f m before '%s'; skipping the stop and continuing to '%s' "
      "(%zu-pose path preempts the running goal).",
      progress().c_str(), last_sign_.c_str(), progress_.distance_to_stop_m(), step.label.c_str(),
      merge_step.label.c_str(), path.poses.size());

    // 갈아끼워진 옛 골은 nav2가 abort로 끝냅니다. 그 결과가 뒤늦게 도착했을 때 ABORTED
    // 재시도 로직이 돌면 방금 보낸 골을 덮어쓰므로, id를 적어 두고 무시합니다.
    superseded_goal_id_ = goal_handle_->get_goal_id();
    has_superseded_goal_ = true;

    step_index_ = merge_index;
    retries_ = 0;
    prearmed_ = false;
    sign_streak_ = 0;
    value_streak_ = 0;
    send_goal(merge_step, path);
    return true;
  }

  // 분기의 prearm -- 차선 안내 신호가 확인되면 서지 않고 그대로 갈래로 들어갑니다.
  //
  // 신호등 prearm과 다른 점은 이어 붙일 구간이 두 코스에 걸친다는 것뿐입니다. 골 경로는
  // "지금 위치 -> 분기 지점(지금 스텝의 코스) -> 고른 갈래의 끝(갈래 코스)"입니다. 갈래
  // CSV의 첫 점이 실제로 분기 지점에 붙어 있는지는 로드 시점에 이미 검사했으므로
  // (mission_loader.hpp의 check_branch_seams) 여기서 이음매를 다시 재지 않습니다.
  //
  // 확인이 안 되면 아무 일도 일어나지 않고 원래 골 그대로 분기 지점에 섭니다. 그러면
  // branch 스텝이 서서 신호를 읽고, 그래도 못 읽으면 default 갈래로 갑니다.
  bool try_preempt_for_branch()
  {
    const Step & step = steps_[step_index_];
    if (!prearmed_ || !goal_handle_ || arrival_requested_) {
      return false;
    }
    const Step & branch = steps_[step.prearm_wait_step];
    std::size_t target = 0;
    std::string matched;
    if (!pick_branch(branch, target, matched)) {
      return false;
    }

    const Step & route_step = steps_[target];
    nav_msgs::msg::Path path;
    const std::vector<PathSegment> segments{segment_of(step), segment_of(route_step)};
    if (build_path(
        segments, route_step.tail_after_label_m, false, route_step.label,
        path) != PathBuild::kOk)
    {
      // 다음 tick에 다시 시도합니다. 끝내 못 만들면 분기 지점에 서서 고릅니다.
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s '%s' confirmed %.1f m before '%s'; taking route '%s' without stopping "
      "(%zu-pose path across courses '%s' -> '%s' preempts the running goal).",
      progress().c_str(), matched.c_str(), progress_.distance_to_stop_m(), step.label.c_str(),
      route_name(branch, target).c_str(), path.poses.size(), course_of(step).name.c_str(),
      course_of(route_step).name.c_str());

    superseded_goal_id_ = goal_handle_->get_goal_id();
    has_superseded_goal_ = true;

    step_index_ = target;
    retries_ = 0;
    prearmed_ = false;
    sign_streak_ = 0;
    value_streak_ = 0;
    send_goal(route_step, path);
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
    if (step.decel_profile_a > 0.0 && !progress_.valid() &&
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

    if (!progress_.valid()) {
      return;
    }

    // 하드 백스톱 2 -- 정지점을 지났으면 속도와 무관하게 취소합니다. 프로파일 스텝에서는
    // 이것이 "정지선을 넘지 않는다"의 마지막 보루입니다.
    if (progress_.distance_to_stop_m() <= 0.0) {
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
    if (progress_.distance_to_stop_m() > step.cancel_on_arrival_m ||
      progress_.stop_point_distance_m() > step.cancel_on_arrival_m ||
      std::fabs(last_speed_) > params_.cancel_on_arrival_speed)
    {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s %.2f m from '%s' at %.2f m/s; canceling instead of crawling into the goal checker.",
      progress().c_str(), progress_.stop_point_distance_m(), step.label.c_str(),
      std::fabs(last_speed_));
    arrival_requested_ = true;
    client_->async_cancel_goal(goal_handle_);
  }

  // ------------------------------------------------------------- decel 프로파일

  // 지금 스텝에 맞는 속도 제한을 골라 내보냅니다. 프로파일 자체는
  // speed_limit.hpp의 decel_profile_speed가 계산합니다.
  void update_speed_limit()
  {
    double limit = 0.0;   // 0.0 = NO_SPEED_LIMIT

    if (phase_ == Phase::kDriving && step_index_ < steps_.size()) {
      const Step & step = steps_[step_index_];
      if (step.decel_profile_a > 0.0) {
        if (!progress_.valid()) {
          // tf를 잠깐 놓친 것뿐일 수 있습니다. 제한을 해제하면 그 순간 vx_max로 튀어
          // 나가므로, 마지막 값을 그대로 둡니다. 오래 끊기면 위 백스톱이 골을 취소합니다.
          return;
        }
        limit = hyper_planner::decel_profile_speed(
          step.decel_profile_a, progress_.distance_to_stop_m(), step.cancel_on_arrival_m,
          params_.controller_vx_max, params_.decel_profile_min_speed);
      }
    }

    speed_limit_->publish(limit, now());
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
        if (distance_to_waypoint(step, distance) && distance <= params_.arrival_slack_m) {
          RCLCPP_WARN(
            get_logger(),
            "%s aborted, but the vehicle is %.2f m from '%s' (within arrival_slack_m %.2f). "
            "Counting it as arrived.",
            progress().c_str(), distance, step.label.c_str(), params_.arrival_slack_m);
          advance();
          return;
        }
        // 이 스텝에서의 abort는 "지금 앞이 막혔다"는 뜻입니다. 재시도 횟수를 태우지 않고
        // 그 자리에 섰다가, 치워지면 이어서 갑니다(enter_blocked 위의 주석 참고).
        if (step.obstacle_hold_s > 0.0) {
          enter_blocked(step);
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
    std::size_t nearest = hyper_planner::nearest_pose_index(
      path, robot.pose.position.x, robot.pose.position.y, distance);

    // 닫힌 코스에서 출발선에 선 차는 코스의 끝점에도 그만큼 가깝습니다. 이때
    // nearest_pose_index가 끝점을 고르면 아래 trim이 코스 전체를 "이미 지나온
    // 구간"으로 버려서, 골이 출발점 옆에 놓이고 goal checker가 곧바로 도착으로
    // 판정합니다("출발하자마자 완주"). 경로의 첫 점과 끝 점이 loop_close_distance_m
    // 안으로 붙어 있고 nearest가 그 끝자락(끝에서 잰 arc 길이 기준)에 붙었으면,
    // 커서를 경로 앞쪽(0)으로 되돌려 한 바퀴를 통째로 보냅니다. -- path_progress.hpp의
    // 전진 커서가 코스 중간의 자기근접 구간을 다루는 것과 같은 취지입니다.
    if (params_.loop_close_distance_m > 0.0 && path.poses.size() > 2 && nearest > 0) {
      const auto & first = path.poses.front().pose.position;
      const auto & last_pose = path.poses.back().pose.position;
      const double seam = std::hypot(last_pose.x - first.x, last_pose.y - first.y);
      if (seam <= params_.loop_close_distance_m) {
        double tail_arc = 0.0;
        for (std::size_t i = path.poses.size() - 1; i > nearest; --i) {
          const auto & a = path.poses[i - 1].pose.position;
          const auto & b = path.poses[i].pose.position;
          tail_arc += std::hypot(b.x - a.x, b.y - a.y);
        }
        if (tail_arc <= params_.loop_close_distance_m) {
          RCLCPP_INFO(
            get_logger(),
            "Closed-course segment (%.2f m seam); the vehicle sits near the end pose too. "
            "Snapping to the start so the whole lap is driven instead of skipped.", seam);
          nearest = 0;
          distance = std::hypot(
            first.x - robot.pose.position.x, first.y - robot.pose.position.y);
        }
      }
    }

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

  bool distance_to_waypoint(const Step & step, double & distance_m)
  {
    const hyper_planner::Course & course = course_of(step);
    geometry_msgs::msg::PoseStamped robot;
    if (step.end_index >= course.waypoints.points.size() ||
      !lookup_robot_pose(course.waypoints.frame_id, robot, params_.tf_timeout_sec))
    {
      return false;
    }
    distance_m = std::hypot(
      course.waypoints.points[step.end_index].x - robot.pose.position.x,
      course.waypoints.points[step.end_index].y - robot.pose.position.y);
    return true;
  }

  // ---------------------------------------------------------------- 신호 대기

  void on_sign(const std::string & value)
  {
    last_sign_ = value;

    // 값별 연속 프레임 수. 스텝 종류와 무관하게 항상 셉니다 -- branch는 "허용 목록 안인가"가
    // 아니라 "어느 값이 확인됐는가"로 갈래를 고르기 때문입니다(pick_branch 참고).
    if (value == streak_value_) {
      ++value_streak_;
    } else {
      streak_value_ = value;
      value_streak_ = 1;
    }

    // 아래는 wait_signal 전용 카운터입니다. 신호를 세는 상황은 둘입니다. kWaiting은
    // 정지선에 서서 기다리는 중이고, kDriving + prearmed_는 정지선으로 다가가며 미리 보는
    // 중입니다. 어느 쪽이든 기준이 되는 값은 wait_signal 스텝에 적힌 accepted입니다.
    const Step * wait_step = nullptr;
    if (phase_ == Phase::kWaiting) {
      wait_step = &steps_[step_index_];
    } else if (phase_ == Phase::kDriving && prearmed_) {
      wait_step = &steps_[steps_[step_index_].prearm_wait_step];
    }
    if (wait_step == nullptr || wait_step->type != StepType::kWaitSignal) {
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
      phase_ == Phase::kHolding || phase_ == Phase::kWaiting || phase_ == Phase::kBlocked)
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
    blocked_ = false;
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
      if (courses_.size() > 1) {
        text += " course=" + course_of(step).name;
      }
    } else if (step.type == StepType::kWaitSignal) {
      text += " value=" + join_values(step.accepted);
    } else if (step.type == StepType::kBranch) {
      text += " cases=" + branch_values(step) + " default=" + step.default_route;
    }
    return text;
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  // ---------------------------------------------------------------- 멤버

  // 파라미터는 전부 여기에 있습니다(src/mission_manager_parameters.yaml에서 생성).
  std::shared_ptr<mission_manager::ParamListener> param_listener_;
  mission_manager::Params params_;
  bool params_logged_once_{false};

  // 미션이 쓰는 코스들. 분기(branch)가 없으면 [0] = "main" 하나뿐이고, 그때 동작은
  // 코스가 하나였던 때와 완전히 같습니다.
  std::vector<hyper_planner::Course> courses_;
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
  // 막힘 상태(kBlocked). blocked_since_는 "연속으로 막혀 있던" 시작 시각이라,
  // 다시 굴러가면(피드백 속도 > unblocked_speed) 다음 막힘에서 새로 잡습니다.
  bool blocked_{false};
  rclcpp::Time blocked_since_;
  rclcpp::Time blocked_retry_at_;
  int sign_streak_{0};
  std::string last_sign_;
  // 같은 값이 연속으로 몇 프레임 나왔는지. branch가 갈래를 고를 때 씁니다(sign_streak_은
  // "허용 목록 안이었는가"라 어느 값이었는지를 잃습니다).
  std::string streak_value_;
  int value_streak_{0};
  // 지금 실행 중인 drive 스텝에서 신호를 미리 보는 중인지.
  bool prearmed_{false};

  // 지금 보낸 경로 위에서의 진행도. prearm과 cancel-on-arrival이 이 값으로 판단합니다.
  hyper_planner::PathProgress progress_;
  rclcpp::Time progress_ok_since_;        // 진행도가 마지막으로 유효했던 시각

  // FollowPath 피드백에서 가져오는 유일한 값. |cmd_vel|이라 프레임과 무관합니다.
  bool have_speed_{false};
  double last_speed_{0.0};
  rclcpp::Time last_feedback_time_;

  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<hyper_planner::SpeedLimitPublisher> speed_limit_;
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
