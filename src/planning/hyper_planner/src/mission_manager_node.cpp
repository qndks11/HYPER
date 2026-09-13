// HYPER 미션 매니저.
//
// mission/<mission>.yaml의 스텝 큐를 순서대로 실행합니다. 핵심 아이디어는
// "한 스텝 = follow_path 골 하나"입니다 -- 정지선/신호등/주차 지점이 곧 세그먼트의 끝이므로
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
//   판정은 "vote_window_s 동안 모은 표의 다수결"입니다. wait_signal의 "허용 목록 안이기만
//   하면 됨"과 다른데, 분기는 어느 값이 나왔는지가 곧 어느 길이기 때문입니다(pick_branch).
//   연속 프레임을 안 쓰는 이유는 갈림길 표지판 세 장이 나란히 깜빡여서 연속이 계속 끊기기
//   때문입니다. 어느 case에도 없는 값은 세기만 하고 이길 수 없으며, 표가 없거나 두 갈래가
//   동점이면 고르지 않고 timeout까지 계속 모읍니다.
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
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hyper_planner/mission_manager_parameters.hpp"
#include "hyper_planner/common.hpp"
#include "hyper_planner/costmap_query.hpp"
#include "hyper_planner/mission_loader.hpp"
#include "hyper_planner/path_loader.hpp"
#include "hyper_planner/path_progress.hpp"
#include "hyper_planner/speed_limit.hpp"

namespace
{
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

// 스텝의 정의와 mission.yaml 로드는 mission_loader.hpp에 있습니다.
using hyper_planner::BranchCase;
using hyper_planner::BranchSelect;
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
    paused_at_ = now();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // spin_thread = true: 아래 lookupTransform()이 타임아웃까지 블록하므로 리스너는
    // 버퍼를 계속 채울 자기 스레드가 필요합니다.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    // 나중에 붙는 RViz/툴이 받을 수 있도록 latch 합니다.
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
    // 펼쳐진 스텝 목록. GUI가 갈래(route) 스텝의 인덱스를 알 수 있는 유일한 방법입니다
    // -- mission_loader가 routes의 스텝을 main 뒤에 덧붙이므로 yaml만 봐서는 셀 수
    // 없습니다. latched라 나중에 붙는 GUI도 그대로 받습니다.
    steps_pub_ = create_publisher<std_msgs::msg::String>(
      "~/steps", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).transient_local());
    // controller_server가 QoS(10)으로 구독합니다.
    speed_limit_ = std::make_unique<hyper_planner::SpeedLimitPublisher>(
      create_publisher<nav2_msgs::msg::SpeedLimit>(params_.speed_limit_topic, rclcpp::QoS(10)),
      params_.frame_id);

    sign_sub_ = create_subscription<std_msgs::msg::String>(
      params_.sign_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {on_sign(msg->data);});

    // 로컬 코스트맵. 콜백은 최신 한 장을 들고만 있습니다 -- 실제로 세는 일은 clearance
    // 분기가 판정 중일 때와 ~/probe_costmap을 부를 때만 일어납니다. 2 Hz에 400x400이므로
    // 들고 있는 값이 싸고(약 160 KB), controller_server는 우리가 듣든 말든 발행합니다.
    costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      params_.costmap_topic, rclcpp::QoS(1),
      [this](const nav2_msgs::msg::Costmap::SharedPtr msg) {
        costmap_ = msg;
        costmap_stamp_ = now();
      });

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
    pause_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/pause",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_pause(response);
      });
    resume_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/resume",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_resume(response);
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
    goto_step_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/goto_step",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_goto_step(response);
      });
    probe_costmap_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/probe_costmap",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_probe_costmap(response);
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
    config.waypoints_dir = params_.waypoints_dir;
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
    publish_steps();
    return true;
  }

  // 펼쳐진 스텝 목록을 `index|type|label|course|route` 한 줄씩 내보냅니다.
  //
  // GUI가 이걸 받아야 하는 이유: mission_loader는 steps를 먼저 펼치고 routes의 스텝을
  // 그 뒤에 덧붙이므로, yaml만 읽어서는 갈래 스텝의 인덱스를 셀 수 없습니다. 그런데
  // '~/goto_step'은 인덱스로 가므로, 갈래 스텝을 시험 주행하려면 노드가 직접 알려
  // 주는 수밖에 없습니다.
  void publish_steps()
  {
    // 갈래 이름은 Step에 없으므로 branch의 case/default target에서 따라 내려가며
    // 붙입니다. 갈래 스텝은 덧붙인 순서대로 이어져 있어 next_index를 따라가면 됩니다.
    std::vector<std::string> route_of(steps_.size());
    for (const Step & step : steps_) {
      if (step.type != StepType::kBranch) {
        continue;
      }
      std::vector<std::pair<std::size_t, std::string>> heads;
      for (const BranchCase & branch_case : step.cases) {
        heads.emplace_back(branch_case.target, branch_case.route);
      }
      heads.emplace_back(step.default_target, step.default_route);
      for (const auto & [head, name] : heads) {
        std::size_t index = head;
        while (index < steps_.size() && route_of[index].empty() && index >= head) {
          route_of[index] = name;
          const std::size_t next = steps_[index].next_index;
          if (next <= index || next >= steps_.size()) {
            break;   // 합류했거나 미션 끝. 갈래는 여기까지입니다.
          }
          index = next;
        }
      }
    }

    std::ostringstream out;
    for (std::size_t i = 0; i < steps_.size(); ++i) {
      const Step & step = steps_[i];
      out << i << '|' << type_name(step.type) << '|' << step.label << '|'
          << (step.type == StepType::kDrive ? course_of(step).name : std::string())
          << '|' << route_of[i] << '\n';
    }
    std_msgs::msg::String msg;
    msg.data = out.str();
    steps_pub_->publish(msg);
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

  // 이긴 값이 최소한 이만큼은 표를 받아야 갈래를 정합니다. 표 한 장으로 길이 갈리는 것을
  // 막는 바닥값입니다(위 pick_branch 참고).
  static constexpr int kMinBranchVotes = 3;

  // 표를 버리고 창을 닫습니다.
  void vote_clear()
  {
    vote_counts_.clear();
    vote_total_ = 0;
    vote_open_ = false;
  }

  // 지금부터 새 창을 엽니다.
  void vote_open_now()
  {
    vote_clear();
    vote_started_ = now();
    vote_open_ = true;
  }

  // 표 현황을 "allow_left 14, allow_right 3, none 6"으로. 어느 case에도 없는 값까지 전부
  // 넣습니다 -- 운영자가 이 줄을 보는 이유가 "카메라가 실제로 뭘 냈는가"이기 때문입니다.
  std::string vote_tally() const
  {
    std::string text;
    for (const auto & entry : vote_counts_) {
      if (!text.empty()) {
        text += ", ";
      }
      text += entry.first + " " + std::to_string(entry.second);
    }
    return text.empty() ? std::string("nothing") : text;
  }

  // 창이 다 찼고 이길 값이 하나로 정해지면 true.
  //
  // wait_signal의 sign_streak_("허용 목록 안이기만 하면 됨")과 다른 이유는 예전과 같습니다
  // -- 분기는 *어느 값이* 나왔는지가 곧 어느 길입니다. 달라진 것은 세는 방법입니다. 갈림길
  // 표지는 세 장이 나란히 깜빡여서 "같은 값이 N프레임 연속"이 거의 성립하지 않으므로,
  // vote_window_s 동안 모아 다수결로 정합니다. 어느 case에도 없는 값(none, red, 짝을 못
  // 찾은 allow ...)은 세기만 하고 이길 수 없습니다. 표가 하나도 없거나 두 갈래가 동점이면
  // 아직 고르지 않습니다 -- 애매할 때 찍지 않는 쪽이 맞습니다.
  bool pick_branch(const Step & branch, std::size_t & target, std::string & matched)
  {
    // 창이 안 열려 있으면 여기서 엽니다. begin_step과 update_prearm이 정상 경로지만,
    // 그 둘을 안 거치고 들어오는 길(resume 직후 등)에서도 스스로 낫게 하려는 것입니다.
    if (!vote_open_) {
      vote_open_now();
      return false;
    }
    const double elapsed = (now() - vote_started_).seconds();
    if (elapsed < branch.vote_window_s) {
      return false;
    }

    // case 단위로 셉니다. 한 case의 `value`에 값이 여러 개 적혀 있을 수 있는데(쉼표로
    // 나뉩니다), 그 값들이 표를 나눠 가졌다고 판정이 막히면 안 됩니다 -- 어차피 같은
    // 길입니다.
    int best_votes = 0;
    bool tied = false;
    for (const auto & branch_case : branch.cases) {
      int votes = 0;
      int top_votes = -1;
      std::string top_value;
      for (const auto & value : branch_case.accepted) {
        const auto found = vote_counts_.find(value);
        const int value_votes = found == vote_counts_.end() ? 0 : found->second;
        votes += value_votes;
        if (value_votes > top_votes) {
          top_votes = value_votes;
          top_value = value;
        }
      }
      if (votes > best_votes) {
        best_votes = votes;
        tied = false;
        target = branch_case.target;
        matched = top_value;
      } else if (votes == best_votes) {
        tied = true;
      }
    }

    // 아직 못 고릅니다. 표는 그대로 둡니다 -- 계속 쌓이다 보면 다음 한 장이 동점을
    // 깹니다. 그래서 이 창은 "정확히 vote_window_s"가 아니라 "적어도 vote_window_s"입니다.
    //
    // 최소 표 수를 따로 두는 이유: prearm에서 연 창을 분기 스텝이 이어받으면 도착 시점에
    // 이미 창이 다 차 있습니다. 그 상태로 표 한 장이 들어오면 "경과 >= vote_window_s"가
    // 이미 참이라 그 한 장으로 길이 정해집니다 -- 투표를 넣은 이유가 없어집니다. 초당 몇
    // 장이 들어오는지는 detection_frequency에 따라 달라져 미리 알 수 없으므로, 비율이
    // 아니라 예전 debounce_frames와 같은 자릿수의 바닥값으로 막습니다.
    if (best_votes < kMinBranchVotes || tied) {
      return false;
    }

    // 경과 시간을 찍습니다(vote_window_s가 아니라): 한 번 못 고르고 지나간 창은 둘이
    // 다르고, 표가 초당 몇 장 들어왔는지를 봐야 판단이 섭니다.
    RCLCPP_INFO(
      get_logger(), "Branch vote over %.1f s: %s (%d sample(s)) -> '%s'.",
      elapsed, vote_tally().c_str(), vote_total_, matched.c_str());
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
    prearmed_ = false;
    arrival_requested_ = false;
    blocked_ = false;
    // 분기로 들어갈 때만 prearm이 모으던 표를 이어받습니다(아래 kBranch). 그 밖의 스텝으로
    // 가면 창을 닫습니다 -- 여기서 안 닫으면 갈래를 고른 뒤에도 계속 세게 됩니다.
    if (step.type != StepType::kBranch) {
      vote_clear();
    }
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
          progress().c_str(), join_values(step.accepted).c_str(), params_.sign_topic.c_str(),
          step.debounce_frames, step.timeout_s);
        break;
      case StepType::kBranch:
        // 분기도 "서서 신호를 본다"는 점은 wait_signal과 같으므로 같은 phase를 씁니다.
        // 다른 것은 판정 결과입니다 -- 다음 스텝으로 가는 게 아니라 갈래를 고릅니다.
        phase_ = Phase::kWaiting;
        wait_until_ = now() + rclcpp::Duration::from_seconds(step.timeout_s);
        sign_streak_ = 0;
        // prearm이 이미 표를 모으고 있었다면 그대로 이어받습니다. 그 표들은 바로 이 분기의
        // 표지를 본 것이고, 여기서 버리면 vote_window_s를 처음부터 다시 채우느라 timeout_s
        // 안에 못 채울 수 있습니다(그러면 무조건 default입니다).
        //
        // 다만 한 장도 못 봤으면 이어받을 것이 없으므로 창을 새로 엽니다. 안 그러면 prearm
        // 내내 열려만 있던 빈 창을 그대로 물려받아 "경과 >= vote_window_s"가 이미 참인 채로
        // 시작하고, 그러면 여기서부터는 사실상 창이 없는 것과 같습니다. 버리는 표가 없으니
        // 손해도 없고, timeout_s > vote_window_s는 로드 시점에 보장됩니다.
        if (step.select == BranchSelect::kClearance) {
          // 콘 판정은 prearm이 없으므로 이어받을 창이 없습니다. 늘 여기서 새로 엽니다.
          clearance_open_now();
        } else if (!vote_open_ || vote_total_ == 0) {
          vote_open_now();
        }
        switch (step.select) {
          case BranchSelect::kPosition:
            RCLCPP_INFO(
              get_logger(), "%s branching on position (%s; timeout %.0f s -> '%s').",
              progress().c_str(), branch_values(step).c_str(), step.timeout_s,
              step.default_route.c_str());
            break;
          case BranchSelect::kClearance:
            RCLCPP_INFO(
              get_logger(),
              "%s branching on costmap clearance (%s; watching %.1f s, timeout %.0f s -> '%s').",
              progress().c_str(), clearance_points(step).c_str(), step.vote_window_s,
              step.timeout_s, step.default_route.c_str());
            break;
          case BranchSelect::kSign:
            RCLCPP_INFO(
              get_logger(), "%s branching on %s (%s; vote over %.1f s, timeout %.0f s -> '%s').",
              progress().c_str(), params_.sign_topic.c_str(), branch_values(step).c_str(),
              step.vote_window_s, step.timeout_s, step.default_route.c_str());
            break;
        }
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

  // "미션이 지금 굴러가고 있는가". 일시정지 중에도 true입니다 -- phase_는 "스텝의 어디쯤"을
  // 뜻하고 멈춤 여부는 paused_가 따로 들고 있으므로, 여기서 paused_를 보면 안 됩니다.
  bool is_running() const
  {
    return phase_ == Phase::kDriving || phase_ == Phase::kStarting ||
           phase_ == Phase::kHolding || phase_ == Phase::kWaiting || phase_ == Phase::kBlocked;
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

    // 일시정지: 아무 것도 진행시키지 않습니다. 골도 안 보내고, 기한도 안 보고, 갈래도 고르지
    // 않습니다. 차는 골이 없으니 워치독이 세우고 있습니다(enter_blocked 위 주석과 같은 방식).
    //
    // update_speed_limit()이 아니라 여기서 직접 0.0(제한 없음)을 내보내는 이유: 취소 결과가
    // 아직 안 왔으면 phase_는 여전히 kDriving이고, 그때 progress_가 유효하지 않으면
    // update_speed_limit은 "마지막 값을 그대로 둔다"며 감속 제한을 붙잡고 있습니다.
    // SpeedLimitPublisher가 같은 값은 걸러 주므로 매 tick 보내도 메시지는 한 번뿐입니다.
    if (paused_) {
      speed_limit_->publish(0.0, now());
      return;
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
        // handoff도 같은 이유로 앞에 둡니다 -- 어느 쪽이든 갈아끼우면 이 스텝은 이미
        // 끝난 것이라 도착 판정을 볼 이유가 없습니다.
        if (!try_preempt() && !try_handoff()) {
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
    if (step.select == BranchSelect::kPosition) {
      if (pick_branch_by_position(step, target)) {
        RCLCPP_INFO(
          get_logger(), "%s branch: taking route '%s' (nearest start point).",
          progress().c_str(), route_name(step, target).c_str());
        goto_step(target);
        return;
      }
      // tf를 아직 못 읽었습니다. 아래 timeout이 default로 받아 줍니다.
    } else if (step.select == BranchSelect::kClearance) {
      if (pick_branch_by_clearance(step, target)) {
        RCLCPP_INFO(
          get_logger(), "%s branch: taking route '%s' (the only clear one; %s).",
          progress().c_str(), route_name(step, target).c_str(), clearance_tally(step).c_str());
        goto_step(target);
        return;
      }
      // 코스트맵이 없거나, 창이 안 찼거나, 애매합니다. timeout이 default로 받아 줍니다.
    } else if (pick_branch(step, target, matched)) {
      RCLCPP_INFO(
        get_logger(), "%s branch: '%s' confirmed; taking route '%s'.",
        progress().c_str(), matched.c_str(), route_name(step, target).c_str());
      goto_step(target);
      return;
    }
    if (now() >= wait_until_) {
      if (step.select == BranchSelect::kPosition) {
        RCLCPP_WARN(
          get_logger(),
          "%s branch: could not read the vehicle pose within %.0f s. Taking the default route "
          "'%s' -- check that odometry is running.",
          progress().c_str(), step.timeout_s, step.default_route.c_str());
      } else if (step.select == BranchSelect::kClearance) {
        // 셀 수를 통째로 찍습니다. 여기서 갈리는 실패가 셋인데 한 줄로는 구별이 안 됩니다:
        // "no costmap"이면 토픽 이름이 틀렸거나 controller_server가 안 떠 있는 것이고,
        // 전부 0이면 라이다가 콘을 못 봤거나(또는 콘이 없거나) 좌표가 틀린 것이며,
        // 둘 다 큰 값이면 콘이 둘 다 서 있거나 반지름이 너무 커서 서로를 먹은 것입니다.
        RCLCPP_WARN(
          get_logger(),
          "%s branch: costmap did not single out one clear route within %.0f s (%s; need exactly "
          "one below %ld cells). Taking the default route '%s'.",
          progress().c_str(), step.timeout_s, clearance_tally(step).c_str(),
          static_cast<long>(params_.cone_min_cells), step.default_route.c_str());
      } else {
        // 표 현황을 통째로 찍습니다. 여기서 갈리는 실패가 셋인데 last_sign_ 하나로는
        // 구별이 안 됩니다: "nothing / 0 sample(s)"이면 검출기가 죽었거나 토픽 이름이
        // 틀린 것이고, "none 40"이면 갈림길 로직이 두 칸을 못 잡은 것이며,
        // "allow_left 19, allow_right 19"면 진짜 동점입니다.
        RCLCPP_WARN(
          get_logger(),
          "%s branch: no lane sign won the vote within %.0f s (votes over %.1f s: %s; "
          "%d sample(s)). Taking the default route '%s' -- check the sign detector.",
          progress().c_str(), step.timeout_s, (now() - vote_started_).seconds(),
          vote_tally().c_str(), vote_total_, step.default_route.c_str());
      }
      goto_step(step.default_target);
    }
  }

  // 위치로 갈래를 고릅니다: 갈래마다 그 첫 drive 스텝의 시작 웨이포인트까지의 거리를 재고
  // 가장 가까운 쪽입니다. 재는 점은 check_branch_seams가 이음매를 재는 그 점과 같습니다
  // -- 즉 "차가 지금 어느 갈래의 출발선에 서 있는가"입니다.
  //
  // tf를 못 읽으면 false. 그때 호출자는 아무 것도 하지 않고, timeout이 default로 받습니다
  // (표지를 못 본 분기와 같은 실패 방식 -- 찍지 않고 정해진 곳으로).
  bool pick_branch_by_position(const Step & branch, std::size_t & target)
  {
    const std::vector<std::size_t> targets = hyper_planner::branch_targets(branch);
    geometry_msgs::msg::PoseStamped robot;
    if (targets.empty() ||
      !lookup_robot_pose(courses_.front().waypoints.frame_id, robot, params_.tf_timeout_sec))
    {
      return false;
    }

    double best = std::numeric_limits<double>::max();
    std::string report;
    for (const std::size_t candidate : targets) {
      if (candidate >= steps_.size() || steps_[candidate].type != StepType::kDrive) {
        continue;
      }
      const Step & first = steps_[candidate];
      const hyper_planner::Course & course = course_of(first);
      if (first.begin_index >= course.waypoints.points.size()) {
        continue;
      }
      const auto & start = course.waypoints.points[first.begin_index];
      const double distance = std::hypot(
        start.x - robot.pose.position.x, start.y - robot.pose.position.y);
      if (!report.empty()) {
        report += ", ";
      }
      report += route_name(branch, candidate) + " " + std::to_string(distance).substr(0, 5) + " m";
      if (distance < best) {
        best = distance;
        target = candidate;
      }
    }
    if (best == std::numeric_limits<double>::max()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s branch selects by position, but no route starts with a usable drive step.",
        progress().c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s branch by position: %s.", progress().c_str(), report.c_str());
    return true;
  }

  // -------------------------------------------------------- 콘 자리로 갈래 고르기
  //
  // 주차 칸 둘 중 하나의 입구에 콘이 서 있고, 두 콘 자리는 미리 알고 있습니다. 그래서 이
  // 판정은 인식 문제가 아니라 "A 주변과 B 주변에 lethal 코스트맵이 얼마나 있는가"라는
  // 국소적인 질문입니다 -- 라이다가 이미 콘을 코스트맵에 찍어 두었습니다.
  //
  // 규칙: 갈래마다 콘 자리 반지름 안의 lethal 셀을 세고, **정확히 한 갈래만** 비어 있으면
  // 그리로 갑니다. 둘 다 비었거나, 둘 다 막혔거나, 코스트맵이 없으면 아무 일도 하지 않고
  // timeout이 default로 받습니다 -- 표지를 못 본 분기와 같은 실패 방식입니다(애매할 때
  // 찍지 않는 쪽).

  // 모은 셀 수를 버리고 창을 닫습니다. 다음 pick_branch_by_clearance가 새로 엽니다
  // -- vote_clear()와 같은 구조입니다.
  void clearance_clear()
  {
    clearance_cells_.clear();
    clearance_clipped_.clear();
    clearance_open_ = false;
  }

  void clearance_open_now()
  {
    clearance_clear();
    clearance_started_ = now();
    clearance_open_ = true;
  }

  // 이 분기가 보고 있는 자리를 "t_left (-2.40, 22.58), t_right (-4.67, 23.35)"로.
  std::string clearance_points(const Step & branch) const
  {
    std::string text;
    for (const BranchCase & branch_case : branch.cases) {
      if (!text.empty()) {
        text += ", ";
      }
      char buffer[64];
      std::snprintf(
        buffer, sizeof(buffer), " (%.2f, %.2f)", branch_case.cone_x, branch_case.cone_y);
      text += branch_case.route + buffer;
    }
    return text.empty() ? std::string("nothing") : text;
  }

  // 창 동안 모은 셀 수를 "t_left 31 cells, t_right 2 cells"로. 코스트맵을 한 번도 못 봤으면
  // 그렇게 말합니다 -- "전부 0"과 "아예 없음"은 전혀 다른 고장입니다.
  std::string clearance_tally(const Step & branch) const
  {
    if (clearance_cells_.empty()) {
      return costmap_ ? std::string("no cells counted yet") : std::string("no costmap received");
    }
    std::string text;
    for (const BranchCase & branch_case : branch.cases) {
      const auto found = clearance_cells_.find(branch_case.target);
      if (!text.empty()) {
        text += ", ";
      }
      text += branch_case.route + " " +
        std::to_string(found == clearance_cells_.end() ? 0 : found->second) + " cells";
      if (clearance_clipped_.count(branch_case.target) != 0) {
        text += " (outside the window)";
      }
    }
    return text;
  }

  // 최신 코스트맵이 쓸 만한가. 오래된 것을 그대로 쓰면 이미 치운 콘이 계속 보입니다.
  bool costmap_fresh() const
  {
    return costmap_ != nullptr &&
           (now() - costmap_stamp_).seconds() <= params_.costmap_timeout_sec;
  }

  // 코스 프레임(보통 map)의 점을 지금 들고 있는 코스트맵의 프레임(rolling이면 odom)으로.
  bool to_costmap_frame(
    double x, double y, const std::string & source_frame,
    geometry_msgs::msg::PointStamped & out)
  {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = source_frame;
    in.point.x = x;
    in.point.y = y;
    try {
      // stamp를 비워 두면 "가장 최근"입니다. 코스트맵이 rolling이라 원점이 매 주기
      // 달라지므로, 프레임도 원점도 지금 처리 중인 메시지의 것을 써야 합니다.
      tf_buffer_->transform(
        in, out, costmap_->header.frame_id, tf2::durationFromSec(params_.tf_timeout_sec));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No '%s' -> '%s' transform for the cone position: %s.",
        source_frame.c_str(), costmap_->header.frame_id.c_str(), ex.what());
      return false;
    }
  }

  // 한 자리의 lethal 셀 수를 셉니다.
  //
  // 잘림(clipped)을 호출자에게 그대로 넘기는 것이 중요합니다. 코스트맵 창 밖은 0개로
  // 보이는데 0은 "비어 있음"으로 읽히므로, 잘린 자리를 그냥 세면 "안 보인다"가 "깨끗하다"가
  // 됩니다 -- 콘이 서 있는 칸으로 들어갈 수 있는 유일한 경로입니다.
  bool count_cone_cells(
    double x, double y, double radius_m, const std::string & source_frame,
    hyper_planner::ClearanceCount & count)
  {
    geometry_msgs::msg::PointStamped point;
    if (!to_costmap_frame(x, y, source_frame, point)) {
      return false;
    }
    count = hyper_planner::count_lethal_near(
      *costmap_, point.point.x, point.point.y, radius_m,
      static_cast<std::uint8_t>(params_.lethal_cost));
    if (count.clipped) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "(%.2f, %.2f) is partly outside the %ux%u cell costmap window, so it can only look "
        "clear -- not treating it as clear. Is the vehicle close enough to it?",
        x, y, costmap_->metadata.size_x, costmap_->metadata.size_y);
    }
    return true;
  }

  bool pick_branch_by_clearance(const Step & branch, std::size_t & target)
  {
    if (!clearance_open_) {
      // begin_step이 정상 경로지만, 그 길을 안 거치고 들어와도(resume 직후 등) 스스로
      // 낫게 합니다. pick_branch가 투표 창에 하는 것과 같습니다.
      clearance_open_now();
      return false;
    }
    if (!costmap_fresh()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s branch selects by clearance, but no costmap on '%s' in the last %.1f s. Is "
        "controller_server up?",
        progress().c_str(), params_.costmap_topic.c_str(), params_.costmap_timeout_sec);
      return false;
    }

    const std::string & frame = courses_.front().waypoints.frame_id;
    clearance_clipped_.clear();
    for (const BranchCase & branch_case : branch.cases) {
      const double radius = branch_case.cone_radius_m > 0.0
        ? branch_case.cone_radius_m : params_.cone_radius_m;
      hyper_planner::ClearanceCount count;
      if (!count_cone_cells(branch_case.cone_x, branch_case.cone_y, radius, frame, count)) {
        return false;
      }
      // 최대값만 남깁니다 -- 위 clearance_cells_ 주석 참고.
      auto & best = clearance_cells_[branch_case.target];
      best = std::max(best, count.lethal);
      // 잘림은 누적하지 않고 매번 새로 봅니다. 차가 움직이면 창도 같이 움직이므로,
      // 지금 잘렸는지가 지금의 판정에 쓸 값입니다.
      if (count.clipped) {
        clearance_clipped_.insert(branch_case.target);
      }
    }

    // 창이 다 차야 정합니다. 한 장으로 정하면 콘이 잠깐 안 잡힌 프레임이 그대로 길이 됩니다.
    if ((now() - clearance_started_).seconds() < branch.vote_window_s) {
      return false;
    }

    // "막히지 않은 갈래가 정확히 하나". 갈래가 둘일 때 이것은 "막힌 갈래가 정확히
    // 하나"와 같은 말이고, 셋 이상이면 이쪽이 맞는 일반화입니다 -- 하나만 막혔다고
    // 나머지 둘 중 어디로 갈지가 정해지지는 않으니까요.
    const auto min_cells = static_cast<std::size_t>(params_.cone_min_cells);
    std::size_t clear_count = 0;
    for (const BranchCase & branch_case : branch.cases) {
      // 창 밖으로 잘린 갈래는 "비어 있다"고 말할 수 없습니다. 세어 본 것이 그 자리의
      // 일부뿐이라 콘이 안 보이는 쪽에 서 있을 수 있습니다. 모르는 것은 고르지 않습니다.
      if (clearance_clipped_.count(branch_case.target) != 0) {
        continue;
      }
      if (clearance_cells_[branch_case.target] < min_cells) {
        ++clear_count;
        target = branch_case.target;
      }
    }
    if (clear_count != 1) {
      return false;
    }
    return true;
  }

  // ~/probe_costmap -- 아무 좌표나 넣고 코스트맵이 뭐라고 하는지 물어봅니다.
  //
  // 좌표를 요청 필드가 아니라 파라미터(probe_points)로 받는 것은 goto_step의 step_label,
  // teleport_service의 label과 같은 방식입니다. 덕분에 인자 없는 Trigger 하나로 끝나고,
  // 이것 하나 때문에 msgs 패키지를 새로 만들지 않아도 됩니다.
  //
  // 미션 상태는 건드리지 않습니다 -- 주행 중에 불러도 안전합니다.
  void handle_probe_costmap(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const std::vector<double> & points = params_.probe_points;
    if (points.size() < 2 || points.size() % 2 != 0) {
      response->success = false;
      response->message =
        "probe_points needs an even number of values (x1, y1, x2, y2, ...); got " +
        std::to_string(points.size()) + ". Try: ros2 param set " + std::string(get_name()) +
        " probe_points \"[-2.4, 22.6, -4.7, 23.3]\"";
      return;
    }
    if (!costmap_fresh()) {
      response->success = false;
      response->message = "no costmap on '" + params_.costmap_topic + "' in the last " +
        std::to_string(params_.costmap_timeout_sec) + " s.";
      return;
    }
    if (courses_.empty()) {
      response->success = false;
      response->message = "no mission loaded, so there is no frame to read the points in.";
      return;
    }

    const std::string & frame = courses_.front().waypoints.frame_id;
    std::string report;
    std::size_t fewest = std::numeric_limits<std::size_t>::max();
    std::string clearest;
    bool tied = false;
    for (std::size_t i = 0; i + 1 < points.size(); i += 2) {
      hyper_planner::ClearanceCount count;
      if (!count_cone_cells(points[i], points[i + 1], params_.cone_radius_m, frame, count)) {
        response->success = false;
        response->message = "could not transform '" + frame + "' -> '" +
          costmap_->header.frame_id + "'.";
        return;
      }
      char buffer[128];
      std::snprintf(
        buffer, sizeof(buffer), "%s(%.2f, %.2f): %zu cells%s",
        report.empty() ? "" : "   ", points[i], points[i + 1], count.lethal,
        count.clipped ? " (outside the window -- unknown, not clear)" : "");
      report += buffer;
      // 잘린 점은 "가장 비었다"의 후보가 아닙니다. 0으로 보이는 이유가 비어서가 아니라
      // 안 보여서일 수 있고, 이 서비스를 부르는 이유가 바로 그 구별이기 때문입니다.
      if (count.clipped) {
        continue;
      }
      if (count.lethal < fewest) {
        fewest = count.lethal;
        clearest = "(" + std::to_string(points[i]).substr(0, 6) + ", " +
          std::to_string(points[i + 1]).substr(0, 6) + ")";
        tied = false;
      } else if (count.lethal == fewest) {
        tied = true;
      }
    }

    char tail[192];
    const std::string verdict = clearest.empty()
      ? std::string("no point could be called clear")
      : (tied ? std::string("tied, nothing to choose") : clearest + " is clearest");
    std::snprintf(
      tail, sizeof(tail), "  ->  %s (r=%.2f m, cost >= %ld, frame '%s')", verdict.c_str(),
      params_.cone_radius_m, static_cast<long>(params_.lethal_cost),
      costmap_->header.frame_id.c_str());
    response->success = true;
    response->message = report + tail;
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
      // 골을 보낸 직후(async_send_goal ~ 이 콜백 사이)에 '~/pause'나 '~/cancel'이 들어오면
      // 그때는 취소할 핸들이 아직 없었습니다. 여기서 받은 핸들이 바로 그 골이므로, 멈추라는
      // 말을 이미 들은 상태라면 붙잡지 않고 곧바로 취소합니다 -- 안 그러면 GUI와 로그는
      // paused/canceled인데 nav2는 계속 차를 몰고 갑니다.
      if (paused_ || phase_ == Phase::kIdle || phase_ == Phase::kFinished ||
        phase_ == Phase::kFailed)
      {
        pause_requested_ = paused_;
        client_->async_cancel_goal(handle);
        return;
      }
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
      // 진입하는 순간 streak을 비우고 투표 창을 새로 엽니다. 멀리서 -- 표지가 아직 몇
      // 픽셀일 때 -- 우연히 쌓인 것이 그대로 통과/분기 판정으로 이어지지 않게 하려는
      // 것입니다. 분기든 신호등이든 가리지 않고 여는 이유는, wait_signal prearm은 표를
      // 읽지 않고 도착할 때 begin_step이 버리기 때문입니다 -- 여기서 종류를 따지면
      // "분기가 무엇인지" 아는 곳이 하나 더 늘 뿐입니다.
      sign_streak_ = 0;
      vote_open_now();
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
    vote_clear();
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
    // begin_step을 안 거치고 갈래로 바로 들어가므로 여기서 직접 창을 닫습니다.
    vote_clear();
    send_goal(route_step, path);
    return true;
  }

  // 컨트롤러를 바꾸려면 새 골을 보내는 수밖에 없는데, 보통의 스텝 전환은 골 판정을
  // 기다리므로 차가 라벨에서 한 번 섭니다. handoff는 그 전환을 prearm과 같은 preemption으로
  // 합니다 -- 골까지 handoff_m가 남으면 "지금 위치 -> 다음 drive 스텝의 끝"을 새 골로
  // 보내고, 옛 골은 취소하지 않고 갈아끼웁니다. 새 골에는 다음 스텝의 controller_id가
  // 실리므로 그 순간 컨트롤러가 바뀝니다(RPP <-> MPPI).
  //
  // 신호를 보지 않는다는 것만 빼면 try_preempt_for_signal과 같습니다. 조건이 맞는지는
  // 로드 시점에 link_handoff_steps가 이미 봤습니다.
  bool try_handoff()
  {
    const Step & step = steps_[step_index_];
    if (!step.handoff_enabled || !goal_handle_ || arrival_requested_ || !progress_.valid()) {
      return false;
    }
    if (progress_.distance_to_stop_m() > step.handoff_m) {
      return false;
    }

    const std::size_t merge_index = step.handoff_merge_step;
    const Step & merge_step = steps_[merge_index];

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
      // 다음 tick에 다시 시도합니다. 끝내 못 만들면 handoff가 그냥 안 일어나고, 원래 골
      // 그대로 라벨에 서서 다음 스텝이 자기 골을 보냅니다 -- 안전한 쪽으로 실패합니다.
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s handing off %.1f m before '%s': continuing to '%s' with controller '%s' "
      "(%zu-pose path preempts the running goal).",
      progress().c_str(), progress_.distance_to_stop_m(), step.label.c_str(),
      merge_step.label.c_str(), merge_step.controller_id.c_str(), path.poses.size());

    superseded_goal_id_ = goal_handle_->get_goal_id();
    has_superseded_goal_ = true;

    step_index_ = merge_index;
    retries_ = 0;
    prearmed_ = false;
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
    // 골이 어떻게 끝났든 "일시정지 때문에 건 취소"라는 표시는 여기서 소비됩니다. 아래
    // CANCELED 분기 안에서만 지우면, 취소가 abort와 엇갈렸을 때 표시가 남아 다음 골의
    // 결과를 잘못 해석할 수 있습니다.
    const bool canceled_for_pause = pause_requested_;
    pause_requested_ = false;
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
        } else if (canceled_for_pause) {
          // '~/pause'가 건 취소입니다. kIdle로 떨어지지 않는 것이 취소와 일시정지의 차이
          // 전부입니다 -- 스텝은 그대로 두고 kStarting으로 되돌려, '~/resume'이 같은 골을
          // 지금 위치에서 다시 보내게 합니다(경로는 trim_to_robot이 다시 자릅니다).
          phase_ = Phase::kStarting;
          starting_since_ = now();
          RCLCPP_INFO(
            get_logger(), "%s pause: goal canceled; '~/resume' re-sends it from here.",
            progress().c_str());
          publish_status(status_text());
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

    // 진입 경로는 차량 헤딩에서 출발하는 곡선이라, 후진 세그먼트에 붙이면 RPP가
    // 방향을 반대로 읽습니다. 전진 세그먼트에서만 씁니다.
    std::size_t lead_in = 0;
    if (!reverse) {
      lead_in = hyper_planner::insert_lead_in(
        path, robot.pose.position.x, robot.pose.position.y,
        hyper_planner::yaw_from_quaternion(robot.pose.orientation),
        params_.lead_in_spacing_m, params_.lead_in_tangent_gain);
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
    // 일시정지 중에는 세지 않습니다. 계속 세면 서 있는 동안 debounce와 표가 채워져,
    // '~/resume'을 부르는 순간 판정이 이미 끝나 있습니다 -- reset_to가 점프 전 연속 프레임을
    // 지우는 것과 같은 이유입니다.
    if (paused_) {
      return;
    }

    last_sign_ = value;

    // 분기의 투표. 창이 열려 있는 동안 들어온 값을 값별로 셉니다. 어느 case에도 없는
    // 값(none, red, 짝을 못 찾은 allow ...)도 셉니다 -- 이길 수는 없지만, 나중에 로그로
    // "카메라가 실제로 뭘 봤는지"를 보여 주는 것이 바로 그 값들이기 때문입니다.
    if (vote_open_) {
      ++vote_counts_[value];
      ++vote_total_;
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
    // paused_를 phase_보다 먼저 봅니다 -- 일시정지 중의 phase_는 kStarting일 수 있어서
    // 아래 검사만으로는 "이미 달리는 중"이라는 엉뚱한 사유가 나갑니다.
    if (paused_) {
      response->success = false;
      response->message = "Mission is paused at step " + std::to_string(step_index_) +
        "; call '~/resume' to continue (or '~/cancel' to give up the step).";
      return;
    }
    if (is_running()) {
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

  // 미션을 지금 스텝의 지금 자리에 그대로 세워 둡니다. '~/cancel'과 달리 kIdle로 떨어지지
  // 않으므로, '~/resume'이 스텝 안에서 이어 갑니다.
  //
  // 왜 phase_를 kPaused로 안 바꾸는가: phase_는 "이 스텝의 어디쯤인가"이고 그건 일시정지
  // 중에도 그대로 유효한 정보입니다. kPaused를 만들면 돌아갈 phase를 따로 들고 있어야 해서
  // 같은 걸 두 번 저장하게 됩니다. 그래서 멈춤 여부만 paused_로 따로 둡니다.
  void handle_pause(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (paused_) {
      response->success = false;
      response->message = "Already paused at step " + std::to_string(step_index_) + ".";
      return;
    }
    if (!is_running()) {
      response->success = false;
      response->message = "Nothing to pause; the mission is not running (" + status_text() + ").";
      return;
    }

    paused_ = true;
    paused_at_ = now();

    // 도착/건너뛰기 취소가 이미 날아가 있으면 건드리지 않습니다. 그 취소는 "이 스텝은
    // 끝났다"는 뜻이므로 제 뜻대로 끝내게 두고, advance()가 다음 스텝을 열어도 paused_가
    // 그 자리에서 다시 얼립니다. 반대로 여기서 취소를 하나 더 걸면 그 뜻을 잃습니다.
    if (goal_handle_ && !arrival_requested_ && !skip_requested_) {
      pause_requested_ = true;
      client_->async_cancel_goal(goal_handle_);
    }

    RCLCPP_WARN(
      get_logger(), "%s paused; call '%s/resume' to continue.", progress().c_str(), get_name());
    publish_status(status_text());
    response->success = true;
    response->message = "Paused at step " + std::to_string(step_index_) + ".";
  }

  // 일시정지를 풀고 스텝 안에서 이어 갑니다.
  //
  // 멈춰 있던 동안 시계도 멈춘 것으로 칩니다. 안 그러면 wait_signal 스텝에서 2분 쉬었다가
  // 재개하는 순간 timeout_s가 이미 지나 있어 곧바로 실패(또는 엉뚱한 default 갈래)로 갑니다.
  void handle_resume(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!paused_) {
      response->success = false;
      response->message = "Mission is not paused (" + status_text() + ").";
      return;
    }

    // 기한을 전부 멈춘 시간만큼 뒤로 밉니다. 멈춘 사이에 새로 잡힌 기한(취소 결과가 늦게
    // 와서 advance()된 경우)은 그만큼 더 밀려 필요보다 오래 기다리지만, 짧아지는 쪽이
    // 아니라 길어지는 쪽이라 안전합니다.
    const rclcpp::Duration paused_for = now() - paused_at_;
    hold_until_ = hold_until_ + paused_for;
    wait_until_ = wait_until_ + paused_for;
    blocked_since_ = blocked_since_ + paused_for;
    blocked_retry_at_ = blocked_retry_at_ + paused_for;

    // 이 셋은 밀지 않고 지금으로 되돌립니다 -- 멈춘 동안 흐른 시간을 "서버를 못 찾았다 /
    // 진행도가 끊겼다"로 오해하면 안 됩니다.
    starting_since_ = now();
    progress_ok_since_ = now();
    last_feedback_time_ = now();
    // 멈추기 전 피드백 속도입니다. cancel-on-arrival이 이 값을 믿으면 서 있는 차를
    // "아직 빠르다"고 볼 수 있으므로 버립니다.
    have_speed_ = false;
    last_speed_ = 0.0;
    // debounce와 표는 처음부터 다시 셉니다(on_sign이 멈춘 동안 세지 않았으므로 값은 멈출
    // 때의 것입니다 -- 그걸 그대로 이어받으면 재개 즉시 판정될 수 있습니다). 창을 뒤로
    // 미는 대신 버리는 이유도 같습니다: 밀어 두면 "3초짜리 창"이 실제로는 몇 분에 걸쳐
    // 가운데가 비어 있는 창이 됩니다. wait_until_은 멈춘 만큼 밀리고 로드 시점에
    // vote_window_s < timeout_s를 보장하므로, 새 창은 언제나 채울 시간이 남습니다.
    // 창을 여는 것은 다음 tick의 pick_branch가 알아서 합니다.
    sign_streak_ = 0;
    vote_clear();
    // 콘 판정도 같은 이유로 버립니다. 창을 밀어 두면 멈춰 있던 시간이 창 안에 들어와
    // "이미 다 찼다"가 되어, 재개 직후 코스트맵 한 장으로 길이 정해집니다. 게다가 멈춘
    // 사이에 누가 콘을 옮겼을 수 있는데 -- 멈추는 이유가 바로 그것일 때가 많습니다 --
    // 모아 둔 최대값은 옮기기 전의 주장입니다. 재개가 그것을 이어받을 이유가 없습니다.
    clearance_clear();

    paused_ = false;
    RCLCPP_INFO(
      get_logger(), "%s resumed after %.1f s.", progress().c_str(), paused_for.seconds());
    publish_status(status_text());
    response->success = true;
    response->message = "Resumed at step " + std::to_string(step_index_) + ".";
  }

  void handle_cancel(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    skip_requested_ = false;
    arrival_requested_ = false;
    // 취소는 일시정지보다 셉니다. 멈춰 있던 미션도 여기서 kIdle로 내려놓습니다.
    paused_ = false;
    pause_requested_ = false;
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
    // 멈춘 상태에서 건너뛰면 advance() -> begin_step()으로 곧바로 출발합니다. 조이스틱
    // 일시정지 중이라면 /estop이 아직 걸려 있어 사람은 차가 서 있는 걸 보고 있으므로,
    // 그렇게 몰래 출발시키지 않고 거절합니다.
    if (paused_) {
      response->success = false;
      response->message = "Mission is paused; call '~/resume' before skipping.";
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

  // 미션을 "index 스텝 앞에서 대기(kIdle)"로 되돌립니다. begin_step()은 부르지 않습니다.
  //
  // 이 구분이 '~/goto_step'의 전부입니다. 안에서 쓰는 goto_step(index)은 곧바로
  // begin_step()을 불러 차가 출발해 버리는데, 여기서 필요한 것은 "스텝만 골라 두고
  // 서 있기"입니다 -- 시뮬에서는 그 사이에 차를 라벨 위치로 순간이동시키고, 실차에서는
  // 사람이 차를 그 지점에 가져다 놓은 뒤에 '~/start'를 부릅니다.
  //
  // handle_restart와 handle_goto_step이 같은 상태를 빠짐없이 되돌리도록 한곳에 모았습니다.
  // 취소한 골의 결과가 뒤늦게 오더라도 on_result가 phase_ == kIdle에서 곧바로 돌아
  // 나가므로 안전합니다.
  void reset_to(std::size_t index)
  {
    skip_requested_ = false;
    arrival_requested_ = false;
    prearmed_ = false;
    blocked_ = false;
    // 스텝을 옮기는 것이므로 일시정지도 함께 풉니다. 어차피 kIdle로 내려놓으니
    // 차가 저절로 출발하지는 않습니다.
    paused_ = false;
    pause_requested_ = false;
    // 갈아끼운 옛 골의 표시도 지웁니다. 그 결과가 아직 날아오는 중일 수 있지만,
    // 위와 같은 이유로 kIdle에서 무시됩니다.
    has_superseded_goal_ = false;
    // 점프 전에 쌓인 연속 프레임과 표가 새 wait_signal/branch를 즉시 만족시키지 않도록.
    sign_streak_ = 0;
    vote_clear();
    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
      goal_handle_.reset();
    }
    step_index_ = index;
    retries_ = 0;
    phase_ = Phase::kIdle;
    publish_status("idle");
  }

  void handle_restart(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    reset_to(0);
    response->success = true;
    response->message = "Reset to step 0; call '~/start' to run.";
  }

  // 임의의 스텝 앞으로 점프합니다. 미션 후반의 스텝 하나를 고치고 확인하려고 코스를
  // 처음부터 돌 이유가 없기 때문입니다.
  //
  // 목적지는 요청 필드가 아니라 파라미터로 받습니다 -- teleport_service의 label,
  // model_service의 모델 이름과 같은 방식이라, hyper_rqt 패널이나 waypoint studio가
  // set_parameters로 값을 밀어 넣고 인자 없는 Trigger를 부르면 됩니다.
  void handle_goto_step(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (steps_.empty()) {
      response->success = false;
      response->message = "Mission was not loaded; see the node log.";
      return;
    }

    // 파라미터는 params_ 스냅샷이 아니라 여기서 직접 읽습니다. 스냅샷은 tick()이
    // param_listener_->is_old()를 볼 때만 갱신되므로, set_parameters 직후에 Trigger가
    // 들어오면 그 사이 tick이 없어 이전 값으로 점프할 수 있습니다 -- 조용히 엉뚱한
    // 스텝으로 가는 사고입니다.
    const std::string label = get_parameter("step_label").as_string();
    const int64_t requested = get_parameter("step_index").as_int();

    std::size_t target = 0;
    if (!label.empty()) {
      bool found = false;
      for (std::size_t i = 0; i < steps_.size(); ++i) {
        if (steps_[i].type == StepType::kDrive && steps_[i].label == label) {
          target = i;
          found = true;
          break;
        }
      }
      if (!found) {
        std::ostringstream out;
        out << "No drive step with until='" << label << "'. Available:";
        for (const Step & step : steps_) {
          if (step.type == StepType::kDrive && !step.label.empty()) {
            out << ' ' << step.label;
          }
        }
        response->success = false;
        response->message = out.str();
        return;
      }
    } else if (requested >= 0) {
      if (static_cast<std::size_t>(requested) >= steps_.size()) {
        response->success = false;
        response->message = "step_index " + std::to_string(requested) +
          " is out of range (0.." + std::to_string(steps_.size() - 1) + ").";
        return;
      }
      target = static_cast<std::size_t>(requested);
    } else {
      response->success = false;
      response->message =
        "Set 'step_label' or 'step_index' first "
        "(ros2 param set /mission_manager step_label <label>).";
      return;
    }

    const bool was_running = is_running();
    reset_to(target);
    if (was_running) {
      RCLCPP_INFO(
        get_logger(), "goto_step canceled the running goal and holds at step %zu.", target);
    }

    response->success = true;
    response->message = "[" + std::to_string(target + 1) + "/" +
      std::to_string(steps_.size()) + "] " + status_text_for(target) +
      " -- call '~/start' to run.";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
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
    return progress() + " " + status_text_for(step_index_);
  }

  // '~/goto_step'이 아직 옮기지 않은 스텝을 응답에 적을 수 있도록 인덱스를 받습니다.
  std::string status_text_for(std::size_t index) const
  {
    if (index >= steps_.size()) {
      return "finished";
    }
    const Step & step = steps_[index];
    std::string text = std::string(type_name(step.type));
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

  // 접두사를 여기서 붙이는 이유: status를 내보내는 곳은 여기만이 아니라 begin_step,
  // enter_blocked, reset_to도 각자 부릅니다. 한곳에서 붙여야 멈춘 사이에 스텝이 넘어가도
  // (늦게 온 도착 결과) GUI에서 "paused"가 사라지지 않습니다.
  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = paused_ ? "paused " + text : text;
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
  // 일시정지('~/pause'). phase_와 나란한 별개의 상태입니다 -- phase_는 "스텝의 어디쯤"을
  // 그대로 들고 있고, paused_는 "지금은 아무 것도 진행시키지 않는다"만 뜻합니다.
  // pause_requested_는 일시정지 때문에 건 취소인지(true) 사람이 부른 '~/cancel'인지
  // on_result에서 가리는 표시입니다(arrival_requested_/skip_requested_와 같은 방식).
  bool paused_{false};
  bool pause_requested_{false};
  rclcpp::Time paused_at_;
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
  // 분기의 투표. 창이 열려 있는 동안 /perception/sign이 낸 값을 값별로 셉니다.
  // 왜 연속 프레임이 아닌가: 갈림길 표지는 세 장이 나란히 깜빡이므로 "같은 값이 N프레임
  // 연속"이 거의 성립하지 않고, 그때마다 판정이 default로 떨어집니다. 대신 일정
  // 시간(vote_window_s) 동안 모아 다수결로 정합니다.
  // std::map인 이유는 로그 줄의 값 순서가 실행마다 같아야 하기 때문입니다.
  std::map<std::string, int> vote_counts_;
  int vote_total_{0};
  rclcpp::Time vote_started_;
  bool vote_open_{false};

  // 최신 로컬 코스트맵 한 장. 콜백이 넣기만 하고, 세는 일은 clearance 분기와
  // ~/probe_costmap에서만 합니다.
  nav2_msgs::msg::Costmap::SharedPtr costmap_;
  rclcpp::Time costmap_stamp_;

  // clearance 분기가 창 동안 모은 갈래별 **최대** lethal 셀 수(키는 target 스텝 인덱스).
  // 최대인 이유: 한 장에서 못 본 콘은 다음 장에서 보이고(라이다 각도, 다른 콘의 그림자),
  // "한 번이라도 확실히 보였다"가 "지금 안 보인다"보다 강한 증거이기 때문입니다.
  std::map<std::size_t, std::size_t> clearance_cells_;
  // 이번 판정에서 반지름이 코스트맵 창 밖으로 잘린 갈래들. 잘린 갈래는 셀이 0으로 보여도
  // "비어 있음"으로 치지 않습니다(count_cone_cells 참고).
  std::set<std::size_t> clearance_clipped_;
  rclcpp::Time clearance_started_;
  bool clearance_open_{false};
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr steps_pub_;
  std::unique_ptr<hyper_planner::SpeedLimitPublisher> speed_limit_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sign_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr skip_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goto_step_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr probe_costmap_srv_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
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
