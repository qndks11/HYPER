#pragma once

// mission.yaml의 스텝 큐를 읽어 실행 가능한 스텝 목록으로 만드는 코드.
// mission_manager_node가 시작할 때 한 번 씁니다 -- 여기서 하는 일은 전부 로드 시점의
// 정적 검증이고, 주행 중에는 이 파일의 코드가 돌지 않습니다.
//
// 노드에서 떼어 낸 이유는 두 가지입니다.
//   1. 미션 파일이 틀렸을 때의 진단(라벨 스냅, 단조 증가, prearm 패턴)이 주행 상태 기계와
//      섞여 있을 이유가 없습니다.
//   2. Step 구조체와 그 불변식(prearm 링크, decel 꼬리)이 한곳에 모여 읽힙니다.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

#include "hyper_planner/common.hpp"
#include "hyper_planner/path_loader.hpp"

namespace hyper_planner
{

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
  // 로드 시점에 이어 둔 prearm 링크(아래 MissionLoader::link_prearm_steps 참고).
  // prearm_enabled면 이 drive 스텝을 달리는 동안 steps[prearm_wait_step]의 신호를 미리 보고,
  // 확인되면 steps[prearm_merge_step]의 끝까지 골을 이어 보냅니다.
  bool prearm_enabled{false};
  std::size_t prearm_wait_step{0};
  std::size_t prearm_merge_step{0};
  // 0보다 크면 이 스텝을 등감속(m/s^2)으로 세웁니다. mission_manager_node.cpp 머리의
  // "decel 프로파일" 참고.
  double decel_profile_a{0.0};
  // 0보다 크면 이 스텝에서의 컨트롤러 abort를 "앞이 막혔다"로 해석합니다. 미션을 실패로
  // 끝내는 대신 그 자리에 세우고(골을 안 보내면 워치독이 세웁니다) 주기적으로 같은 골을
  // 다시 보내, 장애물이 치워지면 스스로 이어서 갑니다. 이 시간(초) 동안 계속 막혀 있으면
  // 그때는 실패로 끝냅니다. 0 = 끔(그때는 goal_retry_limit이 소진되면 바로 실패).
  double obstacle_hold_s{0.0};
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

inline const char * type_name(StepType type)
{
  switch (type) {
    case StepType::kDrive: return "drive";
    case StepType::kStop: return "stop";
    case StepType::kWaitSignal: return "wait_signal";
  }
  return "?";
}

// 로그용. {"green", "left"} -> "green|left".
inline std::string join_values(const std::vector<std::string> & values)
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

// "green, left" -> {"green", "left"}.
inline std::vector<std::string> split_values(const std::string & text)
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

// 로더가 파라미터 전체(mission_manager::Params) 대신 받는 값들. 로드에 실제로 쓰이는
// 것만 추려 두었으므로, 여기 없는 파라미터는 주행 중에만 쓰인다는 뜻입니다.
struct MissionLoadConfig
{
  std::string waypoint_csv;
  double min_spacing_m{0.0};
  std::string frame_id;             // CSV에 frame_id가 없을 때의 기본값
  std::string mission_yaml;

  // mission.yaml의 스텝이 값을 적지 않았을 때 쓰는 기본값.
  std::string controller_id;
  std::string goal_checker_id;
  double cancel_on_arrival_m{0.0};
  double decel_profile_a{0.0};

  double decel_profile_lookahead_m{0.0};
  std::string sign_topic;           // prearm 로그 문구에만 씁니다
};

// CSV + mission.yaml -> 스텝 목록. load()가 false를 반환하면 사유는 이미 로그에 있습니다.
class MissionLoader
{
public:
  MissionLoader(rclcpp::Logger logger, MissionLoadConfig config)
  : logger_(std::move(logger)), config_(std::move(config))
  {
  }

  bool load()
  {
    return load_waypoints() && load_mission_file();
  }

  // load()가 성공한 뒤에만 의미가 있습니다. 호출자가 std::move로 가져가도 됩니다.
  WaypointFile & waypoints() {return waypoints_;}
  std::unordered_map<std::string, std::size_t> & labels() {return label_index_;}
  std::vector<Step> & steps() {return steps_;}

private:
  bool load_waypoints()
  {
    std::string error;
    if (!load_waypoint_csv(config_.waypoint_csv, config_.min_spacing_m, waypoints_, error)) {
      RCLCPP_ERROR(logger_, "%s", error.c_str());
      return false;
    }
    if (waypoints_.frame_id.empty()) {
      waypoints_.frame_id = config_.frame_id;
    }
    RCLCPP_INFO(
      logger_, "Loaded %zu waypoints from '%s' in frame '%s' (%zu rows skipped).",
      waypoints_.points.size(), config_.waypoint_csv.c_str(), waypoints_.frame_id.c_str(),
      waypoints_.skipped_rows);
    return true;
  }

  bool load_mission_file()
  {
    if (config_.mission_yaml.empty()) {
      RCLCPP_ERROR(logger_, "Parameter 'mission_yaml' is empty.");
      return false;
    }

    YAML::Node root;
    try {
      root = YAML::LoadFile(config_.mission_yaml);
    } catch (const YAML::Exception & ex) {
      RCLCPP_ERROR(
        logger_, "Failed to parse '%s': %s", config_.mission_yaml.c_str(), ex.what());
      return false;
    }

    const double snap_tolerance_m = root["label_snap_tolerance_m"]
      ? root["label_snap_tolerance_m"].as<double>() : 1.0;

    if (!root["labels"] || !root["labels"].IsMap() || root["labels"].size() == 0) {
      RCLCPP_ERROR(
        logger_,
        "'%s' has no 'labels'. Place them first:\n"
        "  python3 src/planning/hyper_waypoint/scripts/label_waypoints.py %s",
        config_.mission_yaml.c_str(), config_.waypoint_csv.c_str());
      return false;
    }
    if (!root["steps"] || !root["steps"].IsSequence() || root["steps"].size() == 0) {
      RCLCPP_ERROR(logger_, "'%s' has no 'steps'.", config_.mission_yaml.c_str());
      return false;
    }

    if (!snap_labels(root["labels"], snap_tolerance_m) || !parse_steps(root["steps"])) {
      return false;
    }

    resolve_decel_tails();
    link_prearm_steps();

    RCLCPP_INFO(
      logger_, "Mission '%s' loaded: %zu steps, %zu labels.",
      config_.mission_yaml.c_str(), steps_.size(), label_index_.size());
    return true;
  }

  // 라벨을 최근접 웨이포인트로 스냅합니다. 라벨은 좌표로 저장되어 있어 코스를 다시
  // 녹화해도 살아남지만, 그만큼 엉뚱한 데 붙을 수도 있으므로 여기서 걸러냅니다.
  bool snap_labels(const YAML::Node & labels, double snap_tolerance_m)
  {
    for (const auto & entry : labels) {
      const auto name = entry.first.as<std::string>();
      const YAML::Node & point = entry.second;

      // `이름: last` -- 좌표 대신 "CSV의 마지막 웨이포인트"를 가리키는 센티널입니다.
      // 코스 끝은 좌표로 적는 순간 그 CSV 전용이 되어버립니다(simple.yaml의 course_end가
      // sim.csv 좌표라 real.csv로는 스냅 허용치를 넘겨 로드가 거부되던 문제). "끝까지
      // 간다"는 뜻은 어느 코스에서나 같으므로, 좌표가 아니라 의도를 적게 합니다.
      // 이벤트 지점(정지선/신호등/주차)은 여전히 좌표여야 합니다 -- 코스 중간의 특정
      // 물리적 위치라 label_waypoints.py가 찍어 주는 x/y가 곧 정의입니다.
      if (point.IsScalar() && point.as<std::string>() == "last") {
        if (waypoints_.points.empty()) {
          RCLCPP_ERROR(
            logger_, "Label '%s' is 'last', but '%s' has no waypoints.",
            name.c_str(), config_.waypoint_csv.c_str());
          return false;
        }
        label_index_[name] = waypoints_.points.size() - 1;
        RCLCPP_INFO(
          logger_, "Label '%s' -> last waypoint #%zu (좌표 스냅 생략).",
          name.c_str(), waypoints_.points.size() - 1);
        continue;
      }

      if (!point["x"] || !point["y"]) {
        RCLCPP_ERROR(
          logger_, "Label '%s' has neither x/y nor the 'last' sentinel.", name.c_str());
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
          logger_,
          "Label '%s' (%.3f, %.3f) is %.2f m from the nearest waypoint (#%zu), beyond "
          "label_snap_tolerance_m (%.2f). Re-place it with label_waypoints.py, or check that "
          "mission.yaml and '%s' describe the same course.",
          name.c_str(), x, y, best, nearest, snap_tolerance_m, config_.waypoint_csv.c_str());
        return false;
      }
      label_index_[name] = nearest;
      RCLCPP_INFO(
        logger_, "Label '%s' -> waypoint #%zu (%.2f m away).", name.c_str(), nearest, best);
    }
    return true;
  }

  // 스텝을 읽으면서 drive 세그먼트를 잇습니다. 세그먼트 시작점은 직전 drive 스텝의
  // 도착점이고, 첫 세그먼트만 CSV 처음부터 시작합니다.
  bool parse_steps(const YAML::Node & nodes)
  {
    std::size_t cursor = 0;
    bool seen_drive = false;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const YAML::Node & node = nodes[i];
      const auto type_text = node["type"] ? node["type"].as<std::string>() : std::string();

      Step step;
      if (type_text == "drive") {
        if (!parse_drive_step(node, i, cursor, seen_drive, step)) {
          return false;
        }
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
          RCLCPP_ERROR(logger_, "Step %zu (wait_signal) has an empty 'value'.", i);
          return false;
        }
      } else {
        RCLCPP_ERROR(
          logger_, "Step %zu has unknown type '%s' (expected drive/stop/wait_signal).",
          i, type_text.c_str());
        return false;
      }
      steps_.push_back(step);
    }
    return true;
  }

  bool parse_drive_step(
    const YAML::Node & node, std::size_t i, std::size_t & cursor, bool & seen_drive, Step & step)
  {
    step.type = StepType::kDrive;
    if (!node["until"]) {
      RCLCPP_ERROR(logger_, "Step %zu (drive) has no 'until'.", i);
      return false;
    }
    step.label = node["until"].as<std::string>();
    const auto found = label_index_.find(step.label);
    if (found == label_index_.end()) {
      RCLCPP_ERROR(
        logger_, "Step %zu references label '%s', which is not in 'labels'.",
        i, step.label.c_str());
      return false;
    }
    const std::size_t target = found->second;
    // 코스는 한 번 주행해 녹화한 것이므로 라벨은 CSV를 따라 단조 증가해야 합니다.
    // 그렇지 않으면 세그먼트가 비거나 거꾸로 뒤집힙니다.
    if (seen_drive && target <= cursor) {
      RCLCPP_ERROR(
        logger_,
        "Step %zu: label '%s' is at waypoint #%zu, which is not past the previous step's "
        "#%zu. Labels must advance along the recorded course; re-place '%s'.",
        i, step.label.c_str(), target, cursor, step.label.c_str());
      return false;
    }
    step.begin_index = seen_drive ? cursor : 0;
    step.end_index = target;
    step.reverse = node["reverse"] ? node["reverse"].as<bool>() : false;
    step.controller_id = node["controller"]
      ? node["controller"].as<std::string>() : config_.controller_id;
    step.goal_checker_id = node["goal_checker"]
      ? node["goal_checker"].as<std::string>() : config_.goal_checker_id;
    step.cancel_on_arrival_m = node["cancel_on_arrival_m"]
      ? node["cancel_on_arrival_m"].as<double>() : config_.cancel_on_arrival_m;
    step.decel_profile_a = node["decel_profile_a"]
      ? node["decel_profile_a"].as<double>() : config_.decel_profile_a;
    step.obstacle_hold_s = node["obstacle_hold_s"]
      ? node["obstacle_hold_s"].as<double>() : 0.0;

    // reverse 플래그가 녹화된 실제 주행 방향과 맞는지 확인합니다. 틀리면 RPP가
    // 엉뚱한 방향으로 당기므로 바로 알아채는 편이 낫습니다.
    const double opposing = reverse_fraction(waypoints_.points, step.begin_index, step.end_index);
    if (step.reverse && opposing < 0.5) {
      RCLCPP_WARN(
        logger_,
        "Step %zu (until '%s') is marked reverse, but only %.0f%% of the recorded segment "
        "has the body heading opposing travel. Was this stretch actually recorded driving "
        "backwards?", i, step.label.c_str(), 100.0 * opposing);
    } else if (!step.reverse && opposing > 0.5) {
      RCLCPP_WARN(
        logger_,
        "Step %zu (until '%s') is NOT marked reverse, but %.0f%% of the recorded segment "
        "has the body heading opposing travel. Add 'reverse: true' and a reversing "
        "controller, or MPPI will try to drive it nose-first.",
        i, step.label.c_str(), 100.0 * opposing);
    }

    cursor = target;
    seen_drive = true;
    return true;
  }

  // decel 프로파일을 쓰는 drive 스텝에, 라벨 뒤로 덧붙일 직선 꼬리의 길이를 정해 둡니다.
  //
  // MPPI의 감속은 "경로의 마지막 점"에서 나오므로(mission_manager_node.cpp 머리 주석),
  // 그 점을 local costmap 밖으로 밀어내면 감속 항이 아예 켜지지 않습니다. 감속은 대신
  // /speed_limit이 맡습니다.
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
          logger_,
          "Step %zu (until '%s') is a reverse segment; decel_profile_a is ignored there. "
          "Reverse parking runs on RPP and has to reach the goal checker.",
          i, step.label.c_str());
        step.decel_profile_a = 0.0;
        continue;
      }

      step.tail_after_label_m = config_.decel_profile_lookahead_m;
      RCLCPP_INFO(
        logger_,
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
          logger_,
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
        logger_,
        "Prearm: while driving to '%s', watch %s for '%s' from %.1f m out; if confirmed, roll "
        "straight through to '%s' without stopping.",
        steps_[i].label.c_str(), config_.sign_topic.c_str(),
        join_values(wait_step.accepted).c_str(), steps_[i].prearm_distance_m,
        steps_[i + 2].label.c_str());
    }
  }

  rclcpp::Logger logger_;
  MissionLoadConfig config_;

  WaypointFile waypoints_;
  std::unordered_map<std::string, std::size_t> label_index_;
  std::vector<Step> steps_;
};

}  // namespace hyper_planner
