#pragma once

// mission.yaml의 스텝 큐를 읽어 실행 가능한 스텝 목록으로 만드는 코드.
// mission_manager_node가 시작할 때 한 번 씁니다 -- 여기서 하는 일은 전부 로드 시점의
// 정적 검증이고, 주행 중에는 이 파일의 코드가 돌지 않습니다.
//
// 노드에서 떼어 낸 이유는 두 가지입니다.
//   1. 미션 파일이 틀렸을 때의 진단(라벨 스냅, 단조 증가, prearm 패턴)이 주행 상태 기계와
//      섞여 있을 이유가 없습니다.
//   2. Step 구조체와 그 불변식(prearm 링크, decel 꼬리)이 한곳에 모여 읽힙니다.
//
// 코스와 분기
// -----------
// 웨이포인트 CSV 하나는 "한 번 주행해 녹화한 것"이라 갈림길을 표현할 수 없습니다 --
// 라벨이 CSV를 따라 단조 증가해야 하기 때문입니다. 그래서 갈래마다 CSV를 따로 녹화하고
// (Course), branch 스텝이 신호를 보고 그중 하나(routes의 한 갈래)를 고릅니다.
//
// courses:를 안 쓰면 코스는 waypoint_csv 파라미터가 가리키는 "main" 하나뿐이고, 미션
// 파일의 동작은 분기 기능이 없던 때와 완전히 같습니다(mission.yaml/simple.yaml 그대로).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
  kBranch,
};

// "이 스텝이 끝나면 미션도 끝"을 나타내는 Step::next_index 값.
inline constexpr std::size_t kEndOfMission = static_cast<std::size_t>(-1);

// 미션이 쓰는 코스 하나 = 웨이포인트 CSV 하나 + 그 CSV에 스냅된 라벨들.
//
// 라벨 이름은 코스마다 독립입니다. 같은 이름을 두 코스에 둬도 되고, 한 코스에 있는
// 라벨이 다른 코스에 없어도 됩니다 -- drive 스텝은 자기 코스의 라벨만 찾습니다.
struct Course
{
  std::string name;
  std::string csv_path;
  WaypointFile waypoints;
  std::unordered_map<std::string, std::size_t> label_index;
};

// branch 스텝의 갈래 하나. accepted 중 하나가 debounce_frames 연속으로 확인되면
// target 스텝으로 갑니다.
struct BranchCase
{
  std::vector<std::string> accepted;
  std::string route;                    // mission.yaml의 `goto`
  std::size_t target{kEndOfMission};    // 그 route의 첫 스텝 (resolve_routes가 채웁니다)
};

struct Step
{
  StepType type{StepType::kDrive};

  // 이 스텝이 끝나면 갈 스텝. 보통은 다음 스텝이고, route의 마지막 스텝은 분기 지점
  // 바로 뒤(합류)이거나 kEndOfMission입니다. advance()가 이 값을 씁니다.
  //
  // branch 스텝은 예외입니다. 정상 진행은 next_index가 아니라 고른 갈래(cases/default의
  // target)로 가므로, branch의 next_index는 사실상 "이 분기를 건너뛰면 갈 곳"입니다 --
  // '~/skip' 서비스로 분기 자체를 넘길 때만 쓰입니다(어느 갈래도 달리지 않고 합류
  // 지점으로, 분기가 마지막이면 미션 종료).
  std::size_t next_index{0};

  // drive
  std::string label;                 // mission.yaml의 `until`
  std::size_t course_id{0};          // 이 스텝이 달릴 코스(courses의 인덱스)
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
  //
  // prearm_wait_step이 branch 스텝이면 merge 대상은 하나로 정해지지 않습니다 -- 어느
  // 갈래가 확인되느냐에 따라 그 갈래의 첫 drive 스텝이 됩니다. 그때 prearm_merge_step은
  // 쓰이지 않습니다(노드가 steps[prearm_wait_step].type을 보고 갈라집니다).
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

  // branch. timeout_s / debounce_frames / prearm_distance_m를 wait_signal과 같이 씁니다.
  std::vector<BranchCase> cases;
  std::string default_route;
  std::size_t default_target{kEndOfMission};

  // 두 종류가 같이 쓰는 필드. Step은 종류별로 나뉘지 않은 평평한 구조체입니다.
  //   wait_signal/branch에서: mission.yaml이 적어 준 값. 0보다 크면 prearm을 켭니다.
  //   drive에서:              link_prearm_steps가 뒤의 wait_signal/branch에서 복사해 온 값.
  //                           골까지 남은 거리가 이 값 이하로 들어오면 신호를 미리 보기
  //                           시작합니다.
  double prearm_distance_m{0.0};
};

inline const char * type_name(StepType type)
{
  switch (type) {
    case StepType::kDrive: return "drive";
    case StepType::kStop: return "stop";
    case StepType::kWaitSignal: return "wait_signal";
    case StepType::kBranch: return "branch";
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
  std::string waypoint_csv;         // main 코스의 CSV (courses.main.csv가 덮어씁니다)
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

// CSV + mission.yaml -> 코스 목록 + 스텝 목록. load()가 false를 반환하면 사유는 이미
// 로그에 있습니다.
class MissionLoader
{
public:
  MissionLoader(rclcpp::Logger logger, MissionLoadConfig config)
  : logger_(std::move(logger)), config_(std::move(config))
  {
  }

  bool load()
  {
    return load_mission_file();
  }

  // load()가 성공한 뒤에만 의미가 있습니다. 호출자가 std::move로 가져가도 됩니다.
  std::vector<Course> & courses() {return courses_;}
  std::vector<Step> & steps() {return steps_;}

private:
  // 한 코스 위에서 drive 세그먼트를 이어 붙일 때 쓰는 커서. 세그먼트 시작점은 그 코스의
  // 직전 drive 스텝의 도착점이고, 그 코스의 첫 세그먼트만 CSV 처음(#0)부터 시작합니다.
  struct CursorState
  {
    std::vector<std::size_t> cursor;
    std::vector<char> seen_drive;     // vector<bool>의 프록시 참조를 피합니다
  };

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
    // 갈래 CSV의 첫 점이 분기 지점에서 이만큼 넘게 떨어져 있으면 미션을 거부합니다.
    // 라벨 스냅 허용치와 같은 취지입니다 -- 어긋난 채로 주행이 시작되면 차가 분기 지점에서
    // 갈래 CSV의 첫 점까지 직선(lead-in)으로 건너뛰려 듭니다.
    const double seam_tolerance_m = root["branch_seam_tolerance_m"]
      ? root["branch_seam_tolerance_m"].as<double>() : 2.0;

    if (!root["steps"] || !root["steps"].IsSequence() || root["steps"].size() == 0) {
      RCLCPP_ERROR(logger_, "'%s' has no 'steps'.", config_.mission_yaml.c_str());
      return false;
    }

    if (!load_courses(root, snap_tolerance_m)) {
      return false;
    }

    cursors_.cursor.assign(courses_.size(), 0);
    cursors_.seen_drive.assign(courses_.size(), 0);

    if (!parse_steps(root["steps"], false)) {
      return false;
    }
    main_step_count_ = steps_.size();
    // 마지막 main 스텝의 next_index는 steps_.size()를 가리키고 있습니다. 그대로 두면
    // 아래에서 이어 붙는 route의 첫 스텝으로 흘러 들어가므로 여기서 끊어 줍니다.
    steps_.back().next_index = kEndOfMission;

    if (!resolve_routes(root["routes"])) {
      return false;
    }

    resolve_decel_tails();
    link_prearm_steps();
    if (!check_branch_seams(seam_tolerance_m)) {
      return false;
    }

    RCLCPP_INFO(
      logger_, "Mission '%s' loaded: %zu steps (%zu in the main sequence), %zu course(s).",
      config_.mission_yaml.c_str(), steps_.size(), main_step_count_, courses_.size());
    return true;
  }

  // ------------------------------------------------------------------ 코스 로딩

  // mission.yaml에 적힌 CSV 경로를 실제 파일로 풉니다.
  //
  // 절대 경로면 그대로 씁니다. 상대 경로는 (1) main CSV가 있는 디렉터리, (2) mission.yaml이
  // 있는 디렉터리, (3) 준 그대로(현재 작업 디렉터리) 순으로 찾습니다.
  //
  // (1)이 먼저인 이유: 갈래 CSV는 녹화 코스와 같은 곳(hyper_waypoint/waypoints/)에 둡니다.
  // 그러면 파일 이름만 적으면 되고, waypoint_csv:=.../real.csv로 실차 코스를 실을 때
  // 갈래 CSV도 같이 real 쪽으로 따라갑니다 -- 미션 파일을 안 고쳐도 됩니다.
  std::string resolve_csv_path(const std::string & given) const
  {
    namespace fs = std::filesystem;
    const fs::path path(given);
    if (path.is_absolute()) {
      return given;
    }

    std::vector<fs::path> candidates;
    if (!config_.waypoint_csv.empty()) {
      candidates.push_back(fs::path(config_.waypoint_csv).parent_path() / path);
    }
    if (!config_.mission_yaml.empty()) {
      candidates.push_back(fs::path(config_.mission_yaml).parent_path() / path);
    }
    candidates.push_back(path);

    for (const auto & candidate : candidates) {
      std::error_code ec;
      if (fs::exists(candidate, ec) && !ec) {
        return candidate.string();
      }
    }

    // 못 찾았습니다. 시도한 경로를 전부 남겨야 어느 규칙이 적용되는지 에러만 보고
    // 알 수 있습니다. 실제 "열기 실패" 에러는 add_course가 냅니다.
    std::string tried;
    for (const auto & candidate : candidates) {
      tried += "\n    " + candidate.string();
    }
    RCLCPP_ERROR(
      logger_, "Course CSV '%s' was not found. Tried:%s", given.c_str(), tried.c_str());
    return given;
  }

  bool add_course(const std::string & name, const std::string & csv_path)
  {
    Course course;
    course.name = name;
    course.csv_path = csv_path;

    std::string error;
    if (!load_waypoint_csv(csv_path, config_.min_spacing_m, course.waypoints, error)) {
      RCLCPP_ERROR(logger_, "Course '%s': %s", name.c_str(), error.c_str());
      return false;
    }
    if (course.waypoints.frame_id.empty()) {
      course.waypoints.frame_id = config_.frame_id;
    }
    // 갈래 경로는 분기 지점에서 main 코스에 이어 붙여 골 하나로 보낼 수 있습니다
    // (prearm 통과). 골 경로의 헤더 프레임은 하나뿐이므로 코스끼리 프레임이 달라선 안 됩니다.
    if (!courses_.empty() && course.waypoints.frame_id != courses_.front().waypoints.frame_id) {
      RCLCPP_ERROR(
        logger_,
        "Course '%s' is in frame '%s', but course '%s' is in frame '%s'. All courses of one "
        "mission must share a frame -- goal paths carry a single header frame.",
        name.c_str(), course.waypoints.frame_id.c_str(), courses_.front().name.c_str(),
        courses_.front().waypoints.frame_id.c_str());
      return false;
    }

    RCLCPP_INFO(
      logger_, "Course '%s': %zu waypoints from '%s' in frame '%s' (%zu rows skipped).",
      name.c_str(), course.waypoints.points.size(), csv_path.c_str(),
      course.waypoints.frame_id.c_str(), course.waypoints.skipped_rows);
    courses_.push_back(std::move(course));
    return true;
  }

  bool load_courses(const YAML::Node & root, double snap_tolerance_m)
  {
    const YAML::Node courses_node = root["courses"];
    if (courses_node && !courses_node.IsMap()) {
      RCLCPP_ERROR(logger_, "'courses' must be a map of <name>: {csv: ..., labels: ...}.");
      return false;
    }

    // main 코스는 항상 존재합니다. CSV는 courses.main.csv가 있으면 그것, 없으면
    // waypoint_csv 파라미터입니다(= 분기를 안 쓰는 미션의 예전 동작).
    std::string main_csv = config_.waypoint_csv;
    if (courses_node && courses_node["main"] && courses_node["main"]["csv"]) {
      main_csv = resolve_csv_path(courses_node["main"]["csv"].as<std::string>());
    }
    if (!add_course("main", main_csv)) {
      return false;
    }

    if (courses_node) {
      for (const auto & entry : courses_node) {
        const auto name = entry.first.as<std::string>();
        if (name == "main") {
          continue;   // 위에서 이미 실었습니다.
        }
        if (!entry.second.IsMap() || !entry.second["csv"]) {
          RCLCPP_ERROR(logger_, "Course '%s' has no 'csv'.", name.c_str());
          return false;
        }
        if (!add_course(name, resolve_csv_path(entry.second["csv"].as<std::string>()))) {
          return false;
        }
      }
    }

    // 라벨 스냅. 최상위 labels:는 main 코스의 것입니다 -- label_waypoints.py가 그 블록을
    // 통째로 재작성하므로 위치를 바꾸지 않습니다. 갈래 코스는 courses.<이름>.labels를 씁니다.
    if (!root["labels"] || !root["labels"].IsMap() || root["labels"].size() == 0) {
      RCLCPP_ERROR(
        logger_,
        "'%s' has no 'labels'. Place them first:\n"
        "  python3 src/planning/hyper_waypoint/scripts/label_waypoints.py %s",
        config_.mission_yaml.c_str(), courses_.front().csv_path.c_str());
      return false;
    }
    if (!snap_labels(courses_.front(), root["labels"], snap_tolerance_m)) {
      return false;
    }

    for (std::size_t i = 1; i < courses_.size(); ++i) {
      const YAML::Node labels = courses_node[courses_[i].name]["labels"];
      if (!labels || !labels.IsMap() || labels.size() == 0) {
        RCLCPP_ERROR(
          logger_,
          "Course '%s' has no 'labels'. Each course carries its own labels, snapped to its own "
          "CSV -- a drive step on this course can only reference labels declared here.",
          courses_[i].name.c_str());
        return false;
      }
      if (!snap_labels(courses_[i], labels, snap_tolerance_m)) {
        return false;
      }
    }
    return true;
  }

  // 라벨을 그 코스의 최근접 웨이포인트로 스냅합니다. 라벨은 좌표로 저장되어 있어 코스를
  // 다시 녹화해도 살아남지만, 그만큼 엉뚱한 데 붙을 수도 있으므로 여기서 걸러냅니다.
  bool snap_labels(Course & course, const YAML::Node & labels, double snap_tolerance_m)
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
        if (course.waypoints.points.empty()) {
          RCLCPP_ERROR(
            logger_, "Label '%s' is 'last', but course '%s' ('%s') has no waypoints.",
            name.c_str(), course.name.c_str(), course.csv_path.c_str());
          return false;
        }
        course.label_index[name] = course.waypoints.points.size() - 1;
        RCLCPP_INFO(
          logger_, "Label '%s' [%s] -> last waypoint #%zu (좌표 스냅 생략).",
          name.c_str(), course.name.c_str(), course.waypoints.points.size() - 1);
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
      for (std::size_t i = 0; i < course.waypoints.points.size(); ++i) {
        const double d = std::hypot(
          course.waypoints.points[i].x - x, course.waypoints.points[i].y - y);
        if (d < best) {
          best = d;
          nearest = i;
        }
      }
      if (best > snap_tolerance_m) {
        RCLCPP_ERROR(
          logger_,
          "Label '%s' (%.3f, %.3f) is %.2f m from the nearest waypoint (#%zu) of course '%s', "
          "beyond label_snap_tolerance_m (%.2f). Re-place it with label_waypoints.py, or check "
          "that mission.yaml and '%s' describe the same course.",
          name.c_str(), x, y, best, nearest, course.name.c_str(), snap_tolerance_m,
          course.csv_path.c_str());
        return false;
      }
      course.label_index[name] = nearest;
      RCLCPP_INFO(
        logger_, "Label '%s' [%s] -> waypoint #%zu (%.2f m away).",
        name.c_str(), course.name.c_str(), nearest, best);
    }
    return true;
  }

  // ------------------------------------------------------------------ 스텝 파싱

  // branch가 갈 수 있는 스텝 전부(각 case + default), 중복 없이. default가 어느 case와
  // 같은 갈래를 가리키는 것은 흔하고 정상입니다.
  static std::vector<std::size_t> branch_targets(const Step & branch)
  {
    std::vector<std::size_t> targets;
    const auto add = [&targets](std::size_t target) {
        if (std::find(targets.begin(), targets.end(), target) == targets.end()) {
          targets.push_back(target);
        }
      };
    for (const auto & branch_case : branch.cases) {
      add(branch_case.target);
    }
    add(branch.default_target);
    return targets;
  }

  bool find_course(const std::string & name, std::size_t & id) const
  {
    for (std::size_t i = 0; i < courses_.size(); ++i) {
      if (courses_[i].name == name) {
        id = i;
        return true;
      }
    }
    return false;
  }

  // 스텝을 읽어 steps_ 뒤에 이어 붙입니다. in_route면 route 안의 스텝이라 branch를
  // 허용하지 않습니다(중첩 분기는 합류 지점이 모호해집니다).
  //
  // next_index는 일단 "바로 다음 스텝"으로 채웁니다. 마지막 스텝의 값은 호출자가
  // 고쳐 줍니다(main은 kEndOfMission, route는 합류 지점).
  bool parse_steps(const YAML::Node & nodes, bool in_route)
  {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const YAML::Node & node = nodes[i];
      const auto type_text = node["type"] ? node["type"].as<std::string>() : std::string();
      const std::size_t index = steps_.size();

      Step step;
      if (type_text == "drive") {
        if (!parse_drive_step(node, index, step)) {
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
          RCLCPP_ERROR(logger_, "Step %zu (wait_signal) has an empty 'value'.", index);
          return false;
        }
      } else if (type_text == "branch") {
        if (in_route) {
          RCLCPP_ERROR(
            logger_,
            "Step %zu: a 'branch' inside a route is not supported -- the join point after a "
            "nested branch would be ambiguous. Keep branches in the main 'steps' sequence.",
            index);
          return false;
        }
        if (!parse_branch_step(node, index, step)) {
          return false;
        }
      } else {
        RCLCPP_ERROR(
          logger_, "Step %zu has unknown type '%s' (expected drive/stop/wait_signal/branch).",
          index, type_text.c_str());
        return false;
      }

      step.next_index = index + 1;
      steps_.push_back(step);
      if (steps_.back().type == StepType::kBranch) {
        // 이 갈림길에서의 커서 상태를 적어 둡니다. route의 스텝들은 main 스텝 전부를 읽은
        // 뒤에 파싱되므로, 그때 커서를 여기 값으로 되돌려야 세그먼트가 분기 지점에서
        // 이어집니다(갈래가 main 코스를 계속 쓰는 경우).
        branch_cursors_[index] = cursors_;
      }
    }
    return true;
  }

  bool parse_drive_step(const YAML::Node & node, std::size_t index, Step & step)
  {
    step.type = StepType::kDrive;
    if (!node["until"]) {
      RCLCPP_ERROR(logger_, "Step %zu (drive) has no 'until'.", index);
      return false;
    }
    step.label = node["until"].as<std::string>();

    const auto course_name = node["course"]
      ? node["course"].as<std::string>() : std::string("main");
    if (!find_course(course_name, step.course_id)) {
      RCLCPP_ERROR(
        logger_, "Step %zu references course '%s', which is not in 'courses'.",
        index, course_name.c_str());
      return false;
    }
    const Course & course = courses_[step.course_id];

    const auto found = course.label_index.find(step.label);
    if (found == course.label_index.end()) {
      RCLCPP_ERROR(
        logger_,
        "Step %zu references label '%s', which course '%s' does not declare. Labels belong to "
        "one course; add it under that course's 'labels' block.",
        index, step.label.c_str(), course.name.c_str());
      return false;
    }
    const std::size_t target = found->second;

    // 코스는 한 번 주행해 녹화한 것이므로 라벨은 CSV를 따라 단조 증가해야 합니다.
    // 그렇지 않으면 세그먼트가 비거나 거꾸로 뒤집힙니다. 커서는 코스마다 따로 셉니다 --
    // 갈래 CSV는 자기 처음(#0)부터 시작합니다.
    const bool seen = cursors_.seen_drive[step.course_id] != 0;
    const std::size_t cursor = cursors_.cursor[step.course_id];
    if (seen && target <= cursor) {
      RCLCPP_ERROR(
        logger_,
        "Step %zu: label '%s' is at waypoint #%zu of course '%s', which is not past the "
        "previous step's #%zu. Labels must advance along the recorded course; re-place '%s'.",
        index, step.label.c_str(), target, course.name.c_str(), cursor, step.label.c_str());
      return false;
    }
    step.begin_index = seen ? cursor : 0;
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
    const double opposing = reverse_fraction(
      course.waypoints.points, step.begin_index, step.end_index);
    if (step.reverse && opposing < 0.5) {
      RCLCPP_WARN(
        logger_,
        "Step %zu (until '%s') is marked reverse, but only %.0f%% of the recorded segment "
        "has the body heading opposing travel. Was this stretch actually recorded driving "
        "backwards?", index, step.label.c_str(), 100.0 * opposing);
    } else if (!step.reverse && opposing > 0.5) {
      RCLCPP_WARN(
        logger_,
        "Step %zu (until '%s') is NOT marked reverse, but %.0f%% of the recorded segment "
        "has the body heading opposing travel. Add 'reverse: true' and a reversing "
        "controller, or MPPI will try to drive it nose-first.",
        index, step.label.c_str(), 100.0 * opposing);
    }

    cursors_.cursor[step.course_id] = target;
    cursors_.seen_drive[step.course_id] = 1;
    return true;
  }

  // branch -- 신호를 보고 두 갈래(routes) 중 하나를 고릅니다.
  //
  // wait_signal과 결정적으로 다른 점은 "확인 실패"의 안전한 답이 없다는 것입니다.
  // 신호등은 못 보면 서 있으면 되지만(그게 안전), 갈림길은 어디로든 가야 합니다.
  // 그래서 default가 필수이고, timeout_s는 신호등의 60초와 달리 짧아야 합니다 --
  // 어차피 기다린다고 더 나은 답이 나오지 않습니다.
  bool parse_branch_step(const YAML::Node & node, std::size_t index, Step & step)
  {
    step.type = StepType::kBranch;
    step.timeout_s = node["timeout_s"] ? node["timeout_s"].as<double>() : 10.0;
    step.debounce_frames = node["debounce_frames"] ? node["debounce_frames"].as<int>() : 3;
    step.prearm_distance_m = node["prearm_distance_m"]
      ? node["prearm_distance_m"].as<double>() : 0.0;

    if (!node["default"]) {
      RCLCPP_ERROR(
        logger_,
        "Step %zu (branch) has no 'default'. A branch must name the route to take when the "
        "sign is never confirmed -- unlike a traffic light, standing still is not an answer.",
        index);
      return false;
    }
    step.default_route = node["default"].as<std::string>();

    const YAML::Node cases = node["cases"];
    if (!cases || !cases.IsSequence() || cases.size() == 0) {
      RCLCPP_ERROR(logger_, "Step %zu (branch) has no 'cases'.", index);
      return false;
    }

    std::unordered_set<std::string> seen_values;
    for (std::size_t c = 0; c < cases.size(); ++c) {
      const YAML::Node & entry = cases[c];
      if (!entry["value"] || !entry["goto"]) {
        RCLCPP_ERROR(
          logger_, "Step %zu (branch), case %zu: both 'value' and 'goto' are required.",
          index, c);
        return false;
      }
      BranchCase branch_case;
      branch_case.accepted = split_values(entry["value"].as<std::string>());
      branch_case.route = entry["goto"].as<std::string>();
      if (branch_case.accepted.empty()) {
        RCLCPP_ERROR(
          logger_, "Step %zu (branch), case %zu has an empty 'value'.", index, c);
        return false;
      }
      // 같은 값이 두 갈래에 나오면 어느 쪽으로 갈지가 순서에 숨습니다. 로드 시점에 잡습니다.
      for (const auto & value : branch_case.accepted) {
        if (!seen_values.insert(value).second) {
          RCLCPP_ERROR(
            logger_,
            "Step %zu (branch): sign value '%s' appears in more than one case. Each value must "
            "select exactly one route.", index, value.c_str());
          return false;
        }
      }
      step.cases.push_back(std::move(branch_case));
    }
    return true;
  }

  // ------------------------------------------------------------------ route 해석

  // branch가 가리키는 route들을 읽어 steps_ 뒤에 이어 붙이고, branch의 case/default를
  // 그 route의 첫 스텝 인덱스로 잇습니다.
  bool resolve_routes(const YAML::Node & routes)
  {
    // 어느 route가 어느 branch에 속하는지 모읍니다. 한 route를 두 branch가 가리키면
    // 합류 지점이 갈리므로 금지합니다.
    std::vector<std::pair<std::string, std::size_t>> referenced;   // (route, branch step)
    std::unordered_map<std::string, std::size_t> owner;
    for (std::size_t i = 0; i < main_step_count_; ++i) {
      if (steps_[i].type != StepType::kBranch) {
        continue;
      }
      std::vector<std::string> names;
      for (const auto & branch_case : steps_[i].cases) {
        names.push_back(branch_case.route);
      }
      names.push_back(steps_[i].default_route);
      for (const auto & name : names) {
        const auto found = owner.find(name);
        if (found == owner.end()) {
          owner[name] = i;
          referenced.emplace_back(name, i);
        } else if (found->second != i) {
          RCLCPP_ERROR(
            logger_,
            "Route '%s' is referenced by branch steps %zu and %zu. A route must belong to one "
            "branch -- otherwise there is no single place to rejoin after it.",
            name.c_str(), found->second, i);
          return false;
        }
      }
    }

    if (referenced.empty()) {
      if (routes && routes.IsMap() && routes.size() > 0) {
        RCLCPP_WARN(
          logger_, "'routes' has %zu entries but no branch step references any of them.",
          routes.size());
      }
      return true;
    }
    if (!routes || !routes.IsMap()) {
      RCLCPP_ERROR(
        logger_, "A branch step references routes, but '%s' has no 'routes' map.",
        config_.mission_yaml.c_str());
      return false;
    }

    std::unordered_map<std::string, std::size_t> route_start;
    for (const auto & [name, branch_index] : referenced) {
      const YAML::Node node = routes[name];
      if (!node || !node.IsSequence() || node.size() == 0) {
        RCLCPP_ERROR(
          logger_, "Route '%s' (referenced by branch step %zu) is missing or empty in 'routes'.",
          name.c_str(), branch_index);
        return false;
      }

      // 이 갈림길에서의 커서로 되돌립니다. 갈래가 자기 CSV를 쓰면 그 코스는 아직
      // 안 쓴 상태(#0부터)이고, main 코스를 계속 쓰면 분기 지점에서 이어집니다.
      cursors_ = branch_cursors_.at(branch_index);

      const std::size_t start = steps_.size();
      if (!parse_steps(node, true)) {
        return false;
      }
      route_start[name] = start;

      // route가 끝나면 어디로 가는가. 분기 뒤에 main 스텝이 남아 있으면 거기서 합류하고,
      // branch가 main의 마지막 스텝이면 미션이 끝납니다.
      const std::size_t join = (branch_index + 1 < main_step_count_)
        ? branch_index + 1 : kEndOfMission;
      steps_.back().next_index = join;
      const std::string after = join == kEndOfMission
        ? std::string("the mission ends") : "step " + std::to_string(join);
      RCLCPP_INFO(
        logger_, "Route '%s': steps %zu..%zu, then %s.",
        name.c_str(), start, steps_.size() - 1, after.c_str());
    }

    for (std::size_t i = 0; i < main_step_count_; ++i) {
      Step & step = steps_[i];
      if (step.type != StepType::kBranch) {
        continue;
      }
      for (auto & branch_case : step.cases) {
        branch_case.target = route_start.at(branch_case.route);
      }
      step.default_target = route_start.at(step.default_route);
      RCLCPP_INFO(
        logger_, "Branch step %zu: %zu case(s), default '%s' -> step %zu, timeout %.0f s.",
        i, step.cases.size(), step.default_route.c_str(), step.default_target, step.timeout_s);
    }
    return true;
  }

  // --------------------------------------------------------------- 후처리/검증

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

  // wait_signal/branch의 prearm_distance_m를 그 앞의 drive 스텝에 이어 줍니다. 패턴은
  // 반드시 [drive -> wait_signal -> drive] 또는 [drive -> branch -> (각 갈래의 첫 drive)]
  // 여야 합니다: 신호를 미리 보는 것은 앞의 drive이고, 통과하면 골을 이어 붙일 끝점은
  // 뒤의 drive이기 때문입니다.
  //
  // 조건이 안 맞으면 경고만 남기고 그 자리의 prearm을 끕니다 -- 미션을 거부하지 않는 이유는,
  // prearm이 꺼진 결과가 곧 "정지선에 서서 기다렸다가 간다"라서 안전하기 때문입니다.
  void link_prearm_steps()
  {
    for (std::size_t i = 0; i < steps_.size(); ++i) {
      Step & drive = steps_[i];
      if (drive.type != StepType::kDrive || drive.next_index >= steps_.size()) {
        continue;
      }
      const Step & next = steps_[drive.next_index];
      if (next.prearm_distance_m <= 0.0) {
        continue;
      }
      if (next.type == StepType::kWaitSignal) {
        link_wait_signal_prearm(i, drive.next_index);
      } else if (next.type == StepType::kBranch) {
        link_branch_prearm(i, drive.next_index);
      }
    }
  }

  void link_wait_signal_prearm(std::size_t drive_index, std::size_t wait_index)
  {
    Step & drive = steps_[drive_index];
    const Step & wait_step = steps_[wait_index];
    const std::size_t merge = wait_step.next_index;

    const char * reason = nullptr;
    if (merge >= steps_.size()) {
      reason = "there is no step after the wait_signal to merge into";
    } else if (steps_[merge].type != StepType::kDrive) {
      reason = "the step after the wait_signal is not a drive step";
    } else if (drive.reverse || steps_[merge].reverse) {
      // 두 세그먼트를 한 골로 합치면 그 안에 방향 전환이 들어갑니다. RPP는 이를 처리하지
      // 못하고, MPPI도 PreferForwardCritic 때문에 안정적으로 못 냅니다.
      reason = "merging would put a direction change inside a single goal";
    } else if (drive.controller_id != steps_[merge].controller_id) {
      reason = "the two drive steps use different controllers";
    }
    if (reason != nullptr) {
      RCLCPP_WARN(
        logger_,
        "Step %zu's wait_signal has prearm_distance_m %.1f, but %s. Prearm is off here: the "
        "vehicle will stop at '%s' and wait as usual.",
        wait_index, wait_step.prearm_distance_m, reason, drive.label.c_str());
      return;
    }

    drive.prearm_enabled = true;
    drive.prearm_wait_step = wait_index;
    drive.prearm_merge_step = merge;
    drive.prearm_distance_m = wait_step.prearm_distance_m;
    RCLCPP_INFO(
      logger_,
      "Prearm: while driving to '%s', watch %s for '%s' from %.1f m out; if confirmed, roll "
      "straight through to '%s' without stopping.",
      drive.label.c_str(), config_.sign_topic.c_str(),
      join_values(wait_step.accepted).c_str(), drive.prearm_distance_m,
      steps_[merge].label.c_str());
  }

  // 분기의 prearm. 확인되면 서지 않고 그대로 갈래로 들어갑니다 -- 골을 "지금 위치 ->
  // 분기 지점 -> 고른 갈래의 끝"으로 갈아끼웁니다. 어느 갈래로 갈아끼울지는 주행 중에
  // 정해지므로, 여기서는 "어느 갈래가 나오든 갈아끼울 수 있는가"만 검사합니다.
  void link_branch_prearm(std::size_t drive_index, std::size_t branch_index)
  {
    Step & drive = steps_[drive_index];
    const Step & branch = steps_[branch_index];

    const std::vector<std::size_t> targets = branch_targets(branch);

    const char * reason = nullptr;
    if (drive.reverse) {
      reason = "the drive step before the branch is a reverse segment";
    } else {
      for (const std::size_t target : targets) {
        if (target >= steps_.size() || steps_[target].type != StepType::kDrive) {
          reason = "a route does not start with a drive step";
          break;
        }
        if (steps_[target].reverse) {
          reason = "a route starts with a reverse segment, which cannot be merged into a "
            "single forward goal";
          break;
        }
        if (steps_[target].controller_id != drive.controller_id) {
          reason = "a route's first drive step uses a different controller";
          break;
        }
      }
    }
    if (reason != nullptr) {
      RCLCPP_WARN(
        logger_,
        "Branch step %zu has prearm_distance_m %.1f, but %s. Prearm is off here: the vehicle "
        "will stop at '%s', read the sign, and then take the route.",
        branch_index, branch.prearm_distance_m, reason, drive.label.c_str());
      return;
    }

    drive.prearm_enabled = true;
    drive.prearm_wait_step = branch_index;
    drive.prearm_merge_step = branch_index;   // 분기에서는 안 씁니다(갈래마다 다릅니다).
    drive.prearm_distance_m = branch.prearm_distance_m;
    RCLCPP_INFO(
      logger_,
      "Prearm: while driving to '%s', watch %s from %.1f m out; if a branch sign is confirmed, "
      "roll straight into that route without stopping (default '%s' if nothing is confirmed "
      "by the label).",
      drive.label.c_str(), config_.sign_topic.c_str(), drive.prearm_distance_m,
      branch.default_route.c_str());
  }

  // 갈래 CSV의 첫 점이 실제 분기 지점에 붙어 있는지 봅니다.
  //
  // 안 붙어 있으면 주행 중에 조용히 이상해집니다: 갈래 스텝의 경로는 그 CSV의 #0에서
  // 시작하므로, 차가 분기 지점에 서 있는데 경로가 20 m 떨어진 곳에서 시작하면
  // trim_to_robot이 그 사이를 직선 lead-in으로 메웁니다 -- 코스를 벗어나 가로지릅니다.
  // 라벨 스냅과 같은 이유로 경고가 아니라 거부입니다.
  bool check_branch_seams(double seam_tolerance_m)
  {
    for (std::size_t i = 0; i < main_step_count_; ++i) {
      const Step & branch = steps_[i];
      if (branch.type != StepType::kBranch) {
        continue;
      }

      // 분기 지점 = 이 branch 앞의 가장 가까운 drive 스텝의 도착점.
      const Step * lead = nullptr;
      for (std::size_t j = i; j-- > 0; ) {
        if (steps_[j].type == StepType::kDrive) {
          lead = &steps_[j];
          break;
        }
      }
      if (lead == nullptr) {
        RCLCPP_WARN(
          logger_,
          "Branch step %zu has no drive step before it, so the branch point is unknown and the "
          "seam to each route cannot be checked.", i);
        continue;
      }
      const Course & lead_course = courses_[lead->course_id];
      if (lead->end_index >= lead_course.waypoints.points.size()) {
        continue;
      }
      const Waypoint & fork = lead_course.waypoints.points[lead->end_index];

      for (const std::size_t target : branch_targets(branch)) {
        if (target >= steps_.size() || steps_[target].type != StepType::kDrive) {
          continue;   // link_branch_prearm이 이미 경고했습니다. 여기서는 잴 것이 없습니다.
        }
        const Step & first = steps_[target];
        if (first.course_id == lead->course_id) {
          continue;   // 같은 코스를 이어 달리는 갈래 -- 이음매가 없습니다.
        }
        const Course & course = courses_[first.course_id];
        if (first.begin_index >= course.waypoints.points.size()) {
          continue;
        }
        const Waypoint & start = course.waypoints.points[first.begin_index];
        const double seam = std::hypot(start.x - fork.x, start.y - fork.y);
        if (seam > seam_tolerance_m) {
          RCLCPP_ERROR(
            logger_,
            "Course '%s' starts %.2f m from the branch point at '%s' (wp #%zu of course '%s'), "
            "beyond branch_seam_tolerance_m (%.2f). Re-record '%s' starting at the fork, or the "
            "vehicle will cut a straight line across the gap.",
            course.name.c_str(), seam, lead->label.c_str(), lead->end_index,
            lead_course.name.c_str(), seam_tolerance_m, course.csv_path.c_str());
          return false;
        }
        RCLCPP_INFO(
          logger_, "Branch seam: course '%s' starts %.2f m from '%s'.",
          course.name.c_str(), seam, lead->label.c_str());
      }
    }
    return true;
  }

  rclcpp::Logger logger_;
  MissionLoadConfig config_;

  std::vector<Course> courses_;
  std::vector<Step> steps_;
  // main 'steps' 시퀀스의 길이. 이 뒤에 route의 스텝들이 이어 붙습니다.
  std::size_t main_step_count_{0};

  CursorState cursors_;
  std::unordered_map<std::size_t, CursorState> branch_cursors_;
};

}  // namespace hyper_planner
