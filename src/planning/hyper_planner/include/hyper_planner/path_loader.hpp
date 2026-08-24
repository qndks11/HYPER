#pragma once

// hyper_waypoint가 녹화한 웨이포인트 CSV를 읽어 nav_msgs/Path로 만드는 공용 코드.
// mission_manager_node가 씁니다(follow_path_client_node는 아직 자기 사본을 들고 있습니다).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "hyper_planner/common.hpp"

namespace hyper_planner
{

struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  // CSV에 쓸 만한 yaw 값이 있었는지. false면 이웃 점에서 유도합니다.
  bool has_yaw{false};
};

struct WaypointFile
{
  std::vector<Waypoint> points;
  std::string frame_id;
  std::size_t skipped_rows{0};
};

inline std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    // 공백과 CRLF의 '\r'를 제거합니다.
    const auto begin = field.find_first_not_of(" \t\r\n");
    const auto end = field.find_last_not_of(" \t\r\n");
    fields.push_back(begin == std::string::npos ? "" : field.substr(begin, end - begin + 1));
  }
  return fields;
}

// 헤더 이름으로 읽으므로 최소 레이아웃(idx,x,y,yaw,frame_id)과 레코더의 전체
// 레이아웃(GPS/IMU 컬럼 포함) 양쪽 다 동작합니다.
// 실패하면 false를 반환하고 error에 사람이 읽을 이유를 채웁니다.
inline bool load_waypoint_csv(
  const std::string & csv_path, double min_spacing_m, WaypointFile & out, std::string & error)
{
  out = WaypointFile{};

  if (csv_path.empty()) {
    error = "waypoint CSV path is empty";
    return false;
  }

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    error = "failed to open '" + csv_path + "'";
    return false;
  }

  std::string line;
  if (!std::getline(file, line)) {
    error = "'" + csv_path + "' is empty";
    return false;
  }

  std::unordered_map<std::string, std::size_t> column;
  const auto header = split_csv_line(line);
  for (std::size_t i = 0; i < header.size(); ++i) {
    column[header[i]] = i;
  }
  if (column.count("x") == 0 || column.count("y") == 0) {
    error = "'" + csv_path + "' has no 'x'/'y' header columns";
    return false;
  }

  const std::size_t x_col = column["x"];
  const std::size_t y_col = column["y"];
  const bool has_yaw_col = column.count("yaw") > 0;
  const bool has_frame_col = column.count("frame_id") > 0;

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = split_csv_line(line);
    if (fields.size() <= std::max(x_col, y_col) ||
      fields[x_col].empty() || fields[y_col].empty())
    {
      ++out.skipped_rows;
      continue;
    }

    Waypoint wp;
    try {
      wp.x = std::stod(fields[x_col]);
      wp.y = std::stod(fields[y_col]);
    } catch (const std::exception &) {
      ++out.skipped_rows;
      continue;
    }

    // 미션 세그먼트는 라벨을 CSV 인덱스로 스냅해서 잘라내므로, 여기서 점을 솎아내면
    // 인덱스가 밀립니다. 스냅은 로드가 끝난 뒤에 하므로 순서상 문제는 없습니다.
    if (min_spacing_m > 0.0 && !out.points.empty()) {
      const auto & prev = out.points.back();
      if (std::hypot(wp.x - prev.x, wp.y - prev.y) < min_spacing_m) {
        ++out.skipped_rows;
        continue;
      }
    }

    if (has_yaw_col) {
      const std::size_t yaw_col = column["yaw"];
      if (fields.size() > yaw_col && !fields[yaw_col].empty()) {
        try {
          wp.yaw = std::stod(fields[yaw_col]);
          wp.has_yaw = std::isfinite(wp.yaw);
        } catch (const std::exception &) {
          wp.has_yaw = false;
        }
      }
    }

    if (out.frame_id.empty() && has_frame_col) {
      const std::size_t frame_col = column["frame_id"];
      if (fields.size() > frame_col) {
        out.frame_id = fields[frame_col];
      }
    }

    out.points.push_back(wp);
  }

  if (out.points.empty()) {
    error = "'" + csv_path + "' contains no usable waypoints";
    return false;
  }
  return true;
}

// 포즈의 방향(orientation)은 "차체가 향한 방향"이어야 합니다.
//
// CSV의 yaw 컬럼은 EKF가 준 실제 차체 헤딩이라 전진/후진 상관없이 그대로 씁니다.
// 후진으로 녹화한 구간에서는 이 값이 진행 방향의 반대를 가리키는데, 그게 맞습니다:
//   - nav2 RPP는 allow_reversing일 때 진행 방향을 포즈 방향이 아니라 carrot 점의
//     차체 좌표계 x부호로 판단하므로 여기서 뒤집을 필요가 없고,
//   - SimpleGoalChecker의 yaw 비교는 차체 헤딩 기준이라 뒤집으면 오히려 틀립니다.
//
// yaw 컬럼이 없거나 깨진 경우에만 이웃 점에서 진행 방향을 유도하는데, 이때는 그 값이
// 진행 방향이므로 후진 세그먼트에서 180도 뒤집어야 차체 헤딩이 됩니다.
inline double waypoint_heading(
  const std::vector<Waypoint> & points, std::size_t i, bool reverse)
{
  const Waypoint & wp = points[i];
  if (wp.has_yaw) {
    return wp.yaw;
  }

  double yaw = 0.0;
  if (i + 1 < points.size()) {
    yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
  } else if (i > 0) {
    yaw = std::atan2(points[i].y - points[i - 1].y, points[i].x - points[i - 1].x);
  }
  return reverse ? normalize_angle(yaw + kPi) : yaw;
}

// [first, last] (양끝 포함) 구간을 Path로 만듭니다.
inline nav_msgs::msg::Path make_path(
  const std::vector<Waypoint> & points, std::size_t first, std::size_t last,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp, bool reverse)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  if (points.empty() || first > last || last >= points.size()) {
    return path;
  }

  path.poses.reserve(last - first + 1);
  for (std::size_t i = first; i <= last; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].x;
    pose.pose.position.y = points[i].y;
    pose.pose.orientation = quaternion_from_yaw(waypoint_heading(points, i, reverse));
    path.poses.push_back(pose);
  }
  return path;
}

// 녹화된 헤딩이 실제로 진행 방향을 거스르는 표본의 비율. reverse: true로 표시한
// 구간이 정말 후진으로 녹화된 건지 로드 시점에 확인하는 데 씁니다.
inline double reverse_fraction(
  const std::vector<Waypoint> & points, std::size_t first, std::size_t last)
{
  std::size_t total = 0;
  std::size_t opposing = 0;
  for (std::size_t i = first; i < last && i + 1 < points.size(); ++i) {
    const double dx = points[i + 1].x - points[i].x;
    const double dy = points[i + 1].y - points[i].y;
    if (std::hypot(dx, dy) < 1e-6) {
      continue;
    }
    const double yaw = waypoint_heading(points, i, false);
    ++total;
    if (std::cos(yaw) * dx + std::sin(yaw) * dy < 0.0) {
      ++opposing;
    }
  }
  return total == 0 ? 0.0 : static_cast<double>(opposing) / static_cast<double>(total);
}

inline std::size_t nearest_pose_index(
  const nav_msgs::msg::Path & path, double x, double y, double & distance_m)
{
  std::size_t nearest = 0;
  distance_m = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double d = std::hypot(
      path.poses[i].pose.position.x - x, path.poses[i].pose.position.y - y);
    if (d < distance_m) {
      distance_m = d;
      nearest = i;
    }
  }
  return nearest;
}

// 경로를 spacing_m 간격으로 다시 깔아 줍니다(위치는 선형, 헤딩은 최단호 보간).
//
// 왜 필요한가: nav2 MPPI의 경로 critic들은 "미터"가 아니라 "경로 점 개수"로 앞을 보고,
// 특히 PathAlignCritic은 궤적 점에서 가장 가까운 경로 *점*까지의 거리를 벌점으로 씁니다
// -- 선분에 내린 수선이 아니라 점입니다. 그래서 점 간격이 s면 경로 한가운데를 정확히
// 밟고 달려도 최대 s/2의 벌점이 항상 깔리고, 그만큼 "코너를 잘라도 벌점이 늘지 않는"
// 사각지대가 생깁니다. 녹화 CSV는 간격이 약 0.59 m라 사각지대가 ±0.3 m -- 실제로 보이던
// 코너 컷 크기와 같습니다. 촘촘하게 다시 깔면 이 사각지대가 없어집니다.
//
// 첫/마지막 포즈는 정확히 보존합니다. 마지막 포즈가 곧 goal이라 goal checker 공차에
// 직접 영향을 주기 때문입니다.
//
// 주의 1: 점이 촘촘해지면 offset_from_furthest처럼 "점 개수"로 세는 파라미터의 의미가
//        같이 바뀝니다. nav2_controller.yaml의 해당 값들을 반드시 함께 조정하세요.
// 주의 2: PathAlignCritic의 비용은 batch x (time_steps / trajectory_point_step) x 경로 점
//        개수에 비례합니다. 간격을 절반으로 줄이면 그 항이 두 배가 됩니다.
inline void resample_path(nav_msgs::msg::Path & path, double spacing_m)
{
  if (spacing_m <= 0.0 || path.poses.size() < 2) {
    return;
  }

  std::vector<double> arc(path.poses.size(), 0.0);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    arc[i] = arc[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }

  const double total = arc.back();
  if (total < spacing_m) {
    return;
  }

  const auto steps = static_cast<std::size_t>(std::llround(total / spacing_m));
  // 이미 충분히 촘촘하면 건드리지 않습니다(간격을 늘리는 용도로는 쓰지 않습니다).
  if (steps + 1 <= path.poses.size()) {
    return;
  }

  std::vector<geometry_msgs::msg::PoseStamped> out;
  out.reserve(steps + 1);
  out.push_back(path.poses.front());

  std::size_t seg = 1;
  for (std::size_t k = 1; k < steps; ++k) {
    const double s = total * static_cast<double>(k) / static_cast<double>(steps);
    while (seg + 1 < path.poses.size() && arc[seg] < s) {
      ++seg;
    }

    const auto & a = path.poses[seg - 1];
    const auto & b = path.poses[seg];
    const double span = arc[seg] - arc[seg - 1];
    const double t = span > 1e-9 ? (s - arc[seg - 1]) / span : 0.0;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = a.pose.position.x + t * (b.pose.position.x - a.pose.position.x);
    pose.pose.position.y = a.pose.position.y + t * (b.pose.position.y - a.pose.position.y);

    // 후진 구간에서 헤딩이 ±pi 근처를 넘나들 수 있으므로 최단호로 보간합니다.
    const double yaw_a = yaw_from_quaternion(a.pose.orientation);
    const double delta = normalize_angle(yaw_from_quaternion(b.pose.orientation) - yaw_a);
    pose.pose.orientation = quaternion_from_yaw(normalize_angle(yaw_a + t * delta));

    out.push_back(pose);
  }
  out.push_back(path.poses.back());

  path.poses = std::move(out);
}

// 차량에서 경로 첫 점까지 직선 진입 경로를 spacing 간격으로 깔아 줍니다.
// 차량 헤딩과 회전 반경을 무시하므로 전진 세그먼트에서만 쓰세요.
inline std::size_t insert_lead_in(
  nav_msgs::msg::Path & path, double robot_x, double robot_y, double spacing_m)
{
  if (path.poses.empty() || spacing_m <= 0.0) {
    return 0;
  }
  const auto target = path.poses.front().pose.position;
  const double dx = target.x - robot_x;
  const double dy = target.y - robot_y;
  const double distance = std::hypot(dx, dy);
  if (distance <= spacing_m) {
    return 0;
  }

  const auto orientation = quaternion_from_yaw(std::atan2(dy, dx));
  const auto steps = static_cast<std::size_t>(std::ceil(distance / spacing_m));

  std::vector<geometry_msgs::msg::PoseStamped> lead_in;
  lead_in.reserve(steps);
  for (std::size_t i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = robot_x + t * dx;
    pose.pose.position.y = robot_y + t * dy;
    pose.pose.orientation = orientation;
    lead_in.push_back(pose);
  }

  path.poses.insert(path.poses.begin(), lead_in.begin(), lead_in.end());
  return lead_in.size();
}

}  // namespace hyper_planner
