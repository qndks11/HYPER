#pragma once

// mission.yaml의 `keepout:` 다각형을 local_costmap의 keepout_layer(nav2 StaticLayer)가 먹는
// 마스크(OccupancyGrid)로 굽는 코드. mission_manager_node가 미션을 로드할 때 한 번 굽고 latched로
// 냅니다.
//
// 마스크 값은 OccupancyGrid 관례 그대로입니다: 다각형 안 = 100(StaticLayer가 254 LETHAL로
// 읽고, 뒤의 inflation_layer가 팽창시킵니다), 밖 = 0(비어 있음). -1(모름)을 쓰지 않는 이유:
// 0으로 채워 두면 "여기는 금지 구역이 아니다"가 명시적이라 레이어가 늘 같은 답을 냅니다.
//
// costmap_query.hpp와 마찬가지로 tf도 로그도 없습니다 -- 좌표는 코스 프레임(map) 그대로 받고,
// 프레임 변환은 StaticLayer가 합니다(map -> odom).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace hyper_planner
{

// 한 구역이 이보다 넓으면 좌표 오타로 봅니다. 트랙 전체가 100 m 남짓이고, 0.05 m 격자에서
// 500 m는 이미 1억 셀입니다.
inline constexpr double kMaxKeepoutSpanM = 500.0;

using KeepoutPolygon = std::vector<std::pair<double, double>>;

struct KeepoutZone
{
  std::string name;
  KeepoutPolygon points;   // 코스 프레임(map) 꼭짓점. 닫는 변은 암묵적입니다.
};

// even-odd 규칙. 꼭짓점이 3개 미만이면 늘 false입니다.
inline bool point_in_polygon(double x, double y, const KeepoutPolygon & points)
{
  const std::size_t n = points.size();
  if (n < 3) {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = points[i].first;
    const double yi = points[i].second;
    const double xj = points[j].first;
    const double yj = points[j].second;
    if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
      inside = !inside;
    }
  }
  return inside;
}

inline double point_segment_distance(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq <= 1e-12) {
    return std::hypot(px - ax, py - ay);
  }
  const double t = std::clamp(((px - ax) * dx + (py - ay) * dy) / length_sq, 0.0, 1.0);
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

// 원(중심 cx, cy / 반지름 radius)이 다각형에 조금이라도 걸치는지. 중심이 안에 있거나, 어느
// 변이든 반지름 안으로 들어오면 걸친 것입니다.
inline bool circle_touches_polygon(
  double cx, double cy, double radius, const KeepoutPolygon & points)
{
  if (point_in_polygon(cx, cy, points)) {
    return true;
  }
  const std::size_t n = points.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto & a = points[i];
    const auto & b = points[(i + 1) % n];
    if (point_segment_distance(cx, cy, a.first, a.second, b.first, b.second) <= radius) {
      return true;
    }
  }
  return false;
}

// 구역들을 한 장의 마스크로 굽습니다. 격자는 모든 꼭짓점의 bbox + pad_m이고, 셀 **중심**이
// 어느 다각형 안에 들면 100입니다(costmap_query.hpp와 같은 셀 중심 규칙).
//
// 구역이 없으면 0 한 칸짜리 격자를 돌려줍니다. 그래도 내야 하는 이유: 앞서 받은 마스크를
// keepout_layer가 들고 있으므로, 빈 미션으로 다시 띄웠을 때 옛 구역이 남지 않게 덮어써야
// 합니다.
//
// stamp는 채우지 않습니다 -- 노드 시계를 아는 호출자가 채웁니다.
inline nav_msgs::msg::OccupancyGrid rasterize_keepout(
  const std::vector<KeepoutZone> & zones, double resolution, const std::string & frame_id,
  double pad_m)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame_id;
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.origin.orientation.w = 1.0;

  bool any = false;
  double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
  for (const auto & zone : zones) {
    for (const auto & [x, y] : zone.points) {
      if (!any) {
        min_x = max_x = x;
        min_y = max_y = y;
        any = true;
        continue;
      }
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
  }
  if (!any || resolution <= 0.0) {
    grid.info.width = 1;
    grid.info.height = 1;
    grid.data.assign(1, 0);
    return grid;
  }

  // 원점을 격자 눈금에 맞춥니다 -- 같은 다각형이면 늘 같은 셀이 칠해지도록.
  const double origin_x = std::floor((min_x - pad_m) / resolution) * resolution;
  const double origin_y = std::floor((min_y - pad_m) / resolution) * resolution;
  const auto width = static_cast<std::int64_t>(
    std::max(1.0, std::ceil((max_x + pad_m - origin_x) / resolution)));
  const auto height = static_cast<std::int64_t>(
    std::max(1.0, std::ceil((max_y + pad_m - origin_y) / resolution)));

  grid.info.width = static_cast<std::uint32_t>(width);
  grid.info.height = static_cast<std::uint32_t>(height);
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.data.assign(static_cast<std::size_t>(width * height), 0);

  const auto to_index = [resolution](double world, double origin) {
      return static_cast<std::int64_t>(std::floor((world - origin) / resolution));
    };
  for (const auto & zone : zones) {
    if (zone.points.size() < 3) {
      continue;
    }
    // 이 구역의 bbox 안만 훑습니다. 트랙 한 장을 통째로 도는 대신.
    double zx0 = zone.points.front().first, zx1 = zx0;
    double zy0 = zone.points.front().second, zy1 = zy0;
    for (const auto & [x, y] : zone.points) {
      zx0 = std::min(zx0, x);
      zx1 = std::max(zx1, x);
      zy0 = std::min(zy0, y);
      zy1 = std::max(zy1, y);
    }
    const std::int64_t i0 = std::max<std::int64_t>(0, to_index(zx0, origin_x));
    const std::int64_t i1 = std::min<std::int64_t>(width - 1, to_index(zx1, origin_x));
    const std::int64_t j0 = std::max<std::int64_t>(0, to_index(zy0, origin_y));
    const std::int64_t j1 = std::min<std::int64_t>(height - 1, to_index(zy1, origin_y));
    for (std::int64_t j = j0; j <= j1; ++j) {
      const double cell_y = origin_y + (static_cast<double>(j) + 0.5) * resolution;
      for (std::int64_t i = i0; i <= i1; ++i) {
        const double cell_x = origin_x + (static_cast<double>(i) + 0.5) * resolution;
        if (point_in_polygon(cell_x, cell_y, zone.points)) {
          grid.data[static_cast<std::size_t>(j * width + i)] = 100;
        }
      }
    }
  }
  return grid;
}

}  // namespace hyper_planner
