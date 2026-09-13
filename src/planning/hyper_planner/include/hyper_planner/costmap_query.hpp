#pragma once

// 로컬 costmap의 한 점 주변에 lethal 셀이 몇 개나 있는지 세는 코드.
// mission_manager_node의 `select_by: clearance` 분기와 ~/probe_costmap이 이 값으로
// "저 칸 입구에 콘이 서 있는가"를 판단합니다.
//
// 왜 /local_costmap/costmap이 아니라 /local_costmap/costmap_raw인가:
// OccupancyGrid 쪽은 0~255를 0~100으로 다시 스케일하면서 254(LETHAL)와
// 253(INSCRIBED_INFLATED_OBSTACLE)을 각각 100과 99로 뭉갭니다. 여기서 필요한 구분이
// 정확히 그 둘입니다 -- InflationLayer는 253 이하만 쓰므로, 254만 세면 팽창 후광이
// 저절로 빠집니다. nav2_msgs/Costmap은 원본 코스트를 그대로 실어 옵니다.
//
// nav2_costmap_2d::CostmapSubscriber를 쓰지 않는 이유는 그쪽이 header.frame_id를
// 버리기 때문입니다(costmap_subscriber.hpp). 이 코스트맵은 rolling이라 프레임과 원점이
// 매 주기 달라지므로, 호출자는 지금 처리 중인 메시지의 것을 써야 합니다.
//
// 여기에는 tf도 로그도 없습니다 -- 좌표는 이미 코스트맵 프레임으로 옮겨서 넣어 주고,
// 이 함수는 순수하게 격자만 셉니다.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <nav2_msgs/msg/costmap.hpp>

namespace hyper_planner
{

// count_lethal_near의 결과. 세어 본 셀 수와 창 밖으로 잘렸는지를 같이 돌려줍니다 --
// "0개"가 "깨끗하다"인지 "안 보인다"인지는 호출자가 구별해야 하기 때문입니다.
struct ClearanceCount
{
  std::size_t lethal{0};      // lethal_cost 이상인 셀 수
  std::size_t inspected{0};   // 실제로 들여다본 셀 수 (반지름 안 & 격자 안)
  bool clipped{false};        // 반지름의 일부가 코스트맵 밖이었다
};

// (x, y)를 중심으로 반지름 radius_m 안에서 lethal_cost 이상인 셀을 셉니다.
// x, y는 **코스트맵 자신의 프레임**(local_costmap이면 odom) 좌표여야 합니다.
//
// origin의 yaw는 보지 않습니다. Costmap2D에는 회전이 없고 nav2의 발행자도 늘
// orientation.w = 1로 채웁니다 -- 격자는 언제나 축 정렬입니다.
inline ClearanceCount count_lethal_near(
  const nav2_msgs::msg::Costmap & map, double x, double y, double radius_m,
  std::uint8_t lethal_cost)
{
  ClearanceCount out;
  const double resolution = map.metadata.resolution;
  const auto size_x = static_cast<std::int64_t>(map.metadata.size_x);
  const auto size_y = static_cast<std::int64_t>(map.metadata.size_y);
  if (resolution <= 0.0 || size_x <= 0 || size_y <= 0 ||
    map.data.size() < static_cast<std::size_t>(size_x * size_y))
  {
    out.clipped = true;
    return out;
  }

  const double origin_x = map.metadata.origin.position.x;
  const double origin_y = map.metadata.origin.position.y;

  // 원의 바깥 사각형을 셀 인덱스로 바꾸고, 격자 밖으로 나간 만큼은 잘라 냅니다.
  // 잘렸다는 사실 자체가 결과입니다 -- 콘이 창 밖에 있으면 0개로 보이기 때문입니다.
  const auto to_index = [&](double world, double origin) {
      return static_cast<std::int64_t>(std::floor((world - origin) / resolution));
    };
  const std::int64_t min_i = to_index(x - radius_m, origin_x);
  const std::int64_t max_i = to_index(x + radius_m, origin_x);
  const std::int64_t min_j = to_index(y - radius_m, origin_y);
  const std::int64_t max_j = to_index(y + radius_m, origin_y);
  if (min_i < 0 || min_j < 0 || max_i >= size_x || max_j >= size_y) {
    out.clipped = true;
  }

  const double radius_sq = radius_m * radius_m;
  for (std::int64_t j = std::max<std::int64_t>(min_j, 0);
    j <= std::min<std::int64_t>(max_j, size_y - 1); ++j)
  {
    // 셀의 중심 좌표입니다(모서리가 아니라). 반지름 판정을 셀 중심으로 해야 반 셀씩
    // 어긋나지 않습니다.
    const double cell_y = origin_y + (static_cast<double>(j) + 0.5) * resolution;
    for (std::int64_t i = std::max<std::int64_t>(min_i, 0);
      i <= std::min<std::int64_t>(max_i, size_x - 1); ++i)
    {
      const double cell_x = origin_x + (static_cast<double>(i) + 0.5) * resolution;
      const double dx = cell_x - x;
      const double dy = cell_y - y;
      if (dx * dx + dy * dy > radius_sq) {
        continue;
      }
      ++out.inspected;
      if (map.data[static_cast<std::size_t>(j * size_x + i)] >= lethal_cost) {
        ++out.lethal;
      }
    }
  }
  return out;
}

}  // namespace hyper_planner
