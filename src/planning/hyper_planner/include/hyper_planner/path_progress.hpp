#pragma once

// 우리가 보낸 경로 위에서 차량이 어디까지 왔는지를 세는 코드.
// mission_manager_node의 prearm과 cancel-on-arrival이 이 값으로 판단합니다.
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
// 여기에는 tf도 로그도 없습니다 -- 차량 좌표는 호출자가 넣어 주고, 이 클래스는 순수하게
// 기하만 셉니다.

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <nav_msgs/msg/path.hpp>

namespace hyper_planner
{

class PathProgress
{
public:
  // 경로를 보낼 때 각 인덱스에서 끝까지 남은 길이를 미리 재 둡니다(tick마다 다시 세지 않게).
  void reset(const nav_msgs::msg::Path & path)
  {
    path_ = path;
    cursor_ = 0;
    valid_ = false;
    stop_index_ = 0;
    distance_to_stop_m_ = 0.0;
    stop_point_distance_m_ = 0.0;

    suffix_len_.assign(path_.poses.size(), 0.0);
    if (path_.poses.size() < 2) {
      return;
    }
    for (std::size_t i = path_.poses.size() - 2; ; --i) {
      const auto & a = path_.poses[i].pose.position;
      const auto & b = path_.poses[i + 1].pose.position;
      suffix_len_[i] = suffix_len_[i + 1] + std::hypot(b.x - a.x, b.y - a.y);
      if (i == 0) {
        break;
      }
    }
  }

  // 보낸 경로 위에서 "여기서 서야 한다"는 지점을 정합니다. reset() 다음에 부릅니다.
  //
  // 좌표로 최근접점을 찾지 않는 이유: 주차 진입/출차처럼 같은 길을 되짚는 구간에서는
  // 라벨 좌표 근처를 경로가 두 번 지나므로 엉뚱한 인덱스에 붙을 수 있습니다. 대신
  // "경로 끝에서 남은 길이"로 찾습니다 -- trim/lead-in은 경로 앞쪽만 건드리고, resample은
  // 점을 다시 깔 뿐 꼭짓점을 옮기지 않으므로 꼬리 길이는 tail_after_label_m 그대로입니다.
  //
  // stop_x/stop_y는 그 지점의 실제 좌표입니다(= 라벨의 웨이포인트). 커서와 별개로 "지금
  // 정지점 옆에 있는가"를 재는 데 씁니다.
  void set_stop_point(double tail_after_label_m, double stop_x, double stop_y)
  {
    stop_index_ = path_.poses.empty() ? 0 : path_.poses.size() - 1;
    for (std::size_t i = 0; i < suffix_len_.size(); ++i) {
      if (suffix_len_[i] <= tail_after_label_m) {
        stop_index_ = i;
        break;
      }
    }
    stop_x_ = stop_x;
    stop_y_ = stop_y;
  }

  // 커서는 앞으로만 움직입니다. 그래서 코스가 자기 자신 근처로 돌아오는 구간에서도, 차가
  // 경로에서 잠깐 벗어나도 진행도가 뒤로 튀지 않습니다.
  // search_window_m: 커서에서 앞으로 이만큼(경로 길이)만 최근접점을 찾습니다.
  bool update(double robot_x, double robot_y, double search_window_m)
  {
    valid_ = false;
    if (path_.poses.size() < 2) {
      return false;
    }

    std::size_t best = cursor_;
    double best_distance = std::numeric_limits<double>::max();
    double scanned = 0.0;
    for (std::size_t i = cursor_; i < path_.poses.size(); ++i) {
      const auto & point = path_.poses[i].pose.position;
      const double d = std::hypot(point.x - robot_x, point.y - robot_y);
      if (d < best_distance) {
        best_distance = d;
        best = i;
      }
      if (i + 1 >= path_.poses.size()) {
        break;
      }
      const auto & next = path_.poses[i + 1].pose.position;
      scanned += std::hypot(next.x - point.x, next.y - point.y);
      if (scanned > search_window_m) {
        break;
      }
    }
    cursor_ = best;

    // 정지점은 경로 끝보다 앞에 있을 수 있습니다(decel 프로파일). 커서가 정지점을
    // 지나가면 음수가 되고, 그 부호가 cancel-on-arrival의 하드 백스톱이 됩니다.
    distance_to_stop_m_ = suffix_len_[cursor_] - suffix_len_[stop_index_];
    stop_point_distance_m_ = std::hypot(stop_x_ - robot_x, stop_y_ - robot_y);

    valid_ = true;
    return true;
  }

  // 차량 좌표를 못 구했을 때. 마지막 거리들은 그대로 두고 "못 믿는다"만 표시합니다.
  void invalidate() {valid_ = false;}

  const nav_msgs::msg::Path & path() const {return path_;}
  bool valid() const {return valid_;}
  double distance_to_stop_m() const {return distance_to_stop_m_;}
  double stop_point_distance_m() const {return stop_point_distance_m_;}
  std::size_t stop_index() const {return stop_index_;}
  // 정지점에서 경로 끝까지 남은 길이(= 라벨 뒤로 더 달리는 꼬리의 길이). 로그용입니다.
  double stop_tail_m() const
  {
    return stop_index_ < suffix_len_.size() ? suffix_len_[stop_index_] : 0.0;
  }

private:
  nav_msgs::msg::Path path_;
  std::vector<double> suffix_len_;   // 각 인덱스에서 경로 끝까지 남은 길이
  std::size_t cursor_{0};            // 앞으로만 움직입니다
  bool valid_{false};

  // "여기서 서야 한다"는 지점(= 스텝의 라벨). decel 프로파일을 안 쓰면 경로 끝과 같습니다.
  std::size_t stop_index_{0};        // 보낸 경로 위에서의 인덱스
  double stop_x_{0.0};
  double stop_y_{0.0};
  double distance_to_stop_m_{0.0};   // 커서에서 정지점까지 (지나가면 음수)
  double stop_point_distance_m_{0.0};   // 차량에서 정지점까지 직선거리
};

}  // namespace hyper_planner
