#pragma once

// 등감속(decel) 프로파일을 /speed_limit(nav2_msgs/SpeedLimit)으로 내보내는 코드.
// 왜 MPPI에 맡기지 않고 우리가 속도를 깎는지는 mission_manager_node.cpp 머리의
// "decel 프로파일" 주석을 보십시오.

#include <cmath>
#include <string>
#include <utility>

#include <nav2_msgs/msg/speed_limit.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

namespace hyper_planner
{

// 정지점까지 남은 거리에서 등감속 속도를 냅니다. 0.0 = 제한 없음(NO_SPEED_LIMIT).
//
//     v = sqrt(2 * a * (정지점까지 남은 거리 - cancel_on_arrival_m))
//
// cancel_on_arrival_m를 빼는 이유: 그래야 취소 지점에서 속도가 정확히 하한까지 내려와
// cancel-on-arrival의 속도 조건이 열립니다. 안 빼면 프로파일이 정지점에서야 0이 되는데,
// 그 전에는 계속 속도 조건에 걸려 취소가 안 되고 결국 백스톱까지 밀립니다.
inline double decel_profile_speed(
  double accel, double distance_to_stop_m, double cancel_on_arrival_m, double controller_vx_max,
  double min_speed)
{
  const double braking = std::max(0.0, distance_to_stop_m - cancel_on_arrival_m);
  const double profile = std::sqrt(2.0 * accel * braking);
  // vx_max 이상이면 제한할 게 없습니다. 그대로 실어 보내면 MPPI의 setSpeedLimit이
  // ratio > 1로 오히려 vx_max를 올리므로 반드시 해제(0.0)해야 합니다.
  if (profile >= controller_vx_max) {
    return 0.0;
  }
  return std::max(profile, min_speed);
}

// 값이 실제로 바뀔 때만 내보냅니다(해제 -> 해제는 아무것도 안 보냄).
class SpeedLimitPublisher
{
public:
  SpeedLimitPublisher(
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher, std::string frame_id)
  : publisher_(std::move(publisher)), frame_id_(std::move(frame_id))
  {
  }

  // limit 0.0 = NO_SPEED_LIMIT(해제).
  void publish(double limit, const rclcpp::Time & stamp)
  {
    if (std::fabs(limit - published_) < 1e-3) {
      return;
    }
    published_ = limit;

    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.percentage = false;
    msg.speed_limit = limit;
    publisher_->publish(msg);
  }

  // 마지막으로 내보낸 값. 로그용입니다.
  double last() const {return published_;}

private:
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher_;
  std::string frame_id_;
  double published_{0.0};
};

}  // namespace hyper_planner
