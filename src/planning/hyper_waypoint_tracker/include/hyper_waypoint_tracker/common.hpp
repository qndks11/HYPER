#pragma once

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>

namespace hyper_waypoint_tracker
{

inline constexpr double kPi = 3.1415926535897932384626433832795;
inline constexpr double kEarthRadiusM = 6371000.0;

inline double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

inline double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline double ramp(double current, double target, double up_step, double down_step)
{
  if (target > current) {
    return std::min(target, current + up_step);
  }
  return std::max(target, current - down_step);
}

inline double rate_limit(double current, double target, double max_step)
{
  return clamp(target, current - max_step, current + max_step);
}

inline double haversine_m(double lat1, double lon1, double lat2, double lon2)
{
  const double rad = kPi / 180.0;
  const double phi1 = lat1 * rad;
  const double phi2 = lat2 * rad;
  const double dphi = (lat2 - lat1) * rad;
  const double dlambda = (lon2 - lon1) * rad;
  const double sin_phi = std::sin(0.5 * dphi);
  const double sin_lambda = std::sin(0.5 * dlambda);
  const double a = sin_phi * sin_phi +
    std::cos(phi1) * std::cos(phi2) * sin_lambda * sin_lambda;
  return 2.0 * kEarthRadiusM * std::atan2(
    std::sqrt(std::max(0.0, a)),
    std::sqrt(std::max(0.0, 1.0 - a)));
}

struct LocalXY {double x{0.0}; double y{0.0};};

// Small-scale equirectangular projection: meters East/North of (lat0, lon0).
// Only valid for course-sized areas (well under a few km) -- not a general map projection.
inline LocalXY equirect_local_xy(double lat0, double lon0, double lat, double lon)
{
  const double rad = kPi / 180.0;
  const double x_east = kEarthRadiusM * (lon - lon0) * rad * std::cos(lat0 * rad);
  const double y_north = kEarthRadiusM * (lat - lat0) * rad;
  return {x_east, y_north};
}

}  // namespace hyper_waypoint_tracker
