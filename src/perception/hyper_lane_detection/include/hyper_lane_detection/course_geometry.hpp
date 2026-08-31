#ifndef HYPER_LANE_DETECTION__COURSE_GEOMETRY_HPP_
#define HYPER_LANE_DETECTION__COURSE_GEOMETRY_HPP_

namespace hyper_lane_detection
{

// Width of one driving lane on the course [m]. A property of the course itself, not of any
// camera or of the bird's-eye view -- which is the whole reason it lives here alone now. The
// header this replaced (bev_scale.hpp) also carried a "how many lanes fit across the BEV image"
// constant, and the product of the two *defined* the meters-per-pixel scale everything measured
// against. That made a course property masquerade as a camera calibration: the scale it produced
// had no connection to the homography the warp actually applied, and disagreed with it by ~24%
// laterally while being ~3% short longitudinally. The scale now comes from GroundProjection,
// derived from where the camera is, and this constant is back to meaning only what it says.
//
// 3.0 m is what the simulated course is actually built at -- see hyper_gazebo's driving_course
// model, whose build_course.py rasterizes at MPP=0.10 with lanes 30 px wide. The value used
// before was 3.7 m, a public-highway figure that no part of this course was ever drawn to.
inline constexpr double kLaneWidthMeters = 3.0;

}  // namespace hyper_lane_detection

#endif  // HYPER_LANE_DETECTION__COURSE_GEOMETRY_HPP_
