#ifndef HYPER_LANE_DETECTION__BEV_SCALE_HPP_
#define HYPER_LANE_DETECTION__BEV_SCALE_HPP_

namespace hyper_lane_detection
{

// Shared by LaneDetector's and StoplineDetector's meters-per-pixel scaling, both derived from the
// same bird's-eye-view width the node's bird_eye() produces. Kept in one place deliberately: an
// earlier version of this package split lane and stop-line detection into two files
// (lane_detection.cpp / stopline_detection.cpp) and duplicated these two constants across them
// before being unified into one file -- splitting the detectors again without sharing this
// constant pair would silently reintroduce that exact fork.
inline constexpr double kLaneWidthMeters = 3.7;
inline constexpr double kNumLaneInScreen = 6.2;  // how many lanes fit across the BEV image width

}  // namespace hyper_lane_detection

#endif  // HYPER_LANE_DETECTION__BEV_SCALE_HPP_
