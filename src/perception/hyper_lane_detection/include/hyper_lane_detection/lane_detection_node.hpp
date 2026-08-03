#ifndef HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
#define HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "hyper_lane_detection/elp_camera_capture.hpp"
#include "hyper_lane_detection/input_backend.hpp"

class LaneDetection : public rclcpp::Node
{
public:
  LaneDetection();

private:
  enum class LaneSide { kLeft, kRight };

  /// Which physical camera a frame came from -- selects the publishers/window/fit parameters
  /// process_frame() runs it through. Orthogonal to InputBackend: the backend decides how a frame
  /// arrives (topic vs. direct device), CameraSide decides whose lane it is.
  enum class CameraSide { kFront, kRear };

  struct LaneFitResult
  {
    bool valid{false};
    double offset_m{0.0};
    double steering_angle_deg{0.0};
    double curvature_radius_px{0.0};
    std::vector<cv::Point> curve_points;  // the fitted line's two endpoints, for drawing
  };

  struct StoplineResult
  {
    bool valid{false};
    double distance_m{0.0};
    cv::Rect bounding_box;  // the matched contour's bounding box, for drawing
  };

  /**
   * @brief input_backend:=ros_compressed callback for the front camera (image_transport
   * "compressed" transport). Decodes via cv_bridge and hands off to process_frame().
   *
   * @param msg Incoming compressed-transport camera image.
   */
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /// input_backend:=ros_compressed callback for the rear camera -- see image_callback().
  void rear_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /**
   * @brief input_backend:=ros_raw callback for the front camera: a plain sensor_msgs/Image
   * subscription (no image_transport, no compressed transport). Gazebo's bridged camera image is
   * already the expected simulation input, so this does not rectify -- it decodes via cv_bridge
   * and hands off to process_frame() directly.
   *
   * @param msg Incoming raw camera image.
   */
  void raw_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /// input_backend:=ros_raw callback for the rear camera -- see raw_image_callback().
  void raw_rear_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /**
   * @brief input_backend:=direct_usb setup: opens the physical ELP camera via ElpCameraCapture
   * (device, resolution, framerate, and calibration file all read from ROS parameters) and starts
   * capture_timer_ polling it at the configured framerate. Front camera only -- direct_usb has no
   * rear-camera equivalent, so nothing here waits on or fails without one.
   *
   * @throws std::runtime_error if the camera device can't be opened or its calibration can't be
   * loaded; this is a startup failure and must not be swallowed into a degraded-but-running node.
   */
  void setup_direct_usb_capture();

  /// capture_timer_'s callback: pulls one rectified frame from elp_capture_ and, on success, hands
  /// it to process_frame(). A read failure is already logged inside ElpCameraCapture::read(), so
  /// this just skips the tick rather than treating a transient glitch as fatal.
  void capture_timer_callback();

  /**
   * @brief The full per-frame pipeline shared by every input backend and both cameras: bird's-eye
   * warp, lane and stop-line detection, message publish, and debug view. Factored out so the
   * algorithm itself never depends on how the frame arrived (ROS topic vs. direct device) or which
   * camera it's from -- `side` alone selects the publishers/window/fit parameters below.
   *
   * @param image Camera frame (BGR8) -- already rectified for the front camera under
   * input_backend:=direct_usb; passed through as-is for every other backend/camera combination
   * (ros_compressed's front/rear were already rectified upstream by hyper_camera's RectifyNode
   * before this refactor; ros_raw's simulated frames need no rectification at all).
   * @param stamp Capture timestamp of `image`. Threaded through for future header-stamped outputs
   * and latency logging; the current output messages (std_msgs/Float64MultiArray) carry no header
   * of their own to stamp.
   * @param side Which camera `image` is from -- selects between the front's kLaneCenterOffsetBiasM
   * / fit_lane() / front ROI and the rear's kRearParkingLineStandoffM / fit_lane_curve() / rear
   * ROI (see bird_eye()).
   */
  void process_frame(const cv::Mat & image, const rclcpp::Time & stamp, CameraSide side);

  /**
   * @brief Builds the perspective transform mapping the trapezoidal ROI (sampled from the
   * source image's own dimensions) to a bird's-eye view of the given destination size.
   *
   * @param src_height Source image height [px], used only to locate the ROI corners.
   * @param src_width Source image width [px], used only to locate the ROI corners.
   * @param dst_height Output (bird's-eye) image height [px].
   * @param dst_width Output (bird's-eye) image width [px].
   * @param use_rear_roi False: the front camera's ROI (kRoi*). True: the rear camera's ROI
   * (kRearRoi*), which reaches farther toward the horizon so the rear camera can pick up the
   * parking bay's side line earlier while backing in.
   * @return The 3x3 perspective transform matrix.
   */
  cv::Mat build_transform(
    int src_height, int src_width, int dst_height, int dst_width, bool use_rear_roi) const;

  /**
   * @brief Warps the input image to a bird's-eye view using build_transform().
   *
   * @param use_rear_roi False: the front camera's ROI/BEV height scale (kRoi* /kBevHeightScale).
   * True: the rear camera's (kRearRoi* /kRearBevHeightScale) -- farther-reaching and stretched
   * across more output pixels, for a clearer view of the (comparatively slow, close-range) parking
   * maneuver than the front's ordinary-road-speed settings need.
   */
  cv::Mat bird_eye(const cv::Mat & image, bool use_rear_roi) const;

  /**
   * @brief Produces a binary mask isolating yellow lane paint in HSV space.
   */
  cv::Mat yellow_mask(const cv::Mat & image) const;

  /**
   * @brief Produces a binary mask isolating white paint in HSV space (low saturation, high
   * value) -- unlike yellow_mask, which targets the lane paint's hue band instead, since stop
   * lines and lane lines are painted different colors in this course.
   */
  cv::Mat white_mask(const cv::Mat & image) const;

  /**
   * @brief Walks a chain of yellow pixels along one lane side, starting from its bottom point
   * and stepping outward one nearby pixel at a time.
   *
   * @details The seed point is the bottommost yellow pixel on the requested half of the image
   * (x >= origin.x for kRight, x < origin.x for kLeft), and the initial direction is straight up
   * the image (toward the horizon). Each step: pixels within kNeighborRadius of the current point
   * are filtered to those whose displacement from it has a non-negative dot product with the
   * current direction (i.e. within +/-90 degrees of it); if none remain, the chain stops. The
   * survivors are ranked by the signed sine of their angle to the current direction (cross
   * product over the product of norms, valid without abs() since candidates are already
   * restricted to +/-90 degrees): kRight walks most-positive-angle first, kLeft walks in the
   * opposite order, most-negative first -- with remaining ties broken by preferring the pixel
   * farthest from the current point. The winner becomes the new current point, and the vector
   * from the old point to it becomes the new direction, so the chain keeps following whatever
   * heading the lane is actually curving toward.
   *
   * Nothing stops this walk from drifting across the centerline as it follows a curving lane, so
   * a chain seeded on one side can legitimately end up on the other. When there's really only one
   * lane line straddling origin.x, though, both the left and right seeds can latch onto it and
   * walk out along essentially the same pixels -- the caller distinguishes a genuine crossing lane
   * from this spurious duplicate by checking that the chain still gets farther from the vehicle
   * from start to end (see image_callback).
   *
   * @param yellow_points Every yellow mask pixel coordinate (BEV), e.g. from cv::findNonZero.
   * @param origin The bottom-center point used to pick the seed (x >= origin.x for kRight,
   * x < origin.x for kLeft).
   * @param side Which half of the image to seed the chain from.
   * @return The walked chain's original BEV pixel coordinates, near (bottom) to far.
   */
  std::vector<cv::Point> walk_lane_chain(
    const std::vector<cv::Point> & yellow_points, const cv::Point2d & origin, LaneSide side) const;

  /**
   * @brief Fits a straight line through the walked lane chain by weighted total least squares,
   * pinned to the chain's near point, and derives the heading angle and lateral offset from it.
   *
   * @details The line is anchored exactly at the chain's first (near) point; its direction is
   * the eigenvector, of the weighted scatter matrix of the other chain points relative to that
   * anchor, with the largest eigenvalue -- the axis the points are most spread along, which
   * minimizes the weighted sum of squared perpendicular distances to the line. This total
   * least squares formulation (as opposed to regressing x on y or y on x) has no axis it breaks
   * down along, so it stays well-conditioned for a lane running in any direction, including
   * near-horizontal. A straight-line model has no curvature to derive, so curvature_radius_px is
   * always a large sentinel value. `center_bias_m` is applied with a sign matching `side`: the
   * target sits on the vehicle's side of a right line, but on the far side of a left line -- so
   * for the front camera's paired-lane assumption this places the target at the lane's center,
   * while for the rear camera's single parking-bay line it places the target at a fixed standoff
   * from it. See kLaneCenterOffsetBiasM / kRearParkingLineStandoffM at the call sites.
   *
   * @param points The lane chain from walk_lane_chain(), in BEV coordinates, near to far.
   * @param origin The bottom-center point the chain was anchored to.
   * @param width Bird's-eye image width [px], used to scale pixel offset to meters.
   * @param side Which lane line `points` traces, so the center-offset bias is signed correctly.
   * @param center_bias_m Distance from the detected line to the target, signed per `side`.
   * @return The lane fit result; `valid` is false if there are too few points.
   */
  LaneFitResult fit_lane(
    const std::vector<cv::Point> & points, const cv::Point2d & origin, int width,
    LaneSide side, double center_bias_m) const;

  /**
   * @brief Fits a quadratic curve (x' = a*y'^2 + b*y', in coordinates shifted so the chain's near
   * point is the origin) through the walked lane chain by weighted least squares, and derives the
   * heading angle, lateral offset, and curvature from its near-point tangent -- the rear-camera
   * counterpart to fit_lane()'s straight-line model, for lines that are genuinely curved (e.g. a
   * parking bay's entrance) rather than running straight, where a global straight-line direction
   * systematically mis-estimates heading/offset away from the near point.
   *
   * @details Anchored at the chain's near point p0 exactly (no free constant term), same as
   * fit_lane(). The 2x2 weighted normal-equations system for (a, b) uses the same per-point
   * near-field weighting as fit_lane(), solved via cv::solve(..., cv::DECOMP_SVD) rather than a
   * strict LU decomposition, so a near-singular system degrades gracefully instead of failing
   * outright. Requires more points than fit_lane() (kMinCurveFitPoints) since estimating a
   * curvature term needs more spread than estimating a single direction. steering_angle_deg and
   * offset_m are evaluated from the curve's tangent at the near point (f'(0) = b), using the same
   * formulas as fit_lane() so both fits are drop-in interchangeable for callers; unlike fit_lane(),
   * curvature_radius_px is a real value derived from the curve's second derivative (f''(0) = 2a)
   * rather than always the flat-line sentinel. curve_points samples the fitted parabola across the
   * chain's observed reach, for drawing, rather than fit_lane()'s two-point straight segment.
   *
   * @param points The lane chain from walk_lane_chain(), in BEV coordinates, near to far.
   * @param origin The bottom-center point the chain was anchored to.
   * @param width Bird's-eye image width [px], used to scale pixel offset to meters.
   * @param side Which lane line `points` traces, so the center-offset bias is signed correctly.
   * @param center_bias_m Distance from the detected line to the target, signed per `side` -- same
   * meaning as fit_lane()'s parameter of the same name.
   * @return The curve fit result; `valid` is false if there are too few points or the chain is too
   * straight/short for a stable curvature estimate.
   */
  LaneFitResult fit_lane_curve(
    const std::vector<cv::Point> & points, const cv::Point2d & origin, int width,
    LaneSide side, double center_bias_m) const;

  /**
   * @brief Finds the stop-line bar in a white mask, if any.
   *
   * @details Runs cv::findContours() on the mask and keeps only contours whose bounding box is
   * shaped like a stop-line bar: wide relative to its height (aspect ratio >=
   * kMinStoplineAspectRatio) and wide relative to the frame (spans at least
   * kMinStoplineWidthFraction of the image width). The width-fraction check is what rules out a
   * zebra crossing's individual stripes, which can pass the aspect-ratio test on their own but
   * don't individually span the lane the way an unbroken stop-line bar does. A shape match alone
   * isn't enough at a multi-lane intersection, though, since an adjacent lane's stop-line bar can
   * look identical -- candidates are also rejected if their horizontal center falls outside a
   * one-lane-wide band centered on `origin`'s x, so only a bar actually in the vehicle's own lane
   * survives. Among what's left, the one closest to the vehicle (largest row) is returned, since
   * that's the stop line that actually governs the vehicle's next stop.
   *
   * @param mask White paint mask (BEV, from white_mask()).
   * @param origin The bottom-center point representing the vehicle's position.
   * @param meters_per_pixel Scale factor from BEV pixels to meters.
   * @return The stop-line detection result; `valid` is false if nothing qualifies.
   */
  StoplineResult find_stopline(
    const cv::Mat & mask, const cv::Point2d & origin, double meters_per_pixel) const;

  hyper_lane_detection::InputBackend input_backend_{hyper_lane_detection::InputBackend::kRosCompressed};

  // input_backend:=ros_compressed only.
  image_transport::Subscriber image_subscriber_;
  image_transport::Subscriber rear_image_subscriber_;

  // input_backend:=ros_raw only.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_rear_image_subscriber_;

  // input_backend:=direct_usb only -- front camera exclusively, see setup_direct_usb_capture().
  std::unique_ptr<hyper_lane_detection::ElpCameraCapture> elp_capture_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_center_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stopline_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_lane_center_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_stopline_publisher_;

  std::vector<cv::Point> prev_left_points_;
  std::vector<cv::Point> prev_right_points_;
};

#endif  // HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
