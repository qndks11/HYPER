#include "hyper_lane_detection/lane_detection_node.hpp"

#include <algorithm>
#include <stdexcept>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using hyper_lane_detection::CameraExtrinsics;
using hyper_lane_detection::CameraIntrinsics;
using hyper_lane_detection::GroundProjection;
using hyper_lane_detection::GroundRegion;
using hyper_lane_detection::InputBackend;
using hyper_lane_detection::parse_input_backend;
using hyper_lane_detection::to_string;

namespace
{
// Height [px] of the black strip appended below the BEV image to hold the overlay text. Fixed
// regardless of the incoming frame size so the 8 possible lines (6 top-anchored, 2 bottom-
// anchored) always have enough pitch between them -- the overlap this fixes came from anchoring
// text directly onto the BEV image, whose height depends on the configured ground region and can
// end up shorter than the text block itself.
constexpr int kTextPanelHeight = 360;

// How often to repeat the "this camera has no usable ground projection" complaint [ms]. A
// misconfigured region fails on every single frame, and at camera rate that would bury the log.
constexpr int kProjectionErrorThrottleMs = 5000;

// Default BEV geometry for the simulated front camera, matching what hyper_control's
// parameters.yaml actually configures the Gazebo sensor with (153 deg HFOV, 15 deg of downward
// pitch, 0.145 m ahead of body_link's origin once camera_setback and the housing are taken off
// body_length/2). These are only defaults: the values are ROS parameters, and the numbers
// themselves live in parameters.yaml and vehicle.xacro, which is where they should be changed.
// They are duplicated here solely so the node still produces a metrically correct overlay when
// launched with no configuration at all.
//
// !! The height is the one number that is NOT a straight copy of parameters.yaml. Its
// camera_height (1.2) is the camera joint's z in *body_link*, whereas CameraExtrinsics::height_m
// is height above the *ground plane* -- and body_link rides 0.3 m up, because vehicle.xacro hangs
// the wheel joints at -wheel_radius/2 (-0.1) and the wheels have radius 0.2. Gazebo confirms it:
// the spawned model's world z is exactly 0.3000. So the camera is 1.2 + 0.3 = 1.5 m over the
// ground. Copying the 1.2 straight across drew the whole overlay at 1.2/1.5 = 0.80x true
// distance, since dx scales linearly with height (see GroundProjection). Anything that changes
// wheel_radius changes this constant too.
//
// The ground region reproduces roughly the coverage the old hand-picked ROI happened to have
// (about 0.3..7.6 m ahead, +/-9 m across) at about its true pixel scale, deliberately: several
// detector constants downstream are still denominated in pixels (LaneDetector's
// kChainStepRadius, StoplineDetector's kMinStoplineAreaPx), so preserving the scale keeps their
// physical meaning while the geometry becomes correct. Narrowing half_width_m to something like
// 3 m would put far more pixels on the lane actually being followed, but it is a re-tuning of
// those constants, not part of fixing the scale.
constexpr double kSimFrontHorizontalFovRad = 2.67;
constexpr double kSimFrontCameraHeightM = 1.5;
constexpr double kSimFrontCameraPitchRad = 0.2617994;
constexpr double kSimFrontCameraOffsetM = 0.145;
constexpr double kSimFrontNearM = 0.3;
constexpr double kSimFrontFarM = 7.6;
constexpr double kSimFrontHalfWidthM = 9.0;
constexpr double kSimFrontMetersPerPixel = 0.028125;

// Default BEV geometry for the simulated rear camera (D435i-equivalent RGBD: 87 deg HFOV,
// 848x480, same 1.5 m / 15 deg mounting as the front -- see the front block on why that height is
// 1.5 and not parameters.yaml's 1.2). Unlike the front, this region is NOT a reproduction of the
// old one -- the old rear ROI reached 24 m backward with a scale that was 64% too wide laterally
// and 54% too short longitudinally at the same time, so there was no coherent tuning to preserve.
// It is sized for the job the rear camera has instead: the close, slow parking maneuver.
//
// near_m is 1.8 m because that is genuinely the closest ground this camera can see -- at 1.5 m
// up and only 15 deg of downward pitch, its bottom image row strikes the ground 1.74 m behind
// body_link. That blind zone is a mounting property, not something the warp can recover; a rear
// camera meant for parking wants to be lower, or pitched considerably further down.
constexpr double kSimRearHorizontalFovRad = 1.5184364;
constexpr double kSimRearCameraHeightM = 1.5;
constexpr double kSimRearCameraPitchRad = 0.2617994;
constexpr double kSimRearCameraOffsetM = 0.145;
constexpr double kSimRearNearM = 1.8;
constexpr double kSimRearFarM = 5.0;
constexpr double kSimRearHalfWidthM = 2.5;
constexpr double kSimRearMetersPerPixel = 0.01;

}  // namespace

LaneDetection::LaneDetection(const rclcpp::NodeOptions & options)
: Node{"lane_detection", options}
{
  const std::string backend_param =
    declare_parameter<std::string>("input_backend", "intra_process");
  const auto backend = parse_input_backend(backend_param);
  if (!backend) {
    RCLCPP_FATAL(
      get_logger(),
      "Invalid input_backend '%s' -- expected one of: intra_process, ros_raw",
      backend_param.c_str());
    throw std::invalid_argument("lane_detection_node: invalid input_backend '" + backend_param + "'");
  }
  input_backend_ = *backend;

  // Both cameras' BEV geometry. The defaults describe the simulated cameras; a real vehicle
  // overrides them (notably with explicit fx/fy/cx/cy, since a rectified real lens has neither a
  // centered principal point nor equal focal lengths -- see hyper_camera's ELP calibration).
  BevSettings front_defaults;
  front_defaults.horizontal_fov_rad = kSimFrontHorizontalFovRad;
  front_defaults.extrinsics = CameraExtrinsics{
    kSimFrontCameraHeightM, kSimFrontCameraPitchRad, kSimFrontCameraOffsetM};
  front_defaults.region = GroundRegion{
    kSimFrontNearM, kSimFrontFarM, kSimFrontHalfWidthM, kSimFrontMetersPerPixel};
  front_bev_settings_ = declare_bev_settings("bev", front_defaults);

  BevSettings rear_defaults;
  rear_defaults.horizontal_fov_rad = kSimRearHorizontalFovRad;
  rear_defaults.extrinsics = CameraExtrinsics{
    kSimRearCameraHeightM, kSimRearCameraPitchRad, kSimRearCameraOffsetM};
  rear_defaults.region = GroundRegion{
    kSimRearNearM, kSimRearFarM, kSimRearHalfWidthM, kSimRearMetersPerPixel};
  rear_bev_settings_ = declare_bev_settings("bev_rear", rear_defaults);

  bev_cloud_frame_id_ = declare_parameter<std::string>("bev_cloud_frame_id", "body_link");
  bev_cloud_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("bev_cloud_stride", 2)));
  bev_cloud_z_m_ = declare_parameter<double>("bev_cloud_z_m", 0.02);

  lane_center_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);
  stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/detection", 10);
  rear_lane_center_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/lane/rear_center", 10);
  rear_stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/rear_detection", 10);

  // Depth 1 + best-effort (the usual image-stream profile): a debug view is only ever worth
  // showing at its newest frame, and a slow/absent viewer (RViz on another machine) must never
  // push back on the detection pipeline the way a reliable, queued image topic would. Note this
  // means an RViz Image display left on "System Default" reliability (= Reliable) will not match
  // this publisher and shows nothing -- set its Reliability Policy to Best Effort.
  const auto debug_image_qos = rclcpp::SensorDataQoS().keep_last(1);
  bev_image_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("/lane/bev/image_raw", debug_image_qos);
  bev_cloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("/lane/bev/points", debug_image_qos);

  // Front camera: a plain sensor_msgs/Image subscription either way -- under ros_raw this is
  // Gazebo's bridged sim frame; under intra_process it's hyper_camera's ElpCameraPublisherNode
  // component, loaded into the same ComposableNodeContainer as this node (see
  // hyper_object_detection's perception.launch.py), so the frame arrives by pointer instead of
  // over a serialized topic. Same callback either way.
  raw_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&LaneDetection::raw_image_callback, this, std::placeholders::_1));

  // Rear camera: only ros_raw ever has one -- intra_process (real vehicle) has no rear camera
  // yet, so skip both the subscription and the debug topic that would otherwise never publish.
  if (input_backend_ == InputBackend::kRosRaw) {
    raw_rear_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      "/rear_image_raw", 10,
      std::bind(&LaneDetection::raw_rear_image_callback, this, std::placeholders::_1));

    rear_bev_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("/lane/rear_bev/image_raw", debug_image_qos);
    rear_bev_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/lane/rear_bev/points", debug_image_qos);
  }

  RCLCPP_INFO(
    get_logger(), "LaneDetection started (input_backend=%s, bev_cloud_frame_id=%s)",
    to_string(input_backend_).c_str(), bev_cloud_frame_id_.c_str());
}

LaneDetection::BevSettings LaneDetection::declare_bev_settings(
  const std::string & prefix, const BevSettings & defaults)
{
  BevSettings settings;

  // Intrinsics, two ways. horizontal_fov is enough for an ideal centered pinhole with square
  // pixels (Gazebo); fx/fy/cx/cy are needed for anything else, and win when fx is set.
  settings.horizontal_fov_rad =
    declare_parameter<double>(prefix + ".horizontal_fov", defaults.horizontal_fov_rad);
  settings.intrinsics.fx = declare_parameter<double>(prefix + ".fx", defaults.intrinsics.fx);
  settings.intrinsics.fy = declare_parameter<double>(prefix + ".fy", defaults.intrinsics.fy);
  settings.intrinsics.cx = declare_parameter<double>(prefix + ".cx", defaults.intrinsics.cx);
  settings.intrinsics.cy = declare_parameter<double>(prefix + ".cy", defaults.intrinsics.cy);

  settings.extrinsics.height_m =
    declare_parameter<double>(prefix + ".camera_height", defaults.extrinsics.height_m);
  settings.extrinsics.pitch_rad =
    declare_parameter<double>(prefix + ".camera_pitch", defaults.extrinsics.pitch_rad);
  settings.extrinsics.longitudinal_offset_m = declare_parameter<double>(
    prefix + ".camera_longitudinal_offset", defaults.extrinsics.longitudinal_offset_m);

  settings.region.near_m = declare_parameter<double>(prefix + ".near", defaults.region.near_m);
  settings.region.far_m = declare_parameter<double>(prefix + ".far", defaults.region.far_m);
  settings.region.half_width_m =
    declare_parameter<double>(prefix + ".half_width", defaults.region.half_width_m);
  settings.region.meters_per_pixel =
    declare_parameter<double>(prefix + ".meters_per_pixel", defaults.region.meters_per_pixel);

  return settings;
}

const GroundProjection * LaneDetection::projection_for(CameraSide side, const cv::Size & source)
{
  const bool is_rear = side == CameraSide::kRear;
  auto & cached = is_rear ? rear_projection_ : front_projection_;
  auto & cached_source = is_rear ? rear_projection_source_ : front_projection_source_;
  const BevSettings & settings = is_rear ? rear_bev_settings_ : front_bev_settings_;

  if (cached && cached_source == source) {
    return &cached.value();
  }

  // Explicit intrinsics win when given; otherwise derive an ideal pinhole from the FOV and this
  // frame's own dimensions, which is why this cannot happen before the first frame arrives.
  CameraIntrinsics intrinsics = settings.intrinsics;
  if (!intrinsics.is_set()) {
    intrinsics = CameraIntrinsics::from_horizontal_fov(settings.horizontal_fov_rad, source);
  }

  std::string error;
  auto projection = GroundProjection::create(intrinsics, settings.extrinsics, settings.region,
      error);
  if (!projection) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kProjectionErrorThrottleMs,
      "%s camera has no usable ground projection for a %dx%d frame: %s -- frames dropped",
      is_rear ? "Rear" : "Front", source.width, source.height, error.c_str());
    cached.reset();
    return nullptr;
  }

  cached = std::move(projection);
  cached_source = source;
  RCLCPP_INFO(
    get_logger(), "%s camera BEV projection (%dx%d source): %s", is_rear ? "Rear" : "Front",
    source.width, source.height, cached->describe(source).c_str());
  return &cached.value();
}

void LaneDetection::publish_bev_cloud(
  const cv::Mat & view, const std_msgs::msg::Header & header,
  const GroundProjection & projection, bool is_rear)
{
  const auto & publisher = is_rear ? rear_bev_cloud_publisher_ : bev_cloud_publisher_;
  if (!publisher || publisher->get_subscription_count() == 0) {
    return;
  }

  // Same scale and same origin the detectors measure their offsets/distances against, so the
  // overlay and the numbers can never disagree. Both now come from the projection that built the
  // warp, so the isotropy this arithmetic assumes is a property of the raster rather than a hope:
  // GroundProjection lays the output out in meters before any pixel is sampled.
  const double meters_per_pixel = projection.meters_per_pixel();
  const cv::Point2d origin = projection.origin_px();
  // Rear camera: the top of its BEV is *behind* the vehicle, and image-left is the vehicle's
  // right -- both axes flip, i.e. a 180 deg rotation about z.
  const double facing = is_rear ? -1.0 : 1.0;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = bev_cloud_frame_id_;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  const int rows = (view.rows + bev_cloud_stride_ - 1) / bev_cloud_stride_;
  const int cols = (view.cols + bev_cloud_stride_ - 1) / bev_cloud_stride_;
  modifier.resize(static_cast<size_t>(rows) * static_cast<size_t>(cols));

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud, "rgb");

  size_t point_count = 0;
  for (int row = 0; row < view.rows; row += bev_cloud_stride_) {
    const cv::Vec3b * view_row = view.ptr<cv::Vec3b>(row);
    for (int col = 0; col < view.cols; col += bev_cloud_stride_) {
      const cv::Vec3b & bgr = view_row[col];
      // Un-sampled warp corners -- "no data", not black ground. See publish_bev_cloud()'s docs.
      if (bgr[0] == 0 && bgr[1] == 0 && bgr[2] == 0) {
        continue;
      }
      *iter_x = static_cast<float>((origin.y - row) * meters_per_pixel * facing);
      *iter_y = static_cast<float>((origin.x - col) * meters_per_pixel * facing);
      *iter_z = static_cast<float>(bev_cloud_z_m_);
      // PointCloud2's packed "rgb" float is byte-ordered b, g, r -- the same order cv::Vec3b
      // already holds a BGR pixel in, so this copies straight across.
      iter_rgb[0] = bgr[0];
      iter_rgb[1] = bgr[1];
      iter_rgb[2] = bgr[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
      ++point_count;
    }
  }

  // Shrink to what actually survived the black-pixel skip; the iterators are done with by now.
  modifier.resize(point_count);
  publisher->publish(cloud);
}

void LaneDetection::raw_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  process_frame(cv_ptr->image, msg->header, CameraSide::kFront);
}

void LaneDetection::raw_rear_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception (rear): %s", e.what());
    return;
  }
  process_frame(cv_ptr->image, msg->header, CameraSide::kRear);
}

void LaneDetection::process_frame(
  const cv::Mat & image, const std_msgs::msg::Header & header, CameraSide side)
{
  const bool is_rear = side == CameraSide::kRear;
  const auto & lane_publisher = is_rear ? rear_lane_center_publisher_ : lane_center_publisher_;
  const auto & stopline_publisher = is_rear ? rear_stopline_publisher_ : stopline_publisher_;
  const auto & bev_image_publisher = is_rear ? rear_bev_image_publisher_ : bev_image_publisher_;

  // The warp is now this camera's own ground projection rather than a set of ROI corner ratios
  // picked per camera model: the same code path serves the sim and the real vehicle, and the
  // difference between them lives entirely in parameters.
  const GroundProjection * projection = projection_for(side, image.size());
  if (!projection) {
    return;  // reason already logged (throttled) by projection_for()
  }

  cv::Mat warped;
  cv::warpPerspective(image, warped, projection->homography(), projection->bev_size());
  const cv::Point2d origin = projection->origin_px();
  const double meters_per_pixel = projection->meters_per_pixel();

  const cv::Mat yellow = lane_detector_.yellow_mask(warped);
  const cv::Mat white = stopline_detector_.white_mask(warped);

  // Single shared debug view: both masks are highlighted first (different colors so the two
  // don't read as one blob), before either detector's annotations are drawn on top -- so a mask
  // fill never overwrites an annotation drawn earlier.
  cv::Mat view = warped.clone();
  view.setTo(cv::Scalar(0, 255, 0), yellow);
  view.setTo(cv::Scalar(180, 180, 180), white);

  const auto lane_result =
    lane_detector_.detect_and_draw(yellow, origin, meters_per_pixel, is_rear, view);
  std_msgs::msg::Float64MultiArray lane_msg;
  lane_msg.data = {
    lane_result.left.steering_angle_deg, lane_result.left.offset_m,
    lane_result.left.valid ? 1.0 : 0.0, lane_result.right.steering_angle_deg,
    lane_result.right.offset_m, lane_result.right.valid ? 1.0 : 0.0};
  lane_publisher->publish(lane_msg);

  const auto stopline_result =
    stopline_detector_.detect_and_draw(white, origin, meters_per_pixel, view);
  std_msgs::msg::Float64MultiArray stopline_msg;
  stopline_msg.data = {stopline_result.distance_m, stopline_result.valid ? 1.0 : 0.0};
  stopline_publisher->publish(stopline_msg);

  // Ground overlay first: it must not carry the text panel appended just below, which is a
  // readout rather than anything that exists on the ground.
  publish_bev_cloud(view, header, *projection, is_rear);

  // Dedicated black strip below the BEV image for all overlay text, sized independently of the
  // BEV image's own (config-dependent) height so the lines below never run out of room.
  cv::copyMakeBorder(
    view, view, 0, kTextPanelHeight, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  const int panel_top = view.rows - kTextPanelHeight;

  lane_detector_.draw_text(lane_result, view, panel_top);
  stopline_detector_.draw_text(stopline_result, view, panel_top);

  // The annotated view's only sink: a topic, read by RViz's Image display (or rqt_image_view).
  // This node deliberately opens no cv::imshow window of its own -- a GUI window pins the node to
  // a local X display, which the headless vehicle doesn't have, and makes the whole node crash on
  // toolkit mismatches that have nothing to do with lane detection. Published under the source
  // frame's own header so a viewer can line it up with the rest of the stack; skipped entirely
  // when nothing has subscribed.
  if (bev_image_publisher && bev_image_publisher->get_subscription_count() > 0) {
    bev_image_publisher->publish(
      *cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, view).toImageMsg());
  }
}

// Registers LaneDetection as a loadable rclcpp component (see CMakeLists.txt's
// rclcpp_components_register_node) -- this also generates the standalone `lane_detection_node`
// executable used for input_backend:=ros_raw, alongside the ComposableNodeContainer path used for
// input_backend:=intra_process (see hyper_object_detection's perception.launch.py).
RCLCPP_COMPONENTS_REGISTER_NODE(LaneDetection)
