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
// (about 0.3..7.6 m ahead, +/-9 m across) at about its true pixel scale.
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
  // -0.02 -> -0.15. 2 cm는 RViz가 코스트맵(alpha 0.4)과 이 오버레이(alpha 0.9)를 둘 다
  // 반투명으로 -- 즉 깊이 기록 없이 카메라 거리순으로 -- 그리기에는 너무 얇아서, 시점에
  // 따라 오버레이가 코스트맵 위로 올라오곤 했습니다. 15 cm면 어느 시점에서도 정렬이
  // 뒤집히지 않습니다. 지면보다 아래일 뿐 BEV의 x/y 기하는 그대로이므로 거리 해석에는
  // 영향이 없습니다.
  bev_cloud_z_m_ = declare_parameter<double>("bev_cloud_z_m", -0.15);

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

  // Rear camera: same deal as the front one under either backend -- Gazebo's bridged RGBD sensor
  // under ros_raw, realsense2_camera's D435i component sharing this container under intra_process.
  raw_rear_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/rear_image_raw", 10,
    std::bind(&LaneDetection::raw_rear_image_callback, this, std::placeholders::_1));

  rear_bev_image_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("/lane/rear_bev/image_raw", debug_image_qos);
  rear_bev_cloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("/lane/rear_bev/points", debug_image_qos);

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

  // Same scale and same origin that built the warp, so the isotropy this arithmetic assumes is a
  // property of the raster rather than a hope: GroundProjection lays the output out in meters
  // before any pixel is sampled.
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
  const auto & bev_image_publisher = is_rear ? rear_bev_image_publisher_ : bev_image_publisher_;

  // The warp is this camera's own ground projection rather than a set of ROI corner ratios picked
  // per camera model: the same code path serves the sim and the real vehicle, and the difference
  // between them lives entirely in parameters.
  const GroundProjection * projection = projection_for(side, image.size());
  if (!projection) {
    return;  // reason already logged (throttled) by projection_for()
  }

  cv::Mat warped;
  cv::warpPerspective(image, warped, projection->homography(), projection->bev_size());

  publish_bev_cloud(warped, header, *projection, is_rear);

  // The BEV view's only sink: a topic, read by RViz's Image display (or rqt_image_view). This
  // node deliberately opens no cv::imshow window of its own -- a GUI window pins the node to a
  // local X display, which the headless vehicle doesn't have. Published under the source frame's
  // own header so a viewer can line it up with the rest of the stack; skipped entirely when
  // nothing has subscribed.
  if (bev_image_publisher && bev_image_publisher->get_subscription_count() > 0) {
    bev_image_publisher->publish(
      *cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, warped).toImageMsg());
  }
}

// Registers LaneDetection as a loadable rclcpp component (see CMakeLists.txt's
// rclcpp_components_register_node) -- this also generates the standalone `lane_detection_node`
// executable used for input_backend:=ros_raw, alongside the ComposableNodeContainer path used for
// input_backend:=intra_process (see hyper_object_detection's perception.launch.py).
RCLCPP_COMPONENTS_REGISTER_NODE(LaneDetection)
