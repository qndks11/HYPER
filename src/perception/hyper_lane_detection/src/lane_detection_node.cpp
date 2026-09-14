#include "hyper_lane_detection/lane_detection_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
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

// How often to repeat an image-write failure [ms]. A read-only or full save directory fails on
// every attempt, and even at the modest save rate that would bury the rest of the log.
constexpr int kImageSaveErrorThrottleMs = 5000;

// Default recording rate [Hz] for `~/image_saving`. Slow on purpose: consecutive frames at camera
// rate are near-duplicates, so a dataset gathered at 2 fps holds far more distinct views per
// gigabyte than the same disk spent on 30 fps.
constexpr double kDefaultImageSaveRateHz = 4.0;

// Default BEV geometry for the simulated camera, matching what hyper_control's parameters.yaml
// actually configures the Gazebo sensor with (70 deg HFOV, 5 deg of downward pitch, 0.113 m ahead
// of body_link's origin once camera_setback and the housing are taken off body_length/2). These
// are only defaults: the values are ROS parameters, and the numbers themselves live in
// parameters.yaml and vehicle.xacro, which is where they should be changed. They are duplicated
// here solely so the node still produces a metrically correct overlay when launched with no
// configuration at all.
//
// The vehicle carries a single camera -- the Logitech C920 -- feeding both this node and
// object_detection_node. It replaced a 153 deg ELP fisheye that was mounted lower and tilted
// twice as far down, and that swap is what sets the near edge below: a 70 deg lens 5 deg off
// horizontal simply cannot see the ground close in. See kSimFrontNearM.
//
// !! The height is the one number that is NOT a straight copy of parameters.yaml. Its
// camera_height (1.112) is the camera joint's z in *body_link*, whereas CameraExtrinsics::height_m
// is height above the *ground plane* -- and body_link rides 0.3 m up, because vehicle.xacro hangs
// the wheel joints at -wheel_radius/2 (-0.1) and the wheels have radius 0.2. Gazebo confirms it:
// the spawned model's world z is exactly 0.3000. So the camera is 1.112 + 0.3 = 1.412 m over the
// ground. Copying the 1.112 straight across would draw the whole overlay at 1.112/1.412 = 0.79x
// true distance, since dx scales linearly with height (see GroundProjection). Anything that
// changes wheel_radius changes this constant too.
constexpr double kSimFrontHorizontalFovRad = 1.2217305;
constexpr double kSimFrontCameraHeightM = 1.412;
constexpr double kSimFrontCameraPitchRad = 0.087;
constexpr double kSimFrontCameraOffsetM = 0.113;

// Nearest ground the BEV shows [m]. This is a property of the lens and the mount, not a
// preference: the bottom image row leaves the camera 26.5 deg below horizontal (atan(180 px /
// 457 px focal) + the 5 deg mount pitch), so it strikes the ground 1.412/tan(26.5 deg) = 2.83 m
// ahead in sim and 2.58 m on the car, whose camera sits 1.283 m up. Anything nearer than the
// larger of those is unsampled black rows, so this is set just beyond it and shared by both.
// Tilting the camera further down is what would buy back the near field.
constexpr double kSimFrontNearM = 2.9;
constexpr double kSimFrontFarM = 7.6;

// Half the lateral extent [m]. A 70 deg lens spans +/-d*tan(35 deg) at distance d, so the far row
// (7.6 m) is the widest ground the camera ever sees at +/-5.32 m; past that every column would be
// black. The old 9.0 came from the ELP's 153 deg fisheye, which genuinely saw that wide.
constexpr double kSimFrontHalfWidthM = 5.5;

// Unchanged across the camera swap, deliberately: the detectors' pixel-denominated constants
// (LaneDetector's kChainStepRadius, StoplineDetector's kMinStoplineAreaPx) are all scaled by
// this, so holding it fixed keeps them meaning the same physical distance they always did.
constexpr double kSimFrontMetersPerPixel = 0.028125;

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

  // The camera's BEV geometry. The defaults describe the simulated camera; the real vehicle
  // overrides the one field that genuinely differs (the height above ground, since the sim's
  // body_link rides higher than the real car's) -- see hyper_lane_detection/config/bev_real.yaml.
  BevSettings front_defaults;
  front_defaults.horizontal_fov_rad = kSimFrontHorizontalFovRad;
  front_defaults.extrinsics = CameraExtrinsics{
    kSimFrontCameraHeightM, kSimFrontCameraPitchRad, kSimFrontCameraOffsetM};
  front_defaults.region = GroundRegion{
    kSimFrontNearM, kSimFrontFarM, kSimFrontHalfWidthM, kSimFrontMetersPerPixel};
  front_bev_settings_ = declare_bev_settings("bev", front_defaults);

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

  // A plain sensor_msgs/Image subscription either way -- under ros_raw this is
  // Gazebo's bridged sim frame; under intra_process it's hyper_camera's
  // LogitechCameraPublisherNode component, loaded into the same ComposableNodeContainer as this
  // node (see hyper_object_detection's perception.launch.py), so the frame arrives by pointer
  // instead of over a serialized topic. Same callback either way.
  raw_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&LaneDetection::raw_image_callback, this, std::placeholders::_1));

  // ---- Dataset recording (see handle_image_saving) ----
  // Only the destination and the rate are parameters; whether recording is *running* is not, so
  // that it can only ever be turned on by an explicit call, never by a stale config file.
  // "images", not "data/lane_detection": the vehicle has a single camera, and these frames are
  // that camera's raw output -- nothing about them is lane-specific, and a lane-named folder
  // reads as if object detection needed its own copy.
  image_save_dir_ = declare_parameter<std::string>("image_save_dir", "images");
  const double image_save_rate_hz =
    declare_parameter<double>("image_save_rate", kDefaultImageSaveRateHz);
  if (image_save_rate_hz <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "image_save_rate must be > 0 (got %.3f); falling back to %.1f Hz",
      image_save_rate_hz, kDefaultImageSaveRateHz);
    image_save_period_s_ = 1.0 / kDefaultImageSaveRateHz;
  } else {
    image_save_period_s_ = 1.0 / image_save_rate_hz;
  }
  image_saving_service_ = create_service<std_srvs::srv::SetBool>(
    "~/image_saving",
    std::bind(
      &LaneDetection::handle_image_saving, this, std::placeholders::_1, std::placeholders::_2));

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

const GroundProjection * LaneDetection::projection_for(const cv::Size & source)
{
  auto & cached = front_projection_;
  auto & cached_source = front_projection_source_;
  const BevSettings & settings = front_bev_settings_;

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
      "Front camera has no usable ground projection for a %dx%d frame: %s -- frames dropped",
      source.width, source.height, error.c_str());
    cached.reset();
    return nullptr;
  }

  cached = std::move(projection);
  cached_source = source;
  RCLCPP_INFO(
    get_logger(), "Front camera BEV projection (%dx%d source): %s",
    source.width, source.height, cached->describe(source).c_str());
  return &cached.value();
}

void LaneDetection::publish_bev_cloud(
  const cv::Mat & view, const std_msgs::msg::Header & header,
  const GroundProjection & projection)
{
  const auto & publisher = bev_cloud_publisher_;
  if (!publisher || publisher->get_subscription_count() == 0) {
    return;
  }

  // Same scale and same origin that built the warp, so the isotropy this arithmetic assumes is a
  // property of the raster rather than a hope: GroundProjection lays the output out in meters
  // before any pixel is sampled.
  const double meters_per_pixel = projection.meters_per_pixel();
  const cv::Point2d origin = projection.origin_px();

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
      *iter_x = static_cast<float>((origin.y - row) * meters_per_pixel);
      *iter_y = static_cast<float>((origin.x - col) * meters_per_pixel);
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
  save_frame_if_due(cv_ptr->image);
  process_frame(cv_ptr->image, msg->header);
}

void LaneDetection::handle_image_saving(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock{image_save_mutex_};

  const std::filesystem::path directory{image_save_dir_};
  const std::string absolute = std::filesystem::absolute(directory).string();

  if (!request->data) {
    image_saving_enabled_ = false;
    response->success = true;
    response->message = "image saving off (" + absolute + ")";
    RCLCPP_INFO(get_logger(), "image saving off");
    return;
  }

  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error && !std::filesystem::is_directory(directory)) {
    // Reported as a failed service call rather than logged and forgotten: an operator who asked
    // for a recording and got "success" would go drive the course and come back to nothing.
    image_saving_enabled_ = false;
    response->success = false;
    response->message = "cannot create " + absolute + ": " + error.message();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Restart the clock so the first frame after switching on is saved immediately, rather than up
  // to a period later.
  last_image_save_time_.reset();
  image_saving_enabled_ = true;
  response->success = true;
  response->message = "image saving on at " + std::to_string(1.0 / image_save_period_s_) +
    " fps -> " + absolute;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void LaneDetection::save_frame_if_due(const cv::Mat & image)
{
  if (!image_saving_enabled_ || image.empty()) {
    return;
  }

  std::filesystem::path path;
  {
    std::lock_guard<std::mutex> lock{image_save_mutex_};
    // Re-checked under the lock: the service may have switched recording off between the atomic
    // read above and here, and the directory it named is only valid while it is on.
    if (!image_saving_enabled_) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    if (last_image_save_time_) {
      // Guards against a backwards jump too (a sim reset, or the first message after switching to
      // sim time): a negative elapsed would otherwise stall recording until the clock caught up.
      const double elapsed = (now - *last_image_save_time_).seconds();
      if (elapsed >= 0.0 && elapsed < image_save_period_s_) {
        return;
      }
    }
    last_image_save_time_ = now;

    // Wall clock in the filename, not the node clock: the name is for a human matching frames
    // against a run, and a sim clock starting at 0 would collide across every run.
    const auto wall = std::chrono::system_clock::now();
    const auto wall_time = std::chrono::system_clock::to_time_t(wall);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      wall.time_since_epoch()).count() % 1000;
    std::tm broken_down{};
    localtime_r(&wall_time, &broken_down);

    std::ostringstream name;
    // "rec_" marks a frame from a recording run, as opposed to the "shot_" single frames
    // image_saver_service drops into the same folder.
    name << "rec_" << std::put_time(&broken_down, "%Y%m%d_%H%M%S") << '_' << std::setfill('0')
         << std::setw(3) << millis << ".png";
    path = std::filesystem::path{image_save_dir_} / name.str();
  }

  // Written outside the lock: encoding a PNG is the expensive part of this function, and holding
  // the mutex across it would make the service call block for a frame's worth of compression.
  if (!cv::imwrite(path.string(), image)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kImageSaveErrorThrottleMs, "failed to write %s",
      path.string().c_str());
  }
}

void LaneDetection::process_frame(
  const cv::Mat & image, const std_msgs::msg::Header & header)
{
  const auto & bev_image_publisher = bev_image_publisher_;

  // The warp is the camera's own ground projection rather than a set of ROI corner ratios picked
  // per camera model: the same code path serves the sim and the real vehicle, and the difference
  // between them lives entirely in parameters.
  const GroundProjection * projection = projection_for(image.size());
  if (!projection) {
    return;  // reason already logged (throttled) by projection_for()
  }

  cv::Mat warped;
  cv::warpPerspective(image, warped, projection->homography(), projection->bev_size());

  publish_bev_cloud(warped, header, *projection);

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
