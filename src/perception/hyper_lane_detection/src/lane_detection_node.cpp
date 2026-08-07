#include "hyper_lane_detection/lane_detection_node.hpp"

#include <chrono>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

using hyper_lane_detection::ElpCameraCapture;
using hyper_lane_detection::InputBackend;
using hyper_lane_detection::parse_input_backend;
using hyper_lane_detection::to_string;

namespace
{
// One camera's IPM (inverse perspective mapping) setup: a trapezoidal ROI, as (row_ratio,
// col_ratio) of the source image, warped via getPerspectiveTransform() to a bird's-eye view whose
// output height is `bev_height_scale` times the source frame height (width left unscaled). Bottom
// corner ratios outside [0, 1] flare past the frame's own edges to capture more near-field width
// than the frame alone shows at that row -- but warpPerspective can only sample pixels that
// actually exist in the source frame, so the wider that flare, the more of bird_eye()'s output is
// unavoidably black (see its corner triangles).
//
// A homography like this implicitly assumes a specific camera FOV/lens-distortion/mounting
// geometry -- it's tuned by picking corners that happen to trace a flat rectangle on the ground
// for *one particular camera*, not derived from first principles. That assumption does not carry
// over between cameras with different geometry, so every distinct camera the node can see through
// gets its own RoiConfig instead of sharing one: front vs. rear (mounting position/pitch differ)
// *and* real vs. sim (the real ELP is a ~170 deg fisheye rectified down to a much narrower
// rectilinear projection -- see ELP-USBGS1200P01-KL170.yaml's P vs. K -- while Gazebo's camera is
// a plain rectilinear model at its own 153 deg HFOV and 640x400 16:10 resolution, vs. the real
// camera's 1280x720 16:9). Reusing one camera's numbers for another only coincidentally lines up.
struct RoiConfig
{
  double top_left_row, top_left_col;
  double top_right_row, top_right_col;
  double bottom_left_row, bottom_left_col;
  double bottom_right_row, bottom_right_col;
  double bev_height_scale;
};

// Real ELP front camera (input_backend direct_usb). Tune against actual ELP footage.
constexpr RoiConfig kRealFrontRoi{0.48, 0.35, 0.48, 0.65, 1, -2, 1, 3, 0.5};

// Real rear camera equivalent -- no physical rear camera exists yet, kept for when one is added.
// Reaches farther/closer to the horizon than the front (top row 0.48 -> 0.30, pulling in ground
// closer to the vehicle) and is stretched across more output rows (height scale 0.5 -> 0.8) for
// finer pixel resolution during the (comparatively slow, close-range) parking maneuver than the
// front's ordinary-road-speed settings need. Top col ratios and the bottom flare are left matching
// the front's, since the concern here is depth, not width. Tune against actual rear footage.
constexpr RoiConfig kRealRearRoi{0.30, 0.35, 0.30, 0.65, 1, -2, 1, 3, 0.8};

// Gazebo sim front camera (input_backend ros_raw). Copied from kRealFrontRoi as a starting point
// only -- the real ELP and the sim camera do not share a lens model, FOV, or resolution (see this
// struct's own doc comment above), so a homography tuned for one has no reason to fit the other.
// TUNE against actual Gazebo footage before trusting this for anything but "doesn't crash".
constexpr RoiConfig kSimFrontRoi{0.48, 0.35, 0.48, 0.65, 1, -2, 1, 3, 0.5};

// Gazebo sim rear camera (input_backend ros_raw) -- same caveat as kSimFrontRoi, copied from
// kRealRearRoi as an untuned starting point.
constexpr RoiConfig kSimRearRoi{0.30, 0.35, 0.30, 0.65, 1, -2, 1, 3, 0.8};

/// Picks the RoiConfig matching this frame's camera side and camera model. See RoiConfig's own
/// doc comment for why these four aren't just one shared set of constants.
const RoiConfig & select_roi_config(bool use_rear_roi, bool use_sim_roi)
{
  if (use_rear_roi) {
    return use_sim_roi ? kSimRearRoi : kRealRearRoi;
  }
  return use_sim_roi ? kSimFrontRoi : kRealFrontRoi;
}

constexpr int kWindowWidth = 420;
constexpr int kWindowHeight = 300;

// Height [px] of the black strip appended below the BEV image to hold the overlay text. Fixed
// regardless of the incoming frame size so the 8 possible lines (6 top-anchored, 2 bottom-
// anchored) always have enough pitch between them -- the overlap this fixes came from anchoring
// text directly onto the BEV image, whose height shrinks with the active RoiConfig's
// bev_height_scale and the source frame size, and can end up shorter than the text block itself.
constexpr int kTextPanelHeight = 360;

// Row margin [px] pushing the vehicle's reference point (origin, in process_frame) below the
// BEV frame's bottom edge, rather than pinning it exactly to the last visible row. The camera's
// visible bottom row isn't necessarily where the vehicle physically sits (e.g. a hood/bumper gap
// the camera doesn't capture), so this treats that row as the zero point and moves the true
// reference point this many pixels closer to the vehicle (i.e. "behind" the frame).
constexpr double kOriginBelowFrameMarginPx = 0.0;

}  // namespace

LaneDetection::LaneDetection() : Node{"lane_detection"}
{
  const std::string backend_param =
    declare_parameter<std::string>("input_backend", "direct_usb");
  const auto backend = parse_input_backend(backend_param);
  if (!backend) {
    RCLCPP_FATAL(
      get_logger(),
      "Invalid input_backend '%s' -- expected one of: direct_usb, ros_raw",
      backend_param.c_str());
    throw std::invalid_argument("lane_detection_node: invalid input_backend '" + backend_param + "'");
  }
  input_backend_ = *backend;

  lane_center_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/lane/center", 10);
  stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/detection", 10);
  rear_lane_center_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/lane/rear_center", 10);
  rear_stopline_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/stopline/rear_detection", 10);

  // Only ros_raw is ever the Gazebo sim; direct_usb is always the real ELP camera. Selects which
  // of the four RoiConfig instances (see the anonymous namespace above) bird_eye() warps against
  // for each side.
  const bool is_sim = input_backend_ == InputBackend::kRosRaw;

  cv::namedWindow("LaneDetection", cv::WINDOW_NORMAL);
  cv::resizeWindow(
    "LaneDetection", kWindowWidth,
    static_cast<int>(kWindowHeight * select_roi_config(false, is_sim).bev_height_scale) +
      kTextPanelHeight);

  switch (input_backend_) {
    case InputBackend::kRosRaw:
      // Gazebo simulation: ros_gz_bridge already publishes plain sensor_msgs/Image, so no
      // image_transport/compressed subscription is needed here at all.
      raw_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&LaneDetection::raw_image_callback, this, std::placeholders::_1));
      raw_rear_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/rear_image_raw", 10,
        std::bind(&LaneDetection::raw_rear_image_callback, this, std::placeholders::_1));
      break;

    case InputBackend::kDirectUsb:
      // Real vehicle: this node owns the physical ELP camera directly (see
      // setup_direct_usb_capture()) -- no ROS image topic involved for the front camera, and no
      // rear-camera equivalent.
      setup_direct_usb_capture();
      break;
  }

  // The rear window only ever receives frames under ros_raw -- direct_usb has no rear-camera
  // path, so skip creating a window that would otherwise just sit blank.
  if (input_backend_ != InputBackend::kDirectUsb) {
    cv::namedWindow("LaneDetectionRear", cv::WINDOW_NORMAL);
    cv::resizeWindow(
      "LaneDetectionRear", kWindowWidth,
      static_cast<int>(kWindowHeight * select_roi_config(true, is_sim).bev_height_scale) +
        kTextPanelHeight);
  }

  RCLCPP_INFO(
    get_logger(), "LaneDetection started (input_backend=%s)", to_string(input_backend_).c_str());
}

void LaneDetection::setup_direct_usb_capture()
{
  ElpCameraCapture::Config config;
  config.device = declare_parameter<std::string>("video_device", "/dev/video_elp");
  config.width = declare_parameter<int>("image_width", 1280);
  config.height = declare_parameter<int>("image_height", 720);
  config.framerate = declare_parameter<double>("framerate", 30.0);
  config.calibration_file = declare_parameter<std::string>(
    "calibration_file",
    ament_index_cpp::get_package_share_directory("hyper_camera") +
      "/config/ELP-USBGS1200P01-KL170.yaml");

  elp_capture_ = std::make_unique<ElpCameraCapture>(get_logger());
  if (!elp_capture_->open(config)) {
    // ElpCameraCapture::open() has already logged the specific failure; a node that "starts" with
    // no working camera would otherwise spin forever silently doing nothing.
    throw std::runtime_error("lane_detection_node: direct_usb camera setup failed");
  }

  const auto period_s = std::chrono::duration<double>(1.0 / config.framerate);
  capture_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period_s),
    std::bind(&LaneDetection::capture_timer_callback, this));
}

void LaneDetection::capture_timer_callback()
{
  cv::Mat rectified;
  if (!elp_capture_->read(rectified)) {
    // Read failure is already logged inside ElpCameraCapture::read(); skip this tick rather than
    // aborting the node over what may be a transient USB glitch.
    return;
  }
  process_frame(rectified, now(), CameraSide::kFront);
}

cv::Mat LaneDetection::build_transform(
  int src_height, int src_width, int dst_height, int dst_width, bool use_rear_roi,
  bool use_sim_roi) const
{
  const RoiConfig & roi = select_roi_config(use_rear_roi, use_sim_roi);

  const std::vector<cv::Point2f> src{
    {static_cast<float>(src_width * roi.top_left_col),
     static_cast<float>(src_height * roi.top_left_row)},
    {static_cast<float>(src_width * roi.top_right_col),
     static_cast<float>(src_height * roi.top_right_row)},
    {static_cast<float>(src_width * roi.bottom_right_col),
     static_cast<float>(src_height * roi.bottom_right_row)},
    {static_cast<float>(src_width * roi.bottom_left_col),
     static_cast<float>(src_height * roi.bottom_left_row)}};

  const std::vector<cv::Point2f> dst{
    {0.0f, 0.0f},
    {static_cast<float>(dst_width), 0.0f},
    {static_cast<float>(dst_width), static_cast<float>(dst_height)},
    {0.0f, static_cast<float>(dst_height)}};

  return cv::getPerspectiveTransform(src, dst);
}

cv::Mat LaneDetection::bird_eye(const cv::Mat & image, bool use_rear_roi, bool use_sim_roi) const
{
  const double height_scale = select_roi_config(use_rear_roi, use_sim_roi).bev_height_scale;
  const cv::Size dst_size(image.cols, static_cast<int>(image.rows * height_scale));
  const cv::Mat transform = build_transform(
    image.rows, image.cols, dst_size.height, dst_size.width, use_rear_roi, use_sim_roi);
  cv::Mat warped;
  cv::warpPerspective(image, warped, transform, dst_size);
  return warped;
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
  process_frame(cv_ptr->image, msg->header.stamp, CameraSide::kFront);
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
  process_frame(cv_ptr->image, msg->header.stamp, CameraSide::kRear);
}

void LaneDetection::process_frame(
  const cv::Mat & image, const rclcpp::Time & stamp, CameraSide side)
{
  // Reserved for future header-stamped outputs / latency logging -- the current output messages
  // (std_msgs/Float64MultiArray) carry no header of their own to stamp.
  (void)stamp;

  const bool is_rear = side == CameraSide::kRear;
  // Only ros_raw is ever the Gazebo sim; direct_usb is always the real ELP camera -- see
  // RoiConfig's doc comment for why the two need separate ROI/BEV-scale constants.
  const bool is_sim = input_backend_ == InputBackend::kRosRaw;
  const auto & lane_publisher = is_rear ? rear_lane_center_publisher_ : lane_center_publisher_;
  const auto & stopline_publisher = is_rear ? rear_stopline_publisher_ : stopline_publisher_;
  const std::string window_name = is_rear ? "LaneDetectionRear" : "LaneDetection";

  const cv::Mat warped = bird_eye(image, /*use_rear_roi=*/is_rear, is_sim);
  const cv::Point2d origin(warped.cols / 2.0, warped.rows - 1.0 + kOriginBelowFrameMarginPx);

  const cv::Mat yellow = lane_detector_.yellow_mask(warped);
  const cv::Mat white = stopline_detector_.white_mask(warped);

  // Single shared debug view: both masks are highlighted first (different colors so the two
  // don't read as one blob), before either detector's annotations are drawn on top -- so a mask
  // fill never overwrites an annotation drawn earlier.
  cv::Mat view = warped.clone();
  view.setTo(cv::Scalar(0, 255, 0), yellow);
  view.setTo(cv::Scalar(180, 180, 180), white);

  const auto lane_result =
    lane_detector_.detect_and_draw(yellow, origin, warped.cols, is_rear, view);
  std_msgs::msg::Float64MultiArray lane_msg;
  lane_msg.data = {
    lane_result.left.steering_angle_deg, lane_result.left.offset_m,
    lane_result.left.valid ? 1.0 : 0.0, lane_result.right.steering_angle_deg,
    lane_result.right.offset_m, lane_result.right.valid ? 1.0 : 0.0};
  lane_publisher->publish(lane_msg);

  const auto stopline_result = stopline_detector_.detect_and_draw(white, origin, view);
  std_msgs::msg::Float64MultiArray stopline_msg;
  stopline_msg.data = {stopline_result.distance_m, stopline_result.valid ? 1.0 : 0.0};
  stopline_publisher->publish(stopline_msg);

  // Dedicated black strip below the BEV image for all overlay text, sized independently of the
  // BEV image's own (config-dependent) height so the lines below never run out of room.
  cv::copyMakeBorder(
    view, view, 0, kTextPanelHeight, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  const int panel_top = view.rows - kTextPanelHeight;

  lane_detector_.draw_text(lane_result, view, panel_top);
  stopline_detector_.draw_text(stopline_result, view, panel_top);

  cv::imshow(window_name, view);
  cv::waitKey(1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LaneDetection>());
  } catch (const std::exception & e) {
    // Startup failures (bad input_backend, direct_usb camera/calibration failure) throw rather
    // than leaving a half-initialized node spinning; surface them clearly instead of an opaque
    // uncaught-exception crash.
    RCLCPP_FATAL(rclcpp::get_logger("lane_detection"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
