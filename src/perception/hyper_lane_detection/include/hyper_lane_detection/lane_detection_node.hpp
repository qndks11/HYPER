#ifndef HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
#define HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "hyper_lane_detection/drivable_area.hpp"
#include "hyper_lane_detection/ground_projection.hpp"
#include "hyper_lane_detection/input_backend.hpp"

class LaneDetection : public rclcpp::Node
{
public:
  explicit LaneDetection(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Camera callback: a plain sensor_msgs/Image subscription, shared by both
   * backends. Under ros_raw this is Gazebo's bridged (already-rectified-equivalent) sim frame;
   * under intra_process it's hyper_camera's ElpCameraPublisherNode component, loaded into the
   * same container as this node so the frame arrives by pointer rather than over a serialized
   * topic -- either way this callback just decodes via cv_bridge and hands off to
   * process_frame().
   *
   * @param msg Incoming raw camera image.
   */
  void raw_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /**
   * @brief The full per-frame pipeline shared by every input backend: bird's-eye warp, then
   * publish -- as a debug image and as a ground-projected point cloud. No lane or stop-line
   * detection runs here any more; this node produces the BEV view and nothing else. Factored out
   * so the warp never depends on which backend fed the topic this frame arrived on.
   *
   * @param image Camera frame (BGR8) -- already rectified under input_backend:=intra_process
   * (rectified upstream by hyper_camera's ElpCameraPublisherNode); passed through as-is under
   * ros_raw (the simulated frames need no rectification at all).
   * @param header Header of the frame `image` was decoded from. Copied onto the published debug
   * image and cloud so RViz gets the capture timestamp.
   */
  void process_frame(const cv::Mat & image, const std_msgs::msg::Header & header);

  /**
   * @brief Republishes the annotated bird's-eye view as a colored PointCloud2 laid flat on the
   * ground plane, so it can be seen *in the 3D scene* (under the costmap, paths and footprint)
   * rather than only as a 2D image panel. Each surviving BEV pixel becomes one point at
   * bev_cloud_z_m_ (negative -- just *below* the ground plane, so RViz sorts it behind the
   * costmap instead of over it), positioned by the same scale and origin
   * (`projection`) the lane/stop-line distance outputs already use -- so if this overlay lands
   * visibly wrong against the costmap, that same error is in the published offsets/distances.
   * That is exactly how the previous scale error surfaced: the overlay read about a quarter too
   * wide against the costmap because the offsets it shared a constant with were too.
   *
   * Pure-black pixels are dropped rather than published black: warpPerspective leaves the corners
   * it could not sample from the source frame at zero, and those are "no data", not dark ground --
   * publishing them would paint an opaque black rectangle over the costmap underneath.
   *
   * @param view The warped BEV image (BGR8).
   * @param header Header of the source frame; the stamp is kept, the frame_id replaced with
   * bev_cloud_frame_id_.
   * @param projection The camera's ground projection -- supplies the metric scale and the
   * vehicle-frame origin's position within the BEV raster.
   */
  void publish_bev_cloud(
    const cv::Mat & view, const std_msgs::msg::Header & header,
    const hyper_lane_detection::GroundProjection & projection);

  /**
   * @brief Classifies the front camera's BEV into drivable / undrivable ground and publishes the
   * result as a nav_msgs/OccupancyGrid in bev_cloud_frame_id_, for nav2's DrivableAreaLayer to
   * fold into the local costmap.
   *
   * @details The grid is cropped out of the BEV first (see drivable_max_range_m_ /
   * drivable_max_lateral_m_) and only then classified. Cropping first is not just cheaper: the
   * configured half_width of 9 m puts the BEV's outer columns far past anything the source frame
   * actually sampled well, and feeding that extrapolated fringe to the reachability flood fill
   * lets it decide the road connects to things it does not.
   *
   * Published in the *vehicle* frame with an identity orientation, leaving the transform into the
   * costmap's rolling odom frame to the costmap layer, which has the tf buffer and the message
   * stamp to do it at the right time. Publishing pre-transformed here would bake in this node's
   * idea of "now".
   *
   * @param view The warped BEV (BGR8).
   * @param header Header of the source frame; stamp kept, frame_id replaced.
   * @param projection The front camera's ground projection, for the metric scale and origin.
   */
  void publish_drivable_area(
    const cv::Mat & view, const std_msgs::msg::Header & header,
    const hyper_lane_detection::GroundProjection & projection);

  /**
   * @brief The ground projection for the camera at one source-frame size, building it on first
   * use and rebuilding it if that size ever changes.
   *
   * @details Deferred rather than built in the constructor because the sim path derives its
   * intrinsics from a horizontal FOV plus the frame's own dimensions, and the node does not know
   * those dimensions until a frame arrives. Caching on cv::Size means a camera that is restarted
   * at a different resolution re-derives its homography instead of silently warping with the old
   * one -- the class of drift this whole mechanism replaced.
   *
   * @param source Dimensions of the frame the homography must warp.
   * @return The projection, or nullptr if the configured geometry does not describe a visible
   * ground patch (the reason is logged; the frame is then dropped rather than warped by a
   * meaningless transform).
   */
  const hyper_lane_detection::GroundProjection * projection_for(const cv::Size & source);

  /// The camera's BEV geometry as configured, before it is resolved against a frame size.
  /// Intrinsics arrive one of two ways: `horizontal_fov_rad` (> 0) derives an ideal centered
  /// pinhole from the frame's own dimensions, which is exactly what Gazebo renders; otherwise
  /// `intrinsics` must be filled in explicitly, which is what a real rectified lens needs since
  /// its principal point is not the image center and its two focal lengths are not equal.
  struct BevSettings
  {
    double horizontal_fov_rad{0.0};
    hyper_lane_detection::CameraIntrinsics intrinsics;
    hyper_lane_detection::CameraExtrinsics extrinsics;
    hyper_lane_detection::GroundRegion region;
  };

  /// Declares the camera's BEV parameters under `prefix` and returns them -- the geometry is
  /// described entirely by parameters rather than by hand-tuned constants in the source.
  BevSettings declare_bev_settings(const std::string & prefix, const BevSettings & defaults);

  hyper_lane_detection::InputBackend input_backend_{
    hyper_lane_detection::InputBackend::kIntraProcess};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_subscriber_;

  /// The bird's-eye debug view, as a sensor_msgs/Image. This is the node's only debug
  /// output -- it opens no OpenCV window, so the view is watchable in RViz or over the network
  /// and the node never depends on a local X display.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bev_image_publisher_;

  /// The same view again, ground-projected -- see publish_bev_cloud(). Separate from the image
  /// publishers because the two answer different questions: the image is "what does the camera
  /// see from above", the cloud is "where does that sit relative to the costmap and the planned
  /// path".
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bev_cloud_publisher_;

  /// Vehicle frame the ground overlay is published in. Its origin's position inside the BEV is
  /// computed by GroundProjection::origin_px() rather than assumed to be the image's
  /// bottom-center -- it normally sits below the last row, because a forward-looking camera
  /// cannot see the ground at its own feet. Defaults to body_link, which is what this stack
  /// actually names its base frame (ekf's base_link_frame, nav2's robot_base_frame), not the
  /// ROS-conventional base_link.
  std::string bev_cloud_frame_id_{"body_link"};

  /// Publish every Nth BEV pixel in each axis. 1 is every pixel (a 640x260 BEV is then 166k
  /// points per frame); the default trades resolution the map view doesn't need for a quarter of
  /// the bandwidth.
  int bev_cloud_stride_{2};

  /// Offset [m] along z, in bev_cloud_frame_id_, to sink the overlay below the ground plane.
  /// Nonzero purely to stop it z-fighting with the costmap, which RViz also draws at z = 0; it is
  /// *negative* so RViz sorts the overlay behind the costmap, the paths and the footprint rather
  /// than painting over them. body_link is itself at z = 0 here -- both EKFs run two_d_mode --
  /// so this is the overlay's height above the costmap plane directly.
  double bev_cloud_z_m_{-0.15};

  /// Publishes the drivable-area classification (see publish_drivable_area) and, separately, the
  /// same classification tinted over the BEV for a human. The grid is latched-style reliable
  /// rather than best-effort like the debug streams: a costmap layer that silently drops frames
  /// degrades in a way nobody notices until the vehicle plans through a hedge.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr drivable_grid_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drivable_image_publisher_;

  hyper_lane_detection::DrivableAreaDetector drivable_detector_;

  /// Master switch. Off by default: this node's existing outputs are debug views that cost
  /// nothing when unsubscribed, whereas this one feeds the costmap and therefore the controller,
  /// so turning it on is a decision the launch file should make explicitly.
  bool drivable_enabled_{false};

  /// The BEV crop published as a grid, in meters ahead of and to either side of body_link.
  /// The longitudinal default matches the local costmap's obstacle_max_range (6.0 m) so the
  /// camera and the lidar agree on how far ahead the costmap is allowed to believe anything.
  double drivable_max_range_m_{6.0};
  double drivable_max_lateral_m_{4.0};

  /// The camera's configured BEV geometry, and the projection built from it once a frame size is
  /// known. See projection_for().
  BevSettings front_bev_settings_;
  std::optional<hyper_lane_detection::GroundProjection> front_projection_;
  cv::Size front_projection_source_;
};

#endif  // HYPER_LANE_DETECTION__LANE_DETECTION_NODE_HPP_
