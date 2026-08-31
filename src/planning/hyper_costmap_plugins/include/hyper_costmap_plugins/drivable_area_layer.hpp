#ifndef HYPER_COSTMAP_PLUGINS__DRIVABLE_AREA_LAYER_HPP_
#define HYPER_COSTMAP_PLUGINS__DRIVABLE_AREA_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hyper_costmap_plugins
{

/**
 * @class DrivableAreaLayer
 * @brief Folds a vehicle-relative drivable-area classification (a nav_msgs/OccupancyGrid, as
 * published by hyper_lane_detection) into a nav2 costmap.
 *
 * @details Why this exists rather than feeding a PointCloud2 into the stock ObstacleLayer: that
 * layer's `scan` source runs with `clearing: true` and a 20 m raytrace range, and a lidar sees
 * straight over flat ground. Every ray that crosses a patch of grass the camera just marked
 * raytrace-clears it again on the same cycle. Lifting the points to a fake obstacle height only
 * moves the problem -- the lidar then legitimately clears them, because nothing is there. Paint on
 * the ground is not an obstacle, and the layer that reasons about it cannot be the one built for
 * obstacles.
 *
 * The layer keeps its own costmap as a **confidence accumulator** rather than writing what the
 * latest frame said. Marking raises a cell toward `cost_value`, observing it as drivable pulls it
 * down fast, and every cell decays slowly whether observed or not. A single-frame color
 * misclassification -- glare, a shadow edge, one bad warp row -- therefore cannot put a lethal
 * cell in front of the vehicle; it takes a few consistent frames. Because the accumulator lives in
 * the costmap's own (rolling, odom) frame, that filtering happens in the frame where a ground cell
 * holds still, instead of in the vehicle frame where 2.22 m/s smears it 1.5 cells per frame.
 *
 * The message carries two strengths of "do not drive here" and the layer keeps them apart, because
 * they are not the same instruction:
 *
 * - At or above `lethal_threshold` (hyper_lane_detection's kUndrivable) the cell is ground that is
 *   not road -- grass, dirt, run-off. That saturates at `cost_value`, LETHAL_OBSTACLE by default.
 *   MPPI's ObstaclesCritic then calls it a collision (with `consider_footprint: true` a collision
 *   is exactly "the footprint covers a 254 cell"), so a trajectory leaving the course is discarded
 *   rather than penalised, and InflationLayer propagates from it -- which it does only from LETHAL
 *   cells.
 * - Between `mark_threshold` and `lethal_threshold` (kOffLimits) the cell is road the vehicle is
 *   not allowed to use: the lane paint itself, and the asphalt the paint cuts off. That saturates
 *   at `off_limits_cost`, deliberately below INSCRIBED_INFLATED_OBSTACLE (253), so it never reads
 *   as a collision and never inflates. Crossing a painted line is a rule, not an impact, and a
 *   centre line wearing a 1 m inflation halo would close the lane the vehicle is driving in.
 *   MPPI still avoids it, through the distance it reads back out of any non-lethal cost:
 *   `d = inscribed_radius + ln(253 / cost) / cost_scaling_factor`, about 0.48 m at 200.
 *
 * The accumulator is what makes the lethal half safe to assert: `cost_value` needs five
 * consecutive marking observations of the same cell, and the levels on the way up are sub-lethal.
 * Confidence builds as "avoid this" and only then becomes "wall".
 *
 * The cost of the lethal half: where every sampled trajectory would have to leave the surface,
 * MPPI has no valid control and controller_server aborts after `failure_tolerance`. Setting
 * `cost_value` to 200 as well makes going off-course a strong preference again rather than a
 * collision.
 */
class DrivableAreaLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  DrivableAreaLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  /// True: a clear_costmap service call should be able to wipe this layer's accumulator. Its
  /// whole content is a belief about the world, and clearing exists precisely to drop those.
  bool isClearable() override {return true;}

private:
  /// Stores the incoming grid for the next updateBounds() rather than integrating it here. The
  /// subscription runs on an executor thread; the accumulator is only safe to touch on the
  /// costmap's own update cycle.
  void grid_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /**
   * @brief Integrates one classification grid into the accumulator.
   *
   * @details Walks the message's cells, transforms each into the costmap frame and applies the
   * per-class step. The transform is composed once and then stepped incrementally down each row
   * and column -- a message is up to 260x230 cells and this runs on the costmap thread, so it is
   * worth not doing trigonometry per cell.
   *
   * @return false if the message could not be transformed (the reason is logged, throttled); the
   * accumulator is left alone and simply decays for that cycle.
   */
  bool integrate(const nav_msgs::msg::OccupancyGrid & grid);

  /// Pulls every non-free cell one `decay_step_` toward FREE_SPACE. Runs every cycle, before
  /// integration, so a marked cell that keeps being seen holds its level (mark_step_ exceeds
  /// decay_step_) while one that has left the field of view fades instead of persisting forever
  /// behind the vehicle, where nothing will ever observe it again to clear it.
  void decay();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_subscription_;

  std::mutex grid_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr pending_grid_;

  std::string topic_{"/lane/drivable_area"};

  /// Cost a fully-confident *undrivable* cell saturates at -- not road at all. LETHAL by default;
  /// see the class docs.
  unsigned char cost_value_{nav2_costmap_2d::LETHAL_OBSTACLE};

  /// Cost a fully-confident *off-limits* cell saturates at -- road the vehicle may not use. Held
  /// below INSCRIBED_INFLATED_OBSTACLE (253) so it is never a collision and never inflates; it
  /// acts only through MPPI's cost-to-distance reading, ~0.48 m of standoff at 200.
  unsigned char off_limits_cost_{200};

  /// Occupancy value at or above which a message cell counts as undrivable rather than merely
  /// off-limits. Matches hyper_lane_detection::kUndrivable.
  int8_t lethal_threshold_{100};

  /// Occupancy value at or above which a message cell is marked at all. Cells at exactly 0 are
  /// positive evidence of drivable ground and clear; anything negative is "no data" and is skipped
  /// entirely, so the warp's black corners neither mark nor clear.
  int8_t mark_threshold_{50};

  /// Per-observation steps, in cost units. Clearing is deliberately faster than marking (and both
  /// faster than decay): being wrong about drivable ground stops the vehicle, being wrong about an
  /// obstacle only makes it swerve, so the belief should be quick to let go and slow to commit.
  int mark_step_{60};
  int clear_step_{120};
  int decay_step_{10};

  /// How stale a message may be before it is ignored [s]. Without this the layer would keep
  /// re-integrating the last frame from a camera node that died, holding its belief alive forever.
  double message_timeout_{1.0};

  /// tf lookup slack [s], matching the controller's own transform_tolerance.
  double transform_tolerance_{0.2};

  /// Cached from LayeredCostmap so updateBounds() can roll the accumulator's origin with the
  /// master grid; without that the accumulated cells stay at fixed indices and slide across the
  /// ground as the window moves.
  bool rolling_window_{false};

  /// True once at least one grid has been integrated.
  ///
  /// Also what separates "inert" from "stale" for isCurrent(). A layer that has never been fed is
  /// switched off, not broken -- if it reported not-current, listing it in `plugins` while the
  /// camera side is disabled would make the entire costmap read as stale to everything that asks.
  /// Only a layer that was being fed and then stopped is genuinely not current.
  bool has_data_{false};
};

}  // namespace hyper_costmap_plugins

#endif  // HYPER_COSTMAP_PLUGINS__DRIVABLE_AREA_LAYER_HPP_
