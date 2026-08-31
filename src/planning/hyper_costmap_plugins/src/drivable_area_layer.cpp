#include "hyper_costmap_plugins/drivable_area_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

using nav2_costmap_2d::FREE_SPACE;

namespace hyper_costmap_plugins
{

namespace
{
/// How often to repeat a transform/staleness complaint [ms]. Both fail every cycle once they
/// start failing, and at 5 Hz that buries everything else in the log.
constexpr int kThrottleMs = 5000;
}  // namespace

void DrivableAreaLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"DrivableAreaLayer: owning node has already been destroyed"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("cost_value", rclcpp::ParameterValue(static_cast<int>(cost_value_)));
  declareParameter("off_limits_cost", rclcpp::ParameterValue(static_cast<int>(off_limits_cost_)));
  declareParameter("mark_threshold", rclcpp::ParameterValue(static_cast<int>(mark_threshold_)));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(static_cast<int>(lethal_threshold_)));
  declareParameter("mark_step", rclcpp::ParameterValue(mark_step_));
  declareParameter("clear_step", rclcpp::ParameterValue(clear_step_));
  declareParameter("decay_step", rclcpp::ParameterValue(decay_step_));
  declareParameter("message_timeout", rclcpp::ParameterValue(message_timeout_));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".message_timeout", message_timeout_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".mark_step", mark_step_);
  node->get_parameter(name_ + ".clear_step", clear_step_);
  node->get_parameter(name_ + ".decay_step", decay_step_);

  int cost_value = cost_value_;
  node->get_parameter(name_ + ".cost_value", cost_value);
  // Clamped below NO_INFORMATION (255): that value means "unknown" everywhere else in the costmap,
  // so a layer that wrote it would be asking every consumer to misread a confident answer as the
  // absence of one.
  cost_value_ = static_cast<unsigned char>(std::clamp(cost_value, 1, 254));

  int off_limits_cost = off_limits_cost_;
  node->get_parameter(name_ + ".off_limits_cost", off_limits_cost);
  // Capped below INSCRIBED_INFLATED_OBSTACLE (253) rather than at 254: this level exists precisely
  // to say "not a collision, not inflated", and 253 is already a collision wherever
  // consider_footprint is off.
  off_limits_cost_ = static_cast<unsigned char>(std::clamp(off_limits_cost, 1, 252));

  int mark_threshold = mark_threshold_;
  node->get_parameter(name_ + ".mark_threshold", mark_threshold);
  mark_threshold_ = static_cast<int8_t>(std::clamp(mark_threshold, 1, 100));

  int lethal_threshold = lethal_threshold_;
  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold);
  // Never below mark_threshold: the two split the same axis, and inverting them would make every
  // marked cell lethal, silently undoing the distinction.
  lethal_threshold_ = static_cast<int8_t>(std::clamp(lethal_threshold, mark_threshold, 100));

  // FREE_SPACE, not the base class's NO_INFORMATION default. This map is a confidence
  // accumulator: "no evidence yet" has to be a value that arithmetic can climb out of and that
  // updateCosts() can recognise as "contribute nothing", and NO_INFORMATION is neither.
  default_value_ = FREE_SPACE;
  rolling_window_ = layered_costmap_->isRolling();
  matchSize();
  current_ = false;
  has_data_ = false;

  grid_subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, rclcpp::QoS(1),
    std::bind(&DrivableAreaLayer::grid_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "DrivableAreaLayer '%s' subscribed to %s (off_limits>=%d -> %u, lethal>=%d -> %u, "
    "mark/clear/decay=%d/%d/%d)",
    name_.c_str(), topic_.c_str(), mark_threshold_, off_limits_cost_, lethal_threshold_,
    cost_value_, mark_step_, clear_step_, decay_step_);
}

void DrivableAreaLayer::grid_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock{grid_mutex_};
  pending_grid_ = std::move(msg);
}

void DrivableAreaLayer::decay()
{
  const unsigned int cells = getSizeInCellsX() * getSizeInCellsY();
  for (unsigned int i = 0; i < cells; ++i) {
    unsigned char & cell = costmap_[i];
    if (cell == FREE_SPACE) {
      continue;
    }
    cell = (cell > decay_step_) ? static_cast<unsigned char>(cell - decay_step_) : FREE_SPACE;
  }
}

bool DrivableAreaLayer::integrate(const nav_msgs::msg::OccupancyGrid & grid)
{
  auto node = node_.lock();
  const rclcpp::Logger logger =
    node ? node->get_logger() : rclcpp::get_logger("DrivableAreaLayer");

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), grid.header.frame_id, grid.header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
      logger, "DrivableAreaLayer: cannot transform %s -> %s: %s",
      grid.header.frame_id.c_str(), layered_costmap_->getGlobalFrameID().c_str(), e.what());
    return false;
  }

  // Compose the two rigid transforms -- costmap <- message frame, and message frame <- grid
  // origin -- into one, then step it. Every cell center is
  //   origin_in_costmap + R(theta) * ((i + 0.5) * res, (j + 0.5) * res),
  // so walking i adds a constant vector and walking j adds another; no trigonometry per cell.
  const double tf_yaw = tf2::getYaw(transform.transform.rotation);
  const double theta = tf_yaw + tf2::getYaw(grid.info.origin.orientation);
  const double tf_cos = std::cos(tf_yaw);
  const double tf_sin = std::sin(tf_yaw);
  const double origin_x = transform.transform.translation.x +
    tf_cos * grid.info.origin.position.x - tf_sin * grid.info.origin.position.y;
  const double origin_y = transform.transform.translation.y +
    tf_sin * grid.info.origin.position.x + tf_cos * grid.info.origin.position.y;

  const double res = grid.info.resolution;
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);
  const double step_i_x = cos_t * res;
  const double step_i_y = sin_t * res;
  const double step_j_x = -sin_t * res;
  const double step_j_y = cos_t * res;

  // First cell's center: half a cell in along both grid axes.
  const double first_x = origin_x + 0.5 * (step_i_x + step_j_x);
  const double first_y = origin_y + 0.5 * (step_i_y + step_j_y);

  const unsigned int width = grid.info.width;
  const unsigned int height = grid.info.height;

  for (unsigned int j = 0; j < height; ++j) {
    double wx = first_x + step_j_x * static_cast<double>(j);
    double wy = first_y + step_j_y * static_cast<double>(j);
    const size_t row_base = static_cast<size_t>(j) * static_cast<size_t>(width);

    for (unsigned int i = 0; i < width; ++i, wx += step_i_x, wy += step_i_y) {
      const int8_t occupancy = grid.data[row_base + i];
      // Negative is "no data" -- the warp's un-sampled corners. It must neither mark nor clear,
      // or the black fringe of every frame would erase whatever the previous frames established.
      if (occupancy < 0) {
        continue;
      }

      unsigned int mx = 0;
      unsigned int my = 0;
      if (!worldToMap(wx, wy, mx, my)) {
        continue;  // outside the rolling window
      }

      unsigned char & cell = costmap_[getIndex(mx, my)];
      if (occupancy >= mark_threshold_) {
        // Which ceiling this cell climbs toward is a property of the observation, not of the
        // cell's history: a cell that saturated lethal and is now seen as off-limits road drops to
        // the lower ceiling on the spot, which is the same asymmetry clear_step_ encodes -- let go
        // of the strong claim fast, build it back slowly.
        const unsigned char ceiling =
          (occupancy >= lethal_threshold_) ? cost_value_ : off_limits_cost_;
        cell = static_cast<unsigned char>(
          std::min<int>(ceiling, static_cast<int>(cell) + mark_step_));
      } else {
        cell = (cell > clear_step_) ? static_cast<unsigned char>(cell - clear_step_) : FREE_SPACE;
      }
    }
  }

  return true;
}

void DrivableAreaLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // Must happen before anything reads costmap_, and regardless of `enabled`: the accumulator is
  // indexed in costmap cells, so if the window rolls without it the stored cells stay put in index
  // space and slide across the ground.
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    current_ = true;
    return;
  }

  decay();

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  {
    std::lock_guard<std::mutex> lock{grid_mutex_};
    grid = pending_grid_;
  }

  bool fresh = false;
  auto node = node_.lock();
  if (grid && node) {
    const double age = (node->now() - rclcpp::Time(grid->header.stamp)).seconds();
    if (age > message_timeout_) {
      // Deliberately not re-integrated. A dead camera node leaves its last frame sitting in the
      // subscription forever, and re-marking from it would freeze the belief in place instead of
      // letting decay() walk it back down to free.
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), kThrottleMs,
        "DrivableAreaLayer: %s is %.1f s stale (> %.1f) -- decaying instead of marking",
        topic_.c_str(), age, message_timeout_);
    } else if (integrate(*grid)) {
      has_data_ = true;
      fresh = true;
    }
  }

  // The whole extent, every cycle, because decay() touches every cell: a cell that faded this
  // cycle has to be inside the bounds or updateCosts() never gets the chance to stop contributing
  // it. The layer is the size of the local costmap (20x20 m at 5 cm = 160k cells), so this is a
  // couple of hundred microseconds, not a design compromise worth avoiding.
  touch(getOriginX(), getOriginY(), min_x, min_y, max_x, max_y);
  touch(
    getOriginX() + getSizeInMetersX(), getOriginY() + getSizeInMetersY(),
    min_x, min_y, max_x, max_y);

  // Inert (never fed) counts as current; see has_data_. Only a layer that was working and then
  // went quiet reports stale.
  current_ = !has_data_ || fresh;
}

void DrivableAreaLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_data_) {
    return;
  }

  // CostmapLayer::updateWithMax() would almost do, except for one behaviour that is wrong here:
  // where the master reads NO_INFORMATION it overwrites with this layer's value, so our FREE_SPACE
  // cells -- which mean "no evidence", not "observed clear" -- would erase the master's unknowns.
  // This layer only ever adds cost, never removes it.
  unsigned char * master = master_grid.getCharMap();

  for (int j = min_j; j < max_j; ++j) {
    unsigned int index = master_grid.getIndex(static_cast<unsigned int>(min_i),
        static_cast<unsigned int>(j));
    for (int i = min_i; i < max_i; ++i, ++index) {
      const unsigned char cost = costmap_[index];
      if (cost == FREE_SPACE) {
        continue;
      }
      if (master[index] == nav2_costmap_2d::NO_INFORMATION || master[index] < cost) {
        master[index] = cost;
      }
    }
  }
}

void DrivableAreaLayer::reset()
{
  resetMaps();
  {
    std::lock_guard<std::mutex> lock{grid_mutex_};
    pending_grid_.reset();
  }
  has_data_ = false;
  current_ = false;
}

}  // namespace hyper_costmap_plugins

PLUGINLIB_EXPORT_CLASS(hyper_costmap_plugins::DrivableAreaLayer, nav2_costmap_2d::Layer)
