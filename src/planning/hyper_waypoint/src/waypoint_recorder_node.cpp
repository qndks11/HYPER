#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace
{
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class WaypointRecorder : public rclcpp::Node
{
public:
  WaypointRecorder() : Node("waypoint_recorder")
  {
    output_csv_ = declare_parameter<std::string>("output_csv", "waypoint_record.csv");
    min_spacing_m_ = declare_parameter<double>("min_spacing_m", 0.5);

    csv_.open(output_csv_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open '%s' for writing.", output_csv_.c_str());
      return;
    }
    csv_ << std::fixed << std::setprecision(8);
    csv_ << "idx,stamp_sec,x,y,yaw,frame_id,gps_lat,gps_lon,gps_status,gps_odom_x,gps_odom_y,"
            "cov_xx,cov_yy,odom_vx,odom_vy,odom_vyaw,imu_wz,imu_ax,imu_ay,imu_yaw,"
            "odom_x,odom_y,odom_yaw\n";
    csv_.flush();

    diag_log_path_ = output_csv_ + ".diag.log";
    diag_log_.open(diag_log_path_, std::ios::out | std::ios::trunc);
    diag_log_ << std::fixed << std::setprecision(4);

    RCLCPP_INFO(
      get_logger(), "Recording odometry/filtered_map to '%s' every %.2f m, "
      "non-OK /diagnostics to '%s'.",
      output_csv_.c_str(), min_spacing_m_, diag_log_path_.c_str());

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps/fix", 10,
      std::bind(&WaypointRecorder::on_gps, this, std::placeholders::_1));

    gps_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/gps", 10,
      std::bind(&WaypointRecorder::on_gps_odom, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered_map", 10,
      std::bind(&WaypointRecorder::on_odom, this, std::placeholders::_1));

    raw_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&WaypointRecorder::on_raw_odom, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10,
      std::bind(&WaypointRecorder::on_imu, this, std::placeholders::_1));

    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 50,
      std::bind(&WaypointRecorder::on_diagnostics, this, std::placeholders::_1));
  }

  ~WaypointRecorder() override
  {
    if (csv_.is_open()) {
      csv_.close();
    }
    if (diag_log_.is_open()) {
      diag_log_.close();
    }
  }

private:
  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_lat_ = msg->latitude;
    last_lon_ = msg->longitude;
    last_gps_status_ = msg->status.status;
    has_gps_ = true;
  }

  void on_gps_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_gps_odom_x_ = msg->pose.pose.position.x;
    last_gps_odom_y_ = msg->pose.pose.position.y;
    has_gps_odom_ = true;
  }

  void on_raw_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_x_ = msg->pose.pose.position.x;
    last_odom_y_ = msg->pose.pose.position.y;
    last_odom_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
    last_odom_vx_ = msg->twist.twist.linear.x;
    last_odom_vy_ = msg->twist.twist.linear.y;
    last_odom_vyaw_ = msg->twist.twist.angular.z;
    has_raw_odom_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_wz_ = msg->angular_velocity.z;
    last_imu_ax_ = msg->linear_acceleration.x;
    last_imu_ay_ = msg->linear_acceleration.y;
    last_imu_yaw_ = yaw_from_quaternion(msg->orientation);
    has_imu_ = true;
  }

  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    if (!diag_log_.is_open()) {return;}
    const double stamp =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    for (const auto & status : msg->status) {
      if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {continue;}
      diag_log_ << '[' << stamp << "] level=" << static_cast<int>(status.level)
                << " name=\"" << status.name << "\" message=\"" << status.message << "\"";
      for (const auto & kv : status.values) {
        diag_log_ << " " << kv.key << "=" << kv.value;
      }
      diag_log_ << '\n';
      diag_log_.flush();
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!csv_.is_open()) {return;}
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    if (has_last_) {
      const double dx = x - last_x_;
      const double dy = y - last_y_;
      if (std::hypot(dx, dy) < min_spacing_m_) {return;}
    }

    const double stamp =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    const double yaw = yaw_from_quaternion(msg->pose.pose.orientation);
    csv_ << idx_ << ',' << stamp << ',' << x << ',' << y << ',' << yaw << ','
         << msg->header.frame_id << ',';
    if (has_gps_) {
      csv_ << last_lat_ << ',' << last_lon_ << ',' << last_gps_status_;
    } else {
      csv_ << ",,";
    }
    csv_ << ',';
    if (has_gps_odom_) {
      csv_ << last_gps_odom_x_ << ',' << last_gps_odom_y_;
    } else {
      csv_ << ",";
    }
    // pose.covariance is row-major 6x6 over [x,y,z,roll,pitch,yaw]; index 0 = xx, 7 = yy.
    csv_ << ',' << msg->pose.covariance[0] << ',' << msg->pose.covariance[7];
    csv_ << ',';
    if (has_raw_odom_) {
      csv_ << last_odom_vx_ << ',' << last_odom_vy_ << ',' << last_odom_vyaw_;
    } else {
      csv_ << ",,";
    }
    csv_ << ',';
    if (has_imu_) {
      csv_ << last_imu_wz_ << ',' << last_imu_ax_ << ',' << last_imu_ay_ << ',' << last_imu_yaw_;
    } else {
      csv_ << ",,,";
    }
    csv_ << ',';
    if (has_raw_odom_) {
      csv_ << last_odom_x_ << ',' << last_odom_y_ << ',' << last_odom_yaw_;
    } else {
      csv_ << ",,";
    }
    csv_ << '\n';
    csv_.flush();
    ++idx_;

    last_x_ = x;
    last_y_ = y;
    has_last_ = true;
  }

  std::string output_csv_;
  double min_spacing_m_{0.5};
  std::ofstream csv_;
  std::size_t idx_{0U};
  double last_x_{0.0};
  double last_y_{0.0};
  bool has_last_{false};

  double last_lat_{0.0};
  double last_lon_{0.0};
  int32_t last_gps_status_{0};
  bool has_gps_{false};

  double last_gps_odom_x_{0.0};
  double last_gps_odom_y_{0.0};
  bool has_gps_odom_{false};

  double last_odom_x_{0.0};
  double last_odom_y_{0.0};
  double last_odom_yaw_{0.0};
  double last_odom_vx_{0.0};
  double last_odom_vy_{0.0};
  double last_odom_vyaw_{0.0};
  bool has_raw_odom_{false};

  double last_imu_wz_{0.0};
  double last_imu_ax_{0.0};
  double last_imu_ay_{0.0};
  double last_imu_yaw_{0.0};
  bool has_imu_{false};

  std::string diag_log_path_;
  std::ofstream diag_log_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointRecorder>());
  rclcpp::shutdown();
  return 0;
}
