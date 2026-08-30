#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include <sstream>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    // 기본값 true는 기존 `ros2 run ... waypoint_recorder_node` 사용법을 그대로 두기
    // 위한 것입니다(띄우면 바로 기록). GUI로 시작/정지를 제어할 때만 false로 띄우세요
    // -- false면 ~/start를 부르기 전까지 output_csv를 열지도, 지우지도 않습니다.
    // (기록 시작이 곧 truncate이므로, 이전 녹화본을 실수로 날리지 않는다는 뜻입니다.)
    const bool auto_start = declare_parameter<bool>("auto_start", true);

    diag_log_path_ = output_csv_ + ".diag.log";

    // GUI/RViz가 늦게 붙어도 현재 상태를 받도록 transient_local입니다
    // (mission_manager_node의 ~/status, ~/path와 같은 방식).
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/path", rclcpp::QoS(1).transient_local());

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = start_recording(res->message);
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = stop_recording(res->message);
      });

    // 상태 문자열은 주기적으로도 내보냅니다. 기록 지점 사이(0.5 m 이동)에도 GUI가
    // "다음 점까지 얼마 남았는지"와 GPS 신선도를 실시간으로 보여줘야 하기 때문입니다.
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {publish_status();});

    if (auto_start) {
      std::string message;
      if (!start_recording(message)) {
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        return;
      }
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Idle. Call '%s/start' to begin recording to '%s' (every %.2f m).",
        get_name(), output_csv_.c_str(), min_spacing_m_);
      publish_status();
    }

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
  // 기록 시작. 파일을 여기서 처음 열고 truncate합니다 -- 노드가 떠 있는 것과
  // 기록 중인 것은 다른 상태이고, 그 구분이 곧 "실수로 이전 코스를 덮어쓰지 않는다"입니다.
  bool start_recording(std::string & message)
  {
    if (recording_) {
      message = "already recording to '" + output_csv_ + "'";
      return false;
    }

    // output_csv는 실행 중에 바꿀 수 있습니다 -- GUI가 파일 이름 칸에 적은 값을
    // 파라미터로 밀어 넣고 ~/start를 부르는 흐름(waypoint_record_gui.py). 여기서
    // 다시 읽어 두면 매 녹화가 그 시점의 파라미터 값으로 갑니다.
    output_csv_ = get_parameter("output_csv").as_string();
    diag_log_path_ = output_csv_ + ".diag.log";

    csv_.open(output_csv_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      message = "failed to open '" + output_csv_ + "' for writing";
      return false;
    }
    csv_ << std::fixed << std::setprecision(8);
    csv_ << "idx,stamp_sec,x,y,yaw,frame_id,gps_lat,gps_lon,gps_status,gps_odom_x,gps_odom_y,"
            "cov_xx,cov_yy,odom_vx,odom_vy,odom_vyaw,imu_wz,imu_ax,imu_ay,imu_yaw,"
            "odom_x,odom_y,odom_yaw\n";
    csv_.flush();

    diag_log_.open(diag_log_path_, std::ios::out | std::ios::trunc);
    diag_log_ << std::fixed << std::setprecision(4);

    // 새 녹화는 완전히 처음부터입니다. idx와 경로를 안 비우면 두 번째 녹화가
    // 이전 코스의 마지막 점을 기준으로 간격을 재게 됩니다.
    idx_ = 0U;
    has_last_ = false;
    path_length_m_ = 0.0;
    path_.poses.clear();
    path_.header.frame_id = "map";
    recording_ = true;

    RCLCPP_INFO(
      get_logger(), "Recording odometry/filtered_map to '%s' every %.2f m, "
      "non-OK /diagnostics to '%s'.",
      output_csv_.c_str(), min_spacing_m_, diag_log_path_.c_str());
    message = "recording to '" + output_csv_ + "'";
    publish_status();
    return true;
  }

  bool stop_recording(std::string & message)
  {
    if (!recording_) {
      message = "not recording";
      return false;
    }
    recording_ = false;
    if (csv_.is_open()) {csv_.close();}
    if (diag_log_.is_open()) {diag_log_.close();}

    std::ostringstream out;
    out << "stopped: " << idx_ << " points, " << std::fixed << std::setprecision(1)
        << path_length_m_ << " m -> '" << output_csv_ << "'";
    message = out.str();
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    publish_status();
    return true;
  }

  double seconds_since(const rclcpp::Time & stamp) const
  {
    return stamp.nanoseconds() == 0 ? -1.0 : (now() - stamp).seconds();
  }

  // GUI가 파싱하는 key=value 한 줄. 새 필드를 뒤에 붙여도 기존 GUI가 안 깨지도록
  // 위치가 아니라 이름으로 읽게 되어 있습니다(waypoint_record_gui.py).
  void publish_status()
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "recording=" << (recording_ ? 1 : 0)
        << " file=" << output_csv_
        << " points=" << idx_
        << " spacing=" << min_spacing_m_
        << " length=" << path_length_m_;
    if (has_last_) {
      out << " last_x=" << last_x_ << " last_y=" << last_y_ << " last_yaw=" << last_yaw_;
    }
    if (has_pose_) {
      out << " x=" << pose_x_ << " y=" << pose_y_ << " yaw=" << pose_yaw_
          << " pose_age=" << seconds_since(pose_stamp_);
      if (has_last_) {
        out << " since_last=" << std::hypot(pose_x_ - last_x_, pose_y_ - last_y_);
      }
    }
    if (has_gps_) {
      out << " gps_status=" << last_gps_status_
          << " gps_lat=" << std::setprecision(8) << last_lat_
          << " gps_lon=" << last_lon_ << std::setprecision(3)
          << " gps_age=" << seconds_since(gps_stamp_);
    }
    if (has_cov_) {
      out << " cov_xx=" << cov_xx_ << " cov_yy=" << cov_yy_;
    }
    if (has_raw_odom_) {
      out << " speed=" << last_odom_vx_;
    }
    std_msgs::msg::String msg;
    msg.data = out.str();
    status_pub_->publish(msg);
  }

  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_lat_ = msg->latitude;
    last_lon_ = msg->longitude;
    last_gps_status_ = msg->status.status;
    gps_stamp_ = now();
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
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double pose_yaw = yaw_from_quaternion(msg->pose.pose.orientation);

    // 현재 위치는 기록 중이 아니어도 갱신합니다. GUI가 대기 중에도 "지금 여기서
    // 시작하면 어디부터 찍히는지"와 GPS/공분산 상태를 보여줘야 하기 때문입니다.
    pose_x_ = x;
    pose_y_ = y;
    pose_yaw_ = pose_yaw;
    pose_stamp_ = now();
    has_pose_ = true;
    cov_xx_ = msg->pose.covariance[0];
    cov_yy_ = msg->pose.covariance[7];
    has_cov_ = true;

    if (!recording_ || !csv_.is_open()) {return;}

    if (has_last_) {
      const double dx = x - last_x_;
      const double dy = y - last_y_;
      const double step = std::hypot(dx, dy);
      if (step < min_spacing_m_) {return;}
      path_length_m_ += step;
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
    last_yaw_ = yaw;
    has_last_ = true;

    // 지금까지 찍힌 점들을 그대로 Path로 내보냅니다. GUI가 그리는 미니맵과
    // RViz가 같은 소스를 보게 되고, 녹화 중 "어디까지 찍혔나"를 눈으로 확인할 수
    // 있습니다(mission_manager_node의 ~/path와 같은 방식).
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path_.header.stamp = msg->header.stamp;
    path_.header.frame_id = msg->header.frame_id;
    path_.poses.push_back(pose);
    path_pub_->publish(path_);
    publish_status();
  }

  std::string output_csv_;
  double min_spacing_m_{0.5};
  std::ofstream csv_;
  std::size_t idx_{0U};
  bool recording_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_yaw_{0.0};
  bool has_last_{false};
  double path_length_m_{0.0};
  nav_msgs::msg::Path path_;

  // 마지막으로 들어온 위치. 기록된 점(last_*)과 달리 기록 중이 아니어도 갱신됩니다.
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  rclcpp::Time pose_stamp_{0, 0, RCL_ROS_TIME};
  bool has_pose_{false};
  double cov_xx_{0.0};
  double cov_yy_{0.0};
  bool has_cov_{false};

  double last_lat_{0.0};
  double last_lon_{0.0};
  int32_t last_gps_status_{0};
  rclcpp::Time gps_stamp_{0, 0, RCL_ROS_TIME};
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

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointRecorder>());
  rclcpp::shutdown();
  return 0;
}
