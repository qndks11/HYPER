#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

// nav2의 controller_server는 /cmd_vel(Twist)을 내보내지만 차량은 /velocity와
// /steering_angle(Float64)을 먹습니다. 자전거 모델로 각속도를 조향각으로 바꿉니다.
//   delta = atan(wheelbase * omega / v)
class CmdVelToAckermann : public rclcpp::Node
{
public:
  CmdVelToAckermann() : Node("cmd_vel_to_ackermann")
  {
    wheelbase_ = declare_parameter<double>("wheelbase", 1.005);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.5235988);
    max_velocity_ = declare_parameter<double>("max_velocity", 5.0);
    input_timeout_ = declare_parameter<double>("input_timeout", 0.3);
    min_speed_for_steering_ = declare_parameter<double>("min_speed_for_steering", 0.05);

    velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/velocity", 1);
    steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 1);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1,
      std::bind(&CmdVelToAckermann::on_cmd_vel, this, std::placeholders::_1));

    // /cmd_vel이 끊겼는데 마지막 명령이 그대로 남아 차가 계속 굴러가는 걸 막습니다.
    watchdog_ = create_wall_timer(
      std::chrono::duration<double>(input_timeout_ * 0.5),
      std::bind(&CmdVelToAckermann::on_watchdog, this));

    RCLCPP_INFO(
      get_logger(),
      "/cmd_vel -> /velocity + /steering_angle (wheelbase=%.3f m, "
      "max_steering=%.4f rad, max_velocity=%.2f m/s).",
      wheelbase_, max_steering_angle_, max_velocity_);
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();
    stopped_ = false;

    const double v = std::clamp(msg->linear.x, -max_velocity_, max_velocity_);

    // 저속에서는 omega/v가 발산하므로 직전 조향각을 유지합니다.
    if (std::fabs(v) >= min_speed_for_steering_) {
      const double delta = std::atan(wheelbase_ * msg->angular.z / v);
      steering_angle_ = std::clamp(delta, -max_steering_angle_, max_steering_angle_);
    }

    publish(v, steering_angle_);
  }

  void on_watchdog()
  {
    if (stopped_ || last_cmd_time_.nanoseconds() == 0) {
      return;
    }
    if ((now() - last_cmd_time_).seconds() < input_timeout_) {
      return;
    }
    RCLCPP_WARN(get_logger(), "/cmd_vel timed out after %.2f s; commanding stop.", input_timeout_);
    steering_angle_ = 0.0;
    stopped_ = true;
    publish(0.0, 0.0);
  }

  void publish(double velocity, double steering_angle)
  {
    std_msgs::msg::Float64 velocity_msg;
    velocity_msg.data = velocity;
    velocity_pub_->publish(velocity_msg);

    std_msgs::msg::Float64 steering_msg;
    steering_msg.data = steering_angle;
    steering_pub_->publish(steering_msg);
  }

  double wheelbase_{1.005};
  double max_steering_angle_{0.5235988};
  double max_velocity_{5.0};
  double input_timeout_{0.3};
  double min_speed_for_steering_{0.05};

  double steering_angle_{0.0};
  bool stopped_{true};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToAckermann>());
  rclcpp::shutdown();
  return 0;
}
