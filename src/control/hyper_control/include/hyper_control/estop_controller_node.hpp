#ifndef ESTOP_CONTROLLER_NODE_HPP
#define ESTOP_CONTROLLER_NODE_HPP

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class EstopController : public rclcpp::Node
{
public:
  EstopController();

private:
  /**
   * @brief Edge-triggers off msg->buttons[] to latch/unlatch /estop, pausing
   * and resuming the mission with it.
   *
   * @details Does not touch /velocity or /steering_angle -- this node only
   * ever publishes the /estop latch, so it is safe to run alongside
   * cmd_vel_to_ackermann_node (mission mode) without the topic collision
   * joystick_controller_node would cause.
   *
   * The mission is paused, not canceled: mission_manager holds the step it was
   * on (and the time left in it), so the resume button picks up where the pause
   * left off instead of needing Start on the panel.
   */
  void listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  void publish_estop(bool active);

  /// Best-effort Trigger call -- logs and moves on if the service is absent.
  void call(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, const char * what);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;

  int estop_button_index_;
  int resume_button_index_;

  std::vector<int32_t> previous_buttons_;
  bool estop_active_;
};

#endif  // ESTOP_CONTROLLER_NODE_HPP
