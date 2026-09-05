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
   * @brief Edge-triggers off msg->buttons[] to latch/unlatch /estop.
   *
   * @details Does not touch /velocity or /steering_angle -- this node only
   * ever publishes the /estop latch, so it is safe to run alongside
   * cmd_vel_to_ackermann_node (mission mode) without the topic collision
   * joystick_controller_node would cause.
   */
  void listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  void publish_estop(bool active);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;

  int estop_button_index_;
  int resume_button_index_;

  std::vector<int32_t> previous_buttons_;
  bool estop_active_;
};

#endif  // ESTOP_CONTROLLER_NODE_HPP
