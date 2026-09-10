#include "hyper_control/estop_controller_node.hpp"

EstopController::EstopController() :
  Node{"estop_controller"},
  estop_button_index_{1},
  resume_button_index_{0},
  estop_active_{false}
{
  // Placeholder defaults (1/0) -- controller-model-specific, verify against
  // the real pad with `ros2 topic echo /joy` and override via parameters.yaml
  // before relying on them.
  declare_parameter<int>("estop_button_index", estop_button_index_);
  declare_parameter<int>("resume_button_index", resume_button_index_);
  get_parameter("estop_button_index", estop_button_index_);
  get_parameter("resume_button_index", resume_button_index_);

  subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&EstopController::listener_callback, this, std::placeholders::_1));

  // Latched (transient_local) so a bridge that starts after the button was
  // pressed still picks up the current e-stop state immediately.
  estop_publisher_ = create_publisher<std_msgs::msg::Bool>(
    "/estop", rclcpp::QoS(1).transient_local());
  publish_estop(false);

  // Best-effort: mission_manager may not be running (e.g. manual-driving
  // launch tree) -- that's fine, the halt itself happens downstream at
  // arduino_interface_node regardless of whether these calls succeed.
  //
  // Pause, not cancel: cancel dropped the mission to idle and made every test
  // stop cost a trip to the panel's Start button. '~/pause' keeps the step and
  // the time left in it, so '~/resume' continues from here.
  pause_client_ = create_client<std_srvs::srv::Trigger>("mission_manager/pause");
  resume_client_ = create_client<std_srvs::srv::Trigger>("mission_manager/resume");
}

void EstopController::listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (previous_buttons_.empty()) {
    previous_buttons_ = msg->buttons;
    return;
  }

  auto pressed = [&](int index) {
    return index >= 0 &&
      static_cast<size_t>(index) < msg->buttons.size() &&
      static_cast<size_t>(index) < previous_buttons_.size() &&
      msg->buttons[index] != 0 && previous_buttons_[index] == 0;
  };

  if (pressed(estop_button_index_) && !estop_active_) {
    estop_active_ = true;
    publish_estop(true);
    RCLCPP_WARN(get_logger(), "PAUSED -- vehicle halted, mission holds at the current step");
    call(pause_client_, "pause");
  } else if (pressed(resume_button_index_) && estop_active_) {
    estop_active_ = false;
    // Release the latch *before* resuming: while /estop is true
    // arduino_interface_node zeroes /velocity and /steering_angle, so goals
    // sent in between would go nowhere.
    publish_estop(false);
    call(resume_client_, "resume");
    RCLCPP_INFO(get_logger(), "Resumed -- the mission continues from where it paused");
  }

  previous_buttons_ = msg->buttons;
}

void EstopController::call(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, const char * what)
{
  if (client->service_is_ready()) {
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  } else {
    RCLCPP_WARN(
      get_logger(), "mission_manager/%s not available; only the /estop latch changed", what);
  }
}

void EstopController::publish_estop(bool active)
{
  auto msg{std::make_shared<std_msgs::msg::Bool>()};
  msg->data = active;
  estop_publisher_->publish(*msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstopController>());
  rclcpp::shutdown();
  return 0;
}
