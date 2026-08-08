#ifndef HYPER_CONTROL__STM32_SYSTEM_INTERFACE_HPP_
#define HYPER_CONTROL__STM32_SYSTEM_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hyper_control
{

// ros2_control SystemInterface for the real vehicle: exposes one virtual "steering_joint"
// (position command) and one virtual "rear_axle_joint" (velocity command), and serializes both
// over a serial link to an STM32 board using the packet defined in stm32_protocol.hpp. There is
// no feedback path yet -- read() mirrors the last commanded values back as state.
class Stm32SystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool open_serial_port();
  void close_serial_port();
  bool send_command(double steering_angle, double rear_wheel_velocity);

  std::string serial_port_;
  int baud_rate_{115200};
  int serial_fd_{-1};

  double hw_command_steering_position_{0.0};
  double hw_command_rear_velocity_{0.0};
  double hw_state_steering_position_{0.0};
  double hw_state_rear_velocity_{0.0};

  rclcpp::Logger logger_{rclcpp::get_logger("Stm32SystemInterface")};
};

}  // namespace hyper_control

#endif  // HYPER_CONTROL__STM32_SYSTEM_INTERFACE_HPP_
