#include "hyper_control/stm32_system_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "hyper_control/stm32_protocol.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace hyper_control
{

namespace
{

std::string get_hardware_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & default_value)
{
  const auto it = info.hardware_parameters.find(name);
  return it != info.hardware_parameters.end() ? it->second : default_value;
}

// termios only accepts baud rates through the fixed set of B* constants below.
bool baud_rate_to_speed(int baud_rate, speed_t & speed)
{
  switch (baud_rate) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 38400: speed = B38400; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
    case 230400: speed = B230400; return true;
    case 460800: speed = B460800; return true;
    case 921600: speed = B921600; return true;
    default: return false;
  }
}

}  // namespace

hardware_interface::CallbackReturn Stm32SystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = get_hardware_parameter(info_, "serial_port", "/dev/ttyUSB0");
  baud_rate_ = std::stoi(get_hardware_parameter(info_, "baud_rate", "115200"));

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.name == "steering_joint") {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(logger_, "'steering_joint' must have exactly one 'position' command interface");
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else if (joint.name == "rear_axle_joint") {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_ERROR(logger_, "'rear_axle_joint' must have exactly one 'velocity' command interface");
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      RCLCPP_ERROR(logger_, "unexpected joint '%s', expected 'steering_joint' or 'rear_axle_joint'",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Stm32SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    "steering_joint", hardware_interface::HW_IF_POSITION, &hw_state_steering_position_);
  state_interfaces.emplace_back(
    "rear_axle_joint", hardware_interface::HW_IF_VELOCITY, &hw_state_rear_velocity_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Stm32SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    "steering_joint", hardware_interface::HW_IF_POSITION, &hw_command_steering_position_);
  command_interfaces.emplace_back(
    "rear_axle_joint", hardware_interface::HW_IF_VELOCITY, &hw_command_rear_velocity_);

  return command_interfaces;
}

bool Stm32SystemInterface::open_serial_port()
{
  speed_t speed{};
  if (!baud_rate_to_speed(baud_rate_, speed)) {
    RCLCPP_ERROR(logger_, "unsupported baud rate %d", baud_rate_);
    return false;
  }

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(logger_, "failed to open '%s': %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr on '%s' failed: %s", serial_port_.c_str(), std::strerror(errno));
    close_serial_port();
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr on '%s' failed: %s", serial_port_.c_str(), std::strerror(errno));
    close_serial_port();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);

  return true;
}

void Stm32SystemInterface::close_serial_port()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool Stm32SystemInterface::send_command(double steering_angle, double rear_wheel_velocity)
{
  if (serial_fd_ < 0) {
    return false;
  }

  Stm32CommandPacket packet;
  packet.steering_angle = static_cast<float>(steering_angle);
  packet.rear_wheel_velocity = static_cast<float>(rear_wheel_velocity);
  packet.crc = compute_crc8(
    reinterpret_cast<const uint8_t *>(&packet) + 1, sizeof(packet.steering_angle) + sizeof(packet.rear_wheel_velocity));

  const auto * bytes = reinterpret_cast<const uint8_t *>(&packet);
  std::size_t written = 0;
  while (written < sizeof(packet)) {
    const auto result = ::write(serial_fd_, bytes + written, sizeof(packet) - written);
    if (result < 0) {
      RCLCPP_ERROR(logger_, "write to '%s' failed: %s", serial_port_.c_str(), std::strerror(errno));
      return false;
    }
    written += static_cast<std::size_t>(result);
  }

  return true;
}

hardware_interface::CallbackReturn Stm32SystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_serial_port()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "opened '%s' at %d baud", serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_command_steering_position_ = 0.0;
  hw_command_rear_velocity_ = 0.0;
  hw_state_steering_position_ = 0.0;
  hw_state_rear_velocity_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Stm32SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Best-effort stop command; ignore failures since we're already shutting down.
  send_command(0.0, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Stm32SystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // No feedback link yet -- state mirrors the last command.
  hw_state_steering_position_ = hw_command_steering_position_;
  hw_state_rear_velocity_ = hw_command_rear_velocity_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Stm32SystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!send_command(hw_command_steering_position_, hw_command_rear_velocity_)) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hyper_control

PLUGINLIB_EXPORT_CLASS(hyper_control::Stm32SystemInterface, hardware_interface::SystemInterface)
