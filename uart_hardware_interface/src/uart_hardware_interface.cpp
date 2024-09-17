
#include "uart_hardware_interface/uart_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <iostream>

namespace uart_hardware_interface
{

hardware_interface::CallbackReturn UARTHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters from the hardware_info
  for (const auto & param : info_.hardware_parameters)
  {
    if (param.first == "port_name")
    {
      port_name_ = param.second;
    }
    else if (param.first == "baud_rate")
    {
      baud_rate_ = std::stoi(param.second);
    }
  }

  if (port_name_.empty() || baud_rate_ == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("UARTHardwareInterface"), "UART port name or baud rate not specified");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize joint states and commands
  size_t num_joints = info_.joints.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);
  joint_names_.resize(num_joints);

  // Get joint names
  for (size_t i = 0; i < num_joints; ++i)
  {
    joint_names_[i] = info_.joints[i].name;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UARTHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("UARTHardwareInterface"), "Configuring UART Hardware Interface...");

  uart_comm_ = std::make_unique<UARTComm>(port_name_, baud_rate_);

  if (!uart_comm_->openPort())
  {
    RCLCPP_ERROR(rclcpp::get_logger("UARTHardwareInterface"), "Failed to open UART port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("UARTHardwareInterface"), "UART port opened successfully");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> UARTHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UARTHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn UARTHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("UARTHardwareInterface"), "Activating UART Hardware Interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UARTHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("UARTHardwareInterface"), "Deactivating UART Hardware Interface...");
  uart_comm_->closePort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UARTHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Implement reading from once encoders are on

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UARTHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!uart_comm_->isOpen())
    {
        RCLCPP_ERROR(rclcpp::get_logger("UARTHardwareInterface"), "UART port is not open");
        return hardware_interface::return_type::ERROR;
    }

    // Constants for conversion
    const double steps_per_revolution = 200.0; // Example value, adjust as needed
    const double radians_per_revolution = 2.0 * M_PI;
    const double steps_per_radian = steps_per_revolution / radians_per_revolution;

    // Might not need
    const int speed = 100; // Example speed value

    // Map joint names to motor identifiers
    std::map<std::string, char> joint_to_motor = {
        {"joint1", 'X'},
        {"joint2", 'Y'},
        {"joint3", 'Z'},
        {"joint4", 'E'}
        // Add more mappings as needed
    };

    // For each joint, create and send the command
    for (size_t i = 0; i < hw_commands_.size(); ++i)
    {
        std::string joint_name = joint_names_[i];
        char motor_id = joint_to_motor[joint_name];

        // Calculate the difference between commanded position and current position
        double position_difference = hw_commands_[i] - hw_positions_[i];

        // Calculate the number of steps
        int number_of_steps = static_cast<int>(std::abs(position_difference * steps_per_radian));

        // Determine the direction
        int direction = (position_difference >= 0) ? 1 : 0;

        // Format the command string
        std::stringstream command_stream;
        command_stream << "M" << motor_id
                       << "S" << number_of_steps
                       << "D" << direction
                       << "V" << speed
                       << "\r";
        std::string command = command_stream.str();

        // Send the command over UART
        if (!uart_comm_->writeData(command))
        {
            RCLCPP_ERROR(rclcpp::get_logger("UARTHardwareInterface"), "Failed to write data to UART");
            return hardware_interface::return_type::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("UARTHardwareInterface"), "Sent command: %s", command.c_str());
    }

    hw_positions_ = hw_commands_;

    return hardware_interface::return_type::OK;
}

} // namespace uart_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(uart_hardware_interface::UARTHardwareInterface, hardware_interface::SystemInterface)
