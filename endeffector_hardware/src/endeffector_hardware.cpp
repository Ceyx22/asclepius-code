#include "endeffector_hardware/endeffector_hardware.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace endeffector_hardware
{
  constexpr const char *kEndeffectorHardware = "EndeffectorHardware";
  CallbackReturn EndeffectorHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    config_.usb_port = info_.hardware_parameters["usb_port"];
    config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);

    // Extract joint names and servo IDs
    for (const auto &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger(kEndeffectorHardware),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger(kEndeffectorHardware),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_DEBUG(rclcpp::get_logger(kEndeffectorHardware), "Added motor: %d", std::stoi(joint.parameters.at("id")));

      endeffector_hardware::Motor motor_joint;
      motor_joint.init(joint.name, std::stoi(joint.parameters.at("id")));
      endeffector.push_back(motor_joint);
    }

    motor_pos_publisher = rclcpp::Node::make_shared("motor_position_publisher")->create_publisher<std_msgs::msg::Float64MultiArray>("motor_positions", 10);
    motor_vel_publisher = rclcpp::Node::make_shared("motor_velocity_publisher")->create_publisher<std_msgs::msg::Float64MultiArray>("motor_velocity", 10);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EndeffectorHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint i = 0; i < endeffector.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(endeffector[i].motor_name, hardware_interface::HW_IF_POSITION, &endeffector[i].current_motor_angle));
      state_interfaces.emplace_back(hardware_interface::StateInterface(endeffector[i].motor_name, hardware_interface::HW_IF_VELOCITY, &endeffector[i].current_motor_speed));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EndeffectorHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i = 0; i < endeffector.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(endeffector[i].motor_name, hardware_interface::HW_IF_POSITION, &endeffector[i].desired_motor_angle));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(endeffector[i].motor_name, hardware_interface::HW_IF_VELOCITY, &endeffector[i].desired_motor_speed));
    }

    return command_interfaces;
  }

  CallbackReturn EndeffectorHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "start");

    dynamixel_connection_.connect(config_.usb_port, config_.baud_rate);
    for (uint i = 0; i < endeffector.size(); i++)
    {
      endeffector[i].setup(dynamixel_connection_);
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn EndeffectorHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {

    RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "stop");
    for (uint i = 0; i < endeffector.size(); i++)
    {
      endeffector[i].shutdown();
    }
    dynamixel_connection_.disconnect();

    return CallbackReturn::SUCCESS;
  }

  return_type EndeffectorHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    std_msgs::msg::Float64MultiArray motor_position_message = std_msgs::msg::Float64MultiArray();
    std_msgs::msg::Float64MultiArray motor_velocity_message = std_msgs::msg::Float64MultiArray();

    for (uint i = 0; i < endeffector.size(); i++)
    {
      if (endeffector[i].update_position() && endeffector[i].update_velocity())
      {
        // double motor_position = (std::abs(endeffector[i].current_motor_angle)) * (300) / (double)(1023);
        motor_position_message.data.push_back(endeffector[i].current_motor_pos_);
        motor_velocity_message.data.push_back(endeffector[i].current_motor_vel_);
        RCLCPP_DEBUG(rclcpp::get_logger(kEndeffectorHardware), "Joint: %s , Position: %f", endeffector[i].motor_name.c_str(), endeffector[i].current_motor_angle);
      }
    }
    motor_pos_publisher->publish(motor_position_message);
    motor_vel_publisher->publish(motor_velocity_message);
    return return_type::OK;
  }

  return_type EndeffectorHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    bool is_success = false;
    bool second_is_success = false;

    for (auto &motor : endeffector)
    {
      double cmd = motor.desired_motor_angle;
      is_success = motor.write_position(cmd);
      second_is_success = motor.write_speed(motor.desired_motor_speed);

      // motorPublisher.

      assert(is_success && second_is_success);
    }

    return return_type::OK;
  }

} // namespace endeffector_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(endeffector_hardware::EndeffectorHardware, hardware_interface::SystemInterface)
