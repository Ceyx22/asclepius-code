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
    // endeffector.resize(info_.joints.size(), endeffector_hardware::Motor());
    // motor_names.resize(info_.joints.size(), 0);

    // Extract joint names and servo IDs
    for (const auto &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
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

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EndeffectorHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint i = 0; i < endeffector.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(endeffector[i].name, hardware_interface::HW_IF_POSITION, &endeffector[i].pos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(endeffector[i].name, hardware_interface::HW_IF_VELOCITY, &endeffector[i].vel));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EndeffectorHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i = 0; i < endeffector.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(endeffector[i].name, hardware_interface::HW_IF_POSITION, &endeffector[i].cmd));
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

  return_type EndeffectorHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    double delta_seconds = period.seconds();
    for (uint i = 0; i < endeffector.size(); i++)
    {
      if (endeffector[i].update_position())
      {
        RCLCPP_DEBUG(rclcpp::get_logger(kEndeffectorHardware), "Joint: %s , Position: %f", endeffector[i].name.c_str(), endeffector[i].pos);
      }
      // endeffector[i].pos = p;
      // endeffector[i].pos += v * delta_seconds; // Integrate velocity to get position
    }

    return return_type::OK;
  }

  return_type EndeffectorHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    bool is_success = false;
    for (uint i = 0; i < endeffector.size(); i++)
    {
      is_success = endeffector[i].write_position(endeffector[i].cmd);
      if (!is_success)
      {
        // RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "Command: %f", endeffector[i].cmd);
        RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Failed to write new position Command: %f", endeffector[i].cmd);
      }
    }

    return return_type::OK;
  }

} // namespace endeffector_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(endeffector_hardware::EndeffectorHardware, hardware_interface::SystemInterface)
