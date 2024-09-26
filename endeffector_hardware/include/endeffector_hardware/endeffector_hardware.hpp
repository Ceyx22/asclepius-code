#ifndef ENDEFFECTOR_HARDWARE__ENDEFFECTOR_HARDWARE_HPP_
#define ENDEFFECTOR_HARDWARE__ENDEFFECTOR_HARDWARE_HPP_

#include <string>
#include <vector>
#include <unordered_map>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "endeffector_hardware/visiblity_control.h"
#include "endeffector_hardware/comms.hpp"
#include "endeffector_hardware/motor.hpp"
#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace endeffector_hardware
{

  class EndeffectorHardware : public hardware_interface::SystemInterface
  {
    struct Config
    {
      // std::vector<std::string> motor_names;
      std::string usb_port = "";
      int baud_rate = 0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(EndeffectorHardware)

    ENDEFFECTOR_HARDWARE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    ENDEFFECTOR_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ENDEFFECTOR_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ENDEFFECTOR_HARDWARE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    ENDEFFECTOR_HARDWARE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    ENDEFFECTOR_HARDWARE_PUBLIC
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    ENDEFFECTOR_HARDWARE_PUBLIC
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    endeffector_hardware::Comms dynamixel_connection_;

    std::vector<endeffector_hardware::Motor> endeffector;
    // endeffector_hardware::Motor motor_joint;

    // std::vector<std::string> motor_names;

    Config config_;
  };

} // namespace ENDEFFECTOR_HARDWARE_interface

#endif // ENDEFFECTOR_HARDWARE__ENDEFFECTOR_HARDWARE_HPP_
