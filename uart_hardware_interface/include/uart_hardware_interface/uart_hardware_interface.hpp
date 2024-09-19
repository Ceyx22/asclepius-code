// include/uart_hardware_interface/uart_hardware_interface.hpp

#ifndef UART_HARDWARE_INTERFACE__UART_HARDWARE_INTERFACE_HPP_
#define UART_HARDWARE_INTERFACE__UART_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"

#include "uart_comm.hpp" // Include your UART communication class

namespace uart_hardware_interface
{

class UARTHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UARTHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // UART communication object
  std::unique_ptr<UARTComm> uart_comm_;

  // Port name and baud rate
  std::string port_name_;
  int baud_rate_;

  // Joints data
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<std::string> joint_names_;
};

} // namespace uart_hardware_interface

#endif // UART_HARDWARE_INTERFACE__UART_HARDWARE_INTERFACE_HPP_
