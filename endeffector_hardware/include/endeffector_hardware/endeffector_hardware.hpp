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
#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace endeffector_hardware
{
  // struct JointValue
  // {
  //   double position{0.0};
  //   double velocity{0.0};
  //   // double effort{0.0};
  // };

  // struct Joint
  // {
  //   JointValue state{};
  //   JointValue command{};
  //   JointValue prev_command{};
  // };

  class EndeffectorHardware : public hardware_interface::SystemInterface
  {
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
    return_type enable_torque(const bool enabled);
    return_type ping_servo(uint8_t id);
    return_type initialize_servos();

    return_type reset_command();
    int32_t addToWrite(uint8_t id, double command_position);
    int32_t rad_to_pos(uint8_t id, double command_position);
    int32_t pos_to_rad(double position);

    int32_t read_param(uint8_t id, uint16_t address);

    // Dynamixel SDK members
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupSyncWrite *groupSyncWrite_;

    // Servo IDs and joint names
    std::vector<uint8_t> servo_ids_;
    std::vector<std::string> joint_names_;
    // std::vector<Joint> joints_;
    // std::unordered_map<std::string, double> position_commands_;
    // Joint state and command variables
    std::unordered_map<std::string, double> position_commands_;
    std::unordered_map<std::string, double> position_states_;
    std::unordered_map<std::string, double> velocity_states_;

    // rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_position_publisher_;
  };

} // namespace ENDEFFECTOR_HARDWARE_interface

#endif // ENDEFFECTOR_HARDWARE__ENDEFFECTOR_HARDWARE_HPP_
