#include "endeffector_hardware/endeffector_hardware.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h" // Dynamixel SDK for GroupSyncWrite

namespace endeffector_hardware
{

  // Control table addresses for AX-12A
  constexpr uint16_t ADDR_TORQUE_ENABLE = 24;
  constexpr uint16_t ADDR_GOAL_POSITION = 30;
  constexpr uint16_t ADDR_PRESENT_POSITION = 36;

  constexpr uint8_t TORQUE_ENABLE = 1;
  constexpr uint8_t TORQUE_DISABLE = 0;

  constexpr double PROTOCOL_VERSION = 1.0;

  // Limits for AX-12A
  constexpr int32_t DXL_MIN_POSITION_VALUE = 0;          // Minimum position value
  constexpr int32_t DXL_MAX_POSITION_VALUE = 1023;       // Maximum position value
  constexpr double DXL_MIN_ANGLE = 0.0;                  // Minimum angle in radians
  constexpr double DXL_MAX_ANGLE = 300.0 * M_PI / 180.0; // Maximum angle in radians

  // Motor offset: 511 represents 0 radians
  constexpr int32_t DXL_MOTOR_POSITION_OFFSET = 511;

  CallbackReturn EndeffectorHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Initialize PortHandler instance
    std::string port_name = info_.hardware_parameters["usb_port"];
    int baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);

    portHandler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler_->openPort())
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to open the port!");
      return CallbackReturn::ERROR;
    }

    if (!portHandler_->setBaudRate(baud_rate))
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to set the baudrate!");
      return CallbackReturn::ERROR;
    }

    // Create GroupSyncWrite for synchronizing position commands
    groupSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION, 2);

    // Extract joint names and servo IDs
    for (const auto &joint : info_.joints)
    {
      joint_names_.push_back(joint.name);
      // uint8_t id = static_cast<uint8_t>(std::stoi(joint.parameters.at("id")));
      servo_ids_.push_back(std::stoi(joint.parameters.at("id")));

      // Initialize position commands and states
      position_commands_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      position_states_[joint.name] = std::numeric_limits<double>::quiet_NaN();
    }

    // Initialize servos
    initialize_servos();
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EndeffectorHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (const auto &joint_name : joint_names_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &position_states_[joint_name]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EndeffectorHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (const auto &joint_name : joint_names_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &position_commands_[joint_name]));
    }

    return command_interfaces;
  }

  CallbackReturn EndeffectorHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Initialize position commands to current positions
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto id = servo_ids_[i];
      auto joint_name = joint_names_[i];
      int32_t dxl_pos = read_param(id, ADDR_PRESENT_POSITION);
      if (dxl_pos == -1)
      {
        return CallbackReturn::ERROR;
      }

      // double position = ;
      position_commands_[joint_name] = pos_to_rad(dxl_pos);
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn EndeffectorHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Disable torque
    for (const auto &id : servo_ids_)
    {
      uint8_t dxl_error = 0;
      int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to disable torque for ID %d: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Error disabling torque for ID %d: %s", id, packetHandler_->getRxPacketError(dxl_error));
      }
    }
    return CallbackReturn::SUCCESS;
  }

  return_type EndeffectorHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {

      int32_t dxl_pos = read_param(servo_ids_[i], ADDR_PRESENT_POSITION);
      if (dxl_pos == -1)
      {
        return return_type::ERROR;
      }

      position_states_[joint_names_[i]] = pos_to_rad(dxl_pos);
    }

    return return_type::OK;
  }

  return_type EndeffectorHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    groupSyncWrite_->clearParam();
    // bool value_changed = false;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      // Convert the command position from radians to Dynamixel position value with offset
      int32_t dxl_goal_position = rad_to_pos(servo_ids_[i], position_commands_[joint_names_[i]]);
      RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "DXL goal position for id %d, pos: %d ", servo_ids_[i], dxl_goal_position);
      // RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "DXL goal position for id %d, read pos: %d", servo_ids_[i], read_param(servo_ids_[i], ADDR_GOAL_POSITION));
      // if (dxl_goal_position != read_param(servo_ids_[i], ADDR_GOAL_POSITION))
      // {
      // RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Entered loop");

      dxl_goal_position = std::max(DXL_MIN_POSITION_VALUE, std::min(DXL_MAX_POSITION_VALUE, dxl_goal_position));

      uint8_t param_goal_position[2] = {DXL_LOBYTE(dxl_goal_position), DXL_HIBYTE(dxl_goal_position)};

      // Add to sync write
      if (!groupSyncWrite_->addParam(servo_ids_[i], param_goal_position))
      {
        RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to add servo ID %d to Sync Write", servo_ids_[i]);
        return return_type::ERROR;
      }
      // value_changed = true;
      // }
    }

    // Transmit the sync write packet to all servos value_changed == true &&
    if (groupSyncWrite_->txPacket() != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Sync Write failed");
      return return_type::ERROR;
      // value_changed = false;
    }
    groupSyncWrite_->clearParam();
    // RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Writing new position ");
    return return_type::OK;
  }

  // Helper
  int32_t EndeffectorHardware::read_param(uint8_t id, uint16_t address)
  {
    uint8_t dxl_error = 0;
    uint16_t dxl_present_position = 0;
    int dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, address, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to read position for ID %d: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
      return -1;
    }
    else if (dxl_error != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Error reading position for ID %d: %s", id, packetHandler_->getRxPacketError(dxl_error));
      return -1;
    }

    return dxl_present_position;
  }
  int32_t EndeffectorHardware::rad_to_pos(uint8_t id, double command_position)
  {
    return static_cast<int32_t>(DXL_MOTOR_POSITION_OFFSET + (command_position * (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE) / (DXL_MAX_ANGLE - DXL_MIN_ANGLE)));
  }

  int32_t EndeffectorHardware::pos_to_rad(double position)
  {
    return static_cast<double>(position - DXL_MOTOR_POSITION_OFFSET) * (DXL_MAX_ANGLE - DXL_MIN_ANGLE) / (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE);
  }

  return_type EndeffectorHardware::initialize_servos()
  {
    for (const auto &id : servo_ids_)
    {
      // Ping servo to check connection
      ping_servo(id);

      // Enable torque
      uint8_t dxl_error = 0;
      int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to enable torque for ID %d: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
        return return_type::ERROR;
      }
      else if (dxl_error != 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Error enabling torque for ID %d: %s", id, packetHandler_->getRxPacketError(dxl_error));
        return return_type::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Init Servo %d:", id);
    }
    return return_type::OK;
  }
  return_type EndeffectorHardware::ping_servo(uint8_t id)
  {
    uint16_t model_number = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->ping(portHandler_, id, &model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to ping ID %d: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
      return_type::ERROR;
    }
    else if (dxl_error != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Error pinging ID %d: %s", id, packetHandler_->getRxPacketError(dxl_error));
      return_type::ERROR;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Found Dynamixel ID %d, Model Number %d", id, model_number);
      return_type::ERROR;
    }
    return return_type::OK;
  }

} // namespace endeffector_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(endeffector_hardware::EndeffectorHardware, hardware_interface::SystemInterface)
