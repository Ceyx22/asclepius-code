// File: endeffector_hardware/src/endeffector_hardware.cpp

#include "endeffector_hardware/endeffector_hardware.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
// #include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

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

  constexpr int32_t DXL_MOTOR_POSITION_OFFSET = 512; // 150 degrees in motor position
  constexpr const char *kEndeffectorHardware = "EndeffectorHardware";

  CallbackReturn EndeffectorHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kEndeffectorHardware), "configure");
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    // Initialize the publisher for raw motor positions
    motor_position_publisher_ = rclcpp::Node::make_shared("motor_position_publisher")->create_publisher<std_msgs::msg::Int32MultiArray>("motor_positions", 10);
    joints_.resize(info_.joints.size(), Joint());
    servo_ids_.resize(info_.joints.size(), 0);

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      servo_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
      joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].prev_command.position = joints_[i].command.position;
      RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "joint_id %d: %d", i, servo_ids_[i]);
    }

    // Check if we should use dummy mode
    if (info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
        info_.hardware_parameters.at("use_dummy") == "true")
    {
      use_dummy_ = true;
      RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "Dummy mode enabled.");
      return CallbackReturn::SUCCESS;
    }

    // Initialize PortHandler instance
    auto port_name = info_.hardware_parameters.at("usb_port");
    auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

    RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "usb_port: %s", port_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "baud_rate: %d", baud_rate);

    portHandler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler_->openPort())
    {
      RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Failed to open the port!");
      return CallbackReturn::ERROR;
    }

    if (!portHandler_->setBaudRate(baud_rate))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Failed to set the baudrate!");
      return CallbackReturn::ERROR;
    }
    for (uint i = 0; i < info_.joints.size(); ++i)
    {
      uint16_t model_number = 0;
      if (!ping_servo(servo_ids_[i]))
      {
        // RCLCPP_FATAL(rclcpp::get_logger(kEndeffectorHardware));
        return CallbackReturn::ERROR;
      }
    }
    // enable_torque()
    ///////////////////////////////////////////////////////////
    return CallbackReturn::SUCCESS;
  }

  bool EndeffectorHardware::initialize_servos()
  {
    if (use_dummy_)
    {
      return true; // Skip hardware initialization in dummy mode
    }

    for (const auto &id : servo_ids_)
    {
      // Ping servo to check connection
      if (!ping_servo(id))
      {
        RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Failed to ping servo ID %d", id);
        return false;
      }

      // Enable torque
      uint8_t dxl_error = 0;
      int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Failed to enable torque for ID %d: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
      }
      else if (dxl_error != 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Error enabling torque for ID %d: %s", id, packetHandler_->getRxPacketError(dxl_error));
        return false;
      }
    }
    return true;
  }

  bool EndeffectorHardware::ping_servo(uint8_t id)
  {
    if (use_dummy_)
    {
      RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "Simulating ping for ID %d (dummy mode)", id);
      return true; // Simulate successful ping in dummy mode
    }

    uint16_t model_number = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->ping(portHandler_, id, &model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Failed to ping ID %d: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
      return false;
    }
    else if (dxl_error != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Error pinging ID %d: %s", id, packetHandler_->getRxPacketError(dxl_error));
      return false;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(kEndeffectorHardware), "Found Dynamixel ID %d, Model Number %d", id, model_number);
      return true;
    }
  }

  std::vector<hardware_interface::StateInterface> EndeffectorHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (const auto &joint_name : joint_names_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &position_states_[joint_name]));
      // state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[joint_name]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EndeffectorHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (const auto &joint_name : joint_names_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &position_commands_[joint_name]));
      // command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[joint_name]));
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

      int32_t dxl_present_position = use_dummy_ ? DXL_MIN_POSITION_VALUE : read_position(id);

      if (dxl_present_position == -1)
      {
        return CallbackReturn::ERROR;
      }

      double position = static_cast<double>(dxl_present_position) * (DXL_MAX_ANGLE - DXL_MIN_ANGLE) / (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE) + DXL_MIN_ANGLE;

      position_states_[joint_name] = position;
      position_commands_[joint_name] = position;
    }
    // Call the calibration function
    // calibrate_servos();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn EndeffectorHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    if (!use_dummy_)
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
    }

    return CallbackReturn::SUCCESS;
  }

  return_type EndeffectorHardware::enable_torque(const bool enabled)
  {
    for (const auto &id : servo_ids_)
    {

      // Enable torque
      uint8_t dxl_error = 0;
      int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Failed to enable torque for ID %d: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
        return return_type::ERROR;
      }
      else if (dxl_error != 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger(kEndeffectorHardware), "Error enabling torque for ID %d: %s", id, packetHandler_->getRxPacketError(dxl_error));
        return return_type::ERROR;
      }
    }
    return return_type::OK;
  }

  return_type EndeffectorHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (use_dummy_)
    {
      return return_type::OK;
    }
    std_msgs::msg::Int32MultiArray motor_positions_msg;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto id = servo_ids_[i];
      auto joint_name = joint_names_[i];
      // int32_t raw_motor_position = read_position(id);  // Reading raw position from the motor

      int32_t dxl_present_position = use_dummy_ ? static_cast<int32_t>(DXL_MIN_POSITION_VALUE + std::rand() % (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE)) : read_position(id);

      if (dxl_present_position == -1 && !use_dummy_)
      {
        return return_type::ERROR;
      }
      motor_positions_msg.data.push_back(dxl_present_position);
      double position = static_cast<double>(dxl_present_position - DXL_MOTOR_POSITION_OFFSET) * (DXL_MAX_ANGLE - DXL_MIN_ANGLE) / (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE) + DXL_MIN_ANGLE;

      position_states_[joint_name] = position;

      // Simulate velocity (constant value in dummy mode, or zero if no real data)
      velocity_states_[joint_name] = use_dummy_ ? 0.1 : 0.0;
    }
    motor_position_publisher_->publish(motor_positions_msg);
    return return_type::OK;
  }

  int32_t EndeffectorHardware::read_position(uint8_t id)
  {
    if (use_dummy_)
    {
      return DXL_MIN_POSITION_VALUE + std::rand() % (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE);
    }

    uint8_t dxl_error = 0;
    uint16_t dxl_present_position = 0;
    int dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_PRESENT_POSITION, &dxl_present_position, &dxl_error);
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

  return_type EndeffectorHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto id = servo_ids_[i];
      auto joint_name = joint_names_[i];

      double command_position = position_commands_[joint_name];

      // Convert the command position from radians to Dynamixel position value
      int32_t dxl_goal_position = static_cast<int32_t>((command_position * (DXL_MAX_POSITION_VALUE - DXL_MIN_POSITION_VALUE) / (DXL_MAX_ANGLE - DXL_MIN_ANGLE))) + DXL_MOTOR_POSITION_OFFSET;

      // Clamp the position value within the valid range
      dxl_goal_position = std::max(DXL_MIN_POSITION_VALUE, std::min(DXL_MAX_POSITION_VALUE, dxl_goal_position));

      if (!use_dummy_ && !write_position(id, dxl_goal_position))
      {
        return return_type::ERROR;
      }

      // In dummy mode, we simulate the actuator by updating the position state
      if (use_dummy_)
      {
        position_states_[joint_name] = command_position;
        velocity_states_[joint_name] = 0.1; // Simulated velocity
      }
    }

    return return_type::OK;
  }

  bool EndeffectorHardware::write_position(uint8_t id, int32_t position)
  {
    if (use_dummy_)
    {
      RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Simulating position write for ID %d (dummy mode)", id);
      return true;
    }

    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_GOAL_POSITION, static_cast<uint16_t>(position), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Failed to write position for ID %d: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
      return false;
    }
    else if (dxl_error != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EndeffectorHardware"), "Error writing position for ID %d: %s", id, packetHandler_->getRxPacketError(dxl_error));
      return false;
    }

    return true;
  }
  void EndeffectorHardware::calibrate_servos()
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto id = servo_ids_[i];

      // Move to max position (1023)
      RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Moving servo ID %d to max position", id);
      write_position(id, DXL_MAX_POSITION_VALUE);
      rclcpp::sleep_for(std::chrono::seconds(1)); // Delay for movement

      // Move to min position (0)
      RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Moving servo ID %d to min position", id);
      write_position(id, DXL_MIN_POSITION_VALUE);
      rclcpp::sleep_for(std::chrono::seconds(1)); // Delay for movement

      // Move to center position (512)
      RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Centering servo ID %d at position 512", id);
      write_position(id, DXL_MOTOR_POSITION_OFFSET);
      rclcpp::sleep_for(std::chrono::seconds(1)); // Delay for movement
    }

    RCLCPP_INFO(rclcpp::get_logger("EndeffectorHardware"), "Servo calibration completed.");
  }

} // namespace endeffector_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(endeffector_hardware::EndeffectorHardware, hardware_interface::SystemInterface)
